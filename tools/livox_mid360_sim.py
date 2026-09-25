#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Livox Mid-360 simulator: a fake LiDAR speaking the wire protocol over UDP.

Used by the C++ integration tests (and by hand) while no hardware is available. Design
decisions are recorded in GitHub issue #3. Standard library only.

    python3 tools/livox_mid360_sim.py --base-port 0

Control: JSON lines on stdin, e.g. {"cmd": "silence", "seconds": 2}, {"cmd": "quit"}.
Events:  JSON lines on stdout, e.g. {"event": "ready", "ports": {...}, "ip": "..."}.

Behaviour the simulator assumes and that must be reconciled with hardware (#11):
  * unicast discovery is answered like broadcast discovery,
  * settings persist across 0x0200 reboot except work_tgt_mode,
  * dev_type in the discovery ACK is a provisional value.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import selectors
import socket
import struct
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, field

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import livox_mid360_proto as proto  # noqa: E402

# --------------------------------------------------------------------------- constants
CMD_DISCOVERY, CMD_PARAM_CONFIG, CMD_PARAM_INQUIRE, CMD_INFO_PUSH = 0x0000, 0x0100, 0x0101, 0x0102
CMD_REBOOT, CMD_FACTORY_RESET, CMD_SET_GPS_TIME = 0x0200, 0x0201, 0x0202
REQ, ACK = 0, 1
SENDER_HOST, SENDER_LIDAR = 0, 1

WS_SAMPLING, WS_IDLE, WS_ERROR, WS_SELFCHECK, WS_MOTORSTARTUP, WS_UPGRADE, WS_READY = (
    1,
    2,
    4,
    5,
    6,
    8,
    9,
)

RET_OK, RET_FAIL = 0x00, 0x01
# Parameter errors as listed in the wiki (protocol.hpp RetCode): mirrors kParamNotSupport,
# kParamReadOnly, kParamInvalidLen, kOutOfRange.
RET_PARAM_NOT_SUPPORT, RET_PARAM_READ_ONLY, RET_PARAM_INVALID_LEN, RET_OUT_OF_RANGE = (
    0x20,
    0x22,
    0x23,
    0x03,
)
PROVISIONAL_DEV_TYPE = 9  # [unverified] see docs/protocol_notes.md / #11

KEY_PCL_DATA_TYPE, KEY_PATTERN_MODE, KEY_LIDAR_IPCFG = 0x0000, 0x0001, 0x0004
KEY_STATE_HOST, KEY_PCL_HOST, KEY_IMU_HOST = 0x0005, 0x0006, 0x0007
KEY_INSTALL_ATTITUDE, KEY_FOV0, KEY_FOV1, KEY_FOV_EN = 0x0012, 0x0015, 0x0016, 0x0017
KEY_DETECT_MODE, KEY_FUNC_IO, KEY_WORK_TGT_MODE, KEY_IMU_EN = 0x0018, 0x0019, 0x001A, 0x001C
KEY_SPEED_MODE, KEY_TIME_FILTER, KEY_PC_FREQ_MOD, KEY_IMU_SENSOR_CFG = (
    0x0021,
    0x0026,
    0x0029,
    0x002B,
)
KEY_SN, KEY_PRODUCT_INFO, KEY_VERSION_APP, KEY_VERSION_LOADER, KEY_VERSION_HW = (
    0x8000,
    0x8001,
    0x8002,
    0x8003,
    0x8004,
)
KEY_MAC, KEY_CUR_WORK_STATE, KEY_CORE_TEMP, KEY_POWERUP_CNT = 0x8005, 0x8006, 0x8007, 0x8008
KEY_LOCAL_TIME, KEY_LAST_SYNC_TIME, KEY_TIME_OFFSET, KEY_TIME_SYNC_TYPE = (
    0x8009,
    0x800A,
    0x800B,
    0x800C,
)
KEY_DIAG_STATUS, KEY_FW_TYPE, KEY_HMS = 0x800E, 0x8010, 0x8011

# Writable keys and their value lengths (mirrors keys.cpp key_value_length()).
WRITABLE_LEN = {
    KEY_PCL_DATA_TYPE: 1,
    KEY_PATTERN_MODE: 1,
    KEY_LIDAR_IPCFG: 12,
    KEY_STATE_HOST: 8,
    KEY_PCL_HOST: 8,
    KEY_IMU_HOST: 8,
    KEY_INSTALL_ATTITUDE: 24,
    KEY_FOV0: 20,
    KEY_FOV1: 20,
    KEY_FOV_EN: 1,
    KEY_DETECT_MODE: 1,
    KEY_FUNC_IO: 4,
    KEY_WORK_TGT_MODE: 1,
    KEY_IMU_EN: 1,
    KEY_SPEED_MODE: 1,
    KEY_TIME_FILTER: 1,
    KEY_PC_FREQ_MOD: 1,
    KEY_IMU_SENSOR_CFG: 3,
}
READ_ONLY = {
    KEY_SN,
    KEY_PRODUCT_INFO,
    KEY_VERSION_APP,
    KEY_VERSION_LOADER,
    KEY_VERSION_HW,
    KEY_MAC,
    KEY_CUR_WORK_STATE,
    KEY_CORE_TEMP,
    KEY_POWERUP_CNT,
    KEY_LOCAL_TIME,
    KEY_LAST_SYNC_TIME,
    KEY_TIME_OFFSET,
    KEY_TIME_SYNC_TYPE,
    KEY_DIAG_STATUS,
    KEY_FW_TYPE,
    KEY_HMS,
}

POINTS_PER_PACKET = 96
PCL_PACKET_RATE = 2000.0  # packets/s  (≈192k points/s)
IMU_RATE = 200.0
PUSH_RATE = 1.0


def factory_settings() -> dict[int, bytes]:
    """Writable keys at factory defaults (pcl_data_type=1, imu off, no host configured)."""
    zero = lambda n: b"\0" * n  # noqa: E731
    return {
        KEY_PCL_DATA_TYPE: b"\x01",
        KEY_PATTERN_MODE: b"\x00",
        KEY_LIDAR_IPCFG: bytes([192, 168, 1, 100, 255, 255, 255, 0, 192, 168, 1, 1]),
        KEY_STATE_HOST: zero(8),
        KEY_PCL_HOST: zero(8),
        KEY_IMU_HOST: zero(8),
        KEY_INSTALL_ATTITUDE: zero(24),
        KEY_FOV0: zero(20),
        KEY_FOV1: zero(20),
        KEY_FOV_EN: b"\x00",
        KEY_DETECT_MODE: b"\x00",
        KEY_FUNC_IO: zero(4),
        KEY_WORK_TGT_MODE: bytes([WS_SAMPLING]),
        KEY_IMU_EN: b"\x00",
        KEY_SPEED_MODE: b"\x00",
        KEY_TIME_FILTER: b"\x00",
        KEY_PC_FREQ_MOD: b"\x00",
        KEY_IMU_SENSOR_CFG: b"\x00\x00\x00",
    }


def parse_host_ipcfg(v: bytes) -> tuple[str, int, int] | None:
    ip = ".".join(map(str, v[:4]))
    dst, src = struct.unpack_from("<HH", v, 4)
    if v[:4] == b"\0\0\0\0" or dst == 0:
        return None
    return ip, dst, src


# --------------------------------------------------------------------------- device model
@dataclass
class DeviceModel:
    """Pure state machine + parameter table; no sockets, unit-testable."""

    sn: str = "SIM0000000000001"
    startup_delay: float = 0.3
    settings: dict[int, bytes] = field(default_factory=factory_settings)
    work_state: int = WS_MOTORSTARTUP
    hms: list[int] = field(default_factory=lambda: [0] * 8)
    time_offset_ns: int = 0
    time_sync_type: int = 0
    powerup_cnt: int = 1
    diag_status: int = 0
    state_deadline: float = 0.0  # monotonic time at which the pending transition completes
    on_state: Callable[[int, int], None] | None = None

    # -- lifecycle ---------------------------------------------------------
    def power_on(self, now: float) -> None:
        self._set_state(WS_MOTORSTARTUP)
        self.state_deadline = now + self.startup_delay

    def reboot(self, now: float) -> None:
        self.powerup_cnt += 1
        self.settings[KEY_WORK_TGT_MODE] = bytes([WS_SAMPLING])  # only tgt mode is not persisted
        self.power_on(now)

    def factory_reset(self, now: float) -> None:
        self.settings = factory_settings()
        self.hms = [0] * 8
        self.time_offset_ns = 0
        self.time_sync_type = 0
        self.reboot(now)

    def tick(self, now: float) -> None:
        """Complete pending transitions."""
        if self.work_state == WS_MOTORSTARTUP and now >= self.state_deadline:
            self._set_state(self.target_state())

    def target_state(self) -> int:
        tgt = self.settings[KEY_WORK_TGT_MODE][0]
        return tgt if tgt in (WS_SAMPLING, WS_IDLE, WS_READY) else WS_IDLE

    def _set_state(self, new: int) -> None:
        old = self.work_state
        if old != new:
            self.work_state = new
            if self.on_state:
                self.on_state(old, new)

    @property
    def sampling(self) -> bool:
        return self.work_state == WS_SAMPLING

    # -- parameters --------------------------------------------------------
    def configure(self, kvs: list[tuple[int, bytes]]) -> tuple[int, int]:
        """0x0100 semantics: validate everything first, then apply. Returns (ret, error_key)."""
        for key, value in kvs:
            if key in READ_ONLY:
                return RET_PARAM_READ_ONLY, key
            if key not in WRITABLE_LEN:
                return RET_PARAM_NOT_SUPPORT, key
            if len(value) != WRITABLE_LEN[key]:
                return RET_PARAM_INVALID_LEN, key
            if key == KEY_PCL_DATA_TYPE and value[0] not in (1, 2, 3):
                return RET_OUT_OF_RANGE, key
        for key, value in kvs:
            self.settings[key] = bytes(value)
            if key == KEY_WORK_TGT_MODE and self.work_state != WS_MOTORSTARTUP:
                self._set_state(self.target_state())
        return RET_OK, 0

    def inquire(self, keys: list[int], now_ns: int) -> tuple[int, list[tuple[int, bytes]]]:
        out = []
        for key in keys:
            v = self.read_key(key, now_ns)
            if v is None:
                return RET_PARAM_NOT_SUPPORT, [(key, b"")]
            out.append((key, v))
        return RET_OK, out

    def read_key(self, key: int, now_ns: int) -> bytes | None:
        if key in self.settings:
            return self.settings[key]
        ro = {
            KEY_SN: self.sn.encode().ljust(16, b"\0")[:16],
            KEY_PRODUCT_INFO: b"MID360-SIM".ljust(64, b"\0"),
            KEY_VERSION_APP: bytes([0, 0, 0, 1]),
            KEY_VERSION_LOADER: bytes([0, 0, 0, 1]),
            KEY_VERSION_HW: bytes([0, 0, 0, 1]),
            KEY_MAC: bytes([2, 0, 0, 0, 0, 1]),
            KEY_CUR_WORK_STATE: bytes([self.work_state]),
            KEY_CORE_TEMP: struct.pack("<i", 3500),
            KEY_POWERUP_CNT: struct.pack("<I", self.powerup_cnt),
            KEY_LOCAL_TIME: struct.pack("<Q", now_ns),
            KEY_LAST_SYNC_TIME: struct.pack("<Q", 0),
            KEY_TIME_OFFSET: struct.pack("<q", self.time_offset_ns),
            KEY_TIME_SYNC_TYPE: bytes([self.time_sync_type]),
            KEY_DIAG_STATUS: struct.pack("<H", self.diag_status),
            KEY_FW_TYPE: b"\x00",
            KEY_HMS: struct.pack("<8I", *self.hms),
        }
        return ro.get(key)

    def push_payload(self, now_ns: int) -> bytes:
        keys = [KEY_CUR_WORK_STATE, KEY_DIAG_STATUS, KEY_HMS, KEY_SN, KEY_LOCAL_TIME]
        kvs = [(k, self.read_key(k, now_ns)) for k in keys]
        return struct.pack("<HH", len(kvs), 0) + proto.encode_kv_list(kvs)

    def set_gps_time(self, ns: int, now_ns: int) -> None:
        self.time_offset_ns = ns - now_ns
        self.time_sync_type = 2

    def host(self, key: int) -> tuple[str, int, int] | None:
        return parse_host_ipcfg(self.settings[key])

    @property
    def pcl_data_type(self) -> int:
        return self.settings[KEY_PCL_DATA_TYPE][0]

    @property
    def imu_enabled(self) -> bool:
        return self.settings[KEY_IMU_EN][0] != 0


# --------------------------------------------------------------------------- data generation
class PointSource:
    """Deterministic pseudo-random points, reproducible for a given seed."""

    def __init__(self, seed: int) -> None:
        self._rng = random.Random(seed)

    def samples(self, data_type: int, n: int) -> list[tuple]:
        r = self._rng
        out = []
        for _ in range(n):
            depth_mm = r.randint(500, 40000)
            refl, tag = r.randint(0, 255), r.randint(0, 3)
            if data_type == 1:
                out.append(
                    (
                        r.randint(-depth_mm, depth_mm),
                        r.randint(-depth_mm, depth_mm),
                        r.randint(-2000, 2000),
                        refl,
                        tag,
                    )
                )
            elif data_type == 2:
                d = depth_mm // 10
                out.append((r.randint(-d, d), r.randint(-d, d), r.randint(-200, 200), refl, tag))
            else:
                out.append((depth_mm, r.randint(0, 35999), r.randint(0, 35999), refl, tag))
        return out

    def imu(self) -> tuple:
        r = self._rng
        return (
            r.uniform(-0.01, 0.01),
            r.uniform(-0.01, 0.01),
            r.uniform(-0.01, 0.01),
            r.uniform(-0.02, 0.02),
            r.uniform(-0.02, 0.02),
            1.0 + r.uniform(-0.02, 0.02),
        )


# --------------------------------------------------------------------------- simulator
class Simulator:
    def __init__(self, args: argparse.Namespace, out=sys.stdout, control=sys.stdin) -> None:
        self.args = args
        self.out = out
        self.control = control
        self.verbose = args.verbose
        self.model = DeviceModel(sn=args.sn, startup_delay=args.startup_delay)
        self.model.on_state = self._on_state
        self.points = PointSource(args.seed)
        self.rate = args.rate_multiplier
        self.frame_s = args.frame_ms / 1000.0
        self.drop_rate = args.drop_rate
        self._drop_rng = random.Random(args.seed ^ 0x5A5A)

        self.seq = 0  # LiDAR-originated frames (push)
        self.udp_cnt_pcl = 0
        self.udp_cnt_imu = 0
        self.frame_cnt = 0
        self.frame_started = 0.0
        self.next_pcl = self.next_imu = self.next_push = self.next_stats = 0.0
        self.sent = {"pcl": 0, "imu": 0, "push": 0, "pcl_dropped": 0}
        self.silence_until = 0.0
        self.drop_ack = 0
        self.running = True

        self.sel = selectors.DefaultSelector()
        self.socks: dict[str, socket.socket] = {}
        self.ports: dict[str, int] = {}
        self._open_sockets()

    # -- setup ---------------------------------------------------------------
    def _open_sockets(self) -> None:
        base = self.args.base_port
        names = ["discovery", "cmd", "push", "pcl", "imu"]
        for i, name in enumerate(names):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.setblocking(False)
            s.bind((self.args.bind, 0 if base == 0 else base + 100 * i))
            self.socks[name] = s
            self.ports[name] = s.getsockname()[1]
        self.sel.register(self.socks["discovery"], selectors.EVENT_READ, "discovery")
        self.sel.register(self.socks["cmd"], selectors.EVENT_READ, "cmd")
        if self.control is not None:
            try:
                self.sel.register(self.control, selectors.EVENT_READ, "control")
            except (ValueError, OSError):
                self.control = None

    def lidar_ip(self) -> str:
        if self.args.bind not in ("", "0.0.0.0"):
            return self.args.bind
        return "127.0.0.1"

    # -- events ----------------------------------------------------------------
    def emit(self, **ev) -> None:
        try:
            self.out.write(json.dumps(ev, separators=(",", ":")) + "\n")
            self.out.flush()
        except (BrokenPipeError, ValueError):  # parent closed stdout
            self.running = False

    def log(self, msg: str) -> None:
        if self.verbose:
            sys.stderr.write(f"[sim] {msg}\n")
            sys.stderr.flush()

    def _on_state(self, old: int, new: int) -> None:
        self.emit(event="state", **{"from": old, "to": new})
        if new == WS_SAMPLING:
            now = time.monotonic()
            self.frame_started = now
            self.next_pcl = self.next_imu = now

    # -- main loop -------------------------------------------------------------
    def run(self) -> None:
        now = time.monotonic()
        self.model.power_on(now)
        self.next_push = self.next_stats = now + 1.0
        self.emit(
            event="ready", ip=self.lidar_ip(), ports=self.ports, sn=self.model.sn, pid=os.getpid()
        )
        while self.running:
            now = time.monotonic()
            self.model.tick(now)
            self._send_periodic(now)
            timeout = max(0.0, min(self._next_deadline(now) - now, 0.05))
            for key, _ in self.sel.select(timeout):
                kind = key.data
                if kind == "control":
                    self._handle_control()
                else:
                    self._handle_datagram(kind)
        self.emit(event="exit", sent=self.sent)

    def _next_deadline(self, now: float) -> float:
        d = [self.next_push, self.next_stats]
        if self.model.work_state == WS_MOTORSTARTUP:
            d.append(self.model.state_deadline)
        if self.model.sampling:
            d.append(self.next_pcl)
            if self.model.imu_enabled:
                d.append(self.next_imu)
        return min(d)

    # -- control channel -------------------------------------------------------
    def _handle_control(self) -> None:
        line = self.control.readline()
        if not line:  # EOF: parent went away
            self.sel.unregister(self.control)
            self.control = None
            if self.args.quit_on_eof:
                self.running = False
            return
        line = line.strip()
        if not line:
            return
        try:
            req = json.loads(line)
        except json.JSONDecodeError as e:
            self.emit(event="error", error=f"bad json: {e}")
            return
        self.apply_control(req)

    def apply_control(self, req: dict) -> None:
        cmd = req.get("cmd")
        now = time.monotonic()
        if cmd == "quit":
            self.running = False
        elif cmd == "silence":
            self.silence_until = now + float(req.get("seconds", 1.0))
        elif cmd == "hms":
            codes = [int(c) for c in req.get("codes", [])][:8]
            self.model.hms = (codes + [0] * 8)[:8]
        elif cmd == "drop_ack":
            self.drop_ack += int(req.get("count", 1))
        elif cmd == "reboot":
            self._do_reboot(now)
        elif cmd == "set_state":
            self.model.settings[KEY_WORK_TGT_MODE] = bytes([int(req["state"])])
            self.model._set_state(int(req["state"]))
        elif cmd == "drop_rate":
            self.drop_rate = float(req.get("rate", 0.0))
        elif cmd == "status":
            self.emit(
                event="status",
                state=self.model.work_state,
                sent=self.sent,
                hosts={
                    "pcl": self.model.host(KEY_PCL_HOST),
                    "imu": self.model.host(KEY_IMU_HOST),
                    "push": self.model.host(KEY_STATE_HOST),
                },
            )
        else:
            self.emit(event="error", error=f"unknown control cmd: {cmd!r}")
            return
        self.emit(event="control", cmd=cmd)

    def _do_reboot(self, now: float) -> None:
        self.seq = 0
        self.udp_cnt_pcl = self.udp_cnt_imu = 0
        self.frame_cnt = 0
        self.silence_until = now + self.args.reboot_silence
        self.model.reboot(now + self.args.reboot_silence)

    # -- command handling ------------------------------------------------------
    def _handle_datagram(self, kind: str) -> None:
        sock = self.socks[kind]
        try:
            data, addr = sock.recvfrom(2048)
        except (BlockingIOError, InterruptedError):
            return
        now = time.monotonic()
        if now < self.silence_until:
            self.log(f"ignoring {len(data)}B from {addr} (silence)")
            return
        try:
            frame = proto.CommandFrame.parse(data)
        except ValueError as e:
            self.emit(event="bad_frame", **{"from": f"{addr[0]}:{addr[1]}", "error": str(e)})
            return
        if frame.cmd_type != REQ:
            return
        if kind == "discovery" and frame.cmd_id != CMD_DISCOVERY:
            return
        if kind == "cmd" and frame.cmd_id == CMD_DISCOVERY:
            return
        ret, payload = self._dispatch(frame, addr, now)
        self.emit(
            event="cmd",
            cmd_id=frame.cmd_id,
            seq=frame.seq_num,
            ret=ret,
            **{"from": f"{addr[0]}:{addr[1]}"},
        )
        if payload is None:
            return
        if self.drop_ack > 0:
            self.drop_ack -= 1
            self.emit(event="ack_dropped", cmd_id=frame.cmd_id, seq=frame.seq_num)
            return
        ack = proto.CommandFrame(frame.seq_num, frame.cmd_id, ACK, SENDER_LIDAR, payload).encode()
        sock.sendto(ack, addr)

    def _dispatch(self, f: proto.CommandFrame, addr, now: float) -> tuple[int, bytes | None]:
        now_ns = self.now_ns()
        m = self.model
        if f.cmd_id == CMD_DISCOVERY:
            sn = m.sn.encode().ljust(16, b"\0")[:16]
            ip = bytes(int(x) for x in self.lidar_ip().split("."))
            return RET_OK, struct.pack(
                "<BB16s4sH", RET_OK, PROVISIONAL_DEV_TYPE, sn, ip, self.ports["cmd"]
            )
        if f.cmd_id == CMD_PARAM_CONFIG:
            try:
                n, _ = struct.unpack_from("<HH", f.data, 0)
                kvs = proto.parse_kv_list(f.data[4:], n)
            except (struct.error, ValueError):
                return RET_FAIL, struct.pack("<BH", RET_FAIL, 0)
            ret, err = m.configure(kvs)
            if ret == RET_OK:
                self.log(f"configured {[hex(k) for k, _ in kvs]}")
            return ret, struct.pack("<BH", ret, err)
        if f.cmd_id == CMD_PARAM_INQUIRE:
            try:
                n, _ = struct.unpack_from("<HH", f.data, 0)
                keys = list(struct.unpack_from(f"<{n}H", f.data, 4))
            except struct.error:
                return RET_FAIL, struct.pack("<BH", RET_FAIL, 0)
            ret, kvs = m.inquire(keys, now_ns)
            if ret != RET_OK:
                return ret, struct.pack("<BH", ret, 0)
            return ret, struct.pack("<BH", ret, len(kvs)) + proto.encode_kv_list(kvs)
        if f.cmd_id == CMD_REBOOT:
            # The ACK is sent by the caller before the silence window is checked again.
            self._do_reboot(now)
            return RET_OK, struct.pack("<B", RET_OK)
        if f.cmd_id == CMD_FACTORY_RESET:
            m.factory_reset(now + self.args.reboot_silence)
            self.seq = 0
            self.udp_cnt_pcl = self.udp_cnt_imu = self.frame_cnt = 0
            self.silence_until = now + self.args.reboot_silence
            return RET_OK, struct.pack("<B", RET_OK)
        if f.cmd_id == CMD_SET_GPS_TIME:
            if len(f.data) < 9 or f.data[0] != 2:
                return RET_FAIL, struct.pack("<B", RET_FAIL)
            (ns,) = struct.unpack_from("<Q", f.data, 1)
            m.set_gps_time(ns, now_ns)
            return RET_OK, struct.pack("<B", RET_OK)
        return RET_FAIL, struct.pack("<B", RET_FAIL)  # unknown cmd_id

    # -- periodic senders ------------------------------------------------------
    def now_ns(self) -> int:
        return time.time_ns() + self.model.time_offset_ns

    def _send_periodic(self, now: float) -> None:
        if now < self.silence_until:
            return
        m = self.model
        if m.sampling:
            pcl_host = m.host(KEY_PCL_HOST)
            interval = 1.0 / (PCL_PACKET_RATE * self.rate)
            budget = 256  # bound catch-up bursts
            while now >= self.next_pcl and budget > 0:
                if now - self.frame_started >= self.frame_s:
                    self.frame_cnt = (self.frame_cnt + 1) & 0xFF
                    self.frame_started += self.frame_s
                self._send_pcl(pcl_host, interval)
                self.next_pcl += interval
                budget -= 1
            if now - self.next_pcl > 0.5:  # fell hopelessly behind: resync
                self.next_pcl = now
            if m.imu_enabled:
                imu_host = m.host(KEY_IMU_HOST)
                imu_interval = 1.0 / (IMU_RATE * self.rate)
                while now >= self.next_imu:
                    self._send_imu(imu_host, imu_interval)
                    self.next_imu += imu_interval
                if now - self.next_imu > 0.5:
                    self.next_imu = now
        if now >= self.next_push:
            self._send_push()
            self.next_push += 1.0 / (PUSH_RATE * self.rate)
        if now >= self.next_stats:
            self.emit(event="sent", **self.sent, state=m.work_state)
            self.next_stats += 1.0

    def _send_pcl(self, host, interval_s: float) -> None:
        dt = self.model.pcl_data_type
        pkt = proto.DataPacket(
            time_interval=int(interval_s * 1e7),
            dot_num=POINTS_PER_PACKET,
            udp_cnt=self.udp_cnt_pcl,
            frame_cnt=self.frame_cnt,
            data_type=dt,
            time_type=self.model.time_sync_type,
            timestamp_ns=self.now_ns(),
            data=proto.pack_samples(dt, self.points.samples(dt, POINTS_PER_PACKET)),
        )
        self.udp_cnt_pcl = (self.udp_cnt_pcl + 1) & 0xFFFF
        if host is None:
            return
        if self.drop_rate > 0 and self._drop_rng.random() < self.drop_rate:
            self.sent["pcl_dropped"] += 1
            return
        self._sendto("pcl", pkt.encode(), host)
        self.sent["pcl"] += 1

    def _send_imu(self, host, interval_s: float) -> None:
        pkt = proto.DataPacket(
            time_interval=int(interval_s * 1e7),
            dot_num=1,
            udp_cnt=self.udp_cnt_imu,
            frame_cnt=self.frame_cnt,
            data_type=0,
            time_type=self.model.time_sync_type,
            timestamp_ns=self.now_ns(),
            data=proto.pack_samples(0, [self.points.imu()]),
        )
        self.udp_cnt_imu = (self.udp_cnt_imu + 1) & 0xFFFF
        if host is None:
            return
        self._sendto("imu", pkt.encode(), host)
        self.sent["imu"] += 1

    def _send_push(self) -> None:
        host = self.model.host(KEY_STATE_HOST)
        if host is None:
            return
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        frame = proto.CommandFrame(
            self.seq, CMD_INFO_PUSH, REQ, SENDER_LIDAR, self.model.push_payload(self.now_ns())
        ).encode()
        self._sendto("push", frame, host)
        self.sent["push"] += 1

    def _sendto(self, kind: str, data: bytes, host: tuple[str, int, int]) -> None:
        try:
            self.socks[kind].sendto(data, (host[0], host[1]))
        except OSError as e:
            self.log(f"send {kind} to {host[:2]} failed: {e}")


# --------------------------------------------------------------------------- CLI
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Livox Mid-360 simulator")
    p.add_argument("--bind", default="0.0.0.0", help="address to bind (default 0.0.0.0)")
    p.add_argument(
        "--base-port",
        type=int,
        default=proto.PORT_DISCOVERY,
        help="discovery port; cmd/push/pcl/imu follow at +100..+400. 0 = pick free ports",
    )
    p.add_argument("--sn", default="SIM0000000000001")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument(
        "--startup-delay", type=float, default=0.3, help="seconds spent in MOTORSTARTUP"
    )
    p.add_argument(
        "--reboot-silence", type=float, default=0.5, help="seconds of silence after 0x0200"
    )
    p.add_argument("--frame-ms", type=float, default=100.0)
    p.add_argument("--rate-multiplier", type=float, default=1.0)
    p.add_argument(
        "--drop-rate", type=float, default=0.0, help="fraction of point-cloud packets to drop"
    )
    p.add_argument("--quit-on-eof", action="store_true", default=True)
    p.add_argument("--no-quit-on-eof", dest="quit_on_eof", action="store_false")
    p.add_argument("--verbose", "-v", action="store_true")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if len(args.sn) > 16:
        sys.exit("--sn must be at most 16 characters")
    sim = Simulator(args)
    try:
        sim.run()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

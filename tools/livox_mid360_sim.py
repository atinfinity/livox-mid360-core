#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Livox Mid-360 simulator: a fake LiDAR speaking the wire protocol over UDP.

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
from collections.abc import Callable
from dataclasses import dataclass, field
import json
import math
import os
import random
import selectors
import socket
import struct
import sys
import time

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

# Requestable through work_tgt_mode; the rest are intermediate states (wiki 1.2.1.2.2).
WS_REQUESTABLE = (WS_SAMPLING, WS_IDLE, WS_READY)
WS_ALL = (WS_SAMPLING, WS_IDLE, WS_ERROR, WS_SELFCHECK, WS_MOTORSTARTUP, WS_UPGRADE, WS_READY)

RET_OK, RET_FAIL, RET_NOT_PERMIT_NOW = 0x00, 0x01, 0x02
# Parameter errors as listed in the wiki (protocol.hpp RetCode): mirrors kParamNotSupport,
# kParamReadOnly, kParamInvalidLen, kOutOfRange.
RET_PARAM_NOT_SUPPORT, RET_PARAM_READ_ONLY, RET_PARAM_INVALID_LEN, RET_OUT_OF_RANGE = (
    0x20,
    0x22,
    0x23,
    0x03,
)
RET_PARAM_REBOOT_EFFECT = 0x21  # accepted, takes effect after reboot (kParamRebootEffect)
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
MAX_FOV_DRAWS = 16  # batches of POINTS_PER_PACKET drawn per packet while FOV cropping
PCL_PACKET_RATE = 2000.0  # packets/s  (≈192k points/s)
IMU_RATE = 200.0  # at imu_sensor_cfg output_rate 0
IMU_RATES = {0: 200.0, 1: 500.0, 2: 100.0, 3: 50.0}  # key 0x002B data[0]
PUSH_RATE = 1.0


def factory_settings() -> dict[int, bytes]:
    """Writable keys at factory defaults (pcl_data_type=1, imu off, no host configured)."""
    zero = lambda n: b'\0' * n  # noqa: E731
    return {
        KEY_PCL_DATA_TYPE: b'\x01',
        KEY_PATTERN_MODE: b'\x00',
        KEY_LIDAR_IPCFG: bytes([192, 168, 1, 100, 255, 255, 255, 0, 192, 168, 1, 1]),
        KEY_STATE_HOST: zero(8),
        KEY_PCL_HOST: zero(8),
        KEY_IMU_HOST: zero(8),
        KEY_INSTALL_ATTITUDE: zero(24),
        KEY_FOV0: zero(20),
        KEY_FOV1: zero(20),
        KEY_FOV_EN: b'\x00',
        KEY_DETECT_MODE: b'\x00',
        KEY_FUNC_IO: zero(4),
        KEY_WORK_TGT_MODE: bytes([WS_SAMPLING]),
        KEY_IMU_EN: b'\x00',
        KEY_SPEED_MODE: b'\x00',
        KEY_TIME_FILTER: b'\x00',
        KEY_PC_FREQ_MOD: b'\x00',
        KEY_IMU_SENSOR_CFG: b'\x00\x00\x00',
    }


def parse_host_ipcfg(v: bytes) -> tuple[str, int, int] | None:
    ip = '.'.join(map(str, v[:4]))
    dst, src = struct.unpack_from('<HH', v, 4)
    if v[:4] == b'\0\0\0\0' or dst == 0:
        return None
    return ip, dst, src


# --------------------------------------------------------------------------- device model
@dataclass
class DeviceModel:
    """
    Pure state machine + parameter table; no sockets, unit-testable.

    The work-state machine follows the figure in the protocol wiki (1.2.1.2.2), see
    docs/protocol_notes.md: power-on -> SELFCHECK -> IDLE, then the machine chases
    work_tgt_mode through MOTORSTARTUP / READY. SELFCHECK and MOTORSTARTUP are timed
    (``selfcheck_delay`` / ``startup_delay``); the pass-through READY has no dwell.
    """

    sn: str = 'SIM0000000000001'
    product_info: str = 'MID360-SIM'
    version_app: tuple[int, int, int, int] = (0, 0, 0, 1)
    version_loader: tuple[int, int, int, int] = (0, 0, 0, 1)
    version_hardware: tuple[int, int, int, int] = (0, 0, 0, 1)
    startup_delay: float = 0.3
    selfcheck_delay: float = 0.1
    imu_cfg_unsupported: bool = False  # emulate firmware without key 0x002B
    settings: dict[int, bytes] = field(default_factory=factory_settings)
    work_state: int = WS_SELFCHECK
    hms: list[int] = field(default_factory=lambda: [0] * 8)
    time_offset_ns: int = 0
    time_sync_type: int = 0
    powerup_cnt: int = 1
    diag_status: int = 0
    state_deadline: float = 0.0  # monotonic time at which the timed state completes
    on_state: Callable[[int, int], None] | None = None

    # -- lifecycle ---------------------------------------------------------
    def power_on(self, now: float) -> None:
        self._enter_timed(WS_SELFCHECK, now)

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

    # -- work-state machine ------------------------------------------------
    @property
    def timed(self) -> bool:
        """True while in a state that completes at ``state_deadline``."""
        return self.work_state in (WS_SELFCHECK, WS_MOTORSTARTUP)

    def tick(self, now: float) -> None:
        """Complete the pending timed transition, then chase the target."""
        if self.timed and now >= self.state_deadline:
            # Self-check succeeded -> "Enter idle"; motor started -> READY.
            self._set_state(WS_IDLE if self.work_state == WS_SELFCHECK else WS_READY)
        self._follow_target(now)

    def target_state(self) -> int:
        tgt = self.settings[KEY_WORK_TGT_MODE][0]
        return tgt if tgt in WS_REQUESTABLE else WS_IDLE

    def force_state(self, new: int, now: float) -> None:
        """
        Control-channel override of cur_work_state (e.g. a LiDAR-side ERROR).

        work_tgt_mode is untouched: forcing a work substate models "abnormal
        disappearance", after which the machine chases the target again from the
        next tick.
        """
        if new in (WS_SELFCHECK, WS_MOTORSTARTUP):
            self._enter_timed(new, now)
        else:
            self._set_state(new)

    def _enter_timed(self, state: int, now: float) -> None:
        self._set_state(state)
        delay = self.selfcheck_delay if state == WS_SELFCHECK else self.startup_delay
        self.state_deadline = now + delay

    def _follow_target(self, now: float) -> None:
        """Take the instantaneous edges of the work substate towards work_tgt_mode."""
        tgt = self.target_state()
        if self.work_state == WS_IDLE and tgt in (WS_SAMPLING, WS_READY):
            self._enter_timed(WS_MOTORSTARTUP, now)
        elif self.work_state == WS_SAMPLING and tgt in (WS_IDLE, WS_READY):
            self._set_state(WS_READY)
        if self.work_state == WS_READY and tgt in (WS_SAMPLING, WS_IDLE):
            self._set_state(tgt)

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
    def configure(self, kvs: list[tuple[int, bytes]], now: float = 0.0) -> tuple[int, int]:
        """0x0100 semantics: validate everything first, then apply. Returns (ret, error_key)."""
        for key, value in kvs:
            if key in READ_ONLY:
                return RET_PARAM_READ_ONLY, key
            if key not in WRITABLE_LEN:
                return RET_PARAM_NOT_SUPPORT, key
            if key == KEY_IMU_SENSOR_CFG and self.imu_cfg_unsupported:
                return RET_PARAM_NOT_SUPPORT, key
            if len(value) != WRITABLE_LEN[key]:
                return RET_PARAM_INVALID_LEN, key
            if key == KEY_PCL_DATA_TYPE and value[0] not in (1, 2, 3):
                return RET_OUT_OF_RANGE, key
            if key == KEY_PATTERN_MODE and value[0] != 0:
                # [unverified] the base Mid-360 has only the non-repetitive pattern (#11):
                # the defined 1 / 2 are "not supported", anything else is out of range.
                if value[0] in (1, 2):
                    return RET_PARAM_NOT_SUPPORT, key
                return RET_OUT_OF_RANGE, key
            if key in (KEY_FOV0, KEY_FOV1) and not fov_in_range(value):
                return RET_OUT_OF_RANGE, key
            if key in (KEY_DETECT_MODE, KEY_TIME_FILTER, KEY_IMU_EN) and value[0] > 1:
                return RET_OUT_OF_RANGE, key
            if key == KEY_IMU_SENSOR_CFG and (value[0] > 3 or value[1] > 3 or value[2] > 7):
                return RET_OUT_OF_RANGE, key
            if key == KEY_WORK_TGT_MODE:
                # [unverified] return codes, see #11. ERROR / UPGRADE are left only by
                # reboot / "abnormal disappearance"; 4/5/6/8 exist but are "Not Support".
                if self.work_state in (WS_ERROR, WS_UPGRADE):
                    return RET_NOT_PERMIT_NOW, key
                if value[0] in WS_ALL and value[0] not in WS_REQUESTABLE:
                    return RET_PARAM_NOT_SUPPORT, key
                if value[0] not in WS_REQUESTABLE:
                    return RET_OUT_OF_RANGE, key
        ret = RET_OK
        for key, value in kvs:
            changed = self.settings.get(key) != bytes(value)
            self.settings[key] = bytes(value)
            if key == KEY_LIDAR_IPCFG and changed:
                # [unverified] the wiki lists 0x21 without naming the keys; the LiDAR's own
                # address is the obvious candidate (#11, #50).
                ret = RET_PARAM_REBOOT_EFFECT
        self._follow_target(now)
        return ret, 0

    def inquire(self, keys: list[int], now_ns: int) -> tuple[int, list[tuple[int, bytes]]]:
        out = []
        for key in keys:
            v = self.read_key(key, now_ns)
            if v is None:
                return RET_PARAM_NOT_SUPPORT, [(key, b'')]
            out.append((key, v))
        return RET_OK, out

    def read_key(self, key: int, now_ns: int) -> bytes | None:
        if key == KEY_IMU_SENSOR_CFG and self.imu_cfg_unsupported:
            return None
        if key in self.settings:
            return self.settings[key]
        ro = {
            KEY_SN: self.sn.encode().ljust(16, b'\0')[:16],
            KEY_PRODUCT_INFO: self.product_info.encode().ljust(64, b'\0')[:64],
            KEY_VERSION_APP: bytes(self.version_app),
            KEY_VERSION_LOADER: bytes(self.version_loader),
            KEY_VERSION_HW: bytes(self.version_hardware),
            KEY_MAC: bytes([2, 0, 0, 0, 0, 1]),
            KEY_CUR_WORK_STATE: bytes([self.work_state]),
            KEY_CORE_TEMP: struct.pack('<i', 3500),
            KEY_POWERUP_CNT: struct.pack('<I', self.powerup_cnt),
            KEY_LOCAL_TIME: struct.pack('<Q', now_ns),
            KEY_LAST_SYNC_TIME: struct.pack('<Q', 0),
            KEY_TIME_OFFSET: struct.pack('<q', self.time_offset_ns),
            KEY_TIME_SYNC_TYPE: bytes([self.time_sync_type]),
            KEY_DIAG_STATUS: struct.pack('<H', self.diag_status),
            KEY_FW_TYPE: b'\x00',
            KEY_HMS: struct.pack('<8I', *self.hms),
        }
        return ro.get(key)

    def push_payload(self, now_ns: int) -> bytes:
        # Every read-only key 0x8000-0x8011 [unverified: the real push key set, issue #11].
        keys = [
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
        ]
        kvs = [(k, self.read_key(k, now_ns)) for k in keys]
        return struct.pack('<HH', len(kvs), 0) + proto.encode_kv_list(kvs)

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

    @property
    def imu_rate(self) -> float:
        """Return the IMU packet rate in Hz selected by key 0x002B (200 Hz when unsupported)."""
        if self.imu_cfg_unsupported:
            return IMU_RATE
        return IMU_RATES[self.settings[KEY_IMU_SENSOR_CFG][0]]

    def fov_windows(self) -> list[tuple[int, int, int, int]]:
        """Return the enabled FOV windows as (yaw_start, yaw_stop, pitch_start, pitch_stop)."""
        mask = self.settings[KEY_FOV_EN][0]
        out = []
        for bit, key in ((1, KEY_FOV0), (2, KEY_FOV1)):
            if mask & bit:
                out.append(struct.unpack('<iiiiI', self.settings[key])[:4])
        return out

    def keeps_point(self, data_type: int, sample: tuple) -> bool:
        """
        Decide whether a sample survives the [unverified] FOV cropping (see #11).

        No enabled window keeps everything; otherwise a point stays when it lies inside any
        enabled window. Yaw is [start, stop) with wrap-around when start > stop (start ==
        stop is empty); pitch is [start, stop].
        """
        windows = self.fov_windows()
        if not windows:
            return True
        yaw, pitch = point_angles(data_type, sample)
        return any(in_fov_window(w, yaw, pitch) for w in windows)


# --------------------------------------------------------------------------- FOV
def fov_in_range(value: bytes) -> bool:
    """Wiki ranges for keys 0x0015 / 0x0016: yaw in [0, 360), pitch in (-10, 60)."""
    yaw0, yaw1, pitch0, pitch1, _ = struct.unpack('<iiiiI', value)
    return all(0 <= y < 360 for y in (yaw0, yaw1)) and all(-10 < p < 60 for p in (pitch0, pitch1))


def point_angles(data_type: int, sample: tuple) -> tuple[float, float]:
    """(yaw, pitch) in degrees of a sample; yaw in [0, 360), pitch in [-90, 90]."""
    if data_type == 3:  # (depth, theta zenith, phi azimuth) in 0.01 deg
        return sample[2] / 100.0 % 360.0, 90.0 - sample[1] / 100.0
    x, y, z = sample[0], sample[1], sample[2]
    yaw = math.degrees(math.atan2(y, x)) % 360.0
    pitch = math.degrees(math.atan2(z, math.hypot(x, y)))
    return yaw, pitch


def in_fov_window(window: tuple[int, int, int, int], yaw: float, pitch: float) -> bool:
    yaw0, yaw1, pitch0, pitch1 = window
    if pitch < min(pitch0, pitch1) or pitch > max(pitch0, pitch1):
        return False
    if yaw0 == yaw1:
        return False
    if yaw0 < yaw1:
        return yaw0 <= yaw < yaw1
    return yaw >= yaw0 or yaw < yaw1


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
                out.append((depth_mm, r.randint(0, 18000), r.randint(0, 35999), refl, tag))
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
    """Sockets, threads and JSON control channel around one DeviceModel."""

    def __init__(self, args: argparse.Namespace, out=sys.stdout, control=sys.stdin) -> None:
        self.args = args
        self.out = out
        self.control = control
        self.verbose = args.verbose
        self.model = DeviceModel(
            sn=args.sn,
            product_info=args.product_info,
            version_app=parse_version(args.version_app),
            version_loader=parse_version(args.version_loader),
            version_hardware=parse_version(args.version_hardware),
            startup_delay=args.startup_delay,
            selfcheck_delay=args.selfcheck_delay,
            imu_cfg_unsupported=args.imu_cfg_unsupported,
        )
        self.model.on_state = self._on_state
        self.points = PointSource(args.seed)
        self.rate = args.rate_multiplier
        self.push_rate = args.push_rate
        self.frame_s = args.frame_ms / 1000.0
        self.drop_rate = args.drop_rate
        self._drop_rng = random.Random(args.seed ^ 0x5A5A)

        self.seq = 0  # LiDAR-originated frames (push)
        self.udp_cnt_pcl = 0
        self.udp_cnt_imu = 0
        self.frame_cnt = 0
        self.frame_started = 0.0
        self.next_pcl = self.next_imu = self.next_push = self.next_stats = 0.0
        self.sent = {'pcl': 0, 'imu': 0, 'push': 0, 'pcl_dropped': 0}
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
        names = ['discovery', 'cmd', 'push', 'pcl', 'imu']
        for i, name in enumerate(names):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.setblocking(False)
            s.bind((self.args.bind, 0 if base == 0 else base + 100 * i))
            self.socks[name] = s
            self.ports[name] = s.getsockname()[1]
        self.sel.register(self.socks['discovery'], selectors.EVENT_READ, 'discovery')
        self.sel.register(self.socks['cmd'], selectors.EVENT_READ, 'cmd')
        if self.control is not None:
            try:
                self.sel.register(self.control, selectors.EVENT_READ, 'control')
            except (ValueError, OSError):
                self.control = None

    def lidar_ip(self) -> str:
        if self.args.bind not in ('', '0.0.0.0'):
            return self.args.bind
        return '127.0.0.1'

    # -- events ----------------------------------------------------------------
    def emit(self, **ev) -> None:
        try:
            self.out.write(json.dumps(ev, separators=(',', ':')) + '\n')
            self.out.flush()
        except (BrokenPipeError, ValueError):  # parent closed stdout
            self.running = False

    def log(self, msg: str) -> None:
        if self.verbose:
            sys.stderr.write(f'[sim] {msg}\n')
            sys.stderr.flush()

    def _on_state(self, old: int, new: int) -> None:
        self.emit(event='state', **{'from': old, 'to': new})
        if new == WS_SAMPLING:
            now = time.monotonic()
            self.frame_started = now
            self.next_pcl = self.next_imu = now

    # -- main loop -------------------------------------------------------------
    def run(self) -> None:
        now = time.monotonic()
        self.model.power_on(now)
        self.next_push = now  # pushes run from boot; unsent while no host is configured
        self.next_stats = now + 1.0
        self.emit(
            event='ready', ip=self.lidar_ip(), ports=self.ports, sn=self.model.sn, pid=os.getpid()
        )
        while self.running:
            now = time.monotonic()
            self.model.tick(now)
            self._send_periodic(now)
            timeout = max(0.0, min(self._next_deadline(now) - now, 0.05))
            for key, _ in self.sel.select(timeout):
                kind = key.data
                if kind == 'control':
                    self._handle_control()
                else:
                    self._handle_datagram(kind)
        self.emit(event='exit', sent=self.sent)

    def _next_deadline(self, now: float) -> float:
        d = [self.next_push, self.next_stats]
        if self.model.timed:
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
            self.emit(event='error', error=f'bad json: {e}')
            return
        self.apply_control(req)

    def apply_control(self, req: dict) -> None:
        cmd = req.get('cmd')
        now = time.monotonic()
        if cmd == 'quit':
            self.running = False
        elif cmd == 'silence':
            self.silence_until = now + float(req.get('seconds', 1.0))
        elif cmd == 'hms':
            codes = [int(c) for c in req.get('codes', [])][:8]
            self.model.hms = (codes + [0] * 8)[:8]
        elif cmd == 'drop_ack':
            self.drop_ack += int(req.get('count', 1))
        elif cmd == 'reboot':
            self._do_reboot(now)
        elif cmd == 'set_state':
            self.model.force_state(int(req['state']), now)
        elif cmd == 'drop_rate':
            self.drop_rate = float(req.get('rate', 0.0))
        elif cmd == 'frame_ms':
            self.frame_s = float(req.get('ms', 100.0)) / 1000.0
            self.frame_started = now
        elif cmd == 'status':
            self.emit(
                event='status',
                state=self.model.work_state,
                sent=self.sent,
                hosts={
                    'pcl': self.model.host(KEY_PCL_HOST),
                    'imu': self.model.host(KEY_IMU_HOST),
                    'push': self.model.host(KEY_STATE_HOST),
                },
            )
        else:
            self.emit(event='error', error=f'unknown control cmd: {cmd!r}')
            return
        self.emit(event='control', cmd=cmd)

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
            self.log(f'ignoring {len(data)}B from {addr} (silence)')
            return
        try:
            frame = proto.CommandFrame.parse(data)
        except ValueError as e:
            self.emit(event='bad_frame', **{'from': f'{addr[0]}:{addr[1]}', 'error': str(e)})
            return
        if frame.cmd_type != REQ:
            return
        if kind == 'discovery' and frame.cmd_id != CMD_DISCOVERY:
            return
        if kind == 'cmd' and frame.cmd_id == CMD_DISCOVERY:
            return
        ret, payload = self._dispatch(frame, addr, now)
        self.emit(
            event='cmd',
            cmd_id=frame.cmd_id,
            seq=frame.seq_num,
            ret=ret,
            **{'from': f'{addr[0]}:{addr[1]}'},
        )
        if payload is None:
            return
        if self.drop_ack > 0:
            self.drop_ack -= 1
            self.emit(event='ack_dropped', cmd_id=frame.cmd_id, seq=frame.seq_num)
            return
        ack = proto.CommandFrame(frame.seq_num, frame.cmd_id, ACK, SENDER_LIDAR, payload).encode()
        sock.sendto(ack, addr)

    def _dispatch(self, f: proto.CommandFrame, addr, now: float) -> tuple[int, bytes | None]:
        now_ns = self.now_ns()
        m = self.model
        if f.cmd_id == CMD_DISCOVERY:
            sn = m.sn.encode().ljust(16, b'\0')[:16]
            ip = bytes(int(x) for x in self.lidar_ip().split('.'))
            return RET_OK, struct.pack(
                '<BB16s4sH', RET_OK, PROVISIONAL_DEV_TYPE, sn, ip, self.ports['cmd']
            )
        if f.cmd_id == CMD_PARAM_CONFIG:
            try:
                n, _ = struct.unpack_from('<HH', f.data, 0)
                kvs = proto.parse_kv_list(f.data[4:], n)
            except (struct.error, ValueError):
                return RET_FAIL, struct.pack('<BH', RET_FAIL, 0)
            ret, err = m.configure(kvs, now)
            if ret == RET_OK:
                self.log(f'configured {[hex(k) for k, _ in kvs]}')
            return ret, struct.pack('<BH', ret, err)
        if f.cmd_id == CMD_PARAM_INQUIRE:
            try:
                n, _ = struct.unpack_from('<HH', f.data, 0)
                keys = list(struct.unpack_from(f'<{n}H', f.data, 4))
            except struct.error:
                return RET_FAIL, struct.pack('<BH', RET_FAIL, 0)
            ret, kvs = m.inquire(keys, now_ns)
            # A rejected inquire names the offending key as a zero-length entry.
            return ret, struct.pack('<BH', ret, len(kvs)) + proto.encode_kv_list(kvs)
        if f.cmd_id == CMD_REBOOT:
            # The ACK is sent by the caller before the silence window is checked again.
            self._do_reboot(now)
            return RET_OK, struct.pack('<B', RET_OK)
        if f.cmd_id == CMD_FACTORY_RESET:
            m.factory_reset(now + self.args.reboot_silence)
            self.seq = 0
            self.udp_cnt_pcl = self.udp_cnt_imu = self.frame_cnt = 0
            self.silence_until = now + self.args.reboot_silence
            return RET_OK, struct.pack('<B', RET_OK)
        if f.cmd_id == CMD_SET_GPS_TIME:
            if len(f.data) < 9 or f.data[0] != 2:
                return RET_FAIL, struct.pack('<B', RET_FAIL)
            (ns,) = struct.unpack_from('<Q', f.data, 1)
            m.set_gps_time(ns, now_ns)
            return RET_OK, struct.pack('<B', RET_OK)
        return RET_FAIL, struct.pack('<B', RET_FAIL)  # unknown cmd_id

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
                if self.frame_s > 0 and now - self.frame_started >= self.frame_s:
                    self.frame_cnt = (self.frame_cnt + 1) & 0xFF
                    self.frame_started += self.frame_s
                self._send_pcl(pcl_host, interval)
                self.next_pcl += interval
                budget -= 1
            if now - self.next_pcl > 0.5:  # fell hopelessly behind: resync
                self.next_pcl = now
            if m.imu_enabled:
                imu_host = m.host(KEY_IMU_HOST)
                imu_interval = 1.0 / (m.imu_rate * self.rate)
                while now >= self.next_imu:
                    self._send_imu(imu_host, imu_interval)
                    self.next_imu += imu_interval
                if now - self.next_imu > 0.5:
                    self.next_imu = now
        if now >= self.next_push:
            self._send_push()
            self.next_push += 1.0 / self.push_rate
        if now >= self.next_stats:
            self.emit(event='sent', **self.sent, state=m.work_state)
            self.next_stats += 1.0

    def _cropped_samples(self, dt: int) -> list[tuple]:
        """
        Draw POINTS_PER_PACKET samples inside the enabled FOV windows.

        Up to MAX_FOV_DRAWS batches are drawn; a packet ends up shorter only for a tiny window.
        """
        m = self.model
        if not m.fov_windows():
            return self.points.samples(dt, POINTS_PER_PACKET)
        kept: list[tuple] = []
        for _ in range(MAX_FOV_DRAWS):
            kept.extend(
                p for p in self.points.samples(dt, POINTS_PER_PACKET) if m.keeps_point(dt, p)
            )
            if len(kept) >= POINTS_PER_PACKET:
                break
        return kept[:POINTS_PER_PACKET]

    def _send_pcl(self, host, interval_s: float) -> None:
        dt = self.model.pcl_data_type
        pkt = proto.DataPacket(
            time_interval=min(int(interval_s * 1e7), 0xFFFF),
            dot_num=POINTS_PER_PACKET,
            udp_cnt=self.udp_cnt_pcl,
            frame_cnt=self.frame_cnt,
            data_type=dt,
            time_type=self.model.time_sync_type,
            timestamp_ns=self.now_ns(),
            data=proto.pack_samples(dt, self._cropped_samples(dt)),
        )
        self.udp_cnt_pcl = (self.udp_cnt_pcl + 1) & 0xFFFF
        if host is None:
            return
        if self.drop_rate > 0 and self._drop_rng.random() < self.drop_rate:
            self.sent['pcl_dropped'] += 1
            return
        self._sendto('pcl', pkt.encode(), host)
        self.sent['pcl'] += 1

    def _send_imu(self, host, interval_s: float) -> None:
        pkt = proto.DataPacket(
            time_interval=min(int(interval_s * 1e7), 0xFFFF),
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
        self._sendto('imu', pkt.encode(), host)
        self.sent['imu'] += 1

    def _send_push(self) -> None:
        host = self.model.host(KEY_STATE_HOST)
        if host is None:
            return
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        frame = proto.CommandFrame(
            self.seq, CMD_INFO_PUSH, REQ, SENDER_LIDAR, self.model.push_payload(self.now_ns())
        ).encode()
        self._sendto('push', frame, host)
        self.sent['push'] += 1

    def _sendto(self, kind: str, data: bytes, host: tuple[str, int, int]) -> None:
        try:
            self.socks[kind].sendto(data, (host[0], host[1]))
        except OSError as e:
            self.log(f'send {kind} to {host[:2]} failed: {e}')


# --------------------------------------------------------------------------- CLI
def parse_version(text: str) -> tuple[int, int, int, int]:
    """'a.b.c.d' -> 4 bytes (keys 0x8002-0x8004)."""
    parts = [int(x) for x in text.split('.')]
    if len(parts) != 4 or not all(0 <= x <= 255 for x in parts):
        raise argparse.ArgumentTypeError(f'version must be a.b.c.d with 0-255 each: {text!r}')
    return parts[0], parts[1], parts[2], parts[3]


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description='Livox Mid-360 simulator')
    p.add_argument('--bind', default='0.0.0.0', help='address to bind (default 0.0.0.0)')
    p.add_argument(
        '--base-port',
        type=int,
        default=proto.PORT_DISCOVERY,
        help='discovery port; cmd/push/pcl/imu follow at +100..+400. 0 = pick free ports',
    )
    p.add_argument('--sn', default='SIM0000000000001')
    p.add_argument('--product-info', default='MID360-SIM', help='key 0x8001 (<= 64 chars)')
    p.add_argument('--version-app', default='0.0.0.1', help='key 0x8002 as a.b.c.d')
    p.add_argument('--version-loader', default='0.0.0.1', help='key 0x8003 as a.b.c.d')
    p.add_argument('--version-hardware', default='0.0.0.1', help='key 0x8004 as a.b.c.d')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument(
        '--startup-delay', type=float, default=0.3, help='seconds spent in MOTORSTARTUP'
    )
    p.add_argument(
        '--selfcheck-delay',
        type=float,
        default=0.1,
        help='seconds spent in SELFCHECK after power-on / reboot',
    )
    p.add_argument(
        '--reboot-silence', type=float, default=0.5, help='seconds of silence after 0x0200'
    )
    p.add_argument(
        '--frame-ms',
        type=float,
        default=100.0,
        help='frame_cnt period; 0 = frame_cnt never changes (non-repetitive scan)',
    )
    p.add_argument('--rate-multiplier', type=float, default=1.0)
    p.add_argument(
        '--push-rate',
        type=float,
        default=PUSH_RATE,
        help='0x0102 push rate in Hz, independent of --rate-multiplier (default 1)',
    )
    p.add_argument(
        '--drop-rate', type=float, default=0.0, help='fraction of point-cloud packets to drop'
    )
    p.add_argument(
        '--imu-cfg-unsupported',
        action='store_true',
        help='emulate firmware without key 0x002B (write and read answered with 0x20)',
    )
    p.add_argument('--quit-on-eof', action='store_true', default=True)
    p.add_argument('--no-quit-on-eof', dest='quit_on_eof', action='store_false')
    p.add_argument('--verbose', '-v', action='store_true')
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if len(args.sn) > 16:
        sys.exit('--sn must be at most 16 characters')
    sim = Simulator(args)
    try:
        sim.run()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())

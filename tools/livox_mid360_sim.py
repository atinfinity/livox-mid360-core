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
  * dev_type in the discovery ACK is a provisional value,
  * losing time synchronisation leaves the clock where it was (no step back, #133).
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
import json
import math
import os
import pathlib
import random
import selectors
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import livox_mid360_pcap as pcapfile  # noqa: E402
import livox_mid360_proto as proto  # noqa: E402

# --------------------------------------------------------------------------- constants
CMD_DISCOVERY, CMD_PARAM_CONFIG, CMD_PARAM_INQUIRE, CMD_INFO_PUSH = 0x0000, 0x0100, 0x0101, 0x0102
CMD_REBOOT, CMD_FACTORY_RESET, CMD_SET_GPS_TIME = 0x0200, 0x0201, 0x0202
CMD_PUSH_LOG, CMD_COLLECTION_LOG = 0x0300, 0x0301  # firmware log (#44), LiDAR port 56500
CMD_DEBUG_DATA = 0x0303  # debug raw data (#93), accepted on the log and the command port
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
KEY_LOG_HOST = 0x0009
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
    KEY_LOG_HOST: 8,
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
CATCH_UP_SLICE_S = 0.02  # longest point-cloud burst between two select() calls (#183)
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
        KEY_LOG_HOST: zero(8),
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


LOG_TYPES = (0, 1)  # 0 realtime, 1 exception
LOG_FLAG_ACK, LOG_FLAG_BEGIN, LOG_FLAG_END = 0x01, 0x02, 0x04


@dataclass
class LogStream:
    """One enabled firmware log type: file_index / trans_index as the LiDAR would count."""

    log_type: int
    file_index: int = 1
    trans_index: int = 0  # last sent; 0 = the next chunk begins the file
    requester: tuple[str, int] | None = None

    def new_file(self) -> None:
        self.file_index = (self.file_index + 1) & 0xFF
        self.trans_index = 0


DATA_STREAMS = ('pcl', 'imu')


@dataclass
class PacketFaults:
    """
    Pending network faults of one data stream (#131), applied after udp_cnt is assigned.

    Priority per packet: drop, then hold back (reorder), then send (twice for a duplicate).
    A held packet is released after `depth` more packets have been sent.
    """

    drop: int = 0  # the next N packets are not sent
    reorder: int = 0  # packets still to hold back, one at a time
    depth: int = 1
    duplicate: int = 0  # the next N sent packets go out twice
    held: tuple[bytes, int, int] | None = None  # (datagram, udp_cnt, packets still to pass)


TIME_SYNC_NONE, TIME_SYNC_PTP, TIME_SYNC_GPS = 0, 1, 2  # key 0x800C / data packet time_type
TIME_SYNC_TYPES = {'none': TIME_SYNC_NONE, 'ptp': TIME_SYNC_PTP, 'gps': TIME_SYNC_GPS}
TIME_SYNC_NAMES = {v: k for k, v in TIME_SYNC_TYPES.items()}


@dataclass
class LidarClock:
    """
    The LiDAR's clock (#133): the time in data packets and in keys 0x8009-0x800C.

    Free-running (type none) it counts from power-on at 1 + drift_ppm / 1e6 of the host's
    rate. Synchronised (PTP or GPS) it follows the master, the host's wall clock plus
    `master_offset_ns`, stepping to it when synchronisation is acquired. Losing it falls
    back to free running from where the master left the clock [unverified, #109].
    Times are passed in: `mono` is time.monotonic() seconds, `wall_ns` time.time_ns().
    """

    drift_ppm: float = 0.0
    sync_type: int = TIME_SYNC_NONE
    master_offset_ns: int = 0  # master - host wall clock, while synchronised
    last_sync_ns: int = 0  # 0x800A: the master time the clock last stepped to, 0 = never
    offset_ns: int = 0  # 0x800B: local - source at that step
    anchor_mono: float = 0.0  # the free-running clock read anchor_ns at anchor_mono
    anchor_ns: int = 0

    def power_on(self, mono: float) -> None:
        """Start counting from zero, unsynchronised; the drift is the oscillator's and stays."""
        self.sync_type = TIME_SYNC_NONE
        self.master_offset_ns = self.last_sync_ns = self.offset_ns = 0
        self.anchor_mono, self.anchor_ns = mono, 0

    def free_ns(self, mono: float) -> int:
        elapsed = (mono - self.anchor_mono) * 1e9 * (1.0 + self.drift_ppm * 1e-6)
        return max(0, self.anchor_ns + round(elapsed))

    def now_ns(self, mono: float, wall_ns: int) -> int:
        if self.sync_type != TIME_SYNC_NONE:
            return wall_ns + self.master_offset_ns
        return self.free_ns(mono)

    def sync(self, sync_type: int, master_offset_ns: int, mono: float, wall_ns: int) -> None:
        """Acquire (or re-acquire) synchronisation: step to the master's time."""
        local = self.now_ns(mono, wall_ns)
        self.sync_type, self.master_offset_ns = sync_type, master_offset_ns
        self.last_sync_ns = wall_ns + master_offset_ns
        self.offset_ns = local - self.last_sync_ns

    def lose_sync(self, mono: float, wall_ns: int) -> None:
        """Fall back to free running without a step; 0x800A / 0x800B keep the last sync."""
        if self.sync_type != TIME_SYNC_NONE:
            self.anchor_mono, self.anchor_ns = mono, self.now_ns(mono, wall_ns)
            self.sync_type = TIME_SYNC_NONE

    def set_drift(self, ppm: float, mono: float) -> None:
        """Change the free-running rate from now on, without a step."""
        if not -1e6 < ppm < 1e6:
            raise ValueError(f'drift_ppm {ppm} out of (-1e6, 1e6)')
        self.anchor_mono, self.anchor_ns = mono, self.free_ns(mono)
        self.drift_ppm = ppm

    def describe(self, mono: float, wall_ns: int) -> dict:
        return {
            'type': TIME_SYNC_NAMES[self.sync_type],
            'time_ns': self.now_ns(mono, wall_ns),
            'last_sync_ns': self.last_sync_ns,
            'offset_ns': self.offset_ns,
            'drift_ppm': self.drift_ppm,
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
    # Key 0x0004 restored by a factory reset; None keeps factory_settings()'s 192.168.1.100.
    factory_lidar_ipcfg: bytes | None = None
    settings: dict[int, bytes] = field(default_factory=factory_settings)
    work_state: int = WS_SELFCHECK
    hms: list[int] = field(default_factory=lambda: [0] * 8)
    clock: LidarClock = field(default_factory=LidarClock)
    powerup_cnt: int = 1
    diag_status: int = 0
    core_temp: int = 3500  # 0.01 degC
    push_omit: set[int] = field(default_factory=set)  # keys left out of the push (tests)
    bad_time_offset: int = 0  # 1: answer 0x800B truncated to 4 bytes (tests)
    # key -> value the inquiry answers instead of the real one, or 'omit' / 'unsupported' (tests)
    inquire_overrides: dict[int, bytes | str] = field(default_factory=dict)
    state_deadline: float = 0.0  # monotonic time at which the timed state completes
    on_state: Callable[[int, int], None] | None = None

    # -- lifecycle ---------------------------------------------------------
    def power_on(self, now: float) -> None:
        self.clock.power_on(now)
        self._enter_timed(WS_SELFCHECK, now)

    def reboot(self, now: float) -> None:
        self.powerup_cnt += 1
        self.settings[KEY_WORK_TGT_MODE] = bytes([WS_SAMPLING])  # only tgt mode is not persisted
        self.power_on(now)

    def factory_reset(self, now: float) -> None:
        self.settings = factory_settings()
        if self.factory_lidar_ipcfg is not None:
            self.settings[KEY_LIDAR_IPCFG] = self.factory_lidar_ipcfg
        self.hms = [0] * 8
        self.reboot(now)

    # -- work-state machine ------------------------------------------------
    @property
    def timed(self) -> bool:
        """Return whether the current state completes at ``state_deadline``."""
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
            if key == KEY_FUNC_IO and (
                value[0] != 0 or value[1] != 0 or value[2] > 2 or value[3] > 2
            ):
                # [unverified] IN0 / IN1 have a single defined function each (#11, #52).
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
            rule = self.inquire_overrides.get(key)
            if rule == 'omit':
                continue
            v = None if rule == 'unsupported' else self.read_key(key, now_ns)
            if isinstance(rule, bytes):
                v = rule
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
            KEY_CORE_TEMP: struct.pack('<i', self.core_temp),
            KEY_POWERUP_CNT: struct.pack('<I', self.powerup_cnt),
            KEY_LOCAL_TIME: struct.pack('<Q', now_ns),
            KEY_LAST_SYNC_TIME: struct.pack('<Q', self.clock.last_sync_ns),
            KEY_TIME_OFFSET: struct.pack('<q', self.clock.offset_ns)[
                : 4 if self.bad_time_offset else 8
            ],
            KEY_TIME_SYNC_TYPE: bytes([self.clock.sync_type]),
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
        kvs = [(k, self.read_key(k, now_ns)) for k in keys if k not in self.push_omit]
        return struct.pack('<HH', len(kvs), 0) + proto.encode_kv_list(kvs)

    def set_gps_time(self, ns: int, mono: float, wall_ns: int) -> None:
        """0x0202: `ns` is the GPS time of the PPS edge, taken to be now."""
        self.clock.sync(TIME_SYNC_GPS, ns - wall_ns, mono, wall_ns)

    # Control command `set_status`: any subset of the read-only status fields.
    STATUS_FIELDS = {
        'diag': 'diag_status',
        'core_temp': 'core_temp',
        'bad_time_offset': 'bad_time_offset',
        'powerup_cnt': 'powerup_cnt',
    }

    def set_status(self, fields: dict) -> list[str]:
        """
        Apply the known fields and return the names of the unknown ones.

        Every value is converted first, so a bad one (ValueError / TypeError) applies nothing.
        """
        unknown = [n for n in fields if n != 'omit_keys' and n not in self.STATUS_FIELDS]
        omit = {int(k) for k in fields['omit_keys']} if 'omit_keys' in fields else None
        values = {
            self.STATUS_FIELDS[n]: int(v) for n, v in fields.items() if n in self.STATUS_FIELDS
        }
        if omit is not None:
            self.push_omit = omit
        for attr, value in values.items():
            setattr(self, attr, value)
        return unknown

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

    def install_attitude(self) -> Attitude:
        """Key 0x0012 as (roll, pitch, yaw) in float degrees and (x, y, z) in mm."""
        return struct.unpack('<fffiii', self.settings[KEY_INSTALL_ATTITUDE])

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
            refl = r.randint(0, 255)
            # glue / particles / other each 0..2 (high / medium / low), reserved bits 0 (#34)
            tag = r.randint(0, 2) | (r.randint(0, 2) << 2) | (r.randint(0, 2) << 4)
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


# Deterministic scene (#132): RING_POINTS points at known angles and depths, emitted in index
# order from index 0 at every frame_cnt change and repeated within the frame.
SCENES = ('random', 'ring')
RING_POINTS = 256
RING_PITCH_DEG = (0, 15, -5, 45)  # elevation of point k is RING_PITCH_DEG[k % 4]


def ring_point(k: int) -> tuple[int, int, int, int, int]:
    """
    Return point k of the ring scene as a spherical sample (depth mm, theta, phi, refl, tag).

    theta is the zenith angle and phi the azimuth, both in 0.01 deg. Azimuth steps by 360/256
    deg (k = 0, 64, 128, 192 lie on the axes), depth is 1 m + 30 mm * k, reflectivity is k and
    the tag cycles glue / particles / other through 0..2 (reserved bits 0).
    """
    k %= RING_POINTS
    tag = k % 3 | (k // 3 % 3) << 2 | (k // 9 % 3) << 4
    return 1000 + 30 * k, 9000 - 100 * RING_PITCH_DEG[k % 4], k * 36000 // RING_POINTS, k, tag


def ring_sample(data_type: int, k: int, attitude: Attitude | None = None) -> tuple:
    """
    Point k of the ring scene encoded as `data_type` (Cartesian rounded to mm or cm).

    A Cartesian point is moved by `attitude` before it is rounded; spherical ones never are.
    """
    depth, theta, phi, refl, tag = ring_point(k)
    if data_type == 3:
        return depth, theta, phi, refl, tag
    t, p = math.radians(theta / 100), math.radians(phi / 100)
    r = depth * math.sin(t)
    xyz = (r * math.cos(p), r * math.sin(p), depth * math.cos(t))
    if attitude is not None:
        xyz = transform_mm(attitude, xyz)
    return encode_cartesian(data_type, xyz, refl, tag)


# --------------------------------------------------------------------------- install attitude
# Key 0x0012 as stored: roll, pitch, yaw (float deg), x, y, z (int32 mm).
Attitude = tuple[float, float, float, int, int, int]


def transform_mm(attitude: Attitude, xyz: tuple[float, float, float]) -> tuple[float, ...]:
    """
    Rotate a point in mm by Rz(yaw) * Ry(pitch) * Rx(roll), then add the translation.

    The same convention as the SDK's extrinsic_from() (#135); whether the firmware applies key
    0x0012 at all, and how, is [unverified] (#11).
    """
    roll, pitch, yaw, tx, ty, tz = attitude
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    x, y, z = xyz
    return (
        cy * cp * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z + tx,
        sy * cp * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z + ty,
        -sp * x + cp * sr * y + cp * cr * z + tz,
    )


def encode_cartesian(data_type: int, xyz_mm: tuple[float, ...], refl: int, tag: int) -> tuple:
    """Round a point in mm to a Cartesian32 (mm) or Cartesian16 (cm) sample, clamped to range."""
    scale, lim = (1, 2**31 - 1) if data_type == 1 else (10, 2**15 - 1)
    return (*(max(-lim - 1, min(lim, round(v / scale))) for v in xyz_mm), refl, tag)


def attitude_sample(data_type: int, sample: tuple, attitude: Attitude) -> tuple:
    """Move a Cartesian sample by `attitude`; return a spherical one unchanged."""
    if data_type == 3:
        return sample
    scale = 1 if data_type == 1 else 10
    xyz = transform_mm(attitude, tuple(v * scale for v in sample[:3]))
    return encode_cartesian(data_type, xyz, *sample[3:])


# --------------------------------------------------------------------------- simulator
# --------------------------------------------------------------------------- pcap replay
# LiDAR source port of a replayed datagram -> (stream, key naming its destination) (#134).
REPLAY_STREAMS = {
    proto.PORT_PUSH: ('push', KEY_STATE_HOST),
    proto.PORT_PCL: ('pcl', KEY_PCL_HOST),
    proto.PORT_IMU: ('imu', KEY_IMU_HOST),
    proto.PORT_LOG: ('log', KEY_LOG_HOST),
}
REPLAY_STREAMS_BY_NAME = dict(REPLAY_STREAMS.values())  # stream -> destination key


def replay_datagrams(path: pathlib.Path) -> Iterator[tuple[float, str, bytes]]:
    """
    Yield (capture time, stream, UDP payload) for each datagram sent from a LiDAR data port.

    Command traffic (discovery, 0x0100 and their ACKs) and anything else in the capture is
    skipped: the DeviceModel answers the SDK's own commands.
    """
    for ts, linktype, frame in pcapfile.iter_pcap(path):
        u = pcapfile.parse_udp(linktype, frame)
        if u is not None and u.sport in REPLAY_STREAMS:
            yield ts, REPLAY_STREAMS[u.sport][0], u.payload


class PcapReplay:
    """One pass over a capture, datagram by datagram at the recorded spacing scaled by `rate`."""

    def __init__(self, path: pathlib.Path, rate: float) -> None:
        self.path = path
        self.rate = rate  # 0 = as fast as possible
        self.state = 'waiting'  # -> 'running' -> 'done'
        self.sent = {stream: 0 for stream, _ in REPLAY_STREAMS.values()}
        self.skipped = 0  # no host configured for the stream
        self.started = 0.0
        self._datagrams: Iterator[tuple[float, str, bytes]] = iter(())
        self._next: tuple[float, str, bytes] | None = None
        self._first_ts = 0.0

    def start(self, now: float) -> None:
        self.state = 'running'
        self.started = now
        self._datagrams = replay_datagrams(self.path)
        self._next = next(self._datagrams, None)
        self._first_ts = self._next[0] if self._next is not None else 0.0

    def due(self) -> float:
        """Return the monotonic time the next datagram is due at (inf when none is pending)."""
        if self.state != 'running' or self._next is None:
            return math.inf
        if self.rate == 0:
            return self.started
        return self.started + max(0.0, self._next[0] - self._first_ts) / self.rate

    def pop(self, now: float) -> tuple[str, bytes] | None:
        """Return the next datagram as (stream, payload) if it is due at `now`."""
        if self._next is None or self.due() > now:
            return None
        _, stream, payload = self._next
        self._next = next(self._datagrams, None)
        return stream, payload

    @property
    def exhausted(self) -> bool:
        return self.state == 'running' and self._next is None


class Simulator:
    """Sockets, threads and JSON control channel around one DeviceModel."""

    def __init__(self, args: argparse.Namespace, out=sys.stdout, control=sys.stdin) -> None:
        self.args = args
        self.out = out
        self.control = control
        self.control_buf = b''
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
        self.bound_ip: str | None = None  # set by a reboot that moved the LiDAR (#50)
        self.pending_rebind: str | None = None
        # Key 0x0004 reports the address the simulator actually answers from.
        # A factory reset returns to this address: the simulator cannot move to 192.168.1.100.
        own = bytes(int(x) for x in self.lidar_ip().split('.'))
        self.model.settings[KEY_LIDAR_IPCFG] = own + self.model.settings[KEY_LIDAR_IPCFG][4:]
        self.model.factory_lidar_ipcfg = self.model.settings[KEY_LIDAR_IPCFG]
        self.points = PointSource(args.seed)
        self.scene = args.scene
        self.apply_attitude = args.apply_attitude
        self.ring_next = 0  # index of the next ring point (#132); 0 at every frame_cnt change
        self.rate = args.rate_multiplier
        self.push_rate = args.push_rate
        self.frame_s = args.frame_ms / 1000.0
        self.drop_rate = args.drop_rate
        self._drop_rng = random.Random(args.seed ^ 0x5A5A)
        self.faults = {kind: PacketFaults() for kind in DATA_STREAMS}
        # --pcap (#134): the capture's data streams replace the generated point cloud and IMU.
        self.replay = (
            PcapReplay(pathlib.Path(args.pcap), args.pcap_rate) if args.pcap is not None else None
        )

        self.seq = 0  # LiDAR-originated frames (push)
        self.udp_cnt_pcl = 0
        self.udp_cnt_imu = 0
        self.frame_cnt = 0
        self.frame_started = 0.0
        self.next_pcl = self.next_imu = self.next_push = self.next_stats = 0.0
        self.sent = {
            'pcl': 0,
            'imu': 0,
            'push': 0,
            'pcl_dropped': 0,  # --drop-rate and the drop control
            'pcl_reordered': 0,
            'pcl_duplicated': 0,
            'imu_dropped': 0,
            'imu_reordered': 0,
            'imu_duplicated': 0,
            'log': 0,
            'debug': 0,
            'silenced': 0,  # datagrams generated but withheld by `silence` / reboot silence
        }
        # Firmware log collection (#44): one stream per log_type (0 realtime, 1 exception).
        self.log_streams: dict[int, LogStream] = {}
        self.log_drop = 0  # chunks to skip (trans_index still advances) -> gap on the host
        self.log_acks_received = 0
        self.next_log = 0.0
        # Debug raw data (#93): destination from the last 0x0303 enable, None while disabled.
        self.debug_dest: tuple[str, int] | None = None
        self.debug_sock: socket.socket | None = None  # opened by the first enable
        self.debug_seq = 0
        self.next_debug = 0.0
        self.silence_until = 0.0  # nothing is received or sent before this (link drop, reboot)
        self.drop_ack = 0
        # fail_cmd control: cmd_id -> {'skip', 'count', 'ret', 'key'}
        self.fail_cmds: dict[int, dict] = {}
        self.running = True

        self.sel = selectors.DefaultSelector()
        self.socks: dict[str, socket.socket] = {}
        self.ports: dict[str, int] = {}
        self._open_sockets()

    # -- setup ---------------------------------------------------------------
    def _open_sockets(self) -> None:
        base = self.args.base_port
        names = ['discovery', 'cmd', 'push', 'pcl', 'imu', 'log']
        for i, name in enumerate(names):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if base != 0:
                # Never on an ephemeral port: Linux would then consider a port held by
                # another SO_REUSEADDR socket of the same user free and could hand it out.
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.setblocking(False)
            s.bind((self.args.bind, 0 if base == 0 else base + 100 * i))
            self.socks[name] = s
            self.ports[name] = s.getsockname()[1]
        self.sel.register(self.socks['discovery'], selectors.EVENT_READ, 'discovery')
        self.sel.register(self.socks['cmd'], selectors.EVENT_READ, 'cmd')
        self.sel.register(self.socks['log'], selectors.EVENT_READ, 'log')
        if self.control is not None:
            try:
                self.sel.register(self.control, selectors.EVENT_READ, 'control')
            except (ValueError, OSError):
                self.control = None

    def lidar_ip(self) -> str:
        if self.bound_ip is not None:
            return self.bound_ip
        if self.args.bind not in ('', '0.0.0.0'):
            return self.args.bind
        return '127.0.0.1'

    def advertised_ip(self, peer: tuple[str, int]) -> str:
        """
        lidar_ip of the discovery ACK sent to `peer`.

        Bound to a wildcard address, the simulator answers from whichever local address routes
        to the requester, so it advertises that one; 127.0.0.1 would only be reachable on
        loopback (#203). Key 0x0004 still reports lidar_ip().
        """
        if self.bound_ip is not None or self.args.bind not in ('', '0.0.0.0'):
            return self.lidar_ip()
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(peer)  # looks up the route; nothing is sent
                local = s.getsockname()[0]
        except OSError:
            return self.lidar_ip()
        return local if local != '0.0.0.0' else self.lidar_ip()

    def configured_ip(self) -> str:
        return '.'.join(str(b) for b in self.model.settings[KEY_LIDAR_IPCFG][:4])

    def _apply_pending_rebind(self) -> None:
        """Rebind to the address a reboot scheduled, if any."""
        if self.pending_rebind is not None:
            ip, self.pending_rebind = self.pending_rebind, None
            self._rebind(ip)

    def _rebind(self, ip: str) -> bool:
        """Move every socket to `ip` keeping the port numbers (a reboot after a 0x0004 write)."""
        fresh: dict[str, socket.socket] = {}
        try:
            for name in self.socks:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                if self.args.base_port != 0:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.setblocking(False)
                s.bind((ip, self.ports[name]))
                fresh[name] = s
        except OSError as e:
            for s in fresh.values():
                s.close()
            self.emit(event='error', error=f'rebind to {ip} failed: {e}')
            return False
        for name in ('discovery', 'cmd'):
            self.sel.unregister(self.socks[name])
        for old in self.socks.values():
            old.close()
        self.socks = fresh
        if self.debug_sock is not None:  # reopened on the new address by the next datagram
            self.debug_sock.close()
            self.debug_sock = None
        self.sel.register(self.socks['discovery'], selectors.EVENT_READ, 'discovery')
        self.sel.register(self.socks['cmd'], selectors.EVENT_READ, 'cmd')
        self.bound_ip = ip
        self.emit(event='rebound', ip=ip, ports=self.ports)
        return True

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
        if self.log_streams:
            d.append(self.next_log)
        if self.debug_dest is not None:
            d.append(self.next_debug)
        if self.model.timed:
            d.append(self.model.state_deadline)
        if self.replay is not None:
            d.append(self.replay.due())
        elif self.model.sampling:
            d.append(self.next_pcl)
            if self.model.imu_enabled:
                d.append(self.next_imu)
        return min(d)

    # -- control channel -------------------------------------------------------
    def _handle_control(self) -> None:
        # Read raw bytes rather than readline(): a buffered TextIOWrapper would swallow
        # several lines in one read, and select() never fires again for the buffered rest.
        try:
            data = os.read(self.control.fileno(), 65536)
        except OSError:
            data = b''
        if not data:  # EOF: parent went away
            self.sel.unregister(self.control)
            self.control = None
            if self.args.quit_on_eof:
                self.running = False
            return
        self.control_buf += data
        while b'\n' in self.control_buf:
            raw, _, self.control_buf = self.control_buf.partition(b'\n')
            line = raw.decode('utf-8', 'replace').strip()
            if not line:
                continue
            try:
                req = json.loads(line)
            except json.JSONDecodeError as e:
                self.emit(event='error', error=f'bad json: {e}')
                continue
            self.apply_control(req)

    def apply_control(self, req: object) -> None:
        """Apply one control line; a malformed one emits `error` instead of stopping the loop."""
        if not isinstance(req, dict):
            self.emit(event='error', error=f'control line is not an object: {req!r}')
            return
        try:
            self._apply_control(req)
        except (KeyError, ValueError, TypeError, AttributeError) as e:
            self.emit(event='error', error=f'bad control {req.get("cmd")!r}: {e!r}')

    def _apply_control(self, req: dict) -> None:
        cmd = req.get('cmd')
        now = time.monotonic()
        if cmd == 'quit':
            self.running = False
        elif cmd == 'silence':
            # A link drop: the LiDAR keeps running (udp_cnt, push seq, trans_index advance).
            self.silence_until = max(self.silence_until, now + float(req.get('seconds', 1.0)))
        elif cmd == 'hms':
            codes = [int(c) for c in req.get('codes', [])][:8]
            self.model.hms = (codes + [0] * 8)[:8]
        elif cmd == 'set_status':
            unknown = self.model.set_status({k: v for k, v in req.items() if k != 'cmd'})
            if unknown:
                self.emit(event='error', error=f'unknown status fields: {unknown}')
                return
        elif cmd == 'drop_ack':
            self.drop_ack += int(req.get('count', 1))
        elif cmd == 'fail_cmd':
            key = req.get('key')
            self.fail_cmds[int(req['cmd_id'])] = {
                'skip': int(req.get('skip', 0)),
                'count': int(req.get('count', 1)),
                'ret': int(req.get('ret', RET_FAIL)),
                'key': None if key is None else int(key),
            }
        elif cmd == 'inquire_override':
            key = int(req['key'])
            if req.get('clear'):
                self.model.inquire_overrides.pop(key, None)
            elif req.get('omit'):
                self.model.inquire_overrides[key] = 'omit'
            elif req.get('unsupported'):
                self.model.inquire_overrides[key] = 'unsupported'
            else:
                self.model.inquire_overrides[key] = bytes.fromhex(req.get('value', ''))
        elif cmd == 'log_drop':
            self.log_drop += int(req.get('n', 1))
        elif cmd == 'log_new_file':
            for stream in self.log_streams.values():
                self._send_log_chunk(stream, end=True)
                stream.new_file()
        elif cmd == 'time_sync':
            self._time_sync(req, now)
        elif cmd == 'reboot':
            self._do_reboot(now)
            self._apply_pending_rebind()
        elif cmd == 'set_state':
            state = int(req['state'])
            if state not in WS_ALL:
                raise ValueError(f'unknown work state {state}')
            self.model.force_state(state, now)
        elif cmd == 'drop_rate':
            self.drop_rate = float(req.get('rate', 0.0))
        elif cmd in ('drop', 'reorder', 'duplicate'):
            self._add_fault(cmd, req)
        elif cmd == 'scene':
            name = req.get('name')
            if name not in SCENES:
                raise ValueError(f'unknown scene {name!r}')
            self.scene = name
            self.ring_next = 0
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
                    'log': self.model.host(KEY_LOG_HOST),
                },
                log_enabled=sorted(self.log_streams),
                log_acks_received=self.log_acks_received,
                debug_data={
                    'enabled': self.debug_dest is not None,
                    'dest': self.debug_dest,
                    'port': self.debug_sock.getsockname()[1] if self.debug_sock else None,
                },
                replay=None
                if self.replay is None
                else {
                    'state': self.replay.state,
                    'sent': self.replay.sent,
                    'skipped': self.replay.skipped,
                },
                time=self.model.clock.describe(now, time.time_ns()),
            )
        else:
            self.emit(event='error', error=f'unknown control cmd: {cmd!r}')
            return
        self.emit(event='control', cmd=cmd)

    def _add_fault(self, cmd: str, req: dict) -> None:
        """Validate and queue a drop / reorder / duplicate control (#131)."""
        stream = req.get('stream', 'pcl')
        if stream not in DATA_STREAMS:
            raise ValueError(f'unknown stream {stream!r}')
        count = int(req.get('count', 1))
        if count < 0:
            raise ValueError(f'negative count {count}')
        f = self.faults[stream]
        if cmd == 'drop':
            f.drop += count
        elif cmd == 'duplicate':
            f.duplicate += count
        else:
            depth = int(req.get('depth', 1))
            if depth < 1:
                raise ValueError(f'depth must be at least 1, got {depth}')
            f.reorder += count
            f.depth = depth

    def _time_sync(self, req: dict, now: float) -> None:
        """Control `time_sync`: acquire or lose synchronisation, and set the drift (#133)."""
        kind = req.get('type')
        if kind is not None and kind not in TIME_SYNC_TYPES:
            raise ValueError(f'unknown time sync type {kind!r}')
        offset = int(req.get('offset_ns', 0))
        drift = float(req['drift_ppm']) if 'drift_ppm' in req else None
        clock = self.model.clock
        if drift is not None:
            clock.set_drift(drift, now)  # validates before anything changes
        wall = time.time_ns()
        if kind == 'none':
            clock.lose_sync(now, wall)
        elif kind is not None:
            clock.sync(TIME_SYNC_TYPES[kind], offset, now, wall)
        self.emit(event='time_sync', **clock.describe(now, wall))

    def _do_reboot(self, now: float, factory: bool = False) -> None:
        """0x0200 reboot, or 0x0201 factory reset when `factory` (settings back to defaults)."""
        self.seq = 0
        self.udp_cnt_pcl = self.udp_cnt_imu = 0
        self.frame_cnt = 0
        self.ring_next = 0
        up = now + self.args.reboot_silence
        self.silence_until = max(self.silence_until, up)
        # Powered down: the push and the log chunks resume after the silence, not during it.
        self.next_push = max(self.next_push, up)
        self.next_log = max(self.next_log, up)
        if factory:
            self.model.factory_reset(up)
        else:
            self.model.reboot(up)
        self.debug_dest = None  # [unverified] assumed not to survive a reboot
        for f in self.faults.values():
            f.held = None  # still in the LiDAR when it powered down
        self.debug_seq = 0
        # A changed 0x0004 takes effect now [unverified: the wiki only says "after reboot"].
        # Deferred so the reboot ACK still leaves the old socket.
        if self.configured_ip() != self.lidar_ip():
            self.pending_rebind = self.configured_ip()

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
        if kind == 'log':
            if frame.cmd_id == CMD_PUSH_LOG:  # host ACK for a pushed chunk (REQ 0x0300)
                self._on_log_ack(frame, addr)
                return
            if frame.cmd_id not in (CMD_COLLECTION_LOG, CMD_DEBUG_DATA):
                return
        failed = self._injected_failure(frame)
        ret, payload = failed if failed else self._dispatch(frame, addr, now)
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
        self._apply_pending_rebind()

    def _injected_failure(self, f: proto.CommandFrame) -> tuple[int, bytes] | None:
        """Answer of a request the `fail_cmd` control rejects; the request is not applied."""
        rule = self.fail_cmds.get(f.cmd_id)
        if rule is None:
            return None
        key = 0
        if rule['key'] is not None:
            key = int(rule['key'])
            try:
                n, _ = struct.unpack_from('<HH', f.data, 0)
                if f.cmd_id == CMD_PARAM_CONFIG:
                    keys = [k for k, _ in proto.parse_kv_list(f.data[4:], n)]
                else:
                    keys = list(struct.unpack_from(f'<{n}H', f.data, 4))
            except (struct.error, ValueError):
                return None
            if key not in keys:
                return None
        if rule['skip'] > 0:
            rule['skip'] -= 1
            return None
        rule['count'] -= 1
        if rule['count'] <= 0:
            del self.fail_cmds[f.cmd_id]
        ret = rule['ret']
        if f.cmd_id in (CMD_PARAM_CONFIG, CMD_PARAM_INQUIRE):
            return ret, struct.pack('<BH', ret, key if f.cmd_id == CMD_PARAM_CONFIG else 0)
        return ret, struct.pack('<B', ret)

    def _dispatch(self, f: proto.CommandFrame, addr, now: float) -> tuple[int, bytes | None]:
        now_ns = self.now_ns()
        m = self.model
        if f.cmd_id == CMD_DISCOVERY:
            sn = m.sn.encode().ljust(16, b'\0')[:16]
            ip = bytes(int(x) for x in self.advertised_ip(addr[:2]).split('.'))
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
            self._do_reboot(now, factory=True)
            return RET_OK, struct.pack('<B', RET_OK)
        if f.cmd_id == CMD_SET_GPS_TIME:
            if len(f.data) < 9 or f.data[0] != 2:
                return RET_FAIL, struct.pack('<B', RET_FAIL)
            (ns,) = struct.unpack_from('<Q', f.data, 1)
            m.set_gps_time(ns, now, time.time_ns())
            return RET_OK, struct.pack('<B', RET_OK)
        if f.cmd_id == CMD_COLLECTION_LOG:
            if len(f.data) < 2 or f.data[0] not in LOG_TYPES:
                return RET_FAIL, struct.pack('<B', RET_FAIL)
            self._log_control(f.data[0], f.data[1] != 0, addr)
            return RET_OK, struct.pack('<B', RET_OK)
        if f.cmd_id == CMD_DEBUG_DATA:
            req = proto.parse_debug_data_control(f.data)
            if req is None or not self._debug_control(req[0], (req[1], req[2])):
                return RET_FAIL, struct.pack('<B', RET_FAIL)
            return RET_OK, struct.pack('<B', RET_OK)
        return RET_FAIL, struct.pack('<B', RET_FAIL)  # unknown cmd_id

    # -- firmware log (#44) --------------------------------------------------
    def _log_control(self, log_type: int, enable: bool, addr) -> None:
        stream = self.log_streams.get(log_type)
        if enable:
            if stream is None:
                if not self.log_streams:  # the first enabled type starts the chunk clock
                    self.next_log = time.monotonic() + self.args.log_chunk_interval
                stream = LogStream(log_type=log_type)
                self.log_streams[log_type] = stream
            stream.requester = addr  # fallback destination when key 0x0009 is unset
            self.log(f'log type {log_type} enabled for {addr}')
        elif stream is not None:
            self._send_log_chunk(stream, end=True)
            del self.log_streams[log_type]
            self.log(f'log type {log_type} disabled')

    def _log_dest(self, stream: LogStream) -> tuple[str, int] | None:
        if not self.args.log_ignore_hostcfg:
            host = self.model.host(KEY_LOG_HOST)
            if host is not None:
                return host[0], host[1]
        return stream.requester

    def _send_log_chunk(self, stream: LogStream, end: bool = False) -> None:
        dest = self._log_dest(stream)
        stream.trans_index = (stream.trans_index + 1) & 0xFFFFFFFF
        flags = 0
        if stream.trans_index == 1:
            flags |= LOG_FLAG_BEGIN
        if end:
            flags |= LOG_FLAG_END
        every = self.args.log_ack_every
        if every > 0 and (stream.trans_index % every == 0 or end):
            flags |= LOG_FLAG_ACK
        line = (
            f'{self.model.sn} log{stream.log_type} file{stream.file_index} '
            f'chunk{stream.trans_index} t={time.monotonic():.3f}\n'
        ).encode()
        n = 0 if end else self.args.log_chunk_bytes
        data = (line * (n // len(line) + 1))[:n]
        header = struct.pack(
            '<BBBBIHIH',
            stream.log_type,
            stream.file_index,
            1,
            flags,
            int(time.time()) & 0xFFFFFFFF,
            0,
            stream.trans_index,
            len(data),
        )
        if not end and self.log_drop > 0:
            self.log_drop -= 1
            self.emit(event='log_dropped', file_index=stream.file_index, trans=stream.trans_index)
            return
        if dest is None:
            return
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        if self.silenced():
            self.sent['silenced'] += 1
            return
        frame = proto.CommandFrame(
            self.seq, CMD_PUSH_LOG, REQ, SENDER_LIDAR, header + data
        ).encode()
        try:
            self.socks['log'].sendto(frame, dest)
        except OSError as e:
            self.log(f'send log to {dest} failed: {e}')
            return
        self.sent['log'] += 1

    def _on_log_ack(self, frame: proto.CommandFrame, addr) -> None:
        if len(frame.data) < 7:
            self.emit(
                event='bad_frame', **{'from': f'{addr[0]}:{addr[1]}', 'error': 'short log ack'}
            )
            return
        ret, log_type, file_index, trans = struct.unpack_from('<BBBI', frame.data, 0)
        self.log_acks_received += 1
        self.emit(event='log_ack', ret=ret, log_type=log_type, file_index=file_index, trans=trans)

    # -- periodic senders ------------------------------------------------------
    # -- debug raw data (#93) ------------------------------------------------
    def _debug_socket(self) -> socket.socket | None:
        """
        Return the source socket of the stream, bound on first use.

        60301 is inside the Linux ephemeral range, so binding it at start-up could fail for
        reasons unrelated to the test at hand.
        """
        if self.debug_sock is None:
            port = 0 if self.args.base_port == 0 else self.args.debug_data_port
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                if port != 0:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.setblocking(False)
                s.bind((self.bound_ip or self.args.bind, port))
            except OSError as e:
                s.close()
                self.emit(event='error', error=f'debug data socket on port {port}: {e}')
                return None
            self.debug_sock = s
        return self.debug_sock

    def _debug_control(self, enable: bool, dest: tuple[str, int]) -> bool:
        """
        Apply a 0x0303 request and return whether it was accepted.

        A repeated enable moves the stream to the new destination; a disable while disabled
        is accepted [unverified, #11].
        """
        if not enable:
            self.debug_dest = None
            self.emit(event='debug_data', enabled=False)
            return True
        if dest[1] == 0 or self._debug_socket() is None:
            return False
        if self.debug_dest is None:
            self.next_debug = time.monotonic()
        self.debug_dest = dest
        self.emit(
            event='debug_data',
            enabled=True,
            dest=f'{dest[0]}:{dest[1]}',
            port=self.debug_sock.getsockname()[1],
        )
        return True

    def _send_debug_data(self) -> None:
        """
        Send one synthetic datagram.

        seq u32 (little-endian, from 0), then bytes counting up from the low byte of seq.
        The real layout is unknown.
        """
        sock = self._debug_socket()
        if sock is None or self.debug_dest is None:
            return
        seq = self.debug_seq
        self.debug_seq = (seq + 1) & 0xFFFFFFFF
        if self.silenced():
            self.sent['silenced'] += 1
            return
        n = max(0, self.args.debug_data_bytes - 4)
        data = struct.pack('<I', seq) + bytes((seq + i) & 0xFF for i in range(n))
        try:
            sock.sendto(data, self.debug_dest)
        except OSError as e:
            self.log(f'send debug data to {self.debug_dest} failed: {e}')
            return
        self.sent['debug'] += 1

    def now_ns(self) -> int:
        return self.model.clock.now_ns(time.monotonic(), time.time_ns())

    def silenced(self) -> bool:
        return time.monotonic() < self.silence_until

    def _send_periodic(self, now: float) -> None:
        # Runs during a silence too: the schedules and counters advance and the senders
        # withhold the datagrams, so the host sees a udp_cnt gap and no burst afterwards.
        m = self.model
        if self.replay is not None:
            self._send_replay(now)
        elif m.sampling:
            pcl_host = m.host(KEY_PCL_HOST)
            interval = 1.0 / (PCL_PACKET_RATE * self.rate)
            budget = 256  # bound catch-up bursts
            # Bound them by time too: a narrow FOV window makes each packet slower than the
            # rate, and 256 of them would leave commands unanswered past the host's timeout.
            slice_end = time.monotonic() + CATCH_UP_SLICE_S
            while now >= self.next_pcl and budget > 0:
                # The packet's scheduled time, not `now`: in a catch-up burst `now` is a frame or
                # more ahead of the early packets, and frame_cnt would change on each one (#155).
                if self.frame_s > 0 and self.next_pcl - self.frame_started >= self.frame_s:
                    self.frame_cnt = (self.frame_cnt + 1) & 0xFF
                    self.frame_started += self.frame_s
                    if self.next_pcl - self.frame_started >= self.frame_s:
                        self.frame_started = self.next_pcl  # after a resync: one change, not many
                    self.ring_next = 0
                self._send_pcl(pcl_host, interval)
                self.next_pcl += interval
                budget -= 1
                if time.monotonic() >= slice_end:
                    break
            if now - self.next_pcl > 0.5:  # fell hopelessly behind: resync
                self.next_pcl = now
            if m.imu_enabled:
                imu_host = m.host(KEY_IMU_HOST)
                imu_interval = 1.0 / (m.imu_rate * self.rate)
                budget = 256
                while now >= self.next_imu and budget > 0:
                    self._send_imu(imu_host, imu_interval)
                    self.next_imu += imu_interval
                    budget -= 1
                if now - self.next_imu > 0.5:
                    self.next_imu = now
        if now >= self.next_push:
            if not self._replaying_pushes():
                self._send_push()
            self.next_push += 1.0 / self.push_rate
        if self.log_streams and now >= self.next_log:
            for stream in list(self.log_streams.values()):
                self._send_log_chunk(stream)
            self.next_log = max(self.next_log + self.args.log_chunk_interval, now)
        if self.debug_dest is not None:
            budget = 64  # bound catch-up bursts
            while now >= self.next_debug and budget > 0:
                self._send_debug_data()
                self.next_debug += self.args.debug_data_interval
                budget -= 1
            if now - self.next_debug > 0.5:
                self.next_debug = now
        if now >= self.next_stats:
            self.emit(event='sent', **self.sent, state=m.work_state)
            self.next_stats += 1.0

    def _replaying_pushes(self) -> bool:
        """Return whether the capture's pushes stand in for the simulator's own right now."""
        r = self.replay
        return r is not None and r.state == 'running' and r.sent['push'] > 0

    def _send_replay(self, now: float) -> None:
        """
        Start the replay once the SDK can receive it, then send the datagrams that are due.

        The replay starts when the LiDAR samples and a host is configured for one of the
        replayed streams, and then runs to the end of the capture whatever the work state.
        """
        r = self.replay
        if r.state == 'waiting':
            if not self.model.sampling or all(
                self.model.host(key) is None for _, key in REPLAY_STREAMS.values()
            ):
                return
            r.start(now)
            self.emit(event='replay_start', file=str(r.path), rate=r.rate)
        if r.state != 'running':
            return
        budget = 256  # bound bursts (rate 0, or catching up): commands still get answered
        while budget > 0 and (datagram := r.pop(now)) is not None:
            stream, payload = datagram
            host = self.model.host(REPLAY_STREAMS_BY_NAME[stream])
            if host is None:
                r.skipped += 1
            elif self._sendto(stream, payload, host):
                r.sent[stream] += 1
            budget -= 1
        if r.exhausted:
            r.state = 'done'
            self.emit(
                event='replay_done',
                sent=r.sent,
                skipped=r.skipped,
                seconds=round(now - r.started, 6),
            )

    def _cropped_samples(self, dt: int) -> list[tuple]:
        """
        Draw POINTS_PER_PACKET samples inside the enabled FOV windows.

        Up to MAX_FOV_DRAWS batches are drawn; a packet ends up shorter only for a tiny window.
        """
        m = self.model
        # --apply-attitude (#135): Cartesian points leave in the attitude's frame; the FOV
        # crops them before that, in the sensor frame [unverified, #11].
        att = m.install_attitude() if self.apply_attitude and dt != 3 else None
        if self.scene == 'ring':
            # The next POINTS_PER_PACKET ring points, cropped on their exact angles whatever the
            # data type: the packet carries fewer points instead of drawing more.
            ks = range(self.ring_next, self.ring_next + POINTS_PER_PACKET)
            self.ring_next = (self.ring_next + POINTS_PER_PACKET) % RING_POINTS
            return [ring_sample(dt, k, att) for k in ks if m.keeps_point(3, ring_point(k))]
        if not m.fov_windows():
            kept = self.points.samples(dt, POINTS_PER_PACKET)
        else:
            kept = []
            for _ in range(MAX_FOV_DRAWS):
                kept.extend(
                    p for p in self.points.samples(dt, POINTS_PER_PACKET) if m.keeps_point(dt, p)
                )
                if len(kept) >= POINTS_PER_PACKET:
                    break
            kept = kept[:POINTS_PER_PACKET]
        return kept if att is None else [attitude_sample(dt, p, att) for p in kept]

    def _send_pcl(self, host, interval_s: float) -> None:
        dt = self.model.pcl_data_type
        samples = self._cropped_samples(dt)  # fewer than 96 for a narrow FOV window
        pkt = proto.DataPacket(
            time_interval=min(int(interval_s * 1e7), 0xFFFF),
            dot_num=len(samples),
            udp_cnt=self.udp_cnt_pcl,
            frame_cnt=self.frame_cnt,
            data_type=dt,
            time_type=self.model.clock.sync_type,
            timestamp_ns=self.now_ns(),
            data=proto.pack_samples(dt, samples),
        )
        self.udp_cnt_pcl = (self.udp_cnt_pcl + 1) & 0xFFFF
        if host is None:
            return
        if self.drop_rate > 0 and self._drop_rng.random() < self.drop_rate:
            self.sent['pcl_dropped'] += 1
            return
        self._send_data('pcl', pkt.encode(), pkt.udp_cnt, host)

    def _send_imu(self, host, interval_s: float) -> None:
        pkt = proto.DataPacket(
            time_interval=min(int(interval_s * 1e7), 0xFFFF),
            dot_num=1,
            udp_cnt=self.udp_cnt_imu,
            frame_cnt=self.frame_cnt,
            data_type=0,
            time_type=self.model.clock.sync_type,
            timestamp_ns=self.now_ns(),
            data=proto.pack_samples(0, [self.points.imu()]),
        )
        self.udp_cnt_imu = (self.udp_cnt_imu + 1) & 0xFFFF
        if host is None:
            return
        self._send_data('imu', pkt.encode(), pkt.udp_cnt, host)

    def _send_data(self, kind: str, data: bytes, udp_cnt: int, host) -> None:
        """Send one point-cloud / IMU datagram through the stream's PacketFaults (#131)."""
        f = self.faults[kind]
        if f.drop > 0:
            f.drop -= 1
            self.sent[f'{kind}_dropped'] += 1
            self.emit(event='packet_fault', stream=kind, fault='drop', udp_cnt=udp_cnt)
            return
        if f.held is None and f.reorder > 0:
            f.reorder -= 1
            f.held = (data, udp_cnt, f.depth)
            return
        if self._sendto(kind, data, host):
            self.sent[kind] += 1
        if f.duplicate > 0:
            f.duplicate -= 1
            if self._sendto(kind, data, host):
                self.sent[f'{kind}_duplicated'] += 1
            self.emit(event='packet_fault', stream=kind, fault='duplicate', udp_cnt=udp_cnt)
        if f.held is not None:
            held, held_cnt, left = f.held
            if left > 1:
                f.held = (held, held_cnt, left - 1)
                return
            f.held = None
            if self._sendto(kind, held, host):
                self.sent[kind] += 1
                self.sent[f'{kind}_reordered'] += 1
            self.emit(event='packet_fault', stream=kind, fault='reorder', udp_cnt=held_cnt)

    def _send_push(self) -> None:
        host = self.model.host(KEY_STATE_HOST)
        if host is None:
            return
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        frame = proto.CommandFrame(
            self.seq, CMD_INFO_PUSH, REQ, SENDER_LIDAR, self.model.push_payload(self.now_ns())
        ).encode()
        if self._sendto('push', frame, host):
            self.sent['push'] += 1

    def _sendto(self, kind: str, data: bytes, host: tuple[str, int, int]) -> bool:
        """Send unless silenced; return whether the datagram left."""
        if self.silenced():
            self.sent['silenced'] += 1
            return False
        try:
            self.socks[kind].sendto(data, (host[0], host[1]))
        except OSError as e:
            self.log(f'send {kind} to {host[:2]} failed: {e}')
            return False
        return True


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
        help='discovery port; cmd/push/pcl/imu/log follow at +100..+500. 0 = pick free ports',
    )
    p.add_argument('--sn', default='SIM0000000000001')
    p.add_argument('--product-info', default='MID360-SIM', help='key 0x8001 (<= 64 chars)')
    p.add_argument('--version-app', default='0.0.0.1', help='key 0x8002 as a.b.c.d')
    p.add_argument('--version-loader', default='0.0.0.1', help='key 0x8003 as a.b.c.d')
    p.add_argument('--version-hardware', default='0.0.0.1', help='key 0x8004 as a.b.c.d')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument(
        '--scene',
        choices=SCENES,
        default='random',
        help='point cloud: seeded random points, or the deterministic ring (#132)',
    )
    p.add_argument(
        '--apply-attitude',
        action='store_true',
        help='move Cartesian points by the install attitude in key 0x0012 (#135)',
    )
    p.add_argument(
        '--pcap',
        metavar='FILE',
        help='replay the push / point cloud / IMU / log datagrams of a classic pcap (#134)',
    )
    p.add_argument(
        '--pcap-rate',
        type=float,
        default=1.0,
        help='--pcap speed: 2 = twice the recorded rate, 0 = as fast as possible (default 1)',
    )
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
    p.add_argument(
        '--log-chunk-interval',
        type=float,
        default=0.05,
        help='seconds between firmware log chunks (0x0300) per enabled log type',
    )
    p.add_argument('--log-chunk-bytes', type=int, default=512, help='data bytes per log chunk')
    p.add_argument(
        '--log-ack-every',
        type=int,
        default=1,
        help='request a host ACK on every Nth chunk (0 = never; the file end always asks)',
    )
    p.add_argument(
        '--log-ignore-hostcfg',
        action='store_true',
        help='send log chunks to the 0x0301 sender instead of the key 0x0009 host',
    )
    p.add_argument(
        '--debug-data-port',
        type=int,
        default=proto.PORT_DEBUG_DATA,
        help='source port of the debug raw data stream; 0 or --base-port 0 = pick a free port',
    )
    p.add_argument(
        '--debug-data-interval',
        type=float,
        default=0.01,
        help='seconds between debug raw data datagrams while 0x0303 has enabled them',
    )
    p.add_argument(
        '--debug-data-bytes', type=int, default=1024, help='size of a debug raw data datagram'
    )
    p.add_argument('--quit-on-eof', action='store_true', default=True)
    p.add_argument('--no-quit-on-eof', dest='quit_on_eof', action='store_false')
    p.add_argument('--verbose', '-v', action='store_true')
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if len(args.sn) > 16:
        sys.exit('--sn must be at most 16 characters')
    if args.pcap_rate < 0:
        sys.exit('--pcap-rate must not be negative')
    if args.pcap is not None:
        try:  # fail at start-up, not when the SDK connects
            next(replay_datagrams(pathlib.Path(args.pcap)), None)
        except (OSError, struct.error, SystemExit) as e:
            sys.exit(f'--pcap {args.pcap}: {e}')
    sim = Simulator(args)
    try:
        sim.run()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())

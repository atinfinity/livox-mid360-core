#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Pure-Python reference implementation of the Livox Mid-360 wire protocol.

This module is the *independent* second implementation used to cross-check the C++ library
(tests/generated/golden_vectors.hpp is produced from it by gen_golden_vectors.py) and to
decode pcap captures (livox_mid360_pcap.py). It has no third-party dependencies.

Reference: "Livox LiDAR Communication Protocol - Mid360", wiki rev v1.4.12.
"""
from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field
from typing import Iterator, Sequence

# --------------------------------------------------------------------------- CRC
def crc16_ccitt_false(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc32(data: bytes) -> int:
    """CRC-32 poly 0x04C11DB7 init/xorout 0xFFFFFFFF, reflected: identical to zlib."""
    return zlib.crc32(data) & 0xFFFFFFFF


# --------------------------------------------------------------------------- constants
SOF = 0xAA
CMD_HEADER_SIZE = 24
CMD_FRAME_MAX = 1400
DATA_HEADER_SIZE = 36

PORT_DISCOVERY, PORT_CMD, PORT_PUSH, PORT_PCL, PORT_IMU, PORT_LOG = 56000, 56100, 56200, 56300, 56400, 56500

SAMPLE_SIZE = {0: 24, 1: 14, 2: 8, 3: 10}
SAMPLE_FMT = {0: "<6f", 1: "<iiiBB", 2: "<hhhBB", 3: "<IHHBB"}

KEY_NAMES = {
    0x0000: "pcl_data_type", 0x0001: "pattern_mode", 0x0004: "lidar_ipcfg",
    0x0005: "state_info_host_ipcfg", 0x0006: "pointcloud_host_ipcfg", 0x0007: "imu_host_ipcfg",
    0x0012: "install_attitude", 0x0015: "fov_cfg0", 0x0016: "fov_cfg1", 0x0017: "fov_cfg_en",
    0x0018: "detect_mode", 0x0019: "func_io_cfg", 0x001A: "work_tgt_mode", 0x001C: "imu_data_en",
    0x0021: "speed_mode", 0x0026: "time_filter", 0x0029: "pc_freq_mod", 0x002B: "imu_sensor_cfg",
    0x8000: "sn", 0x8001: "product_info", 0x8002: "version_app", 0x8003: "version_loader",
    0x8004: "version_hardware", 0x8005: "mac", 0x8006: "cur_work_state", 0x8007: "core_temp",
    0x8008: "powerup_cnt", 0x8009: "local_time_now", 0x800A: "last_sync_time",
    0x800B: "time_offset", 0x800C: "time_sync_type", 0x800E: "lidar_diag_status",
    0x8010: "FW_TYPE", 0x8011: "hms_code",
}
WORK_STATE = {1: "SAMPLING", 2: "IDLE", 4: "ERROR", 5: "SELFCHECK", 6: "MOTORSTARTUP", 8: "UPGRADE", 9: "READY"}


# --------------------------------------------------------------------------- command frame
@dataclass
class CommandFrame:
    seq_num: int
    cmd_id: int
    cmd_type: int  # 0 REQ, 1 ACK
    sender_type: int  # 0 host, 1 lidar
    data: bytes = b""
    resv: bytes = b"\0" * 6

    def encode(self) -> bytes:
        if len(self.data) > CMD_FRAME_MAX - CMD_HEADER_SIZE:
            raise ValueError("data too large")
        length = CMD_HEADER_SIZE + len(self.data)
        head18 = struct.pack("<BBHIHBB6s", SOF, 0, length, self.seq_num, self.cmd_id,
                             self.cmd_type, self.sender_type, self.resv)
        c16 = crc16_ccitt_false(head18)
        c32 = crc32(self.data) if self.data else 0
        return head18 + struct.pack("<HI", c16, c32) + self.data

    @classmethod
    def parse(cls, frame: bytes) -> "CommandFrame":
        if len(frame) < CMD_HEADER_SIZE:
            raise ValueError("too short")
        sof, ver, length, seq, cmd_id, ctype, stype, resv, c16, c32 = struct.unpack_from(
            "<BBHIHBB6sHI", frame, 0)
        if sof != SOF:
            raise ValueError("bad SOF")
        if ver != 0:
            raise ValueError("bad version")
        if length < CMD_HEADER_SIZE or length > CMD_FRAME_MAX or length > len(frame):
            raise ValueError("length mismatch")
        if crc16_ccitt_false(frame[:18]) != c16:
            raise ValueError("bad CRC16")
        data = frame[CMD_HEADER_SIZE:length]
        if (crc32(data) if data else 0) != c32:
            raise ValueError("bad CRC32")
        return cls(seq, cmd_id, ctype, stype, data, resv)


# --------------------------------------------------------------------------- key-value
def encode_kv_list(kvs: Sequence[tuple[int, bytes]]) -> bytes:
    return b"".join(struct.pack("<HH", k, len(v)) + v for k, v in kvs)


def parse_kv_list(buf: bytes, key_num: int) -> list[tuple[int, bytes]]:
    out, off = [], 0
    for _ in range(key_num):
        if len(buf) - off < 4:
            raise ValueError("truncated")
        k, n = struct.unpack_from("<HH", buf, off)
        off += 4
        if len(buf) - off < n:
            raise ValueError("truncated")
        out.append((k, buf[off:off + n]))
        off += n
    if off != len(buf):
        raise ValueError("key_num mismatch")
    return out


def encode_param_config(kvs: Sequence[tuple[int, bytes]]) -> bytes:
    return struct.pack("<HH", len(kvs), 0) + encode_kv_list(kvs)


def encode_param_inquire(keys: Sequence[int]) -> bytes:
    return struct.pack("<HH", len(keys), 0) + b"".join(struct.pack("<H", k) for k in keys)


def parse_param_config_ack(d: bytes) -> tuple[int, int]:
    return struct.unpack_from("<BH", d, 0)


def parse_param_inquire_ack(d: bytes) -> tuple[int, list[tuple[int, bytes]]]:
    ret, n = struct.unpack_from("<BH", d, 0)
    return ret, parse_kv_list(d[3:], n)


def parse_info_push(d: bytes) -> list[tuple[int, bytes]]:
    n, _ = struct.unpack_from("<HH", d, 0)
    return parse_kv_list(d[4:], n)


def parse_discovery_ack(d: bytes) -> dict:
    ret, dev, sn, ip, port = struct.unpack_from("<BB16s4sH", d, 0)
    return {"ret_code": ret, "dev_type": dev, "sn": sn.split(b"\0")[0].decode(),
            "lidar_ip": ".".join(map(str, ip)), "cmd_port": port}


def encode_host_ipcfg(ip: str, dst_port: int, src_port: int) -> bytes:
    return bytes(int(x) for x in ip.split(".")) + struct.pack("<HH", dst_port, src_port)


def encode_lidar_ipcfg(ip: str, mask: str, gw: str) -> bytes:
    return b"".join(bytes(int(x) for x in s.split(".")) for s in (ip, mask, gw))


def encode_install_attitude(roll, pitch, yaw, x_mm, y_mm, z_mm) -> bytes:
    return struct.pack("<fffiii", roll, pitch, yaw, x_mm, y_mm, z_mm)


def encode_fov_cfg(yaw_start, yaw_stop, pitch_start, pitch_stop, rsvd=0) -> bytes:
    return struct.pack("<iiiiI", yaw_start, yaw_stop, pitch_start, pitch_stop, rsvd)


def encode_reboot(timeout_ms: int) -> bytes:
    return struct.pack("<H", timeout_ms)


def encode_set_gps_timestamp(ns: int) -> bytes:
    return struct.pack("<BQ", 2, ns)


# --------------------------------------------------------------------------- data packet
@dataclass
class DataPacket:
    time_interval: int  # 0.1 us
    dot_num: int
    udp_cnt: int
    frame_cnt: int
    data_type: int
    time_type: int
    timestamp_ns: int
    data: bytes
    reserved: bytes = b"\0" * 12
    version: int = 0

    def encode(self) -> bytes:
        length = DATA_HEADER_SIZE + len(self.data)
        ts = struct.pack("<Q", self.timestamp_ns)
        c32 = crc32(ts + self.data)
        return struct.pack("<BHHHHBBB12sI", self.version, length, self.time_interval, self.dot_num,
                           self.udp_cnt, self.frame_cnt, self.data_type, self.time_type,
                           self.reserved, c32) + ts + self.data

    @classmethod
    def parse(cls, pkt: bytes, verify_crc: bool = True) -> "DataPacket":
        if len(pkt) < DATA_HEADER_SIZE:
            raise ValueError("too short")
        (ver, length, ti, dn, uc, fc, dt, tt, rsv, c32, ts) = struct.unpack_from("<BHHHHBBB12sIQ", pkt, 0)
        if ver != 0:
            raise ValueError("bad version")
        if length < DATA_HEADER_SIZE or length > len(pkt):
            raise ValueError("length mismatch")
        if dt not in SAMPLE_SIZE:
            raise ValueError("unknown data_type")
        data = pkt[DATA_HEADER_SIZE:length]
        if len(data) != dn * SAMPLE_SIZE[dt]:
            raise ValueError("bad dot_num")
        if verify_crc and crc32(pkt[28:28 + 8 + len(data)]) != c32:
            raise ValueError("bad CRC32")
        return cls(ti, dn, uc, fc, dt, tt, ts, data, rsv, ver)

    def samples(self) -> Iterator[tuple]:
        fmt, size = SAMPLE_FMT[self.data_type], SAMPLE_SIZE[self.data_type]
        for i in range(self.dot_num):
            yield struct.unpack_from(fmt, self.data, i * size)

    def sample_timestamp_ns(self, i: int) -> int:
        if self.dot_num <= 1 or i == 0:
            return self.timestamp_ns
        return self.timestamp_ns + (self.time_interval * 100 * i) // (self.dot_num - 1)


def pack_samples(data_type: int, samples: Sequence[tuple]) -> bytes:
    fmt = SAMPLE_FMT[data_type]
    return b"".join(struct.pack(fmt, *s) for s in samples)


def decode_tag(tag: int) -> dict:
    return {"adjacent_glue": tag & 3, "particles": (tag >> 2) & 3, "other": (tag >> 4) & 3,
            "reserved": (tag >> 6) & 3}


def decode_hms(raw: int) -> tuple[int, int]:
    """Returns (abnormal_id, level)."""
    return raw >> 16, raw & 0xFF

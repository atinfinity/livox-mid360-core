#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Generates tests/data/replay.pcap, the capture the simulator's --pcap replay is tested with (#134).

Run from the repository root:  python3 tools/gen_replay_pcap.py
The output is committed so that CI needs neither hardware captures nor this script at build
time; CI regenerates it and fails on a difference.

Classic pcap (microseconds, little-endian), Ethernet. LiDAR 192.168.1.12, host 192.168.1.50:
  * replayed (LiDAR data ports): 2 pushes (0x0102, core_temp PUSH_CORE_TEMP), PCL_FRAMES frames
    of PCL_PACKETS_PER_FRAME Cartesian32 ring-scene packets, IMU_PACKETS IMU packets and one
    firmware log chunk (0x0300);
  * not replayed: an ARP frame, discovery and 0x0100 with their ACKs, and a DNS datagram.
"""

from __future__ import annotations

import pathlib
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import livox_mid360_proto as p  # noqa: E402
import livox_mid360_sim as sim  # noqa: E402

OUT = pathlib.Path(__file__).resolve().parent.parent / 'tests' / 'data' / 'replay.pcap'

LIDAR_IP, HOST_IP = '192.168.1.12', '192.168.1.50'
LIDAR_MAC, HOST_MAC = bytes.fromhex('3c0e00000001'), bytes.fromhex('020000000050')
T0_S = 1_700_000_000  # capture time of the first record
T0_NS = 1_000_000_000_000  # LiDAR timestamp of the first data packet (time_type 0: since boot)

PCL_FRAMES, PCL_PACKETS_PER_FRAME, PCL_SPACING_MS = 3, 8, 5
IMU_PACKETS, IMU_SPACING_MS = 12, 10
PUSH_CORE_TEMP = 4321  # 0.01 degC: tells the recorded pushes from the simulator's own


def ipv4(src: str, dst: str, ident: int, payload: bytes, proto_num: int = 17) -> bytes:
    head = struct.pack(
        '!BBHHHBBH4s4s',
        0x45,
        0,
        20 + len(payload),
        ident,
        0x4000,  # don't fragment
        64,
        proto_num,
        0,
        bytes(int(x) for x in src.split('.')),
        bytes(int(x) for x in dst.split('.')),
    )
    words = struct.unpack('!10H', head)
    s = sum(words)
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return head[:10] + struct.pack('!H', ~s & 0xFFFF) + head[12:] + payload


def udp_frame(src: str, sport: int, dst: str, dport: int, ident: int, data: bytes) -> bytes:
    udp = struct.pack('!HHHH', sport, dport, 8 + len(data), 0) + data  # checksum 0: unused
    dst_mac = b'\xff' * 6 if dst.endswith('.255') else (HOST_MAC if dst == HOST_IP else LIDAR_MAC)
    src_mac = LIDAR_MAC if src == LIDAR_IP else HOST_MAC
    return dst_mac + src_mac + b'\x08\x00' + ipv4(src, dst, ident, udp)


def arp_frame() -> bytes:
    body = struct.pack(
        '!HHBBH6s4s6s4s',
        1,
        0x0800,
        6,
        4,
        1,  # request: who has 192.168.1.12
        HOST_MAC,
        bytes(int(x) for x in HOST_IP.split('.')),
        b'\0' * 6,
        bytes(int(x) for x in LIDAR_IP.split('.')),
    )
    return b'\xff' * 6 + HOST_MAC + b'\x08\x06' + body


def push(seq: int) -> bytes:
    kvs = [
        (sim.KEY_CUR_WORK_STATE, bytes([sim.WS_SAMPLING])),
        (sim.KEY_CORE_TEMP, struct.pack('<i', PUSH_CORE_TEMP)),
    ]
    data = struct.pack('<HH', len(kvs), 0) + p.encode_kv_list(kvs)
    return p.CommandFrame(seq, sim.CMD_INFO_PUSH, 0, 1, data).encode()


def pcl_packet(i: int) -> bytes:
    first = i * sim.POINTS_PER_PACKET
    samples = [
        sim.ring_sample(1, (first + j) % sim.RING_POINTS) for j in range(sim.POINTS_PER_PACKET)
    ]
    return p.DataPacket(
        time_interval=PCL_SPACING_MS * 10_000,  # 0.1 us
        dot_num=len(samples),
        udp_cnt=i,
        frame_cnt=i // PCL_PACKETS_PER_FRAME,
        data_type=1,
        time_type=0,
        timestamp_ns=T0_NS + i * PCL_SPACING_MS * 1_000_000,
        data=p.pack_samples(1, samples),
    ).encode()


def imu_packet(i: int) -> bytes:
    sample = (0.01 * i, -0.02, 0.03, 0.0, 0.0, 1.0)  # gyro rad/s, acc g
    return p.DataPacket(
        time_interval=0,
        dot_num=1,
        udp_cnt=i,
        frame_cnt=0,
        data_type=0,
        time_type=0,
        timestamp_ns=T0_NS + 2_000_000 + i * IMU_SPACING_MS * 1_000_000,
        data=p.pack_samples(0, [sample]),
    ).encode()


def log_chunk() -> bytes:
    text = b'replay log line\n'
    header = struct.pack(
        '<BBBBIHIH', 0, 1, 1, sim.LOG_FLAG_BEGIN, T0_S, 0, 1, len(text)
    )  # log_type, file_index, 1, flags, time, rsvd, trans_index, length
    return p.CommandFrame(3, sim.CMD_PUSH_LOG, 0, 1, header + text).encode()


def records() -> list[tuple[int, bytes]]:
    """Return (microseconds since T0_S, Ethernet frame) in capture order."""
    out: list[tuple[int, bytes]] = []
    ident = iter(range(1, 1000))

    def lidar(us: int, sport: int, dport: int, data: bytes) -> None:
        out.append((us, udp_frame(LIDAR_IP, sport, HOST_IP, dport, next(ident), data)))

    def host(us: int, dst: str, dport: int, data: bytes) -> None:
        out.append((us, udp_frame(HOST_IP, 56001, dst, dport, next(ident), data)))

    out.append((0, arp_frame()))
    host(1000, '255.255.255.255', p.PORT_DISCOVERY, p.CommandFrame(1, 0, 0, 0).encode())
    ack = bytes([0, p.DEV_TYPE_MID360]) + b'REPLAY0000000001'
    ack += bytes(int(x) for x in LIDAR_IP.split('.')) + struct.pack('<H', p.PORT_CMD)
    lidar(2000, p.PORT_DISCOVERY, 56001, p.CommandFrame(1, 0, 1, 1, ack).encode())
    kvs = [(sim.KEY_PCL_HOST, p.encode_host_ipcfg(HOST_IP, 56301, p.PORT_PCL))]
    config = p.CommandFrame(2, sim.CMD_PARAM_CONFIG, 0, 0, p.encode_param_config(kvs)).encode()
    host(3000, LIDAR_IP, p.PORT_CMD, config)
    lidar(
        4000, p.PORT_CMD, 56001, p.CommandFrame(2, sim.CMD_PARAM_CONFIG, 1, 1, b'\0\0\0').encode()
    )
    lidar(5000, p.PORT_PUSH, 56201, push(1))
    for i in range(PCL_FRAMES * PCL_PACKETS_PER_FRAME):
        lidar(10_000 + i * PCL_SPACING_MS * 1000, p.PORT_PCL, 56301, pcl_packet(i))
    for i in range(IMU_PACKETS):
        lidar(12_000 + i * IMU_SPACING_MS * 1000, p.PORT_IMU, 56401, imu_packet(i))
    host(50_000, '192.168.1.1', 53, b'\x12\x34\x01\x00\x00\x01\x00\x00\x00\x00\x00\x00')
    lidar(60_000, p.PORT_LOG, 56501, log_chunk())
    lidar(100_000, p.PORT_PUSH, 56201, push(2))
    out.sort(key=lambda r: r[0])  # stable: equal times keep their order
    return out


def pcap_bytes() -> bytes:
    out = bytearray(struct.pack('<IHHiIII', 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1))
    for us, frame in records():
        out += struct.pack('<IIII', T0_S + us // 1_000_000, us % 1_000_000, len(frame), len(frame))
        out += frame
    return bytes(out)


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_bytes(pcap_bytes())
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()

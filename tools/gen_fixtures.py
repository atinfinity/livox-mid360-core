#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Generates the synthetic Mid-360 captures in tests/fixtures/ (#10); see tests/fixtures/README.md.

Run from the repository root:  python3 tools/gen_fixtures.py
The output is committed; CI regenerates it and fails on a difference.

The captures are synthetic: no datagram comes from a real device. What they reproduce was
measured on a Mid-360 with firmware 13.18.0244 (packet sizes, header fields, rates, key sets,
ACK shapes, the reboot timeline, the firmware log flags); the point coordinates, the log text,
the debug data bytes, the serial number and the addresses are made up. Classic pcap
(microseconds, little-endian), Ethernet; LiDAR 192.168.1.12, host 192.168.1.50.
"""

from __future__ import annotations

from collections.abc import Callable
import pathlib
import random
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import gen_replay_pcap as pcapw  # noqa: E402

import livox_mid360_proto as p  # noqa: E402
import livox_mid360_sim as sim  # noqa: E402

OUT = pathlib.Path(__file__).resolve().parent.parent / 'tests' / 'fixtures'

LIDAR_IP, HOST_IP = pcapw.LIDAR_IP, pcapw.HOST_IP
SN = 'FIXTURE0000001'  # 14 characters like a Mid-360 serial number
T0_S = 1_760_000_000  # capture time of the first record
HOST_CMD_PORT = 56101
HOST_PUSH, HOST_PCL, HOST_IMU, HOST_LOG, HOST_DEBUG = 56201, 56301, 56401, 56501, 44332
DEBUG_SRC_PORT = 60301

# Measured on firmware 13.18.0244.
PCL_TIME_INTERVAL = 4750  # 0.1 us, every data type
PCL_STEP_NS = 480_000  # timestamp step between point cloud packets
IMU_STEP_NS = 5_000_000  # 200 Hz (the measured step jitters between about 4.2 and 5.8 ms)
DEBUG_BYTES = 1114
FOV_CROPPED = (0, 0, 0, 60, 0)  # a point outside the FOV window: zeroed, reflectivity 60
NO_RETURN_TAGS = (0, 0, 0, 128, 192)  # (0, 0, 0) points without a return carry these tags
UNSUPPORTED_SETTINGS = (0x0021, 0x0026, 0x0029, 0x002B)  # inquire answers 0x20 for these
SETTINGS_KEYS = (0x0000, 0x0001, 0x0004, 0x0005, 0x0006, 0x0007, 0x0012, 0x0015, 0x0016)
SETTINGS_KEYS += (0x0017, 0x0018, 0x0019, 0x001A, 0x001C) + UNSUPPORTED_SETTINGS
STATUS_KEYS = (0x8000, 0x8001, 0x8002, 0x8003, 0x8004, 0x8005, 0x8006, 0x8007, 0x8008)
STATUS_KEYS += (0x8009, 0x800A, 0x800B, 0x800C, 0x800E, 0x8010, 0x8011)


def model() -> sim.DeviceModel:
    """Return the device the pushes and inquire ACKs describe, with the measured identity."""
    m = sim.DeviceModel(
        sn=SN,
        product_info='DevType:Mid-360 FmType:App FmVer:13180244 BuildTime:2025/04/01',
        version_app=(13, 18, 2, 44),
        version_loader=(13, 17, 99, 20),
        version_hardware=(0, 0, 0, 0),
        work_state=sim.WS_SAMPLING,
        core_temp=5979,
        powerup_cnt=41,
        hms=[0x04070002] + [0] * 7,  # PPS / GPS warning, present on every capture
        imu_cfg_unsupported=True,
        inquire_overrides=dict.fromkeys(UNSUPPORTED_SETTINGS, 'unsupported'),
    )
    m.settings[sim.KEY_IMU_EN] = b'\x01'
    m.settings[sim.KEY_LIDAR_IPCFG] = p.encode_lidar_ipcfg(LIDAR_IP, '255.255.255.0', '0.0.0.0')
    m.settings[sim.KEY_STATE_HOST] = p.encode_host_ipcfg(HOST_IP, HOST_PUSH, p.PORT_PUSH)
    m.settings[sim.KEY_PCL_HOST] = p.encode_host_ipcfg(HOST_IP, HOST_PCL, p.PORT_PCL)
    m.settings[sim.KEY_IMU_HOST] = p.encode_host_ipcfg(HOST_IP, HOST_IMU, p.PORT_IMU)
    m.settings[sim.KEY_FOV0] = m.settings[sim.KEY_FOV1] = p.encode_fov_cfg(0, 0, -7, 52)
    return m


class Capture:
    """Records of one fixture: (microseconds since T0_S, Ethernet frame)."""

    def __init__(self) -> None:
        self.records: list[tuple[int, bytes]] = []
        self.ident = 0
        self.host_seq = 0
        self.push_seq = 6000  # the LiDAR's counter; a push advances it by about 26 per second

    def _add(self, us: int, src: str, sport: int, dst: str, dport: int, data: bytes) -> None:
        self.ident += 1
        self.records.append((us, pcapw.udp_frame(src, sport, dst, dport, self.ident, data)))

    def lidar(self, us: int, sport: int, dport: int, data: bytes, dst: str = HOST_IP) -> None:
        self._add(us, LIDAR_IP, sport, dst, dport, data)

    def host(self, us: int, dport: int, data: bytes, dst: str = LIDAR_IP, sport: int = 0) -> None:
        self._add(us, HOST_IP, sport or HOST_CMD_PORT, dst, dport, data)

    def command(
        self, us: int, cmd_id: int, req: bytes, ack: bytes, port: int = p.PORT_CMD
    ) -> None:
        """Record a host request and the LiDAR's ACK 2 ms later."""
        self.host_seq += 1
        self.host(us, port, p.CommandFrame(self.host_seq, cmd_id, 0, 0, req).encode())
        ack_frame = p.CommandFrame(self.host_seq, cmd_id, 1, 1, ack).encode()
        self.lidar(us + 2000, port, HOST_CMD_PORT, ack_frame)

    def config(self, us: int, kvs: list[tuple[int, bytes]], ret: int = 0, key: int = 0) -> None:
        self.command(
            us, sim.CMD_PARAM_CONFIG, p.encode_param_config(kvs), struct.pack('<BH', ret, key)
        )

    def push(self, us: int, m: sim.DeviceModel, local_ns: int) -> None:
        self.push_seq += 26
        frame = p.CommandFrame(self.push_seq, sim.CMD_INFO_PUSH, 0, 1, m.push_payload(local_ns))
        self.lidar(us, p.PORT_PUSH, HOST_PUSH, frame.encode())

    def pcap(self) -> bytes:
        out = bytearray(struct.pack('<IHHiIII', 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1))
        for us, frame in sorted(self.records, key=lambda r: r[0]):
            out += struct.pack(
                '<IIII', T0_S + us // 1_000_000, us % 1_000_000, len(frame), len(frame)
            )
            out += frame
        return bytes(out)


def pcl_packet(data_type: int, udp_cnt: int, ts_ns: int, samples: list[tuple]) -> bytes:
    pkt = p.DataPacket(
        time_interval=PCL_TIME_INTERVAL,
        dot_num=len(samples),
        udp_cnt=udp_cnt & 0xFFFF,
        frame_cnt=0,  # always 0 on a Mid-360
        data_type=data_type,
        time_type=0,
        timestamp_ns=ts_ns,
        data=p.pack_samples(data_type, samples),
    ).encode()
    return pkt[:24] + b'\0\0\0\0' + pkt[28:]  # point cloud crc32 is left at 0 (#219)


def imu_packet(udp_cnt: int, ts_ns: int, sample: tuple) -> bytes:
    return p.DataPacket(
        time_interval=0,
        dot_num=1,
        udp_cnt=udp_cnt & 0xFFFF,
        frame_cnt=0,
        data_type=0,
        time_type=0,
        timestamp_ns=ts_ns,
        data=p.pack_samples(0, [sample]),
    ).encode()


def points(data_type: int, base: int) -> list[tuple]:
    """96 ring-scene points from index `base`; every 7th has no return."""
    out = []
    for i in range(96):
        if (base + i) % 7 == 0:
            out.append((0, 0, 0, 0, NO_RETURN_TAGS[(base + i) % len(NO_RETURN_TAGS)]))
        else:
            out.append(sim.ring_sample(data_type, base + i))
    return out


def stream(
    cap: Capture,
    start_us: int,
    count: int,
    udp_cnt: int,
    ts_ns: int,
    data_type: int = 1,
    sample: Callable[[list[tuple]], list[tuple]] | None = None,
) -> tuple[int, int]:
    """`count` point cloud packets every 480 us; return the next (udp_cnt, timestamp)."""
    for i in range(count):
        pts = points(data_type, (udp_cnt + i) * 96)
        if sample is not None:
            pts = sample(pts)
        us = start_us + i * PCL_STEP_NS // 1000
        cap.lidar(us, p.PORT_PCL, HOST_PCL, pcl_packet(data_type, udp_cnt + i, ts_ns, pts))
        ts_ns += PCL_STEP_NS
    return udp_cnt + count, ts_ns


# --------------------------------------------------------------------------- fixtures
def discovery_setup() -> Capture:
    """Broadcast discovery, the ACK to 255.255.255.255, host setup and the push that follows."""
    cap, m = Capture(), model()
    cap.host(0, p.PORT_DISCOVERY, p.CommandFrame(1, 0, 0, 0).encode(), dst='255.255.255.255')
    ack = struct.pack(
        '<BB16s4sH',
        0,
        p.DEV_TYPE_MID360,
        SN.encode().ljust(16, b'\0'),
        bytes(int(x) for x in LIDAR_IP.split('.')),
        p.PORT_CMD,
    )
    cap.lidar(
        1000,
        p.PORT_DISCOVERY,
        HOST_CMD_PORT,
        p.CommandFrame(1, 0, 1, 1, ack).encode(),
        dst='255.255.255.255',
    )
    cap.host_seq = 1
    cap.config(
        20_000,
        [
            (sim.KEY_STATE_HOST, p.encode_host_ipcfg(HOST_IP, HOST_PUSH, p.PORT_PUSH)),
            (sim.KEY_PCL_HOST, p.encode_host_ipcfg(HOST_IP, HOST_PCL, p.PORT_PCL)),
            (sim.KEY_IMU_HOST, p.encode_host_ipcfg(HOST_IP, HOST_IMU, p.PORT_IMU)),
        ],
    )
    cap.push(35_000, m, 7_000_000_000)  # a 0x0100 request is followed by a push within 20 ms
    cap.push(1_035_000, m, 8_000_000_000)
    return cap


def inquire() -> Capture:
    """0x0101 of every settings key (ret 0x20, four unsupported), every status key, one bad key."""
    cap, m = Capture(), model()
    now_ns = 7_800_000_000
    for i, keys in enumerate((SETTINGS_KEYS, STATUS_KEYS, (0x002B,))):
        ret, kvs = m.inquire(list(keys), now_ns)
        ack = struct.pack('<BH', ret, len(kvs)) + p.encode_kv_list(kvs)
        cap.command(10_000 + i * 20_000, sim.CMD_PARAM_INQUIRE, p.encode_param_inquire(keys), ack)
    cap.push(200_000, m, now_ns)  # no extra push for 0x0101
    return cap


def pcl_types() -> Capture:
    """Each pcl_data_type for 30 packets; udp_cnt runs on across the switches and wraps."""
    cap, m = Capture(), model()
    udp_cnt, ts_ns = 65_500, 9_000_000_000
    us = 0
    for data_type in (1, 2, 3):
        m.settings[sim.KEY_PCL_DATA_TYPE] = bytes([data_type])
        cap.config(us, [(sim.KEY_PCL_DATA_TYPE, bytes([data_type]))])
        cap.push(us + 15_000, m, ts_ns)
        udp_cnt, ts_ns = stream(cap, us + 20_000, 30, udp_cnt, ts_ns, data_type)
        us += 40_000
    return cap


def imu() -> Capture:
    """IMU at 200 Hz between point cloud packets; the rejected imu_sensor_cfg write (0x20)."""
    cap, m = Capture(), model()
    cap.config(
        0,
        [(sim.KEY_IMU_SENSOR_CFG, b'\x00\x00\x00')],
        ret=sim.RET_PARAM_NOT_SUPPORT,
        key=sim.KEY_IMU_SENSOR_CFG,
    )
    cap.push(15_000, m, 9_000_000_000)
    rng = random.Random(200)
    for i in range(20):
        sample = (
            rng.uniform(-0.01, 0.01),
            rng.uniform(-0.01, 0.01),
            rng.uniform(-0.01, 0.01),
            rng.uniform(-0.02, 0.02),
            rng.uniform(-0.02, 0.02),
            1.0 + rng.uniform(-0.02, 0.02),
        )
        ts = 9_000_000_000 + 20_000_000 + i * IMU_STEP_NS
        cap.lidar(20_000 + i * 5000, p.PORT_IMU, HOST_IMU, imu_packet(4000 + i, ts, sample))
    stream(cap, 20_100, 10, 41_000, 9_020_100_000)
    return cap


def fov() -> Capture:
    """Crop below the horizon, then everything: cropped points stay in the packet, zeroed."""
    cap, m = Capture(), model()

    def above_horizon(pts: list[tuple]) -> list[tuple]:
        return [s if s[:3] == (0, 0, 0) or s[2] >= 0 else FOV_CROPPED for s in pts]

    def empty(pts: list[tuple]) -> list[tuple]:
        return [FOV_CROPPED] * len(pts)

    udp_cnt, ts_ns = stream(cap, 0, 10, 100, 9_000_000_000)
    for i, (window, crop) in enumerate(
        (((0, 359, 0, 59), above_horizon), ((90, 90, 0, 0), empty))
    ):
        us = 20_000 + i * 30_000
        cfg = p.encode_fov_cfg(*window)
        m.settings[sim.KEY_FOV0] = cfg
        m.settings[sim.KEY_FOV_EN] = b'\x01'
        cap.config(us, [(sim.KEY_FOV0, cfg), (sim.KEY_FOV_EN, b'\x01')])
        cap.push(us + 15_000, m, ts_ns)
        udp_cnt, ts_ns = stream(cap, us + 10_000, 10, udp_cnt, ts_ns, sample=crop)
    return cap


def firmware_log() -> Capture:
    """0x0301 on, file 7: the first chunk and the next eight ask for an ACK, later ones do not."""
    cap = Capture()
    cap.command(0, sim.CMD_COLLECTION_LOG, b'\x00\x01', b'\x00', port=p.PORT_LOG)
    rng = random.Random(7)
    seq = 2148
    for trans in range(14):
        n = 256 if trans == 0 else (rng.randint(660, 820) if trans <= 8 else rng.randint(546, 632))
        flags = 3 if trans == 0 else (1 if trans <= 8 else 0)
        header = struct.pack('<BBBBIHIH', 0, 7, 0 if trans == 0 else 1, flags, 0, 0, trans, n)
        # Synthetic. Real chunks hold binary records that start with 0x55.
        body = b'\x55' + bytes((trans * 31 + i) & 0xFF for i in range(n - 1))
        us = 6000 + trans * 10_000 if trans <= 9 else 100_000 + (trans - 9) * 150_000
        frame = p.CommandFrame(seq + trans, sim.CMD_PUSH_LOG, 0, 1, header + body).encode()
        cap.lidar(us, p.PORT_LOG, HOST_LOG, frame)
    cap.command(1_000_000, sim.CMD_COLLECTION_LOG, b'\x00\x00', b'\x00', port=p.PORT_LOG)
    return cap


def debug_data() -> Capture:
    """0x0303 on (ACKed on the log port), 1114-byte datagrams from port 60301, 0x0303 off."""
    cap = Capture()
    on = p.encode_debug_data_control(True, HOST_IP, HOST_DEBUG)
    cap.command(0, sim.CMD_DEBUG_DATA, on, b'\x00', port=p.PORT_LOG)
    for i in range(20):
        data = struct.pack('<I', i) + bytes((i + k) & 0xFF for k in range(DEBUG_BYTES - 4))
        cap.lidar(5000 + i * 250, DEBUG_SRC_PORT, HOST_DEBUG, data)
    off = p.encode_debug_data_control(False, HOST_IP, HOST_DEBUG)
    cap.command(20_000, sim.CMD_DEBUG_DATA, off, b'\x00', port=p.PORT_LOG)
    return cap


def reboot() -> Capture:
    """
    0x0200 while sampling, as timed on a Mid-360.

    The LiDAR keeps running for 1.25 s, pushes ERROR twice, is silent until +8.9 s, pushes
    MOTORSTARTUP once a second, READY at +13.54 s and SAMPLING 20 ms later, when the data
    resumes with udp_cnt 0 and the timestamp counting from the new power-on. Data is trimmed
    to a few packets around each edge.
    """
    cap, m = Capture(), model()
    s = 1_000_000
    m.work_state = sim.WS_SAMPLING
    udp_cnt, ts_ns = stream(cap, 0, 10, 30_000, 120_000_000_000)
    cap.command(5000, sim.CMD_REBOOT, struct.pack('<H', 100), b'\x00')
    cap.push(318_000, m, ts_ns)
    stream(cap, 1_240_000, 10, udp_cnt + 2600, ts_ns + 1_240_000_000)
    m.work_state = sim.WS_ERROR
    cap.push(1_248_000, m, ts_ns + 1_250_000_000)
    cap.push(1_318_000, m, ts_ns + 1_320_000_000)
    cap.push_seq = 71  # the counter restarts at power-on
    boot_ns = 0  # LiDAR time counts from the new power-on, 2.1 s before the first push
    m.work_state = sim.WS_MOTORSTARTUP
    for k in range(5):
        cap.push(8_915_000 + k * s, m, boot_ns + 2_100_000_000 + k * 1_000_000_000)
    m.work_state = sim.WS_READY
    cap.push(13_543_000, m, boot_ns + 6_728_000_000)
    m.work_state = sim.WS_SAMPLING
    cap.push(13_563_000, m, boot_ns + 6_748_000_000)
    stream(cap, 13_564_000, 10, 0, 11_016_257_560)
    return cap


FIXTURES: dict[str, Callable[[], Capture]] = {
    'discovery_setup': discovery_setup,
    'inquire': inquire,
    'pcl_types': pcl_types,
    'imu': imu,
    'fov': fov,
    'firmware_log': firmware_log,
    'debug_data': debug_data,
    'reboot': reboot,
}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    for name, build in FIXTURES.items():
        path = OUT / f'{name}.pcap'
        path.write_bytes(build().pcap())
        print(f'wrote {path} ({path.stat().st_size} bytes)')


if __name__ == '__main__':
    main()

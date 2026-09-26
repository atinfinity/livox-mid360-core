#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Generates tests/generated/golden_vectors.hpp from the Python reference implementation.

Run from the repository root:  python3 tools/gen_golden_vectors.py
The output is committed so the C++ tests need no Python at build time.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import livox_mid360_proto as p  # noqa: E402

OUT = pathlib.Path(__file__).resolve().parent.parent / 'tests' / 'generated' / 'golden_vectors.hpp'


def cpp_bytes(name: str, b: bytes) -> str:
    body = ', '.join(f'0x{x:02X}' for x in b)
    return (
        f'inline constexpr unsigned char {name}[] = {{{body}}};\n'
        f'inline constexpr std::size_t {name}_len = {len(b)};\n'
    )


def vectors() -> list[tuple[str, bytes]]:
    """Return the named wire-format vectors, in header order (shared with gen_fuzz_corpus.py)."""
    out: list[tuple[str, bytes]] = []
    # 0x0000 discovery request (no data)
    out.append(('discovery_req', p.CommandFrame(1, 0x0000, 0, 0).encode()))
    # 0x0000 discovery ack (as the LiDAR would send it)
    ack = (
        bytes([0, 9])
        + b'47MDL9K0010001\0\0'
        + bytes([192, 168, 1, 12])
        + (56100).to_bytes(2, 'little')
    )
    out.append(('discovery_ack', p.CommandFrame(1, 0x0000, 1, 1, ack).encode()))

    # 0x0100 configure host ip cfgs + data type + imu enable
    kvs = [
        (0x0005, p.encode_host_ipcfg('192.168.1.5', 56201, 56200)),
        (0x0006, p.encode_host_ipcfg('192.168.1.5', 56301, 56300)),
        (0x0007, p.encode_host_ipcfg('192.168.1.5', 56401, 56400)),
        (0x0000, b'\x01'),
        (0x001C, b'\x01'),
    ]
    out.append(
        (
            'param_config_req',
            p.CommandFrame(2, 0x0100, 0, 0, p.encode_param_config(kvs)).encode(),
        )
    )
    out.append(('param_config_ack_ok', p.CommandFrame(2, 0x0100, 1, 1, bytes([0, 0, 0])).encode()))
    out.append(
        (
            'param_config_ack_err',
            p.CommandFrame(2, 0x0100, 1, 1, bytes([0x22, 0x00, 0x80])).encode(),
        )
    )

    # work_tgt_mode = SAMPLING
    out.append(
        (
            'set_sampling_req',
            p.CommandFrame(3, 0x0100, 0, 0, p.encode_param_config([(0x001A, b'\x01')])).encode(),
        )
    )

    # 0x0101 inquire sn, version_app, cur_work_state
    out.append(
        (
            'param_inquire_req',
            p.CommandFrame(
                4, 0x0101, 0, 0, p.encode_param_inquire([0x8000, 0x8002, 0x8006])
            ).encode(),
        )
    )
    inq_ack = (
        bytes([0])
        + (3).to_bytes(2, 'little')
        + p.encode_kv_list(
            [(0x8000, b'47MDL9K0010001\0\0'), (0x8002, bytes([13, 18, 2, 44])), (0x8006, b'\x01')]
        )
    )
    out.append(('param_inquire_ack', p.CommandFrame(4, 0x0101, 1, 1, inq_ack).encode()))

    # 0x0102 push
    push = (
        (4).to_bytes(2, 'little')
        + b'\0\0'
        + p.encode_kv_list(
            [
                (0x8006, b'\x01'),
                (0x8007, (4321).to_bytes(4, 'little', signed=True)),
                (0x800E, (0x0020).to_bytes(2, 'little')),
                (
                    0x8011,
                    b''.join(
                        x.to_bytes(4, 'little') for x in [0x01020002, 0x02110004, 0, 0, 0, 0, 0, 0]
                    ),
                ),
            ]
        )
    )
    out.append(('info_push', p.CommandFrame(7, 0x0102, 0, 1, push).encode()))

    # control commands
    out.append(('reboot_req', p.CommandFrame(5, 0x0200, 0, 0, p.encode_reboot(100)).encode()))
    out.append(('factory_reset_req', p.CommandFrame(6, 0x0201, 0, 0, b'\0' * 16).encode()))
    out.append(
        (
            'gps_time_req',
            p.CommandFrame(
                8, 0x0202, 0, 0, p.encode_set_gps_timestamp(1700000000_000000000)
            ).encode(),
        )
    )

    # data packets
    pts32 = [
        (1000 + i, -2000 + 3 * i, 500 - i, (i * 7) % 256, i % 4 | ((i % 3) << 2))
        for i in range(96)
    ]
    out.append(
        (
            'pcl32',
            p.DataPacket(
                1035, 96, 17, 0, 1, 1, 1700000000_123456789, p.pack_samples(1, pts32)
            ).encode(),
        )
    )
    pts16 = [(100 + i, -200 + 3 * i, 50 - i, (i * 5) % 256, i % 4) for i in range(96)]
    out.append(
        (
            'pcl16',
            p.DataPacket(1035, 96, 18, 0, 2, 0, 987654321, p.pack_samples(2, pts16)).encode(),
        )
    )
    sph = [
        (1000 + i, (i * 180) % 18001, (i * 360) % 36001, (i * 3) % 256, i % 4) for i in range(96)
    ]
    out.append(
        ('pcl_sph', p.DataPacket(1035, 96, 19, 0, 3, 2, 5555, p.pack_samples(3, sph)).encode())
    )
    imu = [(0.01, -0.02, 0.03, 0.0, 0.0, 1.0)]
    out.append(
        (
            'imu',
            p.DataPacket(0, 1, 42, 0, 0, 1, 1700000000_000000042, p.pack_samples(0, imu)).encode(),
        )
    )

    return out


# Packet sequence for the frame assembler (issue #6): (udp_cnt, frame_cnt) per packet, 8 points
# each, 500 us apart. frame_cnt changes close frames; udp_cnt 3 -> 5 is one drop; udp_cnt 0 at
# the start of frame 2 is the documented per-frame reset, not a drop.
FRAME_SEQ = [(0, 0), (1, 0), (2, 0), (3, 1), (5, 1), (0, 2), (1, 2), (2, 2)]
FRAME_SEQ_T0_NS = 1_700_000_000_000_000_000
FRAME_SEQ_STEP_NS = 500_000
FRAME_SEQ_DOTS = 8


def frame_seq_packets() -> list[bytes]:
    out = []
    for i, (udp_cnt, frame_cnt) in enumerate(FRAME_SEQ):
        pts = [(1000 * i + j, -j, 10 * j, (7 * j) % 256, j % 4) for j in range(FRAME_SEQ_DOTS)]
        out.append(
            p.DataPacket(
                FRAME_SEQ_STEP_NS // 100,
                FRAME_SEQ_DOTS,
                udp_cnt,
                frame_cnt,
                1,
                0,
                FRAME_SEQ_T0_NS + i * FRAME_SEQ_STEP_NS,
                p.pack_samples(1, pts),
            ).encode()
        )
    return out


def frame_seq_expected() -> tuple[list[int], int]:
    """(points per closed frame in order, total drops) from an independent model."""
    frames, drops, cur, last = [], 0, 0, None
    for udp_cnt, frame_cnt in FRAME_SEQ:
        if last is not None:
            if frame_cnt != last[1]:
                frames.append(cur)
                cur = 0
            gap = (udp_cnt - last[0] - 1) & 0xFFFF
            if not (udp_cnt == 0 and frame_cnt != last[1]):
                drops += gap
        cur += FRAME_SEQ_DOTS
        last = (udp_cnt, frame_cnt)
    frames.append(cur)
    return frames, drops


def main() -> None:
    parts = [
        '// GENERATED by tools/gen_golden_vectors.py from tools/livox_mid360_proto.py.'
        ' DO NOT EDIT.\n',
        '#pragma once\n#include <cstddef>\n\nnamespace golden {\n\n',
    ]

    # CRC check values ("123456789")
    parts.append(
        f'inline constexpr unsigned crc16_check = 0x{p.crc16_ccitt_false(b"123456789"):04X};\n'
    )
    parts.append(f'inline constexpr unsigned crc32_check = 0x{p.crc32(b"123456789"):08X};\n')
    hdr18 = bytes([0xAA, 0, 24, 0, 1, 0, 0, 0, 0, 0, 0, 0] + [0] * 6)
    parts.append(
        'inline constexpr unsigned crc16_discovery_header = '
        f'0x{p.crc16_ccitt_false(hdr18):04X};\n\n'
    )

    for name, b in vectors():
        parts.append(cpp_bytes(name, b))

    pkts = frame_seq_packets()
    offsets, blob = [], b''
    for b in pkts:
        offsets.append(len(blob))
        blob += b
    offsets.append(len(blob))
    parts.append('\n// Frame assembler sequence (issue #6)\n')
    parts.append(cpp_bytes('frame_seq', blob))
    parts.append(
        'inline constexpr std::size_t frame_seq_offsets[] = {'
        + ', '.join(str(o) for o in offsets)
        + '};\n'
    )
    parts.append(f'inline constexpr std::size_t frame_seq_count = {len(pkts)};\n')
    frames, drops = frame_seq_expected()
    parts.append(
        'inline constexpr std::size_t frame_seq_points[] = {'
        + ', '.join(str(n) for n in frames)
        + '};\n'
    )
    parts.append(f'inline constexpr std::size_t frame_seq_frames = {len(frames)};\n')
    parts.append(f'inline constexpr unsigned frame_seq_dropped = {drops};\n')
    parts.append(f'inline constexpr unsigned long long frame_seq_t0_ns = {FRAME_SEQ_T0_NS}ull;\n')
    parts.append(f'inline constexpr unsigned frame_seq_step_ns = {FRAME_SEQ_STEP_NS};\n')
    parts.append('\n}  // namespace golden\n')
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(''.join(parts))
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()

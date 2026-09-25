#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generates the libFuzzer seed corpus in tests/fuzz/corpus/<target>/ (issue #24).

Run from the repository root:  python3 tools/gen_fuzz_corpus.py
The seeds are built from the same Python reference implementation as the golden vectors so
that each fuzz target starts from well-formed input and reaches the interesting branches
within a short smoke run. The output is committed; CI regenerates it and diffs.
"""

from __future__ import annotations

import pathlib
import shutil
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import livox_mid360_proto as p  # noqa: E402
from gen_golden_vectors import vectors  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent / "tests" / "fuzz" / "corpus"

LIDAR_IP = "192.168.1.12"
HOST_IP = "192.168.1.5"


def ip_bytes(ip: str) -> bytes:
    return bytes(int(x) for x in ip.split("."))


def endpoint(ip: str, port: int) -> bytes:
    return ip_bytes(ip) + struct.pack("<H", port)


def corrupt(b: bytes, index: int = -1) -> bytes:
    out = bytearray(b)
    out[index] ^= 0xFF
    return bytes(out)


def session_prefix(seq: int, cmd_id: int, from_ip: str = LIDAR_IP, lidar_ip: str = LIDAR_IP):
    """[seq u32][cmd_id u16][from ip 4][from port u16][lidar ip 4]."""
    return struct.pack("<IH", seq, cmd_id) + endpoint(from_ip, p.PORT_CMD) + ip_bytes(lidar_ip)


def discovery_ack(sn: bytes, ret: int = 0, port: int = p.PORT_CMD, seq: int = 1) -> bytes:
    data = (
        bytes([ret, 9]) + sn.ljust(16, b"\0")[:16] + ip_bytes(LIDAR_IP) + struct.pack("<H", port)
    )
    return p.CommandFrame(seq, 0x0000, 1, 1, data).encode()


def build() -> dict[str, dict[str, bytes]]:
    g = dict(vectors())
    frames = {k: v for k, v in g.items() if not k.startswith(("pcl", "imu"))}
    packets = {k: v for k, v in g.items() if k.startswith(("pcl", "imu"))}

    corpus: dict[str, dict[str, bytes]] = {}
    corpus["fuzz_command_frame"] = dict(frames)
    corpus["fuzz_data_packet"] = dict(packets)

    kv = p.encode_kv_list(
        [(0x8000, b"47MDL9K0010001\0\0"), (0x8002, bytes([13, 18, 2, 44])), (0x8006, b"\x01")]
    )
    corpus["fuzz_key_value_list"] = {
        "three_keys": bytes([3]) + kv,
        "host_ipcfg": bytes([1])
        + p.encode_kv_list([(0x0005, p.encode_host_ipcfg(HOST_IP, 56201, 56200))]),
        "hms": bytes([1]) + p.encode_kv_list([(0x8011, bytes(32))]),
        "empty": bytes([0]),
    }

    corpus["fuzz_transport_text"] = {
        "lidar": f"{LIDAR_IP}:{p.PORT_CMD}".encode(),
        "any": b"0.0.0.0:0",
        "max": b"255.255.255.255:65535",
        "no_port": LIDAR_IP.encode(),
        "port_overflow": b"192.168.1.12:65536",
        "trailing_colon": b"192.168.1.12:",
        "spaces": b" 192.168.1.12:56100 ",
        "ipv6": b"[::1]:56100",
    }

    # (seq, cmd_id) of the golden ACKs, see gen_golden_vectors.py
    acks = {
        "discovery_ack": (1, 0x0000),
        "param_config_ack_ok": (2, 0x0100),
        "param_config_ack_err": (2, 0x0100),
        "param_inquire_ack": (4, 0x0101),
    }
    sess: dict[str, bytes] = {}
    for name, (seq, cmd_id) in acks.items():
        frame = g[name]
        sess[f"{name}_match"] = session_prefix(seq, cmd_id) + frame
        sess[f"{name}_late_seq"] = session_prefix(seq + 1, cmd_id) + frame
        sess[f"{name}_other_ip"] = session_prefix(seq, cmd_id, from_ip="192.168.1.99") + frame
        sess[f"{name}_bad_crc"] = session_prefix(seq, cmd_id) + corrupt(frame)
    sess["reboot_ack_match"] = (
        session_prefix(5, 0x0200) + p.CommandFrame(5, 0x0200, 1, 1, bytes([0])).encode()
    )
    sess["config_ack_reboot_effect"] = (
        session_prefix(2, 0x0100) + p.CommandFrame(2, 0x0100, 1, 1, bytes([0x21, 0, 0])).encode()
    )
    sess["info_push_not_ack"] = session_prefix(7, 0x0102) + g["info_push"]
    sess["request_not_ack"] = session_prefix(3, 0x0100) + g["set_sampling_req"]
    corpus["fuzz_session_ack"] = sess

    frm = endpoint(LIDAR_IP, p.PORT_DISCOVERY)
    corpus["fuzz_discovery_ack"] = {
        "golden": frm + g["discovery_ack"],
        "ret_err": frm + discovery_ack(b"47MDL9K0010001", ret=1),
        "sn_no_nul": frm + discovery_ack(b"0123456789ABCDEF"),
        "sn_empty": frm + discovery_ack(b""),
        "cmd_port_zero": frm + discovery_ack(b"47MDL9K0010001", port=0),
        "not_ack": frm + g["discovery_req"],
        "bad_crc": frm + corrupt(g["discovery_ack"]),
    }
    return corpus


def main() -> None:
    corpus = build()
    for target, files in corpus.items():
        d = ROOT / target
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)
        for name, b in files.items():
            (d / name).write_bytes(b)
    n = sum(len(f) for f in corpus.values())
    print(f"wrote {n} seeds under {ROOT}")


if __name__ == "__main__":
    main()

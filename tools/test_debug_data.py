#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Unit tests of livox_mid360_debug_data.py: both file formats and the raw -> sdk2 conversion.

python3 -m unittest tools/test_debug_data.py
"""

from __future__ import annotations

import contextlib
import io
import json
import os
import pathlib
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import livox_mid360_debug_data as dbg  # noqa: E402

SERIAL = 'SIM0000000000001'

# The header Livox-SDK2 (c0796f0) writes for this serial number and dev_type 9, produced by its
# own DebugPointCloudHandler code and FastCRC16::ccitt (#107): file_ver 1, dev_type 9,
# data_type 1, the SN, 107 zero bytes, crc16 0x5A1D little-endian.
SDK2_HEADER = bytes([1, 9, 1]) + SERIAL.encode() + bytes(107) + b'\x1d\x5a'


def sim_payload(seq: int, size: int = 20) -> bytes:
    return struct.pack('<I', seq) + bytes((seq + i) & 0xFF for i in range(size - 4))


def raw_file(payloads: list[bytes], serial: str = SERIAL) -> bytes:
    out = dbg.FILE_HEADER.pack(dbg.MAGIC, dbg.VERSION, serial.encode(), 123)
    for i, p in enumerate(payloads):
        out += dbg.RECORD_HEADER.pack(1000 + i, 60301, len(p)) + p
    return out


class DebugDataTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = pathlib.Path(self.tmp.name)

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def run_tool(self, *argv: str) -> tuple[int, list[dict]]:
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
            status = dbg.main(list(argv))
        return status, [json.loads(line) for line in out.getvalue().splitlines()]

    def write(self, name: str, data: bytes) -> str:
        path = self.dir / name
        path.write_bytes(data)
        return str(path)

    def test_sdk2_header_matches_livox_sdk2(self) -> None:
        self.assertEqual(len(SDK2_HEADER), 128)
        self.assertEqual(dbg.encode_sdk2_header(SERIAL), SDK2_HEADER)

    def test_sdk2_header_pads_a_short_serial(self) -> None:
        h = dbg.encode_sdk2_header('ABC', 9)
        self.assertEqual(h[3:19], b'ABC' + bytes(13))
        self.assertEqual(dbg.parse_header(h).serial, 'ABC')

    def test_reads_an_sdk2_file(self) -> None:
        body = sim_payload(7) + sim_payload(8)
        path = self.write('a.sdk2', SDK2_HEADER + body)
        status, lines = self.run_tool(path)
        self.assertEqual(status, 0)
        self.assertEqual(lines[-1]['format'], 'sdk2')
        self.assertEqual(lines[-1]['serial'], SERIAL)
        self.assertEqual(lines[-1]['dev_type'], 9)
        self.assertIsNone(lines[-1]['packets'])  # no framing
        self.assertEqual(lines[-1]['payload_bytes'], len(body))

        status, lines = self.run_tool(path, '--datagram-size', '20', '--check-sim', '--json')
        self.assertEqual(status, 0)
        self.assertEqual(lines[-1]['packets'], 2)
        self.assertEqual([x['length'] for x in lines[:-1]], [20, 20])
        self.assertIsNone(lines[0]['receive_time_ns'])

    def test_sdk2_check_sim_needs_the_datagram_size(self) -> None:
        path = self.write('a.sdk2', SDK2_HEADER + sim_payload(1))
        status, _ = self.run_tool(path, '--check-sim')
        self.assertEqual(status, 1)

    def test_sdk2_cut_datagram_and_empty_body(self) -> None:
        path = self.write('cut.sdk2', SDK2_HEADER + sim_payload(1) + b'\x02\x00')
        status, lines = self.run_tool(path, '--datagram-size', '20')
        self.assertEqual(status, 2)
        self.assertEqual(lines[-1]['packets'], 1)
        status, _ = self.run_tool(self.write('empty.sdk2', SDK2_HEADER))
        self.assertEqual(status, 3)

    def test_bad_sdk2_crc_is_rejected(self) -> None:
        bad = bytearray(SDK2_HEADER)
        bad[3] ^= 0x01
        status, _ = self.run_tool(self.write('bad.sdk2', bytes(bad)))
        self.assertEqual(status, 2)

    def test_raw_to_sdk2(self) -> None:
        payloads = [sim_payload(3, 30), sim_payload(4, 12)]
        path = self.write('a.raw', raw_file(payloads))
        out = self.dir / 'a.LivoxDebugPointCloudData'
        status, lines = self.run_tool(path, '--to-sdk2', str(out))
        self.assertEqual(status, 0)
        self.assertEqual(lines[-1]['format'], 'raw')
        self.assertEqual(out.read_bytes(), SDK2_HEADER + b''.join(payloads))

        status, _ = self.run_tool(str(out), '--to-sdk2', str(self.dir / 'again'))
        self.assertEqual(status, 1)  # only a raw file converts

    def test_payload_of_both_formats(self) -> None:
        payloads = [sim_payload(1), sim_payload(2)]
        for name, data in (
            ('a.raw', raw_file(payloads)),
            ('a.sdk2', SDK2_HEADER + b''.join(payloads)),
        ):
            with self.subTest(name):
                out = self.dir / (name + '.payload')
                status, _ = self.run_tool(self.write(name, data), '--payload', str(out))
                self.assertEqual(status, 0)
                self.assertEqual(out.read_bytes(), b''.join(payloads))

    def test_unknown_file_is_rejected(self) -> None:
        status, _ = self.run_tool(self.write('junk', bytes(200)))
        self.assertEqual(status, 2)


if __name__ == '__main__':
    unittest.main()

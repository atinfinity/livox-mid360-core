#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Read a debug raw data file written by `livox-mid360-cli debug-data` (docs/debug_data.md).

Usage:
  livox_mid360_debug_data.py FILE               # summary
  livox_mid360_debug_data.py FILE --json        # one JSON object per datagram, then the summary
  livox_mid360_debug_data.py FILE --payload OUT # concatenated payloads, no framing
  livox_mid360_debug_data.py FILE --check-sim   # verify the simulator's counter and pattern

The format is provisional (version 0). Exit codes: 0 ok, 1 usage, 2 bad file, 3 no datagram.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import pathlib
import struct
import sys

MAGIC = b'LMDBGRAW'
VERSION = 0
FILE_HEADER = struct.Struct('<8sI16sQ')
RECORD_HEADER = struct.Struct('<QHI')


class FormatError(Exception):
    pass


@dataclass
class Header:
    version: int
    serial: str
    start_time_ns: int


@dataclass
class Record:
    receive_time_ns: int
    source_port: int
    payload: bytes


def parse_header(data: bytes) -> Header:
    if len(data) < FILE_HEADER.size:
        raise FormatError('file shorter than the header')
    magic, version, serial, start = FILE_HEADER.unpack_from(data)
    if magic != MAGIC:
        raise FormatError('bad magic')
    if version != VERSION:
        raise FormatError(f'unsupported version {version}')
    return Header(version, serial.rstrip(b'\0').decode('ascii', 'replace'), start)


def iter_records(data: bytes):
    """Yield every complete record; raise FormatError at a record that is cut."""
    pos = FILE_HEADER.size
    while pos < len(data):
        if pos + RECORD_HEADER.size > len(data):
            raise FormatError(f'record header cut at offset {pos}')
        t, port, length = RECORD_HEADER.unpack_from(data, pos)
        pos += RECORD_HEADER.size
        if pos + length > len(data):
            raise FormatError(f'payload cut at offset {pos}')
        yield Record(t, port, data[pos : pos + length])
        pos += length


def check_sim(index: int, payload: bytes, first_seq: int) -> str | None:
    """Return what is wrong with a simulator datagram: `seq u32` + bytes `(seq + i) & 0xFF`."""
    if len(payload) < 4:
        return 'shorter than the counter'
    (seq,) = struct.unpack_from('<I', payload)
    if seq != first_seq + index:
        return f'seq {seq}, expected {first_seq + index}'
    body = payload[4:]
    if body != bytes((seq + i) & 0xFF for i in range(len(body))):
        return 'pattern mismatch'
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('file', type=pathlib.Path)
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--payload', type=pathlib.Path)
    ap.add_argument('--check-sim', action='store_true')
    args = ap.parse_args(argv)

    try:
        data = args.file.read_bytes()
        header = parse_header(data)
    except (OSError, FormatError) as e:
        print(f'{args.file}: {e}', file=sys.stderr)
        return 2

    packets = 0
    size = 0
    first_seq = 0
    error = None
    payload_out = args.payload.open('wb') if args.payload else None
    try:
        for rec in iter_records(data):
            if args.check_sim:
                if packets == 0 and len(rec.payload) >= 4:
                    (first_seq,) = struct.unpack_from('<I', rec.payload)
                bad = check_sim(packets, rec.payload, first_seq)
                if bad:
                    error = f'datagram {packets}: {bad}'
                    break
            if args.json:
                print(
                    json.dumps(
                        {
                            'index': packets,
                            'receive_time_ns': rec.receive_time_ns,
                            'source_port': rec.source_port,
                            'length': len(rec.payload),
                        }
                    )
                )
            if payload_out:
                payload_out.write(rec.payload)
            packets += 1
            size += len(rec.payload)
    except FormatError as e:
        error = str(e)
    finally:
        if payload_out:
            payload_out.close()

    print(
        json.dumps(
            {
                'version': header.version,
                'serial': header.serial,
                'start_time_ns': header.start_time_ns,
                'packets': packets,
                'payload_bytes': size,
                'error': error,
            }
        )
    )
    if error:
        print(f'{args.file}: {error}', file=sys.stderr)
        return 2
    return 0 if packets else 3


if __name__ == '__main__':
    sys.exit(main())

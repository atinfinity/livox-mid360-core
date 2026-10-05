#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Read a debug raw data file written by `livox-mid360-cli debug-data` (docs/debug_data.md).

Usage:
  livox_mid360_debug_data.py FILE               # summary
  livox_mid360_debug_data.py FILE --json        # one JSON object per datagram, then the summary
  livox_mid360_debug_data.py FILE --payload OUT # concatenated payloads, no framing
  livox_mid360_debug_data.py FILE --check-sim   # verify the simulator's counter and pattern
  livox_mid360_debug_data.py FILE --to-sdk2 OUT # convert a raw file to .LivoxDebugPointCloudData

Both formats are read: `raw` (provisional, version 0, one record per datagram) and `sdk2`
(the .LivoxDebugPointCloudData file of Livox-SDK2, #107). An sdk2 file has no framing, so
its datagrams are only told apart with --datagram-size. Exit codes: 0 ok, 1 usage, 2 bad
file, 3 no datagram.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
import pathlib
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import livox_mid360_proto as proto  # noqa: E402

MAGIC = b'LMDBGRAW'
VERSION = 0
FILE_HEADER = struct.Struct('<8sI16sQ')
RECORD_HEADER = struct.Struct('<QHI')

# Livox-SDK2 LivoxLidarDebugPointCloudFileHeader (sdk_core/comm/define.h, packed): file_ver,
# dev_type, data_type, sn[16], rsvd[107], crc16 (CRC-16/CCITT-FALSE of the bytes before it).
SDK2_HEADER = struct.Struct('<BBB16s107sH')
SDK2_FILE_VERSION = 1
SDK2_DATA_TYPE = 1
DEV_TYPE_MID360 = 9


class FormatError(Exception):
    pass


@dataclass
class Header:
    format: str  # 'raw' or 'sdk2'
    version: int
    serial: str
    start_time_ns: int | None = None  # raw only
    dev_type: int | None = None  # sdk2 only
    data_type: int | None = None  # sdk2 only
    size: int = FILE_HEADER.size


@dataclass
class Record:
    receive_time_ns: int | None
    source_port: int | None
    payload: bytes


def encode_sdk2_header(serial: str, dev_type: int = DEV_TYPE_MID360) -> bytes:
    """Return the 128-byte header Livox-SDK2 writes in front of the datagrams."""
    head = SDK2_HEADER.pack(
        SDK2_FILE_VERSION, dev_type, SDK2_DATA_TYPE, serial.encode('ascii')[:16], bytes(107), 0
    )
    crc = proto.crc16_ccitt_false(head[: SDK2_HEADER.size - 2])
    return head[:-2] + struct.pack('<H', crc)


def parse_header(data: bytes) -> Header:
    if data[: len(MAGIC)] == MAGIC:
        if len(data) < FILE_HEADER.size:
            raise FormatError('file shorter than the header')
        magic, version, serial, start = FILE_HEADER.unpack_from(data)
        if version != VERSION:
            raise FormatError(f'unsupported version {version}')
        return Header('raw', version, serial.rstrip(b'\0').decode('ascii', 'replace'), start)
    if len(data) < SDK2_HEADER.size:
        raise FormatError('file shorter than the header')
    ver, dev_type, data_type, serial, _, crc = SDK2_HEADER.unpack_from(data)
    if crc != proto.crc16_ccitt_false(data[: SDK2_HEADER.size - 2]):
        raise FormatError('bad magic and no valid Livox-SDK2 header')
    if ver != SDK2_FILE_VERSION:
        raise FormatError(f'unsupported Livox-SDK2 file version {ver}')
    return Header(
        'sdk2',
        ver,
        serial.split(b'\0', 1)[0].decode('ascii', 'replace'),
        dev_type=dev_type,
        data_type=data_type,
        size=SDK2_HEADER.size,
    )


def iter_records(data: bytes, header: Header | None = None, datagram_size: int | None = None):
    """
    Yield every complete record; raise FormatError at a record that is cut.

    An sdk2 file yields nothing without `datagram_size`, and its records carry no receive
    time or port.
    """
    header = header or parse_header(data)
    pos = header.size
    if header.format == 'sdk2':
        if not datagram_size:
            return
        while pos < len(data):
            end = pos + datagram_size
            if end > len(data):
                raise FormatError(f'datagram cut at offset {pos}')
            yield Record(None, None, data[pos:end])
            pos = end
        return
    while pos < len(data):
        if pos + RECORD_HEADER.size > len(data):
            raise FormatError(f'record header cut at offset {pos}')
        t, port, length = RECORD_HEADER.unpack_from(data, pos)
        pos += RECORD_HEADER.size
        end = pos + length
        if end > len(data):
            raise FormatError(f'payload cut at offset {pos}')
        yield Record(t, port, data[pos:end])
        pos = end


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
    ap.add_argument(
        '--datagram-size', type=int, help='split an sdk2 file into datagrams of this many bytes'
    )
    ap.add_argument('--to-sdk2', type=pathlib.Path, help='write a raw file as an sdk2 file')
    ap.add_argument(
        '--dev-type',
        type=int,
        default=DEV_TYPE_MID360,
        help='dev_type of the sdk2 header written by --to-sdk2 (default 9, Mid-360)',
    )
    args = ap.parse_args(argv)
    if args.datagram_size is not None and args.datagram_size <= 0:
        ap.error('--datagram-size must be positive')

    try:
        data = args.file.read_bytes()
        header = parse_header(data)
    except (OSError, FormatError) as e:
        print(f'{args.file}: {e}', file=sys.stderr)
        return 2
    if args.to_sdk2 and header.format != 'raw':
        print(f'{args.file}: --to-sdk2 needs a raw file', file=sys.stderr)
        return 1
    framed = header.format == 'raw' or args.datagram_size is not None

    packets = 0
    size = 0
    first_seq = 0
    error = None
    payload_out = args.payload.open('wb') if args.payload else None
    sdk2_out = args.to_sdk2.open('wb') if args.to_sdk2 else None
    try:
        if sdk2_out:
            sdk2_out.write(encode_sdk2_header(header.serial, args.dev_type))
        if payload_out and not framed:
            payload_out.write(data[header.size :])
        for rec in iter_records(data, header, args.datagram_size):
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
            if sdk2_out:
                sdk2_out.write(rec.payload)
            packets += 1
            size += len(rec.payload)
    except FormatError as e:
        error = str(e)
    finally:
        if payload_out:
            payload_out.close()
        if sdk2_out:
            sdk2_out.close()

    if not framed:
        size = len(data) - header.size
    summary = {
        'format': header.format,
        'version': header.version,
        'serial': header.serial,
        'packets': packets if framed else None,
        'payload_bytes': size,
        'error': error,
    }
    if header.format == 'raw':
        summary['start_time_ns'] = header.start_time_ns
    else:
        summary['dev_type'] = header.dev_type
        summary['data_type'] = header.data_type
    print(json.dumps(summary))
    if error:
        print(f'{args.file}: {error}', file=sys.stderr)
        return 2
    if args.check_sim and not framed:
        print(f'{args.file}: --check-sim on an sdk2 file needs --datagram-size', file=sys.stderr)
        return 1
    return 0 if (packets if framed else size) else 3


if __name__ == '__main__':
    sys.exit(main())

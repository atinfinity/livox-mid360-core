#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Probe a Mid-360 for the protocol questions of issue #11 and record the raw answers.

Sends one request per question (discovery, inquire of whole key sets and of every known key,
unknown keys and command ids, rejected writes, a two-key write with one bad key) and listens
to the 0x0102 push for a while. Every request and ACK is written to a JSON file with its
decoded fields, so that the run is the evidence and can be re-read later.

Safe for a LiDAR in use elsewhere only in the sense that it restores what it changes: a write
the LiDAR accepts is undone with the value read before it, and key 0x0005 (push destination)
is pointed at this tool for the push window and then restored. It never reboots the LiDAR and
never writes its IP configuration, scan pattern, work mode or data type. Nothing else should
talk to the LiDAR while it runs.

Usage:
    livox_mid360_probe.py --lidar-ip 192.168.1.1xx --host-ip 192.168.1.50 --out probe.json
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import datetime
import json
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import livox_mid360_proto as proto  # noqa: E402

CMD_DISCOVERY, CMD_PARAM_CONFIG, CMD_PARAM_INQUIRE, CMD_INFO_PUSH = 0x0000, 0x0100, 0x0101, 0x0102
KEY_STATE_HOST, KEY_DETECT_MODE, KEY_IMU_EN, KEY_TIME_FILTER = 0x0005, 0x0018, 0x001C, 0x0026
KEY_SN, KEY_CORE_TEMP, KEY_CUR_WORK_STATE = 0x8000, 0x8007, 0x8006
UNKNOWN_KEY = 0x7FFF  # not in the wiki's key table
UNKNOWN_CMD_ID = 0x0FFF  # not in the wiki's command table

# Mirrors kSettingsKeys and kStatusKeys of include/livox/mid360/lidar_info.hpp.
SETTINGS_KEYS = [
    0x0000,
    0x0001,
    0x0004,
    0x0005,
    0x0006,
    0x0007,
    0x0012,
    0x0015,
    0x0016,
    0x0017,
    0x0018,
    0x0019,
    0x001A,
    0x001C,
    0x0026,
    0x002B,
]
STATUS_KEYS = [0x8006, 0x8007, 0x8008, 0x8009, 0x800A, 0x800B, 0x800C, 0x800E, 0x8010, 0x8011]


def hexkey(key: int) -> str:
    return f'0x{key:04X}'


def rc(ret: int | None) -> str | None:
    return None if ret is None else f'0x{ret:02X}'


def describe_kvs(kvs: list[tuple[int, bytes]]) -> list[dict]:
    return [
        {'key': hexkey(k), 'name': proto.KEY_NAMES.get(k, '?'), 'len': len(v), 'value': v.hex()}
        for k, v in kvs
    ]


class Prober:
    """Sends requests to one LiDAR and collects the probe records."""

    def __init__(self, args: argparse.Namespace, log: Callable[[str], None]) -> None:
        self.args = args
        self.log = log
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((args.host_ip, args.host_cmd_port))
        self.seq = 0
        self.probes: list[dict] = []
        self.lidar_ip = args.lidar_ip
        self.cmd_port = proto.PORT_CMD

    def close(self) -> None:
        self.sock.close()

    # -- transport ---------------------------------------------------------
    def request(self, cmd_id: int, data: bytes, to: tuple[str, int]) -> dict:
        """Send one REQ and wait for the ACK with the same seq_num and cmd_id."""
        self.seq += 1
        frame = proto.CommandFrame(self.seq, cmd_id, 0, 0, data)
        rec: dict = {
            'cmd_id': hexkey(cmd_id),
            'to': f'{to[0]}:{to[1]}',
            'seq': self.seq,
            'data': data.hex(),
        }
        start = time.monotonic()
        self.sock.sendto(frame.encode(), to)
        deadline = start + self.args.timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                rec['ack'] = None
                return rec
            self.sock.settimeout(left)
            try:
                d, src = self.sock.recvfrom(2048)
            except TimeoutError:
                continue
            try:
                f = proto.CommandFrame.parse(d)
            except ValueError:
                continue
            if f.cmd_type != 1 or f.seq_num != self.seq or f.cmd_id != cmd_id:
                continue
            rec['ack'] = {
                'from': f'{src[0]}:{src[1]}',
                'latency_ms': round((time.monotonic() - start) * 1000, 2),
                'data': f.data.hex(),
                'ret_code': rc(f.data[0]) if f.data else None,
            }
            return rec

    def cmd(self, cmd_id: int, data: bytes) -> dict:
        return self.request(cmd_id, data, (self.lidar_ip, self.cmd_port))

    def inquire(self, keys: list[int]) -> tuple[dict, int | None, list[tuple[int, bytes]]]:
        rec = self.cmd(CMD_PARAM_INQUIRE, proto.encode_param_inquire(keys))
        if rec['ack'] is None:
            return rec, None, []
        try:
            ret, kvs = proto.parse_param_inquire_ack(bytes.fromhex(rec['ack']['data']))
        except (struct.error, ValueError) as e:
            rec['ack']['decode_error'] = str(e)
            return rec, None, []
        rec['ack']['kvs'] = describe_kvs(kvs)
        return rec, ret, kvs

    def configure(self, kvs: list[tuple[int, bytes]]) -> tuple[dict, int | None, int | None]:
        rec = self.cmd(CMD_PARAM_CONFIG, proto.encode_param_config(kvs))
        if rec['ack'] is None:
            return rec, None, None
        try:
            ret, err = proto.parse_param_config_ack(bytes.fromhex(rec['ack']['data']))
        except struct.error as e:
            rec['ack']['decode_error'] = str(e)
            return rec, None, None
        rec['ack']['error_key'] = hexkey(err)
        return rec, ret, err

    def read_one(self, key: int) -> bytes | None:
        _, ret, kvs = self.inquire([key])
        if ret != 0:
            return None
        return next((v for k, v in kvs if k == key), None)

    # -- bookkeeping -------------------------------------------------------
    def record(self, name: str, checklist: str, requests: list[dict], **result) -> None:
        self.probes.append(
            {'name': name, 'checklist': checklist, 'requests': requests, 'result': result}
        )
        summary = ', '.join(f'{k}={v}' for k, v in result.items() if not isinstance(v, list))
        self.log(f'{name}: {summary}')

    def restore(self, key: int, value: bytes, requests: list[dict]) -> bool:
        rec, ret, _ = self.configure([(key, value)])
        rec['purpose'] = 'restore'
        requests.append(rec)
        return ret == 0

    # -- probes ------------------------------------------------------------
    def probe_discovery(self) -> bool:
        if self.lidar_ip:
            to = (self.lidar_ip, self.args.discovery_port)
            check = 'A unicast 0x0000 to port 56000 is answered; dev_type; cmd_port is 56100'
        else:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            to = ('255.255.255.255', self.args.discovery_port)
            check = 'dev_type value in the 0x0000 ACK; cmd_port in the ACK is 56100'
        rec = self.request(CMD_DISCOVERY, b'', to)
        if rec['ack'] is None:
            self.record('discovery', check, [rec], answered=False)
            return False
        ack = proto.parse_discovery_ack(bytes.fromhex(rec['ack']['data']))
        rec['ack'].update(ack)
        self.lidar_ip = self.lidar_ip or ack['lidar_ip']
        self.cmd_port = ack['cmd_port']
        self.record(
            'discovery',
            check,
            [rec],
            answered=True,
            sn=ack['sn'],
            dev_type=ack['dev_type'],
            lidar_ip=ack['lidar_ip'],
            cmd_port=ack['cmd_port'],
        )
        return True

    def probe_inquire_set(self, name: str, keys: list[int], check: str) -> None:
        rec, ret, kvs = self.inquire(keys)
        answered = [hexkey(k) for k, _ in kvs]
        self.record(
            name,
            check,
            [rec],
            ret_code=rc(ret),
            asked=len(keys),
            answered=len(kvs),
            missing=[hexkey(k) for k in keys if hexkey(k) not in answered],
        )

    def probe_inquire_each(self) -> None:
        requests, lengths, failed = [], {}, {}
        for key in sorted(proto.KEY_NAMES):
            rec, ret, kvs = self.inquire([key])
            requests.append(rec)
            if ret == 0 and kvs:
                lengths[hexkey(key)] = len(kvs[0][1])
            else:
                failed[hexkey(key)] = 'no ACK' if rec['ack'] is None else rec['ack']['ret_code']
        self.record(
            'inquire_each_key',
            'Keys the firmware lacks (e.g. imu_sensor_cfg 0x002B) and the value lengths',
            requests,
            supported=len(lengths),
            unsupported=len(failed),
            lengths=[f'{k}:{n}' for k, n in lengths.items()],
            failed=[f'{k}:{r}' for k, r in failed.items()],
        )

    def probe_inquire_unknown(self) -> None:
        rec, ret, kvs = self.inquire([KEY_SN, UNKNOWN_KEY])
        self.record(
            'inquire_unknown_key',
            '0x0101 naming a key the firmware lacks: whole request fails with 0x20 and the key '
            'as a zero-length entry?',
            [rec],
            ret_code=rc(ret),
            entries=[f'{hexkey(k)}:{len(v)}' for k, v in kvs],
        )

    def probe_unknown_cmd(self) -> None:
        rec = self.cmd(UNKNOWN_CMD_ID, b'')
        ack = rec['ack']
        self.record(
            'unknown_cmd_id',
            'Unknown cmd_id: is it ACKed at all, and with which code (0x01)?',
            [rec],
            acked=ack is not None,
            ret_code=ack['ret_code'] if ack else None,
        )

    def probe_rejected_write(self, name: str, check: str, key: int, value: bytes) -> None:
        """Write that should be rejected; restored if the LiDAR accepts it after all."""
        requests = []
        before = self.read_one(key) if key in proto.KEY_NAMES else None
        rec, ret, err = self.configure([(key, value)])
        requests.append(rec)
        restored = None
        if ret == 0 and before is not None:
            restored = self.restore(key, before, requests)
        self.record(
            name,
            check,
            requests,
            ret_code=rc(ret),
            error_key=None if err is None else hexkey(err),
            restored=restored,
        )

    def probe_partial_write(self) -> None:
        requests = []
        before = self.read_one(KEY_DETECT_MODE)
        if before is None:
            self.record(
                'partial_write',
                'two-key 0x0100',
                requests,
                skipped='detect_mode not readable',
            )
            return
        flipped = bytes([before[0] ^ 1])
        rec, ret, err = self.configure([(KEY_DETECT_MODE, flipped), (UNKNOWN_KEY, b'\0')])
        requests.append(rec)
        after = self.read_one(KEY_DETECT_MODE)
        applied = after == flipped
        restored = self.restore(KEY_DETECT_MODE, before, requests) if applied else None
        self.record(
            'partial_write',
            '0x0100 with several keys of which one is bad: nothing applied or partially '
            'applied; content of error_key',
            requests,
            ret_code=rc(ret),
            error_key=None if err is None else hexkey(err),
            good_key_applied=applied,
            restored=restored,
        )

    def probe_push(self) -> None:
        requests = []
        check = 'Keys carried by the 0x0102 push and the push period'
        if self.args.push_seconds <= 0:
            return
        before = self.read_one(KEY_STATE_HOST)
        if before is None:
            self.record('push', check, requests, skipped='key 0x0005 not readable')
            return
        listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listen.bind((self.args.host_ip, self.args.push_port))
        # The address the LiDAR reaches us on, also when bound to 0.0.0.0.
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect((self.lidar_ip, self.cmd_port))
        my_ip = probe.getsockname()[0]
        probe.close()
        port = listen.getsockname()[1]
        rec, ret, _ = self.configure(
            [(KEY_STATE_HOST, proto.encode_host_ipcfg(my_ip, port, proto.PORT_PUSH))]
        )
        rec['purpose'] = f'point the push at {my_ip}:{port}'
        requests.append(rec)
        times, keysets, states, first_values = [], set(), set(), None
        end = time.monotonic() + self.args.push_seconds
        try:
            while ret == 0 and time.monotonic() < end:
                listen.settimeout(max(0.01, end - time.monotonic()))
                try:
                    d, _ = listen.recvfrom(2048)
                except TimeoutError:
                    break
                try:
                    f = proto.CommandFrame.parse(d)
                    kvs = proto.parse_info_push(f.data)
                except (ValueError, struct.error):
                    continue
                if f.cmd_id != CMD_INFO_PUSH:
                    continue
                times.append(time.monotonic())
                keysets.add(tuple(hexkey(k) for k, _ in kvs))
                states.update(v[0] for k, v in kvs if k == KEY_CUR_WORK_STATE and v)
                if first_values is None:
                    first_values = describe_kvs(kvs)
        finally:
            listen.close()
            restored = self.restore(KEY_STATE_HOST, before, requests)
        gaps = [b - a for a, b in zip(times, times[1:], strict=False)]
        self.record(
            'push',
            check,
            requests,
            ret_code=rc(ret),
            pushes=len(times),
            period_ms=round(sum(gaps) / len(gaps) * 1000, 1) if gaps else None,
            period_min_max_ms=[round(min(gaps) * 1000, 1), round(max(gaps) * 1000, 1)]
            if gaps
            else [],
            key_sets=[list(s) for s in sorted(keysets)],
            work_states=sorted(states),
            first_push=first_values or [],
            restored=restored,
        )

    def run(self) -> bool:
        if not self.probe_discovery():
            return False
        self.probe_inquire_set(
            'inquire_settings_keys',
            SETTINGS_KEYS,
            'One 0x0101 with all 16 kSettingsKeys is answered (no key or size limit)',
        )
        self.probe_inquire_set(
            'inquire_status_keys',
            STATUS_KEYS,
            'One 0x0101 with all kStatusKeys is answered',
        )
        self.probe_inquire_each()
        self.probe_inquire_unknown()
        self.probe_unknown_cmd()
        self.probe_rejected_write(
            'write_read_only_key',
            'Return code of a write to a read-only key (0x22)',
            KEY_CORE_TEMP,
            b'\0' * 4,
        )
        self.probe_rejected_write(
            'write_unknown_key',
            'Return code of a write to an unknown key (0x20)',
            UNKNOWN_KEY,
            b'\0',
        )
        self.probe_rejected_write(
            'write_wrong_length',
            'Return code of a write with the wrong value length (0x23)',
            KEY_DETECT_MODE,
            b'\0\0',
        )
        for key in (KEY_DETECT_MODE, KEY_TIME_FILTER, KEY_IMU_EN):
            name = proto.KEY_NAMES[key]
            self.probe_rejected_write(
                f'write_out_of_range_{name}',
                f'{name} ({hexkey(key)}) with a value above 1: 0x03?',
                key,
                b'\x02',
            )
        self.probe_partial_write()
        self.probe_push()
        return True


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    p.add_argument('--lidar-ip', help='unicast discovery to this address (default: broadcast)')
    p.add_argument('--host-ip', default='0.0.0.0', help='local address to bind (default any)')
    p.add_argument('--out', help='JSON file to write (default: stdout)')
    p.add_argument('--timeout', type=float, default=1.0, help='seconds to wait for each ACK')
    p.add_argument('--push-seconds', type=float, default=5.0, help='push window, 0 to skip')
    p.add_argument('--host-cmd-port', type=int, default=56101, help='local command port')
    p.add_argument('--push-port', type=int, default=56201, help='local port for the push')
    p.add_argument('--discovery-port', type=int, default=proto.PORT_DISCOVERY)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    def log(line: str) -> None:
        print(line, file=sys.stderr)

    started = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')
    prober = Prober(args, log)
    try:
        ok = prober.run()
    finally:
        prober.close()
    report = {
        'tool': 'livox_mid360_probe',
        'started': started,
        'args': vars(args),
        'lidar_ip': prober.lidar_ip,
        'probes': prober.probes,
    }
    text = json.dumps(report, indent=2) + '\n'
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text)
        log(f'wrote {args.out}')
    else:
        sys.stdout.write(text)
    if not ok:
        log('no LiDAR answered the discovery')
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())

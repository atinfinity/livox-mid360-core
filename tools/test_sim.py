#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for the simulator's device model and an in-process end-to-end check.

python3 -m unittest tools/test_sim.py
"""

from __future__ import annotations

import io
import json
import os
import socket
import struct
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import livox_mid360_proto as proto  # noqa: E402
import livox_mid360_sim as sim  # noqa: E402


class DeviceModelTest(unittest.TestCase):
    """Work-state machine of DeviceModel, driven without sockets."""

    def setUp(self) -> None:
        self.m = sim.DeviceModel(startup_delay=1.0)
        self.events: list[tuple[int, int]] = []
        self.m.on_state = lambda o, n: self.events.append((o, n))

    def test_lidar_ipcfg_change_answers_reboot_effect(self) -> None:
        cur = self.m.settings[sim.KEY_LIDAR_IPCFG]
        self.assertEqual(self.m.configure([(sim.KEY_LIDAR_IPCFG, cur)]), (sim.RET_OK, 0))
        new = bytes([192, 168, 1, 101]) + cur[4:]
        self.assertEqual(
            self.m.configure([(sim.KEY_LIDAR_IPCFG, new)]), (sim.RET_PARAM_REBOOT_EFFECT, 0)
        )
        self.assertEqual(self.m.settings[sim.KEY_LIDAR_IPCFG], new)

    def test_power_on_selfcheck_idle_motorstartup_ready_sampling(self) -> None:
        # Figure: POWEROFF -> SELFCHECK -> IDLE, then IDLE -> MOTORSTARTUP -> READY -> target.
        self.m.power_on(now=100.0)
        self.assertEqual(self.m.work_state, sim.WS_SELFCHECK)
        self.m.tick(100.05)
        self.assertEqual(self.m.work_state, sim.WS_SELFCHECK)
        self.m.tick(100.1)  # self-check done: IDLE, target SAMPLING -> motor starts
        self.assertEqual(self.m.work_state, sim.WS_MOTORSTARTUP)
        self.m.tick(101.0)
        self.assertEqual(self.m.work_state, sim.WS_MOTORSTARTUP)
        self.m.tick(101.1)
        self.assertEqual(self.m.work_state, sim.WS_SAMPLING)  # factory work_tgt_mode
        self.assertEqual(
            self.events,
            [
                (sim.WS_SELFCHECK, sim.WS_IDLE),
                (sim.WS_IDLE, sim.WS_MOTORSTARTUP),
                (sim.WS_MOTORSTARTUP, sim.WS_READY),
                (sim.WS_READY, sim.WS_SAMPLING),
            ],
        )

    def _boot(self, now: float = 0.0) -> None:
        self.m.power_on(now)
        self.m.tick(now + 10.0)  # SELFCHECK done -> IDLE -> MOTORSTARTUP
        self.m.tick(now + 20.0)  # motor started -> READY -> SAMPLING
        self.assertEqual(self.m.work_state, sim.WS_SAMPLING)
        self.events.clear()

    def _set_target(self, state: int, now: float) -> None:
        ret, err = self.m.configure([(sim.KEY_WORK_TGT_MODE, bytes([state]))], now)
        self.assertEqual((ret, err), (sim.RET_OK, 0))

    def test_sampling_to_idle_passes_through_ready(self) -> None:
        self._boot()
        self._set_target(sim.WS_IDLE, 20.0)
        self.assertEqual(self.m.work_state, sim.WS_IDLE)  # immediate, READY has no dwell
        self.assertEqual(
            self.events, [(sim.WS_SAMPLING, sim.WS_READY), (sim.WS_READY, sim.WS_IDLE)]
        )

    def test_sampling_to_ready_and_back(self) -> None:
        self._boot()
        self._set_target(sim.WS_READY, 20.0)
        self.assertEqual(self.m.work_state, sim.WS_READY)
        self._set_target(sim.WS_SAMPLING, 21.0)
        self.assertEqual(self.m.work_state, sim.WS_SAMPLING)  # READY -> SAMPLING is direct
        self.assertEqual(
            self.events, [(sim.WS_SAMPLING, sim.WS_READY), (sim.WS_READY, sim.WS_SAMPLING)]
        )

    def test_idle_to_sampling_goes_through_motorstartup_and_ready(self) -> None:
        self._boot()
        self._set_target(sim.WS_IDLE, 20.0)
        self.events.clear()
        self._set_target(sim.WS_SAMPLING, 30.0)
        self.assertEqual(self.m.work_state, sim.WS_MOTORSTARTUP)
        self.m.tick(30.5)
        self.assertEqual(self.m.work_state, sim.WS_MOTORSTARTUP)
        self.m.tick(31.0)
        self.assertEqual(self.m.work_state, sim.WS_SAMPLING)
        self.assertEqual(
            self.events,
            [
                (sim.WS_IDLE, sim.WS_MOTORSTARTUP),
                (sim.WS_MOTORSTARTUP, sim.WS_READY),
                (sim.WS_READY, sim.WS_SAMPLING),
            ],
        )

    def test_idle_to_ready_stops_at_ready(self) -> None:
        self._boot()
        self._set_target(sim.WS_IDLE, 20.0)
        self._set_target(sim.WS_READY, 30.0)
        self.assertEqual(self.m.work_state, sim.WS_MOTORSTARTUP)
        self.m.tick(31.0)
        self.assertEqual(self.m.work_state, sim.WS_READY)
        self.m.tick(40.0)
        self.assertEqual(self.m.work_state, sim.WS_READY)
        self._set_target(sim.WS_IDLE, 41.0)
        self.assertEqual(self.m.work_state, sim.WS_IDLE)

    def test_target_written_during_timed_state_is_followed_afterwards(self) -> None:
        # During SELFCHECK / MOTORSTARTUP the target is stored and chased once the timed
        # state completes ([unverified] on hardware, #11).
        self.m.power_on(10.0)
        self._set_target(sim.WS_IDLE, 10.01)
        self.assertEqual(self.m.work_state, sim.WS_SELFCHECK)
        self.m.tick(10.1)
        self.assertEqual(self.m.work_state, sim.WS_IDLE)  # no motor start for target IDLE
        self._set_target(sim.WS_SAMPLING, 11.0)
        self.assertEqual(self.m.work_state, sim.WS_MOTORSTARTUP)
        self._set_target(sim.WS_IDLE, 11.5)  # changed mind during MOTORSTARTUP
        self.assertEqual(self.m.work_state, sim.WS_MOTORSTARTUP)
        self.m.tick(12.0)
        self.assertEqual(self.m.work_state, sim.WS_IDLE)
        self.assertEqual(
            self.events[-2:], [(sim.WS_MOTORSTARTUP, sim.WS_READY), (sim.WS_READY, sim.WS_IDLE)]
        )

    def test_work_tgt_mode_rejects_intermediate_and_undefined_values(self) -> None:
        self._boot()
        for v in (sim.WS_ERROR, sim.WS_SELFCHECK, sim.WS_MOTORSTARTUP, sim.WS_UPGRADE):
            self.assertEqual(
                self.m.configure([(sim.KEY_WORK_TGT_MODE, bytes([v]))], 20.0),
                (sim.RET_PARAM_NOT_SUPPORT, sim.KEY_WORK_TGT_MODE),
                v,
            )
        for v in (0, 3, 7, 10, 255):
            self.assertEqual(
                self.m.configure([(sim.KEY_WORK_TGT_MODE, bytes([v]))], 20.0),
                (sim.RET_OUT_OF_RANGE, sim.KEY_WORK_TGT_MODE),
                v,
            )
        self.assertEqual(self.m.work_state, sim.WS_SAMPLING)
        self.assertEqual(self.m.settings[sim.KEY_WORK_TGT_MODE], bytes([sim.WS_SAMPLING]))
        self.assertEqual(self.events, [])

    def test_error_and_upgrade_reject_target_writes_until_recovery(self) -> None:
        self._boot()
        for forced in (sim.WS_ERROR, sim.WS_UPGRADE):
            self.m.force_state(forced, 20.0)
            self.m.tick(25.0)
            self.assertEqual(self.m.work_state, forced)  # stays until recovery
            self.assertEqual(
                self.m.configure([(sim.KEY_WORK_TGT_MODE, bytes([sim.WS_IDLE]))], 26.0),
                (sim.RET_NOT_PERMIT_NOW, sim.KEY_WORK_TGT_MODE),
            )
            self.assertEqual(self.m.settings[sim.KEY_WORK_TGT_MODE], bytes([sim.WS_SAMPLING]))
            # Other keys are still configurable.
            self.assertEqual(self.m.configure([(sim.KEY_IMU_EN, b'\x01')], 26.0), (sim.RET_OK, 0))
        # "Abnormal disappearance": forced back into a work substate, the machine chases
        # the (unchanged) target again.
        self.m.force_state(sim.WS_IDLE, 30.0)
        self.assertEqual(self.m.work_state, sim.WS_IDLE)
        self.m.tick(30.0)
        self.assertEqual(self.m.work_state, sim.WS_MOTORSTARTUP)
        self.m.tick(31.0)
        self.assertEqual(self.m.work_state, sim.WS_SAMPLING)

    def test_force_state_leaves_target_alone(self) -> None:
        self._boot()
        self._set_target(sim.WS_IDLE, 20.0)
        self.m.force_state(sim.WS_ERROR, 21.0)
        self.assertEqual(self.m.settings[sim.KEY_WORK_TGT_MODE], bytes([sim.WS_IDLE]))
        self.m.force_state(sim.WS_SAMPLING, 22.0)  # forced SAMPLING, but target is IDLE
        self.m.tick(22.0)
        self.assertEqual(self.m.work_state, sim.WS_IDLE)
        self.m.force_state(sim.WS_MOTORSTARTUP, 23.0)  # timed even when forced
        self.m.tick(23.5)
        self.assertEqual(self.m.work_state, sim.WS_MOTORSTARTUP)
        self.m.tick(24.0)
        self.assertEqual(self.m.work_state, sim.WS_IDLE)

    def test_pattern_mode_accepts_only_non_repetitive(self) -> None:
        self._boot()
        self.assertEqual(
            self.m.configure([(sim.KEY_PATTERN_MODE, b'\x00')], 20.0), (sim.RET_OK, 0)
        )
        self.assertEqual(self.m.work_state, sim.WS_SAMPLING)  # no motor restart
        for value in (b'\x01', b'\x02'):
            self.assertEqual(
                self.m.configure([(sim.KEY_PATTERN_MODE, value)], 21.0),
                (sim.RET_PARAM_NOT_SUPPORT, sim.KEY_PATTERN_MODE),
            )
        self.assertEqual(
            self.m.configure([(sim.KEY_PATTERN_MODE, b'\x03')], 22.0),
            (sim.RET_OUT_OF_RANGE, sim.KEY_PATTERN_MODE),
        )
        self.assertEqual(self.m.settings[sim.KEY_PATTERN_MODE], b'\x00')

    def test_configure_rejects_read_only_unknown_and_wrong_length(self) -> None:
        self.assertEqual(
            self.m.configure([(sim.KEY_SN, b'x' * 16)]), (sim.RET_PARAM_READ_ONLY, sim.KEY_SN)
        )
        self.assertEqual(
            self.m.configure([(0x7FFF, b'\x00')]), (sim.RET_PARAM_NOT_SUPPORT, 0x7FFF)
        )
        self.assertEqual(
            self.m.configure([(sim.KEY_IMU_EN, b'\x00\x01')]),
            (sim.RET_PARAM_INVALID_LEN, sim.KEY_IMU_EN),
        )
        self.assertEqual(
            self.m.configure([(sim.KEY_PCL_DATA_TYPE, b'\x07')]),
            (sim.RET_OUT_OF_RANGE, sim.KEY_PCL_DATA_TYPE),
        )
        # Atomic: a bad key later in the list leaves earlier keys unapplied.
        ret, err = self.m.configure([(sim.KEY_IMU_EN, b'\x01'), (sim.KEY_SN, b'x' * 16)])
        self.assertEqual((ret, err), (sim.RET_PARAM_READ_ONLY, sim.KEY_SN))
        self.assertFalse(self.m.imu_enabled)

    def test_fov_out_of_range_is_rejected(self) -> None:
        ok = proto.encode_fov_cfg(0, 359, -9, 59)
        self.assertEqual(self.m.configure([(sim.KEY_FOV0, ok)]), (sim.RET_OK, 0))
        for bad in (
            proto.encode_fov_cfg(360, 0, 0, 0),
            proto.encode_fov_cfg(-1, 0, 0, 0),
            proto.encode_fov_cfg(0, 0, -10, 0),
            proto.encode_fov_cfg(0, 0, 0, 60),
        ):
            self.assertEqual(
                self.m.configure([(sim.KEY_FOV1, bad)]), (sim.RET_OUT_OF_RANGE, sim.KEY_FOV1)
            )
        self.assertEqual(self.m.settings[sim.KEY_FOV0], ok)
        self.assertEqual(self.m.settings[sim.KEY_FOV1], bytes(20))

    def test_fov_cropping_semantics(self) -> None:
        cart = (1000, 1000, 0, 0, 0)  # yaw 45, pitch 0
        self.assertTrue(self.m.keeps_point(1, cart))  # nothing enabled: keep all
        self.m.configure(
            [(sim.KEY_FOV0, proto.encode_fov_cfg(0, 90, -5, 5)), (sim.KEY_FOV_EN, b'\x01')]
        )
        self.assertTrue(self.m.keeps_point(1, cart))
        self.assertFalse(self.m.keeps_point(1, (-1000, 1000, 0, 0, 0)))  # yaw 135
        self.assertFalse(self.m.keeps_point(1, (1000, 1000, 500, 0, 0)))  # pitch ~19.5
        self.assertTrue(self.m.keeps_point(2, (100, 100, 0, 0, 0)))
        self.assertTrue(self.m.keeps_point(3, (5000, 9000, 4500, 0, 0)))  # zenith 90 = pitch 0
        self.assertFalse(self.m.keeps_point(3, (5000, 9000, 27000, 0, 0)))  # yaw 270
        # Wrap-around and half-open yaw; pitch closed; start == stop empty.
        self.m.configure([(sim.KEY_FOV0, proto.encode_fov_cfg(350, 10, 0, 0))])
        self.assertTrue(sim.in_fov_window((350, 10, 0, 0), 355.0, 0.0))
        self.assertTrue(sim.in_fov_window((350, 10, 0, 0), 5.0, 0.0))
        self.assertFalse(sim.in_fov_window((350, 10, 0, 0), 10.0, 0.0))
        self.assertFalse(sim.in_fov_window((350, 10, 0, 0), 5.0, 0.5))
        self.assertFalse(sim.in_fov_window((20, 20, -5, 5), 20.0, 0.0))
        # Second window adds points; mask decides which windows count.
        self.m.configure([(sim.KEY_FOV1, proto.encode_fov_cfg(90, 180, -5, 5))])
        self.assertFalse(self.m.keeps_point(1, (-1000, 1000, 0, 0, 0)))
        self.m.configure([(sim.KEY_FOV_EN, b'\x03')])
        self.assertTrue(self.m.keeps_point(1, (-1000, 1000, 0, 0, 0)))
        self.m.configure([(sim.KEY_FOV_EN, b'\x00')])
        self.assertTrue(self.m.keeps_point(1, (1000, -1000, 0, 0, 0)))

    def test_configure_and_inquire_round_trip(self) -> None:
        cfg = proto.encode_host_ipcfg('192.168.1.5', 56301, 56300)
        self.assertEqual(
            self.m.configure([(sim.KEY_PCL_HOST, cfg), (sim.KEY_IMU_EN, b'\x01')]), (0, 0)
        )
        ret, kvs = self.m.inquire(
            [sim.KEY_PCL_HOST, sim.KEY_IMU_EN, sim.KEY_SN, sim.KEY_CUR_WORK_STATE], 0
        )
        self.assertEqual(ret, sim.RET_OK)
        self.assertEqual(dict(kvs)[sim.KEY_PCL_HOST], cfg)
        self.assertEqual(dict(kvs)[sim.KEY_IMU_EN], b'\x01')
        self.assertEqual(dict(kvs)[sim.KEY_SN], b'SIM0000000000001')
        self.assertEqual(self.m.host(sim.KEY_PCL_HOST), ('192.168.1.5', 56301, 56300))
        self.assertIsNone(self.m.host(sim.KEY_IMU_HOST))
        ret, _ = self.m.inquire([0x7FFF], 0)
        self.assertEqual(ret, sim.RET_PARAM_NOT_SUPPORT)

    def test_reboot_keeps_settings_but_resets_target_mode(self) -> None:
        self.m.power_on(0.0)
        self.m.tick(1.0)
        self.m.configure(
            [(sim.KEY_IMU_EN, b'\x01'), (sim.KEY_WORK_TGT_MODE, bytes([sim.WS_IDLE]))]
        )
        self.m.reboot(5.0)
        self.assertEqual(self.m.work_state, sim.WS_SELFCHECK)
        self.assertTrue(self.m.imu_enabled)
        self.assertEqual(self.m.powerup_cnt, 2)
        self.m.tick(5.1)
        self.assertEqual(self.m.work_state, sim.WS_MOTORSTARTUP)  # target is SAMPLING again
        self.m.tick(6.1)
        self.assertEqual(self.m.work_state, sim.WS_SAMPLING)

    def test_factory_reset_restores_defaults(self) -> None:
        self.m.configure([(sim.KEY_IMU_EN, b'\x01'), (sim.KEY_PCL_DATA_TYPE, b'\x03')])
        self.m.hms = [0x02100003] + [0] * 7
        self.m.factory_reset(0.0)
        self.assertFalse(self.m.imu_enabled)
        self.assertEqual(self.m.pcl_data_type, 1)
        self.assertEqual(self.m.hms, [0] * 8)

    def test_gps_time_sets_offset_and_sync_type(self) -> None:
        self.m.set_gps_time(ns=1_000, now_ns=400)
        self.assertEqual(self.m.time_offset_ns, 600)
        self.assertEqual(self.m.time_sync_type, 2)
        ret, kvs = self.m.inquire([sim.KEY_TIME_OFFSET, sim.KEY_TIME_SYNC_TYPE], 0)
        self.assertEqual(struct.unpack('<q', dict(kvs)[sim.KEY_TIME_OFFSET])[0], 600)

    def test_push_payload_parses(self) -> None:
        self.m.hms = [0x02100003] + [0] * 7
        kvs = dict(proto.parse_info_push(self.m.push_payload(123)))
        self.assertEqual(kvs[sim.KEY_CUR_WORK_STATE], bytes([sim.WS_SELFCHECK]))
        self.assertEqual(struct.unpack('<8I', kvs[sim.KEY_HMS])[0], 0x02100003)
        self.assertEqual(kvs[sim.KEY_LOCAL_TIME], struct.pack('<Q', 123))
        # Every read-only key is pushed [unverified].
        self.assertEqual(sorted(kvs), sorted(range(0x8000, 0x800D)) + [0x800E, 0x8010, 0x8011])


class PointSourceTest(unittest.TestCase):
    """Point generator determinism."""

    def test_deterministic_for_seed(self) -> None:
        a = sim.PointSource(7).samples(1, 96)
        b = sim.PointSource(7).samples(1, 96)
        self.assertEqual(a, b)
        self.assertNotEqual(a, sim.PointSource(8).samples(1, 96))
        for dt in (1, 2, 3):
            data = proto.pack_samples(dt, sim.PointSource(1).samples(dt, 96))
            self.assertEqual(len(data), 96 * proto.SAMPLE_SIZE[dt])


class EndToEndTest(unittest.TestCase):
    """Runs the simulator in a thread with free ports and drives it over UDP."""

    def setUp(self) -> None:
        args = sim.build_parser().parse_args(
            [
                '--bind',
                '127.0.0.1',
                '--base-port',
                '0',
                '--startup-delay',
                '0.05',
                '--selfcheck-delay',
                '0.05',
                '--reboot-silence',
                '0.5',
            ]
        )
        self.out = io.StringIO()
        r, w = os.pipe()
        self.control_file = os.fdopen(r, 'r')
        self.control_w = os.fdopen(w, 'w')
        self.s = sim.Simulator(args, out=self.out, control=self.control_file)
        self.thread = threading.Thread(target=self.s.run, daemon=True)
        self.thread.start()
        self.host = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host.bind(('127.0.0.1', 0))
        self.host.settimeout(3)
        self.pcl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pcl.bind(('127.0.0.1', 0))
        self.pcl.settimeout(3)
        self.seq = 0

    def tearDown(self) -> None:
        self.send_control('{"cmd":"quit"}')
        self.thread.join(timeout=5)
        self.host.close()
        self.pcl.close()
        self.control_w.close()
        self.control_file.close()

    def send_control(self, line: str) -> None:
        self.control_w.write(line + '\n')
        self.control_w.flush()

    def request(self, cmd_id: int, data: bytes, to) -> proto.CommandFrame:
        self.seq += 1
        self.host.sendto(proto.CommandFrame(self.seq, cmd_id, 0, 0, data).encode(), to)
        while True:
            d, _ = self.host.recvfrom(2048)
            f = proto.CommandFrame.parse(d)
            if f.cmd_type == 1 and f.seq_num == self.seq:
                return f

    def events(self) -> list[dict]:
        return [json.loads(line) for line in self.out.getvalue().splitlines()]

    def test_discovery_configure_stream_reboot(self) -> None:
        ports = self.s.ports
        ack = self.request(sim.CMD_DISCOVERY, b'', ('127.0.0.1', ports['discovery']))
        disc = proto.parse_discovery_ack(ack.data)
        self.assertEqual(disc['sn'], 'SIM0000000000001')
        self.assertEqual(disc['cmd_port'], ports['cmd'])
        cmd = (disc['lidar_ip'], disc['cmd_port'])

        pcl_port = self.pcl.getsockname()[1]
        kvs = [
            (sim.KEY_PCL_HOST, proto.encode_host_ipcfg('127.0.0.1', pcl_port, proto.PORT_PCL)),
            (
                sim.KEY_STATE_HOST,
                proto.encode_host_ipcfg('127.0.0.1', self.host.getsockname()[1], proto.PORT_PUSH),
            ),
        ]
        ack = self.request(sim.CMD_PARAM_CONFIG, proto.encode_param_config(kvs), cmd)
        self.assertEqual(proto.parse_param_config_ack(ack.data), (0, 0))

        d, _ = self.pcl.recvfrom(2048)
        first = proto.DataPacket.parse(d)
        self.assertEqual(first.data_type, 1)
        self.assertEqual(first.dot_num, 96)
        d, _ = self.pcl.recvfrom(2048)
        second = proto.DataPacket.parse(d)
        self.assertEqual(second.udp_cnt, (first.udp_cnt + 1) & 0xFFFF)
        self.assertGreaterEqual(second.timestamp_ns, first.timestamp_ns)

        # Push arrives on the host socket within ~1 s.
        deadline = time.time() + 3
        got_push = False
        while time.time() < deadline and not got_push:
            d, _ = self.host.recvfrom(2048)
            f = proto.CommandFrame.parse(d)
            got_push = f.cmd_id == sim.CMD_INFO_PUSH and f.sender_type == 1
        self.assertTrue(got_push)

        # Reboot: ACK, then silence, then back to SAMPLING with udp_cnt reset.
        ack = self.request(sim.CMD_REBOOT, struct.pack('<H', 100), cmd)
        self.assertEqual(ack.data, b'\x00')
        self.pcl.settimeout(0.05)
        time.sleep(0.1)
        while True:  # drain anything already queued
            try:
                self.pcl.recvfrom(2048)
            except TimeoutError:
                break
        with self.assertRaises(socket.timeout):
            self.pcl.recvfrom(2048)  # silent
        self.pcl.settimeout(3)
        d, _ = self.pcl.recvfrom(2048)
        self.assertLess(proto.DataPacket.parse(d).udp_cnt, 50)
        states = [(e['from'], e['to']) for e in self.events() if e['event'] == 'state']
        boot = [
            (sim.WS_SELFCHECK, sim.WS_IDLE),
            (sim.WS_IDLE, sim.WS_MOTORSTARTUP),
            (sim.WS_MOTORSTARTUP, sim.WS_READY),
            (sim.WS_READY, sim.WS_SAMPLING),
        ]
        self.assertEqual(states[: len(boot)], boot)  # power-on
        self.assertEqual(states[-len(boot) :], boot)  # after the reboot
        self.assertEqual(states.count((sim.WS_READY, sim.WS_SAMPLING)), 2)

    def test_control_channel_hms_and_drop_ack(self) -> None:
        cmd = ('127.0.0.1', self.s.ports['cmd'])
        self.send_control('{"cmd":"hms","codes":[34603011]}')  # 0x02100003
        time.sleep(0.1)
        ack = self.request(sim.CMD_PARAM_INQUIRE, proto.encode_param_inquire([sim.KEY_HMS]), cmd)
        ret, kvs = proto.parse_param_inquire_ack(ack.data)
        self.assertEqual(struct.unpack('<8I', dict(kvs)[sim.KEY_HMS])[0], 0x02100003)

        self.send_control('{"cmd":"drop_ack","count":1}')
        time.sleep(0.1)
        self.seq += 1
        self.host.sendto(
            proto.CommandFrame(
                self.seq, sim.CMD_PARAM_INQUIRE, 0, 0, proto.encode_param_inquire([sim.KEY_SN])
            ).encode(),
            cmd,
        )
        self.host.settimeout(0.3)
        with self.assertRaises(socket.timeout):
            self.host.recvfrom(2048)
        self.host.settimeout(3)
        ack = self.request(sim.CMD_PARAM_INQUIRE, proto.encode_param_inquire([sim.KEY_SN]), cmd)
        self.assertEqual(proto.parse_param_inquire_ack(ack.data)[0], 0)
        self.assertTrue(any(e['event'] == 'ack_dropped' for e in self.events()))


if __name__ == '__main__':
    unittest.main()

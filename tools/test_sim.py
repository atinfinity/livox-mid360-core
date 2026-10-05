#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for the simulator's device model and an in-process end-to-end check.

python3 -m unittest tools/test_sim.py
"""

from __future__ import annotations

import argparse
import io
import json
import math
import os
import pathlib
import socket
import struct
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import livox_mid360_pcap as pcapfile  # noqa: E402
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

    def test_settings_keys_range_checks(self) -> None:
        for key in (sim.KEY_DETECT_MODE, sim.KEY_TIME_FILTER, sim.KEY_IMU_EN):
            self.assertEqual(self.m.configure([(key, b'\x01')]), (sim.RET_OK, 0))
            self.assertEqual(self.m.configure([(key, b'\x02')]), (sim.RET_OUT_OF_RANGE, key))
            self.assertEqual(self.m.settings[key], b'\x01')
        key = sim.KEY_IMU_SENSOR_CFG
        self.assertEqual(self.m.configure([(key, b'\x01\x03\x07')]), (sim.RET_OK, 0))
        self.assertEqual(self.m.imu_rate, 500.0)
        for bad in (b'\x04\x00\x00', b'\x00\x04\x00', b'\x00\x00\x08'):
            self.assertEqual(self.m.configure([(key, bad)]), (sim.RET_OUT_OF_RANGE, key))
        self.assertEqual(self.m.settings[key], b'\x01\x03\x07')
        for rate, hz in ((b'\x00', 200.0), (b'\x02', 100.0), (b'\x03', 50.0)):
            self.m.configure([(key, rate + b'\x00\x00')])
            self.assertEqual(self.m.imu_rate, hz)

    def test_imu_cfg_unsupported_firmware(self) -> None:
        m = sim.DeviceModel(imu_cfg_unsupported=True)
        key = sim.KEY_IMU_SENSOR_CFG
        self.assertEqual(m.configure([(key, b'\x01\x00\x00')]), (sim.RET_PARAM_NOT_SUPPORT, key))
        self.assertEqual(m.inquire([key], 0), (sim.RET_PARAM_NOT_SUPPORT, []))
        # The supported keys of the same request are still answered (#228).
        ret, kvs = m.inquire([sim.KEY_IMU_EN, key], 0)
        self.assertEqual((ret, [k for k, _ in kvs]), (sim.RET_PARAM_NOT_SUPPORT, [sim.KEY_IMU_EN]))
        self.assertEqual(m.imu_rate, 200.0)
        # The other keys are untouched.
        self.assertEqual(m.inquire([sim.KEY_IMU_EN], 0), (sim.RET_OK, [(sim.KEY_IMU_EN, b'\x00')]))

    def test_mid360_unsupported_keys(self) -> None:
        m = sim.DeviceModel(unsupported_keys=sim.MID360_UNSUPPORTED_KEYS)
        for key in sorted(sim.MID360_UNSUPPORTED_KEYS):
            self.assertEqual(m.inquire([key], 0), (sim.RET_PARAM_NOT_SUPPORT, []))
            value = b'\0' * sim.WRITABLE_LEN[key]
            self.assertEqual(m.configure([(key, value)]), (sim.RET_PARAM_NOT_SUPPORT, key))
        self.assertEqual(m.imu_rate, 200.0)
        self.assertEqual(sim.parse_key_list('mid360'), [0x0021, 0x0026, 0x0029, 0x002B])
        self.assertEqual(sim.parse_key_list('0x0026,43'), [0x0026, 0x002B])
        with self.assertRaises(argparse.ArgumentTypeError):
            sim.parse_key_list('0x8000')

    def test_factory_fov_and_fw_type(self) -> None:
        # As a Mid-360 on 13.18.0244 answers (#242).
        fov = proto.encode_fov_cfg(0, 0, -7, 52)
        self.assertEqual(self.m.settings[sim.KEY_FOV0], fov)
        self.assertEqual(self.m.settings[sim.KEY_FOV1], fov)
        self.assertEqual(self.m.settings[sim.KEY_FOV_EN], b'\x00')
        self.assertEqual(self.m.fov_windows(), [])
        self.assertEqual(self.m.read_key(sim.KEY_FW_TYPE, 0), b'\x01')

    def test_func_io_out_of_range(self) -> None:
        self.assertEqual(
            self.m.configure([(sim.KEY_FUNC_IO, b'\x00\x00\x02\x01')]), (sim.RET_OK, 0)
        )
        for bad in (
            b'\x01\x00\x00\x00',
            b'\x00\x01\x00\x00',
            b'\x00\x00\x03\x00',
            b'\x00\x00\x00\x03',
        ):
            self.assertEqual(
                self.m.configure([(sim.KEY_FUNC_IO, bad)]), (sim.RET_OUT_OF_RANGE, sim.KEY_FUNC_IO)
            )
        self.assertEqual(self.m.settings[sim.KEY_FUNC_IO], b'\x00\x00\x02\x01')

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
        self.assertEqual(self.m.settings[sim.KEY_FOV1], sim.FACTORY_FOV)

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

    def test_factory_reset_restores_the_factory_lidar_address(self) -> None:
        self.assertEqual(self.m.settings[sim.KEY_LIDAR_IPCFG][:4], bytes([192, 168, 1, 100]))
        own = proto.encode_lidar_ipcfg('127.0.0.1', '255.0.0.0', '0.0.0.0')
        self.m.factory_lidar_ipcfg = own
        self.m.configure([(sim.KEY_LIDAR_IPCFG, bytes([127, 0, 0, 2]) + own[4:])])
        self.m.factory_reset(0.0)
        self.assertEqual(self.m.settings[sim.KEY_LIDAR_IPCFG], own)

    def test_set_status_with_a_bad_value_applies_nothing(self) -> None:
        with self.assertRaises(ValueError):
            self.m.set_status({'diag': 7, 'core_temp': 'hot', 'omit_keys': [sim.KEY_HMS]})
        self.assertEqual(self.m.diag_status, 0)
        self.assertEqual(self.m.push_omit, set())

    def test_gps_time_steps_the_clock_and_reports_the_sync(self) -> None:
        self.m.power_on(10.0)
        self.m.set_gps_time(ns=5_000_000_000, mono=10.4, wall_ns=1_000)
        self.assertEqual(self.m.clock.sync_type, sim.TIME_SYNC_GPS)
        self.assertEqual(self.m.clock.now_ns(10.4, 1_000), 5_000_000_000)
        keys = [sim.KEY_LAST_SYNC_TIME, sim.KEY_TIME_OFFSET, sim.KEY_TIME_SYNC_TYPE]
        ret, kvs = self.m.inquire(keys, 0)
        kvs = dict(kvs)
        self.assertEqual(struct.unpack('<Q', kvs[sim.KEY_LAST_SYNC_TIME])[0], 5_000_000_000)
        # local - source: 0.4 s since power-on against the GPS time.
        self.assertEqual(
            struct.unpack('<q', kvs[sim.KEY_TIME_OFFSET])[0], 400_000_000 - 5_000_000_000
        )
        self.assertEqual(kvs[sim.KEY_TIME_SYNC_TYPE], bytes([sim.TIME_SYNC_GPS]))

    def test_set_status_updates_pushed_keys(self) -> None:
        self.assertEqual(
            self.m.set_status({'diag': 0x0021, 'core_temp': 4321, 'bogus': 1}), ['bogus']
        )
        kvs = dict(proto.parse_info_push(self.m.push_payload(5)))
        self.assertEqual(struct.unpack('<H', kvs[sim.KEY_DIAG_STATUS])[0], 0x0021)
        self.assertEqual(struct.unpack('<i', kvs[sim.KEY_CORE_TEMP])[0], 4321)
        self.m.set_gps_time(ns=1_000, mono=0.0, wall_ns=400)
        kvs = dict(proto.parse_info_push(self.m.push_payload(5)))
        self.assertEqual(struct.unpack('<Q', kvs[sim.KEY_LAST_SYNC_TIME])[0], 1_000)
        self.assertEqual(self.m.set_status({'omit_keys': [sim.KEY_CORE_TEMP]}), [])
        kvs = dict(proto.parse_info_push(self.m.push_payload(5)))
        self.assertNotIn(sim.KEY_CORE_TEMP, kvs)
        self.assertIn(sim.KEY_DIAG_STATUS, kvs)

    def test_log_host_key_is_writable_and_zero_by_default(self) -> None:
        self.assertIsNone(self.m.host(sim.KEY_LOG_HOST))
        cfg = proto.encode_host_ipcfg('192.168.1.5', 56501, 56500)
        self.assertEqual(self.m.configure([(sim.KEY_LOG_HOST, cfg)]), (sim.RET_OK, 0))
        self.assertEqual(self.m.host(sim.KEY_LOG_HOST), ('192.168.1.5', 56501, 56500))
        self.assertEqual(
            self.m.configure([(sim.KEY_LOG_HOST, b'\0' * 7)])[0], sim.RET_PARAM_INVALID_LEN
        )

    def test_push_payload_parses(self) -> None:
        self.m.hms = [0x02100003] + [0] * 7
        kvs = dict(proto.parse_info_push(self.m.push_payload(123)))
        self.assertEqual(kvs[sim.KEY_CUR_WORK_STATE], bytes([sim.WS_SELFCHECK]))
        self.assertEqual(struct.unpack('<8I', kvs[sim.KEY_HMS])[0], 0x02100003)
        self.assertEqual(kvs[sim.KEY_LOCAL_TIME], struct.pack('<Q', 123))
        # The key set of a Mid-360 push (#235).
        writable = [0x0000, 0x0001, 0x0004, 0x0005, 0x0006, 0x0007, 0x0012]
        writable += [0x0015, 0x0016, 0x0017, 0x0018, 0x0019, 0x001A, 0x001C]
        status = list(range(0x8000, 0x800D)) + [0x800E, 0x8010, 0x8011]
        self.assertEqual(list(kvs), writable + status)
        self.assertEqual(kvs[sim.KEY_WORK_TGT_MODE], bytes([sim.WS_SAMPLING]))


class LidarClockTest(unittest.TestCase):
    """The LiDAR clock (#133), driven with explicit host times."""

    WALL = 1_700_000_000_000_000_000  # host wall clock at mono 100.0

    def setUp(self) -> None:
        self.c = sim.LidarClock()
        self.c.power_on(100.0)

    def wall(self, mono: float) -> int:
        return self.WALL + round((mono - 100.0) * 1e9)

    def now(self, mono: float) -> int:
        return self.c.now_ns(mono, self.wall(mono))

    def test_free_running_counts_from_power_on(self) -> None:
        self.assertEqual(self.now(100.0), 0)
        self.assertEqual(self.now(102.5), 2_500_000_000)
        self.assertEqual(self.now(99.0), 0)  # before power-on (a reboot's silence)

    def test_drift_changes_the_rate_without_a_step(self) -> None:
        self.c.set_drift(100.0, 101.0)  # +100 ppm from 1 s after power-on
        self.assertEqual(self.now(101.0), 1_000_000_000)
        self.assertEqual(self.now(111.0), 11_001_000_000)
        with self.assertRaises(ValueError):
            self.c.set_drift(-1e6, 111.0)
        self.assertEqual(self.c.drift_ppm, 100.0)

    def test_sync_steps_to_the_master_and_records_the_step(self) -> None:
        self.c.sync(sim.TIME_SYNC_PTP, 3_000, 102.0, self.wall(102.0))
        master = self.wall(102.0) + 3_000
        self.assertEqual(self.now(102.0), master)
        self.assertEqual(self.now(103.0), master + 1_000_000_000)
        self.assertEqual(self.c.last_sync_ns, master)
        self.assertEqual(self.c.offset_ns, 2_000_000_000 - master)  # local - source

    def test_synchronised_clock_ignores_the_drift(self) -> None:
        self.c.set_drift(500.0, 100.0)
        self.c.sync(sim.TIME_SYNC_GPS, 0, 101.0, self.wall(101.0))
        self.assertEqual(self.now(111.0), self.wall(111.0))

    def test_losing_sync_free_runs_from_the_master_time(self) -> None:
        self.c.set_drift(1_000.0, 100.0)
        self.c.sync(sim.TIME_SYNC_PTP, 0, 101.0, self.wall(101.0))
        self.c.lose_sync(102.0, self.wall(102.0))
        self.assertEqual(self.c.sync_type, sim.TIME_SYNC_NONE)
        self.assertEqual(self.now(102.0), self.wall(102.0))  # no step
        self.assertEqual(self.now(112.0), self.wall(102.0) + 10_010_000_000)  # +1000 ppm
        self.assertEqual(self.c.last_sync_ns, self.wall(101.0))  # the last sync stays
        self.c.lose_sync(113.0, self.wall(113.0))  # already free running: nothing to do
        self.assertEqual(self.now(113.0), self.wall(102.0) + 11_011_000_000)

    def test_power_on_resets_the_sync_but_keeps_the_drift(self) -> None:
        self.c.set_drift(20.0, 100.0)
        self.c.sync(sim.TIME_SYNC_GPS, 0, 101.0, self.wall(101.0))
        self.c.power_on(200.0)
        self.assertEqual(self.now(200.0), 0)
        self.assertEqual((self.c.sync_type, self.c.last_sync_ns, self.c.offset_ns), (0, 0, 0))
        self.assertEqual(self.c.drift_ppm, 20.0)


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

    def test_tags_vary_per_field(self) -> None:
        seen = {'adjacent_glue': set(), 'particles': set(), 'other': set()}
        for s in sim.PointSource(1).samples(1, 960):
            t = proto.decode_tag(s[4])
            self.assertEqual(t['reserved'], 0)
            for k in seen:
                seen[k].add(t[k])
        for k, v in seen.items():
            self.assertEqual(v, {0, 1, 2}, k)


class RingSceneTest(unittest.TestCase):
    """The deterministic ring scene (#132)."""

    def test_ring_points_are_known(self) -> None:
        self.assertEqual(sim.ring_point(0), (1000, 9000, 0, 0, 0))
        self.assertEqual(sim.ring_point(64), (2920, 9000, 9000, 64, 1 | 0 << 2 | 1 << 4))
        self.assertEqual(sim.ring_point(1)[1], 9000 - 1500)  # pitch 15 deg
        self.assertEqual(sim.ring_point(sim.RING_POINTS + 5), sim.ring_point(5))
        pts = [sim.ring_point(k) for k in range(sim.RING_POINTS)]
        self.assertEqual([p[3] for p in pts], list(range(256)))
        seen = {'adjacent_glue': set(), 'particles': set(), 'other': set()}
        for p in pts:
            t = proto.decode_tag(p[4])
            self.assertEqual(t['reserved'], 0)
            for k in seen:
                seen[k].add(t[k])
        for k, v in seen.items():
            self.assertEqual(v, {0, 1, 2}, k)

    def test_data_types_encode_the_same_point(self) -> None:
        for k in range(sim.RING_POINTS):
            depth, theta, phi, refl, tag = sim.ring_sample(3, k)
            t, p = math.radians(theta / 100), math.radians(phi / 100)
            xyz = (
                depth * math.sin(t) * math.cos(p),
                depth * math.sin(t) * math.sin(p),
                depth * math.cos(t),
            )
            for dt, unit in ((1, 1), (2, 10)):
                sample = sim.ring_sample(dt, k)
                self.assertEqual(sample[3:], (refl, tag))
                for got, want in zip(sample[:3], xyz, strict=True):
                    self.assertLessEqual(abs(got * unit - want), unit / 2, (dt, k))
            self.assertEqual(len(proto.pack_samples(1, [sim.ring_sample(1, k)])), 14)
            self.assertEqual(len(proto.pack_samples(2, [sim.ring_sample(2, k)])), 8)


class AttitudeTest(unittest.TestCase):
    """The install attitude transform of --apply-attitude (#135)."""

    def assert_close(self, got, want) -> None:
        for g, w in zip(got, want, strict=True):
            self.assertAlmostEqual(g, w, places=9)

    def test_single_axis_rotations(self) -> None:
        self.assert_close(sim.transform_mm((0, 0, 90, 0, 0, 0), (1, 2, 3)), (-2, 1, 3))
        self.assert_close(sim.transform_mm((0, 90, 0, 0, 0, 0), (1, 2, 3)), (3, 2, -1))
        self.assert_close(sim.transform_mm((90, 0, 0, 0, 0, 0), (1, 2, 3)), (1, -3, 2))
        self.assert_close(sim.transform_mm((0, 0, 0, 10, -20, 30), (1, 2, 3)), (11, -18, 33))

    def test_rotation_order_is_z_y_x_then_translation(self) -> None:
        def rot(axis: int, deg: float) -> list[list[float]]:
            c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            i, j = [(1, 2), (2, 0), (0, 1)][axis]
            m = [[float(r == c_) for c_ in range(3)] for r in range(3)]
            m[i][i], m[i][j], m[j][i], m[j][j] = c, -s_, s_, c
            return m

        def mul(a, b):
            return [
                [sum(a[r][k] * b[k][c_] for k in range(3)) for c_ in range(3)] for r in range(3)
            ]

        att = (10.0, -20.0, 30.0, 100, -200, 300)
        r = mul(rot(2, att[2]), mul(rot(1, att[1]), rot(0, att[0])))
        p = (1234.0, -567.0, 89.0)
        want = [sum(r[i][k] * p[k] for k in range(3)) + att[3 + i] for i in range(3)]
        self.assert_close(sim.transform_mm(att, p), want)

    def test_samples_are_rounded_clamped_and_spherical_is_untouched(self) -> None:
        zero = (0.0, 0.0, 0.0, 0, 0, 0)
        for dt in (1, 2, 3):
            for smp in sim.PointSource(3).samples(dt, 50):
                self.assertEqual(sim.attitude_sample(dt, smp, zero), smp)
            for k in range(sim.RING_POINTS):
                self.assertEqual(sim.ring_sample(dt, k, zero), sim.ring_sample(dt, k))
        shift = (0.0, 0.0, 0.0, 100, -200, 300)
        self.assertEqual(sim.attitude_sample(1, (10, 20, 30, 5, 6), shift), (110, -180, 330, 5, 6))
        self.assertEqual(sim.attitude_sample(2, (10, 20, 30, 5, 6), shift), (20, 0, 60, 5, 6))
        self.assertEqual(
            sim.attitude_sample(3, (1000, 9000, 0, 5, 6), shift), (1000, 9000, 0, 5, 6)
        )
        far = (0.0, 0.0, 0.0, 10000, -10000, 0)
        self.assertEqual(
            sim.attitude_sample(2, (32000, -32000, 0, 1, 2), far), (32767, -32768, 0, 1, 2)
        )


class SimulatorTest(unittest.TestCase):
    """Simulator methods called directly, without the main loop."""

    def setUp(self) -> None:
        # Without the main loop a delayed reboot would never power down.
        args = sim.build_parser().parse_args(
            ['--bind', '127.0.0.1', '--base-port', '0', '--reboot-delay', '0']
        )
        self.out = io.StringIO()
        self.s = sim.Simulator(args, out=self.out, control=None)
        self.s.model.power_on(time.monotonic())
        self.s.next_push = self.s.next_stats = time.monotonic() + 1.0  # as run() would
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(('127.0.0.1', 0))
        self.rx.settimeout(3)
        self.rx_host = ('127.0.0.1', self.rx.getsockname()[1], 0)

    def tearDown(self) -> None:
        self.rx.close()
        for s in self.s.socks.values():
            s.close()

    def events(self) -> list[dict]:
        return [json.loads(line) for line in self.out.getvalue().splitlines()]

    def test_fov_cropped_packet_announces_the_points_it_carries(self) -> None:
        m = self.s.model
        for window, expect_empty in (
            (proto.encode_fov_cfg(0, 90, -5, 5), False),
            (proto.encode_fov_cfg(20, 20, 0, 0), True),
        ):
            m.configure([(sim.KEY_FOV0, window), (sim.KEY_FOV_EN, b'\x01')])
            self.s._send_pcl(self.rx_host, 1 / sim.PCL_PACKET_RATE)
            d, _ = self.rx.recvfrom(2048)
            pkt = proto.DataPacket.parse(d)  # raises 'bad dot_num' on a mismatch
            if expect_empty:
                self.assertEqual(pkt.dot_num, 0)
            else:
                self.assertGreater(pkt.dot_num, 0)
                self.assertTrue(all(m.keeps_point(1, p) for p in pkt.samples()))

    def pushed_states(self) -> list[int]:
        """Drain the pushes received so far; return their cur_work_state values."""
        self.rx.settimeout(0.2)
        got = []
        try:
            while True:
                f = proto.CommandFrame.parse(self.rx.recvfrom(4096)[0])
                got.append(dict(proto.parse_info_push(f.data))[sim.KEY_CUR_WORK_STATE][0])
        except TimeoutError:
            return got

    def test_state_change_and_config_request_push_at_once(self) -> None:
        # A Mid-360 pushes once per state change and after every 0x0100 request, whatever its
        # result, while its periodic push keeps its phase (#235).
        m = self.s.model
        host = proto.encode_host_ipcfg('127.0.0.1', self.rx.getsockname()[1], 0)
        self.assertEqual(m.configure([(sim.KEY_STATE_HOST, host)]), (sim.RET_OK, 0))
        now = time.monotonic()
        periodic = self.s.next_push
        m.force_state(sim.WS_IDLE, now)
        self.assertEqual(self.s._next_deadline(now), now)
        self.s._send_periodic(now)
        self.assertEqual(self.pushed_states(), [sim.WS_IDLE])
        self.assertEqual(self.s.next_push, periodic)
        self.s._send_periodic(now)  # owed once only
        self.assertEqual(self.pushed_states(), [])

        rejected = proto.encode_param_config([(sim.KEY_WORK_TGT_MODE, bytes([sim.WS_ERROR]))])
        req = proto.CommandFrame(1, sim.CMD_PARAM_CONFIG, sim.REQ, 0, rejected)
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(tx.close)
        tx.sendto(req.encode(), ('127.0.0.1', self.s.ports['cmd']))
        time.sleep(0.05)
        self.s._handle_datagram('cmd')
        ack = proto.CommandFrame.parse(tx.recvfrom(2048)[0])
        self.assertNotEqual(proto.parse_param_config_ack(ack.data)[0], sim.RET_OK)
        self.s._send_periodic(time.monotonic())
        self.assertEqual(self.pushed_states(), [sim.WS_IDLE])

        # Inquiries do not cause one.
        req = proto.CommandFrame(
            2, sim.CMD_PARAM_INQUIRE, sim.REQ, 0, proto.encode_param_inquire([0x8006])
        )
        tx.sendto(req.encode(), ('127.0.0.1', self.s.ports['cmd']))
        time.sleep(0.05)
        self.s._handle_datagram('cmd')
        tx.recvfrom(2048)
        self.s._send_periodic(time.monotonic())
        self.assertEqual(self.pushed_states(), [])

    def test_slow_catch_up_returns_to_the_main_loop(self) -> None:
        # A narrow FOV window makes each packet draw all MAX_FOV_DRAWS batches, slower than
        # the packet rate. One catch-up call must still end within a time slice, sending the
        # due push, so that commands are answered (#183).
        m = self.s.model
        m.configure(
            [(sim.KEY_FOV0, proto.encode_fov_cfg(30, 60, 0, 10)), (sim.KEY_FOV_EN, b'\x01')]
        )
        now = time.monotonic()
        m.force_state(sim.WS_SAMPLING, now)
        self.s.next_pcl = now - 0.4  # 800 packets overdue at the full rate
        self.s.next_push = now
        self.s._send_periodic(now)
        self.assertGreater(self.s.udp_cnt_pcl, 0)  # still makes progress
        self.assertLess(self.s.udp_cnt_pcl, 100)  # the count budget alone allows 256
        self.assertGreater(self.s.next_push, now)

    def test_silence_neither_spins_nor_bursts(self) -> None:
        now = time.monotonic()
        self.s.next_push = self.s.next_stats = now - 0.2  # overdue when the silence starts
        self.s.apply_control({'cmd': 'silence', 'seconds': 5})
        self.s._send_periodic(now)
        self.assertGreater(self.s._next_deadline(now), now)
        self.assertEqual(self.s.sent['push'], 0)

    def test_reboot_silence_defers_the_push(self) -> None:
        now = time.monotonic()
        self.s.next_push = now - 0.2
        self.s._do_reboot(now)
        self.s._send_periodic(now)
        self.assertGreater(self.s._next_deadline(now), now)
        self.assertGreaterEqual(self.s.next_push, now + self.s.args.reboot_silence)

    def test_discovery_ack_advertises_the_address_that_reaches_the_host(self) -> None:
        # Bound to 0.0.0.0 the ACK carries the local address on the requester's route, not
        # 127.0.0.1, so the simulator works off loopback too (#203).
        args = sim.build_parser().parse_args(['--bind', '0.0.0.0', '--base-port', '0'])
        wild = sim.Simulator(args, out=io.StringIO(), control=None)
        self.addCleanup(lambda: [s.close() for s in wild.socks.values()])
        req = proto.CommandFrame(1, sim.CMD_DISCOVERY, sim.REQ, 0, b'')
        now = time.monotonic()

        def advertised(peer: str) -> str:
            _, payload = wild._dispatch(req, (peer, 56000), now)
            return proto.parse_discovery_ack(payload)['lidar_ip']

        self.assertEqual(advertised('127.0.0.1'), '127.0.0.1')
        # An explicit --bind is advertised whatever the requester.
        _, payload = self.s._dispatch(req, ('127.0.0.1', 56000), now)
        self.assertEqual(proto.parse_discovery_ack(payload)['lidar_ip'], '127.0.0.1')
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.connect(('192.0.2.1', 9))  # TEST-NET-1: only the route is looked up
                local = probe.getsockname()[0]
        except OSError:
            self.skipTest('no route off loopback')
        if local.startswith('127.') or local == '0.0.0.0':
            self.skipTest('no address off loopback')
        self.assertEqual(advertised(local), local)
        # Key 0x0004 keeps reporting the address the simulator started with.
        self.assertEqual(wild.configured_ip(), '127.0.0.1')

    def test_discovery_ack_carries_the_dev_type_flag(self) -> None:
        req = proto.CommandFrame(1, sim.CMD_DISCOVERY, sim.REQ, 0, b'')
        _, payload = self.s._dispatch(req, ('127.0.0.1', 56000), time.monotonic())
        self.assertEqual(proto.parse_discovery_ack(payload)['dev_type'], proto.DEV_TYPE_MID360)
        args = sim.build_parser().parse_args(['--base-port', '0', '--dev-type', '200'])
        other = sim.Simulator(args, out=io.StringIO(), control=None)
        self.addCleanup(lambda: [s.close() for s in other.socks.values()])
        _, payload = other._dispatch(req, ('127.0.0.1', 56000), time.monotonic())
        self.assertEqual(proto.parse_discovery_ack(payload)['dev_type'], 200)
        with self.assertRaises(SystemExit) as cm:
            sim.main(['--base-port', '0', '--dev-type', '256'])
        self.assertIn('--dev-type', str(cm.exception))

    def test_factory_reset_keeps_the_answering_address(self) -> None:
        self.s._debug_control(True, ('127.0.0.1', self.rx.getsockname()[1]))
        self.s._do_reboot(time.monotonic(), factory=True)
        self.assertEqual(self.s.configured_ip(), self.s.lidar_ip())
        self.assertIsNone(self.s.pending_rebind)
        self.assertIsNone(self.s.debug_dest)
        self.assertEqual(self.s.model.pcl_data_type, 1)

    def send_stream(self, kind: str, n: int) -> list[int]:
        """Generate n packets of `kind` and return the udp_cnt of each datagram received."""
        send = self.s._send_pcl if kind == 'pcl' else self.s._send_imu
        for _ in range(n):
            send(self.rx_host, 0.001)
        self.rx.settimeout(0.2)
        got = []
        try:
            while True:
                d, _ = self.rx.recvfrom(2048)
                got.append(proto.DataPacket.parse(d).udp_cnt)
        except TimeoutError:
            pass
        return got

    def faults(self) -> list[tuple[str, str, int]]:
        return [
            (e['stream'], e['fault'], e['udp_cnt'])
            for e in self.events()
            if e['event'] == 'packet_fault'
        ]

    def test_drop_skips_packets_but_udp_cnt_advances(self) -> None:
        for kind in sim.DATA_STREAMS:
            self.s.apply_control({'cmd': 'drop', 'stream': kind, 'count': 2})
        self.assertEqual(self.send_stream('pcl', 5), [2, 3, 4])
        self.assertEqual(self.send_stream('imu', 5), [2, 3, 4])
        self.assertEqual(self.s.sent['pcl_dropped'], 2)
        self.assertEqual(self.s.sent['imu_dropped'], 2)
        self.assertEqual(self.s.sent['pcl'], 3)
        self.assertEqual(
            self.faults(),
            [('pcl', 'drop', 0), ('pcl', 'drop', 1), ('imu', 'drop', 0), ('imu', 'drop', 1)],
        )

    def test_reorder_sends_each_held_packet_after_depth_others(self) -> None:
        self.s.apply_control({'cmd': 'reorder', 'stream': 'imu', 'count': 2, 'depth': 2})
        self.assertEqual(self.send_stream('imu', 7), [1, 2, 0, 4, 5, 3, 6])
        self.assertEqual(self.s.sent['imu'], 7)
        self.assertEqual(self.s.sent['imu_reordered'], 2)
        self.assertEqual(self.faults(), [('imu', 'reorder', 0), ('imu', 'reorder', 3)])
        self.assertEqual(self.send_stream('pcl', 3), [0, 1, 2])  # the other stream untouched

    def test_duplicate_sends_the_same_datagram_twice(self) -> None:
        self.s.apply_control({'cmd': 'duplicate', 'count': 1})  # stream defaults to pcl
        self.assertEqual(self.send_stream('pcl', 3), [0, 0, 1, 2])
        self.assertEqual(self.s.sent['pcl'], 3)
        self.assertEqual(self.s.sent['pcl_duplicated'], 1)
        self.assertEqual(self.faults(), [('pcl', 'duplicate', 0)])

    def test_reboot_loses_a_held_packet(self) -> None:
        self.s.apply_control({'cmd': 'reorder', 'count': 1, 'depth': 5})
        self.send_stream('pcl', 2)
        self.s._do_reboot(time.monotonic())
        self.assertIsNone(self.s.faults['pcl'].held)

    def ring_packets(self, n: int) -> list[proto.DataPacket]:
        for _ in range(n):
            self.s._send_pcl(self.rx_host, 0.001)
        return [proto.DataPacket.parse(self.rx.recvfrom(2048)[0]) for _ in range(n)]

    def test_ring_scene_emits_points_in_index_order(self) -> None:
        self.s.apply_control({'cmd': 'scene', 'name': 'ring'})
        for dt in (1, 2, 3):
            self.s.model.settings[sim.KEY_PCL_DATA_TYPE] = bytes([dt])
            self.s.apply_control({'cmd': 'scene', 'name': 'ring'})  # restarts at index 0
            pkts = self.ring_packets(3)
            samples = [smp for pkt in pkts for smp in pkt.samples()]
            self.assertEqual(len(samples), 3 * sim.POINTS_PER_PACKET)
            self.assertEqual(samples, [sim.ring_sample(dt, k) for k in range(len(samples))])
        self.s.apply_control({'cmd': 'scene', 'name': 'random'})
        self.assertNotEqual([smp[3] for smp in self.ring_packets(1)[0].samples()], list(range(96)))
        self.s.apply_control({'cmd': 'scene', 'name': 'ring'})
        self.ring_packets(1)
        self.assertEqual(self.s.ring_next, sim.POINTS_PER_PACKET)
        self.s._do_reboot(time.monotonic())  # starts a silence: check the cursor only
        self.assertEqual(self.s.ring_next, 0)

    def test_ring_scene_is_cropped_at_the_window_edges(self) -> None:
        m = self.s.model
        # yaw [0, 90) and pitch [0, 15]: k = 0 kept, k = 64 (yaw 90) not, pitch 15 kept, -5 not.
        window = proto.encode_fov_cfg(0, 90, 0, 15)
        m.configure([(sim.KEY_FOV0, window), (sim.KEY_FOV_EN, b'\x01')])
        self.s.apply_control({'cmd': 'scene', 'name': 'ring'})
        kept = [smp[3] for pkt in self.ring_packets(8) for smp in pkt.samples()]
        ring = [k for k in range(64) if k % 4 in (0, 1)]  # pitch 0 or 15
        self.assertEqual(kept, ring * 3)  # 8 packets of 96 = 3 rings

    def test_apply_attitude_moves_cartesian_points_only(self) -> None:
        self.assertFalse(self.s.apply_attitude)  # off unless --apply-attitude
        att = (10.0, -20.0, 30.0, 100, -200, 300)
        m = self.s.model
        m.configure([(sim.KEY_INSTALL_ATTITUDE, struct.pack('<fffiii', *att))])
        self.assertEqual(m.install_attitude(), att)
        for apply in (False, True):
            self.s.apply_attitude = apply
            for dt in (1, 2, 3):
                m.settings[sim.KEY_PCL_DATA_TYPE] = bytes([dt])
                self.s.apply_control({'cmd': 'scene', 'name': 'ring'})
                samples = [smp for pkt in self.ring_packets(2) for smp in pkt.samples()]
                moved = apply and dt != 3
                want = [sim.ring_sample(dt, k, att if moved else None) for k in range(192)]
                self.assertEqual(samples, want, (apply, dt))
        # The FOV crops in the sensor frame: the same ring points survive the window.
        window = proto.encode_fov_cfg(0, 90, 0, 15)
        m.configure([(sim.KEY_FOV0, window), (sim.KEY_FOV_EN, b'\x01')])
        m.settings[sim.KEY_PCL_DATA_TYPE] = b'\x01'
        self.s.apply_control({'cmd': 'scene', 'name': 'ring'})
        kept = [smp for pkt in self.ring_packets(3) for smp in pkt.samples()]
        inside = [k % 256 for k in range(3 * 96) if k % 256 < 64 and k % 4 in (0, 1)]
        self.assertEqual(kept, [sim.ring_sample(1, k, att) for k in inside])

    def test_time_sync_sets_the_packet_time(self) -> None:
        (free,) = self.ring_packets(1)
        self.assertEqual(free.time_type, sim.TIME_SYNC_NONE)
        self.assertLess(free.timestamp_ns, 60_000_000_000)  # since power-on, not the epoch

        hour = 3600 * 10**9
        self.s.apply_control({'cmd': 'time_sync', 'type': 'ptp', 'offset_ns': hour})
        (ev,) = [e for e in self.events() if e['event'] == 'time_sync']
        self.assertEqual(ev['type'], 'ptp')
        self.assertLess(ev['offset_ns'], 0)  # local (since power-on) - source (epoch + 1 h)
        (ptp,) = self.ring_packets(1)
        self.assertEqual(ptp.time_type, sim.TIME_SYNC_PTP)
        self.assertAlmostEqual(ptp.timestamp_ns - time.time_ns(), hour, delta=10**9)

        self.s.apply_control({'cmd': 'time_sync', 'type': 'none'})
        (lost,) = self.ring_packets(1)
        self.assertEqual(lost.time_type, sim.TIME_SYNC_NONE)
        self.assertGreaterEqual(lost.timestamp_ns, ptp.timestamp_ns)  # no step back
        self.assertAlmostEqual(lost.timestamp_ns - time.time_ns(), hour, delta=10**9)

        self.s._do_reboot(time.monotonic())
        self.assertEqual(self.s.model.clock.sync_type, sim.TIME_SYNC_NONE)
        self.assertLess(self.s.now_ns(), 60_000_000_000)

    def test_malformed_control_lines_emit_error(self) -> None:
        for req in (
            [],
            {'cmd': 'fail_cmd'},
            {'cmd': 'fail_cmd', 'cmd_id': 'x'},
            {'cmd': 'set_state'},
            {'cmd': 'set_state', 'state': 3},
            {'cmd': 'hms', 'codes': 5},
            {'cmd': 'inquire_override', 'key': 1, 'value': 'zz'},
            {'cmd': 'silence', 'seconds': None},
            {'cmd': 'drop', 'stream': 'push'},
            {'cmd': 'duplicate', 'count': -1},
            {'cmd': 'reorder', 'depth': 0},
            {'cmd': 'scene', 'name': 'cube'},
            {'cmd': 'time_sync', 'type': 'ntp'},
            {'cmd': 'time_sync', 'type': 'ptp', 'drift_ppm': 2e6},
        ):
            self.s.apply_control(req)
        self.assertTrue(self.s.running)
        evs = self.events()
        self.assertEqual([e['event'] for e in evs], ['error'] * 14)
        self.assertEqual(self.s.model.clock.sync_type, sim.TIME_SYNC_NONE)
        self.assertEqual(self.s.faults, {k: sim.PacketFaults() for k in sim.DATA_STREAMS})
        self.assertEqual(self.s.fail_cmds, {})
        self.assertEqual(self.s.scene, 'random')


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
                '--reboot-delay',
                '0.2',
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

        # Reboot: ACK, still running for the delay, then silence, then back to SAMPLING with
        # udp_cnt reset.
        ack = self.request(sim.CMD_REBOOT, struct.pack('<H', 100), cmd)
        self.assertEqual(ack.data, b'\x00')
        self.pcl.settimeout(0.05)
        time.sleep(self.s.args.reboot_delay + 0.1)
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
        reboot = len(states) - len(boot)
        self.assertEqual(states[: len(boot)], boot)  # power-on
        down = [(sim.WS_SAMPLING, sim.WS_ERROR), (sim.WS_ERROR, sim.WS_SELFCHECK)]
        self.assertEqual(states[reboot - len(down) : reboot], down)
        self.assertEqual(states[reboot:], boot)  # after the reboot
        self.assertEqual(states.count((sim.WS_READY, sim.WS_SAMPLING)), 2)

    def test_reboot_keeps_running_then_pushes_error_and_comes_back_in_motorstartup(self) -> None:
        # The sequence of a Mid-360 (#236), with the durations scaled down.
        cmd = ('127.0.0.1', self.s.ports['cmd'])
        kvs = [
            (sim.KEY_PCL_HOST, proto.encode_host_ipcfg('127.0.0.1', self.pcl.getsockname()[1], 0)),
            (
                sim.KEY_STATE_HOST,
                proto.encode_host_ipcfg('127.0.0.1', self.host.getsockname()[1], 0),
            ),
        ]
        ack = self.request(sim.CMD_PARAM_CONFIG, proto.encode_param_config(kvs), cmd)
        self.assertEqual(proto.parse_param_config_ack(ack.data), (0, 0))
        self.pcl.recvfrom(2048)  # sampling
        ack = self.request(sim.CMD_REBOOT, struct.pack('<H', 100), cmd)
        t_ack = time.monotonic()
        self.assertEqual(ack.data, b'\x00')
        pushes = []  # (seconds after the ACK, cur_work_state)
        while not pushes or pushes[-1][1] != sim.WS_SAMPLING or len(pushes) < 3:
            f = proto.CommandFrame.parse(self.host.recvfrom(4096)[0])
            if f.cmd_id == sim.CMD_INFO_PUSH:
                state = dict(proto.parse_info_push(f.data))[sim.KEY_CUR_WORK_STATE][0]
                pushes.append((time.monotonic() - t_ack, state))
        states = [s for _, s in pushes]
        error = states.index(sim.WS_ERROR)
        self.assertNotIn(sim.WS_ERROR, states[error + 1 :])
        self.assertTrue(all(s == sim.WS_SAMPLING for s in states[:error]))
        self.assertGreaterEqual(pushes[error][0], self.s.args.reboot_delay - 0.05)
        # Then silence, and SELFCHECK / IDLE are never pushed.
        self.assertEqual(states[error + 1], sim.WS_MOTORSTARTUP)
        self.assertNotIn(sim.WS_SELFCHECK, states)
        self.assertNotIn(sim.WS_IDLE, states)
        gap = pushes[error + 1][0] - pushes[error][0]
        self.assertGreaterEqual(gap, self.s.args.reboot_silence - 0.05)
        # The point cloud kept flowing during the delay: no udp_cnt reset before the ERROR.
        cnts = [proto.DataPacket.parse(self.pcl.recvfrom(2048)[0]).udp_cnt]
        while cnts[-1] >= cnts[0]:  # until the first packet after the reboot
            cnts.append(proto.DataPacket.parse(self.pcl.recvfrom(2048)[0]).udp_cnt)
        self.assertGreater(len(cnts), 10)

    def test_ip_config_change_rebinds_after_reboot(self) -> None:
        cmd = ('127.0.0.1', self.s.ports['cmd'])
        ack = self.request(
            sim.CMD_PARAM_INQUIRE, proto.encode_param_inquire([sim.KEY_LIDAR_IPCFG]), cmd
        )
        _, kvs = proto.parse_param_inquire_ack(ack.data)
        self.assertEqual(dict(kvs)[sim.KEY_LIDAR_IPCFG][:4], bytes([127, 0, 0, 1]))
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind(('127.0.0.2', 0))
        except OSError:
            self.skipTest('127.0.0.2 is not configured on the loopback interface')
        finally:
            probe.close()
        new = bytes([127, 0, 0, 2, 255, 0, 0, 0, 0, 0, 0, 0])
        ack = self.request(
            sim.CMD_PARAM_CONFIG, proto.encode_param_config([(sim.KEY_LIDAR_IPCFG, new)]), cmd
        )
        self.assertEqual(ack.data[0], sim.RET_PARAM_REBOOT_EFFECT)
        self.send_control('{"cmd":"reboot"}')
        deadline = time.monotonic() + 3
        while '"event":"rebound"' not in self.out.getvalue() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertIn('"event":"rebound","ip":"127.0.0.2"', self.out.getvalue())
        time.sleep(self.s.args.reboot_silence + 0.2)
        ack = self.request(sim.CMD_DISCOVERY, b'', ('127.0.0.2', self.s.ports['discovery']))
        self.assertEqual(ack.data[18:22], bytes([127, 0, 0, 2]))

    def test_control_channel_hms_and_drop_ack(self) -> None:
        cmd = ('127.0.0.1', self.s.ports['cmd'])
        self.send_control('{"cmd":"hms","codes":[34603011]}')  # 0x02100003
        time.sleep(0.1)
        ack = self.request(sim.CMD_PARAM_INQUIRE, proto.encode_param_inquire([sim.KEY_HMS]), cmd)
        ret, kvs = proto.parse_param_inquire_ack(ack.data)
        self.assertEqual(struct.unpack('<8I', dict(kvs)[sim.KEY_HMS])[0], 0x02100003)

        self.send_control('{"cmd":"set_status","diag":33,"core_temp":4321}')
        time.sleep(0.1)
        ack = self.request(
            sim.CMD_PARAM_INQUIRE,
            proto.encode_param_inquire([sim.KEY_DIAG_STATUS, sim.KEY_CORE_TEMP]),
            cmd,
        )
        ret, kvs = proto.parse_param_inquire_ack(ack.data)
        self.assertEqual(struct.unpack('<H', dict(kvs)[sim.KEY_DIAG_STATUS])[0], 33)
        self.assertEqual(struct.unpack('<i', dict(kvs)[sim.KEY_CORE_TEMP])[0], 4321)

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

    def test_control_channel_fail_cmd(self) -> None:
        cmd = ('127.0.0.1', self.s.ports['cmd'])
        mode = proto.encode_param_config([(sim.KEY_WORK_TGT_MODE, bytes([1]))])
        imu = proto.encode_param_config([(sim.KEY_IMU_EN, bytes([0]))])
        self.send_control('{"cmd":"fail_cmd","cmd_id":256,"ret":2,"skip":1,"key":26}')
        time.sleep(0.1)
        # Another key does not match, the first match is skipped, the second one fails.
        self.assertEqual(self.request(sim.CMD_PARAM_CONFIG, imu, cmd).data[0], 0)
        self.assertEqual(self.request(sim.CMD_PARAM_CONFIG, mode, cmd).data[0], 0)
        ack = self.request(sim.CMD_PARAM_CONFIG, mode, cmd)
        self.assertEqual(struct.unpack('<BH', ack.data[:3]), (2, sim.KEY_WORK_TGT_MODE))
        self.assertEqual(self.request(sim.CMD_PARAM_CONFIG, mode, cmd).data[0], 0)

        self.send_control('{"cmd":"fail_cmd","cmd_id":257,"count":2}')
        time.sleep(0.1)
        inquire = proto.encode_param_inquire([sim.KEY_SN])
        for expected in (1, 1, 0):
            ack = self.request(sim.CMD_PARAM_INQUIRE, inquire, cmd)
            self.assertEqual(ack.data[0], expected)

    def test_control_channel_inquire_override(self) -> None:
        cmd = ('127.0.0.1', self.s.ports['cmd'])
        inquire = proto.encode_param_inquire([sim.KEY_SN, sim.KEY_CORE_TEMP])

        def ask() -> tuple[int, dict]:
            ack = self.request(sim.CMD_PARAM_INQUIRE, inquire, cmd)
            ret, kvs = proto.parse_param_inquire_ack(ack.data)
            return ret, dict(kvs)

        key = sim.KEY_CORE_TEMP
        self.send_control(f'{{"cmd":"inquire_override","key":{key},"value":"0102"}}')
        time.sleep(0.1)
        self.assertEqual(ask()[1][key], b'\x01\x02')
        self.send_control(f'{{"cmd":"inquire_override","key":{key},"omit":true}}')
        time.sleep(0.1)
        ret, kvs = ask()
        self.assertEqual((ret, list(kvs)), (0, [sim.KEY_SN]))
        self.send_control(f'{{"cmd":"inquire_override","key":{key},"unsupported":true}}')
        time.sleep(0.1)
        ret, kvs = ask()
        self.assertEqual((ret, list(kvs)), (0x20, [sim.KEY_SN]))  # left out, as on a Mid-360
        self.send_control(f'{{"cmd":"inquire_override","key":{key},"clear":true}}')
        time.sleep(0.1)
        self.assertEqual(len(ask()[1][key]), 4)

    def test_firmware_log_stream_gap_and_ack(self) -> None:
        self.s.args.log_chunk_interval = 0.01
        self.s.args.log_chunk_bytes = 64
        log = ('127.0.0.1', self.s.ports['log'])
        # Host log socket; key 0x0009 points the stream at it.
        host_log = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        host_log.bind(('127.0.0.1', 0))
        host_log.settimeout(3)
        try:
            cfg = proto.encode_host_ipcfg('127.0.0.1', host_log.getsockname()[1], 56500)
            ack = self.request(
                sim.CMD_PARAM_CONFIG,
                proto.encode_param_config([(sim.KEY_LOG_HOST, cfg)]),
                ('127.0.0.1', self.s.ports['cmd']),
            )
            self.assertEqual(ack.data[0], sim.RET_OK)
            self.assertEqual(self.request(sim.CMD_COLLECTION_LOG, b'\x00\x01', log).data, b'\x00')
            self.assertEqual(self.request(sim.CMD_COLLECTION_LOG, b'\x00\x01', log).data, b'\x00')
            self.assertEqual(self.request(sim.CMD_COLLECTION_LOG, b'\x05\x01', log).data, b'\x01')

            def chunk():
                d, addr = host_log.recvfrom(2048)
                f = proto.CommandFrame.parse(d)
                self.assertEqual((f.cmd_id, f.cmd_type), (sim.CMD_PUSH_LOG, 0))
                hdr = struct.unpack_from('<BBBBIHIH', f.data, 0)
                self.assertEqual(hdr[7], len(f.data) - 16)
                return hdr, f.data[16:], addr

            hdr, data, addr = chunk()
            self.assertEqual((hdr[0], hdr[1], hdr[6]), (0, 1, 1))
            self.assertTrue(hdr[3] & sim.LOG_FLAG_BEGIN)
            self.assertTrue(hdr[3] & sim.LOG_FLAG_ACK)
            self.assertEqual(len(data), 64)
            self.assertTrue(data.startswith(self.s.model.sn.encode()))
            # Acknowledge it the way the SDK does.
            ack_payload = struct.pack('<BBBI', 0, hdr[0], hdr[1], hdr[6])
            host_log.sendto(
                proto.CommandFrame(9, sim.CMD_PUSH_LOG, 0, 0, ack_payload).encode(), addr
            )
            hdr, _, _ = chunk()
            self.assertEqual(hdr[6], 2)
            self.send_control('{"cmd":"log_drop","n":2}')
            seen = [chunk()[0][6] for _ in range(6)]
            self.assertIn(5, seen)
            self.assertNotIn(3, seen)
            self.assertNotIn(4, seen)
            self.send_control('{"cmd":"log_new_file"}')
            ended = begun = False
            for _ in range(20):
                hdr, data, _ = chunk()
                if hdr[3] & sim.LOG_FLAG_END:
                    ended = True
                    self.assertEqual(len(data), 0)
                if hdr[1] == 2 and hdr[3] & sim.LOG_FLAG_BEGIN:
                    self.assertEqual(hdr[6], 1)
                    begun = True
                    break
            self.assertTrue(ended and begun)
            self.assertEqual(self.request(sim.CMD_COLLECTION_LOG, b'\x00\x00', log).data, b'\x00')
            self.send_control('{"cmd":"status"}')
            deadline = time.monotonic() + 3
            while '"event":"status"' not in self.out.getvalue() and time.monotonic() < deadline:
                time.sleep(0.02)
            status = [e for e in self.events() if e['event'] == 'status'][-1]
            self.assertEqual(status['log_enabled'], [])
            self.assertGreaterEqual(status['log_acks_received'], 1)
            self.assertEqual(status['hosts']['log'][1], host_log.getsockname()[1])
        finally:
            host_log.close()

    def test_debug_data_stream(self) -> None:
        self.s.args.debug_data_interval = 0.005
        self.s.args.debug_data_bytes = 32
        log = ('127.0.0.1', self.s.ports['log'])
        cmd = ('127.0.0.1', self.s.ports['cmd'])
        host = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        other = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            for s in (host, other):
                s.bind(('127.0.0.1', 0))
                s.settimeout(3)

            def control(enable: bool, sock: socket.socket, reserved: int = 0) -> bytes:
                return proto.encode_debug_data_control(
                    enable, '127.0.0.1', sock.getsockname()[1], reserved
                )

            # A short payload is rejected; an enable with port 0 is accepted without a stream,
            # as on a Mid-360 (#106). Nothing is opened for them.
            self.assertEqual(self.request(sim.CMD_DEBUG_DATA, b'\x01\x7f\x00', log).data, b'\x01')
            zero = proto.encode_debug_data_control(True, '127.0.0.1', 0)
            self.assertEqual(self.request(sim.CMD_DEBUG_DATA, zero, log).data, b'\x00')
            self.assertIsNone(self.s.debug_sock)
            self.assertIsNone(self.s.debug_dest)
            # The command port does not answer 0x0303 (#106).
            self.seq += 1
            enable = proto.CommandFrame(self.seq, sim.CMD_DEBUG_DATA, 0, 0, control(True, host))
            self.host.sendto(enable.encode(), cmd)
            self.host.settimeout(0.2)
            try:
                with self.assertRaises(TimeoutError):
                    self.host.recvfrom(2048)
            finally:
                self.host.settimeout(3)
            self.assertIsNone(self.s.debug_dest)
            # A disable before any enable is accepted.
            self.assertEqual(
                self.request(sim.CMD_DEBUG_DATA, control(False, host), log).data, b'\x00'
            )

            self.assertEqual(
                self.request(sim.CMD_DEBUG_DATA, control(True, host), log).data, b'\x00'
            )
            seqs = []
            for _ in range(5):
                d, addr = host.recvfrom(2048)
                self.assertEqual(len(d), 32)
                (seq,) = struct.unpack_from('<I', d, 0)
                self.assertEqual(d[4:], bytes((seq + i) & 0xFF for i in range(28)))
                seqs.append(seq)
            self.assertEqual(seqs, [0, 1, 2, 3, 4])
            # --base-port 0: the source port is a free one, reported by the event and status.
            source_port = addr[1]
            ev = [e for e in self.events() if e['event'] == 'debug_data'][-1]
            self.assertEqual(ev['port'], source_port)
            self.assertEqual(ev['dest'], f'127.0.0.1:{host.getsockname()[1]}')

            # A repeated enable moves the stream.
            self.assertEqual(
                self.request(sim.CMD_DEBUG_DATA, control(True, other, 100), log).data, b'\x00'
            )
            d, addr = other.recvfrom(2048)
            self.assertEqual(addr[1], source_port)
            self.assertGreater(struct.unpack_from('<I', d, 0)[0], 4)

            self.send_control('{"cmd":"status"}')
            deadline = time.monotonic() + 3
            while '"event":"status"' not in self.out.getvalue() and time.monotonic() < deadline:
                time.sleep(0.02)
            status = [e for e in self.events() if e['event'] == 'status'][-1]
            self.assertEqual(
                status['debug_data'],
                {
                    'enabled': True,
                    'dest': ['127.0.0.1', other.getsockname()[1]],
                    'port': source_port,
                },
            )
            self.assertGreaterEqual(status['sent']['debug'], 6)

            self.assertEqual(
                self.request(sim.CMD_DEBUG_DATA, control(False, other), log).data, b'\x00'
            )
            self.assertIsNone(self.s.debug_dest)
            time.sleep(0.05)
            other.setblocking(False)
            while True:  # what was in flight
                try:
                    other.recvfrom(2048)
                except BlockingIOError:
                    break
            time.sleep(0.05)
            with self.assertRaises(BlockingIOError):
                other.recvfrom(2048)
        finally:
            host.close()
            other.close()

    def test_silence_is_a_link_drop_that_leaves_a_udp_cnt_gap(self) -> None:
        cmd = ('127.0.0.1', self.s.ports['cmd'])
        pcl = proto.encode_host_ipcfg('127.0.0.1', self.pcl.getsockname()[1], proto.PORT_PCL)
        ack = self.request(
            sim.CMD_PARAM_CONFIG, proto.encode_param_config([(sim.KEY_PCL_HOST, pcl)]), cmd
        )
        self.assertEqual(ack.data[0], 0)
        prev = proto.DataPacket.parse(self.pcl.recvfrom(2048)[0]).udp_cnt
        self.send_control('{"cmd":"silence","seconds":0.3}')
        deadline = time.monotonic() + 3
        jumps = []
        while not jumps and time.monotonic() < deadline:
            cnt = proto.DataPacket.parse(self.pcl.recvfrom(2048)[0]).udp_cnt
            if cnt != (prev + 1) & 0xFFFF:
                jumps.append((cnt - prev) & 0xFFFF)
            prev = cnt
        # 0.3 s of packets were counted but withheld: one gap, no burst of stale ones. Nominally
        # 600 at 2000 pkt/s, but a starved simulator falls behind the rate, so only require
        # 10 % of that, and that every packet in the gap was withheld (`silenced` also counts
        # the IMU and push datagrams).
        self.assertEqual(len(jumps), 1)
        self.assertGreater(jumps[0], 60)
        self.assertGreaterEqual(self.s.sent['silenced'], jumps[0] - 1)

    def test_control_lines_written_together_are_all_applied(self) -> None:
        # Two lines in one write land in the pipe together; the second must not be
        # left behind in a read buffer that select() never reports again.
        self.send_control('{"cmd":"hms","codes":[34603011]}\n{"cmd":"status"}')
        deadline = time.monotonic() + 3
        while '"event":"status"' not in self.out.getvalue() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertIn('"event":"status"', self.out.getvalue())
        self.assertEqual(self.s.model.hms[0], 34603011)


FIXTURE = pathlib.Path(__file__).resolve().parent.parent / 'tests' / 'data' / 'replay.pcap'
REPLAY_KEYS = {
    'push': sim.KEY_STATE_HOST,
    'pcl': sim.KEY_PCL_HOST,
    'imu': sim.KEY_IMU_HOST,
    'log': sim.KEY_LOG_HOST,
}
RECORDED_CORE_TEMP = 4321  # tools/gen_replay_pcap.py PUSH_CORE_TEMP


def recorded() -> dict[str, list[bytes]]:
    """Return the fixture's LiDAR data-port payloads per stream, read without the simulator."""
    by_port = {
        proto.PORT_PUSH: 'push',
        proto.PORT_PCL: 'pcl',
        proto.PORT_IMU: 'imu',
        proto.PORT_LOG: 'log',
    }
    out: dict[str, list[bytes]] = {stream: [] for stream in by_port.values()}
    for _, linktype, frame in pcapfile.iter_pcap(FIXTURE):
        u = pcapfile.parse_udp(linktype, frame)
        if u is not None and u.src == '192.168.1.12' and u.sport in by_port:
            out[by_port[u.sport]].append(u.payload)
    return out


def core_temp(push: bytes) -> int | None:
    kvs = dict(proto.parse_info_push(proto.CommandFrame.parse(push).data))
    return struct.unpack('<i', kvs[sim.KEY_CORE_TEMP])[0] if sim.KEY_CORE_TEMP in kvs else None


class PcapReplayTest(unittest.TestCase):
    """--pcap (#134): the committed fixture replayed into sockets standing in for the SDK."""

    def start(self, *extra: str) -> None:
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
                '--pcap',
                str(FIXTURE),
                *extra,
            ]
        )
        self.out = io.StringIO()
        self.s = sim.Simulator(args, out=self.out, control=None)
        self.thread = threading.Thread(target=self.s.run, daemon=True)
        self.thread.start()

    def setUp(self) -> None:
        self.s = None
        self.rx: dict[str, socket.socket] = {}
        for stream in REPLAY_KEYS:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            s.bind(('127.0.0.1', 0))
            self.rx[stream] = s
        self.host = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host.bind(('127.0.0.1', 0))
        self.host.settimeout(3)

    def tearDown(self) -> None:
        if self.s is not None:
            self.s.running = False
            self.thread.join(timeout=5)
            for s in self.s.socks.values():
                s.close()
        for s in [*self.rx.values(), self.host]:
            s.close()

    def events(self, name: str) -> list[dict]:
        return [e for e in map(json.loads, self.out.getvalue().splitlines()) if e['event'] == name]

    def configure(self, *streams: str) -> None:
        """Point the given streams' host keys at their sockets with one 0x0100."""
        kvs = [
            (REPLAY_KEYS[s], proto.encode_host_ipcfg('127.0.0.1', self.rx[s].getsockname()[1], 0))
            for s in streams
        ]
        req = proto.CommandFrame(1, sim.CMD_PARAM_CONFIG, 0, 0, proto.encode_param_config(kvs))
        self.host.sendto(req.encode(), ('127.0.0.1', self.s.ports['cmd']))
        ack = proto.CommandFrame.parse(self.host.recvfrom(2048)[0])
        self.assertEqual(proto.parse_param_config_ack(ack.data), (0, 0))

    def wait_done(self) -> dict:
        deadline = time.monotonic() + 5
        while not self.events('replay_done') and time.monotonic() < deadline:
            time.sleep(0.01)
        done = self.events('replay_done')
        self.assertEqual(len(done), 1)
        return done[0]

    def received(self, stream: str) -> list[bytes]:
        """Drain the stream's socket (everything replayed has been sent by now)."""
        s = self.rx[stream]
        s.settimeout(0.2)
        got = []
        try:
            while True:
                got.append(s.recvfrom(4096)[0])
        except TimeoutError:
            pass
        return got

    def test_fixture_holds_every_replayed_stream(self) -> None:
        streams = [stream for _, stream, _ in sim.replay_datagrams(FIXTURE)]
        self.assertEqual(
            {s: streams.count(s) for s in REPLAY_KEYS}, {'push': 2, 'pcl': 24, 'imu': 12, 'log': 1}
        )
        self.assertEqual(
            {s: len(v) for s, v in recorded().items()}, {'push': 2, 'pcl': 24, 'imu': 12, 'log': 1}
        )

    def test_replays_the_lidar_data_ports_byte_for_byte(self) -> None:
        self.start('--pcap-rate', '0', '--push-rate', '0.1')
        self.configure('push', 'pcl', 'imu', 'log')
        done = self.wait_done()
        self.assertEqual(done['sent'], {'push': 2, 'pcl': 24, 'imu': 12, 'log': 1})
        self.assertEqual(done['skipped'], 0)
        want = recorded()
        for stream in REPLAY_KEYS:
            with self.subTest(stream=stream):
                self.assertEqual(self.received(stream), want[stream])
        # Replaced, not merged: the simulator generated no point cloud or IMU of its own.
        self.assertEqual(self.s.sent['pcl'] + self.s.sent['imu'], 0)
        self.assertEqual(len(self.events('replay_start')), 1)

    def test_waits_for_a_host_and_skips_streams_without_one(self) -> None:
        self.start('--pcap-rate', '0', '--push-rate', '0.1')
        time.sleep(0.3)  # sampling by now, but no host configured
        self.assertEqual(self.s.replay.state, 'waiting')
        self.assertEqual(self.events('replay_start'), [])
        self.configure('pcl', 'imu')
        done = self.wait_done()
        self.assertEqual(done['sent'], {'push': 0, 'pcl': 24, 'imu': 12, 'log': 0})
        self.assertEqual(done['skipped'], 3)
        self.assertEqual(self.received('pcl'), recorded()['pcl'])

    def test_recorded_spacing_scaled_by_the_rate(self) -> None:
        self.start('--pcap-rate', '2', '--push-rate', '0.1')
        self.configure('pcl')
        done = self.wait_done()
        # First replayed datagram at 5 ms, last at 125 ms: 120 ms of capture at twice the speed.
        self.assertGreaterEqual(done['seconds'], 0.059)
        self.assertEqual(done['sent']['pcl'], 24)

    def test_recorded_pushes_stand_in_for_the_simulators_own(self) -> None:
        self.start('--pcap-rate', '1', '--push-rate', '50')
        self.configure('push', 'pcl')
        self.wait_done()
        time.sleep(0.1)  # the simulator's own pushes resume
        self.s.running = False  # stop them: received() drains until the socket goes quiet
        self.thread.join(timeout=5)
        temps = [core_temp(d) for d in self.received('push')]
        first = temps.index(RECORDED_CORE_TEMP)
        # 95 ms between the recorded pushes: 4-5 of the simulator's would have fallen there.
        after = first + 2
        self.assertEqual(temps[first + 1], RECORDED_CORE_TEMP)
        self.assertNotIn(RECORDED_CORE_TEMP, temps[after:])
        self.assertGreater(len(temps), after)

    def test_unreadable_capture_fails_at_start_up(self) -> None:
        with self.assertRaises(SystemExit) as cm:
            sim.main(['--base-port', '0', '--pcap', str(FIXTURE.with_name('missing.pcap'))])
        self.assertIn('missing.pcap', str(cm.exception))
        with self.assertRaises(SystemExit) as cm:
            sim.main(['--base-port', '0', '--pcap', str(FIXTURE.with_name('mini.lvx2'))])
        self.assertIn('not a classic pcap', str(cm.exception))
        with self.assertRaises(SystemExit):
            sim.main(['--base-port', '0', '--pcap', str(FIXTURE), '--pcap-rate', '-1'])


if __name__ == '__main__':
    unittest.main()

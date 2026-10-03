#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Runs livox_mid360_probe.py against the simulator in a thread.

python3 -m unittest tools/test_probe.py
"""

from __future__ import annotations

import io
import json
import os
import pathlib
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import livox_mid360_probe as probe  # noqa: E402
import livox_mid360_sim as sim  # noqa: E402


class ProbeTest(unittest.TestCase):
    """The probe's records against the simulator's documented answers."""

    def start_sim(self, *extra: str) -> sim.Simulator:
        args = sim.build_parser().parse_args(
            ['--bind', '127.0.0.1', '--base-port', '0', '--startup-delay', '0.05', *extra]
        )
        r, w = os.pipe()
        control_r, self.control_w = os.fdopen(r, 'r'), os.fdopen(w, 'w')
        s = sim.Simulator(args, out=io.StringIO(), control=control_r)
        thread = threading.Thread(target=s.run, daemon=True)
        thread.start()

        def stop() -> None:
            self.control_w.write('{"cmd":"quit"}\n')
            self.control_w.flush()
            thread.join(timeout=5)
            self.control_w.close()
            control_r.close()

        self.addCleanup(stop)
        return s

    def run_probe(self, s: sim.Simulator, *extra: str) -> tuple[int, dict]:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        out = pathlib.Path(tmp.name) / 'probe.json'
        argv = [
            '--host-ip',
            '127.0.0.1',
            '--host-cmd-port',
            '0',
            '--push-port',
            '0',
            '--discovery-port',
            str(s.ports['discovery']),
            '--push-seconds',
            '1.5',
            '--out',
            str(out),
            *extra,
        ]
        status = probe.main(argv)
        return status, json.loads(out.read_text())

    def results(self, report: dict) -> dict[str, dict]:
        return {p['name']: p['result'] for p in report['probes']}

    def test_answers_match_the_simulator(self) -> None:
        s = self.start_sim()
        host_before = s.model.settings.get(sim.KEY_STATE_HOST)
        status, report = self.run_probe(s, '--lidar-ip', '127.0.0.1')
        self.assertEqual(status, 0)
        r = self.results(report)
        self.assertEqual(r['discovery']['dev_type'], sim.PROVISIONAL_DEV_TYPE)
        self.assertEqual(r['discovery']['cmd_port'], s.ports['cmd'])
        self.assertEqual(r['inquire_settings_keys']['answered'], 16)
        self.assertEqual(r['inquire_status_keys']['answered'], 10)
        self.assertEqual(r['inquire_each_key']['unsupported'], 0)
        self.assertEqual(r['inquire_unknown_key']['ret_code'], '0x20')
        self.assertEqual(r['inquire_unknown_key']['entries'], ['0x7FFF:0'])
        self.assertEqual(r['unknown_cmd_id']['ret_code'], '0x01')
        self.assertEqual(r['write_read_only_key']['ret_code'], '0x22')
        self.assertEqual(r['write_unknown_key']['ret_code'], '0x20')
        self.assertEqual(r['write_wrong_length']['ret_code'], '0x23')
        for name in ('detect_mode', 'time_filter', 'imu_data_en'):
            self.assertEqual(r[f'write_out_of_range_{name}']['ret_code'], '0x03')
        self.assertEqual(r['partial_write']['error_key'], '0x7FFF')
        self.assertFalse(r['partial_write']['good_key_applied'])
        self.assertGreaterEqual(r['push']['pushes'], 1)
        self.assertTrue(r['push']['restored'])
        self.assertEqual(s.model.settings.get(sim.KEY_STATE_HOST), host_before)

    def test_missing_key_is_reported(self) -> None:
        s = self.start_sim('--imu-cfg-unsupported')
        _, report = self.run_probe(s, '--lidar-ip', '127.0.0.1', '--push-seconds', '0')
        r = self.results(report)
        self.assertEqual(r['inquire_settings_keys']['ret_code'], '0x20')
        self.assertEqual(r['inquire_each_key']['failed'], ['0x002B:0x20'])
        self.assertNotIn('push', r)

    def test_accepted_write_is_restored(self) -> None:
        s = self.start_sim()
        s.model.settings[sim.KEY_DETECT_MODE] = b'\x00'
        prober = probe.Prober(
            probe.build_parser().parse_args(
                ['--lidar-ip', '127.0.0.1', '--host-ip', '127.0.0.1', '--host-cmd-port', '0']
            ),
            log=lambda _: None,
        )
        self.addCleanup(prober.close)
        prober.cmd_port = s.ports['cmd']
        prober.probe_rejected_write('accepted', '', sim.KEY_DETECT_MODE, b'\x01')
        result = prober.probes[0]['result']
        self.assertEqual(result['ret_code'], '0x00')
        self.assertTrue(result['restored'])
        self.assertEqual(s.model.settings[sim.KEY_DETECT_MODE], b'\x00')

    def test_no_answer_exits_2(self) -> None:
        s = self.start_sim()
        status, report = self.run_probe(s, '--lidar-ip', '127.0.0.9', '--timeout', '0.2')
        self.assertEqual(status, 2)
        self.assertFalse(self.results(report)['discovery']['answered'])


if __name__ == '__main__':
    unittest.main()

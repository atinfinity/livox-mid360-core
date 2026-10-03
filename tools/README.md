# tools/

Python reference implementation and helpers. No third-party packages required (Python 3.10+),
except for `rerun/make_blueprint.py`.

| File | Purpose |
|---|---|
| `livox_mid360_proto.py` | Independent pure-Python implementation of the Mid-360 wire protocol (CRC, command frames, key-value lists, data packets). Used to cross-check the C++ library. |
| `gen_golden_vectors.py` | Regenerates `tests/generated/golden_vectors.hpp` from the Python implementation. Run after changing either implementation intentionally. |
| `gen_replay_pcap.py` | Regenerates `tests/data/replay.pcap`, the synthetic capture the simulator's `--pcap` replay is tested with. |
| `livox_mid360_pcap.py` | Decodes classic pcap captures of Mid-360 traffic (control frames, push, point cloud, IMU) to JSON/CSV and counts `udp_cnt` gaps. |
| `livox_mid360_sim.py` | Mid-360 simulator: control commands, work-state machine, point-cloud/IMU/push streaming, JSON control channel on stdin. See `docs/simulator.md`. |
| `cli/` | C++ `livox-mid360-cli` (`record` to lvx2 / `replay`, `debug-data`, `info`), built with `LIVOX_MID360_BUILD_TOOLS`. See `docs/lvx2.md` and `docs/debug_data.md`. |
| `rerun/` | C++ `livox-mid360-rerun`: shows a live Mid-360 or an lvx2 file in the Rerun viewer, built with `LIVOX_MID360_BUILD_RERUN` (off by default; needs the Rerun C++ SDK). `rerun/make_blueprint.py` regenerates its default viewer layout and is the one script here that needs a third-party package (`rerun-sdk`). See `docs/rerun.md`. |
| `livox_mid360_debug_data.py` | Reads the file written by `livox-mid360-cli debug-data`. See `docs/debug_data.md`. |
| `test_sim.py` | `unittest` suite for the simulator (`python3 -m unittest tools/test_sim.py`). |

```sh
python3 tools/gen_golden_vectors.py
python3 tools/livox_mid360_pcap.py capture.pcap --json | head
python3 tools/livox_mid360_pcap.py capture.pcap --points > points.csv
python3 tools/livox_mid360_sim.py --bind 127.0.0.1 --base-port 0 --verbose
python3 tools/livox_mid360_sim.py --bind 127.0.0.1 --pcap capture.pcap --pcap-rate 0
python3 -m unittest tools/test_sim.py
```

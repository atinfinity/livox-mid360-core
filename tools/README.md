# tools/

Python reference implementation and helpers. No third-party packages required (Python 3.10+).

| File | Purpose |
|---|---|
| `livox_mid360_proto.py` | Independent pure-Python implementation of the Mid-360 wire protocol (CRC, command frames, key-value lists, data packets). Used to cross-check the C++ library. |
| `gen_golden_vectors.py` | Regenerates `tests/generated/golden_vectors.hpp` from the Python implementation. Run after changing either implementation intentionally. |
| `livox_mid360_pcap.py` | Decodes classic pcap captures of Mid-360 traffic (control frames, push, point cloud, IMU) to JSON/CSV and counts `udp_cnt` gaps. |
| `livox_mid360_sim.py` | Mid-360 simulator: control commands, work-state machine, point-cloud/IMU/push streaming, JSON control channel on stdin. See `docs/simulator.md`. |
| `test_sim.py` | `unittest` suite for the simulator (`python3 -m unittest tools/test_sim.py`). |

```sh
python3 tools/gen_golden_vectors.py
python3 tools/livox_mid360_pcap.py capture.pcap --json | head
python3 tools/livox_mid360_pcap.py capture.pcap --points > points.csv
python3 tools/livox_mid360_sim.py --bind 127.0.0.1 --base-port 0 --verbose
python3 -m unittest tools/test_sim.py
```

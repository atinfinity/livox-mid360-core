# LiDAR simulator

`tools/livox_mid360_sim.py` is a stdlib-only Python program that behaves like a Mid-360 on the
wire: it answers control commands, walks the work-state machine and streams point-cloud, IMU
and push packets. Design decisions are recorded in
[issue #3](https://github.com/atinfinity/livox-mid360-core/issues/3).

It exists so that the session layer (#4, #5, #7, #8) and the receive pipeline (#6) can be
developed and tested without hardware. It is **not** a reference for LiDAR behaviour: wherever
the wiki is silent the simulator makes an assumption, listed at the end of this page, and each
one is due for verification on real hardware (#11).

## Running

```sh
python3 tools/livox_mid360_sim.py                     # 0.0.0.0, ports 56000/56100/56200/56300/56400
python3 tools/livox_mid360_sim.py --bind 127.0.0.1 --base-port 0   # loopback, free ports
python3 tools/livox_mid360_sim.py --verbose --drop-rate 0.01
```

| Option | Default | Meaning |
|---|---|---|
| `--bind` | `0.0.0.0` | address to bind; also reported as `lidar_ip` in the discovery ACK (127.0.0.1 when unspecified) |
| `--base-port` | 56000 | discovery port; cmd, push, pcl, imu follow at +100, +200, +300, +400. `0` picks free ports |
| `--sn` | `SIM0000000000001` | serial number (≤ 16 chars) |
| `--seed` | 1 | seed for deterministic point / IMU data and packet drops |
| `--startup-delay` | 0.3 s | time spent in MOTORSTARTUP after power-on / reboot |
| `--reboot-silence` | 0.5 s | commands are ignored and nothing is sent for this long after 0x0200 / 0x0201 |
| `--frame-ms` | 100 | `frame_cnt` increments at this period; `0` never increments it (what a non-repetitive scanner is expected to do, #11) |
| `--rate-multiplier` | 1.0 | scales the 2000 pkt/s point-cloud and 200 pkt/s IMU rates |
| `--push-rate` | 1.0 | 0x0102 push rate in Hz, not affected by `--rate-multiplier` |
| `--drop-rate` | 0 | fraction of point-cloud packets silently dropped (`udp_cnt` still advances) |
| `--no-quit-on-eof` | | keep running when stdin closes (default: quit) |
| `--verbose` | | log to stderr |

The five sockets are bound at start-up, the same way the real device listens on fixed ports.
Since the real Mid-360 answers discovery on 56000 and commands on 56100, a host using the
default ports talks to the simulator without any change; a test that wants isolation passes
`--base-port 0` and reads the actual ports from the `ready` event.

## Control and events

The process is driven over its standard streams so that any test harness can use it:

- **stdin**: one JSON object per line, `{"cmd": ...}`. EOF quits unless `--no-quit-on-eof`.
- **stdout**: one JSON object per line, `{"event": ...}`. The first line is always `ready`.
- **stderr**: human-readable log with `--verbose`.

| Control | Fields | Effect |
|---|---|---|
| `quit` | | exit with status 0 after emitting `exit` |
| `silence` | `seconds` | ignore commands and stop streaming for `seconds` (simulates a link drop) |
| `hms` | `codes` (≤ 8 ints) | set the HMS code slots reported by 0x800E/0x8011 and the push |
| `drop_ack` | `count` | do not answer the next `count` requests (the request is still processed) |
| `reboot` | | same as receiving 0x0200 |
| `set_state` | `state` | force `cur_work_state` (e.g. 4 ERROR) |
| `drop_rate` | `rate` | change the point-cloud drop fraction at run time |
| `frame_ms` | `ms` | change the `frame_cnt` period at run time (`0` freezes it); the current frame restarts now |
| `status` | | emit a `status` event |

| Event | Fields | When |
|---|---|---|
| `ready` | `ip`, `ports{discovery,cmd,push,pcl,imu}`, `sn`, `pid` | sockets bound, main loop starting |
| `state` | `from`, `to` | work state changed (values as in `WorkState`) |
| `cmd` | `cmd_id`, `seq`, `ret`, `from` | a request was handled |
| `ack_dropped` | `cmd_id`, `seq` | a request was handled but the ACK withheld (`drop_ack`) |
| `bad_frame` | `from`, `error` | a datagram failed to parse |
| `sent` | `pcl`, `imu`, `push`, `pcl_dropped`, `state` | once per second |
| `control` | `cmd` | a control command was applied |
| `status` | `state`, `sent`, `hosts` | answer to `status` |
| `error` | `error` | malformed or unknown control line |
| `exit` | `sent` | leaving the main loop |

## Behaviour

- **Commands** `0x0000` discovery (unicast or broadcast; the ACK carries `dev_type = 9`
  (provisional), the bound address and the real command port), `0x0100` configure, `0x0101`
  inquire, `0x0200` reboot, `0x0201` factory reset, `0x0202` GPS time. Anything else is
  answered with ret `0x01`.
- **Configure** validates every key first with the wiki return codes (`RetCode` in
  `protocol.hpp`): read-only → `0x22`, unknown → `0x20`, wrong length → `0x23`,
  `pcl_data_type` outside 1–3 → `0x03`. All keys are applied only if none failed; the ACK's
  `error_key` names the offender. `0x21` (reboot required) is never produced because no
  simulated key needs a reboot. Value lengths mirror `key_value_length()` in
  `keys.cpp`.
- **State machine** power-on → MOTORSTARTUP → `work_tgt_mode` (SAMPLING by default) after
  `--startup-delay`. Writing `work_tgt_mode` switches immediately when not starting up.
  Reboot and factory reset go back through MOTORSTARTUP, reset `udp_cnt`/`frame_cnt`/`seq`,
  and stay silent for `--reboot-silence`. Reboot keeps every setting except `work_tgt_mode`;
  factory reset restores `factory_settings()`.
- **Streaming** while SAMPLING: point-cloud packets of 96 points in the configured
  `pcl_data_type` at 2000 pkt/s to the host in key `0x0006`, IMU packets at 200 pkt/s to the
  host in `0x0007` when `imu_data_en = 1`, and a `0x0102` push once per second to the host in
  `0x0005`. Nothing is sent to a host whose IP is 0.0.0.0. Packet timestamps are
  `time.time_ns()` plus the GPS offset from 0x0202. The scheduler bounds catch-up bursts to
  256 packets and resynchronises if it falls more than 0.5 s behind.
- **Data** is pseudo-random but deterministic for a seed; the C++ decoder only needs valid
  framing, CRCs and counters.

## Tests

- `python3 -m unittest tools/test_sim.py`: the pure `DeviceModel` (transitions, configure
  rules, reboot / factory-reset persistence, GPS offset, push payload), `PointSource`
  determinism, and an in-process end-to-end run over UDP (discovery → configure → packets →
  push → reboot silence → `udp_cnt` reset; `hms` and `drop_ack` controls).
- `tests/test_sim_smoke.cpp` (Catch2, tag `[sim]`): spawns the simulator with `posix_spawn`
  through `tests/sim_process.hpp`, then discovery → 0x0100 → wait for SAMPLING via 0x0101 →
  receive ≥ 200 point-cloud and ≥ 10 IMU packets through `UdpSocket` / `Poller` → quit. The
  test is skipped when no Python interpreter is found; CMake passes `Python3_EXECUTABLE` as
  `LIVOX_MID360_PYTHON` and the script path as `LIVOX_MID360_SIM_SCRIPT`.

`SimProcess::control()` sends any control line and `wait_event()` blocks for a matching
stdout line, so later session-layer tests can inject reboots, HMS codes or dropped ACKs.

## Assumptions to verify on hardware (#11)

| Topic | Simulator behaviour | Why unverified |
|---|---|---|
| `dev_type` in the discovery ACK | 9 | the wiki lists no value for Mid-360 |
| Unicast discovery | answered like broadcast | wiki says "broadcast only" |
| Discovery ACK `cmd_port` | the bound command port (56100 by default) | |
| Persistence across reboot | all keys except `work_tgt_mode` | wiki only marks `work_tgt_mode` as volatile |
| Silence after reboot | ~0.5 s, then MOTORSTARTUP → target | real duration unknown |
| Write of a read-only key | ret `0x22`, `error_key` = that key | wiki lists the codes but not which the firmware actually uses |
| Unknown key | ret `0x20` | same |
| `lidar_ipcfg` write | ret `0x00` (no reboot required) | real device likely answers `0x21` |
| Unknown `cmd_id` | ret `0x01` | no ACK at all is also plausible |
| Multi-key config with one bad key | nothing applied | vs. partial application |
| Push contents | `cur_work_state`, `diag_status`, `hms`, `sn`, `local_time` | the wiki does not enumerate the pushed keys |
| `frame_cnt` period | 100 ms | |

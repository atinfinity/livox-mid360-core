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
| `--base-port` | 56000 | discovery port; cmd, push, pcl, imu, log follow at +100, +200, +300, +400, +500. `0` picks free ports |
| `--sn` | `SIM0000000000001` | serial number (≤ 16 chars) |
| `--product-info` | `MID360-SIM` | key 0x8001 (≤ 64 chars) |
| `--version-app` / `--version-loader` / `--version-hardware` | `0.0.0.1` | keys 0x8002–0x8004 as `a.b.c.d` |
| `--seed` | 1 | seed for deterministic point / IMU data and packet drops |
| `--startup-delay` | 0.3 s | time spent in MOTORSTARTUP (after power-on / reboot and whenever the motor starts from IDLE) |
| `--selfcheck-delay` | 0.1 s | time spent in SELFCHECK after power-on / reboot |
| `--reboot-silence` | 0.5 s | commands are ignored and nothing is sent for this long after 0x0200 / 0x0201 |
| `--frame-ms` | 100 | `frame_cnt` increments at this period; `0` never increments it (what a non-repetitive scanner is expected to do, #11) |
| `--rate-multiplier` | 1.0 | scales the 2000 pkt/s point-cloud and the IMU rate (200 pkt/s unless `0x002B` selects another) |
| `--push-rate` | 1.0 | 0x0102 push rate in Hz, not affected by `--rate-multiplier` |
| `--drop-rate` | 0 | fraction of point-cloud packets silently dropped (`udp_cnt` still advances) |
| `--imu-cfg-unsupported` | | emulate firmware without key `0x002B`: its write, read and any inquire naming it answer `0x20` |
| `--log-chunk-interval` | 0.05 s | period of firmware log chunks (0x0300) per enabled log type (#44) |
| `--log-chunk-bytes` | 512 | data bytes per log chunk |
| `--log-ack-every` | 1 | ask for a host ACK on every Nth chunk; `0` never (the file-end packet always asks) |
| `--log-ignore-hostcfg` | | send log chunks to the sender of 0x0301 instead of the host in key 0x0009 |
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
| `reboot` | | reboot silence, counters reset; if the stored key 0x0004 address differs from the bound one, every socket is rebound to it keeping the ports and a `rebound` event is emitted (a failed bind emits `error` and keeps the old sockets) |
| `set_status` | any of `diag` (u16 bitfield), `core_temp` (0.01 °C), `time_sync_type`, `time_offset_ns`, `last_sync_time`, `powerup_cnt`, `omit_keys` (list of key ids left out of the push) | overwrite the read-only status keys the push and 0x0101 report (#56 / #55) |
| `reboot` | | same as receiving 0x0200 |
| `set_state` | `state` | force `cur_work_state` (e.g. 4 ERROR); `work_tgt_mode` is untouched, so forcing a work substate makes the machine chase the target again |
| `drop_rate` | `rate` | change the point-cloud drop fraction at run time |
| `frame_ms` | `ms` | change the `frame_cnt` period at run time (`0` freezes it); the current frame restarts now |
| `log_drop` | `n` | skip the next `n` log chunks (`trans_index` still advances → gap on the host) |
| `log_new_file` | | end the current firmware log file(s) and start the next `file_index` |
| `status` | | emit a `status` event |

| Event | Fields | When |
|---|---|---|
| `ready` | `ip`, `ports{discovery,cmd,push,pcl,imu,log}`, `sn`, `pid` | sockets bound, main loop starting |
| `state` | `from`, `to` | work state changed (values as in `WorkState`) |
| `cmd` | `cmd_id`, `seq`, `ret`, `from` | a request was handled |
| `ack_dropped` | `cmd_id`, `seq` | a request was handled but the ACK withheld (`drop_ack`) |
| `bad_frame` | `from`, `error` | a datagram failed to parse |
| `sent` | `pcl`, `imu`, `push`, `pcl_dropped`, `log`, `state` | once per second |
| `log_dropped` | `file_index`, `trans` | a log chunk was withheld (`log_drop`) |
| `log_ack` | `ret`, `log_type`, `file_index`, `trans` | the host acknowledged a log chunk |
| `control` | `cmd` | a control command was applied |
| `status` | `state`, `sent`, `hosts{pcl,imu,push,log}`, `log_enabled`, `log_acks_received` | answer to `status` |
| `error` | `error` | malformed or unknown control line |
| `exit` | `sent` | leaving the main loop |

## Behaviour

- **Firmware log** (#44): a sixth socket at `+500` answers `0x0301` (payload `{log_type,
  enable}`) and streams `0x0300` chunks of `--log-chunk-bytes` synthetic text every
  `--log-chunk-interval` for each enabled type. The first chunk of a file carries the begin
  flag, every `--log-ack-every`th the ACK flag; host ACKs (REQ `0x0300` with `{ret, type,
  file_index, trans_index}`) are counted in `status.log_acks_received`.

- **Commands** `0x0000` discovery (unicast or broadcast; the ACK carries `dev_type = 9`
  (provisional), the bound address and the real command port), `0x0100` configure, `0x0101`
  inquire, `0x0200` reboot, `0x0201` factory reset, `0x0202` GPS time. Anything else is
  answered with ret `0x01`.
- **Configure** validates every key first with the wiki return codes (`RetCode` in
  `protocol.hpp`): read-only → `0x22`, unknown → `0x20`, wrong length → `0x23`,
  `pcl_data_type` outside 1–3 → `0x03`, `pattern_mode` 1 / 2 → `0x20` and any other non-zero
  value → `0x03` (the base Mid-360 only scans non-repetitively, [unverified] which code, #11),
  a FOV window (`0x0015` / `0x0016`) with yaw
  outside [0, 360) or pitch outside (-10, 60) → `0x03`, `detect_mode` / `time_filter` /
  `imu_data_en` above 1 → `0x03`, an `imu_sensor_cfg` byte past its last enumerator (rate
  > 3, accel > 3, gyro > 7) → `0x03`. All keys are applied only if none failed; the ACK's
  `error_key` names the offender. A rejected `0x0101` inquire (unknown key) answers the
  return code with the offending key as a single zero-length entry. A *changed* `lidar_ipcfg` is stored and answered with `0x21`
  (reboot required, [unverified] which keys the LiDAR does this for, #11); writing the current
  value back is a plain `0x00`. Value lengths mirror `key_value_length()` in `keys.cpp`.
- **State machine** the figure in [protocol_notes.md](protocol_notes.md#working-state):
  power-on → SELFCHECK (`--selfcheck-delay`, commands are answered) → IDLE, then the machine
  chases `work_tgt_mode` (SAMPLING by default): IDLE → MOTORSTARTUP (`--startup-delay`) →
  READY → SAMPLING. The pass-through READY has no dwell (SAMPLING → IDLE is immediate), but
  every transition emits a `state` event. `work_tgt_mode` accepts 1 / 2 / 9 only (4 / 5 / 6 /
  8 → `0x20`, undefined → `0x03`, in ERROR / UPGRADE → `0x02`); a write during SELFCHECK /
  MOTORSTARTUP is stored and followed afterwards. ERROR / UPGRADE are entered only by `set_state` and left by
  `set_state` or a reboot. Reboot and factory reset go back through SELFCHECK, reset
  `udp_cnt`/`frame_cnt`/`seq`, and stay silent for `--reboot-silence`. Reboot keeps every
  setting except `work_tgt_mode`; factory reset restores `factory_settings()`. All durations
  and the return codes are assumptions ([#11](https://github.com/atinfinity/livox-mid360-core/issues/11)).
- **Streaming** while SAMPLING: point-cloud packets of 96 points in the configured
  `pcl_data_type` at 2000 pkt/s to the host in key `0x0006`, IMU packets at the `0x002B` rate (200 pkt/s by default) to the
  host in `0x0007` when `imu_data_en = 1`, and a `0x0102` push once per second to the host in
  `0x0005`. Nothing is sent to a host whose IP is 0.0.0.0. Packet timestamps are
  `time.time_ns()` plus the GPS offset from 0x0202. The scheduler bounds catch-up bursts to
  256 packets and resynchronises if it falls more than 0.5 s behind.
- **Data** is pseudo-random but deterministic for a seed; the C++ decoder only needs valid
  framing, CRCs and counters.
- **FOV cropping** [unverified]: when `fov_cfg_en` enables at least one window, a point is
  sent only if it lies inside an enabled window (yaw `[start, stop)` with wrap-around when
  `start > stop`, `start == stop` empty; pitch `[start, stop]`). Cartesian points use
  `yaw = atan2(y, x)`, `pitch = atan2(z, hypot(x, y))`; spherical ones `phi` and
  `90° - theta`. Each packet draws up to 16 batches of 96 points to fill its 96 slots, so a
  narrow window only slows the generator; an empty window sends packets with `dot_num = 0`.

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
| Silence after reboot | ~0.5 s, then SELFCHECK → IDLE → target | real durations unknown |
| SELFCHECK / MOTORSTARTUP | 0.1 s / 0.3 s, commands answered | real durations unknown; the figure gives the edges only |
| `work_tgt_mode` rejections | `0x20` / `0x03` / `0x02` (see State machine) | wiki lists the codes but not which the firmware uses |
| Write of a read-only key | ret `0x22`, `error_key` = that key | wiki lists the codes but not which the firmware actually uses |
| Unknown key | ret `0x20` | same |
| `0x0101` inquire with an unknown key | ret `0x20`, `key_num` 1 and the offending key with length 0 | wiki does not describe a failed inquire ACK |
| `detect_mode` / `time_filter` / `imu_data_en` values > 1 | ret `0x03` | wiki gives the codes, not which one |
| `imu_sensor_cfg` | per-byte range → `0x03`; accepted rate switches the IMU stream to 200 / 500 / 100 / 50 pkt/s at once (and `time_interval`); factory default `0/0/0`; `--imu-cfg-unsupported` → `0x20` on write and read | the wiki gives no default, no firmware version and does not say whether the rate change is immediate |
| `time_filter` | stored only; the simulator has no time source to roll back | wiki describes the rollback behaviour, not testable here |
| `lidar_ipcfg` write | ret `0x21` (reboot required) when the value changes | wiki lists `0x21` but not when the firmware uses it |
| Unknown `cmd_id` | ret `0x01` | no ACK at all is also plausible |
| Multi-key config with one bad key | nothing applied | vs. partial application |
| Push contents | every read-only key `0x8000`–`0x8011` | the wiki does not enumerate the pushed keys |
| Firmware log (#44) | `0x0301` on the log socket enables / disables a type, ret `0x00` even when repeated; chunks go to key `0x0009` (else to the `0x0301` sender); `file_index` starts at 1, `trans_index` at 1 with the begin flag, `file_num` is 1, `timestamp` is Unix seconds; a disable sends one empty end-flagged chunk | the SDK2 source shows the wire format only; the LiDAR's counting, destination choice and end-of-file behaviour are unknown |
| FOV window ranges | yaw outside [0, 360) or pitch outside (-10, 60) → `0x03`; equal / reversed start-stop accepted | the wiki gives the ranges, not the code, nor what a reversed window means |
| FOV write while SAMPLING | applied at once, ret `0x00` (no `0x21`) | the wiki does not say whether FOV keys need a reboot or a motor restart |
| `pattern_mode` | only 0 accepted; 1 / 2 → `0x20`, others → `0x03`; never restarts the motor | the wiki gives the values and the "scan mode changed" edge, not which ones the base Mid-360 accepts nor the code |
| `pcl_data_type` change while SAMPLING | the next packet is already in the new format | the wiki does not say whether the switch is immediate or aligned to a frame |
| FOV cropping | yaw `[start, stop)` wrapping when `start > stop`, `start == stop` empty; pitch `[start, stop]`; keep if inside any enabled window | the wiki defines neither the edge inclusivity nor the wrap-around |
| Inquire of all settings / status keys at once | one ACK with every key | wiki gives no limit on keys per `0x0101` |
| `frame_cnt` period | 100 ms | |

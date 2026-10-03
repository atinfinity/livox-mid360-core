# LiDAR simulator

`tools/livox_mid360_sim.py` is a stdlib-only Python program that behaves like a Mid-360 on the
wire: it answers control commands, walks the work-state machine and streams point-cloud, IMU
and push packets. Design decisions are recorded in
[issue #3](https://github.com/atinfinity/livox-mid360-core/issues/3).

It exists so that the session layer ([#4](https://github.com/atinfinity/livox-mid360-core/issues/4), [#5](https://github.com/atinfinity/livox-mid360-core/issues/5), [#7](https://github.com/atinfinity/livox-mid360-core/issues/7), [#8](https://github.com/atinfinity/livox-mid360-core/issues/8)) and the receive pipeline ([#6](https://github.com/atinfinity/livox-mid360-core/issues/6)) can be
developed and tested without hardware. It is **not** a reference for LiDAR behaviour: wherever
the wiki is silent the simulator makes an assumption, listed at the end of this page, and each
one is due for verification on real hardware ([#11](https://github.com/atinfinity/livox-mid360-core/issues/11)).

## Running

```sh
python3 tools/livox_mid360_sim.py                     # 0.0.0.0, ports 56000/56100/56200/56300/56400
python3 tools/livox_mid360_sim.py --bind 127.0.0.1 --base-port 0   # loopback, free ports
python3 tools/livox_mid360_sim.py --verbose --drop-rate 0.01
python3 tools/livox_mid360_sim.py --pcap capture.pcap --pcap-rate 0.5   # replay a capture at half speed
```

| Option | Default | Meaning |
|---|---|---|
| `--bind` | `0.0.0.0` | address to bind; also reported as `lidar_ip` in the discovery ACK. Bound to `0.0.0.0`, the ACK carries the local address on the route to the requester, so a host off loopback can connect ([#203](https://github.com/atinfinity/livox-mid360-core/issues/203)); key 0x0004 then reports 127.0.0.1 |
| `--base-port` | 56000 | discovery port; cmd, push, pcl, imu, log follow at +100, +200, +300, +400, +500. `0` picks free ports |
| `--sn` | `SIM0000000000001` | serial number (≤ 16 chars) |
| `--product-info` | `MID360-SIM` | key 0x8001 (≤ 64 chars) |
| `--version-app` / `--version-loader` / `--version-hardware` | `0.0.0.1` | keys 0x8002–0x8004 as `a.b.c.d` |
| `--seed` | 1 | seed for deterministic point / IMU data and packet drops |
| `--scene` | `random` | point cloud content: seeded random points, or `ring`, a fixed scene of known points ([#132](https://github.com/atinfinity/livox-mid360-core/issues/132)) |
| `--apply-attitude` | | move Cartesian points by the install attitude in key `0x0012` before sending them ([#135](https://github.com/atinfinity/livox-mid360-core/issues/135)); off by default, because whether the firmware does this is [unverified] |
| `--pcap` | | replay the point cloud, IMU, push and firmware log datagrams of a classic pcap instead of generating them ([#134](https://github.com/atinfinity/livox-mid360-core/issues/134), see Behaviour) |
| `--pcap-rate` | 1.0 | `--pcap` speed: `2` replays twice as fast as recorded, `0` as fast as possible |
| `--startup-delay` | 0.3 s | time spent in MOTORSTARTUP (after power-on / reboot and whenever the motor starts from IDLE) |
| `--selfcheck-delay` | 0.1 s | time spent in SELFCHECK after power-on / reboot |
| `--reboot-silence` | 0.5 s | commands are ignored and nothing is sent for this long after 0x0200 / 0x0201 |
| `--frame-ms` | 100 | `frame_cnt` increments at this period; `0` never increments it (what a non-repetitive scanner is expected to do, [#11](https://github.com/atinfinity/livox-mid360-core/issues/11)) |
| `--rate-multiplier` | 1.0 | scales the 2000 pkt/s point-cloud and the IMU rate (200 pkt/s unless `0x002B` selects another) |
| `--push-rate` | 1.0 | 0x0102 push rate in Hz, not affected by `--rate-multiplier` |
| `--drop-rate` | 0 | fraction of point-cloud packets silently dropped (`udp_cnt` still advances) |
| `--imu-cfg-unsupported` | | emulate firmware without key `0x002B`: its write, read and any inquire naming it answer `0x20` |
| `--log-chunk-interval` | 0.05 s | period of firmware log chunks (0x0300) per enabled log type ([#44](https://github.com/atinfinity/livox-mid360-core/issues/44)) |
| `--log-chunk-bytes` | 512 | data bytes per log chunk |
| `--log-ack-every` | 1 | ask for a host ACK on every Nth chunk; `0` never (the file-end packet always asks) |
| `--log-ignore-hostcfg` | | send log chunks to the sender of 0x0301 instead of the host in key 0x0009 |
| `--debug-data-port` | 60301 | source port of the debug raw data stream ([#93](https://github.com/atinfinity/livox-mid360-core/issues/93)); `0` or `--base-port 0` picks a free port |
| `--debug-data-interval` | 0.01 s | period of the debug raw data datagrams while `0x0303` has enabled them |
| `--debug-data-bytes` | 1024 | size of a debug raw data datagram |
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
| `silence` | `seconds` | simulates a link drop: for `seconds` commands are ignored and nothing is sent, but the LiDAR keeps running, so `udp_cnt`, `frame_cnt`, the push `seq` and the log `trans_index` advance and the host sees a gap afterwards (no catch-up burst). A later `silence` or a reboot only extends it |
| `hms` | `codes` (≤ 8 ints) | set the HMS code slots reported by 0x800E/0x8011 and the push |
| `drop_ack` | `count` | do not answer the next `count` requests (the request is still processed) |
| `fail_cmd` | `cmd_id`, `ret` (default 1), `count` (default 1), `skip` (default 0), `key` (optional) | after letting `skip` of them pass, answer the next `count` requests with this `cmd_id` with `ret` without applying them; with `key` only `0x0100` / `0x0101` requests that name the key match (it is reported as `error_key` of `0x0100`). One rule per `cmd_id` |
| `inquire_override` | `key`, and one of `value` (hex string, may be empty), `omit`, `unsupported`, `clear` (each `true`) | what `0x0101` answers for `key` from now on: these bytes instead of the stored value, the key left out of the ACK, the whole inquire rejected with `0x20` naming the key, or the normal answer again. The push and `0x0100` are not affected |
| `reboot` | | same as receiving 0x0200: reboot silence, counters reset; if the stored key 0x0004 address differs from the bound one, every socket is rebound to it keeping the ports and a `rebound` event is emitted (a failed bind emits `error` and keeps the old sockets) |
| `set_status` | any of `diag` (u16 bitfield), `core_temp` (0.01 °C), `powerup_cnt`, `bad_time_offset` (1: 0x800B answered truncated to 4 bytes), `omit_keys` (list of key ids left out of the push); a value that is not an integer applies nothing | overwrite the read-only status keys the push and 0x0101 report ([#56](https://github.com/atinfinity/livox-mid360-core/issues/56) / [#55](https://github.com/atinfinity/livox-mid360-core/issues/55)); the time keys `0x8009`–`0x800C` follow the clock, see `time_sync` |
| `time_sync` | any of `type` (`none`, `ptp` or `gps`), `offset_ns` (default 0), `drift_ppm` | `ptp` / `gps`: acquire synchronisation to a master at the host's wall clock plus `offset_ns`, stepping the clock to it; `none`: lose it, free running on from the current time; `drift_ppm`: the free-running rate from now on, without a step ([#133](https://github.com/atinfinity/livox-mid360-core/issues/133)). Emits `time_sync` |
| `set_state` | `state` | force `cur_work_state` (e.g. 4 ERROR); `work_tgt_mode` is untouched, so forcing a work substate makes the machine chase the target again |
| `drop_rate` | `rate` | change the point-cloud drop fraction at run time |
| `drop` | `stream` (`pcl` default, or `imu`), `count` (default 1) | do not send the next `count` packets of the stream; `udp_cnt` still advances, so the host sees a gap ([#131](https://github.com/atinfinity/livox-mid360-core/issues/131)) |
| `reorder` | `stream`, `count` (default 1), `depth` (default 1, ≥ 1) | hold the next packet back and send it after `depth` more packets, `count` times one after the other (a late packet: `udp_cnt` goes backwards on the host) |
| `duplicate` | `stream`, `count` (default 1) | send each of the next `count` packets twice |
| `scene` | `name` (`random` or `ring`) | switch the point cloud content at run time; the ring restarts at point 0 |
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
| `sent` | `pcl`, `imu`, `push`, `pcl_dropped` (`--drop-rate` and `drop`), `pcl_reordered`, `pcl_duplicated`, `imu_dropped`, `imu_reordered`, `imu_duplicated`, `log`, `debug`, `silenced` (datagrams withheld by a silence), `state` | once per second, also while silent |
| `packet_fault` | `stream`, `fault` (`drop`, `reorder`, `duplicate`), `udp_cnt` | one packet was affected by a `drop` / `reorder` / `duplicate` control (for `reorder`: when the held packet is sent) |
| `log_dropped` | `file_index`, `trans` | a log chunk was withheld (`log_drop`) |
| `log_ack` | `ret`, `log_type`, `file_index`, `trans` | the host acknowledged a log chunk |
| `debug_data` | `enabled`, `dest`, `port` (both only when enabled) | `0x0303` switched the debug raw data stream; `port` is its source port |
| `control` | `cmd` | a control command was applied |
| `status` | `state`, `sent`, `hosts{pcl,imu,push,log}`, `log_enabled`, `log_acks_received`, `debug_data{enabled,dest,port}`, `replay{state,sent,skipped}` (`null` without `--pcap`), `time` (as in `time_sync`) | answer to `status` |
| `time_sync` | `type`, `time_ns` (the LiDAR clock now), `last_sync_ns` (0x800A), `offset_ns` (0x800B), `drift_ppm` | the `time_sync` control was applied |
| `replay_start` | `file`, `rate` | the `--pcap` replay began |
| `replay_done` | `sent{push,pcl,imu,log}`, `skipped` (datagrams of a stream without a host), `seconds` | the `--pcap` replay sent the capture's last datagram |
| `rebound` | `ip`, `ports` | a reboot moved every socket to the key 0x0004 address |
| `error` | `error` | malformed or unknown control line, or one with a missing or invalid field (nothing of it is applied, the simulator keeps running) |
| `exit` | `sent` | leaving the main loop |

## Behaviour

- **Firmware log** ([#44](https://github.com/atinfinity/livox-mid360-core/issues/44)): a sixth socket at `+500` answers `0x0301` (payload `{log_type,
  enable}`) and streams `0x0300` chunks of `--log-chunk-bytes` synthetic text every
  `--log-chunk-interval` for each enabled type. The first chunk of a file carries the begin
  flag, every `--log-ack-every`th the ACK flag; host ACKs (REQ `0x0300` with `{ret, type,
  file_index, trans_index}`) are counted in `status.log_acks_received`.
- **Debug raw data** ([#93](https://github.com/atinfinity/livox-mid360-core/issues/93)): `0x0303` (payload `{enable, host_ip, host_port, reserved}`) is
  answered on the log socket and on the command socket. While enabled, a datagram of
  `--debug-data-bytes` goes to `host_ip:host_port` every `--debug-data-interval`: a `u32`
  sequence number (little-endian, from 0) followed by bytes counting up from its low byte, so
  that a receiver can detect loss and corruption. The source socket is bound by the first
  enable, not at start-up, because 60301 lies inside the Linux ephemeral port range; it is not
  part of `ready.ports`. The point cloud keeps flowing.
- **Packet faults** ([#131](https://github.com/atinfinity/livox-mid360-core/issues/131)): `drop`, `reorder` and `duplicate` act on a point-cloud or
  IMU packet after its `udp_cnt` and `frame_cnt` are assigned, so the host sees what a network
  would do to real packets. `--drop-rate` is applied first, then per packet a pending `drop`,
  then a `reorder` (one packet held at a time), then the send (twice for a `duplicate`). A
  packet still held back at a reboot is lost.

- **Commands** `0x0000` discovery (unicast or broadcast; the ACK carries `dev_type = 9`
  (provisional), the bound address and the real command port), `0x0100` configure, `0x0101`
  inquire, `0x0200` reboot, `0x0201` factory reset, `0x0202` GPS time. Anything else is
  answered with ret `0x01`.
- **Configure** validates every key first with the wiki return codes (`RetCode` in
  `protocol.hpp`): read-only → `0x22`, unknown → `0x20`, wrong length → `0x23`,
  `pcl_data_type` outside 1–3 → `0x03`, `pattern_mode` 1 / 2 → `0x20` and any other non-zero
  value → `0x03` (the base Mid-360 only scans non-repetitively, [unverified] which code, [#11](https://github.com/atinfinity/livox-mid360-core/issues/11)),
  a FOV window (`0x0015` / `0x0016`) with yaw
  outside [0, 360) or pitch outside (-10, 60) → `0x03`, `detect_mode` / `time_filter` /
  `imu_data_en` above 1 → `0x03`, an `imu_sensor_cfg` byte past its last enumerator (rate
  > 3, accel > 3, gyro > 7) → `0x03`. All keys are applied only if none failed; the ACK's
  `error_key` names the offender. A rejected `0x0101` inquire (unknown key) answers the
  return code with the offending key as a single zero-length entry. A *changed* `lidar_ipcfg` is stored and answered with `0x21`
  (reboot required, [unverified] which keys the LiDAR does this for, [#11](https://github.com/atinfinity/livox-mid360-core/issues/11)); writing the current
  value back is a plain `0x00`. Value lengths mirror `key_value_length()` in `keys.cpp`.
- **State machine** the figure in [protocol_notes.md](protocol_notes.md#working-state):
  power-on → SELFCHECK (`--selfcheck-delay`, commands are answered) → IDLE, then the machine
  chases `work_tgt_mode` (SAMPLING by default): IDLE → MOTORSTARTUP (`--startup-delay`) →
  READY → SAMPLING. The pass-through READY has no dwell (SAMPLING → IDLE is immediate), but
  every transition emits a `state` event. `work_tgt_mode` accepts 1 / 2 / 9 only (4 / 5 / 6 /
  8 → `0x20`, undefined → `0x03`, in ERROR / UPGRADE → `0x02`); a write during SELFCHECK /
  MOTORSTARTUP is stored and followed afterwards. ERROR / UPGRADE are entered only by `set_state` and left by
  `set_state` or a reboot. Reboot and factory reset go back through SELFCHECK, reset
  `udp_cnt`/`frame_cnt`/`seq`, stop the debug raw data stream and stay silent for
  `--reboot-silence`; the push and log chunks resume after the silence. Reboot keeps every
  setting except `work_tgt_mode`; factory reset restores `factory_settings()` except key
  `0x0004`, which goes back to the address the simulator started on (it cannot move to the
  real factory address 192.168.1.100; after a rebind it moves back). All durations
  and the return codes are assumptions ([#11](https://github.com/atinfinity/livox-mid360-core/issues/11)).
- **Streaming** while SAMPLING: point-cloud packets of 96 points in the configured
  `pcl_data_type` at 2000 pkt/s to the host in key `0x0006`, IMU packets at the `0x002B` rate (200 pkt/s by default) to the
  host in `0x0007` when `imu_data_en = 1`, and a `0x0102` push once per second to the host in
  `0x0005`. Nothing is sent to a host whose IP is 0.0.0.0. Packet timestamps and
  `time_type` come from the LiDAR clock (below). The scheduler bounds catch-up bursts to
  256 packets or 20 ms, whichever ends first, so commands are answered and the push is sent
  while it catches up ([#183](https://github.com/atinfinity/livox-mid360-core/issues/183)). It
  resynchronises if it falls more than 0.5 s behind. `frame_cnt` follows the
  packets' scheduled times, so a burst still changes it once per `--frame-ms`, and a resync
  changes it once ([#155](https://github.com/atinfinity/livox-mid360-core/issues/155)).
- **Time** ([#133](https://github.com/atinfinity/livox-mid360-core/issues/133)): the LiDAR clock stamps every data packet and answers keys
  `0x8009`–`0x800C`. Unsynchronised (`time_type` 0) it counts from power-on and restarts at
  0 with every reboot, at the host's monotonic rate times `1 + drift_ppm / 1e6`. `0x0202`
  (GPS, taking the request's arrival as the PPS edge) or a `time_sync` control with `ptp` /
  `gps` synchronises it: from then on it reads the master time, the host's wall clock plus the
  master's offset (for `0x0202`, the GPS time minus the host time at arrival), and data
  packets carry `time_type` 1 or 2. Each synchronisation is a step: `0x800A` becomes the
  master time stepped to and `0x800B` the clock's time just before minus that time
  (local − source). The drift only applies while free running. Losing the sync (`time_sync`
  `none`) is not a step: the clock runs on free from the time it had, and `0x800A` /
  `0x800B` keep the last sync. A reboot loses the sync and keeps the drift. With a steady
  `ptp` master the real LiDAR would refresh `0x800A` / `0x800B` at every sync message; the
  simulator only at the step.
- **Data** with `--scene random` is pseudo-random but deterministic for a seed; the C++
  decoder only needs valid framing, CRCs and counters.
- **Ring scene** ([#132](https://github.com/atinfinity/livox-mid360-core/issues/132)): with `--scene ring` (or the `scene` control) every
  point is one of 256 known points, so a test can check decoded values, not only framing.
  Point `k` (0–255) has azimuth `k * 36000 // 256` in 0.01° (so `k = 0, 64, 128, 192` lie on
  the axes), elevation `(0, 15, -5, 45)[k % 4]`°, depth `1000 + 30 * k` mm, reflectivity `k`
  and tag `k % 3 | (k // 3 % 3) << 2 | (k // 9 % 3) << 4` (every glue / particle / other
  value, reserved bits 0). Packets carry consecutive points, wrapping after 255, and the
  cursor goes back to 0 whenever `frame_cnt` changes, at a reboot and at a `scene` control:
  the first packet of a frame starts at point 0 and every packet at a multiple of 32.
  Spherical packets carry the exact values (`theta` = 90° − elevation); Cartesian32 /
  Cartesian16 ones the same point rounded to mm / cm. `tools/test_sim.py` (`ring_point`)
  and `tests/test_scene.cpp` restate the definition.
- **Install attitude** ([#135](https://github.com/atinfinity/livox-mid360-core/issues/135)) [unverified]: key `0x0012` is stored and read back, and by
  default it does not touch the points. With `--apply-attitude` every Cartesian32 /
  Cartesian16 point is moved by `Rz(yaw) * Ry(pitch) * Rx(roll)`, then by the translation
  (the convention of `extrinsic_from()`), before it is rounded to mm / cm; values past the
  field range are clamped. The key's current value is used, so a write takes effect with the
  next packet. Spherical points are sent unchanged. The FOV crops in the sensor frame, before
  the attitude. A host that also applies `extrinsic_from()` then transforms the cloud twice
  (see [api.md](api.md#install-attitude-and-host-side-extrinsic)).
- **Pcap replay** ([#134](https://github.com/atinfinity/livox-mid360-core/issues/134)): with `--pcap FILE` the point cloud and IMU come from a
  capture instead of the generator. The file is read with `tools/livox_mid360_pcap.py`:
  classic pcap only (convert pcapng with `editcap -F pcap`), Ethernet (with VLAN), raw IPv4
  or Linux cooked, IPv4 UDP. Every datagram sent *from* a LiDAR data port (push 56200, point
  cloud 56300, IMU 56400, log 56500) is replayed byte for byte, from the simulator's socket
  of that stream to the host the SDK configured for it (keys `0x0005`, `0x0006`, `0x0007`,
  `0x0009`); one without a host is counted as `skipped`. Command traffic in the capture
  (discovery, `0x0100`, their ACKs) is not replayed: the `DeviceModel` answers the SDK as
  usual, so a replay connects like the live simulator. The replay starts once the work state
  is SAMPLING and a host is configured for one of these streams (usually by the SDK's first
  `0x0100`), keeps the recorded spacing divided by `--pcap-rate`, then runs to the end of
  the file whatever the work state, and ends with `replay_done`. It runs once; afterwards no
  point cloud or IMU is sent. From the first replayed push to `replay_done` the capture's
  pushes replace the simulator's own, which then resume. Packets keep their recorded
  `udp_cnt`, `frame_cnt` and timestamps. The capture should hold one LiDAR: datagrams from
  several are replayed as if they were one. `tests/data/replay.pcap` is a synthetic
  capture written by `tools/gen_replay_pcap.py` (three frames of eight ring-scene packets,
  twelve IMU packets, two pushes with `core_temp` 43.21 °C, one log chunk, and command and
  unrelated traffic that must be skipped); CI regenerates it and fails on a difference.
- **FOV cropping** [unverified]: when `fov_cfg_en` enables at least one window, a point is
  sent only if it lies inside an enabled window (yaw `[start, stop)` with wrap-around when
  `start > stop`, `start == stop` empty; pitch `[start, stop]`). Cartesian points use
  `yaw = atan2(y, x)`, `pitch = atan2(z, hypot(x, y))`; spherical ones `phi` and
  `90° - theta`. Each packet draws up to 16 batches of 96 points to fill its 96 slots, so a
  narrow window only slows the generator. A narrow enough window makes it slower than the
  packet rate: the stream then runs behind and resynchronises every 0.5 s. `dot_num` is the
  number of points kept, so a tiny window sends shorter packets and an empty one packets
  with `dot_num = 0`. The ring scene instead crops its next 96 points on their exact angles,
  whatever the data type, and sends what is left, so the kept set is known: a window of yaw `[0, 90)` and pitch `[0, 15]` keeps
  exactly the `k < 64` with `k % 4` in {0, 1}.

## Tests

- `python3 -m unittest tools/test_sim.py`: the pure `DeviceModel` (transitions, configure
  rules, reboot / factory-reset persistence, GPS time, push payload), the `LidarClock`
  (power-on time, drift, steps, loss without a step), `PointSource`
  determinism, and an in-process end-to-end run over UDP (discovery → configure → packets →
  push → reboot silence → `udp_cnt` reset; `hms` and `drop_ack` controls), the packet
  fault controls on both data streams, and the ring scene in every data type and under a FOV
  window, the `--apply-attitude` transform, and the `time_sync` control on the packets'
  `time_type` and timestamps. `PcapReplayTest` replays
  `tests/data/replay.pcap` into sockets and compares every datagram byte for byte with the
  capture, and checks the start condition, the skipped streams, the rate and the push
  hand-over.
- `tests/test_sim_smoke.cpp` (Catch2, tag `[sim]`): spawns the simulator with `posix_spawn`
  through `tests/sim_process.hpp`, then discovery → 0x0100 → wait for SAMPLING via 0x0101 →
  receive ≥ 200 point-cloud and ≥ 10 IMU packets through `UdpSocket` / `Poller` → quit. The
  test is skipped when no Python interpreter is found; CMake passes `Python3_EXECUTABLE` as
  `LIVOX_MID360_PYTHON` and the script path as `LIVOX_MID360_SIM_SCRIPT`.

- `tests/test_scene.cpp` (tag `[scene][sim]`): a `Device` against `--scene ring` decodes
  every data type to the expected points (position, tag, `line`, per-point `offset_ns`),
  keeps exactly the expected points under a FOV window, applies a host-side extrinsic, and
  records spherical packets to lvx2 that read back as the same points. With
  `--apply-attitude` the Cartesian frames equal the scene moved by key `0x0012`, spherical
  ones stay as they are, and an `extrinsic_from()` on top gives the double transform.
- `tests/test_replay.cpp` (tag `[replay][sim]`): `--pcap tests/data/replay.pcap` into a
  `Device`, which counts exactly the capture's packets, frames, points, IMU samples and
  pushes, no drops, and reports the recorded `core_temp`.
- `tests/test_time_sync.cpp` (tag `[time][sim]`): frames start on the power-on clock; after
  a `ptp` step `kLidar` frames carry the master time and `Frame::time_type` changes between
  two frames; `time_sync_status()` and the push report the step; a loss keeps the time
  without a step back. `kHostOffsetOnce` passes synchronised time through without touching
  its offset, measures it again after the loss, and keeps its first offset while a 10 %
  drift carries the frames away from the host clock.

`SimProcess::control()` sends any control line and `wait_event()` blocks for a matching
stdout line, so later session-layer tests can inject reboots, HMS codes or dropped ACKs.

## Assumptions to verify on hardware ([#11](https://github.com/atinfinity/livox-mid360-core/issues/11))

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
| Free-running clock | counts from power-on at the host's rate (plus `drift_ppm`), restarts at 0 at a reboot | the protocol says only "counts from power-on" ([#133](https://github.com/atinfinity/livox-mid360-core/issues/133), [#109](https://github.com/atinfinity/livox-mid360-core/issues/109)) |
| Time sync acquired / lost | a step to the master time; `0x800A` = the master time, `0x800B` = local − source at the step; a loss continues from the current time, `0x800A` / `0x800B` unchanged; a reboot loses it | how the firmware steps, slews or refreshes these keys ([#109](https://github.com/atinfinity/livox-mid360-core/issues/109)) |
| `0x0202` GPS time | the request's arrival is taken as the PPS edge; answered `0x00` whatever the PPS input | the LiDAR latches the PPS edge in hardware |
| `time_filter` | stored only; the simulator has no time source to roll back | wiki describes the rollback behaviour, not testable here |
| `lidar_ipcfg` write | ret `0x21` (reboot required) when the value changes | wiki lists `0x21` but not when the firmware uses it |
| Unknown `cmd_id` | ret `0x01` | no ACK at all is also plausible |
| Multi-key config with one bad key | nothing applied | vs. partial application |
| Push contents | every read-only key `0x8000`–`0x8011` | the wiki does not enumerate the pushed keys |
| Debug raw data ([#93](https://github.com/atinfinity/livox-mid360-core/issues/93)) | `0x0303` answers `0x00` on the log and the command port, in every work state, also when repeated (the stream then moves to the new destination) or when disabling a disabled stream; short payloads and an enable with port 0 answer `0x01`; the stream leaves port 60301, does not stop the point cloud and ends with a reboot; the datagram content is synthetic | the protocol document gives the request layout only; ports come from the SDK2 source; nothing about the stream is known |
| Firmware log ([#44](https://github.com/atinfinity/livox-mid360-core/issues/44)) | `0x0301` on the log socket enables / disables a type, ret `0x00` even when repeated; chunks go to key `0x0009` (else to the `0x0301` sender); `file_index` starts at 1, `trans_index` at 1 with the begin flag, `file_num` is 1, `timestamp` is Unix seconds; a disable sends one empty end-flagged chunk | the SDK2 source shows the wire format only; the LiDAR's counting, destination choice and end-of-file behaviour are unknown |
| FOV window ranges | yaw outside [0, 360) or pitch outside (-10, 60) → `0x03`; equal / reversed start-stop accepted | the wiki gives the ranges, not the code, nor what a reversed window means |
| FOV write while SAMPLING | applied at once, ret `0x00` (no `0x21`) | the wiki does not say whether FOV keys need a reboot or a motor restart |
| `pattern_mode` | only 0 accepted; 1 / 2 → `0x20`, others → `0x03`; never restarts the motor | the wiki gives the values and the "scan mode changed" edge, not which ones the base Mid-360 accepts nor the code |
| `pcl_data_type` change while SAMPLING | the next packet is already in the new format | the wiki does not say whether the switch is immediate or aligned to a frame |
| Install attitude `0x0012` | stored only; with `--apply-attitude`, Cartesian points moved by `Rz * Ry * Rx` + translation after the FOV crop, spherical untouched | whether the firmware applies the key to its output at all, in which convention and to which data types ([#110](https://github.com/atinfinity/livox-mid360-core/issues/110)) |
| FOV cropping | yaw `[start, stop)` wrapping when `start > stop`, `start == stop` empty; pitch `[start, stop]`; keep if inside any enabled window | the wiki defines neither the edge inclusivity nor the wrap-around |
| Inquire of all settings / status keys at once | one ACK with every key | wiki gives no limit on keys per `0x0101` |
| `frame_cnt` period | 100 ms (`--frame-ms`) | the wiki marks `frame_cnt` invalid for a non-repetitive scanner. Livox's sample `.lvx2` files have `frame_counter` 0 in every package, but the LVX2 spec marks that field reserved, so they say nothing about the firmware ([#164](https://github.com/atinfinity/livox-mid360-core/issues/164), [lvx2.md](lvx2.md#livox-sample-files)) |

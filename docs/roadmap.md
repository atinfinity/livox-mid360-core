# Roadmap and status

Where `livox-mid360-core` stands, what is planned and which issues track each item.
Last updated 2026-09-26 (version 0.1.0, after the merge of the reconnection work in
PR #37). Open issues carry the labels `no-hardware` (doable against the simulator),
`needs-hardware` (requires a real Mid-360) and `phase-2` / `phase-3`.

## Phases

| Phase | Scope | Status |
| --- | --- | --- |
| 1 | Protocol, transport, session and device layers, simulator, CI | **Done** (v0.1.0) |
| 2 | Typed configuration APIs, diagnostics, samples, hardware verification | In progress |
| 3 | C ABI, ROS 2 driver (`livox-mid360-ros2`, separate repository) | Not started |

## Phase 1: done

All of these are implemented, unit-tested and exercised end to end against the Python
simulator in CI (gcc-13/14, clang-19, Release and Debug with ASan + UBSan, x86-64 and arm64).

- **Protocol layer** (`protocol.hpp`, `keys.hpp`, `hms.hpp`, `crc.hpp`): CRC-16/CCITT-FALSE
  and CRC-32 (`constexpr`, with check vectors); control command frames (24-byte header,
  seq/CRC validation, 1400-byte limit); point-cloud / IMU data packets (36-byte header, data
  types 0–3, per-point timestamp interpolation, tag decoding); key-value lists for `0x0100`
  configure / `0x0101` inquire / `0x0102` push with typed codecs for every documented key;
  HMS diagnostic-code decoding with the official description table. A Python reference
  implementation in `tools/` shares byte-exact golden vectors with the C++ tests (#1).
- **UDP transport** (`transport.hpp`): non-blocking IPv4 sockets, batched receive (`recvmmsg`
  on Linux) with kernel receive timestamps, a `poll`-based `Poller` with cross-thread wake-up
  (#2).
- **LiDAR simulator** (`tools/livox_mid360_sim.py`, stdlib-only Python): answers commands,
  runs the work-state machine, streams point-cloud / IMU / push packets and takes JSON control
  commands for fault injection (#3).
- **Session layer** (`session.hpp`): broadcast / unicast discovery, synchronous command
  round-trips with seq-matched retries and timeouts, typed configure / inquire / reboot
  helpers, work-state polling, cross-thread cancellation (#4). Host setup flow: host IP and
  ports, IMU on/off, work mode (#5).
- **Device layer** (`context.hpp`, `device.hpp`, `frame.hpp`, `event.hpp`): a `Context` owns
  the host receive sockets and one receive thread that dispatches by source IP; a `Device`
  wraps a `Session`, applies the host setup and delivers packets, assembled `Frame`s (frame
  counter or time window, `udp_cnt` drop counting, timestamp policies) and IMU samples through
  callbacks; `BoundedQueue<T>` hands them to another thread (#6, #9). Push handling: stats,
  work-state and HMS events from the `0x0102` push (#7). Automatic reconnection after a cable
  pull or reboot (`ReconnectOptions`) and several LiDARs sharing one `Context`
  (`Context::find()` by serial) (#8).
- **Quality gates**: fuzzers for the protocol, transport and session layers (#24, #28);
  clang-format / clang-tidy / ruff lint (#17); coverage on Codecov (#18); arm64 CI (#19).
- **Documentation**: [architecture.md](architecture.md) (overview), [api.md](api.md),
  [transport.md](transport.md), [session.md](session.md), [simulator.md](simulator.md),
  [protocol_notes.md](protocol_notes.md).

## Phase 2: in progress

### Typed configuration and status APIs on `Device`

- #38 firmware type and version query
- #39 FOV configuration and enable (keys `0x0015` / `0x0016` / `0x0017`)
- #40 coordinate format, scan pattern and point-cloud frame rate
- #41 read-back of stored settings and live status
- #46 detection mode (normal / sensitive, key `0x0018`)
- #47 IMU enable and IMU sensor config (rate, accelerometer range, gyroscope range)
- #50 LiDAR network config (key `0x0004`) with reboot-required handling
- #51 install attitude / extrinsics (key `0x0012`) and optional host-side transform
- #52 function IO config (key `0x0019`: PPS / GPS inputs, safety-zone outputs)
- #53 time-sync status read-back (`0x8009`–`0x800C`) and `set_gps_time` integration with
  `TimestampPolicy`
- #54 time filter (key `0x0026`)
- #55 diag status (key `0x800E`) read-back and change event
- #56 typed snapshot of the full `0x0102` push payload and optional `on_push` callback
- #57 generic typed key access (`set<Key>` / `get<Key>`, batched) over the `keys.hpp` codecs
- #58 typed `DeviceType` from the discovery ACK (needs hardware to confirm values)

### Diagnostics, tooling and samples

- #42 SDK logging: level control, console suppression, file / stderr sinks
- #44 firmware log collection (`0x03xx`, port 56500) API and sample
- #34 decoded point tag accessors (noise confidence per field)
- #35 lvx2 record / replay CLI
- #43 minimal point-cloud and IMU receive sample
- #45 simulator: verify the `work_tgt_mode` list and transitions and model them faithfully

### Hardware verification (`needs-hardware`)

Everything so far was validated against the simulator only. The assumptions to confirm are
listed in [simulator.md](simulator.md) and [protocol_notes.md](protocol_notes.md).

- #10 capture pcaps and add them as test fixtures
- #11 verify open questions (`dev_type`, reserved / `pack_info` fields, minimum firmware)
- #12 verify recovery from disconnect / reboot and multi-device operation
- #13 long-run reception test and comparison with Livox Viewer 2

## Phase 3: not started

- **C ABI**: the mapping table is in [api.md](api.md#c-abi-mapping-phase-3); all output
  structs are already plain data with `static_assert`s in `tests/test_api_skeleton.cpp`.
- **ROS 2 driver** in the separate `livox-mid360-ros2` repository.

## Out of scope

- Firmware upgrade commands (`0x04xx`).
- Mid-360S / Mid-360L specific features (v1 targets the base Mid-360).
- Platforms other than Ubuntu 24.04+ (macOS builds for development only; Jetson / JetPack 6
  is unsupported until a C++23-capable toolchain is confirmed).

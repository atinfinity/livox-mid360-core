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
| 3 | Time-sync verification, C ABI, CLI, ROS 2 driver (`livox-mid360-ros2`, separate repository) | Not started |

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

The generic `Device::set<K>()` / `get<K>()` / `set_many` / `get_many` over `key_traits<K>`
(#57, done) are the primary API; the dedicated methods below are implemented as thin wrappers
over them and only add what the raw key does not express (waits, combined keys, events).

- #38 firmware type and version query (`Device::identity()`, `lidar_info.hpp`): **done**
- #39 FOV configuration and enable (`set_fov()` / `fov()`, `HostSetup::fov`): **done**
- #40 coordinate format, scan pattern and point-cloud frame rate (`set_point_format()`,
  `set_scan_pattern()`, `set_frame_policy()`; no frame-rate key on the Mid-360): **done**
- #41 read-back of stored settings and live status (`settings()` / `status()` / `pushed_status()`): **done**
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
- #57 generic typed key access (`set<Key>` / `get<Key>`, batched) over the `keys.hpp` codecs: **done**
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

- **Time synchronisation on hardware**: PTP (IEEE 1588v2.0 over UDP) with linuxptp `ptp4l` as
  the master, gPTP (L2) and GPS (PPS + GPRMC); confirm `time_type`, the `kLidar` timestamp
  policy and timestamp monotonicity. PTP v2.1 is unsupported by the LiDAR and 1588 + gPTP
  together is discouraged by the wiki.
- **C ABI**: the mapping table is in [api.md](api.md#c-abi-mapping-phase-3); all output
  structs are already plain data with `static_assert`s in `tests/test_api_skeleton.cpp`.
- **CLI** (`livox-mid360-cli`, separate repository): discovery, configuration, lvx2 record /
  replay (#35 starts this inside `tools/`).
- **ROS 2 driver** (`livox-mid360-ros2`, separate repository): rclcpp composable node
  `livox_mid360_driver` in package `livox_mid360_ros2`, publishing output compatible with
  `livox_ros_driver2` (`PointXYZRTLT` point cloud and `CustomMsg`: x, y, z, intensity, tag,
  line, timestamp / offset_time). It uses the C++ API directly, one `Context` per node.
- **Ideas not yet scheduled**: a loader for the official `MID360_config.json` so users can
  migrate from Livox-SDK2 without rewriting their configuration.

## Project decisions

Fixed in phase 0 (2026-09-25) and not expected to change within v1.

| Topic | Decision |
| --- | --- |
| Relation to Livox-SDK2 | Clean-room implementation from the public protocol wiki only. No code is copied from SDK2; it serves solely as a behavioural reference during hardware tests. |
| Device scope | Base Mid-360 only. Mid-360S / Mid-360L keys (`speed_mode` `0x0021`, `pc_freq_mod` `0x0029`) are listed in the key enum but get no typed helpers. |
| Language standard | C++20 code built with `-std=c++23`, because `std::expected` is only enabled under C++23 in libstdc++ and libc++. Clang 18 with libstdc++ cannot use `<expected>`, so Clang 19+ is required. |
| Platform | Ubuntu 24.04 and later only. No Windows, no Ubuntu 18.04 / 20.04 / 22.04. |
| Minimum firmware | Undecided. Firmware v13.18.0244 is the baseline for hardware verification (#11). |
| Tests | Catch2 v3 from apt when available, otherwise FetchContent. |
| Repository split | Core library here; ROS 2 driver and CLI in sister repositories `livox-mid360-ros2` and `livox-mid360-cli`. |
| Lint | `.clang-format` (ROS 2 style: a verbatim copy of the `ament_clang_format` configuration, #63), `.clang-tidy` and `pyproject.toml` (ruff configured to match `ament_flake8` / `ament_pep257`, #64) are shared with `livox-mid360-ros2` through `ament_clang_format --config` (ruff has no ament equivalent; the ROS 2 side uses the `ament_flake8` / `ament_pep257` defaults, which the ruff configuration matches). Of `ament_lint_common`, only `ament_cppcheck` and `ament_lint_cmake` run here (#65); the driver's `colcon test` never lints this library. Not adopted: `ament_cpplint` (tool limitations, its real findings fixed once), `ament_copyright` (files keep the one-line SPDX header; the driver repository uses the ROS 2 header format), `ament_uncrustify` (the driver package uses `ament_cmake_clang_format` with this repository's `.clang-format`). |
| Trademark | The README states up front that the project is unofficial and unaffiliated with Livox / DJI. |

Naming:

- C++ namespace `livox::mid360`, CMake target `livox::mid360_core`, shared library
  `liblivox_mid360_core.so`, include path `<livox/mid360/...>`.
- ROS 2 package `livox_mid360_ros2`, node `livox_mid360_driver`.
- GitHub topics: `livox`, `mid-360`, `lidar`, `ros2`, `cpp20`.

## Out of scope

- Firmware upgrade commands (`0x04xx`).
- Mid-360S / Mid-360L specific features (v1 targets the base Mid-360).
- Platforms other than Ubuntu 24.04+ (macOS builds for development only; Jetson / JetPack 6
  is unsupported until a C++23-capable toolchain is confirmed).

## References

Official:

- Protocol (packet layouts, key-value list, CRC; primary source):
  <https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html>
  (rev v1.4.12, 2026-09-21, adds Mid-360L notes)
- Mid-360 index (time sync, HMS):
  <https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html>
- Time synchronisation (PTP / gPTP / GPS):
  <https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html>
- HMS diagnostic codes (key `0x8011`):
  <https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/hms_code_mid360.html>
- Coordinate system and scan pattern:
  <https://livox-wiki-en.readthedocs.io/en/latest/introduction/Point_Cloud_Characteristics_and_Coordinate_System%20.html>
- Downloads (user manual 2024-04-25, firmware v13.18.0244 with release notes 2025-04-11,
  Livox Viewer 2 for Ubuntu): <https://www.livoxtech.com/mid-360/downloads>
- Livox-SDK2 (behavioural reference only, no code copied): <https://github.com/Livox-SDK/Livox-SDK2>
- livox_ros_driver2 (the point-cloud format downstream expects):
  <https://github.com/Livox-SDK/livox_ros_driver2>

Unofficial:

- <https://github.com/Yancey2023/mid360_driver>: a lightweight SDK2-free driver, useful as a
  minimal-configuration example. Licence unchecked; no code is copied.

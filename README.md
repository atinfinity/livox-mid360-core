# livox-mid360-core

[![CI](https://github.com/atinfinity/livox-mid360-core/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/atinfinity/livox-mid360-core/actions/workflows/ci.yml)

> [!IMPORTANT]
> **Unofficial.** This project is not affiliated with, endorsed by, or supported by Livox or DJI.
> "Livox" and "Mid-360" are trademarks of their respective owners and are used here only to
> identify the device this library talks to.

A clean-room, dependency-free C++ SDK for the **Livox Mid-360** LiDAR, written from the public
protocol specification only. It targets **Ubuntu 24.04 and later** and the **base Mid-360**
(Mid-360S / Mid-360L specific features are out of scope for v1).

## Status

**v0.1 — phase 1: protocol layer.** Pure functions with no I/O:

- CRC-16/CCITT-FALSE and CRC-32, `constexpr`, with check vectors
- Control command frames (24-byte header, seq/CRC validation, 1400-byte limit): parse + build
- Point-cloud / IMU data packets (36-byte header, data types 0–3, per-point timestamp interpolation, tag decoding)
- Key-value lists for `0x0100` configure / `0x0101` inquire / `0x0102` push, typed encoders/decoders for every documented key
- HMS diagnostic-code decoding with the official description table
- A Python reference implementation (`tools/`) and byte-exact golden vectors shared by both
- UDP transport (`transport.hpp`): non-blocking IPv4 sockets, batched receive (`recvmmsg` on
  Linux) with kernel receive timestamps, and a `poll`-based `Poller` with cross-thread wake-up
- A LiDAR simulator (`tools/livox_mid360_sim.py`, stdlib-only Python) that answers commands,
  runs the work-state machine and streams point-cloud/IMU/push packets, with a JSON control
  channel for fault injection; the C++ tests spawn it for an end-to-end smoke test
- Session layer (`session.hpp`): broadcast/unicast discovery, synchronous command round-trips
  with seq-matched retries and timeouts, typed configure/inquire/reboot helpers, work-state
  polling and cross-thread cancellation. No threads; tested against the simulator

Not yet implemented (phase 2+): receive threads / Device abstraction, frame assembly,
C ABI, ROS 2 (`livox-mid360-ros2`, separate repository). Logging (`0x03xx`) and firmware
upgrade (`0x04xx`) commands are intentionally out of scope.

## Requirements

- Ubuntu 24.04+ with GCC 13/14 or Clang 19 (macOS with Apple Clang works for development).
  Clang 18 with libstdc++ does not expose `<expected>` (it reports `__cpp_concepts` 201907);
  use Clang 19+ or `-stdlib=libc++` there.
- CMake ≥ 3.28, Ninja recommended
- Compiler flag `-std=c++23`: the code is C++20 plus `std::expected`, which standard libraries
  ship under C++23 only
- Tests: Catch2 v3 (`apt install catch2`; otherwise fetched automatically with FetchContent)

## Build

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix /usr/local
```

Options: `LIVOX_MID360_BUILD_TESTS`, `LIVOX_MID360_ENABLE_ASAN`, `LIVOX_MID360_ENABLE_UBSAN`,
`LIVOX_MID360_BUILD_FUZZERS` (Clang), `LIVOX_MID360_WARNINGS_AS_ERRORS`, `BUILD_SHARED_LIBS`.

To reproduce CI locally on any Docker host: `docker/check.sh`.

## Usage

```cmake
find_package(livox_mid360_core CONFIG REQUIRED)
target_link_libraries(app PRIVATE livox::mid360_core)
```

```cpp
#include <livox/mid360/mid360.hpp>
using namespace livox::mid360;

// Build the 0x0100 request that points the LiDAR at this host.
const auto pcl = encode_host_ip_config({{192, 168, 1, 5}, kDefaultHostPointCloudPort, kPointCloudPort});
const auto en  = encode_u8(1);
const KeyValue kvs[] = {{static_cast<std::uint16_t>(Key::kPointCloudHostIpCfg), pcl},
                        {static_cast<std::uint16_t>(Key::kImuDataEn), en}};
const auto data  = encode_param_config_request(kvs);
const auto frame = build_command_frame({.seq_num = 1,
                                        .cmd_id  = static_cast<std::uint16_t>(CmdId::kParamConfig),
                                        .data    = data}).value();
// sendto(sock, frame.data(), frame.size(), ...)

// Parse a point cloud datagram.
if (auto pkt = parse_data_packet(datagram)) {
  for (std::size_t i = 0; i < pkt->header.dot_num; ++i) {
    const auto p  = decode_cartesian32(*pkt, i);
    const auto ts = sample_timestamp_ns(pkt->header, i);
    // ...
  }
}
```

Parsers return `std::expected<..., ParseError>` and **non-owning views** into the input buffer;
keep the buffer alive while you use the result.

## Layout

```
include/livox/mid360/   public headers (crc, protocol, keys, hms, bytes, transport, session, mid360 umbrella)
src/                    implementation
tests/                  Catch2 tests, generated golden vectors, libFuzzer targets
tools/                  Python reference implementation, pcap decoder, golden-vector generator, LiDAR simulator
docs/                   protocol_notes.md (wiki ambiguities), transport.md (UDP layer guide), session.md (discovery/commands), simulator.md
docker/                 Ubuntu 24.04 reproduction of CI
```

## References

- Protocol: <https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html> (rev v1.4.12)
- HMS codes: <https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/hms_code_mid360.html>
- Downloads (manual, firmware, Livox Viewer 2): <https://www.livoxtech.com/mid-360/downloads>

The official Livox-SDK2 is used only as a behavioural reference for hardware testing. No code is
copied from it.

## License

Apache-2.0. See `LICENSE`.

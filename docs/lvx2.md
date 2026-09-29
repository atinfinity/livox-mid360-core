# lvx2 record / replay

Issue [#35](https://github.com/atinfinity/livox-mid360-core/issues/35) adds an lvx2 codec to the library (`include/livox/mid360/lvx2.hpp`) and a small
CLI (`livox-mid360-cli`, `tools/cli/`) that records the raw point-cloud packet stream of one
Mid-360 to a `.lvx2` file and replays such a file through the same frame assembly the
`Device` uses. Design decisions are recorded on issue [#35](https://github.com/atinfinity/livox-mid360-core/issues/35); the items that still need a real
Livox Viewer 2 are listed on [#108](https://github.com/atinfinity/livox-mid360-core/issues/108).

## File layout

Reference: "LVX2 Specifications" (Livox, 2023.06 v1.0). All integers are little-endian,
structures are packed. Livox-SDK2 and livox_ros_driver2 contain no lvx2 code; the layout
below is taken from the specification only.

| Block | Size | Fields |
|---|---|---|
| Public header | 24 | `char[16]` signature `"livox_tech"` + zeros, `u8[4]` version `{2,0,0,0}`, `u32` magic `0xAC0EA767` |
| Private header | 5 | `u32` frame_duration (ms, 50), `u8` device_count |
| Device info × N | 63 each | `char[16]` lidar_sn, `char[16]` hub_sn, `u32` lidar_id, `u8` lidar_type (reserved), `u8` device_type (9 = Mid-360, 10 = HAP), `u8` extrinsic_enable, `f32` roll, pitch, yaw (deg), `f32` x, y, z (m) |
| Frame header | 24 | `u64` current_offset, `u64` next_offset, `u64` frame_index; packages follow back-to-back until `next_offset` |
| Package header | 27 | `u8` version (0), `u32` lidar_id, `u8` lidar_type, `u8` timestamp_type, `u64` timestamp (ns), `u16` udp_counter, `u8` data_type, `u32` length (point bytes), `u8` frame_counter, `u8[4]` reserved |
| Points | `length` | identical to the UDP payload: type 1 = 14 B/pt (`i32` x, y, z mm, `u8` reflectivity, `u8` tag), type 2 = 8 B/pt (`i16` x, y, z cm, `u8`, `u8`) |

Only data types 1 and 2 are valid in a file: there is no IMU package and no spherical
package.

## What the writer puts in

| Field | Value |
|---|---|
| `lidar_sn` | serial number from discovery |
| `hub_sn` | zeros (no hub) |
| `lidar_id` | SDK2 "handle": the four IP bytes as one `u32` in network order (`192.168.1.10` → `0x0A01A8C0`) |
| `lidar_type` | 0 |
| `device_type` | 9 |
| extrinsic | key `0x0012` (install attitude) when readable, mm converted to m, `extrinsic_enable` = 1; otherwise zeros and 0 |
| frames | cut on the packet `timestamp` in absolute 50 ms bins (`timestamp / 50 ms`); a bin change flushes the frame |
| package `version` | 0 |
| `timestamp_type` | the packet's `time_type` |
| `frame_counter` | the packet's `frame_cnt` |
| spherical packets (type 3) | converted to type 1 (mm, rounded) |
| IMU packets | not written; `Lvx2Writer::write` returns `false` and `stats().ignored` counts them |

The reader validates the signature, magic and major version (2) and the frame chain
(`current_offset` must equal the file position, `next_offset` must be past the header). It
does not validate `device_type`, `lidar_type`, `frame_duration` or the extrinsic. A file cut
mid-frame (e.g. a recording that was killed) yields every complete package and then sets
`truncated()`.

## Unverified against Livox Viewer 2

The simulator is the only device this has been recorded from. Livox's sample files (next
section) answer some of the questions about files written by Viewer 2, but none about what
Viewer 2 accepts. Tracked on [#108](https://github.com/atinfinity/livox-mid360-core/issues/108):

- whether Viewer 2 requires `lidar_id` to be the SDK2 handle (IP as `u32`) or accepts any
  value. The sample files do use the IP;
- meaning of `lidar_type` (the spec calls it reserved). The sample files have 247 in the
  device info and 8 in every package header, where the writer puts 0;
- whether Viewer 2 expects frames binned on absolute 50 ms boundaries. Its own files are not:
  a frame starts about every 50 ms from the recording's start, and almost every frame spans two
  absolute bins;
- whether files without IMU packages open. The sample files have none either;
- extrinsic units (deg / m) and the sign of `extrinsic_enable`. The sample values do not settle
  the units (see below);
- opening and replaying a file recorded by `livox-mid360-cli record` in Viewer 2. The other
  direction works: `livox-mid360-cli replay` reads the sample files.

## Livox sample files

The [Mid-360 downloads page](https://www.livoxtech.com/mid-360/downloads) has two recordings,
"Point Cloud Data - Indoor" and "- Outdoor". Both replay with `livox-mid360-cli replay`
([#164](https://github.com/atinfinity/livox-mid360-core/issues/164)):

| File | Size | Devices | Length | Replay with `--frame-mode window` |
|---|---|---|---|---|
| `Indoor_sampledata.lvx2` | 223 MB | 1 | 78 s | 777 frames of 20 064 points (the last one partial), 0 dropped |
| `Outdoor_sampledata.lvx2` | 597 MB | 3 | 70 s | per `--lidar-id`: 695 frames of 20 064 points (the last one partial), 0 dropped |

```sh
livox-mid360-cli replay Indoor_sampledata.lvx2 --frame-mode window
livox-mid360-cli replay Outdoor_sampledata.lvx2 --frame-mode window --lidar-id 738306240
```

What they contain:

| Field | Value |
|---|---|
| `frame_duration` | 50 |
| `lidar_sn` | `47MDK9DF710030` (both files), `47MDK9DF710195`, `47MDK9DF710124` |
| `hub_sn` | not zeros: 16 bytes that look uninitialised, different per file |
| `lidar_id` | the IP as `u32`: 738306240 = 192.168.1.44, 3271665856 = 192.168.1.195, 2080483520 = 192.168.1.124 |
| `lidar_type` | 247 in the device info, 8 in every package header |
| `device_type` | 9 |
| extrinsic | `extrinsic_enable` 1. Indoor: all zeros. Outdoor: roll, pitch, yaw, x, y, z = `(-0.91, 0.41, 0.24, 0, 0, 0.8)`, `(0.84, -181.23, -92.12, -36.4, 27.9, 80.1)` and `(-0.42, -178.55, 88.33, -37.7, -30.4, 82.1)`; the translations differ by a factor of about 100 between the first device and the other two |
| frames | 50 ms from the recording's start, not absolute bins; in Outdoor one frame holds the packages of all three devices |
| package `timestamp_type` | 0 (no synchronisation); in Outdoor one LiDAR's clock is 2.66 s ahead of the other two |
| package `data_type` | 1, 96 points (1344 bytes) per package, no `udp_counter` gaps |
| package `frame_counter` | 0 in every package |

Two consequences for a replay:

- `frame_counter` never changes, so the default `--frame-mode counter` falls back to the time
  window after `2 × window` (see "Frame counter mode" in [api.md](api.md)). The first frame
  then covers 200 ms (40 032 points, 776 frames in Indoor). With `--frame-mode window` every
  frame covers 100 ms. Whether the firmware sends `frame_cnt` 0 or Viewer 2 writes 0 is
  open ([#11](https://github.com/atinfinity/livox-mid360-core/issues/11)).
- The Outdoor file holds three LiDARs, so it plays one at a time with `--lidar-id`.

## Library API

```cpp
#include "livox/mid360/lvx2.hpp"

Lvx2DeviceInfo info{.lidar_sn = sn, .lidar_id = id};
Lvx2Writer writer;
writer.open("capture.lvx2", std::span(&info, 1));
device.on_packet([&](const DataPacketView & p, const ReceiveInfo &) { (void)writer.write(0, p); });
// ... later
writer.close();

Lvx2Reader reader;
reader.open("capture.lvx2");
while (auto p = reader.next_packet(); p && *p) {      // nullopt = end of file
  DataPacketView view = (*p)->to_data_packet_view();  // dot_num from the length, no CRC
}

Lvx2Player player({.frame_policy = {}, .rate = 1.0, .loop = false});
player.open("capture.lvx2");
player.on_frame([](Frame && f) { /* same Frame as Device::on_frame */ });
// Lvx2PlayStats{packets, frames, points, dropped_packets, loops}
auto stats = player.run(stop_token);
```

- `Lvx2Player` feeds the recorded timestamps as they are (`TimestampPolicy::kLidar`); frames
  are closed by `Lvx2PlayOptions::frame_policy`, not by the file's 50 ms frames.
- `rate` is relative to the recorded timestamps (1.0 = real time, 0 = as fast as possible). A
  backward timestamp jump does not wait.
- With `loop`, the file is reopened and the frame assembler reset after each pass; the
  partial frame is flushed at the end of every pass. `Frame::index` is not reset: it starts at
  0 for each `run()` and keeps counting across passes
  ([#149](https://github.com/atinfinity/livox-mid360-core/issues/149)).
- `lidar_id` filters a multi-device file to one LiDAR.
- Errors are `std::expected<_, Lvx2Error>` with `kIo` (+ `errno_value`), `kInvalidArgument`,
  `kUnsupportedDataType` and `kBadFile` (+ `detail`).

## CLI

Built with `LIVOX_MID360_BUILD_TOOLS` (on by default when the project is top level) and
installed to `bin/`.

```sh
livox-mid360-cli record --out capture.lvx2 --lidar-ip 192.168.1.10 --host-ip 192.168.1.5 --duration 30
livox-mid360-cli replay capture.lvx2 [--rate 1.0] [--loop] [--frame-mode counter|window] [--window-ms 100] [--lidar-id N] [--quiet]
livox-mid360-cli --version
```

`record` discovers the LiDAR (`--sn` picks one when several answer), opens it, reads the
install attitude, calls `start_sampling` and writes every packet from `on_packet`. It
refuses to overwrite an existing file unless `--force` is given, prints
`packets=N frames=M bytes=B` once per second on stderr and stops after `--duration` seconds
or on SIGINT. Exit codes: 0 ok, 1 usage, 2 setup / I/O failure, 3 no packet recorded.

`replay` prints one line per frame (`frame N points=P t=... dropped=D`) on stdout, unless
`--quiet`, and a summary line (`packets= frames= points= dropped= loops=`) at the end.
SIGINT ends a `--loop` run.

Against the simulator:

```sh
python3 tools/livox_mid360_sim.py --bind 127.0.0.1 &
build/tools/cli/livox-mid360-cli record --out /tmp/sim.lvx2 --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --duration 3
build/tools/cli/livox-mid360-cli replay /tmp/sim.lvx2 --rate 0
```

The ctest `cli_record_replay` does exactly that (2 s) and checks that the replayed packet
count equals the recorded one and the frame count matches the file's frames within two.

## Tests

- `tests/test_lvx2.cpp`: writer → reader round trip (types 1 and 2, spherical conversion,
  50 ms splitting, IMU ignored), the committed fixture `tests/data/mini.lvx2` (written by an
  independent Python `struct.pack` script, not by `Lvx2Writer`), truncated and corrupt files,
  and the player (frames, `lidar_id` filter, loop + stop token, pacing).
- `cli_record_replay`: the golden run against the simulator described above.

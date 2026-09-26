# lvx2 record / replay

Issue #35 adds an lvx2 codec to the library (`include/livox/mid360/lvx2.hpp`) and a small
CLI (`livox-mid360-cli`, `tools/cli/`) that records the raw point-cloud packet stream of one
Mid-360 to a `.lvx2` file and replays such a file through the same frame assembly the
`Device` uses. Design decisions are recorded on issue #35; the items that still need a real
Livox Viewer 2 are listed on #12.

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

The simulator is the only "device" this has been run against. Tracked on #12:

- whether Viewer 2 requires `lidar_id` to be the SDK2 handle (IP as `u32`) or accepts any value;
- meaning of `lidar_type` (the spec calls it reserved);
- whether Viewer 2 expects frames binned on absolute 50 ms boundaries or from the first packet;
- whether files without IMU packages open and replay;
- extrinsic units (deg / m) and the sign of `extrinsic_enable`;
- opening and replaying a file recorded by `livox-mid360-cli record` in Viewer 2, and reading
  a file recorded by Viewer 2 with `livox-mid360-cli replay`.

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
while (auto p = reader.next_packet(); p && *p) {   // nullopt = end of file
  DataPacketView view = (*p)->to_data_packet_view();  // dot_num from the length, no CRC
}

Lvx2Player player({.frame_policy = {}, .rate = 1.0, .loop = false});
player.open("capture.lvx2");
player.on_frame([](Frame && f) { /* same Frame as Device::on_frame */ });
auto stats = player.run(stop_token);  // Lvx2PlayStats{packets, frames, points, dropped_packets, loops}
```

- `Lvx2Player` feeds the recorded timestamps as they are (`TimestampPolicy::kLidar`); frames
  are closed by `Lvx2PlayOptions::frame_policy`, not by the file's 50 ms frames.
- `rate` is relative to the recorded timestamps (1.0 = real time, 0 = as fast as possible). A
  backward timestamp jump does not wait.
- With `loop`, the file is reopened and the frame assembler reset after each pass; the
  partial frame is flushed at the end of every pass.
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
count equals the recorded one and the frame count matches the file's frames within one.

## Tests

- `tests/test_lvx2.cpp`: writer → reader round trip (types 1 and 2, spherical conversion,
  50 ms splitting, IMU ignored), the committed fixture `tests/data/mini.lvx2` (written by an
  independent Python `struct.pack` script, not by `Lvx2Writer`), truncated and corrupt files,
  and the player (frames, `lidar_id` filter, loop + stop token, pacing).
- `cli_record_replay`: the golden run against the simulator described above.

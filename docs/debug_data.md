# Debug raw data collection

Livox support may ask for the "debug raw data" of a LiDAR ([#93](https://github.com/atinfinity/livox-mid360-core/issues/93)). Command `0x0303` switches
the stream on; the datagrams are opaque to this SDK. The library part (`debug_data.hpp`,
`ContextOptions::debug_data_port`, `Device::start_debug_data()` / `on_debug_data()`) is
described in [api.md](api.md#debug-raw-data), the wire format in
[protocol_notes.md](protocol_notes.md). This page covers the CLI and its file.

The hardware results so far and the open points are tracked on
[#106](https://github.com/atinfinity/livox-mid360-core/issues/106).

## CLI

```sh
livox-mid360-cli debug-data --out FILE [--lidar-ip A.B.C.D] [--host-ip A.B.C.D] [--sn SN]
                            [--duration SECONDS] [--port N] [--start-sampling] [--max-size BYTES]
                            [--format raw|sdk2]
```

| Option | Meaning |
| --- | --- |
| `--out FILE` | Output file, overwritten if it exists. |
| `--lidar-ip` | Unicast discovery target; without it the broadcast discovery is used. |
| `--host-ip` | Host address to bind and to name in the request. |
| `--sn` | Pick the LiDAR with this serial number. |
| `--duration` | Seconds to collect. Without it the run lasts until Ctrl-C. |
| `--port` | Host UDP port of the stream, default 44332 (the port Livox-SDK2 uses). |
| `--start-sampling` | Put the LiDAR into SAMPLING first and back to IDLE at the end. Without it the work state is left alone; the Mid-360 sends the stream only while sampling ([#225](https://github.com/atinfinity/livox-mid360-core/issues/225)), so an IDLE LiDAR gives exit code 3. |
| `--max-size` | File size limit in bytes, default 4294967296 (4 GiB). Reaching it ends the run with exit code 0. A datagram that would cross the limit is not written. |
| `--format` | `raw` (default): the format below, with receive time and source port per datagram. `sdk2`: the `.LivoxDebugPointCloudData` file of Livox-SDK2, see [SDK2 file format](#sdk2-file-format). |

The stop request (`0x0303` with `enable = 0`) is sent on every exit path after the device was
opened, including Ctrl-C and a failed start. Progress goes to stderr, the last line is
`wrote FILE: packets=N bytes=M`. Exit codes: 0 ok, 1 usage, 2 setup or I/O failure, 3 no
datagram received.

## File format (provisional, version 0)

Private to the CLI and subject to change. It keeps what the SDK2 file drops (receive time,
source port, datagram boundaries); `--to-sdk2` of the reader converts it. All integers are
little-endian.

File header, 36 bytes:

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 8 | magic `LMDBGRAW` |
| 8 | 4 | version, `0` |
| 12 | 16 | serial number, ASCII, NUL padded |
| 28 | 8 | start time, ns since the Unix epoch (host clock) |

Then one record per datagram, in the order of reception:

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 8 | receive time, ns since the Unix epoch (`DebugDataPacket::host_receive_time_ns`) |
| 8 | 2 | UDP source port |
| 10 | 4 | payload length `n` |
| 14 | `n` | payload, the datagram as received |

A file that was cut (killed process, full disk) ends inside a record; readers keep the
complete records before it.

## SDK2 file format

What Livox-SDK2 writes when its debug point cloud is switched on
([#107](https://github.com/atinfinity/livox-mid360-core/issues/107)), read from the SDK2
source at commit `c0796f0` (`sdk_core/comm/define.h`,
`sdk_core/debug_point_cloud_handler/debug_point_cloud_handler.cpp`). SDK2 names the file
`lidar_<handle>_<YYYY_MM_DD_HH_MM_SS>.LivoxDebugPointCloudData`, where the handle is the
LiDAR IPv4 address as `in_addr.s_addr`. All integers are little-endian.

Header, 128 bytes (packed):

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 1 | `file_ver`, `1` |
| 1 | 1 | `dev_type` from the discovery answer, `9` for a Mid-360 |
| 2 | 1 | `data_type`, `1` |
| 3 | 16 | serial number, ASCII, NUL padded |
| 19 | 107 | reserved, zero |
| 126 | 2 | `crc16`: CRC-16/CCITT-FALSE (FastCRC16 `ccitt`) of bytes 0..125 |

Then the UDP payloads of the stream, concatenated in the order of reception with nothing
between them: no length, time or port. SDK2 keeps only datagrams from LiDAR port 60301 and
stops writing at 4 GiB. Without framing, datagrams can only be told apart when their size is
known (`--datagram-size` of the reader).

The header is checked against one built by SDK2's own struct and CRC code (the golden headers
in `tools/test_debug_data.py`).

**Compared with a real file.** SDK2 at `c0796f0` (its `samples/debug_point_cloud`) wrote a
file from a Mid-360 with firmware 13.18.0244, and `livox-mid360-cli debug-data` recorded the
same LiDAR right after it, once with `--format sdk2` and once with `--format raw`:

- The 128-byte header of the CLI's `sdk2` file, and of the raw file converted with
  `--to-sdk2`, is byte for byte the header of the SDK2 file. The 14-character serial number
  is followed by two NUL bytes.
- In all three files the body is a whole number of 1114-byte datagrams, each starting with
  `0xa5`. The raw file shows that every datagram came from port 60301 and was 1114 bytes
  long, so `--datagram-size 1114` splits a Mid-360 file.

The reference file holds the LiDAR's serial number and is not in the repository; the tests
use a header of the same layout with a made-up one.

## Reader

```sh
python3 tools/livox_mid360_debug_data.py FILE                 # summary as one JSON line
python3 tools/livox_mid360_debug_data.py FILE --json          # plus one line per datagram
python3 tools/livox_mid360_debug_data.py FILE --payload OUT   # concatenated payloads
python3 tools/livox_mid360_debug_data.py FILE --check-sim     # simulator counter and pattern
python3 tools/livox_mid360_debug_data.py FILE --to-sdk2 OUT   # raw file -> SDK2 file
```

The reader takes both formats; it tells them apart by the magic or, failing that, a valid
SDK2 header CRC. An SDK2 file has no datagram count (`packets` is `null`) unless
`--datagram-size N` splits it; `--json` and `--check-sim` need that option on an SDK2 file.
`--to-sdk2` writes the header with `--dev-type` (default 9). Exit codes: 0 ok, 1 usage,
2 bad or cut file, 3 no datagram.

## Against the simulator

```sh
python3 tools/livox_mid360_sim.py --bind 127.0.0.1 --no-quit-on-eof --debug-data-interval 0.02 &
build/tools/cli/livox-mid360-cli debug-data --out /tmp/debug.bin --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --duration 3
python3 tools/livox_mid360_debug_data.py /tmp/debug.bin --check-sim
```

The ctest `cli_debug_data` runs this sequence, then the same with `--format sdk2` and a
`--to-sdk2` conversion, and compares the two. `tools/test_debug_data.py` covers the reader.

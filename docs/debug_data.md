# Debug raw data collection

Livox support may ask for the "debug raw data" of a LiDAR ([#93](https://github.com/atinfinity/livox-mid360-core/issues/93)). Command `0x0303` switches
the stream on; the datagrams are opaque to this SDK. The library part (`debug_data.hpp`,
`ContextOptions::debug_data_port`, `Device::start_debug_data()` / `on_debug_data()`) is
described in [api.md](api.md#debug-raw-data), the wire format in
[protocol_notes.md](protocol_notes.md). This page covers the CLI and its file.

Nothing here has been run against hardware; the open points are tracked on
[#106](https://github.com/atinfinity/livox-mid360-core/issues/106).

## CLI

```sh
livox-mid360-cli debug-data --out FILE [--lidar-ip A.B.C.D] [--host-ip A.B.C.D] [--sn SN]
                            [--duration SECONDS] [--port N] [--start-sampling] [--max-size BYTES]
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
| `--max-size` | File size limit in bytes, default 4294967296 (4 GiB). Reaching it ends the run with exit code 0. |

The stop request (`0x0303` with `enable = 0`) is sent on every exit path after the device was
opened, including Ctrl-C and a failed start. Progress goes to stderr, the last line is
`wrote FILE: packets=N bytes=M`. Exit codes: 0 ok, 1 usage, 2 setup or I/O failure, 3 no
datagram received.

## File format (provisional, version 0)

Private to the CLI and subject to change; the file that Livox-SDK2 writes
(`.LivoxDebugPointCloudData`) is tracked on [#107](https://github.com/atinfinity/livox-mid360-core/issues/107). All integers are little-endian.

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

## Reader

```sh
python3 tools/livox_mid360_debug_data.py FILE                 # summary as one JSON line
python3 tools/livox_mid360_debug_data.py FILE --json          # plus one line per datagram
python3 tools/livox_mid360_debug_data.py FILE --payload OUT   # concatenated payloads
python3 tools/livox_mid360_debug_data.py FILE --check-sim     # simulator counter and pattern
```

Exit codes: 0 ok, 2 bad or cut file, 3 no datagram.

## Against the simulator

```sh
python3 tools/livox_mid360_sim.py --bind 127.0.0.1 --no-quit-on-eof --debug-data-interval 0.02 &
build/tools/cli/livox-mid360-cli debug-data --out /tmp/debug.bin --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --duration 3
python3 tools/livox_mid360_debug_data.py /tmp/debug.bin --check-sim
```

The ctest `cli_debug_data` runs this sequence.

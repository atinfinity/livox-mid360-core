# examples/

`minimal_receive.cpp` is the happy path end to end: `Context::create` → `discover()` →
`Device::open` → `on_frame` / `on_imu` / `on_event` → `start_sampling()`, one line per frame,
one IMU line per `--imu-every` samples (default 200 = once a second), events on stderr, and a
clean shutdown on Ctrl-C (`stop_sampling()`, Device destroyed before the Context). Built with
`LIVOX_MID360_BUILD_EXAMPLES` (on when this project is the top level).

| Flag | Meaning |
|---|---|
| `--lidar-ip A.B.C.D` | unicast discovery to this LiDAR; omitted = broadcast, first answer wins |
| `--host-ip A.B.C.D` | interface to bind the receive sockets and the command socket to (default any) |
| `--push-port` / `--point-port` / `--imu-port` | host ports (defaults 56201 / 56301 / 56401) |
| `--seconds N` | stop after N seconds; 0 = until SIGINT |
| `--imu-every N` | print every Nth IMU sample |

Exit codes: 0 ok, 1 usage, 2 setup failure (context, discovery, open, start_sampling; the
reason is on stderr), 3 no frame received (check `--host-ip` and the firewall).

## Real hardware

```sh
cmake -S . -B build -G Ninja && cmake --build build
build/examples/minimal_receive --lidar-ip 192.168.1.10 --host-ip 192.168.1.5
```

## Simulator

```sh
python3 tools/livox_mid360_sim.py --bind 127.0.0.1 &
build/examples/minimal_receive --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --seconds 5
```

`run_against_sim.sh` does the same for 2 s and is registered as the ctest
`example_minimal_receive` so the sample cannot rot.

## collect_firmware_log

`collect_firmware_log.cpp` collects the LiDAR's own firmware log (#44): `Device::open` →
`on_firmware_log` → `start_firmware_log`, one output file per firmware log file, a progress
line per second on stdout (`file 1: N bytes, M chunks, G gaps`), events (including
`firmware_log_gap`) on stderr, `stop_firmware_log` and a summary on exit.

| Flag | Meaning |
|---|---|
| `--lidar-ip` / `--host-ip` | as above |
| `--out DIR` | output directory (created; default `.`) |
| `--duration N` | stop after N seconds; 0 = until SIGINT |
| `--log-port N` | host port for the log stream (default 56501) |
| `--type realtime\|exception` | log type to collect (default realtime) |
| `--start-sampling` | also start sampling, so the log reflects a working LiDAR |

Files are named `<SN>_<UTC start, e.g. 20260927T101500Z>_<type>_<file_index>.log`; a new one
is opened on every packet flagged "file begin". Exit codes: 0 ok, 1 usage, 2 setup failure,
3 no chunk received (the empty file is kept).

```sh
build/examples/collect_firmware_log --lidar-ip 192.168.1.10 --host-ip 192.168.1.5 --out logs --duration 30
# simulator:
python3 tools/livox_mid360_sim.py --bind 127.0.0.1 &
build/examples/collect_firmware_log --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --out /tmp/fwlog --duration 3
```

The ctest `example_collect_firmware_log` runs it for 2 s against the simulator.

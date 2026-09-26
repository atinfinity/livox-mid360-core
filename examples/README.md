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

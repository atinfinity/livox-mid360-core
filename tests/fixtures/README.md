# Mid-360 capture fixtures

Small classic pcap files (Ethernet / IPv4 / UDP, microsecond timestamps) of the scenarios in
[#10](https://github.com/atinfinity/livox-mid360-core/issues/10). `tests/test_fixtures.cpp`
(tag `[fixtures]`) parses every datagram with the library and checks the fields listed
below.

**The captures are synthetic.** `tools/gen_fixtures.py` writes them, and no datagram comes
from a device. What they reproduce was measured on a Mid-360 with firmware 13.18.0244
(`version_app` 13.18.2.44) on 2026-10-05: packet sizes, header fields, rates, key sets, ACK
shapes, the reboot timeline and the firmware log flags. The real captures are kept outside
the repository. The point coordinates (the simulator's ring scene), the firmware log and
debug data bytes, the serial number (`FIXTURE0000001`), the MAC addresses and the IP
addresses (LiDAR `192.168.1.12`, host `192.168.1.50`) are made up.

Regenerate after changing the generator; CI regenerates the files and fails on a difference:

```sh
python3 tools/gen_fixtures.py
python3 tools/livox_mid360_pcap.py tests/fixtures/<name>.pcap --json   # decode one
```

| File | Scenario | What it shows (as measured) |
|---|---|---|
| `discovery_setup.pcap` | Broadcast `0x0000`, host setup with `0x0100`, two pushes | The 24-byte discovery ACK goes to `255.255.255.255`; `dev_type` 9; a push comes within 20 ms of a `0x0100`. The push is 401 bytes and carries 30 keys: `FW_TYPE` 1, `hms` slot 0 `0x04070002`, FOV windows yaw 0..0 / pitch -7..52. |
| `inquire.pcap` | `0x0101` of every settings key, every status key, then `0x002B` alone | Settings: ret `0x20`, 14 keys answered, and `0x0021` / `0x0026` / `0x0029` / `0x002B` left out. Status: ret 0, 16 keys. A lone unsupported key gets a 3-byte ACK (ret `0x20`, `key_num` 0). |
| `pcl_types.pcap` | `pcl_data_type` 1, 2 and 3, 30 packets each | 1380 / 804 / 996 bytes; 96 points; `time_interval` 4750; a timestamp step of 480 µs; `frame_cnt` 0; `crc32` 0; reserved bytes 0. `udp_cnt` continues across the switches and wraps at 65535. Points without a return are (0, 0, 0) with reflectivity 0. |
| `imu.pcap` | IMU at 200 Hz between point cloud packets; a `0x002B` write | 60 bytes; `time_interval` 0; `crc32` set. The `imu_sensor_cfg` write is rejected with ret `0x20`, key `0x002B`. The real step jitters between 4.2 and 5.8 ms; here it is an even 5 ms. |
| `fov.pcap` | FOV off, window pitch 0..59, then an empty window | Cropped points are not dropped: they stay in the 96-point packet as (0, 0, 0) with reflectivity 60 and tag 0. An empty window crops every point. |
| `firmware_log.pcap` | `0x0301` on, 14 chunks of log file 7, `0x0301` off | Chunk 0 has flags `0x03` (begin, ACK requested) and 256 bytes. Chunks 1–8 have flags `0x01` and 660–820 bytes; later chunks have flags 0 and 546–632 bytes. There is no end chunk when the log is turned off. Host ACKs are left out. |
| `debug_data.pcap` | `0x0303` on, 20 datagrams, `0x0303` off | The ACK (1 byte) comes from the log port 56500. The data goes from port 60301 to host port 44332 in 1114-byte datagrams. |
| `reboot.pcap` | `0x0200` while sampling, until sampling again | The ACK is 1 byte. Data and pushes go on for 1.25 s, then two ERROR pushes (+1.25 s, +1.32 s), then silence until +8.9 s. MOTORSTARTUP pushes come every second, READY at +13.54 s and SAMPLING 20 ms later. The push `seq_num` restarts, and data resumes with `udp_cnt` 0 and a timestamp from the new power-on. The data is trimmed to 10 packets around each edge. |

Not covered: power-on (SELFCHECK), factory reset (`0x0201`) and the other IMU output rates
(the firmware rejects `imu_sensor_cfg`). The differences from the simulator are tracked on
[#11](https://github.com/atinfinity/livox-mid360-core/issues/11).

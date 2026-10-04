# Hardware verification log

Everything in this repository has been developed against the bundled simulator. This page
records the runs on a real Mid-360: how to do one, and what each run found. The plan and the
open questions are on [#11](https://github.com/atinfinity/livox-mid360-core/issues/11); the
first run is [#110](https://github.com/atinfinity/livox-mid360-core/issues/110).

## Before the run

**Host.** Build Release with the compiler you want to verify ([getting_started.md](getting_started.md#2-build-and-test)).
Install `tcpdump` for the capture and `editcap` (`wireshark-common`) to trim it
([#10](https://github.com/atinfinity/livox-mid360-core/issues/10)).

**Network.** The factory setup expects the host at `192.168.1.50/24`; the LiDAR is
`192.168.1.1xx`, where `xx` are the last two digits of its serial number. Give the wired
interface that address ([getting_started.md](getting_started.md#6-connect-a-real-mid-360)).
If another interface of the host, such as Wi-Fi on a home router, is also in
`192.168.1.0/24`, the host has two routes to the subnet and may send to the LiDAR through the
wrong one, and the LiDAR's address may belong to another device on that network. Either
disconnect the other interface for the run, or keep it (for example to stay online during
the run) and route only the LiDAR's address through the wired interface.

**Keeping an overlapping Wi-Fi.** A `/32` route is more specific than the Wi-Fi's
`192.168.1.0/24`, so it wins for that one address. With `enp2s0` as the wired interface and
`192.168.1.1xx` as the LiDAR, create a NetworkManager profile for the wired interface:

```sh
nmcli con add type ethernet ifname enp2s0 con-name mid360 connection.autoconnect-priority 10 \
  ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.never-default yes \
  ipv4.route-metric 1000 ipv4.routes "192.168.1.1xx/32 src=192.168.1.50" ipv6.method disabled
```

With the LiDAR plugged in and the profile active (`nmcli -f DEVICE,STATE,CONNECTION dev`):

```sh
ip route get 192.168.1.1xx    # must print "dev enp2s0 src 192.168.1.50"
ip route get 192.168.1.1      # must still print the Wi-Fi interface
```

- Use the profile rather than `ip addr add`. The interface usually already has a DHCP profile
  ("Wired connection 1" or similar), which NetworkManager activates when the LiDAR's link
  comes up and which can remove a manually added address. `connection.autoconnect-priority 10`
  makes NetworkManager pick `mid360` over it.
- Keep `ipv4.route-metric 1000`. The address adds a `192.168.1.0/24` route on `enp2s0`; with
  NetworkManager's default metric for Ethernet (100) it wins over the Wi-Fi's (600), and the
  rest of the LAN, the router included, becomes unreachable, while the internet may keep
  working through the default route and hide the problem. With `ip addr add`, the same
  happens at metric 0 unless `noprefixroute` is given.
- `--host-ip 192.168.1.50` is enough. Broadcast discovery (`255.255.255.255`) is sent from a
  socket bound to that address and leaves through `enp2s0`, so `--lidar-ip` is not needed.
  Without `--host-ip` the broadcast leaves through the Wi-Fi and finds nothing.
- Before plugging in the LiDAR, `ping 192.168.1.1xx` and `ping 192.168.1.50`. An answer means
  that a device on the Wi-Fi network uses that address. The LiDAR stays reachable through
  the `/32` route, but that device does not, and a device on `192.168.1.50` conflicts with
  the host's wired address.
- Undo after the run: `nmcli con delete mid360`. Until then the profile also comes back after
  a reboot.

**Firewall.** With `ufw` enabled, the run shows which ports have to be opened
(the table in [getting_started.md](getting_started.md#6-connect-a-real-mid-360)). Run once
with the firewall disabled first, so that a failure is not mistaken for a library bug.

## Running the checks

`scripts/hw-first-run.sh` runs the #110 checks in order and keeps going after a failure:

| Step | Passes when |
| --- | --- |
| `livox-mid360-cli info` | Discovery finds the LiDAR and it answers the identity inquire (firmware version, serial number) |
| `minimal_receive` | Discovery finds the LiDAR (by broadcast unless `--lidar-ip` is given), frames arrive and `bad=0` |
| `collect_firmware_log` | At least one firmware log file is written and not empty |
| `livox-mid360-cli record` / `replay` | 10 s are recorded and the replay reads the same number of packets |
| `livox-mid360-cli debug-data` | At least one debug raw data packet is written |
| `tools/livox_mid360_probe.py` | The LiDAR answers the discovery; the answers to the #11 questions are in `probe.json` |

```sh
scripts/hw-first-run.sh --host-ip 192.168.1.50 --pcap enp2s0
# discovery by unicast instead of broadcast:
scripts/hw-first-run.sh --host-ip 192.168.1.50 --lidar-ip 192.168.1.1xx --pcap enp2s0
```

`--pcap` captures the exchange with `tcpdump` (it asks for `sudo` once). Run by hand,
`tcpdump` needs no `-Z root`: on Ubuntu its AppArmor profile denies that and it crashes.
The capture decodes with `tools/livox_mid360_pcap.py` and replays through the simulator's
`--pcap` ([simulator.md](simulator.md)). The output directory,
`hw-run-<UTC time>/` by default, holds every log, the `.lvx2` and debug data files, the
capture, `env.txt` (OS, compiler, commit, addresses, the LiDAR's identity), `probe.json` and `summary.md`, a table to paste into
the results below or into the issue. `--seconds` sets the duration of the first two steps
(default 60, as #110 asks).

To rehearse without a LiDAR, `--sim` runs the same steps against the simulator on
`127.0.0.1`.

The firmware version (`version_app`) and the `dev_type` from discovery, which #11 wants to
confirm, come from the `info` step and are copied into `summary.md`.

The probe step sends the requests that answer the protocol questions of #11: inquire of the
whole settings and status key sets and of every known key, an unknown key and an unknown
command id, writes to a read-only key, an unknown key, with a wrong length and out of range,
a two-key write with one bad key, and a 5 s window of the `0x0102` push. A write the LiDAR
accepts is undone with the value read before it, and the push destination (key `0x0005`) is
restored after the window; it never reboots the LiDAR or writes its IP configuration, scan
pattern, work mode or data type. `probe.json` holds every request and ACK in hex next to the
decoded fields, so a checklist item can be settled from it without another run.

## Results

Add one section per run, newest first:

```markdown
### YYYY-MM-DD: <what was verified>

- Host: <OS, architecture, compiler, build type>
- LiDAR: firmware <version_app>, serial <SN>, <direct link / switch>
- Commit: <hash>

<summary.md table>

Findings: <issues filed, items of #11 confirmed or refuted>
```

### 2026-10-04: first run of every binary, four builds

- Host: Ubuntu 24.04.3 LTS, x86_64, Linux 7.0.0-28. Builds: GCC 13 and Clang 19, each in
  Release and Debug.
- Network:
  - The host is on 192.168.1.50/24 (`enp2s0`, NetworkManager profile with a
    `192.168.1.135/32` route).
  - Wi-Fi is on the same subnet (192.168.1.8/24) and holds the default route.
  - The LiDAR is directly connected.
  - ufw is installed but not enabled (`ENABLED=no`), so the firewall row of #110 is still
    open.
- LiDAR:
  - Mid-360 at 192.168.1.135, serial 47MDM5L0020035, MAC e4:7a:2c:8f:85:49.
  - `product_info` is `FmVer:13180244 BuildTime:2025/04/01`, which `version_app` prints as
    13.18.2.44. `version_loader` is 13.17.99.20.
  - Discovery reports `dev_type` 9 and `cmd_port` 56100.
- Commit: main at 0ab6a10, with the fixes of #217, #219, #221, #223 and #225 merged
  locally (PRs #218, #220, #222, #224, #226). Without them `minimal_receive`, `record`,
  `collect_firmware_log` and `debug-data` fail.
- No pcap: `--pcap` needs `sudo`.

All four builds passed every step. GCC Release:

| Step | Result | Detail |
| --- | --- | --- |
| info | PASS | discovery: sn=47MDM5L0020035 ip=192.168.1.135 cmd_port=56100 dev_type=9 from=192.168.1.135:56000 |
| minimal_receive | PASS | 60 s: 598 frames, 12022 IMU packets; packets=136556 points=11961024 frames=595 imu=11962 bad=0 dropped=0 reordered=0 |
| collect_firmware_log | PASS | 429 chunks, 249624 bytes in 2 files, 0 gaps, 9 ACKs sent, 0 bad packets |
| record / replay | PASS | recorded 20957 packets; replay: packets=20957 frames=100 points=2011872 dropped=0 |
| debug-data | PASS | 40088 packets in 10 s |
| probe | PASS | 14 probes, see the comment on #11 |

The GCC Debug, Clang Release and Clang Debug runs match within normal variation:

- minimal_receive: 136586 to 136625 packets, `bad=0 dropped=0`.
- record / replay: 20872 to 21039 packets, equal on both sides.
- debug-data: 40129 to 40143 packets.

Findings:

- Discovery: the LiDAR answers to `255.255.255.255`, also for a unicast request (#217).
- Point cloud packets carry `crc32 = 0`; IMU packets carry a valid CRC (#219).
- Motor start-up from IDLE takes 6.1 s, and 10.1 s when sampling is requested right after
  a stop (#221).
- Key `0x0009` is not supported. The firmware log goes to the sender of `0x0301` (#223).
- The debug raw data stream flows only while sampling (#225). The other `0x0303` results are
  in #106.
- Protocol answers to the questions of #11 are in `probe.json` and summarised on #11.

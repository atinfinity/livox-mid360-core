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
`192.168.1.1xx` as the LiDAR:

```sh
sudo ip addr add 192.168.1.50/24 dev enp2s0 noprefixroute
sudo ip route add 192.168.1.1xx/32 dev enp2s0 src 192.168.1.50
ip route get 192.168.1.1xx    # must print "dev enp2s0 src 192.168.1.50"
ip route get 192.168.1.1      # must still print the Wi-Fi interface
```

- Keep `noprefixroute`. Without it the address adds a `192.168.1.0/24` route on `enp2s0`
  with metric 0, which wins over the Wi-Fi's (metric 600 under NetworkManager): the rest of
  the LAN, the router included, becomes unreachable, while the internet may keep working
  through the default route and hide the problem.
- `--host-ip 192.168.1.50` is enough. Broadcast discovery (`255.255.255.255`) is sent from a
  socket bound to that address and leaves through `enp2s0`, so `--lidar-ip` is not needed.
  Without `--host-ip` the broadcast leaves through the Wi-Fi and finds nothing.
- Before plugging in the LiDAR, `ping 192.168.1.1xx` and `ping 192.168.1.50`. An answer means
  that a device on the Wi-Fi network uses that address. The LiDAR stays reachable through
  the `/32` route, but that device does not, and a device on `192.168.1.50` conflicts with
  the host's wired address.
- Undo after the run: `sudo ip route del 192.168.1.1xx/32` and
  `sudo ip addr del 192.168.1.50/24 dev enp2s0`. Neither survives a reboot.

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
capture, `env.txt` (OS, compiler, commit, addresses, the LiDAR's identity) and `summary.md`, a table to paste into
the results below or into the issue. `--seconds` sets the duration of the first two steps
(default 60, as #110 asks).

To rehearse without a LiDAR, `--sim` runs the same steps against the simulator on
`127.0.0.1`.

The firmware version (`version_app`) and the `dev_type` from discovery, which #11 wants to
confirm, come from the `info` step and are copied into `summary.md`.

## Results

No run on hardware yet. Add one section per run, newest first:

```markdown
### YYYY-MM-DD: <what was verified>

- Host: <OS, architecture, compiler, build type>
- LiDAR: firmware <version_app>, serial <SN>, <direct link / switch>
- Commit: <hash>

<summary.md table>

Findings: <issues filed, items of #11 confirmed or refuted>
```

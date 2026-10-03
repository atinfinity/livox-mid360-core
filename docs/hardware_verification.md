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
wrong one, and the LiDAR's address may belong to another device on that network. Disconnect
the other interface for the run.

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
scripts/hw-first-run.sh --host-ip 192.168.1.50 --pcap enp3s0
```

`--pcap` captures the exchange with `tcpdump` (it asks for `sudo` once). The output directory,
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

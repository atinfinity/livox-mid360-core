#!/usr/bin/env bash
# First end-to-end run of the examples, the CLI and the #11 protocol probe on a Mid-360
# (issue #110). Runs every step, keeps going after a failure, and writes the logs and a
# Markdown summary into one directory that can be attached to the issue or copied into the
# verification log.
# Usage: scripts/hw-first-run.sh --host-ip A.B.C.D [--lidar-ip A.B.C.D] [--build DIR]
#          [--out DIR] [--seconds N] [--pcap IFACE] [--sim]
#   --host-ip   address of the host interface the LiDAR can reach (192.168.1.50 in the factory setup)
#   --lidar-ip  skip discovery by broadcast for the later steps (the first one always discovers)
#   --build     build directory with examples/ and tools/cli/ (default build)
#   --out       output directory (default hw-run-<UTC timestamp>)
#   --seconds   duration of minimal_receive and collect_firmware_log (default 60)
#   --pcap      also capture the exchange with tcpdump on IFACE (asks for sudo once)
#   --sim       rehearse against tools/livox_mid360_sim.py bound to --lidar-ip (default 127.0.0.1)
# Exit status: 0 when every step passed, 1 otherwise, 2 on bad arguments.
set -uo pipefail
cd "$(dirname "$0")/.."

args="$*"
host_ip=; lidar_ip=; build=build; out=; secs=60; pcap_if=; sim=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --host-ip) host_ip=$2; shift 2 ;;
    --lidar-ip) lidar_ip=$2; shift 2 ;;
    --build) build=$2; shift 2 ;;
    --out) out=$2; shift 2 ;;
    --seconds) secs=$2; shift 2 ;;
    --pcap) pcap_if=$2; shift 2 ;;
    --sim) sim=1; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
if [[ $sim = 1 ]]; then
  lidar_ip=${lidar_ip:-127.0.0.1}
  host_ip=${host_ip:-127.0.0.1}
fi
if [[ -z $host_ip ]]; then echo "--host-ip is required" >&2; exit 2; fi
out=${out:-hw-run-$(date -u +%Y%m%dT%H%M%SZ)}
receive=$build/examples/minimal_receive
fwlog=$build/examples/collect_firmware_log
cli=$build/tools/cli/livox-mid360-cli
for b in "$receive" "$fwlog" "$cli"; do
  if [[ ! -x $b ]]; then echo "$b not found; build first or pass --build" >&2; exit 2; fi
done
mkdir -p "$out"
target=(--host-ip "$host_ip")
if [[ -n $lidar_ip ]]; then target+=(--lidar-ip "$lidar_ip"); fi

pids=()
cleanup() {
  for p in "${pids[@]}"; do kill "$p" 2>/dev/null; done
  if [[ -n ${tcpdump_pid:-} ]]; then
    sudo kill -INT "$tcpdump_pid" 2>/dev/null
    sleep 1
    sudo chown "$(id -u):$(id -g)" "$out/capture.pcap" 2>/dev/null
  fi
}
trap cleanup EXIT

{
  echo "date: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "command: $0 $args"
  echo "kernel: $(uname -srm)"
  grep PRETTY_NAME /etc/os-release 2>/dev/null
  echo "commit: $(git rev-parse --short HEAD 2>/dev/null)$(git diff --quiet 2>/dev/null || echo ' (modified)')"
  echo "cli: $("$cli" --version)"
  grep -E '^CMAKE_(BUILD_TYPE|CXX_COMPILER):' "$build/CMakeCache.txt" 2>/dev/null
  echo "sim: $sim"
  echo "--- ip -br addr"; ip -br addr
  echo "--- ip route"; ip route
} > "$out/env.txt" 2>&1

if [[ $sim = 1 ]]; then
  python3 tools/livox_mid360_sim.py --bind "$lidar_ip" --no-quit-on-eof < /dev/null \
    > "$out/sim.log" 2>&1 &
  pids+=($!)
  sleep 1
fi
if [[ -n $pcap_if ]]; then
  filter="udp and host ${lidar_ip:-$host_ip}"
  if [[ -z $lidar_ip ]]; then filter="udp and portrange 56000-56501"; fi
  sudo -v || exit 1
  # Classic pcap, which tools/livox_mid360_pcap.py reads (#10).
  sudo tcpdump -i "$pcap_if" -U -w "$out/capture.pcap" "$filter" 2> "$out/tcpdump.log" &
  sleep 1
  tcpdump_pid=$(pgrep -n -f "tcpdump -i $pcap_if -U -w $out/capture.pcap")
fi

results=()
result() {  # name, PASS/FAIL, detail
  results+=("| $1 | $2 | $3 |")
  echo "$2 $1: $3"
}

# 0. Identity: discovery and the 0x0101 inquire of the identity keys, read-only. The lines go
# into env.txt so that summary.md carries the firmware version and dev_type (#11).
"$cli" info "${target[@]}" > "$out/info.out" 2> "$out/info.err"
status=$?
identity=$(grep -m1 '^identity: ' "$out/info.out")
version=$(sed -n 's/.* version_app=\([^ ]*\).*/\1/p' <<< "$identity")
grep -E '^(discovery|identity): ' "$out/info.out" >> "$out/env.txt"
if [[ $status = 0 && -n $identity ]]; then verdict=PASS; else verdict=FAIL; fi
result info "$verdict" "exit=$status; ${version:+version_app=$version; }$(grep -m1 '^discovery: ' "$out/info.out" || head -n 1 "$out/info.err")"

# 1. Discovery and reception. Discovery by broadcast unless --lidar-ip was given.
"$receive" "${target[@]}" --seconds "$secs" --imu-every 200 \
  > "$out/minimal_receive.out" 2> "$out/minimal_receive.err"
status=$?
found=$(grep -m1 '^found ' "$out/minimal_receive.err")
stats=$(grep '^stats ' "$out/minimal_receive.err" | tail -n 1)
received=$(grep '^received ' "$out/minimal_receive.out" | tail -n 1)
bad=$(sed -n 's/.* bad=\([0-9]*\).*/\1/p' <<< "$stats")
if [[ $status = 0 && -n $found && ${bad:-1} = 0 ]]; then verdict=PASS; else verdict=FAIL; fi
result minimal_receive "$verdict" "exit=$status; ${found:-no LiDAR found}; ${received:-no frames}; ${stats:-no stats}"
if [[ -z $lidar_ip && -n $found ]]; then
  lidar_ip=$(sed -n 's/^found .* at \([0-9.]*\):.*/\1/p' <<< "$found")
  target+=(--lidar-ip "$lidar_ip")
fi

# 2. Firmware log: files are written and not empty.
"$fwlog" "${target[@]}" --out "$out/firmware_log" --duration "$secs" > "$out/collect_firmware_log.log" 2>&1
status=$?
collected=$(grep '^collected ' "$out/collect_firmware_log.log" | tail -n 1)
nonempty=$(find "$out/firmware_log" -type f -size +0 2>/dev/null | wc -l)
if [[ $status = 0 && $nonempty -gt 0 ]]; then verdict=PASS; else verdict=FAIL; fi
result collect_firmware_log "$verdict" "exit=$status; ${collected:-nothing collected}; non-empty files=$nonempty"

# 3. Record 10 s and replay: the packet counts match.
"$cli" record --out "$out/capture.lvx2" "${target[@]}" --duration 10 --force > "$out/record.log" 2>&1
status=$?
rec=$(sed -n 's/^wrote .* packets=\([0-9]*\) .*/\1/p' "$out/record.log")
replay=$("$cli" replay "$out/capture.lvx2" --rate 0 --quiet 2> "$out/replay.err" | tail -n 1)
rep=$(sed -n 's/^packets=\([0-9]*\) .*/\1/p' <<< "$replay")
if [[ $status = 0 && -n $rec && $rec = "$rep" ]]; then verdict=PASS; else verdict=FAIL; fi
result "record / replay" "$verdict" "exit=$status; recorded packets=${rec:-?}; replay: ${replay:-failed}"

# 4. Debug raw data (0x0303, #106).
"$cli" debug-data --out "$out/debug_data.bin" "${target[@]}" --duration 10 > "$out/debug_data.log" 2>&1
status=$?
wrote=$(grep '^wrote ' "$out/debug_data.log" | tail -n 1)
dpk=$(sed -n 's/^wrote .* packets=\([0-9]*\) .*/\1/p' <<< "$wrote")
if [[ $status = 0 && ${dpk:-0} -gt 0 ]]; then verdict=PASS; else verdict=FAIL; fi
result debug-data "$verdict" "exit=$status; packets=${dpk:-0}"

# 5. Protocol probe for #11: rejected writes, unknown keys and commands, the push. Writes it
# makes are restored; the answers are in probe.json.
python3 tools/livox_mid360_probe.py "${target[@]}" --out "$out/probe.json" > "$out/probe.log" 2>&1
status=$?
probes=$(grep -c '^[a-z_]*: ' "$out/probe.log")
if [[ $status = 0 && -s $out/probe.json ]]; then verdict=PASS; else verdict=FAIL; fi
result probe "$verdict" "exit=$status; probes=$probes; see probe.json"

cleanup
trap - EXIT
if [[ -f $out/capture.pcap ]]; then
  echo "pcap: $(stat -c %s "$out/capture.pcap") bytes" >> "$out/env.txt"
fi
{
  echo "# Mid-360 first run ($(sed -n 's/^date: //p' "$out/env.txt"))"
  echo
  echo '```'
  grep -vE '^---|^[a-z0-9@.-]+ +(UP|DOWN|UNKNOWN)|^default|^[0-9]' "$out/env.txt"
  echo '```'
  echo
  echo "| Step | Result | Detail |"
  echo "| --- | --- | --- |"
  printf '%s\n' "${results[@]}"
} > "$out/summary.md"
echo "summary: $out/summary.md"
! printf '%s\n' "${results[@]}" | grep -q '| FAIL |'

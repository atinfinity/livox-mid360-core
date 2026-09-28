#!/bin/sh
# Issue #147: `live` for 2 s from the simulator and `play` of tests/data/mini.lvx2, both into an
# .rrd file (no viewer needed). Checks the exit codes, that frames were logged and that the file
# is not empty. Usage: test_rerun.sh <rerun binary> <sim script> <mini.lvx2> <out dir>.
# Exit 77 = no python3.
set -u
bin=$1
sim=$2
lvx2=$3
out=$4
mkdir -p "$out"
rm -f "$out/live.rrd" "$out/play.rrd"

# frames=N from the tool's last line; fails unless N > 0 and the .rrd is not empty.
check() {
  name=$1
  log=$2
  rrd=$3
  cat "$log"
  frames=$(sed -n 's/^logged frames=\([0-9]*\) .*/\1/p' "$log")
  if [ -z "$frames" ] || [ "$frames" -eq 0 ]; then
    echo "$name: no frame logged"
    exit 1
  fi
  if [ ! -s "$rrd" ]; then
    echo "$name: $rrd missing or empty"
    exit 1
  fi
  echo "ok: $name frames=$frames rrd=$(wc -c < "$rrd") bytes"
}

"$bin" play "$lvx2" --rate 0 --save "$out/play.rrd" 2> "$out/play.log"
status=$?
if [ $status -ne 0 ]; then
  cat "$out/play.log"
  echo "play exited with $status"
  exit 1
fi
check play "$out/play.log" "$out/play.rrd"

py=$(command -v python3 || true)
if [ -z "$py" ]; then
  echo "python3 not found, skipping live"
  exit 77
fi
fifo=$(mktemp -u)
mkfifo "$fifo"
"$py" "$sim" --bind 127.0.0.1 --no-quit-on-eof > "$fifo" 2>&1 &
sim_pid=$!
exec 3< "$fifo"
rm -f "$fifo"
if ! IFS= read -r line <&3; then
  echo "simulator did not start"
  kill "$sim_pid" 2>/dev/null
  exit 1
fi
echo "sim: $line"
cat <&3 > /dev/null &
"$bin" live --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --duration 2 --save "$out/live.rrd" \
  2> "$out/live.log"
status=$?
kill "$sim_pid" 2>/dev/null
wait "$sim_pid" 2>/dev/null
if [ $status -ne 0 ]; then
  cat "$out/live.log"
  echo "live exited with $status"
  exit 1
fi
check live "$out/live.log" "$out/live.rrd"
exit 0

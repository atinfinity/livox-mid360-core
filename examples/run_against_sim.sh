#!/bin/sh
# Runs minimal_receive for 2 s against tools/livox_mid360_sim.py on 127.0.0.1 (default ports).
# Usage: run_against_sim.sh <minimal_receive binary> <sim script>. Exit 77 = no python3.
set -u
bin=$1
sim=$2
py=$(command -v python3 || true)
if [ -z "$py" ]; then
  echo "python3 not found, skipping"
  exit 77
fi
fifo=$(mktemp -u)
mkfifo "$fifo"
"$py" "$sim" --bind 127.0.0.1 --no-quit-on-eof > "$fifo" 2>&1 &
sim_pid=$!
# The first stdout line is {"event": "ready", ...}; keep draining the rest in the background.
exec 3< "$fifo"
rm -f "$fifo"
if ! IFS= read -r line <&3; then
  echo "simulator did not start"
  kill "$sim_pid" 2>/dev/null
  exit 1
fi
echo "sim: $line"
cat <&3 > /dev/null &
"$bin" --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --seconds 2
status=$?
kill "$sim_pid" 2>/dev/null
wait "$sim_pid" 2>/dev/null
echo "minimal_receive exited with $status"
exit $status

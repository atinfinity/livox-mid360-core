#!/bin/sh
# Golden run for issue #93: collect 2 s of debug raw data from the simulator and check the file
# with the Python reader. Usage: test_debug_data.sh <cli binary> <sim script> <reader script>
# <out file>. Exit 77 = no python3.
set -u
bin=$1
sim=$2
reader=$3
out=$4
py=$(command -v python3 || true)
if [ -z "$py" ]; then
  echo "python3 not found, skipping"
  exit 77
fi
fifo=$(mktemp -u)
mkfifo "$fifo"
"$py" "$sim" --bind 127.0.0.1 --no-quit-on-eof --debug-data-interval 0.02 \
  --debug-data-bytes 200 > "$fifo" 2>&1 &
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
rm -f "$out"
"$bin" debug-data --out "$out" --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --duration 2 2> "$out.log"
status=$?
cat "$out.log"
if [ $status -ne 0 ]; then
  echo "debug-data exited with $status"
  kill "$sim_pid" 2>/dev/null
  exit 1
fi
# --max-size ends the run by itself: header + 3 records of 14 + 200 bytes fit into 700 bytes.
"$bin" debug-data --out "$out.small" --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --max-size 700 \
  2> "$out.small.log"
status=$?
kill "$sim_pid" 2>/dev/null
wait "$sim_pid" 2>/dev/null
cat "$out.small.log"
if [ $status -ne 0 ]; then
  echo "debug-data --max-size exited with $status"
  exit 1
fi
written=$(sed -n 's/^wrote .* packets=\([0-9]*\) .*/\1/p' "$out.log")
summary=$("$py" "$reader" "$out" --check-sim) || { echo "reader failed: $summary"; exit 1; }
echo "reader: $summary"
read_packets=$(echo "$summary" | sed -n 's/.*"packets": \([0-9]*\).*/\1/p')
if [ -z "$written" ] || [ "$written" != "$read_packets" ]; then
  echo "packet count mismatch: written '$written', read '$read_packets'"
  exit 1
fi
if [ "$read_packets" -lt 20 ]; then
  echo "too few datagrams: $read_packets"
  exit 1
fi
summary=$("$py" "$reader" "$out.small" --check-sim) || { echo "reader failed: $summary"; exit 1; }
echo "reader (small): $summary"
small_packets=$(echo "$summary" | sed -n 's/.*"packets": \([0-9]*\).*/\1/p')
if [ "$small_packets" != 3 ]; then
  echo "--max-size 700: expected 3 datagrams, got '$small_packets'"
  exit 1
fi
echo "ok: packets=$read_packets"
exit 0

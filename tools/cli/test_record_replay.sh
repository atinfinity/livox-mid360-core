#!/bin/sh
# Golden run for issue #35: record 2 s from the simulator, replay at full speed, compare counts.
# Usage: test_record_replay.sh <cli binary> <sim script> <out.lvx2>. Exit 77 = no python3.
set -u
bin=$1
sim=$2
out=$3
py=$(command -v python3 || true)
if [ -z "$py" ]; then
  echo "python3 not found, skipping"
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
rm -f "$out"
"$bin" record --out "$out" --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --duration 2 2> "$out.record.log"
status=$?
kill "$sim_pid" 2>/dev/null
wait "$sim_pid" 2>/dev/null
cat "$out.record.log"
if [ $status -ne 0 ]; then
  echo "record exited with $status"
  exit 1
fi
rec_packets=$(sed -n 's/^wrote .* packets=\([0-9]*\) frames=\([0-9]*\).*/\1/p' "$out.record.log")
rec_frames=$(sed -n 's/^wrote .* packets=\([0-9]*\) frames=\([0-9]*\).*/\2/p' "$out.record.log")
summary=$("$bin" replay "$out" --rate 0 --quiet --frame-mode window --window-ms 50) || { echo "replay failed"; exit 1; }
echo "replay: $summary"
rep_packets=$(echo "$summary" | sed -n 's/^packets=\([0-9]*\) .*/\1/p')
rep_frames=$(echo "$summary" | sed -n 's/^packets=[0-9]* frames=\([0-9]*\) .*/\1/p')
if [ "$rec_packets" != "$rep_packets" ]; then
  echo "packet count mismatch: recorded $rec_packets, replayed $rep_packets"
  exit 1
fi
# The writer cuts file frames on absolute 50 ms bins; the 50 ms replay window starts at the
# first packet, so the counts may differ by one.
diff=$((rec_frames - rep_frames))
if [ "$diff" -gt 1 ] || [ "$diff" -lt -1 ]; then
  echo "frame count mismatch: recorded $rec_frames file frames, replayed $rep_frames"
  exit 1
fi
echo "ok: packets=$rep_packets file_frames=$rec_frames frames=$rep_frames"
exit 0

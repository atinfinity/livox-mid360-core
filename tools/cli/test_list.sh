#!/bin/sh
# list against two simulators on 127.0.0.1 and 127.0.0.2: one line per LiDAR, and exit 2 when
# nothing answers. Usage: test_list.sh <cli binary> <sim script>.
# Exit 77 = no python3, or 127.0.0.2 not configured (macOS).
set -u
bin=$1
sim=$2
py=$(command -v python3 || true)
if [ -z "$py" ]; then
  echo "python3 not found, skipping"
  exit 77
fi
if ! "$py" -c 'import socket; socket.socket(socket.AF_INET, socket.SOCK_DGRAM).bind(("127.0.0.2", 0))' \
  2>/dev/null; then
  echo "127.0.0.2 is not configured on the loopback interface, skipping"
  exit 77
fi
pids=
start_sim() {  # address, serial number
  fifo=$(mktemp -u)
  mkfifo "$fifo"
  "$py" "$sim" --bind "$1" --no-quit-on-eof --sn "$2" > "$fifo" 2>&1 &
  pids="$pids $!"
  exec 3< "$fifo"
  rm -f "$fifo"
  if ! IFS= read -r line <&3; then
    echo "simulator on $1 did not start"
    kill $pids 2>/dev/null
    exit 1
  fi
  echo "sim: $line"
  cat <&3 > /dev/null &
  exec 3<&-
}
start_sim 127.0.0.1 LISTTEST0000001
start_sim 127.0.0.2 LISTTEST0000002
out=$("$bin" list --host-ip 127.0.0.1 --lidar-ip 127.0.0.1 --lidar-ip 127.0.0.2)
status=$?
echo "$out"
"$bin" list --host-ip 127.0.0.1 --lidar-ip 127.0.0.3 --timeout-ms 300
none=$?
kill $pids 2>/dev/null
wait 2>/dev/null
if [ $status -ne 0 ]; then
  echo "list exited with $status"
  exit 1
fi
for want in 'sn=LISTTEST0000001 ip=127.0.0.1 ' 'sn=LISTTEST0000002 ip=127.0.0.2 '; do
  case "$out" in
    *"$want"*) ;;
    *) echo "missing '$want'"; exit 1 ;;
  esac
done
if [ "$(echo "$out" | wc -l)" -ne 2 ]; then
  echo "expected 2 lines"
  exit 1
fi
if [ $none -ne 2 ]; then
  echo "list with nobody answering exited with $none, expected 2"
  exit 1
fi
echo "ok"
exit 0

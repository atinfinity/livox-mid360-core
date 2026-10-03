#!/bin/sh
# info against the simulator: discovery and the identity keys come back as the simulator was
# configured, and an unknown --sn fails. Usage: test_info.sh <cli binary> <sim script>.
# Exit 77 = no python3.
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
"$py" "$sim" --bind 127.0.0.1 --no-quit-on-eof --sn INFOTEST0000042 --version-app 1.2.3.4 \
  > "$fifo" 2>&1 &
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
out=$("$bin" info --lidar-ip 127.0.0.1 --host-ip 127.0.0.1)
status=$?
echo "$out"
"$bin" info --lidar-ip 127.0.0.1 --host-ip 127.0.0.1 --sn NOSUCHSN
missing=$?
kill "$sim_pid" 2>/dev/null
wait "$sim_pid" 2>/dev/null
if [ $status -ne 0 ]; then
  echo "info exited with $status"
  exit 1
fi
for want in 'discovery: sn=INFOTEST0000042 ip=127.0.0.1 ' 'identity: sn=INFOTEST0000042 ' \
  ' version_app=1.2.3.4 '; do
  case "$out" in
    *"$want"*) ;;
    *) echo "missing '$want'"; exit 1 ;;
  esac
done
if [ $missing -ne 2 ]; then
  echo "info --sn NOSUCHSN exited with $missing, expected 2"
  exit 1
fi
echo "ok"
exit 0

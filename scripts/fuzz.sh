#!/usr/bin/env bash
# Runs every libFuzzer target briefly against the committed seed corpus.
# Usage: scripts/fuzz.sh <build-dir> [seconds-per-target]   (default 30)
#   <build-dir> must have been configured with -DLIVOX_MID360_BUILD_FUZZERS=ON (Clang) and built
#   (target `fuzzers`). The corpus in tests/fuzz/corpus/<target>/ is copied to a temporary
#   directory so that new inputs found during the run never touch the repository.
set -euo pipefail
cd "$(dirname "$0")/.."
build=${1:?usage: scripts/fuzz.sh <build-dir> [seconds]}
secs=${2:-30}
targets=(fuzz_command_frame fuzz_data_packet fuzz_key_value_list
         fuzz_transport_text fuzz_session_ack fuzz_discovery_ack
         fuzz_session_loopback fuzz_frame_assembler)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
status=0
for t in "${targets[@]}"; do
  mkdir -p "$tmp/$t"
  if [[ -d "tests/fuzz/corpus/$t" ]]; then cp "tests/fuzz/corpus/$t"/* "$tmp/$t/"; fi
  echo "== $t (${secs}s, $(ls "$tmp/$t" | wc -l | tr -d ' ') seeds)"
  if "$build/tests/$t" "$tmp/$t" -max_total_time="$secs" -rss_limit_mb=2048 >"$tmp/$t.log" 2>&1; then
    tail -n 1 "$tmp/$t.log"
  else
    cat "$tmp/$t.log"
    echo "== $t FAILED"
    status=1
  fi
done
exit $status

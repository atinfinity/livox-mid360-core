#!/usr/bin/env bash
# Installs apt packages on a CI runner (#194). `apt-get update` there has hung for half an hour
# with no output, after the runner's azure mirror failed over to archive.ubuntu.com and apt's
# own timeouts did not end it. So the index update and the package download are each bounded
# in time and retried; the install itself then runs from the local cache, so dpkg is never
# killed halfway.
# Usage: scripts/ci-apt-install.sh PACKAGE...
set -euo pipefail

sudo=sudo
if [[ $EUID == 0 ]]; then sudo=; fi
apt_opts=(-o Acquire::Retries=3 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30)

# retry SECONDS COMMAND...: up to 3 attempts, each killed after SECONDS.
retry() {
  local limit=$1 attempt
  shift
  for attempt in 1 2 3; do
    if $sudo timeout --kill-after=10 "$limit" "$@"; then return 0; fi
    echo "ci-apt-install: attempt $attempt of '$*' failed or timed out after ${limit}s" >&2
    sleep 5
  done
  return 1
}

retry 180 apt-get "${apt_opts[@]}" update
retry 600 apt-get "${apt_opts[@]}" install -y --no-install-recommends --download-only "$@"
$sudo apt-get install -y --no-install-recommends "$@"

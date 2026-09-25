#!/usr/bin/env bash
# Builds and tests the library inside ubuntu:24.04 with GCC 13, GCC 14 and Clang 19,
# plus a sanitizer run and a libFuzzer smoke run.
#
# Usage: docker/check.sh [linux/arm64|linux/amd64]
#   The optional argument selects the Docker platform (default: the host's). When the
#   container runs under emulation (architecture differs from the host), only the Release
#   builds, ctest and the Python tests run; sanitizers and fuzzers are skipped because they
#   are slow and unreliable under QEMU. On Apple Silicon linux/arm64 is native.
set -euo pipefail
cd "$(dirname "$0")/.."
platform_args=()
if [[ $# -ge 1 ]]; then platform_args=(--platform "$1"); fi
docker build -q ${platform_args[@]+"${platform_args[@]}"} -t livox-mid360-core-ci docker >/dev/null
normalize() { case "$1" in arm64|aarch64) echo arm64 ;; x86_64|amd64) echo amd64 ;; *) echo "$1" ;; esac; }
host_arch=$(normalize "$(uname -m)")
container_arch=$(normalize "$(docker run --rm ${platform_args[@]+"${platform_args[@]}"} livox-mid360-core-ci uname -m)")
emulated=0
if [[ "$host_arch" != "$container_arch" ]]; then
  emulated=1
  echo "note: host is $host_arch, container is $container_arch (emulated): sanitizers and fuzzers skipped"
fi
docker run --rm ${platform_args[@]+"${platform_args[@]}"} -e EMULATED="$emulated" -v "$PWD:/src:ro" -w /tmp \
  livox-mid360-core-ci bash -euxo pipefail -c '
  uname -m
  cp -r /src /tmp/work && cd /tmp/work && rm -rf build*
  scripts/lint.sh
  for cxx in g++-13 g++-14 clang++-19; do
    cmake -S . -B build-$cxx -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=$cxx >/dev/null
    cmake --build build-$cxx
    ctest --test-dir build-$cxx --output-on-failure -j"$(nproc)" | tail -3
  done
  if [ "$EMULATED" = 0 ]; then
    cmake -S . -B build-san -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=g++-14 \
          -DLIVOX_MID360_ENABLE_ASAN=ON -DLIVOX_MID360_ENABLE_UBSAN=ON >/dev/null
    cmake --build build-san && ctest --test-dir build-san --output-on-failure | tail -3
    cmake -S . -B build-fuzz -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=clang++-19 \
          -DLIVOX_MID360_BUILD_FUZZERS=ON -DLIVOX_MID360_ENABLE_ASAN=ON -DLIVOX_MID360_ENABLE_UBSAN=ON >/dev/null
    cmake --build build-fuzz --target fuzz_command_frame fuzz_data_packet fuzz_key_value_list
    for f in fuzz_command_frame fuzz_data_packet fuzz_key_value_list; do
      ./build-fuzz/tests/$f -max_total_time=10 -rss_limit_mb=2048 2>&1 | tail -1
    done
  else
    echo "sanitizer and fuzz steps: skipped (emulated)"
  fi
  python3 -m unittest tools/test_sim.py
  python3 tools/gen_golden_vectors.py && git init -q . && git add -A && git diff --cached --quiet -- tests/generated || true
  cmake --install build-g++-14 --prefix /tmp/inst >/dev/null && ls /tmp/inst/lib/cmake/livox_mid360_core
'

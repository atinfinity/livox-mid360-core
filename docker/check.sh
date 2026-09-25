#!/usr/bin/env bash
# Builds and tests the library inside ubuntu:24.04 with GCC 13, GCC 14 and Clang 19,
# plus a sanitizer run and a libFuzzer smoke run. Usage: docker/check.sh
set -euo pipefail
cd "$(dirname "$0")/.."
docker build -q -t livox-mid360-core-ci docker >/dev/null
docker run --rm -v "$PWD:/src:ro" -w /tmp livox-mid360-core-ci bash -euxo pipefail -c '
  cp -r /src /tmp/work && cd /tmp/work && rm -rf build*
  for cxx in g++-13 g++-14 clang++-19; do
    cmake -S . -B build-$cxx -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=$cxx >/dev/null
    cmake --build build-$cxx
    ctest --test-dir build-$cxx --output-on-failure -j"$(nproc)" | tail -3
  done
  cmake -S . -B build-san -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=g++-14 \
        -DLIVOX_MID360_ENABLE_ASAN=ON -DLIVOX_MID360_ENABLE_UBSAN=ON >/dev/null
  cmake --build build-san && ctest --test-dir build-san --output-on-failure | tail -3
  cmake -S . -B build-fuzz -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=clang++-19 \
        -DLIVOX_MID360_BUILD_FUZZERS=ON -DLIVOX_MID360_ENABLE_ASAN=ON -DLIVOX_MID360_ENABLE_UBSAN=ON >/dev/null
  cmake --build build-fuzz --target fuzz_command_frame fuzz_data_packet fuzz_key_value_list
  for f in fuzz_command_frame fuzz_data_packet fuzz_key_value_list; do
    ./build-fuzz/tests/$f -max_total_time=10 -rss_limit_mb=2048 2>&1 | tail -1
  done
  python3 -m unittest tools/test_sim.py
  python3 tools/gen_golden_vectors.py && git init -q . && git add -A && git diff --cached --quiet -- tests/generated || true
  cmake --install build-g++-14 --prefix /tmp/inst >/dev/null && ls /tmp/inst/lib/cmake/livox_mid360_core
'

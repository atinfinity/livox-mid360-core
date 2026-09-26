#!/usr/bin/env bash
# Runs the same linters as .github/workflows/lint.yml.
# Usage: scripts/lint.sh [--fix] [--no-tidy] [--no-ament] [--ament-only]
#   --fix         apply clang-format and ruff fixes instead of checking
#   --no-tidy     skip clang-tidy (needs a Clang build with compile_commands.json)
#   --no-ament    skip ament_cppcheck / ament_lint_cmake (the ROS 2 checks, issue #65)
#   --ament-only  run only ament_cppcheck / ament_lint_cmake
# Tool versions: clang-format-19 / clang-tidy-19 (or CLANG_FORMAT / CLANG_TIDY / RUN_CLANG_TIDY
# env vars), ruff (RUFF env var), ament_cppcheck / ament_lint_cmake from ament_lint's jazzy
# branch (AMENT_CPPCHECK / AMENT_LINT_CMAKE env vars) with cppcheck from apt.
set -euo pipefail
cd "$(dirname "$0")/.."
fix=0; tidy=1; ament=1; ament_only=0
for a in "$@"; do
  case "$a" in
    --fix) fix=1 ;;
    --no-tidy) tidy=0 ;;
    --no-ament) ament=0 ;;
    --ament-only) ament_only=1 ;;
    *) echo "unknown option: $a" >&2; exit 2 ;;
  esac
done
# Missing tools are tolerated here so that --ament-only works without a Clang install.
CLANG_FORMAT=${CLANG_FORMAT:-$(command -v clang-format-19 || command -v clang-format || true)}
RUN_CLANG_TIDY=${RUN_CLANG_TIDY:-$(command -v run-clang-tidy-19 || command -v run-clang-tidy || true)}
CLANG_TIDY=${CLANG_TIDY:-$(command -v clang-tidy-19 || command -v clang-tidy || true)}
RUFF=${RUFF:-ruff}
AMENT_CPPCHECK=${AMENT_CPPCHECK:-ament_cppcheck}
AMENT_LINT_CMAKE=${AMENT_LINT_CMAKE:-ament_lint_cmake}
TIDY_BUILD_DIR=${TIDY_BUILD_DIR:-build-tidy}

cxx_files=$(git ls-files 'include/*.hpp' 'src/*.cpp' 'src/*.hpp' 'tests/*.cpp' 'tests/*.hpp' | grep -v '^tests/generated/')
tidy_files=$(git ls-files 'src/*.cpp' 'tests/test_*.cpp')
cmake_files=$(git ls-files 'CMakeLists.txt' '*/CMakeLists.txt' 'cmake/*.cmake' 'cmake/*.cmake.in')

run_ament() {
  # ament_cppcheck refuses cppcheck 2.x (Ubuntu 24.04 ships 2.13) unless told otherwise.
  echo "== ament_cppcheck ($(cppcheck --version))"
  # shellcheck disable=SC2086
  AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 "$AMENT_CPPCHECK" --language c++ --include_dirs include -- \
    include src $(git ls-files 'tests/*.cpp' 'tests/*.hpp' | grep -v '^tests/generated/')
  echo "== ament_lint_cmake"
  # shellcheck disable=SC2086
  "$AMENT_LINT_CMAKE" $cmake_files
}
if [[ $ament_only = 1 ]]; then run_ament; echo "lint: OK"; exit 0; fi

echo "== clang-format ($("$CLANG_FORMAT" --version | head -1))"
if [[ $fix = 1 ]]; then
  echo "$cxx_files" | xargs "$CLANG_FORMAT" -i
else
  echo "$cxx_files" | xargs "$CLANG_FORMAT" --dry-run --Werror
fi

echo "== ruff ($("$RUFF" --version))"
if [[ $fix = 1 ]]; then
  "$RUFF" check --fix tools/ || ruff_status=$?
  "$RUFF" format tools/
else
  "$RUFF" check tools/ || ruff_status=$?
  "$RUFF" format --check tools/ || ruff_status=$?
fi
if [[ ${ruff_status:-0} != 0 ]]; then echo "ruff: findings remain" >&2; exit 1; fi

if [[ $tidy = 1 ]]; then
  echo "== clang-tidy ($("$CLANG_TIDY" --version | grep -o 'version [0-9.]*'))"
  if [[ ! -f "$TIDY_BUILD_DIR/compile_commands.json" ]]; then
    cmake -S . -B "$TIDY_BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_CXX_COMPILER="${TIDY_CXX:-clang++-19}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >/dev/null
    # Generate Catch2 / golden headers so that clang-tidy sees a complete tree.
    cmake --build "$TIDY_BUILD_DIR" --target mid360_core >/dev/null
  fi
  # shellcheck disable=SC2086
  "$RUN_CLANG_TIDY" -p "$TIDY_BUILD_DIR" -clang-tidy-binary "$CLANG_TIDY" -quiet $tidy_files
fi
if [[ $ament = 1 ]]; then run_ament; fi
echo "lint: OK"

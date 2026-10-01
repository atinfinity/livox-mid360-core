#!/usr/bin/env bash
# Runs the ROS 2 Python linters themselves, ament_flake8 and ament_pep257 from ament_lint's
# rolling branch, on tools/ and scripts/. ruff enforces the same style in CI; this confirms by
# hand that the ruff configuration in pyproject.toml still matches them (CONTRIBUTING.md, Lint).
# Needs uv: uvx fetches the tools into its cache, no venv is created (#191).
# Usage: scripts/lint-ament-python.sh
set -euo pipefail
cd "$(dirname "$0")/.."

ament_lint=git+https://github.com/ament/ament_lint.git@rolling
# The flake8 plugins ament_flake8 is configured for (its package.xml dependencies).
flake8_plugins=flake8,flake8-blind-except,flake8-builtins,flake8-class-newline
flake8_plugins+=,flake8-comprehensions,flake8-deprecated,flake8-import-order,flake8-quotes

status=0
echo "== ament_flake8"
uvx --from "${ament_lint}#subdirectory=ament_flake8" --with "$flake8_plugins" \
  ament_flake8 tools/ scripts/ || status=1
echo "== ament_pep257"
uvx --from "${ament_lint}#subdirectory=ament_pep257" --with pydocstyle \
  ament_pep257 tools/ scripts/ || status=1
if [[ $status != 0 ]]; then echo "ament python lint: findings" >&2; exit 1; fi
echo "ament python lint: OK"

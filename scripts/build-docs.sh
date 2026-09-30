#!/usr/bin/env bash
# Builds the API reference from the public headers (issue #104). The Doxyfile turns every
# Doxygen warning into an error, so this fails on undocumented or wrongly documented public
# symbols. Output: docs/reference/ (git-ignored).
# Usage: scripts/build-docs.sh
# Doxygen: the DOXYGEN env var, else the pinned binary from scripts/install-doxygen.sh, else
# doxygen on PATH. Other versions than the pinned one may report different warnings.
set -euo pipefail
cd "$(dirname "$0")/.."
pinned=$(sed -n 's/^version=//p' scripts/install-doxygen.sh)
if [[ -z "${DOXYGEN:-}" ]]; then
  for c in ".cache/doxygen-$pinned/bin/doxygen" ".cache/doxygen-$pinned/doxygen"; do
    if [[ -x "$c" ]]; then DOXYGEN=$c; break; fi
  done
fi
DOXYGEN=${DOXYGEN:-$(command -v doxygen || true)}
if [[ -z "$DOXYGEN" ]]; then
  echo "build-docs: Doxygen not found; run scripts/install-doxygen.sh (Doxygen $pinned)" >&2
  exit 1
fi
found=$("$DOXYGEN" --version | cut -d' ' -f1)
if [[ "$found" != "$pinned" ]]; then
  echo "build-docs: warning: Doxygen $found, CI uses $pinned (scripts/install-doxygen.sh)" >&2
fi

echo "== doxygen ($found)"
rm -rf docs/reference
"$DOXYGEN" Doxyfile
echo "build-docs: API reference in docs/reference/index.html"

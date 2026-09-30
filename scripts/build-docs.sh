#!/usr/bin/env bash
# Builds the project site (issue #104): the API reference from the public headers with Doxygen
# into docs/reference/, then the MkDocs site into site/ (both git-ignored). The Doxyfile turns
# every Doxygen warning into an error, so this fails on undocumented or wrongly documented public
# symbols; mkdocs runs with --strict, so it fails on broken links and anchors.
# Usage: scripts/build-docs.sh [--doxygen-only | --serve]
#   --doxygen-only  only the API reference (no Python needed)
#   --serve         build the reference, then serve the site on http://127.0.0.1:8000 with reload
# MkDocs: the versions locked in requirements-docs.txt, run through uv (https://docs.astral.sh/uv/).
# Doxygen: the DOXYGEN env var, else the pinned binary from scripts/install-doxygen.sh, else
# doxygen on PATH. Other versions than the pinned one may report different warnings.
set -euo pipefail
cd "$(dirname "$0")/.."
mode=build
case "${1:-}" in
  "") ;;
  --doxygen-only) mode=doxygen ;;
  --serve) mode=serve ;;
  *) echo "usage: $0 [--doxygen-only | --serve]" >&2; exit 2 ;;
esac
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

[[ "$mode" == doxygen ]] && exit 0
if ! command -v uv >/dev/null; then
  echo "build-docs: uv not found; see https://docs.astral.sh/uv/ (or use --doxygen-only)" >&2
  exit 1
fi
# Material prints a banner about MkDocs 2.0 on every run (moving to Zensical is a later issue).
export NO_MKDOCS_2_WARNING=1
mkdocs=(uv run --no-project --python 3.12 --with-requirements requirements-docs.txt mkdocs)
if [[ "$mode" == serve ]]; then
  exec "${mkdocs[@]}" serve --strict
fi
echo "== mkdocs"
"${mkdocs[@]}" build --strict
echo "build-docs: site in site/index.html"

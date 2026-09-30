#!/usr/bin/env bash
# Downloads the pinned Doxygen release binary into .cache/doxygen-<version>/ and checks its
# SHA256. scripts/build-docs.sh and the Docs workflow use this binary, so the API reference is
# built with the same Doxygen everywhere (distribution packages lag behind and warn differently).
# Usage: scripts/install-doxygen.sh      prints the path of the doxygen binary on success
# Supported hosts: Linux x86_64 and macOS (arm64 / x86_64); the upstream release has no Linux
# arm64 binary, so on other hosts install that version yourself and set DOXYGEN.
set -euo pipefail
cd "$(dirname "$0")/.."
version=1.18.0
case "$(uname -s)-$(uname -m)" in
  Linux-x86_64)
    asset=doxygen-$version.linux.bin.tar.gz
    sha256=14fa81bdc34171edb5f1f02b1d60e74802f0439b77fa44e592565d517d72df90
    bin=bin/doxygen ;;
  Darwin-arm64)
    asset=doxygen-$version-mac-arm.zip
    sha256=31d1c74467a9f6f456b4e6a4eff0c03e93d1ffa92da5fdf2d53a2f14ebaabbd7
    bin=doxygen ;;
  Darwin-x86_64)
    asset=doxygen-$version-mac-intel.zip
    sha256=8045d72f6f900fd430042339c1493d6eefcc85a7f990361fb57685682f16a2f9
    bin=doxygen ;;
  *)
    echo "install-doxygen: no Doxygen $version release binary for $(uname -s) $(uname -m);" \
      "install it yourself and set DOXYGEN" >&2
    exit 1 ;;
esac
dest=.cache/doxygen-$version
if [[ ! -x "$dest/$bin" ]]; then
  tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' EXIT
  tag=Release_${version//./_}
  curl -fsSL -o "$tmp/$asset" "https://github.com/doxygen/doxygen/releases/download/$tag/$asset"
  if command -v sha256sum >/dev/null; then
    actual=$(sha256sum "$tmp/$asset" | cut -d' ' -f1)
  else
    actual=$(shasum -a 256 "$tmp/$asset" | cut -d' ' -f1)
  fi
  if [[ "$actual" != "$sha256" ]]; then
    echo "install-doxygen: SHA256 mismatch for $asset: $actual" >&2
    exit 1
  fi
  mkdir -p "$tmp/x"
  case "$asset" in
    *.tar.gz) tar -xzf "$tmp/$asset" -C "$tmp/x" ;;
    *.zip) unzip -q "$tmp/$asset" -d "$tmp/x" ;;
  esac
  rm -rf "$dest"
  mkdir -p .cache
  mv "$tmp/x/doxygen-$version" "$dest"
fi
echo "$PWD/$dest/$bin"

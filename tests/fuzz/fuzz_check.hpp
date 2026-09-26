// SPDX-License-Identifier: Apache-2.0
// Invariant check for fuzz targets: traps (so libFuzzer reports a crash) instead of relying on
// assert(), which NDEBUG would strip.
#pragma once

namespace fuzz
{
inline void require(bool ok)
{
  if (!ok) __builtin_trap();
}
}  // namespace fuzz

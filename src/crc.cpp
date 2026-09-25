// SPDX-License-Identifier: Apache-2.0
// The CRC routines are constexpr and live entirely in crc.hpp. This translation unit exists so
// that the tables get one definition in the shared library and to keep the target non-empty.
#include "livox/mid360/crc.hpp"

namespace livox::mid360::crc {
static_assert(detail::kCrc16Table[1] == 0x1021);
static_assert(detail::kCrc32Table[1] == 0x77073096);
}  // namespace livox::mid360::crc

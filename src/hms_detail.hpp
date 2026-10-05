// SPDX-License-Identifier: Apache-2.0
// Internal (not installed): HMS text shared by to_string(Event) and to_string(LidarStatus).
#pragma once

#include <array>
#include <string>

#include "livox/mid360/export.hpp"
#include "livox/mid360/hms.hpp"

LIVOX_MID360_API_BEGIN

namespace livox::mid360::detail
{

/// The active codes as `[0x0103800a:warning,...]`, `[]` when none is active.
[[nodiscard]] std::string hms_list(const std::array<HmsCode, 8> & hms);

}  // namespace livox::mid360::detail

LIVOX_MID360_API_END

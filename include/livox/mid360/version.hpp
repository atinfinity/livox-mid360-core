// SPDX-License-Identifier: Apache-2.0
#pragma once

// Macros on purpose: usable in #if checks by consumers.
// NOLINTBEGIN(cppcoreguidelines-macro-to-enum,modernize-macro-to-enum)
#define LIVOX_MID360_CORE_VERSION_MAJOR 0
#define LIVOX_MID360_CORE_VERSION_MINOR 1
#define LIVOX_MID360_CORE_VERSION_PATCH 0
#define LIVOX_MID360_CORE_VERSION_STRING "0.1.0"
// NOLINTEND(cppcoreguidelines-macro-to-enum,modernize-macro-to-enum)

namespace livox::mid360
{
inline constexpr const char * kVersionString = LIVOX_MID360_CORE_VERSION_STRING;
/// Protocol document revision this implementation was written against.
inline constexpr const char * kProtocolDocRevision = "v1.4.12 (2026-09-21)";
}  // namespace livox::mid360

// SPDX-License-Identifier: Apache-2.0
/// @file
/// Library version and the protocol document revision it implements.
#pragma once

// Macros on purpose: usable in #if checks by consumers.
// NOLINTBEGIN(cppcoreguidelines-macro-to-enum,modernize-macro-to-enum)
#define LIVOX_MID360_CORE_VERSION_MAJOR 0         ///< Major version of the library.
#define LIVOX_MID360_CORE_VERSION_MINOR 1         ///< Minor version of the library.
#define LIVOX_MID360_CORE_VERSION_PATCH 0         ///< Patch version of the library.
#define LIVOX_MID360_CORE_VERSION_STRING "0.1.0"  ///< Version as "MAJOR.MINOR.PATCH".
// NOLINTEND(cppcoreguidelines-macro-to-enum,modernize-macro-to-enum)

namespace livox::mid360
{
/// Library version string, same as LIVOX_MID360_CORE_VERSION_STRING.
inline constexpr const char * kVersionString = LIVOX_MID360_CORE_VERSION_STRING;
/// Protocol document revision this implementation was written against.
inline constexpr const char * kProtocolDocRevision = "v1.4.12 (2026-09-21)";
}  // namespace livox::mid360

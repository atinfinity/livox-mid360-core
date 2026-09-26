// SPDX-License-Identifier: Apache-2.0
// Internal (not installed): how the SDK emits log records. Guard every call site with
// log_enabled() so that the arguments are not evaluated when the level is filtered out.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <format>
#include <string_view>
#include <utility>

#include "livox/mid360/export.hpp"
#include "livox/mid360/log.hpp"

LIVOX_MID360_API_BEGIN

namespace livox::mid360::detail
{

/// One relaxed atomic load.
[[nodiscard]] inline bool log_enabled(LogLevel level) noexcept
{
  return level != LogLevel::kOff && level <= log_level();
}

/// Hands one record to the installed handler (if any). No SDK mutex may be held.
void log_emit(LogLevel level, std::string_view serial_number, std::string_view message) noexcept;

/// Size of the stack buffer a record is formatted into; longer messages are truncated.
inline constexpr std::size_t kLogMessageCapacity = 256;

/// Formats into a stack buffer (no allocation) and emits. Callers check log_enabled() first.
template <class... Args>
void log(
  LogLevel level, std::string_view serial_number, std::format_string<Args...> fmt, Args &&... args)
{
  std::array<char, kLogMessageCapacity> buf{};
  const auto r = std::format_to_n(buf.data(), buf.size(), fmt, std::forward<Args>(args)...);
  const auto n =
    std::min(static_cast<std::size_t>(std::max(r.size, std::ptrdiff_t{0})), buf.size());
  log_emit(level, serial_number, std::string_view(buf.data(), n));
}

}  // namespace livox::mid360::detail

LIVOX_MID360_API_END

/// `LIVOX_LOG(level, serial, fmt, args...)`: the arguments are evaluated only when enabled.
/// An expression (not a statement) so that it is usable anywhere and needs no do/while.
#define LIVOX_LOG(level, serial, ...)                           \
  (::livox::mid360::detail::log_enabled(level)                  \
     ? ::livox::mid360::detail::log(level, serial, __VA_ARGS__) \
     : void())

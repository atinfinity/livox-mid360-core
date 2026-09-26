// SPDX-License-Identifier: Apache-2.0
// Opt-in diagnostic logging (issue #42). Silent by default: the SDK never writes to the
// console unless the application installs a handler. Unrelated to the LiDAR firmware log
// stream (port 56500, 0x03xx commands), see firmware_log.hpp / Device::on_firmware_log (#44).
#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <functional>
#include <string>
#include <string_view>

#include "livox/mid360/event.hpp"
#include "livox/mid360/export.hpp"

LIVOX_MID360_API_BEGIN

namespace livox::mid360
{

/// Severity, ordered: a level is emitted when it is <= the level set with set_log_level().
enum class LogLevel : std::uint8_t
{
  kOff,    ///< default: nothing is emitted
  kError,  ///< socket failures on the receive thread
  kWarn,   ///< disconnects, command timeouts
  kInfo,   ///< state transitions, reconnect attempts, host-setup replay, open / close
  kDebug,  ///< request retries, datagrams from unknown sources
  kTrace,  ///< reserved
};

/// One diagnostic record. The views are valid only during the handler call.
struct LogRecord
{
  LogLevel level = LogLevel::kInfo;
  std::int64_t time_ns = 0;        ///< system clock, nanoseconds since the Unix epoch
  std::string_view serial_number;  ///< empty when the record is not device-scoped
  std::string_view message;        ///< one line, no trailing newline, at most 255 bytes
};

/// Called from the Context receive thread, the Device worker thread and caller threads,
/// never while an SDK mutex is held. It must not throw and must not call back into the SDK.
using LogHandler = std::function<void(const LogRecord &)>;

/// Process-wide. Level filtering happens before any formatting, so kOff / kError cost one
/// atomic load per call site.
void set_log_level(LogLevel level) noexcept;
[[nodiscard]] LogLevel log_level() noexcept;

/// Process-wide; safe to call while a Context is running. A handler that is executing when
/// it is replaced finishes its call; the previous handler object is destroyed when the last
/// such call returns. A default-constructed handler restores "no output".
void set_log_handler(LogHandler handler);

/// `2026-09-26T12:34:56.789Z W [serial] message` (UTC, millisecond precision, one-letter
/// level E/W/I/D/T, `[-]` when the serial is empty). No trailing newline.
[[nodiscard]] std::string format_log_record(const LogRecord & record);

/// Writes format_log_record() + '\n' to stderr.
[[nodiscard]] LogHandler stderr_log_handler();

/// Opens (creates or, with `append`, extends) `path` and returns a handler that writes
/// format_log_record() + '\n' and flushes after every record. On failure the error is
/// DeviceError::Kind::kIo with the errno of the open.
[[nodiscard]] std::expected<LogHandler, DeviceError> file_log_handler(
  const std::filesystem::path & path, bool append = true);

[[nodiscard]] std::string_view to_string(LogLevel level) noexcept;

}  // namespace livox::mid360

LIVOX_MID360_API_END

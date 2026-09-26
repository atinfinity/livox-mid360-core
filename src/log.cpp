// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/log.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <format>
#include <memory>
#include <string>
#include <utility>

#include "log_detail.hpp"

namespace livox::mid360
{

namespace
{
std::atomic<LogLevel> g_level{LogLevel::kOff};
std::atomic<std::shared_ptr<const LogHandler>> g_handler{nullptr};

std::int64_t now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}

char level_letter(LogLevel level) noexcept
{
  switch (level) {
    case LogLevel::kOff:
      return '-';
    case LogLevel::kError:
      return 'E';
    case LogLevel::kWarn:
      return 'W';
    case LogLevel::kInfo:
      return 'I';
    case LogLevel::kDebug:
      return 'D';
    case LogLevel::kTrace:
      return 'T';
  }
  return '?';
}

/// A FILE* the handler copies share; closed when the last copy goes away.
struct FileSink
{
  explicit FileSink(std::FILE * f) : file(f) {}
  FileSink(const FileSink &) = delete;
  FileSink & operator=(const FileSink &) = delete;
  ~FileSink()
  {
    if (file != nullptr) {
      std::fclose(file);
    }
  }
  std::FILE * file;
};

void write_line(std::FILE * f, const LogRecord & record)
{
  std::string line = format_log_record(record);
  line.push_back('\n');
  std::fputs(line.c_str(), f);  // one call per line: stdio locks the stream per call
  std::fflush(f);
}
}  // namespace

void set_log_level(LogLevel level) noexcept { g_level.store(level, std::memory_order_relaxed); }

LogLevel log_level() noexcept { return g_level.load(std::memory_order_relaxed); }

void set_log_handler(LogHandler handler)
{
  std::shared_ptr<const LogHandler> next;
  if (handler) {
    next = std::make_shared<const LogHandler>(std::move(handler));
  }
  g_handler.store(std::move(next), std::memory_order_release);
}

namespace detail
{
void log_emit(LogLevel level, std::string_view serial_number, std::string_view message) noexcept
{
  const auto handler = g_handler.load(std::memory_order_acquire);
  if (!handler) {
    return;
  }
  const LogRecord record{
    .level = level, .time_ns = now_ns(), .serial_number = serial_number, .message = message};
  (*handler)(record);
}
}  // namespace detail

std::string format_log_record(const LogRecord & record)
{
  const std::time_t secs = record.time_ns / 1'000'000'000;
  const auto millis = (record.time_ns % 1'000'000'000) / 1'000'000;
  std::tm tm{};
  ::gmtime_r(&secs, &tm);
  return std::format(
    "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}.{:03}Z {} [{}] {}", tm.tm_year + 1900, tm.tm_mon + 1,
    tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, millis, level_letter(record.level),
    record.serial_number.empty() ? std::string_view("-") : record.serial_number,
    record.message);
}

LogHandler stderr_log_handler()
{
  return [](const LogRecord & record) { write_line(stderr, record); };
}

std::expected<LogHandler, DeviceError> file_log_handler(
  const std::filesystem::path & path, bool append)
{
  std::FILE * f = std::fopen(path.c_str(), append ? "a" : "w");
  if (f == nullptr) {
    return std::unexpected(DeviceError{
      .kind = DeviceError::Kind::kIo,
      .session = std::nullopt,
      .key = std::nullopt,
      .errno_value = errno});
  }
  auto sink = std::make_shared<FileSink>(f);
  return [sink](const LogRecord & record) { write_line(sink->file, record); };
}

std::string_view to_string(LogLevel level) noexcept
{
  switch (level) {
    case LogLevel::kOff:
      return "off";
    case LogLevel::kError:
      return "error";
    case LogLevel::kWarn:
      return "warn";
    case LogLevel::kInfo:
      return "info";
    case LogLevel::kDebug:
      return "debug";
    case LogLevel::kTrace:
      return "trace";
  }
  return "unknown";
}

}  // namespace livox::mid360

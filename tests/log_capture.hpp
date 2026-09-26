// SPDX-License-Identifier: Apache-2.0
// Test helper: installs a capturing log handler for the scope of a test case. All tests share
// one process, so the previous level and handler ("off", none) are restored on destruction.
#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "livox/mid360/log.hpp"

struct CapturedRecord
{
  livox::mid360::LogLevel level;
  std::int64_t time_ns;
  std::string serial_number;
  std::string message;
};

class ScopedLogCapture
{
public:
  explicit ScopedLogCapture(livox::mid360::LogLevel level)
  {
    livox::mid360::set_log_handler([this](const livox::mid360::LogRecord & r) {
      const std::lock_guard lock(mutex_);
      records_.push_back(
        {r.level, r.time_ns, std::string(r.serial_number), std::string(r.message)});
    });
    livox::mid360::set_log_level(level);
  }
  ScopedLogCapture(const ScopedLogCapture &) = delete;
  ScopedLogCapture & operator=(const ScopedLogCapture &) = delete;
  ~ScopedLogCapture()
  {
    livox::mid360::set_log_level(livox::mid360::LogLevel::kOff);
    livox::mid360::set_log_handler({});
  }

  [[nodiscard]] std::vector<CapturedRecord> records() const
  {
    const std::lock_guard lock(mutex_);
    return records_;
  }

  [[nodiscard]] std::size_t count() const
  {
    const std::lock_guard lock(mutex_);
    return records_.size();
  }

  /// Number of records at `level` whose message contains `needle`.
  [[nodiscard]] std::size_t count(livox::mid360::LogLevel level, std::string_view needle) const
  {
    const std::lock_guard lock(mutex_);
    std::size_t n = 0;
    for (const auto & r : records_) {
      if (r.level == level && r.message.find(needle) != std::string::npos) {
        ++n;
      }
    }
    return n;
  }

private:
  mutable std::mutex mutex_;
  std::vector<CapturedRecord> records_;
};

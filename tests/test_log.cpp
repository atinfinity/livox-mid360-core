// SPDX-License-Identifier: Apache-2.0
// SDK logging (issue #42): level filtering, sinks, formatting, handler swap under emission.
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <cerrno>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "livox/mid360/log.hpp"
#include "log_capture.hpp"
#include "log_detail.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{
std::filesystem::path temp_file(const char * name)
{
  const auto dir = std::filesystem::temp_directory_path() / "livox_mid360_test_log";
  std::filesystem::create_directories(dir);
  return dir / (std::string(name) + "." + std::to_string(::getpid()));
}

std::vector<std::string> lines_of(const std::filesystem::path & p)
{
  std::ifstream in(p);
  std::vector<std::string> out;
  for (std::string line; std::getline(in, line);) {
    out.push_back(line);
  }
  return out;
}
}  // namespace

TEST_CASE("Logging is off by default", "[log]")
{
  CHECK(log_level() == LogLevel::kOff);
  CHECK_FALSE(detail::log_enabled(LogLevel::kError));
  // No handler installed: emitting is a no-op rather than a crash.
  LIVOX_LOG(LogLevel::kError, "SN", "dropped {}", 1);
  detail::log_emit(LogLevel::kError, "SN", "dropped");
}

TEST_CASE("Levels are ordered and filter before formatting", "[log]")
{
  STATIC_REQUIRE(LogLevel::kOff < LogLevel::kError);
  STATIC_REQUIRE(LogLevel::kError < LogLevel::kWarn);
  STATIC_REQUIRE(LogLevel::kWarn < LogLevel::kInfo);
  STATIC_REQUIRE(LogLevel::kInfo < LogLevel::kDebug);
  STATIC_REQUIRE(LogLevel::kDebug < LogLevel::kTrace);

  ScopedLogCapture cap(LogLevel::kWarn);
  CHECK(log_level() == LogLevel::kWarn);
  CHECK(detail::log_enabled(LogLevel::kError));
  CHECK(detail::log_enabled(LogLevel::kWarn));
  CHECK_FALSE(detail::log_enabled(LogLevel::kInfo));
  CHECK_FALSE(detail::log_enabled(LogLevel::kOff));

  int evaluated = 0;
  const auto arg = [&] { return ++evaluated; };
  LIVOX_LOG(LogLevel::kInfo, "SN1", "info {}", arg());
  LIVOX_LOG(LogLevel::kWarn, "SN1", "warn {}", arg());
  LIVOX_LOG(LogLevel::kError, "", "error {}", arg());
  CHECK(evaluated == 2);  // the kInfo arguments were never evaluated

  const auto recs = cap.records();
  REQUIRE(recs.size() == 2);
  CHECK(recs[0].level == LogLevel::kWarn);
  CHECK(recs[0].serial_number == "SN1");
  CHECK(recs[0].message == "warn 1");
  CHECK(recs[0].time_ns > 0);
  CHECK(recs[1].level == LogLevel::kError);
  CHECK(recs[1].serial_number.empty());
  CHECK(recs[1].message == "error 2");

  set_log_level(LogLevel::kOff);
  LIVOX_LOG(LogLevel::kError, "SN1", "silent");
  CHECK(cap.count() == 2);
}

TEST_CASE("Messages are truncated to the stack buffer, never allocated", "[log]")
{
  ScopedLogCapture cap(LogLevel::kTrace);
  const std::string longest(1000, 'x');
  LIVOX_LOG(LogLevel::kTrace, "SN", "{}", longest);
  const auto recs = cap.records();
  REQUIRE(recs.size() == 1);
  CHECK(recs[0].level == LogLevel::kTrace);
  CHECK(recs[0].message.size() == detail::kLogMessageCapacity);
  CHECK(recs[0].message == std::string(detail::kLogMessageCapacity, 'x'));
}

TEST_CASE("format_log_record renders UTC time, level letter and serial", "[log]")
{
  LogRecord r;
  r.level = LogLevel::kWarn;
  r.time_ns = 1'700'000'000'123'456'789;  // 2023-11-14T22:13:20.123Z
  r.serial_number = "47MDL9Q0020001";
  r.message = "disconnected: push_timeout";
  CHECK(format_log_record(r) == "2023-11-14T22:13:20.123Z W [47MDL9Q0020001] disconnected: push_timeout");

  r.level = LogLevel::kError;
  r.time_ns = 0;
  r.serial_number = {};
  r.message = "poll failed";
  CHECK(format_log_record(r) == "1970-01-01T00:00:00.000Z E [-] poll failed");

  CHECK(to_string(LogLevel::kOff) == "off");
  CHECK(to_string(LogLevel::kError) == "error");
  CHECK(to_string(LogLevel::kWarn) == "warn");
  CHECK(to_string(LogLevel::kInfo) == "info");
  CHECK(to_string(LogLevel::kDebug) == "debug");
  CHECK(to_string(LogLevel::kTrace) == "trace");
}

TEST_CASE("stderr_log_handler is a callable sink", "[log]")
{
  const LogHandler h = stderr_log_handler();
  REQUIRE(h);
  const LogRecord r{.level = LogLevel::kInfo, .time_ns = 0, .serial_number = "SN", .message = "stderr sink smoke"};
  h(r);  // visible in the test output; must not throw
}

TEST_CASE("file_log_handler creates, appends and reports open errors", "[log]")
{
  const auto path = temp_file("sink");
  std::filesystem::remove(path);

  {
    auto h = file_log_handler(path, /*append=*/false);
    REQUIRE(h.has_value());
    set_log_handler(*h);
    set_log_level(LogLevel::kInfo);
    LIVOX_LOG(LogLevel::kInfo, "SN", "first");
    LIVOX_LOG(LogLevel::kWarn, "SN", "second");
    set_log_level(LogLevel::kOff);
    set_log_handler({});  // the file is closed once the last copy of the handler is gone
  }
  auto lines = lines_of(path);
  REQUIRE(lines.size() == 2);
  CHECK(lines[0].ends_with(" I [SN] first"));
  CHECK(lines[1].ends_with(" W [SN] second"));

  {
    auto h = file_log_handler(path);  // append (default)
    REQUIRE(h.has_value());
    (*h)(LogRecord{.level = LogLevel::kError, .time_ns = 0, .serial_number = {}, .message = "third"});
  }
  lines = lines_of(path);
  REQUIRE(lines.size() == 3);
  CHECK(lines[2] == "1970-01-01T00:00:00.000Z E [-] third");

  {
    auto h = file_log_handler(path, /*append=*/false);  // truncates
    REQUIRE(h.has_value());
  }
  CHECK(lines_of(path).empty());
  std::filesystem::remove(path);

  const auto bad = file_log_handler(std::filesystem::temp_directory_path() / "livox_mid360_no_such_dir" / "x.log");
  REQUIRE_FALSE(bad.has_value());
  CHECK(bad.error().kind == DeviceError::Kind::kIo);
  CHECK(bad.error().errno_value == ENOENT);
  CHECK(to_string(bad.error()).starts_with("io: "));
}

TEST_CASE("Handler swap while another thread is emitting", "[log]")
{
  set_log_level(LogLevel::kDebug);
  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> emitted{0};
  std::thread emitter([&] {
    while (!stop.load(std::memory_order_relaxed)) {
      LIVOX_LOG(LogLevel::kDebug, "SN", "tick {}", emitted.load(std::memory_order_relaxed));
      emitted.fetch_add(1, std::memory_order_relaxed);
    }
  });
  std::atomic<std::uint64_t> seen{0};
  for (int i = 0; i < 200; ++i) {
    set_log_handler([&seen](const LogRecord & r) {
      if (r.message.starts_with("tick ")) {
        seen.fetch_add(1, std::memory_order_relaxed);
      }
    });
    std::this_thread::sleep_for(200us);
    set_log_handler({});
  }
  stop.store(true, std::memory_order_relaxed);
  emitter.join();
  set_log_level(LogLevel::kOff);
  CHECK(emitted.load() > 0);
  CHECK(seen.load() <= emitted.load());
}

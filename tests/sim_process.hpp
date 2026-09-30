// SPDX-License-Identifier: Apache-2.0
// Test helper: spawns tools/livox_mid360_sim.py on free ports and waits for its
// "ready" event. The control channel is the child's stdin (JSON lines); events are
// read from its stdout. Every read is bounded, so a missing event fails the test instead of
// hanging the whole ctest run. See docs/simulator.md.
#pragma once

#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

extern char ** environ;  // NOLINT(readability-redundant-declaration)

#ifndef LIVOX_MID360_SIM_SCRIPT
#error "LIVOX_MID360_SIM_SCRIPT must be defined by CMake"
#endif

/// Minimal extractor for the flat "ready" JSON line; avoids a JSON dependency.
inline std::optional<std::int64_t> json_int(std::string_view line, std::string_view key)
{
  const std::string needle = "\"" + std::string(key) + "\":";
  const auto pos = line.find(needle);
  if (pos == std::string_view::npos) {
    return std::nullopt;
  }
  const char * start = line.data() + pos + needle.size();
  char * end = nullptr;
  const std::int64_t v = std::strtoll(start, &end, 10);
  if (end == start) {
    return std::nullopt;
  }
  return v;
}

inline std::optional<std::string> json_str(std::string_view line, std::string_view key)
{
  const std::string needle = "\"" + std::string(key) + "\":\"";
  const auto pos = line.find(needle);
  if (pos == std::string_view::npos) {
    return std::nullopt;
  }
  const auto begin = pos + needle.size();
  const auto end = line.find('"', begin);
  if (end == std::string_view::npos) {
    return std::nullopt;
  }
  return std::string(line.substr(begin, end - begin));
}

class SimProcess
{
public:
  /// How long read_event() and wait_event() wait by default. Generous: it only has to catch
  /// an event that never comes, not time anything.
  static constexpr std::chrono::milliseconds kEventTimeout{30'000};

  struct Ports
  {
    std::uint16_t discovery = 0, cmd = 0, push = 0, pcl = 0, imu = 0, log = 0;
  };

  /// Python interpreter from LIVOX_MID360_PYTHON (CMake) or PATH; nullopt if none.
  static std::optional<std::string> python()
  {
#ifdef LIVOX_MID360_PYTHON
    if (::access(LIVOX_MID360_PYTHON, X_OK) == 0) {
      return std::string(LIVOX_MID360_PYTHON);
    }
#endif
    if (const char * env = std::getenv("PYTHON"); env != nullptr && ::access(env, X_OK) == 0) {
      return std::string(env);
    }
    for (const char * p :
         {"/usr/bin/python3", "/usr/local/bin/python3", "/opt/homebrew/bin/python3"}) {
      if (::access(p, X_OK) == 0) {
        return std::string(p);
      }
    }
    return std::nullopt;
  }

  /// Spawn with free ports. Returns nullopt (and fills `error`) on any failure.
  static std::optional<SimProcess> start(
    std::string & error, std::vector<std::string> extra_args = {})
  {
    const auto py = python();
    if (!py) {
      error = "python3 not found";
      return std::nullopt;
    }
    int in_pipe[2];
    int out_pipe[2];
    if (::pipe(in_pipe) != 0 || ::pipe(out_pipe) != 0) {
      error = std::string("pipe: ") + std::strerror(errno);
      return std::nullopt;
    }
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, in_pipe[0], STDIN_FILENO);
    posix_spawn_file_actions_adddup2(&fa, out_pipe[1], STDOUT_FILENO);
    for (int fd : {in_pipe[0], in_pipe[1], out_pipe[0], out_pipe[1]}) {
      posix_spawn_file_actions_addclose(&fa, fd);
    }
    std::vector<std::string> args = {
      *py, LIVOX_MID360_SIM_SCRIPT, "--bind", "127.0.0.1", "--base-port",
      "0", "--startup-delay",       "0.1"};
    for (auto & a : extra_args) {
      args.push_back(std::move(a));
    }
    std::vector<char *> argv;
    argv.reserve(args.size() + 1);
    for (auto & a : args) {
      argv.push_back(a.data());
    }
    argv.push_back(nullptr);

    pid_t pid = 0;
    const int rc = ::posix_spawn(&pid, py->c_str(), &fa, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    ::close(in_pipe[0]);
    ::close(out_pipe[1]);
    if (rc != 0) {
      ::close(in_pipe[1]);
      ::close(out_pipe[0]);
      error = std::string("posix_spawn: ") + std::strerror(rc);
      return std::nullopt;
    }
    SimProcess sim;
    sim.pid_ = pid;
    sim.control_fd_ = in_pipe[1];
    sim.events_fd_ = out_pipe[0];
    // Wait for the ready line (bounded by the interpreter start-up time).
    while (true) {
      const auto line = sim.read_event();
      if (!line) {
        error = "simulator exited or timed out before ready";
        return std::nullopt;
      }
      if (line->find(R"("event":"ready")") == std::string::npos) {
        continue;
      }
      sim.ip_ = json_str(*line, "ip").value_or("127.0.0.1");
      sim.sn_ = json_str(*line, "sn").value_or("");
      auto port = [&](const char * k) {
        return static_cast<std::uint16_t>(json_int(*line, k).value_or(0));
      };
      sim.ports_ = {port("discovery"), port("cmd"), port("push"),
                    port("pcl"),       port("imu"), port("log")};
      if (sim.ports_.discovery == 0 || sim.ports_.cmd == 0) {
        error = "ready line lacked ports: " + *line;
        return std::nullopt;
      }
      return sim;
    }
  }

  SimProcess(const SimProcess &) = delete;
  SimProcess & operator=(const SimProcess &) = delete;
  SimProcess(SimProcess && o) noexcept { swap(o); }
  SimProcess & operator=(SimProcess && o) noexcept
  {
    swap(o);
    return *this;
  }
  ~SimProcess() { stop(); }

  [[nodiscard]] const Ports & ports() const noexcept { return ports_; }
  [[nodiscard]] const std::string & ip() const noexcept { return ip_; }
  [[nodiscard]] const std::string & sn() const noexcept { return sn_; }
  [[nodiscard]] pid_t pid() const noexcept { return pid_; }

  /// Send one JSON control line, e.g. R"({"cmd":"hms","codes":[34603011]})".
  [[nodiscard]] bool control(std::string_view json) const
  {
    if (control_fd_ < 0) {
      return false;
    }
    std::string line(json);
    line.push_back('\n');
    return ::write(control_fd_, line.data(), line.size()) == static_cast<ssize_t>(line.size());
  }

  /// Next stdout line; nullopt on EOF, or when no complete line arrives within `timeout`.
  std::optional<std::string> read_event(std::chrono::milliseconds timeout = kEventTimeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
      if (const auto nl = events_buf_.find('\n'); nl != std::string::npos) {
        std::string s = events_buf_.substr(0, nl);
        events_buf_.erase(0, nl + 1);
        if (!s.empty() && s.back() == '\r') {
          s.pop_back();
        }
        return s;
      }
      const auto left = remaining(deadline);
      if (events_fd_ < 0 || left.count() == 0) {
        return std::nullopt;
      }
      pollfd p{.fd = events_fd_, .events = POLLIN, .revents = 0};
      const int rc = ::poll(&p, 1, static_cast<int>(left.count()));
      if (rc <= 0) {
        if (rc < 0 && errno != EINTR) {
          close_events();
        }
        continue;
      }
      char chunk[4096];
      const ssize_t n = ::read(events_fd_, chunk, sizeof chunk);
      if (n > 0) {
        events_buf_.append(chunk, static_cast<std::size_t>(n));
      } else if (n == 0 || errno != EINTR) {  // EOF or error: stop reading
        close_events();
      }
    }
  }

  /// Barrier: control lines are handled in order, so a "status" round trip guarantees that
  /// every earlier control line (e.g. set_status) has been applied.
  [[nodiscard]] bool sync()
  {
    return control(R"({"cmd":"status"})") && wait_event(R"("event":"status")").has_value();
  }

  /// Read events until one contains `needle`. Returns the matching line, or nullopt on EOF or
  /// when none arrives within `timeout`.
  std::optional<std::string> wait_event(
    std::string_view needle, std::chrono::milliseconds timeout = kEventTimeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (auto line = read_event(remaining(deadline))) {
      if (line->find(needle) != std::string::npos) {
        return line;
      }
    }
    return std::nullopt;
  }

  /// Ask the simulator to quit and reap it. Returns the exit status (-1 if killed).
  int stop()
  {
    if (pid_ <= 0) {
      return -1;
    }
    (void)control(R"({"cmd":"quit"})");
    if (control_fd_ >= 0) {
      ::close(control_fd_);
    }
    control_fd_ = -1;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (read_event(remaining(deadline))) {  // drain; read_event() closes the pipe at EOF
    }
    close_events();
    int status = 0;
    while (::waitpid(pid_, &status, WNOHANG) == 0) {
      if (std::chrono::steady_clock::now() > deadline) {
        ::kill(pid_, SIGKILL);
        ::waitpid(pid_, &status, 0);
        pid_ = -1;
        return -1;
      }
      ::usleep(10'000);
    }
    pid_ = -1;
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  }

private:
  SimProcess() = default;

  static std::chrono::milliseconds remaining(std::chrono::steady_clock::time_point deadline)
  {
    const auto left =
      std::chrono::ceil<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    return std::max(left, std::chrono::milliseconds{0});
  }

  void close_events() noexcept
  {
    if (events_fd_ >= 0) {
      ::close(events_fd_);
    }
    events_fd_ = -1;
  }

  void swap(SimProcess & o) noexcept
  {
    std::swap(pid_, o.pid_);
    std::swap(control_fd_, o.control_fd_);
    std::swap(events_fd_, o.events_fd_);
    std::swap(events_buf_, o.events_buf_);
    std::swap(ports_, o.ports_);
    std::swap(ip_, o.ip_);
    std::swap(sn_, o.sn_);
  }

  pid_t pid_ = -1;
  int control_fd_ = -1;
  int events_fd_ = -1;
  std::string events_buf_;  ///< read but not yet returned by read_event()
  Ports ports_;
  std::string ip_;
  std::string sn_;
};

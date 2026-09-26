// SPDX-License-Identifier: Apache-2.0
// Test helper: spawns tools/livox_mid360_sim.py on free ports and waits for its
// "ready" event. The control channel is the child's stdin (JSON lines); events are
// read from its stdout. See docs/simulator.md.
#pragma once

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

extern char ** environ;  // NOLINT(readability-redundant-declaration)

#ifndef LIVOX_MID360_SIM_SCRIPT
#error "LIVOX_MID360_SIM_SCRIPT must be defined by CMake"
#endif

/// Minimal extractor for the flat "ready" JSON line; avoids a JSON dependency.
inline std::optional<long> json_int(std::string_view line, std::string_view key)
{
  const std::string needle = "\"" + std::string(key) + "\":";
  const auto pos = line.find(needle);
  if (pos == std::string_view::npos) {
    return std::nullopt;
  }
  const char * start = line.data() + pos + needle.size();
  char * end = nullptr;
  const long v = std::strtol(start, &end, 10);
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
  struct Ports
  {
    std::uint16_t discovery = 0, cmd = 0, push = 0, pcl = 0, imu = 0;
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
    sim.events_ = ::fdopen(out_pipe[0], "r");
    if (sim.events_ == nullptr) {
      error = "fdopen failed";
      return std::nullopt;
    }
    // Wait for the ready line (bounded by the interpreter start-up time).
    while (true) {
      const auto line = sim.read_event();
      if (!line) {
        error = "simulator exited before ready";
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
      sim.ports_ = {port("discovery"), port("cmd"), port("push"), port("pcl"), port("imu")};
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

  /// Next stdout line (blocking); nullopt on EOF.
  std::optional<std::string> read_event()
  {
    if (events_ == nullptr) {
      return std::nullopt;
    }
    char buf[4096];
    if (std::fgets(buf, sizeof buf, events_) == nullptr) {  // EOF or error: stop reading
      std::fclose(events_);
      events_ = nullptr;
      return std::nullopt;
    }
    std::string s(buf);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) {
      s.pop_back();
    }
    return s;
  }

  /// Read events until one contains `needle` or EOF. Returns the matching line.
  std::optional<std::string> wait_event(std::string_view needle)
  {
    while (auto line = read_event()) {
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
    if (events_ != nullptr) {
      while (read_event()) {  // drain; read_event() closes the stream at EOF
      }
      if (events_ != nullptr) {
        std::fclose(events_);
        events_ = nullptr;
      }
    }
    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
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
  void swap(SimProcess & o) noexcept
  {
    std::swap(pid_, o.pid_);
    std::swap(control_fd_, o.control_fd_);
    std::swap(events_, o.events_);
    std::swap(ports_, o.ports_);
    std::swap(ip_, o.ip_);
    std::swap(sn_, o.sn_);
  }

  pid_t pid_ = -1;
  int control_fd_ = -1;
  std::FILE * events_ = nullptr;
  Ports ports_;
  std::string ip_;
  std::string sn_;
};

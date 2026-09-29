// SPDX-License-Identifier: Apache-2.0
// Shared bits of livox-mid360-rerun (issue #147): options every sub-command takes, the Rerun
// recording stream and how a Frame is logged to it.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <rerun.hpp>
#include <span>
#include <string>
#include <vector>

#include "livox/mid360/mid360.hpp"

namespace viewer
{
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables): set by the SIGINT handler
extern std::atomic<bool> g_stop;
void install_sigint();

/// Where the data goes and how it looks; the options after the sub-command's own ones.
struct Options
{
  std::string connect = "rerun+http://127.0.0.1:9876/proxy";
  bool connect_given = false;
  std::filesystem::path save;  ///< .rrd; empty = none
  bool spawn = false;
  std::optional<std::filesystem::path> blueprint;  ///< replaces the embedded default
  bool no_blueprint = false;
  float radius = 0.02F;  ///< point radius, metres
  livox::mid360::FramePolicy frame_policy;
  bool extrinsic = false;  ///< apply the install attitude on the host
};

enum class Parsed : std::uint8_t
{
  kUsed,     ///< the option was one of the common ones and is valid
  kUnknown,  ///< not a common option
  kInvalid,  ///< a common option with a bad value
};
/// Common options without a value (`--spawn`, ...).
Parsed parse_flag(const std::string & key, Options & o);
/// Common options with a value (`--connect URL`, ...).
Parsed parse_option(const std::string & key, const std::string & value, Options & o);
/// The usage lines of the common options.
extern const char * const kCommonUsage;

/// The viewer layout embedded at build time from default_blueprint.rbl (make_blueprint.py).
std::span<const std::byte> default_blueprint();

/// One recording stream. `sensor_time` is a timestamp for live data (host time) and a
/// duration for recorded LiDAR time, whose epoch is unknown.
class Viewer
{
public:
  enum class SensorTime : std::uint8_t
  {
    kTimestamp,
    kDuration,
  };
  Viewer(Options options, SensorTime sensor_time);

  /// Sets up the sinks and sends the static data and the blueprint. False after printing why.
  bool open();
  /// Logs the points of `frame` (zero points, i.e. no return, are skipped) to `entity`, on that
  /// entity's next `frame` timeline step. The frame is taken by reference so that its points are
  /// not copied.
  void log(const livox::mid360::Frame & frame, const std::string & entity = "lidar/points");
  /// Blocks until the sinks have taken everything logged so far.
  void flush();
  [[nodiscard]] std::uint64_t frames() const noexcept { return frames_; }
  [[nodiscard]] std::uint64_t points() const noexcept { return points_; }

private:
  Options options_;
  SensorTime sensor_time_;
  rerun::RecordingStream rec_;
  std::array<rerun::Color, 256> palette_{};  ///< reflectivity -> colour
  std::vector<rerun::Position3D> positions_;
  std::vector<rerun::Color> colors_;
  std::uint64_t frames_ = 0;  ///< logged frames of every entity
  /// The `frame` timeline of each entity: its logged frames (Frame::index restarts per loop).
  std::map<std::string, std::uint64_t, std::less<>> steps_;
  std::uint64_t points_ = 0;
};

// Exit codes: 0 ok, 1 usage, 2 setup / I/O failure, 3 no frame logged.
int run_live(int argc, char ** argv);
int run_play(int argc, char ** argv);
}  // namespace viewer

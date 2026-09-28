// SPDX-License-Identifier: Apache-2.0
// The Rerun side of livox-mid360-rerun (issue #147): sinks, blueprint, one Points3D per Frame.
#include "viewer.hpp"

#include <algorithm>
#include <csignal>
#include <iostream>
#include <string>
#include <utility>

namespace viewer
{
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables): set by the SIGINT handler
std::atomic<bool> g_stop{false};
namespace
{
/// Must match APP_ID in make_blueprint.py: the viewer applies a blueprint to the recordings of
/// the same application id.
constexpr const char * kApplicationId = "livox_mid360";

void on_sigint(int /*signal*/) { g_stop = true; }

/// Turbo colour map (Google, Apache-2.0), polynomial approximation; t in [0, 1].
rerun::Color turbo(float t)
{
  const float t2 = t * t;
  const float t3 = t2 * t;
  const float t4 = t3 * t;
  const float t5 = t4 * t;
  auto c = [](float v) { return static_cast<std::uint8_t>(std::clamp(v, 0.0F, 1.0F) * 255.0F); };
  return {
    c(0.13572138F + 4.61539260F * t - 42.66032258F * t2 + 132.13108234F * t3 - 152.94239396F * t4 +
      59.28637943F * t5),
    c(0.09140261F + 2.19418839F * t + 4.84296658F * t2 - 14.18503333F * t3 + 4.27729857F * t4 +
      2.82956604F * t5),
    c(0.10667330F + 12.64194608F * t - 60.58204836F * t2 + 110.36276771F * t3 - 89.90310912F * t4 +
      27.34824973F * t5)};
}

bool report(const rerun::Error & e, const char * what)
{
  if (e.is_ok()) {
    return true;
  }
  std::cerr << what << ": " << e.description << "\n";
  return false;
}
}  // namespace

void install_sigint() { (void)std::signal(SIGINT, on_sigint); }

const char * const kCommonUsage =
  "  [--connect URL] [--spawn] [--save FILE.rrd] [--blueprint FILE.rbl | --no-blueprint]\n"
  "  [--radius M] [--frame-mode counter|window] [--window-ms N] [--extrinsic]\n";

Parsed parse_flag(const std::string & key, Options & o)
{
  if (key == "--spawn") {
    o.spawn = true;
  } else if (key == "--no-blueprint") {
    o.no_blueprint = true;
  } else if (key == "--extrinsic") {
    o.extrinsic = true;
  } else {
    return Parsed::kUnknown;
  }
  return Parsed::kUsed;
}

Parsed parse_option(const std::string & key, const std::string & value, Options & o)
{
  using livox::mid360::FramePolicy;
  if (key == "--connect") {
    o.connect = value;
    o.connect_given = true;
  } else if (key == "--save") {
    o.save = value;
  } else if (key == "--blueprint") {
    o.blueprint = value;
  } else if (key == "--radius") {
    o.radius = std::stof(value);
    if (!(o.radius > 0)) {
      return Parsed::kInvalid;
    }
  } else if (key == "--frame-mode") {
    if (value == "counter") {
      o.frame_policy.mode = FramePolicy::Mode::kFrameCounter;
    } else if (value == "window") {
      o.frame_policy.mode = FramePolicy::Mode::kTimeWindow;
    } else {
      return Parsed::kInvalid;
    }
  } else if (key == "--window-ms") {
    const int ms = std::stoi(value);
    if (ms <= 0) {
      return Parsed::kInvalid;
    }
    o.frame_policy.window = std::chrono::milliseconds(ms);
  } else {
    return Parsed::kUnknown;
  }
  return Parsed::kUsed;
}

Viewer::Viewer(Options options, SensorTime sensor_time)
: options_(std::move(options)), sensor_time_(sensor_time), rec_(kApplicationId)
{
  for (std::size_t i = 0; i < palette_.size(); ++i) {
    palette_[i] = turbo(static_cast<float>(i) / 255.0F);
  }
}

bool Viewer::open()
{
  if (options_.blueprint && options_.no_blueprint) {
    std::cerr << "--blueprint and --no-blueprint exclude each other\n";
    return false;
  }
  // --spawn starts a viewer that listens on the default URL; --save alone writes only the file.
  const bool to_viewer = options_.spawn || options_.connect_given || options_.save.empty();
  if (options_.spawn && !report(rerun::spawn(), "spawn")) {
    return false;
  }
  const std::string save = options_.save.string();
  if (to_viewer && !save.empty()) {
    if (!report(
          rec_.set_sinks(rerun::GrpcSink{options_.connect}, rerun::FileSink{save}), "sinks")) {
      return false;
    }
  } else if (to_viewer) {
    if (!report(rec_.connect_grpc(options_.connect), "connect")) {
      return false;
    }
  } else if (!report(rec_.save(save), "save")) {
    return false;
  }

  if (options_.blueprint) {
    if (!report(rec_.try_log_file_from_path(*options_.blueprint), "blueprint")) {
      return false;
    }
  } else if (!options_.no_blueprint) {
    const auto rbl = default_blueprint();
    if (!report(
          rec_.try_log_file_from_contents("default_blueprint.rbl", rbl.data(), rbl.size()),
          "blueprint")) {
      return false;
    }
  }
  rec_.log_static("lidar", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
  return true;
}

void Viewer::log(const livox::mid360::Frame & frame)
{
  positions_.clear();
  colors_.clear();
  positions_.reserve(frame.points.size());
  colors_.reserve(frame.points.size());
  for (const auto & p : frame.points) {
    if (p.x == 0.0F && p.y == 0.0F && p.z == 0.0F) {
      continue;
    }
    positions_.emplace_back(p.x, p.y, p.z);
    colors_.push_back(palette_[p.reflectivity]);
  }
  rec_.set_time_sequence("frame", static_cast<std::int64_t>(frames_));
  const auto t = static_cast<std::int64_t>(frame.base_time_ns);
  if (sensor_time_ == SensorTime::kTimestamp) {
    rec_.set_time_timestamp_nanos_since_epoch("sensor_time", t);
  } else {
    rec_.set_time_duration_nanos("sensor_time", t);
  }
  rec_.log(
    "lidar/points", rerun::Points3D(rerun::Collection<rerun::Position3D>::borrow(positions_))
                      .with_colors(rerun::Collection<rerun::Color>::borrow(colors_))
                      .with_radii({options_.radius}));
  ++frames_;
  points_ += positions_.size();
}

void Viewer::flush()
{
  // Blocks until the sinks have taken everything logged (the .rrd file is complete).
  report(rec_.flush_blocking(), "flush");
}
}  // namespace viewer

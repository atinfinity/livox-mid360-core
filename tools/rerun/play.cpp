// SPDX-License-Identifier: Apache-2.0
// `play`: Lvx2Player -> Rerun, paced by the recorded time (issue #147); every device of the
// file, each to its own entity (#169).
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <iostream>
#include <map>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>

#include "livox/mid360/mid360.hpp"
#include "viewer.hpp"

namespace viewer
{
namespace
{
using namespace livox::mid360;
using namespace std::chrono_literals;

struct Args
{
  std::string file;
  Lvx2PlayOptions play;
  Options view;
};

std::optional<Args> parse_args(int argc, char ** argv)
{
  Args a;
  for (int i = 0; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--loop") {
      a.play.loop = true;
      continue;
    }
    if (const auto f = parse_flag(key, a.view); f != Parsed::kUnknown) {
      if (f == Parsed::kInvalid) {
        return std::nullopt;
      }
      continue;
    }
    if (key.starts_with("--")) {
      if (i + 1 >= argc) {
        return std::nullopt;
      }
      const std::string val = argv[++i];
      if (const auto o = parse_option(key, val, a.view); o != Parsed::kUnknown) {
        if (o == Parsed::kInvalid) {
          return std::nullopt;
        }
      } else if (key == "--rate") {
        a.play.rate = std::stod(val);
        if (a.play.rate < 0) {
          return std::nullopt;
        }
      } else if (key == "--lidar-id") {
        a.play.lidar_id = static_cast<std::uint32_t>(std::stoul(val));
      } else {
        return std::nullopt;
      }
      continue;
    }
    if (!a.file.empty()) {
      return std::nullopt;
    }
    a.file = key;
  }
  if (a.file.empty()) {
    return std::nullopt;
  }
  a.play.frame_policy = a.view.frame_policy;
  return a;
}

/// The file's extrinsic of `dev`, if it is enabled.
std::optional<Extrinsic> file_extrinsic(const Lvx2DeviceInfo & dev)
{
  if (!dev.extrinsic_enable) {
    return std::nullopt;
  }
  InstallAttitude att;
  att.roll_deg = dev.roll_deg;
  att.pitch_deg = dev.pitch_deg;
  att.yaw_deg = dev.yaw_deg;
  att.x_mm = static_cast<std::int32_t>(std::lround(dev.x_m * 1000.0F));
  att.y_mm = static_cast<std::int32_t>(std::lround(dev.y_m * 1000.0F));
  att.z_mm = static_cast<std::int32_t>(std::lround(dev.z_m * 1000.0F));
  return extrinsic_from(att);
}

/// Where the frames of one played device go.
struct Target
{
  std::string entity = "lidar/points";
  std::optional<Extrinsic> extrinsic;
};
}  // namespace

int run_play(int argc, char ** argv)
{
  const auto args = parse_args(argc, argv);
  if (!args) {
    std::cerr << "usage: livox-mid360-rerun play FILE.lvx2 [--rate X] [--loop] [--lidar-id N]\n"
              << kCommonUsage;
    return 1;
  }
  install_sigint();

  Lvx2Player player(args->play);
  if (auto r = player.open(args->file); !r) {
    std::cerr << "open " << args->file << ": " << to_string(r.error()) << "\n";
    if (r.error().kind == Lvx2Error::Kind::kInvalidArgument) {
      std::cerr << "drop --lidar-id to play every device\n";  // not in the file (#169)
    }
    return 2;
  }
  for (const auto & d : player.devices()) {
    std::cerr << std::format(
      "device sn={} lidar_id={} type={} extrinsic={}\n", d.lidar_sn, d.lidar_id, d.device_type,
      d.extrinsic_enable ? "on" : "off");
  }
  // Several devices each go to lidar/<lidar_id>/points, with their own extrinsic (#169).
  const bool several = !args->play.lidar_id && player.devices().size() > 1;
  std::map<std::uint32_t, Target> targets;
  for (const auto & d : player.devices()) {
    if (args->play.lidar_id && d.lidar_id != *args->play.lidar_id) {
      continue;
    }
    Target t;
    if (several) {
      t.entity = std::format("lidar/{}/points", d.lidar_id);
    }
    if (args->view.extrinsic) {
      t.extrinsic = file_extrinsic(d);
      if (!t.extrinsic) {
        std::cerr << std::format(
          "--extrinsic: the file has no enabled extrinsic for lidar_id {}\n", d.lidar_id);
        return 2;
      }
    }
    targets.emplace(d.lidar_id, std::move(t));
  }
  if (args->view.extrinsic && targets.empty()) {
    std::cerr << "--extrinsic: the file has no device info\n";
    return 2;
  }

  // Recorded LiDAR time has no known epoch, hence a duration timeline.
  Viewer viewer(args->view, Viewer::SensorTime::kDuration);
  if (!viewer.open()) {
    return 2;
  }
  const Target no_info;  // a file without device info plays as one unnamed device
  player.on_device_frame([&](const Lvx2DeviceInfo & d, Frame && f) {
    const auto it = targets.find(d.lidar_id);
    const Target & t = it != targets.end() ? it->second : no_info;
    Frame frame = std::move(f);
    if (t.extrinsic) {
      apply(*t.extrinsic, frame);
    }
    viewer.log(frame, t.entity);
  });

  // The player runs on this thread; a helper thread turns SIGINT into a stop request.
  std::stop_source stop;
  std::jthread watcher([&](const std::stop_token & st) {
    while (!st.stop_requested() && !g_stop) {
      std::this_thread::sleep_for(50ms);
    }
    stop.request_stop();
  });
  auto stats = player.run(stop.get_token());
  watcher.request_stop();
  viewer.flush();
  if (!stats) {
    std::cerr << "play: " << to_string(stats.error()) << "\n";
    return 2;
  }
  std::cerr << std::format(
    "logged frames={} points={} (file: packets={} frames={} points={} dropped={} loops={})\n",
    viewer.frames(), viewer.points(), stats->packets, stats->frames, stats->points,
    stats->dropped_packets, stats->loops);
  return viewer.frames() == 0 ? 3 : 0;
}
}  // namespace viewer

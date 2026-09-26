// SPDX-License-Identifier: Apache-2.0
// `replay`: Lvx2Player -> one line per frame (issue #35).
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <iostream>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>

#include "cli.hpp"
#include "livox/mid360/mid360.hpp"

namespace cli
{
namespace
{
using namespace livox::mid360;
using namespace std::chrono_literals;

struct Args
{
  std::string file;
  Lvx2PlayOptions play;
  bool quiet = false;
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
    if (key == "--quiet") {
      a.quiet = true;
      continue;
    }
    if (key.starts_with("--")) {
      if (i + 1 >= argc) {
        return std::nullopt;
      }
      const std::string val = argv[++i];
      if (key == "--rate") {
        a.play.rate = std::stod(val);
        if (a.play.rate < 0) {
          return std::nullopt;
        }
      } else if (key == "--frame-mode") {
        if (val == "counter") {
          a.play.frame_policy.mode = FramePolicy::Mode::kFrameCounter;
        } else if (val == "window") {
          a.play.frame_policy.mode = FramePolicy::Mode::kTimeWindow;
        } else {
          return std::nullopt;
        }
      } else if (key == "--window-ms") {
        a.play.frame_policy.window = std::chrono::milliseconds(std::stoi(val));
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
  return a;
}
}  // namespace

int run_replay(int argc, char ** argv)
{
  const auto args = parse_args(argc, argv);
  if (!args) {
    std::cerr << "usage: livox-mid360-cli replay FILE.lvx2 [--rate X] [--loop] "
                 "[--frame-mode counter|window] [--window-ms N] [--lidar-id N] [--quiet]\n";
    return 1;
  }
  install_sigint();

  Lvx2Player player(args->play);
  if (auto r = player.open(args->file); !r) {
    std::cerr << "open " << args->file << ": " << to_string(r.error()) << "\n";
    return 2;
  }
  for (const auto & d : player.devices()) {
    std::cerr << std::format(
      "device sn={} lidar_id={} type={} extrinsic={}\n", d.lidar_sn, d.lidar_id, d.device_type,
      d.extrinsic_enable ? "on" : "off");
  }
  if (!args->quiet) {
    player.on_frame([](Frame && f) {
      std::cout << std::format(
        "frame {} points={} t={} dropped={}\n", f.index, f.points.size(), f.base_time_ns,
        f.dropped_packets);
    });
  }

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
  if (!stats) {
    std::cerr << "replay: " << to_string(stats.error()) << "\n";
    return 2;
  }
  std::cout << std::format(
    "packets={} frames={} points={} dropped={} loops={}\n", stats->packets, stats->frames,
    stats->points, stats->dropped_packets, stats->loops);
  return 0;
}
}  // namespace cli

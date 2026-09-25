// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/hms.hpp"

namespace livox::mid360 {

std::string_view to_string(HmsLevel l) noexcept {
  switch (l) {
    case HmsLevel::kNone:
      return "none";
    case HmsLevel::kInfo:
      return "info";
    case HmsLevel::kWarning:
      return "warning";
    case HmsLevel::kError:
      return "error";
    case HmsLevel::kFatal:
      return "fatal";
  }
  return "unknown";
}

namespace {
struct Entry {
  std::uint16_t id_lo, id_hi;
  std::string_view description;
  std::string_view suggestion;
};

constexpr std::string_view kTemp = "Please check the environment temperature and heat dissipation";
constexpr std::string_view kRestart = "Please try to restart the device to restore";

// Transcribed from the HMS table (wiki, Mid-360). Ranges are inclusive.
constexpr Entry kTable[] = {
    {0x0102, 0x0102, "Environment temperature is slightly high", kTemp},
    {0x0103, 0x0103, "Environment temperature is relatively high", kTemp},
    {0x0104, 0x0104, "The window is dirty, which will influence the reliability of the point cloud",
     "Please clean the window"},
    {0x0105, 0x0105, "An error occurred during device upgrade process",
     "Please restart the upgrade process"},
    {0x0111, 0x0112, "Abnormal temperature of internal components of the device", kTemp},
    {0x0113, 0x0113, "IMU stopped working", kRestart},
    {0x0114, 0x0114, "Environment temperature is high", kTemp},
    {0x0115, 0x0115, "Environment temperature beyond the limit, the device has stopped working",
     kTemp},
    {0x0116, 0x0116, "Abnormal external voltage", "Please check the external voltage"},
    {0x0117, 0x0117, "Abnormal lidar parameters", kRestart},
    {0x0118, 0x0118, "Internal components of the device are damaged",
     "The device is not working properly, please contact the maintenance personnel"},
    {0x0201, 0x0201, "Scan module is heating", "Please wait for the scan module heating"},
    {0x0210, 0x0219, "Scan module is abnormal (error: the system is trying to recover)",
     "Please wait; if it lasts too long, or the level is fatal, restart the device"},
    {0x021C, 0x021C, "Scan module code disk is dirty, abnormal rotation speed (360L)",
     "Please try to restart the device; return to factory for inspection if not recovered"},
    {0x0304, 0x0304, "TIA DC abnormal, ranging point cloud abnormal", kRestart},
    {0x0401, 0x0401, "Communication link was linked down, now it is recovered (360S)",
     "Please check the communication link"},
    {0x0402, 0x0402, "PTP time synchronization stopped or time gap is too big",
     "Please check the PTP time source"},
    {0x0403, 0x0403, "The PTP version is 1588-v2.1, which the device does not support",
     "Please use IEEE 1588 v2.0 instead of v2.1"},
    {0x0404, 0x0404, "PPS time synchronization abnormal", "Please check the PPS and GPS signal"},
    {0x0405, 0x0405, "There was an exception in time synchronization",
     "Please check the exception reason"},
    {0x0406, 0x0406, "Time synchronization accuracy is low", "Please check the time source"},
    {0x0407, 0x0407, "PPS time synchronization fails because of loss of GPS signal",
     "Please check the GPS signal"},
    {0x0408, 0x0408, "PPS time synchronization fails because of loss of PPS signal",
     "Please check the PPS signal"},
    {0x0409, 0x0409, "GPS signal is abnormal", "Please check the GPS time source"},
    {0x040A, 0x040A, "The PTP and gPTP signals exist at the same time",
     "Please check the network topology; use PTP or gPTP alone to synchronize"},
};

const Entry* lookup(std::uint16_t id) noexcept {
  for (const auto& e : kTable) {
    if (id >= e.id_lo && id <= e.id_hi) return &e;
  }
  return nullptr;
}
}  // namespace

std::string_view hms_description(std::uint16_t abnormal_id) noexcept {
  const auto* e = lookup(abnormal_id);
  return e ? e->description : std::string_view{};
}

std::string_view hms_suggestion(std::uint16_t abnormal_id) noexcept {
  const auto* e = lookup(abnormal_id);
  return e ? e->suggestion : std::string_view{};
}

}  // namespace livox::mid360

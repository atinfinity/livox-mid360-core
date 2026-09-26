// SPDX-License-Identifier: Apache-2.0
#include "livox/mid360/event.hpp"

#include <string>
#include <string_view>

namespace livox::mid360
{

std::string_view to_string(Event::Kind kind) noexcept
{
  switch (kind) {
    case Event::Kind::kStateChanged:
      return "state_changed";
    case Event::Kind::kHms:
      return "hms";
    case Event::Kind::kDisconnected:
      return "disconnected";
    case Event::Kind::kReconnected:
      return "reconnected";
    case Event::Kind::kStats:
      return "stats";
  }
  return "unknown";
}

std::string_view to_string(DisconnectReason reason) noexcept
{
  switch (reason) {
    case DisconnectReason::kNone:
      return "none";
    case DisconnectReason::kPushTimeout:
      return "push_timeout";
    case DisconnectReason::kCommandTimeout:
      return "command_timeout";
    case DisconnectReason::kRebootRequested:
      return "reboot_requested";
    case DisconnectReason::kUser:
      return "user";
  }
  return "unknown";
}

std::string to_string(const Event & event)
{
  std::string out(to_string(event.kind));
  switch (event.kind) {
    case Event::Kind::kStateChanged:
      out += " ";
      out += to_string(event.old_state);
      out += " -> ";
      out += to_string(event.new_state);
      break;
    case Event::Kind::kHms: {
      std::size_t active = 0;
      for (const HmsCode & c : event.hms) {
        active += c.active() ? 1u : 0u;
      }
      out += " active=" + std::to_string(active);
      out += " level=";
      out += to_string(event.hms_level);
      break;
    }
    case Event::Kind::kStats: {
      const DeviceStats & s = event.stats;
      out += " packets=" + std::to_string(s.packets) + " points=" + std::to_string(s.points) +
             " frames=" + std::to_string(s.frames) + " imu=" + std::to_string(s.imu_samples) +
             " bad=" + std::to_string(s.bad_packets) +
             " dropped=" + std::to_string(s.dropped_packets) +
             " reordered=" + std::to_string(s.reordered);
      break;
    }
    case Event::Kind::kDisconnected:
      out += " reason=";
      out += to_string(event.reason);
      break;
    case Event::Kind::kReconnected:
      out += " attempts=" + std::to_string(event.attempts);
      break;
  }
  return out;
}

std::string_view to_string(DeviceError::Kind kind) noexcept
{
  switch (kind) {
    case DeviceError::Kind::kSession:
      return "session";
    case DeviceError::Kind::kInvalidArgument:
      return "invalid_argument";
    case DeviceError::Kind::kInvalidState:
      return "invalid_state";
    case DeviceError::Kind::kAlreadyRegistered:
      return "already_registered";
    case DeviceError::Kind::kNotOpen:
      return "not_open";
    case DeviceError::Kind::kDisconnected:
      return "disconnected";
  }
  return "unknown";
}

std::string to_string(const DeviceError & err)
{
  std::string out(to_string(err.kind));
  if (err.session) {
    out += ": ";
    out += to_string(*err.session);
  }
  return out;
}

}  // namespace livox::mid360

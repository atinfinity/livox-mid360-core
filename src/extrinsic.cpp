// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <numbers>
#include <span>

#include "livox/mid360/frame.hpp"

namespace livox::mid360
{

Extrinsic extrinsic_from(const InstallAttitude & a) noexcept
{
  constexpr float kDegToRad = std::numbers::pi_v<float> / 180.0F;
  const float cr = std::cos(a.roll_deg * kDegToRad);
  const float sr = std::sin(a.roll_deg * kDegToRad);
  const float cp = std::cos(a.pitch_deg * kDegToRad);
  const float sp = std::sin(a.pitch_deg * kDegToRad);
  const float cy = std::cos(a.yaw_deg * kDegToRad);
  const float sy = std::sin(a.yaw_deg * kDegToRad);
  Extrinsic e;
  e.r[0][0] = cy * cp;
  e.r[0][1] = cy * sp * sr - sy * cr;
  e.r[0][2] = cy * sp * cr + sy * sr;
  e.r[1][0] = sy * cp;
  e.r[1][1] = sy * sp * sr + cy * cr;
  e.r[1][2] = sy * sp * cr - cy * sr;
  e.r[2][0] = -sp;
  e.r[2][1] = cp * sr;
  e.r[2][2] = cp * cr;
  e.t[0] = static_cast<float>(a.x_mm) / 1000.0F;
  e.t[1] = static_cast<float>(a.y_mm) / 1000.0F;
  e.t[2] = static_cast<float>(a.z_mm) / 1000.0F;
  return e;
}

void apply(const Extrinsic & e, std::span<Point> points) noexcept
{
  for (Point & p : points) {
    const float x = p.x;
    const float y = p.y;
    const float z = p.z;
    p.x = e.r[0][0] * x + e.r[0][1] * y + e.r[0][2] * z + e.t[0];
    p.y = e.r[1][0] * x + e.r[1][1] * y + e.r[1][2] * z + e.t[1];
    p.z = e.r[2][0] * x + e.r[2][1] * y + e.r[2][2] * z + e.t[2];
  }
}

void apply(const Extrinsic & e, Frame & frame) noexcept
{
  apply(e, std::span<Point>{frame.points});
}

}  // namespace livox::mid360

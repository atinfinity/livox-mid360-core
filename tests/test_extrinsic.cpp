// SPDX-License-Identifier: Apache-2.0
// Issue #51: host-side install attitude transform.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <span>

#include "livox/mid360/frame.hpp"

using namespace livox::mid360;
using Catch::Approx;

namespace
{

Point at(float x, float y, float z)
{
  return Point{.x = x, .y = y, .z = z, .reflectivity = 7, .tag = 1, .line = 2, .offset_ns = 3};
}

void check_point(const Point & p, float x, float y, float z)
{
  CHECK(p.x == Approx(x).margin(1e-6));
  CHECK(p.y == Approx(y).margin(1e-6));
  CHECK(p.z == Approx(z).margin(1e-6));
}

}  // namespace

TEST_CASE("extrinsic_from: identity for a zero attitude", "[extrinsic]")
{
  const Extrinsic e = extrinsic_from({});
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      CHECK(e.r[i][j] == (i == j ? 1.0F : 0.0F));
    }
    CHECK(e.t[i] == 0.0F);
  }
}

TEST_CASE("extrinsic_from: single-axis rotations are right-handed", "[extrinsic]")
{
  Point p[1];
  p[0] = at(1, 0, 0);
  apply(extrinsic_from({.yaw_deg = 90.0F}), std::span<Point>{p});  // x -> y about z
  check_point(p[0], 0, 1, 0);

  p[0] = at(1, 0, 0);
  apply(extrinsic_from({.pitch_deg = 90.0F}), std::span<Point>{p});  // x -> -z about y
  check_point(p[0], 0, 0, -1);

  p[0] = at(0, 1, 0);
  apply(extrinsic_from({.roll_deg = 90.0F}), std::span<Point>{p});  // y -> z about x
  check_point(p[0], 0, 0, 1);
}

TEST_CASE("extrinsic_from: ZYX order and translation in metres after the rotation", "[extrinsic]")
{
  // yaw 90 then pitch 90 (intrinsic): R = Rz(90) * Ry(90). x -> Ry: -z -> Rz: -z.
  // y -> Ry: y -> Rz: -x. z -> Ry: x -> Rz: y.
  const Extrinsic e =
    extrinsic_from({.pitch_deg = 90.0F, .yaw_deg = 90.0F, .x_mm = 1000, .z_mm = -500});
  Point p[3] = {at(1, 0, 0), at(0, 1, 0), at(0, 0, 1)};
  apply(e, std::span<Point>{p});
  check_point(p[0], 1.0F, 0.0F, -1.5F);
  check_point(p[1], 0.0F, 0.0F, -0.5F);
  check_point(p[2], 1.0F, 1.0F, -0.5F);
  CHECK(p[0].reflectivity == 7);
  CHECK(p[0].tag == 1);
  CHECK(p[0].line == 2);
  CHECK(p[0].offset_ns == 3);
}

TEST_CASE("apply(Frame&) transforms every point and nothing else", "[extrinsic]")
{
  Frame f;
  f.index = 5;
  f.base_time_ns = 100;
  f.points = {at(1, 2, 3), at(-1, 0, 0.5F)};
  apply(extrinsic_from({.x_mm = 10, .y_mm = 20, .z_mm = 30}), f);
  check_point(f.points[0], 1.01F, 2.02F, 3.03F);
  check_point(f.points[1], -0.99F, 0.02F, 0.53F);
  CHECK(f.index == 5);
  CHECK(f.base_time_ns == 100);
}

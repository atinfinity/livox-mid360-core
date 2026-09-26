// SPDX-License-Identifier: Apache-2.0
// The device-layer headers (issue #9) are declaration-only until #6/#7/#8 land. This file
// checks that they compile and that the data types keep the layout the C ABI will rely on.
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <thread>
#include <type_traits>

#include "livox/mid360/context.hpp"
#include "livox/mid360/device.hpp"
#include "livox/mid360/event.hpp"
#include "livox/mid360/frame.hpp"

using namespace livox::mid360;

static_assert(std::is_trivially_copyable_v<Point>);
static_assert(std::is_standard_layout_v<Point>);
static_assert(sizeof(Point) == 20);
static_assert(std::is_trivially_copyable_v<ImuData>);
static_assert(std::is_standard_layout_v<ImuData>);
static_assert(std::is_trivially_copyable_v<DeviceStats>);
static_assert(std::is_trivially_copyable_v<ContextStats>);
static_assert(std::is_trivially_copyable_v<Event>);
static_assert(std::is_standard_layout_v<Event>);
static_assert(std::is_trivially_copyable_v<ReceiveInfo>);
static_assert(std::is_nothrow_move_constructible_v<Frame>);
static_assert(!std::is_copy_constructible_v<Device> && !std::is_move_constructible_v<Device>);
static_assert(!std::is_copy_constructible_v<Context> && !std::is_move_constructible_v<Context>);

TEST_CASE("BoundedQueue drops the oldest on overflow and wakes on close")
{
  BoundedQueue<int> q(2);
  q.push(1);
  q.push(2);
  q.push(3);
  CHECK(q.size() == 2);
  CHECK(q.dropped() == 1);
  CHECK(q.try_pop() == 2);
  CHECK(q.pop(std::chrono::milliseconds{10}) == 3);
  CHECK_FALSE(q.pop(std::chrono::milliseconds{1}).has_value());

  std::thread consumer([&] { CHECK_FALSE(q.pop(std::chrono::seconds{10}).has_value()); });
  q.close();
  consumer.join();
  q.push(4);  // ignored after close
  CHECK(q.size() == 0);
}

TEST_CASE("BoundedQueue moves move-only items")
{
  BoundedQueue<Frame> q;
  Frame f;
  f.points.resize(3);
  q.push(std::move(f));
  auto out = q.try_pop();
  REQUIRE(out.has_value());
  CHECK(out->points.size() == 3);
}

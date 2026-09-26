// SPDX-License-Identifier: Apache-2.0
// Host setup flow (config.hpp): pure key-value builder against the golden vector, and the
// full apply_host_setup round trip against the simulator.
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <optional>
#include <string>

#include "generated/golden_vectors.hpp"
#include "livox/mid360/config.hpp"
#include "sim_process.hpp"
#include "test_util.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

#define GOLDEN(name) span_of(golden::name, golden::name##_len)

namespace
{
struct Fixture
{
  std::optional<SimProcess> sim;
  std::string err;

  Fixture() { sim = SimProcess::start(err, {"--startup-delay", "1"}); }

  [[nodiscard]] Session connect(Ipv4 bind = {127, 0, 0, 1}) const
  {
    SessionOptions o;
    o.host_command_port = 0;
    o.bind_address = bind;
    auto s = Session::connect(Endpoint::loopback(sim->ports().cmd), o);
    REQUIRE(s.has_value());
    return std::move(*s);
  }
};

bool same(std::span<const std::byte> a, std::span<const std::byte> b)
{
  return std::ranges::equal(a, b);
}
}  // namespace

TEST_CASE("host_setup_key_values: matches the golden 0x0100 request", "[config][golden]")
{
  // Defaults + host 192.168.1.5 are exactly what gen_golden_vectors.py encodes as
  // param_config_req (keys 0x0005, 0x0006, 0x0007, 0x0000, 0x001C, seq 2).
  const HostSetup setup;
  const auto kvs = host_setup_key_values(setup, {192, 168, 1, 5});
  REQUIRE(kvs.values.size() == 5);
  CHECK(kvs.values[0].key == 0x0005);
  CHECK(kvs.values[1].key == 0x0006);
  CHECK(kvs.values[2].key == 0x0007);
  CHECK(kvs.values[3].key == 0x0000);
  CHECK(kvs.values[4].key == 0x001C);
  for (const auto & kv : kvs.values) {
    CHECK(kv.value.data() >= kvs.storage.data());
    CHECK(kv.value.data() + kv.value.size() <= kvs.storage.data() + kvs.storage.size());
  }
  const auto data = encode_param_config_request(kvs.values);
  const auto frame = build_command_frame({.seq_num = 2, .cmd_id = 0x0100, .data = data}).value();
  CHECK(same(frame, GOLDEN(param_config_req)));

  // Moving keeps the views valid.
  auto moved = host_setup_key_values(setup, {192, 168, 1, 5});
  const HostSetupKeyValues held = std::move(moved);
  CHECK(same(encode_param_config_request(held.values), data));
}

TEST_CASE("host_setup_key_values: ports, data type and imu flag are encoded", "[config]")
{
  HostSetup setup;
  setup.push_port = 1;
  setup.point_port = 2;
  setup.imu_port = 3;
  setup.pcl_data_type = DataType::kSpherical;
  setup.imu_enable = false;
  const auto kvs = host_setup_key_values(setup, {10, 0, 0, 1});
  const auto push = decode_host_ip_config(kvs.values[0].value).value();
  CHECK(push.ip == Ipv4{10, 0, 0, 1});
  CHECK(push.dst_port == 1);
  CHECK(push.src_port == kPushPort);
  CHECK(decode_host_ip_config(kvs.values[1].value)->dst_port == 2);
  CHECK(decode_host_ip_config(kvs.values[1].value)->src_port == kPointCloudPort);
  CHECK(decode_host_ip_config(kvs.values[2].value)->dst_port == 3);
  CHECK(decode_host_ip_config(kvs.values[2].value)->src_port == kImuPort);
  CHECK(decode_u8(kvs.values[3].value).value() == 3);
  CHECK(decode_u8(kvs.values[4].value).value() == 0);
}

TEST_CASE("apply_host_setup: success path reaches the requested state", "[sim][config]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Session s = f.connect();

  HostSetup setup;
  setup.ip = Ipv4{127, 0, 0, 1};
  setup.push_port = 40001;
  setup.point_port = 40002;
  setup.imu_port = 40003;
  setup.pcl_data_type = DataType::kCartesian16;
  setup.work_tgt_mode = WorkState::kIdle;
  setup.wait_timeout = 5s;
  const auto r = apply_host_setup(s, setup);
  REQUIRE(r.has_value());
  CHECK_FALSE(r->reboot_required);
  CHECK(r->final_state == WorkState::kIdle);
  CHECK(s.work_state().value() == WorkState::kIdle);

  const Key keys[] = {Key::kStateInfoHostIpCfg, Key::kPointCloudHostIpCfg, Key::kImuHostIpCfg,
                      Key::kPclDataType,        Key::kImuDataEn,           Key::kWorkTgtMode};
  const auto inq = s.inquire(keys);
  REQUIRE(inq.has_value());
  const auto pcl = decode_host_ip_config(*inq->get(Key::kPointCloudHostIpCfg)).value();
  CHECK(pcl.ip == Ipv4{127, 0, 0, 1});
  CHECK(pcl.dst_port == 40002);
  CHECK(pcl.src_port == kPointCloudPort);
  CHECK(decode_host_ip_config(*inq->get(Key::kStateInfoHostIpCfg))->dst_port == 40001);
  CHECK(decode_host_ip_config(*inq->get(Key::kImuHostIpCfg))->dst_port == 40003);
  CHECK(decode_u8(*inq->get(Key::kPclDataType)).value() == 2);
  CHECK(decode_u8(*inq->get(Key::kImuDataEn)).value() == 1);
  CHECK(decode_u8(*inq->get(Key::kWorkTgtMode)).value() == 2);

  // Back to sampling, no wait: the ACK alone is enough, final_state stays empty.
  HostSetup back;
  back.work_tgt_mode = WorkState::kSampling;
  back.wait_timeout = 0ms;
  const auto r2 = apply_host_setup(s, back);
  REQUIRE(r2.has_value());
  CHECK_FALSE(r2->final_state.has_value());
  CHECK(s.wait_for_state(WorkState::kSampling, 5s).has_value());
  CHECK(s.stats().requests >= 4);
  CHECK(f.sim->stop() == 0);
}

TEST_CASE("apply_host_setup: ip defaults to the session's local address", "[sim][config]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }
  Session s = f.connect();
  const HostSetup setup;  // no ip, no work mode
  REQUIRE(apply_host_setup(s, setup).has_value());
  const Key keys[] = {Key::kImuHostIpCfg};
  const auto inq = s.inquire(keys);
  REQUIRE(inq.has_value());
  const auto imu = decode_host_ip_config(*inq->get(Key::kImuHostIpCfg)).value();
  CHECK(imu.ip == Ipv4{127, 0, 0, 1});
  CHECK(imu.dst_port == kDefaultHostImuPort);
  CHECK(f.sim->stop() == 0);
}

TEST_CASE("apply_host_setup: rejections and invalid arguments", "[sim][config]")
{
  Fixture f;
  if (!f.sim) {
    SKIP("simulator unavailable: " << f.err);
  }

  SECTION("LiDAR rejects the data type with error_key")
  {
    Session s = f.connect();
    HostSetup setup;
    setup.pcl_data_type = static_cast<DataType>(0);
    const auto r = apply_host_setup(s, setup);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == SessionErrorKind::kLidarRejected);
    CHECK(r.error().ret_code == RetCode::kOutOfRange);
    CHECK(r.error().error_key == 0x0000);
    CHECK(r.error().cmd_id == 0x0100);
  }
  SECTION("work mode that cannot be requested")
  {
    Session s = f.connect();
    HostSetup setup;
    setup.work_tgt_mode = WorkState::kError;
    const auto before = s.stats().requests;
    const auto r = apply_host_setup(s, setup);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == SessionErrorKind::kInvalidArgument);
    CHECK(r.error().error_key == 0x001A);
    CHECK(to_string(r.error()) == "invalid_argument cmd 0x0100 after 0 attempt(s): key 0x001a");
    CHECK(s.stats().requests == before);  // nothing was sent
  }
  SECTION("0.0.0.0 bind without an explicit ip")
  {
    Session s = f.connect({0, 0, 0, 0});
    const HostSetup setup;
    const auto before = s.stats().requests;
    const auto r = apply_host_setup(s, setup);
    REQUIRE_FALSE(r.has_value());
    CHECK(r.error().kind == SessionErrorKind::kInvalidArgument);
    CHECK(r.error().error_key == 0x0006);
    CHECK(s.stats().requests == before);
  }
  CHECK(f.sim->stop() == 0);
}

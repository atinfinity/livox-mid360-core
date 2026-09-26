// SPDX-License-Identifier: Apache-2.0
#include <catch2/catch_test_macros.hpp>

#include "livox/mid360/bytes.hpp"
#include "livox/mid360/crc.hpp"
#include "livox/mid360/protocol.hpp"
#include "test_util.hpp"

using namespace livox::mid360;

namespace
{
// Builds a data packet the way the LiDAR would.
std::vector<std::byte> make_packet(
  DataType type, std::uint16_t dot_num, std::span<const std::byte> samples, std::uint64_t ts,
  std::uint16_t time_interval, std::uint16_t udp_cnt = 0, TimeType tt = TimeType::kNoSync)
{
  std::vector<std::byte> p(kDataPacketHeaderSize + samples.size());
  bytes::write_le<std::uint8_t>(p, 0, 0);
  bytes::write_le<std::uint16_t>(p, 1, static_cast<std::uint16_t>(p.size()));
  bytes::write_le<std::uint16_t>(p, 3, time_interval);
  bytes::write_le<std::uint16_t>(p, 5, dot_num);
  bytes::write_le<std::uint16_t>(p, 7, udp_cnt);
  bytes::write_le<std::uint8_t>(p, 9, 0);
  bytes::write_le<std::uint8_t>(p, 10, static_cast<std::uint8_t>(type));
  bytes::write_le<std::uint8_t>(p, 11, static_cast<std::uint8_t>(tt));
  bytes::write_le<std::uint64_t>(p, 28, ts);
  std::copy(samples.begin(), samples.end(), p.begin() + 36);
  bytes::write_le<std::uint32_t>(p, 24, crc::crc32(std::span<const std::byte>(p).subspan(28)));
  return p;
}
}  // namespace

TEST_CASE("sample sizes", "[data]")
{
  STATIC_CHECK(sample_size(DataType::kImu) == 24);
  STATIC_CHECK(sample_size(DataType::kCartesian32) == 14);
  STATIC_CHECK(sample_size(DataType::kCartesian16) == 8);
  STATIC_CHECK(sample_size(DataType::kSpherical) == 10);
  STATIC_CHECK(
    kDataPacketHeaderSize + std::size_t{96} * 14 == 1380);  // fits in one MTU-sized UDP payload
}

TEST_CASE("cartesian32 packet parse and decode", "[data]")
{
  std::vector<std::byte> s(std::size_t{14} * 2);
  bytes::write_le<std::int32_t>(s, 0, -1234);
  bytes::write_le<std::int32_t>(s, 4, 5678);
  bytes::write_le<std::int32_t>(s, 8, 91011);
  bytes::write_le<std::uint8_t>(s, 12, 200);
  bytes::write_le<std::uint8_t>(s, 13, 0b00010110);
  bytes::write_le<std::int32_t>(s, 14, 1);
  auto pkt = make_packet(DataType::kCartesian32, 2, s, 1'000'000, 1000, 7, TimeType::kPtp);
  auto v = parse_data_packet(pkt);
  REQUIRE(v);
  CHECK(v->header.length == 64);
  CHECK(v->header.dot_num == 2);
  CHECK(v->header.udp_cnt == 7);
  CHECK(v->header.data_type == DataType::kCartesian32);
  CHECK(v->header.time_type == TimeType::kPtp);
  CHECK(v->header.timestamp_ns == 1'000'000);
  auto p0 = decode_cartesian32(*v, 0);
  CHECK(p0.x_mm == -1234);
  CHECK(p0.y_mm == 5678);
  CHECK(p0.z_mm == 91011);
  CHECK(p0.reflectivity == 200);
  auto tag = decode_tag(p0.tag);
  CHECK(tag.adjacent_glue == 2);
  CHECK(tag.particles == 1);
  CHECK(tag.other == 1);
  CHECK(tag.reserved == 0);
  CHECK(decode_cartesian32(*v, 1).x_mm == 1);
  CHECK(decode_all_cartesian32(*v).size() == 2);
  CHECK(decode_all_cartesian16(*v).empty());  // wrong type -> empty
  // timestamps: 2 points, interval 1000 * 0.1us = 100us
  CHECK(sample_timestamp_ns(v->header, 0) == 1'000'000);
  CHECK(sample_timestamp_ns(v->header, 1) == 1'100'000);
}

TEST_CASE("per-point timestamp interpolation for 96 points", "[data]")
{
  DataPacketHeader h;
  h.dot_num = 96;
  h.timestamp_ns = 10;
  h.time_interval = 1035;  // 103.5 us
  CHECK(sample_timestamp_ns(h, 0) == 10);
  CHECK(sample_timestamp_ns(h, 95) == 10 + 103'500);
  CHECK(sample_timestamp_ns(h, 1) == 10 + 103'500 / 95);
  h.dot_num = 1;
  CHECK(sample_timestamp_ns(h, 0) == 10);
}

TEST_CASE("cartesian16 and spherical decode", "[data]")
{
  std::vector<std::byte> s16(8);
  bytes::write_le<std::int16_t>(s16, 0, -100);
  bytes::write_le<std::int16_t>(s16, 2, 200);
  bytes::write_le<std::int16_t>(s16, 4, -300);
  bytes::write_le<std::uint8_t>(s16, 6, 9);
  bytes::write_le<std::uint8_t>(s16, 7, 3);
  const auto pkt16 = make_packet(DataType::kCartesian16, 1, s16, 0, 0);
  auto v16 = parse_data_packet(pkt16);
  REQUIRE(v16);
  auto p = decode_cartesian16(*v16, 0);
  CHECK(p.x_cm == -100);
  CHECK(p.y_cm == 200);
  CHECK(p.z_cm == -300);
  CHECK(p.reflectivity == 9);
  CHECK(p.tag == 3);

  std::vector<std::byte> ssp(10);
  bytes::write_le<std::uint32_t>(ssp, 0, 123456);
  bytes::write_le<std::uint16_t>(ssp, 4, 18000);
  bytes::write_le<std::uint16_t>(ssp, 6, 36000);
  bytes::write_le<std::uint8_t>(ssp, 8, 1);
  bytes::write_le<std::uint8_t>(ssp, 9, 2);
  const auto pkts = make_packet(DataType::kSpherical, 1, ssp, 0, 0);
  auto vs = parse_data_packet(pkts);
  REQUIRE(vs);
  auto q = decode_spherical(*vs, 0);
  CHECK(q.depth_mm == 123456);
  CHECK(q.theta_centideg == 18000);
  CHECK(q.phi_centideg == 36000);
  CHECK(q.reflectivity == 1);
  CHECK(q.tag == 2);
}

TEST_CASE("imu packet decode", "[data]")
{
  std::vector<std::byte> s(24);
  const float vals[6] = {0.1f, -0.2f, 0.3f, 0.0f, 0.0f, 1.0f};
  for (std::size_t i = 0; i < 6; ++i) {
    bytes::write_le<float>(s, 4 * i, vals[i]);
  }
  const auto pkt = make_packet(DataType::kImu, 1, s, 77, 0);
  auto v = parse_data_packet(pkt);
  REQUIRE(v);
  auto imu = decode_imu(*v, 0);
  CHECK(imu.gyro_x == 0.1f);
  CHECK(imu.gyro_y == -0.2f);
  CHECK(imu.gyro_z == 0.3f);
  CHECK(imu.acc_z == 1.0f);
  CHECK(decode_all_imu(*v).size() == 1);
}

TEST_CASE("data packet validation errors", "[data]")
{
  std::vector<std::byte> s(14);
  auto ok = make_packet(DataType::kCartesian32, 1, s, 5, 5);
  CHECK(parse_data_packet(std::span(ok).first(20)).error() == ParseError::kTooShort);

  auto bad = ok;
  bad[0] = std::byte{1};
  CHECK(parse_data_packet(bad).error() == ParseError::kBadVersion);

  bad = ok;
  bad[1] = std::byte{0xFF};
  bad[2] = std::byte{0x7F};
  CHECK(parse_data_packet(bad).error() == ParseError::kLengthMismatch);

  bad = ok;
  bad[10] = std::byte{4};
  CHECK(parse_data_packet(bad).error() == ParseError::kUnknownDataType);

  bad = ok;
  bad[5] = std::byte{2};  // dot_num 2 but only 14 bytes
  CHECK(parse_data_packet(bad).error() == ParseError::kBadDotNum);

  bad = ok;
  bad[40] = std::byte{0xAB};  // corrupt payload
  CHECK(parse_data_packet(bad).error() == ParseError::kBadCrc32);
  CHECK(parse_data_packet(bad, /*verify_crc=*/false).has_value());

  bad = ok;
  bad[30] = std::byte{0xAB};  // corrupt timestamp: also covered by CRC
  CHECK(parse_data_packet(bad).error() == ParseError::kBadCrc32);
}

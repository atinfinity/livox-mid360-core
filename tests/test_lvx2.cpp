// SPDX-License-Identifier: Apache-2.0
// lvx2 writer / reader / player (issue #35).
#include <catch2/catch_test_macros.hpp>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include "livox/mid360/bytes.hpp"
#include "livox/mid360/lvx2.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace
{
constexpr std::uint64_t kMs = 1'000'000;

struct Packet
{
  std::vector<std::byte> data;
  DataPacketView view;
};

Packet make_packet(
  std::uint16_t udp_cnt, std::uint8_t frame_cnt, std::uint64_t ts_ns, std::uint16_t dots = 4,
  DataType type = DataType::kCartesian32)
{
  Packet p;
  p.data.resize(static_cast<std::size_t>(dots) * sample_size(type));
  for (std::size_t i = 0; i < p.data.size(); ++i) {
    p.data[i] = static_cast<std::byte>((static_cast<std::size_t>(udp_cnt) * 7 + i) & 0xFF);
  }
  p.view.header.udp_cnt = udp_cnt;
  p.view.header.frame_cnt = frame_cnt;
  p.view.header.timestamp_ns = ts_ns;
  p.view.header.dot_num = dots;
  p.view.header.data_type = type;
  p.view.header.time_type = TimeType::kPtp;
  p.view.data = p.data;
  return p;
}

std::filesystem::path temp_file(const char * name)
{
  return std::filesystem::temp_directory_path() / (std::string("mid360_lvx2_") + name);
}

Lvx2DeviceInfo device_info()
{
  Lvx2DeviceInfo d;
  d.lidar_sn = "SN12345678901234";
  d.lidar_id = 0x0A01A8C0;
  d.extrinsic_enable = true;
  d.roll_deg = 1.5F;
  d.x_m = 0.25F;
  return d;
}

std::vector<Lvx2Packet> read_all(Lvx2Reader & r, std::vector<std::vector<std::byte>> & storage)
{
  std::vector<Lvx2Packet> out;
  while (true) {
    auto p = r.next_packet();
    REQUIRE(p.has_value());
    if (!*p) {
      break;
    }
    storage.emplace_back((*p)->points.begin(), (*p)->points.end());
    out.push_back(**p);
    out.back().points = storage.back();
  }
  return out;
}
}  // namespace

TEST_CASE("lvx2 writer -> reader round trip", "[lvx2]")
{
  const auto path = temp_file("roundtrip.lvx2");
  const auto info = device_info();
  Lvx2Writer w;
  REQUIRE(w.open(path, std::span(&info, 1)));
  // Three packets in 50 ms bins 0, 0, 1 → two file frames.
  const auto p0 = make_packet(10, 1, 10 * kMs);
  const auto p1 = make_packet(11, 1, 49 * kMs, 6, DataType::kCartesian16);
  const auto p2 = make_packet(12, 2, 50 * kMs);
  REQUIRE(w.write(0, p0.view).value());
  REQUIRE(w.write(0, p1.view).value());
  REQUIRE(w.write(0, p2.view).value());
  CHECK(w.stats().frames == 1);  // the second frame is still open
  REQUIRE(w.close());
  CHECK(w.stats().frames == 2);
  CHECK(w.stats().packets == 3);
  CHECK(w.stats().bytes == std::filesystem::file_size(path));
  CHECK(!w.is_open());
  REQUIRE(w.close());  // idempotent

  Lvx2Reader r;
  REQUIRE(r.open(path));
  CHECK(r.header().version[0] == 2);
  CHECK(r.header().frame_duration_ms == 50);
  REQUIRE(r.header().device_count == 1);
  REQUIRE(r.devices().size() == 1);
  CHECK(r.devices()[0].lidar_sn == info.lidar_sn);
  CHECK(r.devices()[0].hub_sn.empty());
  CHECK(r.devices()[0].lidar_id == info.lidar_id);
  CHECK(r.devices()[0].device_type == 9);
  CHECK(r.devices()[0].extrinsic_enable);
  CHECK(r.devices()[0].roll_deg == 1.5F);
  CHECK(r.devices()[0].x_m == 0.25F);

  std::vector<std::vector<std::byte>> storage;
  const auto pk = read_all(r, storage);
  REQUIRE(pk.size() == 3);
  CHECK(!r.truncated());
  CHECK(pk[0].frame_index == 0);
  CHECK(pk[1].frame_index == 0);
  CHECK(pk[2].frame_index == 1);
  CHECK(pk[0].lidar_id == info.lidar_id);
  CHECK(pk[0].timestamp_ns == 10 * kMs);
  CHECK(pk[0].udp_counter == 10);
  CHECK(pk[0].frame_counter == 1);
  CHECK(pk[0].timestamp_type == 1);
  CHECK(pk[0].data_type == DataType::kCartesian32);
  CHECK(std::vector<std::byte>(pk[0].points.begin(), pk[0].points.end()) == p0.data);
  CHECK(pk[1].data_type == DataType::kCartesian16);
  CHECK(std::vector<std::byte>(pk[1].points.begin(), pk[1].points.end()) == p1.data);
  CHECK(pk[2].timestamp_ns == 50 * kMs);

  const auto v = pk[1].to_data_packet_view();
  CHECK(v.header.dot_num == 6);
  CHECK(v.header.data_type == DataType::kCartesian16);
  CHECK(v.header.length == kDataPacketHeaderSize + 48);
  CHECK(v.header.udp_cnt == 11);
  CHECK(v.header.time_type == TimeType::kPtp);
  CHECK(v.data.size() == 48);
  std::filesystem::remove(path);
}

TEST_CASE("lvx2 writer converts spherical to cartesian32 and ignores IMU", "[lvx2]")
{
  const auto path = temp_file("spherical.lvx2");
  const auto info = device_info();
  Lvx2Writer w;
  REQUIRE(w.open(path, std::span(&info, 1)));
  Packet sp;
  sp.data.resize(sample_size(DataType::kSpherical));
  bytes::write_le<std::uint32_t>(sp.data, 0, 2000);  // 2 m
  bytes::write_le<std::uint16_t>(sp.data, 4, 9000);  // theta 90 deg → z = 0
  bytes::write_le<std::uint16_t>(sp.data, 6, 0);     // phi 0 → +x
  sp.data[8] = std::byte{42};
  sp.data[9] = std::byte{0x20};
  sp.view.header.dot_num = 1;
  sp.view.header.data_type = DataType::kSpherical;
  sp.view.header.timestamp_ns = 5 * kMs;
  sp.view.data = sp.data;
  REQUIRE(w.write(0, sp.view).value());
  auto imu = make_packet(1, 0, 6 * kMs, 1, DataType::kImu);
  CHECK_FALSE(w.write(0, imu.view).value());
  CHECK(w.stats().ignored == 1);
  CHECK(!w.write(1, sp.view));  // device index out of range
  REQUIRE(w.close());

  Lvx2Reader r;
  REQUIRE(r.open(path));
  std::vector<std::vector<std::byte>> storage;
  const auto pk = read_all(r, storage);
  REQUIRE(pk.size() == 1);
  CHECK(pk[0].data_type == DataType::kCartesian32);
  REQUIRE(pk[0].points.size() == 14);
  CHECK(bytes::read_le<std::int32_t>(pk[0].points, 0) == 2000);
  CHECK(bytes::read_le<std::int32_t>(pk[0].points, 4) == 0);
  CHECK(bytes::read_le<std::int32_t>(pk[0].points, 8) == 0);
  CHECK(pk[0].points[12] == std::byte{42});
  CHECK(pk[0].points[13] == std::byte{0x20});
  std::filesystem::remove(path);
}

TEST_CASE("lvx2 writer rejects bad arguments", "[lvx2]")
{
  Lvx2Writer w;
  const auto info = device_info();
  auto r = w.open(temp_file("noopen.lvx2"), {});
  REQUIRE(!r);
  CHECK(r.error().kind == Lvx2Error::Kind::kInvalidArgument);
  auto wr = w.write(0, make_packet(0, 0, 0).view);
  REQUIRE(!wr);
  CHECK(wr.error().kind == Lvx2Error::Kind::kInvalidArgument);
  auto io = w.open("/nonexistent_dir_mid360/x.lvx2", std::span(&info, 1));
  REQUIRE(!io);
  CHECK(io.error().kind == Lvx2Error::Kind::kIo);
  CHECK(io.error().errno_value != 0);
  CHECK(to_string(io.error()).starts_with("io: open"));
}

TEST_CASE("lvx2 reader: committed fixture mini.lvx2", "[lvx2]")
{
  Lvx2Reader r;
  REQUIRE(r.open(std::filesystem::path(LIVOX_MID360_TEST_DATA_DIR) / "mini.lvx2"));
  CHECK(r.header().frame_duration_ms == 50);
  REQUIRE(r.devices().size() == 1);
  CHECK(r.devices()[0].lidar_sn == "MINI0001");
  CHECK(r.devices()[0].lidar_id == 0x0A01A8C0);
  CHECK(r.devices()[0].device_type == 9);
  CHECK(r.devices()[0].extrinsic_enable);
  CHECK(r.devices()[0].pitch_deg == -2.5F);
  CHECK(r.devices()[0].yaw_deg == 30.0F);
  CHECK(r.devices()[0].z_m == 0.3F);
  std::vector<std::vector<std::byte>> storage;
  const auto pk = read_all(r, storage);
  REQUIRE(pk.size() == 3);
  CHECK(!r.truncated());
  CHECK(pk[0].frame_index == 0);
  CHECK(pk[0].udp_counter == 5);
  CHECK(pk[0].frame_counter == 3);
  CHECK(pk[0].timestamp_ns == 10 * kMs);
  CHECK(pk[0].data_type == DataType::kCartesian32);
  REQUIRE(pk[0].points.size() == 28);
  const auto v0 = pk[0].to_data_packet_view();
  CHECK(v0.header.dot_num == 2);
  const auto c = decode_cartesian32(v0, 0);
  CHECK(c.x_mm == 1000);
  CHECK(c.y_mm == -2000);
  CHECK(c.z_mm == 3000);
  CHECK(c.reflectivity == 77);
  CHECK(c.tag == 0x10);
  CHECK(pk[1].data_type == DataType::kCartesian16);
  const auto v1 = pk[1].to_data_packet_view();
  CHECK(v1.header.dot_num == 1);
  CHECK(decode_cartesian16(v1, 0).y_cm == -200);
  CHECK(pk[2].frame_index == 1);
  CHECK(pk[2].udp_counter == 7);
  CHECK(pk[2].timestamp_ns == 60 * kMs);
}

TEST_CASE("lvx2 reader: truncated tail and bad files", "[lvx2]")
{
  const auto src = std::filesystem::path(LIVOX_MID360_TEST_DATA_DIR) / "mini.lvx2";
  std::ifstream in(src, std::ios::binary);
  std::vector<char> all((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  REQUIRE(all.size() > 100);

  SECTION("cut inside the last frame's points")
  {
    const auto path = temp_file("trunc.lvx2");
    std::ofstream(path, std::ios::binary)
      .write(all.data(), static_cast<std::streamsize>(all.size() - 5));
    Lvx2Reader r;
    REQUIRE(r.open(path));
    std::vector<std::vector<std::byte>> storage;
    const auto pk = read_all(r, storage);
    CHECK(pk.size() == 2);
    CHECK(r.truncated());
    std::filesystem::remove(path);
  }
  SECTION("cut inside the last frame header")
  {
    const auto path = temp_file("trunc2.lvx2");
    // frame 1 header starts at 92 + 24 + 27 + 28 + 27 + 8 = 206
    std::ofstream(path, std::ios::binary).write(all.data(), 206 + 10);
    Lvx2Reader r;
    REQUIRE(r.open(path));
    std::vector<std::vector<std::byte>> storage;
    const auto pk = read_all(r, storage);
    CHECK(pk.size() == 2);
    CHECK(r.truncated());
    std::filesystem::remove(path);
  }
  SECTION("headers only")
  {
    const auto path = temp_file("empty.lvx2");
    std::ofstream(path, std::ios::binary).write(all.data(), 92);
    Lvx2Reader r;
    REQUIRE(r.open(path));
    auto p = r.next_packet();
    REQUIRE(p);
    CHECK(!*p);
    CHECK(!r.truncated());
    std::filesystem::remove(path);
  }
  SECTION("bad magic")
  {
    const auto path = temp_file("badmagic.lvx2");
    auto copy = all;
    copy[20] ^= 1;
    std::ofstream(path, std::ios::binary)
      .write(copy.data(), static_cast<std::streamsize>(copy.size()));
    Lvx2Reader r;
    auto o = r.open(path);
    REQUIRE(!o);
    CHECK(o.error().kind == Lvx2Error::Kind::kBadFile);
    std::filesystem::remove(path);
  }
  SECTION("unsupported package data type")
  {
    const auto path = temp_file("badtype.lvx2");
    auto copy = all;
    copy[92 + 24 + 17] = 3;  // first package: data_type 3
    std::ofstream(path, std::ios::binary)
      .write(copy.data(), static_cast<std::streamsize>(copy.size()));
    Lvx2Reader r;
    REQUIRE(r.open(path));
    auto p = r.next_packet();
    REQUIRE(!p);
    CHECK(p.error().kind == Lvx2Error::Kind::kUnsupportedDataType);
    std::filesystem::remove(path);
  }
  SECTION("missing file")
  {
    Lvx2Reader r;
    auto o = r.open(temp_file("does_not_exist.lvx2"));
    REQUIRE(!o);
    CHECK(o.error().kind == Lvx2Error::Kind::kIo);
  }
}

TEST_CASE("lvx2 player: frames, loop, stop token and lidar_id filter", "[lvx2]")
{
  const auto path = temp_file("player.lvx2");
  const std::vector<Lvx2DeviceInfo> infos = {device_info(), [] {
                                               auto d = device_info();
                                               d.lidar_id = 2;
                                               d.lidar_sn = "OTHER";
                                               return d;
                                             }()};
  Lvx2Writer w;
  REQUIRE(w.open(path, infos));
  // 20 packets, frame_cnt changes every 5 packets (every 100 ms), one stray packet of device 2.
  std::uint16_t udp = 0;
  for (std::uint64_t t = 0; t < 400 * kMs; t += 20 * kMs) {
    const auto p = make_packet(udp++, static_cast<std::uint8_t>(t / (100 * kMs)), t);
    REQUIRE(w.write(0, p.view));
  }
  REQUIRE(w.write(1, make_packet(0, 0, 100 * kMs).view));
  REQUIRE(w.close());
  CHECK(w.stats().frames == 9);  // 8 bins + the out-of-order device-2 packet reopens bin 2

  SECTION("single pass, counter mode")
  {
    Lvx2PlayOptions o;
    o.rate = 0;
    Lvx2Player pl(o);
    REQUIRE(pl.open(path));
    CHECK(pl.devices().size() == 2);
    std::size_t packets = 0;
    std::vector<Frame> frames;
    pl.on_packet([&](const Lvx2Packet &) { ++packets; });
    pl.on_frame([&](Frame && f) { frames.push_back(std::move(f)); });
    auto s = pl.run();
    REQUIRE(s);
    CHECK(packets == 21);
    CHECK(s->packets == 21);
    CHECK(s->loops == 1);
    CHECK(s->frames == frames.size());
    CHECK(s->points == 84);
    // frame_cnt 0,1,2,3 plus the flushed tail (device 2's packet lands in frame_cnt 0 → drop)
    CHECK(frames.size() >= 4);
    CHECK(frames[0].points.size() == 20);
  }
  SECTION("lidar_id filter")
  {
    Lvx2PlayOptions o;
    o.rate = 0;
    o.lidar_id = infos[0].lidar_id;
    Lvx2Player pl(o);
    REQUIRE(pl.open(path));
    auto s = pl.run();
    REQUIRE(s);
    CHECK(s->packets == 20);
    CHECK(s->frames == 4);
    CHECK(s->dropped_packets == 0);
  }
  SECTION("loop until stop")
  {
    Lvx2PlayOptions o;
    o.rate = 0;
    o.loop = true;
    o.lidar_id = infos[0].lidar_id;
    Lvx2Player pl(o);
    REQUIRE(pl.open(path));
    std::stop_source stop;
    std::size_t packets = 0;
    pl.on_packet([&](const Lvx2Packet &) {
      if (++packets == 45) {
        stop.request_stop();
      }
    });
    auto s = pl.run(stop.get_token());
    REQUIRE(s);
    CHECK(s->loops == 2);
    CHECK(s->packets == 45);
  }
  SECTION("paced playback takes about the recorded time")
  {
    Lvx2PlayOptions o;
    o.rate = 4.0;  // 380 ms of data → ~95 ms
    o.lidar_id = infos[0].lidar_id;
    Lvx2Player pl(o);
    REQUIRE(pl.open(path));
    const auto t0 = std::chrono::steady_clock::now();
    auto s = pl.run();
    REQUIRE(s);
    const auto took = std::chrono::steady_clock::now() - t0;
    CHECK(took >= 80ms);
    CHECK(took < 2s);
  }
  SECTION("not open / bad rate")
  {
    Lvx2Player pl;
    auto s = pl.run();
    REQUIRE(!s);
    CHECK(s.error().kind == Lvx2Error::Kind::kInvalidArgument);
    Lvx2PlayOptions o;
    o.rate = -1;
    Lvx2Player bad(o);
    CHECK(!bad.open(path));
  }
  std::filesystem::remove(path);
}

TEST_CASE("lvx2 error strings", "[lvx2]")
{
  CHECK(to_string(Lvx2Error::Kind::kIo) == "io");
  CHECK(to_string(Lvx2Error::Kind::kInvalidArgument) == "invalid_argument");
  CHECK(to_string(Lvx2Error::Kind::kUnsupportedDataType) == "unsupported_data_type");
  CHECK(to_string(Lvx2Error::Kind::kBadFile) == "bad_file");
  CHECK(to_string(Lvx2Error{Lvx2Error::Kind::kBadFile, 0, ""}) == "bad_file");
  CHECK(to_string(Lvx2Error{Lvx2Error::Kind::kBadFile, 0, "why"}) == "bad_file: why");
  // errno is only printed for kIo
  CHECK(to_string(Lvx2Error{Lvx2Error::Kind::kBadFile, ENOENT, "why"}) == "bad_file: why");
  CHECK(
    to_string(Lvx2Error{Lvx2Error::Kind::kIo, ENOENT, "open x"}) ==
    std::string("io: open x (") + std::strerror(ENOENT) + ")");
}

TEST_CASE("lvx2 writer: open / write preconditions", "[lvx2]")
{
  const auto path = temp_file("precond.lvx2");
  const auto info = device_info();
  Lvx2Writer w;
  CHECK_FALSE(w.is_open());
  CHECK(w.close());  // closing a writer that was never opened is a no-op

  auto zero = w.open(path, std::span(&info, 1), 0);
  REQUIRE(!zero);
  CHECK(zero.error().kind == Lvx2Error::Kind::kInvalidArgument);
  const std::vector<Lvx2DeviceInfo> too_many(256, info);
  auto many = w.open(path, too_many);
  REQUIRE(!many);
  CHECK(many.error().kind == Lvx2Error::Kind::kInvalidArgument);
  CHECK_FALSE(w.is_open());

  REQUIRE(w.open(path, std::span(&info, 1)));
  CHECK(w.is_open());
  auto again = w.open(path, std::span(&info, 1));
  REQUIRE(!again);
  CHECK(again.error().kind == Lvx2Error::Kind::kInvalidArgument);
  CHECK(w.is_open());  // the failed second open leaves the first file open

  auto bad_index = w.write(1, make_packet(0, 0, 0).view);
  REQUIRE(!bad_index);
  CHECK(bad_index.error().kind == Lvx2Error::Kind::kInvalidArgument);
  REQUIRE(w.write(0, make_packet(0, 0, 0).view));
  REQUIRE(w.close());
  CHECK_FALSE(w.is_open());
  CHECK(w.stats().frames == 1);
  CHECK(w.stats().bytes == std::filesystem::file_size(path));
  std::filesystem::remove(path);
}

TEST_CASE("lvx2 reader: structural errors", "[lvx2]")
{
  const auto src = std::filesystem::path(LIVOX_MID360_TEST_DATA_DIR) / "mini.lvx2";
  std::ifstream in(src, std::ios::binary);
  std::vector<char> all((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  REQUIRE(all.size() > 100);
  const auto write_file = [](const std::filesystem::path & path, const std::vector<char> & data) {
    std::ofstream(path, std::ios::binary)
      .write(data.data(), static_cast<std::streamsize>(data.size()));
  };
  // Offsets: 29-byte file header, 63-byte device info, first frame header at 92.
  constexpr std::size_t kFrame0 = 92;
  constexpr std::size_t kPackage0 = kFrame0 + 24;

  SECTION("next_packet before open")
  {
    Lvx2Reader r;
    auto p = r.next_packet();
    REQUIRE(!p);
    CHECK(p.error().kind == Lvx2Error::Kind::kInvalidArgument);
  }
  SECTION("bad signature")
  {
    const auto path = temp_file("badsig.lvx2");
    auto copy = all;
    copy[0] = 'x';
    write_file(path, copy);
    Lvx2Reader r;
    auto o = r.open(path);
    REQUIRE(!o);
    CHECK(o.error().kind == Lvx2Error::Kind::kBadFile);
    CHECK(o.error().detail.find("signature") != std::string::npos);
    std::filesystem::remove(path);
  }
  SECTION("file shorter than the header")
  {
    const auto path = temp_file("short.lvx2");
    write_file(path, {all.begin(), all.begin() + 10});
    Lvx2Reader r;
    auto o = r.open(path);
    REQUIRE(!o);
    CHECK(o.error().kind == Lvx2Error::Kind::kBadFile);
    std::filesystem::remove(path);
  }
  SECTION("unsupported version")
  {
    const auto path = temp_file("badver.lvx2");
    auto copy = all;
    copy[16] = 1;  // lvx (v1) has a different layout
    write_file(path, copy);
    Lvx2Reader r;
    auto o = r.open(path);
    REQUIRE(!o);
    CHECK(o.error().kind == Lvx2Error::Kind::kBadFile);
    CHECK(o.error().detail == "unsupported version 1");
    std::filesystem::remove(path);
  }
  SECTION("device info block cut short")
  {
    const auto path = temp_file("cutinfo.lvx2");
    write_file(path, {all.begin(), all.begin() + 60});
    Lvx2Reader r;
    auto o = r.open(path);
    REQUIRE(!o);
    CHECK(o.error().kind == Lvx2Error::Kind::kBadFile);
    auto p = r.next_packet();  // a failed open leaves the reader closed
    REQUIRE(!p);
    CHECK(p.error().kind == Lvx2Error::Kind::kInvalidArgument);
    std::filesystem::remove(path);
  }
  SECTION("frame header with the wrong current_offset")
  {
    const auto path = temp_file("badcur.lvx2");
    auto copy = all;
    copy[kFrame0] ^= 1;
    write_file(path, copy);
    Lvx2Reader r;
    REQUIRE(r.open(path));
    auto p = r.next_packet();
    REQUIRE(!p);
    CHECK(p.error().kind == Lvx2Error::Kind::kBadFile);
    std::filesystem::remove(path);
  }
  SECTION("frame header whose next_offset points backwards")
  {
    const auto path = temp_file("badnext.lvx2");
    auto copy = all;
    for (std::size_t i = 0; i < 8; ++i) {
      copy[kFrame0 + 8 + i] = 0;
    }
    write_file(path, copy);
    Lvx2Reader r;
    REQUIRE(r.open(path));
    auto p = r.next_packet();
    REQUIRE(!p);
    CHECK(p.error().kind == Lvx2Error::Kind::kBadFile);
    std::filesystem::remove(path);
  }
  SECTION("package length that is not a multiple of the point size")
  {
    const auto path = temp_file("badlen.lvx2");
    auto copy = all;
    copy[kPackage0 + 18] = static_cast<char>(copy[kPackage0 + 18] + 1);
    write_file(path, copy);
    Lvx2Reader r;
    REQUIRE(r.open(path));
    auto p = r.next_packet();
    REQUIRE(!p);
    CHECK(p.error().kind == Lvx2Error::Kind::kBadFile);
    std::filesystem::remove(path);
  }
}

TEST_CASE("lvx2 player: open errors", "[lvx2]")
{
  const auto src = std::filesystem::path(LIVOX_MID360_TEST_DATA_DIR) / "mini.lvx2";
  Lvx2PlayOptions o;
  o.rate = -1;
  Lvx2Player negative(o);
  auto r = negative.open(src);
  REQUIRE(!r);
  CHECK(r.error().kind == Lvx2Error::Kind::kInvalidArgument);

  Lvx2Player missing;
  auto m = missing.open(temp_file("does_not_exist.lvx2"));
  REQUIRE(!m);
  CHECK(m.error().kind == Lvx2Error::Kind::kIo);
}

// ---- edge branches (issue #101) ----------------------------------------------------------

TEST_CASE("lvx2 writer rejects an unknown data_type", "[lvx2]")
{
  const auto path = temp_file("unknown_type.lvx2");
  const auto info = device_info();
  Lvx2Writer w;
  REQUIRE(w.open(path, std::span(&info, 1)));
  REQUIRE(w.write(0, make_packet(0, 0, 0).view));

  auto bad = make_packet(1, 0, 10 * kMs);
  bad.view.header.data_type = static_cast<DataType>(7);
  const auto r = w.write(0, bad.view);
  REQUIRE(!r);
  CHECK(r.error().kind == Lvx2Error::Kind::kUnsupportedDataType);
  CHECK(to_string(r.error()).find("unknown data_type") != std::string::npos);
  CHECK(w.stats().packets == 1);
  CHECK(w.stats().ignored == 0);

  // The writer stays usable and the refused packet left nothing in the file.
  REQUIRE(w.write(0, make_packet(1, 0, 20 * kMs).view));
  REQUIRE(w.close());
  Lvx2Reader reader;
  REQUIRE(reader.open(path));
  std::vector<std::vector<std::byte>> storage;
  const auto pk = read_all(reader, storage);
  REQUIRE(pk.size() == 2);
  CHECK(pk[0].udp_counter == 0);
  CHECK(pk[1].udp_counter == 1);
  CHECK(pk[1].timestamp_ns == 20 * kMs);
  CHECK_FALSE(reader.truncated());
  std::filesystem::remove(path);
}

TEST_CASE("lvx2 reader: file cut inside a package header", "[lvx2]")
{
  const auto path = temp_file("partial_header.lvx2");
  const auto info = device_info();
  Lvx2Writer w;
  REQUIRE(w.open(path, std::span(&info, 1)));
  // One frame (same 50 ms bin) with three packages of 23 + 4 * 14 bytes.
  for (std::uint16_t i = 0; i < 3; ++i) {
    REQUIRE(w.write(0, make_packet(i, 0, i * kMs).view));
  }
  REQUIRE(w.close());
  REQUIRE(w.stats().frames == 1);
  const auto size = std::filesystem::file_size(path);
  constexpr std::uintmax_t kPoints = 4 * 14;
  constexpr std::uintmax_t kHeader = 23;

  // `left` bytes of the last package header remain.
  for (const std::uintmax_t left : {std::uintmax_t{1}, std::uintmax_t{10}, kHeader - 1}) {
    CAPTURE(left);
    std::filesystem::resize_file(path, size - kPoints - kHeader + left);
    Lvx2Reader reader;
    REQUIRE(reader.open(path));
    std::vector<std::vector<std::byte>> storage;
    const auto pk = read_all(reader, storage);
    REQUIRE(pk.size() == 2);
    CHECK(pk[1].udp_counter == 1);
    CHECK(pk[1].points.size() == kPoints);
    CHECK(reader.truncated());
    // The end is sticky.
    const auto again = reader.next_packet();
    REQUIRE(again.has_value());
    CHECK_FALSE(again->has_value());
  }

  // Cut exactly between two packages: nothing is partial inside the frame data, but the
  // frame is shorter than its header says.
  std::filesystem::resize_file(path, size - kPoints - kHeader);
  Lvx2Reader reader;
  REQUIRE(reader.open(path));
  std::vector<std::vector<std::byte>> storage;
  CHECK(read_all(reader, storage).size() == 2);
  CHECK(reader.truncated());
  std::filesystem::remove(path);
}

TEST_CASE("lvx2 player: pacing edge cases", "[lvx2]")
{
  const auto path = temp_file("pacing.lvx2");
  const auto info = device_info();
  const auto record = [&](const std::vector<std::uint64_t> & times_ms) {
    Lvx2Writer w;
    REQUIRE(w.open(path, std::span(&info, 1)));
    std::uint16_t udp = 0;
    for (const std::uint64_t t : times_ms) {
      REQUIRE(w.write(0, make_packet(udp++, 0, t * kMs).view));
    }
    REQUIRE(w.close());
  };
  const auto elapsed = [](std::chrono::steady_clock::time_point t0) {
    return std::chrono::steady_clock::now() - t0;
  };

  SECTION("a timestamp that goes backwards restarts the pacing")
  {
    // An hour of recorded time before the jump back: only the 60 + 40 ms around it are
    // waited for.
    record({3'600'000, 3'600'030, 3'600'060, 0, 20, 40});
    Lvx2Player pl;  // rate 1
    REQUIRE(pl.open(path));
    std::vector<std::uint64_t> seen;
    pl.on_packet([&](const Lvx2Packet & p) { seen.push_back(p.timestamp_ns / kMs); });
    const auto t0 = std::chrono::steady_clock::now();
    const auto s = pl.run();
    const auto took = elapsed(t0);
    REQUIRE(s);
    CHECK(s->packets == 6);
    CHECK(s->loops == 1);
    CHECK(seen == std::vector<std::uint64_t>{3'600'000, 3'600'030, 3'600'060, 0, 20, 40});
    CHECK(took >= 90ms);
    CHECK(took < 2s);
  }
  SECTION("a stop request ends the pacing sleep")
  {
    record({0, 60'000});
    Lvx2Player pl;
    REQUIRE(pl.open(path));
    std::stop_source stop;
    std::size_t packets = 0;
    pl.on_packet([&](const Lvx2Packet &) { ++packets; });
    std::jthread stopper([&] {
      std::this_thread::sleep_for(150ms);
      stop.request_stop();
    });
    const auto t0 = std::chrono::steady_clock::now();
    const auto s = pl.run(stop.get_token());
    const auto took = elapsed(t0);
    REQUIRE(s);
    CHECK(took >= 140ms);
    CHECK(took < 2s);  // the sleep is sliced in 50 ms steps
    CHECK(s->packets == packets);
    CHECK(packets >= 1);
    CHECK(s->loops == 0);  // the pass was not completed
  }
  SECTION("a stop requested before run delivers nothing")
  {
    record({0, 10});
    Lvx2Player pl;
    REQUIRE(pl.open(path));
    std::stop_source stop;
    stop.request_stop();
    const auto s = pl.run(stop.get_token());
    REQUIRE(s);
    CHECK(s->packets == 0);
    CHECK(s->frames == 0);
    CHECK(s->loops == 0);
  }
  SECTION("the file disappears before the next loop")
  {
    record({0, 10, 20});
    Lvx2PlayOptions o;
    o.rate = 0;
    o.loop = true;
    Lvx2Player pl(o);
    REQUIRE(pl.open(path));
    std::size_t packets = 0;
    pl.on_packet([&](const Lvx2Packet &) {
      if (++packets == 3) {
        std::filesystem::remove(path);
      }
    });
    const auto s = pl.run();
    REQUIRE(!s);
    CHECK(s.error().kind == Lvx2Error::Kind::kIo);
    CHECK(packets == 3);
  }
  SECTION("the file is replaced by garbage before the next loop")
  {
    record({0, 10, 20});
    Lvx2PlayOptions o;
    o.rate = 0;
    o.loop = true;
    Lvx2Player pl(o);
    REQUIRE(pl.open(path));
    std::size_t packets = 0;
    pl.on_packet([&](const Lvx2Packet &) {
      if (++packets == 3) {
        std::filesystem::remove(path);
        std::ofstream(path, std::ios::binary) << "this is not an lvx2 file, not even close";
      }
    });
    const auto s = pl.run();
    REQUIRE(!s);
    CHECK(s.error().kind == Lvx2Error::Kind::kBadFile);
    CHECK(packets == 3);
  }
  std::filesystem::remove(path);
}

// SPDX-License-Identifier: Apache-2.0
// The synthetic captures in tests/fixtures/ (issue #10), written by tools/gen_fixtures.py in
// the shape of a Mid-360 with firmware 13.18.0244: every datagram must parse, and the fields
// measured on that device must read back as measured. See tests/fixtures/README.md.
#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "livox/mid360/mid360.hpp"
#include "pcap_reader.hpp"

using namespace livox::mid360;

namespace
{

constexpr std::array<std::uint8_t, 4> kLidarIp{192, 168, 1, 12};
constexpr std::array<std::uint8_t, 4> kHostIp{192, 168, 1, 50};
constexpr std::array<std::uint8_t, 4> kBroadcast{255, 255, 255, 255};
constexpr std::uint16_t kPortDiscovery = 56000;
constexpr std::uint16_t kPortCmd = 56100;
constexpr std::uint16_t kPortPush = 56200;
constexpr std::uint16_t kPortPcl = 56300;
constexpr std::uint16_t kPortImu = 56400;
constexpr std::uint16_t kPortLog = 56500;
constexpr std::uint16_t kPortDebugData = 60301;

std::vector<pcap_reader::Datagram> load(const char * name)
{
  const auto path =
    std::filesystem::path(LIVOX_MID360_TEST_FIXTURES_DIR) / (std::string(name) + ".pcap");
  auto datagrams = pcap_reader::read_udp(path);
  REQUIRE(datagrams.has_value());
  REQUIRE_FALSE(datagrams->empty());
  return std::move(*datagrams);
}

bool from_lidar(const pcap_reader::Datagram & d, std::uint16_t port)
{
  return d.src_ip == kLidarIp && d.src_port == port;
}

/// Every command frame the LiDAR sent from `port`, parsed.
std::vector<CommandFrameView> lidar_frames(
  const std::vector<pcap_reader::Datagram> & ds, std::uint16_t port)
{
  std::vector<CommandFrameView> out;
  for (const auto & d : ds) {
    if (from_lidar(d, port)) {
      auto f = parse_command_frame(d.bytes());
      REQUIRE(f.has_value());
      CHECK(f->header.sender_type == SenderType::kLidar);
      out.push_back(*f);
    }
  }
  return out;
}

/// Every command frame the host sent to `port`, parsed.
std::vector<CommandFrameView> host_frames(
  const std::vector<pcap_reader::Datagram> & ds, std::uint16_t port)
{
  std::vector<CommandFrameView> out;
  for (const auto & d : ds) {
    if (d.src_ip == kHostIp && d.dst_port == port) {
      auto f = parse_command_frame(d.bytes());
      REQUIRE(f.has_value());
      CHECK(f->header.sender_type == SenderType::kHost);
      CHECK(f->header.cmd_type == CmdType::kReq);
      out.push_back(*f);
    }
  }
  return out;
}

/// The pushes in `ds`, each with its frame header (for seq_num) and its key-value list.
std::vector<std::pair<CommandHeader, InfoPush>> pushes(
  const std::vector<pcap_reader::Datagram> & ds)
{
  std::vector<std::pair<CommandHeader, InfoPush>> out;
  for (const auto & f : lidar_frames(ds, kPortPush)) {
    REQUIRE(f.header.cmd_id == static_cast<std::uint16_t>(CmdId::kInfoPush));
    CHECK(f.header.cmd_type == CmdType::kReq);  // a push is sent as a request
    CHECK(f.data.size() == 401);                // the Mid-360's key set (#235)
    auto p = parse_info_push(f.data);
    REQUIRE(p.has_value());
    CHECK(p->values.size() == 30);
    out.emplace_back(f.header, std::move(*p));
  }
  return out;
}

WorkState work_state(const InfoPush & p)
{
  const auto v = find_key(p.values, Key::kCurWorkState);
  REQUIRE(v.has_value());
  const auto ws = key_traits<Key::kCurWorkState>::decode(*v);
  REQUIRE(ws.has_value());
  return *ws;
}

/// The point cloud / IMU packets the LiDAR sent from `port`, parsed (CRC checked when not 0).
std::vector<DataPacketView> data_packets(
  const std::vector<pcap_reader::Datagram> & ds, std::uint16_t port)
{
  std::vector<DataPacketView> out;
  for (const auto & d : ds) {
    if (from_lidar(d, port)) {
      auto pkt = parse_data_packet(d.bytes());
      REQUIRE(pkt.has_value());
      out.push_back(*pkt);
    }
  }
  return out;
}

struct FixturePoint
{
  bool zero_xyz;  ///< no coordinates: x = y = z = 0 (depth 0 for spherical)
  std::uint8_t reflectivity;
  std::uint8_t tag;
};

std::vector<FixturePoint> points(const DataPacketView & pkt)
{
  const std::size_t size = sample_size(pkt.header.data_type);
  std::size_t coord = 4;  // spherical: depth only
  if (pkt.header.data_type == DataType::kCartesian32) {
    coord = 12;
  } else if (pkt.header.data_type == DataType::kCartesian16) {
    coord = 6;
  }
  std::vector<FixturePoint> out;
  for (std::size_t i = 0; i < pkt.header.dot_num; ++i) {
    const auto s = pkt.data.subspan(i * size, size);
    const bool zero = std::all_of(
      s.begin(), s.begin() + static_cast<std::ptrdiff_t>(coord),
      [](std::byte b) { return b == std::byte{0}; });
    out.push_back(
      {zero, std::to_integer<std::uint8_t>(s[size - 2]),
       std::to_integer<std::uint8_t>(s[size - 1])});
  }
  return out;
}

void check_point_cloud_header(const DataPacketHeader & h)
{
  CHECK(h.version == 0);
  CHECK(h.dot_num == 96);
  CHECK(h.time_interval == 4750);
  CHECK(h.frame_cnt == 0);
  CHECK(h.time_type == TimeType::kNoSync);
  CHECK(h.crc32 == 0);  // a Mid-360 leaves it at 0 in point cloud packets (#219)
  CHECK(std::ranges::all_of(h.reserved, [](std::uint8_t b) { return b == 0; }));
}

}  // namespace

TEST_CASE("fixture discovery_setup: broadcast discovery, host setup, push", "[fixtures]")
{
  const auto ds = load("discovery_setup");

  const auto req = host_frames(ds, kPortDiscovery);
  REQUIRE(req.size() == 1);
  CHECK(req[0].header.cmd_id == static_cast<std::uint16_t>(CmdId::kDiscovery));
  CHECK(req[0].data.empty());

  const auto ack_it =
    std::ranges::find_if(ds, [](const auto & d) { return from_lidar(d, kPortDiscovery); });
  REQUIRE(ack_it != ds.end());
  CHECK(ack_it->dst_ip == kBroadcast);  // the ACK is broadcast, not sent to the host
  const auto ack_frame = parse_command_frame(ack_it->bytes());
  REQUIRE(ack_frame.has_value());
  REQUIRE(ack_frame->data.size() == 24);
  const auto ack = parse_discovery_ack(ack_frame->data);
  REQUIRE(ack.has_value());
  CHECK(ack->ret_code == RetCode::kSuccess);
  CHECK(ack->dev_type == 9);
  CHECK(ack->serial_number_view() == "FIXTURE0000001");
  CHECK(ack->lidar_ip == kLidarIp);
  CHECK(ack->cmd_port == kPortCmd);

  const auto acks = lidar_frames(ds, kPortCmd);
  REQUIRE(acks.size() == 1);
  CHECK(acks[0].header.cmd_type == CmdType::kAck);
  const auto cfg = parse_param_config_ack(acks[0].data);
  REQUIRE(cfg.has_value());
  CHECK(cfg->ret_code == RetCode::kSuccess);

  const auto ps = pushes(ds);
  REQUIRE(ps.size() == 2);
  const auto id = decode_identity(ps[0].second.values);
  CHECK(id.serial_number == "FIXTURE0000001");
  CHECK(id.product_info == "DevType:Mid-360 FmType:App FmVer:13180244 BuildTime:2025/04/01");
  CHECK(id.version_app.v == std::array<std::uint8_t, 4>{13, 18, 2, 44});
  CHECK(id.version_loader.v == std::array<std::uint8_t, 4>{13, 17, 99, 20});
  CHECK(id.version_hardware.v == std::array<std::uint8_t, 4>{0, 0, 0, 0});
  const auto fw = find_key(ps[0].second.values, Key::kFwType);
  REQUIRE(fw.has_value());
  CHECK(key_traits<Key::kFwType>::decode(*fw) == FwType::kApp);
  const auto hms = find_key(ps[0].second.values, Key::kHmsCode);
  REQUIRE(hms.has_value());
  CHECK((*key_traits<Key::kHmsCode>::decode(*hms))[0] == 0x04070002U);
  const auto fov = find_key(ps[0].second.values, Key::kFovCfg0);
  REQUIRE(fov.has_value());
  const auto window = key_traits<Key::kFovCfg0>::decode(*fov);
  REQUIRE(window.has_value());
  CHECK(window->yaw_start_deg == 0);
  CHECK(window->yaw_stop_deg == 0);
  CHECK(window->pitch_start_deg == -7);
  CHECK(window->pitch_stop_deg == 52);
  CHECK(work_state(ps[0].second) == WorkState::kSampling);
}

TEST_CASE("fixture inquire: unsupported keys make ret 0x20, the rest are answered", "[fixtures]")
{
  const auto ds = load("inquire");
  const auto acks = lidar_frames(ds, kPortCmd);
  REQUIRE(acks.size() == 3);

  // Every settings key: 0x0021 / 0x0026 / 0x0029 / 0x002B are left out, ret is 0x20.
  const auto settings = parse_param_inquire_ack(acks[0].data);
  REQUIRE(settings.has_value());
  CHECK(settings->ret_code == RetCode::kParamNotSupport);
  CHECK(settings->values.size() == 14);
  for (const auto key : {Key::kSpeedMode, Key::kTimeFilter, Key::kPcFreqMod, Key::kImuSensorCfg}) {
    CHECK_FALSE(find_key(settings->values, key).has_value());
  }
  CHECK(find_key(settings->values, Key::kPclDataType).has_value());

  // Every status key.
  const auto status = parse_param_inquire_ack(acks[1].data);
  REQUIRE(status.has_value());
  CHECK(status->ret_code == RetCode::kSuccess);
  CHECK(status->values.size() == 16);

  // A lone unsupported key: a 3-byte ACK with no values.
  CHECK(acks[2].data.size() == 3);
  const auto single = parse_param_inquire_ack(acks[2].data);
  REQUIRE(single.has_value());
  CHECK(single->ret_code == RetCode::kParamNotSupport);
  CHECK(single->values.empty());
}

TEST_CASE("fixture pcl_types: the three point cloud formats", "[fixtures]")
{
  const auto ds = load("pcl_types");
  const auto pkts = data_packets(ds, kPortPcl);
  REQUIRE(pkts.size() == 90);

  std::optional<std::uint16_t> prev_cnt;
  std::optional<std::uint64_t> prev_ts;
  bool wrapped = false;
  std::size_t no_return = 0;
  for (std::size_t i = 0; i < pkts.size(); ++i) {
    const auto & h = pkts[i].header;
    check_point_cloud_header(h);
    const auto expected_type = static_cast<DataType>(1 + i / 30);
    CHECK(h.data_type == expected_type);
    CHECK(h.length == 36 + 96 * sample_size(expected_type));
    if (prev_cnt) {
      CHECK(h.udp_cnt == static_cast<std::uint16_t>(*prev_cnt + 1));  // runs on across switches
      wrapped = wrapped || h.udp_cnt == 0;
      CHECK(h.timestamp_ns - *prev_ts == 480'000);
    }
    prev_cnt = h.udp_cnt;
    prev_ts = h.timestamp_ns;
    for (const auto & pt : points(pkts[i])) {
      if (pt.zero_xyz) {
        CHECK(pt.reflectivity == 0);  // no return
        ++no_return;
      }
    }
  }
  CHECK(wrapped);
  CHECK(no_return > 0);
  CHECK(pkts[0].header.length == 1380);
  CHECK(pkts[30].header.length == 804);
  CHECK(pkts[60].header.length == 996);

  // Each switch is ACKed and followed by a push carrying the new type.
  const auto ps = pushes(ds);
  REQUIRE(ps.size() == 3);
  constexpr std::array kTypes{DataType::kCartesian32, DataType::kCartesian16, DataType::kSpherical};
  for (std::size_t i = 0; i < ps.size(); ++i) {
    const auto v = find_key(ps[i].second.values, Key::kPclDataType);
    REQUIRE(v.has_value());
    CHECK(key_traits<Key::kPclDataType>::decode(*v) == kTypes.at(i));
  }
}

TEST_CASE("fixture imu: 200 Hz samples with a CRC, imu_sensor_cfg rejected", "[fixtures]")
{
  const auto ds = load("imu");
  const auto imu = data_packets(ds, kPortImu);
  REQUIRE(imu.size() == 20);
  for (std::size_t i = 0; i < imu.size(); ++i) {
    const auto & h = imu[i].header;
    CHECK(h.length == 60);
    CHECK(h.data_type == DataType::kImu);
    CHECK(h.dot_num == 1);
    CHECK(h.time_interval == 0);
    CHECK(h.frame_cnt == 0);
    CHECK(h.crc32 != 0);  // unlike the point cloud, IMU packets carry a CRC (checked on parse)
    if (i > 0) {
      CHECK(h.udp_cnt == static_cast<std::uint16_t>(imu[i - 1].header.udp_cnt + 1));
      CHECK(h.timestamp_ns - imu[i - 1].header.timestamp_ns == 5'000'000);
    }
  }
  for (const auto & pkt : data_packets(ds, kPortPcl)) {
    check_point_cloud_header(pkt.header);
  }

  const auto acks = lidar_frames(ds, kPortCmd);
  REQUIRE(acks.size() == 1);
  const auto cfg = parse_param_config_ack(acks[0].data);
  REQUIRE(cfg.has_value());
  CHECK(cfg->ret_code == RetCode::kParamNotSupport);
  CHECK(cfg->error_key == static_cast<std::uint16_t>(Key::kImuSensorCfg));
}

TEST_CASE(
  "fixture fov: cropped points stay in the packet, zeroed with reflectivity 60", "[fixtures]")
{
  const auto ds = load("fov");
  const auto pkts = data_packets(ds, kPortPcl);
  REQUIRE(pkts.size() == 30);
  const auto cropped = [](const FixturePoint & pt) {
    return pt.zero_xyz && pt.reflectivity == 60 && pt.tag == 0;
  };
  for (std::size_t i = 0; i < pkts.size(); ++i) {
    check_point_cloud_header(pkts[i].header);  // still 96 points per packet
    const auto pts = points(pkts[i]);
    const auto n = std::ranges::count_if(pts, cropped);
    if (i < 10) {
      CHECK(n == 0);  // FOV off
    } else if (i < 20) {
      CHECK(n > 0);  // pitch 0..59: the points at -5 deg are cropped
      CHECK(n < 96);
    } else {
      CHECK(n == 96);  // empty window
    }
  }

  const auto acks = lidar_frames(ds, kPortCmd);
  REQUIRE(acks.size() == 2);
  for (const auto & a : acks) {
    CHECK(parse_param_config_ack(a.data)->ret_code == RetCode::kSuccess);
  }
  const auto ps = pushes(ds);
  REQUIRE(ps.size() == 2);
  const auto en = find_key(ps[1].second.values, Key::kFovCfgEn);
  REQUIRE(en.has_value());
  CHECK(key_traits<Key::kFovCfgEn>::decode(*en)->fov0);
}

TEST_CASE("fixture firmware_log: file 7, ACK requested for the first nine chunks", "[fixtures]")
{
  const auto ds = load("firmware_log");
  const auto frames = lidar_frames(ds, kPortLog);
  std::vector<FirmwareLogPushView> chunks;
  std::vector<CommandFrameView> acks;
  for (const auto & f : frames) {
    if (f.header.cmd_id == static_cast<std::uint16_t>(CmdId::kPushLog)) {
      CHECK(f.header.cmd_type == CmdType::kReq);
      auto c = parse_firmware_log_push(f.data);
      REQUIRE(c.has_value());
      chunks.push_back(*c);
    } else {
      acks.push_back(f);
    }
  }
  REQUIRE(acks.size() == 2);  // 0x0301 on and off
  for (const auto & a : acks) {
    CHECK(a.header.cmd_id == static_cast<std::uint16_t>(CmdId::kCollectionLog));
    REQUIRE(a.data.size() == 1);
    CHECK(parse_simple_ack(a.data)->ret_code == RetCode::kSuccess);
  }
  const auto ctl = host_frames(ds, kPortLog);
  REQUIRE(ctl.size() == 2);
  CHECK(parse_firmware_log_control(ctl[0].data)->enable);
  CHECK_FALSE(parse_firmware_log_control(ctl[1].data)->enable);

  REQUIRE(chunks.size() == 14);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    const auto & h = chunks[i].header;
    CHECK(h.log_type == FirmwareLogType::kRealTime);
    CHECK(h.file_index == 7);
    CHECK(h.trans_index == i);
    CHECK(h.flags.ack_requested() == (i <= 8));
    CHECK(h.flags.file_begin() == (i == 0));
    CHECK_FALSE(h.flags.file_end());  // no end chunk when the log is turned off
    CHECK(chunks[i].data.size() == h.data_length);
    CHECK(chunks[i].data[0] == std::byte{0x55});
  }
  CHECK(chunks[0].header.data_length == 256);
}

TEST_CASE("fixture debug_data: 1114-byte datagrams from port 60301", "[fixtures]")
{
  const auto ds = load("debug_data");
  const auto ctl = host_frames(ds, kPortLog);
  REQUIRE(ctl.size() == 2);
  const auto on = parse_debug_data_control(ctl[0].data);
  REQUIRE(on.has_value());
  CHECK(on->enable);
  CHECK(on->host_ip == kHostIp);
  CHECK(on->host_port == kDefaultHostDebugDataPort);
  CHECK_FALSE(parse_debug_data_control(ctl[1].data)->enable);

  const auto acks = lidar_frames(ds, kPortLog);  // ACKed on the log port, not the command port
  REQUIRE(acks.size() == 2);
  for (const auto & a : acks) {
    CHECK(a.header.cmd_id == static_cast<std::uint16_t>(CmdId::kDebugDataControl));
    REQUIRE(a.data.size() == 1);
    CHECK(parse_simple_ack(a.data)->ret_code == RetCode::kSuccess);
  }

  const auto n =
    std::ranges::count_if(ds, [](const auto & d) { return from_lidar(d, kPortDebugData); });
  CHECK(n == 20);
  for (const auto & d : ds) {
    if (from_lidar(d, kPortDebugData)) {
      CHECK(d.dst_port == kDefaultHostDebugDataPort);
      CHECK(d.payload.size() == 1114);
    }
  }
}

TEST_CASE("fixture reboot: ERROR, silence, MOTORSTARTUP, READY, SAMPLING", "[fixtures]")
{
  const auto ds = load("reboot");
  const auto req = host_frames(ds, kPortCmd);
  REQUIRE(req.size() == 1);
  CHECK(req[0].header.cmd_id == static_cast<std::uint16_t>(CmdId::kReboot));
  CHECK(req[0].data.size() == 2);
  const auto acks = lidar_frames(ds, kPortCmd);
  REQUIRE(acks.size() == 1);
  REQUIRE(acks[0].data.size() == 1);
  CHECK(parse_simple_ack(acks[0].data)->ret_code == RetCode::kSuccess);
  const auto ack_us =
    std::ranges::find_if(ds, [](const auto & d) { return from_lidar(d, kPortCmd); })->time_us;
  // Microseconds since the reboot ACK; data packets before it come out negative.
  const auto since_ack = [ack_us](const pcap_reader::Datagram & d) {
    return static_cast<std::int64_t>(d.time_us) - static_cast<std::int64_t>(ack_us);
  };

  std::vector<std::pair<std::int64_t, WorkState>> states;
  for (const auto & d : ds) {
    if (from_lidar(d, kPortPush)) {
      const auto f = parse_command_frame(d.bytes());
      REQUIRE(f.has_value());
      states.emplace_back(since_ack(d), work_state(*parse_info_push(f->data)));
    }
  }
  const std::vector<WorkState> expected{WorkState::kSampling,     WorkState::kError,
                                        WorkState::kError,        WorkState::kMotorStartup,
                                        WorkState::kMotorStartup, WorkState::kMotorStartup,
                                        WorkState::kMotorStartup, WorkState::kMotorStartup,
                                        WorkState::kReady,        WorkState::kSampling};
  REQUIRE(states.size() == expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    CHECK(states[i].second == expected[i]);
  }
  CHECK(states[1].first > 1'200'000);                    // still running for about 1.25 s
  CHECK(states[3].first - states[2].first > 7'000'000);  // silent while powered down
  CHECK(states[8].first > 13'000'000);                   // READY at about +13.5 s

  // The push counter restarts at power-on.
  const auto ps = pushes(ds);
  CHECK(ps[3].first.seq_num < ps[2].first.seq_num);

  // The data stops before the first ERROR push and resumes from udp_cnt 0 after SAMPLING.
  std::vector<std::pair<std::int64_t, DataPacketHeader>> pcl;
  for (const auto & d : ds) {
    if (from_lidar(d, kPortPcl)) {
      const auto pkt = parse_data_packet(d.bytes());
      REQUIRE(pkt.has_value());
      check_point_cloud_header(pkt->header);
      pcl.emplace_back(since_ack(d), pkt->header);
    }
  }
  REQUIRE(pcl.size() == 30);
  const auto after =
    std::ranges::find_if(pcl, [&](const auto & p) { return p.first > states[9].first; });
  REQUIRE(after != pcl.end());
  CHECK(std::ranges::none_of(
    pcl, [&](const auto & p) { return p.first > states[1].first && p.first < states[9].first; }));
  CHECK(after->second.udp_cnt == 0);
  CHECK(after->second.timestamp_ns < std::prev(after)->second.timestamp_ns);
}

// SPDX-License-Identifier: Apache-2.0
// End-to-end smoke test against tools/livox_mid360_sim.py: discovery -> configure host
// endpoints -> wait for SAMPLING -> receive point-cloud and IMU packets -> quit.
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"
#include "livox/mid360/transport.hpp"
#include "sim_process.hpp"

using namespace livox::mid360;
using namespace std::chrono_literals;

namespace {

/// Send a request on `sock` and wait for the matching ACK. Returns the ACK payload copy.
std::optional<std::vector<std::byte>> request(const UdpSocket& sock, const Endpoint& to,
                                              std::uint32_t seq, std::uint16_t cmd_id,
                                              std::span<const std::byte> payload) {
  const auto frame = build_command_frame({.seq_num = seq, .cmd_id = cmd_id, .data = payload});
  REQUIRE(frame.has_value());
  REQUIRE(sock.send_to(*frame, to).has_value());
  auto poller = Poller::create();
  REQUIRE(poller.has_value());
  REQUIRE(poller->add(sock, 1).has_value());
  std::array<std::byte, kMaxDatagramSize> buf{};
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto ev = poller->wait(500ms);
    REQUIRE(ev.has_value());
    if (ev->empty()) continue;
    while (true) {
      const auto d = sock.recv_one(buf);
      if (!d) break;
      const auto view = parse_command_frame(d->data);
      if (!view) continue;
      if (view->header.cmd_type == CmdType::kAck && view->header.seq_num == seq &&
          view->header.cmd_id == cmd_id) {
        return std::vector<std::byte>(view->data.begin(), view->data.end());
      }
    }
  }
  return std::nullopt;
}

}  // namespace

TEST_CASE("Simulator smoke: discovery, configure, stream, quit", "[sim][smoke]") {
  std::string err;
  auto sim = SimProcess::start(err);
  if (!sim) SKIP("simulator unavailable: " << err);

  // Host sockets on free loopback ports.
  SocketOptions pcl_opts;
  pcl_opts.recv_buffer_bytes = 1 << 20;
  const auto cmd_sock = UdpSocket::open(Endpoint::loopback(0));
  const auto pcl_sock = UdpSocket::open(Endpoint::loopback(0), pcl_opts);
  const auto imu_sock = UdpSocket::open(Endpoint::loopback(0));
  REQUIRE(cmd_sock.has_value());
  REQUIRE(pcl_sock.has_value());
  REQUIRE(imu_sock.has_value());

  // Discovery is unicast to the simulator's discovery port.
  std::uint32_t seq = 1;
  const auto disc = request(*cmd_sock, Endpoint::loopback(sim->ports().discovery), seq++,
                            static_cast<std::uint16_t>(CmdId::kDiscovery), {});
  REQUIRE(disc.has_value());
  const auto ack = parse_discovery_ack(*disc);
  REQUIRE(ack.has_value());
  CHECK(ack->ret_code == RetCode::kSuccess);
  CHECK(ack->serial_number_view() == sim->sn());
  CHECK(ack->cmd_port == sim->ports().cmd);
  const Endpoint lidar_cmd{ack->lidar_ip, ack->cmd_port};
  CHECK(ip_to_string(lidar_cmd.ip) == sim->ip());

  // Point the LiDAR at our sockets and enable the IMU.
  const auto pcl_cfg = encode_host_ip_config({.ip = {127, 0, 0, 1},
                                              .dst_port = pcl_sock->local_endpoint().port,
                                              .src_port = kPointCloudPort});
  const auto imu_cfg = encode_host_ip_config(
      {.ip = {127, 0, 0, 1}, .dst_port = imu_sock->local_endpoint().port, .src_port = kImuPort});
  const auto imu_en = encode_u8(1);
  const KeyValue kvs[] = {
      {static_cast<std::uint16_t>(Key::kPointCloudHostIpCfg), pcl_cfg},
      {static_cast<std::uint16_t>(Key::kImuHostIpCfg), imu_cfg},
      {static_cast<std::uint16_t>(Key::kImuDataEn), imu_en},
  };
  const auto cfg_ack =
      request(*cmd_sock, lidar_cmd, seq++, static_cast<std::uint16_t>(CmdId::kParamConfig),
              encode_param_config_request(kvs));
  REQUIRE(cfg_ack.has_value());
  const auto cfg = parse_param_config_ack(*cfg_ack);
  REQUIRE(cfg.has_value());
  CHECK(cfg->ret_code == RetCode::kSuccess);

  // Poll cur_work_state until SAMPLING.
  const std::uint16_t keys[] = {static_cast<std::uint16_t>(Key::kCurWorkState)};
  const auto inquire = encode_param_inquire_request(keys);
  WorkState state = WorkState::kMotorStartup;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (state != WorkState::kSampling && std::chrono::steady_clock::now() < deadline) {
    const auto r = request(*cmd_sock, lidar_cmd, seq++,
                           static_cast<std::uint16_t>(CmdId::kParamInquire), inquire);
    REQUIRE(r.has_value());
    const auto inq = parse_param_inquire_ack(*r);
    REQUIRE(inq.has_value());
    REQUIRE(inq->ret_code == RetCode::kSuccess);
    const auto v = find_key(inq->values, Key::kCurWorkState);
    REQUIRE(v.has_value());
    state = decode_work_state(*v).value_or(WorkState::kMotorStartup);
    if (state != WorkState::kSampling) std::this_thread::sleep_for(50ms);
  }
  REQUIRE(state == WorkState::kSampling);

  // Receive packets on both data sockets.
  auto poller = Poller::create();
  REQUIRE(poller.has_value());
  REQUIRE(poller->add(*pcl_sock, 1).has_value());
  REQUIRE(poller->add(*imu_sock, 2).has_value());
  std::array<std::array<std::byte, kMaxDatagramSize>, 32> storage{};
  std::array<Datagram, 32> batch{};
  std::size_t pcl_packets = 0, imu_packets = 0, points = 0;
  std::optional<std::uint16_t> last_udp_cnt;
  bool udp_cnt_monotonic = true;
  const auto rx_deadline = std::chrono::steady_clock::now() + 5s;
  while ((pcl_packets < 200 || imu_packets < 10) &&
         std::chrono::steady_clock::now() < rx_deadline) {
    const auto ev = poller->wait(500ms);
    REQUIRE(ev.has_value());
    for (const ReadyEvent& e : *ev) {
      const UdpSocket& s = e.tag == 1 ? *pcl_sock : *imu_sock;
      for (std::size_t i = 0; i < batch.size(); ++i) batch[i].data = storage[i];
      const auto n = s.recv_batch(batch);
      REQUIRE(n.has_value());
      for (std::size_t i = 0; i < *n; ++i) {
        const auto pkt = parse_data_packet(batch[i].data);
        REQUIRE(pkt.has_value());
        if (e.tag == 1) {
          CHECK(pkt->header.data_type == DataType::kCartesian32);
          CHECK(pkt->header.dot_num == kPointsPerPacket);
          if (last_udp_cnt &&
              pkt->header.udp_cnt != static_cast<std::uint16_t>(*last_udp_cnt + 1)) {
            udp_cnt_monotonic = false;
          }
          last_udp_cnt = pkt->header.udp_cnt;
          points += pkt->header.dot_num;
          ++pcl_packets;
        } else {
          CHECK(pkt->header.data_type == DataType::kImu);
          CHECK(pkt->header.dot_num == 1);
          ++imu_packets;
        }
      }
    }
  }
  CHECK(pcl_packets >= 200);
  CHECK(imu_packets >= 10);
  CHECK(points == pcl_packets * kPointsPerPacket);
  CHECK(udp_cnt_monotonic);

  CHECK(sim->stop() == 0);
}

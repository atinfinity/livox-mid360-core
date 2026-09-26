// SPDX-License-Identifier: Apache-2.0
// Session ACK matching and typed post-processing without a socket.
//
// Input layout: [seq u32][cmd_id u16][from ip 4][from port u16][lidar ip 4][datagram...]
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

#include "fuzz_check.hpp"
#include "livox/mid360/protocol.hpp"
#include "session_detail.hpp"

namespace
{
constexpr std::size_t kPrefix = 16;

template <typename T>
T load(const std::uint8_t * p)
{
  T v{};
  std::memcpy(&v, p, sizeof(T));
  return v;
}

bool inside(std::span<const std::byte> view, const std::vector<std::byte> & owner)
{
  if (view.empty()) return true;
  const auto * b = owner.data();
  return view.data() >= b && view.data() + view.size() <= b + owner.size();
}
}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  using namespace livox::mid360;
  if (size < kPrefix) return 0;
  const auto seq = load<std::uint32_t>(data);
  const auto cmd_id = load<std::uint16_t>(data + 4);
  Endpoint from;
  std::memcpy(from.ip.data(), data + 6, 4);
  from.port = load<std::uint16_t>(data + 10);
  Ipv4 lidar_ip;
  std::memcpy(lidar_ip.data(), data + 12, 4);
  const std::span<const std::byte> datagram{
    reinterpret_cast<const std::byte *>(data + kPrefix), size - kPrefix};

  // -- match_ack ---------------------------------------------------------
  const auto parsed = parse_command_frame(datagram);
  const auto m = detail::match_ack(datagram, from, seq, cmd_id, lidar_ip);
  if (m) {
    fuzz::require(parsed.has_value());
    fuzz::require(m->header.seq_num == seq && m->header.cmd_id == cmd_id);
    fuzz::require(m->header.cmd_type == CmdType::kAck && from.ip == lidar_ip);
    fuzz::require(m->data.size() <= kCommandDataMaxSize);
  } else {
    fuzz::require((m.error() == detail::AckMismatch::kBadFrame) == !parsed.has_value());
  }

  // -- typed conversions on the payload, matched or not ------------------
  RawAck raw;
  raw.cmd_id = cmd_id;
  raw.seq_num = seq;
  raw.data.assign(datagram.begin(), datagram.end());

  if (const auto r = detail::to_discovery_ack(raw)) {
    fuzz::require(r->ret_code == RetCode::kSuccess);
    fuzz::require(r->serial_number_view().size() <= r->serial_number.size());
  } else {
    fuzz::require(r.error().cmd_id == cmd_id);
  }
  if (const auto r = detail::to_config_ack(raw)) {
    fuzz::require(r->ret_code == RetCode::kSuccess || r->ret_code == RetCode::kParamRebootEffect);
  } else if (r.error().kind == SessionErrorKind::kLidarRejected) {
    fuzz::require(
      r.error().ret_code != RetCode::kSuccess && r.error().ret_code != RetCode::kParamRebootEffect);
  } else {
    fuzz::require(r.error().kind == SessionErrorKind::kBadResponse);
  }
  (void)detail::to_simple_ack(raw);

  RawAck copy = raw;
  if (auto r = detail::to_inquire_result(std::move(copy))) {
    fuzz::require(r->ret_code == RetCode::kSuccess);
    fuzz::require(r->raw == raw.data);
    for (const auto & kv : r->values) fuzz::require(inside(kv.value, r->raw));
    const auto ws = detail::to_work_state(*r);
    const auto v = r->get(Key::kCurWorkState);
    fuzz::require(v.has_value() || !ws.has_value());
    // Move the result: views must survive because raw is owned by the vector.
    const InquireResult moved = std::move(*r);
    for (const auto & kv : moved.values) fuzz::require(inside(kv.value, moved.raw));
  } else {
    fuzz::require(
      r.error().kind == SessionErrorKind::kBadResponse ||
      r.error().kind == SessionErrorKind::kLidarRejected);
  }
  return 0;
}

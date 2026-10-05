// SPDX-License-Identifier: Apache-2.0
// Minimal classic-pcap reader for the tests: microsecond, little-endian files with Ethernet
// framing and IPv4/UDP datagrams, as tools/gen_fixtures.py and tools/gen_replay_pcap.py write
// them. Anything else (other link types, non-UDP frames) is skipped.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <vector>

namespace pcap_reader
{

struct Datagram
{
  std::uint64_t time_us = 0;  ///< capture time since the epoch
  std::array<std::uint8_t, 4> src_ip{};
  std::uint16_t src_port = 0;
  std::array<std::uint8_t, 4> dst_ip{};
  std::uint16_t dst_port = 0;
  std::vector<std::byte> payload;

  [[nodiscard]] std::span<const std::byte> bytes() const noexcept { return payload; }
};

namespace detail
{
inline std::uint32_t le32(const std::vector<std::uint8_t> & b, std::size_t o)
{
  return static_cast<std::uint32_t>(b[o]) | (static_cast<std::uint32_t>(b[o + 1]) << 8U) |
         (static_cast<std::uint32_t>(b[o + 2]) << 16U) |
         (static_cast<std::uint32_t>(b[o + 3]) << 24U);
}
inline std::uint16_t be16(const std::vector<std::uint8_t> & b, std::size_t o)
{
  return static_cast<std::uint16_t>((b[o] << 8U) | b[o + 1]);
}
}  // namespace detail

/// Every UDP datagram of `path` in file order; nullopt if the file is not a classic pcap.
inline std::optional<std::vector<Datagram>> read_udp(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return std::nullopt;
  }
  const std::vector<std::uint8_t> b(
    (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  constexpr std::size_t kGlobal = 24;
  constexpr std::size_t kRecord = 16;
  constexpr std::uint32_t kEthernet = 1;
  if (b.size() < kGlobal || detail::le32(b, 0) != 0xA1B2C3D4U || detail::le32(b, 20) != kEthernet) {
    return std::nullopt;
  }
  std::vector<Datagram> out;
  std::size_t off = kGlobal;
  while (off + kRecord <= b.size()) {
    const std::uint64_t sec = detail::le32(b, off);
    const std::uint64_t usec = detail::le32(b, off + 4);
    const std::size_t caplen = detail::le32(b, off + 8);
    const std::size_t frame = off + kRecord;
    off = frame + caplen;
    if (off > b.size()) {
      return std::nullopt;  // truncated record
    }
    constexpr std::size_t kEth = 14;
    if (caplen < kEth + 20 + 8 || detail::be16(b, frame + 12) != 0x0800) {
      continue;
    }
    const std::size_t ip = frame + kEth;
    const std::size_t ihl = static_cast<std::size_t>(b[ip] & 0x0FU) * 4;
    if (b[ip + 9] != 17 || ip + ihl + 8 > off) {
      continue;
    }
    const std::size_t udp = ip + ihl;
    const std::size_t udp_len = detail::be16(b, udp + 4);
    if (udp_len < 8 || udp + udp_len > off) {
      continue;
    }
    Datagram d;
    d.time_us = sec * 1'000'000 + usec;
    for (std::size_t i = 0; i < 4; ++i) {
      d.src_ip[i] = b[ip + 12 + i];
      d.dst_ip[i] = b[ip + 16 + i];
    }
    d.src_port = detail::be16(b, udp);
    d.dst_port = detail::be16(b, udp + 2);
    for (std::size_t i = udp + 8; i < udp + udp_len; ++i) {
      d.payload.push_back(static_cast<std::byte>(b[i]));
    }
    out.push_back(std::move(d));
  }
  return out;
}

}  // namespace pcap_reader

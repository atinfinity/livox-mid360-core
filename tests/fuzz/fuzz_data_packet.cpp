// SPDX-License-Identifier: Apache-2.0
#include <cstddef>
#include <cstdint>
#include <span>

#include "livox/mid360/protocol.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  using namespace livox::mid360;
  const std::span<const std::byte> in{reinterpret_cast<const std::byte*>(data), size};
  // Without CRC so the fuzzer reaches the decoders.
  if (auto p = parse_data_packet(in, /*verify_crc=*/false)) {
    (void)decode_all_cartesian32(*p);
    (void)decode_all_cartesian16(*p);
    (void)decode_all_spherical(*p);
    (void)decode_all_imu(*p);
    for (std::size_t i = 0; i < p->header.dot_num; ++i) (void)sample_timestamp_ns(p->header, i);
  }
  (void)parse_data_packet(in, true);
  return 0;
}

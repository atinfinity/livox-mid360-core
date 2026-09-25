// SPDX-License-Identifier: Apache-2.0
#include <cstddef>
#include <cstdint>
#include <span>

#include "livox/mid360/protocol.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  using namespace livox::mid360;
  const std::span<const std::byte> in{reinterpret_cast<const std::byte*>(data), size};
  if (auto f = parse_command_frame(in)) {
    (void)parse_discovery_ack(f->data);
    (void)parse_param_config_ack(f->data);
    (void)parse_param_inquire_ack(f->data);
    (void)parse_info_push(f->data);
    (void)parse_simple_ack(f->data);
  }
  return 0;
}

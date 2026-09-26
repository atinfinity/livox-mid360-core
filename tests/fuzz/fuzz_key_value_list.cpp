// SPDX-License-Identifier: Apache-2.0
#include <cstddef>
#include <cstdint>
#include <span>

#include "livox/mid360/keys.hpp"
#include "livox/mid360/protocol.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  using namespace livox::mid360;
  if (size < 1) return 0;
  const std::size_t key_num = data[0];
  const std::span<const std::byte> in{reinterpret_cast<const std::byte *>(data + 1), size - 1};
  if (auto kvs = parse_key_value_list(in, key_num)) {
    for (const auto & kv : *kvs) {
      (void)decode_u8(kv.value);
      (void)decode_host_ip_config(kv.value);
      (void)decode_lidar_ip_config(kv.value);
      (void)decode_install_attitude(kv.value);
      (void)decode_fov_config(kv.value);
      (void)decode_imu_sensor_config(kv.value);
      (void)decode_work_state(kv.value);
      (void)decode_diag_status(kv.value);
      (void)decode_hms_codes(kv.value);
      (void)decode_string(kv.value);
    }
  }
  return 0;
}

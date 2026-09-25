// SPDX-License-Identifier: Apache-2.0
// Little-endian byte helpers used by the protocol layer. Header-only, no I/O.
#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

namespace livox::mid360::bytes {

/// Reads a little-endian trivially-copyable value from `in` at byte `offset`.
/// The caller guarantees `offset + sizeof(T) <= in.size()`.
template <typename T>
  requires std::is_trivially_copyable_v<T>
[[nodiscard]] inline T read_le(std::span<const std::byte> in, std::size_t offset) noexcept {
  T v{};
  std::memcpy(&v, in.data() + offset, sizeof(T));
  if constexpr (std::endian::native == std::endian::big && std::is_integral_v<T> &&
                sizeof(T) > 1) {
    v = std::byteswap(v);
  }
  return v;
}

/// Writes `v` little-endian into `out` at byte `offset`.
/// The caller guarantees `offset + sizeof(T) <= out.size()`.
template <typename T>
  requires std::is_trivially_copyable_v<T>
inline void write_le(std::span<std::byte> out, std::size_t offset, T v) noexcept {
  if constexpr (std::endian::native == std::endian::big && std::is_integral_v<T> &&
                sizeof(T) > 1) {
    v = std::byteswap(v);
  }
  std::memcpy(out.data() + offset, &v, sizeof(T));
}

/// Reinterprets a contiguous range of trivially-copyable objects as bytes.
template <typename T>
  requires std::is_trivially_copyable_v<T>
[[nodiscard]] inline std::span<const std::byte> as_bytes(std::span<const T> s) noexcept {
  return std::as_bytes(s);
}

[[nodiscard]] inline std::span<const std::byte> as_bytes(const char* s, std::size_t n) noexcept {
  return {reinterpret_cast<const std::byte*>(s), n};
}

}  // namespace livox::mid360::bytes

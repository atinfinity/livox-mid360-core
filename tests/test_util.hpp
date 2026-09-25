// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstddef>
#include <cstring>
#include <span>
#include <string_view>
#include <vector>

inline std::vector<std::byte> bytes_of(std::string_view s) {
  std::vector<std::byte> v(s.size());
  std::memcpy(v.data(), s.data(), s.size());
  return v;
}
inline std::vector<std::byte> bytes_of(std::initializer_list<unsigned> il) {
  std::vector<std::byte> v;
  for (auto u : il) v.push_back(static_cast<std::byte>(u));
  return v;
}
inline std::span<const std::byte> span_of(const unsigned char* p, std::size_t n) {
  return {reinterpret_cast<const std::byte*>(p), n};
}

// SPDX-License-Identifier: Apache-2.0
// lvx2 codec (issue #35). Layout per "LVX2 Specifications 2023.06 v1.0", see docs/lvx2.md.
#include "livox/mid360/lvx2.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <format>
#include <numbers>
#include <thread>
#include <utility>

#include "frame_assembler.hpp"
#include "livox/mid360/bytes.hpp"

namespace livox::mid360
{
namespace
{
using bytes::read_le;
using bytes::write_le;

constexpr std::size_t kPublicHeaderSize = 24;
constexpr std::size_t kPrivateHeaderSize = 5;
constexpr std::size_t kFileHeaderSize = 29;  ///< public + private header
static_assert(kFileHeaderSize == kPublicHeaderSize + kPrivateHeaderSize);
constexpr std::size_t kDeviceInfoSize = 63;
constexpr std::size_t kFrameHeaderSize = 24;
constexpr std::size_t kPackageHeaderSize = 27;
constexpr std::uint32_t kMagic = 0xAC0EA767;
constexpr std::array<char, 16> kSignature = {'l', 'i', 'v', 'o', 'x', '_', 't', 'e',
                                             'c', 'h', 0,   0,   0,   0,   0,   0};
constexpr std::uint8_t kTypeCartesian32 = 1;
constexpr std::uint8_t kTypeCartesian16 = 2;

Lvx2Error io_error(std::string what)
{
  return Lvx2Error{Lvx2Error::Kind::kIo, errno, std::move(what)};
}

Lvx2Error bad_file(std::string what)
{
  return Lvx2Error{Lvx2Error::Kind::kBadFile, 0, std::move(what)};
}

Lvx2Error invalid_argument(std::string what)
{
  return Lvx2Error{Lvx2Error::Kind::kInvalidArgument, 0, std::move(what)};
}

struct FileCloser
{
  void operator()(std::FILE * f) const noexcept
  {
    if (f != nullptr) {
      (void)std::fclose(f);
    }
  }
};
using FilePtr = std::unique_ptr<std::FILE, FileCloser>;

void put_fixed(std::span<std::byte> out, std::size_t offset, std::string_view s, std::size_t n)
{
  std::memset(out.data() + offset, 0, n);
  std::memcpy(out.data() + offset, s.data(), std::min(n, s.size()));
}

std::string get_fixed(std::span<const std::byte> in, std::size_t offset, std::size_t n)
{
  const auto * p = reinterpret_cast<const char *>(in.data() + offset);
  return {p, ::strnlen(p, n)};
}

/// Spherical → Cartesian32 sample (same math as FrameAssembler's convert_point, in mm).
void spherical_to_cartesian32(const DataPacketView & pkt, std::size_t i, std::span<std::byte> out)
{
  const auto p = decode_spherical(pkt, i);
  constexpr double kCentidegToRad = std::numbers::pi / 18000.0;
  const auto d = static_cast<double>(p.depth_mm);
  const double theta = static_cast<double>(p.theta_centideg) * kCentidegToRad;
  const double phi = static_cast<double>(p.phi_centideg) * kCentidegToRad;
  const double st = std::sin(theta);
  write_le<std::int32_t>(out, 0, static_cast<std::int32_t>(std::lround(d * st * std::cos(phi))));
  write_le<std::int32_t>(out, 4, static_cast<std::int32_t>(std::lround(d * st * std::sin(phi))));
  write_le<std::int32_t>(out, 8, static_cast<std::int32_t>(std::lround(d * std::cos(theta))));
  out[12] = static_cast<std::byte>(p.reflectivity);
  out[13] = static_cast<std::byte>(p.tag);
}
}  // namespace

// ---------------------------------------------------------------------------
// Errors

std::string_view to_string(Lvx2Error::Kind kind) noexcept
{
  switch (kind) {
    case Lvx2Error::Kind::kIo:
      return "io";
    case Lvx2Error::Kind::kInvalidArgument:
      return "invalid_argument";
    case Lvx2Error::Kind::kUnsupportedDataType:
      return "unsupported_data_type";
    case Lvx2Error::Kind::kBadFile:
      return "bad_file";
  }
  return "unknown";
}

std::string to_string(const Lvx2Error & err)
{
  std::string s(to_string(err.kind));
  if (!err.detail.empty()) {
    s += ": " + err.detail;
  }
  if (err.kind == Lvx2Error::Kind::kIo && err.errno_value != 0) {
    s += std::format(" ({})", std::strerror(err.errno_value));
  }
  return s;
}

DataPacketView Lvx2Packet::to_data_packet_view() const noexcept
{
  DataPacketHeader h;
  h.version = 0;
  h.length = static_cast<std::uint16_t>(kDataPacketHeaderSize + points.size());
  h.time_interval = 0;
  h.dot_num = static_cast<std::uint16_t>(points.size() / sample_size(data_type));
  h.udp_cnt = udp_counter;
  h.frame_cnt = frame_counter;
  h.data_type = data_type;
  h.time_type = static_cast<TimeType>(timestamp_type);  // NOLINT: recorded value, not validated
  h.crc32 = 0;
  h.timestamp_ns = timestamp_ns;
  return DataPacketView{h, points};
}

// ---------------------------------------------------------------------------
// Writer

struct Lvx2Writer::Impl
{
  FilePtr file;
  std::vector<std::uint32_t> lidar_ids;
  std::uint32_t frame_duration_ms = 50;
  Stats stats;
  std::vector<std::byte> frame;  ///< packages of the open frame (after its 24-byte header)
  std::optional<std::uint64_t> frame_bin;
  std::uint64_t frame_index = 0;
  std::uint64_t offset = 0;  ///< file position of the open frame's header
  std::vector<std::byte> scratch;

  [[nodiscard]] std::expected<void, Lvx2Error> write_bytes(std::span<const std::byte> b) const
  {
    if (b.empty()) {
      return {};
    }
    if (std::fwrite(b.data(), 1, b.size(), file.get()) != b.size()) {
      return std::unexpected(io_error("write"));
    }
    return {};
  }

  std::expected<void, Lvx2Error> flush_frame()
  {
    if (frame.empty()) {
      return {};
    }
    std::array<std::byte, kFrameHeaderSize> h{};
    const std::uint64_t next = offset + kFrameHeaderSize + frame.size();
    write_le<std::uint64_t>(h, 0, offset);
    write_le<std::uint64_t>(h, 8, next);
    write_le<std::uint64_t>(h, 16, frame_index);
    if (auto r = write_bytes(h); !r) {
      return r;
    }
    if (auto r = write_bytes(frame); !r) {
      return r;
    }
    offset = next;
    stats.bytes = next;
    ++stats.frames;
    ++frame_index;
    frame.clear();
    frame_bin.reset();
    return {};
  }
};

Lvx2Writer::Lvx2Writer() : impl_(std::make_unique<Impl>()) {}
Lvx2Writer::~Lvx2Writer()
{
  if (impl_) {
    [[maybe_unused]] const auto r = close();
  }
}
Lvx2Writer::Lvx2Writer(Lvx2Writer &&) noexcept = default;
Lvx2Writer & Lvx2Writer::operator=(Lvx2Writer &&) noexcept = default;

std::expected<void, Lvx2Error> Lvx2Writer::open(
  const std::filesystem::path & path, std::span<const Lvx2DeviceInfo> devices,
  std::uint32_t frame_duration_ms)
{
  if (impl_->file) {
    return std::unexpected(invalid_argument("already open"));
  }
  if (devices.empty() || devices.size() > 255) {
    return std::unexpected(invalid_argument("device count must be 1..255"));
  }
  if (frame_duration_ms == 0) {
    return std::unexpected(invalid_argument("frame_duration_ms must be > 0"));
  }
  FilePtr f(std::fopen(path.c_str(), "wb"));
  if (!f) {
    return std::unexpected(io_error("open " + path.string()));
  }
  std::vector<std::byte> hdr(
    kPublicHeaderSize + kPrivateHeaderSize + kDeviceInfoSize * devices.size());
  std::memcpy(hdr.data(), kSignature.data(), kSignature.size());
  hdr[16] = std::byte{2};
  write_le<std::uint32_t>(hdr, 20, kMagic);
  write_le<std::uint32_t>(hdr, 24, frame_duration_ms);
  hdr[28] = static_cast<std::byte>(devices.size());
  std::size_t off = kFileHeaderSize;
  impl_->lidar_ids.clear();
  for (const auto & d : devices) {
    put_fixed(hdr, off, d.lidar_sn, 16);
    put_fixed(hdr, off + 16, d.hub_sn, 16);
    write_le<std::uint32_t>(hdr, off + 32, d.lidar_id);
    hdr[off + 36] = static_cast<std::byte>(d.lidar_type);
    hdr[off + 37] = static_cast<std::byte>(d.device_type);
    hdr[off + 38] = static_cast<std::byte>(d.extrinsic_enable ? 1 : 0);
    write_le<float>(hdr, off + 39, d.roll_deg);
    write_le<float>(hdr, off + 43, d.pitch_deg);
    write_le<float>(hdr, off + 47, d.yaw_deg);
    write_le<float>(hdr, off + 51, d.x_m);
    write_le<float>(hdr, off + 55, d.y_m);
    write_le<float>(hdr, off + 59, d.z_m);
    off += kDeviceInfoSize;
    impl_->lidar_ids.push_back(d.lidar_id);
  }
  impl_->file = std::move(f);
  impl_->frame_duration_ms = frame_duration_ms;
  impl_->stats = {};
  impl_->frame.clear();
  impl_->frame_bin.reset();
  impl_->frame_index = 0;
  if (auto r = impl_->write_bytes(hdr); !r) {
    impl_->file.reset();
    return r;
  }
  impl_->offset = hdr.size();
  impl_->stats.bytes = hdr.size();
  return {};
}

std::expected<bool, Lvx2Error> Lvx2Writer::write(
  std::size_t device_index, const DataPacketView & packet)
{
  auto & im = *impl_;
  if (!im.file) {
    return std::unexpected(invalid_argument("not open"));
  }
  if (device_index >= im.lidar_ids.size()) {
    return std::unexpected(invalid_argument("device_index out of range"));
  }
  const auto & h = packet.header;
  std::uint8_t type = 0;
  std::span<const std::byte> points = packet.data;
  switch (h.data_type) {
    case DataType::kImu:
      ++im.stats.ignored;
      return false;
    case DataType::kCartesian32:
      type = kTypeCartesian32;
      break;
    case DataType::kCartesian16:
      type = kTypeCartesian16;
      break;
    case DataType::kSpherical: {
      type = kTypeCartesian32;
      const std::size_t n = packet.data.size() / sample_size(DataType::kSpherical);
      im.scratch.resize(n * sample_size(DataType::kCartesian32));
      for (std::size_t i = 0; i < n; ++i) {
        spherical_to_cartesian32(
          packet, i, std::span(im.scratch).subspan(i * sample_size(DataType::kCartesian32)));
      }
      points = im.scratch;
      break;
    }
  }
  if (type == 0) {
    return std::unexpected(
      Lvx2Error{Lvx2Error::Kind::kUnsupportedDataType, 0, "unknown data_type"});
  }
  const std::uint64_t bin =
    h.timestamp_ns / (static_cast<std::uint64_t>(im.frame_duration_ms) * 1'000'000ULL);
  if (im.frame_bin && *im.frame_bin != bin) {
    if (auto r = im.flush_frame(); !r) {
      return std::unexpected(r.error());
    }
  }
  im.frame_bin = bin;
  const std::size_t pos = im.frame.size();
  im.frame.resize(pos + kPackageHeaderSize + points.size());
  const std::span<std::byte> ph = std::span(im.frame).subspan(pos, kPackageHeaderSize);
  ph[0] = std::byte{0};
  write_le<std::uint32_t>(ph, 1, im.lidar_ids[device_index]);
  ph[5] = std::byte{0};  // lidar_type
  ph[6] = static_cast<std::byte>(h.time_type);
  write_le<std::uint64_t>(ph, 7, h.timestamp_ns);
  write_le<std::uint16_t>(ph, 15, h.udp_cnt);
  ph[17] = static_cast<std::byte>(type);
  write_le<std::uint32_t>(ph, 18, static_cast<std::uint32_t>(points.size()));
  ph[22] = static_cast<std::byte>(h.frame_cnt);
  std::memcpy(im.frame.data() + pos + kPackageHeaderSize, points.data(), points.size());
  ++im.stats.packets;
  return true;
}

std::expected<void, Lvx2Error> Lvx2Writer::close()
{
  auto & im = *impl_;
  if (!im.file) {
    return {};
  }
  auto r = im.flush_frame();
  if (std::fflush(im.file.get()) != 0 && r) {
    r = std::unexpected(io_error("flush"));
  }
  im.file.reset();
  return r;
}

bool Lvx2Writer::is_open() const noexcept { return impl_->file != nullptr; }
Lvx2Writer::Stats Lvx2Writer::stats() const noexcept { return impl_->stats; }

// ---------------------------------------------------------------------------
// Reader

struct Lvx2Reader::Impl
{
  FilePtr file;
  Lvx2FileHeader header;
  std::vector<Lvx2DeviceInfo> devices;
  std::vector<std::byte> frame;  ///< body of the current frame
  std::size_t pos = 0;           ///< next package in `frame`
  std::uint64_t frame_index = 0;
  std::uint64_t frame_offset = 0;  ///< file offset of the current frame header
  bool truncated = false;
  bool done = false;

  /// Reads up to `n` bytes; returns the count actually read (short at EOF).
  [[nodiscard]] std::expected<std::size_t, Lvx2Error> read_some(std::span<std::byte> out) const
  {
    const std::size_t n = std::fread(out.data(), 1, out.size(), file.get());
    if (n != out.size() && std::ferror(file.get()) != 0) {
      return std::unexpected(io_error("read"));
    }
    return n;
  }

  /// Loads the next frame body; false at a clean end of file (or truncated tail).
  std::expected<bool, Lvx2Error> load_frame()
  {
    std::array<std::byte, kFrameHeaderSize> h{};
    auto n = read_some(h);
    if (!n) {
      return std::unexpected(n.error());
    }
    if (*n == 0) {
      return false;
    }
    if (*n < h.size()) {
      truncated = true;
      return false;
    }
    const auto current = read_le<std::uint64_t>(h, 0);
    const auto next = read_le<std::uint64_t>(h, 8);
    frame_index = read_le<std::uint64_t>(h, 16);
    if (current != frame_offset) {
      return std::unexpected(
        bad_file(std::format("frame header at {} says current_offset {}", frame_offset, current)));
    }
    if (next < current + kFrameHeaderSize) {
      return std::unexpected(
        bad_file(std::format("frame {} has next_offset {}", frame_index, next)));
    }
    frame.resize(next - current - kFrameHeaderSize);
    auto m = read_some(frame);
    if (!m) {
      return std::unexpected(m.error());
    }
    if (*m < frame.size()) {
      truncated = true;
      frame.resize(*m);
    }
    frame_offset = next;
    pos = 0;
    return true;
  }
};

Lvx2Reader::Lvx2Reader() : impl_(std::make_unique<Impl>()) {}
Lvx2Reader::~Lvx2Reader() = default;
Lvx2Reader::Lvx2Reader(Lvx2Reader &&) noexcept = default;
Lvx2Reader & Lvx2Reader::operator=(Lvx2Reader &&) noexcept = default;

std::expected<void, Lvx2Error> Lvx2Reader::open(const std::filesystem::path & path)
{
  auto & im = *impl_;
  im = Impl{};
  FilePtr f(std::fopen(path.c_str(), "rb"));
  if (!f) {
    return std::unexpected(io_error("open " + path.string()));
  }
  im.file = std::move(f);
  std::array<std::byte, kFileHeaderSize> hdr{};
  auto n = im.read_some(hdr);
  if (!n) {
    im.file.reset();
    return std::unexpected(n.error());
  }
  if (*n < hdr.size() || std::memcmp(hdr.data(), kSignature.data(), kSignature.size()) != 0) {
    im.file.reset();
    return std::unexpected(bad_file("not an lvx2 file (signature)"));
  }
  if (read_le<std::uint32_t>(hdr, 20) != kMagic) {
    im.file.reset();
    return std::unexpected(bad_file("not an lvx2 file (magic)"));
  }
  for (std::size_t i = 0; i < 4; ++i) {
    im.header.version[i] = static_cast<std::uint8_t>(hdr[16 + i]);
  }
  if (im.header.version[0] != 2) {
    im.file.reset();
    return std::unexpected(bad_file(std::format("unsupported version {}", im.header.version[0])));
  }
  im.header.frame_duration_ms = read_le<std::uint32_t>(hdr, 24);
  im.header.device_count = static_cast<std::uint8_t>(hdr[28]);
  std::vector<std::byte> info(kDeviceInfoSize * im.header.device_count);
  auto m = im.read_some(info);
  if (!m) {
    im.file.reset();
    return std::unexpected(m.error());
  }
  if (*m < info.size()) {
    im.file.reset();
    return std::unexpected(bad_file("device info block cut short"));
  }
  for (std::size_t i = 0; i < im.header.device_count; ++i) {
    const std::span<const std::byte> d =
      std::span(info).subspan(i * kDeviceInfoSize, kDeviceInfoSize);
    Lvx2DeviceInfo dev;
    dev.lidar_sn = get_fixed(d, 0, 16);
    dev.hub_sn = get_fixed(d, 16, 16);
    dev.lidar_id = read_le<std::uint32_t>(d, 32);
    dev.lidar_type = static_cast<std::uint8_t>(d[36]);
    dev.device_type = static_cast<std::uint8_t>(d[37]);
    dev.extrinsic_enable = d[38] != std::byte{0};
    dev.roll_deg = read_le<float>(d, 39);
    dev.pitch_deg = read_le<float>(d, 43);
    dev.yaw_deg = read_le<float>(d, 47);
    dev.x_m = read_le<float>(d, 51);
    dev.y_m = read_le<float>(d, 55);
    dev.z_m = read_le<float>(d, 59);
    im.devices.push_back(std::move(dev));
  }
  im.frame_offset = hdr.size() + info.size();
  return {};
}

const Lvx2FileHeader & Lvx2Reader::header() const noexcept { return impl_->header; }
const std::vector<Lvx2DeviceInfo> & Lvx2Reader::devices() const noexcept { return impl_->devices; }
bool Lvx2Reader::truncated() const noexcept { return impl_->truncated; }

std::expected<std::optional<Lvx2Packet>, Lvx2Error> Lvx2Reader::next_packet()
{
  auto & im = *impl_;
  if (!im.file) {
    return std::unexpected(invalid_argument("not open"));
  }
  while (!im.done) {
    if (im.pos + kPackageHeaderSize > im.frame.size()) {
      if (im.pos != im.frame.size()) {
        im.truncated = true;  // partial package header at the end of a cut frame
      }
      auto more = im.load_frame();
      if (!more) {
        return std::unexpected(more.error());
      }
      if (!*more) {
        im.done = true;
        break;
      }
      continue;
    }
    const std::span<const std::byte> ph = std::span(im.frame).subspan(im.pos, kPackageHeaderSize);
    Lvx2Packet p;
    p.frame_index = im.frame_index;
    p.lidar_id = read_le<std::uint32_t>(ph, 1);
    p.lidar_type = static_cast<std::uint8_t>(ph[5]);
    p.timestamp_type = static_cast<std::uint8_t>(ph[6]);
    p.timestamp_ns = read_le<std::uint64_t>(ph, 7);
    p.udp_counter = read_le<std::uint16_t>(ph, 15);
    const auto type = static_cast<std::uint8_t>(ph[17]);
    const auto length = read_le<std::uint32_t>(ph, 18);
    p.frame_counter = static_cast<std::uint8_t>(ph[22]);
    if (type == kTypeCartesian32) {
      p.data_type = DataType::kCartesian32;
    } else if (type == kTypeCartesian16) {
      p.data_type = DataType::kCartesian16;
    } else {
      return std::unexpected(Lvx2Error{
        Lvx2Error::Kind::kUnsupportedDataType, 0,
        std::format("package data_type {} in frame {}", type, im.frame_index)});
    }
    if (length % sample_size(p.data_type) != 0) {
      return std::unexpected(
        bad_file(std::format("package length {} not a multiple of the point size", length)));
    }
    const std::size_t end = im.pos + kPackageHeaderSize + length;
    if (end > im.frame.size()) {
      im.truncated = true;  // points cut short: only possible in a truncated last frame
      im.done = true;
      break;
    }
    p.points = std::span(im.frame).subspan(im.pos + kPackageHeaderSize, length);
    im.pos = end;
    return p;
  }
  return std::optional<Lvx2Packet>{};
}

// ---------------------------------------------------------------------------
// Player

struct Lvx2Player::Impl
{
  Lvx2PlayOptions options;
  std::filesystem::path path;
  Lvx2Reader reader;
  PacketCallback packet_cb;
  std::function<void(Frame &&)> frame_cb;
};

Lvx2Player::Lvx2Player(Lvx2PlayOptions options) : impl_(std::make_unique<Impl>())
{
  impl_->options = options;
}
Lvx2Player::~Lvx2Player() = default;
Lvx2Player::Lvx2Player(Lvx2Player &&) noexcept = default;
Lvx2Player & Lvx2Player::operator=(Lvx2Player &&) noexcept = default;

std::expected<void, Lvx2Error> Lvx2Player::open(const std::filesystem::path & path)
{
  if (impl_->options.rate < 0) {
    return std::unexpected(invalid_argument("rate must be >= 0"));
  }
  if (auto r = impl_->reader.open(path); !r) {
    return r;
  }
  impl_->path = path;
  return {};
}

const Lvx2FileHeader & Lvx2Player::header() const noexcept { return impl_->reader.header(); }
const std::vector<Lvx2DeviceInfo> & Lvx2Player::devices() const noexcept
{
  return impl_->reader.devices();
}
void Lvx2Player::on_packet(PacketCallback cb) { impl_->packet_cb = std::move(cb); }
void Lvx2Player::on_frame(std::function<void(Frame &&)> cb) { impl_->frame_cb = std::move(cb); }

std::expected<Lvx2PlayStats, Lvx2Error> Lvx2Player::run(const std::stop_token & stop)
{
  using clock = std::chrono::steady_clock;
  auto & im = *impl_;
  if (im.path.empty()) {
    return std::unexpected(invalid_argument("not open"));
  }
  Lvx2PlayStats stats;
  const bool paced = im.options.rate > 0;
  while (!stop.stop_requested()) {
    detail::FrameAssembler assembler(im.options.frame_policy, TimestampPolicy::kLidar);
    auto deliver = [&](std::optional<Frame> f) {
      if (!f) {
        return;
      }
      ++stats.frames;
      stats.points += f->points.size();
      if (im.frame_cb) {
        im.frame_cb(std::move(*f));
      }
    };
    std::optional<std::uint64_t> base_ts;
    clock::time_point base_wall{};
    std::uint64_t prev_ts = 0;
    while (!stop.stop_requested()) {
      auto next = im.reader.next_packet();
      if (!next) {
        return std::unexpected(next.error());
      }
      if (!*next) {
        break;
      }
      const Lvx2Packet & p = **next;
      if (im.options.lidar_id && p.lidar_id != *im.options.lidar_id) {
        continue;
      }
      if (paced) {
        if (!base_ts || p.timestamp_ns < prev_ts) {  // first packet, or a backward jump
          base_ts = p.timestamp_ns;
          base_wall = clock::now();
        } else {
          const auto rel = static_cast<double>(p.timestamp_ns - *base_ts) / im.options.rate;
          const auto due = base_wall + std::chrono::nanoseconds(static_cast<std::int64_t>(rel));
          while (clock::now() < due) {
            if (stop.stop_requested()) {
              break;
            }
            std::this_thread::sleep_until(
              std::min(due, clock::now() + std::chrono::milliseconds(50)));
          }
        }
        prev_ts = p.timestamp_ns;
      }
      ++stats.packets;
      if (im.packet_cb) {
        im.packet_cb(p);
      }
      deliver(assembler.push(p.to_data_packet_view(), p.timestamp_ns));
    }
    deliver(assembler.flush());
    stats.dropped_packets += assembler.counters().dropped_packets;
    if (stop.stop_requested()) {
      break;
    }
    ++stats.loops;
    if (!im.options.loop) {
      break;
    }
    if (auto r = im.reader.open(im.path); !r) {
      return std::unexpected(r.error());
    }
  }
  return stats;
}

}  // namespace livox::mid360

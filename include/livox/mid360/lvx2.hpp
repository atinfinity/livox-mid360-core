// SPDX-License-Identifier: Apache-2.0
// lvx2 file codec (issue #35): Lvx2Writer records the raw point-cloud packet stream, Lvx2Reader
// walks a file package by package, Lvx2Player replays it through the same frame assembly the
// Device uses. Format reference and unverified details: docs/lvx2.md.
#pragma once

#include <array>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

#include "livox/mid360/export.hpp"
#include "livox/mid360/frame.hpp"
#include "livox/mid360/protocol.hpp"

LIVOX_MID360_API_BEGIN
namespace livox::mid360
{

struct Lvx2Error
{
  enum class Kind : std::uint8_t
  {
    kIo,                   ///< open/read/write failed, see `errno_value`
    kInvalidArgument,      ///< e.g. no devices, device_index out of range, not open
    kUnsupportedDataType,  ///< reader: package data_type other than 1 / 2
    kBadFile,              ///< magic / version / structure mismatch, see `detail`
  };
  Kind kind = Kind::kIo;
  int errno_value = 0;
  std::string detail;
};

[[nodiscard]] std::string_view to_string(Lvx2Error::Kind kind) noexcept;
[[nodiscard]] std::string to_string(const Lvx2Error & err);

/// One entry of the lvx2 device-info block (63 bytes on disk).
struct Lvx2DeviceInfo
{
  std::string lidar_sn;          ///< at most 16 chars; zero padded
  std::string hub_sn;            ///< empty for a Mid-360
  std::uint32_t lidar_id = 0;    ///< SDK2 handle: the IP as a big-endian u32 (docs/lvx2.md)
  std::uint8_t lidar_type = 0;   ///< reserved in the spec
  std::uint8_t device_type = 9;  ///< 9 = Mid-360 (10 = HAP)
  bool extrinsic_enable = false;
  float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
  float x_m = 0, y_m = 0, z_m = 0;
};

/// Public + private header of a file.
struct Lvx2FileHeader
{
  std::array<std::uint8_t, 4> version{2, 0, 0, 0};
  std::uint32_t frame_duration_ms = 50;
  std::uint8_t device_count = 0;
};

/// One package (packet) of a frame as read from a file. `points` aliases the reader's frame
/// buffer and is valid until the next `next_packet()` / `open()` call.
struct Lvx2Packet
{
  std::uint64_t frame_index = 0;
  std::uint32_t lidar_id = 0;
  std::uint8_t lidar_type = 0;
  std::uint8_t timestamp_type = 0;  ///< TimeType value of the recorded packet
  std::uint64_t timestamp_ns = 0;
  std::uint16_t udp_counter = 0;
  DataType data_type = DataType::kCartesian32;  ///< kCartesian32 / kCartesian16 only
  std::uint8_t frame_counter = 0;
  std::span<const std::byte> points;

  /// A DataPacketView over `points` (dot_num from the length, time_interval 0, no CRC).
  [[nodiscard]] DataPacketView to_data_packet_view() const noexcept;
};

/// Writes an lvx2 file from raw data packets. Frames are cut every `frame_duration_ms` of
/// recording time, which starts at the first package; each device's timestamps join it at the
/// device's first package and after a clock jump (docs/lvx2.md). Spherical packets are
/// converted to Cartesian32; IMU packets are ignored (write() returns false). Not
/// thread-safe; call from one thread (the on_packet callback is fine).
class Lvx2Writer
{
public:
  struct Stats
  {
    std::uint64_t packets = 0;  ///< packages written (or buffered for the open frame)
    std::uint64_t frames = 0;   ///< frames flushed to disk
    std::uint64_t bytes = 0;    ///< file size so far (headers + flushed frames)
    std::uint64_t ignored = 0;  ///< IMU packets seen
  };

  Lvx2Writer();
  ~Lvx2Writer();
  Lvx2Writer(const Lvx2Writer &) = delete;
  Lvx2Writer & operator=(const Lvx2Writer &) = delete;
  Lvx2Writer(Lvx2Writer &&) noexcept;
  Lvx2Writer & operator=(Lvx2Writer &&) noexcept;

  /// Creates (truncates) `path` and writes the headers. `devices` must not be empty.
  [[nodiscard]] std::expected<void, Lvx2Error> open(
    const std::filesystem::path & path, std::span<const Lvx2DeviceInfo> devices,
    std::uint32_t frame_duration_ms = 50);
  /// Appends one packet for `devices[device_index]`. Returns true when a package was written,
  /// false for an IMU packet.
  [[nodiscard]] std::expected<bool, Lvx2Error> write(
    std::size_t device_index, const DataPacketView & packet);
  /// Flushes the open frame and closes the file. Idempotent.
  [[nodiscard]] std::expected<void, Lvx2Error> close();
  [[nodiscard]] bool is_open() const noexcept;
  [[nodiscard]] Stats stats() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Reads an lvx2 file package by package. A file whose tail is cut mid-frame yields the
/// packages that are complete and then sets `truncated()`.
class Lvx2Reader
{
public:
  Lvx2Reader();
  ~Lvx2Reader();
  Lvx2Reader(const Lvx2Reader &) = delete;
  Lvx2Reader & operator=(const Lvx2Reader &) = delete;
  Lvx2Reader(Lvx2Reader &&) noexcept;
  Lvx2Reader & operator=(Lvx2Reader &&) noexcept;

  [[nodiscard]] std::expected<void, Lvx2Error> open(const std::filesystem::path & path);
  [[nodiscard]] const Lvx2FileHeader & header() const noexcept;
  [[nodiscard]] const std::vector<Lvx2DeviceInfo> & devices() const noexcept;
  /// Next package in file order; nullopt at the end (or at a truncated tail). kBadFile /
  /// kUnsupportedDataType stop the iteration.
  [[nodiscard]] std::expected<std::optional<Lvx2Packet>, Lvx2Error> next_packet();
  [[nodiscard]] bool truncated() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct Lvx2PlayOptions
{
  FramePolicy frame_policy;
  double rate =
    1.0;  ///< playback speed relative to the recorded timestamps; 0 = as fast as possible
  bool loop = false;  ///< restart at the end of the file until the stop token is set
  /// Only packages of this lidar_id, which must be one of the file's devices (`open()` fails
  /// with kInvalidArgument otherwise). Unset: every device of the file (#169).
  std::optional<std::uint32_t> lidar_id;
};

/// The share of one played device in Lvx2PlayStats.
struct Lvx2DevicePlayStats
{
  std::uint32_t lidar_id = 0;
  std::uint64_t packets = 0;
  std::uint64_t frames = 0;
  std::uint64_t points = 0;
  std::uint64_t dropped_packets = 0;
};

struct Lvx2PlayStats
{
  std::uint64_t packets = 0;
  std::uint64_t frames = 0;
  std::uint64_t points = 0;
  std::uint64_t dropped_packets = 0;  ///< udp_counter gaps seen by the frame assemblers
  std::uint64_t loops = 0;            ///< completed passes over the file
  /// Packages skipped because the device info lists no device with their lidar_id (only
  /// counted when several devices are played).
  std::uint64_t unlisted_packets = 0;
  std::vector<Lvx2DevicePlayStats> devices;  ///< one per played device, in devices() order
};

/// Replays a file: packets go to on_packet as-is, and through a FrameAssembler per device
/// (timestamps are the recorded ones, TimestampPolicy::kLidar) to on_frame / on_device_frame.
/// Frames are closed by `frame_policy`, not by the file's 50 ms frames, and never mix devices.
/// Each device is paced by its own clock, because unsynchronised LiDARs count from their own
/// boot. Callbacks run on the calling thread.
class Lvx2Player
{
public:
  using PacketCallback = std::function<void(const Lvx2Packet &)>;
  /// `device` is the entry of devices() that the frame belongs to.
  using DeviceFrameCallback = std::function<void(const Lvx2DeviceInfo & device, Frame &&)>;

  explicit Lvx2Player(Lvx2PlayOptions options = {});
  ~Lvx2Player();
  Lvx2Player(const Lvx2Player &) = delete;
  Lvx2Player & operator=(const Lvx2Player &) = delete;
  Lvx2Player(Lvx2Player &&) noexcept;
  Lvx2Player & operator=(Lvx2Player &&) noexcept;

  /// Opens the file for run(). Fails with kInvalidArgument when `lidar_id` is set but not
  /// among the file's devices (the detail names their lidar_ids).
  [[nodiscard]] std::expected<void, Lvx2Error> open(const std::filesystem::path & path);
  [[nodiscard]] const Lvx2FileHeader & header() const noexcept;
  [[nodiscard]] const std::vector<Lvx2DeviceInfo> & devices() const noexcept;
  void on_packet(PacketCallback cb);
  /// Frames without their device. When several devices are played, set on_device_frame
  /// instead (or as well): run() refuses an on_frame alone, whose frames could not be told
  /// apart (#163).
  void on_frame(std::function<void(Frame &&)> cb);
  /// Frames with the device they belong to; also called for a single device.
  void on_device_frame(DeviceFrameCallback cb);
  /// Plays until the end of the file (once, or forever with `loop`) or until `stop` is
  /// requested; the partial frames are flushed at the end of each pass. Frame::index starts at
  /// 0 for each call and device, and keeps counting across `loop` passes. Fails with
  /// kInvalidArgument when not open, or when several devices are played with on_frame set and
  /// on_device_frame not set. A read error ends the run early and is returned.
  [[nodiscard]] std::expected<Lvx2PlayStats, Lvx2Error> run(const std::stop_token & stop = {});

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livox::mid360
LIVOX_MID360_API_END

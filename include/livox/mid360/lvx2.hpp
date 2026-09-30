// SPDX-License-Identifier: Apache-2.0
/// @file
/// lvx2 file codec (issue #35): Lvx2Writer records the raw point-cloud packet stream, Lvx2Reader
/// walks a file package by package, Lvx2Player replays it through the same frame assembly the
/// Device uses. Format reference and unverified details: docs/lvx2.md.
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

/// Error of the lvx2 writer, reader and player.
struct Lvx2Error
{
  /// Error category.
  enum class Kind : std::uint8_t
  {
    kIo,                   ///< open/read/write failed, see `errno_value`
    kInvalidArgument,      ///< e.g. no devices, device_index out of range, not open
    kUnsupportedDataType,  ///< reader: package data_type other than 1 / 2
    kBadFile,              ///< magic / version / structure mismatch, see `detail`
  };
  Kind kind = Kind::kIo;  ///< what went wrong
  int errno_value = 0;    ///< errno of a kIo error, 0 otherwise
  std::string detail;     ///< human-readable context (path, offset, offending value)
};

/// Snake-case name of the kind ("io", "invalid_argument", "unsupported_data_type",
/// "bad_file").
[[nodiscard]] std::string_view to_string(Lvx2Error::Kind kind) noexcept;
/// The kind, the detail and, for kIo, the errno text in one line.
[[nodiscard]] std::string to_string(const Lvx2Error & err);

/// One entry of the lvx2 device-info block (63 bytes on disk).
struct Lvx2DeviceInfo
{
  std::string lidar_sn;           ///< at most 16 chars; zero padded
  std::string hub_sn;             ///< empty for a Mid-360
  std::uint32_t lidar_id = 0;     ///< SDK2 handle: the IP as a big-endian u32 (docs/lvx2.md)
  std::uint8_t lidar_type = 0;    ///< reserved in the spec
  std::uint8_t device_type = 9;   ///< 9 = Mid-360 (10 = HAP)
  bool extrinsic_enable = false;  ///< the extrinsic below is set
  float roll_deg = 0;             ///< extrinsic roll in degrees
  float pitch_deg = 0;            ///< extrinsic pitch in degrees
  float yaw_deg = 0;              ///< extrinsic yaw in degrees
  float x_m = 0;                  ///< extrinsic translation x in metres
  float y_m = 0;                  ///< extrinsic translation y in metres
  float z_m = 0;                  ///< extrinsic translation z in metres
};

/// Public + private header of a file.
struct Lvx2FileHeader
{
  std::array<std::uint8_t, 4> version{2, 0, 0, 0};  ///< file format version; the reader needs 2.x
  std::uint32_t frame_duration_ms = 50;             ///< length of one file frame
  std::uint8_t device_count = 0;                    ///< entries in the device-info block
};

/// One package (packet) of a frame as read from a file. `points` aliases the reader's frame
/// buffer and is valid until the next `next_packet()` / `open()` call.
struct Lvx2Packet
{
  std::uint64_t frame_index = 0;    ///< index of the file frame that holds the package
  std::uint32_t lidar_id = 0;       ///< Lvx2DeviceInfo::lidar_id of the recording device
  std::uint8_t lidar_type = 0;      ///< reserved in the spec
  std::uint8_t timestamp_type = 0;  ///< TimeType value of the recorded packet
  std::uint64_t timestamp_ns = 0;   ///< timestamp of the recorded packet
  std::uint16_t udp_counter = 0;    ///< udp_cnt of the recorded packet
  DataType data_type = DataType::kCartesian32;  ///< kCartesian32 / kCartesian16 only
  std::uint8_t frame_counter = 0;               ///< frame_cnt of the recorded packet
  std::span<const std::byte> points;            ///< the packed point samples

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
  /// Counters of the file being written.
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
  /// Takes over the other writer's file; the moved-from writer must not be used again.
  Lvx2Writer(Lvx2Writer &&) noexcept;
  /// Takes over the other writer's file; the moved-from writer must not be used again.
  Lvx2Writer & operator=(Lvx2Writer &&) noexcept;

  /// Creates (truncates) `path` and writes the headers. `devices` must not be empty. The spec
  /// fixes `frame_duration_ms` at 50 for version 2.0.0.0; other values make files that other
  /// readers may refuse (#174).
  [[nodiscard]] std::expected<void, Lvx2Error> open(
    const std::filesystem::path & path, std::span<const Lvx2DeviceInfo> devices,
    std::uint32_t frame_duration_ms = 50);
  /// Appends one packet for `devices[device_index]`. Returns true when a package was written,
  /// false for an IMU packet.
  [[nodiscard]] std::expected<bool, Lvx2Error> write(
    std::size_t device_index, const DataPacketView & packet);
  /// Flushes the open frame and closes the file. Idempotent.
  [[nodiscard]] std::expected<void, Lvx2Error> close();
  /// True between a successful open() and close().
  [[nodiscard]] bool is_open() const noexcept;
  /// Counters since the last open().
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
  /// Takes over the other reader's file; the moved-from reader must not be used again.
  Lvx2Reader(Lvx2Reader &&) noexcept;
  /// Takes over the other reader's file; the moved-from reader must not be used again.
  Lvx2Reader & operator=(Lvx2Reader &&) noexcept;

  /// Opens `path` and reads the headers and the device-info block, so header() and devices()
  /// are valid afterwards. Reopening starts again at the first package.
  [[nodiscard]] std::expected<void, Lvx2Error> open(const std::filesystem::path & path);
  /// File header of the open file.
  [[nodiscard]] const Lvx2FileHeader & header() const noexcept;
  /// Device-info block of the open file.
  [[nodiscard]] const std::vector<Lvx2DeviceInfo> & devices() const noexcept;
  /// Next package in file order; nullopt at the end (or at a truncated tail). kBadFile /
  /// kUnsupportedDataType stop the iteration.
  [[nodiscard]] std::expected<std::optional<Lvx2Packet>, Lvx2Error> next_packet();
  /// True once the reader has hit the cut-off tail of a truncated file.
  [[nodiscard]] bool truncated() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Settings of an Lvx2Player.
struct Lvx2PlayOptions
{
  FramePolicy frame_policy;  ///< how the frame assemblers close frames
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
  std::uint32_t lidar_id = 0;         ///< Lvx2DeviceInfo::lidar_id of the device
  std::uint64_t packets = 0;          ///< packages played
  std::uint64_t frames = 0;           ///< frames delivered
  std::uint64_t points = 0;           ///< points in the delivered frames
  std::uint64_t dropped_packets = 0;  ///< udp_counter gaps seen by the frame assembler
};

/// Result of Lvx2Player::run(): the totals over all played devices, plus each device's share.
struct Lvx2PlayStats
{
  std::uint64_t packets = 0;          ///< packages played
  std::uint64_t frames = 0;           ///< frames delivered
  std::uint64_t points = 0;           ///< points in the delivered frames
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
/// boot. Points stay in the LiDAR's coordinates: the file's extrinsic is not applied, and the
/// points of one package share its time (no time_interval in the format; docs/lvx2.md, #174).
/// Callbacks run on the calling thread.
class Lvx2Player
{
public:
  /// Receives every played package as read from the file.
  using PacketCallback = std::function<void(const Lvx2Packet &)>;
  /// `device` is the entry of devices() that the frame belongs to.
  using DeviceFrameCallback = std::function<void(const Lvx2DeviceInfo & device, Frame &&)>;

  /// A player that is not open yet; `options` apply to every run().
  explicit Lvx2Player(Lvx2PlayOptions options = {});
  ~Lvx2Player();
  Lvx2Player(const Lvx2Player &) = delete;
  Lvx2Player & operator=(const Lvx2Player &) = delete;
  /// Takes over the other player's file and callbacks; the moved-from player must not be used
  /// again.
  Lvx2Player(Lvx2Player &&) noexcept;
  /// Takes over the other player's file and callbacks; the moved-from player must not be used
  /// again.
  Lvx2Player & operator=(Lvx2Player &&) noexcept;

  /// Opens the file for run(). Fails with kInvalidArgument when `lidar_id` is set but not
  /// among the file's devices (the detail names their lidar_ids).
  [[nodiscard]] std::expected<void, Lvx2Error> open(const std::filesystem::path & path);
  /// File header of the open file.
  [[nodiscard]] const Lvx2FileHeader & header() const noexcept;
  /// Device-info block of the open file.
  [[nodiscard]] const std::vector<Lvx2DeviceInfo> & devices() const noexcept;
  /// Every played package, before it goes to the frame assembler.
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

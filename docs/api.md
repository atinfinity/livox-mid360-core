# Public API design (device layer)

> The overview of all layers, threads and data flows is in [architecture.md](architecture.md).

`include/livox/mid360/context.hpp`, `device.hpp`, `frame.hpp`, `event.hpp`. Design decisions
are recorded in [issue #9](https://github.com/atinfinity/livox-mid360-core/issues/9); this page
describes the resulting shape. The data path (#6: receive thread, dispatch, frames, IMU,
drop counting, `kStats`), push handling (#7: `work_state()`, `hms()`, `kStateChanged`,
`kHms`) and reconnection / multi-device (#8: `kDisconnected`, `kReconnected`,
`ReconnectOptions`, `Context::find()`) are implemented and the headers are part of
`mid360.hpp`.

## Position in the stack

```
④ device     Context (shared receive thread), Device (one LiDAR), callbacks   ← this page
③ session    discover(), Session, HostSetup: commands on the caller's thread
② transport  UdpSocket, Poller
① protocol   CRC, frames, packets, key-value (pure functions)
```

## Why a Context

The host ports the LiDAR sends to (push 56201, point cloud 56301, IMU 56401) are the same for
every Mid-360 on the network; the official SDK also receives on one set of ports and separates
LiDARs by source IP. A receive socket therefore cannot belong to one device. `Context` owns
those three sockets and **one receive thread** (`recvmmsg` + `poll`) and dispatches each
datagram to the `Device` registered for its source IP. Datagrams from an unknown IP are
counted in `ContextStats::unknown_source` and dropped.

The command socket (host port 56101 by default, one per LiDAR, ACKs matched by `seq`) stays
inside the existing `Session`, which `Device` wraps unchanged. `discover()` remains a free
function.

```
Context ── recv thread ──┬── Device A (Session A: command socket) ── callbacks
   3 sockets             └── Device B (Session B: command socket) ── callbacks
```

## Lifecycle

```cpp
auto ctx = Context::create({.bind_address = {192, 168, 1, 5}});  // thread starts here
auto found = discover();
auto dev = Device::open(**ctx, found->front(), {});            // connect + host setup
dev->on_frame([&](Frame && f) { queue.push(std::move(f)); });  // before start
dev->start_sampling();                                         // work_tgt_mode + wait
...
dev->stop_sampling();
dev.reset();  // before ctx
```

- `Context::create(ContextOptions)` binds the sockets and starts the thread; the destructor
  stops and joins it. There is no separate start/stop.
- `Device::open(Context&, DiscoveredDevice, DeviceOptions)` connects the `Session` (its
  `bind_address` defaults to the Context's), registers with the Context, and applies
  `DeviceOptions::host_setup` with the ports taken from the Context (so a device cannot be
  pointed at ports nobody listens on) and the host IP from `host_setup.ip`, or else the
  command socket's local address. It does not change the work mode; `start_sampling()` /
  `stop_sampling()` do (`work_tgt_mode` plus `wait_for_state`, bounded by
  `host_setup.wait_timeout`). `start_sampling()` is idempotent; `stop_sampling()` discards a
  partial frame.
- `Device` and `Context` are non-copyable and non-movable; `open`/`create` return
  `std::unique_ptr` so the pointer doubles as the future C handle. Every Device must be
  destroyed before its Context (asserted in debug builds).
- The Device destructor unregisters from the Context and waits for an in-flight callback to
  return. Destroying a Device from inside one of its own callbacks is therefore forbidden.
- Reconnection after a LiDAR reboot or cable pull happens behind the same API
  (`Event::kDisconnected` / `kReconnected`, see "Reconnection" below).

## Threading rules

1. **The receive thread never blocks on user code or commands.** Parsing runs on it and the
   callbacks are invoked on it; they must return quickly. Heavy work goes through a
   `BoundedQueue<T>` (or your own queue) to another thread.
2. **Commands run on the caller's thread**, serialised by a mutex inside `Device`, so several
   user threads may call them. `cancel()` bypasses the mutex and forwards to
   `Session::cancel()`. `Session` itself stays non-thread-safe.
3. **Never call a command from a callback**: it would block the receive thread (and can
   deadlock with the destructor). Debug builds assert. React to an `Event` from your own
   thread instead.
4. **An exception escaping a callback terminates the process.** Catching it would leave the
   device state undefined and has no meaning across the C ABI.
5. Observation methods (`stats()`, `work_state()`, `hms()`, `pushed_status()`, `info()`) are
   thread-safe snapshots; `identity()` / `settings()` / `status()` are synchronous inquires and
   follow the command rules above.

## Callbacks

One setter per kind, settable while sampling has not been requested: before the first
successful `start_sampling()` and after a successful `stop_sampling()` (otherwise
`DeviceError::Kind::kInvalidState`). The receive thread picks a change up at the next packet
or timer tick. Packets that arrive while a callback is unset are parsed and counted but not
delivered. Fan-out to several consumers is the caller's job; this keeps a one-to-one mapping
to C function pointers.

| Setter | Signature | Ownership |
| --- | --- | --- |
| `on_packet` | `(const DataPacketView&, const ReceiveInfo&)` | non-owning view, valid during the call only |
| `on_frame` | `(Frame &&)` | ownership transferred |
| `on_imu` | `(const ImuData &)` | trivially copyable |
| `on_event` | `(const Event &)` | trivially copyable |
| `on_push` | `(const LidarStatus &)` | the merged snapshot of every 0x0102 push (#56) |

`on_packet` is the raw tier: every accepted point-cloud or IMU packet (parsed, CRC checked when
`DeviceOptions::verify_crc` is on; never a push), before frame assembly, with the kernel
receive time. `on_event` receives a `kStats` snapshot every `DeviceOptions::stats_interval`
(default 1 s, 0 disables). `on_frame` and `on_imu` are the assembled tier. The ROS 2 tier is the `Point`
layout itself (below); converting to a message is the job of `livox-mid360-ros2`.

## Data types

All output structs are plain data (no strings, no virtuals) so the phase-3 C ABI can expose
the same layout; `tests/test_api_skeleton.cpp` pins this with `static_assert`s.

- `Point{float x, y, z (m); uint8 reflectivity; uint8 tag; uint8 line; uint32 offset_ns}`:
  same semantics as livox_ros_driver2's `CustomPoint`. Mid-360 has no physical scan lines, so
  `line = sample index % 4` as the official driver does. `offset_ns` is relative to
  `Frame::base_time_ns`.
- `Frame{index, base_time_ns, end_time_ns, std::vector<Point> points, packets,
  dropped_packets, frame_cnt, source_type, time_type}`. Spherical packets are converted to
  Cartesian. IMU data is never part of a frame (separate topic in ROS 2).
- `ImuData{time_ns, ImuSample sample}` where `ImuSample` is the protocol-layer struct
  (gyro rad/s, acc g).
- `FramePolicy{mode, window}`: `kFrameCounter` (default) closes a frame when the packet
  header's `frame_cnt` changes, a jump caused by lost packets still closes one frame;
  `kTimeWindow` closes every `window` of point time (livox_ros_driver2 publish period).
- `TimestampPolicy`: `kLidar` (packet time as is, for PTP/GPS), `kHostOffsetOnce` (default,
  LiDAR time plus a host-minus-LiDAR offset measured once), `kHostReceive`
  (kernel receive time). Whether a PTP/GPS `time_type` switches automatically to `kLidar` is
  decided in #6.
- `Event{kind, time_ns, old_state, new_state, hms[8], hms_level, diag_old, diag_new, stats}`:
  a union-like struct where `kind` selects the meaningful fields (`kStateChanged`, `kHms`,
  `kDiagChanged`, `kDisconnected`, `kReconnected`, `kStats`). `kHms` fires once per change of the *set* of active codes
  (slot order ignored) and carries `hms_level`, the highest active `HmsLevel`, for
  per-level filtering.
- `DeviceStats{packets, points, frames, imu_samples, bad_packets, dropped_packets (udp_cnt
  gaps), reordered, queue_drops, frame_cnt_fallback, last_packet_time_ns, pushes,
  last_push_time_ns, time_offset_ns, time_offset_valid}` and `ContextStats{datagrams, unknown_source}`. Counters are relaxed
  atomics written by the receive thread only.
- `DeviceError{kind, optional<SessionError> session, optional<Key> key}` with kinds `kSession`,
  `kInvalidArgument`, `kInvalidState`, `kAlreadyRegistered` (same IP opened twice on one
  Context), `kNotOpen`, `kDisconnected` and `kDecodeFailed` (`get<K>()` could not decode
  `key`). Everything returns `std::expected`; `std::error_code` is not used.
  Transport failures surface through the wrapped `SessionError`.
- `BoundedQueue<T>`: bounded (default 8), drops the **oldest** element on overflow (newest
  data wins) and counts it in `dropped()`; `push` from the receive thread, `pop(timeout)` /
  `try_pop()` / `close()` on the consumer side. Independent of `Device`.

## Receive pipeline

Decisions recorded in [issue #6](https://github.com/atinfinity/livox-mid360-core/issues/6).
The packet-to-frame step is the internal, thread-free class `detail::FrameAssembler`
(`src/frame_assembler.hpp`, exported for the tests and `fuzz_frame_assembler`); the receive
thread feeds it one parsed point-cloud packet at a time and delivers whatever it closes.

- **Receive thread** (`src/context.cpp`): one `Poller` over the push, point-cloud and IMU
  sockets, `recv_batch` (`ContextOptions::batch_size` datagrams, up to 8 batches per socket
  per wake-up so one socket cannot starve the others), dispatch by source IP against a
  snapshot of the registry (`mutex + vector<{ip, Device*}>` plus a generation counter). The
  push socket is only counted until #7. Unregistering removes the entry, bumps the generation,
  wakes the poller and waits on a condition variable until the thread has taken a new
  snapshot, so no callback of the removed Device is in flight afterwards. The poll timeout is
  the earliest of the Devices' timers (idle frame close, next `kStats`) and 100 ms, on
  `steady_clock`; packet receive times stay `CLOCK_REALTIME`.

- **Frame counter mode** (default): a frame closes when the header `frame_cnt` changes. The
  Mid-360 is a non-repetitive scanner and the wiki marks `frame_cnt` invalid for that, so if
  the counter has not changed for `2 × window` since the first packet the assembler falls back
  to the time window and counts it in `DeviceStats::frame_cnt_fallback` (to be checked on
  hardware, #11).
- **Time window mode**: packets are never split; a packet whose first point is at or past
  `base_time_ns + window` starts a new frame. `base_time_ns` is the first point of the first
  packet of the frame.
- A packet whose `offset_ns` would overflow `uint32` (4.29 s) forces a close. Empty frames are
  never delivered; `Frame::index` counts delivered frames. The receive thread closes a partial
  frame after `window` without packets (idle close, both modes); `stop_sampling()` and the
  destructor deliver nothing.
- **Drop counting** is per source port (point cloud and IMU separately):
  `expected = previous + 1 mod 65536`, `gap = counter − expected mod 65536`. `udp_cnt == 0`
  together with a `frame_cnt` change is the documented per-frame reset (no gap). A gap above
  32768 is a reordered or duplicated packet, counted in `reordered`, not as a drop. The first
  packet after open only sets the baseline. CRC failures go to `bad_packets`.
- **Timestamps**: `kHostOffsetOnce` measures `offset = receive time − packet timestamp` once,
  at the first point-cloud packet, and applies it to every unsynchronised packet (points and
  IMU); packets whose `time_type` is PTP or GPS pass through unchanged. `kHostReceive` uses
  the kernel receive time as the packet time with the intra-packet interpolation kept
  relative to it. `DeviceStats::time_offset_ns` exposes the measured offset.
- **Conversion**: Cartesian32 × 0.001, Cartesian16 × 0.01, spherical
  `x = d·sinθ·cosφ, y = d·sinθ·sinφ, z = d·cosθ` (θ zenith, φ azimuth, 0.01°), `line =
  index % 4` within the packet, `offset_ns = sample time − base_time_ns`.

## Typed key access

`Device::set<K>()` / `get<K>()` (issue #57) read and write one key with its C++ type;
`set_many<K...>()` / `get_many<K...>()` do the same for several keys in one 0x0100 / 0x0101.
The mapping from `Key` to type is `key_traits<K>` in `keys.hpp` (`key_value_t<K>` for the type,
concepts `typed_key<K>` / `writable_key<K>`). The dedicated APIs of #38–#56 (FOV, IMU config,
network config, ...) are thin wrappers over these; use `set<K>` directly when no wrapper exists
yet, and the raw `configure()` / `inquire()` for keys chosen at run time.

```cpp
auto r = dev->set<Key::kDetectMode>(DetectMode::kSensitive);      // one 0x0100
if (r && r->reboot_required) { /* ret 0x21: effective after reboot */ }
auto sn = dev->get<Key::kSn>();                                   // std::string
auto all = dev->get_many<Key::kFovCfg0, Key::kFovCfgEn, Key::kCoreTemp>();
if (all) { auto & [fov0, en, temp] = *all; }
dev->set_many<Key::kFovCfg0, Key::kFovCfgEn>(fov0, FovEnable{.fov0 = true});
```

- `set` returns `SetResult{reboot_required}`; a LiDAR rejection is `kSession` with
  `session->ret_code` / `session->error_key`, and a batch is all-or-nothing on the LiDAR side.
- `set<Key::kWorkTgtMode>` only waits for the ACK; `start_sampling()` / `stop_sampling()`
  additionally wait for the state change and manage the callbacks.
- `get` returns `kDecodeFailed` with `key` when the ACK lacks the key or the value has the
  wrong length or an out-of-range code. Read-only keys and the Mid-360S / 360L keys
  (`kSpeedMode`, `kPcFreqMod`) have no `set`; the latter have no `get` either (no traits).

| Key | Type (`key_value_t`) | Notes |
| --- | --- | --- |
| `kPclDataType` 0x0000 | `DataType` | 1–3; `kImu` (0) is out of range |
| `kPatternMode` 0x0001 | `std::uint8_t` | only 0 is documented |
| `kLidarIpCfg` 0x0004 | `LidarIpConfig` | `set_lidar_ip_config()`; `reboot_required` after a change (simulator behaviour, [unverified]) |
| `kStateInfoHostIpCfg` / `kPointCloudHostIpCfg` / `kImuHostIpCfg` 0x0005–0x0007 | `HostIpConfig` | normally set by `open()` |
| `kInstallAttitude` 0x0012 | `InstallAttitude` | `set_install_attitude()`; host transform via `extrinsic_from()` |
| `kFovCfg0` / `kFovCfg1` 0x0015 / 0x0016 | `FovConfig` | |
| `kFovCfgEn` 0x0017 | `FovEnable{fov0, fov1}` | bits 0 / 1 |
| `kDetectMode` 0x0018 | `DetectMode` | |
| `kFuncIoCfg` 0x0019 | `FuncIoConfig` | |
| `kWorkTgtMode` 0x001A | `WorkState` | 1 / 2 / 9 accepted by the LiDAR |
| `kImuDataEn` 0x001C, `kTimeFilter` 0x0026 | `bool` | |
| `kImuSensorCfg` 0x002B | `ImuSensorConfig` | |
| `kSn` 0x8000, `kProductInfo` 0x8001 | `std::string` | text before the first NUL |
| `kVersionApp` / `kVersionLoader` / `kVersionHardware` 0x8002–0x8004 | `Version` | |
| `kMac` 0x8005 | `std::array<std::uint8_t, 6>` | |
| `kCurWorkState` 0x8006 | `WorkState` | |
| `kCoreTemp` 0x8007 | `std::int32_t` | raw 0.01 °C units |
| `kPowerupCnt` 0x8008 | `std::uint32_t` | |
| `kLocalTimeNow` / `kLastSyncTime` 0x8009 / 0x800A | `std::uint64_t` | ns |
| `kTimeOffset` 0x800B | `std::int64_t` | ns |
| `kTimeSyncType` 0x800C | `TimeSyncType` | |
| `kLidarDiagStatus` 0x800E | `DiagStatus` | |
| `kFwType` 0x8010 | `FwType` | |
| `kHmsCode` 0x8011 | `std::array<std::uint32_t, 8>` | raw codes; `Device::hms()` decodes them |

## Identity, settings and status

`lidar_info.hpp` (issues #38 / #41) aggregates the read-only keys into plain structs. Each has
a key list, a decoder over any parsed key-value list (an `InquireResult` or a 0x0102 push) and a
one-line `to_string()` with the wire key names (`name=value`, greppable in logs).

```cpp
auto id = dev->identity();                      // one 0x0101 of kIdentityKeys, not cached
if (id) {
  LOG(to_string(*id));
  // sn=47MDL9K0010001 product_info=... version_app=13.18.0.244 ... mac=..
  if (id->version_app.v[0] < 13) { /* firmware too old */ }
}
```

- `DeviceIdentity` (`identity()`, keys 0x8000–0x8005): `serial_number`, `product_info`,
  `version_app` / `version_loader` / `version_hardware` (`Version`, `to_string()` gives
  `aa.bb.cc.dd`) and `mac`. A rejected or unanswered inquire is a `DeviceError`; a key missing
  from the ACK leaves its field empty / zero (`decode_identity()` never fails).
- `identity()` is not cached: `info()` keeps the discovery data (serial, IP, `dev_type`) and is
  answered without a round trip.
- `LidarSettings` (`settings()`, the 16 modelled writable keys `kSettingsKeys`): every field is
  an `std::optional`; a key the LiDAR did not answer, or answered with a value the codec
  rejects, stays empty and is not an error. `to_string()` prints the present keys in wire
  order (`pcl_data_type=CARTESIAN32 lidar_ipcfg=192.168.1.12/255.255.255.0/192.168.1.1 ...`).
- `LidarStatus` (`status()`, keys 0x8006–0x8011 `kStatusKeys`): same rules; `hms_code` holds
  the decoded slots and `time_ns` the host time (CLOCK_REALTIME) at which the ACK arrived.
  `to_string()` prints `core_temp` in °C and only the active HMS codes
  (`hms=[0x0103800a:warning]`); `time_ns` is not printed.
- `pushed_status()` is the same struct fed by the 0x0102 push (#56): the receive thread
  decodes every status key of each push and merges it into the snapshot, so a key a push
  omits keeps the value an earlier push carried and a field is empty only until the first
  push that carries it. `time_ns` is the receive time of the last push. Kept under the push
  lock, returned without a round trip, `nullopt` before the first push; not cleared by a
  reconnect. `work_state()` and `hms()` are views of it. Which keys the real push carries is
  unverified (#11); the simulator pushes all read-only keys.
- `DiagStatus` (key 0x800E, #55) is four `DiagLevel` nibbles (`system`, `scan`, `ranging`,
  `communication`; `kNormal` .. `kSafetyError`, meanings unverified on hardware) with
  `worst()` / `normal()` and `operator==`. `diag_status()` inquires it; the pushed value is
  `pushed_status()->lidar_diag_status`.

```cpp
auto st = dev->status();                          // 0x0101; or dev->pushed_status()
if (st && st->core_temp && *st->core_temp > 8000) { /* 80 °C */ }
auto cfg = dev->settings();
if (cfg && cfg->fov_cfg_en && cfg->fov_cfg_en->fov0) { /* FOV 0 in use */ }
```

| C++ | C |
| --- | --- |
| `identity()` | `livox_mid360_device_identity(dev, livox_mid360_identity_t*)` |
| `settings()` / `status()` / `pushed_status()` | `livox_mid360_device_settings(dev, livox_mid360_settings_t*)` etc.; optionals become a `present` bit mask |
| `diag_status()` | `livox_mid360_device_diag_status(dev, livox_mid360_diag_status_t*)` |
| `on_push()` | `livox_mid360_device_on_push(dev, void (*)(const livox_mid360_status_t*, void*), void*)` |

## FOV

`Device::set_fov()` / `fov()` (issue #39) bundle the three FOV keys 0x0015 (`fov_cfg0`),
0x0016 (`fov_cfg1`) and 0x0017 (`fov_cfg_en`) into `FovSettings`, three optionals. Both
windows are always stored by the LiDAR; the enable mask says which of them crop the point
cloud.

```cpp
FovSettings fov{
  .fov0 = FovConfig{.yaw_start_deg = 0, .yaw_stop_deg = 90, .pitch_start_deg = -5,
                    .pitch_stop_deg = 5, .rsvd = 0},
  .fov1 = std::nullopt,                       // leave window 1 as stored
  .enable = FovEnable{.fov0 = true, .fov1 = false}};
auto r = dev->set_fov(fov);                   // one 0x0100 with 0x0015 and 0x0017
if (!r && r.error().kind == DeviceError::Kind::kInvalidArgument) { /* r.error().key */ }
auto cur = dev->fov();                        // one 0x0101 of the three keys
if (cur) { LOG(to_string(*cur)); }            // fov0=yaw0-90/pitch-5-5 fov1=... enable=fov0:1,fov1:0
```

- `set_fov()` sends the present fields in one request, so the LiDAR applies all or none. It
  validates before any I/O: no field at all → `kInvalidArgument` without `key`; a window
  outside `fov_in_range()` (yaw in [0, 360), pitch in (-10, 60), the wiki ranges) →
  `kInvalidArgument` with `key` naming the window. Equal or reversed start / stop values pass:
  what the LiDAR makes of a wrapped or empty window is unverified (#11). `rsvd` is sent as
  given. The codecs in `keys.hpp` stay pure; `set<Key::kFovCfg0>()` skips the range check.
- `fov()` tolerates a key missing from the ACK (its field stays empty).
- `HostSetup::fov` applies the same settings in the first 0x0100 of `Device::open()` (after
  the host keys) and again on every reconnect, so a window survives a LiDAR reboot or a
  cable pull. The same validation applies (`SessionError::kInvalidArgument` with `error_key`
  0x0015 / 0x0016 through `DeviceError::session`), and `HostSetupResult::reboot_required`
  covers it.

| C++ | C |
| --- | --- |
| `set_fov()` / `fov()` | `livox_mid360_device_set_fov(dev, const livox_mid360_fov_t*)` / `..._fov(dev, livox_mid360_fov_t*)` with a `present` bit mask for the optionals |

## Install attitude and host-side extrinsic

`Device::set_install_attitude(InstallAttitude)` / `install_attitude()` (issue #51) wrap key
0x0012 (`roll_deg`, `pitch_deg`, `yaw_deg` as float degrees, `x_mm`, `y_mm`, `z_mm` as
int32). `install_attitude_valid()` in `keys.hpp` is checked before any I/O
(`kInvalidArgument` with `key` = 0x0012): the angles are finite and within ±180°. The
value is only stored on the LiDAR; whether the firmware applies it to the emitted points,
and in which convention, is [unverified] (#11). The SDK never transforms points by itself.

To transform on the host, `frame.hpp` provides an opt-in extrinsic:

```cpp
const Extrinsic e = extrinsic_from(*dev->install_attitude());   // or any InstallAttitude
dev->on_frame([e](Frame f) { apply(e, f); /* f.points are now p' = r * p + t */ });
```

`extrinsic_from()` builds `Rz(yaw) * Ry(pitch) * Rx(roll)` (intrinsic ZYX, right-handed,
degrees) and a translation in metres (mm / 1000) added after the rotation, the
livox_ros_driver2 convention. `apply()` transforms `x`, `y`, `z` in place and leaves
reflectivity, tag, line and offset untouched; the `Frame&` overload covers `points`.

| C++ | C |
| --- | --- |
| `set_install_attitude()` / `install_attitude()` | `livox_mid360_device_set_install_attitude(dev, const livox_mid360_install_attitude_t*, bool*)` / `..._install_attitude(dev, livox_mid360_install_attitude_t*)` |
| `extrinsic_from()` / `apply()` | `livox_mid360_extrinsic_from(const livox_mid360_install_attitude_t*, livox_mid360_extrinsic_t*)` / `livox_mid360_extrinsic_apply(const livox_mid360_extrinsic_t*, livox_mid360_point_t*, size_t)` |

## Point format, scan pattern and frame policy

Issue #40 covers the three things the wiki calls "coordinate format, scan pattern and
point-cloud frame rate":

- `Device::set_point_format(DataType)` / `point_format()`: key 0x0000. `kImu` is rejected
  before any I/O (`kInvalidArgument`, `key` = 0x0000). The switch is immediate on the LiDAR;
  the receive pipeline closes and delivers the frame being assembled when the first packet in
  the new format arrives, so every `Frame` carries a single `source_type`. The same holds for
  a raw `configure()` of 0x0000, since the assembler decides on the packet header, not on the
  request.
- `Device::set_scan_pattern(ScanPattern)` / `scan_pattern()`: key 0x0001 with the enum
  `kNonRepetitive` (0) / `kRepetitive` (1) / `kLowRateRepetitive` (2). Nothing is pre-checked:
  the base Mid-360 has only the non-repetitive pattern, and the LiDAR's ACK
  (`kLidarRejected`, `ret_code` 0x20 on the simulator) is the answer for the others.
  `LidarSettings::pattern_mode` and `HostSetup::scan_pattern` use the same enum.
- `Device::set_frame_policy(const FramePolicy &)` / `frame_policy()`: the base Mid-360 has
  no frame-rate key (0x0029 exists on other Livox models only and is not modelled), so the
  frame rate is the host-side `FramePolicy` (`window` for `kTimeWindow`, the LiDAR's
  `frame_cnt` period for `kFrameCounter`). The new policy is handed to the receive thread and
  takes effect at the next data packet; the frame in progress is closed by whichever policy
  is active then, and the `kFrameCounter` fallback detection restarts. `window <= 0` →
  `kInvalidArgument`. `frame_policy()` returns the last requested policy.

```cpp
dev->set_point_format(DataType::kSpherical);           // frames from now on: source_type kSpherical
dev->set_frame_policy({.mode = FramePolicy::Mode::kTimeWindow, .window = 50ms});  // 20 Hz frames
if (auto p = dev->set_scan_pattern(ScanPattern::kRepetitive); !p) { /* kLidarRejected */ }
```

**Reconnect replay rule.** The `HostSetup` replayed after a reconnect is not frozen at
`open()`: every key it models (0x0000, 0x0001, 0x0015, 0x0016, 0x0017, 0x0018, 0x001C,
0x0026, 0x002B) that a later successful 0x0100 carried, whether through `set_point_format()`,
`set_fov()`, `set_detect_mode()`, `set<K>()` or a raw `configure()`, overwrites the stored
copy. A reconnect therefore restores the last value the Device successfully wrote, not the
value passed at open. Keys `HostSetup` does not model (install attitude, function IO, ...)
are still not replayed; that is the LiDAR's own persistence.

| C++ | C |
| --- | --- |
| `set_point_format()` / `point_format()` | `livox_mid360_device_set_point_format(dev, int)` / `..._point_format(dev, int*)` |
| `set_scan_pattern()` / `scan_pattern()` | `livox_mid360_device_set_scan_pattern(dev, int)` / `..._scan_pattern(dev, int*)` |
| `set_frame_policy()` / `frame_policy()` | `livox_mid360_device_set_frame_policy(dev, const livox_mid360_frame_policy_t*)` / `..._frame_policy(dev, livox_mid360_frame_policy_t*)` |

## Detection mode, IMU and time filter

Issues #46, #47 and #54 add thin wrappers over `set<K>()` / `get<K>()` for four stored
settings; each setter returns the LiDAR's `SetResult` and each accepted write is folded into
the replayed `HostSetup` (rule above), whose new optionals `detect_mode`, `time_filter` and
`imu_sensor_config` also let `open()` write them:

- `Device::set_detect_mode(DetectMode)` / `detect_mode()`: key 0x0018, `kNormal` (0) /
  `kSensitive` (1). A value outside the enum is `kInvalidArgument` with `key` = 0x0018 before
  any I/O.
- `Device::set_imu_enabled(bool)` / `imu_enabled()`: key 0x001C, the same key
  `HostSetup::imu_enable` writes at open. Disabling stops the IMU stream at the LiDAR; the
  `on_imu` callback simply stops being called.
- `Device::set_imu_sensor_config(const ImuSensorConfig &)` / `imu_sensor_config()`: key
  0x002B, output rate (200 / 500 / 100 / 50 Hz), accelerometer range (±4 g … ±32 g) and
  gyroscope range (±2000 dps … ±15.625 dps). A field past its last enumerator is
  `kInvalidArgument` with `key` = 0x002B before any I/O. Changing the output rate changes the
  IMU packet rate the host receives (`DeviceStats::imu_samples` grows 2.5× faster at 500 Hz)
  and the `time_interval` in each IMU packet; nothing in the receive path assumes 200 Hz.
- `Device::set_time_filter(bool)` / `time_filter()`: key 0x0026. Per the wiki, with 0 a time
  rollback in the sync source interrupts the point cloud, with 1 (GPS-sync abnormal-time
  filtering) it does not. The SDK only stores the bit.

Key 0x002B is absent on older firmware ([unverified] which version added it, #11). No
distinct error kind exists for that: the LiDAR rejects the write, the read and any batched
inquire naming the key with `ret_code` 0x20, which surfaces as the ordinary `kSession` /
`kLidarRejected` error. `settings()` drops such a key and asks again, so
`LidarSettings::imu_sensor_cfg` is simply empty there.

```cpp
dev->set_detect_mode(DetectMode::kSensitive);
dev->set_imu_sensor_config({.output_rate = ImuOutputRate::k500Hz,
                            .accel_range = ImuAccelRange::k8g,
                            .gyro_range = ImuGyroRange::k1000dps});
if (auto r = dev->imu_sensor_config(); !r) {
  const auto & e = r.error();
  if (e.kind == DeviceError::Kind::kSession && e.session->kind == SessionErrorKind::kLidarRejected &&
      e.session->ret_code == RetCode::kParamNotSupport) {
    // firmware without key 0x002B: the IMU runs at its fixed 200 Hz
  }
}
dev->set_time_filter(true);
```

| C++ | C |
| --- | --- |
| `set_detect_mode()` / `detect_mode()` | `livox_mid360_device_set_detect_mode(dev, int)` / `..._detect_mode(dev, int*)` |
| `set_imu_enabled()` / `imu_enabled()` | `livox_mid360_device_set_imu_enabled(dev, bool)` / `..._imu_enabled(dev, bool*)` |
| `set_imu_sensor_config()` / `imu_sensor_config()` | `livox_mid360_device_set_imu_sensor_config(dev, const livox_mid360_imu_sensor_config_t*)` / `..._imu_sensor_config(dev, livox_mid360_imu_sensor_config_t*)` |
| `set_time_filter()` / `time_filter()` | `livox_mid360_device_set_time_filter(dev, bool)` / `..._time_filter(dev, bool*)` |

## LiDAR network config

`Device::set_lidar_ip_config(LidarIpConfig)` / `lidar_ip_config()` (issue #50) wrap key
0x0004 (`ip`, `netmask`, `gateway`). `lidar_ip_config_valid()` in `keys.hpp` is checked
before any I/O (`kInvalidArgument` with `key` = 0x0004): the address is neither unspecified,
broadcast nor the subnet's network / broadcast address, the mask is a contiguous prefix of 1
to 30 bits, and the gateway is 0.0.0.0 or inside the subnet and different from the address.
The LiDAR answers a change with `ret_code` 0x21, surfaced as `SetResult::reboot_required`
exactly as received (whether an unchanged value also answers 0x21 is [unverified], #11); the
SDK never reboots on its own.

```cpp
auto r = dev->set_lidar_ip_config({.ip = {192, 168, 1, 12},
                                   .netmask = {255, 255, 255, 0},
                                   .gateway = {192, 168, 1, 1}});
if (r && r->reboot_required) {
  dev->reboot();   // kDisconnected{kRebootRequested}, then kReconnected at the new address
}
```

After `reboot()` the ordinary reconnection (below) applies: the old endpoint fails, discovery
is filtered by serial and the Device is re-keyed to the new IP. The Device remembers the
address it configured and unicasts discovery to `{that ip, ReconnectOptions::discovery_port}`
(56000 by default) before the user's `discovery_targets` / broadcast, so a move to a subnet
that broadcast does not reach still recovers. The point / IMU / push destinations are the
host's and are replayed unchanged.

| C++ | C |
| --- | --- |
| `set_lidar_ip_config()` / `lidar_ip_config()` | `livox_mid360_device_set_lidar_ip_config(dev, const livox_mid360_lidar_ip_config_t*, bool* reboot_required)` / `..._lidar_ip_config(dev, livox_mid360_lidar_ip_config_t*)` |

## Push handling

The LiDAR sends a 0x0102 info push about once per second to the host push port. The receive
thread parses it, merges every status key into `pushed_status()` (#56, see above) and then,
in this order, raises the change events and calls `on_push` with the merged snapshot:

- `cur_work_state` (0x8006) becomes `Device::work_state()`. A change relative to the
  previous push raises `Event::kStateChanged{old_state, new_state}`; the first push after
  `open()` only records the state. Pushed state is observational: `start_sampling()` /
  `stop_sampling()` still poll 0x8006 until the target state is reached.
- `hms_code` (0x8011) becomes `Device::hms()`. When the set of active codes changes (slot
  order ignored; the baseline after `open()` is all-zero), `Event::kHms{hms, hms_level}` is
  raised with the slots in wire order and `hms_level` = highest active level.

- `lidar_diag_status` (0x800E) raises `Event::kDiagChanged{diag_old, diag_new}` when the
  value differs from the last one seen; the baseline after `open()` is all normal, so a
  first push with any abnormal subsystem raises (#55).
- `on_push(const LidarStatus &)` receives the snapshot every push produced, after that
  push's events. Pushes that fail to parse are neither merged nor delivered.

Temperature and the time-sync keys have no event and are not part of `DeviceStats`: read
them from the snapshot. `DeviceStats::pushes` / `last_push_time_ns` count accepted pushes
(frames that fail to parse count in `bad_packets`); the push is also the heartbeat for
disconnect detection below.

## Reconnection

Decisions recorded in [issue #8](https://github.com/atinfinity/livox-mid360-core/issues/8).
A Device is either *connected* or *disconnected* (`Device::connected()`); the transitions are
`Event::kDisconnected` (with `reason`) and `Event::kReconnected` (with `attempts`), both
delivered on the receive thread like every other event. `DeviceStats::disconnects` /
`reconnects` count them. Options live in `DeviceOptions::reconnect` (`ReconnectOptions`).

**Detection** (connected → disconnected), first of:

- no accepted 0x0102 push for `push_timeout` (3 s by default, checked by the receive thread's
  timer; the LiDAR pushes about once per second) → `kPushTimeout`;
- a command timed out **and** the last push is older than `push_timeout / 3` (one nominal
  push period): a lone lost ACK on a healthy link is only reported to the caller →
  `kCommandTimeout`;
- `reboot()` was acknowledged: the outage is known, no need to wait for the timeout →
  `kRebootRequested`;
- `disconnect()` was called → `kUser`.

Point-cloud / IMU silence is not a signal: the user may simply have stopped sampling.

**While disconnected**, `start_sampling()`, `stop_sampling()`, `configure()`, `inquire()` and
`reboot()` fail fast with `DeviceError::Kind::kDisconnected` instead of waiting for a LiDAR
that is not there. Pushes that do arrive are still parsed (`work_state()` follows them and
`kStateChanged` is raised against the last state seen before the outage; the baseline is
not reset). Callbacks stay frozen for the whole period when sampling had been requested.

**Recovery** (disconnected → connected) is one attempt, shared by the automatic worker and
`Device::reconnect()`:

1. `Session::connect` to the last known command endpoint with the serial number verified
   (a cable pull keeps the address, so this is fast and does not broadcast);
2. otherwise `discover()` (first the address `set_lidar_ip_config()` configured, if any,
   then `discovery_targets` unicast, or broadcast; `discovery_timeout` each)
   filtered by the original serial number — a reboot or DHCP may have moved the LiDAR, and
   the serial, not the IP, is its identity. A new IP re-registers the Device with the Context;
   an IP held by another Device fails the attempt (`kAlreadyRegistered`) and it is retried;
3. `apply_host_setup` with the setup recorded at `open()` (the LiDAR may have rebooted);
4. `work_tgt_mode = SAMPLING` plus the state wait if sampling had been requested;
5. the time offset (`kHostOffsetOnce`) is re-measured at the next packet, the partial frame
   is discarded, and `kReconnected` is raised.

Any failure keeps the Device disconnected. With `enabled` (the default) a worker thread owned
by the Device — started lazily at the first disconnect, joined by the destructor — repeats the
attempt with exponential backoff (`initial_backoff` 500 ms, doubling to `max_backoff` 8 s)
for as long as the Device lives; there is no terminal "failed" state to poll and reset. With
`enabled = false` the Device only detects; `reconnect()` runs one attempt on the caller's
thread (serialised with the commands, `kSession` on failure, no-op when connected).

The destructor requests a stop that the attempt observes within about 100 ms even inside
`discover()` or `Session::connect` (`DiscoveryOptions::stop` / `SessionOptions::stop`,
`std::stop_token`), so tearing down a Device mid-outage does not wait for the timeouts.

## Logging

Decisions recorded in [issue #42](https://github.com/atinfinity/livox-mid360-core/issues/42).
The SDK is silent by default: nothing is written to stdout or stderr unless the application
installs a handler. `log.hpp` provides one process-wide level and one process-wide handler:

```cpp
set_log_level(LogLevel::kInfo);
set_log_handler(stderr_log_handler());                 // or:
auto file = file_log_handler("/var/log/mid360.log");   // expected<LogHandler, DeviceError>
if (file) set_log_handler(*file);
set_log_handler([](const LogRecord & r) { my_logger(r.level, r.serial_number, r.message); });
```

- **Record**: `LogRecord{level, time_ns, serial_number, message}`. `time_ns` is the system
  clock in nanoseconds since the Unix epoch; `serial_number` is empty for records that are
  not device-scoped (Context socket errors, unknown-source datagrams). Both string views are
  valid only during the handler call; copy them to keep them.
- **Levels**, ordered `kOff < kError < kWarn < kInfo < kDebug < kTrace`; a record is emitted
  when its level is `<=` the level set. `kError`: socket failures on the receive thread
  (EAGAIN excluded). `kWarn`: disconnect with its reason, command timeouts after every
  attempt. `kInfo`: `open`/close, work-state transitions, reconnect attempts and their
  backoff, host-setup replay, reconnected. `kDebug`: per-request retries, datagrams from
  unknown sources. `kTrace` is reserved.
- **Cost**: the level check is one relaxed atomic load before any formatting, so `kOff` and
  `kError` are free on the receive thread. A record is formatted with `std::format_to_n`
  into a 256-byte stack buffer (longer messages are truncated) and never allocates.
- **Threads**: records are emitted from the Context receive thread, the Device worker thread
  and caller threads, never while an SDK mutex is held. `set_log_handler` is safe while a
  Context runs: it is an atomic swap of a `shared_ptr`; a handler that is executing at the
  swap finishes its call and the old handler object is destroyed when that call returns. The
  handler must not throw (the call path is `noexcept`, so a throw terminates, as for the
  data callbacks) and must not call back into the SDK.
- **Sinks**: `format_log_record()` renders `2026-09-26T12:34:56.789Z W [serial] message`
  (UTC, millisecond precision, one-letter level, `[-]` without serial) for custom sinks;
  `stderr_log_handler()` writes that line to stderr; `file_log_handler(path, append = true)`
  writes and flushes it per record, and fails with `DeviceError::Kind::kIo` plus
  `errno_value` when the file cannot be opened.
- This is the SDK's own diagnostic trail. The LiDAR **firmware** log stream (port 56500,
  0x03xx commands) is a separate feature, tracked in #44.

## Multiple devices

Every Device opened on a Context is registered under its source IP (dispatch) and its serial
number (identity). `Context::find(serial)` returns the open `Device*` or nullptr and
`Context::devices()` lists them; both are non-owning — the caller keeps the `unique_ptr` and
destroys it before the Context, as before. `Device::open` refuses a serial number (or an IP)
that is already open with `kAlreadyRegistered`. Discovery of every LiDAR on the network stays
the free function `discover()`; auto-creating a Device per serial found is left to the caller
(or a later `DeviceManager`).

Each Device reconnects independently: one LiDAR's outage does not touch the others. Several
Devices need distinct command sockets (`SessionOptions::host_command_port` 0 or distinct
values). In the tests the second simulator answers from 127.0.0.2, which Linux accepts on the
loopback interface as is and macOS after `sudo ifconfig lo0 alias 127.0.0.2` (the test skips
otherwise).

## C ABI mapping (phase 3)

The C header is written once the C++ layer is implemented; this table fixes the shape.

| C++ | C |
| --- | --- |
| `Context*` / `Device*` (`unique_ptr`) | `livox_mid360_context_t*` / `livox_mid360_device_t*` opaque handles |
| `std::function` callback | function pointer + `void* user` |
| `Frame` | `const livox_mid360_frame_t*` with `const livox_mid360_point_t* points, size_t count` |
| `Point`, `ImuData`, `Event`, `DeviceStats` | same layout, `typedef struct` |
| `DeviceError` | `int` code + `livox_mid360_error_string()` |
| `Context::find` / `devices` | `livox_mid360_context_find(ctx, sn)` / `..._devices(ctx, out, cap)` |
| `ReconnectOptions`, `DisconnectReason` | same layout, `typedef struct` / `enum` |
| `std::expected<T, DeviceError>` | `int` return, out-parameter for `T` |
| `set<K>` / `get<K>` (#57) | raw `livox_mid360_device_set_key(dev, key, bytes, len)` / `..._get_key(dev, key, buf, cap, &len)`; typed per-key helpers only where a C++ wrapper (#38–#56) exists |
| `set_log_level` / `set_log_handler` (#42) | `livox_mid360_set_log_level(level)` / `livox_mid360_set_log_handler(cb, user)` with `livox_mid360_log_record_t` (`level`, `time_ns`, NUL-terminated `serial_number` and `message` valid during the call) |

`livox-mid360-ros2` (separate repository) uses the C++ API directly: one `Context`, one
`Device` per LiDAR, `on_frame` publishing from the receive thread or through a queue.

## Split of the implementation

| Issue | Scope |
| --- | --- |
| #6 | Context sockets and receive thread, dispatch by IP, `on_packet`, frame assembly, `on_frame` / `on_imu`, drop counting, timestamp policy, `kStats` events |
| #7 | 0x0102 push parsing, work-state machine, `kStateChanged` / `kHms` events, `work_state()` |
| #56 / #55 | full push snapshot in `pushed_status()` (`time_ns`, carry-over), `on_push`, `DiagStatus` typing, `diag_status()`, `kDiagChanged` |
| #8 | disconnect detection, reconnection, `kDisconnected` / `kReconnected`, multiple devices by serial |

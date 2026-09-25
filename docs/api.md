# Public API design (device layer)

`include/livox/mid360/context.hpp`, `device.hpp`, `frame.hpp`, `event.hpp`. Design decisions
are recorded in [issue #9](https://github.com/atinfinity/livox-mid360-core/issues/9); this page
describes the resulting shape. The data path (#6: receive thread, dispatch, frames, IMU,
drop counting, `kStats`) is implemented and the headers are part of `mid360.hpp`; push /
state / HMS handling (#7) and reconnection (#8) are still to come, so `work_state()` returns
`nullopt` and only `kStats` events are emitted for now.

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
auto ctx = Context::create({.bind_address = {192, 168, 1, 5}});     // thread starts here
auto found = discover();
auto dev = Device::open(**ctx, found->front(), {});                   // connect + host setup
dev->on_frame([&](Frame&& f) { queue.push(std::move(f)); });         // before start
dev->start_sampling();                                                // work_tgt_mode + wait
...
dev->stop_sampling();
dev.reset();                                                          // before ctx
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
- Reconnection after a LiDAR reboot or cable pull is added by #8 behind the same API
  (`Event::kDisconnected` / `kReconnected`).

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
5. Observation methods (`stats()`, `work_state()`, `info()`) are thread-safe snapshots.

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
| `on_frame` | `(Frame&&)` | ownership transferred |
| `on_imu` | `(const ImuData&)` | trivially copyable |
| `on_event` | `(const Event&)` | trivially copyable |

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
  the HANDOFF policy: LiDAR time plus a host-minus-LiDAR offset measured once), `kHostReceive`
  (kernel receive time). Whether a PTP/GPS `time_type` switches automatically to `kLidar` is
  decided in #6.
- `Event{kind, time_ns, old_state, new_state, hms[8], stats}`: a union-like struct where
  `kind` selects the meaningful fields (`kStateChanged`, `kHms`, `kDisconnected`,
  `kReconnected`, `kStats`). Per-level HMS filtering is added by #7.
- `DeviceStats{packets, points, frames, imu_samples, bad_packets, dropped_packets (udp_cnt
  gaps), reordered, queue_drops, frame_cnt_fallback, last_packet_time_ns, time_offset_ns,
  time_offset_valid}` and `ContextStats{datagrams, unknown_source}`. Counters are relaxed
  atomics written by the receive thread only.
- `DeviceError{kind, optional<SessionError> session}` with kinds `kSession`,
  `kInvalidArgument`, `kInvalidState`, `kAlreadyRegistered` (same IP opened twice on one
  Context), `kNotOpen`. Everything returns `std::expected`; `std::error_code` is not used.
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

## C ABI mapping (phase 3)

The C header is written once the C++ layer is implemented; this table fixes the shape.

| C++ | C |
| --- | --- |
| `Context*` / `Device*` (`unique_ptr`) | `livox_mid360_context_t*` / `livox_mid360_device_t*` opaque handles |
| `std::function` callback | function pointer + `void* user` |
| `Frame` | `const livox_mid360_frame_t*` with `const livox_mid360_point_t* points, size_t count` |
| `Point`, `ImuData`, `Event`, `DeviceStats` | same layout, `typedef struct` |
| `DeviceError` | `int` code + `livox_mid360_error_string()` |
| `std::expected<T, DeviceError>` | `int` return, out-parameter for `T` |

`livox-mid360-ros2` (separate repository) uses the C++ API directly: one `Context`, one
`Device` per LiDAR, `on_frame` publishing from the receive thread or through a queue.

## Split of the implementation

| Issue | Scope |
| --- | --- |
| #6 | Context sockets and receive thread, dispatch by IP, `on_packet`, frame assembly, `on_frame` / `on_imu`, drop counting, timestamp policy, `kStats` events |
| #7 | 0x0102 push parsing, work-state machine, `kStateChanged` / `kHms` events, `work_state()` |
| #8 | disconnect detection, reconnection, `kDisconnected` / `kReconnected`, multiple devices by serial |

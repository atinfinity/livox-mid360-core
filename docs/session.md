# Session layer

`include/livox/mid360/session.hpp`, `src/session.cpp`. Design decisions are recorded in
[issue #4](https://github.com/atinfinity/livox-mid360-core/issues/4); this page describes the
result and how to use it.

## Position in the stack

```
④ device               Context / Device, receive thread, callbacks (docs/api.md, #9)
③ session              discover(), Session: commands, retries, work-state   ← this page
② transport            UdpSocket, Poller
① protocol             CRC, frames, packets, key-value (pure functions)
```

The session layer owns one command socket per LiDAR and turns the request/ACK protocol into
synchronous calls. It does not receive point-cloud or IMU data and creates no threads.

## Principles

- **Synchronous and single-threaded.** Every call blocks the calling thread and drives
  `Poller::wait()` itself. The only cross-thread entry point is `cancel()`.
- **No exceptions.** Every fallible call returns `std::expected<T, SessionError>`.
- **Retries reuse the sequence number.** A request gets one `seq_num`; each attempt resends
  the same frame, so a late ACK to an earlier attempt still matches.
- **A LiDAR rejection is final.** `ret_code != 0` is returned immediately as
  `kLidarRejected`, never retried. Malformed datagrams are counted and treated as no response.
- **ACK matching** requires `cmd_type == ACK`, matching `seq_num`, matching `cmd_id` and a
  source IP equal to the LiDAR's. Anything else is counted in `SessionStats` and dropped.

## Discovery

```cpp
DiscoveryOptions o;  // empty targets → broadcast 255.255.255.255:56000, wait the whole timeout
o.targets = {parse_endpoint("192.168.1.12:56000").value()};  // unicast: returns once all answered
auto devices = discover(o);  // std::expected<std::vector<DiscoveredDevice>, SessionError>
```

`discover()` binds an ephemeral socket on `bind_address`, sends `0x0000` and collects ACKs
until the timeout (or, with unicast targets, until every target answered). Devices are
deduplicated by serial number, first ACK wins. `DiscoveredDevice::from` is the address the
ACK actually came from, which differs from `ip` only in NAT-like setups. An empty result is
not an error.

## `Session`

```cpp
auto s = Session::connect(devices->front());  // or connect(Endpoint{ip, 56100})
if (!s) {
  std::cerr << to_string(s.error()) << '\n';
  return;
}

const KeyValue kv{
  static_cast<std::uint16_t>(Key::kPointCloudHostIpCfg),
  encode_host_ip_config({.ip = {192, 168, 1, 5}, .dst_port = 56301, .src_port = 56300})};
auto cfg = s->configure(std::span<const KeyValue>(&kv, 1));  // 0x21 (reboot effect) is a success

const Key keys[] = {Key::kSn, Key::kVersionApp};
auto inq = s->inquire(keys);  // InquireResult owns the ACK bytes
if (inq) {
  std::cout << decode_string(*inq->get(Key::kSn)) << '\n';
}

auto ready = s->wait_for_state(WorkState::kSampling, std::chrono::seconds(10));
```

`connect()` binds `bind_address:host_command_port` (default 56101; 0 = ephemeral), reads key
`0x8000` and, for the `DiscoveredDevice` overload, compares it with the discovered serial
number (`verify_serial`). A mismatch is reported as `kBadResponse`.

Methods:

| Method | Command | Returns |
| --- | --- | --- |
| `request(cmd_id, data)` | any | `RawAck` (owned payload) |
| `discovery_ack()` | 0x0000 | `DiscoveryAck` (real LiDARs may only answer on 56000) |
| `configure(kvs)` | 0x0100 | `ParamConfigAck`; `ret 0x21` is success, other non-zero → `kLidarRejected` with `error_key` |
| `inquire(keys)` | 0x0101 | `InquireResult` with `get(Key)` |
| `reboot(timeout_ms)` | 0x0200 | `SimpleAck` |
| `factory_reset()` | 0x0201 | `SimpleAck` |
| `set_gps_time(ns)` | 0x0202 | `SimpleAck` |
| `work_state()` | 0x0101 key 0x8006 | `WorkState` |
| `wait_for_state(target, timeout)` | polls 0x8006 | `void`; `kUnexpectedState` on ERROR / UPGRADE |
| `cancel()` | — | aborts the blocking call with `kCancelled` |

Every command takes an optional `RequestOptions{timeout (per attempt), attempts}` that
overrides `SessionOptions::request` (default 500 ms × 3).

## Host setup

`include/livox/mid360/config.hpp` bundles the usual first configuration into one call
(design in [issue #5](https://github.com/atinfinity/livox-mid360-core/issues/5)):

```cpp
HostSetup setup;           // ip defaults to the session socket's local address
setup.point_port = 56301;  // host-side ports; defaults are 56201 / 56301 / 56401
setup.pcl_data_type = DataType::kCartesian32;
setup.imu_enable = true;
setup.work_tgt_mode = WorkState::kSampling;  // optional; then waits up to wait_timeout (10 s)
auto r = apply_host_setup(*s, setup);        // std::expected<HostSetupResult, SessionError>
if (r && r->reboot_required) {
  // an ACK said 0x21: reboot to apply
}
```

`apply_host_setup` sends one `0x0100` with keys `0x0005`, `0x0006`, `0x0007` (host ipcfg with
the LiDAR-side source ports 56200 / 56300 / 56400), `0x0000` and `0x001C`, then, when
`work_tgt_mode` is set, a second `0x0100` with `0x001A` followed by `wait_for_state`
(`wait_timeout` 0 skips the wait, `final_state` is then empty). Only SAMPLING / IDLE / READY
can be requested. Arguments the LiDAR would never accept (that mode, or an empty `ip` on a
`0.0.0.0` bind) are reported as `kInvalidArgument` with `error_key` before anything is sent;
everything else is the plain session error, so a rejected key shows up as `kLidarRejected`
with `error_key`. `host_setup_key_values()` exposes the first request's key-value list for
callers that drive `configure()` themselves.

## Errors

`SessionError::kind` is one of `kTransport` (see `transport`), `kTimeout` (`attempts` sent),
`kBadResponse` (see `parse`), `kLidarRejected` (`ret_code`, `error_key`), `kUnexpectedState`
(`work_state`), `kCancelled` and `kInvalidArgument` (`error_key`, nothing was sent). `to_string(err)` gives a one-line summary such as
`timeout cmd 0x0101 after 3 attempt(s)`.

## Statistics

`Session::stats()` counts `requests`, `retries`, `timeouts`, `late_acks` (ACKs that matched no
pending request, including datagrams drained while waiting for a state) and `bad_frames`.

## Testing

`tests/test_session.cpp` runs against `tools/livox_mid360_sim.py` with fault injection
(`drop_ack`, `silence`, `set_state`) and covers unicast discovery, serial verification,
rejections, retry, timeout, `wait_for_state` and cancellation from another thread. Broadcast
discovery is a hidden test (`[.broadcast]`) for manual runs on a LAN. `tests/test_config.cpp`
covers the host setup flow: the key-value list against the golden vector, the round trip to
IDLE and back to SAMPLING, the default host address, and the rejection / invalid-argument paths.

## Not covered here (phase 2)

Receive threads, `0x0102` push handling, reconnection after reboot and multi-device
management are layered on top by the device layer (docs/api.md, #9; implemented in #6, #7 and #8).

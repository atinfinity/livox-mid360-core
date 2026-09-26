# Architecture

How the layers of `livox-mid360-core` fit together: modules and their allowed dependencies,
the threads and locks, the path of a datagram to a user callback, ownership rules, the error
model and how all of it is tested. The per-layer pages hold the detail; this page is the map.

| Layer | Page | Design issue |
| --- | --- | --- |
| ④ device: `Context`, `Device`, frames, events | [api.md](api.md) | #6, #7, #8, #9 |
| ③ session: `discover()`, `Session`, `HostSetup` | [session.md](session.md) | #4, #5 |
| ② transport: `UdpSocket`, `Poller` | [transport.md](transport.md) | #2 |
| ① protocol: CRC, frames, packets, keys, HMS | [protocol_notes.md](protocol_notes.md) | — |
| test double: `tools/livox_mid360_sim.py` | [simulator.md](simulator.md) | #3 |

Last verified against commit `e429f5e` (2026-09-26). When the code moves, update the
diagrams here first and the per-layer pages second.

## 1. Layers and modules

```mermaid
flowchart TB
  subgraph L4["④ device (threads, callbacks)"]
    context["context.hpp / context.cpp<br/>Context: 3 sockets, receive thread, registry"]
    device["device.hpp / device.cpp<br/>Device: one LiDAR, commands, reconnection"]
    frame["frame.hpp / frame_assembler.*<br/>Point, Frame, FrameAssembler (thread-free)"]
    event["event.hpp / event.cpp<br/>Event, DeviceStats, DisconnectReason"]
  end
  subgraph L3["③ session (synchronous commands, no threads)"]
    session["session.hpp / session.cpp<br/>discover(), Session, RequestOptions"]
    config["config.hpp / config.cpp<br/>HostSetup, apply_host_setup()"]
    sdetail["session_detail.hpp (internal)<br/>match_ack, to_*_ack"]
  end
  subgraph L2["② transport (POSIX UDP, no policy)"]
    transport["transport.hpp / transport.cpp<br/>Endpoint, UdpSocket, Poller, Datagram"]
  end
  subgraph L1["① protocol (pure functions, no I/O)"]
    protocol["protocol.hpp / protocol.cpp<br/>command frames, data packets, key-value lists"]
    keys["keys.hpp / keys.cpp<br/>Key enum, typed encode/decode"]
    hms["hms.hpp / hms.cpp<br/>HMS code table"]
    crc["crc.hpp / crc.cpp<br/>CRC-16 / CRC-32"]
    bytes["bytes.hpp<br/>unaligned little-endian reads"]
  end
  device --> context
  device --> session
  device --> config
  device --> frame
  device --> event
  context --> transport
  frame --> protocol
  session --> sdetail
  sdetail --> protocol
  session --> transport
  config --> session
  config --> keys
  protocol --> crc
  protocol --> bytes
  keys --> bytes
  hms --> keys
```

Rules the arrows encode (enforced by review, not by tooling):

- **Dependencies point down only.** Layer ① never includes anything above it; layer ② knows
  nothing about the Mid-360 protocol; layer ③ creates no threads; layer ④ is the only place
  where threads, callbacks and timers exist.
- **`export.hpp`** is included by every public header for the shared-library visibility
  macros and is not part of the graph. `mid360.hpp` is an umbrella that includes everything.
- **Internal headers** (`context_impl.hpp`, `session_detail.hpp`, `frame_assembler.hpp`) are
  not installed. `session_detail.hpp` and `frame_assembler.hpp` export their symbols anyway
  so the fuzzers and tests can link against them; they are not a stable API.
- **The receive thread only sees `detail::Receiver`** (`context_impl.hpp`): two virtual
  methods, `on_datagram(port, datagram)` and `tick(now)`. `Device::Impl` implements it.
  `Context` never includes `device.hpp` beyond the forward declaration it needs for
  `find()`.

## 2. Threads and locks

```mermaid
flowchart LR
  subgraph user["caller threads (any number)"]
    U1["Device::start_sampling / configure / inquire / reboot<br/>Device::reconnect (enabled = false)"]
    U2["Device::on_* setters, stats(), work_state(), hms()"]
    U3["Device::cancel(), Device::disconnect()"]
  end
  subgraph rx["Context receive thread (one per Context)"]
    R1["Poller::wait ≤ 100 ms"]
    R2["recv_batch on push / point / IMU sockets"]
    R3["dispatch by source IP → Receiver::on_datagram"]
    R4["Receiver::tick: idle frame close, kStats,<br/>push timeout, queued kDisconnected / kReconnected"]
    R5["user callbacks: on_packet, on_frame, on_imu, on_event"]
    R1 --> R2 --> R3 --> R5
    R1 --> R4 --> R5
  end
  subgraph worker["Device worker thread (one per Device, lazy)"]
    W1["wait until disconnected"]
    W2["attempt(): connect / discover, host setup, set mode"]
    W3["backoff 500 ms → 8 s"]
    W1 --> W2 --> W3 --> W1
  end
  U1 -. "cmd_mutex" .-> S[(Session<br/>command socket)]
  W2 -. "cmd_mutex" .-> S
  U3 -. "conn_mutex" .-> S
  R4 -. "wake()" .-> R1
  W2 -. "pending + wake()" .-> R4
```

### Threads

| Thread | Created by | Runs | Must not |
| --- | --- | --- | --- |
| receive thread | `Context::create` | `Context::Impl::run()`: poll, drain, dispatch, timers, **every user callback** | block; call a command; destroy a Device |
| worker | `Device::Impl::declare_disconnected` (first disconnect, `reconnect.enabled`) | `run_worker()`: `attempt()` with exponential backoff until connected or stopped | touch the Context after `remove()` (the destructor joins it first) |
| caller threads | the application | all commands (synchronous, on the caller's thread), setters, observation methods | call commands from a callback (asserted in debug builds) |

`Session` is single-threaded by design: it drives its own `Poller::wait()` on the caller's
thread. `Device` serialises access to it.

### Locks in `Device::Impl`

| Mutex | Protects | Held by |
| --- | --- | --- |
| `cmd_mutex` | the command path: one command or one `attempt()` at a time | commands, `attempt()` (worker and `reconnect()`) |
| `conn_mutex` | `info_`, `session` swap, `pending` events, worker hand-off, `cancel()` | short critical sections everywhere; `run_worker()` sleeps on `conn_cv` with it |
| `push_mutex` | pushed work state and HMS set (`work_state()`, `hms()`) | receive thread (write), observers (read) |
| `cb_mutex` | the four callback slots plus `cb_generation` | setters (write), receive thread (copy when the generation changes) |

Lock order when two are needed: **`cmd_mutex` → `conn_mutex`**. `push_mutex` and `cb_mutex`
are leaf locks (never held while taking another). The receive thread takes only leaf locks
and `conn_mutex` for a swap of the `pending` vector, so a slow command can never stall
reception. `Context::Impl::mutex` guards the registry and is likewise never held while calling
into a Device.

Counters (`DeviceStats`, `ContextStats`) are relaxed atomics written by one thread each;
`stats()` is a lock-free snapshot. Connection state is `std::atomic<bool> connected`, changed
with a compare-exchange so that a disconnect is declared exactly once per outage.

### Callback contract

- Callbacks run on the receive thread with no Device lock held; they may take the
  application's own locks and call `stats()`, `work_state()`, `hms()`, `connected()`.
- They may not call a command (`assert(!on_receive_thread())`), `reconnect()`, or destroy
  the Device or Context.
- The receive thread copies the callback slots when `cb_generation` changes (at the next
  datagram or tick), so a setter returns before the old callback is guaranteed idle; setters
  are refused while sampling is requested (`kInvalidState`), which is what makes the copy
  race-free in practice.
- An exception escaping a callback terminates the process.

## 3. Data and control flows

### 3.1 Open, host setup, start sampling

```mermaid
sequenceDiagram
  participant App
  participant Ctx as Context
  participant Dev as Device
  participant Ses as Session
  participant L as LiDAR
  App->>Ctx: Context::create(opts)
  Note over Ctx: bind push / point / IMU sockets,<br/>start receive thread
  App->>Dev: Device::open(ctx, discovered, opts)
  Dev->>Ses: Session::connect(discovered)
  Ses->>L: 0x0101 inquire 0x8000 (serial)
  L-->>Ses: ACK
  Dev->>Ctx: add(ip, serial, receiver)
  Note over Ctx: registered first so the first<br/>datagrams are not "unknown source"
  Dev->>Ses: apply_host_setup(setup with Context ports)
  Ses->>L: 0x0100 host ipcfg 0x0005/6/7, 0x0000, 0x001C
  L-->>Ses: ACK (0x21 = reboot to apply)
  Dev->>Ctx: bind(receiver, Device*)
  Dev-->>App: unique_ptr<Device>
  App->>Dev: on_frame(cb), on_event(cb)
  App->>Dev: start_sampling()
  Dev->>Ses: 0x0100 work_tgt_mode = SAMPLING
  Dev->>Ses: wait_for_state(SAMPLING) polling 0x8006
  L-->>Ctx: point-cloud / IMU / push datagrams begin
```

Failure anywhere in `open()` unwinds: a host-setup error removes the registry entry again and
returns the wrapped `SessionError`.

### 3.2 A point-cloud datagram to `on_frame`

```mermaid
sequenceDiagram
  participant K as kernel
  participant RX as receive thread
  participant Dev as Device::Impl (Receiver)
  participant FA as FrameAssembler
  participant App
  K-->>RX: poll: point socket readable
  RX->>K: recvmmsg (batch_size, ≤ 8 rounds per socket)
  Note over RX: recv_time_ns from SO_TIMESTAMPNS
  RX->>RX: find Entry by source IP (snapshot)
  RX->>Dev: on_datagram(kPoint, datagram)
  Dev->>Dev: parse_data_packet (+ CRC if verify_crc)
  Dev->>Dev: udp_cnt gap → dropped / reordered
  Dev->>App: on_packet(view, ReceiveInfo)  [raw tier]
  Dev->>FA: push(packet, recv_time)
  Note over FA: timestamp policy, unit conversion,<br/>frame_cnt or time-window close
  FA-->>Dev: closed Frame (optional)
  Dev->>App: on_frame(Frame&&)  [assembled tier]
```

IMU packets take the same path with `on_imu` at the end and no assembler. An unknown source
IP increments `ContextStats::unknown_source` and the datagram is dropped before any parsing.
The idle close of a partial frame and the periodic `kStats` event come from `tick()`, not from
a datagram.

### 3.3 A push to `kStateChanged` / `kHms`

```mermaid
sequenceDiagram
  participant RX as receive thread
  participant Dev as Device::Impl
  participant App
  RX->>Dev: on_datagram(kPush, datagram)
  Dev->>Dev: parse_command_frame, cmd_id == 0x0102, parse_info_push
  Dev->>Dev: pushes++, last_push_time_ns, last_push_steady_ns
  Note over Dev: under push_mutex
  Dev->>Dev: cur_work_state (0x8006) → work_state()
  Dev->>Dev: hms_code (0x8011) → hms()
  Dev->>App: on_event(kStateChanged) if the state differs from the last push
  Dev->>App: on_event(kHms) if the set of active codes changed
```

The push is also the heartbeat: `last_push_steady_ns` feeds the push-timeout check below.

### 3.4 Outage, reconnection and replay

```mermaid
sequenceDiagram
  participant RX as receive thread
  participant Dev as Device::Impl
  participant W as worker
  participant Ctx as Context
  participant L as LiDAR
  participant App
  RX->>Dev: tick() sees now >= last push + push_timeout
  Dev->>Dev: declare_disconnected(kPushTimeout)
  Note over Dev: connected = false, discard frame, queue kDisconnected
  Dev->>W: start (first time) or notify conn_cv
  Dev->>Ctx: poller.wake()
  RX->>App: on_event(kDisconnected)
  loop until connected or Device destroyed
    W->>W: attempt() under cmd_mutex
    W->>L: Session::connect(last endpoint, verify serial)
    alt no answer
      W->>L: discover(targets or broadcast, discovery_timeout)
      L-->>W: ACK filtered by serial, maybe a new IP
      W->>Ctx: rekey(receiver, new ip)
    end
    W->>L: apply_host_setup (recorded at open)
    W->>L: work_tgt_mode = SAMPLING and wait (if sampling was requested)
    W->>Dev: rebase time offset, connected = true, queue kReconnected
    W->>Ctx: poller.wake()
    W->>W: on failure sleep backoff, double up to max_backoff
  end
  RX->>App: on_event(kReconnected)
```

The same `attempt()` runs on the caller's thread for `Device::reconnect()` when automatic
recovery is disabled. The other disconnect triggers (`kCommandTimeout` when a command times
out and the push is stale, `kRebootRequested`, `kUser`) enter the same path through
`declare_disconnected()`.

```mermaid
stateDiagram-v2
  [*] --> Connected: open
  Connected --> Disconnected: push timeout, stale command timeout, reboot, disconnect
  Disconnected --> Disconnected: attempt fails, backoff
  Disconnected --> Connected: attempt succeeds, kReconnected
  Connected --> [*]: destructor
  Disconnected --> [*]: destructor, stop_token cancels the attempt
```

While disconnected, commands fail fast with `kDisconnected`; pushes that still arrive are
parsed and update `work_state()`.

## 4. Ownership and lifetime

```mermaid
flowchart TB
  App["application"] -- "unique_ptr" --> Ctx["Context"]
  App -- "unique_ptr" --> DevA["Device A"]
  App -- "unique_ptr" --> DevB["Device B"]
  Ctx -. "registry: non-owning<br/>{ip, serial, Receiver*, Device*}" .-> DevA
  Ctx -.-> DevB
  DevA -- "owns" --> SesA["Session A<br/>(command socket)"]
  DevA -- "owns" --> WA["worker thread A"]
  DevB -- "owns" --> SesB["Session B"]
  Ctx -- "owns" --> Sock["push / point / IMU sockets,<br/>Poller, receive thread"]
```

- **Context outlives its Devices.** Every Device must be destroyed before the Context; the
  Context destructor asserts an empty registry in debug builds. `Context::find()` and
  `devices()` return non-owning pointers.
- **`Device` and `Context` are non-copyable and non-movable**; the `unique_ptr` doubles as
  the future C handle.
- **Device destruction order**: request stop on the `stop_token` → join the worker (so it
  never touches the Context afterwards) → `Context::Impl::remove()`, which bumps the registry
  generation, wakes the poller and waits on a condition variable until the receive thread has
  taken a new snapshot. After that no callback of the Device is in flight, which is why a
  Device may not be destroyed from its own callback (it would wait for itself).
- **Session hand-off during reconnect**: `attempt()` builds a new `Session` and swaps it into
  `Device::Impl` under `conn_mutex`; `cancel()` takes the same mutex so it always reaches the
  live socket. The `HostSetup` recorded at `open()` is immutable and replayed as is.
- **Registry snapshot**: the receive thread works from a copy of the registry refreshed when
  `generation` changes, so user threads never block it while adding, rekeying or removing
  entries.
- **Buffers**: the receive thread owns `batch_size × kMaxDatagramSize` bytes allocated once;
  `Datagram::data` and `DataPacketView` are views into it, valid during the callback only.
  `Frame` owns its points and is moved to the application.

## 5. Error model

```mermaid
flowchart LR
  T["TransportError<br/>{code, errno}"] --> S["SessionError<br/>{kind, transport, ret_code, error_key, ...}"]
  S --> D["DeviceError<br/>{kind, optional&lt;SessionError&gt;}"]
  P["KeyError / ParseError<br/>(protocol, pure)"] --> S
```

- Every fallible call returns `std::expected<T, E>`; there are no exceptions on any path the
  library controls and `std::error_code` is not used. Errors nest by layer: a socket failure
  during a command is `DeviceError{kSession, SessionError{kTransport, TransportError{...}}}`.
- Protocol-layer parsers return views or `std::expected` with a small error enum and never
  allocate; malformed input is data, not an error condition to log.
- Asynchronous conditions are **events**, not errors: `kDisconnected`, `kReconnected`,
  `kHms`, `kStateChanged`, `kStats`. Counters (`bad_packets`, `dropped_packets`,
  `unknown_source`, `late_acks`, ...) record what was silently dropped.
- The library writes nothing to stdout or stderr on its own. The diagnostic trail (#42,
  `log.hpp`) is off by default and goes only to the handler the application installs; see
  [api.md](api.md#logging).

## 6. Performance model

- **One receive thread per Context** with `poll(2)` and `recvmmsg` in batches of
  `ContextOptions::batch_size` (default 32), up to 8 batches per socket per wake-up so one
  socket cannot starve the others; readiness is level-triggered, so a backlog is drained on
  the next iteration.
- **No per-datagram allocation**: caller-owned buffers, views for packets, one `Frame` vector
  per closed frame. Kernel receive timestamps (`SO_TIMESTAMPNS`) avoid a `clock_gettime` per
  packet on Linux.
- **The poll timeout is the earliest timer** (idle frame close, `kStats`, push timeout), capped
  at 100 ms; a `wake()` through the eventfd cuts it short when a registry change or a queued
  event needs delivery.
- **Cost budget**: a Mid-360 sends about 2000 point-cloud and 200 IMU packets per second, so
  the whole path in 3.2 must stay well under 400 µs per datagram in a release build, callbacks
  included. Measured on a 2-vCPU CI runner, a Debug + ASan + UBSan build needs about 2.5 ms per
  datagram; the simulator-driven tests therefore run at `--rate-multiplier 0.05`. When the
  receive thread saturates, pushes are read late and the push timeout fires, which is what
  that setting avoids. The long-run test on hardware (#13) is where the release numbers get
  confirmed; `epoll`, `SO_REUSEPORT` or `io_uring` are only considered if it fails.
- **Callbacks are on the hot path.** Heavy consumers hand off through `BoundedQueue<T>`
  (newest wins on overflow, drops counted) or their own queue.

## 7. Testing architecture

```mermaid
flowchart LR
  subgraph pure["no I/O"]
    U["Catch2 unit tests<br/>crc, bytes, protocol, keys, hms,<br/>frame assembler, session_detail"]
    G["golden vectors<br/>tests/generated/golden_vectors.hpp<br/>from tools/gen_golden_vectors.py"]
    F["libFuzzer targets (8)<br/>tests/fuzz/, corpus from gen_fuzz_corpus.py"]
  end
  subgraph loop["loopback"]
    T["transport tests<br/>sockets and poller on 127.0.0.1"]
    S["simulator-driven tests<br/>session, config, device, reconnect<br/>tests/sim_process.hpp spawns<br/>tools/livox_mid360_sim.py"]
  end
  subgraph hw["needs-hardware"]
    H["#10 pcap fixtures, #11 open questions,<br/>#12 disconnect / multi-device, #13 long run"]
  end
  PY["tools/livox_mid360_proto.py<br/>Python reference implementation"] --> G
  PY --> S
  G --> U
```

- **Python reference first**: `tools/livox_mid360_proto.py` encodes and decodes the protocol
  independently; the golden vectors it generates pin the C++ parsers, and the simulator is
  built on it. CI regenerates the vectors and fails on a diff.
- **Simulator-driven tests** exercise the real sockets and threads on loopback with fault
  injection over the simulator's stdin (`silence`, `drop_ack`, `set_state`, `reboot`). The
  multi-device test needs `127.0.0.2` on the loopback interface and skips otherwise.
- **Hardware tests** are labelled `needs-hardware` and tracked as issues; every simulator
  assumption is listed in [simulator.md](simulator.md) for verification there.
- **CI matrix** (`.github/workflows/ci.yml`): gcc-13, gcc-14 and clang-19 in Release; gcc-14
  and clang-19 in Debug with ASan + UBSan; each on x86-64 and arm64. Separate jobs run the
  Python reference and simulator unit tests, a short run of every fuzzer, clang-format 19,
  clang-tidy 19, ruff and gcovr coverage. `docker/` reproduces the Ubuntu 24.04 image locally.

## 8. Roadmap pointers

The full status and plan, with tracking issues per item, is in [roadmap.md](roadmap.md). The
items that touch this document most:

- Typed parameter APIs on `Device`: firmware version (#38), FOV (#39), coordinate format /
  scan pattern / frame rate (#40), stored settings and live status read-back (#41), detection
  mode (#46), IMU enable and sensor config (#47).
- Diagnostics: firmware log stream on port 56500, 0x03xx (#44); SDK logging (#42) is in
  [api.md](api.md#logging).
- Point tag accessors (#34), lvx2 record / replay CLI (#35), examples (`examples/`), simulator
  state-machine fidelity (#45).
- C ABI: the mapping table in [api.md](api.md#c-abi-mapping-phase-3); all output structs are
  already plain data with `static_assert`s in `tests/test_api_skeleton.cpp`.

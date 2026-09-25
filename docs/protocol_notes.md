# Protocol notes

Observations, ambiguities and decisions that go beyond the wiki text
("Livox LiDAR Communication Protocol – Mid360", rev v1.4.12, 2026-09-21).
Items marked **[unverified]** must be confirmed against hardware in phase 2 and updated here.

## Data packet header, bytes 12–23

The table defines offset 12, size 12 as `reserved`. The accompanying diagram, and the v1.4.3
changelog ("Add tag_type to the point cloud frame header"), show a `pack_info` field before
`reserved`. Neither the size of `pack_info` nor the encoding of `tag_type` is documented.
The library exposes the 12 bytes verbatim as `DataPacketHeader::reserved` and `decode_tag()`
assumes the single documented tag layout. **[unverified]** Capture real packets and record the
actual contents here.

## Discovery ACK `dev_type`

The wiki does not list the enumeration for `dev_type`. The library keeps it as a raw `uint8_t`.
**[unverified]** Record the value a Mid-360 reports.

## Unaligned fields

`0x0101` ACK has `key_num` at offset 1 and the key-value list at offset 3, so most 16/32-bit
fields are misaligned. All parsing uses `memcpy`-based reads; never cast the buffer to a struct.

## Point cloud CRC coverage

`crc32` at offset 24 covers `timestamp` (offset 28, 8 bytes) followed by `data`. These are
contiguous on the wire, so the CRC is computed over `packet[28 : 28 + 8 + data_len]`.

## Per-point timestamps

`t_i = timestamp + i * time_interval * 100 ns / (dot_num - 1)`. Integer division is used
(truncation, sub-nanosecond). With 96 points and the typical ~103.5 µs interval this is a
~1.09 µs point spacing.

## Command frame CRC32 with empty data

The spec says CRC32 "needs to be padded with 0" when the data length is 0. The library writes
`0` in that case and requires `0` when parsing. (CRC-32 of an empty buffer is also 0, so an
implementation that computes it anyway is compatible.)

## Sequence numbers

`seq_num` is a free-running 32-bit counter per host; ACKs echo it. Wrap-around is not special.
The LiDAR-originated `0x0102` push is a REQ with `sender_type = 1` and, per the sequence
diagram, is not acknowledged by the host.

## Working state

Only `SAMPLING (1)`, `IDLE (2)` and `READY (9)` are valid for `work_tgt_mode`. Setting `1|9`
from `IDLE` passes through `MOTORSTARTUP (6)`; the transition is observable via `cur_work_state`
in the push. `decode_work_state()` rejects undocumented values.

## Return code 0x21 (PARAM_REBOOT_EFFECT)

Treated as success-with-note by callers: the parameter was stored but needs a reboot
(`lidar_ipcfg` is the obvious case).

## HMS table

The wiki lists `0x0210–0x0219` twice (error: "trying to recover"; fatal: "abnormal"). The
level byte in the code distinguishes them; the library carries one description for the range.

## Variants

v1 targets the base Mid-360. `speed_mode` (0x0021) and `pc_freq_mod` (0x0029) exist in the
key table but are not given typed helpers. Spherical output (data type 3) and multicast are
modelled because the base model supports them.

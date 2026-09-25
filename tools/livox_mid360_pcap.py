#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Decode Livox Mid-360 traffic from a pcap / pcapng-less classic pcap file (Ethernet/IPv4/UDP).

Usage:
  livox_mid360_pcap.py capture.pcap            # summary + decoded control frames
  livox_mid360_pcap.py capture.pcap --points   # also dump per-point rows (CSV to stdout)
  livox_mid360_pcap.py capture.pcap --json     # one JSON object per packet

No third-party dependencies. Only classic pcap (magic 0xA1B2C3D4 / 0xA1B23C4D, LINKTYPE_ETHERNET
or LINKTYPE_RAW / LINUX_SLL). Convert pcapng with `tshark -F pcap` or `editcap -F pcap` first.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import struct
import sys
from dataclasses import dataclass

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import livox_mid360_proto as p  # noqa: E402

LIVOX_PORTS = {p.PORT_DISCOVERY, p.PORT_CMD, p.PORT_PUSH, p.PORT_PCL, p.PORT_IMU, p.PORT_LOG}


@dataclass
class Udp:
    ts: float
    src: str
    sport: int
    dst: str
    dport: int
    payload: bytes


def iter_pcap(path: pathlib.Path):
    with path.open("rb") as f:
        gh = f.read(24)
        magic = struct.unpack("<I", gh[:4])[0]
        if magic == 0xA1B2C3D4:
            end, nano = "<", False
        elif magic == 0xA1B23C4D:
            end, nano = "<", True
        elif magic == 0xD4C3B2A1:
            end, nano = ">", False
        elif magic == 0x4D3CB2A1:
            end, nano = ">", True
        else:
            raise SystemExit(
                f"not a classic pcap (magic {magic:#x}); convert pcapng with editcap -F pcap"
            )
        linktype = struct.unpack(end + "I", gh[20:24])[0]
        while True:
            ph = f.read(16)
            if len(ph) < 16:
                return
            sec, frac, incl, _orig = struct.unpack(end + "IIII", ph)
            data = f.read(incl)
            ts = sec + frac / (1e9 if nano else 1e6)
            yield ts, linktype, data


def parse_udp(linktype: int, frame: bytes) -> Udp | None:
    if linktype == 1:  # Ethernet
        if len(frame) < 14:
            return None
        eth = struct.unpack("!H", frame[12:14])[0]
        off = 14
        if eth == 0x8100:
            eth = struct.unpack("!H", frame[16:18])[0]
            off = 18
        if eth != 0x0800:
            return None
    elif linktype == 101:  # RAW IPv4
        off = 0
    elif linktype == 113:  # LINUX_SLL
        if struct.unpack("!H", frame[14:16])[0] != 0x0800:
            return None
        off = 16
    else:
        return None
    ip = frame[off:]
    if len(ip) < 20 or ip[0] >> 4 != 4:
        return None
    ihl = (ip[0] & 0xF) * 4
    if ip[9] != 17:
        return None
    flags_frag = struct.unpack("!H", ip[6:8])[0]
    if flags_frag & 0x1FFF:
        return None  # non-first fragment; Livox packets fit in one datagram
    total = struct.unpack("!H", ip[2:4])[0]
    src = ".".join(map(str, ip[12:16]))
    dst = ".".join(map(str, ip[16:20]))
    udp = ip[ihl:total]
    if len(udp) < 8:
        return None
    sport, dport, ulen = struct.unpack("!HHH", udp[:6])
    return Udp(0.0, src, sport, dst, dport, udp[8:ulen])


def fmt_kv(kvs) -> list[dict]:
    out = []
    for k, v in kvs:
        d = {"key": f"0x{k:04X}", "name": p.KEY_NAMES.get(k, "?"), "len": len(v), "hex": v.hex()}
        if k in (0x8006, 0x001A) and len(v) == 1:
            d["value"] = p.WORK_STATE.get(v[0], v[0])
        elif k in (0x8000, 0x8001):
            d["value"] = v.split(b"\0")[0].decode(errors="replace")
        elif k == 0x8011 and len(v) == 32:
            d["value"] = [f"0x{x:08X}" for x in struct.unpack("<8I", v) if x]
        elif k == 0x8007 and len(v) == 4:
            d["value"] = struct.unpack("<i", v)[0] / 100.0
        elif k in (0x8002, 0x8003, 0x8004) and len(v) == 4:
            d["value"] = ".".join(map(str, v))
        elif k in (0x0005, 0x0006, 0x0007) and len(v) == 8:
            d["value"] = {
                "ip": ".".join(map(str, v[:4])),
                **dict(zip(("dst_port", "src_port"), struct.unpack("<HH", v[4:]), strict=True)),
            }
        out.append(d)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("pcap", type=pathlib.Path)
    ap.add_argument("--points", action="store_true", help="dump per-point CSV rows")
    ap.add_argument("--json", action="store_true", help="emit one JSON object per Livox packet")
    ap.add_argument("--no-crc", action="store_true", help="do not verify CRCs")
    a = ap.parse_args()

    stats = {"cmd": 0, "pcl": 0, "imu": 0, "push": 0, "other": 0, "bad": 0, "drops": 0}
    last_udp_cnt: dict[tuple[str, int], int] = {}
    if a.points:
        print("ts_ns,x,y,z,reflectivity,tag")
    for ts, lt, frame in iter_pcap(a.pcap):
        u = parse_udp(lt, frame)
        if not u or (u.sport not in LIVOX_PORTS and u.dport not in LIVOX_PORTS):
            continue
        u.ts = ts
        try:
            if u.sport in (p.PORT_PCL, p.PORT_IMU):
                pkt = p.DataPacket.parse(u.payload, verify_crc=not a.no_crc)
                kind = "imu" if pkt.data_type == 0 else "pcl"
                stats[kind] += 1
                key = (u.src, u.sport)
                prev = last_udp_cnt.get(key)
                if prev is not None and (prev + 1) & 0xFFFF != pkt.udp_cnt and pkt.udp_cnt != 0:
                    stats["drops"] += 1
                last_udp_cnt[key] = pkt.udp_cnt
                if a.json:
                    print(
                        json.dumps(
                            {
                                "t": ts,
                                "src": u.src,
                                "kind": kind,
                                "data_type": pkt.data_type,
                                "time_type": pkt.time_type,
                                "dot_num": pkt.dot_num,
                                "udp_cnt": pkt.udp_cnt,
                                "timestamp_ns": pkt.timestamp_ns,
                                "time_interval_0p1us": pkt.time_interval,
                                "reserved": pkt.reserved.hex(),
                            }
                        )
                    )
                if a.points and pkt.data_type in (1, 2):
                    scale = 1 if pkt.data_type == 1 else 10
                    for i, s in enumerate(pkt.samples()):
                        print(
                            f"{pkt.sample_timestamp_ns(i)},{s[0] * scale},{s[1] * scale},"
                            f"{s[2] * scale},{s[3]},{s[4]}"
                        )
            else:
                fr = p.CommandFrame.parse(u.payload)
                rec = {
                    "t": ts,
                    "src": f"{u.src}:{u.sport}",
                    "dst": f"{u.dst}:{u.dport}",
                    "cmd_id": f"0x{fr.cmd_id:04X}",
                    "type": "ACK" if fr.cmd_type else "REQ",
                    "sender": "lidar" if fr.sender_type else "host",
                    "seq": fr.seq_num,
                    "len": len(fr.data),
                }
                if fr.cmd_id == 0x0000 and fr.cmd_type == 1:
                    rec["discovery"] = p.parse_discovery_ack(fr.data)
                elif fr.cmd_id == 0x0100 and fr.cmd_type == 0:
                    n = struct.unpack_from("<H", fr.data)[0]
                    rec["set"] = fmt_kv(p.parse_kv_list(fr.data[4:], n))
                elif fr.cmd_id == 0x0100 and fr.cmd_type == 1:
                    rec["ret_code"], rec["error_key"] = p.parse_param_config_ack(fr.data)
                elif fr.cmd_id == 0x0101 and fr.cmd_type == 1:
                    rec["ret_code"], kvs = p.parse_param_inquire_ack(fr.data)
                    rec["values"] = fmt_kv(kvs)
                elif fr.cmd_id == 0x0102:
                    rec["push"] = fmt_kv(p.parse_info_push(fr.data))
                    stats["push"] += 1
                stats["cmd"] += fr.cmd_id != 0x0102
                if a.json or not a.points:
                    print(json.dumps(rec, default=str))
        except ValueError as e:
            stats["bad"] += 1
            if a.json:
                print(
                    json.dumps(
                        {
                            "t": ts,
                            "src": u.src,
                            "sport": u.sport,
                            "error": str(e),
                            "hex": u.payload[:40].hex(),
                        }
                    )
                )
    print(json.dumps({"summary": stats}), file=sys.stderr)


if __name__ == "__main__":
    main()

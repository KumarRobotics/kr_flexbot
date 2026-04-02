#!/usr/bin/env python3
"""
pgv_sdo_scan.py
Scans all CANopen objects on a node via SDO reads.
Usage: python3 pgv_sdo_scan.py [interface] [node_id]
Example: python3 pgv_sdo_scan.py can1 5
"""

import subprocess
import time
import sys
import re

IFACE    = sys.argv[1] if len(sys.argv) > 1 else "can1"
NODE_ID  = int(sys.argv[2]) if len(sys.argv) > 2 else 5

SDO_TX   = 0x600 + NODE_ID   # we send to 0x605
SDO_RX   = 0x580 + NODE_ID   # we receive from 0x585

def send_sdo_read(obj_idx, sub_idx=0):
    """Send SDO read request: cansend can1 605#40 idx_lo idx_hi sub 00000000"""
    idx_lo = obj_idx & 0xFF
    idx_hi = (obj_idx >> 8) & 0xFF
    frame = f"{SDO_TX:03X}#40{idx_lo:02X}{idx_hi:02X}{sub_idx:02X}00000000"
    subprocess.run(["cansend", IFACE, frame],
                   capture_output=True)

def read_responses(timeout=0.15):
    """Capture candump output for timeout seconds, return lines matching SDO_RX"""
    rx_id = f"{SDO_RX:03X}"
    result = subprocess.run(
        ["timeout", str(timeout), "candump", IFACE],
        capture_output=True, text=True
    )
    lines = []
    for line in result.stdout.splitlines():
        if rx_id in line:
            lines.append(line.strip())
    return lines

def parse_response(line):
    """Parse SDO response line, return (is_abort, obj_idx, sub_idx, value)"""
    # Extract hex bytes after [8]
    match = re.search(r'\[8\]\s+((?:[0-9A-Fa-f]{2}\s*){8})', line)
    if not match:
        return None
    raw = match.group(1).split()
    if len(raw) < 8:
        return None

    cmd     = int(raw[0], 16)
    sub_idx = int(raw[3], 16)
    obj_idx = int(raw[1], 16) | (int(raw[2], 16) << 8)

    is_abort = (cmd == 0x80)

    # Decode value (little-endian bytes 4-7)
    value = (int(raw[4], 16) |
             (int(raw[5], 16) << 8) |
             (int(raw[6], 16) << 16) |
             (int(raw[7], 16) << 24))

    return is_abort, obj_idx, sub_idx, value

def scan_object(obj_idx, sub_indices=range(0, 5)):
    results = {}
    for sub in sub_indices:
        send_sdo_read(obj_idx, sub)
        time.sleep(0.05)
        lines = read_responses(timeout=0.1)
        for line in lines:
            parsed = parse_response(line)
            if parsed:
                is_abort, ridx, rsub, val = parsed
                if not is_abort:
                    results[sub] = (val, line)
    return results

def main():
    print(f"Scanning {IFACE} node {NODE_ID} (SDO tx=0x{SDO_TX:03X} rx=0x{SDO_RX:03X})")
    print("="*70)

    found = {}

    # Scan ranges most likely to contain config
    ranges = [
        range(0x1000, 0x1030),   # standard CANopen
        range(0x1400, 0x1A00),   # PDO config
        range(0x1F00, 0x1F30),   # sync/timing
        range(0x2000, 0x2020),   # vendor specific
        range(0x2100, 0x2120),
        range(0x2200, 0x2220),
        range(0x2300, 0x2320),
        range(0x3000, 0x3020),   # vendor specific
        range(0x4000, 0x4010),
        range(0x5000, 0x5010),
        range(0x6000, 0x6010),
    ]

    for r in ranges:
        for obj_idx in r:
            send_sdo_read(obj_idx, 0)
            time.sleep(0.05)
            lines = read_responses(timeout=0.08)
            for line in lines:
                parsed = parse_response(line)
                if parsed:
                    is_abort, ridx, rsub, val = parsed
                    if not is_abort:
                        # Found a valid object — scan its sub-indices too
                        print(f"\n  ✓ Object 0x{ridx:04X} sub={rsub} = 0x{val:08X} ({val})")
                        found[ridx] = val
                        # Scan sub-indices 1-8
                        for sub in range(1, 9):
                            send_sdo_read(ridx, sub)
                            time.sleep(0.05)
                            sublines = read_responses(timeout=0.08)
                            for sl in sublines:
                                sp = parse_response(sl)
                                if sp:
                                    sa, si, ss, sv = sp
                                    if not sa:
                                        print(f"      sub={ss} = 0x{sv:08X} ({sv})")

    print("\n" + "="*70)
    print(f"Scan complete. Found {len(found)} objects:")
    for idx, val in sorted(found.items()):
        print(f"  0x{idx:04X} = 0x{val:08X} ({val})")

if __name__ == "__main__":
    main()

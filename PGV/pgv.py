#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PGV100RS Live Dashboard (Non-scrolling)
Shows only meaningful values and updates in place
"""

import struct
import subprocess
import sys
import time
import os

# ─────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────

NODE_ID = 0x05
ANGLE_RES = 0.1
POS_RES = 0.1

TPDO2 = 0x280 + NODE_ID
TPDO3 = 0x380 + NODE_ID
TPDO4 = 0x480 + NODE_ID
PDO5  = 0x680 + NODE_ID


# ─────────────────────────────────────────
# Shared State (latest values)
# ─────────────────────────────────────────

state = {
    "x_position_mm": 0.0,
    "timestamp_ms": 0,
    "y_offset_mm": 0.0,
    "angle_deg": 0.0,
    "speed_mm_s": 0.0,
    "tag_id": 0,
}


# ─────────────────────────────────────────
# Decoders
# ─────────────────────────────────────────

def decode_tpdo2(d):
    if len(d) < 6:
        return
    y_raw = struct.unpack_from('>i', d, 0)[0]
    angle_raw = struct.unpack_from('>h', d, 4)[0]
    state["y_offset_mm"] = round(y_raw * POS_RES, 2)
    state["angle_deg"] = round(angle_raw * ANGLE_RES, 2)


def decode_tpdo3(d):
    if len(d) < 8:
        return
    tag = d[7]
    if tag != 0:
        state["tag_id"] = tag


def decode_tpdo4(d):
    if len(d) < 8:
        return
    x_raw = struct.unpack_from('>i', d, 0)[0]
    ts = struct.unpack_from('>I', d, 4)[0]
    state["x_position_mm"] = round(x_raw * POS_RES, 2)
    state["timestamp_ms"] = ts


def decode_pdo5(d):
    if len(d) < 2:
        return
    speed = struct.unpack_from('>h', d, 0)[0]
    state["speed_mm_s"] = round(speed * 0.1, 2)


# ─────────────────────────────────────────
# Dashboard Print
# ─────────────────────────────────────────

def clear():
    os.system("clear")


def print_dashboard():
    clear()
    print("==============================================")
    print("      PGV100RS LIVE DASHBOARD")
    print("==============================================\n")

    print(" X Position   : {:>8} mm".format(state["x_position_mm"]))
    print(" Timestamp    : {:>8} ms".format(state["timestamp_ms"]))
    print(" Y Offset     : {:>8} mm".format(state["y_offset_mm"]))
    print(" Angle        : {:>8} deg".format(state["angle_deg"]))
    print(" Speed        : {:>8} mm/s".format(state["speed_mm_s"]))
    print(" Tag ID       : {:>8}".format(state["tag_id"]))

    print("\nPress CTRL+C to stop.")


# ─────────────────────────────────────────
# Live CAN Reader
# ─────────────────────────────────────────

def live_dashboard(interface="can1"):

    process = subprocess.Popen(
        ["candump", interface],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        universal_newlines=True
    )

    last_refresh = time.time()

    try:
        for line in process.stdout:

            parts = line.strip().split()
            if len(parts) < 4:
                continue

            try:
                can_id = int(parts[1], 16)
                data = bytes(int(x, 16) for x in parts[3:])
            except:
                continue

            if can_id == TPDO2:
                decode_tpdo2(data)
            elif can_id == TPDO3:
                decode_tpdo3(data)
            elif can_id == TPDO4:
                decode_tpdo4(data)
            elif can_id == PDO5:
                decode_pdo5(data)

            # Refresh screen at 10 Hz
            if time.time() - last_refresh > 0.1:
                print_dashboard()
                last_refresh = time.time()

    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)


# ─────────────────────────────────────────

if __name__ == "__main__":

    interface = "can1"
    if len(sys.argv) > 1:
        interface = sys.argv[1]

    live_dashboard(interface)

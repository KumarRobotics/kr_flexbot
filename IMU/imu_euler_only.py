import serial
import struct
import time

PORT = "/dev/ttymxc4"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

def send_packet(mid, data=b''):
    pkt = bytearray([0xFA, 0xFF, mid, len(data)])
    pkt.extend(data)
    checksum = (256 - (sum(pkt[1:]) & 0xFF)) & 0xFF
    pkt.append(checksum)
    ser.write(pkt)
    time.sleep(0.1)

# 1) Go to config mode
send_packet(0x30)

# 2) Set output configuration: Euler angles @ 100 Hz
# DID = 0x2030, Freq = 100 (0x0064)
cfg = bytes.fromhex("20 30 00 64")
send_packet(0xC0, cfg)

# 3) Go to measurement mode
send_packet(0x10)

print("Streaming Euler angles (roll, pitch, yaw)...")

while True:
    if ser.read(1) != b'\xFA':
        continue
    if ser.read(1) != b'\xFF':
        continue

    mid = ser.read(1)[0]
    length = ser.read(1)[0]
    payload = ser.read(length)
    ser.read(1)  # checksum

    # MTData2
    if mid != 0x36:
        continue

    i = 0
    while i < len(payload):
        did = payload[i] << 8 | payload[i+1]
        size = payload[i+2]
        data = payload[i+3:i+3+size]

        # Euler angles
        if did == 0x2030:
            roll, pitch, yaw = struct.unpack(">fff", data)
            print(f"ROLL {roll: .3f}  PITCH {pitch: .3f}  YAW {yaw: .3f}")

        i += 3 + size

#!/usr/bin/env python3
import serial
import struct
import time

# ---------------- IMU SERIAL ----------------
IMU_PORT = "/dev/ttymxc4"
IMU_BAUD = 115200

imu = serial.Serial(IMU_PORT, IMU_BAUD, timeout=1)

# ---------------- LCD SERIAL ----------------
LCD_PORT = "/dev/ttymxc2"
LCD_BAUD = 57600
CMD = 0xFE

lcd = serial.Serial(LCD_PORT, LCD_BAUD, timeout=1)

LCD_LINES = [0x00, 0x40, 0x14, 0x54]

def lcd_cmd(c):
    lcd.write(bytes([CMD, c]))
    time.sleep(0.002)

def lcd_goto(addr):
    lcd.write(bytes([CMD, 0x45, addr]))
    time.sleep(0.002)

def lcd_write(text):
    lcd.write(text.ljust(20)[:20].encode("ascii"))

def lcd_clear_all():
    for a in LCD_LINES:
        lcd_goto(a)
        lcd_write("")

# ---------------- IMU COMMANDS ----------------
def send_packet(mid, data=b''):
    pkt = bytearray([0xFA, 0xFF, mid, len(data)])
    pkt.extend(data)
    checksum = (256 - (sum(pkt[1:]) & 0xFF)) & 0xFF
    pkt.append(checksum)
    imu.write(pkt)
    time.sleep(0.05)

# ---------------- IMU INIT ----------------
send_packet(0x30)  # Config mode

cfg = bytes.fromhex("40 20 00 64 80 20 00 64")
send_packet(0xC0, cfg)

send_packet(0x10)  # Measurement mode

# ---------------- MAIN LOOP ----------------
last_lcd = 0
LCD_PERIOD = 0.2  # 5 Hz

lcd_cmd(0x41)
time.sleep(0.1)
lcd_clear_all()

print("Streaming IMU data to LCD...")

while True:
    if imu.read(1) != b'\xFA':
        continue
    if imu.read(1) != b'\xFF':
        continue

    mid = imu.read(1)[0]
    length = imu.read(1)[0]
    payload = imu.read(length)
    imu.read(1)  # checksum

    if mid != 0x36:
        continue

    acc_x = acc_y = acc_z = None
    gyr_x = gyr_y = gyr_z = None

    i = 0
    while i < len(payload):
        did = payload[i] << 8 | payload[i+1]
        size = payload[i+2]
        data = payload[i+3:i+3+size]

        if did == 0x4020:
            acc_x, acc_y, acc_z = struct.unpack(">fff", data)
        elif did == 0x8020:
            gyr_x, gyr_y, gyr_z = struct.unpack(">fff", data)

        i += 3 + size

    if acc_x is None or gyr_x is None:
        continue

    if time.time() - last_lcd < LCD_PERIOD:
        continue
    last_lcd = time.time()

    lcd_goto(0x00)
    lcd_write(f"ACC X:{acc_x:5.2f} Y:{acc_y:5.2f}")

    lcd_goto(0x40)
    lcd_write(f"ACC Z:{acc_z:5.2f} m/s2")

    lcd_goto(0x14)
    lcd_write(f"GYR X:{gyr_x:5.2f} Y:{gyr_y:5.2f}")

    lcd_goto(0x54)
    lcd_write(f"GYR Z:{gyr_z:5.2f} rad/s")

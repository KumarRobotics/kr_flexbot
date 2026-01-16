#!/usr/bin/env python3
import serial
import time

LCD_PORT = "/dev/ttymxc2"
BAUDRATE = 57600
CMD = 0xFE


lcd = serial.Serial(
    port=LCD_PORT,
    baudrate=BAUDRATE,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

def lcd_cmd(cmd):
    lcd.write(bytes([CMD, cmd]))
    time.sleep(0.02)

def lcd_goto(pos):
    lcd.write(bytes([CMD, 0x45, pos]))
    time.sleep(0.02)

def lcd_write(text):
    lcd.write(text.encode("ascii"))
    
def lcd_clear_all():
    for pos in (0x00, 0x40, 0x14, 0x54):
        lcd_goto(pos)
        lcd_write(" " * 20)


# ---- TEST ----
lcd_cmd(0x41)        # init
time.sleep(0.1)

lcd_clear_all()

lcd_goto(0x00)
lcd_write("Dhyey is the BEST".ljust(20))

lcd_goto(0x40)
lcd_write("Team UPenn !!!".ljust(20))


lcd.close()

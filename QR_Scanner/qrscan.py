#!/usr/bin/env python3
from evdev import InputDevice, categorize, ecodes
from datetime import datetime
import os

# ==== CONFIG ====
# Path to scanner device
DEVICE_PATH = "/dev/input/by-id/usb-BF_SCAN_SCAN_KEYBOARD_A-00000-event-kbd"
LOGFILE = "scanner.log"

### We can remove if we want appended logging for all sessions 
open(LOGFILE, "w").close()

# ==== KEYMAP ====
# Normal keys
KEYMAP = {
    'KEY_0': '0', 'KEY_1': '1', 'KEY_2': '2', 'KEY_3': '3', 'KEY_4': '4',
    'KEY_5': '5', 'KEY_6': '6', 'KEY_7': '7', 'KEY_8': '8', 'KEY_9': '9',
    'KEY_A': 'a', 'KEY_B': 'b', 'KEY_C': 'c', 'KEY_D': 'd', 'KEY_E': 'e',
    'KEY_F': 'f', 'KEY_G': 'g', 'KEY_H': 'h', 'KEY_I': 'i', 'KEY_J': 'j',
    'KEY_K': 'k', 'KEY_L': 'l', 'KEY_M': 'm', 'KEY_N': 'n', 'KEY_O': 'o',
    'KEY_P': 'p', 'KEY_Q': 'q', 'KEY_R': 'r', 'KEY_S': 's', 'KEY_T': 't',
    'KEY_U': 'u', 'KEY_V': 'v', 'KEY_W': 'w', 'KEY_X': 'x', 'KEY_Y': 'y',
    'KEY_Z': 'z',

    'KEY_MINUS': '-', 'KEY_EQUAL': '=', 'KEY_LEFTBRACE': '[', 'KEY_RIGHTBRACE': ']',
    'KEY_BACKSLASH': '\\', 'KEY_SEMICOLON': ';', 'KEY_APOSTROPHE': "'",
    'KEY_COMMA': ',', 'KEY_DOT': '.', 'KEY_SLASH': '/', 'KEY_GRAVE': '`',
    'KEY_SPACE': ' ',
}

# Shifted symbols
SHIFTED = {
    'KEY_1': '!', 'KEY_2': '@', 'KEY_3': '#', 'KEY_4': '$', 'KEY_5': '%',
    'KEY_6': '^', 'KEY_7': '&', 'KEY_8': '*', 'KEY_9': '(', 'KEY_0': ')',
    'KEY_MINUS': '_', 'KEY_EQUAL': '+', 'KEY_LEFTBRACE': '{', 'KEY_RIGHTBRACE': '}',
    'KEY_BACKSLASH': '|', 'KEY_SEMICOLON': ':', 'KEY_APOSTROPHE': '"',
    'KEY_COMMA': '<', 'KEY_DOT': '>', 'KEY_SLASH': '?', 'KEY_GRAVE': '~',
}

SHIFT_KEYS = {'KEY_LEFTSHIFT', 'KEY_RIGHTSHIFT'}

# ==== INITIALIZE ====
dev = InputDevice(DEVICE_PATH)

# Exclusive access to scanner
dev.grab()  
barcode = ""
shift = False

print(f"Scanner opened: {DEVICE_PATH}")
print(f"Logging to: {LOGFILE}")
print("Waiting for scans...\n")

# MAIN LOOP
for event in dev.read_loop():
    if event.type == ecodes.EV_KEY:
        key = categorize(event)

        # Track shift key state
        if key.keycode in SHIFT_KEYS:
            shift = key.keystate == key.key_down
            continue

        # Only process key_down events
        if key.keystate != key.key_down:
            continue

        # Enter = end of barcode
        if key.keycode == 'KEY_ENTER':
            timestamp = datetime.now().isoformat()
            with open(LOGFILE, "a") as f:
                f.write(f"{timestamp} {barcode}\n")
            # print(f"[{timestamp}] {barcode}")  # Optional print to terminal
            barcode = ""
        else:
            if shift and key.keycode in SHIFTED:
                barcode += SHIFTED[key.keycode]
            else:
                barcode += KEYMAP.get(key.keycode, "")


#!/usr/bin/env python3
import time

# LED paths
leds = {
    'red': '/sys/class/leds/red:status/brightness',
    'green': '/sys/class/leds/green:status/brightness', 
    'blue': '/sys/class/leds/blue:status/brightness'
}

# Turn off all
for led in leds.values():
    with open(led, 'w') as f: f.write('0')
time.sleep(0.5)

# 1. Red for 2 seconds
print("RED")
with open(leds['red'], 'w') as f: f.write('255')
time.sleep(2)

# 2. Green for 2 seconds  
print("GREEN")
with open(leds['red'], 'w') as f: f.write('0')
with open(leds['green'], 'w') as f: f.write('255')
time.sleep(2)

# 3. Blue for 2 seconds
print("BLUE")
with open(leds['green'], 'w') as f: f.write('0')
with open(leds['blue'], 'w') as f: f.write('255')
time.sleep(2)

# 4. Blinking white 5 times
print("BLINKING WHITE")
for i in range(5):
    # All ON
    for led in leds.values():
        with open(led, 'w') as f: f.write('255')
    time.sleep(0.3)
    
    # All OFF
    for led in leds.values():
        with open(led, 'w') as f: f.write('0')
    time.sleep(0.3)

print("Sequence complete")

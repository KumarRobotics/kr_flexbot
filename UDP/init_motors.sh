#!/bin/bash

LED_SCRIPT="/home/bg_bot/UDP/set_led.sh"

echo "=============================="
echo " Initializing Left Motor..."
echo "=============================="

cd /home/bg_bot/MOTORS || exit 1

./init_left_motor

if [ $? -ne 0 ]; then
    echo "Left motor initialization FAILED"
    $LED_SCRIPT 255 0 0
    exit 1
fi

sleep 1

echo "=============================="
echo " Initializing Right Motor..."
echo "=============================="

./init_right_motor

if [ $? -ne 0 ]; then
    echo "Right motor initialization FAILED"
    $LED_SCRIPT 255 0 255
    exit 1
fi

echo "=============================="
echo " Both Motors Initialized"
echo "=============================="

# Yellow = motors initialized, comm not started yet
$LED_SCRIPT 255 255 0

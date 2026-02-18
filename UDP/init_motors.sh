#!/bin/bash

echo "=============================="
echo " Initializing Left Motor..."
echo "=============================="

cd /home/bg_bot/MOTORS || exit 1

./init_left_motor

if [ $? -ne 0 ]; then
    echo "Left motor initialization FAILED"
    exit 1
fi

sleep 1

echo "=============================="
echo " Initializing Right Motor..."
echo "=============================="

./init_right_motor

if [ $? -ne 0 ]; then
    echo "Right motor initialization FAILED"
    exit 1
fi

echo "=============================="
echo " Both Motors Initialized"
echo "=============================="


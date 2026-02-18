#!/bin/bash
set -e
cd /home/bg_bot/UDP

# Start motor controller
./motor_controller > motor_controller.log 2>&1 &
MC_PID=$!
echo "motor_controller PID=$MC_PID"

# Start IMU UDP TX
./imu_udp_tx > imu_udp_tx.log 2>&1 &
IMU_PID=$!
echo "imu_udp_tx PID=$IMU_PID"

# Kill both on Ctrl+C / termination
trap "echo 'Stopping...'; kill $MC_PID $IMU_PID; exit 0" INT TERM

wait

#!/bin/bash
set -e

cd /home/bg_bot/UDP
LED_SCRIPT="/home/bg_bot/UDP/set_led.sh"

# Start motor controller
./motor_controller > motor_controller.log 2>&1 &
MC_PID=$!
echo "motor_controller PID=$MC_PID"

# Start IMU UDP TX
./imu_udp_tx > imu_udp_tx.log 2>&1 &
IMU_PID=$!
echo "imu_udp_tx PID=$IMU_PID"

# Start UDP command server
./udp_cmd_server > udp_cmd_server.log 2>&1 &
CMD_PID=$!
echo "udp_cmd_server PID=$CMD_PID"

sleep 1

# Verify processes are still alive
if ! kill -0 $MC_PID 2>/dev/null || ! kill -0 $IMU_PID 2>/dev/null || ! kill -0 $CMD_PID 2>/dev/null; then
    echo "One or more comm processes failed to start"
    $LED_SCRIPT 255 0 0
    exit 1
fi

# Green = comm stack running
$LED_SCRIPT 0 255 0

cleanup() {
    echo "Stopping..."
    kill $MC_PID $IMU_PID $CMD_PID 2>/dev/null || true
    wait $MC_PID $IMU_PID $CMD_PID 2>/dev/null || true
    # Back to red when stopped/fault
    $LED_SCRIPT 255 0 0
    exit 0
}

trap cleanup INT TERM

wait

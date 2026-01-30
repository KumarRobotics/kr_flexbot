#!/usr/bin/env bash
set -euo pipefail

ETH="eth0"
JETSON_IP="192.168.0.21/24"
IMX7_IP="192.168.0.20"

if [[ $EUID -ne 0 ]]; then
  echo "Run as root: sudo $0"
  exit 1
fi

ip link set "$ETH" up
ip addr flush dev "$ETH" || true
ip addr add "$JETSON_IP" dev "$ETH"

echo "Jetson $ETH up. Jetson IP = $JETSON_IP"
echo "Test from Jetson: ping -c 3 $IMX7_IP"

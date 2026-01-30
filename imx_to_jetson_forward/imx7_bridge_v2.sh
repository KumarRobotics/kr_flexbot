#!/usr/bin/env bash
set -euo pipefail

# Config
ETH_LIDAR="eth1"
ETH_JETSON="eth0"
BR="br0"
IMX7_IP="192.168.0.20/24"

# Must be root
if [[ $EUID -ne 0 ]]; then
  echo "Run as root: sudo $0"
  exit 1
fi

# Bring ports down (ignore errors)
ip link set "$ETH_LIDAR" down 2>/dev/null || true
ip link set "$ETH_JETSON" down 2>/dev/null || true

# If bridge exists from a previous run, remove it (makes this script re-runnable)
if ip link show "$BR" &>/dev/null; then
  ip link set "$BR" down 2>/dev/null || true
  ip link del "$BR" type bridge 2>/dev/null || true
fi

# Create bridge + bring up
ip link add name "$BR" type bridge
ip link set "$BR" up

# Attach both ports to the bridge
ip link set "$ETH_LIDAR" master "$BR"
ip link set "$ETH_JETSON" master "$BR"

# Flush any old IPs on both ports (not just eth1)
ip addr flush dev "$ETH_LIDAR" || true
ip addr flush dev "$ETH_JETSON" || true

# Put IP on the bridge device
ip addr add "$IMX7_IP" dev "$BR"

# Bring ports up
ip link set "$ETH_LIDAR" up
ip link set "$ETH_JETSON" up

echo "Bridge $BR up. IMX7 IP = $IMX7_IP"
echo "Test from IMX7: ping -c 3 192.168.0.21"

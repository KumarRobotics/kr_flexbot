#!/usr/bin/env bash
set -euo pipefail

LIDAR_PORT="${LIDAR_PORT:-2115}"
JETSON_IP="${JETSON_IP:-192.168.0.21}"

echo "Forwarding UDP port $LIDAR_PORT -> $JETSON_IP:$LIDAR_PORT"
echo "Press Ctrl+C to stop."

# Prefer socat if available (best for UDP relay)
if command -v socat &>/dev/null; then
  exec socat -u UDP-RECVFROM:"$LIDAR_PORT",reuseaddr,fork UDP-SENDTO:"$JETSON_IP":"$LIDAR_PORT"
fi

# Next best: ncat (from nmap), supports persistent -k
if command -v ncat &>/dev/null; then
  exec ncat -u -l -k -p "$LIDAR_PORT" | ncat -u "$JETSON_IP" "$LIDAR_PORT"
fi

# Fallback to your original nc loop
echo "Warning: socat/ncat not found; falling back to nc loop (less reliable)."
while true; do
  nc -u -l -p "$LIDAR_PORT" | nc -u "$JETSON_IP" "$LIDAR_PORT"
done

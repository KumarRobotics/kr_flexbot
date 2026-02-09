#!/usr/bin/env python3
"""
High-Level Robot Controller
Sends: (left_rpm, right_rpm) via UDP
Receives: (left_enc, right_enc, timestamp) via UDP
"""
import socket
import struct
import time
import threading

# Configuration
IMX7_IP = '192.168.1.50'  # iMX7 IP address
CMD_PORT = 5001  # Send commands here
ENC_PORT = 5002  # Receive encoder data here

# Robot parameters
WHEEL_RADIUS_M = 0.05  # 5cm wheel radius
WHEEL_BASE_M = 0.3  # 30cm between wheels

# UDP Sockets
cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
enc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
enc_socket.bind(('0.0.0.0', ENC_PORT))
enc_socket.settimeout(0.1)

# Encoder state
latest_left_enc = 0
latest_right_enc = 0
latest_timestamp = 0.0

def send_wheel_velocities(left_rpm, right_rpm):
    """Send wheel velocities to low-level controller"""
    data = struct.pack('ff', left_rpm, right_rpm)
    cmd_socket.sendto(data, (IMX7_IP, CMD_PORT))
    print(f"Sent: L={left_rpm:.1f} R={right_rpm:.1f} RPM")

def differential_drive_ik(linear_vel_ms, angular_vel_rads):
    """
    Inverse kinematics: Convert (v, ω) to wheel velocities
    
    Args:
        linear_vel_ms: Forward velocity in m/s
        angular_vel_rads: Angular velocity in rad/s
    
    Returns:
        (left_rpm, right_rpm)
    """
    # Differential drive kinematics
    v_left = linear_vel_ms - (angular_vel_rads * WHEEL_BASE_M / 2.0)
    v_right = linear_vel_ms + (angular_vel_rads * WHEEL_BASE_M / 2.0)
    
    # Convert m/s to RPM
    # v = ω * r  =>  ω = v / r (rad/s)
    # RPM = (ω * 60) / (2 * π)
    left_rpm = (v_left / WHEEL_RADIUS_M) * 60.0 / (2.0 * 3.14159)
    right_rpm = (v_right / WHEEL_RADIUS_M) * 60.0 / (2.0 * 3.14159)
    
    return left_rpm, right_rpm

def encoder_receiver_thread():
    """Receive encoder data from low-level"""
    global latest_left_enc, latest_right_enc, latest_timestamp
    
    print("Encoder receiver started")
    
    while True:
        try:
            data, addr = enc_socket.recvfrom(1024)
            
            if len(data) == 16:  # 2 ints + 1 double = 16 bytes
                left_enc, right_enc, timestamp = struct.unpack('iid', data)
                
                latest_left_enc = left_enc
                latest_right_enc = right_enc
                latest_timestamp = timestamp
                
                # Debug print occasionally
                if int(timestamp * 2) % 10 == 0:
                    print(f"Encoders: L={left_enc} R={right_enc}")
        
        except socket.timeout:
            continue
        except Exception as e:
            print(f"Encoder receive error: {e}")
            time.sleep(0.1)

# Example usage
if __name__ == "__main__":
    print("="*70)
    print("HIGH-LEVEL CONTROLLER")
    print("="*70)
    
    # Start encoder receiver
    receiver_thread = threading.Thread(target=encoder_receiver_thread, daemon=True)
    receiver_thread.start()
    
    time.sleep(1)
    
    # Example 1: Direct wheel control
    print("\nExample 1: Direct wheel velocities")
    send_wheel_velocities(50, 50)  # Both wheels 50 RPM forward
    time.sleep(2)
    send_wheel_velocities(0, 0)  # Stop
    time.sleep(1)
    
    # Example 2: Using twist commands
    print("\nExample 2: Using differential drive kinematics")
    
    # Forward 0.2 m/s
    left_rpm, right_rpm = differential_drive_ik(0.2, 0.0)
    send_wheel_velocities(left_rpm, right_rpm)
    time.sleep(2)
    
    # Rotate in place (0.5 rad/s)
    left_rpm, right_rpm = differential_drive_ik(0.0, 0.5)
    send_wheel_velocities(left_rpm, right_rpm)
    time.sleep(2)
    
    # Stop
    send_wheel_velocities(0, 0)
    
    print("\n✓ Test complete")
#!/usr/bin/env python3
"""
Low-Level Motor Controller with UDP Communication
Receives: (left_rpm, right_rpm) from high-level
Sends: (left_enc, right_enc, timestamp) to high-level
"""
import can
import struct
import time
import socket
import threading

# Configuration
CAN_INTERFACE = 'can0'
CMD_PER_WHEEL_RPM = 3814.0
UDP_LISTEN_PORT = 5001  # Listen for commands
UDP_SEND_IP = '192.168.0.20'  # Jetson IP
UDP_SEND_PORT = 5002  # Send encoder data here
CONTROL_RATE_HZ = 50  # 50Hz control loop
ENCODER_SEND_RATE_HZ = 20  # 20Hz encoder feedback

# Initialize CAN
bus = can.Bus(channel=CAN_INTERFACE, interface='socketcan')

# UDP Socket for receiving commands
cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_socket.bind(('0.0.0.0', UDP_LISTEN_PORT))
cmd_socket.settimeout(0.01)  # Non-blocking

# UDP Socket for sending encoder data
enc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Shared state
current_left_rpm = 0.0
current_right_rpm = 0.0
state_lock = threading.Lock()

def send_cmd_motor(node, ctrl, vel=0):
    """Send RPDO command to motor"""
    rpdo_id = 0x500 + node
    msg = can.Message(
        arbitration_id=rpdo_id,
        data=struct.pack('<HiH', ctrl, vel, 0),
        is_extended_id=False
    )
    bus.send(msg)
    time.sleep(0.001)

def send_sync():
    """Send SYNC to trigger motor updates"""
    sync = can.Message(arbitration_id=0x080, data=[], is_extended_id=False)
    bus.send(sync)
    time.sleep(0.001)

def read_encoder(node):
    """Read encoder position from motor via SDO (0x6064)"""
    req_id = 0x600 + node
    resp_id = 0x580 + node
    
    # Send SDO read request for position (0x6064)
    msg = can.Message(
        arbitration_id=req_id,
        data=[0x40, 0x64, 0x60, 0x00, 0, 0, 0, 0],
        is_extended_id=False
    )
    bus.send(msg)
    
    # Wait for response
    timeout = time.time() + 0.05
    while time.time() < timeout:
        response = bus.recv(timeout=0.01)
        if response and response.arbitration_id == resp_id:
            if len(response.data) >= 8:
                # Parse 32-bit signed integer (encoder counts)
                encoder_counts = struct.unpack('<i', bytes(response.data[4:8]))[0]
                return encoder_counts
    return None

def udp_receiver_thread():
    """Thread to receive velocity commands from high-level"""
    global current_left_rpm, current_right_rpm
    
    print("UDP receiver started on port", UDP_LISTEN_PORT)
    
    while True:
        try:
            data, addr = cmd_socket.recvfrom(1024)
            
            # Expected format: struct.pack('ff', left_rpm, right_rpm)
            if len(data) == 8:  # 2 floats = 8 bytes
                left_rpm, right_rpm = struct.unpack('ff', data)
                
                with state_lock:
                    current_left_rpm = left_rpm
                    current_right_rpm = right_rpm
                    
                print(f"Received: L={left_rpm:.1f} R={right_rpm:.1f} RPM")
        
        except socket.timeout:
            continue
        except Exception as e:
            print(f"UDP receive error: {e}")
            time.sleep(0.1)

def motor_control_loop():
    """Main control loop - sends commands to motors"""
    print("Motor control loop started")
    
    dt = 1.0 / CONTROL_RATE_HZ
    
    while True:
        start_time = time.time()
        
        # Get current target velocities
        with state_lock:
            left_rpm = current_left_rpm
            right_rpm = current_right_rpm
        
        # Convert to motor commands
        left_cmd = int(round(left_rpm * CMD_PER_WHEEL_RPM))
        right_cmd = int(round(right_rpm * CMD_PER_WHEEL_RPM))
        
        # Send to motors
        send_cmd_motor(1, 0x000F, left_cmd)  # Left motor
        send_cmd_motor(2, 0x000F, right_cmd)  # Right motor
        send_sync()
        
        # Sleep to maintain rate
        elapsed = time.time() - start_time
        sleep_time = max(0, dt - elapsed)
        time.sleep(sleep_time)

def encoder_feedback_loop():
    """Send encoder data back to high-level"""
    print("Encoder feedback loop started")
    
    dt = 1.0 / ENCODER_SEND_RATE_HZ
    
    while True:
        start_time = time.time()
        
        # Read encoders
        left_enc = read_encoder(1)
        right_enc = read_encoder(2)
        timestamp = time.time()
        
        if left_enc is not None and right_enc is not None:
            # Pack: left_enc (int32), right_enc (int32), timestamp (float64)
            data = struct.pack('iid', left_enc, right_enc, timestamp)
            
            # Send via UDP
            enc_socket.sendto(data, (UDP_SEND_IP, UDP_SEND_PORT))
            
            # Debug print occasionally
            if int(timestamp * 2) % 10 == 0:
                print(f"Encoders: L={left_enc} R={right_enc}")
        
        # Sleep to maintain rate
        elapsed = time.time() - start_time
        sleep_time = max(0, dt - elapsed)
        time.sleep(sleep_time)

def emergency_stop():
    """Stop both motors immediately"""
    print("\nEmergency stop!")
    for _ in range(50):
        send_cmd_motor(1, 0x000F, 0)
        send_cmd_motor(2, 0x000F, 0)
        send_sync()
        time.sleep(0.001)

if __name__ == "__main__":
    print("="*70)
    print("LOW-LEVEL MOTOR CONTROLLER")
    print("="*70)
    print(f"Listening for commands on UDP port {UDP_LISTEN_PORT}")
    print(f"Sending encoder data to {UDP_SEND_IP}:{UDP_SEND_PORT}")
    print(f"Control rate: {CONTROL_RATE_HZ} Hz")
    print(f"Encoder rate: {ENCODER_SEND_RATE_HZ} Hz")
    print("="*70)
    
    try:
        # Start threads
        receiver_thread = threading.Thread(target=udp_receiver_thread, daemon=True)
        control_thread = threading.Thread(target=motor_control_loop, daemon=True)
        encoder_thread = threading.Thread(target=encoder_feedback_loop, daemon=True)
        
        receiver_thread.start()
        control_thread.start()
        encoder_thread.start()
        
        # Keep main thread alive
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        emergency_stop()
        bus.shutdown()
        print("\n✓ Shutdown complete")
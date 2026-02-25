#!/usr/bin/env python3
import can
import time

bus = can.Bus(channel='can0', interface='socketcan')

for node in [3, 4]:
    print(f"\nPinging Motor Node {node}...")
    
    req_id = 0x600 + node
    resp_id = 0x580 + node
    
    msg = can.Message(
        arbitration_id=req_id,
        data=[0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0],  # Read statusword
        is_extended_id=False
    )
    bus.send(msg)
    time.sleep(0.1)
    
    response = bus.recv(timeout=0.5)
    if response and response.arbitration_id == resp_id:
        print(f"  ✓ Node {node} responded!")
        print(f"    Response: {response}")
    else:
        print(f"  ✗ No response from Node {node}")

bus.shutdown()
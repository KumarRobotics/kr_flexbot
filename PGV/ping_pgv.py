#!/usr/bin/env python3
import can
import time

# Try both CAN buses
for bus_name in ['can0', 'can1']:
    print(f"\n{'='*60}")
    print(f"Scanning {bus_name} for PGV (Node 5)")
    print('='*60)
    
    try:
        bus = can.Bus(channel=bus_name, interface='socketcan')
        
        # Try SDO ping
        print(f"Trying SDO ping on Node 5...")
        req_id = 0x605
        resp_id = 0x585
        
        msg = can.Message(
            arbitration_id=req_id,
            data=[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0],  # Read device type
            is_extended_id=False
        )
        bus.send(msg)
        time.sleep(0.1)
        
        response = bus.recv(timeout=0.5)
        if response and response.arbitration_id == resp_id:
            print(f"  ✓ Node 5 (PGV) responded on {bus_name}!")
            print(f"    Response: {response}")
        else:
            print(f"  ✗ No SDO response from Node 5")
        
        # Listen for any broadcasts from node 5
        print(f"\nListening for Node 5 broadcasts on {bus_name}...")
        start = time.time()
        node5_ids = set()
        
        while time.time() - start < 3:
            msg = bus.recv(timeout=0.5)
            if msg:
                # Node 5 IDs: 0x185, 0x285, 0x385, 0x485, 0x505, 0x585, 0x605
                if msg.arbitration_id in [0x185, 0x285, 0x385, 0x485, 0x505, 0x585, 0x605]:
                    if msg.arbitration_id not in node5_ids:
                        data_str = ' '.join('%02X' % b for b in msg.data)
                        print(f"  Found: 0x{msg.arbitration_id:03X} [{len(msg.data)}]: {data_str}")
                        node5_ids.add(msg.arbitration_id)
        
        if not node5_ids:
            print(f"  No Node 5 broadcasts detected")
        
        bus.shutdown()
        
    except Exception as e:
        print(f"  Error: {e}")

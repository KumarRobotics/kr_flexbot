#!/usr/bin/env python3
"""
Battery SOC Calculator for 15S LiFePO4 Pack
Uses voltage-based SOC with proper LiFePO4 discharge curve
"""
import can
import struct
import time
import sys

class LiFePO4_SOC:
    """LiFePO4 State of Charge estimator"""
    
    # LiFePO4 voltage-to-SOC lookup table (per cell)
    # Format: (voltage, soc_percent)
    VOLTAGE_SOC_TABLE = [
        (3.65, 100),  # Fully charged
        (3.40, 99),   # Start of flat region
        (3.35, 90),
        (3.32, 70),
        (3.30, 40),
        (3.27, 30),
        (3.25, 20),
        (3.20, 10),
        (3.10, 5),
        (3.00, 0),    # Empty (don't discharge below this!)
        (2.50, 0),    # Dead (protection should kick in)
    ]
    
    @staticmethod
    def voltage_to_soc(cell_voltage):
        """Convert cell voltage to SOC percentage"""
        
        # Clamp to table range
        if cell_voltage >= LiFePO4_SOC.VOLTAGE_SOC_TABLE[0][0]:
            return 100.0
        if cell_voltage <= LiFePO4_SOC.VOLTAGE_SOC_TABLE[-1][0]:
            return 0.0
        
        # Linear interpolation between table points
        for i in range(len(LiFePO4_SOC.VOLTAGE_SOC_TABLE) - 1):
            v_high, soc_high = LiFePO4_SOC.VOLTAGE_SOC_TABLE[i]
            v_low, soc_low = LiFePO4_SOC.VOLTAGE_SOC_TABLE[i + 1]
            
            if v_low <= cell_voltage <= v_high:
                # Linear interpolation
                ratio = (cell_voltage - v_low) / (v_high - v_low)
                soc = soc_low + ratio * (soc_high - soc_low)
                return soc
        
        return 0.0
    
    @staticmethod
    def get_status_message(soc):
        """Get human-readable battery status"""
        if soc > 80:
            return "Excellent", "🟢"
        elif soc > 50:
            return "Good", "🟡"
        elif soc > 20:
            return "Low - Charge Soon", "🟠"
        elif soc > 10:
            return "Critical - Charge Now!", "🔴"
        else:
            return "EMERGENCY - Return to Base!", "⚠️"

def decode_battery_data(can_id, data):
    """Decode battery CAN messages"""
    
    if can_id == 0x1B1:  # Primary battery status
        voltage_raw = struct.unpack('<H', bytes(data[0:2]))[0]
        voltage_v = voltage_raw / 256.0
        
        current_raw = struct.unpack('<h', bytes(data[4:6]))[0]
        current_a = current_raw / 10.0
        
        power_w = voltage_v * current_a
        
        # Calculate SOC using proper LiFePO4 curve
        cell_voltage = voltage_v / 15.0  # 15S pack
        soc_pct = LiFePO4_SOC.voltage_to_soc(cell_voltage)
        
        # Estimate remaining capacity (30Ah nominal)
        capacity_remain_ah = (soc_pct / 100.0) * 30.0
        
        # Estimate runtime (if discharging)
        if current_a < -0.1:  # Discharging
            runtime_hours = capacity_remain_ah / abs(current_a)
            runtime_minutes = runtime_hours * 60
        else:
            runtime_hours = None
            runtime_minutes = None
        
        return {
            'voltage_v': voltage_v,
            'current_a': current_a,
            'power_w': power_w,
            'soc_pct': soc_pct,
            'cell_voltage': cell_voltage,
            'capacity_ah': capacity_remain_ah,
            'runtime_min': runtime_minutes
        }
    
    elif can_id == 0x4B1:  # Temperature
        temp1_raw = struct.unpack('<H', bytes(data[0:2]))[0]
        temp1_c = temp1_raw / 100.0
        
        temp2_raw = struct.unpack('<H', bytes(data[2:4]))[0]
        temp2_c = temp2_raw / 100.0
        
        cycles = struct.unpack('<H', bytes(data[4:6]))[0]
        
        return {
            'temp1_c': temp1_c,
            'temp2_c': temp2_c,
            'cycles': cycles,
        }
    
    return None

def main():
    try:
        bus = can.Bus(channel='can1', interface='socketcan')
    except Exception as e:
        print("Error opening CAN1:", e)
        sys.exit(1)
    
    print("="*70)
    print("BERKSHIRE ROBOT - BATTERY MONITOR")
    print("63V 30Ah LiFePO4 Pack (15S3P)")
    print("="*70)
    print()
    
    battery_state = {}
    last_print = time.time()
    
    try:
        while True:
            msg = bus.recv(timeout=0.5)
            
            if msg and msg.arbitration_id in [0x1B1, 0x4B1]:
                data = decode_battery_data(msg.arbitration_id, msg.data)
                if data:
                    battery_state[msg.arbitration_id] = data
                
                # Print summary every second
                now = time.time()
                if now - last_print >= 1.0:
                    print("\033[2J\033[H")  # Clear screen
                    print("="*70)
                    print(" "*20 + "BATTERY STATUS")
                    print("="*70)
                    print()
                    
                    if 0x1B1 in battery_state:
                        s = battery_state[0x1B1]
                        status_msg, emoji = LiFePO4_SOC.get_status_message(s['soc_pct'])
                        
                        # Main info
                        print(f"  SOC:              {s['soc_pct']:5.1f} %  {emoji}")
                        print(f"  Status:           {status_msg}")
                        print()
                        print(f"  Pack Voltage:     {s['voltage_v']:5.2f} V")
                        print(f"  Cell Voltage:     {s['cell_voltage']:5.3f} V (avg)")
                        print(f"  Current:          {s['current_a']:5.1f} A", end='')
                        if s['current_a'] < -0.1:
                            print(" (discharging)")
                        elif s['current_a'] > 0.1:
                            print(" (charging)")
                        else:
                            print(" (idle)")
                        print(f"  Power:            {abs(s['power_w']):5.0f} W")
                        print()
                        print(f"  Capacity Left:    {s['capacity_ah']:5.1f} Ah (of 30.0 Ah)")
                        
                        if s['runtime_min'] is not None:
                            if s['runtime_min'] > 60:
                                print(f"  Est. Runtime:     {s['runtime_min']/60:5.1f} hours")
                            else:
                                print(f"  Est. Runtime:     {s['runtime_min']:5.0f} minutes")
                        
                        print()
                    
                    if 0x4B1 in battery_state:
                        s = battery_state[0x4B1]
                        print(f"  Temperature 1:    {s['temp1_c']:5.1f} °C")
                        print(f"  Temperature 2:    {s['temp2_c']:5.1f} °C")
                        print(f"  Charge Cycles:    {s['cycles']:5d}")
                    
                    print()
                    print("="*70)
                    
                    # Charging recommendations
                    if 0x1B1 in battery_state:
                        soc = battery_state[0x1B1]['soc_pct']
                        if soc < 20:
                            print("⚠️  CHARGE NOW - Battery critically low!")
                        elif soc < 30:
                            print("⚠️  Battery low - Charge when convenient")
                        elif soc > 95:
                            print("✓  Battery fully charged")
                    
                    print()
                    print("Press Ctrl+C to stop")
                    last_print = now
    
    except KeyboardInterrupt:
        print("\n\n✓ Stopped")
    
    bus.shutdown()

if __name__ == "__main__":
    main()

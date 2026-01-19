"""
Docstring for Communication.imx7_comms
IMX7 SBC - Sensor Data Transmitter & Command Receiver
Receives: RPM commands, LCD display stringm LED color
"""

import socket
import struct
import json
import threading
import time

class IMX7Communication:
    def __init__(self, jetson_ip, send_port, recv_port):
        self.jetson_ip = jetson_ip
        self.send_port = send_port
        self.recv_port = recv_port


        # socket for sending sensor data to Jetson
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # socket for receiving commands from Jetson
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(('', recv_port))
        self.recv_sock.settimeout(0.1)

        # commmand vars

        self.target_rpm = 0.0
        self.lcd_text = ""
        self.led_color = ""

        self.running = True

    def pack_sensor_data(self, rpm, imu_data, encoder_data, tof_data):
        """

        Pack sensor data into a byte stream
        
        :param rpm: rotations per minute
        :param imu_data: imu data to be sent
        :param encoder_data: encoder data to be sent
        :param tof_data: tof data to be send
        """

        data = {
            'rpm': rpm,
            'imu': {
                'accel_x':imu_data['accel_x'],
                'accel_y': imu_data['accel_y'],
                'accel_z': imu_data['accel_z'],
                'gyro_x': imu_data['gyro_x'],
                'gyro_y': imu_data['gyro_y'],
                'gyro_z': imu_data['gyro_z'],
                'mag_x': imu_data.get('mag_x', 0),
                'mag_y': imu_data.get('mag_y', 0),
                'mag_z': imu_data.get('mag_z', 0)
            },
            'encoder': {
                'position': encoder_data['position'],
                'velocity': encoder_data['velocity'],
            },
            'tof': {
                'distance': tof_data['distance']
            },
            'timestamp': time.time()
        }
        return json.dumps(data).encode('utf-8')



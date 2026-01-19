"""
Docstring for Communication.jetson_comms
Jetson - Sensor Data Receiver & Command Transmitter
Receives: IMU, Motor Encoder, ToF data
Sends: RPM commands, LCD display string, LED color
"""

import socket
import json
import threading
import time

class JetsonCommunication:
    def __init__(self, imx7_ip, recv_port=5000, send_port=5001):
        self.imx7_ip = imx7_ip
        self.recv_port = recv_port
        self.send_port = send_port

        # socket for receiving sensor data from IMX7
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(('', recv_port))
        self.recv_sock.settimeout(0.1)

        # socket for sending commands to IMX7
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


        # current sensor data

        self.latest_data = {
            # 'rpm': 0,
            'imu': {},
            'encoder': {},
            'tof': {},
            'timestamp': 0
        }

        self.running = True
        self.data_callback = None

    def receive_sensor_data(self):
        """" 
        receive sensor data from IMX7 (runs in separate thread)
        """

        while self.running:
            try:
                pass
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving sensor data: {e}")

    def send_command(self, left_rpm=None, right_rpm=None, lcd_text=None, led_color=None):
        """
        send command to IMX7
        
        :param self: Description
        :param left_rpm: left wheel rpm command
        :param right_rpm: right wheel rpm command
        :param lcd_text: lcd commmand 
        :param led_color: led command
        """

        cmd = {}

        if left_rpm is not None:
            cmd['left_rpm'] = left_rpm

        if right_rpm is not None:
            cmd['right_rpm'] = right_rpm

        if lcd_text is not None:
            cmd['lcd'] = lcd_text

        if led_color is not None:
            cmd['led'] = led_color

        if cmd:
            try:
                packet = json.dumps(cmd).encode('utf-8')
                self.send_sock.sendto(packet, (self.imx7_ip, self.send_port))
            except Exception as e:
                print(f"Error sending command: {e}")

        def start_receiver_thread(self):
            """
            start the sensor data receiver in a background thread
            """

            recv_thread = threading.Thread(target=self.receive_sensor_data, daemon=True)
            recv_thread.start()
            return recv_thread
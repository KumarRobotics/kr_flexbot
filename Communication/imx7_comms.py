"""
Docstring for Communication.imx7_comms
IMX7 SBC - Sensor Data Transmitter & Command Receiver
Receives: RPM commands, LCD display string, LED color
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

        self.target_left_rpm = 0.0
        self.target_right_rpm = 0.0
        self.lcd_text = ""
        self.led_color = ""

        self.running = True

    def pack_sensor_data(self, l_rpm, r_rpm, imu_data, encoder_data, tof_data):
        """

        Pack sensor data into a byte stream
        
        :param rpm: rotations per minute
        :param imu_data: imu data to be sent
        :param encoder_data: encoder data to be sent
        :param tof_data: tof data to be send
        """

        data = {
            'left_rpm': l_rpm,
            'right_rpm': r_rpm,
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
    
    def send_sensor_data(self, l_rpm, r_rpm, imu_data, encoder_data, tof_data):
        """
        send sensor data to Jetson
        
        """

        packet = self.pack_sensor_data(l_rpm=l_rpm, r_rpm=r_rpm, imu_data=imu_data, encoder_data=encoder_data,tof_data=tof_data)
        try:
            self.send_sock.sendto(packet, (self.jetson_ip, self.send_port))
        except Exception as e:
            print(f"Error sending data: {e}")

    def receive_commands(self):
        while self.running:
            try:
                data, addr = self.recv_sock.recvfrom(4096)
                cmd = json.loads(data.decode('utf-8'))
                if 'left_rpm' in cmd:
                    self.target_left_rpm = cmd['left_rpm']
                    print(f"received rpm command: {self.target_left_rpm}")

                if 'right_rpm' in cmd:
                    self.target_right_rpm = cmd['right_rpm']
                    print(f"received rpm command: {self.target_right_rpm}")

                if 'lcd' in cmd:
                    self.lcd_text = cmd['lcd']
                    print(f"received LCD text: {self.lcd_text}")

                    # TODO: update LCD displays.

                if 'led' in cmd:
                    self.led_color = cmd['led']
                    print(f"recieved LED color: {self.led_color}")
                    # TODO: update LED displays. Perhaps convert to approp. format

            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving commands: {e}")

    def start_receiver_thread(self):
        """
        start the command receiver in background thread
        """
        recv_thread = threading.Thread(target=self.receive_commands, daemon=True)
        recv_thread.start()
        return recv_thread
    
    def close(self):
        self.running = False
        self.send_sock.close()
        self.recv_sock.close()


if __name__=='__main__':
    JETSON_IP = "" # choose a jetson ip.
    SEND_PORT = 5000
    RECV_PORT = 5001

    comm = IMX7Communication(JETSON_IP, SEND_PORT, RECV_PORT)

    # start receiver thread
    comm.start_receiver_thread()

    print(f"Sending sensor data to {JETSON_IP}:{SEND_PORT}")
    print(f"Listening for commands on port {RECV_PORT}")

    try:
        while True:

            # set some dummy values for testing
            left_rpm = 50
            right_rpm = 50

            imu_data = {
                'accel_x': 0.1, 'accel_y' : 0.2, 'accel_z': 9.8,
                'gyro_x': 0.01, 'gyro_y': 0.02, 'gyro_z': 0.03
            }

            encoder_data = {
                'position': 12345,
                'velocity': 10.5
            }

            tof_data = {
                'distance': 250.5
            }

            comm.send_sensor_data(left_rpm, r_rpm=right_rpm, imu_data=imu_data, encoder_data=encoder_data, tof_data=tof_data)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nShutting down...")
        comm.close()

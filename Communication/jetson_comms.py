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
        
        def set_data_callback(self, callback):
            """ Set call back to be called when new data arrives"""
            self.data_callback = callback

        def get_current_data(self):
            return self.latest_data.copy()
        
        def close(self):
            """Clean up sockets"""
            self.running = False
            self.recv_sock.close()
            self.send_sock.close()


if __name__=="__main__":
    IMX7_IP = "" # set an IP for IMX7
    RECV_PORT = 5000
    SEND_PORT = 5001

    def on_data_received(data):
        """ Callback function for processing received sensor data"""

        print(f"\n--- Sensor Data Update ---")
        print(f"Left RPM: {data['left_rpm']}, Right RPM: {data['right_rpm']}")


    comm = JetsonCommunication(IMX7_IP, RECV_PORT, SEND_PORT)
    comm.set_data_callback(on_data_received)

    comm.start_receiver_thread()

    try:
        
        command_counter = 0

        while True:
            time.sleep(1)
            command_counter += 1

            if command_counter % 5 == 0:
                new_left_rpm = 35 + (command_counter * 100) % 20
                new_right_rpm = 35 + (command_counter * 100) % 20
                comm.send_command(left_rpm=new_left_rpm, right_rpm=new_right_rpm)
                print(f"Sent RPM commands: left = {new_left_rpm}, right= {new_right_rpm}")


            if command_counter % 7 == 0:
                # Update LCD
                lcd_msg = f"Status Ok {command_counter}"
                comm.send_command(lcd_text=lcd_msg)
                print(f"Sent LCD text: {lcd_msg}")

            if command_counter % 10 == 0:
                colors = ["red", "green", "blue"]
                color = colors[(command_counter // 10) % len(colors)]
                comm.send_command(led_color=color)
                print(f"Sent LED color: {color}")

            if command_counter % 15 == 0:
                comm.send_command(
                    left_rpm=50,
                    right_rpm=50,
                    lcd_text="Controlled speed",
                    led_color="red"
                )
                print("send all commands for test")


    except KeyboardInterrupt:
        print("\nShutting down...")
        comm.close()

#!/usr/bin/env python3

"""
ROS2 Node

Subscribes to camera servo angle commands and sends the mapped servo value to the Arduino over serial.

Subscribed topics:
    - /motor_cmd
        The node receives the servo angle from this topic which is published by 'motor_input.py' or 
        'panorama_capture.py', and sends the mapped servo value to the Arduino as a newline-terminated serial message.

Launch arguments (serial connection):
    - serial_port (default: /dev/ttyUSB0) 
    - baud_rate (default: 115200)

Angle mapping:
    - Arduino's servo library expects value within 0-180.
    - So, target servo angle from /motor_cmd (range of -110 to +110 degree) is mapped to 0-180 range for Arduino.

    -110 degrees -> 0      (min. angle value)
    0 degrees -> 90        (centre angle value)
    +110 degrees -> 180    (max. angle value)

Related nodes:
    - motor_input.py
        Publishes manual servo angle commands to /motor_cmd.

    - panorama_capture.py
        Publishes servo angle commands to /motor_cmd during an automated
        panorama sweep.    

CH341 / CH340 driver note (for the Jetson):
    The Arduino Nano/CH340 serial adapter may require the CH341/CH340 driver
    on the Jetson if the built-in ch341 driver is missing or not working.

    Initial installation (only needed once):
        git clone https://github.com/juliagoda/CH341SER.git
        cd CH341SER
        make
        sudo make load

    Since CH341SER is already on the Jetson now, normally only this is needed
    if the Arduino does not appear as /dev/ttyUSB0:
        cd CH341SER
        sudo make load

Troubleshooting:
    - If the Arduino device is not listed:

        Check:
            ls /dev/ttyUSB* /dev/ttyACM*
            lsusb
            dmesg | tail -50
            dmesg | grep -iE "ch34|ttyUSB|ttyACM"

    - If CH340 appears in lsusb but /dev/ttyUSB0 does not appear:

        First try:
            sudo modprobe ch341

        Then unplug and reconnect the Arduino USB cable.

    - If the built-in driver still does not work:

        Load the CH341SER driver:
            cd CH341SER
            sudo make load

    - If /dev/ttyUSB0 still does not appear:

        Try a different USB cable or USB port.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Default parameters for serial connection
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        try:
            self.ser = serial.Serial(serial_port, int(baud_rate), timeout=1) # Open serial connection to Arduino.
            time.sleep(2) # Wait for Arduino reset after serial connection opens.
            self.ser.reset_input_buffer() # Clear any startup serial data.
            self.get_logger().info(f"Connected to Arduino on {serial_port} at {baud_rate} baud")

        except Exception as e: # Handle serial connection failure
            self.get_logger().error(f"Serial init failed on {serial_port} at {baud_rate} baud: {e}")
            raise  # Stop node from starting if serial fails
        
        # Subscriber
        self.subscription = self.create_subscription(
            Float64,
            '/motor_cmd',
            self.callback,
            10
        )

        self.get_logger().info("Motor controller ready")

    def callback(self, msg):
        cmd = round(msg.data)

        # Enforce angle limits        
        if cmd > 110:
            self.get_logger().warn(f"Command {cmd} exceeds max limit of 110, clamping to 110")
            cmd = 110
        elif cmd < -110:
            self.get_logger().warn(f"Command {cmd} exceeds min limit of -110, clamping to -110")
            cmd = -110

        # Map target angle (-110 to +110 deg) to 0-180 range for Arduino
        arduino_input = int((cmd + 110) * (180 / 220))

        command = f"{arduino_input}\n" # Add newline character to indicate to the Arduino that the command is complete
        self.ser.write(command.encode())  # Send command over serial to  (what moves it)
        self.get_logger().info(f"Received: {cmd} → Sent to arduino (mapped): {arduino_input}") 

    # Clean-up
    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open: # Check if object has serial connection and if serial port is open
            self.ser.close() # Close serial connection
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

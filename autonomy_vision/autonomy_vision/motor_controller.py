#!/usr/bin/env python3

"""
ROS2 Node

The node 'motor_controller.py' recieves servo angle from /motor_cmd topic which is published by 'motor_input.py' and sends mapped servo value to the Arduino over serial.

Arduino's servo library expects value within 0-180.
So, target servo angle from /motor_cmd (range of -135 to +135 degree) is mapped to 0-180 range for Arduino.

For instance:
-135 degrees corresponds to 0 for Arduio, indicated min. angle value
0 degrees corresponds to 90 for Arduio, indicated centre angle value
+135 degrees corresponds to 180 for Arduio, indicated max. angle value

Troubleshooting:
Device not listed (/dev/ttyUSB* or /dev/ttyACM* missing):

   Check:
       ls /dev/ttyUSB* /dev/ttyACM*   (see if device appears)
       lsusb                         (check if USB device is detected at all)
       dmesg | tail -50             (check kernel detection messages)

   If nothing appears:

   - Reload CH340 driver
         sudo modprobe ch341

   - Then unplug and reconnect the Arduino USB cable

   - If still not detected:
         Try a different USB port or cable
------------------------------------------------------------
Additional check if still failing:
   If 'lsusb' does NOT show the device at all, then it is a hardware/cable/USB port issue

   If 'lsusb' shows CH340 but 'ls /dev/ttyUSB*' does not show anything, then its a driver issue.
       Re-run: sudo modprobe ch341
       Unplug and reconnect the Arduino USB cable
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1) # Open serial connection to Arduino nano
            time.sleep(2) # Wait for Arduino reset after serial connection
            self.ser.reset_input_buffer() # Clear startup serial buffer data
        except Exception as e: # Handle serial connection failure
            self.get_logger().error(f"Serial init failed: {e}")
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
        if cmd > 135:
            self.get_logger().warn(f"Command {cmd} exceeds max limit of 135, clamping to 135")
            cmd = 135
        elif cmd < -135:
            self.get_logger().warn(f"Command {cmd} exceeds min limit of -135, clamping to -135")
            cmd = -135

        # Map target angle (-135 to +135 deg) to 0-180 range for Arduino
        arduino_input = int((cmd + 135) * (180 / 270))

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
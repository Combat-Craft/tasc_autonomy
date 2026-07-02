#!/usr/bin/env python3

"""
ROS2 Node

Allows the user to control the camera servo and trigger a panorama sweep.

User inputs:
    - A number from -135 to +135: 
        Publishes a servo angle command
    - 'p':
        Triggers the panorama sweep
    - 'e':
        Exits the node

Published topics:
    - /motor_cmd
    - /panorama_trigger

Related nodes:
    - motor_controller.py:
        Subscribes to /motor_cmd and sends the converted servo command to Arduino.

    - panorama_stitcher.py:
        Subscribes to /panorama_trigger and begins the panorama sweep.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorInput(Node):
    def __init__(self):
        super().__init__('motor_input')

        # Publisher
        self.pub = self.create_publisher(Float64, '/motor_cmd', 10)
        self.pano_trigger_pub = self.create_publisher(Float64, '/panorama_trigger', 10)


    # Publish user command (target angle) to /motor_cmd
    def send(self, angle):
        msg = Float64()
        msg.data = float(angle)
        self.pub.publish(msg)
        self.get_logger().info(f"Sent: {angle}")

    def trigger_panorama(self):
        msg = Float64()
        msg.data = 999.0
        self.pano_trigger_pub.publish(msg)
        self.get_logger().info("Panorama trigger sent")


def main():
    rclpy.init()
    node = MotorInput()
    try:
        while True: # Recieves input from user continuously and it exits when user inputs 'e'
            val = input("Enter angle (-135 to 135 | 'p' for panorama | 'e' to exit): ") # Prompt user
            if val == 'e': # Exit
                break
            elif val == 'p':
                node.trigger_panorama()
            else:
                try: # Send command to ROS2 topic
                    node.send(float(val)) 
                except ValueError: # Invalid
                    print("Invalid number")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

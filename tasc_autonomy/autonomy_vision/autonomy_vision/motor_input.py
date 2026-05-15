#!/usr/bin/env python3

"""
ROS2 Node

motor_input.py is a ROS2 node that allows the user to enter servo angles within the range of -135 to +135 degrees.
It publishes the command ot the /motor_cmd topic.

ROS2 node 'motor_controller.py' subscribes to /motor_cmd and forwards the servo command to the Arduino.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorInput(Node):
    def __init__(self):
        super().__init__('motor_input')

        # Publisher
        self.pub = self.create_publisher(Float64, '/motor_cmd', 10)

    # Publish user command (target angle) to /motor_cmd
    def send(self, angle):
        msg = Float64()
        msg.data = float(angle)
        self.pub.publish(msg)
        self.get_logger().info(f"Sent: {angle}")

def main():
    rclpy.init()
    node = MotorInput()
    try:
        while True: # Recieves input from user continuously and it exits when user inputs 'e'
            val = input("Enter angle (-135 to 135 | enter 'e' to exit): ") # Prompt user
            if val == 'e': # Exit
                break
            try: # Send command to ROS2 topic
                node.send(float(val)) 
            except ValueError: # Invalid
                print("Invalid number")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
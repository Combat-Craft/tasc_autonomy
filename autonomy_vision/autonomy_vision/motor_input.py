#!/usr/bin/env python3

"""
ROS2 Node

Allows the user to manually control the camera servo and trigger a named panorama sweep.

User inputs:
    - A number from -120 to +120:
        Publishes a servo angle command to /motor_cmd.

    - 'p,folder_name':
        Triggers a panorama sweep and sends the requested folder name to /panorama_trigger.
        The panorama_capture.py node uses this folder name when saving the sweep images.

        Example:
            p,test_sweep_01

    - 'e':
        Exits the node.

Published topics:
    - /motor_cmd
        Type: std_msgs/msg/Float64
        Sends manual servo angle commands.

    - /panorama_trigger
        Type: std_msgs/msg/String
        Sends the requested panorama sweep folder name.

Related nodes:
    - motor_controller.py:
        Subscribes to /motor_cmd and sends the converted servo command to the Arduino.

    - panorama_capture.py:
        Subscribes to /panorama_trigger, creates the requested sweep folder,
        moves the servo through the sweep angles, and saves captured images.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

class MotorInput(Node):
    def __init__(self):
        super().__init__('motor_input')

        # Publishers
        self.servo_pub = self.create_publisher(Float64, '/motor_cmd', 10)
        self.pano_trigger_pub = self.create_publisher(String, '/panorama_trigger', 10)

    def send(self, angle):
        """
        Publishes a manual servo angle command to /motor_cmd.
        """

        msg = Float64()
        msg.data = float(angle)
        self.servo_pub.publish(msg)
        self.get_logger().info(f"Sent: {angle}")

    def trigger_panorama(self, folder_name):
        """
        Publishes a panorama trigger message.

        The folder name is sent as a String so panorama_capture.py can create
        a named output folder for the sweep images and metadata inside the main 
        folder (/panorama_images).
        """

        msg = String()
        msg.data = folder_name
        self.pano_trigger_pub.publish(msg)
        self.get_logger().info(f"Panorama trigger sent with folder name: {folder_name}")


def main():
    rclpy.init()
    node = MotorInput()

    try:
        while True: # Recieves input from user continuously and it exits when user inputs 'e'
            val = input("Enter angle (-120 to 120 | 'p,folder_name' for panorama | 'e' to exit): ").strip() # Prompt user
            if val == 'e': # Exit
                break
            elif val.startswith("p,"): # Trigger panorama sweep with a user-provided folder name
                folder_name = val.split(",", 1)[1].strip()

                if folder_name == "":
                    print("Invalid format. Use: p,folder_name")
                else:
                    node.trigger_panorama(folder_name)
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

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class PanController(Node):

    def __init__(self):
        super().__init__('pan_controller')

        # Adjustable parameters
        self.declare_parameter("min_angle", -135.0)
        self.declare_parameter("max_angle", 135.0)
        self.declare_parameter("max_speed", 5.0) 
        self.declare_parameter("acceleration", 1.0)
        self.declare_parameter("update_rate", 0.05)

        self.min_angle = self.get_parameter("min_angle").value
        self.max_angle = self.get_parameter("max_angle").value
        self.max_speed = self.get_parameter("max_speed").value
        self.acceleration = self.get_parameter("acceleration").value
        update_rate = self.get_parameter("update_rate").value

        # Initialize
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.current_speed = 0.0 

        # Subscriber receives target angle
        self.create_subscription(
            Float64,
            '/pan_angle_cmd',
            self.cmd_callback,
            10
        )

        # Publishes to servo controller
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/camera_pan_position_controller/commands',
            10
        )

        self.timer = self.create_timer(update_rate, self.update)

        self.get_logger().info("Pan Controller Started")

    # Called when a new pan command is recieved
    def cmd_callback(self, msg):

        angle = msg.data

        # Warn when angle limits exceeded
        if angle > self.max_angle or angle < self.min_angle:
            self.get_logger().warn("Command exceeded limits")

        # Clamp within limits
        angle = max(min(angle, self.max_angle), self.min_angle)
        self.target_angle = angle

        self.get_logger().info(f"Target pan angle: {angle:.1f}°")

    # Smooth motion with acceleration ramping
    def update(self):
        error = self.target_angle - self.current_angle

        # Stop if very close to target angle
        if abs(error) < 0.01:
            self.current_angle = self.target_angle
            self.current_speed = 0.0
        else:
            # Determine direction
            direction = 1 if error > 0 else -1

            # Ramp speed up or down
            if abs(error) < (self.current_speed**2) / (2 * self.acceleration):
                # Deceleration
                self.current_speed -= self.acceleration
            else:
                # Acceleration
                self.current_speed += self.acceleration

            # Limit speed
            self.current_speed = max(0.0, min(self.current_speed, self.max_speed))

            # Update current angle
            self.current_angle += direction * self.current_speed

        # Convert to radians
        angle_rad = math.radians(self.current_angle)

        msg = Float64MultiArray()
        msg.data = [angle_rad]

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PanController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
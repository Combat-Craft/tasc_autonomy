import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math


class ScanChecker(Node):
    def __init__(self):
        super().__init__('scan_checker')

        # Subscribe to laserScan because its tuff
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/obstacle_status',
            10
        )

        self.threshold = 0.5  # its in meters
        self.get_logger().info('Scan checker started')

    def scan_callback(self, msg: LaserScan):
        # Remove inf / NaN bcz extraneous and stuff
        valid_ranges = [
            r for r in msg.ranges
            if not math.isinf(r) and not math.isnan(r)
        ]

        if not valid_ranges:
            return

        min_distance = min(valid_ranges)

        status = String()

        if min_distance < self.threshold:
            status.data = f"CLOSE obstacle at {min_distance:.2f} m"
        else:
            status.data = f"FAR – closest obstacle at {min_distance:.2f} m"

        self.publisher.publish(status)
        self.get_logger().info(status.data)


def main(args=None):
    rclpy.init(args=args)
    node = ScanChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

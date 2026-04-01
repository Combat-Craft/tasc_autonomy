import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
from collections import deque


class ScanChecker(Node):
    def __init__(self):
        super().__init__('scan_checker')

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

        # Detection threshold
        self.threshold = 0.35  # meters

        self.min_valid_range = 0.1   
        self.max_valid_range = 8.0   

        # Temporal smoothing
        self.history = deque(maxlen=5)

        # Anti-flicker logic
        self.close_count = 0
        self.required_hits = 3

        self.get_logger().info('Scan checker started')


    def scan_callback(self, msg: LaserScan):
        # Step 1: clean raw scan
        valid_ranges = [
            r for r in msg.ranges
            if self.min_valid_range <= r <= self.max_valid_range
            and not math.isnan(r)
            and not math.isinf(r)
        ]

        if not valid_ranges:
            return

        # Step 2: use percentile instead of raw min
        valid_ranges.sort()
        percentile_index = int(0.05 * len(valid_ranges))  # 5th percentile
        closest_distance = valid_ranges[percentile_index]

        # Step 3: temporal smoothing
        self.history.append(closest_distance)
        smoothed_distance = sum(self.history) / len(self.history)

        # Step 4: persistence check
        status = String()

        if smoothed_distance < self.threshold:
            self.close_count += 1
        else:
            self.close_count = 0

        if self.close_count >= self.required_hits:
            status.data = f"CLOSE obstacle at {smoothed_distance:.2f} m"
        else:
            status.data = f"FAR – closest obstacle at {smoothed_distance:.2f} m"

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

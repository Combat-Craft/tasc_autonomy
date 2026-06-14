import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class FakeGPSPublisher(Node):

    def __init__(self):
        super().__init__('fake_gps')

        self.declare_parameter('loop', True)

        self.loop_ = self.get_parameter('loop').get_parameter_value().bool_value

        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.timer_ = self.create_timer(1.0, self.publish_gps)

        self.index_ = 0
        self.path_ = [
            (43.656050, -79.380280),
            (43.656080, -79.380200),
            (43.656120, -79.380100),
            (43.656200, -79.380050),
            (43.656260, -79.379980),
            (43.656300, -79.379900),
            (43.656350, -79.379820),
            (43.656400, -79.379750),
        ]

        self.get_logger().info(
            f'Fake GPS started — {len(self.path_)} waypoints, loop={self.loop_}')

    def publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude  = self.path_[self.index_][0]
        msg.longitude = self.path_[self.index_][1]
        msg.altitude  = 100.0
        msg.status.status  = 0
        msg.status.service = 1
        msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'[fake] {msg.latitude:.6f}, {msg.longitude:.6f}',
            throttle_duration_sec=2.0)

        if self.loop_:
            self.index_ = (self.index_ + 1) % len(self.path_)
        elif self.index_ < len(self.path_) - 1:
            self.index_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakeGPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
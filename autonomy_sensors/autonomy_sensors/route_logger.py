import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class RouteLogger(Node):

    def __init__(self):
        super().__init__('route_logger')

        self.subscription_ = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        self.publisher_ = self.create_publisher(Path, '/gps/path', 10)

        self.path_msg_ = Path()
        self.path_msg_.header.frame_id = 'map'

        self.origin_set_ = False
        self.origin_lat_ = 0.0
        self.origin_lon_ = 0.0

    def gps_callback(self, msg: NavSatFix):
        if not self.origin_set_:
            self.origin_lat_ = msg.latitude
            self.origin_lon_ = msg.longitude
            self.origin_set_ = True

        x, y = self.latlon_to_xy(msg.latitude, msg.longitude)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        self.path_msg_.header.stamp = self.get_clock().now().to_msg()
        self.path_msg_.poses.append(pose)

        self.publisher_.publish(self.path_msg_)

    def latlon_to_xy(self, lat: float, lon: float):
        R = 6378137.0

        dlat = (lat - self.origin_lat_) * math.pi / 180.0
        dlon = (lon - self.origin_lon_) * math.pi / 180.0
        mean_lat = ((lat + self.origin_lat_) / 2.0) * math.pi / 180.0

        y = R * dlat
        x = R * math.cos(mean_lat) * dlon
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = RouteLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
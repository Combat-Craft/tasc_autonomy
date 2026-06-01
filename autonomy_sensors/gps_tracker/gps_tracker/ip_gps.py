import math
import urllib.request
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class IPGPSPublisher(Node):
    """
    Publishes a NavSatFix derived from your machine's IP geolocation.
    Accuracy is city/neighbourhood level (~1-5 km). Good for pipeline testing.
    Generates a small simulated walk around the detected location.
    """

    def __init__(self):
        super().__init__('ip_gps')

        self.declare_parameter('walk_step_m', 5.0)   # metres per step
        self.declare_parameter('loop', True)

        step_m = self.get_parameter('walk_step_m').get_parameter_value().double_value
        self.loop_ = self.get_parameter('loop').get_parameter_value().bool_value

        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

        lat, lon = self._fetch_location()
        self.get_logger().info(f'IP location: {lat:.5f}, {lon:.5f}')

        # Build a small rectangular walk around the detected point
        # step_m metres converted to degrees (approx)
        d_lat = step_m / 111111.0
        d_lon = step_m / (111111.0 * math.cos(math.radians(lat)))

        self.path_ = [
            (lat,            lon),
            (lat + d_lat,    lon),
            (lat + 2*d_lat,  lon),
            (lat + 2*d_lat,  lon + d_lon),
            (lat + 2*d_lat,  lon + 2*d_lon),
            (lat + d_lat,    lon + 2*d_lon),
            (lat,            lon + 2*d_lon),
            (lat,            lon + d_lon),
        ]

        self.index_ = 0
        self.timer_ = self.create_timer(1.0, self.publish_gps)

        self.get_logger().info(
            f'IP GPS started — {len(self.path_)} waypoints, '
            f'step={step_m:.1f}m, loop={self.loop_}')

    def _fetch_location(self):
        apis = [
            ('https://ipapi.co/json/',        lambda d: (d['latitude'],  d['longitude'])),
            ('https://ip-api.com/json/',       lambda d: (d['lat'],       d['lon'])),
            ('https://ipwho.is/',              lambda d: (d['latitude'],  d['longitude'])),
        ]
        for url, extract in apis:
            try:
                with urllib.request.urlopen(url, timeout=5) as r:
                    data = json.loads(r.read())
                lat, lon = extract(data)
                if lat and lon:
                    return float(lat), float(lon)
            except Exception as e:
                self.get_logger().warn(f'IP lookup failed ({url}): {e}')

        self.get_logger().warn('All IP lookups failed — using Toronto fallback')
        return 43.6532, -79.3832

    def publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude  = self.path_[self.index_][0]
        msg.longitude = self.path_[self.index_][1]
        msg.altitude  = 100.0
        msg.status.status  = 0
        msg.status.service = 1
        # Covariance reflects IP accuracy (~2500 m^2 horizontal)
        msg.position_covariance = [2500.0, 0.0, 0.0, 0.0, 2500.0, 0.0, 0.0, 0.0, 9999.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'[ip] {msg.latitude:.6f}, {msg.longitude:.6f}',
            throttle_duration_sec=2.0)

        if self.loop_:
            self.index_ = (self.index_ + 1) % len(self.path_)
        elif self.index_ < len(self.path_) - 1:
            self.index_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = IPGPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
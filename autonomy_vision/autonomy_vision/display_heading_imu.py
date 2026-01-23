import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu


def yaw_from_quaternion(q) -> float:
    """Compute yaw (heading) in radians from geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def heading_to_compass(degrees: float) -> str:
    """Convert heading in degrees (clockwise from north) to a compass string."""
    if degrees is None:
        return '--'

    angle = round(degrees) % 360
    if angle == 0:
        return 'N'
    if angle == 90:
        return 'E'
    if angle == 180:
        return 'S'
    if angle == 270:
        return 'W'
    if angle < 90:
        return f'N{angle}E'
    if angle < 180:
        return f'S{180 - angle}E'
    if angle < 270:
        return f'S{angle - 180}W'
    return f'N{360 - angle}W'


class HeadingOverlayNode(Node):
    def __init__(self):
        super().__init__('heading_overlay')
        self.heading_deg = None

        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 10)

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = yaw_from_quaternion(q)
        self.heading_deg = (math.degrees(yaw) + 360.0) % 360.0

    def image_callback(self, msg: Image) -> None:
        frame = self.image_msg_to_bgr(msg)
        if frame is None:
            return

        heading_text = heading_to_compass(self.heading_deg)
        label = f'Heading: {heading_text}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, thickness)
        pad = 10
        cv2.rectangle(
            frame,
            (5, 5),
            (5 + text_width + pad * 2, 5 + text_height + baseline + pad * 2),
            (0, 0, 0),
            -1,
        )
        cv2.putText(
            frame,
            label,
            (5 + pad, 5 + pad + text_height),
            font,
            font_scale,
            (0, 255, 0),
            thickness,
            cv2.LINE_AA,
        )

        try:
            cv2.imshow('Heading Overlay', frame)
            cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().warning(f'Failed to display frame: {exc}')

    def image_msg_to_bgr(self, msg: Image):
        """Convert sensor_msgs/Image to a BGR np.array without cv_bridge."""
        encoding = msg.encoding.lower()
        channels_map = {
            'bgr8': 3,
            'rgb8': 3,
            'mono8': 1,
        }
        dtype_map = {
            'bgr8': np.uint8,
            'rgb8': np.uint8,
            'mono8': np.uint8,
        }

        channels = channels_map.get(encoding)
        dtype = dtype_map.get(encoding)

        if channels is None or dtype is None:
            self.get_logger().warning(f'Unsupported image encoding: {msg.encoding}')
            return None

        expected_step = msg.width * channels * np.dtype(dtype).itemsize
        if msg.step != expected_step:
            self.get_logger().debug(
                f'Unexpected image step {msg.step} for encoding {msg.encoding}; expected {expected_step}'
            )

        try:
            frame = np.frombuffer(msg.data, dtype=dtype)
            frame = frame.reshape((msg.height, msg.width, channels))
        except Exception as exc:
            self.get_logger().warning(f'Failed to reshape image buffer: {exc}')
            return None

        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if encoding == 'mono8':
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        return frame


def main(args=None):
    rclpy.init(args=args)
    node = HeadingOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

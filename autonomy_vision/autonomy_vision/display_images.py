import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display')
        self.bridge = CvBridge()
        self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.image_callback,
            10,
        )

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Camera', frame)
            cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().warning(f'Failed to display frame: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()
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

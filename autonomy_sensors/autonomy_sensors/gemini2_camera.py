import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat

class Gemini2CameraNode(Node):
    def __init__(self):
        super().__init__('gemini2_camera')

        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)

        self.pipeline = Pipeline()
        config = Config()
        config.enable_video_stream(OBSensorType.COLOR_SENSOR, width=1280, height=720, fps=30, format=OBFormat.RGB)
        config.enable_video_stream(OBSensorType.DEPTH_SENSOR, width=640, height=400, fps=30)
        self.pipeline.start(config)

        self.timer = self.create_timer(0.033, self.publish_frames)  # ~30 Hz
        self.get_logger().info('Gemini 2 camera node started')

    def publish_frames(self):
        frames = self.pipeline.wait_for_frames(100)
        if not frames:
            return

        now = self.get_clock().now().to_msg()

        # --- Colour ---
        color_frame = frames.get_color_frame()
        if color_frame:
            msg = Image()
            msg.header.stamp = now
            msg.header.frame_id = 'camera_color_optical_frame'
            msg.height = color_frame.get_height()
            msg.width = color_frame.get_width()
            msg.encoding = 'rgb8'
            msg.step = msg.width * 3
            msg.data = bytes(color_frame.get_data())
            self.color_pub.publish(msg)

        # --- Depth ---
        depth_frame = frames.get_depth_frame()
        if depth_frame:
            msg = Image()
            msg.header.stamp = now
            msg.header.frame_id = 'camera_depth_optical_frame'
            msg.height = depth_frame.get_height()
            msg.width = depth_frame.get_width()
            msg.encoding = '16UC1'  # 16-bit unsigned, 1 channel — value = mm
            msg.step = msg.width * 2
            msg.data = bytes(depth_frame.get_data())
            self.depth_pub.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = Gemini2CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

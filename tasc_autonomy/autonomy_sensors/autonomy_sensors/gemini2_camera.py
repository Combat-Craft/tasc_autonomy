import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
import numpy as np
from pyorbbecsdk import (
    Pipeline, Config, OBSensorType, OBFormat, OBAlignMode,
    AlignFilter, PointCloudFilter, OBStreamType
)

class Gemini2CameraNode(Node):
    def __init__(self):
        super().__init__('gemini2_camera')

        # Publishers
        self.color_pub = self.create_publisher(Image,       'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image,       'camera/depth/image_raw', 10)
        self.imu_pub   = self.create_publisher(Imu,         'camera/imu',             10)
        self.pc_pub    = self.create_publisher(PointCloud2, 'camera/depth/points',    10)

        # Main pipeline — colour + depth
        self.pipeline = Pipeline()
        config = Config()

        depth_profile = self.pipeline.get_stream_profile_list(
            OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
        color_profile = self.pipeline.get_stream_profile_list(
            OBSensorType.COLOR_SENSOR).get_default_video_stream_profile()

        config.enable_stream(depth_profile)
        config.enable_stream(color_profile)
        self.pipeline.enable_frame_sync()
        self.pipeline.start(config)

        # Filters for point cloud (same pattern as working SDK example)
        self.align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        self.pc_filter = PointCloudFilter()
        self.pc_filter.set_create_point_format(OBFormat.RGB_POINT)

        # IMU — separate pipeline
        self.imu_pipeline = Pipeline()
        imu_config = Config()
        imu_config.enable_accel_stream()
        imu_config.enable_gyro_stream()
        self.imu_pipeline.start(imu_config)

        self.timer     = self.create_timer(0.033, self.publish_frames)
        self.imu_timer = self.create_timer(0.01,  self.publish_imu)

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
            msg.header.stamp    = now
            msg.header.frame_id = 'camera_color_optical_frame'
            msg.height   = color_frame.get_height()
            msg.width    = color_frame.get_width()
            msg.encoding = 'rgb8'
            msg.step     = msg.width * 3
            msg.data     = bytes(color_frame.get_data())
            self.color_pub.publish(msg)

        # --- Depth ---
        depth_frame = frames.get_depth_frame()
        if depth_frame:
            scale = depth_frame.get_depth_scale()
            raw   = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).copy()
            # Apply scale so values are in mm as uint16
            scaled = (raw.astype(np.float32) * scale).astype(np.uint16)

            msg = Image()
            msg.header.stamp    = now
            msg.header.frame_id = 'camera_depth_optical_frame'
            msg.height   = depth_frame.get_height()
            msg.width    = depth_frame.get_width()
            msg.encoding = '16UC1'
            msg.step     = msg.width * 2
            msg.data     = scaled.tobytes()
            self.depth_pub.publish(msg)

        # --- XYZRGB Point Cloud ---
        if color_frame and depth_frame:
            try:
                aligned     = self.align_filter.process(frames)
                pc_frame    = self.pc_filter.process(aligned)
                if pc_frame:
                    data = np.frombuffer(pc_frame.get_data(), dtype=np.float32).copy()
                    # RGB_POINT layout: X Y Z R G B per point (6 x float32)
                    if data.size % 6 != 0:
                        return
                    points = data.reshape(-1, 6)

                    pc_msg = PointCloud2()
                    pc_msg.header.stamp    = now
                    pc_msg.header.frame_id = 'camera_depth_optical_frame'
                    pc_msg.height      = 1
                    pc_msg.width       = len(points)
                    pc_msg.is_dense    = False
                    pc_msg.is_bigendian = False
                    pc_msg.point_step  = 24  # 6 fields x 4 bytes
                    pc_msg.row_step    = pc_msg.point_step * pc_msg.width
                    pc_msg.fields      = [
                        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                        PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1),
                        PointField(name='g', offset=16, datatype=PointField.FLOAT32, count=1),
                        PointField(name='b', offset=20, datatype=PointField.FLOAT32, count=1),
                    ]
                    pc_msg.data = points.tobytes()
                    self.pc_pub.publish(pc_msg)
            except Exception as e:
                self.get_logger().warn(f'Point cloud error: {e}')

    def publish_imu(self):
        try:
            frames = self.imu_pipeline.wait_for_frames(100)
            if not frames:
                return
            accel_frame = frames.get_accel_frame()
            gyro_frame  = frames.get_gyro_frame()
            if not accel_frame or not gyro_frame:
                return

            a = accel_frame.get_accel_value()
            g = gyro_frame.get_gyro_value()

            msg = Imu()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_imu_optical_frame'
            msg.linear_acceleration.x = a.x
            msg.linear_acceleration.y = a.y
            msg.linear_acceleration.z = a.z
            msg.angular_velocity.x = g.x
            msg.angular_velocity.y = g.y
            msg.angular_velocity.z = g.z
            msg.orientation_covariance[0]         = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
            msg.angular_velocity_covariance[0]    = -1.0
            self.imu_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'IMU error: {e}')

    def destroy_node(self):
        self.pipeline.stop()
        self.imu_pipeline.stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = Gemini2CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

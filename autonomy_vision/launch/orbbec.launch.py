# New iteration of gemini2_edited2.launch.py, Removed CLI override capability for cleaner launchl.

# orbbec.launch.py
# Standalone Orbbec Gemini 2 launch for autonomy_vision
# Color-only mode: 1280x720 @ 30fps compressed (Foxglove-Encoding)
#
# To enable point cloud, uncomment:
#   - enable_depth, depth_* params
#   - enable_point_cloud, enable_colored_point_cloud
#   - depth_registration: must be 'true' for colored point cloud
#   - The image_transport republisher is not needed for point cloud

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_params = {
        'camera_name': 'camera',

        # --- Color: 1280x720 @ 30fps MJPG ---
        'enable_color': 'true',
        'color_width': '1280',
        'color_height': '720',
        'color_fps': '30',
        'color_format': 'MJPG', 
        # A few other color format options we could try, but Foxglove uses JPG, so we'll use MJPG
        
        'flip_color': 'false',
        'color_qos': 'default',
        'color_camera_info_qos': 'default',
        'enable_color_auto_exposure': 'true',
        'color_exposure': '-1',
        'color_gain': '-1',
        'enable_color_auto_white_balance': 'true',
        'color_white_balance': '-1',

        # --- Depth: DISABLED (enable for point cloud) ---
        'enable_depth': 'false',
        # 'depth_width': '640',
        # 'depth_height': '400',
        # 'depth_fps': '15',
        # 'depth_format': 'Y14',
        # 'flip_depth': 'false',
        # 'depth_qos': 'default',
        # 'depth_registration': 'true',   # required for colored point cloud

        # --- IR: DISABLED ---
        'enable_ir': 'false',
        # 'ir_width': '640',
        # 'ir_height': '400',
        # 'ir_fps': '15',
        # 'ir_format': 'Y8',

        # --- IMU: DISABLED ---
        'enable_accel': 'false',
        'enable_gyro': 'false',

        # --- Point cloud: DISABLED (enable with depth above) ---
        'enable_point_cloud': 'false',
        'enable_colored_point_cloud': 'false',
        # 'point_cloud_qos': 'default',
        # 'ordered_pc': 'false',

        # --- General ---
        'device_num': '1',
        'vendor_id': '0x2bc5',
        'connection_delay': '100',
        'publish_tf': 'true',
        'tf_publish_rate': '0.0',
        'log_level': 'none',
        'enable_publish_extrinsic': 'false',
        'enable_d2c_viewer': 'false',
        'enable_ldp': 'true',
        'sync_mode': 'standalone',
        'enable_frame_sync': 'true',
        'use_hardware_time': 'true',
        'enable_depth_scale': 'true',
        'align_mode': 'HW',
        'enable_heartbeat': 'false',
        'retry_on_usb3_detection_failure': 'false',
        'laser_energy_level': '-1',
    }

    orbbec_node = Node(
        package='orbbec_camera',
        executable='orbbec_camera_node',
        name='ob_camera_node',
        namespace='camera',
        parameters=[camera_params],
        output='screen',
    )

    # Republishes /camera/color/image_raw as compressed with correct JPEG
    # encoding metadata so Foxglove can decode it.
    # Topic: /camera/color/image_raw/compressed (sensor_msgs/CompressedImage)
    compressed_republisher = Node(
        package='image_transport',
        executable='republish',
        name='color_compressed_republisher',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in',  '/camera/color/image_raw'),
            ('out', '/camera/color/image_raw'),
        ],
        parameters=[{
            'compressed.format': 'jpeg',
            'compressed.jpeg_quality': 80,
        }],
        output='screen',
    )

    return LaunchDescription([
        orbbec_node,
        compressed_republisher,
    ])
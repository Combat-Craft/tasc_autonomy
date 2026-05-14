from setuptools import find_packages, setup
from glob import glob

package_name = 'autonomy_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toni',
    maintainer_email='tonithetutor@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'panorama_stitcher = autonomy_vision.panorama_stitcher:main',
            'panorama_stitcher_with_heading = autonomy_vision.panorama_stitcher_with_heading:main',
            
            'display_heading_imu = autonomy_vision.display_heading_imu:main',
            'display_heading = autonomy_vision.display_heading:main',
            'display_images = autonomy_vision.display_images:main',

            'pose_cam = autonomy_vision.gst_pose_cam:main',
            'arm_cam = autonomy_vision.gst_arm_cam:main',
            'multi_camera_streamer = autonomy_vision.multi_camera_streamer:main',
            
            'yolo_pc = autonomy_vision.yolo_pc:main',
            'yolo_depth_v1 = autonomy_vision.yolo_depth_v1:main',
            'yolo_depth_v2 = autonomy_vision.yolo_depth_v2:main',
            'webcam_detection2D = autonomy_vision.webcam_detection2D:main',
            'morse_code_detector = autonomy_vision.morse_code_detector:main',
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomy_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=[
        'setuptools',
        'pyserial',  # For serial port communication
        'pynmea2',   # For NMEA GPS parsing
    ],
    zip_safe=True,
    maintainer='toni',
    maintainer_email='tonithetutor@gmail.com',
    description='Autonomy sensors package - GPS, IMU, and odometry publishers',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_broadcaster = autonomy_sensors.gps_broadcaster:main',
            'gps_frame_fixer = autonomy_sensors.gps_frame_fixer:main',
            'gps_imu_broadcaster = autonomy_sensors.gps_imu_broadcaster:main',
            'path_publisher = autonomy_sensors.path_publisher:main',
            'diff_drive_odom = autonomy_sensors.diff_drive_odom:main',
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayush',
    maintainer_email='ayushsehijpal@gmail.com',
    description='GPS tracker package — fake / IP / real modes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_gps     = gps_tracker.fake_gps:main',
            'ip_gps       = gps_tracker.ip_gps:main',
            'route_logger = gps_tracker.route_logger:main',
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autonomy_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
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
            # Panorama
            'panorama_stitcher = autonomy_vision.panorama_stitcher:main',
            'panorama_stitcher2 = autonomy_vision.panorama_stitcher_gstreamer:main',
            # Motors
            'motor_controller = autonomy_vision.motor_controller:main',
            'motor_input = autonomy_vision.motor_input:main',

            # Morse
            'morse_detector = autonomy_vision.morse_code_detector:main',


            # Streaming
            'multi_camera_streamer = autonomy_vision.multi_camera_streamer:main',
            
        ],
    },
)

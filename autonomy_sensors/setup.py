from setuptools import find_packages, setup

package_name = 'autonomy_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'esp32_broadcaster = autonomy_sensors.gps_imu_broadcaster:main',
            #'gps_broadcaster = autonomy_sensors.gps_broadcaster:main',
            #'gps_frame_fixer = autonomy_sensors.gps_frame_fixer:main',
            #'path_publisher = autonomy_sensors.path_publisher:main',
        ],
    },
)

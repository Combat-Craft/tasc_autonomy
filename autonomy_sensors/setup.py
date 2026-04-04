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
    maintainer='ArtemisLee',
    maintainer_email='yr.lee@torontomu.com',
    description='Has all the sensor code, such as GPS, IMU, Orbbec, etc.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_imu_broadcaster = autonomy_sensors.gps_imu_broadcaster:main',
            'gps_node = autonomy_sensors.gps_node:main',
        ],
    },
)

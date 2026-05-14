# Instructions

Set up RPlidar:
1) create another workspace for rplidar
   ```bash
   mkdir -p ~/rplidar_ws/src
   cd ~/rplidar_ws/src
   ```
3) clone the rplidar_ros package for ros2 while in rplidar_ws/src
   ```bash
   git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
   ```
4) build rpidar_ros package
   ```bash
   cd ~/rplidar_ws/
   source /opt/ros/humble/setup.bash
   colcon build 
   ```
5) source the package environment setup
    ```bash
    source install/setup.bash
    ```

6) plug the lidar in, check its there, and ensure we have permission
   ```bash
   lsusb
   ```
   with the ESP32, orbbec, webcame, and lidar, you need to correctly map each USB!!
   ```bash
   ls /dev/ttyUSB*
   ```
   change the number to whatever the lidar is on
   ```bash
   sudo chmod 777 /dev/ttyUSB0
   ```
7) you can now go to tasct_autonomy, do regular set up and call.
   ```bash
   ros2 launch autonomy_sensors sensors.launch.py
   ```
   ***NOTE: You MUST change the /dev/ttyUSB0 of each device correctly! ESP32 (i.e. gps and imu node) share one, lidar has another, each camera has another, etc***

Set up foxglove:
1) in terminal ```sudo foxglove-studio```
2) Click the "Open Connection" Button, should be top right of 4 boxes/buttons
3) Click/Use "Foxglove Websocket" We are using the default of socket 8765
4) Run ``` ros2 launch autonomy_sensors sensors.launch.py ```

# Foxglove
Download from [here](https://foxglove.dev/download) not their doc's or sudo snap.



# Calling Launch files from another Launch File

From [https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html#parent-launch-file)


Basically, use this notation, and take note to include the required imports

```
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('launch_tutorial'),
                'launch',
                'example_substitutions.launch.py'
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': colors['background_r'],
            }.items()
        )
    ])
```

Note that ```'launch'``` refers to the folder/directory of that package. 
Essentially, ``` launch.substitutions.PathJoinSubstitution ``` concats the list of strings into a path, seperated by the slash.
See [doc](https://docs.ros.org/en/jazzy/p/launch/launch.substitutions.path_join_substitution.html)



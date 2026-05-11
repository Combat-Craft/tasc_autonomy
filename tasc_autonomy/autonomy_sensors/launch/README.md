# Instructions

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



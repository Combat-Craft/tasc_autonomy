# tasc_autonomy
Sub-packages: `autonomy_sensors`, `autonomy_vision`

3 terminals:

sudo snap install foxglove-studio
DISPLAY=:0 foxglove-studio --ozone-platform=x11
open connection to localhost:8765 on foxglove-studio

cd ~/AN_combined_ws 
colcon build --packages-select autonomy_sensors
source install/setup.bash
ros2 run autonomy_sensors gemini2_camera

sudo apt install ros-$ROS_DISTRO-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

foxglove-studio --no-sandbox --ozone-platform=x11 --disable-gpu --swiftshader

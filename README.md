# tasc_autonomy
Sub-packages: `autonomy_sensors`, `autonomy_vision`


pyorbbecsdk::

sudo apt update

sudo apt install -y python3-pip python3-dev cmake libusb-1.0-0-dev

pip3 install pyorbbecsdk2

python3 -m pip show pyorbbecsdk2


# FOR GETTING ORBBEC ROS2 SDK GEMINI LAUNCH FILE WORKING: 
possible sources of error: 

Firmware VERSION 

Rviz2 visualization vs. Foxglove 

https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2-Firmware


https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main

FOLLOW README IN THIS REPO ^^

Replace gemini330 series launch command with:

ros2 launch orbbec_camera gemini2.launch.py uvc_backend:=v4l2 color_format:=RGB enable_point_cloud:=false enable_colored_point_cloud:=false

# uvc_backend param might be necessary, if so LinuxUVCBackend XML config file will need to be edited

find ~/ros2_ws -name "OrbbecSDKConfig_v1.0.xml" 2>/dev/null

micro <path_you_found>/OrbbecSDKConfig_v1.0.xml


Look for LinuxUVCBackend, change libuvc to v4l2

then:

cd ~/<your_workspace>

colcon build orbbec_camera

source install/setup.bash

# killing instances 

sudo pkill -9 -f "orbbec_camera"

sudo pkill -9 -f "component_container"

# Test for framerate detection and USB connection type

ros2 topic echo /camera/device_status | grep -E "connection_type|color_frame_rate_cur"

# Fast DDS Tuning
Follow the instructions in this link:
https://github.com/orbbec/OrbbecSDK_ROS2/blob/main/docs/fastdds_tuning.md


# Available color profiles

All available color profiles (Orbbec ROS2 SDK):

[component_container-1] [INFO] [1778778982.239264256] [camera.camera]: Available profiles:

[component_container-1] [INFO] [1778778982.239426945] [camera.camera]: color profile: 1920x1080 30fps RGB888

[component_container-1] [INFO] [1778778982.239439199] [camera.camera]: color profile: 1920x1080 30fps MJPG

[component_container-1] [INFO] [1778778982.239446111] [camera.camera]: color profile: 1920x1080 30fps YUYV

# Explicit force formatting

ros2 launch orbbec_camera gemini2.launch.py \ color_format:=RGB888 \ color_width:=1920 \ color_height:=1080 \ color_fps:=30 \  depth_width = 1920 \ depth_height = 1080 \ depth_fps = 30 \ enable_point_cloud:=false \ enable_colored_point_cloud:=false \





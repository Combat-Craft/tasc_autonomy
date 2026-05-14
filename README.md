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

uvc_backend param might be necessary, if so LinuxUVCBackend XML config file will need to be edited

# Fast DDS Tuning
Follow the instructions in this link:
https://github.com/orbbec/OrbbecSDK_ROS2/blob/main/docs/fastdds_tuning.md






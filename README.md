# tasc_autonomy
Sub-packages: `autonomy_sensors`, `autonomy_vision`


# FOR GETTING ORBBEC ROS2 SDK GEMINI LAUNCH FILE WORKING: 

https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2-Firmware


https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main

FOLLOW README IN THIS REPO ^^

  Replace gemini330 series launch command with:

``` bash
ros2 launch orbbec_camera gemini2.launch.py
```


# killing instances 

``` sudo pkill -9 -f "orbbec_camera" ```

``` sudo pkill -9 -f "component_container" ```

# Test for framerate detection and USB connection type

ros2 topic echo /camera/device_status | grep -E "connection_type|color_frame_rate_cur"

# Fast DDS Tuning
Follow the instructions in this link:
https://github.com/orbbec/OrbbecSDK_ROS2/blob/main/docs/fastdds_tuning.md

Run :

``` sysctl net.core.rmem_max net.core.wmem_max net.core.rmem_default net.core.wmem_default net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh ```

to check if parameters are correct.

# Available color profiles

All available color profiles (Orbbec ROS2 SDK):

[component_container-1] [INFO] [1778778982.239264256] [camera.camera]: Available profiles:

[component_container-1] [INFO] [1778778982.239426945] [camera.camera]: color profile: 1920x1080 30fps RGB888

[component_container-1] [INFO] [1778778982.239439199] [camera.camera]: color profile: 1920x1080 30fps MJPG

[component_container-1] [INFO] [1778778982.239446111] [camera.camera]: color profile: 1920x1080 30fps YUYV

.

.

.

# Explicit force formatting 

``` ros2 launch orbbec_camera gemini2.launch.py \ color_format:=RGB888 \ color_width:=1280 \ color_height:=720 \ color_fps:=40 \  depth_width: = 1920 \ depth_height: = 1080 \ depth_fps := 40 \ enable_point_cloud:=false \ enable_colored_point_cloud:=false  ```

Set to 720p for higher framerate.



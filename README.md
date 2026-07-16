# TASC Autonomy

Status date: 2026-06-03

This repository is a ROS 2 Humble workspace for the TASC autonomy stack. It currently includes:
- `autonomy_bringup` (Python package): Its purpose is to call all other package's launch files and Foxglove
- `autonomy_sensors` (Python package): GPS, IMU, their transform broadcaster nodes, and RPLidar is externally called
- `autonomy_vision` (Python package): YOLO and camera streaming nodes, and Orbbec Gemini2 is externally called

[Current Progress Snapshot](#Current-Progress-Snapshot)

[Workspace Layout](#Workspace-Layout)

[Installation](#Installation)

[Running Camera Streams](#Running-Camera-Streams)

[Running ROS2 Files](#Running-ROS2-Files)

[Troubleshooting](#Troubleshooting)


# Status and Build

## Current Progress Snapshot

Implemented and present in source:
- Sensor nodes: `gps_node`, `imu_node`, `gps_imu_broadcaster`, RPLidar's nodes are launched externally from `sensor.launch.py`, and for the router logger `route_logger`, `ip_gps` and `fake_gps`
- Vision nodes: `multi_camera_streamer`, and can be called from `vision.launch.py`

Known in-progress / issues:
- `multi_camera_streamer` needs to have finalized gstreamer pipelines as more cameras come in
- USB serial path for GPS/IMU and RPLiDAR may need to be changed from `/dev/ttyUSB0` depending on connected device, as they appear identicle due to both exportig serial data.
- cameras....

## Workspace Layout

**YOU MUST ADHERE TO THIS STRUCTURE. PLEASE DO NOT CREATE EXTRA PACKAGES OR SUB FOLDERS WITHOUT CONSULTING THE LEADS**
```
~/autonav_ws # this if your main workspace folder, name it as you please in your PC
  ├── build
  ├── install
  ├── log
  └── src
      └── tasc_autonomy
          ├── autonomy_bringup
          │   ├── autonomy_bringup
          │   ├── launch
          │   ├── resource
          │   └── test
          ├── autonomy_navigation
          │   ├── config
          │   ├── launch
          │   ├── resource
          │   └── test
          ├── autonomy_sensors
          │   ├── autonomy_sensors
          │   ├── firmware
          │   │   ├── imu_gps_serial
          │   │   ├── imu_gps_serial_ahrs
          │   │   └── imu_gps_serial_calibrated
          │   ├── launch
          │   ├── resource
          │   └── test
          ├── autonomy_vision
          │   ├── autonomy_vision
          │   ├── firmware
          │   │   └── panorama_motor
          │   ├── launch
          │   ├── Log
          │   ├── resource
          │   └── test
          ├── camera_streams
```

# Installation 

## Download and link repo

1) Create your workspace folders and cd to the src folder
```bash
mkdir -p ~/<your_folder_name>/src
cd ~/<your_folder_name>/src
```
2) Git clone this main branch
```bash
git clone https://github.com/Combat-Craft/tasc_autonomy.git
```
3) Use Git status to ensure you're on main
```bash
git status
```

## Build
Go to workspace root i.e. ```cd ~/<your_folder_name>```
**THIS IS IMPORTANT** It ensures the build, instal, and log folders and respective files are all within ~/<your_folder_name> and NOT in src or any other folder!

1) Set up humble ROS2
```bash
source /opt/ros/humble/setup.bash

```
2) Get all dependicies installeded, which currently includes RPLidar
```bash
rosdep install --from-paths src -y --ignore-src -r
```

3) Build tasc_autonomy

symlink should allow changes to go through without having to re-colcon build
```bash
colcon build --symlink-install 
```
To rebuild one specific package
```bash
colcon build --packages-select <package> 
```
To build all except a certain package
```bash
colcon build --packages-ignore <name-of-package>
```

4) Source the packages
```bash
source install/setup.bash
```

Notes:
- Use `source install/setup.bash` (not `source install/bash`).
- If you also use a virtual environment, activate it before build/run and ensure ROS Python packages are available there.

# Running Camera Streams

Please double check the device paths for the webcam(s) and Orbbec; should not have changed but just in case.
```bash
v4l2-ctl --list-devices
```

You can view my notes/test logs [here](https://docs.google.com/document/d/1xLJX_WVMZnSbE76I1YRzUpV9hlwl_LD-Z_N9sYjofT8/edit?usp=sharing)
You can view more details on cameras here (to be made later)

## ROS2 Based Multi-camera-streamer

Currently uses gstreamer, with software endocing xh264, and appsink. Has a usb webcam, default Orbbec color, and a test webcam from lead Mina.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomy_vision multi_camera_streamer
```

or

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch autonomy_vision sensors.launch.py
```


## Cameras

Please read https://github.com/Combat-Craft/tasc_autonomy/blob/main/docs/camera_info.md


# Running ROS2 Files

## 1. Launch files

1) Launches foxglove bridge (i.e. websocket) and all sensor nodes by externally calling autonomy_sensors' sensors.launch.py
```bash
ros2 launch autonomy_bringup foxglove_gui.launch.py
```

2) Launch all sensors nodes, which is IMU and GPS - RPLidar is currently exclude as it is yet to be installed on the rover.
```bash
ros2 launch autonomy_sensors sensors.launch.py
```

3) GPS tracker
```bash
ros2 launch gps_tracker gps_tracker.launch.py
```

4) Launches all vision nodes
```bash
ros2 launch autonomy_vision sensors.launch.py
```


## 2. Nodes (direct)

Main GPS and IMU Node
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomy_sensors gps_imu_broadcaster
```

Back up IMU node
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomy_sensors gps_node
```

Back up GPS node
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomy_sensors imu_node
```

GPS Router Logger Node
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gps_tracker route_logger
```

Fake GPS Path for ^ Node
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gps_tracker fake_gps
```

Object Detection Node **In progress, currently 1-5FPS**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomy_vision webcam_detection2D
```

Gstreamer based gstreamer
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomy_vision multi_camera_streamer
```

Other available executables:

```bash
ros2 run autonomy_vision yolo_pc
ros2 run autonomy_vision yolo_depth_v1
ros2 run autonomy_vision morse_code_detector
```

# Troubleshooting


## udev

SUBSYSTEM=="tty", SUBSYSTEMS=="usb", DRIVERS=="usb",ATTRS{idProduct}=="0002",ATTRS{idVendor}=="1d6b", SYMLINK+="imugps_esp32_usb"

sadly, LIDAR and ESP32 both use UART serial outputting, which appears to look the same to linux regard;less. we cannot apply static names as they do not give any unique attributes.

## Serial errors on GPS/IMU

If you see serial open failures:
- Verify device path: `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.
- Verify permissions: user should be in `dialout`
- Ensure `pyserial` is installed and the wrong `serial` package is not shadowing it:

```bash
pip uninstall -y serial
pip install pyserial
```

## H.264 encoder missing

If `x264enc` is not found, install:

```bash
sudo apt install gstreamer1.0-plugins-ugly
```

The node can still run if `avenc_h264_omx` is available.


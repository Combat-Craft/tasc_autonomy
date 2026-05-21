# TASC Autonomy

Status date: 2026-04-04

This repository is a ROS 2 Humble workspace for the TASC autonomy stack. It currently includes:
- `autonomy_bringup` (Python package): Its purpose is to call all other package's launch files and Foxglove
- `autonomy_sensors` (Python package): GPS, IMU, their transform broadcaster nodes, and RPLidar is externally called
- `autonomy_vision` (Python package): YOLO and camera streaming nodes, and Orbbec Gemini2 is externally called
- `gps_tracker` (C++ package): GPS path tracking code

## Current Progress Snapshot

Implemented and present in source:
- Sensor nodes: `gps_node`, `imu_node`, `gps_imu_broadcaster`, RPLidar's nodes are launched externally from `sensor.launch.py`
- Vision nodes: `yolo_pc`, `yolo_depth_v1`, `webcam_detection2D`, `morse_code_detector`, `multi_camera_streamer`, Gemini2's nodes are launched externally from `vision.launch.py`
- Multi-camera H.264 streamer with encoder fallback:
  - prefers `x264enc`
  - falls back to `avenc_h264_omx` when `x264enc` is unavailable

Known in-progress / issues:
- `multi_camera_streamer` is publish-only and needs formatting changes.
  - need to convert to H.265 for for efficient streaming
- all camera streams need to be compressed / lowered in quality to improve FPS
- USB serial path for GPS/IMU may need to be changed from `/dev/ttyUSB0` depending on connected device.
- `gps_tracker` needs to be combined into `autonomy_sensors`. It will either be converted to python code or `autonomy_sensors` will be remaded into a combined C++/python package

## Workspace Layout
```
~/<your_workspace_folder>
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
          ├── gps_tracker
          │   ├── launch
          │   └── src
          ├── install
          └── test
```

## Build

From workspace root (`~/<your_workspace_folder>):
1) Set up humble ROS2
```bash
source /opt/ros/humble/setup.bash

```
2) Get all dependicies insat;;ed, including 3rd party RPLidar and Orbbec
```bash
rosdep install --from-paths src -y --ignore-src -r
```

3) Build tasc_autonomy
```bash
colcon build
```

4) Source the packages
```bash
source install/setup.bash
```

Notes:
- Use `source install/setup.bash` (not `source install/bash`).
- If you also use a virtual environment, activate it before build/run and ensure ROS Python packages are available there.

## Run

### 1. Launch files

1) Launches everything. Currently excludes autonomy_vision as it is incomplete
```bash
ros2 launch autonomy_bringup foxglove_gui.launch.py
```

2) Launch all sensors nodes 
```bash
ros2 launch autonomy_sensors sensors.launch.py
```

3) **Does not work** Need to fix all vision nodes first
```bash
ros2 launch autonomy_vision vision.launch.py
```


### 2. Sensor nodes (direct)


```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomy_sensors gps_node
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomy_sensors imu_node
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomy_sensors gps_imu_broadcaster
```

### 3. Vision nodes

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomy_vision multi_camera_streamer
```

Other available executables:

```bash
ros2 run autonomy_vision yolo_pc
ros2 run autonomy_vision yolo_depth_v1
ros2 run autonomy_vision webcam_detection2D
ros2 run autonomy_vision morse_code_detector
```

## Troubleshooting


### udev

SUBSYSTEM=="tty", SUBSYSTEMS=="usb", DRIVERS=="usb",ATTRS{idProduct}=="0002",ATTRS{idVendor}=="1d6b", SYMLINK+="imugps_esp32_usb"

sadly, LIDAR and ESP32 both use UART serial outputting, which appears to look the same to linux regard;less. we cannot apply static names as they do not give any unique attributes.

### Serial errors on GPS/IMU

If you see serial open failures:
- Verify device path: `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.
- Verify permissions: user should be in `dialout`
- Ensure `pyserial` is installed and the wrong `serial` package is not shadowing it:

```bash
pip uninstall -y serial
pip install pyserial
```

### H.264 encoder missing

If `x264enc` is not found, install:

```bash
sudo apt install gstreamer1.0-plugins-ugly
```

The node can still run if `avenc_h264_omx` is available.


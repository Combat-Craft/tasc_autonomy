# TASC Autonomy

Status date: 2026-04-04

This repository is a ROS 2 Humble workspace for the TASC autonomy stack. It currently includes:
- `autonomy_sensors` (Python package): GPS, IMU, and transform broadcaster nodes
- `autonomy_vision` (Python package): YOLO and camera streaming nodes
- `giskard_description` (CMake package): robot description, xacro, RViz, Gazebo launch
- `giskard_gazebo` (CMake package): Gazebo model/world integration assets

## Current Progress Snapshot

Implemented and present in source:
- Sensor nodes: `gps_node`, `imu_node`, `gps_imu_broadcaster`
- Vision nodes: `yolo_pc`, `yolo_depth_v1`, `webcam_detection2D`, `morse_code_detector`, `multi_camera_streamer`
- Robot model and simulation launch: `giskard_description/launch/test.launch.py`
- Multi-camera H.264 streamer with encoder fallback:
  - prefers `x264enc`
  - falls back to `avenc_h264_omx` when `x264enc` is unavailable

Known in-progress / issues:
- `autonomy_sensors/launch/sensors.launch.py` currently has syntax errors and is not launch-ready.
- `multi_camera_streamer` is publish-only and needs formatting changes.
- USB serial path for GPS/IMU may need to be changed from `/dev/ttyUSB0` depending on connected device.

## Workspace Layout

Expected working directory for commands below:
- `.../tasc_autonav_combined/tasc_autonomy`

Main package directories:
- `autonomy_sensors/`
- `autonomy_vision/`
- `giskard_description/`
- `giskard_gazebo/`

## Build

From workspace root (`.../tasc_autonav_combined/tasc_autonomy`):

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Notes:
- Use `source install/setup.bash` (not `source install/bash`).
- If you also use a virtual environment, activate it before build/run and ensure ROS Python packages are available there.

## Run

### 1. Sensor nodes (direct)

Use direct node runs until `sensors.launch.py` is fixed:

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


# Morse Code Detector

## Subpackage Contents

*   **ROS2 Node (`morse_code_detector`)**: The production node that publishes data to the ROS2 ecosystem. Supports local V4L2 USB devices and IP cameras via RTSP.
*   **Standalone Script**: A standalone OpenCV testing script for rapid hardware verification and threshold debugging without ROS2 overhead.


## ROS2 Interface

### Usage
Launch the production detector with:
ros2 launch autonomy_vision morse_detector.launch.py


### Published Topics

* `/morse_code` (`std_msgs/String`) — Publishes the active accumulating symbol sequence (dots and dashes).
* `/morse_decoded` (`std_msgs/String`) — Publishes the fully accumulated and translated alphanumeric text.
* `/morse_debug_image` (`sensor_msgs/Image`) — Publishes a live BGR visual debug frame containing real-time edge triggers and text overlays.

### Subscribed Topics

* `/morse_reset` (`std_msgs/String`) — Receiving any message on this topic immediately wipes the accumulated decoded text buffer.

### Parameters

Configure these via your launch file, custom parameter files, or inline via `ros2 param set`:

* `camera_index` (int or string, default: `0`):
* Pass an integer (e.g., `0`) or device path (e.g., `/dev/video0`) for local USB cameras.
* Pass a URL (e.g., `rtspt://admin:@192.168.1.116:8554/profile0`) to capture from an IP camera. *Note: Using `rtspt://` explicitly forces TCP transport, eliminating dropped network frames.*


* `publish_hz` (float, default: `10.0`): The execution frequency of the capture callback loop. Recommend matching or exceeding the camera's hardware FPS.
* `log_cpu_stats` (bool, default: `true`): Toggles periodic logging of process CPU consumption, RSS memory usage, and callback latency.
* *Legacy parameters (`threshold_percentile`, `min_bright_fraction`)*: Retained for backward launch compatibility, but internally bypassed in favor of the automated adaptive peak-brightness tracker.


## System Verification & Hardware Deployment Workflow

Follow these validation steps to verify your camera feed before launching the full ROS2 system:

### 1. Environment Initialization

Source the workspace install overlay:

source ~/ros2_ws/install/setup.bash


### 2. Verify Package Integration

Ensure the package index can resolve the binary target:

ros2 pkg list | grep autonomy_vision


### 3. Scan Available Hardware Devices

Check local video nodes or verify network accessibility:

# For USB Camera setups
ls -l /dev/video* || true
lsusb
v4l2-ctl --list-devices 2>/dev/null || true

# For IP Camera setups (Verify network routing to the camera)
ping -c 3 192.168.1.116


### 4. Direct Feed Verification via Python

Run a quick, isolated test to ensure OpenCV can successfully initialize the target video backend


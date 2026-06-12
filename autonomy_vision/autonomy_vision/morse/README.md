# morse

This subpackage contains the morse code detector nodes and helpers.

Usage
-
- Launch with: `ros2 launch autonomy_vision morse_detector.launch.py`
- Topic `/morse_code` publishes the current symbol (dots/dashes).
- Topic `/morse_decoded` publishes accumulated decoded text.
- Topic `/morse_debug_image` publishes a debug `sensor_msgs/Image`.

Parameters (via launch or `ros2 param set`):
- `camera_index` (int) — camera device index
- `threshold_percentile` (float)
- `min_bright_fraction` (float)
- `publish_hz` (float)



# 1) source the workspace overlay (adjust path if your workspace is elsewhere)
source /home/tasc/Documents/ros2_ws/install/setup.bash

# 2) verify the package is visible
ros2 pkg list | grep autonomy_vision

# 3) check video devices and USB
ls -l /dev/video* || true
lsusb
dmesg | tail -n 50

# optional: v4l2-tools info (if installed)
v4l2-ctl --list-devices 2>/dev/null || true

# 4) quick OpenCV test
python3 - <<'PY'
import cv2
cap = cv2.VideoCapture(0)
print('Opened:', cap.isOpened())
cap.release()
PY

# 5) if device present, launch
ros2 launch autonomy_vision morse_detector.launch.py
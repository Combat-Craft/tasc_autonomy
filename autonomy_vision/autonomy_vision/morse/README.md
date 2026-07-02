# morse

This subpackage contains the morse code detector nodes and helpers.

## Morse Code Specification

The decoder follows the **ITU-R M.1677-1 International Morse Code standard** at **18 words per minute** using the PARIS reference:
- **Dit length**: 66.7 ms (1 unit)
- **Dash length**: 200 ms (3 units) 
- **Character space**: 200 ms (3 units)
- **Word space**: 467 ms (7 units)

This timing is detected by analyzing red LED flash durations on the security console. Flash pulses under ~133ms are interpreted as dots (·), longer pulses as dashes (−). Character and word boundaries are inferred from gaps between flashes.

## Usage
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
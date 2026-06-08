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

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

# Servo Motor & Panorama Capture

This section covers the camera servo control and panorama capture workflow. `motor_input.py` lets the user manually send servo commands or trigger a panorama sweep, `motor_controller.py` forwards the mapped servo commands to the Arduino, and `panorama_stitcher.py` rotates the camera through a set range of angles, captures images at each position, and creates a stitched panorama with GPS and heading metadata.

## Table of Contents

- [1) Build](#1-build)
- [2) Topics](#2-topics)
- [3) Launch File](#3-launch-file)
- [4) Nodes](#4-nodes)
- [5) Arduino Servo Firmware](#5-arduino-servo-firmware)
- [6) Manual Commands and Verification](#6-manual-commands-and-verification)
- [7) Hardware and Device Notes](#7-hardware-and-device-notes)
- [8) Dependencies Used by Current Code](#8-dependencies-used-by-current-code)

## 1) Build

Go to the main ROS 2 workspace folder. This folder should contain `src`, `build`, `install`, and `log`.

```bash
cd ~/<your_ros2_ws>
colcon build --packages-select autonomy_vision
source install/setup.bash
```

## 2) Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/motor_cmd` | `std_msgs/Float64` | Input/Output | Servo angle command in degrees, from `-135` to `+135` |
| `/panorama_trigger` | `std_msgs/Float64` | Input/Output | Trigger topic for starting the panorama sweep |
| `/image_raw` | `sensor_msgs/Image` | Input/Output | Raw camera image from the camera node |
| `/arm_cam/image/compressed` | `sensor_msgs/CompressedImage` | Input/Output | Compressed camera image used by `panorama_stitcher` |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Input | GPS position saved with panorama metadata |
| `/heading` | `std_msgs/Float32` | Input | Rover heading in degrees, used to calculate camera heading/cardinal overlay |

## 3) Launch File

### `motor.launch.py`

Launches the motor, camera, and panorama-related nodes needed for manual servo control and panorama capture.

Normal mode with camera:

```bash
ros2 launch autonomy_vision motor.launch.py
```

Test mode without requiring camera frames:

```bash
ros2 launch autonomy_vision motor.launch.py test_mode:=true
```

Select a specific camera device:

```bash
ros2 launch autonomy_vision motor.launch.py camera_device:=/dev/video0
```

Select a specific Arduino serial port:

```bash
ros2 launch autonomy_vision motor.launch.py serial_port:=/dev/ttyUSB0
```

Select a specific baud rate:

```bash
ros2 launch autonomy_vision motor.launch.py baud_rate:=115200
```

Launch arguments:

| Argument        | Description                                                   | Default        |
| --------------- | ------------------------------------------------------------- | -------------- |
| `test_mode`     | Runs `panorama_stitcher` without requiring real camera frames | `false`        |
| `camera_device` | Camera device path used by `v4l2_camera_node`                 | `/dev/video0`  |
| `serial_port`   | Serial port used by the Arduino motor controller              | `/dev/ttyUSB0` |
| `baud_rate`     | Arduino serial communication baud rate                        | `115200`       |

Started nodes:

| Node                | Description                                                            |
| ------------------- | ---------------------------------------------------------------------- |
| `motor_controller.py`  | Subscribes to `/motor_cmd` and forwards mapped servo values to Arduino |
| `motor_input.py`       | Opens a terminal for manual servo input and panorama triggering        |
| `panorama_stitcher.py` | Runs the panorama sweep logic                                          |
| `v4l2_camera_node`  | Publishes raw camera frames                                            |
| `image_republisher` | Converts raw images to compressed images for panorama capture          |

Motor control flow:

```txt
motor_input → /motor_cmd → motor_controller → Arduino Nano → MG90D servo
```

Panorama flow:

```txt
motor_input → /panorama_trigger → panorama_stitcher
panorama_stitcher → /motor_cmd → motor_controller → Arduino → servo sweep
camera → /image_raw → image_transport republish → /arm_cam/image/compressed → panorama_stitcher
```

## 4) Nodes

### `motor_input.py`

Allows the user to control the camera servo and trigger a panorama sweep from the terminal.

User inputs:

| Input                        | Action                          |
| ---------------------------- | ------------------------------- |
| Number from `-135` to `+135` | Publishes a servo angle command |
| `p`                          | Triggers the panorama sweep     |
| `e`                          | Exits the node                  |

Published topics:

| Topic               | Type               | Description                                 |
| ------------------- | ------------------ | ------------------------------------------- |
| `/motor_cmd`        | `std_msgs/Float64` | Manual servo angle command                  |
| `/panorama_trigger` | `std_msgs/Float64` | Trigger message for starting panorama sweep |

Related nodes:

| Node                   | Relationship                                                                |
| ---------------------- | --------------------------------------------------------------------------- |
| `motor_controller.py`  | Subscribes to `/motor_cmd` and sends the converted servo command to Arduino |
| `panorama_stitcher.py` | Subscribes to `/panorama_trigger` and begins the panorama sweep             |

Example prompt:

```txt
Enter angle (-135 to 135 | 'p' for panorama | 'e' to exit):
```

---

### `motor_controller.py`

Subscribes to camera servo angle commands and sends the mapped servo value to the Arduino over serial.

Subscribed topics:

| Topic        | Type               | Description                    |
| ------------ | ------------------ | ------------------------------ |
| `/motor_cmd` | `std_msgs/Float64` | Servo angle command in degrees |

Serial parameters:

| Parameter     | Description                                | Default        |
| ------------- | ------------------------------------------ | -------------- |
| `serial_port` | Serial port for Arduino motor controller   | `/dev/ttyUSB0` |
| `baud_rate`   | Baud rate for Arduino serial communication | `115200`       |

The serial parameters are passed through `motor.launch.py`, so the port and baud rate can be changed from the launch command.

Example:

```bash
ros2 launch autonomy_vision motor.launch.py serial_port:=/dev/ttyUSB1 baud_rate:=115200
```

Angle mapping:

The Arduino Servo library expects values from `0` to `180`, but the ROS 2 control range is `-135` to `+135` degrees. The node maps the ROS 2 angle into the Arduino servo range.

```txt
-135 degrees → 0
0 degrees    → 90
+135 degrees → 180
```

Related nodes:

| Node                   | Relationship                                                                     |
| ---------------------- | -------------------------------------------------------------------------------- |
| `motor_input.py`       | Publishes manual servo angle commands to `/motor_cmd`                            |
| `panorama_stitcher.py` | Publishes automated servo angle commands to `/motor_cmd` during a panorama sweep |

Troubleshooting serial device detection:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
lsusb
dmesg | tail -50
```

If the Arduino uses a CH340/CH341 USB serial chip and the device does not appear, reload the driver:

```bash
sudo modprobe ch341
```

Then unplug and reconnect the Arduino USB cable.

If `lsusb` does not show the device at all, it is likely a cable, USB port, or hardware issue.

If `lsusb` shows the CH340 device but `/dev/ttyUSB*` does not appear, it is likely a driver issue.

---

### `panorama_stitcher.py`

Controls an automated camera panorama sweep.

When triggered, this node moves the camera servo through a sequence of angles from `-135` to `+135` degrees, captures one compressed camera frame at each angle, stores GPS/heading metadata for each frame, and attempts to stitch the captured frames into a panorama image.

Main behavior:

1. Waits for a trigger message on `/panorama_trigger`.
2. Publishes servo angle commands to `/motor_cmd`.
3. Waits for the servo to move and settle.
4. Captures a compressed image frame from `/arm_cam/image/compressed`.
5. Saves the frame, servo angle, GPS coordinates, and heading.
6. Repeats until all sweep angles are complete.
7. Attempts to stitch the captured frames into `panorama.jpg`.
8. Resets and waits for the next trigger.

Published topics:

| Topic        | Type               | Description                                                 |
| ------------ | ------------------ | ----------------------------------------------------------- |
| `/motor_cmd` | `std_msgs/Float64` | Target camera servo angle in degrees, from `-135` to `+135` |

Subscribed topics:

| Topic                       | Type                          | Description                                       |
| --------------------------- | ----------------------------- | ------------------------------------------------- |
| `/panorama_trigger`         | `std_msgs/Float64`            | Trigger message used to start the panorama sweep  |
| `/arm_cam/image/compressed` | `sensor_msgs/CompressedImage` | Compressed camera image used for panorama capture |
| `/gps/fix`                  | `sensor_msgs/NavSatFix`       | GPS coordinates saved with panorama metadata      |
| `/heading`                  | `std_msgs/Float32`            | Rover heading in degrees clockwise from north     |

Parameters:

| Parameter   | Description                                                                            | Default |
| ----------- | -------------------------------------------------------------------------------------- | ------- |
| `test_mode` | Simulates image captures and tests motor movement without requiring real camera frames | `false` |

Test mode example:

```bash
ros2 launch autonomy_vision motor.launch.py test_mode:=true
```

Sweep configuration:

| Setting                   | Value              |
| ------------------------- | ------------------ |
| Sweep range               | `-135°` to `+135°` |
| Number of sweep angles    | `18`               |
| Approximate angle spacing | `15.9°`            |
| Move time                 | `1.0 s`            |
| Settle time               | `1.0 s`            |
| Output file               | `panorama.jpg`     |

The sweep currently uses:

```python
self.sweep_angles = np.linspace(-135, 135, 18)
```

This gives 18 capture positions across a 270-degree range.

Metadata and overlay behavior:

* Stores GPS coordinates from `/gps/fix`.
* Stores rover heading from `/heading`.
* Calculates camera heading as rover heading plus servo angle.
* Converts camera heading into an 8-direction compass label.
* Draws GPS text at the top-left of the panorama.
* Draws heading/cardinal markers near the top of the panorama.
* Draws servo angle markers near the bottom of the panorama.

Heading reference:

```txt
0°   = North
90°  = East
180° = South
270° = West
```

Related nodes:

| Node                            | Relationship                                               |
| ------------------------------- | ---------------------------------------------------------- |
| `motor_input.py`                | Publishes manual servo commands and panorama triggers      |
| `motor_controller.py`           | Receives `/motor_cmd` and sends mapped commands to Arduino |
| Camera node / image republisher | Provides compressed frames on `/arm_cam/image/compressed`  |
| GPS/IMU node or test publisher  | Provides `/gps/fix` and `/heading`                         |

Configuration notes:

The sweep timing and number of angles may need to be adjusted based on:

* camera field of view
* overlap between neighbouring frames
* servo speed
* camera mount vibration
* image blur during capture

## 5) Arduino Servo Firmware

The Arduino firmware is located at:

```txt
autonomy_vision/firmware/panorama_motor/panorama_motor.ino
```

The firmware controls an MG90D servo motor using the Arduino Servo library.

The Arduino expects serial commands from ROS 2 in the range:

```txt
0 to 180
```

The ROS 2 `motor_controller` node maps the camera servo range of `-135` to `+135` degrees into this Arduino range.

```txt
-135 degrees → 0
0 degrees    → 90
+135 degrees → 180
```

### Arduino Pin Wiring

| Servo Wire | Connection |
| ---------- | ---------- |
| Brown      | GND        |
| Orange     | D9         |
| Red        | 5V         |

### Arduino Serial Settings

| Setting           | Value    |
| ----------------- | -------- |
| Default baud rate | `115200` |
| Servo pin         | `D9`     |
| Startup position  | `90`     |

On startup, the Arduino moves the servo to the center position:

```cpp
panServo.write(90);
```

The Arduino prints:

```txt
READY
```

when it starts.

The serial port and baud rate are passed through `motor.launch.py`:

```bash
ros2 launch autonomy_vision motor.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=115200
```

## 6) Manual Commands and Verification

### Move Servo Manually

Move to center:

```bash
ros2 topic pub --once /motor_cmd std_msgs/msg/Float64 "{data: 0.0}"
```

Move to minimum angle:

```bash
ros2 topic pub --once /motor_cmd std_msgs/msg/Float64 "{data: -135.0}"
```

Move to maximum angle:

```bash
ros2 topic pub --once /motor_cmd std_msgs/msg/Float64 "{data: 135.0}"
```

Check that motor commands are being published:

```bash
ros2 topic echo /motor_cmd
```

### Trigger Panorama Manually

```bash
ros2 topic pub --once /panorama_trigger std_msgs/msg/Float64 "{data: 999.0}"
```

The actual value is not important. Receiving a message on `/panorama_trigger` starts the sweep.

Check that panorama trigger messages are being published:

```bash
ros2 topic echo /panorama_trigger
```

### Publish Test GPS and Heading Data

When testing the panorama sweep without the real GPS/IMU hardware, publish fake GPS and heading data in separate terminals. The heading is the rover’s facing direction, measured in degrees clockwise from north, where `0° = North`, `90° = East`, `180° = South`, and `270° = West`.

Terminal 1 — publish fake GPS:

```bash
ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix "{latitude: 43.657000, longitude: -79.380000, altitude: 100.0}" -r 1
```

Terminal 2 — publish fake heading:

```bash
ros2 topic pub /heading std_msgs/msg/Float32 "{data: 90.0}" -r 1
```

Check test GPS and heading data:

```bash
ros2 topic echo /gps/fix
ros2 topic echo /heading
```

### Test Panorama Sweep Without Camera

```bash
ros2 launch autonomy_vision motor.launch.py test_mode:=true
```

This tests the servo sweep without waiting for real camera frames from `/arm_cam/image/compressed`.

### Check Camera Output

Check that the raw camera topic is publishing:

```bash
ros2 topic hz /image_raw
```

Check that the compressed camera topic is publishing:

```bash
ros2 topic hz /arm_cam/image/compressed
```

If the servo moves to the first sweep angle and then stops, check whether `/arm_cam/image/compressed` is publishing. In normal mode, `panorama_stitcher` waits for a compressed camera frame before moving to the next angle.

### Check Output Panorama

After a successful panorama sweep, check that the output image was created:

```bash
ls -l panorama.jpg
```

### General ROS 2 Checks

Verify that the `autonomy_vision` package is visible after building and sourcing:

```bash
ros2 pkg list | grep autonomy_vision
```

List active ROS 2 nodes:

```bash
ros2 node list
```

List active ROS 2 topics:

```bash
ros2 topic list
```

## 7) Hardware and Device Notes

### Arduino Serial Device

The Arduino Nano commonly appears as:

```txt
/dev/ttyUSB0
```

However, the serial port can change depending on the connected USB devices. Use the `serial_port` launch argument if the Arduino appears on a different device path.

Example:

```bash
ros2 launch autonomy_vision motor.launch.py serial_port:=/dev/ttyUSB1
```

Check for connected serial devices:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Check whether USB devices are detected:

```bash
lsusb
```

Check recent kernel messages:

```bash
dmesg | tail -50
```

If the Arduino uses a CH340/CH341 USB serial chip and `/dev/ttyUSB*` does not appear, reload the CH340 driver:

```bash
sudo modprobe ch341
```

Then unplug and reconnect the Arduino USB cable.

If `lsusb` does not show the device at all, it is likely a hardware, cable, or USB port issue.

If `lsusb` shows the CH340 device but `/dev/ttyUSB*` does not appear, it is likely a driver issue.

### Serial Permissions

A temporary permission workaround is:

```bash
sudo chmod 666 /dev/ttyUSB0
```

A better long-term fix is to add the user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
```

Then log out and log back in.

Check current groups:

```bash
groups
```

The output should include:

```txt
dialout
```

### Camera Device

The motor/panorama launch file uses `v4l2_camera` and defaults to:

```txt
/dev/video0
```

To select a different camera device for the motor/panorama launch file:

```bash
ros2 launch autonomy_vision motor.launch.py camera_device:=/dev/video2
```

Check available camera devices:

```bash
ls -l /dev/video*
```

Check available camera devices with `v4l2-ctl`:

```bash
v4l2-ctl --list-devices
```

## 8) Dependencies Used by Current Code

```txt
rclpy
std_msgs
sensor_msgs
OpenCV / cv2
NumPy
PySerial
v4l2_camera
image_transport
```

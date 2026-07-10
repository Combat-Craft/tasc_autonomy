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

This section covers the camera servo control and panorama capture workflow. `motor_input.py` lets the user manually send servo commands or trigger a named panorama sweep, `motor_controller.py` forwards the mapped servo commands to the Arduino, and `panorama_capture.py` rotates the camera through a set range of angles while saving IP camera snapshots and GPS/heading metadata into a named sweep folder. After capture is complete, `panorama_opencv_stitcher.py` can be run separately to stitch the saved images from that folder into a final panorama.

## Table of Contents

- [0) TL;DR](#0-tldr)
- [1) Build](#1-build)
- [2) Topics](#2-topics)
- [3) Launch File](#3-launch-file)
- [4) Nodes](#4-nodes)
- [5) Arduino Servo Firmware](#5-arduino-servo-firmware)
- [6) Manual Commands and Verification](#6-manual-commands-and-verification)
- [7) Hardware and Device Notes](#7-hardware-and-device-notes)
- [8) Dependencies Used by Code](#8-dependencies-used-by-current-code)

## 0) TL;DR

Arduino wiring using the Arduino Nano pinout:

| Servo Wire | Connection |
| ---------- | ---------- |
| Brown      | GND        |
| Orange     | D9         |
| Red        | 5V         |

Load ch341 driver on the Jetson, since Arduino uses a CH340/CH341 USB serial chip:
```bash
cd CH341SER
sudo make load
```

Launching and starting nodes:

### Terminal 1 — Launch Motor Controller and Panorama Capture (Non-Interactive Nodes)

```bash
cd ~/autonomy_ws
colcon build --packages-select autonomy_vision
source install/setup.bash
ros2 launch autonomy_vision motor.launch.py
```

When not using IP camera, put in test mode

```bash
ros2 launch autonomy_vision motor.launch.py test_mode:=true
```

### Terminal 2 — Motor Input (Interactive node used by user to control the motor)

```bash
cd ~/autonomy_ws
source install/setup.bash
ros2 run autonomy_vision motor_input
```

### Terminal 3 — Fake Heading (Testing-only)

```bash
cd ~/autonomy_ws
source install/setup.bash
ros2 topic pub /heading std_msgs/msg/Float32 "{data: 90.0}" -r 1
```

### Terminal 4 — Fake GPS (Testing-only)

```bash
cd ~/autonomy_ws
source install/setup.bash
ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix "{latitude: 43.657000, longitude: -79.380000, altitude: 100.0}" -r 1
```

### Run the Stitcher

After the sweep finishes:

```bash
cd ~/autonomy_ws//src/tasc_autonomy/autonomy_vision/autonomy_vision
python3 panorama_opencv_stitcher.py
```

Example input for when it prompts for folder path:

```txt
~/panorama_images/test_sweep_01
```

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
| `/panorama_trigger` | `std_msgs/String` | Input/Output | Trigger topic for starting a named panorama sweep; the message data is used as the output folder name |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Input | GPS position saved with each captured frame |
| `/heading` | `std_msgs/Float32` | Input | Rover heading in degrees, used to calculate camera heading/cardinal metadata |

## 3) Launch File

### `motor.launch.py`

Launches the motor controller and panorama capture nodes needed for manual servo control and panorama capture.

Normal mode with camera:

```bash
ros2 launch autonomy_vision motor.launch.py
```

Test mode without requiring camera frames:

```bash
ros2 launch autonomy_vision motor.launch.py test_mode:=true
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
| `serial_port`   | Serial port used by the Arduino motor controller              | `/dev/ttyUSB0` |
| `baud_rate`     | Arduino serial communication baud rate                        | `115200`       |

Started nodes:

| Node                | Description                                                            |
| ------------------- | ---------------------------------------------------------------------- |
| `panorama_capture.py`  | Subscribes to `/motor_cmd` and forwards mapped servo values to Arduino |
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

Allows the user to control the camera servo and trigger a named panorama sweep from the terminal.

User inputs:

| Input                        | Action |
| ---------------------------- | ------ |
| Number from `-135` to `+135` | Publishes a servo angle command to `/motor_cmd` |
| `p,folder_name`              | Triggers a panorama sweep and sends `folder_name` to `/panorama_trigger` |
| `e`                          | Exits the node |

Published topics:

| Topic               | Type               | Description |
| ------------------- | ------------------ | ----------- |
| `/motor_cmd`        | `std_msgs/Float64` | Manual servo angle command |
| `/panorama_trigger` | `std_msgs/String`  | Requested panorama sweep folder name |

Related nodes:

| Node                   | Relationship |
| ---------------------- | ------------ |
| `motor_controller.py`  | Subscribes to `/motor_cmd` and sends the converted servo command to Arduino |
| `panorama_capture.py`  | Subscribes to `/panorama_trigger`, creates the requested sweep folder, moves the servo, and saves captured images |

Example prompt:

```txt
Enter angle (-135 to 135 | 'p,folder_name' for panorama | 'e' to exit):
```

Example panorama trigger:

```txt
p,test_sweep_01
```

---

### `motor_controller.py`

Subscribes to camera servo angle commands and sends the mapped servo value to the Arduino over serial.

Subscribed topics:

| Topic        | Type               | Description |
| ------------ | ------------------ | ----------- |
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

| Node                  | Relationship |
| --------------------- | ------------ |
| `motor_input.py`      | Publishes manual servo angle commands to `/motor_cmd` |
| `panorama_capture.py` | Publishes automated servo angle commands to `/motor_cmd` during a panorama sweep |

Troubleshooting serial device detection:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
lsusb
dmesg | tail -50
dmesg | grep -iE "ch34|ttyUSB|ttyACM"
```

If the Arduino uses a CH340/CH341 USB serial chip and the device does not appear, first try reloading the built-in driver:

```bash
sudo modprobe ch341
```

Then unplug and reconnect the Arduino USB cable.

If the built-in driver still does not work and the CH341SER driver is already available on the Jetson, load it with:

```bash
cd CH341SER
sudo make load
```

---

### `panorama_capture.py`

Controls an automated camera panorama capture sweep.

When triggered, this node moves the camera servo through a sequence of angles from `-135°` to `+135°`, captures one IP camera snapshot at each angle, stores GPS/heading metadata for each frame, and saves the captured images into a named sweep folder.

Main behavior:

1. Waits for a trigger message on `/panorama_trigger`.
2. Uses the trigger message as the requested sweep folder name.
3. Creates a unique folder inside `~/panorama_images`.
4. Publishes servo angle commands to `/motor_cmd`.
5. Waits for the servo to move and settle.
6. Captures one JPEG snapshot from the configured IP camera URL.
7. Saves the image as `frame_000.jpg`, `frame_001.jpg`, etc.
8. Stores the servo angle, GPS coordinates, rover heading, camera heading, and camera cardinal direction.
9. Repeats until all sweep angles are complete.
10. Saves `metadata.csv` inside the sweep folder.
11. Resets and waits for the next trigger.

Published topics:

| Topic        | Type               | Description |
| ------------ | ------------------ | ----------- |
| `/motor_cmd` | `std_msgs/Float64` | Target camera servo angle in degrees, from `-135` to `+135` |

Subscribed topics:

| Topic               | Type                    | Description |
| ------------------- | ----------------------- | ----------- |
| `/panorama_trigger` | `std_msgs/String`       | Trigger message used to start the panorama sweep; the message data is used as the output folder name |
| `/gps/fix`          | `sensor_msgs/NavSatFix` | GPS coordinates saved with each captured frame |
| `/heading`          | `std_msgs/Float32`      | Rover heading in degrees clockwise from north |
| `/cardinal_compass` | `std_msgs/String`       | Optional rover cardinal direction input using a 16-point compass rose |

Parameters:

| Parameter   | Description | Default |
| ----------- | ----------- | ------- |
| `test_mode` | Simulates image captures and tests motor movement without requiring the real IP camera | `false` |
| `cam_url`   | IP camera snapshot URL used to capture JPEG images | `http://192.168.1.117:6688/snapshot/PROFILE_000` |
| `save_dir`  | Base directory where named sweep folders are created | `~/panorama_images` |

Output folder structure:

```txt
~/panorama_images/
└── test_sweep_01/
    ├── frame_000.jpg
    ├── frame_001.jpg
    ├── frame_002.jpg
    ├── ...
    └── metadata.csv
```

`metadata.csv` columns:

```txt
frame
servo_angle
gps_latitude
gps_longitude
rover_heading
camera_heading
camera_cardinal
image_path
```

Sweep configuration:

| Setting                   | Value |
| ------------------------- | ----- |
| Sweep range               | `-135°` to `+135°` |
| Number of capture angles  | `10` |
| Approximate angle spacing | `30°` |
| Move time                 | `1.0 s` |
| Settle time               | `1.0 s` |
| ROS sweep output          | `frame_*.jpg` images and `metadata.csv` |
| Stitching output          | `panorama.jpg`, created later by `panorama_opencv_stitcher.py` |

The sweep currently uses:

```python
self.sweep_angles = np.linspace(-135, 135, 10)
```

This gives 10 capture positions across a 270-degree range. Since there are 9 gaps between 10 positions, the spacing is approximately 30 degrees.

Heading reference:

```txt
0°   = North
90°  = East
180° = South
270° = West
```

The camera-facing heading is calculated as:

```txt
camera_heading = rover_heading + servo_angle
```

The code converts the camera heading into a 16-point compass label such as `N`, `NNE`, `NE`, `ENE`, `E`, `ESE`, `SE`, or `SSE`.

Related nodes/scripts:

| Node/Script                    | Relationship |
| ------------------------------ | ------------ |
| `motor_input.py`               | Publishes manual servo commands and named panorama triggers |
| `motor_controller.py`          | Receives `/motor_cmd` and sends mapped commands to Arduino |
| GPS/IMU node or test publisher | Provides `/gps/fix` and `/heading` |
| `panorama_opencv_stitcher.py`  | Loads saved `frame_*.jpg` images and `metadata.csv`, stitches them, draws labels, and saves `panorama.jpg` |

Configuration notes:

The sweep timing and number of angles may need to be adjusted based on:

* camera field of view
* overlap between neighbouring frames
* servo speed
* camera mount vibration
* image blur during capture
* IP camera snapshot response time

---

### `panorama_opencv_stitcher.py`

Standalone OpenCV script for stitching panorama sweep images after they have been captured by `panorama_capture.py`.

This script is intentionally separate from ROS 2 so that stitching can be run after image capture without causing CPU spikes during the live sweep.

Input:

| Input | Description |
| ----- | ----------- |
| Sweep folder path | Folder containing `frame_*.jpg` images and optional `metadata.csv` |

Output:

| Output | Description |
| ------ | ----------- |
| `panorama.jpg` | Final stitched panorama image saved inside the same sweep folder |

Expected folder structure:

```txt
~/panorama_images/
└── test_sweep_01/
    ├── frame_000.jpg
    ├── frame_001.jpg
    ├── frame_002.jpg
    ├── ...
    ├── metadata.csv
    └── panorama.jpg
```

Main behavior:

1. Prompts the user for a sweep folder path.
2. Loads all valid `frame_*.jpg` images in sorted order.
3. Reads `metadata.csv` if it exists.
4. Uses OpenCV's panorama stitcher to create a panorama.
5. Draws the GPS label on the top-left of the final panorama.
6. Draws approximate camera cardinal/heading labels across the top based on frame order.
7. Saves the result as `panorama.jpg` inside the same sweep folder.

Run example:

```bash
python3 panorama_opencv_stitcher.py
```

Example folder path to enter:

```txt
~/panorama_images/test_sweep_01
```

Notes:

* At least 2 valid `frame_*.jpg` images are needed to stitch.
* Stitching can fail if there is not enough overlap between images, if the images are blurry, or if the camera angle spacing is too large.
* Cardinal label positions are approximate because OpenCV warps and crops the input images internally.

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

Use the Arduino Nano pinout diagram. The servo signal wire connects to the Nano's digital pin `D9`.

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

Trigger a named panorama sweep:

```bash
ros2 topic pub --once /panorama_trigger std_msgs/msg/String "{data: 'test_sweep_01'}"
```

The string value is used as the requested output folder name inside `~/panorama_images`.

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

Optional — publish fake rover cardinal direction:

```bash
ros2 topic pub /cardinal_compass std_msgs/msg/String "{data: 'E'}" -r 1
```

Check test GPS, heading, and cardinal data:

```bash
ros2 topic echo /gps/fix
ros2 topic echo /heading
ros2 topic echo /cardinal_compass
```

### Test Panorama Sweep Without IP Camera

```bash
ros2 launch autonomy_vision motor.launch.py test_mode:=true
```

This tests the servo sweep without requiring real IP camera snapshots.

### Check IP Camera Snapshot URL

The capture node uses the `cam_url` parameter to request one JPEG snapshot at each servo angle. The default URL is:

```txt
http://192.168.1.117:6688/snapshot/PROFILE_000
```

Before running a real sweep, check that the IP camera URL is reachable from the Jetson or computer running the node. For example, open the URL in a browser or download one test image:

```bash
wget -O test_snapshot.jpg "http://192.168.1.117:6688/snapshot/PROFILE_000"
```

### Check Output Sweep Folder

After a successful panorama capture sweep, check that the output folder was created:

```bash
ls -l ~/panorama_images
```

Check the saved frames and metadata:

```bash
ls -l ~/panorama_images/test_sweep_01
```

The folder should contain files similar to:

```txt
frame_000.jpg
frame_001.jpg
frame_002.jpg
...
metadata.csv
```

### Run the OpenCV Stitcher

After capturing the sweep images, run the standalone stitcher:

```bash
python3 panorama_opencv_stitcher.py
```

When prompted, enter the sweep folder path:

```txt
~/panorama_images/test_sweep_01
```

After successful stitching, check that the panorama was created:

```bash
ls -l ~/panorama_images/test_sweep_01/panorama.jpg
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
dmesg | grep -iE "ch34|ttyUSB|ttyACM"
```

If the Arduino uses a CH340/CH341 USB serial chip and `/dev/ttyUSB*` does not appear, reload the built-in CH340 driver:

```bash
sudo modprobe ch341
```

Then unplug and reconnect the Arduino USB cable.

If the built-in driver still does not work and the CH341SER driver is already available on the Jetson, load it with:

```bash
cd CH341SER
sudo make load
```

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

### IP Camera Snapshot URL

The current panorama capture workflow uses an IP camera snapshot URL instead of subscribing to a ROS compressed image topic.

Default snapshot URL:

```txt
http://192.168.1.117:6688/snapshot/PROFILE_000
```

The IP camera and the Jetson/computer running the node must be on the same reachable network. If image capture fails, check that the URL can be opened from the same machine running `panorama_capture.py`.

Example test:

```bash
wget -O test_snapshot.jpg "http://192.168.1.117:6688/snapshot/PROFILE_000"
```

## 8) Dependencies Used by Current Code

```txt
rclpy
std_msgs
sensor_msgs
OpenCV / cv2
NumPy
PySerial
csv
os
pathlib
time
urllib.request
```

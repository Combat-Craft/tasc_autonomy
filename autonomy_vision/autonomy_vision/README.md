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

This section covers the camera servo control and panorama capture workflow. Foxglove is used to publish commands through two ROS 2 topics: /motor_cmd, which receives servo angle commands in degrees, and /panorama_trigger, which receives the requested panorama sweep folder name. motor_controller.py subscribes to /motor_cmd and forwards the mapped servo commands to the Arduino, while panorama_capture.py subscribes to /panorama_trigger and rotates the camera through a set range of angles while saving IP camera snapshots and GPS/heading metadata into a named sweep folder. After capture is complete, panorama_opencv_stitcher.py can be run separately to stitch the saved images from that folder into a final panorama.

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

Arduino Nano wiring for the MG90D servo motor, using the Arduino Nano pinout:

| Servo Wire | Connection |
| ---------- | ---------- |
| Brown      | GND        |
| Orange     | D9         |
| Red        | 5V         |

Load the CH341 driver on the Jetson, since the Arduino uses a CH340/CH341 USB serial chip:

```bash
cd CH341SER
sudo make load
```

Launching and starting nodes:

### Terminal 1 — Launch Motor Controller and Panorama Capture

These are the non-interactive nodes.

```bash
cd ~/autonomy_ws
colcon build --packages-select autonomy_vision
source install/setup.bash
ros2 launch autonomy_vision motor.launch.py
```

When testing without the IP camera, use test mode:

```bash
ros2 launch autonomy_vision motor.launch.py test_mode:=true
```

### Foxglove — Control Servo and Trigger Panorama

Foxglove is the main method for sending user commands. Foxglove’s **"Publish"** panels are used to publish messages directly to ROS 2 topics. In the Publish panel, Foxglove provides a message field like:

```json
{
  "data": 
}
```

The value after `"data":` is what we enter depending on the topic.

To manually move the servo, publish to:

```txt
/motor_cmd
```

Message type:

```txt
std_msgs/msg/Float64
```

Example message:

```json
{
  "data": 0.0
}
```

To trigger a named panorama sweep, publish to:

```txt
/panorama_trigger
```

Message type:

```txt
std_msgs/msg/String
```

Example message:

```json
{
  "data": "test_sweep_01"
}
```

For `/panorama_trigger`, only enter the folder name as the string value.

### Alternative — Terminal Motor Input

If Foxglove is not being used, `motor_input.py` can be run separately on a new terminal.

```bash
cd ~/autonomy_ws
source install/setup.bash
ros2 run autonomy_vision motor_input
```

In `motor_input.py`, example commands as follows:

Moves the servo to the center position.
```txt
0
```

Triggers a panorama sweep and saves the output in a folder named `test_sweep_01`.
```txt
p,test_sweep_01
```

Exits the `motor_input.py` node.
```txt
e
```

### Terminal 2 — Fake Heading Data

Testing-only, if the real heading/IMU source is not running:

```bash
cd ~/autonomy_ws
source install/setup.bash
ros2 topic pub /heading std_msgs/msg/Float32 "{data: 90.0}" -r 1
```

### Terminal 3 — Fake GPS Data

Testing-only, if the real GPS source is not running:

```bash
cd ~/autonomy_ws
source install/setup.bash
ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix "{latitude: 43.657000, longitude: -79.380000, altitude: 100.0}" -r 1
```

### Run the Stitcher

After the sweep finishes:

```bash
cd ~/autonomy_ws/src/tasc_autonomy/autonomy_vision/autonomy_vision
python3 panorama_opencv_stitcher.py
```

Example input when prompted for the sweep folder path:

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

| Topic               | Type                    | Direction                                                                           | Description                                                                                                           |
| ------------------- | ----------------------- | ----------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `/motor_cmd`        | `std_msgs/msg/Float64`      | Published by Foxglove or `panorama_capture.py`; subscribed by `motor_controller.py` | Servo angle command in degrees, from `-120` to `+120`. Foxglove can publish to this topic to manually move the servo. |
| `/panorama_trigger` | `std_msgs/msg/String`       | Published by Foxglove; subscribed by `panorama_capture.py`                          | Trigger topic for starting a named panorama sweep. The message data is used as the output folder name.                |
| `/gps/fix`          | `sensor_msgs/msg/NavSatFix` | Subscribed by `panorama_capture.py`                                                 | GPS latitude and longitude saved with each captured frame.                                                            |
| `/heading`          | `std_msgs/msg/Float32`      | Subscribed by `panorama_capture.py`                                                 | Rover heading in degrees, used to calculate camera heading and camera cardinal metadata.                              |
| `/rosout`           | ROS 2 logging topic     | Published by ROS 2 nodes                                                            | ROS 2 log output from `motor_controller.py` and `panorama_capture.py`.                                       |

## 3) Launch File

### `motor.launch.py`

Launches the following nodes needed for camera servo control and panorama capture:

* `motor_controller.py`
* `panorama_capture.py`

In Foxglove, use the **"Publish"** panels to publish directly to `/motor_cmd` for manual servo movement and `/panorama_trigger` for starting a named panorama sweep.

Normal mode with real IP camera snapshots:

```bash
ros2 launch autonomy_vision motor.launch.py
```

Test mode without requiring real IP camera snapshots:

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

Override the IP camera snapshot URL:

```bash
ros2 launch autonomy_vision motor.launch.py cam_url:=http://192.168.1.117:6688/snapshot/PROFILE_000
```

Override the panorama save directory:

```bash
ros2 launch autonomy_vision motor.launch.py save_dir:=~/panorama_images
```

Launch arguments:

| Argument      | Description                                                                                                                           | Default                                          |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| `test_mode`   | Runs `panorama_capture.py` without requiring real IP camera snapshots. This is useful for testing the sweep logic without the camera. | `false`                                          |
| `serial_port` | Serial port used by the Arduino motor controller.                                                                                     | `/dev/ttyUSB0`                                   |
| `baud_rate`   | Arduino serial communication baud rate.                                                                                               | `115200`                                         |
| `cam_url`     | IP camera snapshot URL used by `panorama_capture.py` to capture JPEG images.                                                          | `http://192.168.1.117:6688/snapshot/PROFILE_000` |
| `save_dir`    | Base folder where panorama sweep folders are saved.                                                                                   | `~/panorama_images`                              |

Started nodes:

| Node                  | Description                                                                                                                                                                                                                         |
| --------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `motor_controller.py` | Subscribes to `/motor_cmd`, maps the `-120` to `+120` degree command range into the Arduino Servo library `0` to `180` range, and sends the command to the Arduino over serial.                                                     |
| `panorama_capture.py` | Subscribes to `/panorama_trigger`, `/gps/fix`, and `/heading`; publishes servo commands to `/motor_cmd`; captures IP camera snapshots from `cam_url`; and saves `frame_*.jpg` images plus `metadata.csv` into a named sweep folder. |

Optional alternative input node:

| Node             | Description                                                                                                                                                                                              |
| ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `motor_input.py` | Optional terminal-based alternative to Foxglove. It can publish manual servo commands to `/motor_cmd` and panorama trigger messages to `/panorama_trigger`, but it is not launched by `motor.launch.py`. |

Motor control flow with Foxglove:

```txt
Foxglove Publish panel → /motor_cmd → motor_controller → Arduino Nano → MG90D servo
```

Panorama trigger flow with Foxglove:

```txt
Foxglove Publish panel → /panorama_trigger → panorama_capture
panorama_capture → /motor_cmd → motor_controller → Arduino Nano → MG90D servo sweep
panorama_capture → IP camera snapshot URL → saves frame_*.jpg and metadata.csv
```

Optional terminal-based flow using motor_input.py:

```txt
motor_input → /motor_cmd → motor_controller → Arduino Nano → MG90D servo
motor_input → /panorama_trigger → panorama_capture
```


## 4) Nodes

### `motor_input.py` (Optional alternative to Foxglove)

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
| `/motor_cmd`        | `std_msgs/msg/Float64` | Manual servo angle command |
| `/panorama_trigger` | `std_msgs/msg/String`  | Requested panorama sweep folder name |

Related nodes:

| Node                   | Relationship |
| ---------------------- | ------------ |
| `motor_controller.py`  | Subscribes to `/motor_cmd` and sends the converted servo command to Arduino |
| `panorama_capture.py`  | Subscribes to `/panorama_trigger`, creates the requested sweep folder, moves the servo, and saves captured images |

Example prompt:

```txt
Enter angle (-135 to 135 | 'p,folder_name' for panorama | 'e' to exit):
```

Example input commands are as follows:

Moves the servo to the center position.
```txt
0
```

Triggers a panorama sweep and saves the output in a folder named `test_sweep_01`.
```txt
p,test_sweep_01
```

Exits the `motor_input.py` node.
```txt
e
```

---

### `motor_controller.py`

Subscribes to camera servo angle commands and sends the mapped servo value to the Arduino over serial.

Subscribed topics:

| Topic        | Type               | Description                                                    |
| ------------ | ------------------ | -------------------------------------------------------------- |
| `/motor_cmd` | `std_msgs/msg/Float64` | Servo angle command in degrees, expected from `-120` to `+120` |

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

The Arduino Servo library expects values from `0` to `180`, but the ROS 2 control range is `-120` to `+120` degrees. The node maps the ROS 2 angle into the Arduino servo range.

```txt
-120 degrees → 0
0 degrees    → 90
+120 degrees → 180
```

Related command sources:

| Source                 | Relationship                                                                                     |
| ---------------------- | ------------------------------------------------------------------------------------------------ |
| Foxglove Publish panel | Can publish manual servo angle commands directly to `/motor_cmd`                                 |
| `panorama_capture.py`  | Publishes automated servo angle commands to `/motor_cmd` during a panorama sweep                 |
| `motor_input.py`       | Optional terminal-based alternative that can publish manual servo angle commands to `/motor_cmd` |

Foxglove trigger method:

Foxglove is the main user-control method. Use Foxglove’s **Publish** panel to publish directly to `/motor_cmd`.

Topic:

```txt
/motor_cmd
```

Message type:

```txt
std_msgs/msg/Float64
```

Example Foxglove message:

```json
{
  "data": -45
}
```


Troubleshooting serial device detection:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
lsusb
dmesg | tail -50
dmesg | grep -iE "ch34|ttyUSB|ttyACM"
```

The Arduino uses a CH340/CH341 USB serial chip, so it should usually appear as `/dev/ttyUSB0` or another `/dev/ttyUSB*` device.

On the Jetson, if the Arduino appears in `lsusb` but no `/dev/ttyUSB*` device appears, load the CH341SER driver:

```bash
cd CH341SER
sudo make load
```

On the Jetson, there is usually no need to unplug and reconnect the Arduino USB cable after running `sudo make load`.

If you are using your VM instead of the Jetson, first try reloading the built-in CH341 driver:

```bash
sudo modprobe ch341
```

Then unplug and reconnect the Arduino USB cable.

If the device still does not appear, try a different USB cable, USB port, or check whether the USB device is being passed through correctly to the VM.

---

### `panorama_capture.py`

Controls an automated camera panorama capture sweep.

When triggered, this node moves the camera servo through a sequence of angles from `-120°` to `+120°`, captures one IP camera snapshot at each angle, stores GPS/heading metadata for each frame, and saves the captured images into a named sweep folder.

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
11. Returns the servo to `0°`, resets, and waits for the next trigger.

Published topics:

| Topic        | Type               | Description                                                 |
| ------------ | ------------------ | ----------------------------------------------------------- |
| `/motor_cmd` | `std_msgs/msg/Float64` | Target camera servo angle in degrees, from `-120` to `+120` |

Subscribed topics:

| Topic               | Type                    | Description                                                                                          |
| ------------------- | ----------------------- | ---------------------------------------------------------------------------------------------------- |
| `/panorama_trigger` | `std_msgs/msg/String`       | Trigger message used to start the panorama sweep; the message data is used as the output folder name |
| `/gps/fix`          | `sensor_msgs/msg/NavSatFix` | GPS coordinates saved with each captured frame                                                       |
| `/heading`          | `std_msgs/msg/Float32`      | Rover heading in degrees clockwise from north                                                        |

Parameters:

| Parameter   | Description                                                                      | Default                                          |
| ----------- | -------------------------------------------------------------------------------- | ------------------------------------------------ |
| `test_mode` | Simulates captures and tests motor movement without requiring the real IP camera | `false`                                          |
| `cam_url`   | IP camera snapshot URL used to capture JPEG images                               | `http://192.168.1.117:6688/snapshot/PROFILE_000` |
| `save_dir`  | Base directory where named sweep folders are created                             | `~/panorama_images`                              |

Foxglove trigger method:

Foxglove is the main user-control method. Use Foxglove’s **Publish** panel to publish directly to `/panorama_trigger`.

Topic:

```txt
/panorama_trigger
```

Message type:

```txt
std_msgs/msg/String
```

Example Foxglove message:

```json
{
  "data": "test_sweep_01"
}
```

The string value is used as the requested output folder name inside `~/panorama_images`.

Do not include `p,` when publishing from Foxglove. The `p,folder_name` format is only used by the optional `motor_input.py` terminal input node.

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

| Setting                   | Value                                                          |
| ------------------------- | -------------------------------------------------------------- |
| Sweep range               | `-120°` to `+120°`                                             |
| Number of capture angles  | `12`                                                           |
| Approximate angle spacing | `21.8°`                                                        |
| Move time                 | `1.0 s`                                                        |
| Settle time               | `1.0 s`                                                        |
| ROS sweep output          | `frame_*.jpg` images and `metadata.csv`                        |
| Stitching output          | `panorama.jpg`, created later by `panorama_opencv_stitcher.py` |

The sweep currently uses:

```python
self.sweep_angles = np.linspace(-120, 120, 12)
```

This gives 12 capture positions across a 240-degree range. Since there are 11 gaps between 12 positions, the spacing is approximately 21.8 degrees.

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

The code calculates the camera-facing heading by adding the rover’s current heading to the servo angle, then converts that heading into a 16-point compass label: `N`, `NNE`, `NE`, `ENE`, `E`, `ESE`, `SE`, `SSE`, `S`, `SSW`, `SW`, `WSW`, `W`, `WNW`, `NW`, or `NNW`.

```python
camera_heading = (self.latest_heading + current_servo_angle) % 360.0
camera_cardinal = self.heading_to_cardinal_16(camera_heading)
```

The modulo operation `% 360.0` keeps the heading within the normal compass range of `0°` to `359°`.


Related nodes/scripts:

| Node/Script                    | Relationship                                                                                               |
| ------------------------------ | ---------------------------------------------------------------------------------------------------------- |
| Foxglove Publish panel         | Publishes `/motor_cmd` for manual servo movement and `/panorama_trigger` for panorama sweeps               |
| `motor_controller.py`          | Receives `/motor_cmd` and sends mapped commands to Arduino                                                 |
| GPS/IMU node or test publisher | Provides `/gps/fix` and `/heading`                                                                         |
| `motor_input.py`               | Optional terminal-based alternative to Foxglove                                                            |
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

| Input             | Description                                                        |
| ----------------- | ------------------------------------------------------------------ |
| Sweep folder path | Folder containing `frame_*.jpg` images and optional `metadata.csv` |

Output:

| Output         | Description                                                      |
| -------------- | ---------------------------------------------------------------- |
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
cd ~/autonomy_ws/src/tasc_autonomy/autonomy_vision/autonomy_vision
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

The firmware controls an MG90D servo motor using the Arduino Servo library. The MG90D is a positional servo motor.

The Arduino expects serial commands from ROS 2 in the range:

```txt
0 to 180
```

The ROS 2 `motor_controller` node maps the camera servo range of `-120` to `+120` degrees into this Arduino range.

```txt
-120 degrees → 0
0 degrees    → 90
+120 degrees → 180
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

By default, `motor.launch.py` uses `/dev/ttyUSB0` as the Arduino serial port and `115200` as the baud rate. These values only need to be specified in the launch command if you want to override the defaults.

Default launch command:

```bash
ros2 launch autonomy_vision motor.launch.py
```

Example override if the Arduino appears on a different serial device:

```bash
ros2 launch autonomy_vision motor.launch.py serial_port:=/dev/ttyUSB1 baud_rate:=115200
```

## 6) Manual Commands and Verification

### Move Servo Manually

The main method for manual servo movement is Foxglove’s **"Publish"** panel.

Publish to:

```txt
/motor_cmd
```

Message type:

```txt
std_msgs/msg/Float64
```

Foxglove message format:

```json
{
  "data": 0.0
}
```

Example values:
Moves the servo to the center position.
```json
{
  "data": 0.0
}
```

Moves the servo to the minimum angle.
```json
{
  "data": -120.0
}
```

Moves the servo to the maximum angle.
```json
{
  "data": 120.0
}
```

Terminal command equivalent:

```bash
ros2 topic pub --once /motor_cmd std_msgs/msg/Float64 "{data: 0.0}"
ros2 topic pub --once /motor_cmd std_msgs/msg/Float64 "{data: -120.0}"
ros2 topic pub --once /motor_cmd std_msgs/msg/Float64 "{data: 120.0}"
```

To check motor commands, run this before publishing a command:

```bash
ros2 topic echo /motor_cmd
```

### Trigger Panorama Manually

The main method for triggering a panorama sweep is Foxglove’s **"Publish"** panel.

Publish to:

```txt
/panorama_trigger
```

Message type:

```txt
std_msgs/msg/String
```

Foxglove message format:

```json
{
  "data": "test_sweep_01"
}
```

The string value is used as the requested output folder name inside `~/panorama_images`.

Terminal command equivalent:

```bash
ros2 topic pub --once /panorama_trigger std_msgs/msg/String "{data: 'test_sweep_01'}"
```

To check panorama trigger messages, run this before publishing a trigger:

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

### Test Panorama Sweep Without IP Camera

```bash
ros2 launch autonomy_vision motor.launch.py test_mode:=true
```

This tests the panorama sweep logic without requiring real IP camera snapshots. The launch file still starts `motor_controller.py`, so the Arduino serial connection is still expected unless you modify the launch file or run `panorama_capture.py` separately for software-only testing.

### Check IP Camera Snapshot URL

The capture node uses the `cam_url` parameter to request one JPEG snapshot at each servo angle. The default URL is:

```txt
http://192.168.1.117:6688/snapshot/PROFILE_000
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
cd ~/autonomy_ws/src/tasc_autonomy/autonomy_vision/autonomy_vision
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

The Arduino uses a CH340/CH341 USB serial chip, so it should usually appear as `/dev/ttyUSB0` or another `/dev/ttyUSB*` device.

On the Jetson, if the Arduino appears in `lsusb` but no `/dev/ttyUSB*` device appears, load the CH341SER driver:

```bash
cd CH341SER
sudo make load
```

There is no need to unplug and reconnect the Arduino USB cable when using the Jetson.

Initial CH341SER installation is only needed once and has already been done on the Jetson:

```bash
git clone https://github.com/juliagoda/CH341SER.git
cd CH341SER
make
sudo make load
```

If you are using your own VM instead of the Jetson, first try reloading the built-in CH341 driver:

```bash
sudo modprobe ch341
```

Then unplug and reconnect the Arduino USB cable.

If `lsusb` does not show the device at all, it is likely a hardware, cable, USB passthrough, or USB port issue.

If `lsusb` shows the CH340/CH341 device but `/dev/ttyUSB*` does not appear, it is likely a driver issue.

### Serial Permissions

The user running the ROS 2 nodes needs permission to access the Arduino serial device.

A long-term fix is to add the user to the `dialout` group:

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

## 8) Dependencies Used by Current Code

External ROS2/Python dependencies:

```txt
rclpy 
std_msgs 
sensor_msgs 
launch 
launch_ros 
OpenCV / cv2 
NumPy 
PySerial
```

Python standard library modules used:
```txt
csv
os
pathlib
time
urllib.request
```

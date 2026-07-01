# autonomy_vision

ROS 2 autonomy vision package for the rover. This package currently includes webcam YOLO detection, servo motor control through Arduino serial, and panorama sweeping/stitching using a camera mounted on a servo.

## Package Structure

```txt
autonomy_vision/
├── autonomy_vision/
│   ├── old_code/
│   |   ├── new_pano_stitcher.py
│   |   └── h265_camera_streamer.py  
│   ├── __init__.py
│   ├── morse_code_detector.py
│   ├── motor_controller.py
│   ├── motor_input.py
│   ├── multi_camera_streamer.py
│   ├── panorama_stitcher.py
│   ├── webcam_detection2D.py
│   ├── yolo_depth_v1.py
│   ├── yolo_pc.py
│   └── README.md
│
├── launch/
│   ├── detection.launch.py
│   ├── gemini2_edited2.launch.py
│   ├── morse_detector.launch.py
│   ├── motor.launch.py
│   ├── orbbec.launch.py
│   ├── vision.launch.py
│   └── README.md   
│
├── firmware/
│   ├── hello_morse.mp4
│   └── panorama_motor/
│       └── panorama_motor.ino
│
├── resource/
│   └── autonomy_vision   
│
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
│
├── package.xml
├── README.md
├── setup.cfg
└── setup.py
```

## Build

Go to the main ROS 2 workspace folder. This folder should contain `src`, `build`, `install`, and `log`.

```bash
cd ~/<your_ros2_ws>
colcon build --packages-select autonomy_vision
source install/setup.bash
```

## Launch Files

### `detection.launch.py`

Launches:

1. USB camera node
2. YOLO detection node

```bash
ros2 launch autonomy_vision detection.launch.py
```

System flow:

```txt
USB Camera → usb_cam → /image_raw → webcam_detection2D → /detections + /debug_image
```

Default camera device:

```txt
/dev/back_web_cam
```

Example with a different camera device:

```bash
ros2 launch autonomy_vision detection.launch.py camera_device:=/dev/video6
```

Optional launch arguments:

| Argument        | Description         | Default             |
| --------------- | ------------------- | ------------------- |
| `camera_device` | Camera device path  | `/dev/back_web_cam` |
| `image_width`   | Camera image width  | `640`               |
| `image_height`  | Camera image height | `480`               |
| `framerate`     | Camera frame rate   | `30.0`              |

Example:

```bash
ros2 launch autonomy_vision detection.launch.py camera_device:=/dev/video6 image_width:=640 image_height:=480 framerate:=30.0
```

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

Important topics:

| Topic                       | Description                                           |
| --------------------------- | ----------------------------------------------------- |
| `/motor_cmd`                | Servo angle command in degrees, from `-135` to `+135` |
| `/panorama_trigger`         | Trigger message used to start the panorama sweep      |
| `/image_raw`                | Raw image topic from the camera node                  |
| `/arm_cam/image/compressed` | Compressed image topic used by `panorama_stitcher`    |

### `gemini2_edited2.launch.py`

### `morse_detector.launch.py`

### `orbbec.launch.py`

## Nodes

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

### `webcam_detection2D.py`

Runs YOLOv11n object detection on images from a USB camera.

It subscribes to:

```txt
/image_raw
```

It publishes detection results to:

```txt
/detections
```

It publishes a debug image with bounding boxes to:

```txt
/debug_image
```

The node loads:

```txt
yolo11n.pt
```

It uses CUDA if available. Otherwise, it runs on CPU.

The detector resizes/pads incoming frames to:

```txt
320 x 320
```

The confidence threshold is currently:

```txt
0.4
```

The node logs FPS approximately every 3 seconds.

### `morse_code_detector.py`

### `multi_camera_streamer.py`

### `yolo_depth_v1.py`

### `yolo_pc.py`

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/motor_cmd` | `std_msgs/Float64` | Input/Output | Servo angle command in degrees, from `-135` to `+135` |
| `/panorama_trigger` | `std_msgs/Float64` | Input/Output | Trigger topic for starting the panorama sweep |
| `/image_raw` | `sensor_msgs/Image` | Input/Output | Raw camera image from the camera node |
| `/debug_image` | `sensor_msgs/Image` | Output | YOLO debug image with bounding boxes |
| `/detections` | `vision_msgs/Detection2DArray` | Output | YOLO object detection results |
| `/arm_cam/image/compressed` | `sensor_msgs/CompressedImage` | Input/Output | Compressed camera image used by `panorama_stitcher` |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Input | GPS position saved with panorama metadata |
| `/heading` | `std_msgs/Float32` | Input | Rover heading in degrees, used to calculate camera heading/cardinal overlay |

## Arduino Servo Firmware

The Arduino firmware is located at:

```txt
launch/panorama_motor/panorama_motor.ino
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

## Manual Commands

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

### Trigger Panorama Manually

```bash
ros2 topic pub --once /panorama_trigger std_msgs/msg/Float64 "{data: 999.0}"
```

The actual value is not important. Receiving a message on `/panorama_trigger` starts the sweep.

### Publish Test GPS and Heading Data

When testing the panorama sweep without the real GPS/IMU hardware, publish fake GPS and heading data in separate terminals. The 
heading is the rover’s facing direction, measured in degrees clockwise from north, where `0° = North`, `90° = East`, `180° = South`, and `270° = West`.

Terminal 1 — publish fake GPS:

```bash
ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix "{latitude: 43.657000, longitude: -79.380000, altitude: 100.0}" -r 1
```

Terminal 2 — publish fake heading:

```bash
ros2 topic pub /heading std_msgs/msg/Float32 "{data: 90.0}" -r 1
```

### Test Panorama Sweep Without Camera

```bash
ros2 launch autonomy_vision motor.launch.py test_mode:=true
```

This tests the servo sweep without waiting for real camera frames from `/arm_cam/image/compressed`.

### Check Camera Output

Check raw camera frames:

```bash
ros2 topic hz /image_raw
```

Check compressed camera frames:

```bash
ros2 topic hz /arm_cam/image/compressed
```

If the servo moves to the first sweep angle and then stops, check whether `/arm_cam/image/compressed` is publishing. In normal mode, `panorama_stitcher` waits for a compressed image frame before moving to the next angle.

### Check YOLO Detections

```bash
ros2 topic echo /detections
```

### Check Debug Image

```bash
ros2 topic hz /debug_image
```

## Hardware Notes

### USB Serial Device

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

Check whether the USB device is detected:

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

## Camera Notes

The detection launch file uses `usb_cam` and defaults to:

```txt
/dev/back_web_cam
```

The motor/panorama launch file uses `v4l2_camera` and defaults to:

```txt
/dev/video0
```

To select a different camera device:

```bash
ros2 launch autonomy_vision motor.launch.py camera_device:=/dev/video2
```

Check available camera devices:

```bash
v4l2-ctl --list-devices
```

## Verify

List active ROS 2 nodes:

```bash
ros2 node list
```

List active ROS 2 topics:

```bash
ros2 topic list
```

Check motor command topic:

```bash
ros2 topic echo /motor_cmd
```

Check panorama trigger topic:

```bash
ros2 topic echo /panorama_trigger
```

Check raw camera topic:

```bash
ros2 topic hz /image_raw
```

Check compressed camera topic:

```bash
ros2 topic hz /arm_cam/image/compressed
```

Check detection output:

```bash
ros2 topic echo /detections
```

Check debug image publishing rate:

```bash
ros2 topic hz /debug_image
```

## Dependencies Used by Current Code

The current code uses:

```txt
rclpy
std_msgs
sensor_msgs
vision_msgs
OpenCV / cv2
NumPy
PySerial
Ultralytics YOLO
PyTorch
usb_cam
v4l2_camera
image_transport
```

## Notes

* `motor_controller` uses the `serial_port` and `baud_rate` parameters from `motor.launch.py`.
* `motor.launch.py` defaults to `/dev/video0` for the camera device, but this can be changed with `camera_device:=`.
* `detection.launch.py` defaults to `/dev/back_web_cam`.
* The panorama trigger is published on `/panorama_trigger`.
* The panorama sweep publishes servo commands to `/motor_cmd`.
* The final panorama is saved as `panorama.jpg`.
* In normal mode, `panorama_stitcher` requires compressed camera frames on `/arm_cam/image/compressed`.
* In test mode, `panorama_stitcher` simulates image captures and tests motor movement without camera frames.

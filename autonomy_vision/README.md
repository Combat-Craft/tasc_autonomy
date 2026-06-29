# autonomy_vision

ROS 2 vision package for the rover. This package currently includes webcam YOLO detection, servo motor control through Arduino serial, and panorama sweeping/stitching using a camera mounted on a servo.

## Package Structure

```txt
autonomy_vision/
├── autonomy_vision/
|   ├── old_code/
|   |   ├── new_pano_stitcher.py
|   |   └── h265_camera_streamer.py  
│   ├── __init__.py
│   ├── morse_code_detector.py
│   ├── motor_controller.py
│   ├── motor_input.py
│   ├── multi_camera_streamer.py
│   ├── panorama_stitcher.py
│   ├── webcam_detection2D.py
│   ├── yolo_depth_v1.py
│   └── yolo_pc.py
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
|   ├── hello_morse.mp4
|   └── panorama_motor/
│       └── panorama_motor.ino
|
├── resource/
│   └── autonomy_vision   
|
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
|
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

Launches:

1. `motor_controller`
2. `motor_input`
3. `panorama_stitcher`
4. `v4l2_camera_node`
5. `image_transport` republisher
6. `fake_gps`
7. simulated `imu_node`

```bash
ros2 launch autonomy_vision motor.launch.py
```

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

### `gemini2_edited2.launch.py`

### `morse_detector.launch.py`

### `orbbec.launch.py`

## Nodes

### `Motor_controller.py`

Controls the Arduino Nano over serial.

The node subscribes to:

```txt
/motor_cmd
```

It receives target servo angles in the range:

```txt
-135 degrees to +135 degrees
```

The Arduino Servo library expects values from `0` to `180`, so the ROS 2 command is mapped like this:

```txt
-135 degrees → 0
0 degrees    → 90
+135 degrees → 180
```

The serial port is currently hardcoded as:

```txt
/dev/ttyUSB0
```

The baud rate is:

```txt
115200
```

### `motor_input.py`

Allows the user to type motor commands from the terminal.

It publishes angle commands to:

```txt
/motor_cmd
```

It also publishes panorama trigger commands to:

```txt
/panorama_trigger
```

Input options:

```txt
-135 to 135  → move servo to that angle
p            → trigger panorama sweep
e            → exit
```

Example prompt:

```txt
Enter angle (-135 to 135 | 'p' for panorama | 'e' to exit):
```

### `panorama_stitcher.py`

Controls a servo sweep, captures compressed camera images, and stitches them into a panorama.

The sweep angles are currently:

```python
np.linspace(-135, 135, 10)
```

This means the panorama sweep captures 10 positions from `-135` degrees to `+135` degrees.

The node publishes servo commands to:

```txt
/motor_cmd
```

It subscribes to compressed images from:

```txt
/arm_cam/image/compressed
```

It also subscribes to:

```txt
/panorama_trigger
/gps/fix
/heading
/cardinal_compass
```

The panorama is saved as:

```txt
panorama.jpg
```

There is also a test mode inside the file:

```python
self.TEST_MODE = False
```

When test mode is enabled, the node simulates captures without requiring real camera frames.

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

| Topic                       | Type                           | Direction    | Description                                    |
| --------------------------- | ------------------------------ | ------------ | ---------------------------------------------- |
| `/motor_cmd`                | `std_msgs/Float64`             | Input/Output | Servo angle command in degrees                 |
| `/panorama_trigger`         | `std_msgs/Float64`             | Input/Output | Trigger topic for starting panorama sweep      |
| `/image_raw`                | `sensor_msgs/Image`            | Input        | Raw camera image from USB camera               |
| `/debug_image`              | `sensor_msgs/Image`            | Output       | YOLO debug image with bounding boxes           |
| `/detections`               | `vision_msgs/Detection2DArray` | Output       | YOLO object detection results                  |
| `/arm_cam/image/compressed` | `sensor_msgs/CompressedImage`  | Input        | Compressed camera image for panorama stitching |
| `/gps/fix`                  | `sensor_msgs/NavSatFix`        | Input        | GPS position for panorama metadata             |
| `/heading`                  | `std_msgs/Float32`             | Input        | Heading value for panorama metadata            |
| `/cardinal_compass`         | `std_msgs/String`              | Input        | Cardinal direction for panorama metadata       |

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

The ROS 2 `motor_controller` node maps the rover servo range of `-135` to `+135` degrees into this Arduino range.

### Arduino Pin Wiring

| Servo Wire | Connection |
| ---------- | ---------- |
| Brown      | GND        |
| Orange     | D9         |
| Red        | 5V         |

### Arduino Serial Settings

| Setting          | Value    |
| ---------------- | -------- |
| Baud rate        | `115200` |
| Servo pin        | `D9`     |
| Startup position | `90`     |

On startup, the Arduino moves the servo to the center position:

```cpp
panServo.write(90);
```

The Arduino prints:

```txt
READY
```

when it starts.

## Manual Commands

### Move Servo Manually

Publish a servo command directly:

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

### Check Camera Output

```bash
ros2 topic echo /image_raw
```

```bash
ros2 topic hz /image_raw
```

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

The Arduino Nano is currently expected to appear as:

```txt
/dev/ttyUSB0
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

If the Arduino uses a CH340 USB serial chip and `/dev/ttyUSB0` does not appear, reload the CH340 driver:

```bash
sudo modprobe ch341
```

Then unplug and reconnect the Arduino USB cable.

If `lsusb` does not show the device at all, it is likely a hardware, cable, or USB port issue.

If `lsusb` shows the CH340 device but `/dev/ttyUSB*` does not appear, it is likely a driver issue.

### Serial Permissions

A temporary permission fix is:

```bash
sudo chmod 777 /dev/ttyUSB0
```

A better long-term fix is to add the user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
```

Then log out and log back in.

## Camera Notes

The detection launch file uses `usb_cam` and defaults to:

```txt
/dev/back_web_cam
```

The motor/panorama launch file uses `v4l2_camera` and currently uses:

```txt
/dev/video0
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

Check compressed camera topic:

```bash
ros2 topic echo /arm_cam/image/compressed
```

Check detection output:

```bash
ros2 topic echo /detections
```

Check image publishing rate:

```bash
ros2 topic hz /image_raw
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
gps_tracker
autonomy_sensors
```

## Notes

* `motor_controller` currently uses a hardcoded serial port: `/dev/ttyUSB0`.
* `motor.launch.py` currently uses a hardcoded camera device: `/dev/video0`.
* `detection.launch.py` currently defaults to `/dev/back_web_cam`.
* The panorama trigger is published on `/panorama_trigger`.
* The final panorama is saved as `panorama.jpg`.
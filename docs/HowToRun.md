# General Build, Checks, and Troubleshooting
1. SSH into the Jeston. You don't need Ubuntu, you can just use VSCode on Windows.
   
2. Set up the CH34x driver for the Arduino Nano
```bash
cd ~/CH341SER
```
```bash
sudo make load
```

Now go to our package

```bash
cd ~/autonav_ws
```

3. Check all USB devices, both by their device names 
```bash
lsusb
```
   and their dev names. Make sure you count and check that you see as many devices as they should be connected!
```bash
ls -l /dev/ttyUSB*
```

4. Check static USB names work. For autonav, please check that these two appear and route to a `/dev/tty/USB#`
```bash
ls -l /dev/nano
```
```bash
ls -l /dev/autonavESP32
```
**IF STATIC NAMES DO NOT WORK**
1. Check that the USB Hub is connected to the upper left USB A port on the Jeston. Check that the USB ports on the Hub all go tot their correct spot as per the stickers. Then try again
2. Double check the udev rule files against the attribute walk
   - In Terminal 1, ``udevadm info --name=/dev/ttyUSB# --attribute-walk``, replacing # with the USB # as seen in ``ls -l /dev/ttyUSB*``.
   - In the 3rd set of attributes, look for something like ``KERNELS=”1:1.2.4:1.0”``
   - In Terminal 2, ``sudo vim /etc/udev/rules.d/99-my-cams.rules``, and press E if it complains about changes. Look at the KERNEL parameters.
   - Now, if they first set of numbers in the KERNELS parameters don't match in the attribute walk vs the udev rule file, that means either the USB Hub is not in the correct port for the UDEV rules decided to randomly change the KERNEL settings. Fix them all to match the first however so numbers.


6. Colcon Build tasc_autonomy

```bash
colcon build 
```
To rebuild one specific package
```bash
colcon build --packages-select <package>
```
To build all except a certain package
```bash
colcon build --packages-ignore <name-of-package>
```
   

# Setting up Camera Stream and GUI
### On Jetson via SSH

1. Check that all USB Cameras appear via static names and v4l2.
For the static names, make sure these appear.
The back webcam
```bash
ls -l /dev/back_web_cam 
```
The Orbbec Gemini 2
```bash
ls -l /dev/orbbec_color_cam  
```

To see all ``/dev/video# streams`` and check all 3 exist
```bash
v4l2-ctl --list devices
```
You should see 3 cameras. 1 is Tegra, 1 for the Orbbec with about 8 total paths, and 1 for the back webcam which has 3 or 4.

3. Go to camera_streams folder 
```bash
cd ~/autonav_ws/src/tasc_autonomy/tree/main/camera_streams
```
   
5. Start TCP Server for USB Camera Control. The 1st argument is the Jetson IP, and the 2nd is the Port
```bash
python3 TCPStreamerServer.py 192.168.1.7 54321
```
6. Once GUI on the controller PC has started and manually connected to the TCP/USB camera controls, check terminal that a client has connected to the server,

### On Controller PC
If using Windows, ensure you have set up correctly for the GUI. This means MSYS2, installing Pygobject via MSYS2, PySide6 via ?, and onvif-zeep via pip install.

If you are smart and use Ubuntu/Linux, set up a venv with Pygobject, PySide6 and onvif-zeep all installed via pip install. Make sure to activate the venv!

1. Go to where the *bleep* you saved ``tasc_autonomy_custom_gui``. 

2. Start up the GUI
```bash
python3 main.py
```
3. Use the control panel to connect for camera controls, for USB cams or IP cams.


### NOTE FOR IP CAMERA SETTINGS
You may need more settings / fine control, especially for Morse decoder.

Please go to your web browser on your main OS, type in:
- ``http://192.168.1.117`` for the TOP camera
- ``http://192.168.1.116`` for the ARM camera
- Credentials are Admin and blank for password. Click cancel when it asks you to change from the default.
- Go to Display > Image for more settings such as brightness, contrast, etc.

# Instructions for CIRC Tasks

## For all

### Controlling the Top IP Camera Angle
You can control the angle of the TOP IP Camera. 
- Open up Foxglove,
- connect to the Jetson,
- and open up a publisher for ``/motor_cmd``
  - You can enter degrees from ``-135`` to ``135``, where 0 is facing the center-front.

## Specifics for Snack Run
```bash
ros2 launch autonomy_bringup snack_run.launch.py
```
Nothing else from AutoNav here, but you can watch IMU and GPS on Foxglove.

## Specifics for Rovercooked
```bash
ros2 launch autonomy_bringup rover_cooked.launch.py
```
Nothing else from AutoNav here, but you can watch IMU and GPS on Foxglove.

## Specifics for Exploration
```bash
ros2 launch autonomy_bringup exploration.launch.py
```
 - panorama stuff blah blah
 - route logger stuff blag blah
   
## Specifics for Heist Mission
```bash
ros2 launch autonomy_bringup heist_mission.launch.py
```
***YOU MUST COVER THE CENTER LASER OF THE ORBBEC CAMERA!!!!*** **or else you risk the IR lasers activating and losing us points**. Use a solid piece of cardstock and electrical tape.

- morse decoder stuff blah balh

```bash
```

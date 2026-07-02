# LAN / Network
To connect to our network, you must be connected via switch with ethernet, or from comm's wireless connection. You must manually set your network (ethernet/wired or wiresless) to have an IPv4 of 192.168.1.XXX (replace with a number between 2-254. Currently, the IPs taken are 7,20,23, and 117).

# udev

YOU MUST ACTIATE THE RULES BEFORE USING THE STATIC NAMES
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger

```

Camera udev rules are stored at ``` /etc/udev/rules.d/99-my-cams.rules```
NOTE: You need to do sudo vim to edit it.

Currently only has 3 camera rules

```
SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTR{name}=="USB Webcam gadget: UVC HD Camer", SYMLINK+="mina_cam"

SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTR{name}=="USB Composite Device: USB Camer", SYMLINK+="back_web_cam"

SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTR{name}=="Orbbec(R) Gemini(TM): Orbbec Ge", SYMLINK+="orbbec_color_cam"
```
# Cameras

## Note on SRT on Gstreamer
Port on both caller and listener must be the same. 
Either the caller or listen IP must be default/0.0.0.0, and the other is the IP of the other. 
To ensure the stream is being sent to our bay station PC, we will set our srtsink uri to the client ip and our srtsrc uri to be default. 
We also manually set caller and listener, despite being default, for clarity and to reduce any risk of borking.

## IP Cam
**Brand:** Revotech i706-P-Audio-FHW

**Resolutions:** profile0: 2304 x 1296, 1920 x 1080 | profile1: 720 x 480, 640 x 360, 352 x 288

**Frame Rate:** 1-25 FPS

**Bitrate Control:** VBR or CBR (Can set Bitrate in kbps)

**Power:** 12V DC (Barrel Connector) / 48V POE (Needs 802.3af Standard POE Switch)

**Connection Type:** Ethernet/PoE @ 10M/100M mbps Ethernet (RJ-45) 

**Output Format:** H.265/H.264 


Has no udev name, no need as not connected to Jetson directly

### How to Connect

Connect to the LAN, and ensure your IPv4 settings are on 192.168.1.XXX/24.

The camera's IP is 192.168.1.117/24.

The access urls are:
- rtspt://admin:@192.168.1.117:8554/profile0 
- rtspt://admin:@192.168.1.117:8554/profile1 <-- WE USE THIS ONE FOR LOWER RESOLUTIONS

### Gstreamer Pipeline (Tested via CLI)
**Jetson/Server**
``` N/A ``` Camera is connected via switch, does not need to route through Jetson

**Bay Station/Client**
```bash
gst-launch-1.0 rtspsrc location=rtspt://admin:@192.168.1.117:8554/profile1 latency=0 ! rtph265depay ! h265parse !  avdec_h265 ! autovideosink sync=false
```
NOTE: You will have to change the sink when you add it to PyQT. Probably appsink? 



##  Mina's Webcam
**Static name:** mina_cam

**Output Formats:**
```bash
'MJPG' (Motion-JPEG, compressed)
      Size: Discrete 640x480
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.067s (15.000 fps)
              Interval: Discrete 0.100s (10.000 fps)
      Size: Discrete 1280x720
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.067s (15.000 fps)
              Interval: Discrete 0.100s (10.000 fps)
      Size: Discrete 1920x1080
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.067s (15.000 fps)
              Interval: Discrete 0.100s (10.000 fps)
'YUYV' (YUYV 4:2:2)
      Size: Discrete 640x480
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.067s (15.000 fps)
              Interval: Discrete 0.100s (10.000 fps)
      Size: Discrete 1280x720
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.067s (15.000 fps)
              Interval: Discrete 0.100s (10.000 fps)
'H264' (H.264, compressed)
      Size: Discrete 640x480
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.067s (15.000 fps)
              Interval: Discrete 0.100s (10.000 fps)
      Size: Discrete 1280x720
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.067s (15.000 fps)
              Interval: Discrete 0.100s (10.000 fps)
      Size: Discrete 1920x1080
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.067s (15.000 fps)
              Interval: Discrete 0.100s (10.000 fps)

```

### Gstreamer Pipeline (Tested via CLI)

**REMEMBER TO REPLACE THE IP WITH YOUR PC'S**

**Jetson/Server**
```bash
gst-launch-1.0 v4l2src device=/dev/mina_cam ! "video/x-h264,width=640,height=480,framerate=30/1" ! srtsink uri="srt://192.168.1.XXX:7090?mode=caller" sync=false
```
```bash
gst-launch-1.0 v4l2src device=/dev/mina_cam ! "video/x-raw, format=YUY2, width=640, height=360, framerate=30/1" ! videoconvert ! x264enc bitrate=500 tune=zerolatency speed-preset=ultrafast  ! srtsink uri="srt://192.168.1.XXX:7090?mode=caller" sync=false
```

**Bay Station/Client**
```bash
gst-launch-1.0 srtsrc uri="srt://:7090?mode=listener" ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```


##  Original Webcam
**Static name:** back_web_cam

**Output Formats:**
```bash
'MJPG' (Motion-JPEG, compressed)
      Size: Discrete 1920x1080
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.040s (25.000 fps)
      Size: Discrete 1280x720
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.040s (25.000 fps)
      Size: Discrete 640x480
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.040s (25.000 fps)
      Size: Discrete 640x360
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.040s (25.000 fps)
      Size: Discrete 352x288
              Interval: Discrete 0.033s (30.000 fps)
              Interval: Discrete 0.040s (25.000 fps)
'YUYV' (YUYV 4:2:2)
      Size: Discrete 640x480
              Interval: Discrete 0.033s (30.000 fps)
      Size: Discrete 640x360
              Interval: Discrete 0.033s (30.000 fps)
      Size: Discrete 352x288
              Interval: Discrete 0.033s (30.000 fps)
      Size: Discrete 320x240
              Interval: Discrete 0.033s (30.000 fps)
```

### Gstreamer Pipeline (Tested via CLI)

**REMEMBER TO REPLACE THE IP WITH YOUR PC'S** : 192.168.1.XXX with whatever one you have already set

**Jetson/Server**
Preferred since better quality:
```bash
gst-launch-1.0 v4l2src device=/dev/back_web_cam ! "video/x-raw, format=YUY2, width=640, height=360, framerate=30/1" ! videoconvert ! x264enc bitrate=500 tune=zerolatency speed-preset=ultrafast ! srtsink uri="srt://192.168.1.XXX:7091?mode=caller" sync=false
```
If YUY2 aint working for some reason:
```bash
gst-launch-1.0 v4l2src device=/dev/back_web_cam ! image/jpeg, width=640, height=360, framerate=30/1 ! jpegdec ! videoconvert ! x264enc pass=pass1 bitrate=500 tune=zerolatency speed-preset=ultrafast ! srtsink uri="srt://192.168.1.XXX:7091?mode=caller" sync=false
```

OR


**Bay Station/Client**
```bash
gst-launch-1.0 srtsrc uri="srt://:7091?mode=listener" ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

##  Default Orbbec Gemini2 Connection via V2L4
**Static name:** orbbec_color_cam

**Output Formats:**

For SDK Code params: https://github.com/orbbec/OrbbecSDK_v2/blob/6a35953bf284f91607dc764a68271230a482ec86/include/libobsensor/h/ObTypes.h#L204

For SDK Code Guide: https://orbbec.github.io/docs/OrbbecSDKv2_API_User_Guide/source/3_Application_Guide/ApplicationGuide.html

Device Properties/constant names
```
00. OB_PROP_LDP_BOOL(2), permission=R/W, range=Bool value(min:0, max:1, step:1)
01. OB_PROP_LASER_BOOL(3), permission=R/W, range=Bool value(min:0, max:1, step:1)
02. OB_PROP_DEPTH_MIRROR_BOOL(14), permission=R/W, range=Bool value(min:0, max:1, step:1)
03. OB_PROP_DEPTH_FLIP_BOOL(15), permission=R/W, range=Bool value(min:0, max:1, step:1)
04. OB_PROP_DEPTH_HOLEFILTER_BOOL(17), permission=R/W, range=Bool value(min:0, max:1, step:1)
05. OB_PROP_IR_MIRROR_BOOL(18), permission=R/W, range=Bool value(min:0, max:1, step:1)
06. OB_PROP_IR_FLIP_BOOL(19), permission=R/W, range=Bool value(min:0, max:1, step:1)
07. OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL(24), permission=R/W, range=Bool value(min:0, max:1, step:1)
08. OB_PROP_LDP_STATUS_BOOL(32), permission=R/_, range=Bool value(min:0, max:1, step:1)
09. OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_MAX_DIFF_INT(40), permission=R/W, range=Int value(min:1, max:10000, step:1)
10. OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_MAX_SPECKLE_SIZE_INT(41), permission=R/W, range=Int value(min:1, max:1000, step:1)
11. OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL(42), permission=R/W, range=Bool value(min:0, max:1, step:1)
12. OB_PROP_DEPTH_ALIGN_HARDWARE_MODE_INT(63), permission=R/W, range=Int value(min:0, max:4, step:1)
13. OB_PROP_DEPTH_PRECISION_LEVEL_INT(75), permission=R/W, range=Int value(min:6, max:0, step:1)
14. OB_PROP_COLOR_MIRROR_BOOL(81), permission=R/W, range=Bool value(min:0, max:1, step:1)
15. OB_PROP_COLOR_FLIP_BOOL(82), permission=R/W, range=Bool value(min:0, max:1, step:1)
16. OB_PROP_DISPARITY_TO_DEPTH_BOOL(85), permission=R/W, range=Bool value(min:0, max:1, step:1)
17. OB_PROP_WATCHDOG_BOOL(87), permission=R/W, range=Bool value(min:0, max:1, step:1)
18. OB_PROP_EXTERNAL_SIGNAL_RESET_BOOL(88), permission=R/W, range=Bool value(min:0, max:1, step:1)
19. OB_PROP_HEARTBEAT_BOOL(89), permission=R/W, range=Bool value(min:0, max:1, step:1)
20. OB_PROP_LASER_POWER_LEVEL_CONTROL_INT(99), permission=R/W, range=Int value(min:0, max:5, step:1)
21. OB_PROP_LDP_MEASURE_DISTANCE_INT(100), permission=R/_, range=Int value(min:0, max:3000, step:1)
22. OB_PROP_TIMER_RESET_SIGNAL_BOOL(104), permission=_/W, range=Bool value(min:0, max:1, step:1)
23. OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL(105), permission=R/W, range=Bool value(min:0, max:1, step:1)
24. OB_PROP_TIMER_RESET_DELAY_US_INT(106), permission=R/W, range=Int value(min:0, max:10000000, step:1)
25. OB_PROP_CAPTURE_IMAGE_SIGNAL_BOOL(107), permission=_/W, range=Bool value(min:0, max:1, step:1)
26. OB_PROP_IR_RIGHT_MIRROR_BOOL(112), permission=R/W, range=Bool value(min:0, max:1, step:1)
27. OB_PROP_CAPTURE_IMAGE_FRAME_NUMBER_INT(113), permission=R/W, range=Int value(min:1, max:254, step:1)
28. OB_PROP_IR_RIGHT_FLIP_BOOL(114), permission=R/W, range=Bool value(min:0, max:1, step:1)
29. OB_PROP_COLOR_ROTATE_INT(115), permission=R/W, range=Int value(min:0, max:270, step:90)
30. OB_PROP_IR_ROTATE_INT(116), permission=R/W, range=Int value(min:0, max:270, step:90)
31. OB_PROP_IR_RIGHT_ROTATE_INT(117), permission=R/W, range=Int value(min:0, max:0, step:0)
32. OB_PROP_DEPTH_ROTATE_INT(118), permission=R/W, range=Int value(min:0, max:270, step:90)
33. OB_PROP_LASER_POWER_ACTUAL_LEVEL_INT(119), permission=R/_, range=Int value(min:0, max:5, step:1)
34. OB_PROP_SYNC_SIGNAL_TRIGGER_OUT_BOOL(130), permission=R/W, range=Bool value(min:0, max:1, step:1)
35. OB_PROP_DEVICE_USB2_REPEAT_IDENTIFY_BOOL(141), permission=R/W, range=Bool value(min:0, max:1, step:1)
36. OB_PROP_COLOR_AUTO_EXPOSURE_BOOL(2000), permission=R/W, range=Bool value(min:0, max:1, step:1)
37. OB_PROP_COLOR_EXPOSURE_INT(2001), permission=R/W, range=Int value(min:0, max:33000, step:1)
38. OB_PROP_COLOR_GAIN_INT(2002), permission=R/W, range=Int value(min:1, max:255, step:2)
39. OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL(2003), permission=R/W, range=Bool value(min:0, max:1, step:1)
40. OB_PROP_COLOR_WHITE_BALANCE_INT(2004), permission=R/W, range=Int value(min:2800, max:6800, step:10)
41. OB_PROP_COLOR_BRIGHTNESS_INT(2005), permission=R/W, range=Int value(min:1, max:255, step:1)
42. OB_PROP_COLOR_SHARPNESS_INT(2006), permission=R/W, range=Int value(min:0, max:255, step:1)
43. OB_PROP_COLOR_SATURATION_INT(2008), permission=R/W, range=Int value(min:0, max:255, step:1)
44. OB_PROP_COLOR_CONTRAST_INT(2009), permission=R/W, range=Int value(min:0, max:255, step:1)
45. OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT(2015), permission=R/W, range=Int value(min:0, max:2, step:1)
46. OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL(2016), permission=R/W, range=Bool value(min:0, max:1, step:1)
47. OB_PROP_DEPTH_EXPOSURE_INT(2017), permission=R/W, range=Int value(min:200, max:5000, step:1)
48. OB_PROP_DEPTH_GAIN_INT(2018), permission=R/W, range=Int value(min:1000, max:15000, step:100)
49. OB_PROP_GYRO_ODR_INT(2021), permission=R/W, range=Int value(min:1, max:15, step:1)
50. OB_PROP_ACCEL_ODR_INT(2022), permission=R/W, range=Int value(min:1, max:15, step:1)
51. OB_PROP_IR_AUTO_EXPOSURE_BOOL(2025), permission=R/W, range=Bool value(min:0, max:1, step:1)
52. OB_PROP_IR_EXPOSURE_INT(2026), permission=R/W, range=Int value(min:200, max:5000, step:1)
53. OB_PROP_IR_GAIN_INT(2027), permission=R/W, range=Int value(min:1000, max:15000, step:100)
54. OB_PROP_IR_CHANNEL_DATA_SOURCE_INT(2028), permission=R/W, range=Int value(min:0, max:1, step:1)
55. OB_PROP_DEPTH_RM_FILTER_BOOL(2029), permission=R/W, range=Bool value(min:0, max:1, step:1)
56. OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL(3004), permission=R/W, range=Bool value(min:0, max:1, step:1)
57. OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL(3009), permission=R/W, range=Bool value(min:0, max:1, step:1)
58. OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL(3010), permission=R/W, range=Bool value(min:0, max:1, step:1)

```

```bash

'YUYV' (YUYV 4:2:2)
    Size: Discrete 640x360
            Interval: Discrete 0.017s (60.000 fps)
            Interval: Discrete 0.033s (30.000 fps)
            Interval: Discrete 0.067s (15.000 fps)
            Interval: Discrete 0.100s (10.000 fps)
            Interval: Discrete 0.200s (5.000 fps)
    Size: Discrete 640x480
            Interval: Discrete 0.017s (60.000 fps)
            Interval: Discrete 0.033s (30.000 fps)
            Interval: Discrete 0.067s (15.000 fps)
            Interval: Discrete 0.100s (10.000 fps)
            Interval: Discrete 0.200s (5.000 fps)
    Size: Discrete 1280x720
            Interval: Discrete 0.017s (60.000 fps)
            Interval: Discrete 0.033s (30.000 fps)
            Interval: Discrete 0.067s (15.000 fps)
            Interval: Discrete 0.100s (10.000 fps)
            Interval: Discrete 0.200s (5.000 fps)
    Size: Discrete 1920x1080
            Interval: Discrete 0.017s (60.000 fps)
            Interval: Discrete 0.033s (30.000 fps)
            Interval: Discrete 0.067s (15.000 fps)
            Interval: Discrete 0.100s (10.000 fps)
            Interval: Discrete 0.200s (5.000 fps)
'MJPG' (Motion-JPEG, compressed)
    Size: Discrete 640x360
            Interval: Discrete 0.017s (60.000 fps)
            Interval: Discrete 0.033s (30.000 fps)
            Interval: Discrete 0.067s (15.000 fps)
            Interval: Discrete 0.100s (10.000 fps)
            Interval: Discrete 0.200s (5.000 fps)
    Size: Discrete 640x480
            Interval: Discrete 0.017s (60.000 fps)
            Interval: Discrete 0.033s (30.000 fps)
            Interval: Discrete 0.067s (15.000 fps)
            Interval: Discrete 0.100s (10.000 fps)
            Interval: Discrete 0.200s (5.000 fps)
    Size: Discrete 1280x720
            Interval: Discrete 0.017s (60.000 fps)
            Interval: Discrete 0.033s (30.000 fps)
            Interval: Discrete 0.067s (15.000 fps)
            Interval: Discrete 0.100s (10.000 fps)
            Interval: Discrete 0.200s (5.000 fps)
    Size: Discrete 1920x1080
            Interval: Discrete 0.017s (60.000 fps)
            Interval: Discrete 0.033s (30.000 fps)
            Interval: Discrete 0.067s (15.000 fps)
            Interval: Discrete 0.100s (10.000 fps)
            Interval: Discrete 0.200s (5.000 fps)
```

### Gstreamer Pipeline (Tested via CLI)

The server code will be replcaed by the C++ ORbbec SDK code made in the orbbec_gemini_2 package

**REMEMBER TO REPLACE THE IP WITH YOUR PC'S** : 192.168.1.XXX with whatever one you have already set

**Jetson/Server**

Preferred since better quality:
```bash
gst-launch-1.0 v4l2src device=/dev/orbbec_color_cam ! "video/x-raw, format=YUY2, width=1280, height=720, framerate=30/1" ! videoconvert ! x265enc bitrate=500 tune=zerolatency speed-preset=ultrafast ! srtsink uri="srt://192.168.1.XXX:7092?mode=caller" sync=false 
```

If YUY2 aint working for some reason:
```bash
gst-launch-1.0 v4l2src device=/dev/orbbec_color_cam ! image/jpeg, width=1280, height=720, framerate=30/1 ! jpegdec ! videoconvert ! x264enc pass=pass1 bitrate=500 tune=zerolatency speed-preset=ultrafast ! srtsink uri="srt://192.168.1.XXX:7092?mode=caller" sync=false 
```

**Bay Station/Client**
```bash
gst-launch-1.0 srtsrc uri="srt://:7092?mode=listener" ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false 
```






## 

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

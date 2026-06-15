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
# Camera List

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
Key note on SRT on Gstreamer: Port on both caller and listener must be the same. 
Either the caller or listen IP must be localhost/default, and the other is the IP of the other. 
To ensure the stream is being sent to our bay station PC, we will set our srtsink uri to the client ip and our srtsrc uri to be localhost. 
We also manually set caller and listen, despite being default, for clarity and to reduce any risk of borking.

**REMEMBER TO REPLACE THE IP WITH YOUR PC'S**

**Jetson/Server**
```bash
gst-launch-1.0 v4l2src device=/dev/mina_cam ! "video/x-h264,width=640,height=480,framerate=30/1" ! h264parse config-interval=-1 ! srtsink uri="srt://192.168.1.XXX:7090?mode=caller" sync=false
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
Key note on SRT: The IPs must match the IP of whatever PC it is on, NOT 'aimed' to where you want it to go.

**Jetson/Server**
```bash
gst-launch-1.0 -v v4l2src device=/dev/back_web_cam ! "video/x-h264,width=640,height=480,framerate=30/1" ! h264parse config-interval=-1 ! srtsink uri="srt://192.168.1.20:7090?mode=caller"
```

**Bay Station/Client**
```bash
gst-launch-1.0 -v srtsrc uri="srt://0.0.0.0:7090?mode=listener" ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

##  Mina's Webcam
**Static name:**
**Output Formats:**
```bash
```

### Gstreamer Pipeline (Tested via CLI)
**Jetson/Server**
```bash
```

**Bay Station/Client**
```bash
```

##  Mina's Webcam
**Static name:**
**Output Formats:**
```bash
```

### Gstreamer Pipeline (Tested via CLI)
**Jetson/Server**

**Bay Station/Client**
```bash
```




## 

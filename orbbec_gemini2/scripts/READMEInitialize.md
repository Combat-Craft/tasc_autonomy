# Camera SRT Streaming Script

## Overview
The following script, `initialize.py`, streams a video from a V4L2 camera (e.g. Mina Cam, Back Web Cam, etc.) to a remote PC using the SRT protocol. It constructs and launches a GStreamer pipeline that can:

 1. Captures a raw or compressed video feed from the V4L2 device
 2. Streams it out via srtsink to a target IP/port

The script supports interactive configuration at runtime, and supports a "use default settings" mode.

## Requirements

 - Python 3
 - PyGObject (gi)
 - GStreamer 1.0 with the plugins: 
	 -   `gst-plugins-good` (v4l2src, jpegdec, videoconvert)
	 - `gst-plugins-ugly`or (installs x264enc)
	 - `gst-plugins-bad` (installs srtsink)
- A V4L2 compatible camera, which is available under `/dev/your_camera_name` 
	 
To check, run:
	 `v4l2-ctl --list-devices` 
in terminal to determine if the camera appears/is connected.

## Camera Configuration
At the top of the script there is a list of entries, where each entry describes a camera to stream:

```python
CAMERAS = [
    {
        "name": "back_web_cam",
        "source": "v4l2src",
        "source_uri": "/dev/back_web_cam",
        "port": 7091,
        "caps": {
            "format": "YUY2",
            "width": 640,
            "height": 480,
            "framerate": 30,
        }
    }
]
```
Each field represents:
 - `name`: A Label to describe the camera.
 - `source`: GStreamer source element.
 - `source_uri`: Device path (e.g. `/dev/back_web_cam`)
 - `port`: The UDP port the SRT stream will be sent to.
 - `caps`: Includes format, width, height, framerate, which describe the pixel format, the resolutions of the camera, and the framerate. **Note, the script supports MJPG and YUY2 formats, for now.**

To stream multiple camera, add another dictionary entry to the `CAMERAS` list, each entry get their own pipeline and port.

The script also features global settings:

	 BITRATE =  500  
	 LATENCY =  200  

## How to Run the Script
In the VSCode terminal, in the same directory as the python script, run:

	python3 initialize.py 

or (for windows)
	
	python initialize.py

Once it is ran, you will be prompted with:

	Use Default Settings? [y/n]: 

- `y`: streams using the values in `CAMERAS`
- `n`: iterates through each camera in the `CAMERAS` list and lets you override the entries interactively. Press enter on any prompt to keep the existing/default value.

Then it will prompt:

	Enter PC IP Suffix (192.168.1.XXX):	

Enter the last octet of the receiving PC's IP (assuming you have configurated the IP in network manager). For instance, 41 streams to `192.168.1.41` subnet.

The script will then print the GStreamer pipeline string for each camera and start streaming. Press CTRL+C to stop the pipelines.

## Networking
The script runs `srtsink` configured with `mode=caller`, meaning it connects out to the receiving PC. So, importantly, the receiving PC has to run an **SRT listener** to accept the caller connection, for example:

	gst-launch-1.0 srtsrc uri="srt://:7091?mode=listener" ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
for port 7091.




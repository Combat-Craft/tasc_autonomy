# Camera SRT Streaming Script

## Overview
The following script, `initializev2.py`, is a server that receives a TCP packet from a client that contains the camera name, resolution, fps, and bitrate. Once the data is received, the script constructs the SRT pipeline for that camera.

 1. Receives data necessary for cameras
 2. Streams it out via srtsink to a target IP/port

The script supports interactive configuration at runtime, and supports a "use default settings" mode.

**IMPORTANT NOTE: BACK CAM MUST BE RAN BEFORE MINA CAM, OTHERWISE BACK CAM DOES NOT RUN** 

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
CAMERAS = {

    "back_web_cam": {
        "source": "v4l2src",
        "source_uri": "/dev/back_web_cam",
        "port": 7091,
        "caps": {
            "format": "YUY2",
            "width": 640,
            "height": 480,
            "framerate": 30,
            "bitrate": 500,
        },
    },
}
```
Each field represents:
 - `back_web_cam`: This is an example, but that is the name of the camera.
 - `source`: GStreamer source element.
 - `source_uri`: Device path (e.g. `/dev/back_web_cam`)
 - `port`: The UDP port the SRT stream will be sent to.
 - `caps`: Includes format, width, height, framerate, and bitrate, which describe the pixel format, the resolutions, the framerate, and the bitrate of the camera. **Note, the script supports MJPG and YUY2 formats, for now.**

To stream multiple camera, add another dictionary entry to the `CAMERAS` list, each entry get their own pipeline and port.

The script also features global settings:

	 LATENCY =  200  

The script also supports aliases:

```python
CAMERA_NAMES = {
    "back": "back_web_cam",
    "mina": "mina_cam",
    # "orbbec": "orbbec_cam",
}
```
These aliases are used for the commands in the client-side.


## How to Run the Script
In the VSCode terminal, in the same directory as the python script, run:

	python initializev2.py 

Once it is ran, it waits for a connection request.

Then, locally on your computer, run the client. There is a test client within this directory, that can be used for testing. Command to run client (on powershell or local terminal):

	python client.py

**NOTE: The ip for test_client.py is set to 192.168.1.7 on line 41 (Jetson IP) to run a different ip:**

	python client.py --host 192.168.1.XXX --port XXXX

Also, it does not have to be 192.168.1.XXX, it can be anything you choose.

Once connection handshake succeeds, you are able to play, kill, or pause the camera you desire. For example:

	play back

Will use the default settings of the back_web_cam and construct the pipeline. To use custom settings, the format is (example):

	play back 640x480 30 500

 - `back`: Camera alias.
 - `540x480`: Resolution.
 - `30`: The FPS.
 - `500`: Bitrate.

Refer to the camera_info documentation for valid caps.

Once the play command is ran, the listeners should be ran in another local terminal, and should be ran seperately for each camera.

The script also supports:

    pause back
	kill back

Where pause freezes the camera frame, and kill closes the camera.

The script also has support for mina_cam (mina).


## Networking
The script runs `srtsink` configured with `mode=caller`, meaning it connects out to the receiving PC. So, importantly, the receiving PC has to run an **SRT listener** to accept the caller connection, for example:

	gst-launch-1.0 srtsrc uri="srt://:7091?mode=listener" ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
for port 7091.
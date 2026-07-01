#!/usr/bin/env python3

import time
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

#DEFAULT SETTINGS

BITRATE = 500
LATENCY = 200

#List of dictionary entries for each camera.
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

#This function allows user to configure the settings of each camera, given that they don't want to use default settings.
def camera_configure(camera):

    print(f"\nConfiguring {camera['name']}")

    camera["source"] = (
        input(f"Source [{camera['source']}]: ").strip() or camera["source"]
    )

    camera["port"] = (
        input(f"Port [{camera['port']}]: ").strip() or camera["port"]
    )

    caps = camera["caps"]

    caps["format"] = (
        input(f"Format [{caps['format']}]: ").strip()
        or caps["format"]
    )

    caps["width"] = int(
        input(f"Width [{caps['width']}]: ").strip()
        or caps["width"]
    )

    caps["height"] = int(
        input(f"Height [{caps['height']}]: ").strip()
        or caps["height"]
    )

    caps["framerate"] = int(
        input(f"Framerate [{caps['framerate']}]: ").strip()
        or caps["framerate"]
    )

#Prompt user to determine if they want to use default settings for cameras.
use_defaults = input("Use Default Settings? [y/n]: ").strip().lower()

if use_defaults == "n":
    for camera in CAMERAS:
        camera_configure(camera)


#IP Configuration for SRTSink
srt = int(input("Enter PC IP Suffix (192.168.1.XXX): "))

client_ip = f"192.168.1.{srt}"

print()
print(f"Streaming to {client_ip}")
print()

#Setup Gstreamer
Gst.init(None)

pipelines = []

#Build Pipelines

def build_pipeline(camera):
    
    #The source string for cameras
    source = (
        f"{camera['source']} "
        f"device={camera['source_uri']}"
    )

    #The ports of the cameras
    port = camera["port"]
    
    #Configures the sink
    sink = (
        f'srtsink '
        f'uri="srt://{client_ip}:{port}?mode=caller" '
        f'latency={LATENCY} '
        f'sync=false'
    )

    #Caps and format of the cameras.
    caps = camera["caps"]
    format = caps["format"].upper()

    #If the format "MJPG" is chosen, then a different caps string, which follows the MJPG format, is constructed.
    if format in ("MJPG", "MJPEG"):
        caps_str = (
            f"image/jpeg,"
            f"width={caps['width']},"
            f"height={caps['height']},"
            f"framerate={caps['framerate']}/1"
        )
        decode_str = "! jpegdec "
    
   # Uses default cap string
    else:
        caps_str = (
            f"video/x-raw,"
            f"format={caps['format']},"
            f"width={caps['width']},"
            f"height={caps['height']},"
            f"framerate={caps['framerate']}/1"
        )
        decode_str = ""
    #Constructs the pipeline string for the command to run
    pipeline_str = pipeline_str = f"""
    {source}
    ! {caps_str}
    {decode_str}
    ! videoconvert
    ! x264enc
        pass=pass1
        bitrate={BITRATE}
        tune=zerolatency
        speed-preset=ultrafast
    ! {sink}
"""
    
    print()
    print(camera['name'])
    print(pipeline_str)
    print()

    return Gst.parse_launch(pipeline_str)

#Launch pipelines
    
for camera in CAMERAS:
    pipeline = build_pipeline(camera)

    pipeline.set_state(Gst.State.PLAYING)
    pipelines.append(pipeline)

    print(f"Started {camera['name']}")

print()
print("Now streaming")
print("Press Ctrl+C to stop")
print()
    
try:

    while True:
        time.sleep(1)

except KeyboardInterrupt:

    print("\nStopping pipelines...")

finally:

    for pipeline in pipelines:
        pipeline.set_state(Gst.State.NULL)
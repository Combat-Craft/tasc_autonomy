#!/usr/bin/env python3

import time
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

#DEFAULT SETTINGS
WIDTH = 640
HEIGHT = 480
FRAMERATE = 30

FORMAT = "YUY2"

BITRATE = 500
LATENCY = 200

CAMERAS = [
    {
        "name": "mina_cam",
        "device": "/dev/mina_cam",
        "port": 7090,
    },

    {
        "name": "back_web_cam",
        "device": "/dev/back_web_cam",
        "port": 7091,
    }
]

#FILTER (CAPS) CONFIGURATION
use_defaults = input("Use Default Settings? [y/n]: ").strip().lower()

if use_defaults == "n":
    WIDTH = int(input(f"Width [{WIDTH}] ") or WIDTH)
    HEIGHT = int(input(f"Height [{HEIGHT}] ") or HEIGHT)
    FRAMERATE = int(input(f"Framerate [{FRAMERATE}]: ") or FRAMERATE)
    BITRATE = int(input(f"Bitrate [{BITRATE}]: ") or BITRATE)


#SINK SELECTION
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
    
    source = f"v4l2src device={camera['device']}"

    port = camera["port"]
    sink = (
        f'srtsink '
        f'uri="srt://{client_ip}:{port}?mode=caller" '
        f'latency={LATENCY} '
        f'sync=false'
    )

    pipeline_str = pipeline_str = f"""
    {source}
    ! video/x-raw,
        format={FORMAT},
        width={WIDTH},
        height={HEIGHT},
        framerate={FRAMERATE}/1
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



#!/usr/bin/env python3
#Modified

import time
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

#DEFAULT SETTINGS

BITRATE = 500
LATENCY = 200

#List of dictionary entries for each camera. Check if the default settings match the camera's capabilities.
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
    },

    {
        "name": "mina_cam",
        "source": "v4l2src",
        "source_uri": "/dev/mina_cam",
        "port": 7090,
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
    name = camera["name"]

    pipeline = Gst.Pipeline.new(name)
   
    caps_c = camera["caps"]
    format = caps_c["format"].upper()
    is_MJPG = format in ("MJPG", "MJPEG")

    source = Gst.ElementFactory.make(camera["source"], f"{name}-source")
    source.set_property("device", camera["source_uri"])

    capsfilter = Gst.ElementFactory.make("capsfilter", f"{name}-caps")

    if is_MJPG == True:
        caps = Gst.Caps.from_string(
            f"image/jpeg,"
            f"width={caps_c['width']},"
            f"height={caps_c['height']},"
            f"framerate={caps_c['framerate']}/1"
        )
    else:
        caps = Gst.Caps.from_string(
            f"video/x-raw,"
            f"format={caps_c['format']},"
            f"width={caps_c['width']},"
            f"height={caps_c['height']},"
            f"framerate={caps_c['framerate']}/1"
        )
    capsfilter.set_property("caps", caps)

    if is_MJPG:
        jpegdec = Gst.ElementFactory.make("jpegdec", f"{name}-jpegdec")
    else:
        jpegdec = None
    
    convert = Gst.ElementFactory.make("videoconvert", f"{name}-convert")

    encoder = Gst.ElementFactory.make("x264enc", f"{name}-encoder")
    encoder.set_property("bitrate", BITRATE)
    encoder.set_property("tune", "zerolatency")
    encoder.set_property("speed-preset", "ultrafast")

    sink = Gst.ElementFactory.make("srtsink", f"{name}-srtsink")
    sink.set_property("uri", f"srt://{client_ip}:{camera['port']}?mode=caller")
    sink.set_property("latency", LATENCY)
    sink.set_property("sync", False)

    pipeline.add(source)
    pipeline.add(capsfilter)
    if jpegdec is not None:
        pipeline.add(jpegdec)
    pipeline.add(convert)
    pipeline.add(encoder)
    pipeline.add(sink)

    source.link(capsfilter)
    if jpegdec is not None:
        capsfilter.link(jpegdec)
        jpegdec.link(convert)
    else:
        capsfilter.link(convert)
    convert.link(encoder)
    encoder.link(sink)

   

    print()
    print(name)
    print(
        f"  {camera['source']} device={camera['source_uri']} "
        f"[{format} {caps_c['width']}x{caps_c['height']}@{caps_c['framerate']}] "
        f"-> srt://{client_ip}:{camera['port']}"
    )
    print()

    return pipeline


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
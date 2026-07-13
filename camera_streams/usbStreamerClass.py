#!/usr/bin/env python3


import sys
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GLib', '2.0')
from gi.repository import Gst, GLib

#DEFAULT SETTINGS

#List of dictionary entries for each camera. Check if the default settings match the camera's capabilities.
CAMERAS_DEFAULTS = {
    # test usb cameras
    "webcam": {
         "source": "v4l2src",
         "source_uri": "/dev/video0", 
         "port": 7090,
         "caps": {
             "format": "YUY2",
             "width": 640,
             "height": 480,
             "framerate": 30,
         },
         "bitrate": 500
     },
   
    "back": {
        "source": "v4l2src",
        "source_uri": "/dev/video4", #back_web_cam
        "port": 7091,
        "caps": {
            "format": "YUY2",
            "width": 640,
            "height": 480,
            "framerate": 30
        },
        "bitrate": 500
    },

    "orbbec_color": {
        "source": "v4l2src",
        "source_uri": "/dev/video10", #orbbec_color_cam
        "port": 7092,
        "caps": {
            "format": "YUY2",
            "width": 1280,
            "height": 720,
            "framerate": 30
        },
        "bitrate": 500,
    }  
}

#Build Pipelines

class CameraPipelineStreamer:
    def __init__(self, camera_name, latency, client_ip):

        self.camera_name = camera_name
        self.pipeline = None
        self.bus = None
        self.client_ip = client_ip

        # Save them here, in case we ever want to change these paramaters away from defaultsr
        self.source_plugin = CAMERAS_DEFAULTS[self.camera_name]["source"]
        self.source_uri = CAMERAS_DEFAULTS[self.camera_name]["source_uri"]
        self.port = CAMERAS_DEFAULTS[self.camera_name]["port"]
        self.format = CAMERAS_DEFAULTS[self.camera_name]["caps"]["format"]

        # the following can be changed as needed
        # first set with defaults
        self.latency = latency
        self.width = CAMERAS_DEFAULTS[self.camera_name]["caps"]["width"]
        self.height = CAMERAS_DEFAULTS[self.camera_name]["caps"]["height"]
        self.framerate = CAMERAS_DEFAULTS[self.camera_name]["caps"]["framerate"]
        self.bitrate = CAMERAS_DEFAULTS[self.camera_name]["bitrate"]

    # setter methods for changing parameters
    def set_width(self, width):
        self.width = width  

    def set_height(self, height):
        self.height = height    

    def set_framerate(self, framerate):
        self.framerate = framerate  

    def set_bitrate(self, bitrate):
        self.bitrate = bitrate
    
    def build(self):

        # create gstreamer pipeline
        self.pipeline = Gst.Pipeline.new(self.camera_name)
        
        source = Gst.ElementFactory.make(self.source_plugin, f"{self.camera_name}-source")
        source.set_property("device", self.source_uri)

        capsfilter = Gst.ElementFactory.make("capsfilter", f"{self.camera_name}-caps")
        caps = Gst.Caps.from_string(
            f"video/x-raw,"
            f"format={self.format},"
            f"width={self.width},"
            f"height={self.height},"
            f"framerate={self.framerate}/1"
        )
        capsfilter.set_property("caps", caps)
        
        encoder = Gst.ElementFactory.make("x264enc", f"{self.camera_name}-encoder")
        encoder.set_property("pass", "pass1")
        encoder.set_property("bitrate", self.bitrate)
        encoder.set_property("tune", "zerolatency")
        encoder.set_property("speed-preset", "ultrafast")

        videoconvert = Gst.ElementFactory.make("videoconvert", f"{self.camera_name}-convert")

        sink = Gst.ElementFactory.make("srtsink", f"{self.camera_name}-srtsink")
        sink.set_property("uri", f"srt://{self.client_ip}:{self.port}?mode=caller")
        sink.set_property("sync", False)
        sink.set_property("latency", self.latency)

        # check if all elements were created successfully
        if (source is None or capsfilter is None or videoconvert is None 
           or encoder is None or sink is None):
            raise RuntimeError(f"[{self.camera_name}] failed to create elements for pipeline")

        # add elements to the pipeline
        self.pipeline.add(source)
        self.pipeline.add(capsfilter)
        self.pipeline.add(videoconvert)
        self.pipeline.add(encoder)
        self.pipeline.add(sink)

        # link the elements together (convert raw -> encode -> sink)
        source.link(capsfilter)
        capsfilter.link(videoconvert)
        videoconvert.link(encoder)
        encoder.link(sink)

        print()
        print(self.camera_name)
        print(
            f"  {self.source_plugin} device={self.source_uri} "
            f"[{self.format} {self.width}x{self.height}@{self.framerate}] "
            f"-> srt://{self.client_ip}:{self.port}"
        )
        print()


    def start_pipeline(self):
        # if it does not exist, build it first
        if self.pipeline is None:
            self.build()

        else:
            # if it already exists, stop it first, then rebuild it with the new parameters
            self.stop_pipeline()
            self.build()

        # connect the bus
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.bus_call, None)

        ret = self.pipeline.set_state(Gst.State.PLAYING)

        if ret == Gst.StateChangeReturn.FAILURE:
            print(f"[{self.camera_name}] FAILED to reach PLAYING state")
            self.stop_pipeline()
            raise RuntimeError(f"[{self.camera_name}] pipeline failed to start")
        else:
            print(f"[{self.camera_name}] is PLAYING")


    def stop_pipeline(self):
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.bus = None
        print(f"[{self.camera_name}] is STOPPED")


    def bus_call(self, bus, message, loop):
        t = message.type
        if t == Gst.MessageType.EOS:
            sys.stdout.write("End-of-stream\n")
            #loop.quit()
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            sys.stderr.write("Error: %s: %s\n" % (err, debug))
            #loop.quit()
        return True

    


# Example usage

# create a CameraPipelineStreamer instance 
# set any desired parameters, such as height, width, framerate, bitrate and latency
# then start the pipeline with start_pipelines() method
#  - it will handle building and destroying exsting pipelines if they already exist
# kill whenever
"""

Gst.init(None)
running = True

camera_pipelines = {}

#IP Configuration for SRTSink
srt = input("Enter PC IP Address: ")

#Prompt user to determine if they want to use default settings for cameras.
# use_defaults = input("Use Default Settings? [y/n]: ").strip().lower()

print(" === Valid commands are:")
print(" === play <camera_name> [optional] <width> <height> <framerate> <bitrate>")
print(" === stop <camera_name>")

while running:  
    commandList = input(
        "Enter command OR 'q' to quit: "
    ).lower().split()

    command = commandList[0] if commandList else None

    if command == 'q':
        running = False
        break

    if command not in ['play', 'stop']:
        print(f"Invalid command: {command}")
        continue

    camera_selection = commandList[1]

    if command == 'stop':
        if len(commandList) < 2:
            print("Please specify a camera name to stop.")
            continue

        if camera_selection not in camera_pipelines:
            print(f"No active pipeline for camera: {camera_selection}")
            continue

        pipeline_streamer = camera_pipelines[camera_selection]
        pipeline_streamer.stop_pipeline()
        del camera_pipelines[camera_selection]

    if command == 'play':
        if len(commandList) < 2:
            print("Please specify a camera name to play.")
            continue

        
        if camera_selection not in CAMERAS_DEFAULTS:
            print(f"Invalid camera selection: {camera_selection}")
            continue

        # Create or update the CameraPipelineStreamer instance
        if camera_selection not in camera_pipelines:
            camera_pipelines[camera_selection] = CameraPipelineStreamer(camera_selection, latency=200, client_ip=srt)

        pipeline_streamer = camera_pipelines[camera_selection]

        if len(commandList) == 6:
            # Optional parameters for width, height, framerate, bitrate
            width = int(commandList[2]) if len(commandList) > 2 else CAMERAS_DEFAULTS[camera_selection]["caps"]["width"]
            height = int(commandList[3]) if len(commandList) > 3 else CAMERAS_DEFAULTS[camera_selection]["caps"]["height"]
            framerate = int(commandList[4]) if len(commandList) > 4 else CAMERAS_DEFAULTS[camera_selection]["caps"]["framerate"]
            bitrate = int(commandList[5]) if len(commandList) > 5 else CAMERAS_DEFAULTS[camera_selection]["bitrate"]
            
            pipeline_streamer.set_width(width)
            pipeline_streamer.set_height(height)
            pipeline_streamer.set_framerate(framerate)
            pipeline_streamer.set_bitrate(bitrate)

        try:
            pipeline_streamer.start_pipeline()
        except RuntimeError as e:
            print(e)

"""






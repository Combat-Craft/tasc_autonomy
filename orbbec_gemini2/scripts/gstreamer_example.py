#!/usr/bin/env python

import sys, os
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

def __init__(self):
    # gst-launch-1.0 v4l2src device=/dev/back_web_cam
    # ! video/x-raw, format=YUY2, width=640, height=480, framerate=30/1
    # ! videoconvert ! x264enc pass=pass1 bitrate=500 tune=zerolatency speed-preset=ultrafast
    ##  ! srtsink uri="srt://192.168.1.XXX:7091?mode=caller" latency=200 sync=false
    ## ! autovideosink sync=false

    # Set up the gstreamer pipeline
    self.player = Gst.Pipeline.new("test-stream")
    source = Gst.ElementFactory.make("v4l2src", "back_web_cam")
    encoder = Gst.ElementFactory.make("x264enc", "video-encoder")
    videoconvert = Gst.ElementFactory.make("videoconvert", "video-converter")
    sink = Gst.ElementFactory.make("autovideosink", "video-output")
    caps = Gst.Caps.from_string("video/x-raw, format=YUY2, width=640, height=480, framerate=30/1")

    source.set_property("device", "/dev/back_web_cam")

    encoder.set_property("pass", "pass1")
    encoder.set_property("bitrate", 500)
    encoder.set_property("tune", "zerolatency")
    encoder.set_property("speed-preset", "ultrafast")

    self.player.add(source) 
    self.player.add(encoder) 
    self.player.add(videoconvert)
    self.player.add(sink) 

    bus = self.player.get_bus()
    bus.add_signal_watch()
    bus.enable_sync_message_emission()
    bus.connect("message", self.on_message)
    bus.connect("sync-message::element", self.on_sync_message)
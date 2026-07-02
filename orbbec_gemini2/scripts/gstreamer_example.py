#!/usr/bin/env python

import sys
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GLib', '2.0')
from gi.repository import Gst, GLib

def bus_call(bus, message, loop):
    t = message.type
    if t == Gst.MessageType.EOS:
        sys.stdout.write("End-of-stream\n")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        sys.stderr.write("Error: %s: %s\n" % (err, debug))
        loop.quit()
    return True


def main(args):
    Gst.init(None)
    
    # gst-launch-1.0 v4l2src device=/dev/back_web_cam
    # ! video/x-raw, format=YUY2, width=640, height=480, framerate=30/1
    # ! videoconvert ! x264enc pass=pass1 bitrate=500 tune=zerolatency speed-preset=ultrafast
    ##  ! srtsink uri="srt://192.168.1.XXX:7091?mode=caller" latency=200 sync=false
    ## ! autovideosink sync=false

    # Set up the gstreamer pipeline
    player = Gst.Pipeline.new("test-stream")

    source = Gst.ElementFactory.make("v4l2src", "back_web_cam")
    source.set_property("device", "/dev/back_web_cam")

    caps = Gst.Caps.from_string("video/x-raw, format=YUY2, width=640, height=360, framerate=30/1")
    filter = Gst.ElementFactory.make("capsfilter", "filter")
    filter.set_property("caps", caps)
    
    encoder = Gst.ElementFactory.make("x264enc", "video-encoder")
    encoder.set_property("pass", "pass1")
    encoder.set_property("bitrate", 500)
    encoder.set_property("tune", "zerolatency")
    encoder.set_property("speed-preset", "ultrafast")

    videoconvert = Gst.ElementFactory.make("videoconvert", "video-converter")

    sink = Gst.ElementFactory.make("srtsink", "sink")
    sink.set_property("uri", "srt://192.168.1.23:7092?mode=caller")
    sink.set_property("sync", False)
    sink.set_property("latency", 200)
    
    # add elements to the pipeline
    player.add(source)
    player.add(filter)
    player.add(videoconvert)
    player.add(encoder)
    player.add(sink)

    # link the elements together (convert raw -> encode -> sink)
    source.link(filter)
    filter.link(videoconvert)
    videoconvert.link(encoder)
    encoder.link(sink)

    # create and event loop and feed gstreamer bus mesages to it
    loop = GLib.MainLoop()

    # start play back and listen to event
    bus = player.get_bus()
    bus.add_signal_watch()
    bus.connect("message", bus_call, loop)

    # start play back and listed to events
    player.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except Exception as e:
        pass

    # cleanup
    player.set_state(Gst.State.NULL)


if __name__ == '__main__':
    sys.exit(main(sys.argv))


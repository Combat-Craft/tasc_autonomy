#!/usr/bin/env python3


import sys
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GLib', '2.0')
from gi.repository import Gst, GLib

#DEFAULT SETTINGS

#List of dictionary entries for each camera. Check if the default settings match the camera's capabilities.
CAMERAS = {
    "back": {
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

     "mina": {
         "source": "v4l2src",
         "source_uri": "/dev/mina_cam",
         "port": 7090,
         "caps": {
             "format": "YUY2",
             "width": 640,
             "height": 480,
             "framerate": 30,
             "bitrate": 500,
         },
     },
}

#Build Pipelines

class CameraPipeline:
    def __init__(self, camera_name, config, client_ip):
        self.camera_name = camera_name
        self.config = config
        self.pipeline = None
    
    def build(self):

        caps_c = self.config["caps"]
        format = caps_c["format"].upper()
        is_MJPG = format in ("MJPG", "MJPEG")

        pipeline = Gst.Pipeline.new(self.camera_name)

        source = Gst.ElementFactory.make(self.config["source"], f"{self.camera_name}-source")
        if source is None:
            raise RuntimeError(f"[{self.camera_name}] failed to create element '{self.config['source']}'")
        source.set_property("device", self.config["source_uri"])

        capsfilter = Gst.ElementFactory.make("capsfilter", f"{self.camera_name}-caps")
        if capsfilter is None:
            raise RuntimeError(f"[{self.camera_name}] failed to create element 'capsfilter'")

        if is_MJPG:
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
            jpegdec = Gst.ElementFactory.make("jpegdec", f"{self.camera_name}-jpegdec")
            if jpegdec is None:
                raise RuntimeError(f"[{self.camera_name}] failed to create element 'jpegdec'")
        else:
            jpegdec = None
    
        convert = Gst.ElementFactory.make("videoconvert", f"{self.camera_name}-convert")
        if convert is None:
            raise RuntimeError(f"[{self.camera_name}] failed to create element 'videoconvert'")

        encoder = Gst.ElementFactory.make("x264enc", f"{self.camera_name}-encoder")
        if encoder is None:
            raise RuntimeError(f"[{self.camera_name}] failed to create element 'x264enc'")
        encoder.set_property("bitrate", caps_c["bitrate"])
        encoder.set_property("tune", "zerolatency")
        encoder.set_property("speed-preset", "ultrafast")

        sink = Gst.ElementFactory.make("srtsink", f"{self.camera_name}-srtsink")
        if sink is None:
            raise RuntimeError(f"[{self.camera_name}] failed to create element 'srtsink'")
        sink.set_property("uri", f"srt://{self.client_ip}:{self.config['port']}?mode=caller")
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

        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)
        bus.connect("message::warning", self._on_bus_warning)

        self.pipeline = pipeline

        print()
        print(self.camera_name)
        print(
            f"  {self.config['source']} device={self.config['source_uri']} "
            f"[{format} {caps_c['width']}x{caps_c['height']}@{caps_c['framerate']}] "
            f"-> srt://{self.client_ip}:{self.config['port']}"
        )
        print()

    

    def _on_bus_error(self, bus, message):
        err, debug = message.parse_error()
        print(f"[{self.camera_name}] ERROR: {err.message}")
        if debug:
            print(f"[{self.camera_name}] DEBUG: {debug}")

    def _on_bus_warning(self, bus, message):
        warn, debug = message.parse_warning()
        print(f"[{self.camera_name}] WARNING: {warn.message}")

    def start_pipelines(self):
        if self.pipeline is None:
            self.build()
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print(f"[{self.camera_name}] FAILED to reach PLAYING state")
            self.stop_pipelines()
            raise RuntimeError(f"[{self.camera_name}] pipeline failed to start")
        else:
            print(f"[{self.camera_name}] is PLAYING")
    

    def stop_pipelines(self):
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        print(f"[{self.camera_name}] is KILLED")




mina_cam_pipeline = None

def _get_pipeline(camera_key):
    if camera_key == "mina_cam":
        return mina_cam_pipeline
    return None

def _set_pipeline(camera_key, pipeline):
    global mina_cam_pipeline
    if camera_key == "mina_cam":
        mina_cam_pipeline = pipeline

def handle_camera(requested_name):
    camera_key = CAMERA_NAMES.get(requested_name, requested_name)
    if camera_key not in CAMERAS:
        return None
    return camera_key

def build_config(camera_key, data):
    configuration = json.loads(json.dumps(CAMERAS[camera_key]))
    caps = configuration["caps"]

    resolution = data.get("res")
    if resolution:
        try:
            width, height = str(resolution).lower().split("x")
            caps["width"] = int(width)
            caps["height"] = int(height)
        except ValueError:
            print(f"{camera_key} malformed resolution.")

    fps = data.get("fps")
    if fps:
        caps["framerate"] = int(fps)

    bitrate = data.get("bitrate")
    if bitrate:
        caps["bitrate"] = int(bitrate)

    return configuration

def handle_play(camera_key, data, client_ip):
    existance = _get_pipeline(camera_key)
    if existance is not None:
        print(f"[{camera_key}] already exists, destroying before rebuild")
        existance.stop_pipelines()
        _set_pipeline(camera_key, None)

    configuration = build_config(camera_key, data)
    camera_pipeline = CameraPipeline(camera_key, configuration, client_ip)
    camera_pipeline.start_pipelines()
    _set_pipeline(camera_key, camera_pipeline)

def handle_pause(camera_key):
    existance = _get_pipeline(camera_key)
    if existance is None:
        print(f"[{camera_key}] does not exist, pause ignored")
        return
    existance.pause_pipelines()

def handle_stop(camera_key):
    existance = _get_pipeline(camera_key)
    if existance is None:
        print(f"[{camera_key}] does not exist, kill ignored")
        return
    existance.stop_pipelines()
    _set_pipeline(camera_key, None)
        


if __name__ == "__main__":
    
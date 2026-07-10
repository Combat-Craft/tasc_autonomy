#!/usr/bin/env python3
#Modified22

import gi
import json
import socket
import threading
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

#DEFAULT SETTINGS

HOST = "0.0.0.0"
PORT = 6001

LATENCY = 200

#List of dictionary entries for each camera. Check if the default settings match the camera's capabilities.
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


CAMERA_NAMES = {
    "back": "back_web_cam",
    # "orbbec": "orbbec_cam",
}


#Build Pipelines

class CameraPipeline:
    def __init__(self, camera_name, config, client_ip):
        self.camera_name = camera_name
        self.config = config
        self.client_ip = client_ip
        self.pipeline = None
    
    def build(self):

        name = self.camera_name
        caps_c = self.config["caps"]
        format = caps_c["format"].upper()
        is_MJPG = format in ("MJPG", "MJPEG")

        pipeline = Gst.Pipeline.new(name)

        source = Gst.ElementFactory.make(self.config["source"], f"{name}-source")
        if source is None:
            raise RuntimeError(f"[{name}] failed to create element '{self.config['source']}'")
        source.set_property("device", self.config["source_uri"])

        capsfilter = Gst.ElementFactory.make("capsfilter", f"{name}-caps")
        if capsfilter is None:
            raise RuntimeError(f"[{name}] failed to create element 'capsfilter'")

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
            jpegdec = Gst.ElementFactory.make("jpegdec", f"{name}-jpegdec")
            if jpegdec is None:
                raise RuntimeError(f"[{name}] failed to create element 'jpegdec'")
        else:
            jpegdec = None
    
        convert = Gst.ElementFactory.make("videoconvert", f"{name}-convert")
        if convert is None:
            raise RuntimeError(f"[{name}] failed to create element 'videoconvert'")

        encoder = Gst.ElementFactory.make("x264enc", f"{name}-encoder")
        if encoder is None:
            raise RuntimeError(f"[{name}] failed to create element 'x264enc'")
        encoder.set_property("bitrate", caps_c["bitrate"])
        encoder.set_property("tune", "zerolatency")
        encoder.set_property("speed-preset", "ultrafast")

        sink = Gst.ElementFactory.make("srtsink", f"{name}-srtsink")
        if sink is None:
            raise RuntimeError(f"[{name}] failed to create element 'srtsink'")
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
        print(name)
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

    def pause_pipelines(self):
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.PAUSED)
            print(f"[{self.camera_name}] is PAUSED")


back_web_cam_pipeline = None

def _get_pipeline(camera_key):
    if camera_key == "back_web_cam":
        return back_web_cam_pipeline
    return None

def _set_pipeline(camera_key, pipeline):
    global back_web_cam_pipeline
    if camera_key == "back_web_cam":
        back_web_cam_pipeline = pipeline


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
        

def dispatch(data, client_ip):
    """
    Expected data shape (already parsed from the key=value wire format), e.g.:
      {"state": "play", "camera": "back", "res": "640x360", "fps": "15", "bitrate": "300"}
      {"state": "pause", "camera": "back"}
      {"state": "kill", "camera": "back"}
    res/fps/bitrate are optional and only meaningful for "play".
    """
    
    state = str(data.get("state","")).strip().lower()
    requested_name = data.get("camera")

    if not requested_name:
        print(f"Invalid camera name")
        return
    
    camera_key = handle_camera(requested_name)
    if camera_key is None:
        print(f"Unknown camera")
        return
    
    if state == "play":
        handle_play(camera_key, data, client_ip)
    elif state == "pause":
        handle_pause(camera_key)
    elif state in ("stop", "kill"):
        handle_stop(camera_key)
    else:
        print("Invalid state")

def recv_message(connect):
    buffer = b""
    chunk = connect.recv(4096)
    if not chunk:
        return None
    buffer += chunk

    try:
        return json.loads(buffer.decode("utf-8").strip())
    except json.JSONDecodeError:
        print(f"Malformed message.")
        return {}

def run_server():
    print("Wait for Gstreamer to initialize...")
    Gst.init(None)
    print("Gstreamer initialized.")

    # Bus signal watches (message::error / message::warning) are only
    # delivered while a GLib main loop is running. The main thread below
    # is blocked in socket.accept()/recv(), so without this, every
    # bus watch we attach is silently inert and pipeline errors never
    # get printed anywhere.
    glib_loop = GLib.MainLoop()
    glib_thread = threading.Thread(target=glib_loop.run, daemon=True)
    glib_thread.start()
    print("GLib main loop started for bus message dispatch.")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"TCP Server initialized on {HOST}:{PORT}")

    try:
        while True:
            print("\n Waiting for connection request...")
            connect, addr = server_socket.accept()
            client_ip = addr[0]
            print(f"Connection request received from {client_ip}, handshake completed.")

            try:
                while True:
                    print("Waiting to receive data...")
                    data = recv_message(connect)
                    if data is None:
                        print("No data, client disconnected")
                        break
                    if not data:
                        continue
                    print(f"Received: {data}")
                    try:
                        dispatch(data, client_ip)
                    except Exception as exc:
                        print(f"Error handling {data}: {exc}")
            finally:
                connect.close()
    except KeyboardInterrupt:
        print("Shutting down...")
    
    finally:
        if back_web_cam_pipeline is not None:
            back_web_cam_pipeline.stop_pipelines()

        server_socket.close()
        glib_loop.quit()

if __name__ == "__main__":
    run_server()
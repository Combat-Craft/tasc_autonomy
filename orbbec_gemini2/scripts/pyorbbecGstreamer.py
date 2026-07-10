#!/usr/bin/env python

# the issue, is using that video_frame_callback(), handle_X(), and render_frames()
# to also handle gstreamer appsrc buffer nonsense
# as the conversion is frames -> buffer, then push buffer to appsrc
#

# one solution is to say F frame handling
# set the config file manually via OrbbecSDKConfig.xml
# ~/.local/lib/python3.10/site-packages/pyorbbecsdk/OrbbecSDKConfig.xml
# as per https://github.com/orbbec/pyorbbecsdk/blob/8657b067cfd6d778e289fbc350d657021bcc8e9c/sdk/lib/arm64/OrbbecSDKConfig.md
# then use v4l2src

# other is to keep trying this bs


import os
import sys

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GLib', '2.0')
from gi.repository import Gst, GLib

import math
import threading

from pyorbbecsdk import OBFormat  # type: ignore
from pyorbbecsdk import Config, Context, OBError, OBFrameType, OBSensorType, Pipeline


# look at multi_stream

class GlobalState:
    def __init__(self):
        self.frame_mutex = threading.Lock()
        self.imu_mutex = threading.Lock()
        self.stop_rendering = False
        self.support_dual_ir = True
        self.support_imu = True
        self.support_dual_rgb = False # only 1
        # cached frames for better visualization
        self.cached_frames = {
            "color": None,
            "depth": None,
            "left_ir": None,
            "right_ir": None,
            "ir": None,
            "confidence": None,
            "accel": None,
            "gyro": None,
            "left_color": None,
            "right_color": None,
        }
state = GlobalState()

width = 1280
height = 720
fps = 30


def setup_camera(yesColor = True, yesDepth = False, yesIR = False, yesIMU = False):
    """Setup camera and stream configuration"""
    # 1.Create a pipeline with default device.
    orbbec_pipeline = Pipeline()
    # 2.Get the device from pipeline.
    device = pipeline.get_device()
    # 3.
    device_info = device.get_device_info()
    # 4.Create a config for pipeline.
    config = Config()
    
    # 5. Disable all streams explicityly
    config.disable_all_stream();
    
    # Enable requested sensors
    sensor_list = device.get_sensor_list() # get sensor list
    
    for sensor in range(len(sensor_list)):
        sensor_type = sensor_list[sensor].get_type()
        
        if yesColor:   
            # enable color 
            color_profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_STREAM)
            # only MJPEG and 
            color_profile = color_profile_list.get_video_stream_profile(width, height, OBFormat.YUY2, fps)
            config.enable_stream(color_profile)
            
        if yesDepth:
            depth_profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            # only Y14H14 and RLE
            depth_profile = depth_profile_list.get_video_stream_profile(width, height, OBFormat.Y16, fps)
            config.enable_stream(depth_profile)
            
        if yesIR:
            # format MJPEG and Y8H8
            config.enable_stream(OBSensorType.IR_SENSOR)    
            # config.enable_stream(OBSensorType.LEFT_IR_SENSOR)    
            # config.enable_stream(OBSensorType.RIGHT_IR_SENSOR)  
            
        if yesIMU:
            # orienttion is X-right, y-down, Z-forward 
            config.enable_accel_stream()
            config.enable_gyro_stream()
    
    try:
        pipeline.start(config, video_frame_callback)
    except OBError as e:
        print(f"Error: {e}")
        print("Please connect an Orbbec camera and try again.")
        return None
    return pipeline
    
def process_color(frame):
    """Process color image"""
    pass


def process_depth(frame):
    """Process depth image"""
    pass


def process_ir(ir_frame):
    """Process IR frame (left or right) to RGB image"""
    pass
    
def video_frame_callback(frames):
    if frames is None:
        return None
    with state.frame_mutex:
        if frames:
            state.cached_frames["color"] = process_color(frames.get_color_frame())
            #state.cached_frames["depth"] = process_depth(frames.get_depth_frame())

def render_frames():
    # Window settings
    WINDOW_NAME = "MultiStream Viewer"
    DISPLAY_WIDTH = 1280
    DISPLAY_HEIGHT = 720

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, DISPLAY_WIDTH, DISPLAY_HEIGHT)

    while not state.stop_rendering:
        blocks = []
        check_keys = [
            "color",
            "depth",
            "left_ir",
            "right_ir",
            "ir",
            "confidence",
            "accel",
            "gyro",
            "left_color",
            "right_color",
        ]
        with state.frame_mutex, state.imu_mutex:
            # create display
            for key in check_keys:
                img = state.cached_frames.get(key)
                if img is not None:
                    blocks.append(img)

        if not blocks:
            if cv2.waitKey(5) & 0xFF in [ord("q"), 27]:
                break
            continue

        display = create_display(blocks, DISPLAY_WIDTH, DISPLAY_HEIGHT)
        cv2.imshow(WINDOW_NAME, display)

        # check exit key
        key = cv2.waitKey(1) & 0xFF
        if key in [ord("q"), 27]:  # q or ESC
            break




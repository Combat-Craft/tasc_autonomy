#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from foxglove_msgs.msg import CompressedVideo
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class MultiCameraStreamer(Node):

    def __init__(self):
        super().__init__('multi_camera_streamer')

        Gst.init(None)

        self.cameras = {
            "arm_cam": {
                "pipeline":
                "v4l2src device=/dev/video0 ! "
                "video/x-raw,width=640,height=480,framerate=30/1 ! "
                "videoconvert ! "
                "x264enc tune=zerolatency speed-preset=ultrafast bitrate=2048 ! "
                "video/x-h264,stream-format=byte-stream,alignment=au ! "
                "appsink name=sink0 emit-signals=true sync=false drop=true",
                "topic": "/arm_cam/h264"
            },

            "pose_cam": {
                "pipeline":
                "v4l2src device=/dev/video2 ! "
                "video/x-raw,width=640,height=480,framerate=30/1 ! "
                "videoconvert ! "
                "x264enc tune=zerolatency speed-preset=ultrafast bitrate=2048 ! "
                "video/x-h264,stream-format=byte-stream,alignment=au ! "
                "appsink name=sink1 emit-signals=true sync=false drop=true",
                "topic": "/pose_cam/h264"
            },

            "poe_cam": {
                "pipeline":
                "rtspsrc location=rtsp://192.168.1.50/stream latency=0 ! "
                "rtph265depay ! "
                "h265parse ! "
                "appsink name=sink2 emit-signals=true sync=false drop=true",
                "topic": "/poe_cam/h265"
            }
        }

        self.publishers = {}
        self.pipelines = {}

        for i, (name, config) in enumerate(self.cameras.items()):

            pub = self.create_publisher(
                CompressedVideo,
                config["topic"],
                10
            )

            pipeline = Gst.parse_launch(config["pipeline"])

            sink = pipeline.get_by_name(f"sink{i}")

            sink.connect(
                "new-sample",
                lambda sink, cam=name: self.publish_frame(sink, cam)
            )

            pipeline.set_state(Gst.State.PLAYING)

            self.publishers[name] = pub
            self.pipelines[name] = pipeline

            self.get_logger().info(f"{name} started")

    def publish_frame(self, sink, cam):

        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        msg = CompressedVideo()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.format = "h264"
        msg.data = map_info.data

        self.publishers[cam].publish(msg)

        buf.unmap(map_info)

        return Gst.FlowReturn.OK


def main():

    rclpy.init()

    node = MultiCameraStreamer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
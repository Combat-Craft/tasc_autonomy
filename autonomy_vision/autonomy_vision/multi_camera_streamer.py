#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from foxglove_msgs.msg import CompressedVideo
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class MultiCameraStreamer(Node):

    def __init__(self):
        super().__init__('multi_camera_streamer')

        Gst.init(None)
        encoder_block = self._select_h264_encoder_block()

        self.cameras = {
            "arm_cam": {
                "pipeline":
                "v4l2src device=/dev/video0 ! "
                "image/jpeg,width=640,height=480,framerate=30/1 ! "
                "jpegparse ! "
                "jpegdec ! "
                "videoconvert ! "
                f"{encoder_block}"
                "video/x-h264,stream-format=byte-stream,alignment=au ! "
                "appsink name={sink_name} emit-signals=true sync=false drop=true",
                "topic": "/arm_cam/h264"
            },
                "poe_cam": {
                "pipeline":
                "v4l2src device=/dev/video6 ! "
                "image/jpeg,width=640,height=480,framerate=30/1 ! "
                "jpegparse ! "
                "jpegdec ! "
                "videoconvert ! "
                f"{encoder_block}"
                "video/x-h264,stream-format=byte-stream,alignment=au ! "
                "appsink name={sink_name} emit-signals=true sync=false drop=true",
                "topic": "/poe_cam/h264"
            },
        }

        self.camera_publishers = {}
        self.camera_pipelines = {}
        self.frame_counts = {}
        self.pipeline_buses = {}

        for i, (name, config) in enumerate(self.cameras.items()):

            pub = self.create_publisher(
                CompressedVideo,
                config["topic"],
                qos_profile_sensor_data
            )

            # create pipeline with a unique appsink name for this camera
            pipeline = Gst.parse_launch(config["pipeline"].format(sink_name=f"sink{i}"))

            sink = pipeline.get_by_name(f"sink{i}")
            sink.set_property("max-buffers", 1)
            sink.set_property("drop", True)
            sink.set_property("sync", False)

            sink.connect(
                "new-sample",
                lambda sink, cam=name: self.publish_frame(sink, cam)
            )

            pipeline.set_state(Gst.State.PLAYING)

            self.camera_publishers[name] = pub
            self.camera_pipelines[name] = pipeline
            self.frame_counts[name] = 0
            self.pipeline_buses[name] = pipeline.get_bus()

            self.get_logger().info(f"{name} started")

        self.create_timer(1.0, self._report_pipeline_health)

    def _select_h264_encoder_block(self):
        # Pick the first available encoder so the node can run across systems.
        candidates = [
            ("x264enc", "x264enc tune=zerolatency speed-preset=ultrafast bitrate=2048 ! "),
            ("avenc_h264_omx", "avenc_h264_omx bitrate=2000000 ! "),
        ]

        for element_name, encoder_block in candidates:
            if Gst.ElementFactory.find(element_name):
                self.get_logger().info(f"Using encoder: {element_name}")
                return encoder_block

        raise RuntimeError(
            "No H.264 encoder found. Install gstreamer1.0-plugins-ugly "
            "(x264enc) or provide another H.264 encoder plugin."
        )

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
        msg.data = bytes(map_info.data)

        self.camera_publishers[cam].publish(msg)
        self.frame_counts[cam] += 1

        buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def _report_pipeline_health(self):
        for cam, bus in self.pipeline_buses.items():
            while True:
                msg = bus.timed_pop_filtered(
                    0,
                    Gst.MessageType.ERROR | Gst.MessageType.WARNING
                )
                if msg is None:
                    break

                if msg.type == Gst.MessageType.ERROR:
                    err, debug = msg.parse_error()
                    self.get_logger().error(
                        f"{cam} pipeline error: {err}; debug={debug}"
                    )
                elif msg.type == Gst.MessageType.WARNING:
                    warn, debug = msg.parse_warning()
                    self.get_logger().warning(
                        f"{cam} pipeline warning: {warn}; debug={debug}"
                    )

            fps = self.frame_counts.get(cam, 0)
            self.get_logger().info(f"{cam} publish fps: {fps}")
            self.frame_counts[cam] = 0


def main():

    rclpy.init()

    node = MultiCameraStreamer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
#!/usr/bin/env python3
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32


class PanoramaSweeper(Node):

    def __init__(self):
        super().__init__('panorama_sweeper')

        # Storage
        self.frames = []
        self.angles = []

        # Sweep config (degrees)
        self.sweep_angles = np.linspace(20, 185, 18).astype(int)
        self.sweep_index = 0

        # Timing (seconds)
        self.move_time   = 1.0   # how long servo takes to move
        self.settle_time = 0.5   # extra wait before capture
        self.ready_time  = None

        # State machine
        self.state = "IDLE"   # IDLE → WAIT → CAPTURE → DONE

        # Publisher: servo angle (Int32)
        self.servo_pub = self.create_publisher(
            Int32,
            '/servo_angle',
            10
        )

        # Subscriber: compressed camera image
        self.create_subscription(
            CompressedImage,
            '/arm_cam/image/compressed',
            self.image_callback,
            10
        )

        # Timer to drive logic
        self.timer = self.create_timer(0.1, self.update)

        # Startup delay
        self.start_time = time.time()
        self.started = False

        self.get_logger().info("PanoramaSweeper ready (Int32 servo mode)")

    # -------------------------------------------------
    # Main update loop
    # -------------------------------------------------
    # def update(self):
    #     now = time.time()

    #     if not self.started and now - self.start_time > 2.0:
    #         self.started = True
    #         self.start_sweep()

    #     if self.state == "WAIT" and now >= self.ready_time:
    #         self.state = "CAPTURE"
    def update(self):
        now = time.time()

        if not self.started and now - self.start_time > 2.0:
            self.started = True
            self.start_sweep()

        if self.state == "MOVING":
            self.move_to_next_angle()

        elif self.state == "WAIT" and now >= self.ready_time:
            self.state = "CAPTURE"


    # -------------------------------------------------
    # Start sweep
    # -------------------------------------------------
    def start_sweep(self):
        self.get_logger().info("Starting panorama sweep")
        self.move_to_next_angle()

    # -------------------------------------------------
    # Command servo
    # -------------------------------------------------
    def move_to_next_angle(self):
        if self.sweep_index >= len(self.sweep_angles):
            self.get_logger().info("Sweep complete → stitching")
            self.stitch_panorama()
            self.state = "DONE"
            return

        angle_deg = int(self.sweep_angles[self.sweep_index])

        msg = Int32()
        msg.data = angle_deg
        self.servo_pub.publish(msg)

        self.get_logger().info(
            f"Moving to {angle_deg}° "
            f"({self.sweep_index+1}/{len(self.sweep_angles)})"
        )

        self.ready_time = time.time() + self.move_time + self.settle_time
        self.state = "WAIT"

    # -------------------------------------------------
    # Image callback
    # -------------------------------------------------
    def image_callback(self, msg: CompressedImage):
        if self.state != "CAPTURE":
            return

        # Decode JPEG
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        angle = self.sweep_angles[self.sweep_index]

        self.frames.append(frame)
        self.angles.append(angle)

        self.get_logger().info(
            f"Captured image at {angle}° "
            f"({len(self.frames)}/{len(self.sweep_angles)})"
        )

        # self.sweep_index += 1
        # self.move_to_next_angle()
        self.sweep_index += 1
        self.state = "MOVING"

        

    # -------------------------------------------------
    # Stitch panorama
    # -------------------------------------------------
    def stitch_panorama(self):
        if len(self.frames) < 2:
            self.get_logger().warn("Not enough frames to stitch")
            return

        self.get_logger().info(f"Stitching {len(self.frames)} images")

        stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
        status, pano = stitcher.stitch(self.frames)

        if status != cv2.Stitcher_OK:
            self.get_logger().error(f"Stitch failed (status {status})")
            return

        cv2.imwrite("panorama.jpg", pano)
        self.get_logger().info("Panorama saved → panorama.jpg")

        cv2.imshow("Panorama", pano)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PanoramaSweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

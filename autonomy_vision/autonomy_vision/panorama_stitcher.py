#!/usr/bin/env python3
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, NavSatFix
from std_msgs.msg import Float64, Float32, String


class PanoramaSweeper(Node):

    def __init__(self):
        super().__init__('panorama_stitcher')


        # Testing motor movement when camera isnt connected
        self.TEST_MODE = False

        # Storage
        self.frames = []
        self.angles = []
        self.frame_metadata = []

        # Sweep config (degrees)
        self.sweep_angles = np.linspace(-135, 135, 10)
        self.sweep_index = 0

        # Timing (seconds)
        self.move_time   = 1.0   # how long servo takes to move
        self.settle_time = 0.5   # extra wait before capture
        self.ready_time  = None

        # State machine
        self.state = "IDLE"   # IDLE → WAIT → CAPTURE → DONE

        self.latest_gps = None
        self.latest_heading = None
        self.latest_cardinal = None

        # Publisher: servo angle
        self.servo_pub = self.create_publisher(
            Float64,
            '/motor_cmd',
            10
        )

        # Subscriber: compressed camera image
        self.create_subscription(
            CompressedImage,
            '/arm_cam/image/compressed',
            self.image_callback,
            10
        )

        self.create_subscription(
            Float64, 
            '/panorama_trigger', 
            self.trigger_callback, 
            10
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        self.heading_sub = self.create_subscription(
            Float32,
            '/heading',
            self.heading_callback,
            10
        )

        self.cardinal_sub = self.create_subscription(
            String,
            '/cardinal_compass',
            self.cardinal_callback,
            10
        )

        # Timer to drive logic
        self.timer = self.create_timer(0.1, self.update)

        # Startup delay
        self.triggered = False

        self.get_logger().info("PanoramaSweeper ready | Send 999.0 to /motor_cmd to trigger")

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

        if self.state == "MOVING":
            self.move_to_next_angle()
        elif self.state == "WAIT" and now >= self.ready_time:
            self.state = "CAPTURE"

        # TEST MODE
        elif self.state == "CAPTURE" and self.TEST_MODE:
            self.get_logger().info(
                f"[TEST] Simulated capture at {self.sweep_angles[self.sweep_index]} degrees"
                f"({self.sweep_index+1}/{len(self.sweep_angles)})"
            )
            self.sweep_index += 1
            self.state = "MOVING"


    def gps_callback(self, msg):
        self.latest_gps = (msg.latitude, msg.longitude)

    def heading_callback(self, msg):
        self.latest_heading = msg.data

    def cardinal_callback(self, msg):
        self.latest_cardinal = msg.data

    # -------------------------------------------------
    # Trigger sweep
    # -------------------------------------------------
    def trigger_callback(self, msg):
        self.get_logger().info("Panorama trigger received")
        self.start_sweep()
    
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

        angle_deg = self.sweep_angles[self.sweep_index]

        msg = Float64()
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
        if self.TEST_MODE:
            return

        if self.state != "CAPTURE":
            return

        # Decode JPEG
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        current_servo_angle = self.sweep_angles[self.sweep_index]

        self.frames.append(frame)
        self.angles.append(current_servo_angle)

        metadata = {
            "servo_angle": current_servo_angle,
            "gps": self.latest_gps,
            "heading": self.latest_heading,
            "cardinal": self.latest_cardinal,
        }

        self.frame_metadata.append(metadata)

        self.get_logger().info(
            f"Captured image at {current_servo_angle}° "
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
        if self.TEST_MODE:
            self.get_logger().info("[TEST] Sweep complete - skipping stitch since there are no real camera frames")
            
            # Reset for next sweep
            self.sweep_index = 0
            self.frames = []
            self.angles = []
            self.state = "IDLE"
            return

        if len(self.frames) < 2:
            self.get_logger().warn("Not enough frames to stitch")
            return

        self.get_logger().info(f"Stitching {len(self.frames)} images")

        stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
        status, pano = stitcher.stitch(self.frames)

        if status != cv2.Stitcher_OK:
            self.get_logger().error(f"Stitch failed (status {status})")
            return


        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (255, 255, 255)  # White text
        shadow_color = (0, 0, 0)  # Black shadow for readability

        # Draw angle markers for each captured position
        h, w = pano.shape[:2]
        for i, angle in enumerate(self.angles):
            x = int((i / (len(self.angles) - 1)) * (w - 40)) + 20  # spread across width
            label_angle = f"{int(angle)}"
            cv2.putText(pano, label_angle, (x - 15, h - 20), font, 0.5, shadow_color, 3)
            cv2.putText(pano, label_angle, (x - 15, h - 20), font, 0.5, color, 1)
            cv2.line(pano, (x, h - 40), (x, h - 10), color, 1)  # tick mark

        cv2.imwrite("panorama.jpg", pano)
        self.get_logger().info("Panorama saved → panorama.jpg")

        cv2.imshow("Panorama", pano)
        cv2.waitKey(1)

        # Reset for next sweep
        self.sweep_index = 0
        self.frames = []
        self.angles = []
        self.state = "IDLE"


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
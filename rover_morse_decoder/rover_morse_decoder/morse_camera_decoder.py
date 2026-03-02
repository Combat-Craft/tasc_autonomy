#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

# ==============================
# Morse dictionary (digits only)
# ==============================
MORSE_DICT = {
    ".----": "1",
    "..---": "2",
    "...--": "3",
    "....-": "4",
    ".....": "5",
    "-....": "6",
    "--...": "7",
    "---..": "8",
    "----.": "9",
    "-----": "0"
}

# ==============================
# Timing parameters (seconds)
# ==============================
DOT_MAX = 0.6          # Anything shorter than this = dot
DASH_MIN = 0.6         # Anything longer than this = dash
DIGIT_GAP = 1.2        # OFF time indicating end of digit

# ==============================
# Brightness hysteresis
# ==============================
ON_THRESHOLD_HIGH = 210
ON_THRESHOLD_LOW  = 160


class MorseCameraDecoder(Node):
    def __init__(self):
        super().__init__('morse_camera_decoder')

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos
        )

        self.bridge = CvBridge()

        # State
        self.light_on = False
        self.last_transition_time = time.time()
        self.current_symbol = ""
        self.decoded = ""

        self.get_logger().info("Morse camera decoder running")

    # ==========================================
    # Image callback
    # ==========================================
    def image_callback(self, msg):
        now = time.time()

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = cv_image.shape

        # Sample a larger center ROI for stability
        roi = cv_image[
            h//2 - 25 : h//2 + 25,
            w//2 - 25 : w//2 + 25
        ]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        brightness = float(np.mean(gray))

        # ------------------------------
        # Hysteresis-based detection
        # ------------------------------
        prev_state = self.light_on
        if self.light_on:
            self.light_on = brightness > ON_THRESHOLD_LOW
        else:
            self.light_on = brightness > ON_THRESHOLD_HIGH

        # ------------------------------
        # State change detected
        # ------------------------------
        if self.light_on != prev_state:
            duration = now - self.last_transition_time
            self.last_transition_time = now

            # ON → OFF = measure dot/dash
            if prev_state and not self.light_on:
                if duration < DOT_MAX:
                    self.current_symbol += "."
                else:
                    self.current_symbol += "-"

            # OFF → ON = check for digit gap
            elif not prev_state and self.light_on:
                if duration >= DIGIT_GAP:
                    self.decode_current_symbol()

    # ==========================================
    # Decode symbol
    # ==========================================
    def decode_current_symbol(self):
        if not self.current_symbol:
            return

        digit = MORSE_DICT.get(self.current_symbol, "?")
        self.decoded += digit

        self.get_logger().info(
            f"Decoded: {digit}  | Symbol: {self.current_symbol}  | Full: {self.decoded}"
        )

        self.current_symbol = ""


def main():
    rclpy.init()
    node = MorseCameraDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

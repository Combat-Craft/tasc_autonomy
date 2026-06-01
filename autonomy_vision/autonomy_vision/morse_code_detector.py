#!/usr/bin/env python3

import time
import threading
import collections

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


MORSE_CODE = {
    '.-': 'A', '-...': 'B', '-.-.': 'C', '-..': 'D', '.': 'E',
    '..-.': 'F', '--.': 'G', '....': 'H', '..': 'I', '.---': 'J',
    '-.-': 'K', '.-..': 'L', '--': 'M', '-.': 'N', '---': 'O',
    '.--.': 'P', '--.-': 'Q', '.-.': 'R', '...': 'S', '-': 'T',
    '..-': 'U', '...-': 'V', '.--': 'W', '-..-': 'X', '-.--': 'Y',
    '--..': 'Z',
    '-----': '0', '.----': '1', '..---': '2', '...--': '3',
    '....-': '4', '.....': '5', '-....': '6', '--...': '7',
    '---..': '8', '----.': '9',
    '.-.-.-': '.', '--..--': ',', '..--..': '?', '.----.': "'",
    '-.-.--': '!', '-..-.': '/', '-.--.': '(', '-.--.-': ')',
    '.-...': '&', '---...': ':', '-.-.-.': ';', '-...-': '=',
    '.-.-.': '+', '-....-': '-', '..--.-': '_', '.-..-.': '"',
    '.--.-.': '@',
}


class MorseCodeDetector(Node):

    def __init__(self):
        super().__init__('morse_code_detector')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('threshold_percentile', 85.0)
        self.declare_parameter('min_bright_fraction', 0.01)
        self.declare_parameter('initial_dot_s', 0.15)
        self.declare_parameter('publish_hz', 10.0)

        cam_idx = self.get_parameter('camera_index').value
        self.thresh_pct = self.get_parameter('threshold_percentile').value
        self.min_bright_frac = self.get_parameter('min_bright_fraction').value
        initial_dot = self.get_parameter('initial_dot_s').value
        publish_hz = self.get_parameter('publish_hz').value

        self.pub_morse = self.create_publisher(String, '/morse_code', 10)
        self.pub_decoded = self.create_publisher(String, '/morse_decoded', 10)
        self.pub_debug = self.create_publisher(Image, '/morse_debug_image', 10)

        self.is_on = False
        self.on_time = None
        self.off_time = None
        self.current_sym = ''
        self.message = ''

        self._bright_hist = collections.deque(maxlen=60)
        self._lock = threading.Lock()
        self._latest_frame = None

        self.cam_idx = cam_idx
        self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Camera failed to open")
            raise RuntimeError("Camera not available")

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        self._timer = self.create_timer(1.0 / publish_hz, self._tick)

        self.get_logger().info(
            f"Morse detector started — camera {cam_idx}"
        )

    def _capture_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()

            if not ret or frame is None:
                time.sleep(0.01)
                continue

            with self._lock:
                self._latest_frame = frame

    def _tick(self):
        with self._lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame.copy()

        now = time.monotonic()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        thresh_val = float(np.percentile(gray, self.thresh_pct))
        self._bright_hist.append(thresh_val)
        adaptive_thresh = float(np.mean(self._bright_hist)) * 1.05
        adaptive_thresh = max(adaptive_thresh, 80.0)

        bright_mask = gray > adaptive_thresh
        bright_frac = bright_mask.mean()
        light_on = bright_frac >= self.min_bright_frac

        if light_on and not self.is_on:
            self.is_on = True
            self.on_time = now

            if self.off_time:
                self._handle_gap(now - self.off_time)

        elif not light_on and self.is_on:
            self.is_on = False
            self.off_time = now

            duration = now - self.on_time
            self._handle_pulse(duration)

        elif not light_on and not self.is_on:
            if self.off_time and self.current_sym:
                gap = now - self.off_time
                if gap > 1.0:
                    self._flush(word=True)

        self._publish_debug(frame, gray, adaptive_thresh, light_on, bright_frac)

    def _handle_pulse(self, duration):
        if duration < 0.25:
            self.current_sym += '.'
        else:
            self.current_sym += '-'

        msg = String()
        msg.data = self.current_sym
        self.pub_morse.publish(msg)

    def _handle_gap(self, gap):
        if gap > 1.2:
            self._flush(word=True)
        elif gap > 0.5:
            self._flush(word=False)

    def _flush(self, word=False):
        if self.current_sym:
            letter = MORSE_CODE.get(self.current_sym, '?')
            self.message += letter
            self.current_sym = ''

        if word:
            self.message += ' '

        msg = String()
        msg.data = self.message
        self.pub_decoded.publish(msg)

    def _publish_debug(self, frame, gray, thresh, light_on, bright_frac):

        debug = frame.copy()

        # ---- overlay ----
        color = (0, 255, 0) if light_on else (0, 0, 255)

        cv2.putText(
            debug,
            f"ON={light_on} bright={bright_frac:.3f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )

        debug = np.asarray(debug, dtype=np.uint8)
        debug = np.ascontiguousarray(debug)

        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = "camera"

        img.height = debug.shape[0]
        img.width = debug.shape[1]
        img.encoding = "bgr8"
        img.step = debug.shape[1] * 3

        img.data = debug.tobytes()

        self.pub_debug.publish(img)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = MorseCodeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
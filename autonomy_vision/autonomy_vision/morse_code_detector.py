#!/usr/bin/env python3
#hi!!!!

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
import time


class MorseCodeDetector(Node):
    """
    ROS2 node that detects and decodes morse code signals from camera input.
    Looks for light flashes (bright regions) and interprets them as morse code.
    """

    # Morse code dictionary
    MORSE_CODE = {
        '.-': 'A', '-...': 'B', '-.-.': 'C', '-..': 'D', '.': 'E',
        '..-.': 'F', '--.': 'G', '....': 'H', '..': 'I', '.---': 'J',
        '-.-': 'K', '.-..': 'L', '--': 'M', '-.': 'N', '---': 'O',
        '.--.': 'P', '--.-': 'Q', '.-.': 'R', '...': 'S', '-': 'T',
        '..-': 'U', '...-': 'V', '.--': 'W', '-..-': 'X', '-.--': 'Y',
        '--..': 'Z',
        '-----': '0', '.----': '1', '..---': '2', '...--': '3', '....-': '4',
        '.....': '5', '-....': '6', '--...': '7', '---..': '8', '----.': '9',
        '.-.-.-': '.', '--..--': ',', '..--..': '?', '.----.': "'", '-.-.--': '!',
        '-..-.': '/', '-.--.': '(', '-.--.-': ')', '.-...': '&', '---...': ':',
        '-.-.-.': ';', '-...-': '=', '.-.-.': '+', '-....-': '-', '..--.-': '_',
        '.-..-.': '"', '...-..-': '$', '.--.-.': '@', ' ': ' '
    }

    def __init__(self):
        super().__init__("morse_code_detector")

        # Parameters for morse detection
        self.declare_parameter('video_path', '')         # Path to MP4 video file
        self.declare_parameter('brightness_threshold', 200)  # Threshold for detecting light
        self.declare_parameter('min_area', 100)          # Minimum area of light blob
        self.declare_parameter('dot_duration', 0.2)      # Base unit duration (seconds)
        self.declare_parameter('dash_duration', 0.6)     # Dash = 3 dots
        self.declare_parameter('symbol_gap', 0.3)        # Gap between dots/dashes
        self.declare_parameter('letter_gap', 0.7)        # Gap between letters
        self.declare_parameter('word_gap', 1.4)          # Gap between words
        self.declare_parameter('playback_fps', 30.0)     # FPS for video playback

        # Get parameters
        self.video_path = self.get_parameter('video_path').value
        self.brightness_threshold = self.get_parameter('brightness_threshold').value
        self.min_area = self.get_parameter('min_area').value
        self.dot_duration = self.get_parameter('dot_duration').value
        self.dash_duration = self.get_parameter('dash_duration').value
        self.symbol_gap = self.get_parameter('symbol_gap').value
        self.letter_gap = self.get_parameter('letter_gap').value
        self.word_gap = self.get_parameter('word_gap').value
        self.playback_fps = self.get_parameter('playback_fps').value

        # State tracking — all timing uses frame counts, not wall clock
        self.is_light_on = False
        self.light_on_frame = None      # Frame number when light turned ON
        self.last_off_frame = None      # Frame number when light last turned OFF
        self.current_symbol = ""        # Dots/dashes accumulated for current letter
        self.decoded_message = ""       # Full decoded message so far
        self.last_decoded_frame = None  # Frame when we last decoded a letter (for word gap)

        # Open video file
        if not self.video_path:
            self.get_logger().error("No video path specified! Use parameter 'video_path'")
            raise ValueError("video_path parameter is required")

        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {self.video_path}")
            raise ValueError(f"Cannot open video file: {self.video_path}")

        # Get video properties
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.video_fps <= 0:
            self.get_logger().warn("Could not read video FPS, defaulting to 30.0")
            self.video_fps = 30.0

        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.frame_count = 0

        # Publishers
        self.pub_morse = self.create_publisher(String, "/morse_code", 10)
        self.pub_decoded = self.create_publisher(String, "/morse_decoded", 10)
        self.pub_debug = self.create_publisher(Image, "/morse_debug_image", 10)

        # Create timer to process video frames
        timer_period = 1.0 / self.playback_fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # FPS tracking
        self._fps_start = time.time()
        self._fps_count = 0

        self.get_logger().info(f"Morse Code Detector Node started — reading from {self.video_path}")
        self.get_logger().info(f"Video FPS: {self.video_fps}, Total frames: {self.total_frames}")
        self.get_logger().info(f"Brightness threshold: {self.brightness_threshold}")
        self.get_logger().info(f"Dot duration: {self.dot_duration}s, Dash duration: {self.dash_duration}s")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def ndarray_to_image_msg(self, img):
        """Convert BGR numpy array to ROS2 Image message."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg

    def detect_light(self, frame):
        """
        Detect bright light sources in the frame.
        Returns (light_detected, contours, threshold_image).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, self.brightness_threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > self.min_area:
                return True, contours, thresh

        return False, contours, thresh

    def decode_morse_symbol(self, symbol):
        """Decode a morse code symbol (sequence of dots and dashes)."""
        return self.MORSE_CODE.get(symbol, '?')

    def _frames_to_seconds(self, frames):
        """Convert a frame count difference to seconds using the video's actual FPS."""
        return frames / self.video_fps

    def _flush_symbol(self, word_boundary=False):
        """
        Decode and emit the currently accumulated morse symbol.
        Optionally append a space if we're at a word boundary.
        """
        if self.current_symbol:
            letter = self.decode_morse_symbol(self.current_symbol)
            self.decoded_message += letter
            self.get_logger().info(
                f"Decoded: '{self.current_symbol}' → '{letter}'  |  Message so far: '{self.decoded_message}'"
            )
            self.current_symbol = ""
            self.last_decoded_frame = self.frame_count

        if word_boundary and self.decoded_message and not self.decoded_message.endswith(" "):
            self.decoded_message += " "
            self.get_logger().info("Word gap detected → added space")

        # Always publish updated decoded message
        decoded_msg = String()
        decoded_msg.data = self.decoded_message
        self.pub_decoded.publish(decoded_msg)

    # ------------------------------------------------------------------
    # Main timer callback
    # ------------------------------------------------------------------

    def timer_callback(self):
        """Read one video frame and run the morse state machine."""

        ret, frame = self.cap.read()

        if not ret:
            # End of video — flush any pending symbol, then loop
            self.get_logger().info("End of video reached, looping back to start")
            self._flush_symbol(word_boundary=True)
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.frame_count = 0
            self.is_light_on = False
            self.light_on_frame = None
            self.last_off_frame = None
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to read video frame after reset")
                return

        self.frame_count += 1
        current_frame = self.frame_count

        light_detected, contours, thresh = self.detect_light(frame)

        # ------------------------------------------------------------------
        # State machine
        # ------------------------------------------------------------------

        if light_detected and not self.is_light_on:
            # ── Light just turned ON ──────────────────────────────────────
            self.is_light_on = True
            self.light_on_frame = current_frame

            # If there was a previous signal, check whether the gap since it
            # ended is long enough to be a letter boundary or word boundary.
            if self.last_off_frame is not None:
                gap = self._frames_to_seconds(current_frame - self.last_off_frame)

                if gap >= self.word_gap:
                    # Flush the current symbol (if any) and mark word boundary
                    self._flush_symbol(word_boundary=True)

                elif gap >= self.letter_gap:
                    # Flush the current symbol as a completed letter
                    self._flush_symbol(word_boundary=False)

                # gaps shorter than letter_gap are just inter-symbol gaps;
                # keep accumulating dots/dashes in self.current_symbol

        elif not light_detected and self.is_light_on:
            # ── Light just turned OFF ─────────────────────────────────────
            self.is_light_on = False
            self.last_off_frame = current_frame

            # Use frame-count duration — NOT wall-clock time — so that
            # processing overhead doesn't distort the measurement.
            duration = self._frames_to_seconds(current_frame - self.light_on_frame)

            midpoint = (self.dot_duration + self.dash_duration) / 2.0
            if duration < midpoint:
                self.current_symbol += "."
                self.get_logger().info(f"DOT  ({duration:.3f}s)")
            else:
                self.current_symbol += "-"
                self.get_logger().info(f"DASH ({duration:.3f}s)")

            # Publish the raw morse pattern so far for this letter
            morse_msg = String()
            morse_msg.data = self.current_symbol
            self.pub_morse.publish(morse_msg)

        elif not light_detected and not self.is_light_on:
            # ── Light is still OFF — check if a pending symbol has timed out ──
            if self.current_symbol and self.last_off_frame is not None:
                gap = self._frames_to_seconds(current_frame - self.last_off_frame)

                if gap >= self.word_gap:
                    self._flush_symbol(word_boundary=True)
                elif gap >= self.letter_gap:
                    self._flush_symbol(word_boundary=False)

        # ------------------------------------------------------------------
        # Build and publish debug image
        # ------------------------------------------------------------------

        debug_img = frame.copy()
        cv2.drawContours(debug_img, contours, -1, (0, 255, 255), 2)

        cv2.putText(debug_img, f"Light: {'ON' if light_detected else 'OFF'}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(debug_img, f"Current: {self.current_symbol}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(debug_img, f"Decoded: {self.decoded_message}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Frame: {self.frame_count}/{self.total_frames}",
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

        # Threshold mini-view in bottom-right corner
        thresh_bgr = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        h, w = thresh_bgr.shape[:2]
        small_thresh = cv2.resize(thresh_bgr, (int(w * 0.3), int(h * 0.3)))
        th, tw = small_thresh.shape[:2]
        debug_img[-th - 10:-10, -tw - 10:-10] = small_thresh

        self.pub_debug.publish(self.ndarray_to_image_msg(debug_img))

        # Publish decoded message every frame so subscribers stay current
        if self.decoded_message:
            decoded_msg = String()
            decoded_msg.data = self.decoded_message
            self.pub_decoded.publish(decoded_msg)

        # ------------------------------------------------------------------
        # Periodic FPS log
        # ------------------------------------------------------------------
        self._fps_count += 1
        elapsed = time.time() - self._fps_start
        if elapsed >= 3.0:
            self.get_logger().info(
                f"FPS: {self._fps_count / elapsed:.1f}  |  Decoded: '{self.decoded_message}'"
            )
            self._fps_count = 0
            self._fps_start = time.time()

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = MorseCodeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
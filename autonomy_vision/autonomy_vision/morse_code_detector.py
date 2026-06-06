#!/usr/bin/env python3
"""
Quick Note:
Terminal 1 (Run the detector node with test video): 
ros2 run autonomy_vision morse_code_detector --ros-args -p video_path:~/tasc_ws/src/tasc_autonomy/autonomy_vision/test/hello_morse.mp4
Terminal 2 (View the decoded message): 
ros2 topic echo /morse_decoded
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
import time


class MorseCodeDetector(Node):
    """
    ROS2 node that detects and decodes morse code signals from a video file.

    Uses a two-pass approach:
      Pass 1 — Calibration: scans the entire video, collects all on/off
               durations, and automatically finds dot/dash and gap thresholds
               by looking for the largest natural breaks in the distributions.
      Pass 2 — Decoding: re-reads the video using the discovered thresholds.
    """

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

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('video_path', '')
        self.declare_parameter('brightness_threshold', 200)
        self.declare_parameter('min_area', 100)
        self.declare_parameter('playback_fps', 10.0)

        self.video_path           = self.get_parameter('video_path').value
        self.brightness_threshold = self.get_parameter('brightness_threshold').value
        self.min_area             = self.get_parameter('min_area').value
        self.playback_fps         = self.get_parameter('playback_fps').value

        if not self.video_path:
            self.get_logger().error("No video path specified! Use parameter 'video_path'")
            raise ValueError("video_path parameter is required")

        # ── Open video ────────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {self.video_path}")
            raise ValueError(f"Cannot open video file: {self.video_path}")

        self.video_fps    = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        self.get_logger().info(
            f"Opened '{self.video_path}'  |  {self.video_fps:.1f} fps  |  {self.total_frames} frames"
        )

        # ── Calibration pass ──────────────────────────────────────────────
        self.dot_dash_threshold   = None
        self.symbol_gap_threshold = None
        self.letter_gap_threshold = None
        self._calibrate()

        # Rewind for decoding pass
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

        # ── Decoding state ────────────────────────────────────────────────
        self.frame_count        = 0
        self.is_light_on        = False
        self.light_on_frame     = None
        self.last_off_frame     = None
        self.current_symbol     = ""
        self.decoded_message    = ""
        self.last_decoded_frame = None

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_morse   = self.create_publisher(String, "/morse_code", 10)
        self.pub_decoded = self.create_publisher(String, "/morse_decoded", 10)
        self.pub_debug   = self.create_publisher(Image,  "/morse_debug_image", 10)

        self.timer = self.create_timer(1.0 / self.playback_fps, self.timer_callback)

        self._fps_start = time.time()
        self._fps_count = 0

    # ──────────────────────────────────────────────────────────────────────
    # Calibration
    # ──────────────────────────────────────────────────────────────────────

    def _scan_durations(self):
        """
        Scan the full video once and return two lists:
          on_durations  — how long each light flash lasted (seconds)
          off_durations — how long each dark gap lasted (seconds)
        """
        on_durations  = []
        off_durations = []

        is_on     = False
        on_start  = None
        off_start = None
        frame_num = 0

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            frame_num += 1
            light = self._detect_light_bool(frame)

            if light and not is_on:
                if off_start is not None:
                    off_durations.append((frame_num - off_start) / self.video_fps)
                is_on    = True
                on_start = frame_num

            elif not light and is_on:
                on_durations.append((frame_num - on_start) / self.video_fps)
                is_on     = False
                off_start = frame_num

        # Flush a trailing flash
        if is_on and on_start is not None:
            on_durations.append((frame_num - on_start) / self.video_fps)

        return on_durations, off_durations

    @staticmethod
    def _largest_gap_threshold(durations, n_splits=1):
        """
        Given a list of durations, find the n_splits largest gaps between
        consecutive sorted values and return their midpoints as thresholds.
        Returns a sorted list of n_splits threshold values.
        """
        if len(durations) < 2:
            return [durations[0] * 2.0] * n_splits if durations else [0.5] * n_splits

        arr  = sorted(durations)
        gaps = sorted(
            [(arr[i+1] - arr[i], i) for i in range(len(arr) - 1)],
            reverse=True
        )

        thresholds = []
        for _, idx in gaps[:n_splits]:
            thresholds.append((arr[idx] + arr[idx + 1]) / 2.0)

        return sorted(thresholds)

    def _calibrate(self):
        """Run one full pass over the video to auto-detect timing thresholds."""
        self.get_logger().info("── Calibration pass starting ──")
        on_durations, off_durations = self._scan_durations()

        if not on_durations:
            self.get_logger().error(
                "Calibration failed: no light signals detected. "
                "Check brightness_threshold and min_area."
            )
            self.dot_dash_threshold   = 0.2
            self.symbol_gap_threshold = 0.15
            self.letter_gap_threshold = 0.5
            return

        self.get_logger().info(
            f"  Found {len(on_durations)} flashes, {len(off_durations)} gaps"
        )
        self.get_logger().info(
            f"  ON  durations: {[round(x,3) for x in sorted(on_durations)]}"
        )
        self.get_logger().info(
            f"  OFF durations: {[round(x,3) for x in sorted(off_durations)]}"
        )

        # ── Dot / dash threshold ──────────────────────────────────────────
        unique_on = len(set(round(d, 3) for d in on_durations))
        if unique_on == 1:
            # All pulses the same length — treat all as dots
            self.dot_dash_threshold = on_durations[0] * 2.0
        else:
            self.dot_dash_threshold = self._largest_gap_threshold(on_durations, n_splits=1)[0]

        # ── Gap thresholds ────────────────────────────────────────────────
        # Off-durations form up to 3 clusters:
        #   short  → inter-symbol gap  (between dots/dashes within a letter)
        #   medium → inter-letter gap
        #   long   → inter-word gap
        unique_off = len(set(round(d, 3) for d in off_durations))
        if len(off_durations) >= 3 and unique_off >= 3:
            thresholds = self._largest_gap_threshold(off_durations, n_splits=2)
            self.symbol_gap_threshold = thresholds[0]
            self.letter_gap_threshold = thresholds[1]
        elif len(off_durations) >= 2 and unique_off >= 2:
            thresholds = self._largest_gap_threshold(off_durations, n_splits=1)
            self.symbol_gap_threshold = thresholds[0] * 0.5
            self.letter_gap_threshold = thresholds[0]
        else:
            # Derive proportionally: symbol gap ≈ 1 dot, letter gap ≈ 3 dots
            dot_est = min(on_durations)
            self.symbol_gap_threshold = dot_est * 1.5
            self.letter_gap_threshold = dot_est * 4.0

        self.get_logger().info("── Calibration results ──────────────────────")
        self.get_logger().info(f"  dot/dash threshold : {self.dot_dash_threshold:.3f}s")
        self.get_logger().info(f"  symbol gap max     : {self.symbol_gap_threshold:.3f}s")
        self.get_logger().info(f"  letter gap max     : {self.letter_gap_threshold:.3f}s")
        self.get_logger().info(f"  word gap min       : {self.letter_gap_threshold * 1.5:.3f}s")
        self.get_logger().info("─────────────────────────────────────────────")

    # ──────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────

    def _detect_light_bool(self, frame):
        """Return True if a significant bright blob is present."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, self.brightness_threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(cv2.contourArea(c) > self.min_area for c in contours)

    def detect_light(self, frame):
        """Return (light_bool, contours, threshold_image) for debug display."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, self.brightness_threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        light = any(cv2.contourArea(c) > self.min_area for c in contours)
        return light, contours, thresh

    def _frames_to_seconds(self, frames):
        return frames / self.video_fps

    def decode_morse_symbol(self, symbol):
        return self.MORSE_CODE.get(symbol, '?')

    def _flush_symbol(self, word_boundary=False):
        """Decode the accumulated morse symbol and optionally add a word space."""
        if self.current_symbol:
            letter = self.decode_morse_symbol(self.current_symbol)
            self.decoded_message += letter
            self.get_logger().info(
                f"Decoded: '{self.current_symbol}' → '{letter}'  |  "
                f"Message: '{self.decoded_message}'"
            )
            self.current_symbol     = ""
            self.last_decoded_frame = self.frame_count

        if word_boundary and self.decoded_message and not self.decoded_message.endswith(" "):
            self.decoded_message += " "
            self.get_logger().info("Word gap → space added")

        decoded_msg      = String()
        decoded_msg.data = self.decoded_message
        self.pub_decoded.publish(decoded_msg)

    def ndarray_to_image_msg(self, img):
        msg            = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height, msg.width = img.shape[:2]
        msg.encoding   = "bgr8"
        msg.step       = msg.width * 3
        msg.data       = img.tobytes()
        return msg

    # ──────────────────────────────────────────────────────────────────────
    # Main decode loop
    # ──────────────────────────────────────────────────────────────────────

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().info("End of video — looping")
            self._flush_symbol(word_boundary=True)
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.frame_count    = 0
            self.is_light_on    = False
            self.light_on_frame = None
            self.last_off_frame = None
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to read frame after loop reset")
                return

        self.frame_count += 1
        current_frame = self.frame_count

        light_detected, contours, thresh = self.detect_light(frame)

        word_gap_threshold = self.letter_gap_threshold * 1.5

        # ── State machine ────────────────────────────────────────────────

        if light_detected and not self.is_light_on:
            # Light just turned ON
            self.is_light_on    = True
            self.light_on_frame = current_frame

            if self.last_off_frame is not None:
                gap = self._frames_to_seconds(current_frame - self.last_off_frame)

                if gap >= word_gap_threshold:
                    self._flush_symbol(word_boundary=True)
                elif gap >= self.letter_gap_threshold:
                    self._flush_symbol(word_boundary=False)
                # else: inter-symbol gap, keep accumulating

        elif not light_detected and self.is_light_on:
            # Light just turned OFF
            self.is_light_on    = False
            self.last_off_frame = current_frame

            duration = self._frames_to_seconds(current_frame - self.light_on_frame)

            if duration < self.dot_dash_threshold:
                self.current_symbol += "."
                self.get_logger().info(f"DOT  ({duration:.3f}s)")
            else:
                self.current_symbol += "-"
                self.get_logger().info(f"DASH ({duration:.3f}s)")

            morse_msg      = String()
            morse_msg.data = self.current_symbol
            self.pub_morse.publish(morse_msg)

        elif not light_detected and not self.is_light_on:
            # Still dark — flush pending symbol if gap has exceeded letter/word threshold
            if self.current_symbol and self.last_off_frame is not None:
                gap = self._frames_to_seconds(current_frame - self.last_off_frame)
                if gap >= word_gap_threshold:
                    self._flush_symbol(word_boundary=True)
                elif gap >= self.letter_gap_threshold:
                    self._flush_symbol(word_boundary=False)

        # ── Debug image ──────────────────────────────────────────────────

        debug_img = frame.copy()
        cv2.drawContours(debug_img, contours, -1, (0, 255, 255), 2)

        cv2.putText(debug_img, f"Light: {'ON' if light_detected else 'OFF'}",
                    (10, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0),   2)
        cv2.putText(debug_img, f"Current: {self.current_symbol}",
                    (10, 60),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(debug_img, f"Decoded: {self.decoded_message}",
                    (10, 90),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Frame: {self.frame_count}/{self.total_frames}",
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        cv2.putText(debug_img,
                    f"dot<{self.dot_dash_threshold:.3f}s  "
                    f"ltr<{self.letter_gap_threshold:.3f}s  "
                    f"wrd>{word_gap_threshold:.3f}s",
                    (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        thresh_bgr   = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        h, w         = thresh_bgr.shape[:2]
        small_thresh = cv2.resize(thresh_bgr, (int(w * 0.3), int(h * 0.3)))
        th, tw       = small_thresh.shape[:2]
        debug_img[-th - 10:-10, -tw - 10:-10] = small_thresh

        self.pub_debug.publish(self.ndarray_to_image_msg(debug_img))

        if self.decoded_message:
            decoded_msg      = String()
            decoded_msg.data = self.decoded_message
            self.pub_decoded.publish(decoded_msg)

        # ── FPS log ──────────────────────────────────────────────────────
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

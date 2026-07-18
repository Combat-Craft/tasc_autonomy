#!/usr/bin/env python3

import os
import time
import collections

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge

try:
    import psutil
    _HAVE_PSUTIL = True
except ImportError:
    _HAVE_PSUTIL = False


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

    '.-.-.-': '.', '--..--': ',', '..--..': '?',
    '.----.': "'", '-.-.--': '!', '-..-.': '/',
    '-.--.': '(', '-.--.-': ')',
    '.-...': '&', '---...': ':',
    '-.-.-.': ';', '-...-': '=',
    '.-.-.': '+', '-....-': '-',
    '..--.-': '_', '.-..-.': '"',
    '.--.-.': '@',
}


class MorseCodeDetector(Node):

    # 18 WPM PARIS timing (ITU-R M.1677-1)
    DIT_LENGTH = 0.0667
    DASH_LENGTH = 0.2  # 3 dits

    # Standard Morse spacing
    INTRA_CHAR_GAP = DIT_LENGTH        # 1 dit
    INTER_CHAR_GAP = DIT_LENGTH * 3    # 3 dits
    INTER_WORD_GAP = DIT_LENGTH * 7    # 7 dits

    # Decision thresholds sit at the midpoint between target durations
    DOT_DASH_THRESHOLD = (DIT_LENGTH + DASH_LENGTH) / 2          # ~0.133s
    CHAR_GAP_THRESHOLD = (INTRA_CHAR_GAP + INTER_CHAR_GAP) / 2   # ~0.133s
    WORD_GAP_THRESHOLD = (INTER_CHAR_GAP + INTER_WORD_GAP) / 2   # ~0.333s

    # Ignore pulses/gaps shorter than this fraction of a dit to reject noise
    MIN_VALID_DURATION = DIT_LENGTH * 0.4

    # Tuned to 1 to eliminate frame processing lag in VM/Network streams
    DEBOUNCE_FRAMES = 1


    def __init__(self):
        super().__init__("morse_code_detector")

        # Parameters
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("threshold_percentile", 85.0)
        self.declare_parameter("min_bright_fraction", 0.01)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("log_cpu_stats", True)

        self.thresh_pct = self.get_parameter("threshold_percentile").value
        self.min_bright_frac = self.get_parameter("min_bright_fraction").value
        publish_hz = self.get_parameter("publish_hz").value
        log_cpu_stats = self.get_parameter("log_cpu_stats").value

        # Warn if the configured capture rate is too slow to resolve a single dit
        min_recommended_hz = 3.0 / self.DIT_LENGTH  # ~45 Hz
        if publish_hz < min_recommended_hz:
            self.get_logger().warn(
                f"publish_hz={publish_hz:.1f} is likely too low to "
                f"reliably resolve {self.DIT_LENGTH*1000:.1f}ms dits "
                f"(recommend >= {min_recommended_hz:.0f} Hz). "
                f"Override with --ros-args -p publish_hz:=60.0"
            )

        camera_index_param = self.get_parameter("camera_index").value
        is_rtsp = (
            isinstance(camera_index_param, str)
            and (
                camera_index_param.startswith("rtsp://")
                or camera_index_param.startswith("rtspt://")
            )
        )

        if is_rtsp:
            camera_device = camera_index_param
            # Force TCP transport for the RTSP session to avoid dropped network frames
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
            self.cap = cv2.VideoCapture(camera_device, cv2.CAP_FFMPEG)
        else:
            if isinstance(camera_index_param, str) and camera_index_param.startswith("/dev/"):
                camera_device = camera_index_param
            else:
                camera_device = f"/dev/video{camera_index_param}"

            self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, max(publish_hz, 30.0))

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera source {camera_device}")
        else:
            self.get_logger().info(
                f"Opened camera source {camera_device} "
                f"at {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x"
                f"{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}"
            )

        # ROS interfaces
        self.pub_morse = self.create_publisher(String, "/morse_code", 10)
        self.pub_decoded = self.create_publisher(String, "/morse_decoded", 10)
        self.pub_debug = self.create_publisher(Image, "/morse_debug_image", 10)

        self.reset_sub = self.create_subscription(
            String, "/morse_reset", self._on_reset, 10
        )

        self.bridge = CvBridge()

        # State tracking
        self.is_on = False
        self.on_time = None
        self.off_time = None

        self._raw_hist = collections.deque(maxlen=self.DEBOUNCE_FRAMES)
        self.current_sym = ""
        self.message = ""

        # Track recent peak brightness values to establish accurate dynamic thresholds
        self._peak_hist = collections.deque(maxlen=90)
        self._callback_durations = collections.deque(maxlen=100)

        # Timer-driven capture loop
        timer_period = 1.0 / publish_hz
        self.timer = self.create_timer(timer_period, self.capture_callback)

        # CPU Performance tracking
        self._proc = None
        if log_cpu_stats and _HAVE_PSUTIL:
            self._proc = psutil.Process(os.getpid())
            self._proc.cpu_percent(interval=None)
            self.cpu_log_timer = self.create_timer(2.0, self._log_cpu_stats)
        elif log_cpu_stats and not _HAVE_PSUTIL:
            self.get_logger().warn(
                "log_cpu_stats is enabled but psutil is not installed."
            )

        self.get_logger().info(f"Morse detector reading {camera_device} at {publish_hz:.1f} Hz")


    def _log_cpu_stats(self):
        if self._proc is None:
            return

        cpu_pct = self._proc.cpu_percent(interval=None)
        mem_mb = self._proc.memory_info().rss / (1024 * 1024)

        if self._callback_durations:
            avg_ms = 1000.0 * sum(self._callback_durations) / len(self._callback_durations)
            max_ms = 1000.0 * max(self._callback_durations)
        else:
            avg_ms = 0.0
            max_ms = 0.0

        level = self.get_logger().warn if cpu_pct > 80.0 else self.get_logger().info
        level(
            f"[perf] cpu={cpu_pct:.1f}% mem={mem_mb:.1f}MB "
            f"callback_avg={avg_ms:.1f}ms callback_max={max_ms:.1f}ms"
        )


    def _on_reset(self, msg):
        self.message = ""
        self.current_sym = ""
        self.get_logger().info("Decoded message reset")

        out = String()
        out.data = self.message
        self.pub_decoded.publish(out)


    def capture_callback(self):
        if not self.cap.isOpened():
            return

        cb_start = time.monotonic()
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn("Failed to read frame from camera")
            return

        now = time.monotonic()
        self.process_frame(frame, now)
        self._callback_durations.append(time.monotonic() - cb_start)


    def _debounced_state(self, raw_on):
        self._raw_hist.append(raw_on)
        if len(self._raw_hist) < self._raw_hist.maxlen:
            return raw_on
        on_votes = sum(self._raw_hist)
        return on_votes > (len(self._raw_hist) / 2)


    def process_frame(self, frame, now):
        # Downscale and grayscale for efficient processing
        small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        # Track the absolute brightest pixel in the frame
        peak_brightness = float(np.max(gray))
        self._peak_hist.append(peak_brightness)

        # Compute adaptive threshold halfway between minimum and maximum peak baseline bounds
        val_min = min(self._peak_hist)
        val_max = max(self._peak_hist)
        adaptive_thresh = (val_min + val_max) / 2

        # Filter out minor environmental light noise unless values shift by 40 gray levels
        if (val_max - val_min) > 40.0:
            raw_light_on = peak_brightness > adaptive_thresh
        else:
            raw_light_on = False

        light_on = self._debounced_state(raw_light_on)

        # State transition: OFF -> ON
        if light_on and not self.is_on:
            self.is_on = True
            self.on_time = now

            if self.off_time:
                gap = now - self.off_time
                if gap >= self.MIN_VALID_DURATION:
                    self._handle_gap(gap)

        # State transition: ON -> OFF
        elif not light_on and self.is_on:
            self.is_on = False
            self.off_time = now

            duration = now - self.on_time
            if duration >= self.MIN_VALID_DURATION:
                self._handle_pulse(duration)

        # Continuous OFF state: Check for word boundaries
        elif not light_on and not self.is_on:
            if self.off_time and self.current_sym:
                gap = now - self.off_time
                if gap > self.WORD_GAP_THRESHOLD:
                    self._flush(word=True)

        self.publish_debug(frame, light_on, peak_brightness)


    def _handle_pulse(self, duration):
        if duration < self.DOT_DASH_THRESHOLD:
            self.current_sym += "."
        else:
            self.current_sym += "-"

        msg = String()
        msg.data = self.current_sym
        self.pub_morse.publish(msg)


    def _handle_gap(self, gap):
        if gap > self.WORD_GAP_THRESHOLD:
            self._flush(word=True)
        elif gap > self.CHAR_GAP_THRESHOLD:
            self._flush(word=False)


    def _flush(self, word=False):
        had_pending = bool(self.current_sym)

        if self.current_sym:
            letter = MORSE_CODE.get(self.current_sym, "?")
            self.message += letter
            self.current_sym = ""

        if word and (had_pending or not self.message.endswith(" ")):
            self.message += " "

        msg = String()
        msg.data = self.message
        self.pub_decoded.publish(msg)


    def publish_debug(self, frame, light_on, peak_val):
        debug = frame.copy()
        color = (0, 255, 0) if light_on else (0, 0, 255)

        cv2.putText(
            debug,
            f"ON={light_on} peak={peak_val:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )

        msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        self.pub_debug.publish(msg)


    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = MorseCodeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

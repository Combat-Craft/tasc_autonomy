#!/usr/bin/env python3

import time
import collections

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge


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

    # Standard Morse spacing (measured from end of one element to start of next):
    #   intra-character gap (between symbols in the same letter) = 1 dit
    #   inter-character gap (between letters)                    = 3 dits
    #   inter-word gap                                           = 7 dits
    INTRA_CHAR_GAP = DIT_LENGTH        # 1 dit
    INTER_CHAR_GAP = DIT_LENGTH * 3    # 3 dits
    INTER_WORD_GAP = DIT_LENGTH * 7    # 7 dits

    # Decision thresholds sit at the midpoint between the two things
    # they're distinguishing, same approach as the original dot/dash split.
    DOT_DASH_THRESHOLD = (DIT_LENGTH + DASH_LENGTH) / 2          # ~0.133s
    CHAR_GAP_THRESHOLD = (INTRA_CHAR_GAP + INTER_CHAR_GAP) / 2   # ~0.133s
    WORD_GAP_THRESHOLD = (INTER_CHAR_GAP + INTER_WORD_GAP) / 2   # ~0.333s

    # Ignore pulses/gaps shorter than this fraction of a dit — treated as
    # camera/lighting noise rather than a real signal transition.
    MIN_VALID_DURATION = DIT_LENGTH * 0.4

    # Number of consecutive raw frames required to agree before the
    # debounced on/off state is allowed to flip.
    DEBOUNCE_FRAMES = 5


    def __init__(self):

        super().__init__(
            "morse_code_detector"
        )

        # Parameters

        self.declare_parameter(
            "camera_index",
            0
        )

        self.declare_parameter(
            "threshold_percentile",
            85.0
        )

        self.declare_parameter(
            "min_bright_fraction",
            0.01
        )

        self.declare_parameter(
            "publish_hz",
            10.0
        )


        camera_index = self.get_parameter(
            "camera_index"
        ).value

        self.thresh_pct = self.get_parameter(
            "threshold_percentile"
        ).value

        self.min_bright_frac = self.get_parameter(
            "min_bright_fraction"
        ).value

        publish_hz = self.get_parameter(
            "publish_hz"
        ).value


        # Warn loudly if the configured capture rate is too slow to
        # resolve a single dit. Below ~3 samples per dit, dots become
        # unreliable to detect at all regardless of decoding logic.
        min_recommended_hz = 3.0 / self.DIT_LENGTH  # ~45 Hz

        if publish_hz < min_recommended_hz:

            self.get_logger().warn(
                f"publish_hz={publish_hz:.1f} is likely too low to "
                f"reliably resolve {self.DIT_LENGTH*1000:.1f}ms dits "
                f"(recommend >= {min_recommended_hz:.0f} Hz). "
                f"Override with --ros-args -p publish_hz:=60.0"
            )


        # Direct camera capture (no ROS image topic / usb_cam needed)

        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, max(publish_hz, 30.0))

        if not self.cap.isOpened():

            self.get_logger().error(
                f"Failed to open camera index {camera_index}"
            )

        else:

            self.get_logger().info(
                f"Opened camera index {camera_index} "
                f"at {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x"
                f"{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}"
            )


        # ROS interfaces

        self.pub_morse = self.create_publisher(
            String,
            "/morse_code",
            10
        )

        self.pub_decoded = self.create_publisher(
            String,
            "/morse_decoded",
            10
        )

        self.pub_debug = self.create_publisher(
            Image,
            "/morse_debug_image",
            10
        )


        self.bridge = CvBridge()


        # Debounced on/off state
        self.is_on = False
        self.on_time = None
        self.off_time = None

        # Raw (pre-debounce) light readings, most recent last
        self._raw_hist = collections.deque(maxlen=self.DEBOUNCE_FRAMES)

        self.current_sym = ""
        self.message = ""

        self._bright_hist = collections.deque(
            maxlen=60
        )


        # Timer-driven capture loop, replacing the old image-topic
        # subscription. Runs at publish_hz.
        timer_period = 1.0 / publish_hz

        self.timer = self.create_timer(
            timer_period,
            self.capture_callback
        )


        self.get_logger().info(
            f"Morse detector reading camera index {camera_index} "
            f"at {publish_hz:.1f} Hz"
        )


    def capture_callback(self):

        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()

        if not ret or frame is None:

            self.get_logger().warn(
                "Failed to read frame from camera"
            )

            return

        # time.monotonic() taken immediately after a synchronous cap.read()
        # is a reasonably accurate capture timestamp here — there's no ROS
        # transport/queueing in between to introduce extra jitter, unlike
        # the old topic-subscription version.
        now = time.monotonic()

        self.process_frame(frame, now)


    def _debounced_state(self, raw_on):
        """Majority-vote the last N raw readings to reject single-frame
        camera/lighting flicker before it reaches the edge-detection logic."""

        self._raw_hist.append(raw_on)

        if len(self._raw_hist) < self._raw_hist.maxlen:
            # Not enough history yet — fall back to raw reading.
            return raw_on

        on_votes = sum(self._raw_hist)

        return on_votes > (len(self._raw_hist) / 2)


    def process_frame(self, frame, now):

        # Downscale before analysis — the percentile/mean calculations
        # don't need full resolution, and this keeps the callback fast
        # enough to avoid falling behind on frames (which is what causes
        # the large timing gaps that corrupt short dit/dash pulses).
        small = cv2.resize(
            frame,
            (160, 90),
            interpolation=cv2.INTER_AREA
        )

        gray = cv2.cvtColor(
            small,
            cv2.COLOR_BGR2GRAY
        )


        thresh_val = float(
            np.percentile(
                gray,
                self.thresh_pct
            )
        )


        self._bright_hist.append(
            thresh_val
        )


        adaptive_thresh = (
            float(np.mean(self._bright_hist))
            * 1.05
        )

        adaptive_thresh = max(
            adaptive_thresh,
            80.0
        )


        bright_mask = gray > adaptive_thresh

        bright_frac = bright_mask.mean()

        # Hysteresis (Schmitt trigger): use a higher bar to turn ON and a
        # lower bar to turn OFF. Without this, brightness hovering near a
        # single cutoff (screen refresh flicker, auto-exposure hunting,
        # monitor PWM backlight) can cross that one line many times per
        # second, producing many spurious short pulses instead of one
        # clean on/off segment.
        on_bar = self.min_bright_frac * 1.5
        off_bar = self.min_bright_frac * 0.5

        if self.is_on:
            raw_light_on = bright_frac >= off_bar
        else:
            raw_light_on = bright_frac >= on_bar

        light_on = self._debounced_state(raw_light_on)


        if light_on and not self.is_on:

            self.is_on = True
            self.on_time = now


            if self.off_time:

                gap = now - self.off_time

                if gap >= self.MIN_VALID_DURATION:

                    self._handle_gap(gap)


        elif not light_on and self.is_on:

            self.is_on = False
            self.off_time = now


            duration = (
                now - self.on_time
            )

            if duration >= self.MIN_VALID_DURATION:

                self._handle_pulse(duration)


        elif not light_on and not self.is_on:

            if self.off_time and self.current_sym:

                gap = now - self.off_time

                if gap > self.WORD_GAP_THRESHOLD:

                    self._flush(
                        word=True
                    )


        self.publish_debug(
            frame,
            light_on,
            bright_frac
        )



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

            self._flush(
                word=True
            )

        elif gap > self.CHAR_GAP_THRESHOLD:

            self._flush(
                word=False
            )

        # else: gap is just the normal intra-character space between
        # symbols of the same letter — do nothing, keep accumulating.



    def _flush(self, word=False):

        if self.current_sym:

            letter = MORSE_CODE.get(
                self.current_sym,
                "?"
            )

            self.message += letter

            self.current_sym = ""


        if word:

            self.message += " "


        msg = String()

        msg.data = self.message

        self.pub_decoded.publish(msg)



    def publish_debug(
        self,
        frame,
        light_on,
        bright_frac
    ):


        debug = frame.copy()


        color = (
            (0,255,0)
            if light_on
            else (0,0,255)
        )


        cv2.putText(
            debug,
            f"ON={light_on} bright={bright_frac:.3f}",
            (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )


        msg = self.bridge.cv2_to_imgmsg(
            debug,
            encoding="bgr8"
        )


        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = "camera"


        self.pub_debug.publish(
            msg
        )


    def destroy_node(self):

        if self.cap is not None and self.cap.isOpened():

            self.cap.release()

        super().destroy_node()



def main():

    rclpy.init()

    node = MorseCodeDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()



if __name__ == "__main__":

    main()
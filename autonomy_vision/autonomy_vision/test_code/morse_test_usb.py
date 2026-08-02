import argparse
import threading
import time
import collections

import cv2
import numpy as np

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

# ITU-R M.1677-1, 18 WPM PARIS reference
DIT_LENGTH = 0.0667
DASH_LENGTH = DIT_LENGTH * 3
INTRA_CHAR_GAP = DIT_LENGTH
INTER_CHAR_GAP = DIT_LENGTH * 3
INTER_WORD_GAP = DIT_LENGTH * 7

DOT_DASH_THRESHOLD = (DIT_LENGTH + DASH_LENGTH) / 2
CHAR_GAP_THRESHOLD = (INTRA_CHAR_GAP + INTER_CHAR_GAP) / 2
WORD_GAP_THRESHOLD = (INTER_CHAR_GAP + INTER_WORD_GAP) / 2

MIN_VALID_DURATION = DIT_LENGTH * 0.4
DEBOUNCE_FRAMES = 1  # Set to 1 to eliminate frames-based lag in VM environments


class LatestFrameGrabber:
    """
    Continuously reads frames from a VideoCapture on a background thread
    and always exposes only the most recently captured one.
    """
    def __init__(self, cap):
        self.cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._ok = False
        self._stopped = False
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self):
        while not self._stopped:
            ret, frame = self.cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
                    self._ok = True
            else:
                time.sleep(0.001)

    def read(self):
        with self._lock:
            return self._ok, self._frame

    def stop(self):
        self._stopped = True
        self._thread.join(timeout=1.0)


class MorseTester:

    def __init__(self, threshold_percentile=None, min_bright_fraction=None):
        self.is_on = False
        self.on_time = None
        self.off_time = None

        # Track recent peak brightness values to establish baseline vs flash levels
        self._peak_hist = collections.deque(maxlen=90)

        self.current_sym = ""
        self.message = ""

    def reset(self):
        self.message = ""
        self.current_sym = ""
        print("\n--- message reset ---\n")

    def process_frame(self, frame, now):
        # Downsample and convert to grayscale for faster processing
        small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        # Track the absolute brightest pixel in the frame
        peak_brightness = float(np.max(gray))
        self._peak_hist.append(peak_brightness)

        # Set dynamic threshold between the lowest and highest peaks seen recently
        val_min = min(self._peak_hist)
        val_max = max(self._peak_hist)
        adaptive_thresh = (val_min + val_max) / 2

        # Ignore tiny variations (noise) unless the frame peaks span at least 40 gray levels
        if (val_max - val_min) > 40.0:
            light_on = peak_brightness > adaptive_thresh
        else:
            light_on = False

        # State transition: OFF -> ON
        if light_on and not self.is_on:
            self.is_on = True
            self.on_time = now
            if self.off_time:
                gap = now - self.off_time
                if gap >= MIN_VALID_DURATION:
                    self._handle_gap(gap)

        # State transition: ON -> OFF
        elif not light_on and self.is_on:
            self.is_on = False
            self.off_time = now
            duration = now - self.on_time
            if duration >= MIN_VALID_DURATION:
                self._handle_pulse(duration)

        # Continuous OFF state: Check for word boundaries
        elif not light_on and not self.is_on:
            if self.off_time and self.current_sym:
                gap = now - self.off_time
                if gap > WORD_GAP_THRESHOLD:
                    self._flush(word=True)

        # Returning peak_brightness in place of bright_frac to preserve main loop compatibility
        return light_on, peak_brightness

    def _handle_pulse(self, duration):
        symbol = "." if duration < DOT_DASH_THRESHOLD else "-"
        self.current_sym += symbol
        print(f"  pulse: {symbol} ({duration*1000:.0f}ms) -> current_sym='{self.current_sym}'")

    def _handle_gap(self, gap):
        if gap > WORD_GAP_THRESHOLD:
            self._flush(word=True)
        elif gap > CHAR_GAP_THRESHOLD:
            self._flush(word=False)

    def _flush(self, word=False):
        had_pending = bool(self.current_sym)

        if self.current_sym:
            letter = MORSE_CODE.get(self.current_sym, "?")
            self.message += letter
            print(f"  letter: '{self.current_sym}' -> '{letter}'")
            self.current_sym = ""

        if word and (had_pending or not self.message.endswith(" ")):
            self.message += " "

        print(f"DECODED: {self.message!r}")


def open_camera(camera_arg, width, height, fps):
    try:
        camera_index = int(camera_arg)
        cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    except ValueError:
        cap = cv2.VideoCapture(camera_arg, cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    return cap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="0",
                        help="Camera index or device path. Default: 0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=60.0,
                        help="Requested capture FPS (recommended >= 45)")
    parser.add_argument("--threshold-percentile", type=float, default=85.0)
    parser.add_argument("--min-bright-fraction", type=float, default=0.01)
    parser.add_argument("--show", action="store_true",
                        help="Open a live preview window with ON/OFF overlay")
    parser.add_argument("--debug-bright", action="store_true",
                        help="Print raw peak brightness values every frame")

    args = parser.parse_args()

    if args.fps < 3.0 / DIT_LENGTH:
        print(f"WARNING: fps={args.fps} is below the recommended "
              f"{3.0/DIT_LENGTH:.0f} Hz for reliably resolving "
              f"{DIT_LENGTH*1000:.1f}ms dits.\n")

    cap = open_camera(args.camera, args.width, args.height, args.fps)

    if not cap.isOpened():
        print(f"ERROR: could not open camera '{args.camera}'")
        return

    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Opened camera '{args.camera}' at {actual_w:.0f}x{actual_h:.0f}")
    print("Using threaded latest-frame grabber (avoids stale-buffer issues)")
    print("Reading frames... Ctrl+C to quit"
          + (" | ESC to quit, 'r' to reset (preview window)" if args.show else ""))
    print()

    grabber = LatestFrameGrabber(cap)
    time.sleep(0.3)

    tester = MorseTester()

    frame_count = 0
    last_fps_check = time.monotonic()
    last_processed_frame_id = None

    try:
        while True:
            ret, frame = grabber.read()

            if not ret or frame is None:
                time.sleep(0.001)
                continue

            frame_id = id(frame)
            if frame_id == last_processed_frame_id:
                time.sleep(0.001)
                continue
            last_processed_frame_id = frame_id

            now = time.monotonic()
            light_on, bright_frac = tester.process_frame(frame, now)

            if args.debug_bright:
                print(f"    peak_brightness={bright_frac:.1f} light_on={light_on}")

            frame_count += 1
            if now - last_fps_check >= 5.0:
                actual_fps = frame_count / (now - last_fps_check)
                print(f"[{actual_fps:.1f} fps actual]")
                frame_count = 0
                last_fps_check = now

            if args.show:
                debug = frame.copy()
                color = (0, 255, 0) if light_on else (0, 0, 255)

                cv2.putText(
                    debug,
                    f"ON={light_on} peak={bright_frac:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2
                )
                cv2.putText(
                    debug,
                    f"msg: {tester.message}",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1
                )

                cv2.imshow("Morse Test", debug)
                key = cv2.waitKey(1) & 0xFF

                if key == 27:  # ESC
                    break
                elif key == ord('r'):
                    tester.reset()

    except KeyboardInterrupt:
        print("\nStopped.")

    grabber.stop()
    cap.release()

    if args.show:
        cv2.destroyAllWindows()

    print(f"\nFinal decoded message: {tester.message!r}")


if __name__ == "__main__":
    main()
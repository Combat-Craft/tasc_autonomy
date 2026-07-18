#THIS is for my own testing to try things out at home with a USB camer :D

#pip3 install opencv-python numpy --break-system-packages   # if you don't have them
#python3 morse_test_usb.py --camera 0
# OR for a live preview thing 
# python3 morse_test_usb.py --camera 0 --show


import argparse
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
DEBOUNCE_FRAMES = 5


class MorseTester:

    def __init__(self, threshold_percentile, min_bright_fraction):

        self.thresh_pct = threshold_percentile
        self.min_bright_frac = min_bright_fraction

        self.is_on = False
        self.on_time = None
        self.off_time = None

        self._raw_hist = collections.deque(maxlen=DEBOUNCE_FRAMES)
        self._bright_hist = collections.deque(maxlen=60)

        self.current_sym = ""
        self.message = ""

    def reset(self):
        self.message = ""
        self.current_sym = ""
        print("\n--- message reset ---\n")

    def _debounced_state(self, raw_on):

        self._raw_hist.append(raw_on)

        if len(self._raw_hist) < self._raw_hist.maxlen:
            return raw_on

        on_votes = sum(self._raw_hist)
        return on_votes > (len(self._raw_hist) / 2)

    def process_frame(self, frame, now):

        small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        thresh_val = float(np.percentile(gray, self.thresh_pct))
        self._bright_hist.append(thresh_val)

        adaptive_thresh = float(np.mean(self._bright_hist)) * 1.05
        adaptive_thresh = max(adaptive_thresh, 80.0)

        bright_mask = gray > adaptive_thresh
        bright_frac = bright_mask.mean()

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
                if gap >= MIN_VALID_DURATION:
                    self._handle_gap(gap)

        elif not light_on and self.is_on:

            self.is_on = False
            self.off_time = now

            duration = now - self.on_time
            if duration >= MIN_VALID_DURATION:
                self._handle_pulse(duration)

        elif not light_on and not self.is_on:

            if self.off_time and self.current_sym:
                gap = now - self.off_time
                if gap > WORD_GAP_THRESHOLD:
                    self._flush(word=True)

        return light_on, bright_frac

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
        # A /dev/videoN path or similar string
        cap = cv2.VideoCapture(camera_arg, cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    return cap


def main():

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="0",
                         help="Camera index (e.g. 0) or device path (e.g. /dev/video2). Default: 0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=60.0,
                         help="Requested capture FPS (default: 60, recommended >= 45 for clean dit detection)")
    parser.add_argument("--threshold-percentile", type=float, default=85.0)
    parser.add_argument("--min-bright-fraction", type=float, default=0.01)
    parser.add_argument("--show", action="store_true",
                         help="Open a live preview window with ON/OFF overlay")

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
    print("Reading frames... Ctrl+C to quit"
          + (" | ESC to quit, 'r' to reset (preview window)" if args.show else ""))
    print()

    tester = MorseTester(args.threshold_percentile, args.min_bright_fraction)

    frame_count = 0
    last_fps_check = time.monotonic()

    try:
        while True:

            ret, frame = cap.read()

            if not ret or frame is None:
                print("WARNING: failed to read frame")
                continue

            now = time.monotonic()

            light_on, bright_frac = tester.process_frame(frame, now)

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
                    f"ON={light_on} bright={bright_frac:.3f}",
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

    cap.release()

    if args.show:
        cv2.destroyAllWindows()

    print(f"\nFinal decoded message: {tester.message!r}")


if __name__ == "__main__":
    main()
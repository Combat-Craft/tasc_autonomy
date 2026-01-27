import cv2
import serial
import time
import os

# ---------------- CONFIG ----------------
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

CAMERA_INDEX = 0
OUT_DIR = "panorama_frames"

PAN_START =  0
PAN_END   =  180
PAN_STEP  =  10

SETTLE_TIME = 0.2  # seconds after motion complete
# ----------------------------------------

os.makedirs(OUT_DIR, exist_ok=True)

# Open serial
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
time.sleep(2)  # allow MCU reset
ser.reset_input_buffer()

# Open camera
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    raise RuntimeError("Camera not found")

def move_and_wait(angle):
    cmd = f"PAN {angle}\n"
    ser.write(cmd.encode())

    while True:
        line = ser.readline().decode().strip()
        if line == "OK":
            break

def capture_frame(idx, angle):
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Camera frame grab failed")

    filename = f"{OUT_DIR}/frame_{idx:03d}_{angle}.jpg"
    cv2.imwrite(filename, frame)
    print(f"Saved {filename}")

# ----------- MAIN LOOP ------------------
frame_idx = 0

for angle in range(PAN_START, PAN_END + 1, PAN_STEP):
    print(f"Moving to {angle} deg")
    move_and_wait(angle)

    time.sleep(SETTLE_TIME)

    capture_frame(frame_idx, angle)
    frame_idx += 1

# ----------- CLEANUP --------------------
cap.release()
ser.close()

print("Panorama capture complete.")

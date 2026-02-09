#!/usr/bin/env python3
import time
import serial
import argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="log.csv")
    ap.add_argument("--seconds", type=float, default=180.0)  # 3 minutes
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    t0 = time.time()

    with open(args.out, "w", buffering=1) as f:
        f.write("# Raw ESP32 log. Lines start with IMU, or GPS,\n")
        f.write("# Recommended: keep the robot stationary while logging.\n")
        while time.time() - t0 < args.seconds:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if not line:
                continue
            if line.startswith("IMU,") or line.startswith("GPS,") or line.startswith("#"):
                f.write(line + "\n")

    print(f"Wrote {args.out}")

if __name__ == "__main__":
    main()

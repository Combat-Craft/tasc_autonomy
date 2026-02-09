#!/usr/bin/env python3
import argparse
import math
import numpy as np

def parse_log(path):
    imu = []  # [ms, ax, ay, az, gx, gy, gz]
    gps = []  # [ms, lat, lon, alt, speed, hdop, sats, fix]
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            tag = parts[0]
            try:
                if tag == "IMU" and len(parts) == 8:
                    ms = int(parts[1])
                    vals = list(map(float, parts[2:]))
                    imu.append([ms] + vals)
                elif tag == "GPS" and len(parts) == 10:
                    ms = int(parts[1])
                    # lat lon alt speed hdop sats fix
                    lat = float(parts[2])
                    lon = float(parts[3])
                    alt = float(parts[4])
                    spd = float(parts[5])
                    hdop = float(parts[6])
                    sats = int(float(parts[7]))
                    fix = int(float(parts[8])) if len(parts) == 9 else int(float(parts[9]))
                    # Your format says 9 fields after tag; handle both defensively:
                    # GPS,ms,lat,lon,alt,speed,hdop,sats,fix  -> len=9
                    # If it's len=9, parts[8]=fix.
            except ValueError:
                continue

            # Above is messy for GPS due to defensive parsing; do a simpler, robust version:
    # Re-parse properly
    imu = []
    gps = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            tag = parts[0]
            if tag == "IMU" and len(parts) == 8:
                try:
                    ms = int(parts[1])
                    ax, ay, az = float(parts[2]), float(parts[3]), float(parts[4])
                    gx, gy, gz = float(parts[5]), float(parts[6]), float(parts[7])
                    imu.append([ms, ax, ay, az, gx, gy, gz])
                except ValueError:
                    pass
            elif tag == "GPS":
                # Expect: GPS,ms,lat,lon,alt,speed,hdop,sats,fix  (len=9)
                if len(parts) == 9:
                    try:
                        ms = int(parts[1])
                        lat = float(parts[2]); lon = float(parts[3]); alt = float(parts[4])
                        speed = float(parts[5]); hdop = float(parts[6])
                        sats = int(float(parts[7])); fix = int(float(parts[8]))
                        gps.append([ms, lat, lon, alt, speed, hdop, sats, fix])
                    except ValueError:
                        pass

    imu = np.array(imu, dtype=float) if imu else np.zeros((0, 7))
    gps = np.array(gps, dtype=float) if gps else np.zeros((0, 8))
    return imu, gps

def robust_std(x):
    """Median absolute deviation -> approx std for normal data."""
    x = np.asarray(x)
    x = x[np.isfinite(x)]
    if x.size < 10:
        return float("nan")
    med = np.median(x)
    mad = np.median(np.abs(x - med))
    return 1.4826 * mad

def latlon_to_local_m(lat, lon):
    """Convert lat/lon series to local EN meters using equirectangular approx."""
    lat = np.asarray(lat); lon = np.asarray(lon)
    ok = np.isfinite(lat) & np.isfinite(lon)
    lat0 = np.median(lat[ok])
    lon0 = np.median(lon[ok])
    # meters per degree
    m_per_deg_lat = 111111.0
    m_per_deg_lon = 111111.0 * math.cos(math.radians(lat0))
    north = (lat - lat0) * m_per_deg_lat
    east  = (lon - lon0) * m_per_deg_lon
    return east, north, lat0, lon0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile", default="log.csv")
    ap.add_argument("--gps_window_s", type=float, default=60.0,
                    help="window length for GPS covariance estimate (captures short-term noise, not drift)")
    args = ap.parse_args()

    imu, gps = parse_log(args.logfile)

    if imu.shape[0] < 50:
        print("Not enough IMU samples found.")
        return

    # IMU stats (stationary): use robust std
    ax, ay, az = imu[:,1], imu[:,2], imu[:,3]
    gx, gy, gz = imu[:,4], imu[:,5], imu[:,6]

    acc_std = np.array([robust_std(ax), robust_std(ay), robust_std(az)])
    gyr_std = np.array([robust_std(gx), robust_std(gy), robust_std(gz)])

    acc_var = acc_std**2
    gyr_var = gyr_std**2

    print("\nIMU (stationary) robust std:")
    print(f"  accel std [m/s^2]: {acc_std}")
    print(f"  gyro  std [rad/s]: {gyr_std}")

    print("\nSuggested Imu message covariances (diagonal):")
    print(f"  linear_acceleration_covariance diag: {acc_var}")
    print(f"  angular_velocity_covariance     diag: {gyr_var}")
    print("  orientation_covariance: set [0]=-1 (unknown)")

    # GPS stats
    if gps.shape[0] < 5:
        print("\nNot enough GPS samples found.")
        return

    # Filter to fix==1
    fix = gps[:,7].astype(int)
    gps_fix = gps[fix == 1]
    if gps_fix.shape[0] < 5:
        print("\nGPS has no valid fixes in the log (fix==1).")
        return

    ms = gps_fix[:,0]
    lat = gps_fix[:,1]
    lon = gps_fix[:,2]
    alt = gps_fix[:,3]
    hdop = gps_fix[:,5]
    sats = gps_fix[:,6]

    east, north, lat0, lon0 = latlon_to_local_m(lat, lon)

    # Short-window covariance: take the last gps_window_s seconds worth (or all if shorter)
    t_sec = (ms - ms[0]) / 1000.0
    tmax = t_sec[-1]
    tmin = max(0.0, tmax - args.gps_window_s)
    idx = (t_sec >= tmin) & np.isfinite(east) & np.isfinite(north) & np.isfinite(alt)
    if np.count_nonzero(idx) < 5:
        idx = np.isfinite(east) & np.isfinite(north) & np.isfinite(alt)

    e_std = robust_std(east[idx])
    n_std = robust_std(north[idx])
    u_std = robust_std(alt[idx])

    print("\nGPS (fix==1) reference:")
    print(f"  lat0, lon0 = {lat0:.7f}, {lon0:.7f}")
    print(f"  sats median = {np.median(sats):.1f}")
    print(f"  hdop median = {np.median(hdop[np.isfinite(hdop)]):.2f}")

    print(f"\nGPS robust std over last ~{args.gps_window_s:.0f}s (meters):")
    print(f"  east  std [m]: {e_std:.3f}")
    print(f"  north std [m]: {n_std:.3f}")
    print(f"  up    std [m]: {u_std:.3f}")

    pos_var = np.array([e_std**2, n_std**2, u_std**2])

    print("\nSuggested NavSatFix position_covariance diag (m^2):")
    print(f"  {pos_var}")

    print("\nNotes:")
    print("  - GPS drift over minutes is not 'measurement noise'; keep the window modest (30–120s).")
    print("  - If up(std) is weird, your altitude is noisy; it's common to inflate the Z variance a lot.")

if __name__ == "__main__":
    main()

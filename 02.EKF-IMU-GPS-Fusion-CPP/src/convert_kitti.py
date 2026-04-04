#!/usr/bin/env python3
"""
Convert KITTI raw oxts data to project CSV format.
KITTI oxts fields (per line):
  0:lat 1:lon 2:alt 3:roll 4:pitch 5:yaw
  6:vn 7:ve 8:vf 9:vl 10:vu
  11:ax 12:ay 13:az 14:af 15:al 16:au
  17:wx 18:wy 19:wz 20:wf 21:wl 22:wu
  23:pos_acc 24:vel_acc 25:navstat 26:numsats 27:posmode 28:velmode 29:orimode
"""

import os
import glob
import numpy as np
from datetime import datetime

import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, "..", "data")


def parse_timestamps(path):
    """Parse KITTI timestamp file to seconds from start."""
    times = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            dt = datetime.strptime(line[:26], "%Y-%m-%d %H:%M:%S.%f")
            times.append(dt)
    t0 = times[0]
    return [(t - t0).total_seconds() for t in times]


def latlon_to_local(lat, lon, lat0, lon0):
    """Convert lat/lon to local ENU (East-North) in metres."""
    R_earth = 6378137.0
    dlat = np.radians(lat - lat0)
    dlon = np.radians(lon - lon0)
    x = dlon * R_earth * np.cos(np.radians(lat0))  # East
    y = dlat * R_earth                               # North
    return x, y


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 convert_kitti.py <kitti_drive_dir>")
        print("  e.g. python3 convert_kitti.py ../data/2011_09_26_drive_0005_sync")
        sys.exit(1)

    kitti_base = sys.argv[1]
    # Find oxts directory (handle nested structure)
    if os.path.isdir(os.path.join(kitti_base, "oxts")):
        oxts_dir = os.path.join(kitti_base, "oxts")
    else:
        # Nested: 2011_09_26/2011_09_26_drive_XXXX_sync/oxts
        candidates = glob.glob(os.path.join(kitti_base, "**", "oxts"), recursive=True)
        if not candidates:
            print(f"Error: oxts directory not found under {kitti_base}")
            sys.exit(1)
        oxts_dir = candidates[0]

    print(f"Using oxts dir: {oxts_dir}")

    # Output directory
    drive_name = os.path.basename(os.path.normpath(kitti_base)).replace("_sync", "")
    out_dir = os.path.join(DATA_DIR, "kitti_data_output")
    os.makedirs(out_dir, exist_ok=True)

    ts_path = os.path.join(oxts_dir, "timestamps.txt")
    data_dir = os.path.join(oxts_dir, "data")

    timestamps = parse_timestamps(ts_path)
    files = sorted(glob.glob(os.path.join(data_dir, "*.txt")))

    print(f"Found {len(files)} KITTI oxts frames")

    all_data = []
    for i, fpath in enumerate(files):
        vals = list(map(float, open(fpath).read().strip().split()))
        all_data.append((timestamps[i], vals))

    # Reference point for local coords
    lat0 = all_data[0][1][0]
    lon0 = all_data[0][1][1]

    # ── Write IMU CSV ──
    # Use body-frame accel (ax,ay,az = fields 11,12,13) and gyro (wx,wy,wz = fields 17,18,19)
    imu_path = os.path.join(out_dir, "imu.csv")
    with open(imu_path, "w") as f:
        f.write("timestamp,ax,ay,az,gx,gy,gz\n")
        for t, v in all_data:
            ax, ay, az = v[11], v[12], v[13]
            wx, wy, wz = v[17], v[18], v[19]
            f.write(f"{t:.6f},{ax:.6f},{ay:.6f},{az:.6f},{wx:.6f},{wy:.6f},{wz:.6f}\n")

    # ── Write GPS CSV ──
    # Convert lat/lon to local metres: x=East, y=North
    # Velocity must match: vx=East(ve), vy=North(vn), vz=Down(-vu)
    gps_path = os.path.join(out_dir, "gps.csv")
    with open(gps_path, "w") as f:
        f.write("timestamp,lat,lon,alt,vn,ve,vd\n")
        for t, v in all_data:
            x, y = latlon_to_local(v[0], v[1], lat0, lon0)
            alt = v[2]
            vn, ve = v[6], v[7]
            vu = v[10]
            # Write ve first (East=x), then vn (North=y) to match pos frame
            f.write(f"{t:.6f},{x:.6f},{y:.6f},{alt:.6f},{ve:.6f},{vn:.6f},{-vu:.6f}\n")

    # ── Write Ground Truth CSV ──
    gt_path = os.path.join(out_dir, "ground_truth.csv")
    with open(gt_path, "w") as f:
        f.write("timestamp,px,py,pz,vx,vy,vz,roll,pitch,yaw\n")
        for t, v in all_data:
            x, y = latlon_to_local(v[0], v[1], lat0, lon0)
            alt = v[2]
            roll, pitch, yaw = v[3], v[4], v[5]
            vn, ve, vu = v[6], v[7], v[10]
            # ve first (East=x), then vn (North=y) to match pos frame
            f.write(f"{t:.6f},{x:.6f},{y:.6f},{alt:.6f},{ve:.6f},{vn:.6f},{-vu:.6f},{roll:.6f},{pitch:.6f},{yaw:.6f}\n")

    print(f"Saved: {imu_path}")
    print(f"Saved: {gps_path}")
    print(f"Saved: {gt_path}")
    print(f"\nOutput dir: {out_dir}")
    print(f"Run EKF:  cd build && ./ekf_imu_gps_fusion ../data/kitti_data_output")
    print(f"Plot:     python3 src/plot_results.py data/kitti_data_output")


if __name__ == "__main__":
    main()

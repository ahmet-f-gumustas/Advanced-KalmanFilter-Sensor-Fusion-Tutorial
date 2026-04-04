#!/usr/bin/env python3
"""
EKF IMU+GPS Fusion — Result Plotter
Reads CSV outputs and generates comparison plots.
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless backend — saves to file
import matplotlib.pyplot as plt

DATA_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "data")


def load_csv(filename):
    path = os.path.join(DATA_DIR, filename)
    return np.genfromtxt(path, delimiter=",", skip_header=1)


def main():
    # ── Load ──
    gt  = load_csv("ground_truth.csv")   # t,px,py,pz,vx,vy,vz,r,p,y
    gps = load_csv("gps.csv")            # t,lat,lon,alt,vn,ve,vd
    ekf = load_csv("ekf_output.csv")     # t,x,y,z,vx,vy,vz,roll,pitch,yaw

    print(f"Ground truth : {gt.shape[0]} samples")
    print(f"GPS          : {gps.shape[0]} samples")
    print(f"EKF output   : {ekf.shape[0]} samples")

    # ══════════════════════════════════════════
    #  Figure 1 — Trajectory & Position
    # ══════════════════════════════════════════
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("EKF IMU+GPS Sensor Fusion Results", fontsize=14, fontweight="bold")

    # 2D Trajectory
    ax = axes[0, 0]
    ax.plot(gt[:, 1], gt[:, 2], "g-", lw=2, label="Ground Truth")
    ax.plot(ekf[:, 1], ekf[:, 2], "b-", lw=1.5, label="EKF Estimate")
    ax.scatter(gps[:, 1], gps[:, 2], c="red", s=5, alpha=0.4, label="GPS (noisy)")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("2D Trajectory")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")

    # Position X
    ax = axes[0, 1]
    ax.plot(gt[:, 0], gt[:, 1], "g-", lw=1.5, label="GT X")
    ax.plot(ekf[:, 0], ekf[:, 1], "b-", lw=1, label="EKF X")
    ax.scatter(gps[:, 0], gps[:, 1], c="red", s=3, alpha=0.3, label="GPS X")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("X [m]")
    ax.set_title("Position X vs Time")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Position Y
    ax = axes[1, 0]
    ax.plot(gt[:, 0], gt[:, 2], "g-", lw=1.5, label="GT Y")
    ax.plot(ekf[:, 0], ekf[:, 2], "b-", lw=1, label="EKF Y")
    ax.scatter(gps[:, 0], gps[:, 2], c="red", s=3, alpha=0.3, label="GPS Y")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Position Y vs Time")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2D Position Error — match EKF timestamps to closest GT
    ax = axes[1, 1]
    ekf_sub_list, gt_sub_list = [], []
    for i in range(ekf.shape[0]):
        idx = np.argmin(np.abs(gt[:, 0] - ekf[i, 0]))
        if abs(gt[idx, 0] - ekf[i, 0]) < 0.2:
            ekf_sub_list.append(ekf[i])
            gt_sub_list.append(gt[idx])
    ekf_sub = np.array(ekf_sub_list)
    gt_sub = np.array(gt_sub_list)
    err_2d = np.sqrt((ekf_sub[:, 1] - gt_sub[:, 1])**2 + (ekf_sub[:, 2] - gt_sub[:, 2])**2)
    ax.plot(ekf_sub[:, 0], err_2d, "r-", lw=1)
    ax.fill_between(ekf_sub[:, 0], 0, err_2d, alpha=0.2, color="red")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error [m]")
    ax.set_title(f"2D Position Error (RMSE: {np.sqrt(np.mean(err_2d**2)):.2f} m)")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out1 = os.path.join(DATA_DIR, "ekf_results.png")
    plt.savefig(out1, dpi=150)
    print(f"Saved: {out1}")

    # ══════════════════════════════════════════
    #  Figure 2 — Velocity & Attitude
    # ══════════════════════════════════════════
    fig2, axes2 = plt.subplots(2, 2, figsize=(14, 10))
    fig2.suptitle("EKF Velocity & Attitude Estimates", fontsize=14, fontweight="bold")

    # Velocity X
    ax = axes2[0, 0]
    ax.plot(gt[:, 0], gt[:, 4], "g-", lw=1.5, label="GT Vx")
    ax.plot(ekf[:, 0], ekf[:, 4], "b-", lw=1, label="EKF Vx")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Vx [m/s]")
    ax.set_title("Velocity X")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Velocity Y
    ax = axes2[0, 1]
    ax.plot(gt[:, 0], gt[:, 5], "g-", lw=1.5, label="GT Vy")
    ax.plot(ekf[:, 0], ekf[:, 5], "b-", lw=1, label="EKF Vy")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Vy [m/s]")
    ax.set_title("Velocity Y")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Yaw
    ax = axes2[1, 0]
    ax.plot(gt[:, 0], np.degrees(gt[:, 9]), "g-", lw=1.5, label="GT Yaw")
    ax.plot(ekf[:, 0], np.degrees(ekf[:, 9]), "b-", lw=1, label="EKF Yaw")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Yaw [deg]")
    ax.set_title("Yaw Angle")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Roll & Pitch
    ax = axes2[1, 1]
    ax.plot(ekf[:, 0], np.degrees(ekf[:, 7]), "orange", lw=1, label="EKF Roll")
    ax.plot(ekf[:, 0], np.degrees(ekf[:, 8]), "purple", lw=1, label="EKF Pitch")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle [deg]")
    ax.set_title("Roll & Pitch")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out2 = os.path.join(DATA_DIR, "ekf_velocity_attitude.png")
    plt.savefig(out2, dpi=150)
    print(f"Saved: {out2}")

    print("\nDone! Plots saved to data/ directory.")


if __name__ == "__main__":
    main()

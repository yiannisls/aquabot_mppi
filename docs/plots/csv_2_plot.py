#!/usr/bin/env python3
"""
Generate Results-section plots for the Aquabot MPPI README from a logged
run_data.csv (produced by motion_node.cpp's CSV logger).

Usage:
    python3 plot_results.py run_data.csv [output_dir]

Requires: pandas, matplotlib
    pip install pandas matplotlib --break-system-packages
"""
import sys
import pandas as pd
import matplotlib.pyplot as plt


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_results.py <run_data.csv> [output_dir]")
        sys.exit(1)

    csv_path = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "."

    df = pd.read_csv(csv_path)
    df["time"] = df["time"] - df["time"].iloc[0]  # start the clock at 0
    target_speed = 1.3

    
    # Plot 1: top-down traveled path, colored by cross-track error
    
    fig, ax = plt.subplots(figsize=(8, 8))
    sc = ax.scatter(df["x"], df["y"], c=df["cross_track_error"], cmap="viridis", s=6)
    ax.plot(df["x"], df["y"], color="black", linewidth=0.3, alpha=0.3)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Traveled path, colored by cross-track error")
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("cross-track error [m]")
    fig.tight_layout()
    fig.savefig(f"{out_dir}/path_tracking.png", dpi=150)

    # Plot 2: speed over time vs. target
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df["time"], df["speed"], label="actual speed", linewidth=1)
    ax.axhline(target_speed, color="red", linestyle="--", label=f"target ({target_speed} m/s)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("speed [m/s]")
    ax.set_title("Speed tracking")
    ax.legend()
    fig.tight_layout()
    fig.savefig(f"{out_dir}/speed_profile.png", dpi=150)

    # Plot 3: cross-track error over time
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df["time"], df["cross_track_error"], linewidth=1, color="darkorange")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("cross-track error [m]")
    ax.set_title("Path deviation over time")
    fig.tight_layout()
    fig.savefig(f"{out_dir}/cross_track_error.png", dpi=150)

    
    # Summary stats -- handy for README captions/prose
    print(f"Duration:              {df['time'].iloc[-1]:.1f} s")
    print(f"Mean speed:            {df['speed'].mean():.3f} m/s")
    print(f"Speed std dev:         {df['speed'].std():.3f} m/s")
    print(f"Mean cross-track err:  {df['cross_track_error'].mean():.3f} m")
    print(f"Max cross-track err:   {df['cross_track_error'].max():.3f} m")
    print(f"95th pct cross-track:  {df['cross_track_error'].quantile(0.95):.3f} m")

    print(f"\nSaved: {out_dir}/path_tracking.png")
    print(f"Saved: {out_dir}/speed_profile.png")
    print(f"Saved: {out_dir}/cross_track_error.png")


if __name__ == "__main__":
    main()
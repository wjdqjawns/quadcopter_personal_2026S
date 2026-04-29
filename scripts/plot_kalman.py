#!/usr/bin/env python3
"""Compare EKF estimated position/velocity against ground truth from sim_log.csv."""
import sys
import pandas as pd
import matplotlib.pyplot as plt

log_path = sys.argv[1] if len(sys.argv) > 1 else "data/logs/sim_log.csv"
df = pd.read_csv(log_path)

fig, axes = plt.subplots(2, 3, figsize=(14, 7))
labels = ["x", "y", "z"]

for col, label in enumerate(labels):
    # Position
    ax = axes[0][col]
    ax.plot(df.t, df[f"p{label}"],        label="ground truth", linewidth=1.5)
    ax.plot(df.t, df[f"est_p{label}"],    label="EKF",  linestyle="--", linewidth=1.2)
    ax.set_title(f"Position {label}")
    ax.set_xlabel("time (s)"); ax.set_ylabel("m")
    ax.legend(); ax.grid(True)

    # Velocity
    ax = axes[1][col]
    ax.plot(df.t, df[f"v{label}"],        label="ground truth", linewidth=1.5)
    ax.plot(df.t, df[f"est_v{label}"],    label="EKF",  linestyle="--", linewidth=1.2)
    ax.set_title(f"Velocity {label}")
    ax.set_xlabel("time (s)"); ax.set_ylabel("m/s")
    ax.legend(); ax.grid(True)

plt.suptitle("EKF vs Ground Truth")
plt.tight_layout()
plt.savefig("data/results/kalman.png", dpi=150)
print("Saved data/results/kalman.png")
plt.show()

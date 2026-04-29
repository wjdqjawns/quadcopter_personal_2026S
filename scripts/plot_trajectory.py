#!/usr/bin/env python3
"""Plot 3-D trajectory and per-axis position from sim_log.csv."""
import sys
import pandas as pd
import matplotlib.pyplot as plt

log_path = sys.argv[1] if len(sys.argv) > 1 else "data/logs/sim_log.csv"
df = pd.read_csv(log_path)

fig = plt.figure(figsize=(14, 5))

# 3-D trajectory
ax3d = fig.add_subplot(121, projection="3d")
ax3d.plot(df.px, df.py, df.pz, linewidth=1.5)
ax3d.set_xlabel("x (m)"); ax3d.set_ylabel("y (m)"); ax3d.set_zlabel("z (m)")
ax3d.set_title("3-D Trajectory")

# Per-axis position vs time
ax = fig.add_subplot(122)
ax.plot(df.t, df.px, label="x")
ax.plot(df.t, df.py, label="y")
ax.plot(df.t, df.pz, label="z")
ax.set_xlabel("time (s)"); ax.set_ylabel("position (m)")
ax.set_title("Position vs Time"); ax.legend(); ax.grid(True)

plt.tight_layout()
plt.savefig("data/results/trajectory.png", dpi=150)
print("Saved data/results/trajectory.png")
plt.show()

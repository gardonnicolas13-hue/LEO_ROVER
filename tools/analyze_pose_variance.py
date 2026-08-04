#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analyzes a T1.1 static-drift CSV (from log_pose_static.py).

Reports, per axis (X, Y, Z):
  - std / variance          : spread around the mean -- a covariance/noise
                               tuning question if high but bounded.
  - linear drift rate (m/s) : slope of a least-squares fit position vs time --
                               near-zero means "noise", clearly nonzero means
                               genuine unbounded divergence (extrinsics /
                               initialization problem, NOT a noise param).
  - first vs last 10% mean  : quick sanity check same conclusion a second way.

Usage: python3 analyze_pose_variance.py [csv_path]
"""
import sys
import csv
import statistics

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/t1_1_static_pose.csv"

t, x, y, z = [], [], [], []
with open(path) as f:
    r = csv.DictReader(f)
    for row in r:
        t.append(float(row["stamp"]))
        x.append(float(row["x"]))
        y.append(float(row["y"]))
        z.append(float(row["z"]))

if not t:
    print("No samples in %s" % path)
    sys.exit(1)

t0 = t[0]
t = [ti - t0 for ti in t]
n = len(t)


def linfit_slope(tt, vv):
    n = len(tt)
    mt = sum(tt) / n
    mv = sum(vv) / n
    num = sum((tt[i] - mt) * (vv[i] - mv) for i in range(n))
    den = sum((tt[i] - mt) ** 2 for i in range(n))
    return num / den if den != 0 else 0.0


print("n=%d samples over %.1fs\n" % (n, t[-1]))
for name, v in (("X", x), ("Y", y), ("Z", z)):
    std = statistics.pstdev(v)
    var = std ** 2
    slope = linfit_slope(t, v)
    k = max(1, n // 10)
    first_mean = sum(v[:k]) / k
    last_mean = sum(v[-k:]) / k
    print("%s: std=%.4f m  var=%.6f m^2  drift_rate=%+.5f m/s  "
          "first10%%=%.3f  last10%%=%.3f  delta=%+.3f m"
          % (name, std, var, slope, first_mean, last_mean, last_mean - first_mean))

print("\nInterpretation:")
print("  |drift_rate| < ~0.001 m/s and delta small -> bounded noise, tune")
print("    covariances/noise_p in the relevant config_*.yaml.")
print("  |drift_rate| clearly nonzero / delta grows with n -> unbounded")
print("    divergence, look at extrinsics (config_camera.yaml T_imu_cam,")
print("    still placeholder identity) or config_init.yaml, not noise terms.")

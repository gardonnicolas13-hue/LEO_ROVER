#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fill the MINS LEO config_camera.yaml T_imu_cam blocks from a finished
kalibr_calibrate_imu_camera result.

Kalibr's imucam yaml gives, per camera, `T_cam_imu` = transform IMU->camera
(p_cam = T_cam_imu * p_imu) and a `timeshift_cam_imu`. MINS's config_camera.yaml
wants `T_imu_cam` = "R_CtoI, p_CinI" = the INVERSE (camera->IMU), plus a per-cam
`timeoffset`. This script inverts each T_cam_imu, writes the 4x4 into the
matching cam block, and copies the timeoffset — no hand-editing, no sign errors.

Usage:
  python3 fill_mins_camchain.py <kalibr_imucam.yaml> <mins_config_camera.yaml>

It rewrites only the T_imu_cam matrix rows and the `timeoffset:` line under each
cam{N}; intrinsics/distortion/topic/resolution are left untouched (those came
from the earlier camera-only calibration and are already correct in the config).
"""
import sys
import re
import numpy as np
import yaml


def load_kalibr(path):
    # kalibr yamls start with "%YAML:1.0" which PyYAML dislikes — strip it.
    with open(path) as f:
        txt = f.read()
    txt = re.sub(r"^%YAML:[0-9.]+\s*", "", txt).replace("!!opencv-matrix", "")
    return yaml.safe_load(txt)


def invert_se3(T):
    T = np.array(T, dtype=float)
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    Ti = np.eye(4)
    Ti[0:3, 0:3] = R.T
    Ti[0:3, 3] = -R.T @ t
    return Ti


def fmt_matrix(T):
    rows = []
    for r in range(4):
        vals = ", ".join("%.10g" % T[r, c] for c in range(4))
        rows.append("    - [%s]" % vals)
    return "\n".join(rows)


def main(kalibr_path, config_path):
    k = load_kalibr(kalibr_path)
    with open(config_path) as f:
        cfg = f.read()

    for cam_key in sorted(x for x in k if re.match(r"cam\d+$", x)):
        block = k[cam_key]
        if "T_cam_imu" not in block:
            print("  %s: no T_cam_imu in kalibr output, skipping" % cam_key)
            continue
        T_imu_cam = invert_se3(block["T_cam_imu"])
        toff = block.get("timeshift_cam_imu", 0.0)

        # Replace the T_imu_cam matrix (4 "- [...]" lines) inside this cam block.
        # Match from `<cam>:` up to its T_imu_cam and swap the 4 matrix rows.
        pat_mat = re.compile(
            r"(^%s:\n(?:.*\n)*?\s*T_imu_cam:\s*\n)"
            r"(?:\s*-\s*\[[^\]]*\]\s*\n){4}" % re.escape(cam_key), re.MULTILINE)
        new_mat = fmt_matrix(T_imu_cam) + "\n"
        cfg, n1 = pat_mat.subn(lambda m: m.group(1) + new_mat, cfg)

        # Replace the timeoffset line inside this cam block.
        pat_to = re.compile(
            r"(^%s:\n(?:.*\n)*?\s*timeoffset:\s*)[-\d.eE]+" % re.escape(cam_key),
            re.MULTILINE)
        cfg, n2 = pat_to.subn(lambda m: m.group(1) + ("%.6g" % toff), cfg)

        print("  %s: T_imu_cam %s, timeoffset=%.6g  (matrix subs=%d, toff subs=%d)"
              % (cam_key, "written" if n1 else "NOT FOUND", toff, n1, n2))

    with open(config_path, "w") as f:
        f.write(cfg)
    print("Updated %s" % config_path)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: fill_mins_camchain.py <kalibr_imucam.yaml> <mins_config_camera.yaml>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])

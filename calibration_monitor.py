#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LEO Rover — Kalibr calibration health monitor
================================================
Watches the Kalibr camera-calibration output files on disk (produced by
`kalibr_calibrate_cameras` / `kalibr_calibrate_imu_camera`, see
LEO_Rover_Navigation_System/VINS_Calibration_Procedure.md) and republishes
their key numbers as a JSON ROS topic, so the web cockpit can show
calibration health without anyone re-running Kalibr or reading YAML by hand.

  PUBLISHES
    /leo_vision/calibration_status   std_msgs/String (JSON, latched)
    republished only when bag_final-camchain.yaml's mtime changes, so this
    stays quiet on the bus between calibration runs.

  WATCHES (default paths under calib_dir, override with ROS private params)
    ~camchain     <calib_dir>/bag_final-camchain.yaml   (intrinsics/extrinsics)
    ~results_txt  <calib_dir>/bag_final-results-cam.txt (reprojection error)
    ~baseline     <calib_dir>/calibration_baseline.json (drift reference,
                  created automatically from the first calibration seen —
                  delete it to re-baseline after an intentional recalibration)

  LAUNCH (on PC, same ROS_MASTER_URI as leo_backend.py):
    python3 calibration_monitor.py
  (see start_web.sh — launched alongside rosbridge/web_video_server/backend)
"""

import os
import re
import json
import time

import yaml
import rospy
from std_msgs.msg import String

TOPIC = "/leo_vision/calibration_status"
DEFAULT_CALIB_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "catkin_ws", "calibration_leo_d455")
POLL_PERIOD_S = 5.0

# Drift thresholds — a fixed-lens camera's focal length shouldn't move once
# calibrated; a few % is normal fit noise between runs, more suggests the
# lens/mount was bumped or the target measurements changed between runs.
DRIFT_FOCAL_PCT = 5.0
DRIFT_REPROJ_FACTOR = 2.0

# kalibr_camera_calibration/CameraUtils.py printParameters() output format:
#   cam0 (/camera/infra1/image_rect_raw):
#       ...
#       reprojection error: [0.123456, 0.234567] +- [0.01, 0.02]
_REPROJ_RE = re.compile(
    r"cam(\d+)\s*\(([^)]*)\):.*?"
    r"reprojection error:\s*\[([-\d.eE]+),\s*([-\d.eE]+)\]\s*\+-\s*"
    r"\[([-\d.eE]+),\s*([-\d.eE]+)\]",
    re.DOTALL,
)


def parse_results_txt(path):
    """{cam_id: {"reproj_mean": [mx, my], "reproj_std": [sx, sy]}} or {} if absent."""
    if not os.path.isfile(path):
        return {}
    with open(path, "r") as f:
        text = f.read()
    out = {}
    for m in _REPROJ_RE.finditer(text):
        cid = int(m.group(1))
        out[cid] = {
            "reproj_mean": [float(m.group(3)), float(m.group(4))],
            "reproj_std": [float(m.group(5)), float(m.group(6))],
        }
    return out


def parse_camchain(path):
    if not os.path.isfile(path):
        return None
    with open(path, "r") as f:
        return yaml.safe_load(f)


def build_status(camchain_path, results_txt_path, baseline_path):
    camchain = parse_camchain(camchain_path)
    if not camchain:
        return {"available": False, "checked_at": round(time.time(), 1)}

    reproj_by_cam = parse_results_txt(results_txt_path)
    cams = []
    for cam_key in sorted(k for k in camchain if k.startswith("cam")):
        cid = int(cam_key.replace("cam", ""))
        c = camchain[cam_key] or {}
        # Use the STD (spread of the reprojection error), not the mean: the mean is a
        # near-zero bias term by construction (signed residuals average out) and says
        # nothing about calibration quality — the std is the actual per-axis error size.
        std = reproj_by_cam.get(cid, {}).get("reproj_std")
        reproj_px = round((std[0] ** 2 + std[1] ** 2) ** 0.5, 4) if std else None
        cams.append({
            "cam_id": cid,
            "topic": c.get("rostopic"),
            "model": c.get("camera_model"),
            "resolution": c.get("resolution"),
            "intrinsics": c.get("intrinsics"),          # [fx, fy, cx, cy]
            "distortion": c.get("distortion_coeffs"),
            "reprojection_error_px": reproj_px,
            "has_imu_extrinsics": "T_cam_imu" in c,      # set once IMU-cam step ran
        })

    cam1 = camchain.get("cam1") or {}
    status = {
        "available": True,
        "checked_at": round(time.time(), 1),
        "calibrated_at": os.path.getmtime(camchain_path),
        "camchain_path": camchain_path,
        "cameras": cams,
        "extrinsics_T_cn_cnm1": cam1.get("T_cn_cnm1"),
    }
    status["drift"] = compute_drift(status, baseline_path)
    return status


def compute_drift(status, baseline_path):
    baseline = None
    if os.path.isfile(baseline_path):
        try:
            with open(baseline_path, "r") as f:
                baseline = json.load(f)
        except Exception:
            baseline = None

    if baseline is None:
        # First calibration seen — adopt it as the drift reference.
        try:
            with open(baseline_path, "w") as f:
                json.dump(status, f, indent=2)
        except Exception as e:
            rospy.logwarn("[calibration_monitor] could not write baseline: %s", e)
        return {"baseline_available": False, "alert": False, "reasons": []}

    reasons = []
    for cam in status["cameras"]:
        base_cam = next(
            (b for b in baseline.get("cameras", []) if b["cam_id"] == cam["cam_id"]), None)
        if base_cam is None:
            continue

        base_intr, intr = base_cam.get("intrinsics"), cam.get("intrinsics")
        if base_intr and intr and base_intr[0]:
            fx0, fx1 = base_intr[0], intr[0]
            pct = abs(fx1 - fx0) / fx0 * 100.0
            if pct > DRIFT_FOCAL_PCT:
                reasons.append(
                    "cam%d focal length drifted %.1f%% vs baseline (fx %.1f -> %.1f)"
                    % (cam["cam_id"], pct, fx0, fx1))

        r0 = base_cam.get("reprojection_error_px")
        r1 = cam.get("reprojection_error_px")
        if r0 and r1 and r1 > r0 * DRIFT_REPROJ_FACTOR:
            reasons.append(
                "cam%d reprojection error grew from %.3fpx to %.3fpx (>%gx baseline)"
                % (cam["cam_id"], r0, r1, DRIFT_REPROJ_FACTOR))

    return {
        "baseline_available": True,
        "baseline_calibrated_at": baseline.get("calibrated_at"),
        "alert": len(reasons) > 0,
        "reasons": reasons,
    }


def main():
    rospy.init_node("calibration_monitor", anonymous=False)

    calib_dir = rospy.get_param("~calib_dir", DEFAULT_CALIB_DIR)
    camchain_path = rospy.get_param(
        "~camchain", os.path.join(calib_dir, "bag_final-camchain.yaml"))
    results_txt_path = rospy.get_param(
        "~results_txt", os.path.join(calib_dir, "bag_final-results-cam.txt"))
    baseline_path = rospy.get_param(
        "~baseline", os.path.join(calib_dir, "calibration_baseline.json"))
    period = rospy.get_param("~period_s", POLL_PERIOD_S)

    pub = rospy.Publisher(TOPIC, String, queue_size=1, latch=True)
    rospy.loginfo("[calibration_monitor] watching %s every %.1fs -> %s",
                  camchain_path, period, TOPIC)

    rate = rospy.Rate(1.0 / period)
    last_mtime = None
    announced_missing = False
    while not rospy.is_shutdown():
        try:
            mtime = os.path.getmtime(camchain_path) if os.path.isfile(camchain_path) else None
            if mtime != last_mtime:
                status = build_status(camchain_path, results_txt_path, baseline_path)
                pub.publish(String(data=json.dumps(status)))
                last_mtime = mtime
                announced_missing = False
                rospy.loginfo("[calibration_monitor] published update (calibrated_at changed)")
            elif mtime is None and not announced_missing:
                pub.publish(String(data=json.dumps(
                    {"available": False, "checked_at": round(time.time(), 1)})))
                announced_missing = True
        except Exception as e:
            rospy.logwarn("[calibration_monitor] error: %s", e)
        rate.sleep()


if __name__ == "__main__":
    main()

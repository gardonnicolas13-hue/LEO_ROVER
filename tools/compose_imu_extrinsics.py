#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compose_imu_extrinsics.py — rigid transform between the two IMUs of the LEO Rover.

    T_imu1_imu2  =  T_imu1_cam  ·  inverse(T_imu2_cam)

Both IMUs observe the SAME camera, so the camera frame is the bridge that lets
their relative pose be recovered without ever moving the robot: each IMU has a
camera extrinsic, and composing one with the inverse of the other cancels the
camera out.

    imu2 ──T_cam_imu2──▶ cam ──T_imu1_cam──▶ imu1        (read left to right)

CONVENTION — this is the single most common source of a silent 180 deg error,
so it is stated once and used everywhere below:

    T_A_B maps a point expressed in frame B to frame A:   p_A = T_A_B · p_B

  * Kalibr writes  T_cam_imu   (IMU -> camera).
  * MINS  wants    T_imu_cam   (camera -> IMU), i.e. the inverse.
    See mins/config/leo/config_camera.yaml, which says so explicitly.

  This script accepts either and normalises internally; say which one you are
  handing it with --imu1-convention / --imu2-convention.

THE TWO IMUs ON THIS PLATFORM
  imu1 = body IMU on the CORE2 board. The only inertial source any estimator
         consumes today, along /firmware/imu -> /imu/data_raw ->
         /imu/data_clean (and /imu/data_vins for the VINS branches).
  imu2 = the D455's own motion module. Physically present (it enumerates as a
         USB HID interface and the RealSense driver logs "Motion Module was
         found"), but disabled in the driver: enabling it on this platform
         stalls the RealSense node — see the note in tools/README or the
         2026-09-06 entry in JOURNAL_DE_BORD.md.

WHERE THE INPUTS COME FROM
  imu1: any Kalibr *-camchain-imucam.yaml (calib_data/ holds several), or the
        deployed MINS config_camera.yaml.
  imu2: the D455 factory extrinsic, read from the device with pyrealsense2
        (--imu2-from-device). Note this needs the camera NOT to be held by the
        ROS driver, so stop leo.service first, or run it on a spare host.
        Factory extrinsics are per-unit; do not copy them between cameras.

Usage
  ./compose_imu_extrinsics.py --imu1 calib_data/imucam_...-camchain-imucam.yaml \
                              --imu2 d455_imu_extrinsic.yaml
  ./compose_imu_extrinsics.py --imu1 ... --imu2-from-device
  ./compose_imu_extrinsics.py --selftest

numpy + pyyaml only. No scipy: the rotation helpers below are short, and the
robot does not necessarily carry scipy.
"""

import argparse
import os
import sys

import numpy as np

try:
    import yaml
except ImportError:
    sys.exit("pyyaml missing:  pip3 install pyyaml")

np.set_printoptions(suppress=True, precision=6, linewidth=120)

# Tolerance on R'R = I and det(R) = 1. Kalibr writes ~16 significant digits, so
# a healthy matrix lands near 1e-15; 1e-6 leaves room for a hand-edited or
# truncated YAML while still catching a genuinely non-rigid matrix.
TOL_ORTHO = 1e-6


# ─────────────────────────────────────────────────────────────────────────────
#  Rigid-transform helpers
# ─────────────────────────────────────────────────────────────────────────────
def check_rigid(T, name):
    """Reject anything that is not a proper rigid transform.

    Silent breakage here is expensive: a matrix with det(R) = -1 (a reflection,
    typically from a transposed or hand-copied block) composes without error
    and yields a plausible-looking answer that is wrong. Fail loudly instead.
    """
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError("%s: expected 4x4, got %s" % (name, T.shape))
    R, bottom = T[:3, :3], T[3, :]

    err_ortho = np.abs(R.T @ R - np.eye(3)).max()
    if err_ortho > TOL_ORTHO:
        raise ValueError("%s: R is not orthonormal (max|R'R - I| = %.2e)"
                         % (name, err_ortho))

    det = np.linalg.det(R)
    if abs(det - 1.0) > TOL_ORTHO:
        raise ValueError(
            "%s: det(R) = %+.9f, expected +1. A value near -1 means a "
            "reflection — usually a transposed or mis-copied rotation block."
            % (name, det))

    if np.abs(bottom - np.array([0.0, 0.0, 0.0, 1.0])).max() > TOL_ORTHO:
        raise ValueError("%s: bottom row is %s, expected [0 0 0 1]" % (name, bottom))
    return T


def invert(T):
    """Inverse of a rigid transform, built from the block structure.

    Cheaper and numerically cleaner than np.linalg.inv: it uses R' for the
    rotation instead of a general solve, so the result stays exactly rigid.
    """
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def rot_to_rpy(R):
    """Rotation -> roll/pitch/yaw in radians, ZYX (yaw applied last).

    Handles gimbal lock: at pitch = +/-90 deg roll and yaw are degenerate, so
    roll is pinned to 0 and the whole rotation folded into yaw.
    """
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))          # guard acos/asin against 1+1e-16
    pitch = np.arcsin(sy)
    if abs(sy) > 1.0 - 1e-9:
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def rot_to_axis_angle(R):
    """Rotation -> (unit axis, angle in radians)."""
    cos_t = (np.trace(R) - 1.0) / 2.0
    cos_t = max(-1.0, min(1.0, cos_t))
    theta = np.arccos(cos_t)

    if theta < 1e-9:                       # no rotation: axis is arbitrary
        return np.array([0.0, 0.0, 1.0]), 0.0

    if theta > np.pi - 1e-6:
        # Near 180 deg the skew part vanishes; recover the axis from the
        # largest diagonal entry of (R + I)/2, which stays well conditioned.
        M = (R + np.eye(3)) / 2.0
        k = int(np.argmax(np.diag(M)))
        axis = M[:, k] / np.sqrt(max(M[k, k], 1e-12))
        return axis / np.linalg.norm(axis), theta

    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / (2.0 * np.sin(theta))
    return axis / np.linalg.norm(axis), theta


# ─────────────────────────────────────────────────────────────────────────────
#  Input readers
# ─────────────────────────────────────────────────────────────────────────────
def load_from_yaml(path, cam_key, convention):
    """Read a camera-IMU extrinsic and return it as T_imu_cam (camera -> IMU).

    Accepts both spellings found in this repository: Kalibr's T_cam_imu and
    MINS's T_imu_cam. `convention` says which key to prefer; if that key is
    absent the other is used and inverted, with a note printed.
    """
    with open(path) as fh:
        doc = yaml.safe_load(fh)

    if cam_key not in doc:
        raise KeyError("%s: no '%s' section (found: %s)"
                       % (path, cam_key, ", ".join(sorted(doc))))
    block = doc[cam_key]

    wanted, other = ("T_cam_imu", "T_imu_cam") if convention == "cam_imu" \
                    else ("T_imu_cam", "T_cam_imu")

    if wanted in block:
        key, T = wanted, np.array(block[wanted], dtype=float)
    elif other in block:
        key, T = other, np.array(block[other], dtype=float)
        print("    note: asked for %s, found %s — using it." % (wanted, other))
    else:
        raise KeyError("%s/%s: neither T_cam_imu nor T_imu_cam present"
                       % (path, cam_key))

    check_rigid(T, "%s:%s:%s" % (os.path.basename(path), cam_key, key))
    # Normalise to T_imu_cam. Kalibr's T_cam_imu is IMU -> camera, so invert.
    return invert(T) if key == "T_cam_imu" else T


def load_d455_from_device(verbose=True):
    """Read the D455 factory IMU -> infra1 extrinsic straight off the device.

    Returns T_imu_cam (camera -> IMU) to match load_from_yaml.

    This opens the camera, so it fails while the ROS driver holds it. Stop
    leo.service first. It does NOT need the IMU enabled in the ROS launch:
    librealsense exposes the motion stream profile and its extrinsics without
    the ROS driver streaming it.
    """
    try:
        import pyrealsense2 as rs
    except ImportError:
        raise RuntimeError(
            "pyrealsense2 is not installed here.\n"
            "  Install it (pip3 install pyrealsense2) or run this on the robot,\n"
            "  or pass a saved extrinsic with --imu2 <file.yaml>.")

    ctx = rs.context()
    devices = list(ctx.query_devices())
    if not devices:
        raise RuntimeError("no RealSense device found (is leo.service holding it?)")
    dev = devices[0]
    if verbose:
        print("    device: %s  (serial %s)"
              % (dev.get_info(rs.camera_info.name),
                 dev.get_info(rs.camera_info.serial_number)))

    gyro_profile = cam_profile = None
    for sensor in dev.query_sensors():
        for prof in sensor.get_stream_profiles():
            if prof.stream_type() == rs.stream.gyro and gyro_profile is None:
                gyro_profile = prof
            # infra1 is index 1; it is the frame the estimators actually use.
            if (prof.stream_type() == rs.stream.infrared
                    and prof.stream_index() == 1 and cam_profile is None):
                cam_profile = prof

    if gyro_profile is None:
        raise RuntimeError("this unit exposes no gyro stream profile")
    if cam_profile is None:
        raise RuntimeError("no infra1 stream profile found")

    # librealsense gives the transform FROM the first profile TO the second,
    # i.e. p_cam = E * p_gyro, which is T_cam_imu.
    e = gyro_profile.get_extrinsics_to(cam_profile)
    T_cam_imu = np.eye(4)
    # rotation arrives column-major
    T_cam_imu[:3, :3] = np.array(e.rotation, dtype=float).reshape(3, 3).T
    T_cam_imu[:3, 3] = np.array(e.translation, dtype=float)

    check_rigid(T_cam_imu, "D455 factory T_cam_imu")
    return invert(T_cam_imu)


# ─────────────────────────────────────────────────────────────────────────────
#  Reporting
# ─────────────────────────────────────────────────────────────────────────────
def describe(T, title):
    R, t = T[:3, :3], T[:3, 3]
    rpy = np.degrees(rot_to_rpy(R))
    axis, theta = rot_to_axis_angle(R)

    print("\n" + title)
    print("  " + "-" * (len(title) + 2))
    print("  matrix")
    for row in T:
        print("    [%11.7f %11.7f %11.7f %11.7f]" % tuple(row))
    print("  translation   %8.2f %8.2f %8.2f  mm   (norm %.2f mm)"
          % (t[0] * 1e3, t[1] * 1e3, t[2] * 1e3, np.linalg.norm(t) * 1e3))
    print("  rpy (ZYX)     %8.3f %8.3f %8.3f  deg" % tuple(rpy))
    print("  axis-angle    axis [%6.3f %6.3f %6.3f]   angle %7.3f deg"
          % (axis[0], axis[1], axis[2], np.degrees(theta)))


def emit_yaml(T, path):
    doc = {
        "T_imu1_imu2": [[float(v) for v in row] for row in T],
        "_comment": (
            "Rigid transform from imu2 (D455 motion module) to imu1 (CORE2 "
            "body IMU): p_imu1 = T_imu1_imu2 * p_imu2. Computed by "
            "tools/compose_imu_extrinsics.py as T_imu1_cam * inverse(T_imu2_cam)."),
    }
    with open(path, "w") as fh:
        yaml.safe_dump(doc, fh, default_flow_style=False, sort_keys=False)
    print("\n  written: %s" % path)


# ─────────────────────────────────────────────────────────────────────────────
#  Self-test
# ─────────────────────────────────────────────────────────────────────────────
def selftest():
    """Check the composition against a case whose answer is known in advance.

    Two IMUs are placed at chosen poses relative to the camera; the transform
    between them is then known analytically, and the pipeline must recover it.
    This also exercises invert() and the SO(3) guards.
    """
    print("self-test")
    print("  " + "-" * 9)
    rng = np.random.RandomState(0)
    ok = True

    def rand_rigid():
        A = rng.randn(3, 3)
        Q, S = np.linalg.qr(A)
        Q = Q @ np.diag(np.sign(np.diag(S)))       # fix QR sign ambiguity
        if np.linalg.det(Q) < 0:                   # force a rotation, not a flip
            Q[:, 0] *= -1
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = Q, rng.randn(3) * 0.1
        return T

    for trial in range(200):
        T1c, T2c = rand_rigid(), rand_rigid()      # T_imu1_cam, T_imu2_cam
        got = T1c @ invert(T2c)
        # Independent reference: go imu2 -> cam -> imu1 through a point.
        p2 = rng.randn(3)
        p_cam = invert(T2c)[:3, :3] @ p2 + invert(T2c)[:3, 3]
        p1_ref = T1c[:3, :3] @ p_cam + T1c[:3, 3]
        p1_got = got[:3, :3] @ p2 + got[:3, 3]
        if np.abs(p1_ref - p1_got).max() > 1e-9:
            print("  FAIL: composition disagrees on trial %d" % trial)
            ok = False
            break
        check_rigid(got, "composed")

    print("  composition vs point-transport, 200 random pairs : %s"
          % ("pass" if ok else "FAIL"))

    # invert() must undo itself
    T = rand_rigid()
    ok_inv = np.abs(invert(invert(T)) - T).max() < 1e-12
    print("  invert(invert(T)) == T                           : %s"
          % ("pass" if ok_inv else "FAIL"))

    # A reflection must be rejected, not silently composed.
    bad = np.eye(4)
    bad[0, 0] = -1.0
    try:
        check_rigid(bad, "reflection")
        print("  reflection rejected                              : FAIL")
        ok = False
    except ValueError:
        print("  reflection rejected                              : pass")

    # Known-answer case: imu2 is 10 cm along camera +x from imu1, no rotation.
    T1c = np.eye(4)
    T2c = np.eye(4)
    T2c[:3, 3] = [-0.10, 0.0, 0.0]
    got = T1c @ invert(T2c)
    ok_known = np.abs(got[:3, 3] - np.array([0.10, 0.0, 0.0])).max() < 1e-12
    print("  known-answer 100 mm offset                       : %s"
          % ("pass" if ok_known else "FAIL"))

    return 0 if (ok and ok_inv and ok_known) else 1


# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description="Compute T_imu1_imu2 = T_imu1_cam * inverse(T_imu2_cam).",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--imu1", help="YAML holding the body-IMU camera extrinsic")
    ap.add_argument("--imu1-cam", default="cam0", help="section key (default cam0)")
    ap.add_argument("--imu1-convention", default="cam_imu",
                    choices=["cam_imu", "imu_cam"],
                    help="which key that file uses (default cam_imu, Kalibr)")
    ap.add_argument("--imu2", help="YAML holding the D455 IMU camera extrinsic")
    ap.add_argument("--imu2-cam", default="cam0", help="section key (default cam0)")
    ap.add_argument("--imu2-convention", default="cam_imu",
                    choices=["cam_imu", "imu_cam"])
    ap.add_argument("--imu2-from-device", action="store_true",
                    help="read the D455 factory extrinsic off the camera "
                         "(needs the camera free — stop leo.service first)")
    ap.add_argument("-o", "--out", help="write the result to this YAML")
    ap.add_argument("--selftest", action="store_true",
                    help="verify the maths against known answers and exit")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if not args.imu1:
        ap.error("--imu1 is required (or use --selftest)")
    if not args.imu2 and not args.imu2_from_device:
        ap.error("give --imu2 <file> or --imu2-from-device")

    print("=" * 72)
    print("  IMU-to-IMU extrinsic       T_imu1_imu2 = T_imu1_cam * inv(T_imu2_cam)")
    print("=" * 72)
    print("  imu1 = CORE2 body IMU   (the one the estimators consume)")
    print("  imu2 = D455 motion module")

    print("\n  reading imu1 from %s" % args.imu1)
    T_imu1_cam = load_from_yaml(args.imu1, args.imu1_cam, args.imu1_convention)

    if args.imu2_from_device:
        print("  reading imu2 from the device")
        T_imu2_cam = load_d455_from_device()
    else:
        print("  reading imu2 from %s" % args.imu2)
        T_imu2_cam = load_from_yaml(args.imu2, args.imu2_cam, args.imu2_convention)

    describe(T_imu1_cam, "T_imu1_cam   (camera -> body IMU)")
    describe(T_imu2_cam, "T_imu2_cam   (camera -> D455 IMU)")

    T_imu1_imu2 = check_rigid(T_imu1_cam @ invert(T_imu2_cam), "T_imu1_imu2")
    describe(T_imu1_imu2, "T_imu1_imu2  (D455 IMU -> body IMU)   <<< RESULT")

    lever = np.linalg.norm(T_imu1_imu2[:3, 3])
    print("\n  lever arm between the two IMUs: %.1f mm" % (lever * 1e3))
    print("  This is the arm that turns a body rotation into a difference in")
    print("  measured acceleration: a_imu1 - a_imu2 = w x (w x r) + alpha x r.")
    print("  At 1 rad/s the centrifugal term alone is %.4f m/s^2 — compare it"
          % (lever * 1.0 ** 2))
    print("  against the accelerometer noise floor before assuming it matters.")

    if args.out:
        emit_yaml(T_imu1_imu2, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())

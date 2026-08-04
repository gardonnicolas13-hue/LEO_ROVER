#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LEO Rover — real-time contention monitor
========================================
Diagnoses the IMU rate collapse (3.8 Hz observed on the PC, want >=100 Hz) by
logging, side by side and once per interval:

  * effective Hz of critical ROS topics (subscribed with rospy.AnyMsg, so no
    message-type imports are needed — works for any topic, like `rostopic hz`),
  * per-process CPU% (psutil), with the ROS nodes that matter labelled, plus
    system-wide CPU / load / memory.

WHERE TO RUN IT — this is the whole point:
  * ON THE Pi (ssh pi@10.0.0.1):  shows the TRUE publish rate of /firmware/imu
    and which Pi process (serial_node, realsense2_camera, web_video_server,
    rosbridge) is eating the CPU that starves the serial link.
  * ON THE PC:  shows the rate the PC actually RECEIVES over WiFi.
  If Pi-side /firmware/imu is ~74 Hz but PC-side is ~4 Hz  -> WiFi/network bottleneck.
  If Pi-side /firmware/imu is itself ~4 Hz                 -> Pi CPU / serial bottleneck.
That comparison tells you which fix to reach for, so gather both before changing
anything.

Read-only: subscribes and samples psutil. Publishes nothing, changes nothing.

Usage:
  python3 system_monitor.py                       # default critical topics, 2s interval
  python3 system_monitor.py --interval 1 --top 12
  python3 system_monitor.py --topics /firmware/imu,/imu/data_clean,/camera/infra1/image_rect_raw
  python3 system_monitor.py --log /tmp/leo_contention.csv --duration 60
"""
import argparse
import os
import time
import threading
from collections import deque

import psutil

try:
    import rospy
    from rospy import AnyMsg
    _HAS_ROS = True
except Exception:
    _HAS_ROS = False

DEFAULT_TOPICS = [
    "/firmware/imu",                    # raw onboard IMU (rosserial) — the victim
    "/imu/data_clean",                  # sanitized IMU (what VINS/MINS consume)
    "/ov_msckf/odomimu",                # VINS output (if running)
    "/mins/imu/odom",                   # MINS output (if running)
    "/robot_pose_fused",                # pose_selector fused output
    "/camera/infra1/image_rect_raw",    # stereo IR left (bandwidth hog)
    "/camera/color/image_raw/compressed",  # color stream feeding web_video
    "/firmware/wheel_odom",             # wheel odometry
]

# cmdline substring -> friendly label, so the CPU table names the ROS nodes.
NODE_LABELS = [
    ("serial_node", "serial_node (rosserial<->firmware)"),
    ("firmware_message_converter", "firmware_msg_converter"),
    ("realsense2_camera", "realsense2_camera (D455)"),
    ("web_video_server", "web_video_server (MJPEG)"),
    ("rosbridge", "rosbridge_websocket"),
    ("leo_backend", "leo_backend (vision/ctrl)"),
    ("pose_selector", "pose_selector"),
    ("imu_sanitizer", "imu_sanitizer"),
    ("ov_msckf", "openvins (ov_msckf)"),
    ("mins", "mins"),
    ("nodelet", "nodelet manager"),
]


def label_for(cmdline):
    s = " ".join(cmdline) if cmdline else ""
    for needle, lbl in NODE_LABELS:
        if needle in s:
            return lbl
    return None


class TopicRate:
    """Counts messages on a topic via AnyMsg and reports a windowed Hz."""
    def __init__(self, name, window=5.0):
        self.name = name
        self.window = window
        self.stamps = deque()
        self.lock = threading.Lock()
        self.total = 0

    def cb(self, _msg):
        now = time.time()
        with self.lock:
            self.stamps.append(now)
            self.total += 1

    def hz(self):
        now = time.time()
        with self.lock:
            while self.stamps and now - self.stamps[0] > self.window:
                self.stamps.popleft()
            n = len(self.stamps)
        if n < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (n - 1) / span if span > 0 else 0.0


def sample_processes(procs, top_n):
    rows = []
    for p in list(procs.values()):
        try:
            cpu = p.cpu_percent(None)  # % since last call (non-blocking)
            name = label_for(p.cmdline()) or p.name()
            rows.append((cpu, p.pid, name))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    rows.sort(reverse=True)
    return rows[:top_n]


def refresh_proc_table(procs):
    seen = set()
    for p in psutil.process_iter():
        try:
            seen.add(p.pid)
            if p.pid not in procs:
                procs[p.pid] = p
                procs[p.pid].cpu_percent(None)  # prime
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    for pid in list(procs):
        if pid not in seen:
            del procs[pid]


def main():
    ap = argparse.ArgumentParser(description="LEO Rover contention monitor")
    ap.add_argument("--topics", default=",".join(DEFAULT_TOPICS),
                    help="comma-separated topics to measure Hz")
    ap.add_argument("--interval", type=float, default=2.0, help="report period [s]")
    ap.add_argument("--top", type=int, default=10, help="top-N CPU processes")
    ap.add_argument("--duration", type=float, default=0.0, help="0 = run until Ctrl-C")
    ap.add_argument("--log", default="", help="optional CSV log path")
    ap.add_argument("--imu-topic", default="/firmware/imu",
                    help="topic flagged red when below --imu-min")
    ap.add_argument("--imu-min", type=float, default=100.0,
                    help="Hz floor for the IMU health flag")
    args = ap.parse_args()

    topics = [t.strip() for t in args.topics.split(",") if t.strip()]
    rates = {t: TopicRate(t) for t in topics}

    where = "PC"
    if _HAS_ROS:
        master = os.environ.get("ROS_MASTER_URI", "?")
        # crude: if master is localhost we're likely on the Pi (roscore host)
        where = "Pi (roscore host)" if ("localhost" in master or "127.0.0.1" in master
                                        or "10.0.0.1:" in master and os.uname().nodename.startswith("leo")) else "this host"
        rospy.init_node("system_monitor", anonymous=True, disable_signals=True)
        for t, r in rates.items():
            rospy.Subscriber(t, AnyMsg, r.cb, queue_size=200)
        print("[system_monitor] ROS_MASTER_URI=%s  hostname=%s" % (master, os.uname().nodename))
    else:
        print("[system_monitor] rospy unavailable — CPU only, no topic Hz "
              "(source /opt/ros/noetic/setup.bash to enable topic rates)")

    logf = open(args.log, "w") if args.log else None
    if logf:
        logf.write("time," + ",".join("hz:%s" % t for t in topics) +
                   ",cpu_total," + ",".join("cpu%d:proc" % i for i in range(args.top)) + "\n")

    procs = {}
    refresh_proc_table(procs)
    ncpu = psutil.cpu_count()
    t_start = time.time()
    tick = 0
    try:
        while True:
            time.sleep(args.interval)
            tick += 1
            if tick % 5 == 0:
                refresh_proc_table(procs)

            cpu_total = psutil.cpu_percent(None)
            try:
                load1 = os.getloadavg()[0]
            except OSError:
                load1 = float("nan")
            mem = psutil.virtual_memory().percent
            top = sample_processes(procs, args.top)

            print("\n=== %s  (host role: %s)  CPU %.0f%% of %d cores | load1 %.2f | mem %.0f%% ==="
                  % (time.strftime("%H:%M:%S"), where, cpu_total, ncpu, load1, mem))
            if _HAS_ROS:
                cells = []
                for t in topics:
                    hz = rates[t].hz()
                    flag = ""
                    if t == args.imu_topic and 0 < hz < args.imu_min:
                        flag = " <-- LOW"
                    elif hz == 0.0:
                        flag = " (silent)"
                    cells.append("  %-38s %6.1f Hz%s" % (t, hz, flag))
                print("\n".join(cells))
            print("  --- top CPU processes ---")
            for cpu, pid, name in top:
                print("  %6.1f%%  pid %-7d %s" % (cpu, pid, name))

            if logf:
                hzs = [("%.1f" % rates[t].hz()) for t in topics] if _HAS_ROS else ["" for _ in topics]
                procs_str = [("%s@%.0f%%" % (n, c)) for c, _, n in top]
                procs_str += [""] * (args.top - len(procs_str))
                logf.write(",".join([("%.1f" % (time.time() - t_start))] + hzs +
                                    ["%.0f" % cpu_total] + procs_str) + "\n")
                logf.flush()

            if args.duration and (time.time() - t_start) >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if logf:
            logf.close()
            print("\n[system_monitor] log written to %s" % args.log)


if __name__ == "__main__":
    main()

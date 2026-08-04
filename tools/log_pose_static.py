#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
T1.1 static drift test: logs /robot_pose_fused (X, Y, Z) for N seconds while
the rover sits still, to a CSV. Run from the PC (pose_selector/MINS already
publish there) -- no robot shell access needed for this part.

Usage: python3 log_pose_static.py [duration_s] [out_csv]
"""
import sys
import csv
import time
import rospy
from nav_msgs.msg import Odometry

duration = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
out_csv = sys.argv[2] if len(sys.argv) > 2 else "/tmp/t1_1_static_pose.csv"

rows = []


def cb(msg):
    t = msg.header.stamp.to_sec()
    p = msg.pose.pose.position
    rows.append((t, p.x, p.y, p.z))


rospy.init_node("log_pose_static", anonymous=True)
rospy.Subscriber("/robot_pose_fused", Odometry, cb)

print("Logging /robot_pose_fused for %.0fs -- keep the rover PERFECTLY STILL." % duration)
t0 = time.time()
rate = rospy.Rate(20)
while time.time() - t0 < duration and not rospy.is_shutdown():
    rate.sleep()

with open(out_csv, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["stamp", "x", "y", "z"])
    w.writerows(rows)

print("Wrote %d samples to %s" % (len(rows), out_csv))

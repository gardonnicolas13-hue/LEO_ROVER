#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LEO Rover — Carolus -> MINS "indoor GPS" bridge
===============================================
Turns the Carolus beacon system into a drift-free global anchor for MINS by
republishing the rover's global pose as geometry_msgs/PoseStamped on
/carolus/vicon_pose — the exact type MINS's VICON input consumes (a global
6-DOF fix in a fixed world frame). This is the "virtual GPS": whenever a known
beacon is in view, MINS gets a global position/orientation fix that cancels the
unbounded drift of visual-inertial + wheel odometry.

HOW IT WORKS: the earlier Carolus stack (tf_bridge.py / tf2_code.py) publishes a
fixed, ground-anchored `beacon_link` frame on the first beacon detection, and the
robot's pose is expressed relative to it on the TF tree. This node simply looks
up TF (global_frame -> body_frame) at a fixed rate and republishes it as a
PoseStamped stamped in the global frame. When no beacon has been seen (TF
unavailable) it publishes nothing — correct, because there is no global fix to
give, and MINS just keeps propagating on IMU/camera/wheel until the next fix.

Why TF instead of subscribing to a Carolus pose topic directly: it reuses the
existing, already-validated beacon_link frame chain (the Carolus doc's TF2 fix)
without assuming a particular Carolus message type, and it naturally yields the
pose of whatever body frame MINS's T_imu_vicon is referenced to (base_link).

PUBLISHES  ~out_topic (défaut /mins/external_ref/carolus)  geometry_msgs/PoseStamped

PARAMS
  ~global_frame  (default beacon_link)  fixed world frame from Carolus tf_bridge
  ~body_frame    (default base_link)    body point whose global pose we report
  ~rate_hz       (default 10.0)
  ~max_tf_age    (default 0.5 s)  skip stale lookups (beacon out of view)

LAUNCH: rosrun leo_navigation carolus_vicon_bridge.py
  (requires the Carolus stack running so beacon_link exists on the TF tree)
"""
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


class CarolusViconBridge(object):
    def __init__(self):
        self.global_frame = rospy.get_param("~global_frame", "beacon_link")
        self.body_frame = rospy.get_param("~body_frame", "base_link")
        self.rate_hz = rospy.get_param("~rate_hz", 10.0)
        self.max_tf_age = rospy.get_param("~max_tf_age", 0.5)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        # /mins/external_ref/... : nommage PRIVÉ (verrouillage 2026-07-08) —
        # ce flux est une référence externe destinée EXCLUSIVEMENT au slot
        # VICON de MINS ; le préfixe rend toute autre consommation visible
        # comme une anomalie (règle de surveillance dans leo_watchdog).
        out_topic = rospy.get_param("~out_topic", "/mins/external_ref/carolus")
        self.pub = rospy.Publisher(out_topic, PoseStamped, queue_size=10)

        self._had_fix = False
        rospy.loginfo("[carolus_vicon_bridge] %s -> %s @ %.1f Hz -> %s",
                       self.global_frame, self.body_frame, self.rate_hz, out_topic)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                tf = self.buffer.lookup_transform(
                    self.global_frame, self.body_frame, rospy.Time(0),
                    rospy.Duration(0.05))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                if self._had_fix:
                    rospy.loginfo_throttle(
                        10.0, "[carolus_vicon_bridge] no beacon fix (TF %s->%s "
                        "unavailable) — MINS propagates on IMU/cam/wheel until next fix",
                        self.global_frame, self.body_frame)
                    self._had_fix = False
                rate.sleep()
                continue

            age = (rospy.Time.now() - tf.header.stamp).to_sec()
            if age > self.max_tf_age:
                rate.sleep()
                continue

            ps = PoseStamped()
            ps.header.stamp = tf.header.stamp
            ps.header.frame_id = self.global_frame
            ps.pose.position.x = tf.transform.translation.x
            ps.pose.position.y = tf.transform.translation.y
            ps.pose.position.z = tf.transform.translation.z
            ps.pose.orientation = tf.transform.rotation
            self.pub.publish(ps)
            if not self._had_fix:
                rospy.loginfo("[carolus_vicon_bridge] beacon fix acquired — "
                               "publishing global pose to MINS")
                self._had_fix = True
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("carolus_vicon_bridge")
    CarolusViconBridge().spin()

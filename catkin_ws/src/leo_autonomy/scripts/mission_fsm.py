#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LEO Rover — machine à états mission (smach)
============================================
Boucle : EXPLORE -> (obstacle proche) REFLEX_UTURN -> EXPLORE
                 -> (AprilTag vu)     TAG_APPROACH -> TAG_RESET -> EXPLORE

  EXPLORE       explore_lite pilote move_base vers les frontières ; cet état
                surveille seulement les deux interruptions (obstacle, tag).
  REFLEX_UTURN  obstacle < seuil dans la depth : annulation du goal move_base,
                demi-tour 180 degres en boucle ouverte (gyro), reprise.
  TAG_APPROACH  annulation du goal, servo visuel simple sur la pose du tag
                (apriltag_ros) jusqu'a APPROACH_DIST.
  TAG_RESET     publication de la pose corrigee sur /rtabmap/initialpose
                (relocalisation) — la correction PROPRE de la derive passe
                aussi par les landmarks AprilTag integres au graphe RTAB-Map
                (voir autonomy.launch), ce reset est le rituel explicite.

Interfaces :
  IN  /depth_scan (LaserScan, via depthimage_to_laserscan)
      /tag_detections (apriltag_ros/AprilTagDetectionArray)
      /robot_pose_fused (nav_msgs/Odometry — MINS/VINS via pose_selector)
      /firmware/imu -> non : yaw pris de /robot_pose_fused (deja fusionne)
  OUT /cmd_vel_fsm (geometry_msgs/Twist, priorite 50 dans twist_mux)
      /move_base (actionlib) — annulation/gel des goals
      /leo_autonomy/state (std_msgs/String) — pour le cockpit
"""
import math
import threading

import rospy
import smach
import smach_ros
import actionlib
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
try:
    from apriltag_ros.msg import AprilTagDetectionArray
    HAVE_APRILTAG = True
except ImportError:          # package pas encore installé : FSM dégradée sans tags
    HAVE_APRILTAG = False

OBSTACLE_DIST   = 0.45   # m — seuil réflexe (depth scan, secteur frontal)
UTURN_SPEED     = 0.6    # rad/s
APPROACH_DIST   = 0.60   # m — distance d'arrêt devant le tag
APPROACH_SPEED  = 0.15   # m/s
APPROACH_KP_ANG = 1.2    # gain de centrage sur le tag
TAG_COOLDOWN    = 20.0   # s — anti-redéclenchement après un reset


class Blackboard(object):
    """État partagé entre les états smach (capteurs + publications)."""

    def __init__(self):
        self.obstacle = False
        self.tag_pose = None          # geometry_msgs/Pose du tag le plus proche (repère caméra)
        self.tag_seen_t = 0.0
        self.last_reset_t = 0.0
        self.yaw = None
        self.pose = None              # (x, y) monde

        self.cmd_pub = rospy.Publisher("/cmd_vel_fsm", Twist, queue_size=5)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=2)
        self.state_pub = rospy.Publisher("/leo_autonomy/state", String,
                                         queue_size=2, latch=True)
        self.initpose_pub = rospy.Publisher("/rtabmap/initialpose",
                                            PoseWithCovarianceStamped, queue_size=2)

        rospy.Subscriber("/depth_scan", LaserScan, self._on_scan, queue_size=2)
        rospy.Subscriber("/robot_pose_fused", Odometry, self._on_odom, queue_size=5)
        if HAVE_APRILTAG:
            rospy.Subscriber("/tag_detections", AprilTagDetectionArray,
                             self._on_tags, queue_size=2)

    def _on_scan(self, msg):
        # secteur frontal ±25° du depth_scan
        n = len(msg.ranges)
        if n == 0:
            return
        half = int(math.radians(25) / max(msg.angle_increment, 1e-6))
        mid = n // 2
        sector = msg.ranges[max(0, mid - half):min(n, mid + half)]
        valid = [r for r in sector if msg.range_min < r < msg.range_max]
        if len(valid) > 5:
            close = sorted(valid)[max(1, len(valid) // 20)]  # ~5e centile
            self.obstacle = close < OBSTACLE_DIST

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _on_tags(self, msg):
        if not msg.detections:
            return
        det = min(msg.detections,
                  key=lambda d: d.pose.pose.pose.position.z)
        self.tag_pose = det.pose.pose.pose
        self.tag_seen_t = rospy.Time.now().to_sec()

    # helpers ---------------------------------------------------------------
    def drive(self, lin, ang):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

    def stop(self):
        self.drive(0.0, 0.0)

    def cancel_goals(self):
        self.cancel_pub.publish(GoalID())   # goal vide = tout annuler

    def tag_fresh(self):
        return (self.tag_pose is not None
                and rospy.Time.now().to_sec() - self.tag_seen_t < 0.7)

    def set_state(self, s):
        self.state_pub.publish(String(data=s))


class Explore(smach.State):
    """explore_lite travaille en tâche de fond ; on surveille les interruptions."""

    def __init__(self, bb):
        smach.State.__init__(self, outcomes=["obstacle", "tag", "shutdown"])
        self.bb = bb

    def execute(self, ud):
        self.bb.set_state("EXPLORE")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if (self.bb.tag_fresh()
                    and now - self.bb.last_reset_t > TAG_COOLDOWN):
                self.bb.cancel_goals()
                return "tag"
            if self.bb.obstacle:
                self.bb.obstacle = False
                self.bb.cancel_goals()
                return "obstacle"
            rate.sleep()
        return "shutdown"


class ReflexUturn(smach.State):
    """Demi-tour immédiat en boucle ouverte sur le yaw fusionné."""

    def __init__(self, bb):
        smach.State.__init__(self, outcomes=["done", "shutdown"])
        self.bb = bb

    def execute(self, ud):
        self.bb.set_state("REFLEX_UTURN")
        rospy.loginfo("[FSM] obstacle < %.2fm — demi-tour reflexe", OBSTACLE_DIST)
        start_yaw = self.bb.yaw
        accum = 0.0
        last = start_yaw
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.bb.drive(0.0, UTURN_SPEED)
            y = self.bb.yaw
            if y is not None and last is not None:
                d = math.atan2(math.sin(y - last), math.cos(y - last))
                accum += abs(d)
            last = y
            if accum >= math.pi:
                self.bb.stop()
                return "done"
            rate.sleep()
        self.bb.stop()
        return "shutdown"


class TagApproach(smach.State):
    """Servo visuel : centre le tag, avance jusqu'à APPROACH_DIST."""

    def __init__(self, bb):
        smach.State.__init__(self, outcomes=["reached", "lost", "shutdown"])
        self.bb = bb

    def execute(self, ud):
        self.bb.set_state("TAG_APPROACH")
        rospy.loginfo("[FSM] AprilTag detecte — approche")
        lost_since = None
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if not self.bb.tag_fresh():
                lost_since = lost_since or rospy.Time.now().to_sec()
                if rospy.Time.now().to_sec() - lost_since > 2.0:
                    self.bb.stop()
                    return "lost"
                self.bb.drive(0.0, 0.0)
                rate.sleep()
                continue
            lost_since = None
            p = self.bb.tag_pose.position
            # repère optique : x = droite, z = profondeur
            if p.z <= APPROACH_DIST:
                self.bb.stop()
                return "reached"
            ang = -APPROACH_KP_ANG * math.atan2(p.x, p.z)
            self.bb.drive(APPROACH_SPEED, max(-0.6, min(0.6, ang)))
            rate.sleep()
        self.bb.stop()
        return "shutdown"


class TagReset(smach.State):
    """Reset de la dérive : pose courante re-publiée comme initialpose RTAB-Map.
    (La correction fine est déjà faite en continu par les landmarks AprilTag
    dans le graphe SLAM ; ce reset explicite reproduit le rituel balise.)"""

    def __init__(self, bb):
        smach.State.__init__(self, outcomes=["done"])
        self.bb = bb

    def execute(self, ud):
        self.bb.set_state("TAG_RESET")
        self.bb.last_reset_t = rospy.Time.now().to_sec()
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        if self.bb.pose is not None:
            msg.pose.pose.position.x = self.bb.pose[0]
            msg.pose.pose.position.y = self.bb.pose[1]
        if self.bb.yaw is not None:
            msg.pose.pose.orientation.z = math.sin(self.bb.yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(self.bb.yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05 ** 2
        msg.pose.covariance[35] = math.radians(3) ** 2
        self.bb.initpose_pub.publish(msg)
        rospy.loginfo("[FSM] reset publie sur /rtabmap/initialpose — reprise mission")
        rospy.sleep(1.0)
        return "done"


def main():
    rospy.init_node("mission_fsm")
    bb = Blackboard()

    sm = smach.StateMachine(outcomes=["shutdown"])
    with sm:
        smach.StateMachine.add("EXPLORE", Explore(bb),
                               transitions={"obstacle": "REFLEX_UTURN",
                                            "tag": "TAG_APPROACH",
                                            "shutdown": "shutdown"})
        smach.StateMachine.add("REFLEX_UTURN", ReflexUturn(bb),
                               transitions={"done": "EXPLORE",
                                            "shutdown": "shutdown"})
        smach.StateMachine.add("TAG_APPROACH", TagApproach(bb),
                               transitions={"reached": "TAG_RESET",
                                            "lost": "EXPLORE",
                                            "shutdown": "shutdown"})
        smach.StateMachine.add("TAG_RESET", TagReset(bb),
                               transitions={"done": "EXPLORE"})

    sis = smach_ros.IntrospectionServer("leo_mission_fsm", sm, "/LEO_MISSION")
    sis.start()
    outcome = sm.execute()
    sis.stop()
    rospy.loginfo("[FSM] terminee: %s", outcome)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LEO Rover — wheel-order remap for MINS
=======================================
Republishes /joint_states as a 2-element JointState with velocity[0]=front-left,
velocity[1]=front-right, matching what MINS's ROSHelper::JointState2Data()
hard-codes by INDEX (not name):
    data.m1 = msg->velocity.at(0);  // comment says "front_left"
    data.m2 = msg->velocity.at(1);  // comment says "front_right"

WHY THIS EXISTS: this rover's real /joint_states orders wheels as
  name: [wheel_FL_joint, wheel_RL_joint, wheel_FR_joint, wheel_RR_joint]
(confirmed live, 2026-07-03) i.e. velocity[1] is REAR-left, not front-right.
Feeding /joint_states to MINS directly would silently hand it the rear-left
wheel as its "right wheel" — wrong odometry constraint, hard to diagnose later
(looks like bad wheel calibration, not a wiring bug). This node looks the two
front wheels up BY NAME (robust to any future firmware reordering) and
republishes them in the index order MINS assumes.

4-ENCODER AVERAGING (2026-07-10): the rover is skid-steer — during any turn
the wheels slip by design, and front/rear wheels of one side grip differently.
Feeding MINS a single wheel per side makes its differential model hostage to
whichever wheel slips more. velocity[i] is now the MEAN of the two wheels of
that side ((FL+RL)/2, (FR+RR)/2): the per-side average is a better estimate of
the side's effective tread speed and halves encoder noise. Falls back to the
front wheel alone if a rear joint is absent from the message.

PUBLISHES  ~out_topic (default /joint_states_mins)   sensor_msgs/JointState
             name=[front_left_name, front_right_name],
             velocity=[(v_FL+v_RL)/2, (v_FR+v_RR)/2]
SUBSCRIBES ~in_topic  (default /joint_states)

LAUNCH: rosrun leo_navigation wheel_remap.py
"""
import rospy
from sensor_msgs.msg import JointState


class WheelRemap(object):
    def __init__(self):
        self.fl_name = rospy.get_param("~front_left_name", "wheel_FL_joint")
        self.fr_name = rospy.get_param("~front_right_name", "wheel_FR_joint")
        self.rl_name = rospy.get_param("~rear_left_name", "wheel_RL_joint")
        self.rr_name = rospy.get_param("~rear_right_name", "wheel_RR_joint")
        out_topic = rospy.get_param("~out_topic", "/joint_states_mins")
        in_topic = rospy.get_param("~in_topic", "/joint_states")

        self.pub = rospy.Publisher(out_topic, JointState, queue_size=20)
        self._warned_missing = False
        rospy.Subscriber(in_topic, JointState, self._cb, queue_size=20)
        rospy.loginfo("[wheel_remap] %s -> %s : velocity[0]=%s, velocity[1]=%s "
                       "(MINS reads these two by INDEX, not name)",
                       in_topic, out_topic, self.fl_name, self.fr_name)

    def _cb(self, msg):
        try:
            i_fl = msg.name.index(self.fl_name)
            i_fr = msg.name.index(self.fr_name)
        except ValueError:
            if not self._warned_missing:
                rospy.logerr("[wheel_remap] '%s' or '%s' not found in /joint_states "
                             "name[] (got %s) — not publishing until it appears",
                             self.fl_name, self.fr_name, list(msg.name))
                self._warned_missing = True
            return
        self._warned_missing = False

        # moyenne par côté si la roue arrière est présente (skid-steer :
        # une roue seule est otage de son patinage propre)
        def side_mean(i_front, rear_name):
            try:
                i_rear = msg.name.index(rear_name)
                return 0.5 * (msg.velocity[i_front] + msg.velocity[i_rear])
            except (ValueError, IndexError):
                return msg.velocity[i_front]

        out = JointState()
        out.header = msg.header
        out.name = [self.fl_name, self.fr_name]
        out.velocity = [side_mean(i_fl, self.rl_name),
                        side_mean(i_fr, self.rr_name)]
        if msg.position:
            out.position = [msg.position[i_fl], msg.position[i_fr]]
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("wheel_remap")
    WheelRemap()
    rospy.spin()

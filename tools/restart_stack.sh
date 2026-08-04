#!/bin/bash
# Redémarre la stack navigation proprement (drapeau maintenance pour le
# watchdog), attend l'init MINS, rebascule la source sur MINS.
# REMÈDE DÉTERMINISTE aux nœuds zombies après redémarrage du roscore robot.
# (Version pérenne 2026-07-13 — vivait dans le scratchpad volatile, purgé 2x.)
# Attente MINS via rospy (rostopic echo rame sur xmlrpc quand le PC/Pi est
# chargé -> faux négatifs), timeout -k partout (sonde jamais éternelle).
# shellcheck disable=SC1091
source /home/lab272/TOUT/tools/robot_env.sh   # ROBOT_HOST/ROS_MASTER_URI/ROS_IP centralisés
source /opt/ros/noetic/setup.bash

touch /tmp/leo_maintenance
trap 'rm -f /tmp/leo_maintenance' EXIT

kill -INT $(pgrep -f "roslaunch leo_navigation navigation_master") 2>/dev/null
t0=$(date +%s)
while pgrep -x subscribe > /dev/null 2>&1; do
  [ $(( $(date +%s) - t0 )) -gt 60 ] && { echo "ECHEC: MINS ne s'arrête pas"; exit 1; }
  sleep 2
done

nohup /home/lab272/TOUT/catkin_ws/src/leo_navigation/launch_mins.sh navigation_master.launch \
      > /home/lab272/TOUT/logs/navigation_master.log 2>&1 &
echo "stack relancée"

t0=$(date +%s)
until timeout -k 5 15 python3 -c "
import rospy
from nav_msgs.msg import Odometry
rospy.init_node('wait_mins', anonymous=True)
rospy.wait_for_message('/mins/imu/odom', Odometry, timeout=12)
" > /dev/null 2>&1; do
  [ $(( $(date +%s) - t0 )) -gt 300 ] && { echo "TIMEOUT init MINS"; exit 2; }
  sleep 3
done
echo "MINS initialisé après $(( $(date +%s) - t0 ))s"

# laser_power 0 / png_level 1 (2026-07-13, MESURÉ — voir leo_watchdog.sh) :
# un restart de la caméra remet ces réglages dynamic_reconfigure à leur
# valeur d'usine (laser 150, png 9), et jusqu'ici SEULE la branche "caméra
# muette" du watchdog les réappliquait — un restart_stack.sh (celui-ci,
# déclenché aussi par le watchdog lui-même via ses gardes VINS/ZOMBIES)
# redémarre pourtant tout autant la caméra sans jamais repasser par cette
# branche. Régression constatée en direct le 2026-07-24 (laser_power=150
# après plusieurs restart_stack.sh consécutifs, damier/LEDs mitraillés de
# points IR) : réappliqué ici aussi, au même titre que le watchdog.
timeout -k 5 15 rosrun dynamic_reconfigure dynparam set /camera/stereo_module laser_power 0 > /dev/null 2>&1
timeout -k 5 15 rosrun dynamic_reconfigure dynparam set /camera/depth/image_rect_raw/compressedDepth png_level 1 > /dev/null 2>&1

for i in 1 2 3; do
  out=$(timeout -k 5 10 rosservice call /pose_selector/set_source "data: true" 2>&1)
  echo "$out" | grep -q "success: True" && { echo "  switched to MINS"; break; }
  sleep 3
done
echo "TERMINE"

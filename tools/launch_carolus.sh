#!/usr/bin/env bash
# ═════════════════════════════════════════════════════════════════════════════
# launch_carolus.sh — démarre la détection Carolus (carolus_astrobee) sur le PC.
#
#   Indépendant du stack MINS : source carolus_ws (où vit le paquet
#   carolus_node) + l'env robot (ROS_MASTER_URI vers le roscore du robot), puis
#   roslaunch tools/carolus_detect.launch. Le nœud s'abonne au flux couleur
#   republié /pc/camera/color/image_raw et publie la pose 6-DOF de la balise sur
#   /pose (+ /loc/ar/features).
#
#   PRÉ-REQUIS :
#     - flux couleur activé sur le robot (enable_color=true dans
#       ~/d455_minimal.launch) ET color_republish actif (dans le stack nav).
#     - carolus_ws construit (carolus_ws/devel présent).
#
#   Usage :  tools/launch_carolus.sh      (Ctrl-C pour arrêter)
# ═════════════════════════════════════════════════════════════════════════════
set -e
# shellcheck disable=SC1091
source /home/lab272/TOUT/tools/robot_env.sh      # ROS_MASTER_URI / ROS_IP
source /opt/ros/noetic/setup.bash
source /home/lab272/TOUT/carolus_ws/devel/setup.bash
# Le devel de carolus_ws a été construit à un ancien emplacement
# (/home/lab272/carolus_ws, aujourd'hui absent) : son setup.bash grave un
# ROS_PACKAGE_PATH MORT. On force le vrai chemin src pour que roslaunch
# trouve le paquet carolus_node (le binaire, lui, est bien résolu via
# CMAKE_PREFIX_PATH que le devel pointe correctement sur TOUT/).
export ROS_PACKAGE_PATH="/home/lab272/TOUT/carolus_ws/src:$ROS_PACKAGE_PATH"

echo "[launch_carolus] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[launch_carolus] carolus_node : $(rospack find carolus_node 2>/dev/null || echo INTROUVABLE)"
echo "[launch_carolus] entrée : /pc/camera/color/image_raw  ->  sortie : /pose"
exec roslaunch "$(dirname "$0")/carolus_detect.launch"

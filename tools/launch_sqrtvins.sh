#!/bin/bash
# ============================================================================
# launch_sqrtvins.sh — demarre sqrtVINS (ov_srvins) sur le PC, 2026-08-21.
#
# POURQUOI UN SCRIPT DEDIE PLUTOT QU'UN <include> DANS navigation_master :
# sqrtVINS embarque son PROPRE ov_core, homonyme de celui d'openVINS dans
# catkin_ws. Superposer les deux workspaces rend le paquet ov_core ambigu et
# fait resoudre headers et bibliotheques au hasard de l'ordre de superposition
# — un bug silencieux, qui ne se voit qu'a l'execution. sqrtvins_ws est donc
# configure avec --extend /opt/ros/noetic UNIQUEMENT, et ce script ne source
# que lui. C'est aussi ce qui garantit que le SANCTUAIRE MINS n'est jamais
# effleure : ce script ne connait ni mins_ws ni catkin_ws.
#
# Il suppose un roscore DEJA en marche (celui du robot) et la pile de
# navigation deja lancee — sqrtVINS ne consomme que des topics republies
# (/pc/camera/... et /imu/data_clean), il n'en produit aucun dont MINS ou
# openVINS dependent.
#
# Usage :
#   tools/launch_sqrtvins.sh                  # profil PC par defaut
#   tools/launch_sqrtvins.sh verbosity:=ALL   # arguments roslaunch relayes
# ============================================================================
set -o pipefail

WS=/home/lab272/TOUT/sqrtvins_ws
LOG=/home/lab272/TOUT/logs/sqrtvins.log

# shellcheck disable=SC1091
source /home/lab272/TOUT/tools/robot_env.sh   # ROBOT_HOST / ROS_MASTER_URI / ROS_IP

if [ ! -f "$WS/devel/setup.bash" ]; then
  echo "[sqrtvins] ERREUR : $WS n'est pas compile."
  echo "           cd $WS && catkin build"
  exit 1
fi
# shellcheck disable=SC1091
source "$WS/devel/setup.bash"

# Garde-fou de precision : ce script existe pour lancer la variante DOUBLE
# precision. Si le workspace a ete recompile en simple precision sans qu'on
# s'en apercoive, on le dit AVANT de mesurer quoi que ce soit — c'est
# exactement le piege qui a coute la campagne de mesures precedente.
CACHE="$WS/build/ov_srvins/CMakeCache.txt"
if [ -f "$CACHE" ]; then
  PREC=$(grep -i '^USE_FLOAT' "$CACHE" | cut -d= -f2)
  if [ "$PREC" != "OFF" ]; then
    echo "[sqrtvins] AVERTISSEMENT : USE_FLOAT=$PREC (simple precision)."
    echo "           Le build attendu est USE_FLOAT=OFF. Recompiler avec :"
    echo "           cd $WS && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_FLOAT=OFF && catkin build --force-cmake"
  else
    echo "[sqrtvins] precision : double (USE_FLOAT=OFF) OK"
  fi
fi

echo "[sqrtvins] ROS_MASTER_URI=$ROS_MASTER_URI  ROS_IP=$ROS_IP"
echo "[sqrtvins] journal -> $LOG"
mkdir -p "$(dirname "$LOG")"

exec roslaunch ov_srvins sqrtvins_pc.launch "$@" 2>&1 | tee -a "$LOG"

#!/bin/bash
# ============================================================================
# launch_sqrtvins_pi.sh — sqrtVINS EN LOCAL sur le Raspberry Pi du LEO Rover.
# Depose le 2026-08-21 depuis le PC. Pendant du PC tools/launch_sqrtvins.sh.
#
# DIFFERENCE AVEC LA VERSION PC, ET POURQUOI ELLE COMPTE :
#   - ici les topics camera sont /camera/... (locaux, aucun saut WiFi) et
#     l'IMU est /imu/data_clean_pi (instance Pi) ;
#   - le build est USE_FLOAT=ON (simple precision). Ce n'est PAS un oubli :
#     le papier sqrtVINS revendique la simple precision 32 bits comme son
#     argument central (« twice the speed », Abstract). Le bug qui faisait
#     diverger le filtre n'etait PAS numerique mais logique, donc la simple
#     precision redevient un choix legitime -- et le bon pour un ARM.
#   - nom de noeud ov_srvins, distinct du noeud « sqrtvins » du PC : les deux
#     partagent le MEME roscore, deux homonymes s'entretueraient.
#
# CHARGE CPU : ce Pi fait deja tourner le pilote camera (nodelet ~100 % d'un
# coeur) et serial_node en SCHED_FIFO 25. sqrtVINS est un VIO complet : ne pas
# le laisser tourner en continu sans surveiller les overruns UART
# (/firmware/wheel_states doit rester a ~20 Hz).
#
# Usage : ./launch_sqrtvins_pi.sh [duree_s]   (0 ou absent = sans limite)
# ============================================================================
set -o pipefail
DUREE="${1:-0}"
LOG=/home/pi/sqrtvins_ws/sqrtvins_pi.log

source /opt/ros/noetic/setup.bash
if [ ! -f /home/pi/sqrtvins_ws/devel/setup.bash ]; then
  echo "[sqrtvins-pi] ERREUR : workspace non compile"; exit 1
fi
source /home/pi/sqrtvins_ws/devel/setup.bash

# Garde de correctifs : refuse de demarrer un binaire dont le source ne porte
# pas les deux corrections ZUPT -- sans elles le filtre diverge a 1e15 m et
# une mesure faite avec serait a jeter.
SRC=/home/pi/sqrtvins_ws/src/sqrtVINS/ov_srvins/src
manque=0
grep -q 'block(3, ba_id'  "$SRC/state/StateHelper.cpp"        || { echo "[sqrtvins-pi] MANQUE : correctif bruit de biais"; manque=1; }
grep -q 'chi2_hors_seuil' "$SRC/update/UpdaterZeroVelocity.cpp" || { echo "[sqrtvins-pi] MANQUE : correctif garde chi2"; manque=1; }
[ "$manque" = 1 ] && { echo "[sqrtvins-pi] demarrage refuse."; exit 2; }
echo "[sqrtvins-pi] correctifs ZUPT presents (biais + garde chi2) OK"

echo "[sqrtvins-pi] journal -> $LOG"
if [ "$DUREE" != "0" ]; then
  echo "[sqrtvins-pi] duree limitee a ${DUREE}s"
  exec timeout -k 5 "$DUREE" roslaunch ov_srvins subscribe.launch config:=leo 2>&1 | tee -a "$LOG"
else
  exec roslaunch ov_srvins subscribe.launch config:=leo 2>&1 | tee -a "$LOG"
fi

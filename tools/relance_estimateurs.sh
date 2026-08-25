#!/bin/bash
# ============================================================================
# relance_estimateurs.sh — remise à zéro propre des trois estimateurs.
# Écrit le 2026-08-21 après la campagne dynamique.
#
# POURQUOI CE SCRIPT PLUTÔT QU'UN `leo restart`
# ---------------------------------------------
# `leo restart` reconstruit toute la couche web/cockpit/tunnel en plus de la
# pile. Ici on ne veut QUE remettre les estimateurs à zéro : leur état interne
# (position, vitesse, biais, covariance) est ce qui se corrompt, et il ne se
# nettoie que par un redémarrage de nœud. Toucher au reste rallonge la
# procédure et ajoute des façons d'échouer.
#
# CE QU'IL VÉRIFIE AVANT DE LANCER, ET POURQUOI
# ---------------------------------------------
# Trois conditions matérielles ont fait mentir des mesures le 2026-08-21 :
#   - PC en surchauffe (86 °C relevés, seuil constructeur 82 °C) : un PC qui
#     throttle fait décrocher les filtres, et on mesure alors son propre
#     navigateur au lieu de ses estimateurs ;
#   - Firefox à 92 % d'un cœur (deux fenêtres cockpit, vidéo + canvas) —
#     davantage que MINS lui-même ;
#   - batterie sous 30 % : le lacet ne tient 92-99 % qu'à batterie saine
#     (mesuré le 21/07), donc une rotation commandée n'est plus la rotation
#     réelle, et toute mesure de dérive angulaire devient ininterprétable.
# Le script REFUSE de démarrer si l'une est violée, sauf --force.
#
# Usage :
#   tools/relance_estimateurs.sh          # vérifie puis relance
#   tools/relance_estimateurs.sh --force  # relance malgré un avertissement
#   tools/relance_estimateurs.sh --check  # vérifie seulement, ne relance pas
# ============================================================================
set -o pipefail
cd /home/lab272/TOUT || exit 1
# shellcheck disable=SC1091
source tools/robot_env.sh
source /opt/ros/noetic/setup.bash

FORCE=0; CHECK_ONLY=0
for a in "$@"; do
  [ "$a" = "--force" ] && FORCE=1
  [ "$a" = "--check" ] && CHECK_ONLY=1
done

echo "── Vérifications matérielles ────────────────────────────────────────"
souci=0

# Mediane sur 5 lectures espacees : la temperature d'un CPU oscille de plus de
# 10 degres en quelques secondes selon la charge instantanee (mesure : 86 puis
# 75 en moins d'une minute). Une lecture unique attrape un pic et refuse un
# banc parfaitement utilisable -- ou l'inverse, ce qui serait pire.
TEMP=$(for _ in 1 2 3 4 5; do
         sensors 2>/dev/null | grep -oP 'Package id 0:\s+\+\K[0-9]+' | head -1
         sleep 1
       done | sort -n | awk '{a[NR]=$1} END{print a[int((NR+1)/2)]}')
if [ -n "$TEMP" ]; then
  if [ "$TEMP" -ge 82 ]; then
    echo "  [!] PC à ${TEMP} °C (seuil 82) — laisser refroidir"; souci=1
  else
    echo "  ok  PC à ${TEMP} °C"
  fi
fi

FFCPU=$(ps -eo pcpu,comm --sort=-pcpu | awk '$2=="firefox"{s+=$1} END{printf "%.0f", s+0}')
if [ "${FFCPU:-0}" -ge 50 ]; then
  echo "  [!] Firefox à ${FFCPU} % CPU — fermer une fenêtre cockpit"; souci=1
else
  echo "  ok  Firefox à ${FFCPU:-0} % CPU"
fi

# Batterie : lue sur le topic robot, source unique de vérité.
BAT=$(timeout 12 python3 -c "
import socket; socket.setdefaulttimeout(8)
import rospy
from std_msgs.msg import Float32
v=[]
rospy.init_node('bat_check', anonymous=True)
rospy.Subscriber('/firmware/battery', Float32, lambda m: v.append(m.data))
t0=rospy.get_time()
while rospy.get_time()-t0<6 and not v and not rospy.is_shutdown(): rospy.sleep(0.2)
print('%.2f' % v[-1] if v else 'NA')
" 2>/dev/null | tail -1)
if [ "$BAT" != "NA" ] && [ -n "$BAT" ]; then
  # 11.1 V ~ 30 % sur cette batterie 3S. Sous ce seuil, le lacet n'est plus fiable.
  if awk "BEGIN{exit !($BAT < 11.1)}"; then
    echo "  [!] Batterie à ${BAT} V (< 11.1 V) — recharger avant toute mesure de rotation"; souci=1
  else
    echo "  ok  Batterie à ${BAT} V"
  fi
else
  echo "  ?   Batterie illisible (topic muet)"
fi

echo ""
if [ "$souci" = 1 ] && [ "$FORCE" = 0 ]; then
  echo "  RELANCE REFUSÉE — corriger ci-dessus, ou forcer avec --force"
  echo "  (une mesure prise dans ces conditions ne prouverait rien)"
  exit 2
fi
[ "$CHECK_ONLY" = 1 ] && { echo "  --check : rien relancé."; exit 0; }

echo "── Relance des estimateurs ──────────────────────────────────────────"
# Kill ciblé, jamais `pkill -f` large : le motif doit exclure ce shell.
# roslaunch les respawn tous les trois (respawn="true" dans la supervision).
for pat in "mins/lib/mins/subscribe" "ov_msckf/run_subscribe_msckf" "__name:=sqrtvins"; do
  pgrep -f "$pat" | while read -r p; do
    grep -qa claude "/proc/$p/cmdline" 2>/dev/null && continue
    kill -INT "$p" 2>/dev/null && echo "  SIGINT -> $pat (pid $p)"
  done
done

echo "  attente de la réinitialisation…"
sleep 15

echo ""
echo "── État après relance ───────────────────────────────────────────────"
timeout 40 python3 -c "
import socket; socket.setdefaulttimeout(8)
import math, rospy
from nav_msgs.msg import Odometry
g={}
def mk(k):
    def cb(m):
        p=m.pose.pose.position
        g[k]=(p.x,p.y,p.z,math.sqrt(p.x**2+p.y**2+p.z**2))
    return cb
rospy.init_node('etat_relance', anonymous=True)
for k,t in (('MINS','/mins/imu/odom'),('openVINS','/ov_msckf/odomimu'),('sqrtVins','/sqrtvins/odomimu')):
    rospy.Subscriber(t, Odometry, mk(k))
t0=rospy.get_time()
while rospy.get_time()-t0<20 and not rospy.is_shutdown(): rospy.sleep(0.3)
ok=True
for k in ('MINS','openVINS','sqrtVins'):
    if k not in g:
        print('  %-9s MUET — initialisation en cours (sqrtVINS attend une secousse)' % k); ok=False
    else:
        d=g[k][3]
        print('  %-9s (%7.3f, %7.3f, %7.3f)  d=%7.3f m  %s'
              % (k, g[k][0], g[k][1], g[k][2], d, 'OK' if d < 1.0 else '<-- PAS A L ORIGINE'))
        if d >= 1.0: ok=False
print('')
print('  ' + ('TOUS A L ORIGINE — banc pret pour un essai.' if ok
              else 'Au moins un estimateur n est pas propre : relire ci-dessus.'))
" 2>/dev/null | grep -v "^\["

echo ""
echo "  Source de pose : la remettre sur MINS avant tout passage en AUTO —"
echo "    rosservice call /pose_selector/set_source_by_name \"source: 'MINS'\""

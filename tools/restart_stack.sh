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

# ── Parametres d'exploitation, relayes a CHAQUE relance ──────────────────────
# Sans eux, ce script relance la pile avec les DEFAUTS (gyro_recal_period=0,
# zupt_clamp_enable=false) et efface silencieusement la configuration en cours.
# Constate deux fois le 2026-08-20 : le watchdog, declenche par une divergence
# MINS puis par un redemarrage manuel, a remis les defauts sans rien signaler —
# on ne s'en apercevait qu'en interrogeant rosparam apres coup.
# Modifier ICI pour changer ce que le watchdog restaure.
# ── 2026-08-21 : RETOUR AU POINT DE FONCTIONNEMENT CONNU-BON DE MINS ─────────
# Les trois options ci-dessous (gyro_recal 60, zupt_clamp true, accel_recal
# 120) ont ete activees le 2026-08-20 POUR openVINS, qui n'a aucun point
# d'entree externe pour un ZUPT confirme par les roues. MINS, lui, est deja
# ancre par ses roues : il n'en avait aucun besoin, et il en a fait les frais.
# Retour operateur direct : MINS marchait tres bien AVANT, resultat decevant
# APRES. Le dernier commit (899b656, les trois creneaux d'essai test1/2/3)
# lancait la pile SANS AUCUN argument -> ces trois valeurs par defaut.
# Le plus suspect des trois est zupt_clamp_enable : il ne regle pas un
# parametre, il REMPLACE le signal /imu/data_clean par la moyenne glissante
# pendant les arrets — et son propre commentaire dans navigation_supervision
# .launch le dit : « OFF par defaut, pas encore valide sur le robot ».
# Valeurs ECRITES EXPLICITEMENT (au lieu d'un tableau vide) pour garder la
# garantie d'origine de LEO_ARGS : le watchdog ne peut plus changer la
# configuration en silence, dans un sens comme dans l'autre.
LEO_ARGS=(gyro_recal_period:=0 zupt_clamp_enable:=false accel_recal_period:=0)

nohup /home/lab272/TOUT/catkin_ws/src/leo_navigation/launch_mins.sh navigation_master.launch \
      "${LEO_ARGS[@]}" \
      > /home/lab272/TOUT/logs/navigation_master.log 2>&1 &
echo "stack relancée (${LEO_ARGS[*]})"

# 2026-08-21 : sur lien WiFi robot dégradé, rospy.init_node() peut rester
# bloqué en enregistrement XML-RPC auprès du maître BIEN PLUS que les 12 s du
# wait_for_message interne (celui-ci ne démarre même jamais) -> la sonde
# n'expirait jamais proprement, timeout -k 5 15 finissait par la tuer au
# SIGKILL, et bash affichait un "Killed" bruyant à chaque cycle (vécu en
# direct : deux dumps consécutifs sur un `leo restart`). socket.setdefaulttimeout
# borne AUSSI l'enregistrement XML-RPC (pas seulement wait_for_message), donc
# le script python sort proprement par exception avant que timeout n'ait à
# tuer quoi que ce soit — plus de "Killed", cycles plus courts et plus nombreux
# dans le même budget de 300 s. Le groupe { ...; } 2>/dev/null est une
# ceinture-bretelles : il avale aussi le "Killed" que bash imprimerait sur SON
# PROPRE stderr (pas celui du process, donc invisible au ">/dev/null 2>&1" de
# la commande) si un cas pathologique forçait quand même le SIGKILL.
t0=$(date +%s)
until { timeout -k 3 12 python3 -c "
import socket
socket.setdefaulttimeout(8)
import rospy
from nav_msgs.msg import Odometry
rospy.init_node('wait_mins', anonymous=True)
rospy.wait_for_message('/mins/imu/odom', Odometry, timeout=8)
" > /dev/null 2>&1; } 2>/dev/null; do
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

# EXPOSITION PLAFONNEE (2026-08-24) — meme motif de panne que laser_power
# ci-dessus, meme remede : un redemarrage du robot ou du pilote camera remet
# l'auto-exposition d'usine, et le reglage est perdu SANS AUCUN signal.
# Constate en direct : apres le redemarrage complet du Pi, retour a
# enable_auto_exposure=True / exposure=33000, alors que la mesure de la veille
# avait etabli l'interet du plafond.
#
# POURQUOI 8 ms. Le projecteur IR est eteint (laser_power=0, ligne ci-dessus,
# choix delibere : ses points polluaient le damier de calibration et les LED
# de la balise). La scene infrarouge est donc sombre, et l'auto-exposition
# compense en ouvrant l'obturateur -- mesure : 33 000 us, soit la MOITIE de la
# periode de trame a 15 Hz. La trainee de flou vaut b = omega * t_exp * f :
# avec f = 336.37 px et omega jusqu'a 0.631 rad/s (mesure MINS, ancre roues),
# cela fait 7.0 px a 33 ms contre 1.7 px a 8 ms. Une trainee de 7 px n'efface
# pas le coin detecte par FAST, elle aplatit son gradient -- et
# fast_threshold=30 en exige un franc. Mesure a l'arret, filtres redemarres :
# 12.00 -> 72.00 features consommees par mise a jour.
#
# CONTREPARTIE, assumee : ce wrapper RealSense n'expose AUCUN plafond
# d'auto-exposition (verifie : seuls enable_auto_exposure, exposure et gain
# existent). Brider impose donc le mode MANUEL, donc un gain fige aussi. Le
# rover ne s'adapte plus a un changement de luminosite. En labo a eclairage
# constant c'est sans effet ; en exterieur il faudra revalider.
# gain 16 -> 64 compense les 4.1x de lumiere perdue.
timeout -k 5 15 rosrun dynamic_reconfigure dynparam set /camera/stereo_module enable_auto_exposure false > /dev/null 2>&1
timeout -k 5 15 rosrun dynamic_reconfigure dynparam set /camera/stereo_module exposure 8000 > /dev/null 2>&1
timeout -k 5 15 rosrun dynamic_reconfigure dynparam set /camera/stereo_module gain 64 > /dev/null 2>&1

# ── sqrtVINS : relance CE QUI TOURNAIT, sans jamais en démarrer un second ───
# (2026-08-27, audit TF Validator) Motivation identique à MINS/openVINS
# ci-dessus : sqrtVINS aussi ne relit sa configuration QU'AU DÉMARRAGE, et ce
# script l'ignorait totalement jusqu'ici — une extrinsèque éditée et
# sauvegardée depuis tf_validator.html (instance PC, config/leo_pc/) ne
# prenait donc effet qu'après un redémarrage manuel du nœud, jamais signalé.
#
# EXCLUSIVITÉ, jamais les deux à la fois. sqrtvins_pi_ctl.sh le documente en
# détail : le cockpit n'a qu'un bouton sqrtVINS, et deux instances publiant
# sur le même topic canonique /sqrtvins/odomimu le rendraient ambigu. Ce bloc
# ne DÉMARRE donc jamais une instance qui n'était pas déjà active — il relance
# CELLE qui tournait, et rien d'autre.
#
# Détection par /proc/PID/exe, ni pgrep -f ni pgrep -x : les deux ont été des
# pièges réels sur ce projet — pgrep -f s'attrape parfois lui-même (sa propre
# ligne de commande contient le motif) ; pgrep -x échoue toujours ici car le
# noyau tronque comm à 15 caractères et run_subscribe_msckf en compte 19.
# Même idiome que sqrtvins_pi_ctl.sh, pour ne pas réintroduire l'un ou l'autre.
PC_SQRT=$(for d in /proc/[0-9]*; do
  readlink "$d/exe" 2>/dev/null | grep -q 'sqrtvins_ws.*run_subscribe_msckf' \
    && basename "$d"
done)

if [ -n "$PC_SQRT" ]; then
  echo "sqrtVINS (PC) actif -> relance pour recharger sa configuration"
  for p in $PC_SQRT; do kill -INT "$p" 2>/dev/null; done
  pkill -INT -f 'roslaunch ov_srvins sqrtvins_pc' 2>/dev/null
  sleep 4
  # SIGKILL en tout dernier recours seulement : un -9 direct laisserait
  # l'inscription chez le master, et pose_selector se retrouverait abonné à
  # un fantôme — déjà vécu le 25/08 (« connection refused » au ping, topic
  # toujours listé).
  for d in /proc/[0-9]*; do
    readlink "$d/exe" 2>/dev/null | grep -q 'sqrtvins_ws.*run_subscribe_msckf' \
      && kill -KILL "$(basename "$d")" 2>/dev/null
  done
  nohup /home/lab272/TOUT/tools/launch_sqrtvins.sh \
        > /home/lab272/TOUT/logs/sqrtvins.log 2>&1 &
  disown
  # Même discipline d'attente que MINS ci-dessus (socket.setdefaulttimeout
  # avant rospy, sonde bornée) : sans elle un lien dégradé fait pendre le
  # script au lieu d'échouer proprement.
  t0=$(date +%s)
  until { timeout -k 3 12 python3 -c "
import socket
socket.setdefaulttimeout(8)
import rospy
from nav_msgs.msg import Odometry
rospy.init_node('wait_sqrtvins_pc', anonymous=True)
rospy.wait_for_message('/sqrtvins/odomimu', Odometry, timeout=8)
" > /dev/null 2>&1; } 2>/dev/null; do
    [ $(( $(date +%s) - t0 )) -gt 60 ] && { echo "sqrtVINS (PC) : pas de publication après 60s -- voir logs/sqrtvins.log"; break; }
    sleep 3
  done
  [ $(( $(date +%s) - t0 )) -le 60 ] && echo "sqrtVINS (PC) relancé et opérationnel après $(( $(date +%s) - t0 ))s"

elif bash /home/lab272/TOUT/tools/sqrtvins_pi_ctl.sh status 2>/dev/null | grep -q 'sqrtVINS (Pi) : MARCHE'; then
  echo "sqrtVINS (Pi) actif -> vérification/relance via sqrtvins_pi_ctl.sh"
  # start() est idempotent : ne relance réellement que si le topic est muet,
  # sinon confirme juste que tout va bien — pas de perte d'état inutile sur
  # un filtre embarqué qui tourne déjà correctement. L'édition TF Validator
  # n'écrit QUE config/leo_pc/ (instance PC) : la configuration de l'instance
  # embarquée, elle, n'a pas changé — rien de plus à faire ici que confirmer
  # sa santé.
  bash /home/lab272/TOUT/tools/sqrtvins_pi_ctl.sh start

else
  echo "sqrtVINS : aucune instance active -- rien à relancer"
fi

for i in 1 2 3; do
  out=$(timeout -k 5 10 rosservice call /pose_selector/set_source "data: true" 2>&1)
  echo "$out" | grep -q "success: True" && { echo "  switched to MINS"; break; }
  sleep 3
done
echo "TERMINE"

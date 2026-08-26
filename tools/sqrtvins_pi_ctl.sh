#!/bin/bash
# ============================================================================
# sqrtvins_pi_ctl.sh — pilote l'instance sqrtVINS EMBARQUEE, 2026-08-25.
#
# Appele depuis le PC (leo_backend, bouton du cockpit) ; agit uniquement sur le
# Raspberry Pi. Le PC n'execute jamais sqrtVINS par ce chemin.
#
#   tools/sqrtvins_pi_ctl.sh start    demarre sur le Pi (idempotent)
#   tools/sqrtvins_pi_ctl.sh stop     arrete sur le Pi
#   tools/sqrtvins_pi_ctl.sh status   dit ce qui tourne, ou
#
# POURQUOI PAS `killall -9 sqrtvins_node ov_msckf`
# ------------------------------------------------
# Cette commande, proposee comme nettoyage, ne tue RIEN :
#   * `sqrtvins_node` n'existe pas — le binaire s'appelle run_subscribe_msckf ;
#   * `ov_msckf` est un nom de PAQUET ROS, pas un nom de processus.
# Et la corriger naivement en `killall -9 run_subscribe_msckf` serait pire :
# openVINS et sqrtVINS produisent un binaire de MEME NOM DE BASE
#   openVINS  .../catkin_ws/devel/.private/ov_msckf/lib/ov_msckf/run_subscribe_msckf
#   sqrtVINS  .../sqrtvins_ws/devel/lib/ov_srvins/run_subscribe_msckf
# donc tuer par nom de base ne peut pas les distinguer. Le seul discriminant
# fiable est le CHEMIN COMPLET, d'ou les motifs `sqrtvins_ws` ci-dessous.
#
# SIGINT AVANT SIGKILL, jamais l'inverse : un noeud ROS tue en -9 laisse son
# inscription chez le master, et pose_selector se retrouve abonne a un
# fantome — exactement l'etat trouve le 25/08 (« connection refused » au ping,
# topic toujours liste). -9 n'intervient qu'en dernier recours.
#
# ROS_IP EST OBLIGATOIRE. Sans lui le noeud s'annonce comme http://leo:PORT/,
# un nom que le PC ne resout pas : le topic publie parfaitement sur le robot et
# reste invisible du PC, donc pose_selector ne peut jamais l'atteindre et le
# bouton du cockpit ne fait rien. Panne mesuree le 25/08.
# ============================================================================
set -o pipefail
cd /home/lab272/TOUT || exit 1
# shellcheck disable=SC1091
source tools/robot_env.sh

PI_WS=/home/pi/sqrtvins_ws
PI_MOTIF='sqrtvins_ws.*run_subscribe_msckf'
PI_LAUNCH='roslaunch ov_srvins'
SANIT=/home/pi/leo_tools/imu_sanitizer.py
# Nom CANONIQUE : le noeud embarque tourne sous node_name:=sqrtvins,
# donc ses topics sont identiques a ceux de l'instance PC. Les
# consommateurs (cockpit, export MATLAB) n'ont pas a savoir ou il tourne.
TOPIC=/sqrtvins/odomimu
IMU_PI=/imu/data_clean_pi
# Echelle accelero du robot : 9.790 / 8.6677 (brut MESURE le 25/08, 1661
# echantillons, robot immobile). Sur le Pi ce topic ne sert QUE les VINS —
# aucun MINS a menager — donc la valeur correcte va directement dans
# ~accel_scale.
ACCEL_SCALE_PI=1.1295

# CLE NOMMEE EXPLICITEMENT, et c'est indispensable (2026-08-25).
# leo_backend est lance par roslaunch, donc SANS agent SSH (SSH_AUTH_SOCK
# absent de son environnement, verifie dans /proc/PID/environ). Or
# ~/.ssh/config ne declare la cle que sous l'alias `Host leo`, et ce script se
# connecte par l'ADRESSE IP : aucun bloc Host ne correspond, aucune
# IdentityFile n'est appliquee, et l'authentification echoue par
# « Permission denied (publickey,password) ».
# Symptome vu du cockpit : cliquer sqrtVINS n'a AUCUN effet visible, le
# journal repete « robot injoignable » alors que le meme ssh marche
# parfaitement depuis un terminal — parce qu'un terminal, lui, a l'agent.
# IdentitiesOnly evite qu'ssh essaie d'abord d'autres cles et se fasse
# refuser par MaxAuthTries avant d'arriver a la bonne.
CLE=/home/lab272/.ssh/id_leo_tunnel
SSH_ID=""
[ -f "$CLE" ] && SSH_ID="-i $CLE -o IdentitiesOnly=yes"
# shellcheck disable=SC2086
pi() { timeout 40 ssh $SSH_ID -o ConnectTimeout=10 -o BatchMode=yes "pi@$ROBOT_HOST" "$@" 2>/dev/null; }

pi_env='export ROS_MASTER_URI=http://ROBOTIP:11311; export ROS_IP=ROBOTIP;'
pi_env=${pi_env//ROBOTIP/$ROBOT_HOST}

# Detection par le BINAIRE REEL (/proc/PID/exe), pas par la ligne de commande.
# `pgrep -f 'sqrtvins_ws.*run_subscribe_msckf'` s'attrapait LUI-MEME : la ligne
# de commande du pgrep contient le motif, donc etat() repondait MARCHE meme
# quand rien ne tournait — et l'idempotence de `start` sautait le demarrage.
# readlink /proc/PID/exe donne le chemin du binaire execute, insensible a ce
# piege comme aux processus de compilation qui citent ces chemins.
# Balayage de /proc, et non pgrep. DEUX pieges evites d'un coup :
#   * `pgrep -f <motif>` s'attrape LUI-MEME (sa ligne de commande contient le
#     motif) — etat() repondait MARCHE en permanence ;
#   * `pgrep -x run_subscribe_msckf` ne trouve JAMAIS rien : le noyau tronque
#     le nom de processus a 15 caracteres (TASK_COMM_LEN), et ce binaire en
#     compte 19 — il apparait comme « run_subscribe_m ». Cette fois etat()
#     repondait ARRETE alors que le noeud publiait a 62 Hz.
# /proc/PID/exe est le chemin REEL du binaire execute : ni tronque, ni
# confondu avec un shell ou un processus de compilation qui cite ces chemins.
pids_pi() {
  pi 'for d in /proc/[0-9]*; do
        readlink "$d/exe" 2>/dev/null | grep -q "sqrtvins_ws.*run_subscribe_msckf" \
          && basename "$d"
      done'
}

# Detection du sanitizer par argv[0], pas par la ligne de commande.
# `pgrep -f imu_sanitizer_pi` s'attrapait LUI-MEME (le motif figure dans la
# commande SSH qui l'execute), donc `start` croyait le sanitizer vivant et ne
# le lancait jamais : au retour d'un redemarrage du robot, /imu/data_clean_pi
# restait muet et sqrtVINS n'avait aucune IMU. Meme piege que pour le binaire
# sqrtVINS, a un autre endroit. Ici /proc/PID/exe ne suffit pas (c'est
# python3, partage par d'autres noeuds) : on exige que argv[0] soit python3 ET
# qu'un argument se termine par imu_sanitizer.py.
sanitizer_vivant() {
  pi 'for d in /proc/[0-9]*; do
        c=$(tr "\\0" "\\n" < "$d/cmdline" 2>/dev/null)
        [ -z "$c" ] && continue
        echo "$c" | head -1 | grep -q "python3$" || continue
        echo "$c" | grep -q "imu_sanitizer\\.py$" && { echo VIVANT; exit 0; }
      done'
}

etat() {
  [ -n "$(pids_pi)" ] && echo MARCHE || echo ARRETE
}

nettoyer() {
  # Arret propre puis, seulement si le processus resiste, SIGKILL. Les PID
  # viennent de pids_pi() (binaire reel) : aucun risque de tuer un processus
  # de compilation ni le shell qui porte le motif.
  local l; l=$(pids_pi | tr '\n' ' ')
  pi "pkill -INT -f '$PI_LAUNCH' 2>/dev/null; true"
  [ -n "$l" ] && pi "kill -INT $l 2>/dev/null; true"
  sleep 4
  l=$(pids_pi | tr '\n' ' ')
  pi "pkill -KILL -f '$PI_LAUNCH' 2>/dev/null; true"
  [ -n "$l" ] && pi "kill -KILL $l 2>/dev/null; true"
  sleep 1
}

case "${1:-status}" in

  status)
    e=$(etat)
    echo "sqrtVINS (Pi) : ${e:-INJOIGNABLE}"
    [ -n "$(sanitizer_vivant)" ] && echo "sanitizer (Pi) : MARCHE" \
                                 || echo "sanitizer (Pi) : ARRETE"
    # Le PC ne doit rien faire tourner : deux instances rendraient le bouton
    # unique du cockpit ambigu.
    # Meme precaution cote PC : binaire reel, et exclusion de ce shell.
    n=$(for d in /proc/[0-9]*; do
          readlink "$d/exe" 2>/dev/null | grep -q "sqrtvins_ws.*run_subscribe_msckf" \
            && basename "$d"
        done | wc -l)
    [ "$n" = "0" ] && echo "sqrtVINS (PC)  : ARRETE (attendu)" \
                   || echo "sqrtVINS (PC)  : $n RESIDU(S) — a arreter"
    ;;

  stop)
    echo "[sqrtvins-pi] arret"
    nettoyer
    echo "[sqrtvins-pi] etat : $(etat)"
    ;;

  start)
    # ── Idempotence : ne JAMAIS relancer un noeud deja sain ──────────────
    # Le bouton du cockpit appelle ce chemin a chaque clic. Redemarrer un
    # estimateur qui publie lui ferait perdre tout son etat — trajectoire,
    # covariance, biais — et rendrait toute comparaison impossible. Le
    # critere est le TOPIC, pas le processus : le 25/08 le noeud tournait
    # sans jamais initialiser, vivant et muet.
    if [ "$(etat)" = "MARCHE" ] && pi "source /opt/ros/noetic/setup.bash; $pi_env          timeout 6 rostopic hz -w 5 $TOPIC 2>/dev/null | grep -q 'average rate'"; then
      echo "[sqrtvins-pi] deja en marche et publie — rien a faire"
      exit 0
    fi

    # ── Aucun residu cote PC ────────────────────────────────────────────
    # Filtre sur le chemin, et exclusion explicite de ce shell : un motif
    # large se tuerait lui-meme, et tuerait openVINS avec.
    for d in /proc/[0-9]*; do
      readlink "$d/exe" 2>/dev/null | grep -q "sqrtvins_ws.*run_subscribe_msckf" || continue
      q=$(basename "$d")
      kill -INT "$q" 2>/dev/null && echo "[sqrtvins-pi] residu PC arrete (pid $q)"
    done
    pkill -INT -f 'roslaunch ov_srvins sqrtvins_pc' 2>/dev/null || true

    # ── Ressources du robot ─────────────────────────────────────────────
    lu=$(pi 'A=$(awk "/^cpu /{print \$2+\$3+\$4+\$5+\$6+\$7+\$8; print \$5}" /proc/stat)
             T1=$(echo "$A"|head -1); I1=$(echo "$A"|tail -1); sleep 3
             A=$(awk "/^cpu /{print \$2+\$3+\$4+\$5+\$6+\$7+\$8; print \$5}" /proc/stat)
             T2=$(echo "$A"|head -1); I2=$(echo "$A"|tail -1)
             awk -v t=$((T2-T1)) -v i=$((I2-I1)) "BEGIN{printf \"%.0f \", 100*i/t}"
             vcgencmd get_throttled 2>/dev/null | cut -d= -f2')
    read -r idle thr <<< "$lu"
    if [ -z "$idle" ]; then
      echo "[sqrtvins-pi] ERREUR : robot injoignable ($ROBOT_HOST)"; exit 2
    fi
    echo "[sqrtvins-pi] Pi : idle ${idle}%, throttled=${thr:-?}"
    # Avertissement et non refus : le bouton du cockpit doit rester utilisable.
    # C'est tools/campagne_essai.sh qui refuse, parce qu'une CAMPAGNE sur un
    # robot sature ne prouve rien — un simple essai manuel, si.
    [ "${idle:-0}" -lt 15 ] && echo "[sqrtvins-pi] ATTENTION : moins de 15 % d'idle, cadences a surveiller"

    # ── IMU embarquee (prerequis dur) ───────────────────────────────────
    # Sans elle sqrtVINS n'a AUCUNE entree inertielle et n'initialise jamais.
    # C'est ce qui manquait le 25/08 : la config pointait sur un topic que
    # personne ne publiait.
    if [ -z "$(sanitizer_vivant)" ]; then
      if ! pi "test -f $SANIT"; then
        echo "[sqrtvins-pi] ERREUR : $SANIT absent du robot."
        echo "               scp catkin_ws/src/leo_navigation/scripts/imu_sanitizer.py pi@\$ROBOT_HOST:$SANIT"
        exit 3
      fi
      echo "[sqrtvins-pi] demarrage du sanitizer IMU embarque"
      pi "setsid nohup bash -lc 'source /opt/ros/noetic/setup.bash; $pi_env \
            exec python3 $SANIT __name:=imu_sanitizer_pi \
              _out_topic:=$IMU_PI _vins_topic:=/imu/_sink_pi \
              _zupt_topic:=/imu_sanitizer_pi/is_stationary \
              _accel_scale:=$ACCEL_SCALE_PI _freeze_autoreset:=false' \
          > /tmp/imu_sanitizer_pi.log 2>&1 < /dev/null & disown"
      sleep 8
    fi

    # ── sqrtVINS ────────────────────────────────────────────────────────
    nettoyer
    echo "[sqrtvins-pi] lancement"
    pi "setsid nohup bash -lc 'source /opt/ros/noetic/setup.bash; \
          source $PI_WS/devel/setup.bash; $pi_env \
          exec $PI_LAUNCH sqrtvins_robot.launch' \
        > /tmp/sqrtvins_robot.log 2>&1 < /dev/null & disown"

    # ── Verification : le processus ne suffit pas, il faut le TOPIC ─────
    # Le 25/08 le noeud tournait sans jamais initialiser (try_zupt: false,
    # « no jerk detected » en boucle) : vivant et muet.
    echo -n "[sqrtvins-pi] attente de $TOPIC "
    for _ in $(seq 1 12); do
      sleep 5; echo -n "."
      if pi "source /opt/ros/noetic/setup.bash; $pi_env \
             timeout 6 rostopic hz -w 5 $TOPIC 2>/dev/null | grep -q 'average rate'"; then
        echo " publie."
        echo "[sqrtvins-pi] PRET — le bouton sqrtVINS du cockpit le selectionnera"
        exit 0
      fi
    done
    echo ""
    echo "[sqrtvins-pi] noeud lance mais $TOPIC MUET apres 60 s."
    echo "               causes deja rencontrees : try_zupt:false dans"
    echo "               config/leo/estimator_config.yaml (n'initialise jamais a"
    echo "               l'arret), ou $IMU_PI muet. Journal : /tmp/sqrtvins_robot.log"
    exit 4
    ;;

  *)
    echo "usage : $0 {start|stop|status}"; exit 1 ;;
esac

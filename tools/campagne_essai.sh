#!/bin/bash
# ============================================================================
# campagne_essai.sh — orchestre un essai comparatif complet, 2026-08-24.
#
# CE QU'IL LANCE, DANS CET ORDRE, ET POURQUOI CET ORDRE
#   1. verifications materielles       (refus si le banc n'est pas exploitable)
#   2. sqrtVINS sur le Pi              (le plus lent a demarrer : init statique)
#   3. moniteur de sante sur le PC     (doit tourner AVANT l'enregistrement,
#                                       sinon une famine passee inapercue
#                                       contaminerait un bag qu'on croira bon)
#   4. rosbag record                   (en dernier : tout doit deja tourner)
#
# AVERTISSEMENT DE COMPARABILITE, A LIRE AVANT D'INTERPRETER LES DONNEES.
# Les trois estimateurs NE consomment PAS les memes entrees dans cette
# configuration :
#     MINS, openVINS  (PC)  : /pc/camera/infra{1,2}/...  +  /imu/data_clean
#     sqrtVINS        (Pi)  : /camera/infra{1,2}/...     +  /imu/data_clean_pi
# Les flux /pc/ sont une republication locale des flux robot : memes images,
# mais un saut WiFi et un decodage de plus. Un ecart de sortie entre le Pi et
# les deux autres melange donc la difference d'ARCHITECTURE a celle
# d'estimateur. C'est un essai valable -- c'est meme la question posee par le
# deploiement sur le robot -- mais ce n'est PAS le test "a entrees
# identiques". Pour ce dernier, utiliser l'instance sqrtVINS du PC
# (tools/launch_sqrtvins.sh) et ne pas lancer celle du Pi.
#
# Usage :
#   tools/campagne_essai.sh <nom_essai> [duree_s]     # defaut 180 s
#   tools/campagne_essai.sh --stop                    # tout arreter
# ============================================================================
set -o pipefail
cd /home/lab272/TOUT || exit 1
# shellcheck disable=SC1091
source tools/robot_env.sh
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash 2>/dev/null

JOURNAL=/home/lab272/TOUT/logs
BAGS=/home/lab272/TOUT/data/trajectories
mkdir -p "$JOURNAL" "$BAGS"

arrete_tout() {
  echo "── Arret de la campagne ─────────────────────────────────────────────"
  # Kill CIBLE, jamais `pkill -f` large : le motif doit exclure ce shell.
  for pat in "rosbag record.*data/trajectories" "pi_health_monitor" "telemetrie_pi"; do
    pgrep -f "$pat" | while read -r p; do
      grep -qa claude "/proc/$p/cmdline" 2>/dev/null && continue
      # SIGINT et non SIGKILL sur rosbag : il doit fermer proprement son
      # index, sinon le fichier est illisible (.bag.active).
      kill -INT "$p" 2>/dev/null && echo "  arrete : $pat (pid $p)"
    done
  done
  ssh -o ConnectTimeout=6 -o BatchMode=yes "pi@$ROBOT_HOST" \
      'pkill -INT -f "roslaunch ov_srvins" 2>/dev/null' 2>/dev/null \
      && echo "  arrete : sqrtVINS sur le Pi"
  sleep 3
  echo "  fait."
}

[ "$1" = "--stop" ] && { arrete_tout; exit 0; }

NOM="${1:-campagne}"
DUREE="${2:-180}"
NOM=$(echo "$NOM" | tr -cd '[:alnum:]_-' | cut -c1-40)
[ -z "$NOM" ] && NOM=campagne
HORO=$(date +%Y%m%d_%H%M%S)

# ── 1. Verifications materielles ────────────────────────────────────────────
echo "── 1/4 Verifications materielles ────────────────────────────────────"
if ! bash tools/relance_estimateurs.sh --check; then
  echo ""
  echo "  Campagne ANNULEE. Une mesure prise dans ces conditions ne prouverait"
  echo "  rien -- voir le detail ci-dessus. Forcer : corriger puis relancer."
  exit 2
fi

# ── 1bis. Verifications du Pi ───────────────────────────────────────────────
# Le PC n'est pas le goulot d'etranglement : le Pi l'est. Mesure du 2026-08-24,
# pile complete SANS sqrtVINS : 54 % d'occupation, 46 % d'idle sur 4 coeurs,
# soit ~1,8 coeur libre. sqrtVINS mono-thread (num_opencv_threads: 1) en
# consomme ~1. La marge existe mais elle est mince, donc on la MESURE avant
# chaque campagne au lieu de la supposer.
#
# `load average` est ecarte volontairement : il comptait 4,18 sur 4 coeurs au
# moment ou l'occupation reelle etait de 54 %, parce qu'il additionne les
# processus en attente d'E/S (etat D). Lu comme une saturation, il aurait fait
# annuler une campagne parfaitement realisable.
echo ""
echo "── 1bis Verifications du robot ──────────────────────────────────────"
PI_ETAT=$(ssh -o ConnectTimeout=10 -o BatchMode=yes "pi@$ROBOT_HOST" '
  A=$(awk "/^cpu /{print \$2+\$3+\$4+\$5+\$6+\$7+\$8; print \$5}" /proc/stat)
  T1=$(echo "$A"|head -1); I1=$(echo "$A"|tail -1); sleep 5
  A=$(awk "/^cpu /{print \$2+\$3+\$4+\$5+\$6+\$7+\$8; print \$5}" /proc/stat)
  T2=$(echo "$A"|head -1); I2=$(echo "$A"|tail -1)
  awk -v t=$((T2-T1)) -v i=$((I2-I1)) "BEGIN{printf \"%.0f \", 100*i/t}"
  vcgencmd measure_temp 2>/dev/null | grep -oP "[0-9.]+" | head -1 | tr "\n" " "
  vcgencmd get_throttled 2>/dev/null | cut -d= -f2 | tr "\n" " "
  df --output=pcent / | tail -1 | tr -dc "0-9"
' 2>/dev/null)
read -r PI_IDLE PI_TEMP PI_THR PI_DISQUE <<< "$PI_ETAT"
if [ -z "$PI_IDLE" ]; then
  echo "  [!] Robot injoignable ($ROBOT_HOST) — campagne ANNULEE"
  exit 2
fi
souci_pi=0
# 25 % d'idle = 1 coeur : le strict minimum pour sqrtVINS, sans reserve.
if [ "${PI_IDLE:-0}" -lt 25 ]; then
  echo "  [!] Pi a ${PI_IDLE} % d'idle (< 25 %) — pas la place pour sqrtVINS"; souci_pi=1
else
  echo "  ok  Pi a ${PI_IDLE} % d'idle (~$(awk "BEGIN{printf \"%.1f\", $PI_IDLE*4/100}") coeur libre)"
fi
# throttled != 0x0 : le SoC a bride, ou l'a fait depuis le boot. C'est
# exactement le mecanisme qui a produit les overruns UART du 28/07.
if [ "$PI_THR" != "0x0" ]; then
  echo "  [!] Pi bride : throttled=$PI_THR (${PI_TEMP} °C) — les cadences vont chuter"; souci_pi=1
else
  echo "  ok  Pi a ${PI_TEMP} °C, aucun bridage"
fi
if [ "${PI_DISQUE:-0}" -ge 92 ]; then
  echo "  [!] Disque Pi a ${PI_DISQUE} % — leo.service a deja crashe pour cette raison"; souci_pi=1
else
  echo "  ok  Disque Pi a ${PI_DISQUE} %"
fi
if [ "$souci_pi" = 1 ]; then
  echo ""
  echo "  Campagne ANNULEE — le robot n'a pas les ressources pour cet essai."
  exit 2
fi

# ── 2. sqrtVINS sur le Pi ───────────────────────────────────────────────────
echo ""
echo "── 2/4 sqrtVINS sur le robot ────────────────────────────────────────"
if ssh -o ConnectTimeout=6 -o BatchMode=yes "pi@$ROBOT_HOST" \
      'pgrep -f "roslaunch ov_srvins" > /dev/null' 2>/dev/null; then
  echo "  deja en marche"
else
  # setsid : sans lui le processus meurt avec la session SSH (constate).
  ssh -o ConnectTimeout=8 -o BatchMode=yes "pi@$ROBOT_HOST" \
    "setsid nohup bash -lc 'source /opt/ros/noetic/setup.bash; \
     source /home/pi/sqrtvins_ws/devel/setup.bash; \
     exec roslaunch ov_srvins sqrtvins_robot.launch' \
     > /tmp/sqrtvins_robot.log 2>&1 < /dev/null & disown" 2>/dev/null
  echo "  lance (respawn actif : le crash d'init connu est rattrape)"
  sleep 12
fi

# ── 3. Moniteur de sante ────────────────────────────────────────────────────
echo ""
echo "── 3/4 Moniteur de sante (sur le PC, pas sur le Pi) ─────────────────"
if pgrep -f "pi_health_monitor" > /dev/null 2>&1; then
  echo "  deja en marche"
else
  SANTE="$JOURNAL/sante_${NOM}_${HORO}.log"
  setsid nohup rosrun leo_navigation pi_health_monitor.py \
    > "$SANTE" 2>&1 < /dev/null &
  disown 2>/dev/null
  echo "  lance -> $SANTE"
  # Le moniteur attend que ses fenetres glissantes se remplissent avant de
  # juger ; le laisser prendre cette avance avant d'enregistrer.
  sleep 8
fi

# ── 4. Enregistrement ───────────────────────────────────────────────────────
echo ""
echo "── 4/4 Enregistrement rosbag ────────────────────────────────────────"
BAG="$BAGS/${NOM}_${HORO}"
TOPICS=(
  /mins/imu/odom                # MINS      (PC, roues + visuel + inertiel)
  /ov_msckf/odomimu             # openVINS  (PC, VIO pur)
  /sqrtvins/odomimu             # sqrtVINS  (PC, VIO pur, double precision)
  /sqrtvins/odomimu             # sqrtVINS  (nom CANONIQUE depuis le 25/08 :
                                #  l'instance embarquee tourne sous
                                #  node_name:=sqrtvins, donc ce topic la
                                #  designe qu'elle soit sur le Pi ou le PC)
  /ov_srvins/odomimu            # ancien nom, garde : rosbag accepte un topic
                                #  inexistant sans broncher, et le capter
                                #  coute moins qu'un essai perdu
  /robot_pose_fused             # source active, apres correction SE(3)
  /leo_navigation/pose_source   # QUELLE source etait active, et quand
  /firmware/wheel_states        # temoin INDEPENDANT : le robot bougeait-il ?
  /imu/data_clean               # entree commune des deux estimateurs PC
  /ov_msckf/points_msckf        # sante du canal visuel openVINS
  /sqrtvins/points_msckf        # idem sqrtVINS PC
)
# Les IMAGES ne sont volontairement PAS enregistrees : 15 Hz x 640x480 x 2
# camera pendant 3 min font plusieurs gigaoctets, et la chaine d'analyse
# MATLAB n'en a pas besoin -- elle travaille sur les poses. Pour un essai ou
# les images comptent, utiliser tools/record_trajectories.sh.
echo "  fichier : ${BAG}.bag"
echo "  topics  : ${#TOPICS[@]}"
echo "  duree   : ${DUREE}s"

# Telemetrie du Pi pendant l'essai. Le moniteur de sante voit les EFFETS depuis
# le PC (une cadence qui tombe) ; la CAUSE (CPU sature, SoC bride) ne se lit que
# sur le Pi. Ce n'est pas une contradiction avec la regle "ne rien faire tourner
# sur le Pi" : lire /proc et vcgencmd toutes les 2 s ne coute rien, la ou un
# noeud ROS abonne a un flux 84 Hz couterait du CPU reel.
TELE="$JOURNAL/telemetrie_pi_${NOM}_${HORO}.log"
ssh -o ConnectTimeout=8 -o BatchMode=yes "pi@$ROBOT_HOST" '
  while true; do
    A=$(awk "/^cpu /{print \$2+\$3+\$4+\$5+\$6+\$7+\$8; print \$5}" /proc/stat)
    T1=$(echo "$A"|head -1); I1=$(echo "$A"|tail -1); sleep 2
    A=$(awk "/^cpu /{print \$2+\$3+\$4+\$5+\$6+\$7+\$8; print \$5}" /proc/stat)
    T2=$(echo "$A"|head -1); I2=$(echo "$A"|tail -1)
    printf "%s idle=%.0f%% temp=%s throttled=%s\n" "$(date +%H:%M:%S)" \
      "$(awk -v t=$((T2-T1)) -v i=$((I2-I1)) "BEGIN{print 100*i/t}")" \
      "$(vcgencmd measure_temp 2>/dev/null | cut -d= -f2)" \
      "$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)"
  done' > "$TELE" 2>/dev/null &
PID_TELE=$!

echo ""
echo "  >>> FAIS TON PARCOURS MAINTENANT <<<"
echo ""
rosbag record --duration="${DUREE}" -O "$BAG" "${TOPICS[@]}" 2>&1 | tail -3
kill "$PID_TELE" 2>/dev/null

# ── Bilan ───────────────────────────────────────────────────────────────────
echo ""
echo "── Bilan ────────────────────────────────────────────────────────────"
if [ ! -f "${BAG}.bag" ]; then
  echo "  AUCUN fichier produit -- verifier le journal ci-dessus."
  [ -f "${BAG}.bag.active" ] && echo "  (un .bag.active traine : rosbag n'a pas ferme son index)"
  exit 1
fi

# Cadence EFFECTIVE relue depuis le bag, et non le compteur d'un noeud vivant :
# c'est la seule facon de savoir si ce qui a ete ECRIT sur le disque tient la
# cadence. Un topic peut publier a 18 Hz et n'en voir que 12 arriver au bag.
python3 - "${BAG}.bag" <<'PY'
import sys, rosbag
NOMINAL = {  # Hz mesures le 2026-08-24, cf. pi_health_monitor.py
    '/firmware/wheel_states': 18.4, '/imu/data_clean': 83.9,
    '/mins/imu/odom': 20.0, '/ov_msckf/odomimu': 84.0,
    '/sqrtvins/odomimu': 84.0, '/ov_srvins/odomimu': 84.0,
}
try:
    b = rosbag.Bag(sys.argv[1])
except Exception as e:
    print('  bag illisible : %s' % e); sys.exit(1)
i = b.get_type_and_topic_info()[1]
duree = b.get_end_time() - b.get_start_time()
print('  duree %.1f s, %d topics, %.1f Mo\n' % (
    duree, len(i), b.size / 1e6))
print('  %-30s %8s %9s %9s  %s' % ('topic', 'msgs', 'Hz reel', 'attendu', 'verdict'))
for t, inf in sorted(i.items()):
    hz = inf.message_count / duree if duree > 0 else 0
    nom = NOMINAL.get(t)
    if inf.message_count == 0:
        v = 'VIDE -- producteur absent'
    elif nom is None:
        v = ''
    elif hz < 0.70 * nom:
        v = 'CHUTE (%.0f %% du nominal)' % (100 * hz / nom)
    else:
        v = 'ok'
    print('  %-30s %8d %9.1f %9s  %s' % (
        t[:30], inf.message_count, hz, ('%.1f' % nom) if nom else '-', v))
b.close()
PY

echo ""
echo "  Sante des flux pendant l'essai (vue PC) :"
SANTE_F=$(ls -t "$JOURNAL"/sante_*.log 2>/dev/null | head -1)
if [ -n "$SANTE_F" ] && [ -s "$SANTE_F" ]; then
  n=$(grep -ac "DEGRADE\|AUCUN message" "$SANTE_F" 2>/dev/null)
  if [ "${n:-0}" = "0" ]; then
    echo "   aucune degradation -- les cadences ont tenu tout l'essai."
  else
    echo "   ${n} evenement(s) :"
    grep -a "DEGRADE\|AUCUN message\|retabli" "$SANTE_F" | tail -10 | sed 's/^/     /'
  fi
else
  echo "   (journal de sante vide)"
fi

echo ""
echo "  Ressources du robot pendant l'essai (vue Pi) :"
if [ -s "$TELE" ]; then
  awk '{ for(i=1;i<=NF;i++){
           if($i ~ /^idle=/){ gsub(/idle=|%/,"",$i); s+=$i; n++;
                              if(mn==""||$i<mn) mn=$i }
           if($i ~ /^temp=/){ gsub(/temp=|.C/,"",$i); if($i>tmx) tmx=$i }
           if($i ~ /^throttled=/ && $i != "throttled=0x0") br++ } }
       END{ if(n>0) printf "   idle moyen %.0f %%, minimum %.0f %% | temp max %.1f C | %d echantillon(s) bride(s)\n", s/n, mn, tmx, br+0 }' "$TELE"
  # 10 % d'idle = les 4 coeurs quasi satures : c'est la signature de la famine
  # qui affame serial_node et fait tomber wheel_states.
  mini=$(awk '{for(i=1;i<=NF;i++) if($i ~ /^idle=/){gsub(/idle=|%/,"",$i); if(m==""||$i<m) m=$i}} END{print int(m)}' "$TELE")
  if [ -n "$mini" ] && [ "$mini" -lt 10 ]; then
    echo "   [!] idle tombe a ${mini} % — le Pi a ete sature ; correler avec les"
    echo "       chutes de cadence ci-dessus avant d'exploiter ces trajectoires."
  fi
else
  echo "   (telemetrie vide -- SSH interrompu ?)"
fi

echo ""
echo "  Analyse : python3 tools/resample_trajectoires.py --bag ${BAG}.bag --nom $NOM"
echo "  Arret   : tools/campagne_essai.sh --stop"

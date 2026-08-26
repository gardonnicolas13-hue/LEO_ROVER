#!/bin/bash
# ============================================================================
# diag_amorcage.sh — correle l'ETAT INITIAL de sqrtVINS avec le resultat,
# sur N relances. 2026-08-26.
#
# LE DEFAUT QU'IL SERT A CARACTERISER
# -----------------------------------
# A configuration et conditions STRICTEMENT identiques, une relance sur trois
# de sqrtVINS embarque part dans un mode degrade : ~0.87 m/s de vitesse
# fantome au repos, contre 0.012 m/s en mode sain — facteur 73. Les deux
# modes sont des grappes serrees, jamais rien entre les deux.
#
# La garde d'excitation (init_imu_thresh) a ECARTE l'explication evidente :
# la fenetre d'initialisation est calme dans les DEUX cas (0.068 mesure pour
# un seuil a 0.400). Ce n'est donc pas l'agitation physique.
#
# CE QUE CE SCRIPT MESURE, ET POURQUOI
# ------------------------------------
# L'initialiseur statique derive d'UNE SEULE moyenne le biais gyro, le biais
# accelero ET l'attitude. La longueur de cette fenetre n'est pas fixe : son
# debut vient du plus ancien echantillon IMU du tampon (borne a 1.1 s), sa fin
# de `prev_static_timestamp_`, qui est le dernier horodatage CAMERA de
# l'iteration precedente. Deux flux, deux cadences (85 Hz et 15 Hz), et un
# ordonnancement qui varie : la fenetre peut donc etre courte sans rien avoir
# d'anormal.
#
# Or la garde teste la DISPERSION des echantillons, pas l'erreur-type de la
# MOYENNE, qui decroit en sigma/racine(N). Une fenetre de 10 echantillons
# passe la garde avec la meme dispersion qu'une fenetre de 85, tout en donnant
# une moyenne trois fois plus bruitee — et cette moyenne devient l'etat
# initial, qu'aucune mise a jour ulterieure ne peut rejeter.
#
# HYPOTHESE A REFUTER OU CONFIRMER : les mauvaises relances ont un N plus
# petit. Si N est identique entre bons et mauvais tirages, l'hypothese tombe
# et il faut chercher ailleurs.
#
# Usage :  tools/diag_amorcage.sh [nombre_de_relances]     # defaut 12
# ============================================================================
set -o pipefail
cd /home/lab272/TOUT || exit 1
# shellcheck disable=SC1091
source tools/robot_env.sh
source /opt/ros/noetic/setup.bash

N="${1:-12}"
CLE=/home/lab272/.ssh/id_leo_tunnel
SSH_ID=""
[ -f "$CLE" ] && SSH_ID="-i $CLE -o IdentitiesOnly=yes"
# shellcheck disable=SC2086
pi() { timeout 40 ssh $SSH_ID -o ConnectTimeout=10 -o BatchMode=yes "pi@$ROBOT_HOST" "$@" 2>/dev/null; }

RES=/home/lab272/TOUT/logs/amorcage_$(date +%Y%m%d_%H%M%S).csv
echo "relance,N,duree_s,a_avg,w_avg,bg_norme,ba_norme,v_repos,chemin_m_min,verdict" > "$RES"

echo "── Caracterisation de l'amorcage : $N relances ─────────────────────"
echo "   resultats -> $RES"
echo ""
printf "  %-3s %5s %8s %9s %10s %11s %9s\n" \
       "n" "N" "duree" "|a_avg|" "|bg|" "|v| repos" "verdict"
echo "  ------------------------------------------------------------------"

for i in $(seq 1 "$N"); do
  # Arret par le BINAIRE reel : ni pgrep -f (qui s'attrape lui-meme), ni
  # pgrep -x (le noyau tronque le nom a 15 caracteres, celui-ci en fait 19).
  pi 'pkill -INT -f "roslaunch ov_srvins" 2>/dev/null
      for d in /proc/[0-9]*; do
        readlink "$d/exe" 2>/dev/null | grep -q "sqrtvins_ws.*run_subscribe_msckf" \
          && kill -INT "$(basename "$d")"
      done; sleep 4'

  pi "setsid nohup bash -lc 'source /opt/ros/noetic/setup.bash; \
        source /home/pi/sqrtvins_ws/devel/setup.bash; \
        export ROS_MASTER_URI=http://$ROBOT_HOST:11311; export ROS_IP=$ROBOT_HOST; \
        exec roslaunch ov_srvins sqrtvins_robot.launch verbosity:=INFO' \
      > /tmp/amorcage.log 2>&1 < /dev/null & disown"
  sleep 28

  # ── Empreinte de l'etat initial, telle que l'initialiseur l'a posee ──
  EMP=$(pi 'grep -a "ETAT INITIAL" /tmp/amorcage.log | tail -1')
  Nech=$(sed -n 's/.*N=\([0-9]*\).*/\1/p' <<< "$EMP")
  DUR=$(sed -n 's/.*t=\([0-9.]*\).*/\1/p' <<< "$EMP")
  AAVG=$(sed -n 's/.*|a_avg|=\([0-9.]*\).*/\1/p' <<< "$EMP")
  WAVG=$(sed -n 's/.*|w_avg|=\([0-9.]*\).*/\1/p' <<< "$EMP")
  BG=$(sed -n 's/.*bg=\[\([^]]*\)\].*/\1/p' <<< "$EMP")
  BA=$(sed -n 's/.*ba=\[\([^]]*\)\].*/\1/p' <<< "$EMP")
  BGN=$(python3 -c "import sys,math;v=[float(x) for x in '''$BG'''.split()] or [0];print('%.5f'%math.sqrt(sum(x*x for x in v)))" 2>/dev/null || echo 0)
  BAN=$(python3 -c "import sys,math;v=[float(x) for x in '''$BA'''.split()] or [0];print('%.5f'%math.sqrt(sum(x*x for x in v)))" 2>/dev/null || echo 0)

  # ── Resultat : vitesse et chemin au repos ────────────────────────────
  MES=$(timeout 60 python3 - <<'PY' 2>/dev/null
import socket; socket.setdefaulttimeout(8)
import rospy, math
from nav_msgs.msg import Odometry
rospy.init_node('amorc', anonymous=True)
P=[]; V=[]
def cb(m):
    p=m.pose.pose.position; v=m.twist.twist.linear
    P.append((p.x,p.y,p.z)); V.append(math.sqrt(v.x**2+v.y**2+v.z**2))
rospy.Subscriber('/sqrtvins/odomimu', Odometry, cb, queue_size=300)
D=25; t0=rospy.get_time()
while rospy.get_time()-t0<D and not rospy.is_shutdown(): rospy.sleep(0.3)
if len(P)<2: print('nan nan')
else:
    ch=sum(math.dist(P[i],P[i+1]) for i in range(len(P)-1))
    print('%.4f %.2f' % (sum(V)/len(V), ch*60/D))
PY
)
  VREPOS=$(awk '{print $1}' <<< "$MES"); CHEMIN=$(awk '{print $2}' <<< "$MES")
  # Seuil a 0.1 m/s : les deux modes observes sont a 0.012 et 0.87 m/s, donc
  # tout seuil entre les deux separe sans ambiguite. Aucune relance n'a jamais
  # atterri entre les deux grappes.
  VERDICT=$(awk -v v="$VREPOS" 'BEGIN{print (v=="nan"||v=="")?"MUET":((v<0.1)?"SAIN":"DEGRADE")}')

  printf "  %-3d %5s %8s %9s %10s %11s %9s\n" \
         "$i" "${Nech:-?}" "${DUR:-?}" "${AAVG:-?}" "${BGN:-?}" "${VREPOS:-?}" "$VERDICT"
  echo "$i,${Nech},${DUR},${AAVG},${WAVG},${BGN},${BAN},${VREPOS},${CHEMIN},${VERDICT}" >> "$RES"
done

echo ""
echo "── Correlation ─────────────────────────────────────────────────────"
python3 - "$RES" <<'PY'
import csv, sys, statistics as st
lignes=[l for l in csv.DictReader(open(sys.argv[1])) if l['verdict'] in ('SAIN','DEGRADE')]
if not lignes: print('  aucune relance exploitable'); raise SystemExit
for v in ('SAIN','DEGRADE'):
    g=[l for l in lignes if l['verdict']==v]
    if not g: print('  %-8s aucune' % v); continue
    def col(c):
        x=[float(l[c]) for l in g if l[c] not in ('','?','nan')]
        return (st.mean(x), min(x), max(x)) if x else (float('nan'),)*3
    n=col('N'); d=col('duree_s'); a=col('a_avg'); b=col('bg_norme')
    print('  %-8s %2d relances | N %5.1f [%.0f-%.0f] | duree %.3f s | |a_avg| %.4f | |bg| %.5f'
          % (v, len(g), n[0], n[1], n[2], d[0], a[0], b[0]))
sains=[float(l['N']) for l in lignes if l['verdict']=='SAIN' and l['N'] not in ('','?')]
degr =[float(l['N']) for l in lignes if l['verdict']=='DEGRADE' and l['N'] not in ('','?')]
print('')
if sains and degr:
    if abs(st.mean(sains)-st.mean(degr)) > 0.25*max(st.mean(sains),1):
        print('  => N DIFFERE nettement entre les deux modes : la longueur de')
        print('     fenetre est bien la variable qui decide. Correctif : exiger')
        print('     un N minimal avant d accepter l initialisation.')
    else:
        print('  => N est SEMBLABLE dans les deux modes : hypothese REFUTEE,')
        print('     la longueur de fenetre ne decide pas. Chercher ailleurs')
        print('     (ordre des premieres images, etat du suiveur au boot).')
else:
    print('  => un seul mode observe sur cet echantillon ; relancer avec plus')
    print('     de repetitions pour trancher.')
PY
echo ""
echo "  CSV complet : $RES"

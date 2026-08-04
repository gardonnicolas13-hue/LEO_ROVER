#!/usr/bin/env bash
# ═════════════════════════════════════════════════════════════════════════════
# record_trajectories.sh — enregistre les poses MINS et openVINS dans un rosbag
#   pendant une conduite manuelle, pour comparaison hors-ligne (Matlab).
#
#   Usage :
#     tools/record_trajectories.sh                 # durée illimitée (Ctrl-C pour finir)
#     tools/record_trajectories.sh 90              # arrêt automatique après 90 s
#     tools/record_trajectories.sh 90 mon_essai    # + nom de base personnalisé
#
#   Le bag est écrit dans  ~/TOUT/data/trajectories/<nom>_<horodatage>.bag
#   puis converti en CSV avec  tools/bag_to_csv.py  (à passer ensuite à
#   plot_trajectories.m). Voir l'en-tête de ces deux fichiers.
#
#   PRÉ-REQUIS : la stack navigation tourne (leo start) ET, pour avoir une VRAIE
#   trace VINS, il faut CONDUIRE (openVINS s'initialise sur une secousse
#   accéléromètre — un aller-retour de ~1 s ; voir §sec:ovinit du rapport).
#   MINS, lui, publie à l'arrêt.
# ═════════════════════════════════════════════════════════════════════════════
set -euo pipefail

DURATION="${1:-0}"                 # 0 = illimité (Ctrl-C)
BASENAME="${2:-traj}"
TOUT="/home/lab272/TOUT"
OUTDIR="$TOUT/data/trajectories"

# Environnement ROS (ROS_MASTER_URI / ROS_IP centralisés côté PC).
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$TOUT/tools/robot_env.sh"

mkdir -p "$OUTDIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="$OUTDIR/${BASENAME}_${STAMP}"

# ── Topics enregistrés ───────────────────────────────────────────────────────
# Les deux estimateurs bruts (le cœur de la comparaison), plus le contexte
# utile : la source servie au robot, le flux fusionné, et le repère global
# Carolus quand une balise est en vue (référence quasi-vérité-terrain, la
# seule ancre absolue du système — voir carolus_vicon_bridge.py).
TOPICS=(
  /mins/imu/odom                 # MINS   (nav_msgs/Odometry)
  /ov_msckf/odomimu              # VINS   (nav_msgs/Odometry)
  /robot_pose_fused              # source active servie (nav_msgs/Odometry)
  /leo_navigation/pose_source    # "VINS"|"MINS" au fil du temps (std_msgs/String)
  /pose                          # fix balise CAROLUS (geometry_msgs/PoseStamped)
                                 #   sortie reelle de carolus_astrobee ; c'est
                                 #   le topic nomme par le superviseur.
  /mins/external_ref/carolus     # ancien creneau du pont MINS — TOUJOURS VIDE
                                 #   (aucun noeud ne publie le TF beacon_link
                                 #   dont carolus_vicon_bridge depend). Garde
                                 #   pour ne pas casser la relecture des bags
                                 #   anterieurs.
  /firmware/wheel_odom           # odométrie roues brute (référence)
)

# ── Vérif de présence des estimateurs (avertissement, pas blocage) ───────────
echo ""
echo "  [record_trajectories] cible : $BAG.bag"
for t in /mins/imu/odom /ov_msckf/odomimu; do
  if timeout -k 3 6 rostopic info "$t" >/dev/null 2>&1; then
    echo "    ✓ $t publié"
  else
    echo "    ⚠ $t ABSENT — la trace correspondante sera vide dans le bag"
    [ "$t" = /ov_msckf/odomimu ] && \
      echo "      (VINS : conduis quelques secondes pour l'initialiser, puis relance)"
  fi
done

echo ""
echo "  ▶ CONDUIS LE ROBOT EN MANUEL MAINTENANT."
if [ "$DURATION" -gt 0 ] 2>/dev/null && [ "$DURATION" -ne 0 ]; then
  echo "    Enregistrement pendant ${DURATION}s (arrêt auto)…"
  DUR_ARG=(--duration="${DURATION}")
else
  echo "    Enregistrement illimité — Ctrl-C pour terminer proprement."
  DUR_ARG=()
fi
echo ""

# -O : nom de sortie ; --lz4 : compression rapide (les Odometry sont petites,
# mais wheel_odom + fused peuvent monter à quelques Mo/min sur un long essai).
rosbag record "${DUR_ARG[@]}" --lz4 -O "$BAG" "${TOPICS[@]}"

echo ""
echo "  ✓ bag écrit : $BAG.bag"
echo "  → conversion CSV :  python3 $TOUT/tools/bag_to_csv.py $BAG.bag"
echo "  → tracé Matlab  :  plot_trajectories('${BAG}')   (dans Matlab, voir plot_trajectories.m)"

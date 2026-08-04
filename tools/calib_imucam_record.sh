#!/usr/bin/env bash
# ═════════════════════════════════════════════════════════════════════════════
# calib_imucam_record.sh — enregistre le rosbag pour la calibration Kalibr
#   IMU <-> paire stereo infrarouge du D455.
#
#   POURQUOI (2026-07-28) : openVINS donne 298 m de deplacement pour 6 m reels
#   (49x, mesure contre l'odometrie roues). Toutes les autres causes ont ete
#   eliminees par la mesure : crash mutex corrige, accelerometre corrige
#   (9.78 m/s^2), biais gyro corrige (0.008 deg/s), ZUPT active, et les QUATRE
#   combinaisons extrinsics/ZUPT testees donnent le meme echec. VINS est JUSTE
#   a l'arret et FAUX des qu'il roule : la seule chose que le mouvement
#   sollicite et qui n'a jamais ete calibree, c'est la geometrie camera<->IMU.
#
#   ETAT CONSTATE : aucun fichier *-camchain-imucam.yaml ne correspond a la
#   paire stereo. Les seuls qui existent portent sur /camera/image_raw (mono,
#   inutilisee). Les calibrations stereo existantes sont en 848x480 alors que
#   le systeme tourne en 640x480. Le T_cam_imu actuel de
#   open_vins/config/leo/kalibr_imucam_chain.yaml est une matrice IDEALISEE
#   (0 et +/-1 parfaits) : mesuree a la main, jamais calibree.
#
#   CE SCRIPT NE FAIT QUE L'ENREGISTREMENT. Le calcul se fait ensuite avec
#   tools/calib_imucam_run.sh (Kalibr, hors ligne, plusieurs minutes).
#
#   Usage :  tools/calib_imucam_record.sh [duree_s]     (defaut 90)
# ═════════════════════════════════════════════════════════════════════════════
set -u
DUR="${1:-90}"
OUT=/home/lab272/TOUT/calib_data
STAMP=$(date +%Y%m%d_%H%M%S)
BAG="$OUT/imucam_$STAMP"

source /opt/ros/noetic/setup.bash
source /home/lab272/TOUT/tools/robot_env.sh
mkdir -p "$OUT"

# Topics : les MEMES que ceux consommes par openVINS, sinon la calibration
# ne s'applique pas au systeme reel (cf. estimator_config.yaml ->
# kalibr_imucam_chain.yaml : /pc/camera/infra{1,2}/image_rect_raw, et
# kalibr_imu_chain.yaml : /imu/data_clean).
# NB /imu/data_clean est le flux ASSAINI (anti-spike, anti-gel, correction
# d'echelle accelero + biais gyro) : c'est bien celui-la qu'il faut, puisque
# c'est celui que les estimateurs recoivent.
CAM1=/pc/camera/infra1/image_rect_raw
CAM2=/pc/camera/infra2/image_rect_raw
IMU=/imu/data_clean

echo "══════════════════════════════════════════════════════════════"
echo "  CALIBRATION IMU <-> CAMERA — enregistrement"
echo "══════════════════════════════════════════════════════════════"
for t in "$CAM1" "$CAM2" "$IMU"; do
  hz=$(timeout 6 python3 - "$t" <<'PY' 2>/dev/null
import sys, rospy, time
from sensor_msgs.msg import Image, Imu
rospy.init_node('chk', anonymous=True, disable_signals=True)
t=sys.argv[1]; c={'n':0}
typ = Imu if 'imu' in t else Image
rospy.Subscriber(t, typ, lambda m: c.update(n=c['n']+1))
time.sleep(4); print(round(c['n']/4.0,1))
PY
)
  printf "  %-42s %s Hz\n" "$t" "${hz:-0}"
  if [ "${hz:-0}" = "0" ] || [ "${hz:-0}" = "0.0" ]; then
    echo "  ERREUR : ce topic ne publie pas — calibration impossible." >&2
    exit 1
  fi
done

cat <<'PROTO'

  ── MIRE : VOTRE DAMIER DE BALISE ───────────────────────────────
  Damier 6x9 cases de 25.4 mm
  (catkin_ws/calibration_leo_d455/target.yaml)
  RIEN A IMPRIMER : c'est le damier deja utilise par la balise.
  Le poser RIGIDE et IMMOBILE, eclairage homogene, sans reflet.
  NB : un damier doit etre vu ENTIEREMENT a chaque image (contrairement
  a un AprilGrid qui tolere les vues partielles) -> garder toute la
  mire dans le champ des DEUX cameras pendant tout l'enregistrement.

  ── MOUVEMENT (le point critique) ───────────────────────────────
  Kalibr doit observer les 6 degres de liberte, sinon la geometrie
  reste inobservable — c'est exactement ce qui bloque VINS aujourd'hui.
  NE PAS conduire le rover : PRENDRE LA CAMERA EN MAIN (ou le rover)
  et l'agiter devant la mire, mire toujours pleinement visible :
    * rotations franches autour des 3 axes (lacet, tangage, roulis)
    * translations sur les 3 axes (avant/arriere, gauche/droite, haut/bas)
    * mouvements AMPLES et VIFS (l'IMU doit etre excitee) mais SANS FLOU
    * varier la distance : 0.5 m a 2 m
  Un rover qui roule a plat ne produit QUE du plan : roulis et tangage
  restent inobservables et la calibration echouera silencieusement.

PROTO
read -r -p "  Pret ? [Entree pour enregistrer ${DUR}s, Ctrl-C pour annuler] " _

echo "  Enregistrement ${DUR}s -> $BAG.bag"
rosbag record -O "$BAG" --duration="$DUR" "$CAM1" "$CAM2" "$IMU"

echo
echo "  ✓ bag : $BAG.bag"
echo "  → calcul : tools/calib_imucam_run.sh $BAG.bag"

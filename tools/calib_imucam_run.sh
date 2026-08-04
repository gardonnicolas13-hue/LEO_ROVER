#!/usr/bin/env bash
# ═════════════════════════════════════════════════════════════════════════════
# calib_imucam_run.sh — lance Kalibr sur le bag produit par
#   tools/calib_imucam_record.sh, puis affiche ce qu'il faut reporter.
#
#   Deux etapes, dans cet ordre OBLIGATOIRE :
#     1. kalibr_calibrate_cameras     -> intrinseques + stereo (camchain)
#     2. kalibr_calibrate_imu_camera  -> T_cam_imu + timeshift (camchain-imucam)
#   L'etape 2 a besoin du camchain de l'etape 1.
#
#   POURQUOI REFAIRE L'ETAPE 1 : les camchain stereo existants sont en
#   848x480 alors que le systeme tourne en 640x480 (verifie 2026-07-28).
#   Une intrinseque calibree a une autre resolution fausse silencieusement la
#   projection. Si vous avez DEJA un camchain 640x480 valide, passez-le en
#   2e argument et l'etape 1 est sautee.
#
#   Usage :
#     tools/calib_imucam_run.sh <bag>                  # etapes 1 + 2
#     tools/calib_imucam_run.sh <bag> <camchain.yaml>  # etape 2 seule
#
#   Duree : 10 a 40 min selon la longueur du bag. Gourmand en CPU.
# ═════════════════════════════════════════════════════════════════════════════
set -u
BAG="${1:?usage: calib_imucam_run.sh <bag> [camchain.yaml]}"
CAMCHAIN="${2:-}"
DIR=$(dirname "$BAG"); BASE=$(basename "$BAG" .bag)
TARGET=/home/lab272/TOUT/catkin_ws/calibration_leo_d455/target.yaml
IMUYAML="$DIR/${BASE}-imu.yaml"

source /opt/ros/noetic/setup.bash
source /home/lab272/TOUT/kalibr_ws/devel/setup.bash

[ -f "$BAG" ]    || { echo "bag introuvable : $BAG" >&2; exit 1; }
[ -f "$TARGET" ] || { echo "mire introuvable : $TARGET" >&2; exit 1; }

# Bruits IMU : on REUTILISE ceux deja mesures sur ce rover par imu_utils
# (open_vins/config/leo/kalibr_imu_chain.yaml). Les redecouvrir n'apporte
# rien et Kalibr en a besoin dans SON format.
cat > "$IMUYAML" <<EOF
rostopic: /imu/data_clean
update_rate: 80.0
accelerometer_noise_density: 2.3314541632569977e-02
accelerometer_random_walk:   8.9292060085650762e-04
gyroscope_noise_density:     1.2636973145788728e-03
gyroscope_random_walk:       3.3010020064503652e-05
EOF
echo "  imu.yaml genere : $IMUYAML"

if [ -z "$CAMCHAIN" ]; then
  echo
  echo "═══ ETAPE 1/2 : intrinseques + stereo ═══"
  kalibr_calibrate_cameras \
    --bag "$BAG" \
    --topics /pc/camera/infra1/image_rect_raw /pc/camera/infra2/image_rect_raw \
    --models pinhole-radtan pinhole-radtan \
    --target "$TARGET" \
    --bag-from-to 2 1000 \
    --dont-show-report || { echo "ETAPE 1 ECHOUEE" >&2; exit 2; }
  CAMCHAIN="$DIR/${BASE}-camchain.yaml"
fi
[ -f "$CAMCHAIN" ] || { echo "camchain introuvable : $CAMCHAIN" >&2; exit 3; }

echo
echo "═══ ETAPE 2/2 : IMU <-> camera ═══"
kalibr_calibrate_imu_camera \
  --bag "$BAG" \
  --cam "$CAMCHAIN" \
  --imu "$IMUYAML" \
  --target "$TARGET" \
  --dont-show-report || { echo "ETAPE 2 ECHOUEE" >&2; exit 4; }

RES="$DIR/${BASE}-camchain-imucam.yaml"
echo
echo "══════════════════════════════════════════════════════════════"
echo "  RESULTAT : $RES"
echo "══════════════════════════════════════════════════════════════"
[ -f "$RES" ] && head -20 "$RES"
cat <<'NEXT'

  ── CONTROLE AVANT D'APPLIQUER (ne pas sauter) ──────────────────
  Ouvrir le rapport PDF *-report-imucam.pdf et verifier :
    * reprojection error  < ~0.5 px  (au-dela : mire floue ou mouvement
      trop mou, RECOMMENCER l'enregistrement)
    * les residus IMU ne doivent pas diverger en fin de sequence
    * timeshift_cam_imu de l'ordre de quelques ms (pas 0.4 s comme
      l'ancienne calib mono de mai, qui etait aberrante)
  Comparer p_CinI au reel : la camera est ~15 cm devant et ~10 cm au-dessus
  de l'IMU. Un resultat a 50 cm ou avec un signe inverse = calibration ratee.

  ── APPLIQUER ────────────────────────────────────────────────────
    tools/calib_imucam_apply.sh <resultat-camchain-imucam.yaml>
NEXT

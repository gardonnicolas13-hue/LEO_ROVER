#!/usr/bin/env bash
# ═════════════════════════════════════════════════════════════════════════════
# calib_imucam_apply.sh — installe un resultat Kalibr dans la config openVINS,
#   avec sauvegarde et verification de coherence physique.
#
#   PIEGE DE CONVENTION (le point qui casse tout si on se trompe) :
#     Kalibr  -> T_cam_imu  : transforme un point IMU vers le repere CAMERA
#     openVINS-> T_cam_imu  : MEME convention  => copie DIRECTE, sans inverser
#     MINS    -> T_imu_cam  : convention INVERSE => il faut inverser
#   Le fichier open_vins/config/leo/kalibr_imucam_chain.yaml documente deja ce
#   piege ("C'est l'INVERSE du T_imu_cam de MINS"). Ce script ne touche QUE la
#   config openVINS et laisse MINS tranquille, pour ne pas melanger les deux.
#
#   Usage : tools/calib_imucam_apply.sh <resultat-camchain-imucam.yaml>
# ═════════════════════════════════════════════════════════════════════════════
set -u
SRC="${1:?usage: calib_imucam_apply.sh <camchain-imucam.yaml>}"
DST=/home/lab272/TOUT/catkin_ws/src/open_vins/config/leo/kalibr_imucam_chain.yaml
[ -f "$SRC" ] || { echo "introuvable : $SRC" >&2; exit 1; }

python3 - "$SRC" "$DST" <<'PY'
import sys, math, shutil, datetime, re

src, dst = sys.argv[1], sys.argv[2]
txt = open(src).read()

# Parse minimal (yaml peut manquer) : recupere T_cam_imu de cam0
def grab(block, key, n):
    m = re.search(r'%s:\s*\n((?:\s*-\s*\[.*\]\s*\n){%d})' % (key, n), block)
    if not m: return None
    rows=[]
    for line in m.group(1).strip().splitlines():
        rows.append([float(v) for v in re.findall(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?', line)])
    return rows

cam0 = txt.split('cam1:')[0]
T = grab(cam0, 'T_cam_imu', 4)
if not T:
    print("ERREUR : T_cam_imu introuvable dans", src); sys.exit(2)

R = [r[:3] for r in T[:3]]
p_IinC = [T[i][3] for i in range(3)]
# p_CinI = -R^T . p_IinC   (position de la camera DANS le repere IMU)
p_CinI = [-sum(R[k][i]*p_IinC[k] for k in range(3)) for i in range(3)]

print("  ── CONTROLE PHYSIQUE ─────────────────────────────────")
print(f"  p_CinI (camera dans repere IMU) = "
      f"({p_CinI[0]:+.3f}, {p_CinI[1]:+.3f}, {p_CinI[2]:+.3f}) m")
print(f"  attendu, mesure a la main       = (+0.150, +0.000, +0.100) m")
d = math.dist(p_CinI, (0.15, 0.0, 0.10))
print(f"  ecart = {d:.3f} m")
ts = re.search(r'timeshift_cam_imu:\s*(-?\d+\.?\d*(?:[eE][-+]?\d+)?)', cam0)
if ts:
    print(f"  timeshift_cam_imu = {float(ts.group(1))*1000:+.1f} ms")

if d > 0.20:
    print()
    print("  *** REFUS : ecart > 20 cm avec la mesure physique.")
    print("      Calibration probablement ratee (mouvement insuffisant,")
    print("      mire mal vue, ou 6-DOF non excites). NE PAS APPLIQUER.")
    print("      Refaire tools/calib_imucam_record.sh avec des mouvements")
    print("      plus amples sur les 3 axes de rotation.")
    sys.exit(3)

bak = dst + '.bak-' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
shutil.copy(dst, bak)
shutil.copy(src, dst)
print()
print(f"  ✓ applique   : {dst}")
print(f"  ✓ sauvegarde : {bak}")
PY
rc=$?
[ $rc -ne 0 ] && exit $rc

cat <<'NEXT'

  ── ENSUITE ──────────────────────────────────────────────────────
  1. Verifier que les rostopic du fichier applique pointent bien sur
     /pc/camera/infra1|2/image_rect_raw (Kalibr recopie ceux du bag).
  2. Relancer openVINS :
       kill -INT $(pgrep -f run_subscribe_msckf)
  3. Initialiser : robot immobile 2-3 s, puis secousse nette.
  4. VALIDER PAR LA MESURE, pas a l'oeil : rouler ~5 m tout droit et
     comparer le deplacement net a /wheel_odom_with_covariance.
     Avant calibration : VINS annoncait 298 m pour 6 m reels (49x).
     Objectif : ratio VINS/roues entre 0.7 et 1.4.
NEXT

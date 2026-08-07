#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_tilt_check.py — teste si l'inclinaison mesuree le 2026-08-05
(6.67 deg de tangage, 3.95 deg de roulis, cf. Fusion_Campaign.tex
S:aug05tilt) vient du SOL ou du MONTAGE de l'IMU.

PRINCIPE (deux lancements, robot IMMOBILE a chaque fois) :
    1. avant : python3 imu_tilt_check.py avant
    2. tourner le robot de 180 degres SUR PLACE (pivot, pas de translation)
    3. apres : python3 imu_tilt_check.py apres

Si le sol n'est pas de niveau, le robot penche TOUJOURS dans la meme
direction MONDE -> vu du robot, apres un demi-tour, l'inclinaison mesuree
change de SIGNE (roulis et tangage s'inversent).
Si l'IMU est montee de travers dans le chassis, l'inclinaison est
SOLIDAIRE du robot -> elle reste IDENTIQUE apres le demi-tour, quelle que
soit l'orientation dans la piece.

Formules identiques a celles du rapport (convention REP-103, x avant/
y gauche/z haut) :
    roll  = atan2(ay, az)
    pitch = atan2(-ax, hypot(ay, az))

Etat conserve entre les deux lancements dans un petit fichier JSON,
volontairement HORS du depot (scratch), pour ne jamais polluer git avec
une mesure de terrain.
"""
import argparse
import json
import math
import os
import sys
import time

STATE_FILE = "/tmp/imu_tilt_check_state.json"
TOPIC = "/imu/data_clean"
DURATION_S = 8.0


def capture():
    import rospy
    from sensor_msgs.msg import Imu

    samples = []
    rospy.init_node("imu_tilt_check", anonymous=True, disable_signals=True)
    rospy.Subscriber(TOPIC, Imu, lambda m: samples.append(
        (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)))
    print(f"capture de {DURATION_S:.0f} s sur {TOPIC} -- robot IMMOBILE svp...")
    t0 = time.time()
    while time.time() - t0 < DURATION_S and not rospy.is_shutdown():
        time.sleep(0.1)
    if len(samples) < 10:
        print(f"ERREUR : seulement {len(samples)} echantillons recus sur {TOPIC}.")
        print("Le topic publie-t-il ? (rostopic hz {})".format(TOPIC))
        sys.exit(1)
    n = len(samples)
    ax = sum(s[0] for s in samples) / n
    ay = sum(s[1] for s in samples) / n
    az = sum(s[2] for s in samples) / n
    sx = (sum((s[0] - ax) ** 2 for s in samples) / n) ** 0.5
    sy = (sum((s[1] - ay) ** 2 for s in samples) / n) ** 0.5
    sz = (sum((s[2] - az) ** 2 for s in samples) / n) ** 0.5
    if max(sx, sy, sz) > 0.05:
        print(f"ATTENTION : ecart-type eleve (x={sx:.3f} y={sy:.3f} z={sz:.3f}) "
              "-- le robot a peut-etre bouge pendant la capture. Recommence.")
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return {"n": n, "ax": ax, "ay": ay, "az": az, "norm": norm,
            "roll_deg": roll, "pitch_deg": pitch}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("phase", choices=["avant", "apres"],
                     help="'avant' = avant le demi-tour, 'apres' = apres")
    args = ap.parse_args()

    r = capture()
    print()
    print(f"  norme         : {r['norm']:.4f} m/s2  (g local = 9.790, "
          f"ecart {r['norm']-9.790:+.4f})")
    print(f"  roulis         : {r['roll_deg']:+.2f} deg")
    print(f"  tangage        : {r['pitch_deg']:+.2f} deg")

    if args.phase == "avant":
        with open(STATE_FILE, "w") as f:
            json.dump(r, f)
        print()
        print(f"Enregistre dans {STATE_FILE}.")
        print("Tourne maintenant le robot de 180 degres SUR PLACE (pivot pur,")
        print("pas de translation), puis relance : python3 imu_tilt_check.py apres")
        return

    # phase == "apres"
    if not os.path.isfile(STATE_FILE):
        print(f"ERREUR : {STATE_FILE} absent -- lance d'abord la phase 'avant'.")
        sys.exit(1)
    with open(STATE_FILE) as f:
        before = json.load(f)

    print()
    print("=== comparaison ===")
    print(f"  {'':14s}  avant       apres")
    print(f"  {'roulis':14s}  {before['roll_deg']:+7.2f}    {r['roll_deg']:+7.2f}")
    print(f"  {'tangage':14s}  {before['pitch_deg']:+7.2f}    {r['pitch_deg']:+7.2f}")
    print()

    # Un demi-tour vrai devrait rapprocher (sol) ou eloigner (montage) des
    # deux hypotheses ; on teste laquelle colle le mieux aux deux mesures,
    # sans affirmer une conclusion tranchee si les deux sont mauvaises (un
    # demi-tour imparfait, un sol qui varie localement, etc. resteraient
    # possibles).
    roll_flip_err = abs(r["roll_deg"] - (-before["roll_deg"]))
    roll_same_err = abs(r["roll_deg"] - before["roll_deg"])
    pitch_flip_err = abs(r["pitch_deg"] - (-before["pitch_deg"]))
    pitch_same_err = abs(r["pitch_deg"] - before["pitch_deg"])

    print(f"  ecart au scenario 'meme signe' (montage)   : "
          f"roulis {roll_same_err:.2f} deg, tangage {pitch_same_err:.2f} deg")
    print(f"  ecart au scenario 'signe inverse' (sol)    : "
          f"roulis {roll_flip_err:.2f} deg, tangage {pitch_flip_err:.2f} deg")
    print()

    total_same = roll_same_err + pitch_same_err
    total_flip = roll_flip_err + pitch_flip_err
    if abs(total_same - total_flip) < 1.0:
        print("VERDICT : ambigu (moins de 1 deg d'ecart entre les deux hypotheses).")
        print("  Le demi-tour n'etait peut-etre pas un pivot pur, ou le sol varie")
        print("  localement. Refaire la mesure, en verifiant l'ecart-type ci-dessus.")
    elif total_same < total_flip:
        print("VERDICT : MONTAGE. L'inclinaison est restee quasi identique apres le")
        print("  demi-tour -- elle est solidaire du robot, pas du sol. A corriger")
        print("  dans l'URDF (base_link -> imu_frame), pas en deplacant le robot.")
    else:
        print("VERDICT : SOL. L'inclinaison s'est inversee avec le robot -- c'est")
        print("  le support qui n'est pas de niveau, pas le montage de l'IMU.")

    os.remove(STATE_FILE)
    print()
    print(f"({STATE_FILE} efface -- relance 'avant' pour un nouvel essai)")


if __name__ == "__main__":
    main()

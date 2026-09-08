#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verif_minimu9.py — contrôle et pré-calibration du Pololu MinIMU-9 v5.

À LANCER DÈS QUE LE CAPTEUR EST CÂBLÉ, avant de le brancher dans la chaîne
d'estimation. Il ne modifie rien : il mesure et il conclut.

    # 1. le capteur répond-il seulement ?
    i2cdetect -y 1                 # 0x6B ou 0x6A (LSM6DS33), 0x1E ou 0x1C (LIS3MDL)

    # 2. contrôle statique, robot IMMOBILE et de niveau
    python3 tools/verif_minimu9.py

    # 3. estimation du facteur d'échelle sur plusieurs orientations
    python3 tools/verif_minimu9.py --orientation "a plat"
    #   ... retourner le robot ...
    python3 tools/verif_minimu9.py --orientation "sur le flanc gauche"
    python3 tools/verif_minimu9.py --bilan      # combine les orientations

POURQUOI 9,790 ET NON 9,81
--------------------------
La pesanteur normale (9,80665) vaut au niveau de la mer à 45° de latitude.
Ce projet utilise 9,790 m/s², valeur retenue pour Melbourne (Floride, ~28°N)
et déjà inscrite dans imu_sanitizer.py, où ~accel_scale vaut par défaut
9.790 / 8.7525 = 1.1186. Utiliser 9,81 ici introduirait un écart de 0,2 %
avec le reste de la chaîne — assez pour rendre deux mesures incomparables.

CE QUE CE SCRIPT NE FAIT PAS
----------------------------
Il n'écrit aucun paramètre et ne corrige rien. Une seule orientation ne
permet PAS d'estimer un facteur d'échelle honnête : à plat, on ne mesure que
l'axe Z, et le biais y est indiscernable de l'erreur d'échelle. C'est
exactement le défaut connu de ~accel_scale, qui reste une constante unique
mesurée sur une seule pose (voir imu_sanitizer.py, section ACCEL SCALE).
Le mode --bilan sert à ne pas refaire cette erreur avec le nouveau capteur.
"""

import argparse
import json
import math
import os
import sys
import time

try:
    import rospy
    from sensor_msgs.msg import Imu as SensorImu
except Exception as e:                                    # pragma: no cover
    sys.stderr.write("ROS introuvable : source /opt/ros/noetic/setup.bash\n")
    sys.stderr.write(str(e) + "\n")
    sys.exit(1)

try:
    from leo_msgs.msg import Imu as LeoImu
except Exception:
    LeoImu = None

G_LOCALE = 9.790            # m/s^2, Melbourne FL — cf. imu_sanitizer.py
ETAT = "/tmp/verif_minimu9_orientations.json"

# Seuils. Volontairement explicites plutôt que magiques.
TOL_NORME = 0.05            # m/s^2 : 0,5 % de g, au-delà on parle d'échelle
TOL_ECART_TYPE = 0.08       # m/s^2 : au-delà, le robot n'était pas immobile
TOL_GYRO_REPOS = 0.02       # rad/s : biais gyro acceptable au repos


class Collecteur(object):
    def __init__(self, topic, duree):
        self.duree = duree
        self.a = []          # (ax, ay, az)
        self.g = []          # (gx, gy, gz)
        self.t = []          # horodatages d'arrivée, pour le débit réel
        self.type_msg = None

        # Le capteur peut publier en sensor_msgs (chaîne VINS) ou en
        # leo_msgs (chaîne firmware, celle que lit imu_sanitizer). On
        # accepte les deux plutôt que d'imposer un branchement.
        info = self._type_du_topic(topic)
        if info == "leo_msgs/Imu" and LeoImu is not None:
            self.type_msg = "leo_msgs"
            rospy.Subscriber(topic, LeoImu, self._cb_leo, queue_size=200)
        else:
            self.type_msg = "sensor_msgs"
            rospy.Subscriber(topic, SensorImu, self._cb_sensor, queue_size=200)

    @staticmethod
    def _type_du_topic(topic):
        try:
            for nom, typ in rospy.get_published_topics():
                if nom == topic:
                    return typ
        except Exception:
            pass
        return None

    def _cb_sensor(self, m):
        a, g = m.linear_acceleration, m.angular_velocity
        self.a.append((a.x, a.y, a.z))
        self.g.append((g.x, g.y, g.z))
        self.t.append(time.time())

    def _cb_leo(self, m):
        self.a.append((m.accel_x, m.accel_y, m.accel_z))
        self.g.append((m.gyro_x, m.gyro_y, m.gyro_z))
        self.t.append(time.time())

    def collecter(self):
        t0 = time.time()
        while time.time() - t0 < self.duree and not rospy.is_shutdown():
            time.sleep(0.05)
        return len(self.a)


def moyenne(v):
    return sum(v) / len(v) if v else float("nan")


def ecart_type(v):
    if len(v) < 2:
        return float("nan")
    m = moyenne(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def ligne(etat, libelle, valeur, detail=""):
    marque = {"ok": "  OK  ", "ko": "ECHEC ", "??": " INFO "}[etat]
    print("  [%s] %-34s %s   %s" % (marque, libelle, valeur, detail))


def analyser(col, attendu_immobile=True):
    n = len(col.a)
    if n == 0:
        ligne("ko", "flux", "AUCUN message")
        print("\n  Le topic ne publie rien. Vérifier que minimu9_node tourne,")
        print("  puis que i2cdetect voit bien la puce.")
        return None

    duree = col.t[-1] - col.t[0] if len(col.t) > 1 else 0.0
    hz = (n - 1) / duree if duree > 0 else 0.0

    ax = [p[0] for p in col.a]
    ay = [p[1] for p in col.a]
    az = [p[2] for p in col.a]
    normes = [math.sqrt(x * x + y * y + z * z) for x, y, z in col.a]
    gx = [p[0] for p in col.g]
    gy = [p[1] for p in col.g]
    gz = [p[2] for p in col.g]

    print("\n  --- flux ---")
    ligne("??", "type de message", col.type_msg)
    ligne("ok" if hz > 40 else "ko", "débit", "%.1f Hz" % hz,
          "(cible 50-100 ; MINS tourne à 85,7 Hz aujourd'hui)")
    ligne("??", "échantillons", "%d sur %.1f s" % (n, duree))

    print("\n  --- immobilité (préalable à toute conclusion) ---")
    et = max(ecart_type(ax), ecart_type(ay), ecart_type(az))
    immobile = et < TOL_ECART_TYPE
    ligne("ok" if immobile or not attendu_immobile else "ko",
          "écart-type accéléro max", "%.4f m/s²" % et,
          "(seuil %.2f)" % TOL_ECART_TYPE)
    if not immobile and attendu_immobile:
        print("\n  Le robot BOUGEAIT. Aucune conclusion sur l'échelle ou le")
        print("  biais n'est recevable dans cet état — recommencer à l'arrêt.")

    print("\n  --- gravité statique ---")
    nm, ns = moyenne(normes), ecart_type(normes)
    ecart = nm - G_LOCALE
    ligne("ok" if abs(ecart) < TOL_NORME else "ko",
          "|a| moyen", "%.4f m/s²" % nm,
          "écart %+.4f vs %.3f" % (ecart, G_LOCALE))
    ligne("??", "écart-type de |a|", "%.4f m/s²" % ns)
    ligne("??", "facteur d'échelle implicite", "%.4f" % (G_LOCALE / nm if nm else float("nan")),
          "(1.0000 = capteur juste)")

    print("\n  --- axes (à plat : Z ≈ ±g, X et Y ≈ 0) ---")
    for nom, serie in (("X", ax), ("Y", ay), ("Z", az)):
        ligne("??", "accel_%s" % nom, "%+8.4f m/s²" % moyenne(serie),
              "sigma %.4f" % ecart_type(serie))

    print("\n  --- biais gyroscope (robot immobile => doit valoir 0) ---")
    pire = 0.0
    for nom, serie in (("X", gx), ("Y", gy), ("Z", gz)):
        b = moyenne(serie)
        pire = max(pire, abs(b))
        ligne("ok" if abs(b) < TOL_GYRO_REPOS else "ko",
              "biais gyro_%s" % nom, "%+8.5f rad/s" % b,
              "%+.3f deg/s" % math.degrees(b))

    return {
        "n": n, "hz": hz, "immobile": immobile,
        "norme": nm, "sigma_norme": ns,
        "ax": moyenne(ax), "ay": moyenne(ay), "az": moyenne(az),
        "bgx": moyenne(gx), "bgy": moyenne(gy), "bgz": moyenne(gz),
        "gyro_pire": pire,
    }


def verdict(r):
    if r is None:
        return 1
    print("\n" + "-" * 72)
    soucis = []
    if r["hz"] < 40:
        soucis.append("débit trop bas (%.1f Hz)" % r["hz"])
    if not r["immobile"]:
        soucis.append("robot non immobile — mesure non recevable")
    if abs(r["norme"] - G_LOCALE) >= TOL_NORME:
        soucis.append("|a| à %.4f au lieu de %.3f" % (r["norme"], G_LOCALE))
    if r["gyro_pire"] >= TOL_GYRO_REPOS:
        soucis.append("biais gyro %.5f rad/s" % r["gyro_pire"])

    if not soucis:
        print("  VERDICT : capteur conforme.")
        print()
        print("  RAPPEL D'INTÉGRATION — ne pas sauter cette étape :")
        print("  imu_sanitizer applique ~accel_scale, réglé à 1.1297 pour")
        print("  compenser l'accéléromètre CORE2 hors spec (il lit 8,75 au")
        print("  lieu de 9,790). Appliqué à ce capteur-ci, qui est juste, il")
        print("  porterait la gravité à %.2f m/s²." % (r["norme"] * 1.1297))
        print("  METTRE ~accel_scale À 1.0 en même temps que la bascule.")
    else:
        print("  VERDICT : %d point(s) à régler" % len(soucis))
        for s in soucis:
            print("    - " + s)
    print("-" * 72)
    return 0 if not soucis else 2


def enregistrer_orientation(nom, r):
    """Empile une pose pour l'estimation multi-orientations."""
    d = {}
    if os.path.exists(ETAT):
        try:
            d = json.load(open(ETAT))
        except Exception:
            d = {}
    d[nom] = {"norme": r["norme"], "ax": r["ax"], "ay": r["ay"], "az": r["az"]}
    json.dump(d, open(ETAT, "w"), indent=2)
    print("\n  orientation « %s » enregistrée (%d au total) -> %s"
          % (nom, len(d), ETAT))
    if len(d) < 3:
        print("  Il en faut au moins 3, d'axes différents, avant --bilan.")


def bilan():
    if not os.path.exists(ETAT):
        print("  Aucune orientation enregistrée. Utiliser --orientation d'abord.")
        return 1
    d = json.load(open(ETAT))
    print("\n  === BILAN MULTI-ORIENTATIONS (%d poses) ===\n" % len(d))
    print("  %-26s %10s %10s %10s %10s" % ("pose", "|a|", "ax", "ay", "az"))
    normes = []
    for nom, v in d.items():
        print("  %-26s %10.4f %10.4f %10.4f %10.4f"
              % (nom[:26], v["norme"], v["ax"], v["ay"], v["az"]))
        normes.append(v["norme"])

    print()
    if len(d) < 3:
        print("  Moins de 3 poses : un facteur d'échelle unique estimé ici")
        print("  reproduirait exactement le défaut connu de ~accel_scale.")
        return 1

    m, s = moyenne(normes), ecart_type(normes)
    print("  |a| moyen sur les poses : %.4f m/s²  (sigma %.4f)" % (m, s))
    print("  facteur d'échelle global : %.4f" % (G_LOCALE / m))
    print()
    if s > TOL_NORME:
        print("  ATTENTION : |a| varie de %.4f m/s² selon l'orientation." % s)
        print("  Un capteur idéal donne la même norme dans TOUTES les poses.")
        print("  Cette dispersion est le signe d'une erreur d'échelle par axe")
        print("  ou d'un désalignement — un scalaire unique ne la corrigera")
        print("  pas, il en fera seulement la moyenne. C'est précisément la")
        print("  limite documentée de ~accel_scale sur l'IMU actuelle.")
    else:
        print("  Dispersion faible : un facteur scalaire unique est défendable")
        print("  pour ce capteur, contrairement au CORE2.")
    return 0


def main():
    p = argparse.ArgumentParser(description="Contrôle du MinIMU-9 v5")
    p.add_argument("--topic", default="/imu/data_raw",
                   help="topic à écouter (défaut /imu/data_raw)")
    p.add_argument("--duree", type=float, default=15.0,
                   help="durée de collecte en secondes (défaut 15)")
    p.add_argument("--orientation", default=None,
                   help="enregistre la mesure sous ce nom de pose")
    p.add_argument("--bilan", action="store_true",
                   help="combine les poses enregistrées et conclut")
    p.add_argument("--en-mouvement", action="store_true",
                   help="ne pas exiger l'immobilité")
    a = p.parse_args()

    if a.bilan:
        return bilan()

    rospy.init_node("verif_minimu9", anonymous=True, disable_signals=True)
    print("=" * 72)
    print("  CONTRÔLE MinIMU-9 v5 — topic %s, %.0f s" % (a.topic, a.duree))
    print("  robot supposé IMMOBILE et de niveau")
    print("=" * 72)

    col = Collecteur(a.topic, a.duree)
    time.sleep(0.5)
    col.collecter()
    r = analyser(col, attendu_immobile=not a.en_mouvement)
    code = verdict(r)
    if r is not None and a.orientation:
        enregistrer_orientation(a.orientation, r)
    return code


if __name__ == "__main__":
    sys.exit(main())

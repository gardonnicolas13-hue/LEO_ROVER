#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calib_terrain.py — confronte ce que le robot CROIT parcourir a ce qu'il
parcourt REELLEMENT, au metre ruban et au rapporteur.

POURQUOI (2026-07-29, demande operateur) : toutes les corrections apportees
jusqu'ici comparaient des capteurs ENTRE EUX — VINS contre les roues, MINS
contre les roues. Aucune n'etait ancree au monde reel. Or les roues elles-memes
sont fausses (patinage mecanum), donc « VINS/roues = 0.69 » ne dit PAS lequel
des deux se trompe, ni de combien. Sans verite terrain on ne peut que constater
un desaccord, jamais le corriger.

Deux mesures, deux inconnues differentes :

  droite   1 m au sol -> facteur d'ECHELLE en translation.
           Corrige le rayon effectif des roues et revele si VINS/MINS
           sous-estiment (mesure du 28/07 : ratio 0.69 contre les roues).

  rotation 90 deg au sol -> facteur d'ECHELLE du GYROSCOPE.
           C'est la mesure la plus precieuse : imu_sanitizer ne corrige
           aujourd'hui que le BIAIS du gyro, et sa propre documentation dit
           qu'une calibration a UNE SEULE orientation ne peut pas distinguer
           une erreur d'echelle. Ce facteur n'a donc JAMAIS ete mesure. Un
           gyro qui multiplie par 1.1 fausse tous les caps, donc toutes les
           trajectoires, quel que soit l'estimateur.

METHODE : on ne fait aucune hypothese sur les capteurs. L'operateur marque le
depart au sol, roule, s'arrete, mesure la realite au ruban, et le programme
compare a ce que chaque source a enregistre sur exactement le meme intervalle.

PLUSIEURS PASSES : la calibration se fait sur la MOYENNE de plusieurs essais,
jamais sur un seul. Un essai isole melange l'erreur d'echelle (systematique,
ce qu'on cherche) avec l'erreur de pose de depart et le patinage du moment
(aleatoires). L'ecart-type affiche dit si le resultat est exploitable :
disperse = refaire, pas appliquer.

SENS DE ROTATION : gauche et droite sont mesures SEPAREMENT. Une asymetrie
signale un probleme mecanique (trim, patinage d'un cote), pas une erreur
d'echelle — et il ne faut surtout pas la corriger par un facteur global.

Usage :
    python3 tools/calib_terrain.py droite            (3 passes par defaut)
    python3 tools/calib_terrain.py rotation gauche
    python3 tools/calib_terrain.py rotation droite
    python3 tools/calib_terrain.py droite 5          (5 passes)
"""
import math
import sys

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

SOURCES = [
    ("roues", "/wheel_odom_with_covariance"),
    ("MINS",  "/mins/imu/odom"),
    ("VINS",  "/ov_msckf/odomimu"),
]
IMU_TOPIC = "/imu/data_clean"


def yaw_of(q):
    """Lacet depuis un quaternion, sans dependance a tf."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Capture(object):
    """Retient la DERNIERE pose de chaque source + integre le gyro.

    On integre le gyro nous-memes plutot que de lire un cap tout fait : c'est
    la seule facon d'isoler l'echelle du CAPTEUR de celle de l'estimateur qui
    le consomme (lequel melange gyro, roues et vision).
    """

    def __init__(self):
        self.last = {}
        self.gyro_yaw = 0.0          # integrale de wz, en rad
        self._t_prev = None
        for nom, topic in SOURCES:
            rospy.Subscriber(topic, Odometry, self._cb(nom), queue_size=50)
        rospy.Subscriber(IMU_TOPIC, Imu, self._cb_imu, queue_size=400)

    def _cb(self, nom):
        def f(m):
            p = m.pose.pose.position
            self.last[nom] = (p.x, p.y, p.z, yaw_of(m.pose.pose.orientation))
        return f

    def _cb_imu(self, m):
        t = m.header.stamp.to_sec()
        if self._t_prev is not None:
            dt = t - self._t_prev
            # Un trou du flux IMU (mesure : 13-21 par 40 s quand le Pi est
            # bride thermiquement) fausserait l'integrale ; on saute les pas
            # aberrants plutot que d'integrer un dt de 40 ms comme s'il etait
            # normal.
            if 0.0 < dt < 0.05:
                self.gyro_yaw += m.angular_velocity.z * dt
        self._t_prev = t

    def snapshot(self):
        s = dict(self.last)
        s["_gyro"] = self.gyro_yaw
        return s


def attendre(msg):
    try:
        input("  " + msg)
    except EOFError:
        print("\n  entree indisponible — lancez le script dans un terminal.")
        sys.exit(1)


def demander_nombre(msg):
    while True:
        try:
            v = input("  " + msg).strip().replace(",", ".")
        except EOFError:
            sys.exit(1)
        try:
            x = float(v)
            if x > 0:
                return x
            print("  valeur positive attendue.")
        except ValueError:
            print("  nombre attendu (ex : 98.5)")


def une_passe(cap, mode, i, n):
    print()
    print("  ---- passe %d / %d ----" % (i, n))
    attendre("Placez le robot et MARQUEZ SA POSITION AU SOL. [Entree] quand pret : ")
    a = cap.snapshot()
    if len([k for k in a if not k.startswith("_")]) == 0:
        print("  aucune source ne publie — pile arretee ?")
        return None
    if mode == "droite":
        attendre("Roulez TOUT DROIT (~1 m) puis ARRETEZ. [Entree] a l'arret : ")
    else:
        attendre("Tournez SUR PLACE d'environ 90 deg puis ARRETEZ. [Entree] a l'arret : ")
    b = cap.snapshot()

    if mode == "droite":
        reel = demander_nombre("Distance REELLE mesuree au ruban, en cm : ") / 100.0
        unite = "m"
    else:
        reel = math.radians(demander_nombre("Angle REEL mesure, en degres : "))
        unite = "deg"

    res = {}
    for nom, _ in SOURCES:
        if nom not in a or nom not in b:
            continue
        if mode == "droite":
            mesure = math.dist(b[nom][:3], a[nom][:3])
        else:
            d = b[nom][3] - a[nom][3]
            mesure = abs(math.atan2(math.sin(d), math.cos(d)))   # repli -pi..pi
        res[nom] = mesure
    if mode == "rotation":
        d = b["_gyro"] - a["_gyro"]
        res["gyro brut"] = abs(d)

    print()
    for nom, mesure in res.items():
        if mode == "droite":
            print("    %-10s a cru parcourir %6.3f m   (reel %.3f m)" % (nom, mesure, reel))
        else:
            print("    %-10s a cru tourner   %6.1f deg (reel %.1f deg)"
                  % (nom, math.degrees(mesure), math.degrees(reel)))
    return reel, res, unite


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode not in ("droite", "rotation"):
        print(__doc__)
        sys.exit(1)
    sens, rest = "", sys.argv[2:]
    if mode == "rotation" and rest and rest[0] in ("gauche", "droite"):
        sens, rest = rest[0], rest[1:]
    n = int(rest[0]) if rest else 3

    rospy.init_node("calib_terrain", anonymous=True, disable_signals=True)
    cap = Capture()
    rospy.sleep(2.0)

    print()
    print("  " + "=" * 66)
    print("  CALIBRATION TERRAIN — %s%s, %d passes"
          % (mode, (" " + sens) if sens else "", n))
    if mode == "droite":
        print("  Mesurez au ruban entre les MEMES reperes (ex : l'axe des roues")
        print("  avant), sinon vous calibrez sur votre facon de viser.")
    else:
        print("  Marquez l'axe du robot au sol AVANT et APRES (ruban adhesif),")
        print("  et mesurez l'angle entre les deux traces au rapporteur.")
    print("  " + "=" * 66)

    passes = []
    for i in range(1, n + 1):
        r = une_passe(cap, mode, i, n)
        if r:
            passes.append(r)
    if not passes:
        print("\n  aucune passe exploitable.")
        return

    # ── Synthese ────────────────────────────────────────────────────────────
    print()
    print("  " + "=" * 66)
    print("  RESULTAT — facteur = reel / mesure  (a MULTIPLIER a la valeur actuelle)")
    print("  " + "=" * 66)
    noms = []
    for _, res, _ in passes:
        for k in res:
            if k not in noms:
                noms.append(k)

    facteurs = {}
    for nom in noms:
        fs = [reel / res[nom] for reel, res, _ in passes if res.get(nom, 0) > 1e-9]
        if not fs:
            continue
        moy = sum(fs) / len(fs)
        ec = (sum((f - moy) ** 2 for f in fs) / len(fs)) ** 0.5
        facteurs[nom] = (moy, ec, len(fs))
        erreur = (1.0 / moy - 1.0) * 100.0
        drapeau = ""
        if ec > 0.10 * moy:
            drapeau = "  <-- DISPERSE, ne pas appliquer : refaire les passes"
        elif abs(erreur) < 2.0:
            drapeau = "  <-- deja juste, ne rien changer"
        print("    %-10s facteur %.4f  (ecart-type %.4f sur %d passes)"
              % (nom, moy, ec, len(fs)))
        print("               le capteur %s de %+.1f %%%s"
              % ("surestime" if erreur > 0 else "sous-estime", erreur, drapeau))

    print()
    print("  OU APPLIQUER")
    if mode == "droite":
        print("    roues : le rayon effectif vit dans le firmware du robot")
        print("            (r_eff = 62.2 mm au 21/07). Nouveau r_eff =")
        if "roues" in facteurs:
            print("            62.2 x %.4f = %.1f mm"
                  % (facteurs["roues"][0], 62.2 * facteurs["roues"][0]))
        print("    MINS/VINS : une echelle fausse ici vient de l'accelerometre")
        print("            ou de la geometrie camera-IMU, PAS d'un parametre a")
        print("            multiplier. Ne forcez pas — notez l'ecart et")
        print("            comparez-le au facteur des roues.")
    else:
        if "gyro brut" in facteurs:
            g = facteurs["gyro brut"][0]
            print("    gyro : ~gyro_scale dans imu_sanitizer.py = %.4f" % g)
            print("           (parametre AJOUTE le 29/07 ; il valait 1.0, donc")
            print("            non corrige, faute de mesure de verite terrain)")
        print("    Refaites la mesure dans l'AUTRE sens avant d'appliquer :")
        print("    si gauche et droite different de plus de ~5 %, le probleme")
        print("    est mecanique (patinage, trim) et un facteur global")
        print("    l'aggraverait au lieu de le corriger.")
    print("  " + "=" * 66)


if __name__ == "__main__":
    main()

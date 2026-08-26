#!/usr/bin/env python3
"""Verifie qu'un estimateur SUIT REELLEMENT le mouvement du robot.

POURQUOI CET OUTIL
------------------
Un estimateur qui publie n'est pas un estimateur qui fonctionne. Le 25/08,
sqrtVINS embarque publiait a 62 Hz, tenait sa position a 4 mm pres a l'arret,
et ne bougeait PAS quand on deplacait le robot : ZUPT etait accepte a chaque
image, et quand ZUPT s'applique VioManager saute toute la mise a jour camera.
Le filtre etait donc aveugle, en marche, et silencieux.

Aucune mesure a l'arret ne peut detecter ca. Il faut bouger, et comparer a une
reference INDEPENDANTE du filtre teste : les roues, via MINS qu'elles ancrent.

CE QUI EST MESURE
-----------------
  * roues        : le robot a-t-il bouge ? Sans ca, rien n'est concluant.
  * deplacement  : distance debut->fin, la grandeur qui compte.
  * chemin cumule: somme des pas ; tres superieur au deplacement = agitation.
  * ratio        : deplacement estimateur / deplacement MINS. Proche de 1 = il
                   suit ; proche de 0 = il est fige.
  * features     : consommees par la mise a jour visuelle. A l'ARRET, 0 est
                   NORMAL (ZUPT court-circuite la camera par conception) ; en
                   MOUVEMENT, 0 signifie que le filtre navigue a l'inertie
                   seule et derivera.

USAGE
-----
    python3 tools/essai_mouvement.py            # 60 s
    python3 tools/essai_mouvement.py 120        # 120 s

Lancer, PUIS conduire le robot pendant toute la fenetre.
"""
import math
import sys

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from leo_msgs.msg import WheelStates

# MINS est la reference : les roues l'ancrent, donc son deplacement ne depend
# d'aucun des filtres visuels qu'on teste contre lui.
SOURCES = [
    ('MINS (reference roues)', '/mins/imu/odom'),
    ('openVINS',               '/ov_msckf/odomimu'),
    ('sqrtVINS',               '/sqrtvins/odomimu'),
]
REFERENCE = 'MINS (reference roues)'
# Sous ce deplacement de reference, le trajet est trop court pour conclure :
# le bruit de chaque estimateur y pese autant que le signal.
DEPLACEMENT_MINIMAL_M = 0.30
# Une roue tourne : seuil bas, le but est de distinguer l'arret du mouvement,
# pas de mesurer une vitesse.
ROUE_BOUGE_RAD_S = 0.05


def main():
    duree = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rospy.init_node('essai_mouvement', anonymous=True)

    traces = {nom: [] for nom, _ in SOURCES}
    roues, features = [], []

    def collecteur(nom):
        def cb(m):
            p = m.pose.pose.position
            traces[nom].append((p.x, p.y, p.z))
        return cb

    for nom, topic in SOURCES:
        rospy.Subscriber(topic, Odometry, collecteur(nom), queue_size=300)
    rospy.Subscriber('/firmware/wheel_states', WheelStates,
                     lambda m: roues.append(max(abs(v) for v in m.velocity)),
                     queue_size=50)
    rospy.Subscriber('/sqrtvins/points_msckf', PointCloud2,
                     lambda m: features.append(m.width), queue_size=50)

    print('')
    print('  >>> CONDUIS LE ROBOT MAINTENANT — %.0f s <<<' % duree)
    print('')
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < duree and not rospy.is_shutdown():
        rospy.sleep(0.5)

    # ── Le robot a-t-il bouge ? Question prealable a toute conclusion ────────
    n_bouge = sum(1 for w in roues if w > ROUE_BOUGE_RAD_S)
    print('')
    print('roues : %d/%d echantillons en mouvement (max %.3f rad/s)'
          % (n_bouge, len(roues), max(roues) if roues else -1))
    if n_bouge == 0:
        print('')
        print('LE ROBOT N\'A PAS BOUGE — essai NON CONCLUANT.')
        print('Relancer et conduire pendant toute la fenetre.')
        return 2

    def parcours(nom):
        p = traces[nom]
        if len(p) < 2:
            return None
        dep = math.dist(p[0], p[-1])
        ch = sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))
        return dep, ch, len(p)

    ref = parcours(REFERENCE)
    print('')
    print('%-24s %13s %13s %8s' % ('', 'deplacement', 'chemin', 'n'))
    for nom, _ in SOURCES:
        r = parcours(nom)
        if r is None:
            print('%-24s %13s' % (nom, 'AUCUNE DONNEE'))
            continue
        print('%-24s %10.3f m %10.3f m %8d' % (nom, r[0], r[1], r[2]))

    if ref is None or ref[0] < DEPLACEMENT_MINIMAL_M:
        print('')
        print('Trajet trop court (%.2f m < %.2f m) — le bruit y pese autant '
              'que le signal.' % (ref[0] if ref else 0.0, DEPLACEMENT_MINIMAL_M))
        return 2

    print('')
    for nom, _ in SOURCES:
        if nom == REFERENCE:
            continue
        r = parcours(nom)
        if r is None:
            continue
        ratio = r[0] / ref[0]
        if ratio < 0.25:
            verdict = 'FIGE — ne suit pas le mouvement'
        elif ratio < 0.60:
            verdict = 'suit partiellement'
        elif ratio > 2.0:
            verdict = 'SUR-ESTIME (derive)'
        else:
            verdict = 'suit'
        print('%-24s ratio/MINS = %5.2f   %s' % (nom, ratio, verdict))

    moy = sum(features) / len(features) if features else -1
    print('')
    print('sqrtVINS, features consommees : %.1f en moyenne (%d messages)'
          % (moy, len(features)))
    if moy == 0:
        print('  0 EN MOUVEMENT = le filtre navigue a l\'inertie seule.')
        print('  Cause deja rencontree : ZUPT accepte en roulage (verifier')
        print('  zupt_chi2_multipler != 0 et zupt_max_velocity < vitesse reelle).')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass

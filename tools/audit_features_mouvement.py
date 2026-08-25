#!/usr/bin/env python3
"""Pourquoi les features meurent-elles en mouvement ? Mesure corrélée aux roues.

LE FAIT À EXPLIQUER
-------------------
Mesuré le 2026-08-24, exposition déjà bridée à 8 ms :
  - à l'arrêt   : openVINS 5,00 features / 100 % de MAJ non nulles
  - en mouvement: openVINS 0,89 features /  29 % de MAJ non nulles
Le flou cinétique n'explique plus cela : à 8 ms et ω = 0,631 rad/s la traînée
vaut 1,7 px, contre 7,0 px à 33 ms. Il faut donc chercher ailleurs.

CE QUE CET OUTIL SÉPARE
-----------------------
Il échantillonne chaque image ET l'état des roues, puis compare les deux
régimes (arrêt / mouvement) sur QUATRE grandeurs indépendantes :

  1. NETTETÉ (variance du laplacien) — s'effondre si l'image est floue.
  2. COINS FAST au seuil réel du filtre (fast_threshold=30) — c'est le
     prédicteur DIRECT du nombre de features exploitables. La netteté peut
     rester correcte pendant que les coins disparaissent, si le contraste
     local baisse sans que l'image soit floue.
  3. LUMINANCE et écart-type — détecte une image qui s'assombrit ou perd son
     contraste pendant le mouvement (auto-exposition figée : une rotation
     vers une zone sombre n'est plus compensée).
  4. CADENCE et TROUS — détecte une perte d'images pendant le mouvement
     (charge CPU, WiFi) que le compteur global masquerait.

Chaque grandeur pointe vers une cause différente :
  netteté ↓ seule            -> flou résiduel malgré les 8 ms
  coins ↓ mais netteté OK    -> perte de contraste / scène pauvre en texture
  luminance ↓                -> exposition figée inadaptée au mouvement
  cadence ↓ / trous ↑        -> perte d'images, pas un problème optique

USAGE
-----
    python3 tools/audit_features_mouvement.py 120
    (faire rouler le robot pendant la capture)
"""
import socket
socket.setdefaulttimeout(8)
import statistics
import sys

import numpy as np
import rospy
from sensor_msgs.msg import Image
from leo_msgs.msg import WheelStates

TOPIC = '/pc/camera/infra1/image_rect_raw'
SEUIL_ROUE = 0.01          # rad/s au-delà duquel on considère le robot en mouvement
FAST_SEUIL = 30            # doit rester aligné sur fast_threshold du YAML


def main():
    duree = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    try:
        import cv2
        from cv_bridge import CvBridge
    except ImportError:
        print('cv2/cv_bridge indisponibles'); return 1

    br = CvBridge()
    fast = cv2.FastFeatureDetector_create(threshold=FAST_SEUIL)
    ech = []                     # (t, bouge, nettete, coins, lum, ecart_type)
    roue = {'v': 0.0}
    stamps = []

    def cb_roue(m):
        roue['v'] = max(abs(x) for x in m.velocity)

    def cb_img(m):
        # L'état des roues est lu au moment de l'image : c'est la seule façon
        # d'attribuer chaque image au bon régime. Corréler après coup sur deux
        # séries échantillonnées différemment introduirait un décalage.
        bouge = roue['v'] > SEUIL_ROUE
        stamps.append(m.header.stamp.to_sec())
        img = br.imgmsg_to_cv2(m, desired_encoding='mono8')
        ech.append((rospy.get_time(), bouge,
                    float(cv2.Laplacian(img, cv2.CV_64F).var()),
                    len(fast.detect(img, None)),
                    float(img.mean()), float(img.std())))

    rospy.init_node('audit_feat_mvt', anonymous=True)
    rospy.Subscriber('/firmware/wheel_states', WheelStates, cb_roue, queue_size=20)
    rospy.Subscriber(TOPIC, Image, cb_img, queue_size=10)

    print('>>> FAIS ROULER LE ROBOT — capture %.0f s <<<' % duree)
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < duree and not rospy.is_shutdown():
        rospy.sleep(0.5)

    if len(ech) < 30:
        print('ECHEC : %d images seulement' % len(ech)); return 1

    arret = [e for e in ech if not e[1]]
    mvt = [e for e in ech if e[1]]
    print('')
    print('  %d images : %d a l arret, %d en mouvement (%.0f %%)'
          % (len(ech), len(arret), len(mvt), 100.0 * len(mvt) / len(ech)))
    if not mvt:
        print('  AUCUNE image en mouvement -- le robot n a pas bouge.')
        return 1
    if not arret:
        print('  AUCUNE image a l arret -- pas de reference de comparaison.')
        return 1

    print('')
    print('  %-22s %12s %12s %10s' % ('grandeur', 'ARRET', 'MOUVEMENT', 'variation'))
    print('  %-22s %12s %12s %10s' % ('-'*22, '-'*12, '-'*12, '-'*10))
    for i, (nom, unite) in enumerate([('nettete (laplacien)', ''), ('coins FAST (seuil %d)' % FAST_SEUIL, ''),
                                      ('luminance moyenne', '/255'), ('ecart-type (contraste)', '')], start=2):
        a = statistics.median([e[i] for e in arret])
        m = statistics.median([e[i] for e in mvt])
        var = ((m - a) / a * 100.0) if a else float('nan')
        print('  %-22s %12.1f %12.1f %9.0f %%' % (nom + unite, a, m, var))

    # Cadence et trous, par régime
    print('')
    dts = [b - a for a, b in zip(stamps, stamps[1:])]
    if dts:
        med = statistics.median([d for d in dts if d > 0])
        trous = sum(1 for d in dts if d > 1.5 * med)
        print('  cadence %.2f Hz | trous (>1.5x dt median) : %d sur %d intervalles (%.1f %%)'
              % (1.0 / med if med else 0, trous, len(dts), 100.0 * trous / len(dts)))

    print('')
    print('  === LECTURE ===')
    n_a = statistics.median([e[2] for e in arret]); n_m = statistics.median([e[2] for e in mvt])
    c_a = statistics.median([e[3] for e in arret]); c_m = statistics.median([e[3] for e in mvt])
    l_a = statistics.median([e[4] for e in arret]); l_m = statistics.median([e[4] for e in mvt])
    if c_m < 0.5 * c_a and n_m < 0.7 * n_a:
        print('  Les coins ET la nettete s effondrent -> FLOU residuel malgre les 8 ms.')
    elif c_m < 0.5 * c_a and n_m >= 0.7 * n_a:
        print('  Les coins s effondrent mais la nettete TIENT -> ce n est pas du flou.')
        print('  Piste : contraste local / scene pauvre en texture dans la direction')
        print('  parcourue, ou saturation-assombrissement (exposition figee).')
    elif abs(l_m - l_a) > 0.25 * l_a:
        print('  La LUMINANCE change fortement -> exposition figee inadaptee ;')
        print('  le rover traverse des zones d eclairage differentes.')
    else:
        print('  Ni les coins ni la nettete ne s effondrent nettement sur CETTE')
        print('  capture -> la perte de features vient d ailleurs que de l image')
        print('  (appariement stereo, triangulation, ou garde chi2 du filtre).')
    return 0


if __name__ == '__main__':
    sys.exit(main())

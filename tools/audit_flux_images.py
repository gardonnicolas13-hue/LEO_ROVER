#!/usr/bin/env python3
"""Audit du flux d'images stéréo : cadence, trous, jitter, synchronisme, netteté.

POURQUOI CET OUTIL
------------------
Le 2026-08-21, openVINS et sqrtVINS ont perdu la totalité de leur canal visuel
en 2 s dès que le rover a bougé (11 features -> 0), puis ont fabriqué une
vitesse fantôme par intégration d'IMU pure. MINS a survécu grâce à ses roues.
La question n'est donc pas « quel filtre est buggé » mais « pourquoi les pixels
meurent en mouvement ». Trois suspects, que ce script sépare :

  1. RÉSEAU     — les images n'arrivent plus, ou arrivent en retard/désordre.
                  Mesuré par la cadence réelle, les trous et le jitter sur les
                  timestamps CAPTEUR (header), jamais l'heure d'arrivée : la
                  seconde mélange le jitter WiFi au jitter capteur.
  2. SYNCHRO    — les deux caméras se désynchronisent, et la triangulation
                  stéréo devient fausse sans que rien ne le signale.
  3. OPTIQUE    — l'image est là, à l'heure, mais FLOUE. Mesuré par la variance
                  du laplacien, l'indicateur de netteté standard : un flou de
                  mouvement l'effondre alors que la cadence reste parfaite.

Le point 3 est celui que ni ROS ni le filtre ne signalent jamais : un flux à
15 Hz impeccable peut ne contenir que de la bouillie.

CONTEXTE MESURÉ SUR CE ROBOT (2026-08-21) :
  - projecteur IR ÉTEINT (laser_power = 0, choix délibéré : il polluait le
    damier de calibration et les LED de la balise) ;
  - auto-exposition ACTIVE, temps de pose mesuré à 33 000 µs = 33 ms, soit la
    MOITIÉ de la période de trame à 15 Hz ;
  - focale 336,37 px, rotation corps mesurée jusqu'à 0,631 rad/s.
  => flou cinétique attendu  b = omega * t_exp * f  =  1,7 px (médiane)
     à 7,0 px (pic). À comparer à min_px_dist = 20 px.

USAGE
-----
    python3 tools/audit_flux_images.py 60          # 60 s sur les deux caméras
    python3 tools/audit_flux_images.py 60 --nettete  # + variance du laplacien
                                                     # (coûteux : décode chaque image)
"""
import socket
socket.setdefaulttimeout(8)
import statistics
import sys

import numpy as np
import rospy
from sensor_msgs.msg import Image

CAM = {
    'infra1': '/pc/camera/infra1/image_rect_raw',
    'infra2': '/pc/camera/infra2/image_rect_raw',
}
NOMINAL_HZ = 15.0


def bilan_temps(nom, stamps, arrivees):
    if len(stamps) < 10:
        print('  %-8s TROP PEU D IMAGES (%d)' % (nom, len(stamps)))
        return None
    dts = [b - a for a, b in zip(stamps, stamps[1:])]
    pos = [d for d in dts if d > 0]
    recul = sum(1 for d in dts if d < 0)
    nuls = sum(1 for d in dts if d == 0)
    med = statistics.median(pos)
    sd = statistics.pstdev(pos) if len(pos) > 1 else 0.0
    span = stamps[-1] - stamps[0]
    hz = len(stamps) / span if span > 0 else 0.0
    # Un « trou » = un intervalle qui vaut au moins 2 périodes nominales :
    # au moins une image a été perdue, pas seulement retardée.
    seuil = 1.5 / NOMINAL_HZ
    trous = [d for d in pos if d > seuil]
    perdues = sum(int(round(d * NOMINAL_HZ)) - 1 for d in trous)
    lat = [b - a for a, b in zip(stamps, arrivees)]
    print('  %-8s %5d images sur %5.1f s  ->  %5.2f Hz (nominal %.0f)'
          % (nom, len(stamps), span, hz, NOMINAL_HZ))
    print('           dt médian %6.1f ms | jitter %5.1f ms (%4.1f %%) | min %5.1f max %6.1f'
          % (med * 1e3, sd * 1e3, 100 * sd / med if med else 0, min(pos) * 1e3, max(pos) * 1e3))
    print('           TROUS (>1.5 periode) : %d  ->  %d image(s) perdue(s) = %.1f %%'
          % (len(trous), perdues, 100.0 * perdues / (len(stamps) + perdues) if perdues else 0.0))
    print('           reculs %d | doublons %d | latence capteur->arrivee : med %.0f ms max %.0f ms'
          % (recul, nuls, statistics.median(lat) * 1e3, max(lat) * 1e3))
    return {'hz': hz, 'perdues': perdues, 'n': len(stamps), 'recul': recul,
            'jitter_pct': 100 * sd / med if med else 0}


def main():
    duree = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 60.0
    nettete = '--nettete' in sys.argv

    stamps = {k: [] for k in CAM}
    arrivees = {k: [] for k in CAM}
    lap = {k: [] for k in CAM}
    bridge = None
    if nettete:
        try:
            from cv_bridge import CvBridge
            import cv2
            bridge = (CvBridge(), cv2)
        except ImportError:
            print('cv_bridge/cv2 absent -> netteté désactivée')
            nettete = False

    def cb(msg, cle):
        stamps[cle].append(msg.header.stamp.to_sec())
        arrivees[cle].append(rospy.get_time())
        if nettete and bridge is not None:
            br, cv2 = bridge
            try:
                img = br.imgmsg_to_cv2(msg, desired_encoding='mono8')
                # Variance du laplacien : mesure de netteté standard. Un flou
                # de mouvement l'effondre, alors que la cadence reste parfaite
                # -- c'est exactement la panne qu'aucun compteur de Hz ne voit.
                lap[cle].append(float(cv2.Laplacian(img, cv2.CV_64F).var()))
            except Exception:
                pass

    rospy.init_node('audit_flux_images', anonymous=True)
    for cle, topic in CAM.items():
        rospy.Subscriber(topic, Image, (lambda c: lambda m: cb(m, c))(cle),
                         queue_size=60)
    print('Audit du flux stéréo pendant %.0f s%s...'
          % (duree, ' (avec netteté)' if nettete else ''))
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < duree and not rospy.is_shutdown():
        rospy.sleep(0.5)

    print('')
    print('=== CADENCE / PERTES / JITTER (temps CAPTEUR) ===')
    res = {}
    for cle in CAM:
        res[cle] = bilan_temps(cle, stamps[cle], arrivees[cle])

    print('')
    print('=== SYNCHRONISME STÉRÉO ===')
    a, b = stamps['infra1'], stamps['infra2']
    if len(a) > 10 and len(b) > 10:
        n = min(len(a), len(b))
        # Appariement au plus proche : les deux flux peuvent avoir perdu des
        # images differentes, un appariement par indice mentirait.
        bb = np.asarray(b)
        ecarts = [float(np.min(np.abs(bb - x))) for x in a[:n]]
        print('  écart cam0<->cam1 : médian %.2f ms | max %.2f ms'
              % (statistics.median(ecarts) * 1e3, max(ecarts) * 1e3))
        print('  images cam0 sans partenaire à moins de 5 ms : %d / %d'
              % (sum(1 for e in ecarts if e > 0.005), n))
    else:
        print('  pas assez d images pour juger')

    if nettete:
        print('')
        print('=== NETTETÉ (variance du laplacien ; s effondre au flou) ===')
        for cle in CAM:
            v = lap[cle]
            if len(v) > 5:
                print('  %-8s médiane %8.1f | min %8.1f | max %8.1f | 10e centile %8.1f'
                      % (cle, statistics.median(v), min(v), max(v),
                         float(np.percentile(v, 10))))

    print('')
    print('=== VERDICT ===')
    alertes = []
    for cle, r in res.items():
        if r is None:
            alertes.append('%s : flux absent' % cle); continue
        if r['hz'] < 0.8 * NOMINAL_HZ:
            alertes.append('%s : cadence %.1f Hz au lieu de %.0f (-%.0f %%)'
                           % (cle, r['hz'], NOMINAL_HZ, 100 * (1 - r['hz'] / NOMINAL_HZ)))
        if r['perdues'] > 0.02 * r['n']:
            alertes.append('%s : %d images perdues (%.1f %%)'
                           % (cle, r['perdues'], 100.0 * r['perdues'] / r['n']))
        if r['recul']:
            alertes.append('%s : %d RECUL(S) temporel(s) — fatal au filtre' % (cle, r['recul']))
        if r['jitter_pct'] > 25:
            alertes.append('%s : jitter %.0f %% du dt' % (cle, r['jitter_pct']))
    if alertes:
        for x in alertes:
            print('  [!] ' + x)
    else:
        print('  Flux image SAIN : cadence, pertes, ordre et synchronisme dans les clous.')
        print('  -> si les features meurent quand meme, la cause est OPTIQUE')
        print('     (flou de mouvement) ou ALGORITHMIQUE (reglages KLT), pas reseau.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

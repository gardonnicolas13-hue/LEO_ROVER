#!/usr/bin/env python3
"""Audit de la regularite temporelle du flux IMU entrant dans openVINS.

POURQUOI (Patrick Geneva, doc openVINS « Filter Tuning / common issues ») : la
propagation IMU d'un MSCKF integre entre deux timestamps CAPTEUR. Un dt
irregulier ne bruite pas seulement l'etat — il decale la fenetre de clones
par rapport aux images, et une feature triangulee contre des poses mal datees
tombe hors du seuil chi2 SANS que rien ne signale la cause. C'est un suspect
de premier plan quand les features sont rejetees alors que l'extracteur
fonctionne (cf. leo-openvins-features-rejetees).

CE QU'IL MESURE, sur les timestamps du HEADER (temps capteur), jamais sur
l'heure d'arrivee : celle-ci melange le jitter reseau WiFi au jitter capteur,
et seul le second entre dans le filtre.
  - dt median / moyen / ecart-type  -> cadence reelle
  - jitter = ecart-type des dt      -> le chiffre que Geneva regarde
  - trous (dt > 3x median)          -> paquets manquants
  - doublons / retours en arriere   -> desordre temporel (fatal au filtre)

Usage : python3 tools/audit_imu_jitter.py [duree_s] [topic]
"""
import socket
socket.setdefaulttimeout(8)
import statistics
import sys

import rospy
from sensor_msgs.msg import Imu

DUREE = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
TOPIC = sys.argv[2] if len(sys.argv) > 2 else '/imu/data_clean'

stamps = []      # temps CAPTEUR (header)
arrivees = []    # temps de reception, pour comparaison seulement


def cb(msg):
    stamps.append(msg.header.stamp.to_sec())
    arrivees.append(rospy.get_time())


def bilan(nom, ts, unite_attendue=None):
    if len(ts) < 20:
        print('%s : trop peu d echantillons (%d)' % (nom, len(ts)))
        return None
    dts = [b - a for a, b in zip(ts, ts[1:])]
    croissants = [d for d in dts if d > 0]
    recul = [d for d in dts if d < 0]
    nuls = [d for d in dts if d == 0]
    med = statistics.median(croissants) if croissants else float('nan')
    moy = statistics.fmean(croissants) if croissants else float('nan')
    sd = statistics.pstdev(croissants) if len(croissants) > 1 else float('nan')
    trous = [d for d in croissants if d > 3 * med]
    print('')
    print('--- %s ---' % nom)
    print('  echantillons   : %d sur %.1f s' % (len(ts), ts[-1] - ts[0]))
    print('  cadence        : %.2f Hz (dt median %.4f s)' % (1.0 / med if med else 0, med))
    print('  dt moyen       : %.4f s' % moy)
    print('  JITTER (sigma) : %.4f s  = %.1f %% du dt median' % (sd, 100.0 * sd / med if med else 0))
    print('  dt min / max   : %.4f / %.4f s' % (min(croissants), max(croissants)))
    print('  trous (>3x med): %d  (%.2f %% des intervalles)'
          % (len(trous), 100.0 * len(trous) / len(dts)))
    if trous:
        pires = sorted(trous, reverse=True)[:5]
        print('     pires trous : %s' % ', '.join('%.3f s' % t for t in pires))
    print('  dt <= 0        : %d recul(s), %d doublon(s)' % (len(recul), len(nuls)))
    return {'med': med, 'sd': sd, 'trous': len(trous), 'recul': len(recul),
            'nuls': len(nuls), 'n': len(dts)}


rospy.init_node('audit_imu_jitter', anonymous=True)
rospy.Subscriber(TOPIC, Imu, cb)
print('Audit de %s pendant %.0f s...' % (TOPIC, DUREE))
t0 = rospy.get_time()
while rospy.get_time() - t0 < DUREE and not rospy.is_shutdown():
    rospy.sleep(0.5)

if len(stamps) < 20:
    print('ECHEC : %d messages recus sur %s' % (len(stamps), TOPIC))
    raise SystemExit(1)

r = bilan('TEMPS CAPTEUR (header) — celui qui entre dans le filtre', stamps)
bilan('TEMPS D ARRIVEE (reseau) — pour comparaison seulement', arrivees)

print('')
print('=== VERDICT ===')
alertes = []
if r['recul']:
    alertes.append('%d RECUL(S) TEMPOREL(S) — fatal, le filtre ne peut pas propager en arriere'
                   % r['recul'])
if r['nuls']:
    alertes.append('%d timestamp(s) DUPLIQUE(S) — dt=0, division par zero potentielle' % r['nuls'])
if r['sd'] > 0.5 * r['med']:
    alertes.append('jitter TRES eleve (sigma > 50 %% du dt) — propagation IMU peu fiable')
elif r['sd'] > 0.2 * r['med']:
    alertes.append('jitter eleve (sigma > 20 %% du dt) — a surveiller')
if 100.0 * r['trous'] / r['n'] > 1.0:
    alertes.append('%.1f %% d intervalles sont des trous (>3x dt median)'
                   % (100.0 * r['trous'] / r['n']))
if alertes:
    for a in alertes:
        print('  [!] ' + a)
else:
    print('  Flux IMU REGULIER : pas de recul, pas de doublon, jitter contenu.')
    print('  -> le jitter IMU n est PAS la cause du rejet de features.')

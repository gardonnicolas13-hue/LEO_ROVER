#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vins_divergence_probe.py — localise l'INSTANT et le CONTEXTE ou openVINS
decroche, au lieu de constater l'ecart final.

POURQUOI (2026-07-28) : openVINS annonce 105 a 298 m pour des trajets reels de
2 a 6 m (ratio ~50x, reproduit 5 fois contre l'odometrie roues). Sept
hypotheses ont ete testees et ecartees par la mesure : crash mutex, echelle
accelero, biais gyro, ZUPT, les 4 combinaisons extrinsics/ZUPT, l'alignement
sur la config MINS, les covariances IMU. Toutes ces mesures comparaient un
ETAT FINAL. Aucune ne dit QUAND la divergence naît ni ce que faisait le robot
a cet instant — or c'est exactement ce qu'il faut pour savoir si un
garde-fou (divergence guard) a un sens et sur quel critere le declencher.

METHODE : on suit en continu le rapport
    distance parcourue VINS / distance parcourue ROUES
sur une fenetre glissante. Les roues servent de reference physique (elles se
trompent de ~30 % en valeur absolue, mais jamais d'un facteur 50). Quand le
rapport franchit un seuil, on enregistre l'INSTANT et le CONTEXTE :
vitesse de rotation, acceleration, temps ecoule depuis l'init.

Lecture des resultats :
  * decrochage des les premieres secondes -> probleme d'INITIALISATION
    (attitude/gravite mal estimees au demarrage)
  * decrochage correle a une forte rotation -> geometrie camera-IMU
    (bras de levier faux : c'est la rotation qui l'excite)
  * decrochage correle a une acceleration -> echelle/biais accelero
  * derive lente et continue sans evenement -> biais residuel integre

Usage :  python3 tools/vins_divergence_probe.py [duree_s]     (defaut 180)
         Le robot doit ROULER pendant la mesure.
"""
import sys
import time
import math

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

VINS_TOPIC  = "/ov_msckf/odomimu"
WHEEL_TOPIC = "/wheel_odom_with_covariance"
IMU_TOPIC   = "/imu/data_clean"

WIN_S        = 2.0    # fenetre glissante d'analyse
MIN_WHEEL_M  = 0.05   # deplacement roues minimal pour que le ratio ait un sens
RATIO_ALERTE = 3.0    # au-dela, on considere que VINS decroche


class Probe(object):
    def __init__(self, duration):
        self.duration = duration
        self.vins, self.wheel, self.imu = [], [], []
        self.t_first_vins = None
        self.events = []
        rospy.Subscriber(VINS_TOPIC, Odometry, self._cb_vins, queue_size=200)
        rospy.Subscriber(WHEEL_TOPIC, Odometry, self._cb_wheel, queue_size=200)
        rospy.Subscriber(IMU_TOPIC, Imu, self._cb_imu, queue_size=400)

    def _cb_vins(self, m):
        p = m.pose.pose.position
        if self.t_first_vins is None:
            self.t_first_vins = time.time()
        self.vins.append((time.time(), p.x, p.y, p.z))

    def _cb_wheel(self, m):
        p = m.pose.pose.position
        self.wheel.append((time.time(), p.x, p.y, p.z))

    def _cb_imu(self, m):
        g, a = m.angular_velocity, m.linear_acceleration
        self.imu.append((time.time(),
                         math.sqrt(g.x ** 2 + g.y ** 2 + g.z ** 2),
                         math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)))

    @staticmethod
    def _path(seq):
        return sum(math.dist(seq[i][1:], seq[i - 1][1:])
                   for i in range(1, len(seq)))

    def _window(self, seq, t0, t1):
        return [s for s in seq if t0 <= s[0] <= t1]

    def run(self):
        t_start = time.time()
        last_report = t_start
        while time.time() - t_start < self.duration and not rospy.is_shutdown():
            time.sleep(0.5)
            now = time.time()
            wv = self._window(self.vins, now - WIN_S, now)
            ww = self._window(self.wheel, now - WIN_S, now)
            wi = self._window(self.imu, now - WIN_S, now)
            if len(wv) < 10 or len(ww) < 5 or not wi:
                continue
            d_w = self._path(ww)
            if d_w < MIN_WHEEL_M:        # robot quasi immobile : ratio non defini
                continue
            d_v = self._path(wv)
            ratio = d_v / d_w
            gyro_max = max(s[1] for s in wi)
            acc_amp = max(s[2] for s in wi) - min(s[2] for s in wi)
            t_init = (now - self.t_first_vins) if self.t_first_vins else -1
            if ratio > RATIO_ALERTE:
                self.events.append((t_init, ratio, gyro_max, acc_amp, d_w))
            if now - last_report >= 10.0:
                last_report = now
                print("    t+%5.1fs  ratio=%7.2f  gyro_max=%5.2f rad/s  "
                      "accel_amp=%5.2f  roues=%.2f m"
                      % (t_init, ratio, gyro_max, acc_amp, d_w), flush=True)
        self.report()

    def report(self):
        print()
        print("  " + "=" * 62)
        if not self.vins:
            print("  VINS n'a JAMAIS publie — pas initialise (secousse requise)")
            return
        if not self.events:
            print("  Aucun decrochage detecte (ratio reste sous %.1f)" % RATIO_ALERTE)
            print("  -> soit le robot n'a pas assez roule, soit VINS a tenu")
            return
        first = self.events[0]
        print("  PREMIER DECROCHAGE :")
        print("    %.1f s apres la premiere pose VINS" % first[0])
        print("    ratio VINS/roues        : %.1f" % first[1])
        print("    rotation a cet instant  : %.2f rad/s (%.0f deg/s)"
              % (first[2], math.degrees(first[2])))
        print("    variation acceleration  : %.2f m/s^2" % first[3])
        print()
        # Correlation grossiere : le decrochage arrive-t-il plutot en rotation
        # ou en acceleration ? On compare la mediane des evenements a ce qu'on
        # observe globalement.
        gy = sorted(e[2] for e in self.events)
        ac = sorted(e[3] for e in self.events)
        allg = sorted(s[1] for s in self.imu) if self.imu else [0]
        print("  Sur %d fenetres en decrochage :" % len(self.events))
        print("    rotation mediane   : %.2f rad/s  (mediane globale %.2f)"
              % (gy[len(gy) // 2], allg[len(allg) // 2]))
        print("    accel_amp mediane  : %.2f m/s^2" % ac[len(ac) // 2])
        print()
        if first[0] < 5.0:
            print("  LECTURE : decrochage IMMEDIAT -> suspecter l'INITIALISATION")
        elif gy[len(gy) // 2] > 2 * max(allg[len(allg) // 2], 1e-3):
            print("  LECTURE : decrochage correle a la ROTATION")
            print("            -> geometrie camera-IMU (bras de levier)")
        else:
            print("  LECTURE : derive continue sans evenement declencheur net")
            print("            -> biais residuel integre")
        print("  " + "=" * 62)


if __name__ == "__main__":
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    rospy.init_node("vins_divergence_probe", anonymous=True, disable_signals=True)
    print("  Sonde de divergence — %.0f s. FAITES ROULER LE ROBOT." % dur)
    print("  (avancez, tournez, alternez — pour couvrir les deux regimes)")
    Probe(dur).run()

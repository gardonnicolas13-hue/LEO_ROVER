#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hotrun_monitor.py — constantes vitales pendant un essai a chaud.

POURQUOI (2026-07-30) : l'operateur a decide d'activer la camera COULEUR sans
refroidissement actif, sur un Pi deja a 85,7 C et en limite thermique douce.
C'est un risque assume, mais il ne doit pas etre subi en aveugle : la couleur
est nommement responsable, dans /etc/ros/robot.launch, du « throttling
thermique + desync rosserial », et son activation a deja provoque une PERTE DE
CONTROLE du robot le 27/07.

Ce moniteur surveille les trois grandeurs qui, ensemble, annoncent cette
panne AVANT qu'elle ne devienne une perte de pilotage :

  1. TEMPERATURE + get_throttled  — la cause
  2. FREQUENCE CPU                — l'effet immediat (bridage a 600 MHz)
  3. TROUS DANS L'IMU             — la consequence qui casse les estimateurs

Le troisieme est le plus important et le moins evident : quand le Pi est
affame, la chaine serie n'est plus servie a temps et des echantillons IMU sont
perdus SANS ERREUR VISIBLE. C'est ce qui faisait mourir openVINS sur
Propagator.cpp:101. Un taux de trous qui grimpe est le signal d'arret.

CRITERE D'ARRET propose (affiche en clair a chaque ligne) :
    temp > 88 C  OU  trous > 25 / 20 s  OU  odometrie roues < 10 Hz
Le dernier signifie que la liaison serie decroche : c'est le seuil au-dela
duquel le pilotage devient incertain.

Usage :  python3 tools/hotrun_monitor.py [duree_s]      (defaut : illimite)
         Ctrl-C pour arreter. Une ligne toutes les ~20 s.
"""
import math
import subprocess
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

ROBOT = "10.200.0.2"
WIN_S = 20.0
T_ALERTE = 88.0
TROUS_ALERTE = 25
ODOM_MIN_HZ = 10.0


def robot_vitals():
    """Temp / bridage / frequence, via une seule connexion SSH pour ne pas
    ajouter nous-memes de la charge sur un Pi deja sature."""
    cmd = ("vcgencmd measure_temp; vcgencmd get_throttled; "
           "vcgencmd measure_clock arm; cut -d' ' -f1 /proc/loadavg")
    try:
        out = subprocess.run(
            ["ssh", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=no",
             "pi@" + ROBOT, cmd],
            capture_output=True, text=True, timeout=15).stdout.split("\n")
        temp = float(out[0].split("=")[1].replace("'C", ""))
        thr = out[1].split("=")[1].strip()
        freq = int(out[2].split("=")[1]) / 1e6
        load = float(out[3])
        return temp, thr, freq, load
    except Exception as e:
        return None, str(e)[:24], None, None


class Monitor(object):
    def __init__(self):
        self.imu = []
        self.odom = []
        rospy.Subscriber("/imu/data_clean", Imu,
                         lambda m: self.imu.append(m.header.stamp.to_sec()),
                         queue_size=2000)
        rospy.Subscriber("/wheel_odom_with_covariance", Odometry,
                         lambda m: self.odom.append(time.time()),
                         queue_size=500)

    def fenetre(self):
        """Trous IMU et cadence odometrie sur la fenetre ecoulee."""
        imu, self.imu = self.imu, []
        odo, self.odom = self.odom, []
        trous, hz_imu = 0, 0.0
        if len(imu) > 10:
            d = [imu[i] - imu[i - 1] for i in range(1, len(imu))]
            d = [x for x in d if x > 0]
            if d:
                med = sorted(d)[len(d) // 2]
                trous = len([x for x in d if x > 3 * med])
                hz_imu = 1.0 / med
        return trous, hz_imu, len(odo) / WIN_S

    def run(self, duree):
        t0 = time.time()
        print("  %-8s %-7s %-11s %-8s %-6s %-7s %-8s %s"
              % ("t", "temp", "bridage", "freq", "trous", "IMU Hz",
                 "odom Hz", "VERDICT"), flush=True)
        while not rospy.is_shutdown():
            if duree and time.time() - t0 > duree:
                break
            time.sleep(WIN_S)
            trous, hz_imu, hz_odom = self.fenetre()
            temp, thr, freq, _load = robot_vitals()
            alertes = []
            if temp is not None and temp > T_ALERTE:
                alertes.append("TEMP")
            if trous > TROUS_ALERTE:
                alertes.append("IMU")
            if hz_odom < ODOM_MIN_HZ:
                alertes.append("SERIE")
            verdict = "ARRET : " + "+".join(alertes) if alertes else "ok"
            print("  %-8s %-7s %-11s %-8s %-6d %-7.1f %-8.1f %s"
                  % ("%.0fs" % (time.time() - t0),
                     "%.1fC" % temp if temp is not None else "?",
                     thr or "?",
                     "%.0fMHz" % freq if freq is not None else "?",
                     trous, hz_imu, hz_odom, verdict), flush=True)


if __name__ == "__main__":
    d = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    rospy.init_node("hotrun_monitor", anonymous=True, disable_signals=True)
    print("  Moniteur essai a chaud — fenetre %.0f s" % WIN_S, flush=True)
    print("  Seuils d'arret : temp>%.0fC | trous>%d | odom<%.0f Hz"
          % (T_ALERTE, TROUS_ALERTE, ODOM_MIN_HZ), flush=True)
    Monitor().run(d)

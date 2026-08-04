#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vins_guard_validate.py — mesure ce que le garde-fou apporte REELLEMENT.

POURQUOI un script separe de vins_divergence_probe.py :
la sonde de divergence repondait a « ou et quand openVINS decroche-t-il ? ».
Elle a repondu : exclusivement en rotation (ratio 30.2 a 0.64 rad/s, contre
0.92-1.36 en ligne droite). Le garde-fou a ete construit sur cette reponse.
Reste la question qui n'est PAS la meme : « le garde-fou corrige-t-il ce
decrochage, et a quel prix ? ». Y repondre demande de comparer le brut et le
filtre SUR LE MEME TRAJET — sinon on compare deux parcours differents et la
mesure ne prouve rien.

METHODE : on ecoute simultanement les trois sources
    /ov_msckf/odomimu_raw   openVINS brut   (ce qu'on aurait sans garde-fou)
    /ov_msckf/odomimu       sortie filtree  (ce que le trace affiche)
    /wheel_odom_with_covariance             reference physique
Les roues se trompent de ~30 % en absolu mais jamais d'un facteur 50 : elles
servent d'etalon, pas de verite. On separe ensuite les fenetres en deux
regimes selon le gyroscope, parce que c'est precisement la ou le garde-fou
agit :
    ligne droite (< seuil) : le garde ne doit RIEN changer. Si le ratio filtre
                             s'y ecarte du brut, le garde mord a tort et
                             mange du deplacement reel — c'est un echec.
    rotation     (> seuil) : c'est la que le brut explosait. Le ratio filtre
                             doit y retomber vers 1.

LE COUT, mesure explicitement : le garde gele la position pendant les
rotations, donc une translation reelle effectuee EN TOURNANT (virage large)
est perdue. La distance gelee est comptee et affichee — c'est la contrepartie
assumee, pas un detail a cacher.

Usage :  python3 tools/vins_guard_validate.py [duree_s]     (defaut 180)
         ROULEZ pendant la mesure, en ALTERNANT lignes droites et pivots :
         sans les deux regimes la comparaison n'a aucun sens.
"""
import sys
import time
import math

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

RAW_TOPIC   = "/ov_msckf/odomimu_raw"
OUT_TOPIC   = "/ov_msckf/odomimu"
WHEEL_TOPIC = "/wheel_odom_with_covariance"
IMU_TOPIC   = "/imu/data_clean"

WIN_S       = 2.0    # fenetre glissante
MIN_WHEEL_M = 0.05   # sous ce deplacement roues, le ratio n'a pas de sens
# Seuil LU sur le garde-fou lui-meme, jamais recopie en dur : il a ete
# reajuste en direct (0.25 -> 0.10) et une copie figee reclasserait les
# virages en "ligne droite", faussant tout le verdict.
GATE_FALLBACK = 0.10
GATE = GATE_FALLBACK  # remplace au demarrage par la valeur du serveur de params


class Validator(object):
    def __init__(self, duration):
        self.duration = duration
        self.raw, self.out, self.wheel, self.imu = [], [], [], []
        self.rot_win, self.straight_win = [], []
        rospy.Subscriber(RAW_TOPIC, Odometry, self._cb(self.raw), queue_size=300)
        rospy.Subscriber(OUT_TOPIC, Odometry, self._cb(self.out), queue_size=300)
        rospy.Subscriber(WHEEL_TOPIC, Odometry, self._cb(self.wheel), queue_size=300)
        rospy.Subscriber(IMU_TOPIC, Imu, self._cb_imu, queue_size=500)

    @staticmethod
    def _cb(store):
        def f(m):
            p = m.pose.pose.position
            store.append((time.time(), p.x, p.y, p.z))
        return f

    def _cb_imu(self, m):
        g = m.angular_velocity
        self.imu.append((time.time(),
                         math.sqrt(g.x ** 2 + g.y ** 2 + g.z ** 2)))

    @staticmethod
    def _path(seq):
        return sum(math.dist(seq[i][1:], seq[i - 1][1:])
                   for i in range(1, len(seq)))

    @staticmethod
    def _win(seq, t0, t1):
        return [s for s in seq if t0 <= s[0] <= t1]

    def run(self):
        t_start = time.time()
        last = t_start
        while time.time() - t_start < self.duration and not rospy.is_shutdown():
            time.sleep(0.5)
            now = time.time()
            wr = self._win(self.raw, now - WIN_S, now)
            wo = self._win(self.out, now - WIN_S, now)
            ww = self._win(self.wheel, now - WIN_S, now)
            wi = self._win(self.imu, now - WIN_S, now)
            if len(wr) < 10 or len(wo) < 10 or len(ww) < 5 or not wi:
                continue
            d_w = self._path(ww)
            if d_w < MIN_WHEEL_M:
                continue
            d_r, d_o = self._path(wr), self._path(wo)
            gyro = max(s[1] for s in wi)
            rec = (d_r / d_w, d_o / d_w, gyro, d_w)
            (self.rot_win if gyro > GATE else self.straight_win).append(rec)
            if now - last >= 10.0:
                last = now
                regime = "ROTATION " if gyro > GATE else "d.droite "
                print("    %s ratio brut=%8.2f  filtre=%6.2f  gyro=%.2f rad/s"
                      % (regime, rec[0], rec[1], gyro), flush=True)
        self.report()

    @staticmethod
    def _med(v):
        return sorted(v)[len(v) // 2] if v else float("nan")

    @staticmethod
    def _p90(v):
        """90e centile. La mediane ment ici : sur la mesure de 21h16 elle
        valait 0.69 en ligne droite alors que le pire cas du meme groupe
        etait a 19.96. Ce qui ruine le trace, c'est la queue."""
        if not v:
            return float("nan")
        w = sorted(v)
        return w[min(len(w) - 1, int(0.9 * len(w)))]

    def report(self):
        print(flush=True)
        print("  " + "=" * 64, flush=True)
        if not self.raw or not self.out:
            print("  Pas de donnees VINS — estimateur non initialise.", flush=True)
            print("  " + "=" * 64, flush=True)
            return
        print("  VALIDATION DU GARDE-FOU — ratio distance / distance roues", flush=True)
        print("  (1.0 = suivi parfait ; les roues se trompent de ~30 % en absolu)",
              flush=True)
        print(flush=True)
        for label, wins in (("LIGNE DROITE (gyro < %.2f)" % GATE, self.straight_win),
                            ("ROTATION     (gyro > %.2f)" % GATE, self.rot_win)):
            print("  %s  — %d fenetres" % (label, len(wins)), flush=True)
            if not wins:
                print("      aucune donnee : ce regime n'a pas ete parcouru",
                      flush=True)
                continue
            mr, mo = self._med([w[0] for w in wins]), self._med([w[1] for w in wins])
            qr, qo = self._p90([w[0] for w in wins]), self._p90([w[1] for w in wins])
            print("      ratio median  brut %8.2f   ->  filtre %6.2f" % (mr, mo),
                  flush=True)
            print("      90e centile   brut %8.2f   ->  filtre %6.2f   <= ce qui compte"
                  % (qr, qo), flush=True)
            print("      pire ratio    brut %8.2f   ->  filtre %6.2f"
                  % (max(w[0] for w in wins), max(w[1] for w in wins)), flush=True)
        print(flush=True)

        # Verdict. Deux conditions INDEPENDANTES : le garde doit corriger la
        # rotation ET ne pas abimer la ligne droite. Reussir l'une en ratant
        # l'autre n'est pas un succes.
        ok_rot = ok_str = None
        if self.rot_win:
            mr = self._p90([w[0] for w in self.rot_win])
            mo = self._p90([w[1] for w in self.rot_win])
            ok_rot = mo < 3.0
            print("  ROTATION : %s (p90 %.2f -> %.2f)"
                  % ("CORRIGE" if ok_rot else "NON CORRIGE", mr, mo), flush=True)
        if self.straight_win:
            mr = self._p90([w[0] for w in self.straight_win])
            mo = self._p90([w[1] for w in self.straight_win])
            ok_str = mo < 3.0 and mo <= mr * 1.05
            print("  LIGNE DROITE : %s (p90 %.2f -> %.2f)"
                  % ("INTACTE" if ok_str else "DEGRADEE — le garde mord a tort",
                     mr, mo), flush=True)
        print(flush=True)

        # Le cout : distance que le garde a retiree du trace.
        if self.raw and self.out:
            lost = self._path(self.raw) - self._path(self.out)
            d_wheel = self._path(self.wheel) if len(self.wheel) > 1 else 0.0
            print("  COUT — distance retiree du trace : %.2f m" % lost, flush=True)
            print("         (roues sur la meme periode : %.2f m)" % d_wheel,
                  flush=True)
            print("         Une partie est du deplacement REEL effectue en", flush=True)
            print("         tournant : c'est la contrepartie assumee du gel.", flush=True)
        print(flush=True)
        if ok_rot and ok_str:
            print("  -> Garde-fou VALIDE. Reste un pansement : le correctif de",
                  flush=True)
            print("     fond est la calibration Kalibr IMU-camera.", flush=True)
        elif ok_rot is None or ok_str is None:
            print("  -> INCOMPLET : un des deux regimes n'a pas ete parcouru.",
                  flush=True)
            print("     Refaites la mesure en alternant droites et pivots.", flush=True)
        else:
            print("  -> A REGLER : ajuster gate_rad_s / max_speed dans",
                  flush=True)
            print("     navigation_supervision.launch selon le regime fautif.",
                  flush=True)
        print("  " + "=" * 64, flush=True)


if __name__ == "__main__":
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    rospy.init_node("vins_guard_validate", anonymous=True, disable_signals=True)
    GATE = float(rospy.get_param("/vins_rotation_guard/gate_rad_s", GATE_FALLBACK))
    print("  seuil du garde-fou lu : %.2f rad/s (%.0f deg/s)"
          % (GATE, math.degrees(GATE)), flush=True)
    print("  Validation du garde-fou — %.0f s." % dur, flush=True)
    print("  ROULEZ EN ALTERNANT lignes droites ET pivots sur place.", flush=True)
    print("  (sans les deux regimes la comparaison ne prouve rien)", flush=True)
    Validator(dur).run()

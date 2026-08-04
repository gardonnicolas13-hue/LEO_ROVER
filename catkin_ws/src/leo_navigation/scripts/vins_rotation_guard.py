#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vins_rotation_guard.py — garde-fou anti-divergence pour openVINS.

POURQUOI (mesure du 2026-07-28, tools/vins_divergence_probe.py) :
openVINS suit CORRECTEMENT la reference roues en translation — ratio mesure
0.92 a 1.36, soit un suivi quasi parfait. Il decroche EXCLUSIVEMENT en
rotation :
    t+172 s  ratio  1.36   rotation 0.05 rad/s   (ligne droite)
    t+182 s  ratio  0.92   rotation 0.06 rad/s   (ligne droite)
    t+185 s  ratio 30.2    rotation 0.64 rad/s   (pivot)  <-- decrochage
    rotation mediane pendant les decrochages : 0.60 rad/s
    rotation mediane globale                 : 0.00 rad/s
C'est la signature d'un bras de levier camera-IMU (p_CinI) faux : en
translation pure une erreur de position de la camera n'a presque pas d'effet ;
en rotation elle engendre un deplacement fantome proportionnel a l'erreur.
Le vrai correctif reste la calibration Kalibr IMU-camera (item 2 du registre
du rapport, jamais faite). Ce noeud est un PANSEMENT, pas une correction.

CE QU'IL FAIT — et surtout ce qu'il NE fait PAS :
On ne gele PAS toute l'estimation. Geler l'orientation ferait perdre le
changement de cap pendant le pivot : a la reprise le filtre se croirait
oriente comme avant, et TOUT le deplacement suivant partirait de travers —
un remede pire que le mal. On gele donc UNIQUEMENT LA POSITION, et on laisse
passer l'orientation, qui est fiable (biais gyro ramene a 0.016 deg/s par
imu_sanitizer). Pendant un pivot sur place le robot ne se translate quasiment
pas : figer la position y est physiquement legitime.

CONTINUITE : la position de sortie ne saute jamais. Un offset accumule
absorbe ce que l'entree a derive pendant le gel, puis reste constant.
    sortie = entree - offset
    pendant le gel : offset += (entree - entree_precedente)   => sortie figee
    hors gel       : offset inchange                          => sortie suit

LIMITE ASSUMEE : si le robot translate REELLEMENT en tournant (virage large
et rapide), ce deplacement-la est perdu. Le seuil est donc choisi au-dessus
des virages doux et en dessous des pivots.

PARAMS
  ~in_topic   (defaut /ov_msckf/odomimu_raw)   sortie brute d'openVINS
  ~out_topic  (defaut /ov_msckf/odomimu)       ce que pose_selector consomme
  ~imu_topic  (defaut /imu/data_clean)
  ~gate_rad_s (defaut 0.25) seuil de rotation declenchant le gel
  ~release_s  (defaut 0.4)  temps sous le seuil avant de degeler (anti-bascule)
"""
import math

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class RotationGuard(object):
    def __init__(self):
        self.gate = float(rospy.get_param("~gate_rad_s", 0.25))
        self.release = float(rospy.get_param("~release_s", 0.4))
        # Plafond physique du rover (~1 m/s) + marge : au-dela, l'increment
        # est un saut de correction du filtre, pas un deplacement reel.
        self.max_speed = float(rospy.get_param("~max_speed", 2.0))
        out_topic = rospy.get_param("~out_topic", "/ov_msckf/odomimu")
        in_topic = rospy.get_param("~in_topic", "/ov_msckf/odomimu_raw")
        imu_topic = rospy.get_param("~imu_topic", "/imu/data_clean")

        self.offset = [0.0, 0.0, 0.0]
        self.prev_in = None
        self.frozen = False
        self.t_below = None          # depuis quand sous le seuil
        self.n_freeze = 0            # nombre d'episodes de gel
        self.n_jump = 0              # increments rejetes pour vitesse absurde
        self.prev_t = 0.0
        self.t_frozen_total = 0.0
        self.t_freeze_start = None
        self._last_report = rospy.Time.now()

        self.pub = rospy.Publisher(out_topic, Odometry, queue_size=50)
        rospy.Subscriber(imu_topic, Imu, self._on_imu, queue_size=200)
        rospy.Subscriber(in_topic, Odometry, self._on_odom, queue_size=200)
        rospy.logwarn("[vins_guard] %s -> %s | seuil %.2f rad/s (%.0f deg/s), "
                      "relache %.2f s. Gele la POSITION en rotation, laisse "
                      "passer l'orientation.",
                      in_topic, out_topic, self.gate,
                      math.degrees(self.gate), self.release)

    def _on_imu(self, m):
        g = m.angular_velocity
        w = math.sqrt(g.x ** 2 + g.y ** 2 + g.z ** 2)
        now = rospy.Time.now()
        if w > self.gate:
            self.t_below = None
            if not self.frozen:
                self.frozen = True
                self.n_freeze += 1
                self.t_freeze_start = now
        else:
            # Hysteresis temporelle : on ne degele qu'apres release_s sous le
            # seuil, sinon le bruit gyro ferait basculer l'etat en permanence.
            if self.t_below is None:
                self.t_below = now
            elif self.frozen and (now - self.t_below).to_sec() >= self.release:
                self.frozen = False
                if self.t_freeze_start is not None:
                    self.t_frozen_total += (now - self.t_freeze_start).to_sec()
                    self.t_freeze_start = None

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        cur = [p.x, p.y, p.z]
        t = msg.header.stamp.to_sec()
        if self.prev_in is None:
            self.prev_in, self.prev_t = cur, t

        # ── Critere 2 : VITESSE IMPLAUSIBLE (ajoute sur retour operateur) ──
        # Le seuil gyro seul est AVEUGLE : il ne consulte jamais ce que la
        # camera estime reellement. Ici on juge le mouvement ESTIME lui-meme
        # (issu de la fusion camera+IMU d'openVINS) : un increment impliquant
        # une vitesse superieure a max_speed est physiquement impossible pour
        # ce rover (plafond ~1 m/s, cf. Velocity Limits du cockpit), donc
        # c'est un saut de correction du filtre, pas un deplacement.
        # Avantage sur le seuil gyro : attrape l'anomalie REELLE quelle qu'en
        # soit la cause, et laisse passer une rotation accompagnee d'une
        # translation legitime (virage large) au lieu de la geler a tort.
        dt = t - self.prev_t
        jump = False
        if dt > 1e-6:
            v = math.dist(cur, self.prev_in) / dt
            if v > self.max_speed:
                jump = True
                self.n_jump += 1

        if self.frozen or jump:
            # Tout ce que l'entree derive pendant le gel part dans l'offset :
            # la sortie reste donc strictement immobile, sans discontinuite.
            for i in range(3):
                self.offset[i] += cur[i] - self.prev_in[i]
        self.prev_in, self.prev_t = cur, t

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = cur[0] - self.offset[0]
        out.pose.pose.position.y = cur[1] - self.offset[1]
        out.pose.pose.position.z = cur[2] - self.offset[2]
        # Orientation et twist inchanges : le cap doit continuer de vivre
        # pendant le gel, c'est tout l'interet du dispositif.
        out.pose.pose.orientation = msg.pose.pose.orientation
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist
        self.pub.publish(out)

        now = rospy.Time.now()
        if (now - self._last_report).to_sec() >= 30.0:
            self._last_report = now
            rospy.loginfo("[vins_guard] %d gels (%.1f s), %d sauts rejetes, "
                          "offset accumule %.2f m",
                          self.n_freeze, self.t_frozen_total, self.n_jump,
                          math.sqrt(sum(o * o for o in self.offset)))


if __name__ == "__main__":
    rospy.init_node("vins_rotation_guard")
    RotationGuard()
    rospy.spin()

#!/usr/bin/env python3
"""
LEO Rover — Machine à états multi-cibles (QR code OU lumière bleue)
==================================================================
Caméra : Intel RealSense D455 (flux couleur /camera/color/image_raw).

Déclencheurs :
  - Source A : QR code 'qrleoimage' (pyzbar)
  - Source B : source de lumière bleue (seuillage HSV OpenCV)

Séquence exécutée dès qu'une cible est détectée :
  1. VALIDATION  : avancer 1 s puis reculer 1 s
  2. SCAN        : tour complet sur soi (360°)
  3. APPROCHE    : se rapprocher de la cible pendant 20 s (asservissement P
                   sur l'écart horizontal ; ne se bloque PAS si la cible
                   disparaît — on continue tout droit, le timer fait foi)
  4. RESET       : rotation de 180°
  5. retour en mode RECHERCHE

Robustesse : chaque état possède une sortie temporelle (ou odométrique) ;
aucun état ne dépend de la présence continue de la cible. rospy.is_shutdown()
géré proprement, arrêt du robot garanti à l'extinction.

Usage : python3 leo_target_behavior.py
"""

import math
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import cv2
import numpy as np


def imgmsg_to_bgr(msg):
    """Image ROS -> BGR numpy SANS cv_bridge (cassé sur ce Pi).

    Le driver realsense publie en 'rgb8' par défaut ; on gère aussi 'bgr8'
    et 'mono8'. On renvoie toujours du BGR (standard OpenCV) pour que le
    reste du pipeline (BGR2HSV, BGR2GRAY) fonctionne sans changement.
    """
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if enc == "rgb8" else arr
    if enc == "mono8":
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"encodage non géré : {msg.encoding}")

try:
    from pyzbar import pyzbar
    _HAVE_PYZBAR = True
except Exception:
    _HAVE_PYZBAR = False

# ── Paramètres comportementaux ────────────────────────────────────────────────
TARGET_QR     = "qrleoimage"

SEARCH_SPEED  = 0.4     # rad/s — rotation lente de recherche
DRIVE_SPEED   = 0.2     # m/s   — marche avant/arrière (validation + approche)
ROT_SPEED     = 0.6     # rad/s — vitesse des rotations 360°/180°
APPROACH_KP   = 1.2     # gain P de centrage horizontal pendant l'approche

VALIDATE_T    = 1.0     # s — durée avance, puis durée recul
APPROACH_T    = 20.0    # s — durée de l'approche
RATE_HZ       = 20

IMG_TIMEOUT   = 1.0     # s — watchdog flux caméra

# Seuillage bleu (HSV OpenCV : H 0-179). Bleu ~ 100-130.
BLUE_LOW      = np.array([100,  80,  60])
BLUE_HIGH     = np.array([130, 255, 255])
BLUE_MIN_AREA = 400     # px² — surface minimale pour valider une tache bleue

# ── États ─────────────────────────────────────────────────────────────────────
SEARCH, VALIDATE_FWD, VALIDATE_BACK, SCAN, APPROACH, RESET = (
    "SEARCH", "VALIDATE_FWD", "VALIDATE_BACK", "SCAN", "APPROACH", "RESET")


class TargetBehavior:
    def __init__(self):
        rospy.init_node("leo_target_behavior", anonymous=False)

        self.image_topic = rospy.get_param("~image_topic",
                                            "/camera/color/image_raw")
        self.odom_topic  = rospy.get_param("~odom_topic",
                                            "/wheel_odom_with_covariance")

        # Détection (mise à jour par le callback image)
        self.det_kind   = None          # 'QR' | 'BLUE' | None
        self.det_offset = 0.0           # écart horizontal normalisé [-1, 1]
        self.last_img_t = rospy.Time(0)
        self.last_det_t = rospy.Time(0)

        # Odométrie (pour 360°/180°)
        self.yaw = None

        # Machine à états
        self.state       = SEARCH
        self.phase_start = rospy.Time.now()
        self.locked_kind = None         # cible verrouillée pour la séquence
        self.rot_target  = 0.0          # angle restant à parcourir (rotations)
        self.rot_last_yaw = None

        # I/O
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self._on_image,
                         queue_size=1, buff_size=2**24)
        rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=10)

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"[Behavior] image : {self.image_topic}")
        rospy.loginfo(f"[Behavior] odom  : {self.odom_topic}")
        rospy.loginfo(f"[Behavior] pyzbar : {_HAVE_PYZBAR}  (QR='{TARGET_QR}')")
        rospy.loginfo("[Behavior] état initial : RECHERCHE")
        rospy.loginfo("=" * 60)

    # ══════════════════════════════════════════════════════════════════════ #
    # Callbacks
    # ══════════════════════════════════════════════════════════════════════ #
    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def _on_image(self, msg):
        now = rospy.Time.now()
        self.last_img_t = now
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as e:
            rospy.logwarn_throttle(5, f"[Behavior] conversion image : {e}")
            return

        kind, offset = self._detect(bgr)
        if kind is not None:
            self.det_kind, self.det_offset, self.last_det_t = kind, offset, now

    # ══════════════════════════════════════════════════════════════════════ #
    # Détection unifiée : retourne (type, offset_horizontal_normalisé)
    # ══════════════════════════════════════════════════════════════════════ #
    def _detect(self, bgr):
        h, w = bgr.shape[:2]
        cx_img = w / 2.0

        # --- Source A : QR code (prioritaire) ---
        if _HAVE_PYZBAR:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            for o in pyzbar.decode(gray):
                if o.type == "QRCODE" and \
                        o.data.decode("utf-8", "replace") == TARGET_QR:
                    cx = np.mean([p.x for p in o.polygon]) if o.polygon \
                         else o.rect.left + o.rect.width / 2.0
                    return "QR", (cx - cx_img) / cx_img

        # --- Source B : lumière/tache bleue ---
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)
        mask = cv2.medianBlur(mask, 5)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) >= BLUE_MIN_AREA:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = M["m10"] / M["m00"]
                    return "BLUE", (cx - cx_img) / cx_img

        return None, 0.0

    # ══════════════════════════════════════════════════════════════════════ #
    # Helpers
    # ══════════════════════════════════════════════════════════════════════ #
    def _img_alive(self, now):
        return (self.last_img_t != rospy.Time(0) and
                (now - self.last_img_t).to_sec() < IMG_TIMEOUT)

    def _target_fresh(self, now):
        """Cible vue très récemment (utile pour l'asservissement d'approche)."""
        return (self.last_det_t != rospy.Time(0) and
                (now - self.last_det_t).to_sec() < 0.4)

    def _pub(self, lin=0.0, ang=0.0):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = lin, ang
        self.cmd_pub.publish(cmd)

    def _elapsed(self, now):
        return (now - self.phase_start).to_sec()

    def _enter(self, state, now):
        self.state = state
        self.phase_start = now

    # --- Gestion des rotations sur angle (odométrie, repli chronométré) ---
    def _begin_rotation(self, angle_rad, now):
        self.rot_target   = abs(angle_rad)
        self.rot_done     = 0.0
        self.rot_last_yaw = self.yaw
        self.phase_start  = now

    def _rotation_complete(self, now):
        if self.yaw is not None and self.rot_last_yaw is not None:
            d = self.yaw - self.rot_last_yaw
            d = math.atan2(math.sin(d), math.cos(d))
            self.rot_done += abs(d)
            self.rot_last_yaw = self.yaw
            if self.rot_done >= self.rot_target:
                return True
        # Sécurité chronométrique (1.5x temps théorique) — évite tout blocage
        max_t = 1.5 * (self.rot_target / ROT_SPEED)
        return self._elapsed(now) > max_t

    # ══════════════════════════════════════════════════════════════════════ #
    # Boucle principale (machine à états)
    # ══════════════════════════════════════════════════════════════════════ #
    def run(self):
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # ---------- RECHERCHE ----------
            if self.state == SEARCH:
                if not self._img_alive(now):
                    self._pub(0, 0)
                    rospy.logwarn_throttle(
                        2, "[Behavior] pas de flux caméra -> attente "
                           "(vérifiez la RealSense)")
                elif self._target_fresh(now):
                    self.locked_kind = self.det_kind
                    rospy.loginfo(f"[Behavior] CIBLE '{self.locked_kind}' "
                                  f"détectée -> VALIDATION (avance 1 s)")
                    self._enter(VALIDATE_FWD, now)
                    self._pub(0, 0)
                else:
                    self._pub(0, SEARCH_SPEED)      # rotation lente de recherche

            # ---------- VALIDATION : avance 1 s ----------
            elif self.state == VALIDATE_FWD:
                if self._elapsed(now) < VALIDATE_T:
                    self._pub(DRIVE_SPEED, 0)
                else:
                    rospy.loginfo("[Behavior] VALIDATION : recul 1 s")
                    self._enter(VALIDATE_BACK, now)

            # ---------- VALIDATION : recul 1 s ----------
            elif self.state == VALIDATE_BACK:
                if self._elapsed(now) < VALIDATE_T:
                    self._pub(-DRIVE_SPEED, 0)
                else:
                    rospy.loginfo("[Behavior] SCAN : tour complet 360°")
                    self._begin_rotation(2 * math.pi, now)
                    self.state = SCAN

            # ---------- SCAN : 360° ----------
            elif self.state == SCAN:
                if self._rotation_complete(now):
                    rospy.loginfo("[Behavior] APPROCHE : 20 s vers la cible")
                    self._enter(APPROACH, now)
                else:
                    self._pub(0, ROT_SPEED)

            # ---------- APPROCHE : 20 s ----------
            elif self.state == APPROACH:
                if self._elapsed(now) >= APPROACH_T:
                    rospy.loginfo("[Behavior] RESET : rotation 180°")
                    self._begin_rotation(math.pi, now)
                    self.state = RESET
                else:
                    # Asservissement : on centre la cible si elle est fraîche,
                    # sinon on avance tout droit (PAS de blocage).
                    if self._target_fresh(now):
                        ang = -APPROACH_KP * self.det_offset
                        ang = max(-ROT_SPEED, min(ROT_SPEED, ang))
                        self._pub(DRIVE_SPEED, ang)
                        rospy.loginfo_throttle(
                            2, f"[Behavior] approche cible '{self.locked_kind}' "
                               f"offset={self.det_offset:+.2f}")
                    else:
                        self._pub(DRIVE_SPEED, 0)
                        rospy.loginfo_throttle(
                            2, "[Behavior] cible perdue -> avance tout droit")

            # ---------- RESET : 180° ----------
            elif self.state == RESET:
                if self._rotation_complete(now):
                    rospy.loginfo("[Behavior] séquence terminée -> RECHERCHE")
                    self.det_kind = None
                    self.last_det_t = rospy.Time(0)   # évite re-trigger immédiat
                    self.locked_kind = None
                    self._enter(SEARCH, now)
                else:
                    self._pub(0, ROT_SPEED)

            rate.sleep()

        # Arrêt propre garanti
        self._pub(0, 0)
        rospy.loginfo("[Behavior] arrêt propre.")


if __name__ == "__main__":
    try:
        TargetBehavior().run()
    except rospy.ROSInterruptException:
        pass

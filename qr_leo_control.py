#!/usr/bin/env python3
"""
QR Control — LEO Rover (ROS 1 Noetic)
=====================================
Comportement :
  - Détection CONTINUE du QR code 'qrleoimage' pendant >= CONFIRM_TIME secondes.
  - Si le QR disparaît avant la fin du compte à rebours -> compteur remis à zéro.
  - Une fois confirmé -> rotation de 360° (un tour complet du châssis).
  - La caméra étant FIXE sur le châssis, "tourner la tête" = pivoter tout le robot.

Décodage : pyzbar (vérifie le CONTENU du QR). Repli cv2 si libzbar0 absent
           (détecte la présence sans vérifier le contenu — avertissement).
Image    : cv_bridge si disponible, sinon conversion manuelle (portabilité).
Rotation : boucle fermée sur l'odométrie si dispo, sinon chronométrée.

Usage :
    python3 qr_leo_control.py                    # topic image auto-détecté
    python3 qr_leo_control.py /camera/image_raw  # topic image forcé
"""

import sys
import math
import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── Paramètres comportementaux ────────────────────────────────────────────────
TARGET_QR    = "qrleoimage"   # contenu attendu du QR code
CONFIRM_TIME = 10.0           # secondes de détection continue requises
ANGULAR_SPD  = 0.5            # rad/s — vitesse de rotation du 360°
RATE_HZ      = 20             # fréquence de la boucle de contrôle
GRACE_S      = 0.5            # tolérance aux trames perdues (voir note ci-dessous)
IMG_TIMEOUT  = 1.5            # watchdog : arrêt si plus d'image depuis N s

# Note GRACE_S : une caméra perd parfois une trame isolée. Plutôt que de remettre
# le compteur à zéro au moindre trou (fragile en extérieur), on considère le QR
# "perdu" seulement s'il n'est pas vu pendant plus de GRACE_S. Mettre 0.0 pour un
# comportement strict (reset à la moindre trame sans QR).

# ── Décodeur QR : pyzbar prioritaire, repli cv2 ───────────────────────────────
try:
    from pyzbar import pyzbar
    _HAVE_PYZBAR = True
except Exception:
    _HAVE_PYZBAR = False

import cv2

# cv_bridge prioritaire, repli conversion manuelle
try:
    from cv_bridge import CvBridge
    _BRIDGE = CvBridge()
    _HAVE_BRIDGE = True
except Exception:
    _BRIDGE = None
    _HAVE_BRIDGE = False

import numpy as np

_ENC = {
    "rgb8":  ("uint8",  3, cv2.COLOR_RGB2GRAY),
    "bgr8":  ("uint8",  3, cv2.COLOR_BGR2GRAY),
    "mono8": ("uint8",  1, None),
}


def compressed_to_gray(msg):
    """CompressedImage (JPEG) -> niveaux de gris numpy via cv2.imdecode."""
    arr = np.frombuffer(msg.data, np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)


def image_to_gray(msg):
    """Image brute ROS -> niveaux de gris (cv_bridge si dispo, sinon manuel)."""
    if _HAVE_BRIDGE:
        bgr = _BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    dtype, ch, cvt = _ENC.get(msg.encoding.lower(), ("uint8", 3, cv2.COLOR_BGR2GRAY))
    arr = np.frombuffer(msg.data, dtype=np.dtype(dtype))
    arr = arr.reshape(msg.height, msg.width, ch) if ch > 1 \
          else arr.reshape(msg.height, msg.width)
    return cv2.cvtColor(arr, cvt) if cvt is not None else arr


def qr_is_target(gray):
    """True si le QR cible est présent dans l'image."""
    if _HAVE_PYZBAR:
        for obj in pyzbar.decode(gray):
            if obj.type == "QRCODE" and obj.data.decode("utf-8") == TARGET_QR:
                return True
        return False
    # Repli : cv2 détecte la présence d'un QR mais ne vérifie pas le contenu
    _, pts, _ = cv2.QRCodeDetector().detectAndDecode(gray)
    return pts is not None


# ── États de la machine ───────────────────────────────────────────────────────
SEARCHING = "SEARCHING"   # aucun QR, robot immobile
DETECTING = "DETECTING"   # QR vu, comptage en cours, robot immobile
ROTATING  = "ROTATING"    # exécution du 360°


class QRControlNode:

    # Le Leo (raspicam_node) ne publie QUE le compressé -> on le préfère.
    CANDIDATE_TOPICS = ["/camera/image_raw/compressed",
                        "/camera/color/image_raw/compressed",
                        "/camera/image_raw",
                        "/camera/color/image_raw"]

    def __init__(self, image_topic=None):
        rospy.init_node("qr_leo_control", anonymous=False)

        self.topic      = image_topic or self._detect_topic()
        self.odom_topic = rospy.get_param("~odom_topic",
                                          "/wheel_odom_with_covariance")

        # État détection
        self.last_qr_time = rospy.Time(0)   # dernière trame AVEC le QR cible
        self.last_img_time = rospy.Time(0)  # dernière image reçue (watchdog)

        # État rotation
        self.state          = SEARCHING
        self.detect_start   = None
        self.yaw            = None          # cap courant (odométrie)
        self.rot_start_yaw  = None
        self.rot_accum      = 0.0
        self.rot_last_yaw   = None
        self.rot_start_time = None

        # I/O — flux compressé (CompressedImage) ou brut (Image) selon le topic
        self.compressed = self.topic.endswith("/compressed")
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        if self.compressed:
            rospy.Subscriber(self.topic, CompressedImage, self._on_image,
                             queue_size=1, buff_size=2**24)
        else:
            rospy.Subscriber(self.topic, Image, self._on_image,
                             queue_size=1, buff_size=2**24)
        rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=10)

        rospy.loginfo("=" * 55)
        rospy.loginfo(f"[QRControl] image  : {self.topic}")
        rospy.loginfo(f"[QRControl] odom   : {self.odom_topic}")
        rospy.loginfo(f"[QRControl] cible  : '{TARGET_QR}'  "
                      f"confirmation : {CONFIRM_TIME:.0f}s")
        rospy.loginfo(f"[QRControl] décodeur : "
                      f"{'pyzbar (contenu vérifié)' if _HAVE_PYZBAR else 'cv2 (PRÉSENCE SEULE)'}")
        if not _HAVE_PYZBAR:
            rospy.logwarn("[QRControl] pyzbar absent : le contenu du QR n'est PAS "
                          "vérifié. Installez : sudo apt install libzbar0 ; "
                          "pip3 install pyzbar")
        rospy.loginfo("=" * 55)

    # ── détection du topic caméra ──
    def _detect_topic(self):
        rospy.loginfo("[QRControl] recherche du topic caméra...")
        deadline = rospy.Time.now() + rospy.Duration(5)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            names = {t for t, _ in rospy.get_published_topics()}
            for c in self.CANDIDATE_TOPICS:
                if c in names:
                    rospy.loginfo(f"[QRControl] topic trouvé : {c}")
                    return c
            rospy.sleep(0.5)
        return self.CANDIDATE_TOPICS[0]

    # ── callbacks ──
    def _on_image(self, msg):
        now = rospy.Time.now()
        self.last_img_time = now
        try:
            gray = compressed_to_gray(msg) if self.compressed \
                   else image_to_gray(msg)
        except Exception as e:
            rospy.logwarn_throttle(5, f"[QRControl] conversion image : {e}")
            return
        if gray is not None and qr_is_target(gray):
            self.last_qr_time = now

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        # yaw depuis le quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    # ── helpers ──
    def _qr_present(self, now):
        return (self.last_qr_time != rospy.Time(0) and
                (now - self.last_qr_time).to_sec() < GRACE_S)

    def _img_alive(self, now):
        return (self.last_img_time != rospy.Time(0) and
                (now - self.last_img_time).to_sec() < IMG_TIMEOUT)

    def _publish(self, wz):
        cmd = Twist()
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    # ── transitions de la machine d'état ──
    def _start_rotation(self):
        self.state          = ROTATING
        self.rot_start_yaw  = self.yaw
        self.rot_last_yaw   = self.yaw
        self.rot_accum      = 0.0
        self.rot_start_time = rospy.Time.now()
        mode = "odométrie" if self.yaw is not None else "chronométré"
        rospy.loginfo(f"[QRControl] >>> Confirmation après {CONFIRM_TIME:.0f}s")
        rospy.loginfo(f"[QRControl] >>> Exécution du 360 ({mode})")

    def _update_rotation(self, now):
        """Retourne True tant que la rotation continue."""
        # Sécurité chronométrique : 1.5x le temps théorique d'un tour
        max_dur = 1.5 * (2 * math.pi / ANGULAR_SPD)
        if (now - self.rot_start_time).to_sec() > max_dur:
            rospy.logwarn("[QRControl] timeout rotation -> arrêt")
            return False

        if self.yaw is not None:
            # Boucle fermée : on intègre la variation de cap (gestion du wrap)
            d = self.yaw - self.rot_last_yaw
            d = math.atan2(math.sin(d), math.cos(d))   # normalise [-pi, pi]
            self.rot_accum += abs(d)
            self.rot_last_yaw = self.yaw
            rospy.loginfo_throttle(
                1.0, f"[QRControl] rotation : "
                     f"{math.degrees(self.rot_accum):.0f}° / 360°")
            if self.rot_accum >= 2 * math.pi:
                return False
        # (si pas d'odométrie, seule la sécurité chronométrique arrête le tour)
        return True

    # ── boucle principale ──
    def run(self):
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            present = self._qr_present(now)

            if self.state == ROTATING:
                if self._update_rotation(now):
                    self._publish(ANGULAR_SPD)
                else:
                    self._publish(0.0)
                    rospy.loginfo("[QRControl] Rotation 360 terminée.")
                    self.state = SEARCHING
                    self.detect_start = None
                    self.last_qr_time = rospy.Time(0)   # évite un re-trigger immédiat

            elif self.state == DETECTING:
                if not present:
                    rospy.loginfo("[QRControl] QR perdu -> compteur réinitialisé.")
                    self.state = SEARCHING
                    self.detect_start = None
                elif (now - self.detect_start).to_sec() >= CONFIRM_TIME:
                    self._start_rotation()
                else:
                    elapsed = (now - self.detect_start).to_sec()
                    rospy.loginfo_throttle(
                        1.0, f"[QRControl] Détection en cours... "
                             f"{elapsed:.1f}/{CONFIRM_TIME:.0f}s")
                self._publish(0.0)   # immobile pendant la confirmation

            else:  # SEARCHING
                if present:
                    self.state = DETECTING
                    self.detect_start = now
                    rospy.loginfo("[QRControl] Détection en cours...")
                self._publish(0.0)

            rate.sleep()

        self._publish(0.0)   # arrêt propre
        rospy.loginfo("[QRControl] arrêt.")


if __name__ == "__main__":
    topic = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        QRControlNode(topic).run()
    except rospy.ROSInterruptException:
        pass

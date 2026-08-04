#!/usr/bin/env python3
"""
QR Search & Move — LEO Rover (ROS 1 Noetic)
===========================================
Comportement :
  1. RECHERCHE : le robot tourne sur lui-même en continu (360° permanent)
     tant qu'il ne voit pas le QR code 'qrleoimage'.
  2. À la détection : il s'arrête, avance 1 s puis recule 1 s.
  3. Séquence terminée -> arrêt.

Sécurité : si AUCUNE image n'arrive (caméra HS), le robot NE TOURNE PAS
           (rotation à l'aveugle interdite) et l'affiche clairement.

Flux caméra : compressé (raspicam_node) décodé par cv2.imdecode, ou brut.
Usage : python3 qr_search_move.py [topic]   (topic auto-détecté par défaut)
"""

import sys
import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist

# ── Paramètres ────────────────────────────────────────────────────────────────
TARGET_QR     = "qrleoimage"
SEARCH_SPEED  = 0.5    # rad/s — vitesse de rotation pendant la recherche
DRIVE_SPEED   = 0.2    # m/s   — vitesse marche avant / arrière
MOVE_TIME     = 1.0    # s     — durée de la marche avant ET de la marche arrière
RATE_HZ       = 20
IMG_TIMEOUT   = 1.0    # s     — watchdog : flux considéré mort au-delà

# ── Décodeur QR ───────────────────────────────────────────────────────────────
try:
    from pyzbar import pyzbar
    _HAVE_PYZBAR = True
except Exception:
    _HAVE_PYZBAR = False

import cv2
import numpy as np


def gray_from_compressed(msg):
    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)


def gray_from_raw(msg):
    arr = np.frombuffer(msg.data, np.uint8)
    if msg.encoding.lower() in ("rgb8", "bgr8"):
        return cv2.cvtColor(arr.reshape(msg.height, msg.width, 3),
                            cv2.COLOR_BGR2GRAY)
    return arr.reshape(msg.height, msg.width)


def qr_is_target(gray):
    if gray is None:
        return False
    if _HAVE_PYZBAR:
        return any(o.type == "QRCODE" and o.data.decode("utf-8", "replace") == TARGET_QR
                   for o in pyzbar.decode(gray))
    _, pts, _ = cv2.QRCodeDetector().detectAndDecode(gray)   # présence seule
    return pts is not None


# ── États ─────────────────────────────────────────────────────────────────────
SEARCHING = "SEARCHING"
FORWARD   = "FORWARD"
BACKWARD  = "BACKWARD"
DONE      = "DONE"


class QRSearchMove:

    CANDIDATE_TOPICS = ["/camera/image_raw/compressed",
                        "/camera/color/image_raw/compressed",
                        "/camera/image_raw",
                        "/camera/color/image_raw"]

    def __init__(self, topic=None):
        rospy.init_node("qr_search_move", anonymous=False)
        self.topic = topic or self._detect_topic()
        self.compressed = self.topic.endswith("/compressed")

        self.last_img_time = rospy.Time(0)
        self.last_qr_time  = rospy.Time(0)
        self.state = SEARCHING
        self.phase_start = None

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        msg_type = CompressedImage if self.compressed else Image
        rospy.Subscriber(self.topic, msg_type, self._on_image,
                         queue_size=1, buff_size=2**24)

        rospy.loginfo("=" * 55)
        rospy.loginfo(f"[Search&Move] image : {self.topic}")
        rospy.loginfo(f"[Search&Move] cible : '{TARGET_QR}'")
        rospy.loginfo(f"[Search&Move] décodeur : "
                      f"{'pyzbar' if _HAVE_PYZBAR else 'cv2 (présence seule)'}")
        rospy.loginfo("[Search&Move] RECHERCHE : rotation jusqu'à détection...")
        rospy.loginfo("=" * 55)

    def _detect_topic(self):
        rospy.loginfo("[Search&Move] recherche du topic caméra...")
        deadline = rospy.Time.now() + rospy.Duration(5)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            names = {t for t, _ in rospy.get_published_topics()}
            for c in self.CANDIDATE_TOPICS:
                if c in names:
                    rospy.loginfo(f"[Search&Move] topic trouvé : {c}")
                    return c
            rospy.sleep(0.5)
        return self.CANDIDATE_TOPICS[0]

    def _on_image(self, msg):
        now = rospy.Time.now()
        self.last_img_time = now
        try:
            gray = gray_from_compressed(msg) if self.compressed \
                   else gray_from_raw(msg)
        except Exception as e:
            rospy.logwarn_throttle(5, f"[Search&Move] conversion : {e}")
            return
        if qr_is_target(gray):
            self.last_qr_time = now

    def _img_alive(self, now):
        return (self.last_img_time != rospy.Time(0) and
                (now - self.last_img_time).to_sec() < IMG_TIMEOUT)

    def _qr_seen(self, now):
        return (self.last_qr_time != rospy.Time(0) and
                (now - self.last_qr_time).to_sec() < 0.5)

    def _pub(self, lin=0.0, ang=0.0):
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

    def run(self):
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self.state == SEARCHING:
                if not self._img_alive(now):
                    # Pas d'images -> on NE tourne PAS (sécurité)
                    self._pub(0.0, 0.0)
                    rospy.logwarn_throttle(
                        2.0, "[Search&Move] pas de flux caméra -> rotation "
                             "désactivée (vérifiez la caméra)")
                elif self._qr_seen(now):
                    rospy.loginfo("[Search&Move] QR DÉTECTÉ -> marche avant 1 s")
                    self.state = FORWARD
                    self.phase_start = now
                    self._pub(0.0, 0.0)
                else:
                    self._pub(0.0, SEARCH_SPEED)   # rotation de recherche

            elif self.state == FORWARD:
                if (now - self.phase_start).to_sec() < MOVE_TIME:
                    self._pub(DRIVE_SPEED, 0.0)
                else:
                    rospy.loginfo("[Search&Move] marche arrière 1 s")
                    self.state = BACKWARD
                    self.phase_start = now

            elif self.state == BACKWARD:
                if (now - self.phase_start).to_sec() < MOVE_TIME:
                    self._pub(-DRIVE_SPEED, 0.0)
                else:
                    rospy.loginfo("[Search&Move] séquence terminée -> arrêt.")
                    self.state = DONE
                    self._pub(0.0, 0.0)

            else:  # DONE
                self._pub(0.0, 0.0)

            rate.sleep()

        self._pub(0.0, 0.0)   # arrêt propre
        rospy.loginfo("[Search&Move] arrêt.")


if __name__ == "__main__":
    topic = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        QRSearchMove(topic).run()
    except rospy.ROSInterruptException:
        pass

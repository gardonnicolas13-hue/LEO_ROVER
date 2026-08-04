#!/usr/bin/env python3
"""
Diagnostic caméra + QR — LEO Rover
Gère les flux BRUTS (Image) ET COMPRESSÉS (CompressedImage).
Sur le Leo, raspicam_node ne publie QUE le compressé : /camera/image_raw/compressed
Usage : python3 diag_qr.py [topic]   (défaut : /camera/image_raw/compressed)
"""
import sys
import rospy
from sensor_msgs.msg import Image, CompressedImage

try:
    from pyzbar import pyzbar
    HAVE_PYZBAR = True
except Exception:
    HAVE_PYZBAR = False

import cv2
import numpy as np


class Diag:
    def __init__(self, topic):
        rospy.init_node("diag_qr")
        self.topic = topic
        self.count = 0
        self.last_codes = []
        self.cv2_present = False
        self.last_info = ""

        if topic.endswith("/compressed"):
            rospy.Subscriber(topic, CompressedImage, self.cb_compressed,
                             queue_size=1, buff_size=2**24)
            kind = "CompressedImage"
        else:
            rospy.Subscriber(topic, Image, self.cb_raw,
                             queue_size=1, buff_size=2**24)
            kind = "Image"
        rospy.loginfo(f"[DIAG] écoute {topic} ({kind}) — pyzbar={HAVE_PYZBAR}")

    def _analyse(self, gray):
        codes = []
        if HAVE_PYZBAR and gray is not None:
            for o in pyzbar.decode(gray):
                codes.append(f"{o.type}:'{o.data.decode('utf-8', 'replace')}'")
        self.last_codes = codes
        if gray is not None:
            _, pts, _ = cv2.QRCodeDetector().detectAndDecode(gray)
            self.cv2_present = pts is not None

    def cb_compressed(self, msg):
        self.count += 1
        arr = np.frombuffer(msg.data, np.uint8)
        gray = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        self.last_info = (f"{gray.shape[1]}x{gray.shape[0]} format={msg.format}"
                          if gray is not None else "DÉCODAGE ÉCHOUÉ")
        self._analyse(gray)

    def cb_raw(self, msg):
        self.count += 1
        self.last_info = f"{msg.width}x{msg.height} encoding={msg.encoding}"
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            if msg.encoding.lower() in ("rgb8", "bgr8"):
                arr = arr.reshape(msg.height, msg.width, 3)
                gray = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
            else:
                gray = arr.reshape(msg.height, msg.width)
            self._analyse(gray)
        except Exception as e:
            self.last_info += f"  ERREUR: {e}"

    def run(self):
        rate = rospy.Rate(1)
        prev = 0
        while not rospy.is_shutdown():
            fps = self.count - prev
            prev = self.count
            if self.count == 0:
                rospy.logwarn(f"[DIAG] AUCUNE image sur {self.topic} !")
            else:
                rospy.loginfo(
                    f"[DIAG] {fps} img/s | {self.last_info} | "
                    f"pyzbar={self.last_codes if self.last_codes else 'rien'} | "
                    f"cv2_QR={self.cv2_present}")
            rate.sleep()


if __name__ == "__main__":
    topic = sys.argv[1] if len(sys.argv) > 1 else "/camera/image_raw/compressed"
    Diag(topic).run()

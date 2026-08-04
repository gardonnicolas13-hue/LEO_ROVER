#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LEO Rover — NASA MISSION CONTROL (ROS Noetic + Tkinter)
=======================================================
Logiciel de contrôle autonome, version finale "NASA style". Deux classes :

  * TrackingMap        — cerveau ROS : caméra, détection de balise (CROIX de
                         4 LED + AprilTag), odométrie/carte, machine d'états de
                         recherche, /cmd_vel, RViz, sécurité moteur (_safe_stop),
                         état ARMED/DISARMED, journal de détection.
  * NASA_ControlPanel  — interface Tkinter rétro-futuriste : dark mode profond,
                         police monospace, flux caméra, télémétrie X/Y/Z @10 Hz,
                         badge "NASA MISSION CONTROL", indicateur ARMED/DISARMED,
                         log temps réel des détections, pilotage.

VISION (robuste aux reflets) :
  a) blobs HSV (teinte/luminosité réglables) ;
  b) barycentre du groupe ;
  c) vérification géométrique CROIX (symétrie des bras à PATTERN_TOLERANCE_PX) —
     toute config non conforme est REJETÉE ;
  d) AprilTag au-dessus -> identité. Tag + croix à bonne distance (0.5–4 m) ->
     reset (0,0,0) + log "Balise ID X détectée - Reset coordonnées".

ARCHITECTURE / THREADS :
  - thread PRINCIPAL  : boucle Tkinter (affichage) — Tk doit tourner dans le
    thread principal (contrainte de Tkinter) ;
  - thread de CONTRÔLE (démon) : la boucle ROS `while not rospy.is_shutdown()`
    qui pilote /cmd_vel — c'est ELLE qui est déportée pour ne PAS bloquer l'UI ;
  - threads d'abonnement rospy : callbacks image / odométrie / batterie.
  État partagé protégé par un verrou ; la GUI lit des instantanés et n'écrit que
  des consignes (mode, manuel, armement) ; le journal passe par une file.

Topic caméra dynamique (anti-crash) :
  rospy.get_param('~image_topic', '/raspicam_node/image')  (sinon auto-détection).

Lancement (PC avec écran, relié au master du robot) :  python3 leo_tracking_map.py
Sans interface (robot headless) :                       python3 leo_tracking_map.py _gui:=false
"""

import os
import math
import time
import queue
import signal
import threading
import collections

import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float32
import tf2_ros
from geometry_msgs.msg import TransformStamped

import cv2
import numpy as np

try:
    _ARUCO = cv2.aruco
    _HAVE_ARUCO = True
except Exception:
    _ARUCO, _HAVE_ARUCO = None, False

try:
    from leo_msgs.msg import WheelOdom
    _HAVE_LEO_MSGS = True
except Exception:
    WheelOdom, _HAVE_LEO_MSGS = None, False

try:
    import tkinter as tk
    _HAVE_TK = True
except Exception:
    tk, _HAVE_TK = None, False
try:
    from PIL import Image as PILImage, ImageTk
    _HAVE_PIL = True
except Exception:
    _HAVE_PIL = False

# ══════════════════════════════════════════════════════════════════════════════
# CONSTANTES (réglables ici)
# ══════════════════════════════════════════════════════════════════════════════
FRAME_ID      = "map"

# --- Mouvement ---
SEARCH_SPEED  = 0.4
ADVANCE_SPEED = 0.2
ADVANCE_T     = 1.0
RATE_HZ       = 20
MANUAL_LIN    = 0.2
MANUAL_ANG    = 0.6
TARGET_KP     = 0.004
TARGET_STOP_M = 0.6

# --- Sécurité / robustesse ---
IMG_TIMEOUT   = 1.0     # s — sans image, ARRÊT (pas de pilotage à l'aveugle)
WARN_TIMEOUT  = 5.0     # s — au-delà : alerte "IMAGE PERDUE" dans l'interface
DETECT_MIN_DT = 0.07    # s — bride la détection (~14 Hz) pour ménager le CPU
STOP_REPEATS  = 5       # nb de Twist nuls à l'arrêt (garantie moteur coupé)

PATH_MIN_DIST = 0.02
PATH_MAX_PTS  = 5000
BEACON_COOLDOWN = 3.0
TAG_VALID_S     = 1.5

# --- Caméra : auto-détection. COMPRESSÉ EN PREMIER = essentiel sur Wi-Fi (le
# flux brut ~27 Mo/s sature le réseau et fait "sauter" caméra ET odométrie). ---
CAMERA_CANDIDATES = [
    "/camera/color/image_raw/compressed",   # compressé d'abord (léger réseau)
    "/raspicam_node/image/compressed",
    "/camera/image_raw/compressed",
    "/raspicam_node/image_raw/compressed",
    "/camera/color/image_raw",              # brut en repli (exécution sur le robot)
    "/raspicam_node/image", "/raspicam_node/image_raw", "/camera/image_raw",
]

# --- Marqueur de la balise ---
# La PHOTO de la balise montre un DAMIER (checkerboard), PAS un AprilTag.
# On détecte donc le damier (cv2.findChessboardCorners) ; l'AprilTag reste géré
# en repli s'il est présent. findChessboardCorners exige le nb de coins INTERNES
# (cases-1) ; comme on ne le connaît pas exactement, on essaie ces tailles.
CHECKERBOARD_SIZES = [(7, 7), (6, 6), (7, 9), (6, 8), (5, 7), (8, 8)]
TAG_FAMILIES = ["DICT_APRILTAG_36H11", "DICT_APRILTAG_36H10",
                "DICT_APRILTAG_25H9", "DICT_ARUCO_ORIGINAL"]

# --- LED / CROIX (paramètres HSV + tolérance de forme) ---
LED_HUE_LOW   = 80      # teinte mini (HSV OpenCV 0-179)
LED_HUE_HIGH  = 135     # teinte maxi
LED_SAT_MIN   = 10      # saturation mini (cœur LED ~blanc)
LED_VAL_MIN   = 200     # luminosité mini (LED = point très lumineux)
LED_MIN_AREA  = 8.0
LED_MAX_AREA  = 4000.0
LED_MIN_CIRC  = 0.45
LED_CLUSTER_PX = 320              # rayon de regroupement des LED en UNE balise
MIN_LIGHTS    = 3                 # balise réelle (photo) = 3 LED groupées (G/C/D)
PATTERN_TOLERANCE_PX = 18         # tolérance de forme/alignement (rejet reflets)

# --- Distance de LECTURE autorisée ---
DIST_MIN, DIST_MAX = 0.5, 4.0
# ── Intrinseques : ALIGNEES sur leo_backend.py (audit 2026-07-31) ────────────
# Avant harmonisation ce fichier portait un jeu DIFFERENT du backend :
#     384.65, 384.91, 327.62, 238.80   (ici)
#     384.65, 384.65, 320.00, 240.00   (leo_backend.py, production)
# Deux outils lisant la meme camera et calculant des portees differentes :
# l'ecart sur cx (327.62 vs 320.00, soit 7.6 px) suffisait a decaler l'angle
# de gisement d'environ 1.1 deg a toute distance. On s'aligne sur le backend,
# qui est le noeud reellement en production.
#
# Ces valeurs ne sont la calibration mesuree d'AUCUN flux (couleur mesuree :
# fx=380.25, cx=320.40, cy=243.72 ; infrarouge : fx=fy=386.79, cx=320.06,
# cy=236.77). 384.65 vient tres probablement d'une calibration INFRAROUGE
# historique, conservee apres le passage au flux couleur. Approximation
# bornee a ~1-2 % sous 3 m. Voir le commentaire detaille dans leo_backend.py
# et l'annexe B du rapport.
CAM_FX, CAM_FY, CAM_CX, CAM_CY = 384.65, 384.65, 320.00, 240.00
BEACON_WIDTH_M = 0.165            # empan LED gauche-droite (à vérifier !)
TAG_SIZE_M = 0.0

# --- Thème NASA (dark mode rétro-futuriste) ---
FONT_MONO   = "Courier New"
NASA_BG     = "#020611"   # bleu nuit quasi-noir
NASA_PANEL  = "#081225"
NASA_EDGE   = "#15314f"
NASA_CYAN   = "#27e0ff"
NASA_WHITE  = "#dff0ff"
NASA_GREEN  = "#2dff8a"
NASA_AMBER  = "#ffb000"
NASA_RED    = "#ff3b30"
NASA_DIM    = "#3c5a78"

AUTO, MANUAL, TARGET = "AUTO", "MANUEL", "CIBLER"


# ══════════════════════════════════════════════════════════════════════════════
# Helpers image
# ══════════════════════════════════════════════════════════════════════════════
def imgmsg_to_bgr(msg):
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        n = msg.height * msg.width * 3
        if len(msg.data) < n:
            raise ValueError("buffer image tronqué")
        arr = np.frombuffer(msg.data, np.uint8, count=n).reshape(
            msg.height, msg.width, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if enc == "rgb8" else arr
    if enc == "mono8":
        n = msg.height * msg.width
        arr = np.frombuffer(msg.data, np.uint8, count=n).reshape(
            msg.height, msg.width)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"encodage non géré : {msg.encoding}")


def compressed_to_bgr(msg):
    bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("cv2.imdecode a échoué")
    return bgr


def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def build_aruco():
    if not _HAVE_ARUCO:
        return []
    get_dict = getattr(_ARUCO, "Dictionary_get", None) or \
               getattr(_ARUCO, "getPredefinedDictionary", None)
    make_par = getattr(_ARUCO, "DetectorParameters_create", None) or \
               getattr(_ARUCO, "DetectorParameters", None)
    out = []
    for name in TAG_FAMILIES:
        const = getattr(_ARUCO, name, None)
        if const is None or get_dict is None:
            continue
        try:
            out.append((name, get_dict(const), make_par()))
        except Exception:
            pass
    return out


# ══════════════════════════════════════════════════════════════════════════════
# CLASSE ROS — TrackingMap
# ══════════════════════════════════════════════════════════════════════════════
class TrackingMap:
    def __init__(self):
        rospy.init_node("leo_tracking_map", anonymous=False, disable_signals=True)

        # Topic caméra dynamique (anti-crash) : explicite, sinon auto-détection
        topic_param = rospy.get_param("~image_topic", "/raspicam_node/image")
        if topic_param and topic_param not in ("auto", "/raspicam_node/image"):
            self.image_topic = topic_param
        else:
            self.image_topic = self._detect_image_topic()
        self.compressed = self.image_topic.endswith("/compressed")
        self.odom_topic = rospy.get_param("~odom_topic", "/firmware/wheel_odom")

        self._aruco = build_aruco()
        fx = rospy.get_param("~fx", CAM_FX)
        self.cam_fx = fx
        self._cam_mtx = np.array([[fx, 0, rospy.get_param("~cx", CAM_CX)],
                                  [0, rospy.get_param("~fy", CAM_FY),
                                   rospy.get_param("~cy", CAM_CY)],
                                  [0, 0, 1]], float)
        self._cam_dist = np.zeros((4, 1))

        # Réglages détection (modifiables en direct par la GUI)
        self.hue_low, self.hue_high = LED_HUE_LOW, LED_HUE_HIGH
        self.val_min, self.min_lights = LED_VAL_MIN, MIN_LIGHTS

        # Odométrie / carte
        self.origin = None
        self.cur_rel = (0.0, 0.0, 0.0)
        self.raw_yaw = None
        self.path = Path(); self.path.header.frame_id = FRAME_ID
        self.markers = []
        self.traj = []
        self.beacons = []
        self.beacon_count = 0

        # Télémétrie carte
        self.batt_v = None

        # Détection (partagé sous verrou)
        self._lock = threading.Lock()
        self.latest_bgr = None
        self.det = {"leds": [], "cluster": [], "center": None, "dist": None,
                    "checker": None, "tag": None, "mask": None}
        self.img_w = 0
        self.last_tag_id = None
        self.last_tag_t = rospy.Time(0)
        self.beacon_prev = False
        self.last_beacon_t = rospy.Time(0)

        # Journal de détection (file thread-safe vers la GUI)
        self.logq = queue.Queue(maxsize=500)

        # Vitesse des roues (télémétrie)
        self.wheel_vel = [0.0, 0.0, 0.0, 0.0]

        # Buffer caméra 1-slot : le callback ROS y dépose la trame BRUTE ; le
        # thread vision la consomme (découplage réception <-> traitement lourd).
        self._cam_msg = None
        self._cam_seq = 0
        self._proc_seq = 0

        # Watchdogs
        self.last_img_t = rospy.Time(0)
        self.last_odom_t = rospy.Time(0)
        self.last_wheel_t = rospy.Time(0)
        self._stopped = False
        self.ready = False

        # Consignes (écrites par la GUI)
        self.mode = AUTO
        self.manual = (0.0, 0.0)
        self.armed = False        # SÉCURITÉ : DISARMED au démarrage (pas de mvt)

        # I/O
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/tracking/path", Path, queue_size=1)
        self.marker_pub = rospy.Publisher("/tracking/markers", MarkerArray,
                                          queue_size=1)
        # Abonnements (tcp_nodelay=True : pas de buffering Nagle -> latence mini ;
        # queue_size=1 : on garde la trame la plus FRAÎCHE et on jette le retard).
        if _HAVE_LEO_MSGS:
            rospy.Subscriber(self.odom_topic, WheelOdom, self._on_odom,
                             queue_size=20, tcp_nodelay=True)
            try:
                from leo_msgs.msg import WheelStates
                rospy.Subscriber("/firmware/wheel_states", WheelStates,
                                 self._on_wheels, queue_size=5, tcp_nodelay=True)
            except Exception:
                pass
        rospy.Subscriber("/firmware/battery", Float32, self._on_battery,
                         queue_size=5, tcp_nodelay=True)
        ctype = CompressedImage if self.compressed else Image
        rospy.Subscriber(self.image_topic, ctype, self._on_image,
                         queue_size=1, buff_size=2**24, tcp_nodelay=True)

        # Service de RESET MATÉRIEL de l'odométrie (coordonnées physiques -> 0)
        self._reset_srv = None
        try:
            from std_srvs.srv import Trigger
            rospy.wait_for_service("firmware/reset_odometry", timeout=2.0)
            self._reset_srv = rospy.ServiceProxy("firmware/reset_odometry", Trigger)
        except Exception:
            pass

        self._static_br = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = FRAME_ID
        t.child_frame_id = "tracking_root"
        t.transform.rotation.w = 1.0
        self._static_br.sendTransform(t)

        rospy.on_shutdown(self._safe_stop)

        # THREAD VISION DÉDIÉ : décode + détecte hors des callbacks ROS.
        self._proc_thread = threading.Thread(target=self._process_loop,
                                             name="vision", daemon=True)
        self._proc_thread.start()

        self._log("SYSTÈME INITIALISÉ — DISARMED")
        rospy.loginfo(f"[Map] image : {self.image_topic} "
                      f"({'compressé' if self.compressed else 'brut'})")

    # ── Journal ───────────────────────────────────────────────────────────────
    def _log(self, text):
        line = f"[{time.strftime('%H:%M:%S')}] {text}"
        try:
            self.logq.put_nowait(line)
        except queue.Full:
            pass
        rospy.loginfo(f"[Map] {text}")

    # ── Auto-détection caméra ─────────────────────────────────────────────────
    def _detect_image_topic(self):
        rospy.loginfo("[Map] détection automatique du topic caméra...")
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            published = {n for n, _ in rospy.get_published_topics()}
            for cand in CAMERA_CANDIDATES:
                if cand not in published:
                    continue
                mt = CompressedImage if cand.endswith("/compressed") else Image
                try:
                    rospy.wait_for_message(cand, mt, timeout=1.5)
                    rospy.loginfo(f"[Map] caméra : {cand}")
                    return cand
                except rospy.ROSException:
                    pass
            rospy.sleep(0.3)
        return CAMERA_CANDIDATES[0]

    # ── Callbacks capteurs ────────────────────────────────────────────────────
    def _on_battery(self, msg):
        try:
            self.batt_v = float(msg.data)
        except Exception:
            pass

    def _on_odom(self, msg):
        try:
            px, py, yaw = float(msg.pose_x), float(msg.pose_y), float(msg.pose_yaw)
        except (AttributeError, TypeError, ValueError) as e:
            rospy.logwarn_throttle(5, f"[Map] odométrie illisible : {e}")
            return
        self.last_odom_t = rospy.Time.now()
        self.raw_yaw = yaw
        try:
            if self.origin is None:
                self.origin = (px, py, yaw)
            ox, oy, oyaw = self.origin
            dx, dy = px - ox, py - oy
            c, s = math.cos(-oyaw), math.sin(-oyaw)
            rx, ry = c * dx - s * dy, s * dx + c * dy
            ryaw = math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw))
            self.cur_rel = (rx, ry, ryaw)
            with self._lock:
                if not self.traj or math.hypot(rx - self.traj[-1][0],
                                               ry - self.traj[-1][1]) > PATH_MIN_DIST:
                    self.traj.append((rx, ry))
                    if len(self.traj) > PATH_MAX_PTS:
                        self.traj.pop(0)
            self._append_path(rx, ry)
        except Exception as e:
            rospy.logwarn_throttle(5, f"[Map] traitement odométrie : {e}")

    def _append_path(self, rx, ry):
        if self.path.poses:
            last = self.path.poses[-1].pose.position
            if math.hypot(rx - last.x, ry - last.y) < PATH_MIN_DIST:
                return
        ps = PoseStamped()
        ps.header.frame_id = FRAME_ID
        ps.header.stamp = rospy.Time.now()
        ps.pose.position = Point(rx, ry, 0.0)
        ps.pose.orientation.w = 1.0
        self.path.poses.append(ps)
        if len(self.path.poses) > PATH_MAX_PTS:
            self.path.poses.pop(0)

    def _on_wheels(self, msg):
        """leo_msgs/WheelStates : velocity[4] (FL, RL, FR, RR)."""
        try:
            self.wheel_vel = list(msg.velocity)
            self.last_wheel_t = rospy.Time.now()
        except Exception:
            pass

    def _on_image(self, msg):
        """CALLBACK ULTRA-LÉGER : on ne fait QUE déposer la trame brute + l'heure.
        Tout le décodage/détection (lourd) est fait par _process_loop dans un AUTRE
        thread -> le thread de réception ROS n'est jamais bloqué -> caméra ET
        odométrie continuent d'arriver, même quand la détection est coûteuse."""
        self.last_img_t = rospy.Time.now()
        with self._lock:
            self._cam_msg = msg
            self._cam_seq += 1

    def _process_loop(self):
        """Thread VISION dédié : consomme la dernière trame à son propre rythme
        (~14 Hz), totalement découplé des callbacks ROS et de la GUI."""
        rate = rospy.Rate(max(1, int(1.0 / DETECT_MIN_DT)))
        while not rospy.is_shutdown():
            msg = None
            with self._lock:
                if self._cam_seq != self._proc_seq:
                    msg, self._proc_seq = self._cam_msg, self._cam_seq
            if msg is not None:
                try:
                    bgr = compressed_to_bgr(msg) if self.compressed \
                        else imgmsg_to_bgr(msg)
                    self._process_frame(bgr, rospy.Time.now())
                except Exception as e:
                    rospy.logwarn_throttle(5, f"[Map] traitement image : {e}")
            rate.sleep()

    def _process_frame(self, bgr, now):
        det = self._detect_specific_beacon(bgr)
        with self._lock:
            self.latest_bgr = bgr
            self.det = det
            self.img_w = bgr.shape[1]
        if det["tag"] is not None:
            self.last_tag_id, self.last_tag_t = det["tag"][0], now

        lights = bool(det["cluster"])               # amas de LED suffisant ?
        marker = det["checker"] is not None or det["tag"] is not None
        center, dist = det["center"], det["dist"]
        in_range = (dist is not None and DIST_MIN <= dist <= DIST_MAX)
        valid = lights and marker and in_range       # BALISE complète + bonne dist

        if lights and not marker:
            rospy.loginfo_throttle(3.0, "[Map] LED vues mais pas de damier/tag")
        elif lights and marker and not in_range:
            d_txt = "?" if dist is None else f"{dist:.2f}m"
            rospy.loginfo_throttle(3.0, f"[Map] balise vue (dist={d_txt}) hors "
                                   f"plage [{DIST_MIN}-{DIST_MAX}] m")

        if valid and not self.beacon_prev and \
                (now - self.last_beacon_t).to_sec() > BEACON_COOLDOWN:
            self._on_beacon_seen(now, dist, det)
            self.last_beacon_t = now
        self.beacon_prev = valid

    # ── Détection : blobs -> barycentre -> CROIX ──────────────────────────────
    def _detect_leds(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        mask = ((V >= self.val_min) & (S >= LED_SAT_MIN) &
                (H >= self.hue_low) & (H <= self.hue_high)).astype(np.uint8) * 255
        mask = cv2.GaussianBlur(mask, (3, 3), 0)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        leds = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < LED_MIN_AREA or area > LED_MAX_AREA:
                continue
            per = cv2.arcLength(c, True)
            if per <= 0 or 4.0 * math.pi * area / (per * per) < LED_MIN_CIRC:
                continue
            M = cv2.moments(c)
            if M["m00"] > 0:
                leds.append((M["m10"] / M["m00"], M["m01"] / M["m00"], area))
        return leds, mask

    def _find_cross(self, pts):
        """4 LED en CROIX autour du barycentre, vérif géométrique. None sinon."""
        if len(pts) < self.min_lights:
            return None
        cx = sum(p[0] for p in pts) / len(pts)         # (b) barycentre
        cy = sum(p[1] for p in pts) / len(pts)
        left = right = top = bot = None
        for (x, y) in pts:
            dx, dy = x - cx, y - cy
            if abs(dx) >= abs(dy):
                if dx < 0 and (left is None or x < left[0]):
                    left = (x, y)
                elif dx > 0 and (right is None or x > right[0]):
                    right = (x, y)
            else:
                if dy < 0 and (top is None or y < top[1]):
                    top = (x, y)
                elif dy > 0 and (bot is None or y > bot[1]):
                    bot = (x, y)
        if not (left and right and top and bot):
            return None
        tol = PATTERN_TOLERANCE_PX                      # (c) tolérance forme
        ccx = (top[0] + bot[0]) / 2.0
        ccy = (left[1] + right[1]) / 2.0
        if abs(left[1] - right[1]) > tol or abs(top[0] - bot[0]) > tol:
            return None
        if abs(top[0] - ccx) > tol or abs(bot[0] - ccx) > tol:
            return None
        if abs(left[1] - ccy) > tol or abs(right[1] - ccy) > tol:
            return None
        if abs(abs(left[0]-ccx) - abs(right[0]-ccx)) > tol:
            return None
        if abs(abs(top[1]-ccy) - abs(bot[1]-ccy)) > tol:
            return None
        if min(abs(left[0]-ccx), abs(right[0]-ccx),
               abs(top[1]-ccy), abs(bot[1]-ccy)) < tol:
            return None
        return (left, right, top, bot), (ccx, ccy)

    def _detect_tag(self, bgr):
        if not self._aruco:
            return None
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        for _n, dic, params in self._aruco:
            try:
                corners, ids, _ = _ARUCO.detectMarkers(gray, dic,
                                                        parameters=params)
            except Exception:
                continue
            if ids is not None and len(ids) > 0:
                quad = corners[0][0]
                return (int(ids.flatten()[0]),
                        (float(np.mean(quad[:, 0])), float(np.mean(quad[:, 1]))),
                        quad)
        return None

    def _tag_distance(self, quad):
        if quad is None or TAG_SIZE_M <= 0 or not _HAVE_ARUCO:
            return None
        try:
            _r, tv, _ = _ARUCO.estimatePoseSingleMarkers(
                [quad.reshape(1, 4, 2)], TAG_SIZE_M, self._cam_mtx, self._cam_dist)
            return float(np.linalg.norm(np.asarray(tv).reshape(3)))
        except Exception:
            return None

    def _cluster_lights(self, leds):
        """(a) Regroupe les LED proches (≥ min_lights) autour de la plus grosse.
        Coh. spatiale : rejette les reflets isolés. Retourne (cluster, centre)."""
        if len(leds) < self.min_lights:
            return [], None
        leds = sorted(leds, key=lambda l: l[2], reverse=True)
        ax, ay, _ = leds[0]
        cluster = [(x, y) for (x, y, _a) in leds
                   if math.hypot(x - ax, y - ay) <= LED_CLUSTER_PX]
        if len(cluster) < self.min_lights:
            return [], None
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        return cluster, (cx, cy)

    def _detect_checkerboard(self, gray):
        """(b) Détecte le DAMIER (marqueur réel de la balise, cf. photo).
        FAST_CHECK = sortie rapide si aucun damier -> peu coûteux. Renvoie
        (taille, coins) ou None."""
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
        for size in CHECKERBOARD_SIZES:
            try:
                found, corners = cv2.findChessboardCorners(gray, size, flags=flags)
            except Exception:
                found, corners = False, None
            if found:
                return (size, corners)
        return None

    def _detect_specific_beacon(self, bgr):
        """Détection de la balise RÉELLE (photo) : amas de LED bleues + DAMIER
        (ou AprilTag en repli). Distance via l'empan horizontal des LED."""
        leds, mask = self._detect_leds(bgr)
        cluster, center = self._cluster_lights(leds)
        dist = None
        if cluster:
            xs = [p[0] for p in cluster]
            span = max(xs) - min(xs)
            dist = (self.cam_fx * BEACON_WIDTH_M / span) if span >= 1 else None
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        return {"leds": leds, "cluster": cluster, "center": center, "dist": dist,
                "checker": self._detect_checkerboard(gray),
                "tag": self._detect_tag(bgr), "mask": mask}

    # compat (auto-test) : balise présente ?
    def _detect_beacon(self, bgr):
        d = self._detect_specific_beacon(bgr)
        present = bool(d["cluster"]) and (d["checker"] is not None
                                          or d["tag"] is not None)
        return present, d["center"], d["dist"]

    # ── Balise validée -> RESET physique (0,0,0) + marqueur + LOG ────────────
    def _on_beacon_seen(self, now, dist, det):
        if det.get("tag") is not None:
            id_txt = str(det["tag"][0])
        elif det.get("checker") is not None:
            sz = det["checker"][0]
            id_txt = f"DAMIER {sz[0]}x{sz[1]}"
        else:
            id_txt = "?"
        label = f"BALISE {id_txt}"
        self.beacon_count += 1

        # RESET PHYSIQUE des coordonnées : service matériel du firmware si dispo,
        # + re-zéro de notre repère carte. La mission AUTO continue sans coupure.
        hw = ""
        if self._reset_srv is not None:
            try:
                self._reset_srv()
                hw = " [odom HW remis à 0]"
            except Exception:
                hw = ""
        self._log(f"BALISE [{id_txt}] REPÉRÉE - RESET EFFECTUÉ{hw}")

        self.origin = None
        self.path = Path(); self.path.header.frame_id = FRAME_ID
        self.markers = []
        with self._lock:
            self.traj = []
            self.beacons = [(0.0, 0.0, label)]
        sphere = self._make_marker("beacon", 1, Marker.SPHERE, 0, 0, 0, 0.20,
                                   ColorRGBA(0.0, 1.0, 0.2, 1.0))
        txt = self._make_marker("beacon_text", 2, Marker.TEXT_VIEW_FACING,
                                0, 0, 0.30, 0.15, ColorRGBA(1, 1, 1, 1))
        txt.text = label
        self.markers += [sphere, txt]

    def _make_marker(self, ns, mid, mtype, x, y, z, scale, color):
        m = Marker()
        m.header.frame_id = FRAME_ID
        m.header.stamp = rospy.Time.now()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.position = Point(x, y, z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = color
        m.lifetime = rospy.Duration(0)
        return m

    def _robot_marker(self):
        rx, ry, ryaw = self.cur_rel
        m = Marker()
        m.header.frame_id = FRAME_ID
        m.header.stamp = rospy.Time.now()
        m.ns, m.id, m.type, m.action = "robot", 0, Marker.ARROW, Marker.ADD
        m.pose.position = Point(rx, ry, 0.05)
        m.pose.orientation = yaw_to_quat(ryaw)
        m.scale.x, m.scale.y, m.scale.z = 0.35, 0.06, 0.06
        m.color = ColorRGBA(1, 1, 1, 1)
        m.lifetime = rospy.Duration(0)
        return m

    # ── Sécurité moteur (NE PAS CASSER) ──────────────────────────────────────
    def _publish_cmd(self, cmd):
        try:
            self.cmd_pub.publish(cmd)
        except Exception as e:
            rospy.logwarn_throttle(5, f"[Map] publication /cmd_vel : {e}")

    def _safe_stop(self):
        """Arrêt moteur GARANTI : plusieurs Twist nuls (sécurité prioritaire)."""
        if self._stopped:
            return
        self._stopped = True
        stop = Twist()
        for _ in range(STOP_REPEATS):
            try:
                self.cmd_pub.publish(stop)
            except Exception:
                break
            time.sleep(0.02)
        rospy.loginfo("[Map] STOP moteur (Twist nul).")

    def _publish_map(self, now):
        try:
            self.path.header.stamp = now
            self.path_pub.publish(self.path)
            arr = MarkerArray()
            arr.markers = self.markers + [self._robot_marker()]
            self.marker_pub.publish(arr)
        except Exception as e:
            rospy.logwarn_throttle(5, f"[Map] publication carte : {e}")

    def camera_alive(self, now=None):
        now = now or rospy.Time.now()
        return (self.last_img_t != rospy.Time(0) and
                (now - self.last_img_t).to_sec() <= IMG_TIMEOUT)

    def image_age(self):
        if self.last_img_t == rospy.Time(0):
            return 999.0
        return (rospy.Time.now() - self.last_img_t).to_sec()

    # ── Consignes / état (depuis la GUI) ─────────────────────────────────────
    def set_mode(self, mode):
        self.mode = mode
        if mode == AUTO:
            self._start_scan(rospy.Time.now())
        self.manual = (0.0, 0.0)
        self._log(f"MODE -> {mode}")

    def set_manual(self, lin, ang):
        self.manual = (lin, ang)

    def set_armed(self, armed):
        self.armed = bool(armed)
        self._log("ARMED — moteurs actifs" if armed else "DISARMED — moteurs coupés")
        if not armed:
            self._publish_cmd(Twist())

    def request_stop(self):
        self.mode = MANUAL
        self.manual = (0.0, 0.0)
        self.armed = False
        self._publish_cmd(Twist())
        self._log("STOP — DISARMED")

    def _age(self, stamp):
        if stamp == rospy.Time(0):
            return 999.0
        return (rospy.Time.now() - stamp).to_sec()

    def telemetry(self):
        rx, ry, ryaw = self.cur_rel
        with self._lock:
            locked = bool(self.det.get("cluster")) and (
                self.det.get("checker") is not None or self.det.get("tag") is not None)
            dist = self.det["dist"]
            nleds = len(self.det["leds"])
        return {"x": rx, "y": ry, "z": 0.0, "yaw": math.degrees(ryaw),
                "batt": self.batt_v, "mode": self.mode, "armed": self.armed,
                "beacons": self.beacon_count,
                "cam_age": self.image_age(), "odom_age": self._age(self.last_odom_t),
                "wheel_age": self._age(self.last_wheel_t),
                "wheels": list(self.wheel_vel), "locked": locked,
                "dist": dist, "nleds": nleds}

    def snapshot(self):
        with self._lock:
            return (self.latest_bgr, self.det, list(self.traj),
                    list(self.beacons), self.cur_rel)

    # ── Auto-test ─────────────────────────────────────────────────────────────
    def self_test(self):
        ctype = CompressedImage if self.compressed else Image
        ok_cam = ok_odom = False
        try:
            rospy.wait_for_message(self.image_topic, ctype, timeout=5.0)
            ok_cam = True
            self._log(f"CAMÉRA OK ({self.image_topic})")
        except rospy.ROSException:
            self._log(f"CAMÉRA ABSENTE ({self.image_topic})")
        if _HAVE_LEO_MSGS:
            try:
                rospy.wait_for_message(self.odom_topic, WheelOdom, timeout=5.0)
                ok_odom = True
                self._log("ODOMÉTRIE OK")
            except rospy.ROSException:
                self._log("ODOMÉTRIE ABSENTE")
        return ok_cam and ok_odom

    # ── Boucle de contrôle (thread de fond) ──────────────────────────────────
    def _start_scan(self, now):
        self.search_state = "SCAN"
        self.scan_accum = 0.0
        self.scan_last_yaw = self.raw_yaw
        self.scan_start_t = now

    def run(self):
        self.ready = self.self_test()
        scan_max_t = 1.5 * (2 * math.pi / SEARCH_SPEED)
        self.advance_start = None
        self._start_scan(rospy.Time.now())
        rate = rospy.Rate(RATE_HZ)
        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now()
                cmd = Twist()
                if not self.ready and self.camera_alive(now) and \
                        self.last_odom_t != rospy.Time(0):
                    self.ready = True

                # ARMEMENT + watchdog caméra : sinon, robot À L'ARRÊT
                if (not self.armed) or (not self.camera_alive(now)) \
                        or (not self.ready):
                    self._publish_cmd(Twist())
                    self._publish_map(now)
                    rate.sleep()
                    continue

                mode = self.mode
                if mode == MANUAL:
                    cmd.linear.x, cmd.angular.z = self.manual
                elif mode == TARGET:
                    with self._lock:
                        center = self.det["center"]; dist = self.det["dist"]
                        w = self.img_w
                    if center is not None and w:
                        err = center[0] - w / 2.0
                        cmd.angular.z = max(-0.6, min(0.6, -TARGET_KP * err))
                        cmd.linear.x = 0.0 if (dist is not None and
                                               dist < TARGET_STOP_M) else 0.15
                else:  # AUTO
                    if self.search_state == "SCAN":
                        cmd.angular.z = SEARCH_SPEED
                        if self.raw_yaw is not None:
                            if self.scan_last_yaw is None:
                                self.scan_last_yaw = self.raw_yaw
                            d = math.atan2(
                                math.sin(self.raw_yaw - self.scan_last_yaw),
                                math.cos(self.raw_yaw - self.scan_last_yaw))
                            self.scan_accum += abs(d)
                            self.scan_last_yaw = self.raw_yaw
                        if self.scan_accum >= 2 * math.pi or \
                                (now - self.scan_start_t).to_sec() > scan_max_t:
                            self.search_state = "ADVANCE"
                            self.advance_start = now
                    elif self.search_state == "ADVANCE":
                        if (now - self.advance_start).to_sec() < ADVANCE_T:
                            cmd.linear.x = ADVANCE_SPEED
                        else:
                            self._start_scan(now)

                self._publish_cmd(cmd)
                self._publish_map(now)
                rate.sleep()
        except Exception as e:
            rospy.logerr(f"[Map] erreur boucle -> arrêt : {e}")
        finally:
            self._safe_stop()


# ══════════════════════════════════════════════════════════════════════════════
# CLASSE GUI — NASA_ControlPanel (Tkinter, dark mode rétro-futuriste)
# ══════════════════════════════════════════════════════════════════════════════
class NASA_ControlPanel:
    def __init__(self, root, tm):
        self.root, self.tm = root, tm
        root.title("LEO ROVER — NASA MISSION CONTROL")
        root.configure(bg=NASA_BG)
        self.show_mask = tk.BooleanVar(value=False)
        self._photo = None
        self._build()
        self._bind_keys()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._update)        # télémétrie ~10 Hz

    # ---- widgets utilitaires ----
    def _mono(self, size, bold=False):
        return (FONT_MONO, size, "bold" if bold else "normal")

    def _btn(self, parent, text, cmd, fg=NASA_CYAN):
        return tk.Button(parent, text=text, command=cmd, bg=NASA_PANEL, fg=fg,
                         activebackground=fg, activeforeground=NASA_BG,
                         relief="flat", bd=0, font=self._mono(11, True),
                         padx=10, pady=6, highlightthickness=1,
                         highlightbackground=NASA_EDGE)

    def _build(self):
        # En-tête : badge MISSION CONTROL + ARMED/DISARMED
        head = tk.Frame(self.root, bg=NASA_BG)
        head.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=6)
        tk.Label(head, text="◤ NASA MISSION CONTROL ◢", bg=NASA_BG, fg=NASA_CYAN,
                 font=self._mono(15, True)).pack(side="left")
        tk.Label(head, text="  LEO ROVER  •  AUTONOMOUS TRACKING",
                 bg=NASA_BG, fg=NASA_DIM, font=self._mono(10)).pack(side="left")
        self.lbl_arm = tk.Label(head, text="● DISARMED", bg=NASA_BG,
                                fg=NASA_GREEN, font=self._mono(14, True))
        self.lbl_arm.pack(side="right")

        # Bannière d'alerte
        self.banner = tk.Label(self.root, text="", bg=NASA_BG, fg=NASA_RED,
                               font=self._mono(12, True))
        self.banner.grid(row=1, column=0, columnspan=2, sticky="ew")

        # Caméra (gauche)
        camf = self._panel("VIDEO FEED")
        camf.grid(row=2, column=0, padx=10, pady=6, sticky="n")
        self.cam_label = tk.Label(camf, bg="#000000")
        self.cam_label.pack(padx=6, pady=6)
        self.det_label = tk.Label(camf, text="--", bg=NASA_PANEL, fg=NASA_WHITE,
                                  font=self._mono(11, True))
        self.det_label.pack(pady=(0, 4))
        tune = tk.Frame(camf, bg=NASA_PANEL)
        tune.pack(fill="x", padx=6, pady=2)
        self.s_hlo = self._scale(tune, "HUE.LO", 0, 179, self.tm.hue_low, 0)
        self.s_hhi = self._scale(tune, "HUE.HI", 0, 179, self.tm.hue_high, 1)
        self.s_val = self._scale(tune, "VAL.MIN", 0, 255, self.tm.val_min, 2)
        self.s_nled = self._scale(tune, "LEDS.MIN", 1, 6, self.tm.min_lights, 3)
        tk.Checkbutton(tune, text="MASK VIEW", variable=self.show_mask,
                       bg=NASA_PANEL, fg=NASA_CYAN, selectcolor=NASA_BG,
                       activebackground=NASA_PANEL, font=self._mono(9)).grid(
            row=4, column=0, columnspan=2, sticky="w")

        # Colonne droite : télémétrie + carte + log
        right = tk.Frame(self.root, bg=NASA_BG)
        right.grid(row=2, column=1, padx=10, pady=6, sticky="n")

        telf = self._panel("TELEMETRY", parent=right)
        telf.pack(fill="x")
        self.telem = tk.Canvas(telf, width=360, height=170, bg="#02060f",
                               highlightthickness=0)
        self.telem.pack(padx=6, pady=6)

        mapf = self._panel("MAP — TOP VIEW", parent=right)
        mapf.pack(fill="x", pady=(8, 0))
        self.mw = self.mh = 300
        self.canvas = tk.Canvas(mapf, width=self.mw, height=self.mh,
                                bg="#02060f", highlightthickness=0)
        self.canvas.pack(padx=6, pady=6)

        logf = self._panel("DETECTION LOG", parent=right)
        logf.pack(fill="x", pady=(8, 0))
        self.logbox = tk.Text(logf, width=46, height=7, bg="#02060f",
                              fg=NASA_GREEN, font=self._mono(9),
                              relief="flat", highlightthickness=0, state="disabled")
        self.logbox.pack(padx=6, pady=6)

        # Boutons
        act = tk.Frame(self.root, bg=NASA_BG)
        act.grid(row=3, column=0, columnspan=2, sticky="w", padx=10, pady=8)
        self.b_arm = self._btn(act, "ARM", self._toggle_arm, fg=NASA_AMBER)
        self.b_arm.pack(side="left", padx=4)
        self.b_auto = self._btn(act, "AUTO", lambda: self._mode(AUTO))
        self.b_auto.pack(side="left", padx=4)
        self.b_man = self._btn(act, "MANUEL", lambda: self._mode(MANUAL))
        self.b_man.pack(side="left", padx=4)
        self.b_tgt = self._btn(act, "CIBLER", lambda: self._mode(TARGET))
        self.b_tgt.pack(side="left", padx=4)
        self._btn(act, "■ STOP", self._stop, fg=NASA_RED).pack(side="left", padx=12)
        tk.Label(act, text="MANUEL: ↑↓←→", bg=NASA_BG, fg=NASA_DIM,
                 font=self._mono(9)).pack(side="left", padx=8)

    def _panel(self, title, parent=None):
        parent = parent or self.root
        f = tk.Frame(parent, bg=NASA_PANEL, highlightthickness=1,
                     highlightbackground=NASA_EDGE)
        tk.Label(f, text="▎" + title, bg=NASA_PANEL, fg=NASA_CYAN,
                 font=self._mono(9, True)).pack(anchor="w", padx=6, pady=2)
        return f

    def _scale(self, parent, label, lo, hi, val, row):
        tk.Label(parent, text=label, bg=NASA_PANEL, fg=NASA_DIM, width=9,
                 anchor="w", font=self._mono(8)).grid(row=row, column=0, sticky="w")
        s = tk.Scale(parent, from_=lo, to=hi, orient="horizontal", length=150,
                     bg=NASA_PANEL, fg=NASA_WHITE, troughcolor=NASA_BG,
                     highlightthickness=0, bd=0, font=self._mono(7))
        s.set(val)
        s.grid(row=row, column=1, sticky="w")
        return s

    def _bind_keys(self):
        self.root.bind("<KeyPress-Up>",    lambda e: self.tm.set_manual(MANUAL_LIN, 0))
        self.root.bind("<KeyPress-Down>",  lambda e: self.tm.set_manual(-MANUAL_LIN, 0))
        self.root.bind("<KeyPress-Left>",  lambda e: self.tm.set_manual(0, MANUAL_ANG))
        self.root.bind("<KeyPress-Right>", lambda e: self.tm.set_manual(0, -MANUAL_ANG))
        for k in ("<KeyRelease-Up>", "<KeyRelease-Down>",
                  "<KeyRelease-Left>", "<KeyRelease-Right>"):
            self.root.bind(k, lambda e: self.tm.set_manual(0, 0))

    # ---- actions ----
    def _toggle_arm(self):
        self.tm.set_armed(not self.tm.armed)

    def _mode(self, m):
        self.tm.set_mode(m)
        for b, mm in ((self.b_auto, AUTO), (self.b_man, MANUAL), (self.b_tgt, TARGET)):
            b.config(fg=NASA_BG if mm == m else NASA_CYAN,
                     bg=NASA_CYAN if mm == m else NASA_PANEL)

    def _stop(self):
        self.tm.request_stop()
        self._mode(MANUAL)

    # ---- boucle d'affichage (thread principal, ~10 Hz) ----
    def _update(self):
        if rospy.is_shutdown():
            self.root.destroy()
            return
        self.tm.hue_low = int(self.s_hlo.get())
        self.tm.hue_high = int(self.s_hhi.get())
        self.tm.val_min = int(self.s_val.get())
        self.tm.min_lights = int(self.s_nled.get())

        bgr, det, traj, beacons, pose = self.tm.snapshot()
        t = self.tm.telemetry()

        # ARMED/DISARMED
        self.lbl_arm.config(text="● ARMED" if t["armed"] else "● DISARMED",
                            fg=NASA_AMBER if t["armed"] else NASA_GREEN)
        self.b_arm.config(text="DISARM" if t["armed"] else "ARM",
                          fg=NASA_RED if t["armed"] else NASA_AMBER)

        # alerte image
        age = t["cam_age"]
        if age > WARN_TIMEOUT:
            self.banner.config(text="⚠  SIGNAL VIDÉO PERDU  —  ROVER STOPPÉ  ⚠",
                               bg=NASA_RED, fg="#ffffff")
        elif age > IMG_TIMEOUT:
            self.banner.config(text="⚠ flux vidéo instable…", bg=NASA_BG,
                               fg=NASA_AMBER)
        else:
            self.banner.config(text="", bg=NASA_BG)

        if bgr is not None and _HAVE_PIL:
            self._show_camera(bgr, det)
        elif not _HAVE_PIL:
            self.cam_label.config(text="pip3 install pillow", fg=NASA_RED)
        self._update_det_label(det, age)
        self._draw_telemetry(t)
        self._draw_map(traj, beacons, pose)
        self._drain_log()
        self.root.after(100, self._update)

    def _show_camera(self, bgr, det):
        view = det["mask"] if (self.show_mask.get() and det["mask"] is not None) \
            else bgr
        view = self._overlay(view, det)
        h, w = view.shape[:2]
        sc = min(480 / w, 360 / h, 1.0)
        if sc < 1.0:
            view = cv2.resize(view, (int(w*sc), int(h*sc)))
        rgb = cv2.cvtColor(view, cv2.COLOR_BGR2RGB)
        self._photo = ImageTk.PhotoImage(PILImage.fromarray(rgb))
        self.cam_label.config(image=self._photo, text="")

    def _overlay(self, view, det):
        view = cv2.cvtColor(view, cv2.COLOR_GRAY2BGR) if view.ndim == 2 \
            else view.copy()
        for (x, y, _a) in det.get("leds", []):
            cv2.circle(view, (int(x), int(y)), 6, (255, 224, 39), 2)
        # amas de LED validé -> contour + centre + distance
        cluster = det.get("cluster") or []
        if cluster:
            for p in cluster:
                cv2.circle(view, (int(p[0]), int(p[1])), 8, (60, 255, 138), -1)
            cc = det.get("center")
            if cc:
                t = "LIGHTS" + (f" {det['dist']:.2f}m" if det.get("dist") else "")
                cv2.putText(view, t, (int(cc[0])-44, int(cc[1])-14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 255, 138), 2)
        # damier détecté
        if det.get("checker") is not None:
            size, corners = det["checker"]
            cv2.drawChessboardCorners(view, size, corners, True)
            cv2.putText(view, f"DAMIER {size[0]}x{size[1]}", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 176, 0), 2)
        if det.get("tag") is not None:
            q = det["tag"][2].astype(int)
            cv2.polylines(view, [q], True, (255, 176, 0), 2)
            cv2.putText(view, f"ID {det['tag'][0]}",
                        (int(q[0][0]), int(q[0][1])-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 176, 0), 2)
        return view

    def _update_det_label(self, det, age):
        if age > IMG_TIMEOUT:
            self.det_label.config(text="NO SIGNAL", fg=NASA_RED)
            return
        n = len(det.get("leds", []))
        lights = bool(det.get("cluster"))
        marker = det.get("checker") is not None or det.get("tag") is not None
        if lights and marker and det.get("dist") is not None:
            inr = DIST_MIN <= det["dist"] <= DIST_MAX
            self.det_label.config(
                text=f"BEACON LOCK  {det['dist']:.2f}m  " +
                     ("[RESET]" if inr else "[OUT 0.5-4m]"),
                fg=NASA_GREEN if inr else NASA_AMBER)
        elif lights:
            self.det_label.config(text=f"{n} LED  •  NO MARKER", fg=NASA_AMBER)
        else:
            self.det_label.config(text=f"{n} LED  •  NO BEACON", fg=NASA_DIM)

    def _draw_telemetry(self, t):
        c = self.telem
        c.delete("all")
        # --- Bloc HAUT-GAUCHE : POSITION X / Y / Z ---
        c.create_text(10, 8, anchor="w", text="POSITION (m)", fill=NASA_DIM,
                      font=self._mono(8))
        for i, (ax, val) in enumerate((("X", t["x"]), ("Y", t["y"]), ("Z", t["z"]))):
            c.create_text(12, 28 + i*22, anchor="w", text=ax, fill=NASA_CYAN,
                          font=self._mono(11, True))
            c.create_text(36, 28 + i*22, anchor="w", text=f"{val:+08.3f}",
                          fill=NASA_WHITE, font=self._mono(13, True))
        # --- Bloc HAUT-DROITE : cap / énergie / état ---
        c.create_text(190, 8, anchor="w", text="STATUS", fill=NASA_DIM,
                      font=self._mono(8))
        c.create_text(192, 28, anchor="w", text=f"YAW {t['yaw']:+06.1f}",
                      fill=NASA_WHITE, font=self._mono(11, True))
        if t["batt"] is not None:
            pct = max(0, min(100, (t["batt"]-10.0)/2.6*100))
            col = NASA_GREEN if pct > 50 else NASA_AMBER if pct > 20 else NASA_RED
            c.create_text(192, 50, anchor="w",
                          text=f"BAT {t['batt']:.1f}V {pct:.0f}%", fill=col,
                          font=self._mono(11, True))
        else:
            c.create_text(192, 50, anchor="w", text="BAT --", fill=NASA_DIM,
                          font=self._mono(11, True))
        c.create_text(192, 72, anchor="w",
                      text=f"MODE {t['mode']}  B:{t['beacons']}", fill=NASA_CYAN,
                      font=self._mono(11, True))
        # voyants capteurs (vert/rouge)
        for i, (lab, age, lim) in enumerate(
                (("CAM", t["cam_age"], IMG_TIMEOUT),
                 ("ODOM", t["odom_age"], 1.5))):
            x = 192 + i * 90
            ok = age < lim
            c.create_oval(x, 92, x + 11, 103,
                          fill=NASA_GREEN if ok else NASA_RED, outline="")
            c.create_text(x + 15, 98, anchor="w", text=lab, fill=NASA_WHITE,
                          font=self._mono(8))
        # --- Bloc BAS : vitesse des 4 roues (sorties moteurs) ---
        c.create_text(10, 118, anchor="w", text="WHEELS (rad/s)  FL · RL · FR · RR",
                      fill=NASA_DIM, font=self._mono(8))
        names = ["FL", "RL", "FR", "RR"]
        wlive = t["wheel_age"] < 2.0
        base = 150
        for i in range(4):
            x = 16 + i * 56
            v = t["wheels"][i] if i < len(t["wheels"]) else 0.0
            act = wlive and abs(v) > 0.05
            col = NASA_GREEN if act else "#244055"
            c.create_rectangle(x, base - 16, x + 22, base + 16, outline=NASA_EDGE)
            h = max(-16, min(16, v * 5))
            if h >= 0:
                c.create_rectangle(x, base - h, x + 22, base, outline="", fill=col)
            else:
                c.create_rectangle(x, base, x + 22, base - h, outline="", fill=col)
            c.create_text(x + 11, base + 24, text=f"{names[i]} {v:+.1f}",
                          fill=NASA_WHITE if act else NASA_DIM, font=self._mono(7))

    def _draw_map(self, traj, beacons, pose):
        c = self.canvas
        c.delete("all")
        W, H, sc = self.mw, self.mh, 46.0
        ox, oy = W/2, H/2

        def px(wx, wy):
            return ox + wx*sc, oy - wy*sc
        for g in range(-3, 4):
            x, _ = px(g, 0); _, y = px(0, g)
            c.create_line(x, 0, x, H, fill="#0c1c30")
            c.create_line(0, y, W, y, fill="#0c1c30")
        c.create_line(0, oy, W, oy, fill=NASA_EDGE)
        c.create_line(ox, 0, ox, H, fill=NASA_EDGE)
        if len(traj) > 1:
            pts = []
            for (wx, wy) in traj:
                a, b = px(wx, wy); pts += [a, b]
            c.create_line(*pts, fill=NASA_CYAN, width=2)
        for (wx, wy, label) in beacons:
            a, b = px(wx, wy)
            c.create_oval(a-7, b-7, a+7, b+7, fill=NASA_GREEN, outline="white")
            c.create_text(a, b-13, text=label, fill="white", font=self._mono(8))
        rx, ry, ryaw = pose
        a, b = px(rx, ry)
        ex, ey = a + 18*math.cos(ryaw), b - 18*math.sin(ryaw)
        c.create_oval(a-5, b-5, a+5, b+5, fill="white", outline=NASA_CYAN)
        c.create_line(a, b, ex, ey, fill="white", width=3, arrow="last")

    def _drain_log(self):
        wrote = False
        while True:
            try:
                line = self.tm.logq.get_nowait()
            except queue.Empty:
                break
            self.logbox.config(state="normal")
            self.logbox.insert("end", line + "\n")
            wrote = True
        if wrote:
            self.logbox.see("end")
            # limite à ~200 lignes
            if int(self.logbox.index("end-1c").split(".")[0]) > 200:
                self.logbox.delete("1.0", "50.0")
            self.logbox.config(state="disabled")

    def _on_close(self):
        try:
            self.tm.request_stop()
            rospy.signal_shutdown("interface fermée")
        except Exception:
            pass
        self.root.destroy()


# ══════════════════════════════════════════════════════════════════════════════
# Démarrage : GUI (thread principal) + boucle de contrôle ROS (thread démon)
# ══════════════════════════════════════════════════════════════════════════════
def main():
    def _sigterm(_s, _f):
        rospy.signal_shutdown("SIGTERM")
    try:
        signal.signal(signal.SIGTERM, _sigterm)
    except (ValueError, OSError):
        pass

    tm = TrackingMap()
    use_gui = rospy.get_param("~gui", True) and _HAVE_TK

    ctrl = threading.Thread(target=tm.run, name="control-loop", daemon=True)
    ctrl.start()

    if use_gui:
        try:
            root = tk.Tk()
        except Exception as e:
            rospy.logwarn(f"[GUI] pas d'affichage ({e}) -> headless")
            use_gui = False
    if use_gui:
        NASA_ControlPanel(root, tm)
        try:
            root.mainloop()
        except KeyboardInterrupt:
            pass
        rospy.signal_shutdown("fin GUI")
    else:
        rospy.loginfo("[Map] mode HEADLESS.")
        try:
            ctrl.join()
        except KeyboardInterrupt:
            rospy.signal_shutdown("Ctrl-C")
    tm._safe_stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

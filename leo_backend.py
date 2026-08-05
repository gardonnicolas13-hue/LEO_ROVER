#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LEO Rover — Headless ROS backend
=================================
Headless ROS node: multiprocess vision (brightness LED + checkerboard),
autonomous state machine (PATROL -> LOCK -> WAIT -> U_TURN), D455 obstacle
detection. Exposes everything to the browser via ROS instead of a local GUI.

  PUBLISHES
    /mission/telemetry        std_msgs/String  (JSON, ~10 Hz) — full state
    /mission/log              std_msgs/String  (one line per event)
    /mission/image_annotated  sensor_msgs/Image (BGR8) — served as MJPEG by
                              web_video_server (annotated video stream)
    /map/markers              std_msgs/String  (JSON, permanent markers)
    /cmd_vel                  geometry_msgs/Twist (motor commands)

  SUBSCRIBES
    /mission/command          std_msgs/String  (JSON) — orders from browser
    /camera/infra1/image_rect_raw   (D455 stereo IR, mono8 — color stream is
                                     broken on this unit, see CAM_TOPIC_DEFAULT)
    /firmware/wheel_odom, /firmware/battery, /firmware/wheel_states

  Browser interface:
    rosbridge_server   (ws://<host>:9090)   <- roslibjs (telemetry + commands)
    web_video_server   (http://<host>:8080) <- annotated MJPEG stream

  LAUNCH (on PC, ROS Noetic, connected to robot master):
    export ROS_MASTER_URI=http://10.0.0.1:11311
    export ROS_IP=$(hostname -I | awk '{print $1}')
    python3 leo_backend.py
  (see start_web.sh to launch everything: rosbridge + video + backend)

  JSON contract — /mission/command (sent by browser):
    {"action": "set_mode", "mode": "AUTO"|"MANUEL"}
    {"action": "stop"}
    {"action": "reset"}                 # software reset X/Y/Z to 0,0,0
    {"action": "clear_map"}             # erase trajectory + beacons + counter
    {"action": "set_params", "hue_low":80,"hue_high":135,"v_min":200,"minled":3}
    {"action": "manual", "lin":0.2, "ang":0.0}   # manual drive (deadman 0.5 s)
    {"action": "set_view", "mask": true|false}    # normal / HSV mask stream
"""

import os
import sys
import json
import math
import time
import datetime
import signal
import queue
import threading
import multiprocessing as mp
import random
from collections import deque

import numpy as np
import cv2

try:
    import psutil as _psutil
    _PSUTIL = True
except Exception:
    _PSUTIL = False

# ── Topics ────────────────────────────────────────────────────────────────────
ROBOT_IP_DEFAULT  = "10.0.0.1"
# D455 color stream is broken on this unit (RGB8 format negotiation failure,
# root-caused earlier this project) — infra1 (mono8/Y8) is the confirmed
# reliable feed, used throughout every calibration recording this session.
# Cockpit video is grayscale IR instead of color as a result; see _decode()
# below, which already handles mono8 via cv2.COLOR_GRAY2BGR.
# /pc/camera/... = local raw republish of the Pi's JPEG stream (see
# navigation_supervision.launch) - subscribing the robot's raw /camera/...
# from the PC saturates and can crash the WiFi AP.
CAM_TOPIC_DEFAULT = "/pc/camera/infra1/image_rect_raw"
ODOM_TOPIC        = "/firmware/wheel_odom"
# Sources comparées par la CALIBRATION TERRAIN (onglet Trajectory). On lit les
# trois estimateurs plus le gyro BRUT : le gyro seul isole l'échelle du
# CAPTEUR, là où les estimateurs mélangent gyro, roues et vision.
CALIB_WHEEL_TOPIC = "/wheel_odom_with_covariance"
CALIB_MINS_TOPIC  = "/mins/imu/odom"
CALIB_VINS_TOPIC  = "/ov_msckf/odomimu"
CALIB_IMU_TOPIC   = "/imu/data_clean"

# ── FIX 6-DOF CAROLUS (exigence superviseur, 2026-07-29) ────────────────────
# Le fix global de la balise, en LECTURE SEULE. La regle d'exclusivite du
# 08/07 interdit de PUBLIER sur ce topic (creneau prive du bridge MINS) ; s'y
# abonner ne l'affecte en rien et n'ajoute qu'un abonne — il n'en avait aucun.
# On ne republie donc RIEN ici : on lit, on convertit, on affiche.
CAROLUS_FIX_TOPIC = "/mins/external_ref/carolus"
# Au-dela de ce delai le fix est declare PERIME. Un fix de balise vieux de
# quelques secondes reste affichable, mais le presenter comme courant serait
# pire que ne rien afficher : l'operateur croirait la balise en vue.
CAROLUS_FIX_STALE_S = 3.0
# ── PERMUTATION D'AXES CAROLUS -> LEO (Lot B, v4, 2026-07-30) ───────────────
# Carolus publie sa pose dans le repere CAMERA ; le rover raisonne en
# base_link. La correspondance des LETTRES est etablie (doc Turki Yassin
# §10.2.5), seuls les SIGNES du quaternion restent a determiner :
#     X_leo = -Z_carolus     Y_leo = -X_carolus     Z_leo = -Y_carolus
# Sans cette conversion le panneau 6 DDL affiche des valeurs correctes mais
# dans le MAUVAIS REPERE — defaut introduit le 29/07 en cablant /pose brut.
CAROLUS_POS_PERM = (("z", -1.0), ("x", -1.0), ("y", -1.0))   # -> X, Y, Z leo
# Quaternion : lettres connues (qz, qx, qy, qw), signes a trouver parmi 16
# combinaisons (doc §13.2.2). Defaut = celle du code qui FONCTIONNE dans le
# document (tf2_code.py §13.1 : x=-q.z, y=+q.x, z=+q.y, w=+q.w).
CAROLUS_QUAT_LETTERS = ("z", "x", "y", "w")
CAROLUS_QUAT_SIGNS_DEFAULT = "-+++"

# ── LOT D : perte de balise (v4, 2026-07-30) ────────────────────────────────
# Inspire de `marker_timeout` (doc Turki Yassin §3.4) : un asservissement
# visuel qui ne voit plus sa cible continue sur sa derniere commande, donc
# fonce a l'aveugle. Deux niveaux, volontairement separes :
#   - SIGNALEMENT (toujours actif) : bandeau ambre au cockpit.
#   - ARRET (opt-in, defaut DESACTIVE) : n'agit que dans les etats qui
#     DEPENDENT de la balise. Couper la patrouille parce qu'aucune balise
#     n'est en vue serait absurde — elle en cherche precisement une.
BEACON_LOST_TIMEOUT_S = 2.0
BEACON_DEPENDENT_STATES = ("LOCK", "GOTO_BEACON", "WAIT")
# La campagne de calibration est ECRITE SUR DISQUE a chaque passe. Elle ne
# doit pas vivre uniquement en memoire : un redemarrage du backend (respawn,
# deploiement, watchdog) effacerait 9 passes de terrain que l'operateur a
# payees au metre ruban. Perdu une premiere fois le 29/07.
CALIB_STATE_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "logs", "calib_terrain.json")
CMD_TOPIC         = "/cmd_vel"

TELEMETRY_TOPIC   = "/mission/telemetry"
LOG_TOPIC         = "/mission/log"
COMMAND_TOPIC     = "/mission/command"
IMAGE_OUT_TOPIC   = "/mission/image_annotated"
VISION_TOPIC      = "/vision/targets"       # real-time detection targets (JSON)

# ── LED detection ─────────────────────────────────────────────────────────────
LED_HUE_LOW   = 80
LED_HUE_HIGH  = 135
LED_V_MIN     = 200
LED_S_MIN     = 10
LED_MIN_AREA   = 3.0    # min blob area — reject sub-pixel noise
LED_MAX_AREA   = 2500.0 # max blob area — reject large ceiling lights / reflections
# Bornes élargies (2026-07-24, demande opérateur : détection fluide de 0,5 à
# 4 m) — c'était le vrai goulot d'étranglement portée, pas l'AprilTag (déjà
# correctement dimensionné : tag réel 20,32 cm mesuré, tag_decimate=1.0
# pleine résolution, voir catkin_ws/src/leo_autonomy/config/{tags,apriltag}.yaml,
# non touché). L'aire apparente d'une LED physique suit une loi en 1/d² : sur
# 0,5-4 m (rapport 8×), l'aire varie d'un facteur 64 — aucune fenêtre [MIN,MAX]
# fixe ne peut border les deux bouts sans marge si elle est calée sur une
# seule distance de référence (l'ancien seuil bas=10 était calibré sur une
# image réelle À 1 M, cf. LED_BRIGHT_THRESH ci-dessous ; à 4 m un blob ~16×
# plus petit qu'à 1 m tombait sous ce seuil et disparaissait). Élargi plutôt
# que rendu adaptatif (pas de distance connue tant qu'aucune LED n'est
# détectée — problème de l'œuf et la poule) : la charge de rejet du bruit
# repose maintenant davantage sur LED_MIN_CIRC + le clustering densité
# (_cluster) + la validation spatiale LED/damier (2026-07-24,
# BEACON_SPATIAL_MAX_FRAC — ne couvre que le damier, pas le tag : le
# déclenchement du reset n'a de toute façon lieu qu'à courte portée, où
# c'est le damier qui fait foi) plutôt que sur l'aire seule. Premier réglage,
# PAS mesuré sur image réelle à 4 m — à vérifier en conditions réelles
# (item ouvert, cf. rapport).
LED_MIN_CIRC   = 0.45
LED_CLUSTER_PX = 280    # max spread of the LED cluster (beacon ≈ 165mm wide)
                         # — déjà large marge à 4 m (~16 px de spread réel,
                         # bien en-dessous de 280) : pas resserré par la portée
MIN_LIGHTS    = 3        # beacon = 4 LEDs; accept 3 (tolerates one missed LED)
N_LEDS        = 4        # keep the 4 brightest LEDs (beacon has exactly 4)

# ── Distance estimation (LED span, Carolus geometry 0.165 m) + D455 focal ─────
CAM_FX         = 384.65
BEACON_WIDTH_M = 0.165
BEACON_COOLDOWN = 3.0    # s — minimum interval between two beacon validations
# Dé-duplication carte : une balise (ou un obstacle) déjà cartographié à
# moins de ce rayon n'est PAS ré-ajouté — sans ça, chaque re-détection au
# même endroit incrémentait le compteur et empilait des marqueurs (pollution
# de la carte constatée sur le terrain le 2026-07-07).
BEACON_DEDUP_RADIUS_M   = 1.2
OBSTACLE_DEDUP_RADIUS_M = 0.8

# ── Camera calibration (D455, 640×480) ────────────────────────────────────────
# ATTENTION (audit 2026-07-31) : CAM_FX = 384.65 n'est la calibration mesuree
# d'AUCUN des deux flux. Valeurs relevees en direct sur camera_info :
#     couleur  fx=380.25  fy=379.26  cx=320.40  cy=243.72
#     infra1   fx=fy=386.79          cx=320.06  cy=236.77
# 384.65 est donc 1.16 % au-dessus du fx couleur et 0.55 % en dessous du fx
# infrarouge. Origine la plus plausible : une calibration faite du temps ou la
# detection de balise tournait sur le flux INFRAROUGE, conservee telle quelle
# au passage a la couleur. Le nom « colour sensor » de l'ancien commentaire
# etait donc trompeur.
#
# POURQUOI ON NE LA CHANGE PAS : cette constante propage dans des resultats
# deja publies (gain de boucle du controleur LOCK, estimation de portee, cf.
# rapport §Research Methodology et annexe B). La remplacer sans refaire ces
# calculs creerait une incoherence pire que l'approximation actuelle, qui
# reste bornee a ~1-2 % a moins de 3 m, bien en deca des autres sources
# d'erreur de la chaine balise.
#
# A NE PAS CONFONDRE avec le defaut de carolus_astrobee_rex (fx=644.12,
# cx=643.47, calibre en 1280x720 et applique a du 640x480), qui lui place le
# point principal HORS de l'image et gonfle toute portee de 69.3 %.
#
# Remplacer par la sortie de cv2.calibrateCamera sur votre mire physique pour
# une precision solvePnP maximale. Par defaut on utilise CAM_FX pour les deux
# focales et le centre optique au centre de l'image.
CAM_K = [
    [CAM_FX, 0.0,    320.0],   # [fx,  0, cx]
    [0.0,    CAM_FX, 240.0],   # [ 0, fy, cy]
    [0.0,    0.0,    1.0  ],   # [ 0,  0,  1]
]
CAM_D = [0.0, 0.0, 0.0, 0.0]  # [k1, k2, p1, p2] — D455 factory-calibrated, near zero
REPROJ_ERR_MAX = 2.0           # px — poses above this reprojection RMS are discarded
PNP_SKIP_MS    = 40.0          # ms — skip PnP on next frame if last run exceeded this

# ── Timing / rates ────────────────────────────────────────────────────────────
CB_SCALE       = 1.0     # checkerboard detection image scale factor — 1.0 = full 640×480
                          # (0.45 made the board too small at >0.5 m operating distance)
CONTROL_HZ     = 20      # Hz — drive control loop
MAX_PID_DT_S   = 3.0 / CONTROL_HZ  # s — au-delà, un PID.update() saute son
                          # terme D ce tick-là (tick anormalement lent,
                          # _control_loop n'a pas de garantie de cadence
                          # stricte) sans perdre l'intégrale (2026-07-22)
TELEMETRY_HZ   = 10      # Hz — JSON telemetry publish rate
PUBLISH_IMG_HZ = 20      # Hz — annotated image publish rate (web stream)
CENTER_HOLD_S  = 0.5     # s — target persistence (stable lock)
CAM_ALIVE_TIMEOUT = 2.0  # s — was 1.0 s; widened 2026-07-21 after AUTO mode was
                         # observed hard-stopping ("micro stops", fully immobile,
                         # confirmed live) on brief camera gaps under today's
                         # WiFi-only image path (report item 14). D455 nominal
                         # is 15 Hz (~67 ms/frame), so 2.0 s still stops well
                         # before the ~60-90 s sustained "muette" outages the
                         # watchdog itself handles — this only tolerates the
                         # shorter gaps that don't need a full recovery cycle.
DEPTH_ALIVE_TIMEOUT = 1.0  # s (2026-07-21) — the color/IR stream feeding
                         # cam_alive and the depth stream feeding obstacle
                         # detection are two INDEPENDENT WiFi-crossing topics
                         # (/pc/camera/infra1/... vs /pc/camera/depth/...);
                         # PATROL only ever checked the former. If depth
                         # specifically stalls (bandwidth contention is a
                         # documented, live risk this session) while IR keeps
                         # flowing, cam_alive stays true and nothing stopped
                         # forward motion — the robot drove into a wall it was
                         # facing head-on with no fresh obstacle data at all.
                         # Kept tighter than CAM_ALIVE_TIMEOUT on purpose: this
                         # gates collision safety, not just beacon detection —
                         # at ADVANCE_LIN (0.2 m/s) 1.0 s bounds blind travel
                         # to ~20 cm, comfortably inside OBSTACLE_DIST_MM's
                         # 600 mm trigger margin.
MANUAL_DEADMAN = 1.0     # s — no recent manual command → stop (safety).
                         # Was 0.5 s (sized for the wired/Ethernet ground
                         # station); widened 2026-07-21 once same-subnet
                         # Wi-Fi (both PC and operator device) made 0.5 s
                         # marginal against normal WebSocket jitter while the
                         # operator's own device roams between APs walking
                         # with the robot — see report_latex/Fusion_Campaign.tex
                         # §Field session, July 21.

# ── Vision pipeline ───────────────────────────────────────────────────────────
LED_DETECT_HZ      = 20.0  # Hz — LED detection rate (very fast, runs every frame)
LANDMARK_DETECT_HZ = 5.0   # Hz — checkerboard detection rate (expensive; rate-limited)
LED_STABLE_FRAMES  = 3     # consecutive valid LED frames before LANDMARK mode
LED_EMA_ALPHA      = 0.45  # EMA alpha for LED coordinates (higher = more responsive)
LM_EMA_ALPHA       = 0.60  # EMA alpha for landmark coordinates
MJPEG_MAX_W        = 640   # px — max width before JPEG encode
VISION_HYSTERESIS  = 8     # frames to hold last valid detection before clearing
LM_MISS_MAX        = 25    # consecutive landmark misses before returning to LED mode (~5s at 5Hz)
LED_BRIGHT_THRESH = 200  # seuil luminosité (0-255). REJEU SUR IMAGE RÉELLE
                         # (2026-07-13, 1 m, laser 0) : les LEDs de la balise
                         # culminent SOUS 240 en IR (0/4 à 240, 3/4 à 220,
                         # 4/4 à 200 en croix parfaite). Le rejet des
                         # plafonniers est le travail de la garde y ci-dessous,
                         # PAS du seuil — ne plus jamais le remonter pour ça.
LED_MAX_Y_FRAC    = 0.35 # les blobs au-dessus de 35 % de l'image sont rejetés :
                         # la balise est AU SOL (caméra ~niveau), ses LEDs sont
                         # toujours sous l'horizon ; tous les faux positifs mesurés
                         # étaient des plafonniers à y=38-160 px (8-33 %).
VISION_CONFIRM_FRAMES = 3  # consecutive valid LED frames required before firing RESET (anti-jitter)
# Validation spatiale LED/damier (2026-07-24, revue vision — gap réel trouvé) :
# _dual_beacon_confirmed() exigeait LED valide ET damier valide au même tick,
# mais jamais qu'ils soient au MÊME ENDROIT de l'image — un plafonnier validé
# par _detect_leds_bright (LED_MAX_Y_FRAC le laisse passer s'il est bas dans
# l'image, ex. reflet sur une vitre) PENDANT qu'un damier réel est détecté
# ailleurs dans le même frame (mur/poster à motif) satisferait les deux
# conditions indépendamment sans jamais être la même balise physique.
# Tolérance RELATIVE à la taille du damier détecté (pas un seuil pixel fixe) :
# l'empreinte apparente du damier ET l'écart LED-damier scalent ensemble avec
# la distance, un seuil fixe serait soit trop strict de près, soit inutile de
# loin.
BEACON_SPATIAL_MAX_FRAC = 1.5  # écart centre-LED/centre-damier toléré, en
                                # multiples de la largeur bbox du damier

# ── Enregistreur de trajectoires MINS vs openVINS (2026-07-27) ────────────────
# Bruit de fond léger : le backend bufferise en continu les poses BRUTES des
# deux estimateurs (+ le fix Carolus) pour l'export Matlab en un clic depuis le
# cockpit (bouton "Exporter Matlab"). Throttlé à TRAJ_HZ (pas besoin des 135 Hz
# de MINS pour une trajectoire), buffer borné -> mémoire négligeable
# (~TRAJ_MAXLEN×3 tuples). Complète — sans le remplacer — l'enregistrement
# rosbag hors-ligne (tools/record_trajectories.sh) pour l'analyse lourde.
TRAJ_HZ      = 20.0     # Hz d'échantillonnage stocké par source
TRAJ_MAXLEN  = 60000    # ~50 min à 20 Hz ; borne dure (deque) contre la fuite mémoire

# ── Enregistrement "robuste" Test 1/Test 2 (2026-07-27 soir) ────────────────
# Le buffer en mémoire ci-dessus (self._traj) ne survit PAS à un redémarrage
# de leo_backend.py — fatal pour un essai de terrain sous une tempête de
# redémarrages watchdog (retour opérateur direct : "le site se perd... j'ai
# des ennormes décalages"). Contrainte explicite : rester sur le site, pas de
# terminal. Solution : le clic Test 1/Test 2 déclenche un `rosbag record`
# DÉTACHÉ (tools/rosbag_daemon.sh, même double-fork que matlab_daemon.sh) —
# écrit sur disque au fil de l'eau, survit à un redémarrage du backend. L'état
# (test, préfixe du bag, PID) est persisté dans ROBUST_REC_STATE_FILE pour
# être retrouvé même après un redémarrage — jamais gardé QUE en mémoire.
_TOUT_ROOT = os.path.dirname(os.path.abspath(__file__))
ROBUST_REC_DIR        = os.path.join(_TOUT_ROOT, "data", "trajectories")
ROBUST_REC_STATE_FILE = os.path.join(ROBUST_REC_DIR, ".active_recording.json")
ROBUST_REC_DAEMON_SH  = os.path.join(_TOUT_ROOT, "tools", "rosbag_daemon.sh")
ROBUST_REC_BAG2CSV    = os.path.join(_TOUT_ROOT, "tools", "bag_to_csv.py")
# Carolus EXCLU : /mins/external_ref/carolus est privé au slot VICON de MINS
# (règle verrouillée 08/07, cf. leo_watchdog.sh "ALERTE EXCLUSIVITE") — un
# rosbag record qui s'y abonne compte comme abonné non-MINS, même chose que
# la violation déjà retirée de leo_backend.py plus tôt ce 27/07.
ROBUST_REC_TOPICS = [
    "/mins/imu/odom", "/ov_msckf/odomimu", "/robot_pose_fused",
    "/leo_navigation/pose_source", "/firmware/wheel_odom",
    # /tag_detections (2026-08-05) : les AprilTag posés aux coins du rectangle
    # sont enregistrés comme SIMPLE OBSERVATION, jamais appliqués à la pose.
    # C'est la distinction qui rend la mesure valide : si on recalait la pose
    # sur les coins, la trajectoire épouserait le rectangle PAR CONSTRUCTION et
    # ne mesurerait plus rien (le piège "lisse, reproductible et métriquement
    # dénué de sens" de §12.13). Enregistrées à côté, elles donnent au
    # contraire la position VRAIE à 4 instants connus : la différence avec la
    # pose estimée est l'erreur ABSOLUE, bien plus forte qu'une fermeture de
    # boucle, et c'est la mesure que l'annexe D signale comme jamais faite.
    # S'abonner ici est sans risque : contrairement au slot VICON de MINS
    # ci-dessus, /tag_detections n'a aucune règle d'exclusivité.
    "/tag_detections",
]
CAROLUS_LAUNCH = os.path.join(                     # source des params affichés
    os.path.dirname(os.path.abspath(__file__)),
    "carolus_ws", "src", "launch", "carolusBlobDetection.launch")
# Sous-ensemble des params Carolus exposés au cockpit (nom -> libellé court).
# Uniquement les leviers de détection réellement utiles à régler ; les
# intrinsèques caméra / topics ne sont PAS éditables (config figée).
CAROLUS_TUNABLE = {
    "min_circularity":     "Circularité min",
    "saturation_threshold": "Seuil saturation",
    "min_area":            "Aire min (px²)",
    "max_area":            "Aire max (px²)",
    "max_distance_lim":    "Distance max (mm)",
    "lb_hue":              "Teinte basse",
    "ub_hue":              "Teinte haute",
    "image_threshold":     "Seuil image",
}

# ── Operator safety / FSM ─────────────────────────────────────────────────────
HEARTBEAT_TIMEOUT  = 12.0  # s — failsafe -> MANUAL si le cockpit se tait en AUTO.
                           # 1.5 s était intenable : les navigateurs étranglent les
                           # timers des onglets CACHÉS (jusqu'à 1/min) — l'opérateur
                           # qui regarde Trajectory faisait tomber le failsafe toutes
                           # les 60 s. 12 s couvre refresh + throttling, et reste un
                           # vrai homme-mort (le front émet aussi sur RÉCEPTION de
                           # télémétrie, insensible au throttling — cf. app.js).
HEARTBEAT_STALE_S  = 5.0   # s — beyond this the browser is considered disconnected;
                             #     reset armed state so a fresh reconnect works cleanly
FSM_RESET_FLASH_S = 0.8  # s — duration the RESET_ODOMETRY state stays visible
# Mission FSM states (mutually exclusive, single source of truth for the UI)
FSM_MANUAL        = "MANUAL"
FSM_AUTO_PATROL   = "AUTO_PATROL"
FSM_AUTO_NOCAM    = "AUTO_NO_CAMERA"   # AUTO demandé mais caméra morte -> immobile
FSM_AUTO_NODEPTH  = "AUTO_NO_DEPTH"    # AUTO demandé mais depth mort -> immobile (2026-07-21)
FSM_LOCK_BEACON   = "LOCK_BEACON"
FSM_RESET_ODOM    = "RESET_ODOMETRY"
FSM_AVOID_OBST    = "AVOID_OBSTACLE"
FSM_GOTO_BEACON   = "GOTO_BEACON"

# ── GOTO_BEACON navigation ────────────────────────────────────────────────────
GOTO_BEACON_DIST_TOL  = 0.50  # m — declare arrived within this distance
GOTO_BEACON_LIN_SPEED = 0.15  # m/s — approach speed
GOTO_BEACON_ANG_SPEED = 0.40  # rad/s — turn speed during approach
GOTO_BEACON_ANG_TOL   = 0.12  # rad — heading error before forward motion

# ── Speeds ────────────────────────────────────────────────────────────────────
SEARCH_ANG    = 0.20     # rad/s — rotation de recherche (>= zone morte 0.12)
                         # 0.4 était trop rapide : flou de rotation sur l'IR
                         # 15 Hz -> la vision ratait la balise (terrain 07/07)
ADVANCE_LIN   = 0.2      # m/s — advance toward next beacon
# Polarité de traction (2026-07-08) : après le montage des roues rigides, le
# sens physique avant/arrière s'est retrouvé INVERSÉ par rapport au signe de
# cmd_vel.linear.x. Correction au POINT DE SORTIE UNIQUE (_publish), valable
# pour TOUS les modes (AUTO, MANUEL, RTB, GOTO, AVOID). Remettre à +1.0 si le
# câblage/firmware moteur est corrigé un jour.
DRIVE_POLARITY = -1.0
# ANG_POLARITY = +1 (2026-07-13, VÉRITÉ GYRO) : le remontage 180° inverse le
# linéaire (une inversion : sens de roulement) mais PAS le lacet (deux
# inversions — échange des côtés x sens de roulement — s'annulent). Mesuré au
# gyro : /cmd_vel -0.30 -> -0.248 physique (83 %). L'inversion « constatée »
# plus tôt venait du cap MINS corrompu en rotation par les landmarks SLAM
# (profondeur inobservable en pivot pur) — jamais du firmware. NE PAS re-flip
# sur la foi d'un cap ESTIMÉ : seul le gyro brut fait foi pour ce signe.
ANG_POLARITY = 1.0
# Trim de lacet (2026-07-21) : dérive gauche mesurée en conduite droite tout
# à fait indépendante du couple moteur (les 4 roues tournent en parfaite
# symétrie de commande, ratio D/G=1.0000 sur 536 échantillons — donc pas un
# défaut PID) mais avec un biais gyro réel +0.053 rad/s à ω_roue≈3.22 rad/s
# (289 échantillons à ω G/D symétrique <3%, gyro apparié) : effet PHYSIQUE
# (rayon effectif / friction roue droite ~1.6% supérieur), cf. rapport item 1
# (mesure roll-out définitive encore à faire). Correction provisoire au POINT
# DE SORTIE UNIQUE (_publish), proportionnelle à la vitesse commandée (le
# biais physique scale avec ω roue, pas une constante) : à retirer si la
# vraie mesure au réglet donne un rayon/entraxe corrigé dans config_wheel.yaml.
YAW_TRIM_PER_MPS = -0.0175 / 0.2012  # rad/s de correction par m/s de lin commandé
                         # (2026-07-21, bissection par retour opérateur qualitatif :
                         # plein -0.053 -> "trop" à DROITE ; -0.0265 -> "un peu
                         # trop encore" à DROITE ; -0.015 -> "très très
                         # légèrement" à GAUCHE (a franchi le zéro) -> ajusté
                         # entre les deux derniers points, proche de -0.015 car
                         # le dépassement à gauche y était minime.
                         # La mesure gyro automatisée pendant la conduite réelle
                         # est peu fiable ici : l'opérateur corrige naturellement
                         # la dérive ressentie, ce qui masque le résidu dans le
                         # gyro moyen — le retour qualitatif direct est plus sûr
                         # que cette mesure batch pour cet ajustement fin.
TARGET_KP     = 0.004    # centering gain (per pixel offset)
TARGET_STOP_M = 0.6      # m — stop threshold (robot close enough to beacon)
TURN_SPEED    = 0.35     # rad/s — rampe 2026-07-13 (batterie saine) : 97 %
                         # d'autorité jusqu'à 0.40 ; zone morte < ~0.12 rad/s.
                         # (Le « décrochage rouleaux » du 10/07 = batterie 10.3 V.)
POST_BEACON_TURN_RAD = math.radians(180)  # rotation après la pause balise
                                         # — demi-tour complet (2026-07-22,
                                         # remplace le 90° perpendiculaire de
                                         # la refonte 2026-07-08 : compromis
                                         # de couverture patrouille assumé,
                                         # explicitement demandé)
ADVANCE_SEEK_T = 6.0     # s — ADVANCE timeout before re-launching SCAN 360°

# ── Autonomous state machine (PATROL → LOCK → WAIT → U_TURN) ─────────────────
LOCK_ALIGN_SPEED  = 0.25     # rad/s — centrage LOCK (97 % autorité mesurée) ;
                             # réduit avec SEARCH_ANG (même contrainte de flou)
LOCK_CENTER_TOL   = 0.08     # FOV fraction — centering tolerance (±8%), entry
LOCK_CENTER_EXIT_TOL = 0.14  # FOV fraction (2026-07-21) — wider exit band:
                             # a single hard threshold made lock_centered
                             # chatter true/false when the detected centre
                             # jittered near ±8% (known-noisy LED/checkerboard
                             # detection), alternating advance/re-centre
                             # commands; entering "centered" still needs the
                             # tight ±8%, but only LEAVES it past ±14%.
LOCK_TIMEOUT      = 6.0      # s — alignment timeout on partial beacon
# Anti-artefact (2026-07-10) : une "cible" toujours visible mais jamais centrée
# malgré la rotation est solidaire de la caméra (reflet/plafonnier vu comme
# LEDs) — LOCK_TIMEOUT n'arme que si la cible DISPARAÎT, donc le robot
# tournait sur lui-même sans fin. Sans progrès de centrage -> abandon + avance.
LOCK_STALL_S      = 12.0     # s — LOCK sans centrage acquis = cible non atteignable
LOCK_COOLDOWN_S   = 45.0     # s — après abandon : avancer loin de l'artefact (25 s re-lockait le même plafonnier)
DUAL_CONFIRM_GRACE_S = 4.0   # s — une fois arrivé devant la balise (mesure ou
                             # depth), délai d'attente pour que LED ET damier
                             # soient confirmés ENSEMBLE avant d'abandonner
                             # sans enregistrer de balise (2026-07-22).
WAIT_DURATION     = 5.0      # s — pause sur place après balise (refonte
                             # 2026-07-08 : 30 s -> 5 s, spec patrouille simple)
UTURN_SPEED       = 0.35     # rad/s — pivots AVOID/ESCAPE/U-turn (rampe 13/07)
PATROL_ADVANCE_S  = 3.0      # s — advance duration between patrol scans
# ── Return To Base ────────────────────────────────────────────────────────────
RTB_DIST_TOL    = 0.08   # m — distance to origin to declare "at base"
RTB_OVERSHOOT   = 0.008  # m — overshoot detection threshold (8 mm)
RTB_ANG_TOL     = 0.18   # rad — heading correction during advance (≈10°)
RTB_LIN_SPEED   = 0.20   # m/s — full speed (dist > 0.40 m)
RTB_SLOW_DIST   = 0.40   # m — deceleration start
RTB_SLOW_SPEED  = 0.08   # m/s — slow speed (0.20–0.40 m)
RTB_CRAWL_DIST  = 0.20   # m — crawl start
RTB_CRAWL_SPEED = 0.03   # m/s — crawl speed (< 0.20 m)
RTB_ANG_SPEED   = 0.40   # rad/s — max steering correction rate while reversing
# Retraçage de trajectoire (2026-07-23, retour opérateur : RTB en ligne droite
# vers l'origine "ne suit pas du tout ses anciens emplacements") — trace en
# repère MONDE, séparée de self.traj (repère local, effacé à chaque reset) :
# elle doit survivre aux resets odométrie/LED exactement comme world_origin,
# sinon le retraçage se limite au dernier segment depuis le dernier reset.
RTB_TRAIL_SPACING = 0.25   # m — espacement mini entre deux points enregistrés
RTB_TRAIL_MAX     = 2000   # points — borne mémoire (~500 m d'historique)
# Précision RTB (2026-07-23, retour opérateur) : /robot_pose_fused (VINS ou
# MINS selon la sélection cockpit) plutôt que l'odométrie roues brute pour la
# trace et la cible RTB uniquement — voir _rtb_world_pos().
RTB_FUSED_MAX_AGE = 1.0    # s — au-delà, on retombe sur l'odométrie roues
# Garde de sécurité RTB (2026-07-23, MISSION CRITIQUE "le rover fonce dans
# les murs") : root cause confirmée par le journal (auto_entries.json) —
# 5 tentatives RTB consécutives (22/07 15:56 et 16:08, 23/07 12:41/12:49/12:53)
# jamais suivies d'un RTB_COMPLETE, contre 3/3 réussies après ce correctif.
# RTB recule en continu (marche arrière) SANS AUCUNE vérification de la
# profondeur ni du déplacement — la D455 regarde vers l'AVANT (même
# contrainte physique que ESCAPE_BACK_LIN, aucun capteur ne couvre
# l'arrière) et la branche RTB de _drive() ne consultait ni _obstacle_flag
# ni le détecteur de blocage générique (qui exclut déjà RTB de sa liste
# d'états couverts). Seul signal disponible dans le sens de marche :
# l'odométrie/pose fusionnée elle-même — si elle n'avance pas alors qu'un
# recul est commandé, c'est un contact physique.
RTB_STALL_WINDOW_S   = 2.0   # s — fenêtre d'immobilité avant arrêt d'urgence
RTB_STALL_MIN_DISP_M = 0.03  # m — déplacement mini attendu (RTB_CRAWL_SPEED
                             # × RTB_STALL_WINDOW_S ≈ 0.06 m — marge ~50 %)
# Gouverneur de résistance RTB (2026-07-24, retour opérateur : réagir "un
# peu avant" le blocage complet, pas seulement au moment du contact) —
# ADDITIF au double arrêt dur ci-dessus, ne le remplace pas. Fenêtre plus
# courte que RTB_STALL_WINDOW_S : compare la vitesse RÉELLEMENT observée
# (déplacement/temps) à la vitesse qui vient d'être commandée. Un roue qui
# patine/résiste contre un obstacle fait chuter ce ratio BIEN avant l'arrêt
# quasi total que détecte le garde RTB_STALL_* — on ralentit alors plutôt
# que d'attendre le blocage dur. Seul signal disponible dans le sens de
# marche réel (aucun capteur ne couvre l'arrière).
RTB_RESIST_WINDOW_S  = 0.6   # s — fenêtre d'évaluation (plus courte que
                             # RTB_STALL_WINDOW_S -> réaction plus rapide)
RTB_RESIST_RATIO_LO  = 0.35  # vitesse observée/commandée en dessous de
                             # laquelle le plancher de ralentissement s'applique
RTB_RESIST_RATIO_HI  = 0.75  # au-dessus : résistance jugée négligeable (bruit
                             # d'odométrie normal), pas de ralentissement
RTB_RESIST_MIN_SCALE = 0.35  # plancher — ne s'arrête jamais seul (l'arrêt dur
                             # reste le garde RTB_STALL_*/obstacle ci-dessus)
LANDMARK_SQUARE_M   = 0.020  # m — physical size of one checkerboard square
LANDMARK_COOLDOWN   = 5.0    # s — debounce between two landmark registrations
OBSTACLE_DIST_MM  = 600      # mm — depth threshold triggering obstacle alarm
# Gouverneur PID de vitesse (2026-07-22) — additif, ne remplace ni ne pilote
# le déclenchement binaire OBSTACLE_DIST_MM/le pivot ci-dessus, qui restent
# le filet de sécurité inchangé. Ralentit continûment ADVANCE_LIN à mesure
# que _corridor_p5_mm approche OBSTACLE_SLOWDOWN_MM, pour ne plus avancer à
# vitesse constante jusqu'au déclenchement dur.
OBSTACLE_SLOWDOWN_MM     = 1500  # mm — début de la zone de ralentissement (~0.9 m de rampe)
OBSTACLE_MIN_SPEED_SCALE = 0.3   # plancher — ne s'arrête jamais seul, ne fait que ralentir
OBSTACLE_GOV_KP          = 0.0012  # échelle/mm — volontairement plus raide que le point
                          # d'ajustement exact (~0.00078) : atteint le plancher un peu
                          # AVANT le seuil dur, marge de prudence pour un premier essai
                          # terrain, à assouplir si trop prudent en pratique
# ROI as width/height FRACTIONS (x0,x1,y0,y1) — the old pixel tuple assumed a
# hardcoded 848x480 depth frame; the minimal camera profile streams 424x240,
# and fractions survive any future resolution change.
OBSTACLE_ROI_FRAC = (0.20, 0.80, 0.333, 0.70)
# Géométrie (2026-07-10) : l'ancienne ROI (0.405-0.595 = ±8° soit ±6 cm à
# 45 cm) ne couvrait PAS le gabarit du rover (~±25 cm -> ±29° -> 60 % de la
# largeur image). Un obstacle hors axe passait sous le radar. Verticalement
# 0.70 max : à la hauteur caméra, le sol n'entre dans la bande qu'à ~0.73 m,
# au-delà du seuil 0.45 m — pas de faux positif sol.
# /pc/camera/... = local decompressed republish of the Pi's compressedDepth
# stream (navigation_supervision.launch) — same WiFi-saturation rule as the IR
# streams: never subscribe the robot's raw depth from the PC.
DEPTH_TOPIC       = "/pc/camera/depth/image_rect_raw"
# ── AVOID: contournement réactif d'obstacle ──────────────────────────────────
# Le décalage fixe 35° systématique (2026-07-24, demande opérateur initiale)
# est remplacé le même jour par un évitement VFH+ (Borenstein & Koren) après
# une seconde demande explicite : "évitement intelligent façon IA" — DWA/TEB
# écartés (supposent costmap + SLAM + un modèle cinématique différentiel
# propre ; ce rover n'a ni l'un ni l'autre, et son lacet dérivé des roues est
# déjà connu corrompu par le glissement des rouleaux mecanum, cf. rapport).
# VFH+ retenu : léger, réactif, adapté à un unique capteur avant, et
# construit directement sur l'histogramme polaire déjà calculé en continu
# dans _on_depth() (_sector_mm, percentile 20 — fix du jour contre le biais
# médiane). Voir _vfh_steer() ci-dessous pour la fonction de score.
AVOID_TURN_RAD    = math.radians(90)  # angle de repli si AUCUN secteur VFH+
                                      # n'est exploitable (voir BOUNCE_MIN/MAX_RAD)
AVOID_SIDE_S      = 3.0     # s — (héritage, plus utilisé par le pivot simple)
BOUNCE_MIN_RAD    = math.radians(45)  # repli AVOID sans secteur exploitable :
BOUNCE_MAX_RAD    = math.radians(135) # angle aléatoire borné (couverture ergodique)
SPIRAL_AFTER_S    = 45.0   # s sans balise vue -> spirale d'exploration
SPIRAL_ANG0       = 0.35   # rad/s initial de la spirale (rayon croissant :
SPIRAL_DECAY      = 0.08   #   ang = ANG0 / (1 + DECAY·t))
SPIRAL_MAX_S      = 30.0   # s — durée max d'une spirale avant retour ligne droite
STUCK_PIVOTS      = 4      # pivots dans la fenêtre -> coincé (niveau 1)
STUCK_WINDOW_S    = 40.0   # pivot 90° a ~0.34 rad/s réels = ~4.6 s ; 4 pivots
                           # + avancées tiennent dans 40 s
STUCK_MIN_DISP_M  = 0.15   # déplacement monde min sur la fenêtre (niveau 2)
STUCK_DISP_WIN_S  = 20.0   # un pivot légitime (~5 s immobile) ne doit pas
                           # déclencher un faux ESCAPE
ESCAPE_BACK_LIN   = -0.10  # m/s — MARCHE ARRIÈRE D'ÉCHAPPEMENT UNIQUEMENT :
ESCAPE_BACK_S     = 2.0    # la depth NE COUVRE PAS l'arrière -> lent et court
                           # (0,2 m < 0,3 m de limite absolue)
ESCAPE_TURN_MIN   = math.radians(120)  # grand pivot aléatoire post-recul
ESCAPE_TURN_MAX   = math.radians(180)
ESCAPE_MAX        = 2      # échappements dans la fenêtre -> ARRÊT SÉCURISÉ
ESCAPE_WINDOW_S   = 60.0
TAG_MEMORY_S      = 120.0  # mémoire de la dernière direction de balise aperçue
# ── VFH+ : évitement continu et anticipé (2026-07-24) ────────────────────────
# La bande de profondeur est découpée en 9 secteurs angulaires (~10° chacun
# sur ~87° de FOV depth, cf. _on_depth). _vfh_steer() note chaque secteur
# encore exploitable sur trois critères pondérés — ouverture, écart au but
# (0 = tout droit en patrouille -> "reste bien droit dans le couloir" ;
# cap balise mémorisé sinon), écart au cap précédemment choisi (hystérésis
# EXPLICITE dans le score, pas seulement en aval par EMA — c'est ce qui
# manquait à l'ancienne attraction "grand chemin" pour ne pas zigzaguer) —
# et retourne le secteur au meilleur score, ou None si aucun n'est
# exploitable (l'appelant doit alors se rabattre sur le filet de sécurité
# discret : déclenchement dur _obstacle_flag + pivot AVOID).
OPEN_SECTORS      = 9
VFH_MIN_CLEARANCE_M   = 1.0  # m — un secteur sous ce seuil est EXCLU des
                              # candidats (pas seulement pénalisé) : VFH+ ne
                              # choisit jamais une direction que le seuil dur
                              # (600 mm, ROI centrale) va de toute façon
                              # vetoer sous peu — pas de fausse promesse
VFH_OPEN_CAP_M        = 4.0  # m — au-delà, "plus profond" n'ajoute plus de
                              # score (évite qu'un secteur à 8 m écrase le
                              # terme but/continuité par pur artefact d'échelle)
VFH_W_OPEN            = 1.0  # poids du terme ouverture
VFH_W_GOAL            = 0.6  # poids du terme alignement-but
VFH_W_SMOOTH          = 0.5  # poids du terme continuité (hystérésis)
VFH_HEADING_EMA_ALPHA = 0.3  # lissage EMA du cap choisi (bruit résiduel
                              # entre secteurs adjacents à score proche)
VFH_AIM_MIN_RAD       = math.radians(15)  # sous cet écart le meilleur
                              # secteur est déjà "dans l'axe" — pas de pivot
                              # AVOID pour un si petit écart, sur-réaction
                              # inutile
OPEN_STEER_MAX    = 0.25   # rad/s — inflexion max du cap (PID _pid_patrol)
OPEN_STEER_KP     = 0.8    # gain (rad/s par rad d'écart angulaire)
OPEN_FRESH_S      = 0.7    # fraîcheur max de l'analyse secteurs
DEPTH_HFOV_RAD    = math.radians(87)  # FOV horizontal du flux depth D455
OPEN_BEARING_EMA_ALPHA = 0.3  # lissage du cap "grand chemin" (2026-07-21) —
                         # sans lissage, deux secteurs de profondeur voisins
                         # et proches en profondeur peuvent alterner d'une
                         # frame à l'autre (bruit capteur normal), faisant
                         # zigzaguer le cap de patrouille à chaque bascule.
DEPTH_LR_EMA_ALPHA = 0.4  # lissage médianes profondeur G/D (2026-07-21) —
                         # la direction de contournement (_enter_avoid) était
                         # choisie sur UNE SEULE frame instantanée ; un
                         # glissement ponctuel du capteur pouvait faire
                         # pivoter du mauvais côté.
AVOID_MAX_TRIES   = 3       # essais réactifs avant repli sur le demi-tour
MAP_MARKERS_TOPIC     = "/map/markers"
VELOCITY_CONFIG_TOPIC = "/leo_rover/config/velocity"
MAX_LIN_VEL_DEFAULT   = 1.0   # m/s — default maximum linear velocity
MAX_ANG_VEL_DEFAULT   = 2.0   # rad/s — default maximum angular velocity

# ── Checkerboard sizes tried by subprocess (best locked → instant detection) ──
# Primary target 5×10 squares = (4,9) inner corners — tried first.
# Larger patterns come before smaller ones: a size (N,M) can match a central
# sub-grid of a board that has MORE squares, so we prefer the largest match
# to cover the full beacon rather than locking onto a small central sub-pattern.
CHECKERBOARD_SIZES = [
    (4, 9), (9, 4),            # primary target — 5×10 squares
    (8, 8), (7, 8), (8, 7),   # large patterns first (avoid central sub-match)
    (7, 7), (6, 8), (8, 6),
    (6, 9), (9, 6), (7, 6), (6, 7),
    (6, 6), (5, 8), (8, 5),
    (5, 7), (7, 5), (5, 6), (6, 5),
    (5, 5), (4, 7), (7, 4),
    (4, 6), (6, 4), (4, 5), (5, 4),
    (4, 4),
]

# ── Multiprocessing context (spawn = fresh interpreter, no inherited ROS state) ─
_MP_CTX = mp.get_context('spawn')


def _mp_vision_worker(frame_q, result_q, stop_event, cfg):
    """Vision subprocess — pure cv2/numpy, zero ROS, runs on a dedicated CPU core.

    Receives compressed JPEG bytes from frame_q (maxsize=1 — frame dropping built-in).
    Sends detection result dicts back via result_q.

    Detection strategy:
      • LED mode  : brightness threshold (cv2.threshold) — LEDs are always the
                    brightest blobs; no HSV params required.
      • LANDMARK  : checkerboard (same algorithm as main process).
      • Mode switch: LED stable ≥ LED_STABLE_FRAMES → LANDMARK;
                     LANDMARK miss ≥ LM_MISS_MAX     → LED.
      • Prediction : hold last N valid LED positions for VISION_HYSTERESIS frames.
    """
    import cv2                                # type: ignore
    import numpy as np                        # type: ignore
    import math, time

    CAM_FX_v    = cfg['cam_fx']
    BEACON_W_v  = cfg['beacon_width_m']
    CB_SCALE_v  = cfg['cb_scale']
    LM_SQ_v     = cfg['landmark_square_m']
    SIZES_v     = cfg['checkerboard_sizes']
    BRIGHT_v    = cfg['bright_thresh']
    MIN_A_v     = cfg['led_min_area']
    MAX_A_v     = cfg['led_max_area']
    CLU_PX_v    = cfg['led_cluster_px']
    N_v         = cfg['n_leds']
    MINLED_v    = cfg['minled']
    STABLE_v    = cfg['led_stable_frames']
    MISS_MAX_v  = cfg['lm_miss_max']
    HYST_v      = cfg['vision_hysteresis']
    CB_PERIOD_v  = 1.0 / cfg.get('landmark_detect_hz', 5.0)  # s between CB runs
    CAM_K_v      = np.array(cfg.get('cam_k',
                       [[384.65,0.,320.],[0.,384.65,240.],[0.,0.,1.]]),
                       dtype=np.float64)
    CAM_D_v      = np.array(cfg.get('cam_d', [0.,0.,0.,0.]),
                       dtype=np.float64).reshape(-1, 1)
    REPROJ_MAX_v = float(cfg.get('reproj_err_max', 2.0))
    PNP_SKIP_v   = float(cfg.get('pnp_skip_ms', 40.0))

    # ── Local vision state ────────────────────────────────────────────────
    mode        = 'LED'
    led_stable  = 0
    lm_miss     = 0
    led_hyst    = 0
    lm_hyst     = 0
    last_lr     = None
    last_lm     = None
    cb_size     = None    # cached checkerboard size
    last_cb_t   = 0.0    # wall-time of last checkerboard computation (rate-limiter)
    lm_hits        = 0      # consecutive valid CB detections → confidence ramp
    lm_valid_ts    = 0.0    # wall-time of last successful CB detection
    CB_HOLD_S      = 0.5    # seconds: hold last valid pose before marking as lost
    led_valid_ts   = 0.0    # wall-time of last successful LED detection (2026-07-27)
    LED_HOLD_S     = 1.0    # seconds: bound on holding LED "valid" while in LANDMARK
                             # mode. Bug found live (opérateur, capture d'écran) : the
                             # LANDMARK branch below held the LAST real LED detection
                             # as "valid" UNCONDITIONALLY for as long as the system
                             # stayed in LANDMARK mode (up to LM_MISS_MAX=25 misses,
                             # ~5s, or longer if the checkerboard kept getting
                             # occasional hits) — a robot pointed at an EMPTY room
                             # still showed "4/4 LEDs, VALID BEACON" from a ghost of
                             # whatever it saw last. Combined with today's earlier fix
                             # (reset no longer requires "not _held"), a stale ghost
                             # could fire a false LED_RESET on a beacon that was no
                             # longer in view. Age-bounded like the checkerboard's own
                             # CB_HOLD_S, instead of an unconditional hold.
    _pnp_skip_next = False   # skip PnP on next CB hit when last run was overloaded
    _last_pnp_ms   = 0.0    # elapsed ms of most recent PnP computation

    # ── Brightness LED detection ───────────────────────────────────────────
    def _detect_leds_bright(bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        _, thr = cv2.threshold(gray, BRIGHT_v, 255, cv2.THRESH_BINARY)
        k   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thr = cv2.morphologyEx(thr, cv2.MORPH_CLOSE, k)
        cnts, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        leds = []
        MIN_CIRC = cfg.get('led_min_circ', LED_MIN_CIRC)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < MIN_A_v or area > MAX_A_v:
                continue
            perim = cv2.arcLength(c, True)
            if perim < 1.0:
                continue
            circ = (4.0 * math.pi * area) / (perim * perim)
            if circ < MIN_CIRC:
                continue   # reject elongated/asymmetric blobs
            # Use minimum-enclosing-circle centre — less shifted by asymmetric bloom
            (cx, cy), _ = cv2.minEnclosingCircle(c)
            # Plausibilité balise-au-sol : rejeter le tiers haut de l'image
            # (plafonniers/reflets hauts — la balise ne peut pas y être)
            if cy < LED_MAX_Y_FRAC * gray.shape[0]:
                continue
            leds.append((float(cx), float(cy), area))
        leds.sort(key=lambda x: -x[2])
        return leds[:N_v * 2]

    def _cluster(leds):
        if len(leds) < MINLED_v:
            return [], None
        # Density-based anchor: pick the blob that has the most neighbors within
        # CLU_PX_v — robust to false positives (ceiling lights, reflections) which
        # tend to be isolated, while the real beacon LEDs are a tight group.
        best_grp = []
        for ax, ay, _ in leds:
            grp = [(x, y, a) for x, y, a in leds
                   if math.hypot(x - ax, y - ay) <= CLU_PX_v]
            if len(grp) > len(best_grp):
                best_grp = grp
        if len(best_grp) < MINLED_v:
            return [], None
        # Keep only the N_v brightest (largest area) within the winning cluster
        best_grp = sorted(best_grp, key=lambda l: -l[2])[:N_v]
        cl = [(x, y) for x, y, _ in best_grp]
        cx = sum(p[0] for p in cl) / len(cl)
        cy = sum(p[1] for p in cl) / len(cl)
        return cl, (cx, cy)

    def _build_lr(bgr):
        leds_all = _detect_leds_bright(bgr)
        cluster, center = _cluster(leds_all)
        valid = len(cluster) >= MINLED_v
        bbox  = None
        if cluster:
            xs = [p[0] for p in cluster]; ys = [p[1] for p in cluster]
            pad = 18
            bbox = [min(xs)-pad, min(ys)-pad, max(xs)+pad, max(ys)+pad]
        dist = None
        if len(cluster) >= 2:
            span = max(p[0] for p in cluster) - min(p[0] for p in cluster)
            if span >= 1:
                dist = round(CAM_FX_v * BEACON_W_v / span, 2)
        return {
            "valid":       valid,
            "leds_pos":    [[round(float(x),1), round(float(y),1)] for x,y,_ in leds_all[:N_v]],
            "cluster_pos": [[round(float(x),1), round(float(y),1)] for x,y in cluster],
            "center":      [round(float(center[0]),1), round(float(center[1]),1)] if center else None,
            "bbox":        [round(float(v),1) for v in bbox] if bbox else None,
            "dist":        dist,
        }

    # ── Checkerboard detection ─────────────────────────────────────────────
    def _cb_fast(img, size):
        try:
            ok, c = cv2.findChessboardCorners(
                img, size, flags=(cv2.CALIB_CB_ADAPTIVE_THRESH |
                                  cv2.CALIB_CB_FAST_CHECK |
                                  cv2.CALIB_CB_NORMALIZE_IMAGE))
            return c if ok else None
        except Exception:
            return None

    def _cb_sb(img, size):
        try:
            ok, c = cv2.findChessboardCornersSB(img, size,
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            return c if ok else None
        except Exception:
            return None

    def _detect_cb(gray):
        """Three-stage checkerboard detection with sub-pixel refinement.

        Stage 1 — Fast path : SB on the cached size (~10-15 ms).
        Stage 2 — Discovery : full SB+fast scan over all known sizes.
        Stage 3 — Active Search : CLAHE contrast enhancement when stages 1-2
                  fail (degraded lighting, motion blur, heavy perspective).

        Returns (size, corners_f32, active_search: bool) or None.
        corners_f32 — shape (N,1,2), full-resolution, sub-pixel refined via
        cornerSubPix on the original gray (on top of findChessboardCornersSB's
        own Niblack/Sauvola pre-refinement).
        """
        nonlocal cb_size

        def _scan(img, fixed_size=None):
            """Return (size, raw_corners) or (None, None).
            NOTE: does NOT write cb_size — caller updates it after return."""
            if fixed_size is not None:
                c = _cb_sb(img, fixed_size) or _cb_fast(img, fixed_size)
                return (fixed_size, c) if c is not None else (None, None)
            for sz in SIZES_v:
                c = _cb_sb(img, sz) or _cb_fast(img, sz)
                if c is not None:
                    return sz, c
            return None, None

        small = cv2.resize(gray, None, fx=CB_SCALE_v, fy=CB_SCALE_v,
                           interpolation=cv2.INTER_AREA)
        # Light Gaussian blur removes JPEG 8×8 block artefacts without blurring corners
        blurred = cv2.GaussianBlur(small, (3, 3), 0)

        # Stage 1: fast path on cached size (try both raw and blurred)
        found_sz, c_raw = _scan(small, cb_size) if cb_size is not None else (None, None)
        if c_raw is None and cb_size is not None:
            found_sz, c_raw = _scan(blurred, cb_size)

        # Stage 2: full size discovery (raw first, then blurred)
        if c_raw is None:
            found_sz, c_raw = _scan(small)
        if c_raw is None:
            found_sz, c_raw = _scan(blurred)

        # Stage 3: Active Search — CLAHE-enhanced image
        active_search = False
        if c_raw is None:
            clahe     = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
            enhanced  = clahe.apply(small)
            found_sz, c_raw = _scan(enhanced, cb_size) if cb_size is not None else (None, None)
            if c_raw is None:
                found_sz, c_raw = _scan(enhanced)
            if c_raw is None:
                # Last resort: CLAHE + blur
                found_sz, c_raw = _scan(clahe.apply(blurred))
            if c_raw is not None:
                active_search = True

        if c_raw is None:
            cb_size = None
            return None

        cb_size = found_sz
        # Scale raw corners back to full-resolution space
        corners_full = np.asarray(c_raw, dtype=np.float32) / CB_SCALE_v

        # Sub-pixel refinement on full-resolution gray
        # (findChessboardCornersSB already refines at the downscaled level;
        #  a second pass on the full-res image squeezes out residual error)
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        try:
            corners_full = cv2.cornerSubPix(gray, corners_full, (5, 5), (-1, -1), crit)
        except Exception:
            pass  # keep unrefined corners on error

        return found_sz, corners_full, active_search

    def _build_lm(gray):
        """Build landmark detection result with full topology validation.

        Validates that the COMPLETE grid was detected (findChessboardCornersSB
        returns all N×M inner corners or nothing — partial results are not
        possible).  Adds:
          corners       — all inner corner positions (sub-pixel, full-res)
          outer_corners — the 4 extreme grid corners (for topology quad overlay
                          and perspective homography visualisation)
          active_search — True when CLAHE fallback was required
          confidence    — placeholder 0.0; caller fills it in based on
                          consecutive-hit count (see LANDMARK mode block)
        """
        result = _detect_cb(gray)
        if result is None:
            return {"valid": False, "checker_size": None, "center": None,
                    "bbox": None, "dist_est": None, "angle_deg": None,
                    "corners": [], "outer_corners": None,
                    "active_search": False, "confidence": 0.0}
        size, corners, active_search = result
        pts   = corners.reshape(-1, 2)
        n_cols, n_rows = size[0], size[1]   # inner corners per axis
        x0, y0 = float(pts[:, 0].min()), float(pts[:, 1].min())
        x1, y1 = float(pts[:, 0].max()), float(pts[:, 1].max())
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        pix_w  = x1 - x0
        phys_w = max(size[0], size[1]) * LM_SQ_v
        dist   = round(CAM_FX_v * phys_w / pix_w, 2) if pix_w > 2 else None
        angle  = round(math.degrees(math.atan2(cx - gray.shape[1] / 2.0, CAM_FX_v)), 1)

        # Four extreme inner corners: TL, TR, BL, BR
        outer = [
            [round(float(pts[0,            0]), 1), round(float(pts[0,            1]), 1)],
            [round(float(pts[n_cols - 1,   0]), 1), round(float(pts[n_cols - 1,   1]), 1)],
            [round(float(pts[(n_rows-1)*n_cols, 0]), 1) if (n_rows-1)*n_cols < len(pts) else x0,
             round(float(pts[(n_rows-1)*n_cols, 1]), 1) if (n_rows-1)*n_cols < len(pts) else y1],
            [round(float(pts[n_rows*n_cols - 1, 0]), 1), round(float(pts[n_rows*n_cols - 1, 1]), 1)],
        ]

        # Perspective-aware outer board boundary (physical edges = inner_corners ± 1 square).
        # Uses local step vectors at each corner so foreshortening is handled correctly.
        board_corners = None
        try:
            g = pts.reshape(n_rows, n_cols, 2)          # shape (rows, cols, 2)
            sR_TL = g[0, 1]  - g[0, 0]                 # rightward step — top row
            sD_TL = g[1, 0]  - g[0, 0]                 # downward step  — left col
            sR_TR = g[0, -1] - g[0, -2]
            sD_TR = g[1, -1] - g[0, -1]
            sR_BR = g[-1, -1] - g[-1, -2]
            sD_BR = g[-1, -1] - g[-2, -1]
            sR_BL = g[-1,  1] - g[-1,  0]
            sD_BL = g[-1,  0] - g[-2,  0]
            board_corners = [
                [round(float(g[0,  0][0] - sR_TL[0] - sD_TL[0]), 1),
                 round(float(g[0,  0][1] - sR_TL[1] - sD_TL[1]), 1)],   # outer TL
                [round(float(g[0, -1][0] + sR_TR[0] - sD_TR[0]), 1),
                 round(float(g[0, -1][1] + sR_TR[1] - sD_TR[1]), 1)],   # outer TR
                [round(float(g[-1,-1][0] + sR_BR[0] + sD_BR[0]), 1),
                 round(float(g[-1,-1][1] + sR_BR[1] + sD_BR[1]), 1)],   # outer BR
                [round(float(g[-1, 0][0] - sR_BL[0] + sD_BL[0]), 1),
                 round(float(g[-1, 0][1] - sR_BL[1] + sD_BL[1]), 1)],   # outer BL
            ]
        except Exception:
            board_corners = None

        return {
            "valid":         True,
            "checker_size":  list(size),
            "center":        [round(cx, 1), round(cy, 1)],
            "bbox":          [round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1)],
            "dist_est":      dist,
            "angle_deg":     angle,
            "corners":       [[round(float(p[0]), 1), round(float(p[1]), 1)] for p in pts],
            "outer_corners": outer,
            "board_corners": board_corners,
            "active_search": active_search,
            "confidence":    0.0,   # filled by caller
        }

    # ── 3D Pose Estimation ────────────────────────────────────────────────
    def _build_pose(corners, checker_size):
        """Independent pose estimation module — designed to be called only after
        a FULL checkerboard grid has been validated by findChessboardCornersSB.

        Uses ALL N×M inner corners (not just the 4 outer ones) for maximum
        solvePnP accuracy.  Image points are undistorted via undistortPoints
        before the solve (equivalent to cv2.undistort on the full frame but
        orders-of-magnitude cheaper — only the corner coordinates are corrected).

        Returns:
            dict  reliable=True  + distance_m, roll/pitch/yaw_deg, reprojection_err
            dict  reliable=False + reprojection_err  (RMS exceeds REPROJ_MAX_v)
            None                                      (bad input or exception)
        """
        try:
            n_cols = int(checker_size[0])
            n_rows = int(checker_size[1])
            n_pts  = n_cols * n_rows
            if n_cols < 2 or n_rows < 2 or len(corners) != n_pts:
                return None

            sq_mm = LM_SQ_v * 1000.0  # m → mm for numerical stability in solvePnP

            # Object points: Z=0 plane, world origin at TL inner corner.
            # Ordering matches findChessboardCornersSB (row-major, left-to-right).
            obj_pts = np.array([
                [float(c) * sq_mm, float(r) * sq_mm, 0.0]
                for r in range(n_rows)
                for c in range(n_cols)
            ], dtype=np.float64).reshape(-1, 1, 3)

            img_pts = np.array(corners, dtype=np.float64).reshape(-1, 1, 2)

            # Undistort image points — corrects lens distortion at corner locations
            # without processing the full frame (same geometric result, ~100× faster)
            img_pts_u = cv2.undistortPoints(img_pts, CAM_K_v, CAM_D_v, P=CAM_K_v)

            # Solve PnP — iterative Levenberg-Marquardt on all N×M corners
            _D0 = np.zeros((4, 1), dtype=np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts_u, CAM_K_v, _D0,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                return None

            # Reprojection error — RMS pixel residual between detected and re-projected
            proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, CAM_K_v, _D0)
            err_px = float(np.sqrt(np.mean(
                np.sum((proj.reshape(-1, 2) - img_pts_u.reshape(-1, 2)) ** 2, axis=1))))

            if err_px > REPROJ_MAX_v:
                return {"reliable": False,
                        "reprojection_err": round(err_px, 3)}

            # Distance — Euclidean norm of translation vector (mm → m)
            dist_m = round(float(np.linalg.norm(tvec)) / 1000.0, 3)

            # Rotation matrix → Tait-Bryan Euler angles (ZYX convention)
            R, _ = cv2.Rodrigues(rvec)
            sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
            if sy > 1e-6:           # standard case
                roll  = math.degrees(math.atan2( R[2, 1],  R[2, 2]))
                pitch = math.degrees(math.atan2(-R[2, 0],  sy))
                yaw   = math.degrees(math.atan2( R[1, 0],  R[0, 0]))
            else:                   # gimbal lock (pitch ≈ ±90°)
                roll  = math.degrees(math.atan2(-R[1, 2],  R[1, 1]))
                pitch = math.degrees(math.atan2(-R[2, 0],  sy))
                yaw   = 0.0

            # Translation in camera frame (mm → m)
            tx, ty, tz = [round(float(v) / 1000.0, 4) for v in tvec.flatten()]

            return {
                "reliable":         True,
                "distance_m":       dist_m,
                "roll_deg":         round(roll,   2),
                "pitch_deg":        round(pitch,  2),
                "yaw_deg":          round(yaw,    2),
                "reprojection_err": round(err_px, 3),
                "tvec_m":           [tx, ty, tz],   # [X, Y, Z] in camera frame
            }
        except Exception:
            return None

    # ── Main loop ──────────────────────────────────────────────────────────
    while not stop_event.is_set():
        try:
            item = frame_q.get(timeout=0.1)
        except Exception:
            continue
        if item is None:
            break
        # (seq, jpeg) historique, ou (seq, jpeg, params) depuis 2026-07-10 :
        # les curseurs du cockpit (seuil de luminosité, nb LED min) arrivent
        # ENFIN jusqu'au détecteur — avant, la config était figée au démarrage
        # du sous-processus et set_params ne réglait rien.
        live = None
        if len(item) == 3:
            seq, frame_bytes, live = item
        else:
            seq, frame_bytes = item
        if live:
            BRIGHT_v  = int(live.get('bright_thresh', BRIGHT_v))
            MINLED_v  = int(live.get('minled', MINLED_v))

        try:
            buf = np.frombuffer(frame_bytes, np.uint8)
            bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if bgr is None:
                continue
        except Exception:
            continue

        h, w = bgr.shape[:2]
        lr, lm = {}, {}
        now_t = time.time()

        if mode == 'LED':
            try:
                raw = _build_lr(bgr)
            except Exception:
                raw = {"valid": False}
            if raw.get("valid"):
                led_stable += 1; led_hyst = HYST_v; last_lr = raw; lr = raw
                led_valid_ts = now_t
            else:
                led_stable = 0
                if led_hyst > 0 and last_lr is not None:
                    led_hyst -= 1
                    lr = dict(last_lr); lr["_held"] = True
                else:
                    led_hyst = 0; last_lr = None; lr = raw
            if led_stable >= STABLE_v:
                mode = 'LANDMARK'; lm_miss = 0; last_cb_t = 0.0  # force immediate CB run

        else:  # LANDMARK
            # Hold last LED result — mais BORNÉ dans le temps (2026-07-27) :
            # au-delà de LED_HOLD_S sans réelle re-détection, on ne peut plus
            # affirmer que les LEDs sont toujours dans le champ (le robot a pu
            # bouger/pivoter). Passé ce délai, valid=False plutôt qu'un
            # maintien fantôme indéfini tant que le mode reste LANDMARK.
            if last_lr is not None and (now_t - led_valid_ts) < LED_HOLD_S:
                lr = dict(last_lr); lr["_held"] = True
            else:
                lr = {"valid": False}
            # Rate-limit checkerboard detection: expensive, run at LANDMARK_DETECT_HZ
            if now_t - last_cb_t >= CB_PERIOD_v:
                last_cb_t = now_t
                try:
                    gray   = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                    raw_lm = _build_lm(gray)
                except Exception:
                    raw_lm = {"valid": False}
                if raw_lm.get("valid"):
                    lm_miss    = 0
                    lm_hyst    = HYST_v
                    lm_hits   += 1
                    lm_valid_ts = now_t
                    raw_lm["confidence"] = round(min(1.0, lm_hits / 5.0), 2)

                    # ── 3D Pose Estimation (skip-frame when CPU-bound) ────
                    if not _pnp_skip_next:
                        _t0 = time.time()
                        raw_lm["pose3d"] = _build_pose(
                            raw_lm.get("corners", []),
                            raw_lm.get("checker_size", [6, 6]))
                        _last_pnp_ms   = (time.time() - _t0) * 1000.0
                        _pnp_skip_next = _last_pnp_ms > PNP_SKIP_v
                    else:
                        _pnp_skip_next = False
                        raw_lm["pose3d"] = (last_lm or {}).get("pose3d")

                    last_lm    = raw_lm
                    lm         = raw_lm
                    led_stable = STABLE_v + 1
                else:
                    lm_hits = 0
                    lm_miss += 1
                    age = now_t - lm_valid_ts
                    if last_lm is not None and age < CB_HOLD_S:
                        # Timestamp-based 500ms pose extrapolation
                        decay = 1.0 - age / CB_HOLD_S
                        lm = dict(last_lm)
                        lm["_held"]      = True
                        lm["confidence"] = round(last_lm.get("confidence", 0.5) * decay, 2)
                    elif lm_hyst > 0 and last_lm is not None:
                        lm_hyst -= 1
                        lm = dict(last_lm)
                        lm["_held"]      = True
                        lm["confidence"] = 0.0
                    else:
                        lm_hyst = 0
                    if lm_miss >= MISS_MAX_v:
                        mode = 'LED'; led_stable = 0; lm_miss = 0
                        lm_hits = 0; last_lm = None; lm = {}
            else:
                # Between CB checks: hold last valid landmark result with time-decay
                if last_lm is not None:
                    age   = now_t - lm_valid_ts
                    decay = max(0.0, 1.0 - age / CB_HOLD_S) if age < CB_HOLD_S else 0.0
                    lm = dict(last_lm)
                    lm["_held"]      = True
                    lm["confidence"] = round(last_lm.get("confidence", 0.5) * max(decay, 0.15), 2)

        result = {
            "seq": seq, "frame_w": w, "frame_h": h,
            "vision_mode": mode,
            "lm_miss":     lm_miss,
            "lm_hits":     lm_hits,
            "led_stable":  led_stable,
            "led_reset":    lr,
            "map_landmark": lm,
        }
        try:
            result_q.put_nowait(result)
        except Exception:
            try:    result_q.get_nowait()
            except Exception: pass
            try:    result_q.put_nowait(result)
            except Exception: pass


class _EMAFilter:
    """Exponential Moving Average for (x, y) pixel coordinates."""
    def __init__(self, alpha):
        self.alpha = alpha
        self.x = None
        self.y = None

    def reset(self):
        self.x = None
        self.y = None

    def update(self, x, y):
        if self.x is None:
            self.x, self.y = float(x), float(y)
        else:
            a = self.alpha
            self.x = a * float(x) + (1.0 - a) * self.x
            self.y = a * float(y) + (1.0 - a) * self.y
        return self.x, self.y


class PID:
    """Contrôleur PID générique (2026-07-22) — Kp/Ki/Kd, intégrale et sortie
    clampées séparément, dérivée lissée en interne (passe-bas fixe 50/50) :
    seul le signal de correction obstacle (_corridor_p5_mm) n'a pas déjà
    d'EMA amont dans ce fichier, ce lissage lui sert de filet.

    update(err, now) réutilise le `now` déjà calculé par _drive() — pas de
    time.time() supplémentaire. Premier appel après reset() (ou dt<=0,
    défensif) : P seul, pas de I/D sur un dt indéfini. dt anormalement grand
    (tick lent, > MAX_PID_DT_S) : intègre et applique P mais saute le terme D
    ce tick-là, sans perdre la mémoire d'intégrale (reset() s'en charge
    explicitement aux points de transition d'état — voir appelants)."""

    def __init__(self, Kp, Ki, Kd, out_min, out_max, i_min=None, i_max=None):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.out_min, self.out_max = out_min, out_max
        self.i_min = i_min if i_min is not None else out_min
        self.i_max = i_max if i_max is not None else out_max
        self.reset()

    def reset(self):
        self._integral   = 0.0
        self._prev_err   = None
        self._last_t     = None
        self._prev_deriv = 0.0
        self.last_err    = 0.0   # dernière erreur/sortie — lecture seule pour
        self.last_output = 0.0   # la télémétrie (2026-07-22), pas de recalcul

    def update(self, err, now):
        err = float(err)
        if self._last_t is None or (now - self._last_t) <= 0:
            self._prev_err = err
            self._last_t   = now
            out = max(self.out_min, min(self.out_max, self.Kp * err))
            self.last_err, self.last_output = err, out
            return out
        dt = now - self._last_t
        self._integral = max(self.i_min, min(self.i_max,
                              self._integral + err * dt))
        if dt <= MAX_PID_DT_S:
            raw_d = (err - self._prev_err) / dt
            self._prev_deriv = 0.5 * raw_d + 0.5 * self._prev_deriv
        # sinon : tick anormalement lent — saute le D ce tour-ci, garde le dernier lissé
        out = self.Kp * err + self.Ki * self._integral + self.Kd * self._prev_deriv
        out = max(self.out_min, min(self.out_max, out))
        self._prev_err = err
        self._last_t   = now
        self.last_err, self.last_output = err, out
        return out


class AutoLogbook:
    """Writes structured session events to web/auto_entries.json for the logbook UI."""

    MAX_ENTRIES = 200
    _write_lock = threading.Lock()

    def __init__(self, path):
        self.path = path
        self.entries = []
        self._load()

    def _load(self):
        try:
            with open(self.path, 'r') as f:
                self.entries = json.load(f).get('entries', [])
        except Exception:
            self.entries = []

    def add(self, event_type, title, details='', tags=None):
        entry = {
            "id":         f"auto_{len(self.entries):04d}",
            "timestamp":  datetime.datetime.now().isoformat(timespec='seconds'),
            "event_type": event_type,
            "title":      title,
            "details":    details,
            "tags":       tags or [],
            "auto":       True,
        }
        self.entries.insert(0, entry)
        if len(self.entries) > self.MAX_ENTRIES:
            self.entries = self.entries[:self.MAX_ENTRIES]
        with self._write_lock:
            try:
                os.makedirs(os.path.dirname(self.path), exist_ok=True)
                with open(self.path, 'w') as f:
                    json.dump({"entries": self.entries}, f, indent=2)
            except Exception:
                pass


class LeoBackend:
    def __init__(self):
        # --- État partagé entre threads ---
        self.connected   = False
        self._lock       = threading.Lock()
        self._tel_event  = threading.Event()   # wake up telemetry loop on demand
        self.latest_bgr  = None          # dernière image DÉCODÉE (BGR)
        self.img_t       = 0.0           # horodatage réception image

        # Buffer caméra 1-slot (le callback ROS n'est jamais bloqué)
        self._cam_msg   = None
        self._cam_seq   = 0
        self._dec_seq   = 0
        self._dec_proc      = 0
        self._closing       = False
        self.compressed = False  # CAM_TOPIC_DEFAULT is now infra1 (sensor_msgs/Image, mono8), not compressed

        # Paramètres de détection (réglés en direct par le navigateur)
        self._hue_low, self._hue_high = LED_HUE_LOW, LED_HUE_HIGH
        # _v_min pilote désormais le SEUIL DE LUMINOSITÉ du détecteur (le
        # pipeline est en luminosité pure sur l'infra — la teinte est un
        # vestige de l'ancienne caméra couleur, sans effet ici)
        self._v_min, self._minled     = LED_BRIGHT_THRESH, MIN_LIGHTS
        self.show_mask = False

        # Checkerboard size cache (updated from subprocess results)
        self.beacon_count = 0
        self._cb_size = None
        self._reset_srv = None

        # Navigation source (VINS/MINS), driven by leo_navigation/pose_selector
        # — best-effort, stays "N/A" if that node isn't running.
        self.pose_source = "N/A"
        self.pose_source_pending = ""
        self._pose_source_srv = None

        # Odométrie / carte
        self.origin   = None
        self.pose     = (0.0, 0.0, 0.0)
        self.raw_yaw  = None
        self.traj     = []
        # deque(maxlen=1000) plutôt que [] nu (2026-07-23, audit — item
        # Mineur C2) : évènements discrets réels, pas de haute fréquence,
        # mais toutes les autres structures du fichier qui grandissent en
        # continu utilisent déjà maxlen — bornage par cohérence, pas parce
        # qu'une session normale approcherait jamais 1000 balises.
        self.beacons  = deque(maxlen=1000)

        # Détection courante (remplie par le thread détection)
        self.det = {"leds": [], "near": [], "cluster": [], "checker": None,
                    "center": None, "led_center": None, "dist": None,
                    "tag": None, "mask": None}
        self.last_w = 0

        # Télémétrie carte électronique
        self.batt_v   = None
        self.wheels   = {"vel": [0]*4, "pwm": [0]*4, "torque": [0]*4}
        self.last_cmd = (0.0, 0.0)
        self._t = {"cam": 0.0, "odom": 0.0, "batt": 0.0, "wheels": 0.0}

        # Modes / commandes
        self.mode = "MANUEL"             # MANUEL | AUTO
        self.manual = [0.0, 0.0]
        self.manual_t = 0.0
        # Machine d'états AUTO : PATROL | LOCK | WAIT | U_TURN
        self.auto_state = "PATROL"
        self.scan_accum = 0.0
        self.scan_last_yaw = None
        self.adv_start = 0.0
        self.turn_accum = 0.0
        self.turn_last_yaw = None
        self.last_beacon_t = 0.0
        # PATROL : sous-phase (scan 360° ou avance)
        self._patrol_advancing = False
        # LOCK state
        self.lock_start_t = 0.0
        self.lock_centered = False
        self._dual_confirm_t0 = None  # ts: fenêtre de grâce LED+damier (2026-07-22)
        # WAIT state
        self.wait_start_t = 0.0
        # U_TURN state
        self.uturn_accum = 0.0
        self.uturn_last_yaw = None
        self._uturn_reason  = "patrol"   # "patrol" | "obstacle" — drives FSM display
        self._uturn_resume  = "PATROL"   # state to restore after U_TURN completes
        # AVOID state — contournement réactif d'obstacle (turn/sidestep/returnturn)
        self._avoid_phase    = "turn"
        self._avoid_resume   = "PATROL"  # état repris une fois l'obstacle contourné
        self._avoid_dir      = 1.0       # toujours +1 = gauche (2026-07-24, décalage fixe)
        self._avoid_accum    = 0.0
        self._avoid_last_yaw = None
        self._avoid_t0       = 0.0
        self._avoid_tries    = 0
        self._avoid_target   = AVOID_TURN_RAD   # angle du pivot AVOID courant
        self._vfh_prev_bearing = None   # cap VFH+ précédent (hystérésis + lissage EMA)
        # PID (2026-07-22) — voir classe PID et les 3 sites d'usage dans _drive()
        self._pid_patrol        = PID(Kp=OPEN_STEER_KP, Ki=0.0, Kd=0.1,
                                       out_min=-OPEN_STEER_MAX, out_max=OPEN_STEER_MAX)
        self._pid_lock_align    = PID(Kp=LOCK_ALIGN_SPEED, Ki=0.05, Kd=0.06,
                                       out_min=-LOCK_ALIGN_SPEED, out_max=LOCK_ALIGN_SPEED,
                                       i_min=-0.05, i_max=0.05)
        self._pid_lock_approach = PID(Kp=TARGET_KP, Ki=0.0, Kd=0.05,
                                       out_min=-0.3, out_max=0.3)
        self._pid_obstacle      = PID(Kp=OBSTACLE_GOV_KP, Ki=0.0, Kd=0.15,
                                       out_min=OBSTACLE_MIN_SPEED_SCALE - 1.0, out_max=0.0)
        self._last_obstacle_scale = 1.0  # caché pour la télémétrie (2026-07-22)
        # Couverture "aspirateur" : spirale / anti-coincement / mémoire balise
        self._avoid_times    = deque(maxlen=10)   # timestamps des pivots
        self._escape_times   = deque(maxlen=5)
        self._escape_phase   = "back"
        self._escape_t0      = 0.0
        self._escape_target  = math.pi
        self._escape_dir     = 1.0
        self._escape_accum   = 0.0
        self._escape_last_yaw = None
        self._spiral_t0      = 0.0
        self._spiral_dir     = 1.0
        self._last_beacon_seen_t = 0.0
        self._beacon_dir_world   = None            # cap monde vers la dernière balise aperçue
        self._disp_hist      = deque(maxlen=40)    # (t, wx, wy) — détection coincé niv. 2
        self._disp_last_t    = 0.0
        # GOTO_BEACON navigation target
        self._target_beacon_id  = None   # beacon index (1-based among beacons)
        self._target_beacon_pos = None   # (wx, wy) world coordinates
        # RTB state
        self._rtb_phase     = "TURN"   # TURN | DRIVE
        self._rtb_prev_dist = float('inf')
        self._rtb_trail     = []   # world-frame breadcrumb (wx, wy) — survit aux resets
        self._rtb_waypoints = []   # file de points restant à viser pendant un RTB actif
        self._rtb_stall_ref_pos = None  # (wx, wy) — référence détection de blocage RTB
        self._rtb_stall_ref_t   = 0.0
        self._rtb_resist_ref_pos = None  # (wx, wy) — référence gouverneur de résistance RTB
        self._rtb_resist_ref_t   = 0.0
        self._rtb_resist_ref_cmd = 0.0   # vitesse commandée au début de la fenêtre courante
        self._rtb_resist_scale   = 1.0   # facteur [RTB_RESIST_MIN_SCALE, 1.0] appliqué à lin
        self._rtb_last_lin_cmd   = 0.0   # dernière vitesse recul commandée (référence résistance)
        self._fused_x = 0.0    # /robot_pose_fused — VINS ou MINS selon la sélection
        self._fused_y = 0.0
        self._fused_yaw = 0.0
        self._fused_t = 0.0    # dernière réception — fraîcheur, voir _rtb_world_pos()
        # ── Operator heartbeat failsafe + FSM display ────────────────────────
        self._last_heartbeat_t = 0.0     # last UI heartbeat timestamp
        self._hb_armed         = False   # armed once the first heartbeat arrives
        self._hb_lost          = False   # true while failsafe is latched
        self._fsm_reset_t      = 0.0     # last odometry-reset event (RESET_ODOMETRY flash)
        self._led_confirm      = 0       # consecutive valid LED frames (anti-jitter)
        # Repère monde (survit aux resets odométriques locaux)
        self.world_origin_x = 0.0
        self.world_origin_y = 0.0
        self.world_heading  = 0.0
        # Détection d'obstacles (profondeur D455)
        self._obstacle_flag  = False
        self._obs_cooldown   = 0.0
        self._obs_count      = 0
        # Espace libre gauche/droite (médiane profondeur, mm) — choix du côté
        # de contournement. 8000 = "rien de proche" tant qu'aucune frame reçue.
        self._depth_left_mm  = 8000
        self._depth_right_mm = 8000
        # Marqueurs permanents publiés sur /map/markers — deque(maxlen=1000),
        # voir commentaire sur self.beacons ci-dessus (2026-07-23).
        self.map_markers = deque(maxlen=1000)
        self._map_pub     = None
        self._vision_pub  = None
        # Limites de vitesse configurables depuis le cockpit (sliders)
        self.max_lin_vel = MAX_LIN_VEL_DEFAULT
        self.max_ang_vel = MAX_ANG_VEL_DEFAULT

        # Persistance de la cible
        self._last_center = None
        self._last_center_t = 0.0
        self._last_landmark_t = 0.0   # anti-rebond enregistrement map landmark

        # Zero-lag multiprocess vision pipeline
        self._mp_frame_q       = _MP_CTX.Queue(maxsize=1)   # main → subprocess (JPEG bytes)
        self._mp_result_q      = _MP_CTX.Queue(maxsize=5)   # subprocess → main (results)
        self._mp_stop          = None    # mp.Event — set on shutdown
        self._mp_proc          = None    # vision subprocess handle
        self._vision_mode      = "LED"
        self._led_result       = {}
        self._landmark_result  = {}
        self._led_stable_count = 0
        self._lm_miss          = 0
        self._lm_hits          = 0
        self._led_stable       = 0
        self._led_ema          = _EMAFilter(LED_EMA_ALPHA)
        self._lm_ema           = _EMAFilter(LM_EMA_ALPHA)

        # Historiques / Hz / mission
        self.vel_meas    = (0.0, 0.0)
        self.cam_times   = deque(maxlen=120)
        self.odom_times  = deque(maxlen=120)
        self._mission_start = None
        self._img_pub_t  = 0.0

        # Enregistreur de trajectoires (MINS/VINS/Carolus) pour l'export Matlab.
        # Tuples (t_abs, x, y, z, qx, qy, qz, qw) ; t relatif calculé à l'export.
        self._traj_lock = threading.Lock()
        self._traj = {"mins": deque(maxlen=TRAJ_MAXLEN),
                      "vins": deque(maxlen=TRAJ_MAXLEN),
                      "carolus": deque(maxlen=TRAJ_MAXLEN)}
        self._traj_last = {"mins": 0.0, "vins": 0.0, "carolus": 0.0}
        self._last_export = None       # métadonnées du dernier export (-> télémétrie)
        # Sélecteur de test (2026-07-27, onglet Trajectory) : étiquette libre
        # ("test1"/"test2"/...) choisie côté cockpit avant un roulage, utilisée
        # comme préfixe des fichiers d'export au lieu du nom horodaté générique
        # — permet ex. "test1_mins.csv" / "test2_vins.csv" pour deux essais
        # MINS/VINS comparés ensuite dans le rapport. Purement une étiquette :
        # ne filtre ni ne bloque l'enregistrement (mins ET vins restent
        # toujours capturés simultanément, c'est tout l'intérêt de l'outil).
        self._current_test = "test1"
        # État de l'enregistrement robuste (rosbag détaché) en cours, s'il y
        # en a un — reconstruit depuis ROBUST_REC_STATE_FILE (pas seulement
        # initialisé vide) pour retrouver un enregistrement qui tournait déjà
        # AVANT ce redémarrage du backend, exactement le cas qu'il doit
        # couvrir (le double-fork le fait survivre au redémarrage lui-même ;
        # encore faut-il que leo_backend redémarré sache qu'il tourne).
        self._robust_rec = self._recover_robust_record_state()
        if self._robust_rec is not None:
            # self._current_test DOIT suivre l'enregistrement retrouvé, pas
            # rester au défaut "test1" — même bug de désync que
            # _set_current_test (27/07 soir), version "au redémarrage".
            self._current_test = self._robust_rec["test"]
        # Resultat du DERNIER lancement MATLAB GUI. Necessaire parce que le
        # processus est volontairement DETACHE (double-fork) : le backend n'a
        # aucun lien parent/enfant avec lui et ne peut donc pas recolter son
        # code de sortie. On surveille son journal a la place.
        self._matlab_launch = None
        self._carolus_params = self._load_carolus_params()
        # _drive_params NON charge ici : cette ligne s'execute AVANT
        # rospy.init_node() (~66 lignes plus bas), donc tout appel au serveur
        # de parametres leve et serait avale en None. Charge apres init_node,
        # puis re-tente si le robot n'etait pas encore la (voir _build_telemetry).
        self._drive_params = {"wheel_radius": None, "wheel_separation": None,
                              "angular_velocity_multiplier": None}

        _script_dir = os.path.dirname(os.path.abspath(__file__))
        self._autolog = AutoLogbook(os.path.join(_script_dir, 'web', 'auto_entries.json'))
        self._export_dir = os.path.join(_script_dir, 'web', 'exports')
        self._tools_dir  = os.path.join(_script_dir, 'tools')
        # MATLAB détecté ? (le binaire, pas l'API Python matlab.engine — absente
        # ici). Si présent, l'export peut ensuite lancer plot_trajectories.m en
        # tâche de fond via `matlab -batch`. Exposé au cockpit pour afficher/
        # masquer le bouton correspondant. Sonde le PATH ET les emplacements
        # d'install connus : le backend est lancé par rosmon avec un PATH
        # restreint (souvent sans /usr/local/bin), donc which() seul rate MATLAB.
        import shutil as _shutil, glob as _glob
        # R2025b explicitement PRÉFÉRÉE (2026-07-27, audit complet du bouton
        # Export — voir _launch_matlab_gui_worker) : deux versions coexistent
        # sur cette machine (R2025b, R2026a). R2026a plante de façon
        # intermittente au lancement headless (DISPLAY/XAUTHORITY reconstitués
        # depuis un process rosmon) sur un crash glibc confirmé par capture de
        # log — "Inconsistency detected by ld.so: ../elf/dl-tls.c:517:
        # _dl_allocate_tls_init: Assertion `listp != NULL' failed" — reproduit
        # à répétition, y compris avec LD_BIND_NOW. R2025b, testée dans les
        # mêmes conditions (30 s, plusieurs lancements), n'a JAMAIS craché.
        # Un tri "version la plus récente d'abord" avait été essayé ici puis
        # abandonné pour cette raison précise : "plus récent" ne veut pas
        # dire "fonctionne" dans ce contexte de lancement particulier.
        self._matlab_bin = None
        for _cand in ["/usr/local/MATLAB/R2025b/bin/matlab", "/usr/local/bin/matlab"]:
            if os.path.isfile(_cand) and os.access(_cand, os.X_OK):
                self._matlab_bin = _cand
                break
        if not self._matlab_bin:
            self._matlab_bin = _shutil.which("matlab")
        if not self._matlab_bin:
            for _cand in sorted(_glob.glob("/usr/local/MATLAB/*/bin/matlab")):
                if os.path.isfile(_cand) and os.access(_cand, os.X_OK):
                    self._matlab_bin = _cand
                    break

    # ══════════════════════════════════════════════════════════════════════ #
    # Démarrage ROS
    # ══════════════════════════════════════════════════════════════════════ #
    def run(self):
        # Valeurs ROS par défaut si l'environnement n'est pas déjà configuré
        os.environ.setdefault("ROS_MASTER_URI",
                              f"http://{ROBOT_IP_DEFAULT}:11311")
        os.environ.setdefault("ROS_IP", self._guess_ip())

        import rospy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Image, CompressedImage
        from std_msgs.msg import Float32, String
        from nav_msgs.msg import Odometry
        self._rospy = rospy
        self._Twist = Twist
        self._Image = Image
        self._Compressed = CompressedImage
        self._String = String
        self._Odometry = Odometry
        try:
            from leo_msgs.msg import WheelOdom, WheelStates
            self._WheelOdom = WheelOdom
            self._WheelStates = WheelStates
        except Exception:
            self._WheelOdom = None
            self._WheelStates = None

        rospy.init_node("leo_backend", anonymous=False, disable_signals=True)
        # Le serveur de parametres n'est joignable qu'apres init_node.
        self._drive_params = self._load_drive_params()
        # Lot D : seuil de perte + arret optionnel.
        self._beacon_lost_timeout = float(rospy.get_param(
            "~beacon_lost_timeout", BEACON_LOST_TIMEOUT_S))
        self._beacon_estop = bool(rospy.get_param("~beacon_lost_estop", False))
        self._log("Perte de balise : seuil %.1f s, arrêt automatique %s"
                  % (self._beacon_lost_timeout,
                     "ARMÉ" if self._beacon_estop else "désarmé"))

        # --- Publishers ---
        self.cmd_pub  = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)
        self.tel_pub  = rospy.Publisher(TELEMETRY_TOPIC, String, queue_size=2)
        self.log_pub  = rospy.Publisher(LOG_TOPIC, String, queue_size=10)
        self.img_pub  = rospy.Publisher(IMAGE_OUT_TOPIC, Image, queue_size=1)
        self._map_pub    = rospy.Publisher(MAP_MARKERS_TOPIC, String, queue_size=10)
        self._vision_pub = rospy.Publisher(VISION_TOPIC, String, queue_size=1)

        # --- Subscribers ---
        ctype = CompressedImage if self.compressed else Image
        rospy.Subscriber(CAM_TOPIC_DEFAULT, ctype, self._on_image,
                         queue_size=1, buff_size=2**24, tcp_nodelay=True)
        if self._WheelOdom is not None:
            rospy.Subscriber(ODOM_TOPIC, self._WheelOdom, self._on_odom,
                             queue_size=20, tcp_nodelay=True)
        # RTB de précision (2026-07-23) : /robot_pose_fused = sortie de
        # pose_selector, reflète déjà VINS ou MINS selon la sélection cockpit
        # (voir pose_selector.py). Utilisé UNIQUEMENT pour la trace/cible RTB
        # (voir _rtb_world_pos()) — le pilotage PATROL/LOCK/AVOID et le PID
        # restent sur l'odométrie roues ci-dessus, qui elle ne dépend d'aucun
        # estimateur et ne peut jamais tomber en panne d'initialisation.
        rospy.Subscriber("/robot_pose_fused", Odometry, self._on_pose_fused,
                         queue_size=10, tcp_nodelay=True)
        rospy.Subscriber("/firmware/battery", Float32, self._on_battery,
                         queue_size=5, tcp_nodelay=True)
        if self._WheelStates is not None:
            rospy.Subscriber("/firmware/wheel_states", self._WheelStates,
                             self._on_wheels, queue_size=5, tcp_nodelay=True)
        # ── CALIBRATION TERRAIN ──────────────────────────────────────────
        self._calib_last = {}          # derniere pose de chaque source
        self._calib_gyro_yaw = 0.0     # integrale de wz (cap gyro pur)
        self._calib_imu_t = None
        self._calib = {"mode": "rotation", "sens": "gauche", "state": "idle",
                       "passes": [], "sets": {}, "start": None, "end": None,
                       "preview": None, "result": None, "refus": None}
        try:
            from nav_msgs.msg import Odometry as _Odom
            from sensor_msgs.msg import Imu as _Imu
            for _nom, _top in (("roues", CALIB_WHEEL_TOPIC),
                               ("MINS",  CALIB_MINS_TOPIC),
                               ("VINS",  CALIB_VINS_TOPIC)):
                rospy.Subscriber(_top, _Odom, self._calib_on_odom(_nom),
                                 queue_size=5, tcp_nodelay=True)
            rospy.Subscriber(CALIB_IMU_TOPIC, _Imu, self._calib_on_imu,
                             queue_size=50, tcp_nodelay=True)
        except Exception as _e:
            self._log(f"Calibration terrain indisponible : {_e}")
        # ── Fix 6-DOF Carolus, lecture seule (voir CAROLUS_FIX_TOPIC) ────
        self._caro_fix = None          # dict des 6 DDL + horodatage, ou None
        self._beacon_estop_fired = False   # arret deja declenche (anti-repetition)
        self._beacon_lost_since = None
        # Lot B : conversion de repere. `apply=False` restitue le comportement
        # du 29/07 (valeurs BRUTES camera) pour pouvoir comparer les deux.
        self._caro_apply_perm = bool(rospy.get_param(
            "~carolus_apply_permutation", True))
        sg = str(rospy.get_param("~carolus_quat_signs",
                                 CAROLUS_QUAT_SIGNS_DEFAULT))
        if len(sg) != 4 or any(c not in "+-" for c in sg):
            self._log(f"carolus_quat_signs invalide ({sg!r}) — "
                      f"repli sur {CAROLUS_QUAT_SIGNS_DEFAULT}")
            sg = CAROLUS_QUAT_SIGNS_DEFAULT
        self._caro_quat_signs = sg
        try:
            from geometry_msgs.msg import PoseStamped as _PoseStamped
            # Topic parametrable : le defaut est le creneau reel, mais on
            # peut le detourner vers un topic bac a sable pour valider la
            # conversion RPY et la fraicheur SANS publier sur le creneau
            # prive du bridge MINS (y injecter un faux fix global corromprait
            # l'estimateur — regle d'exclusivite du 08/07).
            _cft = rospy.get_param("~carolus_fix_topic", CAROLUS_FIX_TOPIC)
            rospy.Subscriber(_cft, _PoseStamped,
                             self._on_carolus_fix, queue_size=5)
            self._log("Fix Carolus 6-DOF : écoute sur " + _cft
                      + " (lecture seule)")
        except Exception as _e:
            self._log(f"Fix Carolus indisponible : {_e}")
        self._calib_load()
        # Ancre la serie courante dans sets des le demarrage, pour que la
        # toute premiere passe soit publiee et sauvegardee comme les autres.
        self._calib["passes"] = self._calib["sets"].setdefault(
            self._calib_key(self._calib), self._calib["passes"])
        rospy.Subscriber(COMMAND_TOPIC, String, self._on_command, queue_size=20)
        rospy.Subscriber(VELOCITY_CONFIG_TOPIC, String, self._on_velocity_config, queue_size=5)
        # AprilTag = détecteur de balise LONGUE PORTÉE (le tag 20 cm se décode
        # à 4-6 m là où le damier 640x480 meurt vers 1,5 m). Alimente
        # _target_center comme repli du pipeline vision — la patrouille LOCK
        # sur le tag de loin, le damier+LED confirment de près (reset).
        try:
            from apriltag_ros.msg import AprilTagDetectionArray as _TagArray
            rospy.Subscriber("/tag_detections", _TagArray, self._on_tag_detections,
                             queue_size=2)
            self._log("AprilTag long-range detection: ACTIF (/tag_detections)")
        except ImportError:
            self._log("apriltag_ros absent — détection longue portée désactivée")
        # Navigation pose-source status (leo_navigation/pose_selector) — best-effort.
        try:
            rospy.Subscriber("/leo_navigation/pose_source", String,
                             self._on_pose_source, queue_size=2)
        except Exception:
            pass
        # Source armée mais pas encore appliquée (2026-07-24, voir
        # pose_selector.py _switch_to()) — relayée telle quelle au cockpit.
        try:
            rospy.Subscriber("/leo_navigation/pose_source_pending", String,
                             self._on_pose_source_pending, queue_size=2)
        except Exception:
            pass
        # Enregistreur de trajectoires (2026-07-27) : poses BRUTES des deux
        # estimateurs + fix Carolus, pour l'export Matlab. Best-effort, jamais
        # bloquant si une source est absente (VINS pas encore initialisé, etc.).
        for _topic, _src in (("/mins/imu/odom", "mins"),
                             ("/ov_msckf/odomimu", "vins")):
            try:
                rospy.Subscriber(_topic, Odometry, self._on_traj_odom,
                                 callback_args=_src, queue_size=5)
            except Exception:
                pass
        # PAS d'abonnement à /mins/external_ref/carolus ici : topic PRIVÉ au
        # slot VICON de MINS (règle verrouillée 2026-07-08, cf. leo_watchdog.sh
        # "ALERTE EXCLUSIVITE") — un abonné hors mins_subscribe déclenchait
        # cette alerte en continu (retiré 2026-07-27). La colonne "carolus" de
        # l'export Matlab reste en place (best-effort, vide sans source).
        # Profondeur D455 — détection obstacles (best-effort, ne bloque pas si absent)
        try:
            rospy.Subscriber(DEPTH_TOPIC, Image, self._on_depth,
                             queue_size=1, buff_size=2**24, tcp_nodelay=True)
        except Exception:
            pass

        # Service de RESET MATÉRIEL optionnel (best-effort, jamais bloquant)
        try:
            from std_srvs.srv import Trigger
            rospy.wait_for_service("firmware/reset_odometry", timeout=2.0)
            self._reset_srv = rospy.ServiceProxy("firmware/reset_odometry",
                                                 Trigger)
        except Exception:
            self._reset_srv = None

        # Service de switch VINS/MINS (leo_navigation/pose_selector), optionnel
        try:
            from std_srvs.srv import SetBool
            rospy.wait_for_service("/pose_selector/set_source", timeout=2.0)
            self._pose_source_srv = rospy.ServiceProxy("/pose_selector/set_source",
                                                        SetBool)
        except Exception:
            self._pose_source_srv = None

        self.connected = True
        self._mission_start = time.time()
        print(f"[ROS] Subscriber active: {COMMAND_TOPIC} ready", flush=True)
        self._log("LEO backend started — headless vision + control online.")

        # --- Threads de travail ---
        for fn, name in ((self._decode_loop,        "decode"),
                         (self._mp_result_loop,     "vision-result"),
                         (self._control_loop,       "control"),
                         (self._publish_image_loop, "image"),
                         (self._telemetry_loop,     "telemetry")):
            threading.Thread(target=fn, name=name, daemon=True).start()
        self._start_vision_process()

        signal.signal(signal.SIGINT, lambda *_: self.shutdown())
        signal.signal(signal.SIGTERM, lambda *_: self.shutdown())
        rospy.spin()

    def _guess_ip(self):
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((ROBOT_IP_DEFAULT, 1))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    def shutdown(self):
        self._closing = True
        # Terminate vision subprocess cleanly
        try:
            if self._mp_stop is not None:
                self._mp_stop.set()
        except Exception:
            pass
        try:
            if self._mp_proc is not None and self._mp_proc.is_alive():
                self._mp_proc.join(timeout=2.0)
                if self._mp_proc.is_alive():
                    self._mp_proc.terminate()
        except Exception:
            pass
        try:
            self._safe_stop()
        except Exception:
            pass
        try:
            self._rospy.signal_shutdown("arrêt backend")
        except Exception:
            pass
        sys.exit(0)

    # ══════════════════════════════════════════════════════════════════════ #
    # Journal
    # ══════════════════════════════════════════════════════════════════════ #
    def _log(self, text):
        line = f"[{time.strftime('%H:%M:%S')}] {text}"
        try:
            self.log_pub.publish(self._String(data=line))
        except Exception:
            pass
        print(line, flush=True)

    # ══════════════════════════════════════════════════════════════════════ #
    # Callbacks ROS (ultra-légers : on stocke, on ne calcule pas ici)
    # ══════════════════════════════════════════════════════════════════════ #
    def _on_image(self, msg):
        t = time.time()
        with self._lock:
            self._cam_msg = msg
            self._cam_seq += 1
            self.img_t = t
            self._t["cam"] = t
            self.cam_times.append(t)

    def _decode(self, msg):
        if self.compressed:
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                                cv2.IMREAD_COLOR)
        enc = msg.encoding.lower()
        a = np.frombuffer(msg.data, np.uint8)
        if enc == "rgb8":
            return cv2.cvtColor(a.reshape(msg.height, msg.width, 3),
                                cv2.COLOR_RGB2BGR)
        if enc == "bgr8":
            return a.reshape(msg.height, msg.width, 3)
        if enc == "mono8":
            return cv2.cvtColor(a.reshape(msg.height, msg.width),
                                cv2.COLOR_GRAY2BGR)
        return None

    def _on_odom(self, msg):
        try:
            px, py, yaw = float(msg.pose_x), float(msg.pose_y), float(msg.pose_yaw)
        except Exception:
            return
        # Compensation de repère (2026-07-08, roues rigides) : le firmware
        # considère « l'avant » à 180° de la caméra (mesuré : -0.186 m le long
        # de son cap pour une impulsion avant confirmée côté caméra). On
        # bascule le cap dans le repère CAMÉRA ici, au point d'entrée unique :
        # trajectoire locale, RTB, world_heading et bearings deviennent tous
        # cohérents avec « avant = caméra ». (Positions px/py inchangées :
        # elles sont vraies dans l'espace, seul le sens du cap différait.)
        yaw = math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi))
        now = time.time()
        self._t["odom"] = now
        self.odom_times.append(now)
        try:
            vlin, vang = float(msg.velocity_lin), float(msg.velocity_ang)
            self.vel_meas = (vlin, vang)
        except Exception:
            pass
        self.raw_yaw = yaw
        if self.origin is None:
            self.origin = (px, py, yaw)
        ox, oy, oyaw = self.origin
        dx, dy = px - ox, py - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        rx = c * dx - s * dy
        ry = s * dx + c * dy
        ryaw = math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw))
        self.pose = (rx, ry, ryaw)
        if not self.traj or math.hypot(rx - self.traj[-1][0],
                                       ry - self.traj[-1][1]) > 0.02:
            self.traj.append((rx, ry))
            if len(self.traj) > 5000:
                self.traj.pop(0)
        # Trace RTB (repère monde — voir constantes RTB_TRAIL_*) : indépendante
        # de self.traj, jamais effacée par un reset odométrie/LED. Position
        # de précision (VINS/MINS via pose_selector si frais, sinon roues).
        wx, wy, _ = self._rtb_world_pos()
        if not self._rtb_trail or math.hypot(wx - self._rtb_trail[-1][0],
                                              wy - self._rtb_trail[-1][1]) > RTB_TRAIL_SPACING:
            self._rtb_trail.append((wx, wy))
            if len(self._rtb_trail) > RTB_TRAIL_MAX:
                self._rtb_trail.pop(0)

    def _on_battery(self, msg):
        try:
            self.batt_v = float(msg.data)
            self._t["batt"] = time.time()
        except Exception:
            pass

    def _on_wheels(self, msg):
        try:
            self.wheels = {"vel": list(msg.velocity),
                           "pwm": list(msg.pwm_duty_cycle),
                           "torque": list(msg.torque)}
            self._t["wheels"] = time.time()
        except Exception:
            pass

    def _on_pose_source(self, msg):
        self.pose_source = msg.data

    def _on_pose_source_pending(self, msg):
        self.pose_source_pending = msg.data or ""

    # ══════════════════════════════════════════════════════════════════════ #
    # Enregistreur de trajectoires MINS/VINS/Carolus (export Matlab)
    # ══════════════════════════════════════════════════════════════════════ #
    def _traj_push(self, src, stamp, p, q):
        """Ajoute un échantillon throttlé à TRAJ_HZ. p=position, q=orientation."""
        now = time.time()
        if now - self._traj_last[src] < 1.0 / TRAJ_HZ:
            return
        self._traj_last[src] = now
        t = stamp.to_sec() if stamp and stamp.to_sec() > 0 else now
        self._traj[src].append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def _on_traj_odom(self, msg, src):
        self._traj_push(src, msg.header.stamp,
                        msg.pose.pose.position, msg.pose.pose.orientation)

    @staticmethod
    def _yaw_from_quat(qx, qy, qz, qw):
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny, cosy)

    @staticmethod
    def _rpy_from_quat(qx, qy, qz, qw):
        """Roll/Pitch/Yaw (rad) — convention ZYX standard."""
        sinr = 2.0 * (qw * qx + qy * qz)
        cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr, cosr)
        sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        pitch = math.asin(sinp)
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny, cosy)
        return roll, pitch, yaw

    def _traj_reset(self):
        with self._traj_lock:
            for d in self._traj.values():
                d.clear()
        for k in self._traj_last:
            self._traj_last[k] = 0.0
        self._discard_robust_record()
        # NE PAS relancer automatiquement sous la même étiquette (corrigé
        # 2026-07-28, blocage signalé en direct) : ça rendait le passage à un
        # AUTRE test IMPOSSIBLE — Reset relançait test1, donc test2 restait
        # refusé, et le seul chemin vers test2 passait par Export (pas du tout
        # évident). On laisse maintenant l'enregistrement ARRÊTÉ : l'opérateur
        # choisit librement Test 1 ou Test 2, et le clic démarre le nouvel
        # enregistrement.
        self._log("Enregistrement trajectoires remis à zéro — "
                  "choisis Test 1 ou Test 2 pour démarrer le prochain essai")

    def _set_current_test(self, test, finalize=False):
        """Étiquette du run en cours (ex. "test1"/"test2"), utilisée comme
        préfixe de fichiers par _export_matlab_worker. Best-effort : une
        étiquette vide/invalide est ignorée (garde la précédente) plutôt que
        de casser un export en cours avec un préfixe vide.

        Démarre AUSSI un enregistrement robuste (rosbag détaché) pour cette
        étiquette (2026-07-27 soir, retour terrain "le site se perd... des
        ennormes décalages", contrainte explicite de rester sur le site) —
        idempotent si déjà en cours pour la même étiquette.

        finalize=True : un enregistrement d'un AUTRE test tourne encore ->
        on le termine et on l'exporte proprement, PUIS on démarre le
        nouveau. Ajouté le 2026-07-28 : sans ça l'opérateur restait bloqué
        (le bouton "revenait tout seul") sans chemin évident vers l'autre
        essai. Enchaîné dans un thread pour ne pas bloquer le callback ROS,
        l'export lisant le bag (conversion CSV) prend quelques secondes."""
        import re as _re
        test = str(test or "").strip()
        if not test:
            return
        # Slug défensif : seulement alphanumérique/-/_ dans un nom de fichier.
        safe = _re.sub(r"[^A-Za-z0-9_-]", "", test)
        if not safe:
            return
        # BUG CORRIGÉ (2026-07-27 soir, repéré en direct via la console
        # opérateur : "Enregistrement robuste démarré (test1)" suivi de
        # "Export Matlab OK (test2)" — les données de test1 exportées sous le
        # nom test2) : self._current_test ne doit JAMAIS changer sans que
        # l'enregistrement robuste ait RÉELLEMENT basculé — sinon le nom de
        # fichier à l'export (base = self._current_test) et les données
        # réellement enregistrées (celles de l'ancien test, toujours actif)
        # désynchronisent silencieusement. On tente le démarrage D'ABORD ;
        # self._current_test ne suit QUE si ça a réussi (ou si rien
        # n'était en cours). _export_matlab_worker utilise en plus, en
        # second filet, l'étiquette propre de l'enregistrement arrêté —
        # jamais self._current_test — pour nommer les fichiers.
        started = self._start_robust_record(safe)
        if started:
            self._current_test = safe
            self._log(f"Test sélectionné : {safe}")
        elif finalize:
            # Un autre essai tourne : on le termine + exporte, puis on
            # enchaîne sur le nouveau. Thread : l'export lit et convertit le
            # bag (quelques secondes), le callback ROS ne doit pas bloquer.
            prev = self._robust_rec.get("test") if self._robust_rec else "?"
            self._log(f"Fin de {prev} demandée pour passer à {safe} — "
                      f"export en cours…")
            threading.Thread(target=self._finalize_and_switch,
                             args=(safe,), daemon=True).start()
        else:
            self._log(f"Test {safe} NON sélectionné — "
                      f"{self._current_test} reste actif (Export ou Reset d'abord)")

    def _finalize_and_switch(self, new_test):
        """Termine+exporte l'essai en cours, puis démarre le suivant."""
        try:
            self._export_matlab_worker(open_matlab=False)
        except Exception as e:
            self._log(f"Export avant bascule échoué : {e}")
        # _export_matlab_worker a arrêté l'enregistrement : la voie est libre.
        if self._start_robust_record(new_test):
            self._current_test = new_test
            self._log(f"Test sélectionné : {new_test} (prêt à rouler)")
        else:
            self._log(f"Bascule vers {new_test} impossible — "
                      f"un enregistrement est toujours actif")

    # ── Enregistrement robuste (rosbag détaché) ────────────────────────────
    def _write_robust_record_state(self, state):
        try:
            with open(ROBUST_REC_STATE_FILE, "w") as f:
                json.dump(state, f)
        except Exception:
            pass

    def _clear_robust_record_state(self):
        try:
            os.remove(ROBUST_REC_STATE_FILE)
        except FileNotFoundError:
            pass
        except Exception:
            pass

    def _recover_robust_record_state(self):
        """Appelé au démarrage : retrouve un enregistrement laissé actif par
        une instance précédente de leo_backend.py (redémarrage watchdog en
        plein milieu d'un essai — exactement le cas que ce mécanisme doit
        couvrir). Le process rosbag lui-même a survécu (double-fork,
        détaché) ; seul l'état EN MÉMOIRE de ce process-ci a été perdu."""
        try:
            with open(ROBUST_REC_STATE_FILE) as f:
                state = json.load(f)
        except FileNotFoundError:
            return None
        except Exception:
            return None
        pid = state.get("pid")
        if pid:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                self._log("Enregistrement robuste retrouvé mais process déjà "
                          "terminé — état effacé")
                self._clear_robust_record_state()
                return None
            except Exception:
                pass
        self._log(f"Enregistrement robuste retrouvé après redémarrage backend : "
                  f"{state.get('test')} (pid {pid}, toujours actif — continue)")
        return state

    def _start_robust_record(self, test_label):
        """Démarre un enregistrement rosbag détaché (tools/rosbag_daemon.sh)
        pour l'essai <test_label>. No-op (True) si déjà actif pour la MÊME
        étiquette (clic répété). Si un AUTRE enregistrement tourne déjà,
        REFUSE (False) explicitement plutôt que de perdre silencieusement des
        données — l'opérateur doit Exporter ou Réinitialiser d'abord.
        Valeur de retour utilisée par _set_current_test pour savoir si
        self._current_test doit suivre l'étiquette demandée (uniquement si
        l'enregistrement a réellement basculé dessus — sinon désync entre le
        nom affiché et les données réellement capturées, cf. bug du
        27/07 soir)."""
        import subprocess
        if self._robust_rec is not None:
            if self._robust_rec.get("test") == test_label:
                return True
            self._log(f"Enregistrement {self._robust_rec.get('test')} déjà en "
                      f"cours — Export ou Reset avant de démarrer {test_label}")
            return False
        try:
            os.makedirs(ROBUST_REC_DIR, exist_ok=True)
            # Aucun enregistrement déclaré, mais un process fantôme peut
            # survivre à un redémarrage backend et continuer d'écrire : on
            # balaie AVANT de démarrer, sinon deux enregistreurs coexistent et
            # l'essai est corrompu sans le moindre message (2026-07-28).
            self._sweep_orphan_recorders()
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_prefix = os.path.join(ROBUST_REC_DIR, f"{test_label}_{stamp}")
            pidfile = bag_prefix + ".pid"
            subprocess.Popen([ROBUST_REC_DAEMON_SH, pidfile, bag_prefix]
                             + ROBUST_REC_TOPICS, start_new_session=True)
            # rosbag_daemon.sh écrit le PID de façon asynchrone (double-fork) —
            # on l'attend un court instant avant de persister l'état.
            pid = None
            for _ in range(30):  # jusqu'à 3 s
                if os.path.isfile(pidfile):
                    try:
                        with open(pidfile) as f:
                            pid = int(f.read().strip())
                        break
                    except Exception:
                        pass
                time.sleep(0.1)
            if pid is None:
                self._log(f"Enregistrement robuste : PID jamais vu pour "
                          f"{test_label} (rosbag a peut-être échoué au "
                          f"démarrage — voir {bag_prefix}.log)")
            state = {"test": test_label, "bag_prefix": bag_prefix, "pid": pid,
                     "started_ts": time.time()}
            self._robust_rec = state
            self._write_robust_record_state(state)
            self._log(f"Enregistrement robuste démarré ({test_label}) -> "
                      f"{os.path.basename(bag_prefix)}.bag (survit à un "
                      f"redémarrage du backend)")
            return True
        except Exception as e:
            self._log(f"Échec démarrage enregistrement robuste : {e}")
            return False

    def _stop_robust_record(self):
        """Arrête proprement l'enregistrement robuste en cours (SIGINT,
        rosbag flush + ferme le .bag avant de sortir — même mécanisme qu'un
        Ctrl-C terminal). Renvoie l'état {"test", "bag_prefix", ...} de
        l'enregistrement arrêté, ou None si rien n'était en cours (l'appelant
        doit alors se rabattre sur l'ancien chemin buffer-mémoire).
        Renvoie l'ÉTAT COMPLET (pas juste bag_prefix) : l'appelant doit nommer
        ses fichiers d'après state["test"] — l'étiquette RÉELLE de ce qui a
        été enregistré — jamais self._current_test, qui peut avoir déjà
        changé entretemps (bug du 27/07 soir, cf. _set_current_test)."""
        state = self._robust_rec
        if state is None:
            return None
        pid = state.get("pid")
        bag_prefix = state.get("bag_prefix")
        if pid:
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            except Exception as e:
                self._log(f"Arrêt enregistrement robuste : {e}")
            for _ in range(50):  # jusqu'à 5 s pour laisser rosbag flush/fermer
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    break
                except Exception:
                    break
                time.sleep(0.1)
        self._robust_rec = None
        self._clear_robust_record_state()
        # Balayage de sécurité (2026-07-28, blocage signalé : "le reset reste
        # bloqué, le redémarrage de l'essai ne fonctionne pas") : le PID
        # mémorisé ne suffit pas toujours. rosbag lance TROIS process (wrapper
        # bash -> python rosbag -> binaire C++ lib/rosbag/record) et seul le
        # python est mémorisé ; si le SIGINT ne se propage pas, ou si l'état
        # a été perdu (redémarrage backend), des process continuent d'écrire
        # dans le dossier et le prochain démarrage se retrouve en conflit.
        # On balaie donc par MOTIF, pas seulement par PID.
        self._sweep_orphan_recorders()
        self._log(f"Enregistrement robuste arrêté ({state.get('test')}) -> "
                  f"{os.path.basename(bag_prefix)}.bag")
        return state

    def _sweep_orphan_recorders(self):
        """Tue tout `rosbag record` orphelin écrivant dans ROBUST_REC_DIR.
        Filet de sécurité : le nettoyage ne doit JAMAIS dépendre d'un seul PID
        mémorisé, sinon un essai suivant est bloqué par un process fantôme."""
        import subprocess, glob as _glob
        killed = 0
        try:
            out = subprocess.run(["pgrep", "-af", "rosbag"], capture_output=True,
                                 text=True, timeout=10).stdout
        except Exception:
            return
        for line in out.splitlines():
            # On ne tue QUE ce qui écrit dans notre dossier d'essais — jamais
            # un rosbag lancé à la main par l'opérateur pour autre chose.
            if ROBUST_REC_DIR not in line:
                continue
            try:
                pid = int(line.split(None, 1)[0])
            except (ValueError, IndexError):
                continue
            if pid == os.getpid():
                continue
            try:
                os.kill(pid, signal.SIGINT)
                killed += 1
            except ProcessLookupError:
                pass
            except Exception:
                pass
        if killed:
            time.sleep(2.0)          # laisse rosbag fermer proprement ses .bag
            self._log(f"Nettoyage : {killed} enregistreur(s) orphelin(s) arrêté(s)")

    def _discard_robust_record(self):
        """Arrête ET supprime l'enregistrement en cours (bag + log + pidfile)
        — utilisé par _traj_reset pour repartir propre après un essai raté,
        sans laisser de bag orphelin sur le disque."""
        state = self._stop_robust_record()
        if state is None:
            return
        bag_prefix = state["bag_prefix"]
        for ext in (".bag", ".bag.active", ".log", ".pid"):
            try:
                os.remove(bag_prefix + ext)
            except FileNotFoundError:
                pass
            except Exception:
                pass
        self._log(f"Enregistrement robuste annulé et supprimé "
                  f"({os.path.basename(bag_prefix)}.bag)")

    def _read_bag_as_snapshot(self, bag_prefix):
        """Convertit <bag_prefix>.bag en CSV (tools/bag_to_csv.py, sous-
        processus synchrone — on est déjà dans le thread d'export) puis les
        relit dans le MÊME format que le snapshot du buffer mémoire
        ({"mins": [(t,x,y,z,qx,qy,qz,qw), ...], "vins": [...], "carolus":
        [...]}), pour que le reste de _export_matlab_worker reste inchangé."""
        import subprocess, csv as _csv
        snap = {"mins": [], "vins": [], "carolus": []}
        bag_path = bag_prefix + ".bag"
        if not os.path.isfile(bag_path):
            self._log(f"Export robuste : bag introuvable ({bag_path})")
            return snap
        try:
            subprocess.run([sys.executable, ROBUST_REC_BAG2CSV, bag_path],
                           check=True, capture_output=True, text=True, timeout=120)
        except Exception as e:
            msg = getattr(e, "stderr", "") or str(e)
            self._log(f"Conversion bag->CSV échouée : {msg}")
            return snap
        for key in ("mins", "vins", "carolus"):
            path = f"{bag_prefix}_{key}.csv"
            if not os.path.isfile(path):
                continue
            try:
                with open(path, newline="") as f:
                    reader = _csv.reader(f)
                    next(reader, None)  # en-tête
                    for row in reader:
                        t, x, y, z, qx, qy, qz, qw = (float(v) for v in row[:8])
                        snap[key].append((t, x, y, z, qx, qy, qz, qw))
            except Exception as e:
                self._log(f"Lecture {path} échouée : {e}")
        return snap

    def _export_matlab(self, open_matlab=False):
        """Vide les buffers de trajectoire vers web/exports/ en CSV (un par
        source, format identique à bag_to_csv.py) + un .mat unique. Publie les
        noms de fichiers dans la télémétrie pour que le cockpit offre le
        téléchargement. Si open_matlab=True, OUVRE ensuite MATLAB (GUI) sur le
        tracé (demande opérateur 2026-07-27). Tourne dans un thread."""
        threading.Thread(target=self._export_matlab_worker, args=(open_matlab,),
                         daemon=True).start()

    def _export_matlab_worker(self, open_matlab=False):
        import csv as _csv
        try:
            os.makedirs(self._export_dir, exist_ok=True)
            # Chemin robuste EN PRIORITÉ (2026-07-27 soir) : si un
            # enregistrement rosbag détaché est (ou était, avant un
            # redémarrage backend entretemps) en cours pour ce test, on
            # l'arrête proprement et on lit SES données — écrites sur
            # disque au fil de l'eau, jamais perdues par un redémarrage.
            # Repli sur l'ancien buffer mémoire SEULEMENT si aucun
            # enregistrement robuste n'a jamais été démarré (ex. bouton
            # Export cliqué sans être passé par le sélecteur Test 1/Test 2).
            rec_state = self._stop_robust_record()
            if rec_state is not None:
                snap = self._read_bag_as_snapshot(rec_state["bag_prefix"])
                source_desc = "rosbag robuste"
                # Étiquette RÉELLE de ce qui a été enregistré — PAS
                # self._current_test, qui peut avoir déjà changé entretemps
                # sans que l'enregistrement ait réellement basculé dessus
                # (bug du 27/07 soir : "test1" enregistré mais exporté sous
                # "test2" parce que self._current_test avait déjà changé).
                base = rec_state["test"]
            else:
                with self._traj_lock:
                    snap = {k: list(v) for k, v in self._traj.items()}
                source_desc = "buffer mémoire (legacy, pas d'enregistrement robuste actif)"
                base = self._current_test
            firsts = [rows[0][0] for rows in snap.values() if rows]
            if not firsts:
                self._log(f"Export Matlab : aucune donnée de trajectoire ({source_desc})")
                return
            t0 = min(firsts)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            # Préfixe = étiquette de test (2026-07-27, sélecteur Trajectory) :
            # "test1_mins.csv"/"test2_vins.csv" — noms PRÉVISIBLES (pas
            # horodatés) exprès, pour que le script Matlab de comparaison
            # puisse charger test1_*/test2_* directement sans avoir à
            # deviner/globber le dernier export. Un ré-export sous la même
            # étiquette écrase volontairement le précédent (refaire un essai
            # raté doit remplacer, pas accumuler des variantes).
            # 11 colonnes : quaternion CONSERVÉ (sans perte) + Roll/Pitch/Yaw
            # dérivés (2026-07-27, demande : le .mat doit porter t,X,Y,Z,R,P,Y
            # pour MINS et VINS). plot_trajectories.m / bag_to_csv.py lisent le
            # même format.
            hdr = ["t", "x", "y", "z", "qx", "qy", "qz", "qw",
                   "roll_rad", "pitch_rad", "yaw_rad"]
            counts, files = {}, {}
            # .mat : un STRUCT MATLAB par source, champs nommés vecteurs colonne
            # (glisser-déposer direct dans MATLAB -> variables MINS / VINS /
            # CAROLUS). Sources vides incluses (arrays vides) pour que la
            # variable existe toujours côté MATLAB.
            mat = {"columns": np.array(hdr, dtype=object)}
            FIELDS = ["t", "x", "y", "z", "qx", "qy", "qz", "qw",
                      "roll", "pitch", "yaw"]
            for src, rows in snap.items():
                counts[src] = len(rows)
                cols = [[] for _ in FIELDS]
                fname = "%s_%s.csv" % (base, src)
                if rows:
                    with open(os.path.join(self._export_dir, fname), "w", newline="") as f:
                        w = _csv.writer(f)
                        w.writerow(hdr)
                        for (t, x, y, z, qx, qy, qz, qw) in rows:
                            roll, pitch, yaw = self._rpy_from_quat(qx, qy, qz, qw)
                            rel = t - t0
                            vals = (rel, x, y, z, qx, qy, qz, qw, roll, pitch, yaw)
                            w.writerow(["%.6f" % v for v in vals])
                            for i, v in enumerate(vals):
                                cols[i].append(v)
                    files[src] = fname
                # struct MATLAB : champs = vecteurs colonne (N×1), même vide.
                key = src.upper()      # MINS / VINS / CAROLUS
                mat[key] = {FIELDS[i]: np.array(cols[i], dtype=float).reshape(-1, 1)
                            for i in range(len(FIELDS))}
            # .mat unique (scipy.io.savemat) — structs nommés, chargeable direct.
            mat_name = None
            try:
                from scipy.io import savemat
                mat_name = "%s.mat" % base
                savemat(os.path.join(self._export_dir, mat_name), mat)
            except Exception as e:
                self._log("Export .mat ignoré (%s) — les CSV restent disponibles" % e)
            self._last_export = {"ts": stamp, "base": base, "files": files,
                                 "mat": mat_name, "counts": counts}
            n = ", ".join("%s=%d" % (k, counts[k]) for k in ("mins", "vins", "carolus"))
            self._log("Export Matlab OK [%s] (%s) -> web/exports/%s_*  [%s]"
                      % (source_desc, base, base, n))
            # Ouvrir MATLAB (GUI) sur le tracé, si demandé (le clic Export du
            # cockpit) — enchaîné APRÈS l'écriture pour que le .mat existe.
            if open_matlab and mat_name:
                self._launch_matlab_gui(os.path.join(self._export_dir, base))
        except Exception as e:
            self._log("Export Matlab ÉCHEC : %s" % e)

    def _gui_env(self):
        """Environnement pour lancer une appli GUI (MATLAB) sur l'écran de la
        session : le backend est lancé par rosmon SANS DISPLAY/XAUTHORITY, on
        les reconstitue. Best-effort, valeurs sondées à défaut d'être héritées."""
        import glob as _glob
        env = dict(os.environ)
        if not env.get("DISPLAY"):
            socks = sorted(_glob.glob("/tmp/.X11-unix/X*"))
            env["DISPLAY"] = ":" + socks[0].split("/X")[-1] if socks else ":0"
        if not env.get("XAUTHORITY") or not os.path.isfile(env.get("XAUTHORITY", "")):
            uid = os.getuid()
            for cand in (["/run/user/%d/gdm/Xauthority" % uid,
                          os.path.expanduser("~/.Xauthority")] +
                         _glob.glob("/run/user/%d/.mutter-Xwaylandauth.*" % uid)):
                if os.path.isfile(cand):
                    env["XAUTHORITY"] = cand
                    break
        # XDG_RUNTIME_DIR / DBUS_SESSION_BUS_ADDRESS (2026-07-27, audit
        # "MATLAB s'ouvre puis se ferme tout seul" — persistant même après le
        # fix pty) : leo_backend.py tourne sans AUCUNE session — pas juste
        # sans DISPLAY, mais aussi lancé (via le watchdog cron) dans le
        # cgroup system.slice/cron.service, sans XDG_RUNTIME_DIR ni bus
        # D-Bus de session (confirmé vide dans /proc/<pid>/environ, alors
        # qu'une session utilisateur normale les a toujours). Une
        # reproduction manuelle DEPUIS un vrai shell de session (donc avec
        # ces variables héritées) survivait 60s+ sans problème ; la même
        # commande via subprocess.Popen SANS elles échouait en quelques
        # secondes, de façon reproductible. Bus/runtime dir manquants =
        # suspect probable pour un toolkit desktop moderne (intégration
        # système, notifications) qui échoue silencieusement sans repli
        # robuste. Reconstruits ici sur le même modèle que DISPLAY/XAUTHORITY
        # (chemin standard /run/user/<uid>, vérifié existant avant usage).
        uid = os.getuid()
        if not env.get("XDG_RUNTIME_DIR"):
            cand = "/run/user/%d" % uid
            if os.path.isdir(cand):
                env["XDG_RUNTIME_DIR"] = cand
        if not env.get("DBUS_SESSION_BUS_ADDRESS"):
            bus_sock = "/run/user/%d/bus" % uid
            if os.path.exists(bus_sock):
                env["DBUS_SESSION_BUS_ADDRESS"] = "unix:path=%s" % bus_sock
        # LD_BIND_NOW (2026-07-27, audit du bouton Export) : tenté comme
        # correctif pour un crash glibc capturé en direct (grâce à stdout/
        # stderr enfin loggés au lieu de /dev/null) — "Inconsistency detected
        # by ld.so: ../elf/dl-tls.c:517: _dl_allocate_tls_init: Assertion
        # `listp != NULL' failed!", une race connue du linker dans
        # l'allocation de TLS au chargement différé de bibliothèques. N'A PAS
        # supprimé le crash de façon fiable en test (encore observé avec ce
        # réglage) — la VRAIE cause était le choix de version MATLAB (R2026a
        # plante ici, R2025b ne l'a jamais fait, voir le constructeur).
        # Gardé quand même : sans effet mesuré négatif, et l'un des
        # correctifs standard documentés pour cette classe de bug si le
        # problème réapparaît sous une autre version.
        env["LD_BIND_NOW"] = "1"
        return env

    def _plot_matlab(self):
        """Ouvre MATLAB (GUI) sur le tracé du dernier export."""
        if not self._matlab_bin:
            self._log("MATLAB introuvable — indisponible")
            return
        ex = self._last_export
        if not ex or not ex.get("mat"):
            self._log("MATLAB : exporte d'abord des données (aucun .mat récent)")
            return
        self._launch_matlab_gui(os.path.join(self._export_dir, ex["base"]))

    def _set_matlab_launch(self, state, detail):
        """Publie l'etat du dernier lancement MATLAB pour le cockpit."""
        self._matlab_launch = {"state": state, "detail": detail,
                               "ts": time.strftime("%H:%M:%S")}
        self._log("MATLAB GUI [%s] %s" % (state, detail))

    def _watch_matlab_launch(self, log_path):
        """Surveille le journal du MATLAB detache et en deduit un verdict.

        POURQUOI (retour operateur 2026-07-29 : « le bouton EXPORT ne fait
        rien ») : l'export lui-meme reussissait — CSV et .mat ecrits,
        last_export publie en 2.5 s — mais quand open_matlab=True, le seul
        resultat VISIBLE etait l'ouverture de MATLAB. Or MATLAB echouait sur
        sa licence (error 5001 / Licensing shutdown), dans un processus
        detache dont le backend ignorait tout. Le cockpit attendait une
        fenetre qui n'arrivait jamais, sans proposer de repli. Vu de
        l'operateur : le clic n'avait servi a rien.

        Le double-fork est deliberement conserve (il protege la session MATLAB
        des redemarrages de stack) ; on renonce donc au code de sortie et on
        lit le journal, qui est la seule trace disponible.
        """
        marqueurs = ("error 5001", "license error", "licensing shutdown",
                     "unable to launch mvm", "license checkout failed",
                     "license manager error")
        t0 = time.time()
        while time.time() - t0 < 120.0 and not self._closing:
            time.sleep(2.0)
            try:
                with open(log_path, errors="replace") as f:
                    txt = f.read()
            except OSError:
                continue                      # pas encore cree
            bas = txt.lower()
            trouve = next((m for m in marqueurs if m in bas), None)
            if trouve:
                self._set_matlab_launch(
                    "failed", "licence MATLAB indisponible (%s) — "
                              "le .mat reste telechargeable" % trouve)
                return
            # `script` ecrit COMMAND_EXIT_CODE en fin de session : MATLAB a
            # quitte. Non-nul = echec, nul = l'operateur a ferme la fenetre.
            if 'COMMAND_EXIT_CODE="' in txt:
                code = txt.split('COMMAND_EXIT_CODE="')[1].split('"')[0]
                if code != "0":
                    self._set_matlab_launch(
                        "failed", "MATLAB a quitté (code %s) — "
                                  "le .mat reste telechargeable" % code)
                else:
                    self._set_matlab_launch("closed", "MATLAB fermé")
                return
            # Pas d'erreur apres 35 s : l'interface est vraisemblablement
            # ouverte (le demarrage prend ~20-30 s sur cette machine).
            if time.time() - t0 > 35.0:
                self._set_matlab_launch("running", "MATLAB ouvert sur le tracé")
                return
        self._set_matlab_launch("unknown",
                                "verdict indéterminé après 120 s — "
                                "voir web/exports/matlab_launch.log")

    def _launch_matlab_gui(self, base):
        """Lance MATLAB en GUI (fenêtre visible) sur <base>.mat, non-bloquant.
        `-r` (pas `-batch`) garde le bureau MATLAB ouvert après le tracé."""
        if not self._matlab_bin:
            return
        threading.Thread(target=self._launch_matlab_gui_worker, args=(base,),
                         daemon=True).start()

    def _launch_matlab_gui_worker(self, base):
        import subprocess
        try:
            mat = base + ".mat"
            # self._matlab_bin, PAS une détection de session déjà ouverte :
            # essayé (matcher la version d'un process MATLAB actif), abandonné
            # (2026-07-27) — la machine a une session R2026a active en
            # permanence (ServiceHost) et R2026a est précisément la version
            # qui plante ici (voir constructeur). Matcher "ce qui tourne déjà"
            # aurait sélectionné la version cassée à chaque fois.
            matlab_bin = self._matlab_bin
            # -sd : dossier de démarrage = tools (plot_trajectories.m y est) ;
            # -r : exécute puis LAISSE MATLAB ouvert (contrairement à -batch).
            cmd = [matlab_bin, "-sd", self._tools_dir, "-r",
                   "plot_trajectories('%s')" % mat]
            env = self._gui_env()
            # Log capturé (2026-07-27, audit) : DEVNULL rendait un échec
            # totalement invisible — si MATLAB se ferme tôt (licence,
            # conflit de version), ceci en garde la trace exacte.
            log_path = os.path.join(self._export_dir, "matlab_launch.log")
            # Purge du journal AVANT lancement : sans ca, la surveillance
            # relirait l'echec de la fois precedente et le rapporterait comme
            # celui de ce clic-ci.
            try:
                os.remove(log_path)
            except OSError:
                pass
            self._set_matlab_launch("starting", "démarrage de MATLAB…")
            # Double-fork via matlab_daemon.sh (2026-07-27) : start_new_session
            # =True (setsid) protège d'un os.killpg() ciblant le groupe de
            # process de leo_backend (c'est ainsi que roslaunch arrête ses
            # nœuds) — testé, INSUFFISANT quand même : un restart de stack
            # déclenché par le watchdog pendant une session MATLAB ouverte la
            # tuait sans aucune trace d'erreur côté MATLAB (log propre jusqu'au
            # prompt). matlab_daemon.sh fait `cmd &` puis sort aussitôt sans
            # wait : le job orpheliné est réattaché à init (ou au subreaper de
            # session), donc structurellement hors de la descendance PPID de
            # leo_backend.py — insensible à tout nettoyage par groupe OU par
            # arbre de process. Vérifié en test contrôlé (ps -o ppid après
            # double-fork : PPID quitte bien la lignée leo_backend/roslaunch).
            daemon_sh = os.path.join(self._tools_dir, "matlab_daemon.sh")
            subprocess.Popen([daemon_sh, log_path] + cmd, cwd=self._tools_dir,
                             env=env, start_new_session=True)
            # Le processus est detache : on ne peut pas l'attendre. On surveille
            # son journal dans un thread pour rendre un verdict au cockpit.
            threading.Thread(target=self._watch_matlab_launch,
                             args=(log_path,), daemon=True).start()
            self._log("MATLAB : ouverture de l'application (%s, DISPLAY=%s, ~30 s)…"
                      % (os.path.basename(os.path.dirname(os.path.dirname(matlab_bin))),
                         env.get("DISPLAY", "?")))
        except Exception as e:
            self._log("MATLAB : erreur d'ouverture (%s)" % e)

    # ── Params Carolus (visualisation + réglage) ──────────────────────────────
    def _beacon_watch(self, now):
        """Lot D — surveille la perte de balise et renvoie l'etat pour le cockpit.

        Deux niveaux separes a dessein : SIGNALER est toujours utile, ARRETER
        ne l'est que dans les etats qui dependent de la balise. Un arret en
        PATROL serait absurde : la patrouille cherche justement une balise
        qu'elle ne voit pas encore.

        `fired` n'est arme qu'UNE FOIS par episode de perte : sans cela, on
        republierait un arret a 10 Hz, ce qui empecherait l'operateur de
        reprendre la main en manuel.
        """
        fix = self._caro_fix
        age = None if not fix else max(0.0, now - fix["t"])
        # Aucun fix depuis le demarrage : ce n'est pas une PERTE, c'est une
        # absence. Ne pas declencher — sinon tout demarrage sans balise
        # armerait l'arret.
        lost = bool(fix is not None and age > self._beacon_lost_timeout)

        if not lost:
            self._beacon_lost_since = None
            self._beacon_estop_fired = False
        elif self._beacon_lost_since is None:
            self._beacon_lost_since = now

        depends = self.auto_state in BEACON_DEPENDENT_STATES
        if (lost and depends and self._beacon_estop
                and not self._beacon_estop_fired):
            self._beacon_estop_fired = True
            try:
                self.set_manual(0.0, 0.0)
                self._log("ARRÊT : balise perdue depuis %.1f s en %s "
                          "(seuil %.1f s)"
                          % (age, self.auto_state, self._beacon_lost_timeout))
            except Exception as e:
                self._log("Arrêt sur perte de balise IMPOSSIBLE : %s" % e)

        return {"lost": lost,
                "age_s": None if age is None else round(age, 1),
                "timeout_s": self._beacon_lost_timeout,
                "estop_armed": bool(self._beacon_estop),
                "estop_fired": bool(self._beacon_estop_fired),
                "state_depends": bool(depends),
                "state": self.auto_state}

    def _drive_params_or_retry(self):
        dp = self._drive_params
        if dp and any(v is not None for v in dp.values()):
            return dp
        self._drive_params = self._load_drive_params()
        return self._drive_params

    def _load_drive_params(self):
        """Parametres cinematiques du firmware (Lot C, v4, 2026-07-30).

        LUS DANS LE SERVEUR DE PARAMETRES, pas dans /etc/ros/param.yaml :
        (a) ce fichier vit sur le ROBOT alors que ce backend tourne sur le PC ;
        (b) surtout, le serveur porte la valeur REELLEMENT ACTIVE, qui peut
            differer du fichier si quelqu'un l'a changee a chaud. Afficher le
            fichier donnerait une fausse assurance.

        `angular_velocity_multiplier` est expose parce qu'il MULTIPLIE
        directement le lacet : c'est un candidat serieux a l'ecart de 7-9 %
        mesure en rotation le 29/07, et ni l'operateur ni moi ne l'avions
        considere avant la lecture du document Turki Yassin (§7.3).

        LECTURE SEULE, deliberement : ecrire ces valeurs impose d'editer
        /etc/ros/param.yaml sur le robot PUIS `systemctl restart leo`, ce qui
        coupe le pilotage. Ce doit rester une action humaine deliberee.
        """
        import rospy          # importe localement : ce module ne l'a PAS au
                              # niveau global (cf. run(), ligne ~1531)
        out = {}
        for key in ("wheel_radius", "wheel_separation",
                    "angular_velocity_multiplier"):
            try:
                out[key] = rospy.get_param("/firmware/diff_drive/" + key)
            except Exception as e:
                out[key] = None
                rospy.logwarn_throttle(
                    60.0, "[drive_params] %s indisponible : %r", key, e)
        return out

    def _load_carolus_params(self):
        """Parse le launchfile Carolus et renvoie les params réglables (valeurs
        courantes). Lecture seule au démarrage ; best-effort (dict vide si le
        launch est absent)."""
        import re
        out = {}
        try:
            with open(CAROLUS_LAUNCH, "r") as f:
                txt = f.read()
            for key in CAROLUS_TUNABLE:
                m = re.search(r'name="%s"\s+value="([^"]+)"' % re.escape(key), txt)
                if m:
                    try:
                        out[key] = float(m.group(1))
                    except ValueError:
                        out[key] = m.group(1)
        except Exception:
            pass
        return out

    def _set_carolus_param(self, key, value):
        """Met à jour un param Carolus : sur le serveur de params ROS (effectif
        au PROCHAIN lancement du nœud Carolus) et dans l'état affiché. Le nœud
        Carolus n'a pas de dynamic_reconfigure — pas d'effet à chaud, c'est
        explicite côté cockpit."""
        if key not in CAROLUS_TUNABLE:
            self._log("Param Carolus refusé (inconnu) : %s" % key)
            return
        try:
            val = float(value)
        except (TypeError, ValueError):
            self._log("Param Carolus refusé (valeur non numérique) : %s=%r" % (key, value))
            return
        self._carolus_params[key] = val
        try:
            self._rospy.set_param("/carolus_astrobee_rex/%s" % key, val)
        except Exception:
            pass
        self._log("Param Carolus %s=%g (effectif au prochain lancement Carolus)"
                  % (key, val))

    # ══════════════════════════════════════════════════════════════════════ #
    # Thread DÉCODAGE — rapide, alimente l'image affichée
    # ══════════════════════════════════════════════════════════════════════ #
    def _decode_loop(self):
        while not self._closing:
            msg = None
            with self._lock:
                if self._cam_seq != self._dec_proc:
                    msg, self._dec_proc = self._cam_msg, self._cam_seq
            if msg is None:
                time.sleep(0.002)
                continue
            try:
                bgr = self._decode(msg)
                if bgr is not None:
                    with self._lock:
                        self.latest_bgr = bgr
                        self._dec_seq += 1
                        seq = self._dec_seq
                    # Feed subprocess with compressed JPEG bytes (small, fast pickle)
                    if self.compressed:
                        raw = bytes(msg.data)
                    else:
                        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        raw = buf.tobytes() if ok else None
                    if raw is not None:
                        live = {'bright_thresh': self._v_min,
                                'minled': self._minled}
                        try:
                            self._mp_frame_q.put_nowait((seq, raw, live))
                        except Exception:
                            try:    self._mp_frame_q.get_nowait()
                            except Exception: pass
                            try:    self._mp_frame_q.put_nowait((seq, raw, live))
                            except Exception: pass
            except Exception:
                pass

    def _rebuild_det(self):
        """Assemble self.det from the two independent pipeline results. Call with lock held."""
        lr = self._led_result or {}
        lm = self._landmark_result or {}
        center = lm.get("center") if lm.get("valid") else lr.get("center")
        dist   = lm.get("dist_est") if lm.get("valid") else lr.get("dist")
        self.det = {
            "led_reset":    lr,
            "map_landmark": lm,
            "leds":         lr.get("leds_pos", []),
            "near":         lr.get("cluster_pos", []),
            "cluster":      lr.get("cluster_pos", []),
            "checker":      lm.get("checker_size"),
            "center":       center,
            "led_center":   lr.get("center"),
            "dist":         dist,
            "tag":          None,
            "mask":         lr.get("mask"),
            "leds_pos":     lr.get("leds_pos", []),
            "cluster_pos":  lr.get("cluster_pos", []),
            "valid":        lr.get("valid", False) or lm.get("valid", False),
        }

    def _start_vision_process(self):
        """Start the vision subprocess with a spawn context (no inherited ROS fds)."""
        cfg = {
            'cam_fx':            CAM_FX,
            'beacon_width_m':    BEACON_WIDTH_M,
            'cb_scale':          CB_SCALE,
            'landmark_square_m': LANDMARK_SQUARE_M,
            'checkerboard_sizes': list(CHECKERBOARD_SIZES),
            'bright_thresh':     LED_BRIGHT_THRESH,
            'led_min_area':      LED_MIN_AREA,
            'led_max_area':      LED_MAX_AREA,
            'led_cluster_px':    LED_CLUSTER_PX,
            'n_leds':            N_LEDS,
            'minled':            self._minled,
            'led_stable_frames':    LED_STABLE_FRAMES,
            'lm_miss_max':          LM_MISS_MAX,
            'vision_hysteresis':    VISION_HYSTERESIS,
            'landmark_detect_hz':   LANDMARK_DETECT_HZ,
            'cam_k':                CAM_K,
            'cam_d':                CAM_D,
            'reproj_err_max':       REPROJ_ERR_MAX,
            'pnp_skip_ms':          PNP_SKIP_MS,
        }
        self._mp_stop = _MP_CTX.Event()
        self._mp_proc = _MP_CTX.Process(
            target=_mp_vision_worker,
            args=(self._mp_frame_q, self._mp_result_q, self._mp_stop, cfg),
            daemon=True,
            name='leo-vision',
        )
        self._mp_proc.start()
        self._log(f"Vision subprocess started (PID {self._mp_proc.pid}) — brightness LED + checkerboard")

    def _mp_result_loop(self):
        """Lightweight thread — drains the result queue from the vision subprocess,
        applies EMA smoothing, updates self.det, and fires mission events."""
        while not self._closing:
            try:
                result = self._mp_result_q.get(timeout=0.1)
            except Exception:
                continue

            try:
                lr = result.get("led_reset",    {})
                lm = result.get("map_landmark", {})

                # EMA smoothing in main process (keeps subprocess stateless on coordinates)
                if lr.get("valid") and lr.get("center") and not lr.get("_held"):
                    ex, ey = self._led_ema.update(lr["center"][0], lr["center"][1])
                    lr = dict(lr); lr["center"] = [round(ex, 1), round(ey, 1)]
                elif not lr.get("valid") and not lr.get("_held"):
                    self._led_ema.reset()

                if lm.get("valid") and lm.get("center") and not lm.get("_held"):
                    ex, ey = self._lm_ema.update(lm["center"][0], lm["center"][1])
                    lm = dict(lm); lm["center"] = [round(ex, 1), round(ey, 1)]
                elif not lm.get("valid") and not lm.get("_held"):
                    self._lm_ema.reset()

                now = time.time()
                _new_mode = result.get("vision_mode", "LED")
                _old_mode = self._vision_mode       # read without lock (benign for logging)
                with self._lock:
                    self._vision_mode      = _new_mode
                    self._led_result       = lr
                    self._landmark_result  = lm
                    self._led_stable_count = 1 if lr.get("valid") else 0
                    self._lm_miss          = result.get("lm_miss", 0)
                    self._lm_hits          = result.get("lm_hits", 0)
                    self._led_stable       = result.get("led_stable", 0)
                    if lm.get("checker_size"):
                        self._cb_size = tuple(lm["checker_size"])
                    self._rebuild_det()
                    det_snap  = self.det
                    mode_snap = self.mode

                if _old_mode != _new_mode:
                    self._log(f"[VISION] mode : {_old_mode} → {_new_mode}")

                # Anti-jitter: count consecutive valid LED frames. A reset only
                # fires once the detection is confirmed over VISION_CONFIRM_FRAMES,
                # so a single flickering frame (e.g. 2 LEDs instead of 4) never
                # zeroes the odometry.
                # NE PLUS exiger "not _held" ici (2026-07-27, bug trouvé en
                # direct) : dès que le pipeline vision bascule en mode
                # LANDMARK, le LED reste EN PERMANENCE _held=True ("Hold last
                # LED result", cf. _mp_vision_worker) — le compteur restait
                # donc bloqué à sa valeur au moment du switch (souvent 0-2,
                # jamais assez pour VISION_CONFIRM_FRAMES) et le reset ne
                # pouvait plus JAMAIS se déclencher une fois en LANDMARK.
                # "valid" seul suffit : en hold, la position LED est gelée à
                # sa dernière valeur réelle, toujours significative tant que
                # la balise n'a pas bougé dans l'image.
                if lr.get("valid"):
                    self._led_confirm += 1
                else:
                    self._led_confirm = 0

                shape = (result.get("frame_h", 480), result.get("frame_w", 640), 3)
                if mode_snap == "MANUEL":
                    led_confirmed = (lr.get("valid")
                                      and self._led_confirm >= VISION_CONFIRM_FRAMES)
                    cb_confirmed  = (self._vision_mode == "LANDMARK"
                                      and lm.get("valid") and not lm.get("_held"))
                    # Reset odométrie sur LED SEULE (2026-07-27, revert du
                    # 22/07 sur décision opérateur explicite) : l'exigence
                    # double LED+damier laissait le reset ne jamais se
                    # déclencher — le damier utilise une détection rate-
                    # limitée (LANDMARK_DETECT_HZ) avec hystérésis/hold
                    # (_held), et sa fenêtre de fraîcheur coïncide trop
                    # rarement avec celle du LED pour une confirmation
                    # simultanée fiable, y compris à courte portée où le
                    # damier lui-même devient instable (mesuré en direct :
                    # confiance décroissant à 0 en continu). Le damier reste
                    # utilisé pour la précision sub-pixel de la carte
                    # (_try_map_landmark_event ci-dessous, inchangé), mais
                    # plus comme condition bloquante du reset.
                    if led_confirmed:
                        self._try_led_reset_event(lr, now)
                    if cb_confirmed:
                        self._try_map_landmark_event(lm, now)
                self._publish_vision_targets(det_snap, shape)
            except Exception as _e:
                self._log(f"[VISION-LOOP] error processing result: {_e}")

    def _publish_vision_targets(self, det, shape):
        """Publie les cibles de détection sur /vision/targets (deux pipelines séparés)."""
        try:
            if self._vision_pub is None:
                return
            h, w  = shape[:2]
            lr    = det.get("led_reset", {})
            lm    = det.get("map_landmark", {})
            center = det.get("center")
            dist   = det.get("dist")
            payload = {
                "ts":       round(time.time(), 3),
                "frame_w":  w,
                "frame_h":  h,
                # ── Pipeline 1 : LED Reset ──────────────────────────────
                "led_reset": {
                    "valid":   lr.get("valid", False),
                    "count":   len(lr.get("cluster", [])),
                    "leds":    lr.get("leds_pos", []),
                    "cluster": lr.get("cluster_pos", []),
                    "center":  ([round(float(lr["center"][0]), 1),
                                 round(float(lr["center"][1]), 1)]
                                if lr.get("center") else None),
                },
                # ── Pipeline 2 : Map Landmark ────────────────────────────
                "map_landmark": {
                    "valid":        lm.get("valid", False),
                    "checker_size": lm.get("checker_size"),
                    "center":  ([round(float(lm["center"][0]), 1),
                                 round(float(lm["center"][1]), 1)]
                                if lm.get("center") else None),
                    "dist_est":  lm.get("dist_est"),
                    "angle_deg": lm.get("angle_deg"),
                },
                # ── Legacy (compat consommateurs ROS existants) ──────────
                "leds":    lr.get("leds_pos", []),
                "cluster": lr.get("cluster_pos", []),
                "beacon": {
                    "detected": center is not None,
                    "cx":      round(float(center[0]), 1) if center else None,
                    "cy":      round(float(center[1]), 1) if center else None,
                    "dist":    round(dist, 2) if dist is not None else None,
                    "checker": lm.get("valid", False),
                },
            }
            self._vision_pub.publish(self._String(data=json.dumps(payload)))
        except Exception:
            pass

    def _fire_hw_reset(self):
        try:
            self._reset_srv()
        except Exception:
            pass

    # ── Événement LED Reset ───────────────────────────────────────────────
    def _try_led_reset_event(self, lr, now):
        """RESET_ODOMETRY: fired when BOTH the LED cluster AND the checkerboard
        are confirmed simultaneously (MANUAL mode) — the checkerboard's
        sub-pixel pose is required alongside the LED for a reset to be
        trusted (2026-07-22).
        Debounced by BEACON_COOLDOWN. Zeroes local odometry to (0,0,0).
        Caller gates this on VISION_CONFIRM_FRAMES consecutive valid LED
        frames AND a currently valid checkerboard detection.
        """
        if (now - self.last_beacon_t) < BEACON_COOLDOWN:
            return
        self.last_beacon_t = now
        self._fsm_reset_t  = now
        cx, cy = (lr["center"][0], lr["center"][1]) if lr.get("center") else (0, 0)
        n = len(lr.get("cluster_pos", []))
        self._log(f"LED RESET — {n} LEDs @ ({cx:.0f},{cy:.0f})px — odometry = 0,0,0")
        # Continuité du repère monde (même logique que reset_coords_local) :
        # sans cette accumulation, wx/wy retombaient sur l'ancien world_origin
        # après un reset LED en MANUEL et la carte (trajectoire globale,
        # marqueurs à venir) sautait — les balises doivent rester où elles
        # sont sur la carte à travers les resets.
        wx, wy = self._get_world_pos()
        self.world_origin_x = wx
        self.world_origin_y = wy
        self.world_heading  = (self.raw_yaw if self.raw_yaw is not None
                               else self.world_heading + float(self.pose[2]))
        with self._lock:
            self.origin = None
            self.traj   = []
        self._autolog.add(
            "LED_RESET", "LED Reset triggered — odometry zeroed",
            f"Cluster at ({cx:.0f}, {cy:.0f}) px — {n} LEDs",
            tags=["reset", "odometry", "led"],
        )
        if self._reset_srv is not None:
            threading.Thread(target=self._fire_hw_reset, daemon=True).start()
        self._tel_event.set()

    # ── Événement Map Landmark ────────────────────────────────────────────
    def _try_map_landmark_event(self, lm, now):
        """MAP LANDMARK : enregistre le damier comme repère permanent sur la carte (mode MANUEL).
        Anti-rebond LANDMARK_COOLDOWN. Incrémente beacon_count, publie le marqueur.
        """
        if (now - self._last_landmark_t) < LANDMARK_COOLDOWN:
            return
        self._last_landmark_t = now
        self.beacon_count += 1
        wx, wy = self._get_world_pos()
        # Estimate actual beacon world position from dist_est + angle_deg
        dist_m = lm.get("dist_est")
        ang_d  = lm.get("angle_deg")
        if dist_m and ang_d is not None:
            robot_h = self.world_heading + float(self.pose[2])
            bearing = robot_h + math.radians(float(ang_d))
            bwx = wx + float(dist_m) * math.cos(bearing)
            bwy = wy + float(dist_m) * math.sin(bearing)
        else:
            bwx, bwy = wx, wy
        # Anti-doublon (même règle qu'en AUTO)
        if self._nearest_marker("beacon", bwx, bwy) < BEACON_DEDUP_RADIUS_M:
            self.beacon_count -= 1   # annule l'incrément fait plus haut
            self._log(f"landmark déjà cartographié près de ({bwx:.2f}, {bwy:.2f}) — ignoré")
            return
        with self._lock:
            label = f"LM{self.beacon_count}"
            self.beacons.append((bwx, bwy, label))
        size  = lm.get("checker_size")
        ident = f"DAMIER {size[0]}×{size[1]}" if size else "DAMIER"
        dist_s = f" @ {lm['dist_est']} m" if lm.get("dist_est") else ""
        ang_s  = (f"  {lm['angle_deg']:+.1f}°" if lm.get("angle_deg") is not None else "")
        self._log(
            f"MAP LANDMARK #{self.beacon_count} [{ident}]{dist_s}{ang_s}"
            f" — monde ({wx:.2f}, {wy:.2f})")
        self._publish_marker("beacon", bwx, bwy, label)
        self._autolog.add(
            "MAP_LANDMARK", f"Map landmark #{self.beacon_count} — {ident}",
            f"Beacon world ({bwx:.2f}, {bwy:.2f}){dist_s}",
            tags=["landmark", "mapping", "checkerboard"],
        )
        self._tel_event.set()

    # ══════════════════════════════════════════════════════════════════════ #
    # Obstacle — callback profondeur D455 (throttlé par cooldown)
    # ══════════════════════════════════════════════════════════════════════ #
    def _on_depth(self, msg):
        """Flag obstacle si objet < OBSTACLE_DIST_MM dans le ROI central, et
        mesure l'espace libre gauche/droite (médianes) pour choisir le côté de
        contournement. ROI en fractions — indépendant de la résolution."""
        try:
            now = time.time()
            # Validation d'entrée (space-grade audit 2026-07-13) : un message
            # malformé ne doit ni lever silencieusement (la chaîne obstacles
            # serait désactivée sans témoin), ni faire un reshape faux.
            if (msg.width < 64 or msg.height < 48
                    or len(msg.data) != msg.width * msg.height * 2):
                self._depth_err(now, "frame depth invalide %dx%d len=%d"
                                % (msg.width, msg.height, len(msg.data)))
                return
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            # Décimation x2 (profilée 2026-07-13) : 4.04 -> 1.68 ms/frame
            # (-58 %), médianes déviant <0.5 %, p5 <7 % pire-cas — le seuil
            # 600 mm garde sa marge. Les stats se font sur la grille ::2.
            arr = arr[::2, ::2]
            w_d, h_d = arr.shape[1], arr.shape[0]
            fx0, fx1, fy0, fy1 = OBSTACLE_ROI_FRAC
            x0, x1 = int(w_d * fx0), max(int(w_d * fx0) + 1, int(w_d * fx1))
            y0, y1 = int(h_d * fy0), max(int(h_d * fy0) + 1, int(h_d * fy1))
            # espace libre par côté (même bande verticale, moitiés gauche/droite
            # de l'image entière) — mis à jour à chaque frame, sert au choix du
            # côté de contournement dans l'état AVOID
            band = arr[y0:y1, :]
            lv = band[:, :w_d // 2]
            rv = band[:, w_d // 2:]
            lv = lv[(lv > 100) & (lv < 8000)]
            rv = rv[(rv > 100) & (rv < 8000)]
            # Percentile 20, pas médiane (2026-07-24, MISSION CRITIQUE : "au
            # lieu de l'esquiver il s'en rapproche") — la MÉDIANE d'un
            # côté/secteur est insensible à un obstacle qui n'occupe qu'une
            # minorité de ses pixels (poteau, pied de chaise, coin de mur en
            # biais) : ce côté/secteur continue de lire "dégagé" alors qu'un
            # vrai obstacle y est présent. C'est exactement le signal qui
            # choisit le côté de pivot ici (_enter_avoid, repli sans "grand
            # chemin") ET le secteur visé par le cap "grand chemin" en
            # patrouille — un biais de mesure ici pouvait donc bel et bien
            # diriger le robot VERS l'obstacle au lieu de s'en éloigner. Le
            # percentile 20 reste beaucoup plus tolérant au bruit que le p5
            # du déclenchement dur (600 mm) — ce n'est qu'une préférence de
            # cap, pas un arrêt — mais ne peut plus être trompé par un
            # obstacle occupant une part significative du côté/secteur.
            raw_left_mm  = int(np.percentile(lv, 20)) if len(lv) > 20 else 8000
            raw_right_mm = int(np.percentile(rv, 20)) if len(rv) > 20 else 8000
            a = DEPTH_LR_EMA_ALPHA
            self._depth_left_mm  = int(a * raw_left_mm  + (1 - a) * self._depth_left_mm)
            self._depth_right_mm = int(a * raw_right_mm + (1 - a) * self._depth_right_mm)
            # Analyse "grand chemin" : même garde percentile 20 par secteur
            # angulaire (voir note ci-dessus). Conservateur : les pixels
            # invalides/hors portée ne comptent pas (inconnu != sûr) ; un
            # couloir > 8 m sature à la portée max.
            sw = w_d // OPEN_SECTORS
            sectors = []
            for i in range(OPEN_SECTORS):
                sv = band[:, i*sw:(i+1)*sw]
                sv = sv[(sv > 100) & (sv < 8000)]
                sectors.append(int(np.percentile(sv, 20)) if len(sv) > 30 else None)
            self._sector_mm = sectors
            self._sector_w  = w_d          # largeur de la grille DÉCIMÉE (cohérent avec les secteurs)
            self._sectors_t = now
            if now < self._obs_cooldown:
                return
            roi = arr[y0:y1, x0:x1]
            # ROI large = un obstacle fin (pied de chaise) se dilue dans le
            # percentile global. On découpe le couloir en 3 tiers : le p5 de
            # CHAQUE tiers est comparé au seuil — sensibilité locale conservée.
            w3 = max(1, roi.shape[1] // 3)
            p5s = []
            for k in range(3):
                sub = roi[:, k*w3:(k+1)*w3]
                v = sub[(sub > 100) & (sub < 8000)]
                if len(v) > 20:
                    p5s.append(int(np.percentile(v, 5)))
            self._corridor_p5_mm = min(p5s) if p5s else 8000
            self._corridor_t = now
            if p5s and min(p5s) < OBSTACLE_DIST_MM:
                self._obstacle_flag = True
                self._obs_cooldown = now + 2.0
        except Exception as e:
            # JAMAIS silencieux : une exception ici = chaîne obstacles morte.
            self._depth_err(time.time(), repr(e))

    def _depth_err(self, now, txt):
        """Erreur du callback depth, loggée au plus 1x/10 s (pas de spam à 15 Hz)."""
        if now - getattr(self, "_depth_err_t", 0.0) > 10.0:
            self._depth_err_t = now
            self._log(f"[DEPTH] ERREUR callback (chaîne obstacles dégradée) : {txt}")

    # ══════════════════════════════════════════════════════════════════════ #
    # Repère monde — persistance à travers les resets odométriques
    # ══════════════════════════════════════════════════════════════════════ #
    def _vfh_steer(self, now, goal_bearing=0.0):
        """VFH+ (Borenstein & Koren) simplifié : note chaque secteur de
        l'histogramme polaire déjà calculé en continu (_sector_mm, percentile
        20 — cf. 2026-07-24, ne peut plus être trompé par un obstacle fin
        occupant une minorité d'un secteur, contrairement à l'ancienne
        médiane) et renvoie le cap du meilleur candidat, lissé par EMA, ou
        None si aucun secteur n'atteint VFH_MIN_CLEARANCE_M — l'appelant doit
        alors se rabattre sur le filet de sécurité discret (_obstacle_flag,
        pivot AVOID), VFH+ n'invente jamais une échappatoire qui n'existe pas
        vraiment.

        score(secteur) = W_OPEN·ouverture − W_GOAL·|écart au but|
                                            − W_SMOOTH·|écart au cap précédent|

        bearing > 0 = à GAUCHE (convention yaw ROS, +z = CCW), même
        convention que le reste du fichier. goal_bearing=0 = "reste bien
        droit" (patrouille) ; passer le cap monde vers la balise mémorisée
        pour un but différent (cf. appel dans _enter_avoid)."""
        secs = getattr(self, "_sector_mm", None)
        if not secs or now - getattr(self, "_sectors_t", 0) > OPEN_FRESH_S:
            return None
        w = self._sector_w
        prev = self._vfh_prev_bearing
        best_i, best_bearing, best_score = None, None, float("-inf")
        for i, d_mm in enumerate(secs):
            if d_mm is None or d_mm / 1000.0 < VFH_MIN_CLEARANCE_M:
                continue  # exclu, pas seulement pénalisé (voir docstring)
            sector_center_px = (i + 0.5) * (w / float(OPEN_SECTORS))
            bearing_i = -(sector_center_px - w / 2.0) * (DEPTH_HFOV_RAD / w)
            openness = min(d_mm / 1000.0, VFH_OPEN_CAP_M) / VFH_OPEN_CAP_M
            goal_err = abs(math.atan2(math.sin(bearing_i - goal_bearing),
                                       math.cos(bearing_i - goal_bearing))) / math.pi
            smooth_err = 0.0
            if prev is not None:
                smooth_err = abs(math.atan2(math.sin(bearing_i - prev),
                                             math.cos(bearing_i - prev))) / math.pi
            score = (VFH_W_OPEN * openness
                     - VFH_W_GOAL * goal_err
                     - VFH_W_SMOOTH * smooth_err)
            if score > best_score:
                best_i, best_bearing, best_score = i, bearing_i, score
        if best_i is None:
            return None
        a = VFH_HEADING_EMA_ALPHA
        self._vfh_prev_bearing = (best_bearing if prev is None
                                   else a * best_bearing + (1 - a) * prev)
        return self._vfh_prev_bearing

    def _obstacle_speed_scale(self, now):
        """Facteur [OBSTACLE_MIN_SPEED_SCALE, 1.0] appliqué à ADVANCE_LIN —
        gouverneur PID continu additif (2026-07-22), ralentit progressivement
        avant le déclenchement binaire OBSTACLE_DIST_MM/le pivot AVOID, ne les
        remplace pas. Renvoie 1.0 (pas de ralentissement) si _corridor_p5_mm
        n'est pas frais — ne pas inventer un comportement "inconnu = lent",
        depth_alive gère déjà l'arrêt total si le flux depth est mort."""
        if now - getattr(self, "_corridor_t", 0.0) >= 1.0:
            self._last_obstacle_scale = 1.0  # caché pour la télémétrie (2026-07-22)
            return 1.0
        err = self._corridor_p5_mm - OBSTACLE_SLOWDOWN_MM  # positif = dégagé
        scale = max(OBSTACLE_MIN_SPEED_SCALE,
                    min(1.0, 1.0 + self._pid_obstacle.update(err, now)))
        self._last_obstacle_scale = scale
        return scale

    def _reset_all_pids(self):
        """Remet à zéro les 4 PID (+ la mémoire de cap VFH+, 2026-07-24) d'un
        coup — pour les portes de sécurité de _drive() (cam_alive/depth_alive
        morts, abandons internes en LOCK) qui ne sont PAS des transitions
        d'état FSM et donc pas couvertes par les reset() faits aux points
        d'entrée d'état (2026-07-22). Reset groupé plutôt que sélectif : même
        logique que la porte depth_alive elle-même, qui arrête TOUT le
        mouvement sans distinguer quel sous-comportement avait besoin de la
        profondeur — un cap VFH+ mémorisé d'avant une coupure depth n'a plus
        aucune raison d'être pondéré une fois le flux revenu, potentiellement
        après un déplacement significatif du robot ou de l'environnement."""
        self._pid_patrol.reset()
        self._pid_lock_align.reset()
        self._pid_lock_approach.reset()
        self._pid_obstacle.reset()
        self._vfh_prev_bearing = None

    def _nearest_marker(self, mtype, wx, wy):
        """Distance du marqueur existant de ce type le plus proche (inf si aucun)."""
        best = float("inf")
        for m in self.map_markers:
            if m.get("type") == mtype:
                best = min(best, math.hypot(wx - m["x"], wy - m["y"]))
        return best

    def _map_obstacle(self, context):
        """Marque l'obstacle courant sur la carte (repère monde, persistant),
        sauf si un obstacle est déjà cartographié à proximité (anti-doublon)."""
        wx, wy = self._get_world_pos()
        if self._nearest_marker("obstacle", wx, wy) < OBSTACLE_DEDUP_RADIUS_M:
            self._log(f"obstacle déjà cartographié près de ({wx:.2f}, {wy:.2f}) — pas de doublon")
            return
        self._obs_count += 1
        self._publish_marker("obstacle", wx, wy, f"OBS{self._obs_count}")
        self._log(f"OBSTACLE #{self._obs_count} at ({wx:.2f}, {wy:.2f}) — contournement")
        self._autolog.add("OBSTACLE_MAPPED",
                          f"Obstacle #{self._obs_count} mapped",
                          f"World ({wx:.2f}, {wy:.2f}) — {context}",
                          tags=["obstacle", "autonomous", context.lower()])

    def _enter_avoid(self, resume, now):
        """Entre en contournement réactif — appelé quand le déclenchement dur
        (_obstacle_flag, ROI centrale <600 mm) a frappé, c.-à-d. quand VFH+
        (_vfh_steer, consommé en continu par PATROL) n'a pas pu réagir à
        temps ou n'avait déjà plus de secteur exploitable. Pivot VISÉ vers
        le meilleur secteur ENCORE exploitable (même fonction de score que
        le pilotage continu — pas une heuristique séparée) si un tel secteur
        existe hors axe ; sinon repli sur un bounce aléatoire borné vers le
        côté le moins pire (2026-07-24 : VFH+ remplace le décalage fixe 35°
        systématique du point précédent — deuxième demande opérateur
        explicite, "évitement intelligent"). Chaque appel compte comme un
        essai — au-delà de AVOID_MAX_TRIES l'état AVOID se replie sur le
        demi-tour historique."""
        # Anti-coincement niveau 1 : trop de pivots en peu de temps -> échappement
        self._avoid_times.append(now)
        recent = [t for t in self._avoid_times if now - t < STUCK_WINDOW_S]
        if len(recent) >= STUCK_PIVOTS:
            self._avoid_times.clear()
            self._enter_escape(now, "pivots répétés (%d en %.0fs)" % (len(recent), STUCK_WINDOW_S))
            return
        self.auto_state      = "AVOID"
        self._avoid_resume   = resume
        self._avoid_phase    = "turn"
        self._avoid_tries   += 1
        ob = self._vfh_steer(now, 0.0)  # but = tout droit : "reviens vers l'axe"
        if ob is not None and abs(ob) > VFH_AIM_MIN_RAD:
            self._avoid_dir    = 1.0 if ob > 0 else -1.0
            self._avoid_target = min(max(abs(ob), BOUNCE_MIN_RAD * 0.5), BOUNCE_MAX_RAD)
            self._open_aimed   = True
        else:
            # aucun secteur exploitable : repli sur le côté le moins pire
            # (percentile 20, fix du jour) + bounce aléatoire borné —
            # composante stochastique volontaire, couverture ergodique
            self._avoid_dir    = 1.0 if self._depth_left_mm >= self._depth_right_mm else -1.0
            self._avoid_target = random.uniform(BOUNCE_MIN_RAD, BOUNCE_MAX_RAD)
            self._open_aimed   = False
        self._avoid_accum    = 0.0
        self._avoid_last_yaw = self.raw_yaw
        self._avoid_t0       = now
        side = "gauche" if self._avoid_dir > 0 else "droite"
        kind = "VISÉ (VFH+)" if self._open_aimed else "bounce (repli, rien d'exploitable)"
        self._log(f"AVOID {kind} {math.degrees(self._avoid_target):.0f}° par la {side} "
                  f"(libre G={self._depth_left_mm}mm D={self._depth_right_mm}mm, "
                  f"essai {self._avoid_tries}/{AVOID_MAX_TRIES})")

    def _enter_escape(self, now, reason):
        """Échappement anti-enfermement : recul court sécurisé puis grand pivot
        aléatoire. En dernier recours (répétition) : ARRÊT SÉCURISÉ + alerte."""
        self._escape_times.append(now)
        recent = [t for t in self._escape_times if now - t < ESCAPE_WINDOW_S]
        if len(recent) > ESCAPE_MAX:
            self._safe_stop()
            self.mode = "MANUEL"
            self._log("ALERTE: robot coincé malgré %d échappements — ARRÊT SÉCURISÉ, "
                      "reprise manuelle requise" % ESCAPE_MAX)
            self._autolog.add("STUCK_SAFE_STOP", "Robot coincé — arrêt sécurisé",
                              reason, tags=["safety", "autonomous", "stuck"])
            return
        self.auto_state      = "ESCAPE"
        self._escape_phase   = "back"
        self._escape_t0      = now
        self._escape_dir     = 1.0 if self._depth_left_mm >= self._depth_right_mm else -1.0
        self._escape_target  = random.uniform(ESCAPE_TURN_MIN, ESCAPE_TURN_MAX)
        self._escape_accum   = 0.0
        self._escape_last_yaw = self.raw_yaw
        self._log(f"ÉCHAPPEMENT ({reason}) : recul {abs(ESCAPE_BACK_LIN)*ESCAPE_BACK_S:.1f} m "
                  f"puis pivot {math.degrees(self._escape_target):.0f}°")

    def _get_world_pos(self):
        """Convertit la pose locale courante (odométrie roues) en position
        repère monde. Utilisée par PATROL/LOCK/AVOID/PID : ne dépend d'aucun
        estimateur, donc ne peut jamais tomber en panne d'initialisation."""
        lx, ly = float(self.pose[0]), float(self.pose[1])
        c = math.cos(self.world_heading)
        s = math.sin(self.world_heading)
        return (self.world_origin_x + c * lx - s * ly,
                self.world_origin_y + s * lx + c * ly)

    def _on_pose_fused(self, msg):
        self._fused_x = float(msg.pose.pose.position.x)
        self._fused_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self._fused_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._fused_t = time.time()

    def _rtb_world_pos(self):
        """Position + cap pour la trace/cible/pilotage RTB uniquement
        (2026-07-23) : /robot_pose_fused (VINS ou MINS, selon la sélection
        cockpit — voir pose_selector.py) si reçu depuis moins de
        RTB_FUSED_MAX_AGE, plus précis que l'odométrie roues seule ; sinon
        repli sur l'odométrie roues pour ne jamais bloquer RTB si
        l'estimateur actif est indisponible. Position et cap toujours pris
        de la MÊME source (jamais mélangés) pour que l'angle de pilotage
        reste cohérent avec la cible."""
        if time.time() - self._fused_t < RTB_FUSED_MAX_AGE:
            return (self._fused_x, self._fused_y, self._fused_yaw)
        wx, wy = self._get_world_pos()
        return (wx, wy, self.world_heading + float(self.pose[2]))

    def _publish_marker(self, mtype, wx, wy, label):
        """Publie un marqueur permanent sur /map/markers et l'ajoute à la liste interne."""
        marker = {
            "type":  mtype,
            "x":     round(wx, 3),
            "y":     round(wy, 3),
            "label": label,
            "id":    len(self.map_markers) + 1,
            "ts":    round(time.time(), 2),
        }
        self.map_markers.append(marker)
        try:
            if self._map_pub is not None:
                self._map_pub.publish(self._String(data=json.dumps(marker)))
        except Exception:
            pass


    # ══════════════════════════════════════════════════════════════════════ #
    # Thread CONTRÔLE — machine d'états (identique au dashboard)
    # ══════════════════════════════════════════════════════════════════════ #
    def _control_loop(self):
        period = 1.0 / CONTROL_HZ
        while not self._closing:
            t0 = time.time()
            try:
                self._check_heartbeat(t0)
                with self._lock:
                    img_t = self.img_t
                cam_alive = bool(img_t) and (time.time() - img_t) < CAM_ALIVE_TIMEOUT
                depth_t = getattr(self, "_corridor_t", 0.0)
                depth_alive = bool(depth_t) and (time.time() - depth_t) < DEPTH_ALIVE_TIMEOUT
                self._drive(cam_alive, depth_alive)
            except Exception:
                pass
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    def _check_heartbeat(self, now):
        """Operator deadman: if the UI heartbeat stops while the robot is in
        AUTO, force MANUAL with zero velocity.

        Armed only after the first heartbeat is seen.  If the browser has been
        silent for HEARTBEAT_STALE_S (browser closed / page reloaded), we
        reset the armed flag so the next reconnection re-arms cleanly and does
        not immediately trip the failsafe when the operator enters AUTO."""
        if not self._hb_armed:
            return
        hb_age = now - self._last_heartbeat_t
        # Browser gone for too long → silently disarm so a fresh connect works.
        if hb_age > HEARTBEAT_STALE_S and self.mode != "AUTO":
            self._hb_armed = False
            self._hb_lost  = False
            return
        if self.mode == "AUTO" and hb_age > HEARTBEAT_TIMEOUT:
            self.mode   = "MANUEL"
            self.manual = [0.0, 0.0]
            self._safe_stop()
            self._hb_lost = True
            self._log("HEARTBEAT LOST — failsafe → MANUAL, motors stopped")
            self._autolog.add(
                "HEARTBEAT_LOST", "Operator heartbeat lost — failsafe engaged",
                f"No UI heartbeat for >{HEARTBEAT_TIMEOUT:.1f}s while in AUTO",
                tags=["safety", "heartbeat", "failsafe"])

    def _fsm_state(self, now):
        """Single source of truth for the mission FSM — one exclusive state.

        MANUAL overrides everything (operator authority). AVOID_OBSTACLE and
        LOCK_BEACON only appear in AUTO; RESET_ODOMETRY is a brief flash shown
        right after any odometry zeroing, in either mode."""
        reset_active = (now - self._fsm_reset_t) < FSM_RESET_FLASH_S
        if self.mode == "MANUEL":
            return FSM_RESET_ODOM if reset_active else FSM_MANUAL
        # AUTO sans caméra : _drive() refuse de bouger (patrouille aveugle =
        # ni balises ni obstacles) — l'état doit le DIRE au lieu d'afficher
        # un PATROL immobile inexplicable.
        cam_ok = bool(self.img_t) and (now - self.img_t) < CAM_ALIVE_TIMEOUT
        if not cam_ok and self.auto_state not in ("RTB", "GOTO_BEACON"):
            return FSM_AUTO_NOCAM
        # AUTO sans depth (2026-07-21) : flux indépendant du cam_ok ci-dessus —
        # _drive() refuse aussi de bouger dans ce cas (voir DEPTH_ALIVE_TIMEOUT).
        depth_t = getattr(self, "_corridor_t", 0.0)
        depth_ok = bool(depth_t) and (now - depth_t) < DEPTH_ALIVE_TIMEOUT
        if not depth_ok and self.auto_state not in ("RTB", "GOTO_BEACON"):
            return FSM_AUTO_NODEPTH
        # AUTO sub-states (priority: obstacle > reset flash > lock > patrol)
        if self.auto_state in ("AVOID", "ESCAPE"):
            return FSM_AVOID_OBST
        if self.auto_state == "SPIRAL":
            return FSM_AUTO_PATROL
        if self.auto_state == "U_TURN" and self._uturn_reason == "obstacle":
            return FSM_AVOID_OBST
        if reset_active:
            return FSM_RESET_ODOM
        if self.auto_state in ("LOCK", "WAIT"):
            return FSM_LOCK_BEACON
        if self.auto_state == "GOTO_BEACON":
            return FSM_GOTO_BEACON
        return FSM_AUTO_PATROL

    def _drive(self, cam_alive, depth_alive=True):
        if not self.connected or self.cmd_pub is None:
            return
        now = time.time()
        lin = ang = 0.0

        if self.mode == "AUTO" and self.auto_state not in ("RTB", "GOTO_BEACON"):
            if not cam_alive:
                self._reset_all_pids()  # porte de sécurité, pas une transition d'état (2026-07-22)
                self._publish(0, 0)
                return
            if not depth_alive:
                # cam_alive (color/IR) and depth are independent WiFi-crossing
                # topics — a live IR feed does not mean obstacle sensing is
                # live. Never advance blind: see DEPTH_ALIVE_TIMEOUT.
                self._reset_all_pids()
                self._publish(0, 0)
                return

        if self.mode == "MANUEL":
            # Homme-mort : sans ordre récent -> arrêt (sécurité tablette).
            if now - self.manual_t < MANUAL_DEADMAN:
                lin, ang = self.manual
            else:
                lin, ang = 0.0, 0.0

        elif self.mode == "AUTO":
            det = self.det
            c = self._target_center(det)
            # ── Couverture : mémoire de balise + détection de coincement ──
            if c is not None:
                self._last_beacon_seen_t = now
                # direction monde approximative de la balise (cap + offset px,
                # ~0.00236 rad/px pour fx=336 @640) — sert de biais de recherche
                if self.last_w:
                    off = (c[0] - self.last_w / 2.0) * 0.00236
                    self._beacon_dir_world = (self.world_heading + float(self.pose[2])) - off
            if now - self._disp_last_t > 1.0:
                self._disp_last_t = now
                wx_s, wy_s = self._get_world_pos()
                self._disp_hist.append((now, wx_s, wy_s))
                # niveau 2 : quasi-immobile sur la fenêtre alors qu'on "roule"
                # LOCK et GOTO_BEACON ajoutés (2026-07-24, revue de fiabilité
                # AUTO) : ces deux états avancent aussi vers une cible et
                # n'avaient jusqu'ici AUCUN filet de sécurité indépendant de
                # la depth — un obstacle bas/haut hors de la bande verticale
                # de l'OBSTACLE_ROI_FRAC (ou toute autre cause de blocage
                # physique) pouvait y pousser le robot indéfiniment contre un
                # obstacle sans jamais déclencher l'échappement. RTB et
                # ESCAPE restent volontairement exclus : RTB a son propre
                # garde dédié plus réactif (RTB_STALL_*), et ESCAPE EST déjà
                # la réponse à ce détecteur (l'y inclure créerait une boucle).
                if (self.auto_state in ("PATROL", "SPIRAL", "AVOID", "LOCK", "GOTO_BEACON")
                        and len(self._disp_hist) > 3):
                    t_old, x_old, y_old = self._disp_hist[0]
                    if (now - t_old >= STUCK_DISP_WIN_S
                            and math.hypot(wx_s - x_old, wy_s - y_old) < STUCK_MIN_DISP_M):
                        self._disp_hist.clear()
                        self._enter_escape(now, "déplacement < %.2f m en %.0f s"
                                           % (STUCK_MIN_DISP_M, STUCK_DISP_WIN_S))

            # ── Obstacle (phase d'avance de PATROL) ───────────────────────
            # `c is None` = priorité balise : si la vision voit la balise, ce
            # qui est devant N'EST PAS un obstacle à contourner, c'est la
            # cible — on laisse les branches LOCK prendre la main.
            if (self.auto_state == "PATROL" and self._patrol_advancing
                    and self._obstacle_flag and c is None):
                self._obstacle_flag = False
                self._obs_cooldown = now + 2.0
                self._map_obstacle("PATROL")
                self._enter_avoid("PATROL", now)
                lin, ang = 0.0, 0.0

            elif self.auto_state == "PATROL":
                # ── Refonte 2026-07-08 : LIGNE DROITE permanente ──────────
                # Avance tout droit ; balise vue -> LOCK ; obstacle -> pivot
                # 90° (branche ci-dessus). Pas de scan 360°.
                if c is not None and now >= getattr(self, "_lock_block_until", 0.0):
                    self.auto_state = "LOCK"
                    self.lock_start_t = now
                    self.lock_centered = False
                    self._dual_confirm_t0 = None
                    self._pid_lock_align.reset()
                    self._pid_lock_approach.reset()
                    src = "vision" if (det.get("center") is not None) else "tag"
                    h_img = (self.last_w * 3 // 4) if self.last_w else 480
                    ypc = int(100.0 * c[1] / h_img) if len(c) > 1 else -1
                    self._log(f"Beacon detected — LOCK ON (source={src}, y={ypc}%)")
                elif now - self._last_beacon_seen_t > SPIRAL_AFTER_S:
                    # longtemps sans balise -> spirale d'exploration (rayon
                    # croissant), orientée vers la dernière direction de balise
                    # aperçue si la mémoire est fraîche (gradient balise)
                    self.auto_state  = "SPIRAL"
                    self._spiral_t0  = now
                    self._spiral_dir = random.choice((-1.0, 1.0))
                    hint = ""
                    if (self._beacon_dir_world is not None
                            and now - self._last_beacon_seen_t < TAG_MEMORY_S):
                        cur = self.world_heading + float(self.pose[2])
                        d = math.atan2(math.sin(self._beacon_dir_world - cur),
                                       math.cos(self._beacon_dir_world - cur))
                        self._spiral_dir = 1.0 if d > 0 else -1.0
                        hint = " (biais vers dernière balise aperçue)"
                    self._log(f"Aucune balise depuis {SPIRAL_AFTER_S:.0f}s — "
                              f"spirale d'exploration{hint}")
                else:
                    lin = ADVANCE_LIN * self._obstacle_speed_scale(now)
                    # VFH+ (2026-07-24) : pilotage continu, remplace le
                    # "tout droit jusqu'à obstacle" de la simplification du
                    # même jour — deuxième demande opérateur explicite,
                    # "évitement intelligent". But = tout droit (0) par
                    # défaut, ce qui EST le comportement "reste bien droit
                    # dans le couloir" demandé : quand tous les secteurs
                    # sont dégagés et similaires, le terme but du score
                    # domine et VFH+ choisit le secteur le plus proche de
                    # l'axe. Biais vers la dernière balise aperçue si sa
                    # mémoire est fraîche (même logique que SPIRAL
                    # ci-dessus) — chercher la balise reste la mission,
                    # pas seulement "avancer". Balise EN VUE (branche LOCK
                    # ci-dessus) reste prioritaire sur tout ceci ; le
                    # déclenchement dur (_obstacle_flag, ROI centrale
                    # <600 mm) reste inchangé au-dessus et bascule en AVOID
                    # AVANT ce bloc si quelque chose est droit devant, trop
                    # près pour qu'un simple pilotage de cap suffise.
                    goal_bearing = 0.0
                    if (self._beacon_dir_world is not None
                            and now - self._last_beacon_seen_t < TAG_MEMORY_S):
                        cur = self.world_heading + float(self.pose[2])
                        goal_bearing = math.atan2(
                            math.sin(self._beacon_dir_world - cur),
                            math.cos(self._beacon_dir_world - cur))
                    bearing = self._vfh_steer(now, goal_bearing)
                    if bearing is not None:
                        ang = self._pid_patrol.update(bearing, now)
                        if now - getattr(self, "_vfh_log_t", 0.0) > 5.0:
                            self._vfh_log_t = now
                            self._log(f"VFH+ cap {math.degrees(bearing):+.0f}° "
                                      f"(but {math.degrees(goal_bearing):+.0f}°)")
                    else:
                        # aucun secteur exploitable : pas d'invention de cap,
                        # tout droit par défaut (ang=0, déjà la valeur de
                        # base) — le déclenchement dur ci-dessus reste le
                        # seul filet de sécurité si ça se dégrade encore
                        self._vfh_prev_bearing = None
                        self._pid_patrol.reset()

            elif self.auto_state == "LOCK":
                # ── Centrage caméra sur la balise ─────────────────────────
                progress_t = max(self.lock_start_t,
                                 getattr(self, "_lock_progress_t", 0.0))
                if (not self.lock_centered) and (now - progress_t) > LOCK_STALL_S:
                    self._lock_block_until = now + LOCK_COOLDOWN_S
                    self._log(f"LOCK sans convergence ({LOCK_STALL_S:.0f} s) — "
                              f"cible non atteignable (artefact/reflet ?) — "
                              f"abandon, PATROL sans re-lock {LOCK_COOLDOWN_S:.0f} s")
                    self._start_patrol()
                elif c is None and (now - self.lock_start_t) > LOCK_TIMEOUT:
                    self._log("Lock timeout — balise perdue — retour PATROL")
                    self._start_patrol()
                elif self.lock_centered:
                    # Centrage acquis : APPROCHE tant que la balise est loin
                    # (distance tag AprilTag, sinon damier), puis rituel.
                    d_tag = (self._tag_dist
                             if (getattr(self, "_tag_dist", None) is not None
                                 and now - getattr(self, "_tag_center_t", 0.0) < 0.7)
                             else None)
                    det_lm = det.get("map_landmark", {})
                    d_cb = det_lm.get("dist_est") if det_lm.get("valid") else None
                    dist = d_tag if d_tag is not None else d_cb
                    if dist is not None and dist > TARGET_STOP_M:
                        self._dual_confirm_t0 = None  # encore en approche : pas de fenêtre de grâce active
                        # Obstruction pendant l'approche : la cible est censée
                        # être à >0.6 m mais le depth voit un objet plus proche
                        # devant — contradiction (artefact ou balise occluse)
                        # -> abandon + contournement. (Avant : approche AVEUGLE,
                        # collisions sur faux LOCK.)
                        if self._obstacle_flag:
                            self._obstacle_flag = False
                            self._lock_block_until = now + LOCK_COOLDOWN_S
                            self._log("Obstruction pendant l'approche LOCK — "
                                      "abandon + contournement")
                            self._map_obstacle("LOCK")
                            self._enter_avoid("PATROL", now)
                            self._reset_all_pids()
                            return  # tick sans publication — AVOID publie dès le tick suivant
                        # avance en gardant le centrage (correction douce)
                        self._lock_progress_t = now
                        lin = ADVANCE_LIN * self._obstacle_speed_scale(now)
                        if c is not None and self.last_w:
                            err = c[0] - self.last_w / 2.0
                            ang = -self._pid_lock_approach.update(err, now)
                            # Hystérésis (2026-07-21, mêmes unités que l'entrée
                            # ci-dessous) : ne redevient "non centré" que
                            # passé LOCK_CENTER_EXIT_TOL, plus large que le
                            # seuil d'entrée LOCK_CENTER_TOL — sans ça un
                            # centre détecté qui oscille près de la frontière
                            # fait clignoter lock_centered et alterne les
                            # commandes avance/recentrage à chaque tick.
                            err_frac = err / (self.last_w / 2.0 + 1e-6)
                            self.lock_centered = abs(err_frac) < LOCK_CENTER_EXIT_TOL
                    elif dist is not None:
                        # TOUT-EN-UN (2026-07-13), durci (2026-07-22) : balise
                        # trouvée ET distance MESURÉE (tag AprilTag ou damier)
                        # <= 0.6 m -> plus assez pour reset seul. On exige EN
                        # PLUS que LED et damier soient confirmés SIMULTANÉMENT
                        # (self._dual_beacon_confirmed) avant de zéroter
                        # l'odométrie — le damier donne une pose sub-pixel bien
                        # plus fiable que le centre LED seul, donc un reset
                        # sans lui n'est plus jugé assez précis. Fenêtre de
                        # grâce bornée (DUAL_CONFIRM_GRACE_S) le temps que les
                        # deux confirment ; abandon + contournement sinon (pas
                        # de blocage indéfini face à la balise).
                        self._last_lock_dist = float(dist)
                        if self._dual_confirm_t0 is None:
                            self._dual_confirm_t0 = now
                        # Reset sur LED SEULE (2026-07-27, même revert qu'en
                        # MANUEL ci-dessus) : cb_ok toujours calculé (log/
                        # diagnostic) mais plus exigé pour déclencher.
                        led_ok, cb_ok = self._dual_beacon_confirmed(det)
                        if led_ok:
                            self._dual_confirm_t0 = None
                            self._lock_block_until = time.time() + LOCK_COOLDOWN_S
                            # ^ garde post-visite (2026-07-13) : sans elle le robot
                            # re-verrouillait la balise qu'il venait de visiter
                            # (encore en vue pendant le demi-tour) — ping-pong
                            self.reset_coords_local()
                            self.auto_state = "WAIT"
                            self.wait_start_t = now
                            self._log(f"Balise trouvée à {dist:.2f} m — LED confirmée "
                                      f"(damier {'aussi' if cb_ok else 'non'}) — reset "
                                      f"local — WAIT {WAIT_DURATION:.0f} s")
                        elif now - self._dual_confirm_t0 > DUAL_CONFIRM_GRACE_S:
                            self._dual_confirm_t0 = None
                            self._lock_block_until = now + LOCK_COOLDOWN_S
                            self._log(f"Arrivé à {dist:.2f} m mais LED+damier jamais "
                                      f"confirmés ensemble ({DUAL_CONFIRM_GRACE_S:.0f} s) "
                                      f"— abandon + contournement")
                            self._map_obstacle("LOCK")
                            self._enter_avoid("PATROL", now)
                            self._reset_all_pids()
                            return  # tick sans publication — AVOID publie dès le tick suivant
                        # sinon : robot immobile (lin=ang=0, défaut), on tient
                        # la fenêtre de grâce en attendant la double confirmation
                    elif self.lock_centered:
                        # SANS distance mesurée (tag absent de cette face,
                        # damier hors portée >1.5 m) : on approche quand même,
                        # le DEPTH du couloir central fait foi (la balise est
                        # un objet physique). Arrivée < 0.7 m -> reset. Le
                        # seuil est AU-DESSUS du seuil obstacle (0.55 m) :
                        # l'arrivée se déclenche avant tout conflit.
                        cp = getattr(self, "_corridor_p5_mm", 8000)
                        fresh = now - getattr(self, "_corridor_t", 0.0) < 1.0
                        if fresh and cp < 700:
                            # Reset sur LED SEULE (2026-07-27, revert de la
                            # double confirmation du 22/07, sur décision
                            # opérateur explicite) : cette branche existe
                            # PRÉCISÉMENT pour le cas où le damier est hors
                            # portée — exiger cb_ok ici expirait donc
                            # quasi systématiquement la fenêtre de grâce
                            # sans jamais confirmer la balise. cb_ok reste
                            # calculé pour le log, plus pour bloquer.
                            self._obstacle_flag = False
                            self._last_lock_dist = cp / 1000.0
                            if self._dual_confirm_t0 is None:
                                self._dual_confirm_t0 = now
                            led_ok, cb_ok = self._dual_beacon_confirmed(det)
                            if led_ok:
                                self._dual_confirm_t0 = None
                                # garde post-visite (2026-07-13) : sans elle le
                                # robot re-verrouillait la balise qu'il venait de
                                # visiter (encore en vue pendant le demi-tour)
                                self._lock_block_until = time.time() + LOCK_COOLDOWN_S
                                self.reset_coords_local()
                                self.auto_state = "WAIT"
                                self.wait_start_t = now
                                self._log(f"Balise atteinte (depth {cp/1000.0:.2f} m, "
                                          f"LED confirmée{', damier aussi' if cb_ok else ''}) "
                                          f"— reset local — WAIT {WAIT_DURATION:.0f} s")
                            elif now - self._dual_confirm_t0 > DUAL_CONFIRM_GRACE_S:
                                self._dual_confirm_t0 = None
                                self._lock_block_until = now + LOCK_COOLDOWN_S
                                self._log(f"Arrivé (depth {cp/1000.0:.2f} m) mais LED+damier "
                                          f"jamais confirmés ensemble ({DUAL_CONFIRM_GRACE_S:.0f} s) "
                                          f"— abandon + contournement")
                                self._map_obstacle("LOCK")
                                self._enter_avoid("PATROL", now)
                                self._reset_all_pids()
                                return
                            # sinon : robot immobile, fenêtre de grâce en cours
                        else:
                            # ── Garde manquante (2026-07-24, MISSION CRITIQUE
                            # collision) : cette branche (LOCK centré mais SANS
                            # distance métrique — détection LED seule, ni tag
                            # ni damier valide, cf. detection.checker=false
                            # dans la télémétrie de l'incident) n'avait NI
                            # vérification _obstacle_flag NI gouverneur de
                            # vitesse, contrairement à sa branche sœur
                            # "dist is not None" ci-dessus ET à PATROL/SPIRAL :
                            # le robot avançait à ADVANCE_LIN plein régime,
                            # aveugle à tout obstacle, jusqu'à ce que le
                            # corridor descende sous 700 mm (aucun frein
                            # progressif avant). Même garde que la branche sœur.
                            if self._obstacle_flag:
                                self._obstacle_flag = False
                                self._lock_block_until = now + LOCK_COOLDOWN_S
                                self._log("Obstruction pendant l'approche LOCK "
                                          "(détection sans distance mesurée) — "
                                          "abandon + contournement")
                                self._map_obstacle("LOCK")
                                self._enter_avoid("PATROL", now)
                                self._reset_all_pids()
                                return  # tick sans publication — AVOID publie dès le tick suivant
                            self._dual_confirm_t0 = None  # encore en approche
                            self._lock_progress_t = now
                            lin = ADVANCE_LIN * self._obstacle_speed_scale(now)
                            if c is not None and self.last_w:
                                err = c[0] - self.last_w / 2.0
                                ang = -self._pid_lock_approach.update(err, now)
                elif c is not None:
                    if self.last_w:
                        err = c[0] - self.last_w / 2.0
                        err_frac = err / (self.last_w / 2.0 + 1e-6)
                        if abs(err_frac) > LOCK_CENTER_TOL:
                            ang = -self._pid_lock_align.update(err_frac, now)
                        else:
                            self.lock_centered = True
                            self._pid_lock_align.reset()

            elif self.auto_state == "WAIT":
                # ── Attente immobile WAIT_DURATION secondes ───────────────
                if now - self.wait_start_t >= WAIT_DURATION:
                    wx, wy = self._get_world_pos()
                    # Estimate real beacon position: robot was aligned → angle ≈ 0
                    det_lm = det.get("map_landmark", {})
                    b_dist = det_lm.get("dist_est") if det_lm.get("valid") else None
                    b_ang  = det_lm.get("angle_deg", 0.0) if det_lm.get("valid") else 0.0
                    if b_dist is None:
                        # tout-en-un : sans damier, la distance du LOCK (tag)
                        # place la balise — le robot était centré dessus (angle 0)
                        b_dist = getattr(self, "_last_lock_dist", None)
                        b_ang  = 0.0
                    if b_dist:
                        robot_h = self.world_heading + float(self.pose[2])
                        bearing = robot_h + math.radians(float(b_ang))
                        bwx = wx + float(b_dist) * math.cos(bearing)
                        bwy = wy + float(b_dist) * math.sin(bearing)
                    else:
                        bwx, bwy = wx, wy
                    # Anti-doublon : la MÊME balise revisitée ne doit ni
                    # ré-incrémenter le compteur ni empiler un marqueur —
                    # le reset odométrique et le demi-tour ont déjà eu lieu,
                    # le cycle continue simplement.
                    if self._nearest_marker("beacon", bwx, bwy) < BEACON_DEDUP_RADIUS_M:
                        self._log(f"balise déjà cartographiée près de ({bwx:.2f}, {bwy:.2f})"
                                  " — pas de doublon, reprise du cycle")
                    else:
                        self.beacon_count += 1
                        label = f"B{self.beacon_count}"
                        with self._lock:
                            self.beacons.append((bwx, bwy, label))
                        self._publish_marker("beacon", bwx, bwy, label)
                        self._log(
                            f"BEACON #{self.beacon_count} at ({bwx:.2f}, {bwy:.2f}) — U-TURN")
                        self._autolog.add("BEACON_LOCKED",
                                          f"Beacon #{self.beacon_count} acquired",
                                          f"World ({bwx:.2f}, {bwy:.2f}) — U-turn.",
                                          tags=["beacon", "autonomous", "lock"])
                    self.auto_state = "U_TURN"
                    self._uturn_reason = "patrol"
                    self._uturn_resume  = "PATROL"
                    # demi-tour (180°) après la pause balise (2026-07-22)
                    self._uturn_target  = POST_BEACON_TURN_RAD
                    self.uturn_accum = 0.0
                    self.uturn_last_yaw = self.raw_yaw
                # lin = ang = 0.0 — robot immobile

            elif self.auto_state == "AVOID":
                # ── Contournement réactif d'obstacle ──────────────────────
                # turn (pivot vers le côté libre) -> sidestep (avance) ->
                # returnturn (re-pivot) -> reprise du cycle. La balise garde
                # priorité absolue : si la vision la voit, LOCK immédiat.
                if c is not None and now >= getattr(self, "_lock_block_until", 0.0):
                    self._avoid_tries = 0
                    self.auto_state = "LOCK"
                    self.lock_start_t = now
                    self.lock_centered = False
                    self._dual_confirm_t0 = None
                    self._pid_lock_align.reset()
                    self._pid_lock_approach.reset()
                    self._log("Balise vue pendant contournement — LOCK ON")
                elif self._avoid_tries > AVOID_MAX_TRIES:
                    # zone trop encombrée — repli sur le comportement
                    # historique : demi-tour, puis reprise du cycle
                    self._avoid_tries  = 0
                    self.auto_state    = "U_TURN"
                    self._uturn_reason = "obstacle"
                    self._uturn_resume = self._avoid_resume
                    self._uturn_target = math.pi   # repli anti-enfermement : 180°
                    self.uturn_accum   = 0.0
                    self.uturn_last_yaw = self.raw_yaw
                    self._log("Contournement impossible (zone encombrée) — demi-tour")
                else:
                    # ── Pivot vers _avoid_dir/_avoid_target, choisis dans
                    # _enter_avoid (visée VFH+ ou repli bounce, 2026-07-24),
                    # puis reprise tout droit dans le NOUVEAU cap. Direction
                    # et angle ne sont jamais recalculés ici — un obstacle
                    # encore présent après ce pivot redéclenche tout le
                    # cycle (_enter_avoid ré-évalué, nouveau essai).
                    ang = self._avoid_dir * UTURN_SPEED
                    if self.raw_yaw is not None:
                        if self._avoid_last_yaw is None:
                            self._avoid_last_yaw = self.raw_yaw
                        d_yaw = math.atan2(
                            math.sin(self.raw_yaw - self._avoid_last_yaw),
                            math.cos(self.raw_yaw - self._avoid_last_yaw))
                        self._avoid_accum += abs(d_yaw)
                        self._avoid_last_yaw = self.raw_yaw
                    if self._avoid_accum >= getattr(self, '_avoid_target', AVOID_TURN_RAD):
                        self._avoid_tries   = 0
                        self._obstacle_flag = False
                        resume = self._avoid_resume
                        if resume == "GOTO_BEACON" and self._target_beacon_pos is not None:
                            self.auto_state = "GOTO_BEACON"
                        else:
                            self.auto_state        = "PATROL"
                            self._patrol_advancing = True
                            self.adv_start         = now
                        self._log("Pivot 90° terminé — reprise tout droit")

            elif self.auto_state == "SPIRAL":
                # ── Spirale d'exploration : rayon croissant ────────────────
                if c is not None and now >= getattr(self, "_lock_block_until", 0.0):
                    self.auto_state = "LOCK"
                    self.lock_start_t = now
                    self.lock_centered = False
                    self._dual_confirm_t0 = None
                    self._pid_lock_align.reset()
                    self._pid_lock_approach.reset()
                    self._log("Balise vue en spirale — LOCK ON")
                elif self._obstacle_flag:
                    self._obstacle_flag = False
                    self._obs_cooldown = now + 2.0
                    self._map_obstacle("SPIRAL")
                    self._enter_avoid("PATROL", now)
                elif now - self._spiral_t0 > SPIRAL_MAX_S:
                    self._start_patrol()
                    self._log("Spirale terminée — reprise ligne droite")
                else:
                    lin = ADVANCE_LIN * self._obstacle_speed_scale(now)
                    ang = self._spiral_dir * SPIRAL_ANG0 / (
                        1.0 + SPIRAL_DECAY * (now - self._spiral_t0))

            elif self.auto_state == "ESCAPE":
                # ── Échappement anti-enfermement ──────────────────────────
                # Recul COURT et LENT : la profondeur ne couvre pas l'arrière,
                # c'est la seule marche arrière autorisée du système (0,2 m).
                if self._escape_phase == "back":
                    if now - self._escape_t0 < ESCAPE_BACK_S:
                        lin = ESCAPE_BACK_LIN
                    else:
                        self._escape_phase   = "turn"
                        self._escape_accum   = 0.0
                        self._escape_last_yaw = self.raw_yaw
                else:
                    ang = self._escape_dir * UTURN_SPEED
                    if self.raw_yaw is not None:
                        if self._escape_last_yaw is None:
                            self._escape_last_yaw = self.raw_yaw
                        d_yaw = math.atan2(
                            math.sin(self.raw_yaw - self._escape_last_yaw),
                            math.cos(self.raw_yaw - self._escape_last_yaw))
                        self._escape_accum += abs(d_yaw)
                        self._escape_last_yaw = self.raw_yaw
                    if self._escape_accum >= self._escape_target:
                        self._log("Échappement terminé — reprise patrouille")
                        self._start_patrol()

            elif self.auto_state == "U_TURN":
                # ── Demi-tour 180° ────────────────────────────────────────
                ang = UTURN_SPEED
                if self.raw_yaw is not None:
                    if self.uturn_last_yaw is None:
                        self.uturn_last_yaw = self.raw_yaw
                    d_yaw = math.atan2(
                        math.sin(self.raw_yaw - self.uturn_last_yaw),
                        math.cos(self.raw_yaw - self.uturn_last_yaw))
                    self.uturn_accum += abs(d_yaw)
                    self.uturn_last_yaw = self.raw_yaw
                if self.uturn_accum >= getattr(self, "_uturn_target", math.pi):
                    self._last_center   = None
                    self._last_center_t = 0.0
                    resume = getattr(self, '_uturn_resume', 'PATROL')
                    if resume == "GOTO_BEACON" and self._target_beacon_pos is not None:
                        self.auto_state     = "GOTO_BEACON"
                        self._obstacle_flag = False
                        self._log("Demi-tour terminé — reprise GOTO_BEACON")
                    else:
                        self._start_patrol()
                        self._log("Demi-tour terminé — reprise PATROL")
                    self._uturn_resume = "PATROL"

            elif self.auto_state == "GOTO_BEACON":
                # ── Navigation vers une balise enregistrée ─────────────────
                if self._obstacle_flag and c is None:
                    # Obstacle (et PAS la balise — priorité vision) →
                    # contournement, puis reprise du GOTO_BEACON
                    self._obstacle_flag = False
                    self._obs_cooldown  = now + 3.0
                    self._map_obstacle("GOTO_BEACON")
                    self._enter_avoid("GOTO_BEACON", now)
                    lin, ang = 0.0, 0.0
                elif self._target_beacon_pos is None:
                    self._start_patrol()
                else:
                    tx, ty = self._target_beacon_pos
                    wx_g, wy_g = self._get_world_pos()
                    dist_to_tgt = math.hypot(tx - wx_g, ty - wy_g)
                    if c is not None:
                        # Beacon visible → passer directement en LOCK
                        self.auto_state   = "LOCK"
                        self.lock_start_t = now
                        self.lock_centered = False
                        self._dual_confirm_t0 = None
                        self._pid_lock_align.reset()
                        self._pid_lock_approach.reset()
                        self._log(f"[GOTO] Balise #{self._target_beacon_id} visible — LOCK ON")
                    elif dist_to_tgt < GOTO_BEACON_DIST_TOL:
                        # Position atteinte sans beacon visible → arrêt + PATROL
                        self._safe_stop()
                        self.mode   = "MANUEL"
                        self.manual = [0.0, 0.0]
                        self._log(f"[GOTO] Position balise #{self._target_beacon_id} atteinte — arrêt")
                        self._autolog.add("GOTO_COMPLETE",
                                          f"Goto beacon #{self._target_beacon_id} complete",
                                          f"World ({wx_g:.2f}, {wy_g:.2f})",
                                          tags=["goto_beacon"])
                        self._target_beacon_id  = None
                        self._target_beacon_pos = None
                        self._tel_event.set()
                    else:
                        # Navigation RTB-like vers la cible
                        robot_h = self.world_heading + float(self.pose[2])
                        ang_to_tgt = math.atan2(ty - wy_g, tx - wx_g)
                        ang_err = math.atan2(
                            math.sin(ang_to_tgt - robot_h),
                            math.cos(ang_to_tgt - robot_h))
                        if abs(ang_err) > GOTO_BEACON_ANG_TOL:
                            ang = math.copysign(GOTO_BEACON_ANG_SPEED, ang_err)
                        else:
                            # Déjà protégé par la garde _obstacle_flag dure en
                            # tête de cette branche (if/elif/else exclusif) ;
                            # gouverneur ajouté (2026-07-24) pour un frein
                            # progressif AVANT ce seuil dur, comme PATROL/SPIRAL.
                            lin = min(GOTO_BEACON_LIN_SPEED, dist_to_tgt * 0.4) \
                                * self._obstacle_speed_scale(now)
                            if abs(ang_err) > 0.06:  # correction cap douce
                                ang = math.copysign(
                                    GOTO_BEACON_ANG_SPEED * 0.3, ang_err)

            elif self.auto_state == "RTB":
                # ── Return To Base : retrace la trace enregistrée (repère
                # monde, VINS/MINS via pose_selector si frais — voir
                # _rtb_world_pos()), point par point, plutôt qu'une ligne
                # droite vers l'origine — voir _start_rtb() et RTB_TRAIL_* ──
                wx, wy, robot_heading = self._rtb_world_pos()

                # ── Garde de sécurité RTB (2026-07-23, voir RTB_STALL_*) ──
                # Double garde ADDITIVE au pilotage ci-dessous, arrêt complet
                # dans les deux cas (pas de recul supplémentaire, pas de
                # pivot — RTB l'interdit déjà pour la précision de pose, et
                # reculer/pivoter contre un contact déjà établi aggraverait
                # les choses) :
                #  1) obstacle AVANT (depth, si frais) : la correction de cap
                #     peut amener l'avant vers un danger que la depth voit.
                #  2) blocage ARRIÈRE (odométrie/pose fusionnée) : seul signal
                #     disponible dans le sens de marche réel (aucun capteur
                #     ne couvre l'arrière, cf. ESCAPE_BACK_LIN) — si la
                #     position ne bouge pas alors qu'un recul est commandé,
                #     c'est un contact physique.
                if self._obstacle_flag:
                    self._obstacle_flag = False
                    self._rtb_stall_ref_pos = None
                    self.mode = "MANUEL"
                    self.manual = [0.0, 0.0]
                    self._safe_stop()
                    self._log("RTB — obstacle détecté (avant) — arrêt d'urgence, RTB interrompu")
                    self._autolog.add(
                        "RTB_OBSTACLE", "RTB interrupted — obstacle detected",
                        f"Forward depth obstacle flag during reverse retrace at "
                        f"({wx:.2f}, {wy:.2f}) — {len(self._rtb_waypoints)} waypoint(s) remaining",
                        tags=["safety", "rtb", "obstacle"])
                    self._tel_event.set()
                    self._publish(0, 0)
                    return
                if self._rtb_stall_ref_pos is None or math.hypot(
                        wx - self._rtb_stall_ref_pos[0], wy - self._rtb_stall_ref_pos[1]
                        ) > RTB_STALL_MIN_DISP_M:
                    self._rtb_stall_ref_pos = (wx, wy)
                    self._rtb_stall_ref_t   = now
                elif now - self._rtb_stall_ref_t > RTB_STALL_WINDOW_S:
                    self._rtb_stall_ref_pos = None
                    self.mode = "MANUEL"
                    self.manual = [0.0, 0.0]
                    self._safe_stop()
                    self._log(
                        f"RTB — blocage détecté (<{RTB_STALL_MIN_DISP_M*100:.0f} cm en "
                        f"{RTB_STALL_WINDOW_S:.1f} s malgré commande de recul) — "
                        f"arrêt d'urgence, RTB interrompu")
                    self._autolog.add(
                        "RTB_STALL", "RTB interrupted — no rear sensor, motion stalled",
                        f"No displacement >{RTB_STALL_MIN_DISP_M:.2f} m in "
                        f"{RTB_STALL_WINDOW_S:.1f} s while reversing at ({wx:.2f}, {wy:.2f}) "
                        f"— likely physical obstruction, {len(self._rtb_waypoints)} waypoint(s) remaining",
                        tags=["safety", "rtb", "stall"])
                    self._tel_event.set()
                    self._publish(0, 0)
                    return

                # ── Gouverneur de résistance (voir RTB_RESIST_*) — ralentit
                # PROGRESSIVEMENT dès qu'une résistance apparaît, plutôt que
                # d'attendre le blocage quasi total détecté ci-dessus.
                if (self._rtb_resist_ref_pos is None
                        or now - self._rtb_resist_ref_t >= RTB_RESIST_WINDOW_S):
                    if self._rtb_resist_ref_pos is not None:
                        elapsed = now - self._rtb_resist_ref_t
                        disp = math.hypot(wx - self._rtb_resist_ref_pos[0],
                                           wy - self._rtb_resist_ref_pos[1])
                        expected = abs(self._rtb_resist_ref_cmd)
                        if expected > 1e-3 and elapsed > 1e-3:
                            ratio = (disp / elapsed) / expected
                            if ratio <= RTB_RESIST_RATIO_LO:
                                self._rtb_resist_scale = RTB_RESIST_MIN_SCALE
                            elif ratio >= RTB_RESIST_RATIO_HI:
                                self._rtb_resist_scale = 1.0
                            else:
                                f = ((ratio - RTB_RESIST_RATIO_LO)
                                     / (RTB_RESIST_RATIO_HI - RTB_RESIST_RATIO_LO))
                                self._rtb_resist_scale = (
                                    RTB_RESIST_MIN_SCALE + f * (1.0 - RTB_RESIST_MIN_SCALE))
                        else:
                            self._rtb_resist_scale = 1.0
                    self._rtb_resist_ref_pos = (wx, wy)
                    self._rtb_resist_ref_t   = now
                    self._rtb_resist_ref_cmd = self._rtb_last_lin_cmd

                tx, ty = self._rtb_waypoints[0] if self._rtb_waypoints else (0.0, 0.0)
                dist = math.hypot(wx - tx, wy - ty)
                # Détection dépassement : dist augmente après avoir été proche
                overshot = (self._rtb_prev_dist < RTB_SLOW_DIST and
                            dist > self._rtb_prev_dist + RTB_OVERSHOOT)
                if (dist < RTB_DIST_TOL or overshot) and len(self._rtb_waypoints) > 1:
                    # Point intermédiaire atteint : passe au suivant sans
                    # s'arrêter, la trace continue dans le même tick.
                    self._rtb_waypoints.pop(0)
                    tx, ty = self._rtb_waypoints[0]
                    dist = math.hypot(wx - tx, wy - ty)
                    overshot = False
                    self._log(f"RTB — point atteint, {len(self._rtb_waypoints)} restant(s)")
                self._rtb_prev_dist = dist
                if dist < RTB_DIST_TOL or overshot:
                    self.mode = "MANUEL"
                    self.manual = [0.0, 0.0]
                    self._safe_stop()
                    self._rtb_waypoints = []
                    self._rtb_trail     = []   # nouvelle trace pour la prochaine sortie
                    self._rtb_stall_ref_pos = None
                    self._rtb_resist_ref_pos = None
                    self._rtb_resist_scale   = 1.0
                    reason = "overshoot" if overshot else f"dist={dist:.2f}m"
                    self._log(f"RTB — BASE ATTEINTE ({reason}) — arrêt")
                    self._autolog.add("RTB_COMPLETE", "Return to base complete",
                                      f"Stopped: {reason}",
                                      tags=["rtb", "navigation"])
                    self._tel_event.set()   # force telemetry update immédiate
                else:
                    # robot_heading vient de _rtb_world_pos() ci-dessus — même
                    # source que wx,wy (fusionné ou roues), jamais mélangée.
                    # Marche arrière continue, jamais de demi-tour sur place
                    # (2026-07-23, demande opérateur) : comparaison au cap
                    # ARRIÈRE du robot (cap+180°), pas au cap avant — le
                    # robot recule tout du long avec correction de virage
                    # proportionnelle, sans jamais s'arrêter pour pivoter.
                    # Bénéfice précision, pas seulement comportemental : un
                    # pivot dégrade l'estimation de pose sur ce robot (roues
                    # rigides, glissement — cf. mémoire session).
                    angle_to_target = math.atan2(ty - wy, tx - wx)
                    backward_heading = robot_heading + math.pi
                    angle_error = math.atan2(
                        math.sin(angle_to_target - backward_heading),
                        math.cos(angle_to_target - backward_heading))
                    # Décélération progressive selon la distance (en recule)
                    if dist < RTB_CRAWL_DIST:
                        lin = -RTB_CRAWL_SPEED
                    elif dist < RTB_SLOW_DIST:
                        lin = -RTB_SLOW_SPEED
                    else:
                        lin = -RTB_LIN_SPEED
                    # Ralentissement progressif AVANT l'arrêt dur, deux sources
                    # combinées : résistance arrière (ci-dessus, seul signal
                    # dans le sens de marche) et obstacle AVANT — même
                    # gouverneur PID continu que PATROL (_obstacle_speed_scale,
                    # rampe dès OBSTACLE_SLOWDOWN_MM=1.5 m), réutilisé tel quel
                    # ici : la depth reste pertinente pendant les corrections
                    # de cap qui balaient l'avant du robot.
                    lin *= self._rtb_resist_scale * self._obstacle_speed_scale(now)
                    self._rtb_last_lin_cmd = lin
                    # Correction de virage proportionnelle sur toute la plage
                    # d'erreur (pas de pivot de secours), bornée à RTB_ANG_SPEED.
                    if abs(angle_error) > RTB_ANG_TOL:
                        corr = RTB_ANG_SPEED * (angle_error / math.pi)
                        ang = max(-RTB_ANG_SPEED, min(RTB_ANG_SPEED, corr))

        self._publish(lin, ang)

    def _on_tag_detections(self, msg):
        """Projette le tag le plus proche en coordonnées pixel (intrinsèques
        Kalibr cam0) pour servir de cible à PATROL/LOCK à longue portée."""
        best = None
        for d in msg.detections:
            p = d.pose.pose.pose.position
            if p.z and p.z > 0.05 and (best is None or p.z < best.z):
                best = p
        if best is None:
            return
        # cam0 Kalibr 640x480 : fx, fy, cx, cy
        px = 336.372 * (best.x / best.z) + 310.200
        py = 338.487 * (best.y / best.z) + 243.171
        self._tag_center = (float(px), float(py))
        self._tag_dist   = float(best.z)   # distance métrique du tag (approche LOCK)
        self._tag_center_t = time.time()

    def _dual_beacon_confirmed(self, det):
        """LED ET damier confirmés simultanément — même logique que le garde
        MANUEL de _mp_result_loop, réutilisée ici comme condition de
        finalisation LOCK en AUTO (2026-07-22). Validation SPATIALE ajoutée
        (2026-07-24) : LED et damier doivent en plus être dans la MÊME zone
        de l'image — sans ça, un plafonnier valide bas-image + un damier réel
        détecté ailleurs dans le même frame (mur/poster à motif) satisferait
        les deux conditions indépendamment sans être la même balise physique."""
        lr = det.get("led_reset", {}) or {}
        lm = det.get("map_landmark", {}) or {}
        # Pas de "not _held" ici non plus (2026-07-27, même fix que le garde
        # MANUEL — voir _mp_result_loop) : gelé en LANDMARK, valid reste le
        # seul signal fiable.
        led_confirmed = (bool(lr.get("valid"))
                          and self._led_confirm >= VISION_CONFIRM_FRAMES)
        cb_confirmed  = (self._vision_mode == "LANDMARK"
                          and bool(lm.get("valid")) and not lm.get("_held"))
        if led_confirmed and cb_confirmed:
            led_c, cb_bbox = lr.get("center"), lm.get("bbox")
            if led_c and cb_bbox:
                cb_x0, cb_y0, cb_x1, cb_y1 = cb_bbox
                cb_w = max(cb_x1 - cb_x0, 1.0)
                cb_cx, cb_cy = (cb_x0 + cb_x1) / 2.0, (cb_y0 + cb_y1) / 2.0
                dist = math.hypot(led_c[0] - cb_cx, led_c[1] - cb_cy)
                if dist > BEACON_SPATIAL_MAX_FRAC * cb_w:
                    self._log(f"LED et damier tous deux valides mais DISJOINTS "
                              f"({dist:.0f}px > {BEACON_SPATIAL_MAX_FRAC:.1f}× "
                              f"largeur damier {cb_w:.0f}px) — pas la même balise, "
                              f"confirmation refusée")
                    return led_confirmed, False
        return led_confirmed, cb_confirmed

    def _target_center(self, det):
        c = det.get("center")
        now = time.time()
        if c is not None:
            self._last_center = c
            self._last_center_t = now
            return c
        # repli longue portée : centre du tag AprilTag (frais < 0.7 s)
        tc = getattr(self, "_tag_center", None)
        if tc is not None and (now - getattr(self, "_tag_center_t", 0.0)) < 0.7:
            self._last_center = tc
            self._last_center_t = now
            return tc
        if self._last_center is not None and \
                (now - self._last_center_t) < CENTER_HOLD_S:
            return self._last_center
        return None

    def _goto_beacon(self, beacon_id):
        """Passe en AUTO GOTO_BEACON vers la balise d'indice beacon_id (1-based)."""
        targets = [m for m in self.map_markers if m.get("type") == "beacon"]
        if not targets or beacon_id < 1 or beacon_id > len(targets):
            self._log(f"[BEACON] ID {beacon_id} inconnu — {len(targets)} balise(s) en mémoire")
            return
        t = targets[beacon_id - 1]
        self._target_beacon_id  = beacon_id
        self._target_beacon_pos = (float(t["x"]), float(t["y"]))
        self._uturn_resume      = "GOTO_BEACON"
        self._obstacle_flag     = False
        self.mode               = "AUTO"
        self.auto_state         = "GOTO_BEACON"
        self.manual             = [0.0, 0.0]
        self._safe_stop()
        self._log(
            f"[BEACON] GOTO #{beacon_id} '{t.get('label','')}' → ({t['x']:.2f}, {t['y']:.2f})")
        self._autolog.add("GOTO_BEACON",
                          f"Navigation vers balise #{beacon_id}",
                          f"Target world ({t['x']:.2f}, {t['y']:.2f})",
                          tags=["goto_beacon", "autonomous"])
        self._tel_event.set()

    def _start_patrol(self):
        self.auto_state = "PATROL"
        # Refonte 2026-07-08 : patrouille LIGNE DROITE — le robot avance tout
        # droit en permanence ; le balayage de la pièce émerge des pivots 90°
        # (obstacles) et des rotations 90° (balises). Plus de phase scan 360°.
        self.scan_accum = 0.0
        self.scan_last_yaw = self.raw_yaw
        self._patrol_advancing = True
        self.adv_start = time.time()
        # Purge des historiques anti-coincement (2026-07-13) : l'historique de
        # position accumulé pendant l'attente en MANUEL déclenchait un FAUX
        # échappement 20 s après l'entrée en AUTO (« déplacement < 0.15 m »
        # calculé sur des échantillons datant d'avant le départ).
        self._disp_hist.clear()
        self._avoid_times.clear()
        self._disp_last_t = time.time()
        self._obstacle_flag = False
        self.lock_centered = False
        # VFH+ (2026-07-24) : cap précédent remis à zéro à l'entrée en
        # PATROL — un cap mémorisé d'avant un LOCK/WAIT/U_TURN (donc dans un
        # cap monde potentiellement très différent) biaiserait le tout
        # premier choix VFH+ de la nouvelle patrouille via le terme de
        # continuité (VFH_W_SMOOTH).
        self._vfh_prev_bearing = None
        self._dual_confirm_t0 = None
        self._pid_lock_align.reset()
        self._pid_lock_approach.reset()
        self._pid_patrol.reset()
        self._pid_obstacle.reset()
        # horloges de couverture : repartir proprement
        self._last_beacon_seen_t = time.time()
        self._disp_hist.clear()

    def _start_rtb(self):
        wx, wy, _ = self._rtb_world_pos()
        dist = math.hypot(wx, wy)
        if dist < RTB_DIST_TOL:
            self.stop()
            self._log("RTB — déjà à la base, arrêt")
            return
        # File de points à retracer : la trace enregistrée (la plus récente en
        # tête, donc la plus proche du robot maintenant), puis (0,0) en dernier
        # recours garanti — au cas où l'échantillonnage (RTB_TRAIL_SPACING)
        # laisse un petit résidu entre le point le plus ancien et la vraie
        # origine. Points déjà proches du robot (< tolérance) retirés d'emblée
        # pour ne pas viser un point pratiquement sur place.
        waypoints = list(reversed(self._rtb_trail))
        waypoints = [p for p in waypoints if math.hypot(wx - p[0], wy - p[1]) > RTB_DIST_TOL]
        if not waypoints or math.hypot(waypoints[-1][0], waypoints[-1][1]) > RTB_DIST_TOL:
            waypoints.append((0.0, 0.0))
        self._rtb_waypoints = waypoints
        self.mode = "AUTO"
        self.auto_state = "RTB"
        self._rtb_phase     = "TURN"
        self._rtb_prev_dist = dist
        self._rtb_stall_ref_pos = None  # nouvelle fenêtre de détection de blocage
        self._rtb_resist_ref_pos = None
        self._rtb_resist_scale   = 1.0
        self._rtb_last_lin_cmd   = 0.0
        tx, ty = self._rtb_waypoints[0]
        self._log(f"RTB — retraçage de {len(self._rtb_waypoints)} point(s), "
                  f"dist totale={dist:.2f}m, 1er point à "
                  f"{math.degrees(math.atan2(ty - wy, tx - wx)):.0f}°")
        self._autolog.add("RTB_START", "Return to base initiated",
                          f"Retracing {len(self._rtb_waypoints)} waypoint(s), "
                          f"distance to origin: {dist:.2f} m",
                          tags=["rtb", "navigation"])

    def _on_velocity_config(self, msg):
        """Applique les limites de vitesse envoyées par les sliders du cockpit."""
        try:
            cfg = json.loads(msg.data)
            if "max_lin" in cfg:
                self.max_lin_vel = max(0.1, min(1.0, float(cfg["max_lin"])))
            if "max_ang" in cfg:
                self.max_ang_vel = max(0.1, min(2.0, float(cfg["max_ang"])))
            self._log(f"Velocity limits: lin≤{self.max_lin_vel:.2f} m/s  ang≤{self.max_ang_vel:.2f} rad/s")
        except Exception:
            pass

    def _publish(self, lin, ang):
        lin = float(lin)
        if lin != 0.0:
            ang = float(ang) + YAW_TRIM_PER_MPS * lin   # cf. constantes : trim lacet
        lin = DRIVE_POLARITY * lin   # cf. constantes : remontage 180°
        ang = ANG_POLARITY * float(ang)
        lin = max(-self.max_lin_vel, min(self.max_lin_vel, lin))
        ang = max(-self.max_ang_vel, min(self.max_ang_vel, float(ang)))
        self.last_cmd = (lin, ang)
        if lin != 0.0 or ang != 0.0:
            print(f"[CMD:vel] /cmd_vel  lin={lin:.3f}  ang={ang:.3f}", flush=True)
        try:
            t = self._Twist()
            t.linear.x = lin
            t.angular.z = ang
            self.cmd_pub.publish(t)
        except Exception as e:
            print(f"[CMD:vel] PUBLISH FAILED: {e}", flush=True)

    def _safe_stop(self):
        for _ in range(5):
            self._publish(0.0, 0.0)
            time.sleep(0.02)

    # ══════════════════════════════════════════════════════════════════════ #
    # Image annotée -> /mission/image_annotated (servie en MJPEG)
    # ══════════════════════════════════════════════════════════════════════ #
    def _publish_image_loop(self):
        period = 1.0 / PUBLISH_IMG_HZ
        _last_pub_seq = -1
        _last_view    = None
        while not self._closing:
            t0 = time.time()
            with self._lock:
                bgr = self.latest_bgr
                det = self.det
                seq = self._dec_seq
            try:
                if bgr is not None:
                    # Re-annotate only when there is actually a new frame from the camera.
                    # Avoids running _draw_overlay at 20 Hz when camera only delivers 9 fps.
                    if seq != _last_pub_seq:
                        _last_pub_seq = seq
                        base = det["mask"] if (self.show_mask and
                                               det.get("mask") is not None) else bgr
                        _last_view = self._draw_overlay(base, det)
                        # Resize after overlay so coordinates stay correct
                        if _last_view is not None and _last_view.shape[1] > MJPEG_MAX_W:
                            scale = MJPEG_MAX_W / _last_view.shape[1]
                            _last_view = cv2.resize(
                                _last_view,
                                (MJPEG_MAX_W, int(_last_view.shape[0] * scale)),
                                interpolation=cv2.INTER_LINEAR)
                    if _last_view is not None:
                        self.img_pub.publish(self._to_imgmsg(_last_view))
                else:
                    # No camera frame yet — publish diagnostic placeholder so the
                    # MJPEG topic stays alive and the browser <img> never breaks.
                    self.img_pub.publish(self._to_imgmsg(self._placeholder_frame()))
            except Exception:
                pass
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    def _placeholder_frame(self):
        """Frame de diagnostic publiée tant qu'aucune image caméra n'arrive.
        L'horloge en bas rend le flux visiblement « vivant » (il défile)."""
        img = np.zeros((360, 640, 3), np.uint8)
        img[:] = (12, 11, 9)                       # fond zinc-950 (BGR)
        cv2.putText(img, "EN ATTENTE DU FLUX CAMERA", (70, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (210, 200, 60), 2, cv2.LINE_AA)
        cv2.putText(img, "topic  : " + CAM_TOPIC_DEFAULT, (70, 198),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (140, 140, 140), 1, cv2.LINE_AA)
        age = (time.time() - self.img_t) if self.img_t else -1.0
        info = ("aucune frame recue depuis le demarrage" if age < 0
                else "derniere frame il y a %.1f s" % age)
        cv2.putText(img, info, (70, 228),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (90, 90, 215), 1, cv2.LINE_AA)
        cv2.putText(img, "vision/web OK -- verifier la camera du robot", (70, 258),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (110, 110, 110), 1, cv2.LINE_AA)
        cv2.putText(img, time.strftime("%H:%M:%S"), (70, 308),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (160, 160, 160), 2, cv2.LINE_AA)
        return img

    def _to_imgmsg(self, bgr):
        if bgr.ndim == 2:
            bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)
        msg = self._Image()
        msg.header.stamp = self._rospy.Time.now()
        msg.header.frame_id = "camera"
        h, w = bgr.shape[:2]
        msg.height, msg.width = h, w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = np.ascontiguousarray(bgr).tobytes()
        return msg

    def _draw_overlay(self, view: "np.ndarray", det: dict) -> "np.ndarray":
        """Spotlight isolation — dims non-target areas so targets stand out.

        Marker overlays (grid, reticles, HUD) are rendered by the canvas in app.js.
        This function only handles the pixel-level spotlight effect baked into the
        MJPEG stream:
          1. Convert frame to dimmed grayscale background (35%)
          2. Restore original BGR colour at LED and checkerboard patch locations

        Fail-safe: any exception returns the raw BGR frame unchanged.
        """
        if not isinstance(view, np.ndarray) or view.ndim not in (2, 3):
            self._log(f"[OVERLAY] bad view type={type(view)} ndim={getattr(view,'ndim','?')}")
            return view
        try:
            bgr = view if view.ndim == 3 else cv2.cvtColor(view, cv2.COLOR_GRAY2BGR)
            H, W = bgr.shape[:2]
            self.last_w = W

            lr = det.get("led_reset",    {}) if isinstance(det, dict) else {}
            lm = det.get("map_landmark", {}) if isinstance(det, dict) else {}

            led_pos = lr.get("cluster_pos", [])
            lm_draw = (bool(lm.get("valid"))
                       and float(lm.get("confidence", 0.0)) >= 0.1)
            lm_bbox = lm.get("bbox")

            # No targets — raw frame, zero processing overhead
            if not led_pos and not lm_draw:
                return bgr

            # Full colour frame — overlays drawn directly on colour image
            canvas = bgr.copy()
            sorted_leds = sorted(led_pos, key=lambda p: p[0])   # L→R for stable numbering

            # ── 3. Marker overlays on the MJPEG stream (secondary visual) ─
            # These run even if the browser canvas fails — operators always
            # see target feedback on the raw video feed.

            # CB overlay: multi-color corners + bounding box + label
            if lm_draw and lm_bbox:
                conf    = float(lm.get("confidence", 0.0))
                is_lock = conf >= 0.6
                cb_col  = (0, 220, 150) if is_lock else (0, 140, 255)   # green / orange

                # Rainbow corner grid (cv2.drawChessboardCorners — plusieurs couleurs)
                cb_corners = lm.get("corners", [])
                cb_sz      = lm.get("checker_size")
                if cb_corners and cb_sz:
                    pts_f32 = np.array(cb_corners, dtype=np.float32).reshape(-1, 1, 2)
                    cv2.drawChessboardCorners(canvas, tuple(cb_sz), pts_f32, True)

                # Full board outline (perspective-extrapolated physical boundary)
                brd = lm.get("board_corners")
                if brd and len(brd) == 4:
                    brd_pts = np.array(brd, dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(canvas, [brd_pts], True, cb_col, 2, cv2.LINE_AA)
                else:
                    # Fallback: inner-corner bbox
                    p1 = (max(0, int(lm_bbox[0])), max(0, int(lm_bbox[1])))
                    p2 = (min(W - 1, int(lm_bbox[2])), min(H - 1, int(lm_bbox[3])))
                    cv2.rectangle(canvas, p1, p2, cb_col, 1)

                # Label near top of board outline
                tag_x = int(brd[0][0]) if brd else max(0, int(lm_bbox[0]))
                tag_y = int(brd[0][1]) if brd else max(8, int(lm_bbox[1]) - 4)
                tag = (f"{cb_sz[0]}x{cb_sz[1]} {round(conf * 100)}%" if cb_sz else
                       f"CB {round(conf * 100)}%")
                cv2.putText(canvas, tag, (tag_x, max(8, tag_y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, cb_col, 1, cv2.LINE_AA)

                # Centre crosshair on CB
                if lm.get("center"):
                    cx, cy = int(lm["center"][0]), int(lm["center"][1])
                    cv2.line(canvas, (cx - 10, cy), (cx - 3, cy), cb_col, 1, cv2.LINE_AA)
                    cv2.line(canvas, (cx + 3,  cy), (cx + 10, cy), cb_col, 1, cv2.LINE_AA)
                    cv2.line(canvas, (cx, cy - 10), (cx, cy - 3), cb_col, 1, cv2.LINE_AA)
                    cv2.line(canvas, (cx, cy + 3),  (cx, cy + 10), cb_col, 1, cv2.LINE_AA)

            # LED numbered reticles (sorted L→R, 1-indexed)
            LED_COL = (0, 80, 255)   # BGR orange-red
            for idx, (lx, ly) in enumerate(sorted_leds):
                px, py = int(lx), int(ly)
                arm, gap = 9, 3
                cv2.line(canvas, (px - arm, py), (px - gap, py), LED_COL, 1, cv2.LINE_AA)
                cv2.line(canvas, (px + gap, py), (px + arm, py), LED_COL, 1, cv2.LINE_AA)
                cv2.line(canvas, (px, py - arm), (px, py - gap), LED_COL, 1, cv2.LINE_AA)
                cv2.line(canvas, (px, py + gap), (px, py + arm), LED_COL, 1, cv2.LINE_AA)
                cv2.circle(canvas, (px, py), 2, LED_COL, -1, cv2.LINE_AA)
                label_y = min(H - 2, py + 14)
                cv2.putText(canvas, str(idx + 1), (px - 3, label_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, LED_COL, 1, cv2.LINE_AA)

            return canvas

        except Exception as exc:
            try:
                self._log(f"[OVERLAY-ERR] {exc}")
            except Exception:
                pass
            return view if view.ndim == 3 else cv2.cvtColor(view, cv2.COLOR_GRAY2BGR)

    # ══════════════════════════════════════════════════════════════════════ #
    # Télémétrie JSON -> /mission/telemetry
    # ══════════════════════════════════════════════════════════════════════ #
    def _hz(self, times):
        now = time.time()
        return sum(1 for t in list(times) if now - t <= 1.0)


    # ══════════════════════════════════════════════════════════════════════ #
    #  CALIBRATION TERRAIN (2026-07-29)                                      #
    #                                                                        #
    #  Confronte ce que le robot CROIT parcourir à ce qu'il parcourt         #
    #  RÉELLEMENT, mesuré au ruban et au rapporteur par l'opérateur.         #
    #                                                                        #
    #  POURQUOI : toutes les corrections précédentes comparaient les         #
    #  capteurs ENTRE EUX (VINS contre roues, MINS contre roues). Or les     #
    #  roues patinent, donc « VINS/roues = 0.69 » ne dit pas lequel se       #
    #  trompe. Sans vérité terrain on constate un désaccord, on ne le        #
    #  corrige pas. Version web de tools/calib_terrain.py, pour être pilotée #
    #  depuis un téléphone à côté du robot.                                  #
    # ══════════════════════════════════════════════════════════════════════ #

    def _calib_save(self):
        """Ecrit la campagne apres CHAQUE modification. Fichier minuscule,
        ecriture atomique (fichier temporaire + rename) pour qu'une coupure
        en plein milieu ne laisse jamais un JSON tronque."""
        c = self._calib
        try:
            os.makedirs(os.path.dirname(CALIB_STATE_FILE), exist_ok=True)
            data = {"sets": c["sets"], "mode": c["mode"], "sens": c["sens"]}
            tmp = CALIB_STATE_FILE + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f)
            os.replace(tmp, CALIB_STATE_FILE)
        except Exception as e:
            self._log(f"Calibration : sauvegarde impossible ({e})")

    def _calib_load(self):
        try:
            with open(CALIB_STATE_FILE, encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            return
        c = self._calib
        c["sets"] = data.get("sets", {}) or {}
        c["mode"] = data.get("mode", c["mode"])
        c["sens"] = data.get("sens", c["sens"])
        c["passes"] = c["sets"].setdefault(self._calib_key(c), [])
        c["result"] = self._calib_compute()
        n = sum(len(v) for v in c["sets"].values())
        if n:
            self._log(f"Calibration : campagne rechargée ({n} passe(s) sur disque)")

    @staticmethod
    def _quat_to_rpy(q):
        """Quaternion -> (roll, pitch, yaw) en radians, convention ZYX.

        Ecrit a la main plutot que via tf.transformations : ce backend tourne
        deja sans session graphique et on evite une dependance de plus pour
        quatre lignes de trigonometrie. Le clamp sur l'argument de asin est
        indispensable — une norme de quaternion legerement > 1 (arrondi en
        virgule flottante) ferait lever un ValueError sur math.asin et tuerait
        le callback, donc la telemetrie entiere.
        """
        sr = 2.0 * (q.w * q.x + q.y * q.z)
        cr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sr, cr)
        sp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sp)
        sy = 2.0 * (q.w * q.z + q.x * q.y)
        cy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(sy, cy)
        return roll, pitch, yaw

    def _carolus_convert(self, pos, quat):
        """Repere CAMERA -> repere LEO (Lot B). Renvoie (x,y,z,qx,qy,qz,qw).

        Les lettres sont figees (doc §10.2.5, verifiees) ; les SIGNES du
        quaternion sont lus dans `~carolus_quat_signs` pour permettre de
        balayer les 16 combinaisons EN DIRECT lors de la prochaine session,
        sans toucher au code ni redemarrer autre chose que ce backend.
        """
        px = {"x": pos.x, "y": pos.y, "z": pos.z}
        out_pos = [px[axe] * sgn for axe, sgn in CAROLUS_POS_PERM]
        q = {"x": quat.x, "y": quat.y, "z": quat.z, "w": quat.w}
        sg = self._caro_quat_signs
        out_q = [q[l] * (-1.0 if sg[i] == "-" else 1.0)
                 for i, l in enumerate(CAROLUS_QUAT_LETTERS)]
        return out_pos + out_q

    def _on_carolus_fix(self, msg):
        """Fix global de la balise : 6 DDL + fraicheur. LECTURE SEULE.

        On retient DEUX temps : l'horodatage du message (fraicheur reelle de
        la mesure) et l'instant de reception. Le second sert de repli si le
        publieur envoie un stamp nul, cas deja vu sur ce robot.
        """
        try:
            p, o = msg.pose.position, msg.pose.orientation
            t_msg = msg.header.stamp.to_sec()
            src_frame = msg.header.frame_id or "camera_frame"

            if self._caro_apply_perm:
                x, y_, z, qx, qy, qz, qw = self._carolus_convert(p, o)
                frame_out = "base_link"
            else:
                x, y_, z = p.x, p.y, p.z
                qx, qy, qz, qw = o.x, o.y, o.z, o.w
                frame_out = src_frame

            class _Q:                       # petit porteur pour _quat_to_rpy
                pass
            qq = _Q(); qq.x, qq.y, qq.z, qq.w = qx, qy, qz, qw
            r, pi_, yw = self._quat_to_rpy(qq)

            with self._lock:
                self._caro_fix = {
                    "x": round(x, 4), "y": round(y_, 4), "z": round(z, 4),
                    "roll":  round(math.degrees(r), 2),
                    "pitch": round(math.degrees(pi_), 2),
                    "yaw":   round(math.degrees(yw), 2),
                    "t": t_msg if t_msg > 0 else time.time(),
                    # `frame` = repere des valeurs AFFICHEES ; `src_frame` =
                    # celui du message d'origine. Les distinguer est le coeur
                    # du Lot B : le 29/07 on affichait du camera_frame sous une
                    # etiquette laissant croire a une pose robot.
                    "frame": frame_out,
                    "src_frame": src_frame,
                    "converted": bool(self._caro_apply_perm),
                    "signs": self._caro_quat_signs,
                    # Brut conserve : indispensable pour balayer les 16
                    # combinaisons de signes en comparant a la reference.
                    "raw": {"x": round(p.x, 4), "y": round(p.y, 4),
                            "z": round(p.z, 4)},
                }
        except Exception as e:
            # Un fix malforme ne doit JAMAIS faire tomber la telemetrie.
            # `import rospy` LOCAL : sans lui cette branche levait un
            # NameError — le garde-fou aurait donc plante exactement dans le
            # cas qu'il devait couvrir. Defaut latent trouve le 30/07.
            try:
                import rospy
                rospy.logwarn_throttle(30.0,
                                       "[carolus_fix] message ignore : %s", e)
            except Exception:
                pass

    @staticmethod
    def _calib_key(c):
        """Clé de série : les deux sens de rotation sont comptés séparément.
        Une asymétrie gauche/droite signale un défaut mécanique, pas une
        erreur d'échelle — les confondre masquerait exactement ce qu'on veut
        voir."""
        return c["mode"] + ("_" + c["sens"] if c["mode"] == "rotation" else "")

    @staticmethod
    def _calib_yaw(q):
        """Lacet depuis un quaternion, sans dépendance à tf."""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _calib_on_odom(self, nom):
        def f(m):
            p = m.pose.pose.position
            with self._lock:
                self._calib_last[nom] = (p.x, p.y, p.z,
                                         self._calib_yaw(m.pose.pose.orientation))
        return f

    def _calib_on_imu(self, m):
        """Intègre wz pour obtenir un cap gyro pur.

        Les pas aberrants sont ignorés : le flux IMU présente 13 à 21 trous
        par 40 s quand le Pi est bridé thermiquement (mesuré le 28/07), et
        intégrer un dt de 40 ms comme s'il était normal fausserait le cap.
        """
        t = m.header.stamp.to_sec()
        with self._lock:
            if self._calib_imu_t is not None:
                dt = t - self._calib_imu_t
                if 0.0 < dt < 0.05:
                    self._calib_gyro_yaw += m.angular_velocity.z * dt
            self._calib_imu_t = t

    def _calib_snapshot(self):
        with self._lock:
            snap = dict(self._calib_last)
            snap["_gyro"] = self._calib_gyro_yaw
        return snap

    def _calib_cmd(self, cmd):
        op = cmd.get("op", "")
        c = self._calib
        if op == "set_mode":
            mode = cmd.get("mode", "rotation")
            if mode not in ("droite", "rotation"):
                return
            # On ne mélange pas des mètres et des degrés dans une même
            # moyenne — mais on n'EFFACE PAS pour autant. Chaque série est
            # rangée sous sa propre clé et retrouvée en revenant dessus.
            # (Corrigé le 29/07 : la version initiale vidait les passes au
            # changement de mode, sans avertissement ; un aller-retour entre
            # « 90° G » et « 90° D » — exactement le protocole demandé, qui
            # impose de comparer les deux sens — détruisait la campagne.)
            c["sets"][self._calib_key(c)] = c["passes"]
            c["mode"] = mode
            c["sens"] = cmd.get("sens", c["sens"])
            # setdefault et non get : la liste active doit ETRE l'objet rangé
            # dans sets. Avec get(k, []) une série neuve recevait une liste
            # detachee — sa premiere passe n'etait ni publiee ni sauvegardee.
            c["passes"] = c["sets"].setdefault(self._calib_key(c), [])
            c["result"] = self._calib_compute()
            c["preview"] = None
            c["state"] = "idle"
            self._calib_save()
            self._log(f"Calibration : mode {mode}"
                      + (f" {c['sens']}" if mode == "rotation" else ""))
        elif op == "mark_start":
            c["refus"] = None
            c["start"] = self._calib_snapshot()
            c["state"] = "armed"
            self._log("Calibration : départ marqué — roulez, puis ARRÊTEZ")
        elif op == "mark_end":
            if c["state"] != "armed":
                self._log("Calibration : marquez d'abord le départ")
                return
            c["end"] = self._calib_snapshot()
            c["state"] = "pending"
            c["preview"] = self._calib_measures(c["start"], c["end"], c["mode"])
            self._log("Calibration : arrivée marquée — saisissez la mesure réelle")
        elif op == "submit":
            if c["state"] != "pending":
                return
            try:
                v = float(cmd.get("value"))
            except (TypeError, ValueError):
                self._log("Calibration : valeur illisible")
                return
            if v <= 0:
                self._log("Calibration : valeur positive attendue")
                return
            # UNITE : des METRES en ligne droite. La version initiale
            # demandait des centimetres alors que le bouton s'appelle « 1 m » :
            # l'operateur a saisi 1 pour un metre, lu comme 1 cm, d'ou un
            # facteur 0.0085 (29/07). Incoherence de ma conception, pas une
            # faute d'usage.
            reel = v if c["mode"] == "droite" else math.radians(v)
            mes = c["preview"] or {}

            # GARDE DE VRAISEMBLANCE : trois capteurs independants ne se
            # trompent pas d'un facteur 100 ensemble. Si la valeur saisie
            # s'ecarte a ce point de ce qu'ILS ONT TOUS vu, c'est une erreur
            # d'unite ou de frappe, jamais une decouverte de calibration —
            # et l'enregistrer polluerait la moyenne en silence.
            vus = sorted(x for x in mes.values() if x > 1e-9)
            # Cas distinct : le robot n'a PAS bouge entre les deux reperes.
            # Rien n'est calibrable, et parler d'unite induirait en erreur.
            seuil = 0.02 if c["mode"] == "droite" else math.radians(2.0)
            if not vus or vus[len(vus) // 2] < seuil:
                c["refus"] = ("Le robot n'a pas bougé entre les deux repères "
                              "— refaites la manœuvre.")
                self._log("Calibration REFUSEE : " + c["refus"])
                return
            if vus:
                med = vus[len(vus) // 2]
                if not (0.2 * med <= reel <= 5.0 * med):
                    att = ("%.2f m" % med) if c["mode"] == "droite" \
                          else ("%.0f deg" % math.degrees(med))
                    msg = (f"Saisie refusée : incohérente avec les capteurs "
                           f"(ils ont vu ~{att}). Vérifiez l'unité.")
                    # Remonté en TELEMETRIE et pas seulement dans le journal :
                    # trajectory.html n'affiche pas /mission/log, un refus
                    # silencieux serait pire qu'une valeur fausse — l'operateur
                    # croirait sa passe enregistree.
                    c["refus"] = msg
                    self._log("Calibration REFUSEE : " + msg)
                    return
            c["refus"] = None
            c["passes"].append({"reel": reel, "mesures": mes})
            c["state"] = "idle"
            c["preview"] = None
            c["result"] = self._calib_compute()
            self._calib_save()
            self._log(f"Calibration : passe {len(c['passes'])} enregistrée")
        elif op == "undo_pass":
            if c["passes"]:
                c["passes"].pop()
                c["result"] = self._calib_compute()
                self._calib_save()
                self._log("Calibration : dernière passe annulée")
        elif op == "reset":
            c["passes"] = []
            c["sets"][self._calib_key(c)] = []
            c["result"] = None
            self._calib_save()
            c["state"] = "idle"
            c["preview"] = None
            self._log("Calibration : remise à zéro")

    def _calib_measures(self, a, b, mode):
        """Déplacement (m) ou rotation (rad) vu par chaque source."""
        out = {}
        for nom in ("roues", "MINS", "VINS"):
            if nom not in a or nom not in b:
                continue
            if mode == "droite":
                out[nom] = math.dist(b[nom][:3], a[nom][:3])
            else:
                d = b[nom][3] - a[nom][3]
                out[nom] = abs(math.atan2(math.sin(d), math.cos(d)))
        if mode == "rotation":
            out["gyro brut"] = abs(b.get("_gyro", 0.0) - a.get("_gyro", 0.0))
        return out

    def _calib_compute(self):
        """Facteur = réel / mesuré, moyenné sur les passes.

        L'écart-type est publié avec : une calibration issue d'une seule
        passe mélange l'erreur d'échelle recherchée (systématique) avec
        l'erreur de pose de départ et le patinage du moment (aléatoires).
        Dispersé = refaire, surtout pas appliquer.
        """
        c = self._calib
        if not c["passes"]:
            return None
        noms = []
        for p in c["passes"]:
            for k in p["mesures"]:
                if k not in noms:
                    noms.append(k)
        res = {}
        for nom in noms:
            fs = [p["reel"] / p["mesures"][nom] for p in c["passes"]
                  if p["mesures"].get(nom, 0.0) > 1e-9]
            if not fs:
                continue
            moy = sum(fs) / len(fs)
            ec = (sum((f - moy) ** 2 for f in fs) / len(fs)) ** 0.5
            res[nom] = {
                "facteur": round(moy, 4),
                "ecart_type": round(ec, 4),
                "n": len(fs),
                "erreur_pct": round((1.0 / moy - 1.0) * 100.0, 1),
                "disperse": bool(ec > 0.10 * moy),
                "deja_juste": bool(abs(1.0 / moy - 1.0) < 0.02),
            }
        return res

    def _telemetry_loop(self):
        period = 1.0 / TELEMETRY_HZ
        while not self._closing:
            t0 = time.time()
            try:
                data = self._build_telemetry()
                self.tel_pub.publish(self._String(data=json.dumps(data)))
            except Exception:
                pass
            dt = time.time() - t0
            remaining = period - dt
            if remaining > 0:
                self._tel_event.wait(timeout=remaining)
                self._tel_event.clear()

    def _get_sysinfo(self):
        if not _PSUTIL:
            return None
        try:
            cpu_pct = _psutil.cpu_percent(interval=None)
            per_core = _psutil.cpu_percent(percpu=True, interval=None)
            cpu_temp = None
            temps = _psutil.sensors_temperatures() if hasattr(_psutil, 'sensors_temperatures') else {}
            for key in ('coretemp', 'cpu_thermal', 'k10temp', 'acpitz'):
                if key in temps and temps[key]:
                    cpu_temp = round(temps[key][0].current, 1)
                    break
            return {
                "cpu_pct": round(cpu_pct, 1),
                "cpu_per_core": [round(p, 1) for p in (per_core or [])],
                "cpu_temp": cpu_temp,
            }
        except Exception:
            return None

    def _build_telemetry(self):
        now = time.time()
        with self._lock:
            det = self.det
            pose = self.pose
            traj = [(round(x, 3), round(y, 3)) for (x, y) in self.traj[-200:]]
            beacons = list(self.beacons)
            vel = self.vel_meas
            cmd = self.last_cmd
            batt = self.batt_v
            wheels = dict(self.wheels)
            img_t = self.img_t
            cb_size = self._cb_size
        cluster = det.get("cluster") or []
        checker = det.get("checker")
        valid   = det.get("valid", False)
        pct = None
        if batt is not None:
            pct = round(max(0.0, min(100.0, (batt - 10.0) / 2.6 * 100)), 0)

        def live(key, t=1.5):
            return (now - self._t.get(key, 0.0)) < t

        center  = det.get("center")
        det_lr  = det.get("led_reset", {})
        det_lm  = det.get("map_landmark", {})
        wx, wy  = self._get_world_pos()
        with self._lock:
            _bgr_shape = self.latest_bgr.shape if self.latest_bgr is not None else None
        c = self._calib
        # ── Fix Carolus : on JOINT la fraicheur a la donnee ────────────────
        # L'age est calcule ici, a l'instant d'emission, et pas cote navigateur :
        # le PC et le robot n'ont pas forcement la meme horloge, et laisser le
        # client soustraire deux temps d'origines differentes produirait des
        # ages negatifs ou fantaisistes. `stale` est la seule chose que
        # l'interface doit croire.
        with self._lock:
            cf = dict(self._caro_fix) if self._caro_fix else None
        if cf is not None:
            age = max(0.0, now - cf.pop("t"))
            cf["age_s"] = round(age, 1)
            cf["stale"] = bool(age > CAROLUS_FIX_STALE_S)
        return {
            "t": round(now, 2),
            # 6 DDL de la balise, ou None si aucun fix n'a JAMAIS ete recu.
            # A ne pas confondre avec "pose", qui est la pose interne du robot.
            "carolus_fix": cf,
            "connected": self.connected,
            # Calibration terrain : on publie aussi la mesure BRUTE de la
            # passe en attente (preview), pour que l'opérateur voie ce que le
            # robot a cru parcourir AVANT de saisir la réalité — sinon la
            # saisie influence la lecture.
            "calib": {"mode": c["mode"], "sens": c["sens"], "state": c["state"],
                      "n_passes": len(c["passes"]),
                      "preview": ({k: round(v, 4) for k, v in c["preview"].items()}
                                  if c["preview"] else None),
                      "result": c["result"],
                      # Récap de TOUTES les séries : le protocole impose de
                      # comparer gauche et droite, il faut donc voir d'un coup
                      # d'oeil ce qui est déjà fait sans changer de mode.
                      "series": {k: len(v) for k, v in c["sets"].items() if v},
                      "refus": c["refus"]},
            "cam_alive": bool(img_t) and (now - img_t) < CAM_ALIVE_TIMEOUT,
            "mode": self.mode,
            "auto_state": self.auto_state,
            "pose_source": self.pose_source,
            "pose_source_pending": self.pose_source_pending,
            "pose_source_available": self._pose_source_srv is not None,
            "traj_rec": {"mins": len(self._traj["mins"]),
                         "vins": len(self._traj["vins"]),
                         "carolus": len(self._traj["carolus"]),
                         "last_export": self._last_export,
                         "matlab_available": self._matlab_bin is not None,
                         # Verdict du dernier lancement GUI : le cockpit doit
                         # pouvoir DIRE a l'operateur que MATLAB a echoue, au
                         # lieu de le laisser attendre une fenetre absente.
                         "matlab_launch": self._matlab_launch,
                         "current_test": self._current_test,
                         "robust_recording": (
                             {"test": self._robust_rec["test"],
                              "started_ts": self._robust_rec["started_ts"]}
                             if self._robust_rec else None)},
            "carolus_params": self._carolus_params,
            # Lot D : etat de perte de balise (signalement + arret optionnel).
            "beacon_watch": self._beacon_watch(now),
            # Lot C : cinematique firmware, LECTURE SEULE (voir _load_drive_params).
            # Re-tente tant que vide : le backend peut demarrer AVANT le robot,
            # auquel cas le premier essai renvoie None sans que rien ne soit
            # casse — il suffit d'attendre que le master publie les params.
            "drive_params": self._drive_params_or_retry(),
            "fsm_state": self._fsm_state(now),
            "target_beacon_id": self._target_beacon_id,
            "vision_mode": self._vision_mode,
            "lm_miss":     self._lm_miss,
            "lm_hits":     self._lm_hits,
            "led_stable":  self._led_stable,
            "heartbeat": {"armed": self._hb_armed, "lost": self._hb_lost,
                          "age_s": round(now - self._last_heartbeat_t, 1)
                                   if self._hb_armed else None},
            "mission_s": int(now - self._mission_start) if self._mission_start else 0,
            "beacon_count": self.beacon_count,
            "world_heading": round(self.world_heading, 4),
            "pose": {"x": round(pose[0], 3), "y": round(pose[1], 3),
                     "yaw": round(pose[2], 4),
                     "yaw_deg": round(math.degrees(pose[2]), 1),
                     "raw_yaw": round(self.raw_yaw, 4) if self.raw_yaw is not None else None,
                     "wx": round(wx, 3), "wy": round(wy, 3)},
            "vel": {"lin": round(vel[0], 3), "ang": round(vel[1], 3)},
            "cmd": {"lin": round(cmd[0], 3), "ang": round(cmd[1], 3)},
            "battery": {"v": round(batt, 2) if batt is not None else None,
                        "pct": pct},
            "wheels": {"vel": [round(float(x), 2) for x in wheels.get("vel", [0]*4)],
                       "pwm": [round(float(x), 2) for x in wheels.get("pwm", [0]*4)]},
            "hz": {"cam": self._hz(self.cam_times), "odom": self._hz(self.odom_times)},
            "detection": {
                "leds_all": len(det.get("leds", [])),
                "near": len(det.get("near", [])),
                "cluster": len(cluster),
                "checker": checker is not None,
                "checker_size": list(cb_size) if cb_size else None,
                "dist": round(det["dist"], 2) if det.get("dist") is not None else None,
                "center": [round(center[0], 1), round(center[1], 1)] if center else None,
                "valid": valid,
                # ── Pipeline 1 : LED Reset ──────────────────────────────
                "led_reset": {
                    "valid":   det_lr.get("valid", False),
                    # cluster_pos, PAS "cluster" : le dict brut _build_lr ne
                    # publie que cluster_pos — l'ancien nom donnait toujours 0
                    # (2026-07-27, croix 4 LEDs ne s'allumait jamais).
                    "count":   len(det_lr.get("cluster_pos", [])),
                    "center":  ([round(float(det_lr["center"][0]), 1),
                                 round(float(det_lr["center"][1]), 1)]
                                if det_lr.get("center") else None),
                    "bbox":    ([round(float(v), 1) for v in det_lr["bbox"]]
                                if det_lr.get("bbox") else None),
                    "leds_pos":    det_lr.get("leds_pos", []),
                    "cluster_pos": det_lr.get("cluster_pos", []),
                },
                # ── Pipeline 2 : Map Landmark (damier) ──────────────────
                "map_landmark": {
                    "valid":         det_lm.get("valid", False),
                    "checker_size":  det_lm.get("checker_size"),
                    "center":  ([round(float(det_lm["center"][0]), 1),
                                 round(float(det_lm["center"][1]), 1)]
                                if det_lm.get("center") else None),
                    "bbox":    ([round(float(v), 1) for v in det_lm["bbox"]]
                                if det_lm.get("bbox") else None),
                    "dist_est":      det_lm.get("dist_est"),
                    "angle_deg":     det_lm.get("angle_deg"),
                    "confidence":    det_lm.get("confidence", 0.0),
                    "active_search": det_lm.get("active_search", False),
                    "outer_corners":  det_lm.get("outer_corners"),
                    "board_corners":  det_lm.get("board_corners"),
                    "corners":        det_lm.get("corners", []),
                    "pose3d":        det_lm.get("pose3d"),
                },
                # ── Champs legacy (compat frontend existant) ─────────────
                "leds_pos":    det_lr.get("leds_pos", []),
                "cluster_pos": det_lr.get("cluster_pos", []),
                "frame_w": _bgr_shape[1] if _bgr_shape is not None else 640,
                "frame_h": _bgr_shape[0] if _bgr_shape is not None else 480,
            },
            "beacons": [[round(b[0], 3), round(b[1], 3), b[2]] for b in beacons],
            "traj": traj,
            "health": {"cam": live("cam"), "odom": live("odom"),
                       "batt": live("batt", 3.0), "wheels": live("wheels", 2.0)},
            "params": {"hue_low": self._hue_low, "hue_high": self._hue_high,
                       "v_min": self._v_min, "minled": self._minled,
                       "mask": self.show_mask},
            "sysinfo": self._get_sysinfo(),
            "map_markers": list(self.map_markers),
            "wait_remaining": (max(0, int(WAIT_DURATION - (now - self.wait_start_t)))
                               if self.auto_state == "WAIT" else None),
            "vel_limits": {"max_lin": round(self.max_lin_vel, 2),
                           "max_ang": round(self.max_ang_vel, 2)},
            # État PID live (2026-07-22) — valeurs déjà calculées par _drive()
            # ce tick, jamais recalculées ici (un second appel à .update()
            # depuis la télémétrie fausserait l'intégrale/dérivée du PID).
            "pid": {
                "patrol_ang":       round(self._pid_patrol.last_output, 3),
                "lock_align_ang":   round(self._pid_lock_align.last_output, 3),
                "lock_approach_ang": round(self._pid_lock_approach.last_output, 3),
                "obstacle_scale":   round(self._last_obstacle_scale, 2),
                "corridor_mm":      getattr(self, "_corridor_p5_mm", None),
            },
        }

    # ══════════════════════════════════════════════════════════════════════ #
    # Commandes du navigateur -> /mission/command (JSON)
    # ══════════════════════════════════════════════════════════════════════ #
    def _on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except Exception:
            return
        action = cmd.get("action")
        if action != "heartbeat":
            print(f"[CMD:rx] action={action}  connected={self.connected}  mode={self.mode}",
                  flush=True)
        try:
            if action == "set_mode":
                self.set_mode(cmd.get("mode", "MANUEL"))
            elif action == "stop":
                self.stop()
            elif action == "reset":
                self.reset_coords()
            elif action == "clear_map":
                self.clear_map()
            elif action == "set_params":
                self.set_params(cmd)
            elif action == "manual":
                lin = cmd.get("lin", 0.0)
                ang = cmd.get("ang", 0.0)
                self.set_manual(lin, ang)
                if lin != 0.0 or ang != 0.0:
                    print(f"[CMD:manual] lin={lin}  ang={ang}  manual_t_set={time.time():.3f}",
                          flush=True)
            elif action == "set_view":
                self.show_mask = bool(cmd.get("mask", False))
            elif action == "park":
                self._start_rtb()
            elif action == "goto_beacon":
                bid = cmd.get("beacon_id")
                if bid is not None:
                    self._goto_beacon(int(bid))
            elif action == "set_pose_source":
                self.set_pose_source(cmd.get("source", ""))
            elif action == "export_matlab":
                self._export_matlab(open_matlab=bool(cmd.get("open_matlab", False)))
            elif action == "plot_matlab":
                self._plot_matlab()
            elif action == "traj_reset":
                self._traj_reset()
            elif action == "set_test":
                self._set_current_test(cmd.get("test", ""),
                                       finalize=bool(cmd.get("finalize", False)))
            elif action == "set_carolus_param":
                self._set_carolus_param(cmd.get("key", ""), cmd.get("value"))
            elif action == "set_beacon_estop":
                self._beacon_estop = bool(cmd.get("armed", False))
                if "timeout" in cmd:
                    try:
                        self._beacon_lost_timeout = max(
                            0.2, float(cmd.get("timeout")))
                    except (TypeError, ValueError):
                        pass
                self._log("Arrêt sur perte de balise : %s (seuil %.1f s)"
                          % ("ARMÉ" if self._beacon_estop else "désarmé",
                             self._beacon_lost_timeout))
            elif action == "set_carolus_signs":
                # Balayage des 16 combinaisons de signes du quaternion
                # (doc §13.2.2) SANS edition de code ni redemarrage : on
                # compare le trace obtenu a la reference et on garde celle
                # qui superpose. `apply` permet aussi de revenir au brut.
                sg = str(cmd.get("signs", "")).strip()
                if len(sg) == 4 and all(c in "+-" for c in sg):
                    self._caro_quat_signs = sg
                    self._log(f"Carolus : combinaison de signes -> {sg}")
                elif sg:
                    self._log(f"Carolus : signes invalides ({sg!r}) — ignoré")
                if "apply" in cmd:
                    self._caro_apply_perm = bool(cmd.get("apply"))
                    self._log("Carolus : permutation "
                              + ("ACTIVE" if self._caro_apply_perm
                                 else "DESACTIVEE (valeurs brutes caméra)"))
            elif action == "calib":
                self._calib_cmd(cmd)
            elif action == "recover_camera":
                self._recover_camera(hard=bool(cmd.get("hard", False)))
            elif action == "heartbeat":
                self._last_heartbeat_t = time.time()
                self._hb_armed = True
                self._hb_lost  = False
        except Exception as e:
            self._log(f"invalid command ({action}): {e}")

    _CAM_RECOVER_COOLDOWN_S = 15.0

    def _recover_camera(self, hard=False):
        """Relance manuelle de la caméra (bouton cockpit) — même échelle que
        le watchdog : cycle STOP/START rosmon (+ reset matériel D455 si
        hard=True), purge des republish PC, réapplication laser 0 / png 1.
        Tourne dans un thread pour ne jamais geler la boucle mission."""
        import rospy, threading, subprocess
        now = time.time()
        if now - getattr(self, "_cam_recover_t", 0.0) < self._CAM_RECOVER_COOLDOWN_S:
            self._log("Relance caméra ignorée (cooldown 15 s — déjà en cours)")
            return
        self._cam_recover_t = now
        self._log(f"Relance caméra demandée (opérateur){' — reset matériel' if hard else ''}")

        def _worker():
            try:
                from rosmon_msgs.srv import StartStop
                srv = rospy.ServiceProxy("/rosmon/start_stop", StartStop)
                if hard:
                    try:
                        from std_srvs.srv import Trigger
                        rospy.ServiceProxy("/camera/realsense2_camera/reset",
                                           Trigger).call()
                        time.sleep(3.0)
                    except Exception as e:
                        self._log(f"[CAM] reset matériel indisponible : {e}")
                for node in ("realsense2_camera", "realsense2_camera_manager"):
                    srv.call(node=node, ns="/camera", action=StartStop._request_class.STOP)
                time.sleep(2.0)
                for node in ("realsense2_camera_manager", "realsense2_camera"):
                    srv.call(node=node, ns="/camera", action=StartStop._request_class.START)
                time.sleep(4.0)
                # republish PC : purge des abonnements morts (respawn supervision)
                subprocess.run(["pkill", "-f", "image_transport"],
                               capture_output=True, timeout=5)
                time.sleep(3.0)
                try:
                    import dynamic_reconfigure.client as drc
                    drc.Client("/camera/stereo_module", timeout=5).update_configuration(
                        {"laser_power": 0})
                    drc.Client("/camera/depth/image_rect_raw/compressedDepth",
                              timeout=5).update_configuration({"png_level": 1})
                except Exception as e:
                    self._log(f"[CAM] réapplication params non critique échouée : {e}")
                self._log("Relance caméra terminée — vérifier le flux (peut prendre 10-15 s de plus)")
            except Exception as e:
                self._log(f"[CAM] relance ÉCHOUÉE : {e}")

        threading.Thread(target=_worker, daemon=True).start()

    def set_mode(self, mode):
        mode = mode.upper()
        if mode not in ("MANUEL", "AUTO"):
            mode = "MANUEL"
        self.mode = mode
        if mode == "AUTO":
            self._start_patrol()
        self.manual = [0.0, 0.0]
        self._safe_stop()
        self._log(f"Mode: {mode}")
        self._autolog.add("MODE_CHANGE", f"Mode set to {mode}",
                          f"Beacon count at transition: {self.beacon_count}",
                          tags=["mode", mode.lower()])

    def stop(self):
        self.mode = "MANUEL"
        self.manual = [0.0, 0.0]
        self._safe_stop()
        self._log("EMERGENCY STOP")
        self._autolog.add("EMERGENCY_STOP", "Emergency stop triggered",
                          f"Beacons reached before stop: {self.beacon_count}",
                          tags=["safety", "stop"])

    def set_pose_source(self, source):
        source = (source or "").upper()
        if source not in ("VINS", "MINS"):
            self._log(f"pose source ignored (unknown: {source})")
            return
        if self._pose_source_srv is None:
            self._log("pose source switch ignored — pose_selector service unavailable")
            return
        threading.Thread(target=self._fire_pose_source_switch, args=(source,),
                         daemon=True).start()

    def _fire_pose_source_switch(self, source):
        try:
            resp = self._pose_source_srv(data=(source == "MINS"))
            # resp.message nomme déjà l'état exact ("switched to X" /
            # "ARMED: ..." / "already on X" / "armed switch to X cancelled") —
            # relayé tel quel plutôt que préfixé par "Pose source -> X", qui
            # affirmerait à tort une bascule immédiate en cas d'armement
            # (2026-07-24, voir pose_selector.py _switch_to()).
            if resp.success:
                self._log(f"[pose_selector] {resp.message}")
            else:
                self._log(f"Pose source switch to {source} refused: {resp.message}")
        except Exception as e:
            self._log(f"Pose source switch to {source} failed: {e}")

    def reset_coords(self):
        """Bouton cockpit 'Reset 0,0,0' — remet l'odométrie locale à zéro.
        Doit préserver le repère monde (même logique que reset_coords_local /
        le reset LED) : sans recalculer world_origin AVANT le reset matériel,
        _get_world_pos() retombe sur (0,0) même si le robot n'a pas bougé,
        corrompant silencieusement la cible de Return To Base (2026-07-23,
        signalé par l'opérateur : RTB ignorait ses anciens emplacements)."""
        wx, wy = self._get_world_pos()
        self.world_origin_x = wx
        self.world_origin_y = wy
        self.world_heading  = (self.raw_yaw if self.raw_yaw is not None
                               else self.world_heading + float(self.pose[2]))
        with self._lock:
            self.origin = None
            self.traj = []
        self._log("Manual coordinate reset (0,0,0) — world frame preserved")
        self._autolog.add("COORD_RESET", "Manual coordinate reset to (0, 0, 0)",
                          "World frame re-anchored so Return To Base stays valid",
                          tags=["odometry", "reset"])
        if self._reset_srv is not None:
            threading.Thread(target=self._fire_hw_reset, daemon=True).start()

    def reset_coords_local(self):
        """Reset local odometry to (0,0,0) while preserving the world frame and markers."""
        wx, wy = self._get_world_pos()
        new_heading = (self.raw_yaw if self.raw_yaw is not None
                       else self.world_heading + float(self.pose[2]))
        self.world_origin_x = wx
        self.world_origin_y = wy
        self.world_heading  = new_heading
        self._fsm_reset_t   = time.time()
        with self._lock:
            self.origin = None
            self.traj   = []
        self._log("Local odometry reset — world frame preserved")
        if self._reset_srv is not None:
            threading.Thread(target=self._fire_hw_reset, daemon=True).start()

    def clear_map(self):
        prev_count   = self.beacon_count
        prev_markers = len(self.map_markers)
        with self._lock:
            self.traj    = []
            self.beacons = deque(maxlen=1000)
            self.origin  = None
        self.beacon_count    = 0
        self.map_markers     = deque(maxlen=1000)
        self._obs_count      = 0
        self.world_origin_x  = 0.0
        self.world_origin_y  = 0.0
        self.world_heading   = 0.0
        self._obstacle_flag  = False
        self._rtb_trail      = []
        self._rtb_waypoints  = []
        self._rtb_stall_ref_pos = None
        self._rtb_resist_ref_pos = None
        self._rtb_resist_scale   = 1.0
        self._log("Map cleared")
        self._autolog.add("MAP_CLEAR", "Navigation map cleared",
                          f"Discarded {prev_count} beacon(s), "
                          f"{prev_markers} marker(s) and full trajectory.",
                          tags=["map", "reset"])

    def set_params(self, cmd):
        def clamp(v, lo, hi):
            return max(lo, min(hi, int(v)))
        if "hue_low" in cmd:
            self._hue_low = clamp(cmd["hue_low"], 0, 179)
        if "hue_high" in cmd:
            self._hue_high = clamp(cmd["hue_high"], 0, 179)
        if "v_min" in cmd:
            self._v_min = clamp(cmd["v_min"], 0, 255)
        if "minled" in cmd:
            self._minled = clamp(cmd["minled"], 1, 6)

    def set_manual(self, lin, ang):
        lin, ang = float(lin), float(ang)
        # Une commande à vitesse non-nulle prend le contrôle (sécurité opérateur).
        # Une commande à zéro (stopDrive / mouseleave) ne casse PAS l'AUTO.
        if self.mode != "MANUEL" and (lin != 0.0 or ang != 0.0):
            self.mode = "MANUEL"
            self._log("Manual control engaged")
            self._autolog.add("MANUAL_TAKEOVER", "Operator took manual control",
                              f"lin={lin:.2f} m/s  ang={ang:.2f} rad/s",
                              tags=["manual", "override"])
        self.manual = [lin, ang]
        self.manual_t = time.time()


def main():
    LeoBackend().run()


if __name__ == "__main__":
    main()

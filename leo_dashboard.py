#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LEO Rover — Tableau de bord (tkinter)
=====================================
Fenêtre tout-en-un à lancer SUR LE PC (pas sur le robot). Elle se connecte au
master ROS du robot (10.0.0.1) et affiche EN DIRECT :

  * la vue caméra avec la détection dessinée par-dessus (LED bleues, croix,
    AprilTag, distance) -> pour voir tout de suite si les lumières sont vues ;
  * un mode "masque" + des curseurs (teinte / luminosité / nb de LED) pour
    RÉGLER la détection jusqu'à ce que les LED apparaissent bien ;
  * la CARTE (vue de dessus) : trajectoire du robot + balises déposées ;
  * des boutons : [Se connecter] [Auto / Manuel] [Cibler une balise] [STOP].

Modes :
  - AUTO    : recherche (tourne 360°, avance 1 s, recommence) ; dépose une
              balise sur la carte quand la croix de LED est vue à bonne distance.
  - MANUEL  : tu pilotes avec les flèches du clavier ou les boutons.
  - CIBLER  : le robot se centre sur la balise vue et avance vers elle.

Prérequis sur le PC : python3, rospy (ROS Noetic), opencv (cv2), numpy,
                      Pillow (PIL). Si Pillow manque : pip3 install pillow
Côté robot : la caméra D455 doit tourner (roslaunch realsense2_camera ...).

Lancement :  python3 leo_dashboard.py
"""

import os
import math
import socket
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk

import numpy as np
import cv2

# Pillow pour afficher une image OpenCV dans tkinter
try:
    from PIL import Image as PILImage, ImageTk
    _HAVE_PIL = True
except Exception:
    _HAVE_PIL = False

from collections import deque

# matplotlib intégré à Tkinter (graphiques temps réel). Optionnel : si absent,
# l'interface fonctionne sans les courbes.
try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    _HAVE_MPL = True
except Exception:
    _HAVE_MPL = False

# ── Paramètres par défaut ─────────────────────────────────────────────────────
ROBOT_IP_DEFAULT   = "10.0.0.1"
# Topic caméra : compressé par défaut (léger sur le Wi-Fi). Mettre le brut si besoin.
CAM_TOPIC_DEFAULT  = "/camera/color/image_raw/compressed"
ODOM_TOPIC         = "/firmware/wheel_odom"
CMD_TOPIC          = "/cmd_vel"

# Détection LED (valeurs de départ ; réglables en direct par les curseurs)
LED_HUE_LOW   = 80
LED_HUE_HIGH  = 135
LED_V_MIN     = 200
LED_S_MIN     = 10
LED_MIN_AREA  = 8.0
LED_MAX_AREA  = 4000.0
LED_MIN_CIRC  = 0.45
LED_CLUSTER_PX = 320
MIN_LIGHTS    = 3        # balise = 4 LED ; on VALIDE dès 3 (tolère 1 LED ratée)
N_LEDS        = 4        # on garde les 4 meilleures LED (la balise en a 4)
PATTERN_TOLERANCE_PX = 18

# Distance (empan LED gauche-droite, géométrie Carolus 0.165 m) + focale D455
CAM_FX         = 384.65
BEACON_WIDTH_M = 0.165
DIST_MIN, DIST_MAX = 0.5, 4.0
BEACON_COOLDOWN = 3.0   # s — délai mini entre deux validations de balise

# Performance / fluidité
DETECT_PERIOD = 0.10    # s — période de la détection LOURDE (~10 Hz) ; l'affichage
                        #     vidéo, lui, tourne à pleine cadence (découplé)
CB_SCALE      = 0.5     # réduction d'image pour la détection du DAMIER (×4 + rapide)
TICK_MS       = 33      # ms — cadence d'affichage caméra (~30 FPS)
CENTER_HOLD_S = 0.5     # s — on garde la dernière cible vue (visée plus stable)

# Vitesses
SEARCH_ANG  = 0.4     # rad/s rotation de recherche
ADVANCE_LIN = 0.2     # m/s avance entre deux tours
ADVANCE_T   = 1.0     # s
MANUAL_LIN  = 0.2     # m/s pilotage manuel
MANUAL_ANG  = 0.6     # rad/s pilotage manuel
TARGET_KP   = 0.004   # gain de centrage (par pixel d'écart)
TARGET_STOP_M = 0.6   # on s'arrête d'avancer sous cette distance

# Manœuvre après avoir trouvé une balise (mode "Cibler ∞ balises")
RECUL_SPEED = 0.2     # m/s   — vitesse de recul
RECUL_T     = 1.5     # s     — durée du recul
TURN_SPEED  = 0.6     # rad/s — vitesse du demi-tour (180°)

# DAMIER (checkerboard) = marqueur réel de la balise. Détection = coins INTERNES
# (cases-1). Tailles essayées (la bonne est MÉMORISÉE dès qu'elle marche -> ensuite
# détection instantanée). >>> Si ton damier n'est pas trouvé, COMPTE ses cases et
# ajoute (largeur-1, hauteur-1) en TÊTE de liste. <<<
CHECKERBOARD_SIZES = [(7, 7), (6, 6), (8, 8), (5, 5), (6, 8), (8, 6),
                      (7, 6), (6, 7), (7, 9), (9, 7), (5, 7), (7, 5),
                      (9, 6), (6, 9), (4, 4)]
# Marge (en multiples de la taille du damier) pour chercher les LED AUTOUR de lui.
# Petit = strict (rejette les reflets éloignés) ; les LED sont juste sous le damier.
LED_ROI_MARGIN = 0.6

AprilTag_FAMILIES = ["DICT_APRILTAG_36H11", "DICT_APRILTAG_36H10",
                     "DICT_APRILTAG_25H9", "DICT_ARUCO_ORIGINAL"]


def local_ip_towards(host):
    """IP locale du PC sur le réseau du robot (pour ROS_IP)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((host, 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def build_aruco():
    try:
        ar = cv2.aruco
    except Exception:
        return [], None
    get_dict = getattr(ar, "Dictionary_get", None) or \
               getattr(ar, "getPredefinedDictionary", None)
    make_par = getattr(ar, "DetectorParameters_create", None) or \
               getattr(ar, "DetectorParameters", None)
    out = []
    for name in AprilTag_FAMILIES:
        const = getattr(ar, name, None)
        if const is None or get_dict is None:
            continue
        try:
            out.append((get_dict(const), make_par()))
        except Exception:
            pass
    return out, ar


class Dashboard:
    def __init__(self, root):
        self.root = root
        root.title("LEO Rover — Tableau de bord")

        # --- État ROS / partagé entre threads ---
        self.connected   = False
        self.node_inited = False
        self.cmd_pub     = None
        self._cam_ssh    = None          # session SSH qui lance la caméra robot
        self._cam_last_line = ""         # dernière ligne du log caméra (diagnostic)
        self._cam_ok     = False         # images caméra reçues au moins une fois
        self._lock       = threading.Lock()
        self.latest_bgr  = None          # dernière image DÉCODÉE (BGR) pour la GUI
        self.img_t       = 0.0           # horodatage RÉCEPTION image (watchdog)
        self._aruco, self._ar = build_aruco()

        # --- Buffer caméra 1-slot + thread vision (anti-saturation) ---
        # Le callback ROS y dépose la trame BRUTE ; le thread vision la décode
        # et fait la détection -> les callbacks ROS ne sont JAMAIS bloqués.
        self._cam_msg   = None       # trame BRUTE (du callback ROS)
        self._cam_seq   = 0
        self._dec_seq   = 0          # n° de la dernière trame DÉCODÉE
        self._dec_proc  = 0          # trame brute déjà décodée (thread décodage)
        self._det_proc  = 0          # trame décodée déjà analysée (thread détection)
        self._vision_started = False
        self._closing   = False

        # Valeurs de réglage copiées depuis les curseurs (lecture des IntVar
        # UNIQUEMENT dans le thread Tk ; le thread vision lit ces ints simples).
        self._hue_low, self._hue_high = LED_HUE_LOW, LED_HUE_HIGH
        self._v_min, self._minled     = LED_V_MIN, MIN_LIGHTS

        # Validation balise + cache de la taille du damier (détection instantanée)
        self._beacon_prev = False
        self.beacon_count = 0
        self._reset_srv = None
        self._cb_size = None             # taille du damier mémorisée une fois trouvée
        self._cb_sb_t = 0.0              # throttle de la découverte robuste (SB)

        # Journal système (file thread-safe -> zone de texte de la GUI)
        self.logq = queue.Queue(maxsize=500)

        # Odométrie / carte
        self.origin   = None             # (x,y,yaw) première odom = origine carte
        self.pose     = (0.0, 0.0, 0.0)  # pose relative (x,y,yaw)
        self.raw_yaw  = None
        self.traj     = []               # trajectoire [(x,y), ...]
        self.beacons  = []               # balises [(x,y,label), ...]

        # Détection courante (remplie par le thread vision)
        self.det = {"leds": [], "near": [], "cluster": [], "checker": None,
                    "center": None, "led_center": None, "dist": None,
                    "tag": None, "mask": None}
        self.last_tag_id = None
        self.last_w = 0                  # largeur de la dernière image (pour CIBLER)
        self.compressed = True

        # --- Télémétrie carte électronique (batterie, moteurs, sorties) ---
        self.batt_v   = None             # tension batterie (V)
        self.wheels   = {"vel": [0]*4, "pwm": [0]*4, "torque": [0]*4}
        self.last_cmd = (0.0, 0.0)       # dernière consigne (lin, ang) envoyée
        # horodatages d'activité par capteur -> voyants "en direct"
        self._t = {"cam": 0.0, "odom": 0.0, "batt": 0.0, "wheels": 0.0}

        # Modes / commandes
        self.mode = "MANUEL"             # MANUEL | AUTO | CIBLER | INFINI
        self.manual = [0.0, 0.0]         # [lin, ang] pilotage manuel
        self.search_state = "SCAN"
        self.scan_accum = 0.0
        self.scan_last_yaw = None
        self.adv_start = 0.0
        self.last_beacon_t = 0.0

        # Mode "Cibler ∞ balises" : machine d'états SEEK -> RECUL -> TURN180 -> ...
        self.inf_state = "SEEK"
        self.inf_t = 0.0
        self._inf_last_count = 0         # nb de balises au moment du dernier SEEK
        self.turn_accum = 0.0            # angle accumulé pour le 180°
        self.turn_last_yaw = None

        # Cadence d'affichage + persistance de la cible (visée stable)
        self._tick_count = 0
        self._last_center = None         # dernière cible vue (px)
        self._last_center_t = 0.0

        # --- Centre d'opérations : historiques + Hz + temps de mission ---
        self.vel_meas    = (0.0, 0.0)    # vitesse MESURÉE (lin, ang) du robot
        self.vel_hist    = deque(maxlen=300)   # (t, lin, ang) pour la courbe
        self.dist_hist   = deque(maxlen=300)   # (t, distance balise)
        self.cam_times   = deque(maxlen=120)   # horodatages images -> Hz caméra
        self.odom_times  = deque(maxlen=120)   # horodatages odom   -> Hz odom
        self._mission_start = None             # début de mission (à la connexion)
        self._cam_low_warned = 0.0             # anti-spam de l'alerte flux bas
        self._mpl_canvas = None

        # Paramètres réglables (curseurs)
        self.p_hue_low = tk.IntVar(value=LED_HUE_LOW)
        self.p_hue_high = tk.IntVar(value=LED_HUE_HIGH)
        self.p_v_min   = tk.IntVar(value=LED_V_MIN)
        self.p_minled  = tk.IntVar(value=MIN_LIGHTS)
        self.show_mask = tk.BooleanVar(value=False)
        self.robot_ip  = tk.StringVar(value=ROBOT_IP_DEFAULT)
        self.cam_topic = tk.StringVar(value=CAM_TOPIC_DEFAULT)
        # Pas de mot de passe pre-rempli par defaut (audit securite 2026-08-04) :
        # l'authentification par cle (voir tools/robot_env.sh, annexe D §D.4.4)
        # est le chemin recommande pour tout usage automatise ; ce champ reste
        # un filet de secours pour l'usage interactif, a saisir a la main.
        self.ssh_pw    = tk.StringVar(value="")

        self._build_ui()
        self._bind_keys()

        # Boucle périodique (affichage + contrôle) dans le thread tkinter
        self.root.after(80, self._tick)
        self.root.after(500, self._update_graphs)   # graphiques ~2 Hz (peu coûteux)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ══════════════════════════════════════════════════════════════════════ #
    # Interface
    # ══════════════════════════════════════════════════════════════════════ #
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.grid(row=0, column=0, sticky="nsew")

        # Barre de connexion
        bar = ttk.Frame(top)
        bar.grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))
        ttk.Label(bar, text="Robot IP:").pack(side="left")
        ttk.Entry(bar, textvariable=self.robot_ip, width=12).pack(side="left", padx=4)
        ttk.Label(bar, text="Topic caméra:").pack(side="left")
        ttk.Entry(bar, textvariable=self.cam_topic, width=34).pack(side="left", padx=4)
        self.btn_conn = ttk.Button(bar, text="Se connecter", command=self.connect)
        self.btn_conn.pack(side="left", padx=4)
        self.lbl_status = ttk.Label(bar, text="● déconnecté", foreground="red")
        self.lbl_status.pack(side="left", padx=8)

        # 2e ligne : gestion automatique de la caméra du robot (par SSH)
        bar2 = ttk.Frame(top)
        bar2.grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 6))
        ttk.Label(bar2, text="Mot de passe SSH (pi):").pack(side="left")
        ttk.Entry(bar2, textvariable=self.ssh_pw, width=12,
                  show="*").pack(side="left", padx=4)
        ttk.Button(bar2, text="Démarrer la caméra (robot)",
                   command=self.start_camera_robot).pack(side="left", padx=4)
        self.lbl_cam = ttk.Label(bar2, text="caméra : —", foreground="#888")
        self.lbl_cam.pack(side="left", padx=8)

        # Vue caméra (gauche)
        left = ttk.LabelFrame(top, text="Caméra (détection en direct)", padding=4)
        left.grid(row=2, column=0, sticky="nsew", padx=(0, 6))
        self.cam_label = ttk.Label(left)
        self.cam_label.pack()
        self.lbl_det = ttk.Label(left, text="—", font=("TkDefaultFont", 10, "bold"))
        self.lbl_det.pack(pady=2)

        # Réglages détection
        tune = ttk.Frame(left)
        tune.pack(fill="x", pady=2)
        self._slider(tune, "Teinte min", self.p_hue_low, 0, 179, 0)
        self._slider(tune, "Teinte max", self.p_hue_high, 0, 179, 1)
        self._slider(tune, "Luminosité min", self.p_v_min, 0, 255, 2)
        self._slider(tune, "Nb LED mini", self.p_minled, 1, 6, 3)
        ttk.Checkbutton(tune, text="Voir le masque (réglage)",
                        variable=self.show_mask).grid(row=4, column=0,
                                                      columnspan=2, sticky="w")

        # Colonne DROITE : GRAPHIQUES (en haut, bien visibles) + CARTE (dessous)
        rightcol = ttk.Frame(top)
        rightcol.grid(row=2, column=1, sticky="n")
        self._build_graphs(rightcol)               # <-- graphiques EN HAUT À DROITE

        mapf = ttk.LabelFrame(rightcol, text="Carte (vue de dessus)", padding=4)
        mapf.pack(fill="x", pady=(6, 0))
        self.map_w = self.map_h = 250
        self.canvas = tk.Canvas(mapf, width=self.map_w, height=self.map_h,
                                bg="#101418", highlightthickness=0)
        self.canvas.pack()
        ttk.Button(mapf, text="Effacer la carte",
                   command=self._clear_map).pack(pady=3)

        # Boutons d'action
        actions = ttk.Frame(top)
        actions.grid(row=3, column=0, columnspan=2, sticky="w", pady=6)
        self.btn_mode = ttk.Button(actions, text="Mode : MANUEL",
                                   command=self.toggle_mode, width=18)
        self.btn_mode.pack(side="left", padx=4)
        ttk.Button(actions, text="Cibler une balise", width=16,
                   command=self.target_beacon).pack(side="left", padx=4)
        ttk.Button(actions, text="Cibler ∞ balises", width=16,
                   command=self.infinite_beacons).pack(side="left", padx=4)
        ttk.Button(actions, text="STOP", width=10,
                   command=self.stop).pack(side="left", padx=4)
        ttk.Button(actions, text="Reset manuel (0,0,0)", width=18,
                   command=self._manual_reset).pack(side="left", padx=4)
        ttk.Label(actions, text="  (Manuel : flèches ↑↓←→)").pack(
            side="left", padx=8)

        # Bandeau d'indicateurs numériques (mission / Hz / total balises)
        indic = ttk.Frame(top)
        indic.grid(row=4, column=0, columnspan=2, sticky="w", pady=(0, 2))
        self.lbl_indic = tk.Label(indic, text="Mission --:--   |   Caméra -- Hz"
                                  "   |   Odom -- Hz   |   Balises 0",
                                  font=("Courier New", 11, "bold"),
                                  fg="#27e0ff", bg="#0b0f17")
        self.lbl_indic.pack(fill="x")

        # Panneau TÉLÉMÉTRIE (batterie + moteurs + consigne + voyants en direct)
        telem = ttk.LabelFrame(top, text="Télémétrie carte (en direct)", padding=4)
        telem.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(2, 4))
        self.telem_w, self.telem_h = 770, 140
        self.telem = tk.Canvas(telem, width=self.telem_w, height=self.telem_h,
                               bg="#0b0f17", highlightthickness=0)
        self.telem.pack()

        # Ligne du bas : POSITION X/Y/Z + LOG SYSTÈME
        bottom = ttk.Frame(top)
        bottom.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        posf = ttk.LabelFrame(bottom, text="Position (X / Y / Z)", padding=4)
        posf.pack(side="left", fill="y")
        self.lbl_xyz = tk.Label(posf, text="X = --\nY = --\nZ = --\nYAW = --",
                                font=("Courier New", 13, "bold"), justify="left",
                                bg="#0b0f17", fg="#dfe9f7", width=14, anchor="w")
        self.lbl_xyz.pack(padx=4, pady=4)
        logf = ttk.LabelFrame(bottom, text="Log système", padding=4)
        logf.pack(side="left", fill="both", expand=True, padx=(8, 0))
        self.logbox = tk.Text(logf, height=6, width=66, bg="#0b0f17",
                              fg="#2dd06f", font=("Courier New", 9),
                              relief="flat", highlightthickness=0, state="disabled")
        self.logbox.pack(fill="both", expand=True, padx=2, pady=2)

    def _build_graphs(self, parent):
        graphf = ttk.LabelFrame(parent, text="Graphiques (temps réel)", padding=2)
        graphf.pack(fill="x")
        if not _HAVE_MPL:
            ttk.Label(graphf, text="matplotlib absent -> pip3 install matplotlib "
                      "(graphiques désactivés)").pack()
            return
        # 2 sous-graphes EMPILÉS (verticalement) pour rester larges et lisibles
        fig = Figure(figsize=(4.4, 2.7), dpi=100, facecolor="#0b0f17")
        self._ax_vel = fig.add_subplot(2, 1, 1)
        self._ax_dist = fig.add_subplot(2, 1, 2)
        for ax, title in ((self._ax_vel, "Vitesse robot (m/s · rad/s)"),
                          (self._ax_dist, "Distance balise (m)")):
            ax.set_facecolor("#05080f")
            ax.tick_params(colors="#9fb3c8", labelsize=7)
            for sp in ax.spines.values():
                sp.set_color("#33424f")
            ax.set_title(title, color="#27e0ff", fontsize=8)
            ax.set_xlim(-20, 0)
            ax.grid(True, color="#15314f", lw=0.4)
        (self._ln_lin,) = self._ax_vel.plot([], [], color="#27e0ff", lw=1.3,
                                            label="lin")
        (self._ln_ang,) = self._ax_vel.plot([], [], color="#ffb000", lw=1.3,
                                            label="ang")
        leg = self._ax_vel.legend(fontsize=6, loc="upper left",
                                  facecolor="#0b0f17", edgecolor="#33424f")
        for _txt in leg.get_texts():          # couleur du texte (compat mpl 3.1)
            _txt.set_color("#dfe9f7")
        (self._ln_dist,) = self._ax_dist.plot([], [], color="#2dff8a", lw=1.5)
        fig.subplots_adjust(left=0.13, right=0.97, top=0.90, bottom=0.12,
                            hspace=0.55)
        self._mpl_canvas = FigureCanvasTkAgg(fig, master=graphf)
        self._mpl_canvas.get_tk_widget().pack(fill="x")

    def _slider(self, parent, label, var, lo, hi, row):
        ttk.Label(parent, text=label, width=14).grid(row=row, column=0, sticky="w")
        ttk.Scale(parent, from_=lo, to=hi, variable=var, orient="horizontal",
                  length=180).grid(row=row, column=1, sticky="w")

    def _bind_keys(self):
        # Pilotage manuel au clavier (flèches)
        self.root.bind("<KeyPress-Up>",    lambda e: self._set_manual(MANUAL_LIN, 0))
        self.root.bind("<KeyPress-Down>",  lambda e: self._set_manual(-MANUAL_LIN, 0))
        self.root.bind("<KeyPress-Left>",  lambda e: self._set_manual(0, MANUAL_ANG))
        self.root.bind("<KeyPress-Right>", lambda e: self._set_manual(0, -MANUAL_ANG))
        for k in ("<KeyRelease-Up>", "<KeyRelease-Down>",
                  "<KeyRelease-Left>", "<KeyRelease-Right>"):
            self.root.bind(k, lambda e: self._set_manual(0, 0))

    def _set_manual(self, lin, ang):
        self.manual = [lin, ang]

    # ══════════════════════════════════════════════════════════════════════ #
    # Connexion ROS
    # ══════════════════════════════════════════════════════════════════════ #
    def connect(self):
        ip = self.robot_ip.get().strip()
        # 1) Le master répond-il ? (test socket rapide, pas de blocage)
        try:
            s = socket.create_connection((ip, 11311), timeout=3)
            s.close()
        except Exception as e:
            self._set_status(f"● master injoignable ({e})", "red")
            return

        os.environ["ROS_MASTER_URI"] = f"http://{ip}:11311"
        os.environ["ROS_IP"] = local_ip_towards(ip)

        try:
            import rospy
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import Image, CompressedImage
            from std_msgs.msg import Float32
            self._rospy = rospy
            self._Twist = Twist
            self._Image = Image
            self._Compressed = CompressedImage
            self._Float32 = Float32
            try:
                from leo_msgs.msg import WheelOdom, WheelStates
                self._WheelOdom = WheelOdom
                self._WheelStates = WheelStates
            except Exception:
                self._WheelOdom = None
                self._WheelStates = None

            if not self.node_inited:
                rospy.init_node("leo_dashboard", anonymous=True,
                                disable_signals=True)
                self.node_inited = True

            # (Re)création des abonnements / publication
            self.cmd_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)
            topic = self.cam_topic.get().strip()
            self.compressed = topic.endswith("/compressed")
            ctype = CompressedImage if self.compressed else Image
            # tcp_nodelay=True (latence mini) + queue_size=1 (on garde la trame
            # la plus FRAÎCHE et on jette le retard -> pas d'accumulation).
            if hasattr(self, "_sub_cam") and self._sub_cam:
                self._sub_cam.unregister()
            self._sub_cam = rospy.Subscriber(topic, ctype, self._on_image,
                                             queue_size=1, buff_size=2**24,
                                             tcp_nodelay=True)
            if self._WheelOdom is not None:
                if hasattr(self, "_sub_odom") and self._sub_odom:
                    self._sub_odom.unregister()
                self._sub_odom = rospy.Subscriber(ODOM_TOPIC, self._WheelOdom,
                                                  self._on_odom, queue_size=20,
                                                  tcp_nodelay=True)
            # Télémétrie carte : batterie + état des 4 roues/moteurs
            for attr in ("_sub_batt", "_sub_wheels"):
                if getattr(self, attr, None):
                    getattr(self, attr).unregister()
            self._sub_batt = rospy.Subscriber("/firmware/battery", Float32,
                                              self._on_battery, queue_size=5,
                                              tcp_nodelay=True)
            if self._WheelStates is not None:
                self._sub_wheels = rospy.Subscriber(
                    "/firmware/wheel_states", self._WheelStates,
                    self._on_wheels, queue_size=5, tcp_nodelay=True)

            # Service de RESET MATÉRIEL de l'odométrie (coordonnées physiques -> 0)
            try:
                from std_srvs.srv import Trigger
                rospy.wait_for_service("firmware/reset_odometry", timeout=2.0)
                self._reset_srv = rospy.ServiceProxy("firmware/reset_odometry",
                                                     Trigger)
            except Exception:
                self._reset_srv = None

            # DEUX threads séparés : décodage/affichage (rapide) + détection (lourd)
            # -> le flux vidéo ne se fige JAMAIS, même quand la détection cherche.
            if not self._vision_started:
                self._vision_started = True
                threading.Thread(target=self._decode_loop, name="decode",
                                 daemon=True).start()
                threading.Thread(target=self._detect_loop, name="detect",
                                 daemon=True).start()

            self.connected = True
            if self._mission_start is None:
                self._mission_start = time.time()    # chrono de mission
            self._set_status(f"● connecté ({ip})", "#19a319")
            self._log(f"Connecté à {ip}")
            # Démarrage automatique de la caméra du robot si elle ne tourne pas
            threading.Thread(target=self._auto_start_camera_if_needed,
                             daemon=True).start()
        except Exception as e:
            self._set_status(f"● erreur ROS : {e}", "red")

    # ══════════════════════════════════════════════════════════════════════ #
    # Caméra du robot lancée AUTOMATIQUEMENT par SSH (paramiko)
    # ══════════════════════════════════════════════════════════════════════ #
    def start_camera_robot(self):
        """Bouton : (re)lance la caméra sur le robot, dans un thread."""
        threading.Thread(target=self._ssh_launch_camera, daemon=True).start()

    def _auto_start_camera_if_needed(self):
        """Si aucune image n'arrive ~2.5 s après connexion -> on lance la caméra."""
        import time
        time.sleep(2.5)
        with self._lock:
            fresh = bool(self.img_t) and (time.time() - self.img_t) < 1.0
        if fresh:
            self._cam_status("caméra : déjà active ✓", "#19a319")
        else:
            self._ssh_launch_camera()

    def _ssh_launch_camera(self):
        """Ouvre une session SSH (pi@robot) et lance le driver RealSense.

        get_pty=True : quand on ferme la session (fermeture de la fenêtre), le
        roslaunch distant reçoit SIGHUP et s'arrête -> pas de processus orphelin.
        """
        try:
            import paramiko
        except Exception:
            self._cam_status("paramiko absent (pip3 install paramiko) -> "
                             "lance la caméra à la main", "red")
            return
        ip = self.robot_ip.get().strip()
        pw = self.ssh_pw.get()
        self._cam_status("caméra : connexion SSH…", "#cc8800")
        try:
            cli = paramiko.SSHClient()
            cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            cli.connect(ip, username="pi", password=pw, timeout=8)
            # IMPORTANT : `bash -ic` = shell INTERACTIF -> charge ~/.bashrc, donc
            # le workspace catkin où est installé realsense2_camera (sinon le
            # paquet est "introuvable" dans un shell non-interactif).
            # 60 FPS source (couleur seule) pour un flux le + fluide possible.
            # Si le Wi-Fi/Pi saturent et que ça saccade -> repasser color_fps:=30.
            launch = ("roslaunch realsense2_camera rs_camera.launch "
                      "enable_depth:=false color_width:=640 "
                      "color_height:=480 color_fps:=60")
            inner = (f"pkill -f realsense2_camera 2>/dev/null; sleep 1; "
                     f"stdbuf -oL {launch}")
            cmd = f"bash -ic '{inner}' 2>&1"
            stdin, stdout, stderr = cli.exec_command(cmd, get_pty=True)
            self._cam_ssh = cli
            threading.Thread(target=self._drain_cam, args=(stdout,),
                             daemon=True).start()
            threading.Thread(target=self._cam_watchdog, daemon=True).start()
            self._cam_status("caméra : démarrage… (~20 s)", "#cc8800")
        except Exception as e:
            self._cam_status(f"caméra : échec SSH ({e})", "red")

    def _cam_watchdog(self):
        """Si aucune image n'arrive ~30 s après le lancement -> diagnostic clair."""
        import time
        time.sleep(30)
        with self._lock:
            fresh = bool(self.img_t) and (time.time() - self.img_t) < 1.5
        if not fresh:
            tail = (self._cam_last_line or "(aucune sortie)")[:70]
            self._cam_status(f"caméra KO — robot dit : {tail}", "red")

    def _drain_cam(self, stdout):
        """Lit la sortie du roslaunch distant pour suivre l'état de la caméra."""
        try:
            for line in iter(stdout.readline, ""):
                if not line:
                    break
                line = line.strip()
                if line:
                    self._cam_last_line = line        # gardé pour le diagnostic
                low = line.lower()
                if "realsense node is up" in low:
                    self._cam_status("caméra : EN LIGNE ✓", "#19a319")
                elif ("not found" in low or "cannot locate" in low or
                      "resource not found" in low):
                    self._cam_status("caméra KO : paquet realsense introuvable "
                                     "sur le robot", "red")
        except Exception:
            pass

    def _stop_camera_robot(self):
        """Coupe la caméra du robot (à la fermeture de la fenêtre)."""
        cli = self._cam_ssh
        self._cam_ssh = None
        if cli is None:
            return
        try:
            cli.exec_command("pkill -f realsense2_camera")
        except Exception:
            pass
        try:
            cli.close()
        except Exception:
            pass

    def _cam_status(self, text, color):
        # Mise à jour thread-safe du label (on repasse par le thread tkinter)
        self.root.after(0, lambda: self.lbl_cam.config(text=text, foreground=color))

    def _set_status(self, text, color):
        self.lbl_status.config(text=text, foreground=color)

    # ══════════════════════════════════════════════════════════════════════ #
    # Callbacks ROS (threads de fond) — on stocke juste les données
    # ══════════════════════════════════════════════════════════════════════ #
    def _on_image(self, msg):
        # CALLBACK ULTRA-LÉGER : on ne fait QUE déposer la trame brute + l'heure.
        # Le décodage est fait par _decode_loop et la détection par _detect_loop
        # (deux threads séparés) -> ce callback ROS rend la main en microsecondes
        # -> caméra ET odométrie ne saturent jamais.
        t = time.time()
        with self._lock:
            self._cam_msg = msg
            self._cam_seq += 1
            self.img_t = t
            self._t["cam"] = t
            self.cam_times.append(t)           # -> Hz caméra

    def _decode(self, msg):
        if self.compressed:
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
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

    def _decode_loop(self):
        """Thread DÉCODAGE/AFFICHAGE — ULTRA-RAPIDE, JAMAIS bloqué par la détection.
        Il ne fait QUE décoder la dernière trame brute -> image affichée. C'est ce
        thread qui garantit un flux vidéo FLUIDE en continu (le décodage JPEG ~3 ms)."""
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
                        self.latest_bgr = bgr        # AFFICHAGE : tout de suite
                        self._dec_seq += 1
            except Exception:
                pass

    def _detect_loop(self):
        """Thread DÉTECTION — séparé, peut être LENT (damier SB) sans jamais figer
        l'affichage. Analyse la dernière image décodée toutes les DETECT_PERIOD."""
        last = 0.0
        while not self._closing:
            now = time.time()
            if now - last < DETECT_PERIOD:
                time.sleep(0.01)
                continue
            with self._lock:
                bgr = self.latest_bgr
                seq = self._dec_seq
            if bgr is None or seq == self._det_proc:
                time.sleep(0.01)
                continue
            last, self._det_proc = now, seq
            try:
                det = self._analyse(bgr)
                with self._lock:
                    self.det = det
                    if det.get("dist") is not None:
                        self.dist_hist.append((now, det["dist"]))  # courbe distance
                self._validate_beacon(det, now)
            except Exception:
                pass

    def _validate_beacon(self, det, now):
        """Balise VALIDE = DAMIER trouvé + assez de LED groupées AUTOUR de lui.
        Le damier est un marqueur UNIQUE -> on ne bloque PAS sur la distance
        (estimation peu fiable). Front montant + anti-rebond -> mémorisation +
        reset (0,0,0) + log. La mission continue immédiatement."""
        cluster = det.get("cluster") or []
        checker = det.get("checker")
        valid = (checker is not None and len(cluster) >= self._minled)
        if valid and not self._beacon_prev and \
                (now - self.last_beacon_t) > BEACON_COOLDOWN:
            self._on_beacon_validated(det)
            self.last_beacon_t = now
        self._beacon_prev = valid

    def _on_beacon_validated(self, det):
        # Identité : AprilTag si présent, sinon damier (un damier n'a pas d'ID).
        if det.get("tag") is not None:
            ident = f"tag {det['tag'][0]}"
        elif det.get("checker") is not None:
            s = det["checker"][0]
            ident = f"DAMIER {s[0]}x{s[1]}"
        else:
            ident = "balise"
        # Mémorisation + compteur + RESET des coordonnées (0,0,0) : tout est LOCAL
        # et immédiat -> le compteur monte, le log s'affiche, la position repart à
        # zéro à coup sûr. (origin=None -> la prochaine odométrie devient l'origine,
        # donc la pose affichée redevient 0,0,0, sans appel bloquant ni à-coup.)
        self.beacon_count += 1
        with self._lock:
            self.beacons = [(0.0, 0.0, ident)]
            self.origin = None
            self.traj = []
        self._log(f"BALISE [{ident}] REPÉRÉE - RESET EFFECTUÉ")

    def _on_odom(self, msg):
        try:
            px, py, yaw = float(msg.pose_x), float(msg.pose_y), float(msg.pose_yaw)
        except Exception:
            return
        now = time.time()
        self._t["odom"] = now
        self.odom_times.append(now)            # -> Hz odométrie
        # vitesse MESURÉE du robot (champs velocity_lin/ang de leo_msgs/WheelOdom)
        try:
            vlin, vang = float(msg.velocity_lin), float(msg.velocity_ang)
            self.vel_meas = (vlin, vang)
            self.vel_hist.append((now, vlin, vang))
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

    def _on_battery(self, msg):
        import time
        try:
            self.batt_v = float(msg.data)
            self._t["batt"] = time.time()
        except Exception:
            pass

    def _on_wheels(self, msg):
        """leo_msgs/WheelStates : velocity[4], torque[4], pwm_duty_cycle[4]
        (ordre firmware : FL, RL, FR, RR)."""
        import time
        try:
            self.wheels = {"vel": list(msg.velocity),
                           "pwm": list(msg.pwm_duty_cycle),
                           "torque": list(msg.torque)}
            self._t["wheels"] = time.time()
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════════════ #
    # Détection (LED bleues -> croix) — paramètres pris des curseurs
    # ══════════════════════════════════════════════════════════════════════ #
    def _detect_leds(self, bgr):
        # NB : on lit les ints simples (_v_min, _hue_low/high) copiés depuis les
        # curseurs par le thread Tk -> pas d'accès Tkinter hors thread principal.
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        mask = ((V >= self._v_min) & (S >= LED_S_MIN) &
                (H >= self._hue_low) &
                (H <= self._hue_high)).astype(np.uint8) * 255
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
            if per <= 0:
                continue
            if 4.0 * math.pi * area / (per * per) < LED_MIN_CIRC:
                continue
            M = cv2.moments(c)
            if M["m00"] > 0:
                leds.append((M["m10"] / M["m00"], M["m01"] / M["m00"], area))
        return leds, mask

    def _cb_fast(self, img, size):
        """Détecteur RAPIDE (findChessboardCorners + FAST_CHECK). None si absent."""
        try:
            ok, c = cv2.findChessboardCorners(
                img, size, flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            return c if ok else None
        except Exception:
            return None

    def _cb_sb(self, img, size):
        """Détecteur ROBUSTE (findChessboardCornersSB) — angles/lumière difficiles."""
        try:
            ok, c = cv2.findChessboardCornersSB(
                img, size, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            return c if ok else None
        except Exception:
            return None

    def _detect_checkerboard(self, gray):
        """Détection du DAMIER, robuste ET rapide :
          1) taille DÉJÀ connue -> SB (robuste) sur CETTE taille -> instantané ;
          2) découverte rapide (FAST_CHECK) sur toutes les tailles ;
          3) découverte robuste (SB) sur les tailles probables, throttlée (cas durs).
        La taille trouvée est MÉMORISÉE -> ensuite il la retrouve tout de suite."""
        small = cv2.resize(gray, None, fx=CB_SCALE, fy=CB_SCALE,
                           interpolation=cv2.INTER_AREA)

        # 1) Taille connue -> on ne teste QUE celle-là (SB robuste, repli rapide)
        if self._cb_size is not None:
            c = self._cb_sb(small, self._cb_size)
            if c is None:
                c = self._cb_fast(small, self._cb_size)
            if c is not None:
                return (self._cb_size, np.asarray(c) / CB_SCALE)

        # 2) Découverte RAPIDE (peu coûteuse) sur toutes les tailles
        for size in CHECKERBOARD_SIZES:
            c = self._cb_fast(small, size)
            if c is not None:
                return self._lock_cb(size, c)

        # 3) Découverte ROBUSTE (SB, plus lente) sur les tailles probables,
        #    limitée à ~2 Hz pour ne pas saturer (cas damier incliné / sombre).
        now = time.time()
        if now - self._cb_sb_t > 0.5:
            self._cb_sb_t = now
            for size in CHECKERBOARD_SIZES[:6]:
                c = self._cb_sb(small, size)
                if c is not None:
                    return self._lock_cb(size, c)
        return None

    def _lock_cb(self, size, corners_small):
        if self._cb_size != size:
            self._cb_size = size
            self._log(f"DAMIER {size[0]}x{size[1]} verrouillé")
        return (size, np.asarray(corners_small) / CB_SCALE)

    def _cluster_lights(self, leds):
        """Regroupe les LED proches (autour de la plus grosse) et ne garde que les
        N_LEDS plus grosses -> exactement les 4 vraies LED, sans les parasites.
        Renvoie (liste_cluster, centre) ou ([], None) si pas assez de LED."""
        if len(leds) < self._minled:
            return [], None
        leds = sorted(leds, key=lambda l: l[2], reverse=True)   # par taille
        ax, ay, _ = leds[0]
        grouped = [(x, y, a) for (x, y, a) in leds
                   if math.hypot(x - ax, y - ay) <= LED_CLUSTER_PX]
        if len(grouped) < self._minled:
            return [], None
        grouped = grouped[:N_LEDS]                  # on garde les 4 plus grosses
        cluster = [(x, y) for (x, y, _a) in grouped]
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        return cluster, (cx, cy)

    def _detect_tag(self, bgr):
        if not self._aruco:
            return None
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        for dic, params in self._aruco:
            try:
                corners, ids, _ = self._ar.detectMarkers(gray, dic,
                                                         parameters=params)
            except Exception:
                continue
            if ids is not None and len(ids) > 0:
                quad = corners[0][0]
                cx = float(np.mean(quad[:, 0]))
                cy = float(np.mean(quad[:, 1]))
                return int(ids.flatten()[0]), (cx, cy), quad
        return None

    def _analyse(self, bgr):
        leds_all, mask = self._detect_leds(bgr)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        checker = self._detect_checkerboard(gray)

        # Si le DAMIER est trouvé, on ne garde QUE les LED situées autour de lui
        # (le damier sert d'ancre) -> élimine les reflets parasites ailleurs.
        cb_center = None
        if checker is not None:
            pts = checker[1].reshape(-1, 2)
            x0, y0 = float(pts[:, 0].min()), float(pts[:, 1].min())
            x1, y1 = float(pts[:, 0].max()), float(pts[:, 1].max())
            w, h = x1 - x0, y1 - y0
            mx, my = w * LED_ROI_MARGIN, h * LED_ROI_MARGIN
            rx0, rx1 = x0 - mx, x1 + mx
            ry0, ry1 = y0 - my, y1 + my * 1.6          # un peu + bas (LED dessous)
            cb_center = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
            near = [l for l in leds_all if rx0 <= l[0] <= rx1 and ry0 <= l[1] <= ry1]
        else:
            near = leds_all

        cluster, led_center = self._cluster_lights(near)
        dist = None
        if cluster:
            xs = [p[0] for p in cluster]
            span = max(xs) - min(xs)
            dist = (CAM_FX * BEACON_WIDTH_M / span) if span >= 1 else None
        # cible à VISER = centre du damier si dispo, sinon centre des LED
        center = cb_center if cb_center is not None else led_center
        # Marqueur = DAMIER -> pas de détection AprilTag (inutile, on gagne du temps)
        return {"leds": leds_all, "near": near, "cluster": cluster,
                "checker": checker, "center": center, "led_center": led_center,
                "dist": dist, "tag": None, "mask": mask}

    # ══════════════════════════════════════════════════════════════════════ #
    # Boucle périodique : affichage + contrôle
    # ══════════════════════════════════════════════════════════════════════ #
    def _tick(self):
        # 1) Copie des curseurs (lecture des IntVar dans le thread Tk) vers des
        #    ints simples que le thread vision lit sans toucher Tkinter.
        self._hue_low = int(self.p_hue_low.get())
        self._hue_high = int(self.p_hue_high.get())
        self._v_min = int(self.p_v_min.get())
        self._minled = int(self.p_minled.get())

        # 2) Lecture de l'image décodée + détection publiées par le thread vision.
        #    (latest_bgr est remplacé en bloc -> lire la référence suffit, sans copie)
        with self._lock:
            bgr = self.latest_bgr
            det = self.det
            img_age = time.time() - self.img_t if self.img_t else 999

        cam_alive = self.img_t and img_age < 1.0
        if cam_alive and not self._cam_ok:
            self._cam_ok = True
            self.lbl_cam.config(text="caméra : EN LIGNE ✓", foreground="#19a319")
        elif not cam_alive and self._cam_ok:
            self._cam_ok = False

        # --- À CHAQUE tick (~30 FPS) : image vidéo + pilotage (fluide) ---
        if bgr is not None:
            view = det["mask"] if (self.show_mask.get() and det.get("mask") is not None) \
                else bgr
            view = self._draw_overlay(view, det)
            self._show_image(view)
            self._update_det_label(det, cam_alive)
        self._drive(cam_alive)

        # --- Redraws LOURDS moins souvent (~7 Hz) : carte + télémétrie + X/Y/Z ---
        self._tick_count += 1
        if self._tick_count % 4 == 0:
            self._draw_map()
            self._draw_telemetry()
            self._update_xyz()
            self._update_indicators()
        self._drain_log()
        self.root.after(TICK_MS, self._tick)

    def _log(self, text):
        line = f"[{time.strftime('%H:%M:%S')}] {text}"
        try:
            self.logq.put_nowait(line)
        except queue.Full:
            pass

    def _drain_log(self):
        wrote = False
        while True:
            try:
                line = self.logq.get_nowait()
            except queue.Empty:
                break
            self.logbox.config(state="normal")
            self.logbox.insert("end", line + "\n")
            wrote = True
        if wrote:
            self.logbox.see("end")
            if int(self.logbox.index("end-1c").split(".")[0]) > 200:
                self.logbox.delete("1.0", "60.0")
            self.logbox.config(state="disabled")

    def _update_xyz(self):
        rx, ry, ryaw = self.pose
        self.lbl_xyz.config(
            text=f"X = {rx:+07.3f} m\nY = {ry:+07.3f} m\nZ = {0.0:+07.3f} m\n"
                 f"YAW = {math.degrees(ryaw):+06.1f}°")

    # ── Centre d'opérations : indicateurs, graphiques, reset manuel ──────────
    def _hz(self, times):
        """Fréquence (Hz) = nb d'évènements dans la dernière seconde."""
        now = time.time()
        return sum(1 for t in list(times) if now - t <= 1.0)

    def _update_indicators(self):
        # temps de mission
        if self._mission_start is None:
            mt = "--:--"
        else:
            s = int(time.time() - self._mission_start)
            mt = f"{s // 60:02d}:{s % 60:02d}"
        cam = self._hz(self.cam_times)
        odm = self._hz(self.odom_times)
        self.lbl_indic.config(
            text=f"Mission {mt}   |   Caméra {cam:2d} Hz   |   "
                 f"Odom {odm:2d} Hz   |   Balises {self.beacon_count}")
        # Alerte mission : flux caméra bas (connecté mais < 8 Hz), anti-spam 5 s
        if self.connected and cam < 8 and self.img_t and \
                (time.time() - self._cam_low_warned) > 5.0:
            self._cam_low_warned = time.time()
            self._log(f"Alerte : flux caméra bas ({cam} Hz)")

    def _update_graphs(self):
        """Redessine les courbes (vitesses + distance) ~2 Hz. Léger : draw_idle."""
        if self._mpl_canvas is not None:
            now = time.time()
            with self._lock:
                vh = list(self.vel_hist)
                dh = list(self.dist_hist)
            if vh:
                tv = [t - now for (t, _l, _a) in vh]
                self._ln_lin.set_data(tv, [l for (_t, l, _a) in vh])
                self._ln_ang.set_data(tv, [a for (_t, _l, a) in vh])
                self._ax_vel.relim(); self._ax_vel.autoscale_view(scalex=False)
            if dh:
                td = [t - now for (t, _d) in dh]
                self._ln_dist.set_data(td, [d for (_t, d) in dh])
                self._ax_dist.relim(); self._ax_dist.autoscale_view(scalex=False)
            try:
                self._mpl_canvas.draw_idle()
            except Exception:
                pass
        self.root.after(500, self._update_graphs)

    def _manual_reset(self):
        """Bouton 'Reset manuel' : force la remise à (0,0,0) sans balise."""
        with self._lock:
            self.origin = None        # -> prochaine odométrie devient l'origine
            self.traj = []
        self._log("Reset MANUEL des coordonnées (0,0,0)")

    def _safe_stop(self):
        """Arrêt moteur GARANTI : plusieurs Twist nuls (sécurité)."""
        for _ in range(5):
            self._publish(0.0, 0.0)
            time.sleep(0.02)

    # ── Télémétrie carte électronique (batterie, moteurs, consigne, voyants) ──
    def _draw_telemetry(self):
        import time
        c = self.telem
        c.delete("all")
        now = time.time()

        def live(key, t=1.5):
            return (now - self._t.get(key, 0.0)) < t

        # --- 1) Batterie ---
        c.create_text(12, 12, anchor="w", text="BATTERIE", fill="#5aa9ff",
                      font=("Helvetica", 9, "bold"))
        if self.batt_v is not None and live("batt", 3.0):
            v = self.batt_v
            pct = max(0, min(100, (v - 10.0) / 2.6 * 100))   # 3S : 10.0–12.6 V
            col = "#19a319" if pct > 50 else "#cc8800" if pct > 20 else "#e23b3b"
            c.create_text(12, 38, anchor="w", text=f"{v:4.1f} V", fill="#dfe9f7",
                          font=("Helvetica", 18, "bold"))
            c.create_text(110, 40, anchor="w", text=f"{pct:.0f}%", fill=col,
                          font=("Helvetica", 12, "bold"))
            c.create_rectangle(12, 58, 172, 78, outline="#33424f")
            c.create_rectangle(12, 58, 12 + 160 * pct / 100, 78, outline="", fill=col)
        else:
            c.create_text(12, 40, anchor="w", text="—", fill="#5b6b82",
                          font=("Helvetica", 14))

        # --- 2) Moteurs / roues (sorties carte) ---
        c.create_text(210, 12, anchor="w", text="MOTEURS (roues)", fill="#5aa9ff",
                      font=("Helvetica", 9, "bold"))
        names = ["FL", "RL", "FR", "RR"]          # ordre firmware WheelStates
        vel = self.wheels.get("vel", [0]*4)
        pwm = self.wheels.get("pwm", [0]*4)
        wlive = live("wheels", 2.0)
        for i in range(4):
            x = 210 + i * 80
            v = vel[i] if i < len(vel) else 0.0
            p = pwm[i] if i < len(pwm) else 0.0
            active = wlive and (abs(v) > 0.05 or abs(p) > 0.02)
            col = "#19a319" if active else "#3a4658"
            c.create_text(x + 24, 30, text=names[i], fill="#dfe9f7",
                          font=("Helvetica", 10, "bold"))
            # barre PWM (sortie carte) verticale, centrée
            base = 92
            h = max(-40, min(40, p * 40))
            c.create_rectangle(x + 14, base - 40, x + 34, base + 40,
                               outline="#33424f")
            if h >= 0:
                c.create_rectangle(x + 14, base - h, x + 34, base, outline="", fill=col)
            else:
                c.create_rectangle(x + 14, base, x + 34, base - h, outline="", fill=col)
            c.create_text(x + 24, base + 50, text=f"{v:+.1f}", fill="#9fb3c8",
                          font=("Helvetica", 8))

        # --- 3) Consigne /cmd_vel ---
        c.create_text(560, 12, anchor="w", text="/cmd_vel (consigne)",
                      fill="#5aa9ff", font=("Helvetica", 9, "bold"))
        lin, ang = self.last_cmd
        c.create_text(560, 40, anchor="w", text=f"v = {lin:+.2f} m/s",
                      fill="#dfe9f7", font=("Helvetica", 12, "bold"))
        c.create_text(560, 62, anchor="w", text=f"ω = {ang:+.2f} rad/s",
                      fill="#dfe9f7", font=("Helvetica", 12, "bold"))

        # --- 4) Santé capteurs (OK / FAIL) ---
        c.create_text(560, 92, anchor="w", text="SANTÉ CAPTEURS", fill="#5aa9ff",
                      font=("Helvetica", 9, "bold"))
        items = [("Caméra", "cam"), ("Odométrie", "odom"),
                 ("Batterie", "batt"), ("Moteurs", "wheels")]
        for i, (label, key) in enumerate(items):
            x = 560 + (i % 2) * 110
            y = 110 + (i // 2) * 20
            on = live(key)
            c.create_oval(x, y, x + 12, y + 12,
                          fill="#19a319" if on else "#e23b3b", outline="")
            c.create_text(x + 18, y + 6, anchor="w",
                          text=f"{label}: {'OK' if on else 'FAIL'}",
                          fill="#9fb3c8" if on else "#e23b3b",
                          font=("Helvetica", 9, "bold" if not on else "normal"))

    def _drive(self, cam_alive):
        if not self.connected or self.cmd_pub is None:
            return
        import time
        now = time.time()
        lin = ang = 0.0

        if not cam_alive and self.mode in ("AUTO", "CIBLER", "INFINI"):
            # Sécurité : pas d'image -> on ne bouge pas en automatique
            self._publish(0, 0)
            return

        if self.mode == "MANUEL":
            lin, ang = self.manual

        elif self.mode == "CIBLER":
            det = self.det
            c = self._target_center(det)
            if c is not None and self.last_w:
                err = c[0] - self.last_w / 2.0
                ang = max(-0.6, min(0.6, -TARGET_KP * err))
                d = det.get("dist")
                lin = 0.0 if (d is not None and d < TARGET_STOP_M) else 0.15
            else:
                lin, ang = 0.0, 0.0      # balise perdue -> on attend

        elif self.mode == "INFINI":
            # Cibler une INFINITÉ de balises :
            #   SEEK : si balise visible -> on la cible (centre + approche),
            #          sinon on tourne pour chercher. Quand une balise est
            #          VALIDÉE (trouvée) -> RECUL puis demi-tour, puis on repart.
            if self.inf_state == "RECUL":
                if now - self.inf_t < RECUL_T:
                    lin = -RECUL_SPEED
                else:
                    self.inf_state = "TURN180"
                    self.turn_accum = 0.0
                    self.turn_last_yaw = self.raw_yaw

            elif self.inf_state == "TURN180":
                ang = TURN_SPEED
                if self.raw_yaw is not None:
                    if self.turn_last_yaw is None:
                        self.turn_last_yaw = self.raw_yaw
                    d = math.atan2(math.sin(self.raw_yaw - self.turn_last_yaw),
                                   math.cos(self.raw_yaw - self.turn_last_yaw))
                    self.turn_accum += abs(d)
                    self.turn_last_yaw = self.raw_yaw
                if self.turn_accum >= math.pi:            # 180° atteint
                    self.inf_state = "SEEK"
                    self._inf_last_count = self.beacon_count
                    self._log("demi-tour terminé -> recherche d'une autre balise")

            else:  # SEEK
                if self.beacon_count > self._inf_last_count:
                    # une nouvelle balise vient d'être trouvée -> recul + 180°
                    self._inf_last_count = self.beacon_count
                    self.inf_state = "RECUL"
                    self.inf_t = now
                    self._log("balise trouvée -> recul + demi-tour")
                else:
                    det = self.det
                    c = self._target_center(det)
                    if c is not None and self.last_w:
                        err = c[0] - self.last_w / 2.0
                        ang = max(-0.6, min(0.6, -TARGET_KP * err))
                        d = det.get("dist")
                        lin = 0.0 if (d is not None and d < TARGET_STOP_M) else 0.15
                    else:
                        ang = SEARCH_ANG        # pas de balise -> on tourne

        elif self.mode == "AUTO":
            # La VALIDATION de balise (croix+tag) + le reset sont gérés par le
            # thread vision (_validate_beacon). Ici on se contente de la mission
            # de recherche, qui continue sans interruption après chaque reset.
            if self.search_state == "SCAN":
                ang = SEARCH_ANG
                if self.raw_yaw is not None:
                    if self.scan_last_yaw is None:
                        self.scan_last_yaw = self.raw_yaw
                    d = math.atan2(math.sin(self.raw_yaw - self.scan_last_yaw),
                                   math.cos(self.raw_yaw - self.scan_last_yaw))
                    self.scan_accum += abs(d)
                    self.scan_last_yaw = self.raw_yaw
                if self.scan_accum >= 2 * math.pi:
                    self.search_state = "ADVANCE"
                    self.adv_start = now
            elif self.search_state == "ADVANCE":
                if now - self.adv_start < ADVANCE_T:
                    lin = ADVANCE_LIN
                else:
                    self._start_scan()

        self._publish(lin, ang)

    def _target_center(self, det):
        """Centre de la cible, avec PERSISTANCE : si la balise vient d'être perdue
        (détection à 10 Hz), on garde la dernière position connue un court instant
        -> visée stable, pas de à-coups entre deux détections."""
        c = det.get("center")
        now = time.time()
        if c is not None:
            self._last_center = c
            self._last_center_t = now
            return c
        if self._last_center is not None and \
                (now - self._last_center_t) < CENTER_HOLD_S:
            return self._last_center
        return None

    def _start_scan(self):
        self.search_state = "SCAN"
        self.scan_accum = 0.0
        self.scan_last_yaw = self.raw_yaw

    def _publish(self, lin, ang):
        self.last_cmd = (float(lin), float(ang))   # pour l'affichage télémétrie
        try:
            t = self._Twist()
            t.linear.x = float(lin)
            t.angular.z = float(ang)
            self.cmd_pub.publish(t)
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════════════ #
    # Rendu image + carte
    # ══════════════════════════════════════════════════════════════════════ #
    def _draw_overlay(self, view, det):
        if view.ndim == 2:
            view = cv2.cvtColor(view, cv2.COLOR_GRAY2BGR)
        else:
            view = view.copy()
        self.last_w = view.shape[1]
        # LED candidates (toutes) en cyan pâle, LED RETENUES (près du damier) en vert vif
        for (x, y, _a) in det.get("leds", []):
            cv2.circle(view, (int(x), int(y)), 5, (120, 150, 0), 1)
        for (x, y, _a) in det.get("near", []):
            cv2.circle(view, (int(x), int(y)), 7, (60, 255, 138), 2)
        # DAMIER
        if det.get("checker") is not None:
            size, corners = det["checker"]
            try:
                cv2.drawChessboardCorners(view, size, corners, True)
            except Exception:
                pass
        # croix de visée sur le centre de la balise (cible)
        cc = det.get("center")
        if cc is not None:
            cv2.drawMarker(view, (int(cc[0]), int(cc[1])), (0, 255, 0),
                           cv2.MARKER_CROSS, 28, 2)
            txt = "BALISE"
            if det.get("dist") is not None:
                txt += f" {det['dist']:.2f} m"
            cv2.putText(view, txt, (int(cc[0]) - 44, int(cc[1]) - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if det.get("tag") is not None:
            quad = det["tag"][2].astype(int)
            cv2.polylines(view, [quad], True, (255, 200, 0), 2)
        return view

    def _show_image(self, bgr):
        if not _HAVE_PIL:
            self.cam_label.config(
                text="Installez Pillow pour la vue caméra :\n"
                     "pip3 install pillow")
            return
        h, w = bgr.shape[:2]
        scale = min(520 / w, 400 / h, 1.0)
        if scale < 1.0:
            bgr = cv2.resize(bgr, (int(w * scale), int(h * scale)))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self._photo = ImageTk.PhotoImage(PILImage.fromarray(rgb))
        self.cam_label.config(image=self._photo)

    def _update_det_label(self, det, cam_alive):
        if not cam_alive:
            self.lbl_det.config(text="⚠ pas d'image", foreground="red")
            return
        nall = len(det.get("leds", []))
        cluster = det.get("cluster") or []
        checker = det.get("checker") is not None
        d = det.get("dist")
        dt = f"{d:.2f} m" if d is not None else "?"
        if checker and len(cluster) >= self._minled:
            self.lbl_det.config(
                text=f"BALISE VALIDE — damier + {len(cluster)} LED — {dt} ✓",
                foreground="#19a319")
        elif checker:
            self.lbl_det.config(
                text=f"DAMIER vu — {len(cluster)}/{self._minled} LED autour",
                foreground="#cc8800")
        elif cluster:
            self.lbl_det.config(text=f"{len(cluster)} LED groupées — pas de damier",
                                foreground="#cc8800")
        else:
            self.lbl_det.config(text=f"{nall} LED détectées — pas de balise",
                                foreground="#8aa0b6")

    def _clear_map(self):
        self.traj = []
        self.beacons = []
        self.origin = None
        self.beacon_count = 0
        self._beacon_prev = False

    def _draw_map(self):
        cv = self.canvas
        cv.delete("all")
        W, H = self.map_w, self.map_h
        cx0, cy0 = W / 2, H / 2
        scale = 55.0   # pixels par mètre

        def to_px(wx, wy):
            return cx0 + wx * scale, cy0 - wy * scale

        # grille + axes
        for gx in range(-4, 5):
            x, _ = to_px(gx, 0)
            cv.create_line(x, 0, x, H, fill="#1e2630")
        for gy in range(-4, 5):
            _, y = to_px(0, gy)
            cv.create_line(0, y, W, y, fill="#1e2630")
        cv.create_line(0, cy0, W, cy0, fill="#33424f")
        cv.create_line(cx0, 0, cx0, H, fill="#33424f")
        cv.create_text(W - 28, cy0 - 10, text="x", fill="#5b6b7a")
        cv.create_text(cx0 + 12, 12, text="y", fill="#5b6b7a")

        # trajectoire
        if len(self.traj) > 1:
            pts = []
            for (wx, wy) in self.traj:
                px, py = to_px(wx, wy)
                pts.extend([px, py])
            cv.create_line(*pts, fill="#6fd0ff", width=2)

        # balises
        for (wx, wy, label) in self.beacons:
            px, py = to_px(wx, wy)
            cv.create_oval(px-7, py-7, px+7, py+7, fill="#22dd55", outline="white")
            cv.create_text(px, py-14, text=label, fill="white",
                           font=("TkDefaultFont", 8))

        # robot (flèche)
        rx, ry, ryaw = self.pose
        px, py = to_px(rx, ry)
        ex, ey = px + 18 * math.cos(ryaw), py - 18 * math.sin(ryaw)
        cv.create_oval(px-6, py-6, px+6, py+6, fill="#ffffff", outline="#88aacc")
        cv.create_line(px, py, ex, ey, fill="#ffffff", width=3, arrow="last")

        # compteur PERSISTANT (s'incrémente à chaque balise trouvée)
        cv.create_text(8, H-10, anchor="w", fill="#7fd",
                       text=f"mode: {self.mode}   balises trouvées: {self.beacon_count}")

    # ══════════════════════════════════════════════════════════════════════ #
    # Boutons
    # ══════════════════════════════════════════════════════════════════════ #
    def toggle_mode(self):
        order = {"MANUEL": "AUTO", "AUTO": "MANUEL", "CIBLER": "MANUEL",
                 "INFINI": "MANUEL"}
        self.mode = order.get(self.mode, "MANUEL")
        if self.mode == "AUTO":
            self._start_scan()
        self.manual = [0.0, 0.0]
        self.btn_mode.config(text=f"Mode : {self.mode}")
        self.stop()

    def target_beacon(self):
        self.mode = "CIBLER"
        self.btn_mode.config(text="Mode : CIBLER")

    def infinite_beacons(self):
        """Mode 'Cibler ∞ balises' : enchaîne balise -> recul + 180° -> suivante."""
        self.mode = "INFINI"
        self.inf_state = "SEEK"
        self._inf_last_count = self.beacon_count   # ne pas déclencher sur du périmé
        self.manual = [0.0, 0.0]
        self.btn_mode.config(text="Mode : INFINI")
        self._log("Mode CIBLER ∞ BALISES activé")

    def stop(self):
        self.mode = "MANUEL"
        self.manual = [0.0, 0.0]
        self._safe_stop()
        self._log("STOP")

    def _on_close(self):
        self._closing = True             # arrête proprement le thread vision
        try:
            if self.connected:
                self._safe_stop()        # arrêt moteur GARANTI
        except Exception:
            pass
        self._stop_camera_robot()        # coupe la caméra du robot proprement
        if self.node_inited:
            try:
                self._rospy.signal_shutdown("fermeture GUI")
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    Dashboard(root)
    root.mainloop()


if __name__ == "__main__":
    main()

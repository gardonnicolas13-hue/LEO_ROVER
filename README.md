# LEO Rover Mission Control — Full-Stack Autonomous Ground Vehicle Navigation System

Système complet de navigation autonome pour rover terrestre, développé au
Florida Institute of Technology (campagne juin–juillet 2026). Le dépôt couvre
la chaîne entière : nœuds ROS embarqués, estimation d'état multicapteur,
machine à états autonome, cockpit opérateur temps réel, et les sources LaTeX
du rapport final.

---

## Architecture

**Deux machines, et elles ne sont pas équivalentes.** Le robot porte un
Raspberry Pi 4 : il possède les capteurs, les moteurs, et il héberge le *ROS
master*. Le poste de travail exécute les estimateurs et sert le cockpit. Cette
asymétrie est la source de la majorité des confusions initiales ; elle est
détaillée au début de l'annexe D du rapport.

- **Backend multiprocessus** (`leo_backend.py`, ~5 300 lignes) : boucle de
  contrôle à 20 Hz, télémétrie à 10 Hz et détection visuelle en sous-processus
  dédié, état partagé sous verrou unique tenu uniquement pendant les lectures
  et écritures, jamais pendant les E/S ou les opérations OpenCV.
- **Estimation d'état** : MINS (caméra + IMU + odométrie roues) et openVINS
  `ov_msckf` (MSCKF, caméra + IMU), sélectionnables à chaud via
  `pose_selector`, avec garde de plausibilité et arbre TF strictement
  linéaire `odom → base_footprint → base_link`.
- **Machine à états autonome** : patrouille, évitement d'obstacle, approche et
  verrouillage de balise, retour à la base, avec six couches de sécurité
  indépendantes (homme-mort, garde profondeur, plafonds de vitesse, arrêt sur
  perte de balise).
- **Cockpit opérateur** : quatre pages connectées en direct via WebSocket
  `roslibjs` (`ops.html`, `trajectory.html`, `pid.html`, `demo.html`), plus
  plusieurs pages statiques annexes (accueil, journal, modes de navigation,
  description du robot), esthétique *glassmorphic*, double transport
  (WebSocket direct en LAN, WSS via tunnel TLS pour l'accès public).
  `ops.html` pilote, `demo.html` est en lecture seule par construction
  (aucun publisher, aucune liaison de commande) pour les démonstrations.
- **Auto-guérison** : watchdog cron surveillant les ports, la caméra, la
  liaison série et la divergence de l'estimateur, avec sondes de handshake
  WebSocket réelles plutôt que de simples tests de port.

---

## Structure du dépôt

```
.
├── leo_backend.py              Backend principal : FSM, télémétrie, vision, contrôle
├── start_leo.sh                Séquence de démarrage du poste de travail
├── requirements.txt            Dépendances Python (voir aussi python3-tk via apt)
├── JOURNAL_DE_BORD.md          Journal de bord versionné de la campagne
│
├── catkin_ws/src/
│   ├── leo_navigation/         Nœuds ROS du projet : pose_selector, imu_sanitizer,
│   │                           wheel_remap, carolus_tf_bridge, fichiers .launch
│   └── leo_autonomy/           Paquet SLAM / exploration ; seul son détecteur
│                                AprilTag (acquisition balise) tourne en
│                                production, le reste est construit mais
│                                non lancé
│
├── web/                        Cockpit opérateur (front-end statique)
│   ├── ops.html                Contrôle : pilotage, AUTO, source de pose, export
│   ├── trajectory.html         Carte temps réel, rendu Canvas par lots
│   ├── logbook.html            Journal, alimenté par auto_entries.json
│   ├── demo.html               Vue de soutenance, lecture seule stricte
│   ├── app.js · i18n.js        Contrôleur et traductions partagés
│   └── serve.py                Serveur statique avec règles de cache
│
├── tools/                      Scripts utilitaires
│   ├── leo_watchdog.sh         Watchdog cron auto-guérisseur
│   ├── restart_stack.sh        Relance de la pile de navigation
│   ├── record_trajectories.sh  Capture rosbag robuste (chemin recommandé)
│   ├── plot_trajectories.m     Comparaison MATLAB à neuf panneaux
│   └── update_report.sh        Compilation et publication du rapport
│
├── data/trajectories/          Trajectoires exportées (CSV, PNG)
│
└── report_latex/               Sources LaTeX + **main.pdf** (rapport compilé)
```

**Non versionné et pourquoi :** les paquets amont (openVINS, MINS, Kalibr,
11 Go) se clonent depuis leurs dépôts d'origine ; les enregistrements
`.bag` bruts (760 Mo) dépassent ce qu'un dépôt git doit porter. Le
`.gitignore` fonctionne par liste d'inclusion et documente chaque décision.

---

## 📘 Le rapport

**Le document de référence de ce projet est [`report_latex/main.pdf`](report_latex/main.pdf), 412 pages.**

Le code seul ne se suffit pas. Le rapport contient :

- **La théorie mathématique** : modèle sténopé et propagation d'erreur,
  alignement rigide contre similitude (Kabsch et Umeyama, dérivation SVD
  complète), algèbre du sous-espace de jauge démontrant ce qu'un recalage
  a posteriori absorbe, modèle d'échéance de la FIFO UART, observabilité des
  mises à jour à vitesse nulle.
- **L'architecture réseau** : double transport du cockpit, tunnel Cloudflare
  avec ses règles d'ingress, WireGuard et la question de la topologie
  `Endpoint`.
- **L'annexe D, guide de reproduction exhaustif** : reconstruction depuis une
  machine vierge, commande par commande, avec la sortie attendue à chaque
  étape. Elle s'ouvre sur un index des sept pannes qui ont coûté au moins une
  journée chacune, et dont le symptôme désigne systématiquement le mauvais
  sous-système.

Les limites connues y sont documentées aussi franchement que les résultats,
notamment la calibration des intrinsèques couleur qui reste **bloquante**
avant toute publication de métrique dimensionnelle.

---

## Démarrage rapide

```bash
pip install -r requirements.txt
sudo apt install python3-tk

source tools/robot_env.sh
touch /tmp/leo_maintenance      # suspend le watchdog pendant le démarrage
bash web/start_web.sh
bash tools/restart_stack.sh
rm -f /tmp/leo_maintenance      # NE PAS OUBLIER

# cockpit : http://localhost:8000/ops.html
```

La procédure complète, avec les vérifications intermédiaires, se trouve à
l'annexe D du rapport.

---

## Environnement

ROS Noetic · Ubuntu 20.04 · Python 3.8 · Raspberry Pi 4 (robot) ·
Intel RealSense D455 · construit avec `catkin_tools` (`catkin build`,
**pas** `catkin_make`).

---

*Nicolas Gardon — Florida Institute of Technology, été 2026.*

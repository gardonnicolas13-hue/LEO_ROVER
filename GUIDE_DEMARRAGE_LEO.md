# Guide de démarrage — LEO Rover avec détection de balises
## Pour débutants complets : étape par étape, pas à pas

---

## Schéma des deux machines

```
┌─────────────────────────────┐        Wi-Fi Leo        ┌──────────────────────────┐
│  PC (Ubuntu)                │ ◄─────────────────────► │  ROBOT LEO               │
│  Utilisateur : lab272       │                         │  Utilisateur : pi        │
│  IP : (variable)            │                         │  IP : 10.0.0.1           │
│                             │                         │  Mot de passe : voir opérateur│
└─────────────────────────────┘                         └──────────────────────────┘
```

**RÈGLE D'OR** : Il y a TOUJOURS deux fenêtres de terminal ouvertes :
- **Terminal A** = votre PC (Ubuntu, utilisateur lab272)
- **Terminal B** = le robot via SSH (utilisateur pi sur 10.0.0.1)

Ne jamais mélanger les deux. Quand vous doutez, regardez le début de la ligne :
- `lab272@OptiPlex:~$` → vous êtes **sur le PC**
- `pi@leo:~$` → vous êtes **sur le robot**

---V

## PARTIE 1 — Préparation (à faire une seule fois)

### Étape 1.1 — Connecter le PC au réseau Wi-Fi du robot

Le robot LEO crée son propre réseau Wi-Fi.

1. Allumez le robot (bouton sur le dessus).
2. Attendez 30 secondes que le robot démarre complètement.
3. Sur le PC, cliquez sur l'icône réseau en haut à droite.
4. Connectez-vous au réseau Wi-Fi du robot (il s'appelle quelque chose comme `LeoRover-XXXX`).
5. Attendez que la connexion soit établie.

**Vérification** : Ouvrez **Terminal A** (sur le PC) et tapez :
```bash
ping 10.0.0.1
```
Vous devez voir des lignes comme `64 bytes from 10.0.0.1 ...`.
Pour arrêter le ping : appuyez sur `Ctrl + C`.

Si le ping ne répond pas → le robot n'est pas allumé, ou vous n'êtes pas sur le bon réseau Wi-Fi.

---

### Étape 1.2 — Envoyer le programme sur le robot (une seule fois, ou après modification)

> ⚠️ **Cette commande se tape dans Terminal A (le PC), PAS dans le robot.**

Dans **Terminal A** (PC) :
```bash
scp /home/lab272/TOUT/leo_tracking_map.py pi@10.0.0.1:/home/pi/
```

On vous demande un mot de passe : tapez le mot de passe SSH du robot (demandez-le à l'opérateur si vous ne l'avez pas) puis appuyez sur **Entrée**.

(Le mot de passe ne s'affiche pas pendant que vous tapez, c'est normal.)

**Ce que ça fait** : copie le fichier `leo_tracking_map.py` depuis votre PC vers le dossier `/home/pi/` du robot.

**Confirmation de succès** : la commande affiche quelque chose comme :
```
leo_tracking_map.py    100%   18KB   1.2MB/s   00:00
```

---

## PARTIE 2 — Connexion SSH au robot

### Étape 2.1 — Ouvrir une connexion SSH

Dans **Terminal B** (ou une nouvelle fenêtre de terminal sur le PC) :
```bash
ssh pi@10.0.0.1
```

On vous demande le mot de passe : tapez le mot de passe SSH du robot puis **Entrée**.

La première fois, il peut demander :
```
Are you sure you want to continue connecting (yes/no)?
```
Tapez `yes` puis **Entrée**.

**Confirmation** : le début de la ligne change et affiche :
```
pi@leo:~ $
```
Vous êtes maintenant dans le terminal du robot.

---

## PARTIE 3 — Démarrage de ROS sur le robot

> ⚠️ Toutes les commandes de cette partie se font dans **Terminal B (robot)**.

### Étape 3.1 — Vérifier que ROS est accessible

```bash
source /opt/ros/noetic/setup.bash
echo $ROS_DISTRO
```

Vous devez voir : `noetic`

### Étape 3.2 — Vérifier que le robot (roscore) tourne déjà

```bash
rostopic list
```

Si vous voyez une liste de topics (lignes commençant par `/`), ROS tourne déjà. **Passez à la Partie 4.**

Si vous voyez une erreur `Unable to communicate with master` → ROS n'est pas lancé.
Dans ce cas, ouvrez un **troisième terminal** (Terminal C), connectez-vous en SSH :
```bash
ssh pi@10.0.0.1
```
Puis dans Terminal C :
```bash
source /opt/ros/noetic/setup.bash
roscore
```
Laissez ce terminal ouvert (roscore tourne en permanence).

### Étape 3.3 — Vérifier que la caméra RealSense est active

Dans **Terminal B** :
```bash
rostopic hz /camera/color/image_raw
```

Vous devez voir des lignes comme :
```
average rate: 28.000
```

Si vous voyez `WARNING: topic [/camera/color/image_raw] does not appear to be published yet` pendant plus de 10 secondes → la caméra RealSense n'est pas lancée.

**Lancer la caméra RealSense** (dans Terminal B) :
```bash
source /opt/ros/noetic/setup.bash
roslaunch realsense2_camera rs_camera.launch
```
Laissez ce terminal ouvert. Ouvrez un **nouveau Terminal B2** (SSH) pour la suite.

Appuyez sur `Ctrl + C` pour arrêter le `rostopic hz` quand vous avez vérifié.

### Étape 3.4 — Vérifier que l'odométrie du robot est publiée

Dans **Terminal B** :
```bash
rostopic echo /firmware/wheel_odom --once
```

Vous devez voir des données avec `pose_x`, `pose_y`, `pose_yaw`.

Si erreur → le firmware du robot n'est pas lancé. Vérifiez que `leo_bringup` tourne :
```bash
systemctl status leo
```

---

## PARTIE 4 — Lancer le programme de détection de balises

> ⚠️ Dans **Terminal B** (robot, connecté en SSH).

### Étape 4.1 — Lancer le programme

```bash
source /opt/ros/noetic/setup.bash
python3 /home/pi/leo_tracking_map.py
```

### Étape 4.2 — Lire les messages de démarrage (auto-test)

Le programme effectue d'abord un **auto-test des composants**. Vous verrez quelque chose comme :

```
########## AUTO-TEST DES COMPOSANTS ##########
[ OK ]  cv2.aruco (AprilTag) : familles: APRILTAG_36H11, APRILTAG_36H10, ...
[ OK ]  Caméra (/camera/color/image_raw) : 640x480 encoding=rgb8
[ OK ]  Conversion image : BGR 640x480
[ OK ]  Détection (image test) : aucun tag dans le champ ; 0 LED(s) détectée(s)
[ OK ]  leo_msgs/WheelOdom (type odom) : module importé
[ OK ]  Odométrie (/firmware/wheel_odom) : pose=(0.00, 0.00, 0°)
[ OK ]  /cmd_vel (moteurs) : firmware abonné
#############################################
[Map] RECHERCHE : tour 360° pour trouver une balise...
```

> **Note importante** : la « balise » n'est **PAS un QR code** mais un **AprilTag**
> (AR tag) imprimé en haut du panneau noir + des **LED bleues/cyan** en bas. Le
> programme lit l'AprilTag avec `cv2.aruco` (intégré à OpenCV, déjà sur le Pi) —
> plus besoin de pyzbar. L'**ID du tag** (un nombre) sert d'étiquette à la balise.

**Si un test FAIL (critique)**, le programme s'arrête et affiche `DÉMARRAGE ANNULÉ`.
Les `WARN` ne bloquent pas. Voir la section "Problèmes courants" en bas.

### Étape 4.3 — Observer le comportement du robot

Une fois démarré, le robot :
1. **Tourne sur lui-même** (360°) lentement pour chercher une balise.
2. Si rien trouvé → **avance 1 seconde** puis refait un tour.
3. Si une **balise est vue** (≥2 LED bleues groupées + AprilTag au-dessus) **ET située entre 0,5 m et 4 m** :
   - Le programme affiche : `[Map] BALISE vue #1 'tag 7' à 1.20 m -> reset origine`
   - Le robot réinitialise sa position à (0,0,0)
   - Un point est marqué sur la carte RViz, étiqueté avec l'ID du tag
   - **Hors de la plage 0,5–4 m**, la balise est **ignorée** (message : `balise vue (dist=...) hors plage [0.5, 4.0] m -> ignorée`).

> **Plage de lecture 0,5 m – 4 m** : la distance est estimée à partir de l'écart
> entre les LED gauche et droite de la balise (paramètre `BEACON_WIDTH_M`, par
> défaut **0,165 m** = écart des LED Carolus). **Vérifiez cet écart sur votre
> balise** et ajustez `BEACON_WIDTH_M` en haut du programme si besoin, sinon la
> distance estimée (et donc la plage) sera faussée. L'auto-test affiche la
> distance estimée quand la balise est dans le champ — pratique pour calibrer.

**Pour arrêter le programme** : appuyez sur `Ctrl + C` dans le terminal.
Le robot s'arrête automatiquement (vitesse remise à zéro).

---

## PARTIE 5 — Visualisation avec RViz (optionnel, sur le PC)

> ⚠️ Dans **Terminal A** (PC).

### Étape 5.1 — Configurer ROS pour voir le robot depuis le PC

```bash
export ROS_MASTER_URI=http://10.0.0.1:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
echo "Mon IP : $ROS_IP"
```

Vérifiez que `$ROS_IP` affiche bien une IP sur le réseau du robot (exemple : `192.168.50.xx`).

### Étape 5.2 — Lancer RViz

```bash
source /opt/ros/noetic/setup.bash
rosrun rviz rviz
```

### Étape 5.3 — Configurer RViz

Dans la fenêtre RViz :

1. **Fixed Frame** : en haut à gauche, changer `map` (normalement déjà bon).
2. Cliquer sur **"Add"** en bas à gauche.
3. Ajouter **"Path"** → choisir topic `/tracking/path` → cliquer OK.
4. Cliquer sur **"Add"** à nouveau.
5. Ajouter **"MarkerArray"** → choisir topic `/tracking/markers` → cliquer OK.

Vous verrez maintenant :
- Une **ligne blanche** = trajectoire du robot depuis la dernière balise.
- Une **flèche blanche** = position et orientation actuelle du robot.
- Des **sphères vertes** = balises détectées (avec leur étiquette QR).

---

## RÉCAPITULATIF RAPIDE (usage quotidien)

Une fois la configuration initiale faite, voici les étapes du quotidien :

| # | Où ? | Commande |
|---|------|----------|
| 1 | PC | Se connecter au Wi-Fi du robot |
| 2 | Terminal A (PC) | `scp /home/lab272/TOUT/leo_tracking_map.py pi@10.0.0.1:/home/pi/` (seulement si le fichier a changé) |
| 3 | Terminal B (robot SSH) | `ssh pi@10.0.0.1` |
| 4 | Terminal B | `rostopic list` (vérifier que ROS tourne) |
| 5 | Terminal B | `source /opt/ros/noetic/setup.bash && python3 /home/pi/leo_tracking_map.py` |
| 6 | `Ctrl+C` | Arrêter le programme proprement |

---

## PROBLÈMES COURANTS ET SOLUTIONS

### "Permission denied (publickey, password)"
→ Le mot de passe est incorrect. Redemandez-le à l'opérateur (il n'est plus documenté ici depuis l'audit sécurité du 2026-08-04, ce dépôt étant public).
→ Vérifiez que vous tapez `pi@10.0.0.1` (pas `lab272@10.0.0.1`)

### "No route to host" ou "Connection refused"
→ Vous n'êtes pas connecté au Wi-Fi du robot.
→ Le robot n'est pas allumé, attendez 30s après l'allumage.

### "Unable to communicate with master"
→ `roscore` ne tourne pas. Lancez-le (voir Étape 3.2).

### Auto-test échoue sur `/camera/color/image_raw`
→ La caméra RealSense D455 n'est pas lancée.
→ Dans un terminal SSH séparé : `source /opt/ros/noetic/setup.bash && roslaunch realsense2_camera rs_camera.launch`

### Auto-test échoue sur `/firmware/wheel_odom`
→ Le firmware du robot n'est pas prêt. Vérifiez : `systemctl status leo`
→ Si inactif : `sudo systemctl start leo`

### Le robot tourne mais ne détecte pas la balise
→ La balise doit avoir ses **LED bleues/cyan allumées** (≥ 2 visibles, groupées) ET un **AprilTag** (PAS un QR code) sur le panneau au-dessus.
→ **Réglage des LED** : en haut de `leo_tracking_map.py`, les paramètres viennent du système Carolus (teinte `LED_HUE_LOW=80`..`LED_HUE_HIGH=135`, luminosité `LED_V_MIN=200`). Si les LED ne sont pas détectées : baissez `LED_V_MIN` (ex. 170) ou élargissez la teinte. Si trop de fausses détections : remontez `LED_V_MIN` ou resserrez la teinte vers 90-110.
→ Le nombre de LED détectées s'affiche dans l'auto-test (`Détection (image test) : ... N LED(s)`). Pointez la caméra sur la balise allumée et relancez pour calibrer.
→ **AprilTag** : il doit y avoir une marge blanche autour du tag (zone de silence) et il doit être net à la distance actuelle. Si le tag n'est pas lu, la balise est quand même détectée par les LED, mais l'étiquette sera `balise ?` au lieu de `tag N`.
→ Si vos balises utilisent des marqueurs **ALVAR** (exemple LEO `follow_ar_tag`) et non des AprilTags imprimés, `cv2.aruco` ne les lira pas : il faudrait alors `ar_track_alvar`. Dites-le moi.

### Le robot ne tourne pas (vitesse = 0)
→ La caméra ne reçoit pas d'images. Vérifiez la RealSense.
→ Regardez les messages dans le terminal : `[Map] pas de flux caméra -> attente`

### "scp: no such file or directory"
→ Vérifiez que vous êtes bien dans **Terminal A (PC)** et non dans le robot.
→ Le fichier source : `/home/lab272/TOUT/leo_tracking_map.py` doit exister.
→ Vérifiez : `ls /home/lab272/TOUT/leo_tracking_map.py`

---

## ORGANISATION DES FICHIERS

```
Sur le PC (/home/lab272/TOUT/) :
  leo_tracking_map.py      ← programme principal (À MODIFIER ICI)
  leo_target_behavior.py   ← ancien programme (multi-cibles sans carte)
  qr_search_move.py        ← ancien programme (recherche simple)
  diag_qr.py               ← outil de diagnostic caméra/QR
  GUIDE_DEMARRAGE_LEO.md   ← ce guide

Sur le robot (/home/pi/) :
  leo_tracking_map.py      ← copie envoyée par scp (NE PAS MODIFIER ICI)
```

**Règle** : on modifie toujours les fichiers **sur le PC**, puis on les envoie sur le robot avec `scp`.

---

*Programme testé sur LEO Rover avec ROS 1 Noetic, caméra Intel RealSense D455, Ubuntu 20.04.*

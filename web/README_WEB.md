# LEO Rover — Interface Web Headless

Migration du dashboard Tkinter (`leo_dashboard.py`) vers une architecture
**Backend Python sans tête + Frontend HTML/JS** (Tailwind CSS).

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│ ROBOT LEO (10.0.0.1)                                           │
│   roscore · realsense2_camera · leo_bringup (odométrie)        │
└───────────────────────────────┬────────────────────────────────┘
                                 │ réseau ROS (Wi-Fi)
┌───────────────────────────────┴────────────────────────────────┐
│ PC (Ubuntu 20.04 · ROS Noetic)                                 │
│   leo_backend.py      → vision hybride + contrôle (headless)   │
│   rosbridge_server    → ws://PC:9090   (roslibjs)              │
│   web_video_server    → http://PC:8080 (flux MJPEG annoté)     │
│   http.server         → http://PC:8000 (pages HTML)            │
└───────────────────────────────┬────────────────────────────────┘
                                 │ HTTP / WebSocket
┌───────────────────────────────┴────────────────────────────────┐
│ TABLETTE / NAVIGATEUR                                          │
│   index.html (accueil) · ops.html (cockpit) · logbook.html     │
└────────────────────────────────────────────────────────────────┘
```

## Prérequis (une fois)

```bash
sudo apt install ros-noetic-rosbridge-server ros-noetic-web-video-server
# Python : numpy, opencv (déjà présents avec ROS Noetic)
```

## Lancement (tout-en-un)

```bash
cd /home/lab272/TOUT/web
./start_web.sh
```

Le script affiche l'URL à ouvrir, par ex. `http://192.168.x.x:8000/index.html`.
- Sur le **PC lui-même** : ouvrir `http://localhost:8000/index.html`.
- Sur une **tablette** (même Wi-Fi) : ouvrir `http://<IP-du-PC>:8000/index.html`.

Dans `ops.html`, le champ **IP** = l'hôte qui fait tourner rosbridge + la vidéo
(c.-à-d. le PC). Il est pré-rempli automatiquement avec l'hôte qui sert la page.

`Ctrl+C` dans le terminal arrête toute la pile.

## Lancement manuel (debug)

```bash
export ROS_MASTER_URI=http://10.0.0.1:11311
export ROS_IP=$(hostname -I | awk '{print $1}')

roslaunch rosbridge_server rosbridge_websocket.launch        # :9090
rosrun web_video_server web_video_server _port:=8080         # :8080
python3 /home/lab272/TOUT/leo_backend.py                     # vision+contrôle
cd /home/lab272/TOUT/web && python3 -m http.server 8000      # pages
```

## Contrat ROS (topics du backend)

| Topic | Type | Sens | Contenu |
|-------|------|------|---------|
| `/mission/telemetry` | `std_msgs/String` | backend → web | JSON état complet (~10 Hz) |
| `/mission/log` | `std_msgs/String` | backend → web | 1 ligne de log par évènement |
| `/mission/image_annotated` | `sensor_msgs/Image` | backend → web_video_server | flux BGR8 + overlay détection |
| `/mission/command` | `std_msgs/String` | web → backend | ordres JSON (voir ci-dessous) |
| `/cmd_vel` | `geometry_msgs/Twist` | backend → robot | consigne moteurs |

### Commandes JSON (`/mission/command`)

```json
{"action":"set_mode","mode":"AUTO"}      // ou "MANUEL"
{"action":"target_beacon"}               // mode CIBLER
{"action":"infinite_beacons"}            // mode INFINI (∞ balises)
{"action":"stop"}                        // arrêt + retour MANUEL
{"action":"reset"}                       // X/Y/Z -> 0,0,0
{"action":"clear_map"}                   // efface traj + balises + compteur
{"action":"set_params","hue_low":80,"hue_high":135,"v_min":200,"minled":3}
{"action":"manual","lin":0.2,"ang":0.0}  // pilotage (homme-mort 0.5 s)
{"action":"set_view","mask":true}        // flux normal / masque HSV
```

## Ce qui a été conservé du dashboard

- **Vision hybride** : damier (`findChessboardCornersSB` + cache de taille) comme
  ancre + 4 LED filtrées par ROI, distance estimée sur l'empan des LED.
- **Threads optimisés** : `_decode_loop` (rapide) + `_detect_loop` (lourd, isolé)
  → flux fluide sans gel.
- **Machine d'états** : MANUEL / AUTO / CIBLER / INFINI (SEEK→RECUL→TURN180→SEEK).
- **Validation balise** : damier + ≥ N LED, anti-rebond, compteur + reset logiciel
  (jamais d'appel bloquant avant l'incrément — correctif v2.3 préservé).

## Ce qui a été retiré

- **Tkinter** et **matplotlib** : remplacés par le frontend web (Chart.js).
- L'affichage local : remplacé par le flux MJPEG + la télémétrie JSON.

## Sécurité

- **Homme-mort** : en pilotage manuel, sans ordre reçu depuis 0,5 s, le robot
  s'arrête (utile si la tablette se déconnecte).
- **Pas d'image → pas de mouvement** en mode automatique.
- Bouton **ARRÊT D'URGENCE** (rouge) : 5× Twist nul + retour en MANUEL.

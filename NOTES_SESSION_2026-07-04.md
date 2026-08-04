# Notes de session — nuit du 3 au 4 juillet 2026

## Verdicts techniques (avec preuves)

1. **Caméra D455 : SAINE — dossier clos.**
   - Test isolé pyrealsense2 (zéro ROS) : 30/30 frames.
   - `d455_minimal.launch` (infra1+infra2 seuls, 640×480) : 30,0 Hz tenus, gigue 0,6 ms, mesuré sur le Pi.
   - Le coupable historique : la config complète (depth 848×480 + color 1280×720 + compression theora/JPEG du color) écrasait le CPU du Pi → throttling thermique (`vcgencmd get_throttled` = 0xe0000) et corruption du lien série rosserial.
   - Les warnings `errno 11 control_transfer index 768` sont **non fatals** (le flux continue à 30 Hz).

2. **IMU firmware : nouveau mode de panne identifié — le GEL.**
   - Après une rafale de désync rosserial, le firmware LeoCore répète la même mesure à ~30 Hz (valeurs bit-identiques, physiquement absurdes : |a|=13 m/s², gyro 134°/s robot immobile).
   - Détection : compter les valeurs distinctes sur `/firmware/imu` (1 seule = gelé).
   - Remède : `rosservice call /firmware/reset_board` (pas de power-cycle nécessaire).
   - **`imu_sanitizer.py` détecte désormais le gel automatiquement** (200 échantillons identiques) et déclenche lui-même le reset (throttlé à 1/min, désactivable par `~freeze_autoreset`).

3. **WiFi : les crashs de l'AP étaient auto-infligés.**
   - Lien mesuré ~6 Mo/s ; MINS + leo_backend tiraient ~36 Mo/s d'images brutes → saturation → crash du firmware WiFi du Pi (2 coupures dans la nuit).
   - Fix : hop WiFi en JPEG compressé via 2 nœuds `image_transport republish` (PC), qui re-servent en local sur `/pc/camera/infra{1,2}/image_rect_raw`. MINS et leo_backend pointent dessus. Résultat : MINS est passé de 2,9 Hz caméra à pleine cadence (730 features trackées, 50 MSCKF/update).
   - **Règle : ne JAMAIS s'abonner aux topics image bruts du robot depuis le PC. Mesurer les débits caméra sur le Pi.**

4. **Cockpit web : cause de la panne = handshake WebSocket rejeté.**
   - autobahn vérifie le port du header `Host` ; via le tunnel public il arrive sans port (=443) alors que le serveur écoute sur 9443 → rejet.
   - Fix : `_websocket_external_port:=443` sur le wrapper TLS 9443 (et PAS sur le 9090 LAN).

## Changements de configuration/code

| Quoi | Où |
|---|---|
| Nœuds republish JPEG→raw + respawn | `catkin_ws/src/leo_navigation/launch/navigation_supervision.launch` |
| Topics cam0/cam1 → `/pc/camera/...` | `MINS-master/mins/config/leo/config_camera.yaml` |
| `CAM_TOPIC_DEFAULT` → `/pc/camera/...` | `leo_backend.py` |
| Caméra minimale 640×480 (= calibration Kalibr) | `tools/d455_minimal.launch` (+ copie `~/d455_minimal.launch` sur le robot, lancée en nohup) |
| Garde anti-gel IMU + auto reset_board | `catkin_ws/src/leo_navigation/scripts/imu_sanitizer.py` |
| Refonte : plus de nœuds dupliqués, nohup partout, external_port corrigé, tunnel inclus | `web/start_web.sh` |
| Watchdog web+navigation (cron 1 min, flag `/tmp/leo_maintenance`) | `tools/leo_watchdog.sh` (cron à installer manuellement) |
| Autoconnect WiFi infini vers LeoRover-e138 | NetworkManager (nmcli) |
| Wrapper bash leo_backend sous supervision | `scripts/leo_backend_launch.sh` + `CMakeLists.txt` (install(), PAS catkin_install_python — shim Python ≠ bash) |

## Procédures à retenir

- **Démarrage pile web** : `web/start_web.sh` (tout en nohup, plus besoin de garder un terminal ouvert).
- **Démarrage stack navigation** : `catkin_ws/src/leo_navigation/launch_mins.sh navigation_master.launch`.
- **Avant toute manip volontaire** (si le cron watchdog est installé) : `touch /tmp/leo_maintenance`, puis `rm -f /tmp/leo_maintenance` à la fin.
- **Caméra sur le robot** : `nohup roslaunch ~/d455_minimal.launch > ~/d455_minimal.log 2>&1 &` (mode minimal infra-only — à pérenniser dans `/etc/ros/robot.launch` à terme).

## Ouvert / prochaine étape

- **T1.1 (dérive statique 5 min)** : campagne lancée cette nuit — voir résultats dans la conversation / `logs/t1_1_static_pose.csv`.
- Dérive Z de MINS (z≈-292 m observé avant redémarrage) : suspect n°1 = `T_imu_cam` encore identité placeholder (caméra « regarderait vers le haut ») → la correction attendra les chiffres T1.1, conformément à la consigne.
- Kalibr imu-camera (T_cam_imu réel) toujours à refaire (approche monoculaire cam0 recommandée).
- Pérenniser le mode caméra minimal dans `/etc/ros/robot.launch` (rosmon) sur le robot.
- `raspicam_node` et `rplidar_node` toujours CRASHED dans rosmon (non investigué).

#!/usr/bin/env bash
# =============================================================================
# leo_watchdog.sh — relance ce qui est mort (pile web + stack navigation).
# Lancé par cron toutes les minutes. Journal: TOUT/logs/watchdog.log
#
# /tmp/leo_maintenance : si ce fichier existe, le watchdog ne fait RIEN.
# À créer au début de toute opération volontaire (restart de stack, test,
# campagne de mesure) et à supprimer à la fin — sinon le watchdog relancera
# ce que vous venez d'arrêter, en plein milieu de votre manip.
#   touch /tmp/leo_maintenance    # début de manip
#   rm -f /tmp/leo_maintenance    # fin de manip
# =============================================================================
[ -f /tmp/leo_maintenance ] && exit 0

TOUT=/home/lab272/TOUT
# Point de vérité réseau centralisé (2026-07-20) — voir tools/robot_env.sh :
# ROBOT_HOST/ROS_MASTER_URI/ROS_IP viennent d'ici, plus jamais codés en dur.
# shellcheck disable=SC1091
source "$TOUT/tools/robot_env.sh"
# CORRECTION 2026-07-28 — BUG D'ORDRE, cause des tempetes de relances auto.
# Ce source vivait ligne 141, APRES les deux gardes de presence
# (mins_subscribe ligne ~100, ov_msckf ligne ~123). Sous cron le PATH est
# minimal : `rosnode` n'existait pas encore quand ces gardes l'appelaient,
# la commande echouait, et les deux compteurs montaient a 6/6 CHAQUE MINUTE
# quoi que fasse le robot -> restart_stack en boucle toutes les ~2 min
# (28/07 21h : 6 relances en 14 min, journal watchdog). Les gardes n'ont
# donc jamais rien supervise sous cron : ils ne faisaient que detruire.
# En ligne de commande le bug etait invisible, le PATH y etant deja bon.
source /opt/ros/noetic/setup.bash 2>/dev/null
# Auto-diagnostic : sous cron ce script échouait silencieusement (cron mailait
# la sortie vers un MTA absent). Toute erreur atterrit désormais ici :
mkdir -p "$TOUT/logs"
exec 2>> "$TOUT/logs/watchdog_err.log"
echo "$(date '+%F %T') run pid=$$" >> "$TOUT/logs/watchdog_err.log"
LOG="$TOUT/logs/watchdog.log"
mkdir -p "$TOUT/logs"
# garde le journal sous ~500 lignes
[ -f "$LOG" ] && [ "$(wc -l < "$LOG")" -gt 500 ] && tail -200 "$LOG" > "$LOG.tmp" && mv "$LOG.tmp" "$LOG"

note() { echo "$(date '+%F %T') $*" >> "$LOG"; }

# La pile web (site + cockpit + rapport) ne dépend PAS du robot : elle se
# soigne AVANT la porte ping — robot éteint ne doit plus = site mort.
port_ok() { ss -ltn 2>/dev/null | grep -q ":$1 "; }

web_down=0
for p in 8000 8080 9090 9443; do
  port_ok "$p" || { note "port $p mort"; web_down=1; }
done
pgrep -x cloudflared > /dev/null 2>&1 || { note "cloudflared mort"; web_down=1; }

# ── Sonde de NOYADE rosbridge — 9090 ET 9443 (2026-07-13, étendue 31/07) ────
# Après une longue absence du master (robot hors WiFi), ou après un simple
# redémarrage du master ROS, un bridge garde son port lié mais perd ses
# poignées ROS : les clients restent en CONNECTING pour toujours, la page se
# croit connectée et tous les boutons sont inertes. Le test de port ne voit
# RIEN, puisque le port EST lié. Seul un vrai handshake tranche.
#
# ÉTENDU AU 9443 le 31/07 : jusque-là seul le 9090 était sondé, alors que le
# mode de défaillance « port zombie » observé le 30/07 touchait le 9443. La
# seule sonde capable de le détecter ne couvrait pas le port concerné.
#
# ATTENTION — le 9443 est en TLS (listenSSL dans rosbridge_websocket_tunnel.py).
# Dupliquer la sonde en clair telle quelle donne un ÉCHEC SYSTÉMATIQUE, donc
# une purge des deux bridges à chaque minute : exactement la tempête de
# relances corrigée le 29/07. Vérifié le 31/07 : sonde en clair sur 9443 =
# AssertionError, sonde TLS = HTTP/1.1 400, soit une réponse, donc vivant.
# Un 400 suffit : on teste que le processus répond, pas qu'il accepte.
ws_probe() {          # $1 = port, $2 = plain|tls  → 0 si vivant
  timeout -k 3 10 python3 - "$1" "$2" <<'PYEOF' > /dev/null 2>&1
import socket, ssl, base64, os, sys
port, mode = int(sys.argv[1]), sys.argv[2]
raw = socket.create_connection(('127.0.0.1', port), timeout=4)
if mode == 'tls':
    sock = ssl._create_unverified_context().wrap_socket(raw, server_hostname='localhost')
else:
    sock = raw
sock.sendall(('GET / HTTP/1.1\r\nHost: 127.0.0.1:%d\r\nUpgrade: websocket\r\n'
              'Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n'
              'Sec-WebSocket-Version: 13\r\n\r\n'
              % (port, base64.b64encode(os.urandom(16)).decode())).encode())
sock.settimeout(6)
assert sock.recv(64)
PYEOF
}

drowned=""
port_ok 9090 && { ws_probe 9090 plain || drowned="9090"; }
port_ok 9443 && { ws_probe 9443 tls   || drowned="${drowned:+$drowned et }9443"; }
if [ -n "$drowned" ]; then
  # On purge LES DEUX bridges même si un seul est noyé : ils partagent le
  # même graphe ROS et start_web.sh les relance ensemble de façon idempotente.
  note "rosbridge $drowned NOYÉ (port lié, handshake gelé) — purge des deux bridges"
  kill $(pgrep -f "__name:=rosbridge_server") $(pgrep -f "rosbridge_websocket_tunnel") 2>/dev/null
  sleep 2
  web_down=1
fi

if [ "$web_down" = 1 ]; then
  note "relance pile web (start_web.sh)"
  bash "$TOUT/web/start_web.sh" >> "$LOG" 2>&1
fi

# Le robot doit être joignable pour la suite (stack nav, caméra), sinon on
# attend le retour du WiFi (autoconnect NetworkManager s'en charge).
ping -c 1 -W 2 "$ROBOT_HOST" > /dev/null 2>&1 || { note "robot ($ROBOT_HOST) injoignable — attente (web OK)"; exit 0; }

if ! pgrep -f "roslaunch leo_navigation navigation_master" > /dev/null 2>&1; then
  note "stack navigation morte — relance"
  nohup "$TOUT/catkin_ws/src/leo_navigation/launch_mins.sh" navigation_master.launch \
        > "$TOUT/logs/navigation_master.log" 2>&1 &
else
  # ── Zombies post-reboot roscore (2026-07-13, débounce élargi 2026-07-21) ──
  # roslaunch PC vivant mais mins_subscribe ABSENT du master = le roscore du
  # robot a redémarré (power-cycle) et les enregistrements sont morts. C'était
  # la dernière étape manuelle de la chaîne de récupération.
  #
  # 2026-07-21 : le débounce à 3 ticks/10s (prévu pour un raté isolé de
  # `rosnode list` "sous charge") s'est avéré insuffisant lors du premier
  # test terrain en mouvement (couloir, campus WiFi) — corrélation exacte
  # entre watchdog.log et les bascules pose_source MINS->VINS observées en
  # direct : le probe échouait de façon SOUTENUE (plusieurs minutes d'affilée)
  # pendant les déplacements, probablement le roaming WiFi de wlan_int (le
  # robot change de point d'accès en marchant) qui casse transitoirement
  # l'aller-retour XML-RPC PC->robot, bien plus long qu'un raté isolé.
  # Résultat mesuré : la stack entière s'est relancée toutes les 1 à 3 min
  # PENDANT TOUT LE TRAJET (voir watchdog.log 18:01-18:16) — le test a
  # réussi malgré ces relances, pas parce que MINS est resté stable. Élargi
  # à 6 ticks (~6 min de perte soutenue, pas juste un aller-retour raté) et
  # le timeout par appel doublé (10s -> 20s) pour absorber une latence de
  # roaming isolée sans consommer un tick entier pour ça. À revalider lors
  # du prochain test en mouvement (fréquence des faux positifs restants).
  ZFAIL=/tmp/leo_zombie_fails
  if ! timeout -k 5 20 rosnode list 2>/dev/null | grep -q "/mins_subscribe"; then
    zf=$(cat "$ZFAIL" 2>/dev/null || echo 0); zf=$((zf + 1)); echo "$zf" > "$ZFAIL"
    if [ "$zf" -ge 6 ]; then    # 6 ticks : voir note ci-dessus (2026-07-21)
      echo 0 > "$ZFAIL"
      note "ZOMBIES post-reboot (roslaunch vivant, mins_subscribe non enregistré x$zf/6) — restart_stack AUTO"
      nohup bash "$TOUT/tools/restart_stack.sh" >> "$LOG" 2>&1 &
      exit 0
    fi
  else
    echo 0 > "$ZFAIL"
  fi

  # ── Supervision ov_msckf / VINS (2026-07-23, audit — item Majeur #5) ────
  # Symétrique à la garde mins_subscribe ci-dessus : MINS avait sa
  # supervision de présence, VINS n'en avait aucune (item 20 du rapport,
  # ov_msckf trouvé totalement arrêté le 22/07 sans que rien ne le
  # détecte). ov_msckf a déjà respawn="true" au niveau roslaunch (relance
  # sur crash), donc ce garde-fou couvre le même angle mort que pour MINS :
  # process vivant mais DÉSENREGISTRÉ du graphe ROS (roaming WiFi cassant
  # l'aller-retour XML-RPC), ce que respawn ne détecte jamais puisque le
  # process lui-même ne meurt pas. Même débounce (6 ticks) et même remède
  # (restart_stack.sh) que mins_subscribe, pour rester cohérent.
  VFAIL=/tmp/leo_vins_fails
  if ! timeout -k 5 20 rosnode list 2>/dev/null | grep -q "/ov_msckf"; then
    vf=$(cat "$VFAIL" 2>/dev/null || echo 0); vf=$((vf + 1)); echo "$vf" > "$VFAIL"
    if [ "$vf" -ge 6 ]; then
      echo 0 > "$VFAIL"
      note "VINS ABSENT (ov_msckf non enregistré x$vf/6) — restart_stack AUTO"
      nohup bash "$TOUT/tools/restart_stack.sh" >> "$LOG" 2>&1 &
      exit 0
    fi
  else
    echo 0 > "$VFAIL"
  fi
fi

# ── Auto-guérison caméra D455 ────────────────────────────────────────────────
# En batterie faible (<~10.5V) le rail USB du Pi s'affaisse, la D455 décroche,
# les nodes rosmon sortent proprement (IDLE) et ne sont jamais relancés →
# cam_alive=False, le mode AUTO refuse de bouger. On détecte le silence
# d'infra1 et on redemande le START à rosmon (côté robot).
# ROS_MASTER_URI/ROS_IP déjà exportés par robot_env.sh en tête de script.
# ── Échelle d'escalade caméra (compteur persistant entre les ticks, débounce
#    élargi 2026-07-21 — même cause que le garde zombie ci-dessus : sous
#    roaming WiFi en mouvement, un aller-retour rostopic peut échouer
#    plusieurs ticks de suite sans que la caméra soit réellement en cause.
#    Le cycle STOP/START (tick 1) reste immédiat — coût faible, utile pour un
#    vrai décrochage. Le RESET MATÉRIEL (35+ s sans vision, mode AUTO refuse
#    de bouger pendant ce temps) est nettement plus coûteux si déclenché à
#    tort : repoussé de 2 à 4 échecs, probe élargie 6 -> 12 s. ─────────────
CAMFAIL=/tmp/leo_cam_fails
if ! timeout -k 5 12 rostopic echo -n1 /camera/infra1/camera_info > /dev/null 2>&1; then
  fails=$(cat "$CAMFAIL" 2>/dev/null || echo 0)
  fails=$((fails + 1)); echo "$fails" > "$CAMFAIL"
  batt=$(timeout -k 5 5 rostopic echo -n1 /firmware/battery 2>/dev/null | grep -o "[0-9.]*" | head -1)
  if [ "$fails" -ge 4 ]; then
    note "caméra muette (échec $fails, batt=${batt}V) — RESET MATÉRIEL D455 + cycle"
    timeout -k 5 12 rosservice call /camera/realsense2_camera/reset > /dev/null 2>&1
    sleep 15
  else
    note "caméra muette (échec $fails, batt=${batt}V) — cycle STOP/START rosmon"
  fi
  [ "$fails" -ge 3 ] && note "ALERTE: caméra muette depuis $fails min — vérifier BATTERIE/USB (rail < ~10.5V = décrochages)"
  timeout -k 5 8 rosservice call /rosmon/start_stop "{node: 'realsense2_camera', ns: '/camera', action: 2}" > /dev/null 2>&1
  timeout -k 5 8 rosservice call /rosmon/start_stop "{node: 'realsense2_camera_manager', ns: '/camera', action: 2}" > /dev/null 2>&1
  sleep 4
  timeout -k 5 8 rosservice call /rosmon/start_stop "{node: 'realsense2_camera_manager', ns: '/camera', action: 1}" > /dev/null 2>&1
  timeout -k 5 8 rosservice call /rosmon/start_stop "{node: 'realsense2_camera', ns: '/camera', action: 1}" > /dev/null 2>&1
  # les republish PC gardent des abonnements morts après un reset caméra —
  # on les tue, la supervision roslaunch les respawne proprement en 5 s
  pkill -f "image_transport" 2>/dev/null
  # laser_power 0 (2026-07-13, MESURÉ) : le projecteur dégradait TOUT —
  # depth 56 %->81 % SANS laser (stéréo passive sur tapis texturé), damier
  # mitraillé de points, LEDs noyées, tags pollués. Réappliqué après chaque
  # guérison caméra (le réglage dynamic_reconfigure se perd au restart).
  sleep 20
  timeout -k 5 15 rosrun dynamic_reconfigure dynparam set /camera/stereo_module laser_power 0 > /dev/null 2>&1
  # png_level 1 (2026-07-13) : le niveau 9 d'usine mettait le compresseur PNG
  # du Pi à genoux (depth 15 Hz -> 1-3 Hz au PC = marge obstacle rongée).
  timeout -k 5 15 rosrun dynamic_reconfigure dynparam set /camera/depth/image_rect_raw/compressedDepth png_level 1 > /dev/null 2>&1
else
  # flux robot OK : si on sort d'une panne, consigner la guérison
  if [ -s "$CAMFAIL" ] && [ "$(cat "$CAMFAIL")" != "0" ]; then
    note "caméra REVENUE (après $(cat "$CAMFAIL") échec(s))"
  fi
  echo 0 > "$CAMFAIL"
fi
# ── Sonde de FAMINE IMU (2026-07-13, débounce élargi 2026-07-21) ────────────
# Le serial_node (rosserial, Pi) peut se dégrader après désyncs/power-cycles :
# il produit à 90 Hz mais ne pousse plus que 3-8 Hz vers le PC, PENDANT que
# les autres flux robot->PC restent nominaux (radio hors de cause). MINS ne
# peut alors ni s'initialiser ni tenir. Remède mesuré : cycle rosmon du
# serial_node (3-8 Hz -> 79 Hz). Sonde : moins de ~25 msgs IMU en 3 s alors
# que la caméra parle = famine -> cycle. Une souscription fraîche qui démarre
# pendant un pic de latence réseau (roaming WiFi en mouvement, même cause que
# le garde zombie plus haut) peut légitimement rater ce seuil sans famine
# réelle ; élargi de 2 à 4 ticks consécutifs (anti-transitoire).
IMUFAIL=/tmp/leo_imu_fails
imu_n=$(timeout -k 5 12 python3 -c "
import rospy, time
from sensor_msgs.msg import Imu
rospy.init_node('wd_imu', anonymous=True)
n = [0]
rospy.Subscriber('/imu/data_clean', Imu, lambda m: n.__setitem__(0, n[0]+1), queue_size=300)
time.sleep(3)
print(n[0])" 2>/dev/null | tail -1)
if [ -n "$imu_n" ] && [ "$imu_n" -lt 25 ] 2>/dev/null; then
  if timeout -k 5 8 rostopic echo -n1 /camera/infra1/camera_info > /dev/null 2>&1; then
    inf=$(cat "$IMUFAIL" 2>/dev/null || echo 0); inf=$((inf + 1)); echo "$inf" > "$IMUFAIL"
    if [ "$inf" -ge 4 ]; then
      echo 0 > "$IMUFAIL"
      note "FAMINE IMU ($imu_n msgs/3 s, caméra OK x$inf) — cycle rosmon serial_node"
      timeout -k 5 10 rosservice call /rosmon/start_stop "{node: 'serial_node', ns: '', action: 2}" > /dev/null 2>&1
      sleep 4
      timeout -k 5 10 rosservice call /rosmon/start_stop "{node: 'serial_node', ns: '', action: 1}" > /dev/null 2>&1
    fi
  fi
else
  echo 0 > "$IMUFAIL"
fi

# ── Règle d'exclusivité Carolus (verrouillage 2026-07-08) ───────────────────
# /mins/external_ref/carolus est PRIVÉ au slot VICON de MINS. Tout autre
# abonné (hors outils d'inspection ros*) est une fuite d'architecture -> ALERTE.
subs=$(timeout -k 5 8 rostopic info /mins/external_ref/carolus 2>/dev/null | sed -n '/Subscribers:/,$p' | grep -o '/[a-zA-Z0-9_]*' | grep -v "^/mins_subscribe$\|^/ros" | head -3)
[ -n "$subs" ] && note "ALERTE EXCLUSIVITE: abonné(s) non-MINS sur /mins/external_ref/carolus : $subs"

# ── Garde anti-divergence MINS (2026-07-13) ─────────────────────────────────
# Un filtre qui diverge (position km, vitesse ~1000 m/s, vécu 2x) reste muet
# pour tout le monde : le cockpit affiche « n'importe quoi » et la carte se
# fait empoisonner. Position > 100 m = impossible dans le labo -> relance
# propre de la stack (restart_stack pose lui-même le drapeau maintenance).
podom=$(timeout -k 5 8 rostopic echo -n1 /mins/imu/odom/pose/pose/position 2>/dev/null | grep -E "^[xy]:" | awk '{print $2}')
if [ -n "$podom" ]; then
  insane=$(echo "$podom" | awk 'function abs(v){return v<0?-v:v} abs($1)>100{f=1} END{print f+0}')
  if [ "$insane" = "1" ]; then
    note "ALERTE: MINS DIVERGÉ (|position| > 100 m) — relance stack automatique"
    nohup bash "$TOUT/tools/restart_stack.sh" >> "$LOG" 2>&1 &
    exit 0
  fi
fi

# republish zombies sans caméra morte : JPEG circule mais /pc/camera muet
if timeout -k 5 6 rostopic echo -n1 /camera/infra1/camera_info > /dev/null 2>&1 \
   && ! timeout -k 5 6 rostopic echo -n1 /pc/camera/infra1/image_rect_raw > /dev/null 2>&1; then
  note "republish zombies — purge (respawn supervision)"
  pkill -f "image_transport" 2>/dev/null
fi

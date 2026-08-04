#!/usr/bin/env bash
# =============================================================================
# start_web.sh — Lance la pile WEB du LEO Rover (et uniquement elle).
#
#   rosbridge_server   ws://<PC>:9090   -> télémétrie/commandes en LAN direct
#   rosbridge TLS      wss://<PC>:9443  -> même chose, via le tunnel Cloudflare
#   web_video_server   http://<PC>:8080 -> flux MJPEG
#   http.server        http://<PC>:8000 -> pages index/ops/logbook/navigation_modes
#   calibration_monitor                 -> /leo_vision/calibration_status
#   cloudflared                         -> expose le tout sur cockpit.leo-rover-gardon.dev
#
# CE SCRIPT NE LANCE PLUS leo_backend / pose_selector / imu_sanitizer :
# ils vivent désormais dans navigation_supervision.launch (respawn géré par
# roslaunch), démarrés via :
#   catkin_ws/src/leo_navigation/launch_mins.sh navigation_master.launch
# Les lancer ici en double créerait des conflits de noms ROS (le master tue
# l'ancien homonyme) et une guerre de respawn.
#
# Tous les services partent en nohup : ils survivent à la fermeture du
# terminal ET aux coupures WiFi (contrairement à l'ancienne version où tout
# mourait avec la session).
#
# IDEMPOTENT (2026-07-13) : chaque composant n'est (re)lancé QUE s'il est
# réellement mort. L'ancienne version repartait de zéro à chaque appel — or le
# watchdog appelle ce script dès qu'UN port manque : un 9443 en échec de
# démarrage (WiFi qui flappe) faisait donc tuer le rosbridge 9090 SAIN toutes
# les minutes — le cockpit de l'opérateur était déconnecté en boucle. Un
# composant en cours de démarrage (process présent, port pas encore lié) est
# LAISSÉ TRANQUILLE : rosbridge peut mettre >1 min à se lier sous charge.
# =============================================================================

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="$(dirname "$HERE")/logs"
CERTDIR="$HOME/.leo_certs"
mkdir -p "$LOGDIR"

# Point de vérité réseau centralisé (2026-07-20) : ROBOT_IP/ROS_MASTER_URI/
# ROS_IP venaient d'une détection propre à ce script — désormais partagée
# via tools/robot_env.sh (même logique, même comportement, un seul endroit
# à changer pour migrer de réseau). Voir tools/robot_env.sh pour le détail.
# shellcheck disable=SC1091
source "$(dirname "$HERE")/tools/robot_env.sh"
ROBOT_IP="$ROBOT_HOST"
export ROS_IP

source /opt/ros/noetic/setup.bash

echo "──────────────────────────────────────────────────────────"
echo "  LEO Rover — pile web   (ROS_IP=$ROS_IP)"
echo "──────────────────────────────────────────────────────────"

# Libère un port TCP resté squatté par un lancement précédent mal arrêté.
free_port() {
  local p="$1" held
  held="$(ss -ltnp 2>/dev/null | grep ":$p " | grep -oP 'pid=\K[0-9]+' | sort -u)"
  if [ -n "$held" ]; then
    echo "[..] port $p occupé (PID $held) — libération"
    # shellcheck disable=SC2086
    kill $held 2>/dev/null
    sleep 0.5
  fi
}

# --- 1) rosbridge LAN (9090) — _websocket_external_port EXPLICITE à 9090 :
#     autobahn vérifie le header Host (un client LAN envoie Host <ip>:9090).
#     Toujours passer la valeur explicitement : rospy relit sinon un éventuel
#     ~websocket_external_port fantôme resté sur le param server d'un
#     lancement précédent (nous a coûté le mode localhost pendant 2 jours).
if ss -ltn 2>/dev/null | grep -q ":9090 "; then
  echo "[= ] rosbridge LAN        :9090 déjà actif — INTACT"
elif pgrep -f "__name:=rosbridge_server" > /dev/null 2>&1; then
  echo "[..] rosbridge LAN        :9090 en cours de démarrage — patience"
else
  nohup rosrun rosbridge_server rosbridge_websocket __name:=rosbridge_server \
        _port:=9090 _websocket_external_port:=9090 \
        > "$LOGDIR/rosbridge_9090.log" 2>&1 &
  echo "[ok] rosbridge LAN        :9090 lancé"
fi

# --- 2) rosbridge TLS (9443) pour le tunnel — AVEC external_port 443 :
#     les handshakes arrivent de cloudflared avec Host public SANS port
#     (= 443 implicite) ; sans ce paramètre autobahn rejette tout
#     (« missing port in HTTP Host header ... non-standard port 9443 »).
if ss -ltn 2>/dev/null | grep -q ":9443 "; then
  echo "[= ] rosbridge TLS tunnel :9443 déjà actif — INTACT"
elif pgrep -f "rosbridge_websocket_tunnel" > /dev/null 2>&1; then
  echo "[..] rosbridge TLS tunnel :9443 en cours de démarrage — patience"
else
  nohup python3 "$(dirname "$HERE")/tools/rosbridge_websocket_tunnel.py" \
        __name:=rosbridge_websocket_tunnel _port:=9443 _websocket_external_port:=443 \
        _certfile:="$CERTDIR/rosbridge_cert.pem" _keyfile:="$CERTDIR/rosbridge_key.pem" \
        > "$LOGDIR/rosbridge_9443.log" 2>&1 &
  echo "[ok] rosbridge TLS tunnel :9443 lancé (external 443)"
fi

# --- 3) web_video_server (MJPEG) ---
# 2026-07-13 : DÉPLACÉ sous la supervision roslaunch (navigation_supervision
# .launch, respawn) — il ne sert que l'image annotée du backend et doit vivre
# avec la stack. Ne PAS le lancer ici : free_port 8080 tuait l'instance
# supervisée à chaque guérison web, et la doublonner crée une guerre de noms.
ss -ltn 2>/dev/null | grep -q ":8080 " || echo "[note] web_video_server absent (relancé par la supervision nav sous ~5 s si la stack tourne)"
echo "[ok] web_video_server     :8080"

# --- 4) pages statiques ---
if ss -ltn 2>/dev/null | grep -q ":8000 "; then
  echo "[= ] interface web        :8000 déjà active — INTACTE"
else
  nohup python3 "$HERE/serve.py" > "$LOGDIR/http_8000.log" 2>&1 &
  echo "[ok] interface web        :8000  -> http://$ROS_IP:8000/index.html"
fi

# --- 5) calibration_monitor ---
if pgrep -f "calibration_monitor.py" > /dev/null 2>&1; then
  echo "[= ] calibration_monitor déjà actif"
else
  nohup python3 "$(dirname "$HERE")/calibration_monitor.py" \
        > "$LOGDIR/calibration_monitor.log" 2>&1 &
  echo "[ok] calibration_monitor lancé"
fi

# --- 6) tunnel Cloudflare (cockpit.leo-rover-gardon.dev) ---
if ! pgrep -x cloudflared > /dev/null 2>&1; then
  # Chemin absolu (2026-07-20) : `cloudflared` seul échouait sous cron/watchdog
  # (PATH minimal n'incluant pas ~/.local/bin) -> "No such file or directory"
  # silencieux dans cloudflared.log, tunnel public mort sans alerte visible.
  CFD="$(command -v cloudflared || echo "$HOME/.local/bin/cloudflared")"
  nohup "$CFD" tunnel run > "$LOGDIR/cloudflared.log" 2>&1 &
  echo "[ok] cloudflared (tunnel public)"
else
  echo "[= ] cloudflared déjà actif"
fi

echo "──────────────────────────────────────────────────────────"
echo "  Cockpit public : https://cockpit.leo-rover-gardon.dev/ops.html"
echo "  Stack navigation (MINS & co, si pas déjà lancée) :"
echo "    $(dirname "$HERE")/catkin_ws/src/leo_navigation/launch_mins.sh navigation_master.launch"
echo "──────────────────────────────────────────────────────────"

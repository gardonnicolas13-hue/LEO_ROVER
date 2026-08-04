#!/bin/bash
# ============================================================
#  start_leo.sh  —  Lance tout l'écosystème LEO Rover
#  Usage : ./start_leo.sh   ou   leo-start
# ============================================================

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://10.0.0.1:11311
export ROS_IP=10.0.0.10

LOGDIR="$HOME/.leo_logs"
mkdir -p "$LOGDIR"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo ""
echo "  ██╗     ███████╗ ██████╗     ██████╗  ██████╗ ██╗   ██╗███████╗██████╗ "
echo "  ██║     ██╔════╝██╔═══██╗    ██╔══██╗██╔═══██╗██║   ██║██╔════╝██╔══██╗"
echo "  ██║     █████╗  ██║   ██║    ██████╔╝██║   ██║██║   ██║█████╗  ██████╔╝"
echo "  ██║     ██╔══╝  ██║   ██║    ██╔══██╗██║   ██║╚██╗ ██╔╝██╔══╝  ██╔══██╗"
echo "  ███████╗███████╗╚██████╔╝    ██║  ██║╚██████╔╝ ╚████╔╝ ███████╗██║  ██║"
echo "  ╚══════╝╚══════╝ ╚═════╝     ╚═╝  ╚═╝ ╚═════╝   ╚═══╝  ╚══════╝╚═╝  ╚═╝"
echo "  Florida Tech Robotics Lab"
echo ""

# ── 1. Ping robot ─────────────────────────────────────────────────────────────
echo -n "  [1/4] Robot 10.0.0.1 ... "
if ! ping -c 1 -W 2 10.0.0.1 > /dev/null 2>&1; then
    echo "HORS LIGNE ✗"
    echo ""
    echo "  ERREUR : impossible de joindre 10.0.0.1"
    echo "  → Vérifie que le rover est allumé et que tu es sur le Wi-Fi du robot."
    echo ""
    exit 1
fi
echo "en ligne ✓"

# Vérifie si un port TCP est réellement ouvert (process vivant ≠ port ouvert)
port_open() { ss -tlnp | grep -q ":$1 "; }

# Tue proprement un process et ses zombies sur un port donné
kill_and_wait() {
    local pattern="$1"
    pkill -f "$pattern" 2>/dev/null
    sleep 0.5
    pkill -9 -f "$pattern" 2>/dev/null
    sleep 0.3
}

# ── 2. rosbridge ──────────────────────────────────────────────────────────────
echo -n "  [2/4] rosbridge (ws:9090) ... "
if port_open 9090; then
    echo "déjà actif ✓"
else
    # Tue tout process rosbridge zombie (port fermé mais process vivant)
    kill_and_wait "rosbridge_websocket"
    rosrun rosbridge_server rosbridge_websocket _port:=9090 \
        > "$LOGDIR/rosbridge.log" 2>&1 &
    sleep 2
    if port_open 9090; then
        echo "démarré ✓"
    else
        echo "ÉCHEC ✗  (voir $LOGDIR/rosbridge.log)"
    fi
fi

# ── 3. web_video_server ───────────────────────────────────────────────────────
echo -n "  [3/4] web_video_server (:8080) ... "
if port_open 8080; then
    echo "déjà actif ✓"
else
    kill_and_wait "web_video_server"
    rosrun web_video_server web_video_server _port:=8080 _allow_origin:="*" \
        > "$LOGDIR/web_video.log" 2>&1 &
    sleep 1.5
    if port_open 8080; then
        echo "démarré ✓"
    else
        echo "ÉCHEC ✗  (voir $LOGDIR/web_video.log)"
    fi
fi

# ── 4. leo_backend ────────────────────────────────────────────────────────────
echo -n "  [4/4] leo_backend ... "
if pgrep -f "leo_backend.py" > /dev/null 2>&1; then
    echo "déjà actif ✓"
else
    cd "$SCRIPT_DIR"
    python3 leo_backend.py > "$LOGDIR/backend.log" 2>&1 &
    sleep 1.5
    if pgrep -f "leo_backend.py" > /dev/null 2>&1; then
        echo "démarré ✓"
    else
        echo "ÉCHEC ✗  (voir $LOGDIR/backend.log)"
    fi
fi

# ── Serveur web ───────────────────────────────────────────────────────────────
# Chemin absolu garanti — insensible au répertoire courant à l'appel
WEB_DIR="/home/lab272/TOUT/web"
echo -n "  [web]   http.server (:8000) ... "
pkill -f "http.server 8000" 2>/dev/null
sleep 0.3
python3 -m http.server 8000 --directory "$WEB_DIR" \
    > "$LOGDIR/webserver.log" 2>&1 &
sleep 0.5
if port_open 8000; then
    echo "démarré ✓  ($WEB_DIR)"
else
    echo "ÉCHEC ✗  (voir $LOGDIR/webserver.log)"
fi

echo ""
echo "  ─────────────────────────────────────────────"
echo "  Cockpit  →  http://localhost:8000/ops.html"
echo "  Logs     →  $LOGDIR/"
echo "  Stopper  →  leo-stop"
echo "  ─────────────────────────────────────────────"
echo ""

sleep 0.5
xdg-open http://localhost:8000/ops.html 2>/dev/null &

exit 0

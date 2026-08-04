#!/bin/bash
# ============================================================
#  stop_leo.sh  —  Arrête tous les services LEO Rover
#  Usage : ./stop_leo.sh   ou   leo-stop
# ============================================================

echo ""
echo "  [LEO] Arrêt des services..."
echo ""

stop_proc() {
    local name="$1"
    local pattern="$2"
    if pgrep -f "$pattern" > /dev/null 2>&1; then
        pkill -f "$pattern" 2>/dev/null
        sleep 0.3
        echo "  ✓ $name arrêté"
    else
        echo "  - $name non actif"
    fi
}

stop_proc "leo_backend"       "leo_backend.py"
stop_proc "web_video_server"  "web_video_server"
stop_proc "rosbridge"         "rosbridge_websocket"
stop_proc "serveur web"       "http.server 8000"

echo ""
echo "  [LEO] Tout est arrêté."
echo ""
exit 0

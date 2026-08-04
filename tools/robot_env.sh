#!/usr/bin/env bash
# =============================================================================
# robot_env.sh — point de vérité UNIQUE pour l'adresse du robot.
#
# Avant (2026-07) : 10.0.0.1 (robot) et 10.0.0.10 (PC) codés en dur dans 5
# scripts (~/bin/leo, tools/leo_watchdog.sh, tools/restart_stack.sh,
# web/start_web.sh, launch_mins.sh) — changer de réseau (ex. migration vers
# FloridaTech-Guest, IP attribuée par DHCP campus, non fixe) exigeait 5
# éditions manuelles, source d'oubli garantie.
#
# Maintenant : tous ces scripts font `source robot_env.sh` et lisent
# $ROBOT_HOST / $ROS_MASTER_URI / $ROS_IP au lieu de valeurs figées.
#
# Pour changer de réseau SANS TOUCHER AU CODE :
#   echo 'LEO_ROBOT_HOST=leo.local' > ~/.leo_network      # nom mDNS
#   echo 'LEO_ROBOT_HOST=192.168.1.42' > ~/.leo_network   # IP DHCP campus
#   rm -f ~/.leo_network                                   # retour au défaut (hotspot robot)
#
# Défaut préservé = comportement identique à avant cette refonte tant que
# ~/.leo_network n'existe pas (10.0.0.1, hotspot LeoRover-e138).
# =============================================================================

[ -f "$HOME/.leo_network" ] && source "$HOME/.leo_network"

export ROBOT_HOST="${LEO_ROBOT_HOST:-10.0.0.1}"
export ROS_MASTER_URI="http://${ROBOT_HOST}:11311"

# ROS_IP = adresse du PC SUR LA ROUTE VERS le robot — auto-détectée via la
# table de routage plutôt que figée, pour rester correcte quel que soit le
# réseau actif (hotspot robot, FloridaTech-Guest, ou autre plus tard). Un PC
# multi-réseaux (Ethernet + WiFi) peut avoir plusieurs IP : celle qui compte
# est celle de l'interface qui sait effectivement joindre $ROBOT_HOST.
_leo_ros_ip="$(ip route get "$ROBOT_HOST" 2>/dev/null | grep -oP 'src \K[0-9.]+' | head -1)"
export ROS_IP="${_leo_ros_ip:-$(hostname -I 2>/dev/null | awk '{print $1}')}"
unset _leo_ros_ip

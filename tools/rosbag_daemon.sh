#!/usr/bin/env bash
# rosbag_daemon.sh — lance `rosbag record` détaché (double-fork), pour que
# l'enregistrement d'un essai Test 1/Test 2 (déclenché depuis le site web)
# survive à un redémarrage de leo_backend.py.
#
# Même technique que tools/matlab_daemon.sh (2026-07-27) : `cmd &` puis
# sortie IMMÉDIATE de ce script (sans wait) orpheline le job en arrière-plan,
# réattaché à init/subreaper — structurellement hors de la descendance
# PPID de leo_backend.py, donc insensible à un nettoyage par groupe ou par
# arbre de process (killpg roslaunch, restart_stack.sh, etc.).
#
# rosbag record répond proprement à SIGINT (flush + fermeture du .bag),
# exactement comme un Ctrl-C au terminal — utilisé ici par
# leo_backend.py::_stop_robust_record() pour arrêter un enregistrement en
# cours, y compris après son propre redémarrage (le PID est retrouvé via un
# fichier d'état, pas via une variable en mémoire qui ne survivrait pas).
#
# Sortie de rosbag capturée dans <bag_prefix>.log (PAS /dev/null — une
# leçon coûteuse de ce même 27/07 avec MATLAB : DEVNULL rend un échec
# totalement invisible).
#
# Usage : rosbag_daemon.sh <pidfile> <bag_prefix> <topic...>
set -u
pidfile="$1"; shift
bag_prefix="$1"; shift

setsid bash -c '
  rosbag record --lz4 -O "$1" "${@:2}" > "$1.log" 2>&1 &
  echo $! > "$0"
  wait
' "$pidfile" "$bag_prefix" "$@" < /dev/null > /dev/null 2>&1 &
disown
exit 0

#!/usr/bin/env bash
# matlab_daemon.sh — détachement total (double-fork) + pty pour un lancement
# GUI MATLAB déclenché depuis leo_backend.py.
#
# Historique : d'abord écrit pour un seul problème (survivre à un restart de
# stack déclenché par le watchdog PENDANT une session MATLAB ouverte). Le
# double-fork (`cmd &` puis sortie immédiate, sans wait) règle bien CE
# problème-là — vérifié : le process réattaché à init/subreaper sort de la
# filiation PPID de leo_backend.py.
#
# MAIS un second bug, distinct et plus fondamental, causait la même
# symptomatologie ("MATLAB s'ouvre puis se ferme tout seul") même SANS aucun
# restart : leo_backend.py lance MATLAB sans terminal contrôleur (rosmon n'en
# fournit aucun) — `tty` sur ce stdin répond "not a tty". Testé en direct
# (2026-07-27) : `matlab -r "plot_trajectories(...)"` avec stdin non-tty finit
# par un `exit(0)` PROPRE dès que la commande -r se termine (le script tourne
# bien jusqu'au bout, PNG généré, aucun crash) — contrairement à l'hypothèse
# initiale ("-r garde toujours le bureau ouvert", vraie seulement avec un vrai
# terminal attaché). Avec un pty alloué via `script`, confirmé : MATLAB reste
# ouvert indéfiniment après la même commande.
#
# Fix : `script` (util-linux) alloue un pty au process MATLAB, en plus du
# double-fork qui le détache de la filiation ROS. Les deux protections sont
# orthogonales et cumulées : pty = MATLAB ne s'auto-quitte plus ; double-fork
# = un restart de stack ne peut plus le tuer par arbre de process.
#
# EFFET DE BORD, ACCEPTÉ : avec un vrai pty, MATLAB route son texte
# (bannière, fprintf du script -r) vers la fenêtre GUI plutôt que vers le
# pty lui-même (constaté : sans pty — redirection fichier plate — la
# bannière ET la sortie du script sont capturées en entier ; avec pty,
# <logfile> ne contient que les lignes "Script started/done
# [COMMAND_EXIT_CODE=...]" de `script`). Le code de sortie/signal reste
# visible (utile pour détecter un crash : SIGKILL -> 137, segfault -> 139),
# la bannière ne l'est plus — accepté, l'objectif premier (ne plus fermer
# tout seul) prime sur le confort de logging.
#
# Usage : matlab_daemon.sh <logfile> <bin> [args...]
log="$1"; shift
cmdstr=$(printf '%q ' "$@")
setsid script -qefc "$cmdstr" "$log" < /dev/null > /dev/null 2>&1 &
disown
exit 0

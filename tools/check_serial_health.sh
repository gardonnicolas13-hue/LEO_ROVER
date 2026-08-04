#!/usr/bin/env bash
# =============================================================================
# check_serial_health.sh — mesure rapide du taux d'erreurs checksum sur la
# liaison série Pi<->CORE2 (serial_node, /dev/serial0, 250000 bauds).
#
# Contexte : item 19 du rapport (non résolu au 2026-07-23) — 20 295 erreurs
# "wrong checksum for topic id and msg" mesurées en 2 h le 22/07. Ce script
# donne un chiffre comparable avant/après une intervention physique sur le
# câble/connecteur (voir tools/diag_serial_pi_core2.md) — sans lui, toute
# évaluation d'un correctif physique reste une impression, pas une mesure.
#
# Usage : ./tools/check_serial_health.sh          (fenêtre des 5 dernières minutes)
#         ./tools/check_serial_health.sh 15min     (fenêtre personnalisée)
#         watch -n 2 ./tools/check_serial_health.sh  (suivi en continu)
# =============================================================================
set -euo pipefail

WINDOW="${1:-5min}"

echo "── Liaison série Pi<->CORE2 — fenêtre: -${WINDOW} ──"

out=$(timeout -k 5 12 ssh leo-guest \
  "sudo -n journalctl -u leo --no-pager --since '-${WINDOW}' 2>/dev/null | grep -c 'wrong checksum'" \
  2>/dev/null || echo "?")

sync_out=$(timeout -k 5 12 ssh leo-guest \
  "sudo -n journalctl -u leo --no-pager --since '-${WINDOW}' 2>/dev/null | grep -c 'lost sync\|Mismatched protocol version'" \
  2>/dev/null || echo "?")

if [ "$out" = "?" ]; then
  echo "injoignable (SSH/robot indisponible — réessayer)"
  exit 1
fi

# Minutes dans la fenêtre, pour un taux/minute comparable entre deux mesures
# de durées différentes (ex. 5min vs 15min).
mins=$(echo "$WINDOW" | grep -oE '^[0-9]+')
mins=${mins:-5}
rate=$(awk -v n="$out" -v m="$mins" 'BEGIN { printf "%.1f", (m>0 ? n/m : 0) }')

echo "checksum erronés : $out   (~${rate}/min)"
echo "pertes de sync / version protocole : $sync_out"

if [ "$out" -eq 0 ] 2>/dev/null; then
  echo "-> RAS sur cette fenêtre."
elif [ "$out" -lt 30 ] 2>/dev/null; then
  echo "-> présent mais faible — comparer à la mesure de référence avant intervention."
else
  echo "-> taux élevé, cohérent avec l'incident du 22/07 (~170/min mesuré ce soir-là)."
fi

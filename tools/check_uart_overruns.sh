#!/usr/bin/env bash
# =============================================================================
# check_uart_overruns.sh — lit le compteur noyau d'overruns FIFO PL011
# (/proc/tty/driver/ttyAMA, champ "oe:"), PAS le compteur de checksums
# rosserial que lit déjà check_serial_health.sh.
#
# Pourquoi un script séparé : ce sont deux signaux différents. Un checksum
# erroné est une conséquence côté protocole (rosserial détecte une trame
# corrompue) ; un overrun est l'événement matériel qui la cause (le noyau a
# jeté un octet parce que la FIFO de réception était pleine). C'est le
# compteur "oe" que le rapport utilise depuis le 30/07 (§eq:missprob,
# 137 000 → 676 751 → etc.) — comparer un taux de checksums à ce chiffre
# aurait mélangé deux mesures non commensurables.
#
# Usage :
#   ./tools/check_uart_overruns.sh              (fenêtre de 180 s, comme le 30/07)
#   ./tools/check_uart_overruns.sh 60            (fenêtre personnalisée, en secondes)
#
# Sortie : delta d'overruns sur la fenêtre + taux en s⁻¹, directement
# comparable à la valeur de référence du 30/07 : 553 en 180 s = 3,07 s⁻¹.
# =============================================================================
set -euo pipefail

WINDOW="${1:-180}"

read_oe() {
  timeout -k 5 12 ssh leo-guest "sudo -n cat /proc/tty/driver/ttyAMA" 2>/dev/null \
    | grep -oE 'oe:[0-9]+' | grep -oE '[0-9]+' || echo ""
}

echo "── Overruns FIFO UART (PL011, irq 14) — fenêtre : ${WINDOW}s ──"

oe1=$(read_oe)
if [ -z "$oe1" ]; then
  echo "injoignable (SSH/robot indisponible, ou sudo -n a échoué) — réessayer"
  exit 1
fi
t1=$(date +%s.%N)
echo "  T+0s    oe=${oe1}"

sleep "$WINDOW"

oe2=$(read_oe)
t2=$(date +%s.%N)
if [ -z "$oe2" ]; then
  echo "injoignable en fin de fenêtre — mesure invalide, relancer"
  exit 1
fi
echo "  T+${WINDOW}s  oe=${oe2}"

delta=$((oe2 - oe1))
dt=$(awk -v a="$t1" -v b="$t2" 'BEGIN{printf "%.1f", b-a}')
rate=$(awk -v d="$delta" -v t="$dt" 'BEGIN { printf "%.3f", (t>0 ? d/t : 0) }')

echo ""
echo "Δoe = ${delta}  sur ${dt}s  →  ${rate} s⁻¹"
echo "Référence 30/07 (sans ventilos, charge nodelet couleur 185 %) : 3.07 s⁻¹ (553 en 180s)"
echo ""

# Comparaison qualitative — volontairement large, ce n'est pas un verdict
# statistique (un seul échantillon), juste une lecture rapide sur le terrain.
awk -v r="$rate" 'BEGIN {
  if (r < 0.05)      print "-> quasi nul : très en dessous de la référence 30/07.";
  else if (r < 1.0)  print "-> présent mais nettement inférieur à la référence 30/07.";
  else if (r < 2.5)  print "-> du même ordre de grandeur que la référence 30/07 — pas de conclusion sans la charge exacte confirmée.";
  else               print "-> comparable ou supérieur à la référence 30/07, malgré le refroidissement actif.";
}'

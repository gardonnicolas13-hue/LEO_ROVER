#!/usr/bin/env bash
# =============================================================================
# repiloter.sh — retrouver le robot, relancer la pile, et VERIFIER qu'on peut
# reellement le piloter depuis le site.
#
#   Usage :  bash tools/repiloter.sh
#
# -----------------------------------------------------------------------------
# POURQUOI CE SCRIPT EXISTE (2026-09-11)
#
# `leo start` echoue en 30 s quand l'adresse epinglee dans ~/.leo_network ne
# correspond plus a celle du robot, et son message (« Allume le rover ») envoie
# chercher une panne materielle alors que le rover est allume. Le robot a trois
# adresses possibles selon le reseau qu'il a reussi a rejoindre, et l'operateur
# n'a aucune raison de savoir laquelle essayer.
#
# Ce script fait la sequence complete dans le bon ordre, et surtout il
# VERIFIE LE PILOTAGE plutot que la sante apparente. La distinction n'est pas
# theorique : le 18/08, toute la telemetrie etait saine (roues, IMU, odometrie
# a 18 Hz, cockpit ONLINE, video OK) et pourtant AUCUNE commande ne passait,
# parce que serial_node avait garde ses publications sortantes et perdu son
# abonnement entrant a /cmd_vel. Un controle « le robot repond » aurait dit
# vert. Le seul test qui tranche est : /cmd_vel a-t-il un abonne ?
#
# CE QU'IL FAIT, DANS L'ORDRE :
#   1. cherche le robot sur ses trois chemins connus, puis dans le cache ARP
#   2. aligne ~/.leo_network sur l'adresse trouvee
#   3. relance la pile de navigation puis la pile web (ordre impose)
#   4. verifie le pilotage de bout en bout, et dit lequel des maillons manque
# =============================================================================
set -u

RACINE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CANDIDATS=("10.0.0.1" "10.154.6.41" "10.200.0.2")

titre() { echo; echo "═══ $* ═══"; }
ok()    { echo "  ✓ $*"; }
ko()    { echo "  ✗ $*"; }

# --- 1. retrouver le robot ---------------------------------------------------
titre "1. Recherche du robot"
TROUVE=""
for a in "${CANDIDATS[@]}"; do
  printf "  %-14s " "$a"
  if ping -c 1 -W 2 "$a" >/dev/null 2>&1; then
    echo "REPOND"; TROUVE="$a"; break
  fi
  echo "muet"
done

# Le cache ARP voit les voisins du meme sous-reseau meme quand l'adresse a
# change : un Raspberry Pi s'y reconnait a son prefixe constructeur.
if [ -z "$TROUVE" ]; then
  echo "  -- recherche par prefixe Raspberry Pi dans le cache ARP --"
  while read -r ip _ _ _ mac _; do
    case "$mac" in
      b8:27:eb:*|dc:a6:32:*|e4:5f:01:*|28:cd:c1:*)
        printf "  %-16s (%s) " "$ip" "$mac"
        if ping -c 1 -W 2 "$ip" >/dev/null 2>&1; then echo "REPOND"; TROUVE="$ip"; break
        else echo "muet"; fi ;;
    esac
  done < <(ip neigh 2>/dev/null)
fi

if [ -z "$TROUVE" ]; then
  ko "robot introuvable sur les trois chemins connus."
  echo
  echo "  Le rover est-il allume et son point d'acces visible ?"
  echo "      nmcli dev wifi list | grep -i leo"
  echo "  Si l'AP est la mais la cle WPA2 inconnue, il n'existe plus de chemin"
  echo "  logiciel : il faut un acces physique (cable ethernet sur le Pi,"
  echo "  ecran+clavier, ou la carte SD dans un lecteur). Voir §D.7 du rapport."
  exit 1
fi
ok "robot joignable a $TROUVE"

# --- 2. aligner le point de verite d'adresse ---------------------------------
titre "2. ~/.leo_network"
AVANT="$(cat "$HOME/.leo_network" 2>/dev/null || echo '<absent>')"
echo "  avant : $AVANT"
echo "LEO_ROBOT_HOST=$TROUVE" > "$HOME/.leo_network"
echo "  apres : $(cat "$HOME/.leo_network")"

# --- 3. relancer, dans l'ordre -----------------------------------------------
# L'ordre n'est pas cosmetique : relancer le sanitizer seul laisse MINS sur une
# initialisation perimee, et la pile web a besoin du master du robot pour que
# rosbridge se lie.
titre "3. Relance de la pile"
touch /tmp/leo_maintenance          # suspend le watchdog pendant l'operation
bash "$RACINE/tools/restart_stack.sh" 2>&1 | sed 's/^/    /'
bash "$RACINE/web/start_web.sh"     2>&1 | sed 's/^/    /'
rm -f /tmp/leo_maintenance
ok "watchdog reactive"

# --- 4. verifier le PILOTAGE, pas la sante apparente -------------------------
titre "4. Verification du pilotage"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash 2>/dev/null
# shellcheck disable=SC1091
source "$RACINE/tools/robot_env.sh" 2>/dev/null
export ROS_MASTER_URI ROS_IP

PROBLEME=0

if timeout 8 rostopic list >/dev/null 2>&1; then
  ok "master ROS joignable ($ROS_MASTER_URI)"
else
  ko "master ROS injoignable — le robot repond au ping mais roscore est mort"
  echo "      ssh pi@$TROUVE 'sudo systemctl restart leo'"
  exit 1
fi

# LE test. Une telemetrie saine ne prouve PAS qu'une commande arrivera.
ABONNES="$(timeout 8 rostopic info /cmd_vel 2>/dev/null | sed -n '/Subscribers/,$p' | grep -c '^ \* ')"
if [ "${ABONNES:-0}" -ge 1 ]; then
  ok "/cmd_vel a $ABONNES abonne(s) — les commandes atteignent la carte moteur"
else
  ko "/cmd_vel SANS ABONNE : le cockpit enverra des commandes dans le vide"
  echo "      serial_node publie mais ne s'est pas reabonne (panne du 18/08)."
  echo "      ssh pi@$TROUVE 'sudo systemctl restart leo'   puis relancer ce script"
  PROBLEME=1
fi

# Batterie : cause connue de moteurs muets, a lire AVANT tout audit logiciel.
VOLT="$(timeout 6 rostopic echo -n1 /firmware/battery 2>/dev/null | grep -oE '[0-9]+\.[0-9]+' | head -1)"
if [ -n "$VOLT" ]; then
  if awk "BEGIN{exit !($VOLT < 10.3)}"; then
    ko "batterie $VOLT V — SOUS le seuil projet de 10,3 V"
    echo "      a 9,4 V le 02/09 les moteurs etaient muets alors que tout"
    echo "      le reste paraissait sain. Recharger avant de conclure."
    PROBLEME=1
  else
    ok "batterie $VOLT V"
  fi
else
  echo "  ? batterie illisible"
fi

for p in 9090 8000; do
  if ss -ltn 2>/dev/null | grep -q ":$p "; then ok "port $p ouvert"
  else ko "port $p ferme — le cockpit ne se connectera pas"; PROBLEME=1; fi
done

titre "Resultat"
if [ "$PROBLEME" -eq 0 ]; then
  echo "  PILOTAGE OPERATIONNEL"
  echo "    cockpit : http://localhost:8000/ops.html"
else
  echo "  PILOTAGE NON CONFIRME — voir les lignes ✗ ci-dessus."
  echo "  Ne pas conclure « le robot est casse » : chacune de ces pannes a"
  echo "  deja eu une cause banale et documentee."
fi

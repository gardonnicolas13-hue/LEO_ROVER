#!/usr/bin/env bash
# =============================================================================
# recuperer_robot_hotspot.sh — reprendre la main sur le robot quand il est
# ALLUME mais INJOIGNABLE, parce qu'il est retombe sur son point d'acces
# propre (SSID « LeoRover-e138 », adresse 10.0.0.1).
#
#   Usage :  bash tools/recuperer_robot_hotspot.sh            # diagnostique puis REVIENT sur le WiFi campus
#            bash tools/recuperer_robot_hotspot.sh --rester   # diagnostique et RESTE sur le robot
#
# -----------------------------------------------------------------------------
# POURQUOI CE SCRIPT EXISTE (2026-09-07)
#
# Le robot etait introuvable sur ses DEUX chemins habituels (WiFi campus
# 10.154.6.41 et WireGuard 10.200.0.2, 100 % de perte sur les deux). Le
# balayage des SSID a montre « LeoRover-e138 » a **signal 100/100** : le robot
# etait allume, a quelques metres, et avait bascule sur son repli AP faute de
# pouvoir rejoindre le WiFi du campus. Le tunnel WireGuard ne pouvait pas
# remonter non plus, par construction : c'est le ROBOT qui compose vers
# l'endpoint du PC, donc sans reseau cote robot, pas de handshake.
#
# LA CONTRAINTE QUI JUSTIFIE UN SCRIPT PLUTOT QU'UNE SUITE DE COMMANDES :
# ce PC n'a qu'UNE radio WiFi (wlx347de44fd621) et son port ethernet, bien que
# le cable soit branche (carrier=1), n'obtient AUCUN bail DHCP. Se connecter au
# point d'acces du robot coupe donc l'internet du PC pendant toute la duree de
# l'operation. Tout ce qui doit etre fait sur le robot doit donc etre PREVU
# D'AVANCE et s'executer sans supervision : d'ou ce script, qui bascule,
# diagnostique, journalise, repare si possible, et revient tout seul.
#
# CE QU'IL FAIT, DANS L'ORDRE :
#   1. memorise le profil WiFi actif pour pouvoir le restaurer
#   2. bascule la radio sur « LeoRover-e138 »
#   3. attend que 10.0.0.1 reponde
#   4. corrige ~/.leo_network, qui epinglait encore l'IP DHCP MORTE
#      (10.154.6.41) — exactement le piege documente dans robot_env.sh
#   5. collecte, par SSH, POURQUOI le robot a quitte le WiFi campus
#   6. tente de le faire re-rejoindre le WiFi campus
#   7. journalise tout dans un fichier lisible APRES coup
#   8. restaure le WiFi campus du PC (sauf --rester)
# =============================================================================
set -u

WIFI_IF="${WIFI_IF:-wlx347de44fd621}"
AP_PROFIL="${AP_PROFIL:-LeoRover-e138}"
AP_HOTE="${AP_HOTE:-10.0.0.1}"
CAMPUS_PROFIL="${CAMPUS_PROFIL:-FLTech-Guest 2}"
SSH_ID="${SSH_ID:-$HOME/.ssh/id_leo_tunnel}"
SSH_USER="${SSH_USER:-pi}"

RESTER=0
[ "${1:-}" = "--rester" ] && RESTER=1

JOURNAL="/tmp/recuperation_robot_$(date +%Y%m%d_%H%M%S).log"

# Tout passe par ici : la sortie doit survivre a la coupure d'internet, donc
# elle va SIMULTANEMENT a l'ecran et dans un fichier.
exec > >(tee -a "$JOURNAL") 2>&1

titre() { echo; echo "═══ $* ═══"; }

echo "Journal : $JOURNAL"
echo "Debut   : $(date -Is)"

# --- 1. memoriser l'etat de depart -------------------------------------------
titre "1. Etat de depart"
PROFIL_INITIAL="$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null \
                  | awk -F: -v i="$WIFI_IF" '$2==i {print $1}' | head -1)"
echo "Profil WiFi actif au demarrage : ${PROFIL_INITIAL:-<aucun>}"
[ -n "$PROFIL_INITIAL" ] && CAMPUS_PROFIL="$PROFIL_INITIAL"

# --- 2. basculer sur le point d'acces du robot -------------------------------
titre "2. Bascule vers « $AP_PROFIL »"
echo "  (l'internet de ce PC est coupe a partir d'ici)"
if ! nmcli device wifi connect "$AP_PROFIL" ifname "$WIFI_IF" 2>&1; then
  echo "ECHEC de l'association. Le robot diffuse-t-il encore ? Verifier :"
  echo "  nmcli dev wifi list | grep -i leo"
  exit 1
fi

# --- 3. attendre que le robot reponde ----------------------------------------
titre "3. Attente de $AP_HOTE"
JOIGNABLE=0
for i in $(seq 1 20); do
  if ping -c 1 -W 2 "$AP_HOTE" >/dev/null 2>&1; then
    echo "  repond apres ${i} tentative(s)"
    JOIGNABLE=1
    break
  fi
  sleep 2
done

if [ "$JOIGNABLE" -eq 0 ]; then
  echo "  INJOIGNABLE malgre l'association WiFi."
  echo "  => le robot diffuse son AP mais ne route pas : suspecter le Pi lui-meme"
  echo "     (carte SD pleine — deja vu le 21/07 —, ou leo.service mort)."
  [ "$RESTER" -eq 0 ] && nmcli connection up "$CAMPUS_PROFIL" ifname "$WIFI_IF" >/dev/null 2>&1
  exit 1
fi

# --- 4. corriger le point de verite d'adresse --------------------------------
titre "4. Correction de ~/.leo_network"
# robot_env.sh lit ce fichier pour construire ROS_MASTER_URI. Il epinglait
# 10.154.6.41, adresse DHCP qui n'appartient plus au robot : tant qu'elle y
# reste, TOUS les scripts du projet visent dans le vide, meme robot joignable.
if [ -f "$HOME/.leo_network" ]; then
  echo "  avant : $(cat "$HOME/.leo_network")"
  cp "$HOME/.leo_network" "$HOME/.leo_network.avant_recuperation"
fi
echo "LEO_ROBOT_HOST=$AP_HOTE" > "$HOME/.leo_network"
echo "  apres : $(cat "$HOME/.leo_network")"

SSH_OPTS="-i $SSH_ID -o IdentitiesOnly=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10"
sur_robot() { ssh $SSH_OPTS "${SSH_USER}@${AP_HOTE}" "$@" 2>&1; }

# --- 5. pourquoi a-t-il quitte le WiFi campus ? ------------------------------
titre "5. Diagnostic : pourquoi le repli AP ?"

echo "--- 5a. le robot voit-il encore le SSID du campus ? ---"
sur_robot "sudo iwlist wlan0 scan 2>/dev/null | grep -iE 'ESSID|Quality' | grep -iB1 -A0 'FLTech|Florida' | head -20 || echo '(scan indisponible)'"

echo "--- 5b. etat de wpa_supplicant / du service WiFi ---"
sur_robot "systemctl is-active wpa_supplicant 2>/dev/null; systemctl status wpa_supplicant --no-pager -n 15 2>/dev/null | tail -20"

echo "--- 5c. dernieres tentatives d'association (journal noyau) ---"
sur_robot "sudo journalctl -u wpa_supplicant --no-pager -n 40 2>/dev/null | tail -40 || dmesg | grep -iE 'wlan|auth|assoc|deauth' | tail -30"

echo "--- 5d. le reseau campus est-il configure cote robot ? ---"
sur_robot "sudo grep -iE 'ssid|key_mgmt|disabled' /etc/wpa_supplicant/wpa_supplicant.conf 2>/dev/null | sed 's/psk=.*/psk=<masque>/' || echo '(fichier illisible)'"

echo "--- 5e. ETAT DISQUE (cause connue de panne complete, 21/07) ---"
sur_robot "df -h / /var/log 2>/dev/null | head -5"

echo "--- 5f. batterie (seuil projet 10,3 V — cause connue de moteurs muets) ---"
sur_robot "source /opt/ros/noetic/setup.bash 2>/dev/null; timeout 5 rostopic echo -n1 /firmware/battery 2>/dev/null || echo '(ROS non interrogeable)'"

echo "--- 5g. pile ROS : que tourne-t-il ? ---"
sur_robot "systemctl is-active leo.service 2>/dev/null; ps aux | grep -E 'ros|mins|vins' | grep -v grep | awk '{print \$11, \$12, \$13}' | head -15"

# --- 6. tenter de le remettre sur le WiFi campus -----------------------------
titre "6. Tentative de retour sur le WiFi campus"
# On ne force RIEN de destructif : on demande juste a wpa_supplicant de
# reconsiderer ses reseaux. Si le SSID campus est configure et a portee, il
# doit se reassocier seul. Si ca marche, le robot redevient joignable par son
# chemin normal ET le tunnel WireGuard peut remonter.
sur_robot "sudo wpa_cli -i wlan0 reconfigure 2>/dev/null; sleep 3; sudo wpa_cli -i wlan0 reassociate 2>/dev/null; sleep 8; ip -brief addr show wlan0"

echo "--- adresse obtenue cote robot apres tentative ---"
sur_robot "ip -4 addr show wlan0 2>/dev/null | grep inet || echo '(toujours pas d adresse campus)'"

# --- 7. restauration du PC ---------------------------------------------------
if [ "$RESTER" -eq 1 ]; then
  titre "7. PC MAINTENU sur le point d'acces du robot (--rester)"
  echo "  Le robot est joignable a $AP_HOTE (ROS_MASTER_URI deja aligne)."
  echo "  Pour revenir au campus :  nmcli connection up '$CAMPUS_PROFIL' ifname $WIFI_IF"
else
  titre "7. Retour du PC sur « $CAMPUS_PROFIL »"
  nmcli connection up "$CAMPUS_PROFIL" ifname "$WIFI_IF" 2>&1 | tail -2
  sleep 4
  if ping -c 2 -W 2 1.1.1.1 >/dev/null 2>&1; then
    echo "  internet retabli"
  else
    echo "  ATTENTION : internet NON retabli. Reconnecter a la main :"
    echo "    nmcli connection up '$CAMPUS_PROFIL' ifname $WIFI_IF"
  fi
fi

echo
echo "Fin : $(date -Is)"
echo "═══════════════════════════════════════════════════════════════════"
echo " JOURNAL COMPLET : $JOURNAL"
echo " Le relire avec :  cat $JOURNAL"
echo "═══════════════════════════════════════════════════════════════════"

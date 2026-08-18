#!/usr/bin/env bash
# =============================================================================
# diag_reseau_robot.sh — diagnostic + correctifs reseau cote PC, quand le
# cockpit affiche « WS DISCONNECTED » et que le robot est injoignable.
#
# A LANCER AVEC SUDO :  sudo bash tools/diag_reseau_robot.sh
#
# Ecrit le 2026-08-18 apres une panne ou le robot etait allume mais
# introuvable depuis le PC. Ce que l'audit de ce jour-la avait etabli SANS
# sudo (et qui n'a donc pas besoin d'etre reverifie a la main) :
#   - le serveur web local et le tunnel Cloudflare repondaient 200 OK
#   - le pare-feu ufw etait desactive (donc hors de cause)
#   - le WiFi invite FLTech-Guest bloque le multicast : aucune decouverte
#     mDNS d'un client a l'autre n'est possible sur ce reseau
#   - l'ancienne IP WiFi du robot codee en dur dans ~/.ssh/config
#     (10.154.27.235) ne lui appartenait plus
# Les trois points ci-dessous sont les seuls qui exigeaient les droits root.
# =============================================================================
set -u

WIFI_IF="${WIFI_IF:-wlx347de44fd621}"   # surchargeable : WIFI_IF=... sudo -E bash ...

echo "═══════════════════════════════════════════════════════════════════"
echo " 1. WireGuard — etat reel du tunnel"
echo "═══════════════════════════════════════════════════════════════════"
# La ligne qui compte est « latest handshake ». Pas de handshake, ou un
# handshake vieux de plusieurs heures = le robot n'a jamais joint ce PC
# depuis son dernier demarrage : le probleme est en amont du tunnel.
wg show 2>&1 || echo "  (wg absent ou interface non montee)"

echo
echo "═══════════════════════════════════════════════════════════════════"
echo " 2. Config wg0 — les deux champs qui decident si le tunnel peut vivre"
echo "═══════════════════════════════════════════════════════════════════"
# ListenPort cote PC et Endpoint cote pair. L'annexe D du rapport documente
# le montage retenu ici : c'est le ROBOT qui porte l'Endpoint et pointe vers
# ce PC sur le port 51820/udp. Or l'audit du 18/08 a mesure, via
# /proc/net/udp, que le socket noyau WireGuard de ce PC ecoutait sur un port
# ALEATOIRE (33699 ce jour-la), pas sur 51820 — ce qui arrive quand
# ListenPort est absent du fichier. Un port aleatoire est retire au sort a
# CHAQUE montage de l'interface, donc a chaque redemarrage : les paquets du
# robot arrivent alors sur un port ferme et aucun handshake n'aboutit.
if [ -r /etc/wireguard/wg0.conf ]; then
  grep -vi 'privatekey' /etc/wireguard/wg0.conf   # jamais afficher la cle privee
  echo
  grep -qi '^[[:space:]]*ListenPort' /etc/wireguard/wg0.conf \
    && echo "  -> ListenPort present : OK" \
    || echo "  -> ListenPort ABSENT : port aleatoire a chaque montage. Ajouter"
  echo "     'ListenPort = 51820' sous [Interface], puis :"
  echo "     systemctl restart wg-quick@wg0"
else
  echo "  /etc/wireguard/wg0.conf illisible ou absent"
fi

echo
echo "═══════════════════════════════════════════════════════════════════"
echo " 3. Economie d'energie du dongle WiFi — correctif immediat + permanent"
echo "═══════════════════════════════════════════════════════════════════"
# Defaut connu du rtl88x2bu (annexe D) : il met l'antenne en veille entre les
# paquets et le lien devient intermittent. Le correctif `iw ... power_save
# off` NE SURVIT PAS a un redemarrage — verifie une fois de plus le 18/08,
# ou le reglage etait revenu a « on » apres le reboot de 16h05. D'ou le
# fichier NetworkManager ci-dessous, qui le rend definitif.
if ! ip link show "$WIFI_IF" >/dev/null 2>&1; then
  echo "  interface '$WIFI_IF' introuvable — corriger WIFI_IF en tete de script"
  echo "  interfaces WiFi presentes :"
  ip -br link show | grep -E '^wl' || echo "    (aucune)"
else
  echo "  avant : $(iw dev "$WIFI_IF" get power_save 2>&1)"
  iw dev "$WIFI_IF" set power_save off 2>&1
  echo "  apres : $(iw dev "$WIFI_IF" get power_save 2>&1)"

  NM_CONF=/etc/NetworkManager/conf.d/wifi-powersave-off.conf
  if [ -f "$NM_CONF" ]; then
    echo "  persistance : $NM_CONF existe deja"
  else
    printf '[connection]\nwifi.powersave = 2\n' > "$NM_CONF"
    echo "  persistance : $NM_CONF cree (2 = economie d'energie desactivee)"
    echo "                actif au prochain redemarrage, ou tout de suite via :"
    echo "                systemctl restart NetworkManager"
  fi
fi

echo
echo "═══════════════════════════════════════════════════════════════════"
echo " Rappel : ces trois points sont cote PC. Si le robot reste introuvable"
echo " apres ca, c'est qu'il n'est pas associe au reseau — a verifier sur le"
echo " robot lui-meme (nmcli device status / iwconfig / journalctl -u NetworkManager)."
echo "═══════════════════════════════════════════════════════════════════"

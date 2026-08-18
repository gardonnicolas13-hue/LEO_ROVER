#!/bin/bash
# Plafonne les journaux ROS du robot (2026-08-18, audit).
#
# POURQUOI : ce Pi a un disque de 30 Go et l'audit du 18/08 l'a trouve a 97 %,
# soit 934 Mo de marge. Un disque plein tue leo.service, et jusqu'a ce meme
# audit l'unite etait en Restart=no : la pile ne revenait pas. rosmon.log a
# lui seul pesait 783 Mo, sans aucune rotation (logrotate n'est pas installe
# sur cette machine).
#
# POURQUOI TRONQUER ET NON SUPPRIMER : le fichier est tenu ouvert en
# permanence par rosmon. Le supprimer ne rendrait pas l'espace tant que le
# processus vit. Verifie le 18/08 : le descripteur porte O_APPEND (flags
# 02402001), donc `truncate -s 0` est propre -- l'ecrivain reprend a la fin
# du fichier vide, sans trou creux ni espace fantome.
SEUIL_MO=100
JOURNAL=/var/log/cap_ros_logs.log

for f in /var/ros/log/rosmon.log /var/ros/log/*/rosout.log; do
  [ -f "$f" ] || continue
  mo=$(( $(stat -c%s "$f" 2>/dev/null || echo 0) / 1048576 ))
  if [ "$mo" -ge "$SEUIL_MO" ]; then
    truncate -s 0 "$f"
    echo "$(date '+%F %T') tronque $f (${mo} Mo)" >> "$JOURNAL"
  fi
done

# Le journal de ce script ne doit pas devenir le probleme qu'il resout.
[ -f "$JOURNAL" ] && [ "$(stat -c%s "$JOURNAL")" -gt 262144 ] && tail -n 200 "$JOURNAL" > "$JOURNAL.tmp" && mv "$JOURNAL.tmp" "$JOURNAL"
exit 0

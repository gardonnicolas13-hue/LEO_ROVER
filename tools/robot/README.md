# Fichiers déployés **sur le robot**, pas sur le PC

Ces quatre fichiers ne servent à rien depuis ce dépôt : ils vivent sur le Pi.
Ils sont versionnés ici parce qu'un `git clone` ne reconstruit pas un robot, et
qu'une réinstallation de la carte SD les perdrait sans trace — exactement le
piège déjà rencontré avec `config/leo/` d'openVINS (voir `.gitignore`, section
« Exception chirurgicale »).

Origine : audit du 2026-08-18. Le disque du robot avait atteint **97 %**, soit
934 Mo de marge, et `leo.service` était en `Restart=no` : un disque plein tue la
pile, et rien ne la relève. Les deux moitiés du problème sont traitées ici.

| Fichier | Destination sur le robot |
|---|---|
| `cap_ros_logs.sh` | `/usr/local/sbin/cap_ros_logs.sh` (`chmod +x`) |
| `cap-ros-logs.service` | `/etc/systemd/system/cap-ros-logs.service` |
| `cap-ros-logs.timer` | `/etc/systemd/system/cap-ros-logs.timer` |
| `leo.service.d-20-restart.conf` | `/etc/systemd/system/leo.service.d/20-restart.conf` |

## Réinstallation

```bash
scp tools/robot/cap_ros_logs.sh              leo-guest:/tmp/
scp tools/robot/cap-ros-logs.*               leo-guest:/tmp/
scp tools/robot/leo.service.d-20-restart.conf leo-guest:/tmp/

ssh leo-guest '
  sudo install -m 755 /tmp/cap_ros_logs.sh /usr/local/sbin/cap_ros_logs.sh
  sudo install -m 644 /tmp/cap-ros-logs.service /etc/systemd/system/
  sudo install -m 644 /tmp/cap-ros-logs.timer   /etc/systemd/system/
  sudo mkdir -p /etc/systemd/system/leo.service.d
  sudo install -m 644 /tmp/leo.service.d-20-restart.conf \
       /etc/systemd/system/leo.service.d/20-restart.conf
  sudo systemctl daemon-reload
  sudo systemctl enable --now cap-ros-logs.timer
'
```

## Deux pièges vérifiés sur place, à ne pas re-découvrir

**`cron` n'existe pas sur ce Pi.** `/usr/bin/crontab` est absent et
`cron.service` est inactive — une entrée crontab échoue silencieusement. Les
timers systemd, eux, y tournent déjà (`man-db`, `apt-daily`) : d'où le choix du
timer. Ne pas « simplifier » en revenant à cron.

**Tronquer, jamais supprimer.** `rosmon.log` est tenu ouvert en permanence par
`rosmon` ; le supprimer ne rendrait pas l'espace tant que le processus vit. Le
descripteur porte `O_APPEND` (flags `02402001`, vérifié le 18/08), donc
`truncate -s 0` est propre : l'écrivain reprend à la fin du fichier vide, sans
fichier creux ni espace fantôme.

## Vérifier que c'est actif

```bash
ssh leo-guest 'systemctl is-active cap-ros-logs.timer; \
               systemctl list-timers cap-ros-logs.timer --no-pager; \
               systemctl show leo.service -p Restart; \
               df -h / | tail -1'
```

Attendu : `active`, une prochaine échéance horaire, `Restart=on-failure`, et un
taux d'occupation qui ne remonte plus vers 90 %.

Le complément côté PC est dans `tools/leo_watchdog.sh` (garde en tête de
script) : mêmes raisons, mêmes précautions, seuil à 200 Mo.

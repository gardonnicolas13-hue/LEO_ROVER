# Structure de `TOUT/` — ce qui est à toi, ce qui ne l'est pas

Rangement du **2026-09-08**. Avant, 38 dossiers se mélangeaient à la racine :
ton code, des dépôts clonés, des essais de 2025, des installeurs et tes
photos personnelles, tous au même niveau. On ne pouvait pas répondre d'un
coup d'œil à la seule question qui compte : *qu'est-ce que j'ai écrit, moi ?*

Maintenant, **ce qui est à la racine est ton travail**. Le reste est isolé
dans trois dossiers préfixés `_`.

---

## Ton travail (racine)

| Dossier | Poids | Ce que c'est |
|---|---|---|
| `web/` | 86 Mo | Le cockpit opérateur. Entièrement écrit par toi. |
| `tools/` | 948 Ko | Tes scripts : watchdog, relances, préflight, MATLAB, capture rosbag. |
| `report_latex/` | 17 Mo | Les sources du rapport + `main.pdf`, le document de référence. |
| `catkin_ws/` | 11 Go | Tes paquets ROS (`leo_navigation`, `leo_autonomy`). Le volume vient des dépôts amont construits dedans, pas de ton code. |
| `data/` | 12 Go | Tes enregistrements et trajectoires exportées. |
| `calib_data/` | 2,4 Go | Ta campagne de calibration : bags D455, camchain, cibles AprilTag. |
| `docs/` | 72 Ko | Ta documentation technique et ta correspondance (Hector, Michael, RPNG). |
| `patches/`, `patches_mins/`, `patches_openvins/` | 152 Ko | Tes correctifs sur les dépôts amont, tracés hors de leurs arbres. |
| `config_limo_0903/` | 32 Ko | Ton instantané de configuration MINS du 03/09. |
| `logs/`, `outputs/`, `bin/` | 132 Mo | Tes journaux et sorties. |
| `archives/` | 56 Mo | Rangement de la racine (voir `archives/README.md`). |

**Espaces de travail actifs, laissés en place** — ils portent tes
modifications mais tournent en production, donc on n'y touche pas :

- `mins_ws/` (2,3 Go) — **MINS s'exécute depuis ici en ce moment**
  (`mins_ws/devel/.private/mins/lib/mins/subscribe`).
- `LEO_Rover_Navigation_System/` (368 Mo) — cible du lien symbolique
  `mins_ws/src/MINS`. Contient tes configs MINS gelées.
- `sqrtvins_ws/` (2,4 Go) — sqrtVINS avec tes correctifs ZUPT et tag.
- `kalibr_ws/` (1,3 Go) — référencé par `tools/calib_imucam_run.sh`.
- `carolus_ws/` (66 Mo) — référencé 3 fois dans tes scripts.

---

## Ce qui a été isolé

### `_amont/` — 1,9 Go, 7 dossiers
Code que tu n'as pas écrit : dépôts clonés et installeurs tiers.
`astrobee_ws`, `kalibr`, `matlab_installer`, `leo_firmware`, `mins-mc`,
`leorover`, `moveuvgs`. Ils se reclonent depuis leur dépôt d'origine ; ce
ne sont pas des sources de vérité de ce projet.

### `_obsolete/` — 6,7 Go, 8 dossiers
Essais abandonnés, **0 référence** dans tout le projet :
`test_carolus_ws` (avril 2026), `test2_carolus_ws` (**octobre 2025**),
`moveuvgsTemp`, `session2`, `AuHasard`, `calib_tentative_matlab`,
`leo_aruco_ws`, `snap (copy)`.

### `_personnel/` — 54 Go, 7 dossiers
Sans rapport avec le projet : `Bureau` (27 Go), `Downloads` (28 Go),
`Pictures`, `Documents`, `MATLAB-Drive`, `Matlab`, `snap`.
C'est **79 % du poids de `TOUT/`** — et rien de tout cela n'est le projet.

---

## La règle appliquée

> Rien d'actif, de référencé ou de suivi par git n'a été déplacé.

Vérifié avant chaque déplacement, pas après :

- **Processus en cours** — `mins_ws` (3 processus), `catkin_ws` (8) : gardés.
- **Références dans les scripts** — `kalibr_ws` (1), `carolus_ws` (3) :
  gardés. Les 22 autres dossiers déplacés en avaient **zéro**.
- **Suivi git** — aucun des dossiers déplacés n'était suivi (`git ls-files
  --error-unmatch`), donc l'historique est intact et aucun commit n'est
  nécessaire.

Contrôlé après coup : `leo_backend`, MINS et `pose_selector` vivants, site
en HTTP 200, aucun fichier suivi disparu.

**Rien n'a été supprimé.** Tout est déplaçable en sens inverse :

```bash
mv _personnel/Downloads .     # remettre un dossier à sa place
```

---

## Deux corrections faites en route

Deux dossiers que j'allais classer « à isoler » sont en fait **ton
travail**, et sont restés à la racine :

- **`calib_data/`** — ce n'est pas de la donnée jetable mais ta campagne de
  calibration (bags D455, camchain Kalibr, cibles AprilTag).
- **`config_limo_0903/`** — configuration MINS que tu as produite le 03/09.

Et **`kalibr_ws/`**, que sa taille désignait comme un dépôt amont, est
appelé par `tools/calib_imucam_run.sh` : le déplacer aurait cassé la
calibration IMU-caméra sans message d'erreur.

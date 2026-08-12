# Grille de vérification — exigences Pr. Hector Gutierrez

Chaque ligne a été vérifiée par lecture directe du fichier cité (pas par
mémoire d'une discussion antérieure). Un écart réel avec ce que le Technical
Guide décrivait jusqu'ici est signalé explicitement plutôt que masqué —
voir l'avertissement sous le point 1.

| # | Exigence | Chemin exact | Statut |
|---|---|---|---|
| 1 | TF2 : ancrage `beacon_link` et transformation optique → repère robot | `catkin_ws/src/leo_navigation/scripts/carolus_tf_bridge.py` | **Présent** (⚠️ voir note) |
| 2a | rosbag → CSV | `tools/bag_to_csv.py` | Présent |
| 2b | Comparaison Matlab (drive simultané) | `tools/plot_trajectories.m` | Présent |
| 2c | Comparaison Matlab (rectangle mesuré) | `tools/plot_rectangle_test.m` (201 lignes) | Présent |
| 3a | Pont VICON (Carolus → MINS) | `catkin_ws/src/leo_navigation/scripts/carolus_vicon_bridge.py` | Présent |
| 3b | Paramétrage VICON | `mins_ws/src/MINS/mins/config/leo/config_vicon.yaml` | Présent (`vicon.enabled: false`, documenté) |
| 4 | Centrage Carolus + instances PID | `leo_backend.py`, classe `PID` (~L1204), `_pid_lock_align`/`_pid_lock_approach` (~L1393-1397) | Présent |
| 5 | Arbitrage `_target_center()` + pixels Kalibr | `leo_backend.py`, `_target_center()` (~L4170), `_on_tag_detections()` (~L4121, `fx=336.372, cx=310.200`) | Présent |

## ⚠️ Correction nécessaire sur la Section 1.1 déjà rédigée

Le point 1 est présent, mais **sous un fichier et avec une logique différents
de l'exemple générique que je t'ai donné dans la Section 1.1** du guide. Ce
n'est pas un détail cosmétique — l'exemple que j'avais écrit casserait
réellement l'arbre TF sur ce robot :

- Mon exemple de Section 1.1 publiait une transformation statique
  `base_link -> camera_frame`, sur le modèle du document LIMO/Turki.
- Le vrai nœud de ce projet, `carolus_tf_bridge.py`, documente dans son
  propre en-tête que cette transformation **existe déjà** sur l'arbre TF
  réel de ce robot (`base_link -> camera_frame`, statique, vérifié le 30/07
  par écoute directe de `/tf` pendant 10 s), aux côtés de
  `odom -> base_link` (dynamique) et `base_footprint -> base_link`
  (statique). Republier la chaîne du document donnerait **deux parents** à
  `base_link` et casserait l'arbre — c'est explicitement pour ça que le
  nœud a un garde-fou `~publish_body_chain` (défaut `False`) qui bloque ce
  comportement par défaut, avec un `rospy.logerr` si on l'active sans
  vérification préalable.
- Deuxième écart : ce nœud n'est **déclaré dans aucun fichier `.launch`**
  (vérifié — aucune correspondance). Il est volontairement lancé à la main
  (`rosrun leo_navigation carolus_tf_bridge.py`), documenté ainsi dans son
  propre en-tête, précisément parce que ce nœud publie des TF et que cette
  plateforme a déjà connu un vrai conflit de TF (MINS volant le frame
  `imu` de l'URDF, d'où l'isolement sur `/tf_mins`, déjà documenté ailleurs
  dans le rapport).

**Recommandation :** remplacer l'exemple générique de la Section 1.1 par
une description de `carolus_tf_bridge.py` tel qu'il existe réellement — il
est d'ailleurs plus riche que mon exemple (service `~reanchor` pour
re-figer `beacon_link` sans relancer le nœud, balayage des 16 combinaisons
de signes de quaternion via le paramètre `~quat_signs` sans toucher au
code, garde-fou anti-collision d'arbre TF documenté). C'est un meilleur
contenu pour le guide, pas seulement une correction défensive. Dis-moi si
tu veux que je réécrive la 1.1 avec ça maintenant.

## Verdict global

5/5 exigences matérialisées par un fichier réel et fonctionnel. Aucune
absence. Un écart de contenu entre le guide déjà rédigé et le code réel a
été trouvé et signalé (point 1) plutôt que laissé tel quel.

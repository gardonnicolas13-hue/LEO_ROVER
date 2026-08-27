# Configuration sqrtVINS **embarquée**, et `/etc/ros/robot.launch`

Ces fichiers vivent sur la carte SD du robot. Ils sont versionnés ici pour la
même raison que le dossier parent : un `git clone` ne reconstruit pas un robot,
et une réécriture de carte les perdrait sans trace.

**Ce n'est pas théorique.** Le correctif de la garde χ² du ZUPT, appliqué au PC
le 21/08, n'a jamais atteint le robot — sa config vivait hors du dépôt. Il a
fallu le redécouvrir le 25/08 en lisant les journaux : `zupt_chi2_multipler: 0`
sur le Pi contre `1` sur le PC, avec un seuil affiché « chi2 < 0.000 » qui
laissait passer toutes les mises à jour.

| Fichier ici | Destination sur le robot |
|---|---|
| `../etc-ros-robot.launch` | `/etc/ros/robot.launch` |
| `config-leo-estimator_config.yaml` | `~/sqrtvins_ws/src/sqrtVINS/config/leo/estimator_config.yaml` |
| `config-leo-kalibr_imucam_chain.yaml` | `~/sqrtvins_ws/src/sqrtVINS/config/leo/kalibr_imucam_chain.yaml` |
| `config-leo-kalibr_imu_chain.yaml` | `~/sqrtvins_ws/src/sqrtVINS/config/leo/kalibr_imu_chain.yaml` |
| `ov_srvins-launch-sqrtvins_robot.launch` | `~/sqrtvins_ws/src/sqrtVINS/ov_srvins/launch/sqrtvins_robot.launch` |

## Ce que cet état contient, et pourquoi

**`enable_color` en argument, défaut `false`** (`/etc/ros/robot.launch`).
Le flux couleur 640×480@15 ne sert qu'à la détection de balise Carolus. Coupé
au niveau du PILOTE : nodelet caméra **133,4 % → 85,0 %** de CPU, idle global
**10,5 % → 34,4 %**, charge **7,16 → 4,53**. Les flux infra du VIO restent à
14,6 / 14,1 Hz.
À ne pas confondre avec l'essai du 28/07 qui coupait le REPUBLIEUR côté PC et
n'avait donné aucun gain : le coût est dans le décodage USB, pas dans la
compression JPEG. Pour un essai balise : `enable_color:=true`.

**`zupt_chi2_multipler: 1`** — était à `0`, donc la garde χ² du ZUPT était
DÉSACTIVÉE et le seuil s'affichait « < 0.000 ». Toute mise à jour de vitesse
nulle était acceptée, y compris absurde. Réactivée le 25/08 : seuil réel
≈ 51, vitesse estimée au repos passée de 0,138 à 0,012 m/s.

**`zupt_max_velocity: 1.5`** — et surtout PAS une valeur dérivée de la vitesse
physique du rover (0,264 m/s). Ce seuil gate la vitesse ESTIMÉE PAR LE FILTRE,
pas la vitesse réelle. L'avoir baissé à 0,15 le 25/08 a créé un verrou
circulaire : l'estimation dépasse le seuil, ZUPT est refusé, l'estimation
grimpe, et rien ne la ramène (0,139 → 0,507 m/s mesurés).

**`fast_threshold: 30`, `num_pts: 300`** — repassés à leur valeur d'origine le
27/08, **à l'encontre de l'allégement du 26/08 ci-dessus** (qui les avait
posés à 15/150 pour soulager un Pi bridé thermiquement). Motif : l'
instrumentation par étage `[MSCKF-ETAGES]` a montré qu'à 150/15 le SORTIE
moyen (features réellement utilisées par la mise à jour MSCKF) plafonnait à
0,68/cycle, et qu'en rotation le lacet divergeait totalement (−197,7° estimé
contre +5-12° réel). Revenir à 300/30 (aligné sur `config-leo_pc` et sur
openVINS, jamais l'inverse) a porté SORTIE à 1,12/cycle et éliminé tout saut
de position > 0,3 m sur 60 s de rotation active (52 sauts avant, dont
plusieurs de 5-10 m).
**Non résolu** : le compromis thermique du 26/08 n'a pas été re-mesuré après
ce retour en arrière. CPU observé pendant les essais de rotation qui ont
suivi : 58-70 % de charge (Pi non injoignable, pas de symptôme du 26/08) —
mais aucune mesure en régime long/chaud n'a été refaite spécifiquement à
300/30 sur ce point précis. À surveiller si le robot redevient injoignable
sous charge prolongée.

**`max_clones: 25`, `up_msckf_chi2_multipler: 5`, `fi_max_baseline: 200`**
(27/08) — même resynchronisation, mêmes valeurs déjà validées côté
`config-leo_pc`/openVINS depuis le 20-21/08 mais jamais reportées sur ce
profil embarqué. `max_clones` élargit la fenêtre de parallaxe disponible à un
rover lent (0,26 m/s max) ; `fi_max_baseline` desserre le gate SVD de
triangulation (défaut code 40 → 200) ; `up_msckf_chi2_multipler` desserre le
gate χ² MSCKF (defaut 1 → 5). Avec le rejet triangulation stage-relatif mesuré
à 47,2 % avant / 44,9 % après et le rejet jacobien (« Negative depth
detected ») à 42,2 % avant / 17,0 % après (population entrant dans chaque
étage), l'essentiel du gain vient de moins de triangulations mal
conditionnées, pas d'un chi² plus permissif.

**`up_msckf_sigma_px` : resté à `1.0`, délibérément** (28/08). Hypothèse
testée : le bruit pixel isotrope à 1,0 px sous-estime le résidu épipolaire
structurel du rig (0,89° / ~5,2 px, mesuré par calcul direct de `T_cam1_cam0`
— rig correctement calibré Kalibr, ce n'est pas une erreur de calibration
mais son désalignement physique réel), ce qui ferait rejeter au χ² des
features légitimes. Balayage 1,5 / 2,0 / 3,0 px, 60-75 s de rotation active
par palier : SORTIE monte bien de façon monotone (1,12 → 1,56 → 2,51 →
3,28/cycle) et le rejet χ² stage-relatif baisse (77,4 % → 60,7 % → 44,9 % →
30,7 %), mais LES TROIS valeurs testées réintroduisent des sauts de position
(20 à 56 sur la fenêtre, contre 0 à 1,0 px) et dégradent la cohérence du lacet
avec MINS/gyro (jusqu'à une inversion de signe complète à 2,0 px). Desserrer
le modèle de bruit fait accepter plus de résidu SANS le rendre moins bruité :
contrairement à `max_clones`/`fi_max_baseline` ci-dessus, ça ne change pas la
qualité géométrique des features, juste la confiance qu'on leur accorde.
Aucune des trois valeurs ne bat `1.0` sur l'objectif réel (stabilité en
rotation) ; conservé tel quel.

**`T_cam_imu` calibrée** (`kalibr_imucam_chain.yaml`). Le robot portait une
permutation d'axes ÉCRITE À LA MAIN — 9 coefficients sur 9 valant exactement
0 ou ±1, ce qu'une calibration Kalibr ne produit jamais. Écart avec la vraie
géométrie : **5,83° et 17,2 cm** de bras de levier. Conséquences mesurées :
`sin(5,83°) × 9,79 = 0,994 m/s²` de gravité qui fuit dans l'horizontale
(dérive en LIGNE DROITE, robot immobile), et `|ω × d| = 0,17 m/s` de vitesse
fantôme à 1 rad/s — l'ordre de grandeur de la vitesse max du rover.
Les intrinsèques et coefficients de distorsion sont IDENTIQUES à ceux du profil
PC au chiffre près : même caméra, même session de calibration. Seule
l'extrinsèque avait été remplacée.

**`node_name` défaut `sqrtvins`** et **`launch-prefix="nice -n 10"`**
(`sqrtvins_robot.launch`). Le nom canonique fait que ses topics sont
`/sqrtvins/...` que le nœud tourne sur le Pi ou sur le PC — sans quoi déplacer
le nœud casse l'onglet Trajectory, l'export MATLAB et le rééchantillonneur.
Le `nice` fait CÉDER sqrtVINS devant `serial_node` (SCHED_FIFO 25), le réseau
et la caméra : une pose qui arrive 20 ms plus tard reste exploitable, un octet
UART perdu ne se rattrape pas et fait tomber l'ancrage roues de MINS.

## Réinstallation

```bash
scp tools/robot/etc-ros-robot.launch leo-guest:/tmp/robot.launch
ssh leo-guest 'sudo install -m 644 /tmp/robot.launch /etc/ros/robot.launch'

D=~/sqrtvins_ws/src/sqrtVINS
scp tools/robot/sqrtvins/config-leo-estimator_config.yaml      leo-guest:$D/config/leo/estimator_config.yaml
scp tools/robot/sqrtvins/config-leo-kalibr_imucam_chain.yaml   leo-guest:$D/config/leo/kalibr_imucam_chain.yaml
scp tools/robot/sqrtvins/config-leo-kalibr_imu_chain.yaml      leo-guest:$D/config/leo/kalibr_imu_chain.yaml
scp tools/robot/sqrtvins/ov_srvins-launch-sqrtvins_robot.launch leo-guest:$D/ov_srvins/launch/sqrtvins_robot.launch
```

Les YAML sont relus par **OpenCV FileStorage**, pas par un analyseur YAML
standard : un commentaire en fin de valeur suivi de lignes indentées casse le
fichier sans message clair. Les commentaires doivent occuper leur propre ligne,
à l'indentation de la clé. Vérifier après toute édition :

```bash
python3 -c "import cv2,sys; fs=cv2.FileStorage(sys.argv[1],0); print(fs.isOpened())" <fichier>
```

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

**`fast_threshold: 15`** — était à 30, soit trois fois la valeur que le code
lui-même documente comme normale. Mesuré sur une image réelle du robot :
102 coins à 30, **454 à 15**, pour 300 features demandées. Le suiveur tournait
au tiers de sa capacité, sans réserve pour remplacer les pistes perdues en
virage.

**`num_pts: 150`** — abaissé depuis 300 le 26/08. Avec 454 coins disponibles il
reste une réserve de 3×, à la moitié du coût de suivi KLT. C'est le rééquilibrage
du point précédent, qui avait triplé la charge de suivi sur un Pi déjà bridé.

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

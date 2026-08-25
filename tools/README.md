# `tools/` — chaîne d'analyse des estimateurs

Chaque script porte sa documentation complète dans son en-tête. Ce fichier
documente ce qu'aucun en-tête ne peut dire : **l'ordre dans lequel ils
s'enchaînent**, et ce qu'on attend de chaque étape.

---

## Vérifier la chaîne, sans robot

```bash
bash tools/dry_run_pipeline.sh
```

Exerce les cinq maillons sur des données fabriquées et sort `0` si tout passe.
**À lancer après toute modification de la chaîne** — le pire moment pour
découvrir un maillon cassé, c'est juste après un roulage, robot présent et
batterie qui descend.

> Un dry-run vert prouve que la tuyauterie fonctionne. Il ne dit **rien** sur
> le robot ni sur les estimateurs : les données sont fausses par construction.

---

## Au retour du robot : essai de la recalibration gyro

Le mécanisme de recalibration périodique du biais gyro est **désactivé par
défaut**. Il vise la dérive mesurée en t³/t⁴ d'openVINS (voir plus bas).

### 1. Lancer la pile avec la recalibration active

```bash
roslaunch leo_navigation navigation_master.launch gyro_recal_period:=60 zupt_clamp_enable:=true
```

`zupt_clamp_enable` est facultatif (voir la section dédiée plus bas) mais
recommandé avec la recalibration : il rend les arrêts plus fiablement détectés
par openVINS lui-même, sur lesquels la recalibration gyro compte aussi.

Vérifier au démarrage que le nœud annonce les DEUX mécanismes :

```
[imu_sanitizer] recalibration gyro PÉRIODIQUE active (60 s, immobilité
                confirmée par /firmware/wheel_states)
[imu_sanitizer] détecteur ZUPT actif -> /imu_sanitizer/is_stationary (roues
                immobiles >= 3.0s ET écart-type accéléro <= 0.050 m/s^2 sur 1.0s)
```

Sans ces lignes, les paramètres n'ont pas pris — ne pas conduire pour rien.

### 2. Rouler, en ménageant des arrêts

Un tour de labo, **avec au moins deux arrêts de plus de 3 secondes**. Les trois
gardes sont volontairement sévères : sans arrêt franc, la recalibration ne se
déclenchera jamais et l'essai ne prouvera rien.

```bash
bash tools/record_trajectories.sh
```

### 3. Convertir, puis mesurer l'exposant

```bash
python3 tools/bag_to_csv.py <bag>
python3 tools/drift_exponent.py <préfixe>
```

### 4. Lire le résultat

| Exposant obtenu | Lecture |
|---|---|
| **n tombe de ~3 vers ~2** | La cause est confirmée. Le biais gyro ne domine plus ; il reste l'erreur d'échelle accéléromètre en dessous. |
| **n reste ~3** | La recalibration n'a pas mordu. Compter dans le journal du nœud les `recalib gyro APPLIQUÉE` contre les `REFUSÉE` — le plus probable est qu'aucun arrêt n'a duré 3 s. |
| **R² < 0,9** | L'ajustement est peu fiable, l'exposant ne veut pas dire grand-chose. Refaire avec un roulage plus long. |

### 5. Figure et métriques

```bash
matlab -batch "addpath('tools'); compare_estimators('<préfixe>')"
```

Produit dans `report_latex/figures/` : la figure, `report_metrics.tex`
(`\input`-able) et `metrics_summary.json`. Les exposants y sont joints
automatiquement — `compare_estimators.m` appelle `drift_exponent.py` plutôt que
de refaire le calcul, pour qu'il n'existe qu'en un seul exemplaire.

---

## Le clamp ZUPT (`zupt_clamp_enable`)

Ajouté le 2026-08-19 dans `imu_sanitizer.py`. **Désactivé par défaut** — comme
`gyro_recal_period`, ce chemin change ce que `/imu/data_clean` publie à chaque
arrêt confirmé, pour les deux estimateurs et pour la conduite, et n'est pas
encore validé sur le robot.

**Le problème qu'il adresse.** openVINS a déjà son propre ZUPT natif
(`try_zupt: true`, actif depuis le 28/07 dans
`catkin_ws/src/open_vins/config/leo/estimator_config.yaml`), déclenché par
SON test interne (chi², variance, disparité image) — mais ce test ne voit que
le bruit du capteur CORE2 ; il n'a aucun moyen de savoir que les ROUES
confirment un arrêt réel, et ce bruit suffit parfois à l'empêcher de se
déclencher à un arrêt pourtant bien réel.

**Ce qu'il fait.** Dès que `/imu_sanitizer/is_stationary` passe à vrai (roues
immobiles **ET** écart-type accéléro sous seuil — deux témoins cumulatifs, ni
l'un ni l'autre ne suffit seul, voir l'en-tête du script), `/imu/data_clean`
publie la MOYENNE de la fenêtre glissante au lieu de l'échantillon brut, tant
que l'arrêt dure. Ça ne fabrique rien : la moyenne n'est utilisée qu'une fois
que sa PROPRE variance a déjà validé le critère d'immobilité. Le signal brut
(`_last_good`) reste inchangé pour le repli anti-glitch — le clamp ne
s'applique qu'à ce qui sort vers les estimateurs.

**Ce qu'il ne fait PAS.** Il ne patch pas openVINS et ne force rien dans son
filtre de Kalman — il n'existe aucun point d'entrée externe pour ça sans
toucher au code source C++ (option envisagée puis écartée pour ce premier
essai, au profit de ce clamp, plus léger et réversible par un seul
paramètre). Il ne corrige pas non plus la dérive DÉJÀ accumulée pendant que
le robot roulait — c'est le rôle de `gyro_recal_period`. Le clamp est un
complément aux deux mécanismes existants (ZUPT natif d'openVINS + recalibration
gyro), pas un remplacement, et il n'est pas censé, à lui seul, faire tomber
l'exposant t³/t⁴.

**Vérifier qu'il se déclenche réellement**, à chaque transition (pas en
continu) :

```
[imu_sanitizer] ZUPT is_stationary -> True (roues=True accel=True)
```

Si cette ligne n'apparaît jamais pendant un arrêt tenu >3 s, le second témoin
ne valide pas — le robot vibre probablement plus que le seuil (surface,
moteurs en roue libre non freinée), pas la peine de chercher plus loin côté
roues.

**Paramètres associés** (tous dans `imu_sanitizer.py`, exposés en `<arg>` de
`navigation_master.launch` / `navigation_supervision.launch`) :

| Paramètre | Défaut | Rôle |
|---|---|---|
| `zupt_clamp_enable` | `false` | active/désactive le clamp lui-même |
| `zupt_accel_std_max` | `0.05` m/s² | seuil du second témoin (accéléro) — **non mesuré pour cet usage précis**, à resserrer après une vraie capture à l'arrêt |
| `zupt_window_s` | `1.0` s | fenêtre glissante de la moyenne / de l'écart-type |
| `wheel_still_time` | `3.0` s | déjà existant (recalibration gyro), réutilisé tel quel comme premier témoin |

**Lecture dans `drift_exponent.py`.** Le clamp n'entre pas dans le calcul de
l'exposant — il agit en amont, en donnant à openVINS et à la recalibration
gyro des arrêts plus fiablement reconnus. Un exposant qui descend avec
`zupt_clamp_enable:=true` mais pas sans lui indique que c'est le clamp qui a
fait la différence, pas la recalibration seule — utile à savoir pour un essai
ultérieur qui isolerait les deux mécanismes séparément.

---

## Recalibration de l'échelle accéléro (`accel_recal_period`)

Ajoutée le 2026-08-20 dans `imu_sanitizer.py`, après l'essai propre du soir
même : `gyro_recal_period` + `zupt_clamp_enable` ont fait tomber l'exposant de
t³·²⁵ à **n≈0,74** (R²=0,94) — le biais gyro est neutralisé, mais 545 m de
dérive en 10 min restent, avec la signature n~1 d'une **erreur de vitesse**.
`~accel_scale` (facteur unique 1,1186) n'a été mesuré **qu'une fois**, le
28/07 — exactement le défaut déjà corrigé côté gyro.

**Pourquoi c'est sûr, exactement comme pour le gyro** : `|a|` à l'arrêt vaut
la gravité locale, une **norme**, invariante par rotation — même argument que
« le taux gyro vrai à l'arrêt est nul, quelle que soit l'orientation ». Réutilise
directement `is_stationary` (roues + variance déjà vérifiées), pas de nouvelle
garde à dupliquer.

**Ce qu'elle ne fait PAS** : un biais 3 axes. Contrairement au gyro, la valeur
vraie de l'accéléro à l'arrêt dépend de l'orientation — et ce rover roule
toujours à peu près à plat. Échantillonner répétitivement à la même
inclinaison ne peut pas distinguer un vrai défaut capteur d'un désalignement
de montage ou d'un sol simplement pas parfaitement horizontal — la même limite
déjà documentée pour la mesure `accel_scale` à une seule orientation. Une
vraie calibration de biais exige plusieurs orientations (voir
`tools/calib_terrain.py`), pas un ajout à `is_stationary`.

```bash
roslaunch leo_navigation navigation_master.launch \
    gyro_recal_period:=60 zupt_clamp_enable:=true accel_recal_period:=60
```

`~accel_recal_max_delta` (défaut 3 % relatif) — **non mesuré**, à resserrer
après une vraie campagne longue durée. Testé hors ligne (16 cas,
`tools/test_gyro_recal_guards.py`), **pas encore validé sur le robot**.

---

## Le verrou ZUPT d'openVINS (`zupt_max_velocity`)

Porté de `0.1` à **`1.5`** le 2026-08-20 dans
`catkin_ws/src/open_vins/config/leo/estimator_config.yaml`.

**Ce qui a été mesuré**, robot vérifié immobile (gyro 0,0003 rad/s, roues
nulles, `is_stationary = true`) :

| sur 45 s à l'arrêt | openVINS | MINS |
|---|---|---|
| déplacement net | 0,014 m | 0,055 m |
| chemin cumulé | **95,4 m** | 0,361 m |
| vitesse estimée | **1,295 m/s** | — |

**Le verrou.** `zupt_max_velocity` est le plafond de vitesse *estimée* sous
lequel openVINS accepte d'appliquer sa correction de vitesse nulle. À `0.1`,
avec une vitesse estimée à 1,295 m/s, le ZUPT ne pouvait **jamais** se
déclencher — circulaire : la vitesse estimée est fausse → le ZUPT refuse →
rien ne la corrige → elle reste fausse. Le robot n'allait nulle part, mais
openVINS accumulait 2 m/s de bruit permanent, la caméra (scène fixe) tirant
en sens inverse à chaque mise à jour.

**L'ordre de grandeur s'explique** : le résidu accéléromètre de ~1,06 m/s²
(mesuré le 28/07) intégré sur ~1,2 s donne 1,25 m/s. C'est bien le même
défaut amont qui ressort ici.

**Ce qui protège encore.** Les autres gardes sont inchangées et restent le
vrai filet : `zupt_max_disparity: 0.5` exige une image quasi fixe, et le test
chi². Un robot qui roulerait réellement ne passerait pas la garde de
disparité — il roule d'ailleurs à 0,2 m/s, très en dessous de 1,5.

**À resserrer** une fois le biais accéléromètre corrigé (`~accel_recal_period`
dans `imu_sanitizer.py`, écrit et testé, désactivé par défaut). Ce seuil
compense un défaut amont, il ne le corrige pas.

> **Piège à ne pas rediscovrir** : ce fichier est lu par le parseur OpenCV
> FileStorage, pas par PyYAML. Un commentaire long y a déjà provoqué un crash
> au démarrage (1324 occurrences, cf. la note sur le crash mutex). Les
> commentaires y restent donc courts — le raisonnement vit ici.

---

## Diagnostiquer openVINS : par où commencer

Session du 2026-08-20, six hypothèses testées. **Le seul indicateur fiable de
santé est le nombre de features RÉELLEMENT CONSOMMÉES** par le filtre :

```bash
rostopic echo /ov_msckf/points_msckf   # la LARGEUR du nuage = features utilisées
```

Il en faut ~40 (`max_msckf_in_update`). Mesuré sur ce robot : **0,75 en
moyenne**. L'apparence du tracé, le nombre de points dessinés dans
`/ov_msckf/trackhist` (~11 850 pixels !) et l'absence d'erreur dans les logs
ne prouvent **rien** — l'extracteur peut travailler pendant que le filtre
rejette tout.

**Ordre de diagnostic, du moins cher au plus cher :**

| Étape | Commande / critère | Ce que ça élimine |
|---|---|---|
| 1 | `\|a\|` au repos sur `/imu/data_clean` — écart à 9,790 > 0,02 ? | échelle accéléro périmée |
| 2 | `\|v\|` openVINS à l'arrêt vs `zupt_max_velocity` | verrou ZUPT |
| 3 | largeur de `points_msckf` en **mouvement** | canal visuel |
| 4 | `T_cn_cnm1` de Kalibr vs géométrie implicite de la config | appariement stéréo |

**Ce qui a été éliminé par la mesure** (ne pas y repasser sans raison neuve) :
scène sans texture, extrinsèque caméra-IMU globale, seuil χ², parallaxe de
translation. Détails et chiffres dans les notes de session.

> **Piège** : `up_msckf_chi2_multipler` desserré de 1 à 5 n'a rien donné
> (0,58 → 0,75). Si le rejet se produit à l'**appariement stéréo**, il est en
> amont du χ² et aucun réglage de ce seuil n'y changera quoi que ce soit.

---

## Supervision en direct

```bash
python3 tools/watch_vins_live.py            # ne parle que sur anomalie
python3 tools/watch_vins_live.py --zmax 0.5 # garde verticale plus stricte
```

Surveille : vitesse fantôme roues immobiles, sauts de position, écart
VINS/MINS par paliers, features consommées en mouvement, et **l'axe Z de
MINS**.

> **Pourquoi la garde Z existe** : MINS n'a **aucune** contrainte verticale.
> Ses roues tiennent X et Y, la caméra tient Z. Une extrinsèque caméra
> modifiée l'a fait passer à **−25,9 m en Z** pendant que X et Y restaient
> parfaits — sans qu'aucune alarme ne se déclenche. Vérifier cet axe après
> toute modification de géométrie caméra.

---

## Piloter la pile depuis le cockpit

Les estimateurs ne lisent leur configuration **qu'au démarrage**. Écrire une
extrinsèque depuis `web/tf_validator.html` ne change donc rien tant que la
pile n'est pas relancée.

```bash
python3 tools/tf_save_server.py    # 127.0.0.1:8010, démarré par web/start_web.sh
```

Deux routes : `/save_tf` (écrit les repères) et `/reload_stack` (relance via
`restart_stack.sh`). Le bouton **RELANCER LA PILE** de la page les utilise et
relit le disque 25 s après, pour que l'affichage montre ce qui est
**réellement** chargé.

> **Boucle locale uniquement, par conception.** Le port 8000 est publié sur
> internet par le tunnel Cloudflare ; une route d'écriture y serait ouverte à
> tous, et filtrer par IP ne protégerait de rien puisque cloudflared se
> connecte lui-même en 127.0.0.1. Conséquence assumée : la sauvegarde et la
> relance **échouent à distance**.

---

## Ce que l'exposant signifie

`drift_exponent.py` ajuste `d(t) ~ t^n` en log-log. L'exposant dit **quelle
erreur domine**, ce qu'aucune distance finale ne dit :

| n | Cause dominante |
|---|---|
| ~1 | erreur de vitesse |
| ~2 | **accéléromètre** — biais ou erreur d'échelle |
| ~3 | **gyroscope** — biais constant non compensé |
| ≥4 | **gyroscope** — biais qui dérive lui-même |

Mesuré le 2026-08-18 sur le roulage du 13/08 : MINS `t^-0,01` (plat, ses roues
l'ancrent), openVINS `t^3,25` et `t^4,26`. C'est ce qui a désigné le gyroscope
plutôt que l'accéléromètre.

---

## Les scripts, par rôle

| Script | Rôle |
|---|---|
| `record_trajectories.sh` | capture rosbag (les 3 estimateurs) |
| `bag_to_csv.py` | bag → CSV, un par estimateur |
| `drift_exponent.py` | exposant de dérive, `--json` pour MATLAB |
| `compare_estimators.m` | 3 estimateurs sur **un** enregistrement, + exports |
| `compare_tests.m` | **deux roulages** différents — à ne pas confondre |
| `make_mock_srv.py` | série synthétique, pour tester hors robot |
| `test_gyro_recal_guards.py` | gardes de la recalibration gyro (sans ROS) |
| `dry_run_pipeline.sh` | exerce tout ce qui précède, hors ligne |

---

## Deux pièges à ne pas re-découvrir

**`compare_tests.m` et `compare_estimators.m` ne font pas la même chose.** Le
premier compare deux **roulages** (deux moments, deux trajectoires) ; le second
compare trois **estimateurs** sur un **seul** enregistrement. Seul le second
autorise à dire qu'un estimateur est meilleur qu'un autre : les conditions y
sont identiques par construction. Ne pas les fusionner.

**Les données synthétiques ne doivent jamais être citées.** `make_mock_srv.py`
refuse d'écrire dans `data/trajectories/`, préfixe ses fichiers par `MOCK_` et
dépose un témoin `.SYNTHETIC.txt`. En aval, tout est marqué `[MOCK]` dans le
tableau LaTeX et `"synthetic": true` dans le JSON. Le marquage est volontairement
**large** : un lot de test voit toutes ses séries marquées, même celles copiées
de vraies mesures. Un `[MOCK]` de trop coûte une seconde de perplexité ; un
`[MOCK]` manquant met un chiffre fabriqué dans un rapport.

## Auditer le jitter IMU d'openVINS (`tools/audit_imu_jitter.py`)

Recommandation de Patrick Geneva (doc openVINS, « filter tuning ») : un `dt`
IMU irrégulier décale la fenêtre de clones par rapport aux images, et une
feature triangulée contre des poses mal datées tombe hors du seuil chi2 **sans
que rien ne signale la cause**.

```bash
python3 tools/audit_imu_jitter.py 100 /imu/data_clean
```

Le script mesure sur les timestamps du **header** (temps capteur — le seul qui
entre dans le filtre), pas sur l'heure d'arrivée, qui mélange le jitter WiFi au
jitter capteur. Il compare quand même les deux, pour situer.

**Mesure du 2026-08-21** (100 s, robot immobile, 8431 échantillons) :

| grandeur | temps capteur | temps d'arrivée |
|---|---|---|
| cadence | 90,9 Hz (dt médian 11,0 ms) | 88,2 Hz |
| **jitter (sigma)** | **4,4 ms = 40,4 % du dt** | 7,2 ms = 63,3 % |
| dt min / max | 3,5 / 49,0 ms | 0,0 / 73,6 ms |
| trous (>3x médian) | 26 (0,31 %) | 117 (1,39 %) |
| reculs / doublons | **0 / 0** | 0 / 0 |

Lecture : **aucun recul temporel ni doublon** — le flux est ordonné, donc pas
de cause fatale de ce côté. Mais **40 % de jitter sur le temps capteur** est
élevé (le dt varie de 3,5 à 49 ms pour une cadence nominale de 11 ms). Ce
n'est pas du bruit réseau : le temps d'arrivée est mesuré séparément et se
dégrade davantage (63 %), ce qui confirme que les 40 % viennent bien d'en
amont, du côté CORE2.

## Piège de méthode : la mesure de features openVINS est BIMODALE

À configuration **strictement identique**, `points_msckf` donne soit 0 message
soit ~1,4 feature de moyenne, selon le respawn (mesuré le 2026-08-21 : essais
3 et 4 même config, 0 contre 1350 mises à jour). Conséquence : **ne jamais
conclure sur une relance unique**. Et au repos avec `try_zupt: true`,
`VioManager.cpp:299-305` fait un `return` immédiat qui saute toute la mise à
jour caméra — zéro feature y est le comportement NORMAL, pas un symptôme.
Toute mesure qui compte doit se faire **robot en mouvement**.

## sqrtVINS (`ov_srvins`) — troisième estimateur, build PC double précision

### Ce qui existait déjà, et ce qui manquait

`sqrtVINS` ([rpng/sqrtVINS](https://github.com/rpng/sqrtVINS), paquet
`ov_srvins`) était déjà cloné, compilé et testé **sur le Raspberry Pi**
(`/home/pi/sqrtvins_ws`, commit `30fafc8`). Il divergeait sans borne —
43 742 m sur un tour de labo d'environ 20 m, et jusqu'à 3,4e17 m à l'arrêt
même après le correctif ZUPT décrit dans
`docs/rpng_sqrtvins_zupt_bug_report.md`.

**Cause trouvée le 2026-08-21, au niveau du système de build :**

```
ov_core/cmake/ROS1.cmake:24     option(USE_FLOAT "Use float version when built" ON)
build/ov_srvins/CMakeCache.txt  USE_FLOAT:BOOL=ON          (sur le Pi)
ov_core/src/utils/DataType.h    #if USE_FLOAT → typedef float DataType
```

`DataType` est le type dont dérivent **tous** les `Vec2/3/4`, `Mat2/3/4`,
`VecX` et **`MatX`** — donc le facteur de covariance en racine carrée
lui-même. Le filtre entier tournait en **float32, ~7 chiffres significatifs**.

C'est l'ironie du dossier : un filtre à racine carrée existe précisément pour
**diviser par deux le conditionnement numérique** (il propage le facteur de
Cholesky R, avec P = RᵀR, au lieu de P). Le compiler en simple précision lui
reprend exactement l'avantage pour lequel on le choisit. Une position à 1e15 m
en float32 n'est pas un réglage à ajuster : c'est un effondrement par
annulation catastrophique.

### Le build PC

```bash
cd /home/lab272/TOUT/sqrtvins_ws
catkin config --extend /opt/ros/noetic \
              --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_FLOAT=OFF
catkin build
```

Deux points non négociables :

- **`--extend /opt/ros/noetic` UNIQUEMENT.** sqrtVINS embarque son *propre*
  `ov_core`, homonyme de celui d'openVINS dans `catkin_ws`. Superposer les
  deux workspaces rend le paquet ambigu et fait résoudre headers et
  bibliothèques au hasard de l'ordre de superposition — un bug silencieux,
  visible seulement à l'exécution.
- **`-DUSE_FLOAT=OFF` en argument, pas en modification du source.** `option()`
  accepte une valeur en ligne de commande : le source amont reste intact,
  donc directement comparable au build Pi.

Vérification (`USE_FLOAT:BOOL=OFF` dans le cache, et `-DUSE_FLOAT=1` absent
de tous les `flags.make`) : le script de lancement la refait à chaque
démarrage et prévient si le workspace a été recompilé en simple précision.

### Lancer

```bash
tools/launch_sqrtvins.sh                  # profil PC
tools/launch_sqrtvins.sh verbosity:=INFO  # arguments roslaunch relayés
```

Le script ne source QUE `sqrtvins_ws` — il ne connaît ni `mins_ws` ni
`catkin_ws`, ce qui garantit que le sanctuaire MINS n'est jamais effleuré.

| | MINS | openVINS | sqrtVINS |
|---|---|---|---|
| nœud | `mins_subscribe` | `ov_msckf` | **`sqrtvins`** |
| odométrie | `/mins/imu/odom` | `/ov_msckf/odomimu` | **`/sqrtvins/odomimu`** |
| features | — | `/ov_msckf/points_msckf` | **`/sqrtvins/points_msckf`** |
| TF | `/tf_mins` | `/tf_vins` | **`/tf_sqrtvins`** |

Le nom `sqrtvins` (et non `ov_srvins`) est délibéré : l'instance du Pi
s'appelle déjà `ov_srvins` et partage le **même roscore**. Deux nœuds
homonymes sur un master — le second tue le premier.

**Entrées identiques aux deux autres** (`/pc/camera/infra{1,2}/image_rect_raw`
et `/imu/data_clean`, config `config/leo_pc/`) : à entrée différente, un écart
de sortie ne prouverait rien sur l'estimateur. La config `config/leo/` du Pi
est conservée intacte à côté, pour que la comparaison Pi/PC reste possible.

### Initialisation : il faut BOUGER

`init_dyn_use: false` → initialisation **statique**, qui attend une secousse.
Robot immobile, le nœud le dit lui-même :

```
[init]: (zvupt: false, dyna init: false) no jerk detected, platform is stationary
[Disparity Check]: window time: 1.010 sec, uv disparity is 0.098 (10.00 static thres)
```

Ce n'est pas une panne : `/sqrtvins/odomimu` reste muet **par conception**
tant que le robot n'a pas bougé. Toute mesure de sqrtVINS demande donc le
robot en mouvement — comme celle d'openVINS, pour une raison différente
(`try_zupt` y saute la mise à jour caméra à l'arrêt).

## Les trois modes : commutation, visualisation, export (2026-08-21)

### Commuter — `leo_navigation/SetPoseSource`

`std_srvs/SetBool` ne pouvait porter que DEUX états. Un service typé a été
ajouté ; **l'ancien est conservé intact** (5 appelants : `leo_backend.py`,
`restart_stack.sh`, `trajectory.html`, la doc ×2).

```bash
rosservice call /pose_selector/set_source_by_name "source: 'SQRTVINS'"
```

```
string source          →  bool success / string message / string active / string pending
```

Insensible à la casse. Un nom inconnu est refusé **avec la liste des noms
valides**, sans corrompre la source active. Testé de bout en bout depuis
`/mission/command`, le chemin exact du cockpit : 4 bascules sur 4.

`pose_selector.py` dérive désormais tout de son tuple `SOURCES` — ajouter un
4ᵉ estimateur ne demande qu'une entrée et un paramètre `~<nom>_topic`.

**Repli sûr** : si `leo_navigation` n'a pas été recompilé, le service typé est
absent ; `leo_backend` **refuse alors explicitement** `SQRTVINS` au lieu de le
traduire silencieusement en `VINS` — ce qu'aurait fait `data=(source=="MINS")`.
Le cockpit grise le bouton et dit pourquoi.

| | MINS | openVINS | sqrtVINS |
|---|---|---|---|
| couleur | orange `#ff6b35` | cyan `#22d3ee` | **ambre `#fbbf24`** |
| odométrie | `/mins/imu/odom` | `/ov_msckf/odomimu` | `/sqrtvins/odomimu` |
| TF | `/tf_mins` | `/tf_vins` | `/tf_sqrtvins` |

### Visualiser

- **TF Validator** : troisième panneau sqrtVINS. `export_tf_frames.py` lit
  maintenant les trois camchains, et la cohérence est vérifiée pour chacun
  **contre MINS** pris comme référence. Mesuré : écart `0.00e+00` partout —
  les trois décrivent exactement la même géométrie.
- **Trajectory** : la page ne suivait que `/robot_pose_fused`, c'est-à-dire la
  seule source ACTIVE — on ne pouvait donc pas comparer. Elle s'abonne
  désormais aux trois topics bruts, tracés en surimpression (trait fin
  pointillé + point de tête). Ils sont gardés **séparés** de l'historique
  fusionné : celui-ci porte les corrections SE(3) appliquées aux bascules, les
  mélanger ferait croire à une concordance qui n'existe pas.

### Exporter vers MATLAB — `tools/export_matlab.py`

```bash
python3 tools/export_matlab.py 120 nom_essai          # .mat + CSV
python3 tools/export_matlab.py 120 nom_essai --csv-seulement
```

Depuis le cockpit : action `export_3modes` (`duree`, `nom`) sur
`/mission/command`. Elle lance le script en **sous-processus détaché**, pour
que la capture survive à un redémarrage du backend.

**Distinct de l'export `export_matlab` existant**, qui exporte le tampon de
`leo_backend` — vidé à chaque relance de la pile et transitant par rosbridge.
Celui-ci s'abonne directement aux topics : ni navigateur, ni backend, ni tunnel.

Contenu par estimateur : `t` (horodatage **capteur**), `p`, `q`, `v`, `w`,
`P_pose` et `P_twist` (6×6 aplaties en 36 colonnes), `sigma3`, et pour les deux
VINS `feat_t`/`feat_n` (features réellement consommées). Plus `roues`, le
témoin indépendant qui dit si le robot bougeait vraiment.

**Piège de covariance, à ne pas rater :**

```matlab
P = reshape(S.MINS.P_pose(k,:), 6, 6)';   % la transposée est OBLIGATOIRE
```

ROS écrit en ligne-major, MATLAB lit en colonne-major. Sans la transposée on
obtient la transposée de la vraie matrice — **qui reste symétrique en
apparence**, donc l'erreur ne se voit pas. Vérifié sur le fichier produit :
`|P − Pᵀ|max ≈ 1e-17`.

**Piège attrapé à l'écriture** : les callbacks ROS continuent d'alimenter les
listes pendant l'écriture des fichiers (constaté : `t` à 3413 contre `P_pose`
à 3412, `np.hstack` refuse). `fige()` tronque tout à la longueur du plus court
**avant** toute lecture — désaligner les lignes aurait fait correspondre la
covariance `k` au mauvais instant, sans qu'aucune erreur ne le signale.

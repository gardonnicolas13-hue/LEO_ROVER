# Passation — prochain·e étudiant·e sur le LEO Rover

*Préparé le 2026-09-11 par Nicolas Gardon (Florida Institute of Technology).
Point d'entrée volontairement synthétique : chaque item cite son numéro dans
le **registre des points ouverts** (`report_latex/Fusion_Campaign.tex`,
`\label{tab:registre}`, 50 entrées) pour le détail complet — mesures,
citations de code, tentatives déjà faites. Ne rien re-tenter de ce qui est
listé comme déjà essayé sans lire l'entrée correspondante d'abord.*

**À lire avant tout** : `README.md` (architecture), puis l'annexe D de
`report_latex/main.pdf` (reproduction pas-à-pas depuis une machine vierge).
Le rapport (586 pages) documente ses limites aussi franchement que ses
résultats — ce n'est pas un document à parcourir en diagonale, les deux
chantiers prioritaires ci-dessous s'y trouvent développés en plusieurs pages
chacun.

---

## Chantier 1 — Fiabiliser sqrtVINS (priorité demandée)

**État exact (item 50 du registre, §sec:sqrtvins)** : tourne en direct sur le
Pi à 30–70 Hz sur données capteur réelles. Un bug a été trouvé, corrigé et
**vérifié en conditions réelles** :

- Cause : `UpdaterZeroVelocity.cpp` — un `LLT().solve()` non vérifié plus une
  condition de rejet que le chemin disparity-only contourne
  inconditionnellement à l'arrêt.
- Correctif : `docs/sqrtvins_zupt_numerical_guard.patch` (64 lignes,
  garde de validité numérique). Testé 60 s en direct : 588 rejets corrects,
  241 acceptations correctes, zéro χ² `inf`/aberrant accepté. **Le bug
  rapporté ne se reproduit plus.**
- Rapport de bug prêt pour RPNG (auteurs d'openVINS/sqrtVINS) :
  `docs/rpng_sqrtvins_zupt_bug_report.md`, mis à jour avec le correctif,
  **pas encore soumis** — à faire, c'est un gain rapide et redonne de la
  visibilité au projet en amont.

**Ce qui reste (le vrai chantier)** : sur ce même run, correctif actif, le
filtre **diverge quand même**, en douceur, $10^{15} \to 3{,}4 \times
10^{17}$ m en 60 s. Un **second** défaut numérique survit en dehors de
l'updater ZUPT. Personne n'a encore ouvert `Propagator.cpp` ni
`UpdaterMSCKF.cpp` avec la même discipline que celle qui a trouvé le premier
bug (lecture ligne à ligne contre le calcul mathématique attendu, pas
supposition).

**Méthode imposée par l'item 41 lui-même** : condition *à l'arrêt d'abord* —
ne pas lancer de test en roulage tant que la divergence statique n'est pas
comprise. Un run roulant sur un filtre qui diverge déjà à l'arrêt ne
produira que du bruit non interprétable.

**Premiers pas concrets** :
1. Soumettre le rapport de bug à RPNG (correctif déjà rédigé).
2. Lire `Propagator.cpp` et `UpdaterMSCKF.cpp` en cherchant le même type de
   défaut (solve non vérifié, division silencieuse, matrice non
   symétrisée) — c'est très probablement une garde numérique manquante du
   même genre, pas un problème de tuning.
3. Reproduire la divergence à l'arrêt en isolant propagation vs update
   (ex. désactiver temporairement l'update caméra pour voir si la
   propagation seule diverge déjà).

---

## Chantier 2 — Remplacer l'IMU, le tester, le comparer sur toutes les navigations (priorité demandée)

**Pourquoi** : l'accéléromètre CORE2 lit hors-spec (~-10,6 % sur $|g|$),
c'est une erreur d'échelle qui dérive sur des semaines — voir la mémoire de
session et item 45 du registre pour la caractérisation complète.

### Deux pistes explorées — n'en relancer qu'une

**Piste A — IMU du D455 (bloquée, ne pas retenter sans plan)**
Présence matérielle confirmée à 4 niveaux (descripteur USB, modules noyau
`hid_sensor_accel_3d`/`gyro_3d`, nœuds IIO, log driver). Mais **l'activer
fait perdre les 3 flux caméra** : le nœud RealSense n'atteint jamais
`RealSense Node Is Up` et boucle en restart, avec ou sans
`unite_imu_method`. Bande passante USB et le warning `libusb` déjà écartés
par la mesure. Voir `docs/hector_dual_imu_reply.md` pour l'historique complet
envoyé au Pr. Gutierrez. **Ne pas re-belier tant qu'un spike timeboxé**
(version librealsense → binding HID noyau → firmware caméra, dans cet ordre)
**n'a pas été planifié** — c'est maintenant le chemin critique de toute
stratégie double-IMU, item 45.

**Piste B — MinIMU-9 v5 externe (déjà en chantier, c'est probablement ta piste)**
Branche `feature/minimu9-driver` (2 commits, 2026-09-07/08) : driver C++
complet pour LSM6DS33 + LIS3MDL sur bus I2C du Pi, chaque registre vérifié
contre les datasheets ST officielles. Déjà fait :
- Lit accel+gyro+température en une rafale I2C, cadencé sur le data-ready
  register du capteur (104 Hz réel, sondé à 250 Hz) plutôt qu'en roue libre.
- Publie `leo_msgs/Imu` sur `/minimu9/imu` — même type que
  `firmware_message_converter`, donc **remplacement à la source** :
  `imu_sanitizer` n'a que son `in_topic` à repointer, toutes ses gardes
  avales (rejet de pics, verrou d'horodatage, détection ZUPT, rescale)
  survivent intactes.
- Bascule déjà câblée dans `navigation_supervision.launch` via l'argument
  `imu_source` (`core2` par défaut ↔ `minimu9`), qui lie `in_topic` **et**
  `accel_scale` ensemble exprès — vérifié par `--dump-params` que les deux
  modes donnent exactement les valeurs attendues (`/firmware/imu` + 1,1297
  vs `/minimu9/imu` + 1,0). **Préparée, pas armée.**

**⚠️ Jamais testé contre le vrai capteur.** Le capteur a été commandé le
jour du premier commit ; tout le driver a été écrit avant son arrivée. Rien
n'a encore été vérifié par une transaction I2C réelle.

**⚠️ La branche est en retard sur `main`** (divergée le 2026-09-07 ; `main`
a depuis reçu beaucoup de nettoyage — rebaser avant de travailler dessus,
`git diff --stat main feature/minimu9-driver` donne une idée de l'écart).

**Étapes concrètes, dans l'ordre** :
1. Rebaser/merger `feature/minimu9-driver` sur la branche courante.
2. Câbler le capteur, puis **`i2cdetect -y 1` avant toute confiance dans un
   échantillon publié** (instruction explicite laissée dans le commit).
3. Vérifier avec `tools/verif_minimu9.py`, déjà écrit pour ça — contrôle et
   pré-calibration du MinIMU-9.
4. **Caractériser le bruit par variance d'Allan sur l'unité réellement
   installée.** Ne PAS réutiliser les densités de `config_imu.yaml` (celles
   du CORE2) — c'est la pratique du projet (voir item 45) et c'est
   important : réutiliser une config d'un autre capteur reviendrait au
   même piège que celui qu'on essaie de corriger.
5. `minimu9_node` tourne **sur le robot** (bus I2C = matériel du Pi), pas
   sur le poste de travail — volontairement absent de
   `navigation_supervision.launch` qui est un launch poste de travail.
6. Seulement alors, basculer `imu_source:=minimu9`, reconfirmer
   `accel_scale: 1.0` via `--dump-params` avant tout run qui compte.

**Réserve écrite dans le rapport (à ne pas ignorer)** : l'erreur dominante
est une erreur d'*échelle*, pas de bruit — remplacer par un second capteur
non calibré ne la corrige pas automatiquement si ce second capteur a lui
aussi une erreur d'échelle non mesurée. D'où l'étape 4 : sans variance
d'Allan sur l'unité réelle, la comparaison ne prouvera rien.

### Comparer sur l'ensemble des navigations

L'outillage existe déjà, pas besoin de le réinventer :
- `tools/record_trajectories.sh` — capture rosbag robuste, indépendante du
  site web (le site n'est pas fiable pour un essai qui compte).
- `tools/compare_tests.m` — accepte `.mat` ou `.csv`, protocole Test1/Test2.
- Procédure NEES du rapport (§sec:nees) — c'est la même utilisée pour
  comparer MINS "avant/après" (§sec:mesureactuelle) : à réutiliser telle
  quelle pour comparer CORE2 vs MinIMU-9.

Faire tourner le protocole identique (même trajectoire, même durée) sur
chacun des modes de navigation du FSM (patrouille, évitement d'obstacle,
approche/verrouillage de balise, retour à la base) avec CORE2 puis avec
MinIMU-9, et comparer. Un point méthodologique déjà découvert par le
rapport et qui s'appliquera ici aussi : **ni openVINS ni sqrtVINS ne
s'initialisent à l'arrêt** (item 50) — toute course de comparaison doit
commencer par une excitation avant de s'immobiliser, jamais l'inverse, sous
peine de zéro échantillon VIO à comparer.

---

## Chantier 3 — Autres points ouverts à forte valeur (par ordre de priorité)

Détail complet de chacun dans le registre (`report_latex/Fusion_Campaign.tex`,
chercher le numéro d'item).

| # | Sujet | Pourquoi ça compte |
|---|---|---|
| 47 | MINS diverge spontanément, entrées saines, seul remède connu = respawn | Cause interne au filtre non identifiée ; récidive en 30 min la dernière fois. Instrumenter covariance/taux d'acceptation χ² dans le temps, pas seulement la pose de sortie. |
| 48 | `pose_selector` garde une correction SE(3) périmée qui survit à un reset de l'estimateur | Peut afficher un robot "divergé" alors que l'estimateur dessous est sain au mm près. Fix durable : invalider le cache dès qu'une source amont signale un reset, plutôt que de compter sur l'ordre manuel de relance. |
| 49 | Calibration IMU-caméra existe, à moitié installée, ne peut pas être terminée avec l'enregistrement actuel | Bag quasi immobile → translation faiblement observable malgré un bon résidu de reprojection. Ré-enregistrer avec une excitation six-axes vigoureuse. Bloquant maintenant que `do_calib_ext` est gelé. |
| 46 | MINS ne peut pas ingérer les mesures vicon en mode live — abonnement jamais câblé côté ROS1 | Bloque tout le plan "ancre de balise" (AprilTag/Carolus) — la seule voie vers un repère monde surveillé. Plomberie pure, le callback existe déjà ×4 à copier. |
| 50 (moitié ouverte) | Datation de la covariance des deux VIO | Aucun des deux ne publie à l'arrêt — nécessite une course excitation-puis-immobile, sans redémarrage entre les deux. |
| 44 | Batterie sous le seuil projet (10,3 V), coïncidence non démontrée avec une panne moteur silencieuse | Recharger et re-lire `/cmd_vel` d'abord dans toute future investigation moteur ; exposer une alarme opérateur en dessous de 10,3 V. |

---

## État du labo au moment de la passation (à ne pas perdre)

- Une expérience est actuellement **non committée** sur
  `catkin_ws/src/open_vins/config/leo/estimator_config.yaml` (branche
  `robustesse-numerique-vio`) : retour vers les valeurs par défaut
  d'openVINS (`max_clones`, `num_pts`, `grid_x/y`, `knn_ratio`,
  `up_msckf_chi2_multipler`, suppression de `fi_max_baseline`) au lieu des
  réglages "alignés MINS" accumulés fin août. **Testé le 2026-09-10, pas
  d'amélioration mesurée.** À décider : garder telle quelle pour repartir
  d'une base propre, ou `git checkout --` pour revenir aux réglages
  précédents — mais dans les deux cas, documenter la décision (voir
  `MEMORY.md`/workflow : incrémenter `report_latex` + `JOURNAL_DE_BORD.md` à
  chaque avancée, ne pas laisser ce diff traîner indéfiniment non tranché).
- Item 50 du registre est notre meilleur résumé de "qu'est-ce qui a
  vraiment été mesuré sur le logiciel qui tourne aujourd'hui" — s'y référer
  avant de faire confiance à un chiffre du rapport daté de plus d'une
  semaine.

---

## Ordre suggéré pour les deux premières semaines

1. Lire le rapport (au moins Introduction, §sec:sqrtvins, item 45, annexe D).
2. Rebaser `feature/minimu9-driver`, câbler le capteur, `i2cdetect`.
3. Variance d'Allan sur le MinIMU-9, écrire `config_imu_minimu9.yaml` séparé
   (ne pas toucher aux densités CORE2 existantes).
4. Pendant que le capteur/l'Allan variance tourne (temps mort), lire
   `Propagator.cpp`/`UpdaterMSCKF.cpp` pour le second défaut sqrtVINS —
   travail qui n'a pas besoin du robot.
5. Soumettre le rapport de bug ZUPT à RPNG (30 minutes, gain indépendant du
   reste).
6. Bascule `imu_source:=minimu9`, campagne de comparaison sur les 4 modes
   de navigation avec le protocole Test1/Test2 existant.
7. Trancher le diff `estimator_config.yaml` en suspens avant qu'il ne soit
   oublié.

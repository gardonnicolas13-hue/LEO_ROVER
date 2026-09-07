# Patchs sqrtVINS — LEO Rover / Florida Tech, 2026-08-24

Quatre patchs indépendants contre [rpng/sqrtVINS](https://github.com/rpng/sqrtVINS)
au commit `30fafc8`. Chacun se tient seul et peut être soumis, accepté ou
rejeté séparément — ils sont volontairement découpés par *sujet*, pas par
fichier.

**Exception : le patch 05 (ajouté le 27/08) n'est pas indépendant** — c'est un
correctif au patch 03, pas un nouveau sujet, et il ne s'applique qu'une fois
03 déjà en place (voir §05).

**Le patch 06 (ajouté le 07/09) est d'une autre nature que 01-05** : ce n'est
pas un correctif de bug amont soumettable à rpng/sqrtVINS, c'est une
**fonctionnalité neuve, propre à cette plateforme** (Phase 1 de correction de
dérive par AprilTag — voir §06). Il n'est vérifié que par-dessus 01-05 déjà
appliqués (c'est l'état réel de ce dépôt) ; son point d'insertion dans
`VioManager.cpp` est du code amont vierge, mais ses commentaires font
explicitement référence au mécanisme de reset-sur-divergence des patchs 03/05
— l'appliquer seul sur du sqrtVINS pristine compilerait sans doute, mais les
commentaires n'auraient plus de sens sans ce contexte.

Appliquer dans l'ordre :

```bash
cd <racine du dépôt sqrtVINS>
git apply --check patches/01_llt_guards_20260824.patch   # toujours vérifier d'abord
git apply         patches/01_llt_guards_20260824.patch
# ... 02, 03, 04 ...
git apply --check patches/05_reset_covariance_20260827.patch   # necessite 03 deja applique
git apply         patches/05_reset_covariance_20260827.patch
# ... puis 06, qui suppose 01-05 deja en place ...
git apply --check patches/06_tag_correction_phase1_20260907.patch
git apply         patches/06_tag_correction_phase1_20260907.patch
```

---

## 01 — Gardes de factorisation Cholesky (5 fichiers, +1 en-tête)

**Le défaut.** Un audit de ce paquet a trouvé **neuf** sites de factorisation
Cholesky, dont **aucun** ne vérifiait `Eigen::LLT::info()`. Quand une LLT
échoue — matrice non définie positive, rang déficient, ou contaminée par un
`NaN` venu d'une seule mesure — Eigen **ne lève pas d'exception** : il renvoie
une matrice **non initialisée**, qui part directement dans le calcul suivant.
La défaillance est silencieuse par construction.

**Le remède**, dans un en-tête partagé `ov_core/src/utils/llt_guard.h`, avec
**trois** fonctions parce que les trois usages appellent trois réponses
différentes :

| fonction | usage | comportement en cas d'échec |
|---|---|---|
| `llt_guarded()` | facteur alimentant l'**état** (bruit de process, mise à jour SRF) | régularisation de Tikhonov `eps*I`, puis abandon si l'échec persiste |
| `chi2_guarded()` | **garde χ²** (validation de mesure) | **rejet** de la mesure, jamais de régularisation |
| `llt_solve_guarded()` | **incrément** appliqué à l'état (pas LM, correction) | abandon du pas |

La distinction est le cœur du patch. Régulariser un χ² reviendrait à
**inventer une preuve** : un χ² issu d'une factorisation ratée n'est pas
« petit », il n'est **pas calculable** — et comme `NaN` se compare faux à tout
seuil, une valeur non vérifiée franchit la garde comme si elle avait réussi.
À l'inverse, régulariser une covariance de bruit revient à déclarer un peu
plus d'incertitude : conservateur, et sans commune mesure avec le risque de
propager une matrice non initialisée.

**Sites couverts :** `Propagator.cpp` (bruit de process), `StateHelper.cpp`
(×3 : mise à jour SRF, incrément d'état, χ²), `UpdaterSLAM.cpp` (χ²),
`DynamicInitializer.cpp` (×2 : pas Levenberg-Marquardt).

**Antériorité.** Le rapport de bug déposé depuis cette plateforme
(`docs/rpng_sqrtvins_zupt_bug_report.md`) avait corrigé **un** de ces sites et
noté : *« the `LLT().solve()`-without-`.info()`-check pattern in particular
seems like something worth checking for elsewhere in the codebase too »*. Ce
patch est le résultat de cette vérification.

**Mesuré :** compile sans avertissement ; en fonctionnement nominal les gardes
ne se déclenchent **jamais** (0 sur les cinq compteurs), donc aucune
dégradation du chemin normal.

---

## 02 — Gardes ZUPT : χ² non contournable et plafond physique absolu

**Défaut A — la disparité court-circuitait la garde χ².**

```cpp
// commentaire d'origine : "We need to pass the chi2 and not be above our velocity threshold"
if (!disparity_passed && (chi2 > seuil || vel > seuil_v)) { rejeter }
```

Le `!disparity_passed &&` verrouille **tout** le rejet : dès que la disparité
passe, ni le χ² ni le seuil de vitesse ne sont évalués. Mesuré garde χ²
pourtant active (`zupt_chi2_multipler: 1`, seuil correct à 65,171) :
**290 acceptations, 0 rejet**, avec χ² = 1,34×10⁵⁷ et |v| = 1,0×10²³ m/s. Le
commentaire décrivait la bonne intention ; le code faisait l'inverse.

**Défaut B — pas de plafond physique.** Un seuil réglable par YAML peut être
désarmé. Ajout d'un plafond compilé (3,0 m/s), non contournable, laissant un
facteur 11 sur la vitesse réelle maximale de la plateforme (0,264 m/s mesurée
par l'estimateur ancré sur les roues).

**Résultat :** |v| 1,0×10²³ → **0,030 m/s** ; χ² 1,34×10⁵⁷ → **1,1–1,5** ;
position finale 2,06×10¹⁵ m → **0,019 m**.

---

## 03 — Réinitialisation sur divergence prouvée

Répond au `// TODO: Or if we are trying to reset the system, then do that
here!` jamais implémenté en amont.

Le plafond du patch 02 empêche d'appliquer une mise à jour calculée sur un
état corrompu, mais se comporte en **verrou** : au-dessus du plafond, toute
correction est refusée, y compris ZUPT, seul mécanisme capable de ramener le
filtre. Mesuré rover à l'arrêt : **1 acceptation contre 2235 rejets**, vitesse
montant sans frein, position à 34 km.

Le déclencheur n'est pas une heuristique mais une **contradiction prouvée
entre deux sources indépendantes** : le filtre affirme une vitesse impossible
pour la plateforme pendant que la disparité d'image — qui ne dépend d'aucun
état du filtre — affirme la scène statique. Un état qui contredit une mesure
indépendante à cet ordre de grandeur ne contient plus d'information à
préserver ; on le reconstruit.

**L'état est remis à zéro explicitement.** Un premier jet se contentait de
baisser `is_initialized_vio` en comptant sur l'initialiseur : il ne remet
**pas** la position, et 1392 resets enchaînés ont laissé la position
s'accumuler jusqu'à 40 857 m. Un reset qui ne remet pas à zéro ce qu'il
prétend réinitialiser n'est pas un reset.

---

## 04 — Instrumentation par étage (diagnostic, pas correctif)

`UpdaterMSCKF.cpp` ne contenait **aucune** sortie de diagnostic — ni
`PRINT_ALL`, ni `PRINT_DEBUG` — et son unique compteur était commenté. Une
capture en `verbosity: ALL` ne révélait donc rien de cet étage.

Cinq compteurs, un par `erase` du pipeline, dans l'ordre d'exécution :

```
[MSCKF-ETAGES]: entree N | <2mesures N | avant_tri N | tri N | refine N
                | jacob N | reproj N | chi2 N | SORTIE N
```

**Une métrique invalidée au passage.** `/points_msckf` était lu comme
« features consommées par mise à jour ». Il est alimenté par
`good_features_MSCKF`, vidé uniquement dans `do_feature_propagate_update` :
quand un ZUPT préempte le chemin caméra, cette fonction n'est jamais atteinte,
le vecteur garde son contenu, et le visualiseur le republie indéfiniment. Une
capture montrait 1352 messages à 78 features de moyenne pendant que
`UpdaterMSCKF::update()` était appelé **zéro** fois.

**Ce patch est optionnel pour l'amont** — c'est un outil de diagnostic, pas une
correction. Il est fourni parce que sans lui, les mesures des patchs 01 à 03 ne
sont pas reproductibles.

---

## 05 — Réinitialisation sur divergence : la covariance manquait

Complète le patch 03 (s'applique par-dessus, une fois 01-03 en place). S'applique
au même bloc de reset, dans le même fichier.

**Le défaut.** Le patch 03 remet l'état MOYEN à zéro (quaternion, position,
vitesse, biais) mais **pas** la covariance : le facteur racine carrée `U_`
survivait tel quel à la réinitialisation. Un état frais avec une incertitude
encore énorme donne, dès la mesure suivante, un gain de Kalman disproportionné
— l'état tout juste remis à zéro est immédiatement recorrompu selon une forme
que l'ancienne covariance encode encore.

**Mesuré.** 212 resets capturés en une session, dont les `(|v|, |p|)` au
déclenchement clustent sur deux valeurs quasi exactes — 3,00–3,09 m/s et 3,4
OU 3,6 m — au lieu d'être dispersées comme le serait une vraie divergence
bruitée. Le filtre rejouait presque la même trajectoire de divergence à
chaque cycle, d'où le motif périodique observé à l'écran (~2,3 s), aussi bien
à l'arrêt qu'en rotation.

**Le remède.** `StateHelper::set_initial_imu_square_root_covariance()` sur les
mêmes 15 termes diagonaux que l'initialisation à froid
(`StaticInitializer.cpp`) : q=0,017, p=0,05, v=0,01, bg=0,02, ba=0,02. Dupliqué
plutôt que partagé — cinq constantes ne valent pas l'indirection — mais doit
rester synchronisé avec l'initialiseur si ces valeurs sont un jour retouchées.

**Résultat.** À l'arrêt : 212 resets/session → **0 reset sur 10 min**, saut
maximal < 1 mm (mesuré isolément, avant toute autre modification). En
rotation active, combiné à la resynchronisation de config du patch de suivi
de piste (`tools/robot/sqrtvins/`, 2026-08-27) : 52 sauts de position > 0,3 m
sur 60 s (dont plusieurs de 5-10 m) → **0**.

---

## 06 — Phase 1 : correction de dérive par AprilTag (2026-09-07)

**Demande.** Hector, feuille de route Phase 1/2/3 (1 tag → plusieurs tags →
balises SVGS façon Vicon). Cette Phase 1 : un seul tag (`tag36h11`, ID 0,
0,17 m de côté), re-ancrer l'estimateur dessus quand il est vu.

**Constat préalable, vérifié avant d'écrire une ligne de code.** sqrtVINS n'a
**aucun** mécanisme de pose absolue — ni GPS, ni vicon, ni fermeture de
boucle. Seuls trois `Updater` existent (MSCKF, SLAM, ZeroVelocity), tous
visuels/ZUPT. `TrackAruco` existe mais traite les coins du marqueur comme des
features visuelles ordinaires (sa propre docstring : *« the actual size of
the tags do not matter »*) — famille `DICT_6X6_1000`, pas `tag36h11` de
toute façon. Il n'y avait donc rien à brancher : le mécanisme restait à
écrire.

**Architecture retenue, et deux autres écartées.** (a) Un vrai `Updater`
(Jacobien + bruit + porte χ²) — écarté : ce filtre a déjà eu plusieurs bugs
subtils cette même campagne (patch 01, patch 02, et 03/05 ci-dessus), y
ajouter de nouvelles maths de filtre est le choix le plus risqué. (b) Une
correction purement externe, hors du filtre (façon `pose_selector.py` du
projet) — écarté : ne corrige pas l'état interne, ne « zero out » rien, juste
republie une pose corrigée en aval. **Retenu : réutiliser exactement les
primitifs déjà validés par 03/05 ci-dessus**
(`StateHelper::set_initial_imu_square_root_covariance`, purge des bases de
features, `is_initialized_vio = false`), réamorcés avec la pose du tag au
lieu de zéro. Coût assumé et documenté dans le code : chaque correction force
un redémarrage à froid complet, pas une correction en douceur — `is_initialized_vio`
repasse à `false`, quelques secondes de reconvergence. L'alternative plus
douce existante dans le code (`initialize_with_gt`, utilisée par la
simulation) ne convient pas ici : elle ne s'exécute qu'avant qu'aucun
clone/landmark n'existe, alors qu'un correctif en direct doit composer avec
un état déjà peuplé — les laisser en place pendant un saut de pose les
rendrait géométriquement incohérents avec la nouvelle pose.

**Géométrie**, dérivée deux fois indépendamment et vérifiée contre trois
usages réels du code (pas la doc générique openVINS) :
```
R_GtoI = R_ItoC^T · R_CtoTag^T
p_IinG = R_CtoTag^T · (p_IinC − p_TagInCam)
```
avec repère monde := repère du tag (pas de verrouillage au premier fix
contrairement à `carolus_tf_bridge.py` du même projet : le tag ne bouge pas,
chaque détection stable redonne la même réponse par construction).

**Gate anti-détection isolée.** 5 détections consécutives du même ID,
mutuellement cohérentes à 3 cm / 5° près, avant d'agir ; recul de 10 s entre
deux corrections (chaque correction étant un redémarrage à froid, la laisser
se redéclencher à chaque frame serait absurde).

**Portée.** `VioManagerOptions.h/.cpp` (nouveaux réglages,
`tag_correction_enabled` **false** par défaut), `VioManager.h/.cpp`
(`reset_to_known_pose`, nouvelle méthode publique), `ros/TagPoseCorrector.h/.cpp`
(nouveau, abonnement + gate + géométrie), `run_subscribe_msckf.cpp` (câblage
ROS1 uniquement — aucun déploiement ROS2 de sqrtVINS sur ce projet),
`cmake/ROS1.cmake` + `package.xml` (dépendance `apriltag_ros`, absente
avant).

**Vérifié.** Compile proprement (`catkin build ov_srvins` : 0 avertissement,
0 échec), symboles confirmés présents dans la bibliothèque partagée ET
l'exécutable final. **Non vérifié : aucun test avec une vraie balise
physique ni un sqrtVINS lancé** — `tag_correction_enabled` reste à `false`
tant que ce test n'a pas eu lieu.

---

## 07 — Argument de lancement pour `tag_correction_enabled` (2026-09-07)

Complète le patch 06. Sans lui, activer la Phase 1 pour un essai exige
d'éditer `config/leo_pc/estimator_config.yaml` à la main (le seul défaut
`false` y est câblé) — possible, mais source d'oubli de repasser à `false`
ensuite. **Piège identifié et évité** : `rosparam set /sqrtvins/... true`
AVANT `roslaunch` ne fonctionne PAS ici, parce que le nœud `sqrtvins` porte
`clear_params="true"` — roslaunch purge tout son espace de noms privé au
moment de CE lancement, avant même que le code du nœud ne lise quoi que ce
soit, donc un paramètre posé à la main juste avant est effacé sans avertissement.
Le remède est le même patron déjà utilisé quatre fois dans ce fichier
(`verbosity`, `use_stereo`, `max_cameras`, `dotime`) : un `<arg>` avec
défaut `false`, relayé en `<param>` à l'intérieur du `<node>` — ce qui,
contrairement à `rosparam set`, survit à `clear_params` car c'est roslaunch
lui-même qui pose ce paramètre en posant le nœud.

```bash
roslaunch ov_srvins sqrtvins_pc.launch tag_correction_enabled:=true
# ou, via le script du projet :
tools/launch_sqrtvins.sh tag_correction_enabled:=true
```

**Vérifié** : XML valide, et rechargé par la vraie bibliothèque `roslaunch`
(pas seulement un parseur XML générique) avec le workspace sqrtVINS
correctement sourcé — confirme que toutes les substitutions `$(arg …)`
existantes se résolvent encore sans erreur après l'ajout.

---

## 08 — Correctif de rotation : une transposition en trop (2026-09-08)

**Trouvé en relisant le patch 06 pour préparer la Phase 2**, pas au hasard :
généraliser à des poses de tag *absolues* rend une erreur de rotation
mesurable et fausse, alors qu'en Phase 1 (repère monde = repère du tag) une
même erreur ne produit qu'un repère arbitraire mais cohérent — ce qui a
probablement permis à l'essai terrain du 07/09 de « réussir » quand même :
seule la position (correcte) est ce qu'on remarque en vérifiant vite.

**Le défaut.** Le quaternion brut d'`apriltag_ros` était stocké dans une
variable nommée `R_CtoTag`, puis transposé une seconde fois dans le calcul
final — annulant silencieusement la quantité qu'il contenait déjà. La
convention réelle (confirmée contre la sémantique `geometry_msgs/Pose` de
ROS, « pose du tag, dans le repère caméra ») est **Tag→Caméra**, pas
Caméra→Tag : le nom de la variable était l'inverse de ce qu'elle contenait.

**Vérifié par un test numérique indépendant**, pas seulement re-dérivé à la
main (une seconde dérivation à la main peut reproduire la même erreur si
elle repose sur la même hypothèse fautive) : matrices homogènes 4×4
arbitraires, vérité terrain calculée par composition directe
(`inv(T_TagToCam) @ T_ItoC`), comparée aux deux formules.

| | erreur rotation (max, élément de matrice) | erreur position |
|---|---|---|
| Formule déployée (fautive) | **1,57** | 2,5×10⁻¹⁶ m |
| Formule corrigée | **3,3×10⁻¹⁶** | 2,5×10⁻¹⁶ m |

La position était juste depuis le début — c'est probablement pour ça que le
test terrain a semblé marcher.

**Le remède.** Suppression de la transposition en trop, et **renommage** de
la variable vers son sens réel (`R_TagToCam`) pour qu'un futur lecteur ne
retombe pas dans le même piège en se fiant au nom plutôt qu'à ce que la
variable contient réellement.

**Vérifié.** Compile de nouveau proprement (`All 2 packages succeeded`,
0 avertissement). sqrtVINS n'étant pas lancé au moment du correctif, aucun
processus vivant touché. **Non revérifié en direct** — le prochain essai
terrain devra confirmer l'orientation, pas seulement la position.

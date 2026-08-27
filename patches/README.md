# Patchs sqrtVINS — LEO Rover / Florida Tech, 2026-08-24

Quatre patchs indépendants contre [rpng/sqrtVINS](https://github.com/rpng/sqrtVINS)
au commit `30fafc8`. Chacun se tient seul et peut être soumis, accepté ou
rejeté séparément — ils sont volontairement découpés par *sujet*, pas par
fichier.

**Exception : le patch 05 (ajouté le 27/08) n'est pas indépendant** — c'est un
correctif au patch 03, pas un nouveau sujet, et il ne s'applique qu'une fois
03 déjà en place (voir §05).

Appliquer dans l'ordre :

```bash
cd <racine du dépôt sqrtVINS>
git apply --check patches/01_llt_guards_20260824.patch   # toujours vérifier d'abord
git apply         patches/01_llt_guards_20260824.patch
# ... 02, 03, 04 ...
git apply --check patches/05_reset_covariance_20260827.patch   # necessite 03 deja applique
git apply         patches/05_reset_covariance_20260827.patch
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

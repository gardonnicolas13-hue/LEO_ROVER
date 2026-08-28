# Patchs openVINS — LEO Rover / Florida Tech, 2026-08-28

Contre [rpng/open_vins](https://github.com/rpng/open_vins). Répertoire séparé
de `/patches/` (qui vise `rpng/sqrtVINS`, un fork distinct) — même mécanisme,
projet amont différent.

```bash
cd <racine du dépôt open_vins>
git apply --check patches_openvins/01_reset_covariance_20260828.patch
git apply         patches_openvins/01_reset_covariance_20260828.patch
```

---

## 01 — Réinitialisation sur divergence : l'état n'était pas remis à zéro

**Contexte.** openVINS portait déjà un mécanisme de reset sur divergence
(`is_initialized_vio = false` sur `|v| > 3 m/s` ou `|p| > 1000 m`, la même
contradiction prouvée vitesse-estimée / disparité-image que sur sqrtVINS).
Mais il s'arrêtait là : ni l'état moyen ni la covariance n'étaient remis à
zéro, comptant sur `try_to_initialize()` pour tout reconstruire au prochain
cycle.

**Le défaut.** C'est exactement le « premier jet » que sqrtVINS avait déjà
essayé et abandonné (`patches/03_reset_on_divergence_20260824.patch`, dans le
dépôt LEO Rover) : mesuré à l'époque, 1392 resets enchaînés avaient laissé la
position s'accumuler jusqu'à 40 857 m au lieu de repartir de zéro. openVINS
sur ce projet n'avait jamais reçu la correction ultérieure — ni la remise à
zéro explicite de l'état, ni celle de la covariance
(`patches/05_reset_covariance_20260827.patch`).

**Le remède**, porté des deux patchs sqrtVINS ci-dessus vers l'API openVINS
(`state->_imu`, `PoseJPL` pour le quaternion+position, `StateHelper::
set_initial_covariance` avec une matrice pleine plutôt qu'un facteur racine
carrée) :
1. État moyen remis à zéro explicitement (quaternion identité, tout le reste
   à zéro) — ordre du vecteur 16×1 : quaternion(4), position(3), vitesse(3),
   biais gyro(3), biais accel(3).
2. Covariance remise aux mêmes valeurs que l'initialisation à froid
   (`ov_init/src/static/StaticInitializer.cpp`, bloc « Create base
   covariance ») : q=0,02, p=0,05, v=0,01, bg=0,02, ba=0,02 (écarts-types).
   Dupliqué plutôt que partagé — doit rester synchronisé avec cet
   initialiseur si ces valeurs sont un jour retouchées.

**Correction du 2026-08-28 (même soirée) : la première version de ce patch
était incomplète et a produit une régression mesurée en direct.** Compilée
et deployée sans purge des clones, elle a produit 9089 resets consécutifs,
`|v|`/`|p|` croissant de façon quasi exponentielle d'un reset au suivant —
3 m / 8 m / 184 m / 16 km / 530 000 km / 2 200 000 km aux resets #1 / #10 /
#100 / #1000 / #5000 / #9089. Cause : `set_initial_covariance()` ne touche
que le bloc diagonal IMU-IMU de `state->_Cov` ; les clones de la fenêtre
glissante (`state->_clones_IMU`) restaient en place avec leurs cross-termes
de covariance intacts. Une mesure triangulée contre ces clones périmés
produit une mise à jour fausse, appliquée avec une confiance disproportionnée
(covariance IMU toute fraîche donc petite) — divergence immédiate, pire à
chaque cycle. Ajout : purge complète des clones via
`StateHelper::marginalize()` (retaille proprement `state->_Cov`, pas un
simple `.clear()` de la map) **avant** la remise à zéro de la covariance.

**Vérifié en direct** (pas seulement à la compilation) : redémarré, surveillé
~1,5 min de fonctionnement réel (48 631 lignes de log, dont des mesures et
mises à jour normales) — **0 reset**, contre 9089 avec la version
incomplète. Source VINS rebasculée active, relais `/robot_pose_fused`
confirmé continu sur 6 s (497 échantillons, pas de gel). Reste à faire :
protocole rotation + comptage de sauts > 0,3 m complet, comme pour sqrtVINS
— cette vérification n'est qu'un premier passage en conditions réelles, pas
la validation complète.

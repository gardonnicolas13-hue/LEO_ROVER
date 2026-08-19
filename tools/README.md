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
roslaunch leo_navigation navigation_master.launch gyro_recal_period:=60
```

Vérifier au démarrage que le nœud l'annonce :

```
[imu_sanitizer] recalibration gyro PÉRIODIQUE active (60 s, immobilité
                confirmée par /firmware/wheel_states)
```

Sans cette ligne, le paramètre n'a pas pris — ne pas conduire pour rien.

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

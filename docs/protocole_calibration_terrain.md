# Protocole de calibration terrain — marquage au sol

**Version 2026-07-29.** Remplace le marquage à l'adhésif « à vue » et la
lecture au rapporteur, qui sont la cause suspectée de l'écart de 7 à 10 %
observé lors de la première campagne.

---

## Pourquoi ce protocole existe

Première campagne, part symétrique du facteur `réel / mesuré` :

| source | facteur |
|---|---|
| gyroscope | 0,9329 |
| odométrie roues | 0,9123 |

Le lacet des roues est calculé par odométrie différentielle : **il n'utilise
pas le gyroscope**. Ce sont donc deux capteurs physiquement indépendants qui
affirment tous les deux que le robot a tourné 7 à 10 % **de plus** que ce que
le rapporteur indiquait. (MINS et VINS ne comptent pas comme témoins : ils
intègrent le gyro, donc héritent de son erreur.)

Deux capteurs indépendants d'accord entre eux et en désaccord avec la
référence : c'est la **référence** qu'il faut soupçonner en premier.

Trois faiblesses de la méthode initiale, par ordre d'importance :

1. **Le rapporteur sur moquette.** Bras courts, alignement difficile, lecture
   à ±2 ou 3° sans effort particulier. Sur 90°, cela fait déjà 3 %.
2. **Le marquage du châssis à vue.** Le robot n'a pas de ligne de référence
   nette, et le report au sol depuis le dessus introduit de la parallaxe.
3. **Le centre de rotation n'est pas fixe.** Sur des roues mecanum en pivot,
   il glisse. « Le même point avant et après » n'est donc pas bien défini.

**Le principe du nouveau protocole : on ne mesure plus un angle, on le
construit — et on ne lit plus qu'un écart linéaire sur un long bras de
levier.**

---

## Matériel

- un mètre ruban (2 m minimum, graduations millimétriques)
- de l'adhésif de masquage
- une ficelle et deux poids, ou deux fils à plomb improvisés (un écrou sur un
  fil suffit)
- un cordeau ou une règle longue pour tracer des lignes droites nettes
- de quoi noter (les valeurs se saisissent ensuite dans le cockpit)

Aucun rapporteur. C'est volontaire.

---

## Étape 1 — Construire l'angle droit de référence (une seule fois)

On n'utilise pas de rapporteur : un ruban construit un angle droit bien mieux,
par la règle du 3-4-5.

1. Choisir un point d'origine **O** au sol, dans une zone dégagée d'au moins
   2 × 2 m. Le marquer d'une croix d'adhésif.
2. Tracer une ligne droite **A** depuis O, longue de **1,5 m au moins**
   (cordeau ou règle — la ligne doit être nette, pas approximative).
3. Marquer un point **P** sur A, à exactement **90,0 cm** de O.
4. Tracer une ligne provisoire **B** depuis O, à peu près perpendiculaire, et
   y marquer un point **Q** à exactement **120,0 cm** de O.
5. Mesurer la diagonale **PQ**. Faire pivoter B autour de O jusqu'à ce que
   **PQ = 150,0 cm**.
6. Fixer B à l'adhésif et la prolonger à **1,5 m au moins**.

**Pourquoi c'est meilleur.** Par dérivation de la loi des cosinus,
`dθ = dc · c / (a·b·sin θ)`, soit `dθ = dc / 72` en radians pour cette
construction. Une erreur de **1 mm** sur la diagonale de 150 cm correspond donc
à **0,08°** sur l'angle. Le rapporteur, lui, se lit au mieux à ±1°.
Gain : un facteur **13**.

> Vérification : la diagonale doit valoir 150,0 cm à ±1 mm. Si vous n'y
> arrivez pas, vos lignes A et B ne sont pas assez droites — reprenez le
> tracé, tout le reste en dépend.

---

## Étape 2 — Donner au robot une direction lisible

**Ne marquez pas le châssis à l'œil.** Utilisez deux fils à plomb.

1. Choisir deux points de fixation sur l'**axe longitudinal** du robot, aussi
   éloignés que possible l'un de l'autre (un à l'avant, un à l'arrière).
   Repères durables : un coin de platine, une vis, un trou de fixation.
2. Suspendre un fil à plomb à chacun. Laisser le poids se stabiliser au sol.
3. La droite joignant les deux marques au sol **est** le cap du robot.

**Pourquoi c'est meilleur.** Un fil à plomb élimine la parallaxe : la marque
est exactement sous le point de fixation, quel que soit l'angle de vue. Et
comme on n'utilise que la **direction** de la droite — jamais la position d'un
point — le glissement du centre de rotation pendant le pivot n'a aucun effet
sur la mesure. C'est le point clé : ce protocole est insensible au patinage.

Notez la distance **L** entre les deux points de fixation. Plus elle est
grande, plus la mesure est précise. Visez **L ≥ 40 cm**.

---

## Étape 3 — Passe de ROTATION (90° gauche, puis 90° droite)

1. Placer le robot de sorte que **les deux marques de fil à plomb tombent sur
   la ligne A**. Prendre le temps de bien aligner : c'est le zéro de la mesure.
2. Sur le cockpit, onglet Trajectory, sélectionner **90° G** (ou **90° D**),
   puis **MARQUER LE DÉPART**.
3. Faire pivoter le robot d'environ 90°, lentement et d'un seul geste.
   L'angle n'a **pas** besoin d'être juste — c'est tout l'intérêt : on va
   mesurer ce qu'il vaut réellement.
4. Arrêter le robot. Cliquer **MARQUER L'ARRIVÉE**. Le cockpit fige
   l'instant : vous avez tout le temps de mesurer.
5. Marquer au sol les deux nouveaux points de fil à plomb.
6. **Mesurer, sans rapporteur** :
   - la marque la plus proche de O définit l'origine du cap final ;
   - mesurer l'écart perpendiculaire **d** entre la marque la plus **éloignée**
     et la ligne **B** ;
   - noter la distance **L** entre les deux marques.
7. Calculer l'angle résiduel puis l'angle réel :

   ```
   ε = atan(d / L)            en degrés
   angle réel = 90 − ε        si la marque est EN DEÇÀ de B (pas assez tourné)
   angle réel = 90 + ε        si la marque est AU-DELÀ de B (trop tourné)
   ```

8. Saisir **l'angle réel** dans le champ du cockpit, puis **VALIDER LA PASSE**.

### Précision obtenue

Avec **L = 1,0 m** et une lecture de **d** à ±2 mm :

```
ε lisible à ±0,11°  →  soit 0,12 % sur 90°
```

Contre environ **3 %** avec un rapporteur sur moquette. **Facteur 25.**

### Exemple chiffré

`d = 12 mm`, `L = 950 mm`, marque au-delà de B :

```
ε = atan(12 / 950) = 0,72°
angle réel = 90 + 0,72 = 90,7°   → saisir 90,7
```

---

## Étape 4 — Passe de LIGNE DROITE (1 m)

1. Tendre une ficelle au sol sur **2 m au moins**, bien droite. Ce sera la
   trajectoire de référence.
2. Aligner le robot dessus (les deux marques de fil à plomb sur la ficelle).
3. **MARQUER LE DÉPART**. Marquer au sol le fil à plomb **avant** uniquement,
   et retenir lequel : c'est le seul point qui compte.
4. Avancer d'environ 1 m, lentement. Là encore, la distance exacte est
   indifférente.
5. Arrêter. **MARQUER L'ARRIVÉE**. Marquer le **même** fil à plomb avant.
6. Mesurer :
   - **a** = distance entre les deux marques, mesurée **le long** de la ficelle ;
   - **e** = écart latéral final de la marque par rapport à la ficelle.
7. La distance réellement parcourue :

   ```
   distance réelle = √(a² + e²)      en mètres
   ```

   (si `e < 20 mm`, `a` seul suffit : la correction est inférieure à 0,02 %)

8. Saisir **la distance réelle en MÈTRES** (ex. `1,043`), puis **VALIDER**.

> **Le piège à éviter.** Mesurez toujours entre les marques du **même** fil à
> plomb. Utiliser le point avant au départ et le point arrière à l'arrivée
> ajoute silencieusement la longueur du robot à la mesure.

---

## Étape 5 — Répéter, et lire le résultat

**Trois passes par configuration**, soit neuf au total (90° G, 90° D, 1 m).

Règles :

- Entre chaque passe, **effacer les anciennes marques** de fil à plomb et
  refaire l'alignement de départ. Ne réutilisez jamais un marquage.
- Enchaîner les passes d'une même série : température et batterie restent
  ainsi comparables.
- Noter la tension batterie au début et à la fin.
- **Saisir toujours la valeur mesurée, jamais la valeur visée.** Un 90,7°
  honnête vaut infiniment mieux qu'un 90° supposé — c'est exactement l'écart
  entre les deux qui a rendu la première campagne inutilisable.

### Comment interpréter

Le panneau affiche l'écart-type sur les trois passes. Il tranche la question
posée en tête de ce document :

| écart-type | lecture |
|---|---|
| **< 2 %** | méthode fiable. Le facteur restant est une **vraie** erreur de capteur → applicable. |
| **2 – 5 %** | zone grise. Faire trois passes de plus avant de conclure. |
| **> 5 %** | c'est encore la **méthode** qui domine. Ne rien appliquer, reprendre le protocole. |

Le cockpit signale lui-même `⚠ dispersé — refaire` au-delà de 10 %.

### Le résultat qu'on cherche vraiment

Comparer les parts symétriques du **gyroscope** et des **roues**, qui sont
indépendants :

- **Les deux retombent vers 1,00** → la première campagne mesurait bien le
  rapporteur, pas les capteurs. Aucune correction à appliquer, et
  `~gyro_scale` reste à 1,0.
- **Les deux restent vers 0,93** → l'erreur est réelle et partagée. Elle
  devient applicable, mais il faudra alors expliquer *pourquoi* deux capteurs
  de principes différents dérivent du même montant — ce n'est pas anodin.
- **Ils divergent l'un de l'autre** → chacun a son propre défaut. Traiter
  séparément : `~gyro_scale` pour le gyro, le rayon effectif ou l'empattement
  pour les roues.

Dans tous les cas : **ne touchez pas à `~gyro_scale` avant d'avoir les trois
passes dans les deux sens.** Ce paramètre affecte MINS et VINS simultanément,
donc une erreur s'y propage partout.

---

## Rappel des correspondances cockpit

| bouton | ce qu'il fait |
|---|---|
| **90° G / 90° D / 1 m** | choisit la série ; les séries sont conservées séparément |
| **MARQUER LE DÉPART** | fige la pose de départ des quatre sources |
| **MARQUER L'ARRIVÉE** | fige la pose d'arrivée ; affiche ce que chaque source a cru |
| **VALIDER LA PASSE** | enregistre la passe et recalcule la moyenne |
| **↶ passe** | annule la dernière passe de la série courante |
| **↺ tout** | vide la série courante (les autres sont préservées) |

La campagne est écrite sur disque à chaque passe
(`logs/calib_terrain.json`) : un redémarrage du backend ne la perd plus.

En secours, si rosbridge décroche en pleine série :
`python3 tools/calib_terrain.py rotation gauche 3`

# Diagnostic — liaison série Pi ↔ carte CORE2

**Contexte (item 19 du rapport, non résolu au 2026-07-23) :** `serial_node`
(rosserial, port `/dev/serial0`, 250 000 bauds) a enregistré 20 295 erreurs
`wrong checksum for topic id and msg` en 2 h le 22/07, avec des messages
`lost sync` / `Mismatched protocol version` par intermittence. C'est la
cause la plus plausible (mais non prouvée) de la perte d'abonnement roues
de `firmware_message_converter` ce même soir, et un suspect raisonnable
pour la divergence d'initialisation de VINS observée le 23/07 (des données
IMU corrompues en amont dégraderaient n'importe quel estimateur en aval).

Ce document ne remplace pas une inspection physique — il existe
spécifiquement parce que cette inspection ne peut pas être automatisée
depuis ce poste. À faire par l'opérateur, robot à l'arrêt et hors tension.

## 1. Mesure de référence (avant intervention)

```bash
./tools/check_serial_health.sh
```

Note le taux d'erreur actuel (`erreurs / minute`) avant toute manipulation
physique — c'est la seule façon de savoir après coup si une intervention a
réellement changé quelque chose, plutôt que de se fier à une impression.

## 2. Points à vérifier physiquement, dans l'ordre

1. **Connecteur côté carte CORE2** — le connecteur du câble série est-il
   complètement enfoncé ? Un connecteur à mi-course peut fonctionner par
   intermittence selon les vibrations (le robot roule — les vibrations sont
   le facteur qui manque en test statique sur banc).
2. **Connecteur côté Raspberry Pi** (`/dev/serial0`, broches UART GPIO14/15
   sur le header 40 broches, ou adaptateur USB-série selon le montage
   actuel — à confirmer physiquement, la doc ne suffit pas ici) — mêmes
   vérifications : enfoncement complet, pas de jeu.
3. **État visuel du câble** — pincement, écrasement, isolant fendu,
   surtout aux points de flexion répétée (passage de câble près d'une
   charnière ou d'un bord de châssis).
4. **Test de flexion en direct** — `check_serial_health.sh` en continu
   (voir §3) pendant qu'on manipule doucement le câble sur toute sa
   longueur à la main. Si le taux d'erreur réagit à la manipulation à un
   endroit précis, c'est la localisation du défaut.
5. **Masse / blindage** — un défaut de masse commune Pi↔CORE2 peut produire
   exactly ce genre d'erreurs intermittentes plutôt qu'une panne franche.
   Vérifier qu'il n'y a pas de connecteur de masse supplémentaire débranché.
6. **Test de substitution** — si un câble/connecteur de rechange est
   disponible, le substituer et relancer la mesure de référence (§1) pour
   comparer. C'est le test le plus concluant : si le taux tombe à ~0 avec
   un autre câble, le câble est confirmé responsable.

## 3. Surveillance continue pendant la manipulation

```bash
watch -n 2 ./tools/check_serial_health.sh
```

## 4. Après intervention

Reprendre la mesure de référence (§1) dans les mêmes conditions
(robot allumé, immobile, quelques minutes de fonctionnement normal) et
comparer explicitement au chiffre du §1 — ne pas se contenter d'une
impression qualitative ("ça a l'air mieux").

## 5. Si le défaut ne se localise pas physiquement

Corréler un événement de désync horodaté avec l'état d'abonnement de
`firmware_message_converter` (item 19 du rapport) — nécessite d'instrumenter
les deux côtés simultanément, hors du périmètre de ce document.

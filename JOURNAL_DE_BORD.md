# RAPPORT FINAL D'EXPÉRIMENTATION — PROJET LEO ROVER
## Navigation Autonome par Détection de Balises Visuelles
### Système embarqué ROS Noetic · Intel RealSense D455 · Ubuntu 20.04

---

> **Classification** : Rapport technique interne  
> **Rédacteur** : Nicolas Gardon  
> **Dernière mise à jour** : 31 juillet 2026  
> **Version logicielle** : plateforme web + fusion MINS/VINS v3.35  
> **Dépôt** : `/home/lab272/TOUT/`

---

## TABLE DES MATIÈRES

1. [Synthèse du projet](#1-synthèse-du-projet)
2. [Historique des versions](#2-historique-des-versions)
3. [Architecture technique](#3-architecture-technique)
4. [Bilan des tests et corrections](#4-bilan-des-tests-et-corrections)
5. [Procédure d'utilisation (SOP)](#5-procédure-dutilisation-standard-operating-procedure)
6. [Planification des futurs tests](#6-planification-des-futurs-tests)
7. [Annexes](#7-annexes)

---

## 1. SYNTHÈSE DU PROJET

### 1.1 Objectifs

Le projet LEO Rover vise à développer un système de navigation autonome pour le robot terrestre LEO équipé d'une caméra RGB Intel RealSense D455. L'objectif final est qu'un robot soit capable de :

1. **Détecter des balises visuelles** (marqueur damier + 4 LED bleues) dans son champ de vue
2. **Se déplacer de manière autonome** vers une balise ciblée
3. **Réinitialiser son référentiel spatial** (coordonnées X/Y/Z → 0,0,0) à chaque balise validée
4. **Enchaîner les balises** en mode infini (recul + demi-tour + nouvelle recherche)
5. **Superviser la mission** depuis un PC via une interface graphique dédiée

### 1.2 Contraintes système

| Contrainte | Valeur | Justification |
|------------|--------|---------------|
| Système ROS | Noetic (ROS 1) | Firmware LEO `leo_bringup` non porté sur ROS 2 |
| Caméra | D455 USB3 (pas Pi CSI) | CSI débranchée volontairement ; D455 seule active |
| Réseau | Wi-Fi LEO (10.0.0.1) | Liaison unique PC↔Robot ; débit limite à ~30 Hz image |
| Python | 3.8 (Ubuntu 20.04) | Pas de f-strings 3.10+, pas de walrus operator |
| OpenCV | livrée avec ROS Noetic | Version système, pas de pip upgrade |
| matplotlib | 3.1.2 | Version ancienne : `labelcolor` kwarg absent |
| cv_bridge | Cassé sur Pi | Conversion manuelle numpy (évitée : traitement sur PC) |

### 1.3 Périmètre fonctionnel atteint

```
┌──────────────────────────────────────────────────────────┐
│                   leo_dashboard.py                        │
│  ┌──────────┐  ┌───────────────┐  ┌──────────────────┐  │
│  │ CAMÉRA   │  │  DÉTECTION    │  │   NAVIGATION     │  │
│  │ SSH auto │  │  Damier+4LED  │  │  AUTO/MANUEL/    │  │
│  │ 30 FPS   │  │  ~10 Hz       │  │  CIBLER/INFINI   │  │
│  └──────────┘  └───────────────┘  └──────────────────┘  │
│  ┌──────────┐  ┌───────────────┐  ┌──────────────────┐  │
│  │  CARTE   │  │  GRAPHIQUES   │  │  TÉLÉMÉTRIE      │  │
│  │  Trajectoire│ Vitesse+dist  │  │  Batt/Roues/Hz   │  │
│  │  + balises│  │  matplotlib   │  │  Chrono mission  │  │
│  └──────────┘  └───────────────┘  └──────────────────┘  │
└──────────────────────────────────────────────────────────┘
                         ↕ Wi-Fi 10.0.0.1
┌──────────────────────────────────────────────────────────┐
│                   Robot LEO (Pi 4)                        │
│   realsense2_camera  │  leo_bringup  │  /cmd_vel sub     │
│   /camera/color/...  │  /wheel_odom  │  /battery         │
└──────────────────────────────────────────────────────────┘
```

---

> **Processus de documentation (depuis le 2026-07-07)** : ce journal ET le rapport LaTeX
> (`report_latex/`, publié sur le site : `/rapport.pdf`) sont incrémentés à chaque avancée
> du projet. Le log d'événements du site (`logbook.html`) est alimenté automatiquement en
> continu par `leo_backend` (`web/auto_entries.json`). Références détaillées :
> `NOTES_SESSION_2026-07-04.md`, `MINS_TUNING_AUDIT.md`.

## 2. HISTORIQUE DES VERSIONS

### 2.1 Tableau des évolutions majeures

> **Note de provenance (31/07/2026)** : les entrées `v3.33.1` à `v3.33.6`
> (22, 23, 24, 27, 28 et 29 juillet) ont été rédigées **rétroactivement** le
> 31 juillet, à partir des sources du rapport (`report_latex/Fusion_Campaign.tex`),
> des logs de session et des fichiers modifiés. Elles n'ont pas été consignées
> au jour le jour. La numérotation intercalaire préserve les versions déjà
> publiées plutôt que de les renuméroter.


| Version | Date | Fichier | Description | Statut |
|---------|------|---------|-------------|--------|
| **v3.33** | 2026-07-21 | `leo_backend.py` + `web/app.js` + `web/i18n.js` + robot (`/var/ros/log`, journal, `leo.service`) + `report_latex/*.tex` | **Mode AUTO durci (4 correctifs) puis, en plein test, une VRAIE panne robot (disque plein → crash du service) trouvée et réparée, avant de découvrir le trou de sécurité le plus important du jour.** (1) DIAGNOSTIC MODE AUTO (« loin d'être satisfaisant » — patrouille erratique, évitement peu fiable, lock capricieux) : les 3 causes trouvées par LECTURE DE CODE (pas mesure — le motif était assez structurel pour ça), un seul schéma répété : une décision recalculée chaque tick à partir d'UN SEUL échantillon brut, sans mémoire du tick précédent. Cap « grand chemin » de patrouille : lissé par EMA (α=0,3) au lieu de suivre chaque bascule brutale entre deux secteurs de profondeur voisins et bruités. Direction de contournement d'obstacle : les médianes profondeur G/D lissées par EMA (α=0,4) avant de choisir le sens du pivot, au lieu d'une seule frame instantanée pouvant faire pivoter du mauvais côté. Clignotement du lock balise : hystérésis ajoutée sur le seuil de centrage (entrée ±8 %, sortie ±14 % au lieu d'un seuil unique) pour ne plus alterner avance/recentrage à chaque tick quand la détection tremble près de la frontière. Les 3 déployés à chaud. (2) PENDANT LE TEST QUI DEVAIT VALIDER CES CORRECTIFS : caméra qui refuse de revenir même après 2 resets matériels (jamais vu jusqu'ici) — une commande SSH sans rapport révèle la vraie cause : **disque du Raspberry Pi à 100 %, 0 octet libre**. Nettoyage (autorisé explicitement par l'opérateur, le garde-fou anti-destruction a refusé un `rm -rf` sans confirmation) : 2,6 Go de logs `roslaunch` jamais purgés (`/var/ros/log`, 37 dossiers horodatés) + 2,9 Go de journal systemd → 5,3 Go libérés, disque 100 %→82 %. Puis DEUXIÈME panne, plus grave, révélée seulement parce que la première venait d'être réglée : `leo.service` avait crashé (SIGSEGV, exit-code 139, plausiblement une écriture qui a heurté le disque plein) et, sans politique de redémarrage, restait mort — TOUTE l'instabilité des 25 minutes suivantes (master ROS injoignable pendant qu'ICMP/SSH restaient sains, `restart_stack.sh` qui retente contre un master inexistant, dizaines de nœuds zombies `wait_mins_*`/`mon_tour_*` accumulés) venait de ce service mort, pas d'un nouveau problème réseau. `sudo systemctl restart leo` a tout réglé instantanément une fois trouvé. (3) LE VRAI TROU DE SÉCURITÉ, signalé en direct par l'opérateur au test suivant (« il fonçait dans les murs quand il était en face d'eux ») : le seul garde-fou anti-conduite-aveugle du mode AUTO vérifiait `cam_alive` (flux couleur/IR, sert à la détection de balise) — mais la détection d'obstacles vient d'un flux DIFFÉRENT (`/pc/camera/depth/...` vs `/pc/camera/infra1/...`), republié séparément sur le même WiFi déjà documenté comme sous contention aujourd'hui (item 14 du rapport). Si le depth spécifiquement se fige pendant que l'IR continue, `cam_alive` reste vrai et PATROL continuait d'avancer sur un drapeau obstacle périmé ou absent — aucune différence avec un capteur débranché. Corrigé : nouvelle garde `DEPTH_ALIVE_TIMEOUT` (1,0 s, volontairement plus stricte que `CAM_ALIVE_TIMEOUT`=2,0 s puisque celle-ci protège directement contre la collision — borne le trajet aveugle à 20 cm à ADVANCE_LIN=0,2 m/s, bien en dessous de la marge de déclenchement obstacle à 600 mm) au même endroit que la garde caméra existante dans `_drive()`, plus un nouvel état FSM `AUTO_NO_DEPTH` (cockpit + i18n FR/EN) pour que l'arrêt soit visible et expliqué plutôt que silencieux. RAPPORT : deux nouvelles sous-sections (`sec:automode`, `sec:diskcrash`), items 15-16 ajoutés au registre (validation terrain du mode AUTO encore à faire — le créneau a été mangé par la panne disque ; sonde d'espace disque + `Restart=on-failure` pour `leo.service` à ajouter). Recompilé propre (0 erreur, 0 référence non résolue, 0 débordement), 235→238 pages. Leçon du jour, une fois de plus : une supposition plausible (« sûrement la batterie ») aurait manqué la vraie cause — c'est une commande SSH sans rapport avec le diagnostic en cours qui a révélé le disque plein. | **ACTUEL** |
| **v3.33.1** | 2026-07-22 | `leo_backend.py` + `catkin_ws/src/leo_navigation/**` (MINS `ROSPublisher`/`ROSHelper`/`SimVisualizer`) | **Première application du contrôle continu, et une panne de deux heures diagnostiquée comme réseau alors qu'elle était logicielle.** (1) **Reset balise conditionné à double confirmation** : le reset du référentiel n'est plus déclenché par la seule détection LED, mais par l'accord de deux indices indépendants (LED + damier), avec demi-tour post-visite corrigé. (2) **PID sur trois boucles du mode AUTO** : patrouille, centrage balise et gouverneur de vitesse, en réutilisant les EMA anti-chatter déjà en place comme entrée du terme dérivé plutôt qu'en dérivant le signal brut. (3) **Item 10 récidive** : `firmware_message_converter` resté enregistré auprès du master tout en publiant zéro `/joint_states`, ce qui bloque MINS indéfiniment sans aucune alerte. Environ 2 h perdues sur des hypothèses réseau avant de reconnaître le même défaut que le 20/07. Le correctif recommandé (sonde de débit, pas de présence) n'était toujours pas implémenté, d'où la rechute. (4) **Migration tf1 → tf2_ros de MINS** : trois fichiers portés sur le build catkin réel, compilés et vérifiés en direct (`/tf_mins` valide). (5) Environ 20 000 erreurs de checksum série sur la liaison Pi↔CORE2 constatées sans être expliquées (cause trouvée seulement le 28/07). | ✅ |
| **v3.33.2** | 2026-07-23 | `leo_backend.py` + `catkin_ws/src/leo_navigation/launch/*` + `web/app.js` | **Retour à la base reconstruit, sélection de source de pose simplifiée après deux itérations, et un garde-fou de plausibilité sur VINS.** (1) **Return-to-Base** : préservation du repère monde et conservation de la trajectoire complète, au lieu d'un retour calculé sur la seule position courante. (2) **Sélection de source de pose** : deux itérations avant d'arriver à une forme simple et honnête, l'interface ne prétendant plus qu'un basculement est acquis quand il n'est qu'armé. (3) **Divergence d'initialisation VINS** : position initiale aberrante détectée, garde de plausibilité ajouté. La cause profonde (accéléromètre hors spec) ne sera trouvée que le 28/07 ; le garde traite le symptôme. | ✅ |
| **v3.33.3** | 2026-07-24 | `leo_backend.py` + `catkin_ws/src/leo_navigation/**` + `web/app.js` | **Un incident critique de sécurité, un écart documentation/réalité sur le matériel, et six correctifs de comportement.** (1) **Marche arrière aveugle du Return-to-Base (critique)** : le robot reculait sans aucun capteur orienté vers l'arrière. Le LEO Rover n'a pas de capteur arrière, donc la capacité manquante ne peut pas être ajoutée en logiciel : la manœuvre a été supprimée et remplacée par des primitives sûres. (2) **Deuxième écart constructeur / tel-que-construit** documenté sur la plateforme. (3) **Approche sans distance métrique** : deuxième angle mort, l'approche se faisait sur une fraction d'image sans échelle. (4) **Médiane contre moyenne** dans la sélection de direction : angle mort statistique corrigé, le dommage vit dans la queue de distribution. (5) **Simplification comportementale délibérée** : demi-tours à angle fixe et déterministes, plus faciles à valider qu'une politique adaptative. (6) **Boost manuel** borné, et **enveloppe de détection étendue à 0,5–4 m** sur un critère d'aire absolue. (7) **Audit tf2_ros complet** : une dernière dépendance héritée trouvée et corrigée. | ✅ |
| **v3.33.4** | 2026-07-27 | `tools/{bag_to_csv.py,plot_trajectories.m,compare_tests.m,record_trajectories.sh}` + `leo_backend.py` + `web/*` + `/etc/ros/robot.launch` (robot) | **Une chaîne de comparaison hors ligne construite de bout en bout, et un bouton qui « s'ouvre puis se ferme » dont la vraie cause était la troisième.** (1) **Pipeline rosbag → CSV → MATLAB** et export en un clic depuis le cockpit. (2) **MATLAB détaché : trois modes de défaillance empilés** : d'abord un double-fork mal tué (`killpg`), puis un `stdin` non-tty faisant quitter `-r` proprement (contourné par `script` et un pty), puis la **vraie** cause finale : `leo_backend` tournant sans session utilisateur (`XDG_RUNTIME_DIR` et DBUS absents sous `cron.service`). Prouvé par test A/B et vérifié plus de 105 s sur le vrai bouton. Piège associé : `kill -0` sur un PID recyclé ment, il faut lire `/proc/<pid>/cmdline`. (3) **Lecture 6-DOF de la balise Carolus** et seuils LED fondés sur la géométrie plutôt que sur des constantes. (4) **Flux couleur D455 réactivé** à distance de travail, avec throttle CPU (`topic_tools/throttle` à 4 Hz). (5) **Protocole à deux tests** Test 1 (MINS) / Test 2 (openVINS), sélecteur cockpit et métrique de fermeture de boucle. (6) **Exclusivité de topic rétablie** : `/mins/external_ref/carolus` est le créneau VICON privé de MINS (règle du 08/07), violé par l'enregistreur de trajectoires puis corrigé. (7) **Enregistrement robuste par rosbag** : le site web n'est pas fiable pour un essai qui compte, le buffer du backend étant vidé à chaque redémarrage. | ✅ |
| **v3.33.5** | 2026-07-28 | `/etc/ros/` (robot, drop-in `leo.service`) + `catkin_ws/src/open_vins/config/**` + `report_latex/Fusion_Campaign.tex` | **Trois découvertes majeures qui forment une seule chaîne causale : une liaison série affamée, un accéléromètre hors spec, et un booléen valant quatre ordres de grandeur.** (1) **Overruns UART = famine CPU, pas défaut matériel** : le compteur noyau `oe` à 137 000 et progressant de 63 par 10 s a tranché là où les erreurs de checksum ne tranchaient pas. Deux causes conjuguées : bridage thermique à 600 MHz et `serial_node` en priorité ordinaire. Correctif = `chrt -f 25` **et** drop-in `LimitRTPRIO=50` sur `leo.service` ; sans le second, le nœud série refuse de démarrer, ce qui présente comme « le correctif a cassé la liaison » et pousse à annuler la bonne modification. Résultat : odométrie roues 1 Hz → 19 Hz. Bonus : un `rosbridge_server` fantôme tournait **sur le robot** à 96 % d'un cœur pour ne servir personne, tué. (2) **`try_zupt: false` → 41 824 m ; `true` → 0,002 m.** Une seule ligne de `estimator_config.yaml`. À l'arrêt, la caméra n'apporte aucune parallaxe et l'accéléromètre ne voit que la gravité plus ses propres erreurs : sans mise à jour de vitesse nulle, le résidu s'intègre deux fois sans rien pour s'y opposer. MINS n'en a pas besoin car les roues fournissent gratuitement et en continu la même contrainte. (3) **Accéléromètre CORE2 à 8,7525 m/s² contre 9,790 réels (−10,6 %)**, erreur dans le capteur brut et non dans le sanitizer. Le résidu de 1,06 m/s², doublement intégré, donne environ 1 900 m après une minute, ce qui est cohérent avec les 41 824 m observés sur environ 4,7 min. (4) **Crash mutex 1 324 fois, une fois toutes les 46 s** : deux causes, un commentaire en ligne trop long dans le YAML lu par `FileStorage` d'OpenCV au démarrage, et le pool de threads OpenCV à l'exécution (`num_opencv_threads: 1`). (5) **Piège méthodologique** : tester une configuration depuis une copie dans `/tmp` alors que le nœud lit celle du workspace donne un faux négatif. | ✅ |
| **v3.33.6** | 2026-07-29 | `tools/leo_watchdog.sh` + `tools/plot_trajectories.m` + `report_latex/*.tex` + `web/logbook.html` | **La cause prouvée des tempêtes de relances, qui invalide rétroactivement des mesures, et l'alignement de trajectoires traité comme une mesure et non comme un prétraitement.** (1) **Bug de PATH sous cron dans `leo_watchdog.sh`** : `setup.bash` était sourcé ligne 141, soit **après** les gardes `rosnode` des lignes 100 et 123. Sous cron le PATH est minimal, donc les deux gardes échouaient à chaque minute et la pile était détruite toutes les ~2 min. Source déplacée ligne 29 ; zéro relance automatique ensuite. **Toute mesure d'estimateur prise pendant une tempête est sans valeur**, ce qui invalide rétroactivement plusieurs comparaisons. (2) **Alignement rigide (Kabsch) contre similitude (Umeyama)** : identité ε²_rig − ε²_sim = σ_p²(s*−1)² établie, et distinction entre longueur de chemin cumulée et étendue spatiale sous bruit, E[L̂] ≈ L[1+(σf/v)²], quadratique en fréquence d'échantillonnage. (3) **Figure à neuf panneaux** et export HD par panneau à 200 dpi. (4) **Bug de rendu MATLAB** : `plot(x,y,'-')` au-delà de 300 points ne rasterise pas les segments plats dans ce MATLAB headless ; correctif = ajouter un marqueur `.`. Le fichier avait été vérifié par son existence, jamais par son contenu. (5) **Premières mesures de calibration par vérité terrain** sur cette plateforme. | ✅ |
| **v3.34** | 2026-07-30 | `tools/leo_watchdog.sh` + `leo_backend.py` + `web/{app.js,ops.html,trajectory.html,i18n.js,serve.py}` + `catkin_ws/src/leo_navigation/launch/*` + `tools/{plot_trajectories.m,record_trajectories.sh,bag_to_csv.py,update_report.sh}` + `report_latex/*.tex` | **Une cause racine trouvée invalide deux jours de mesures, un correctif de ma main s'avère être le défaut qu'il devait corriger, et la chaîne Carolus est débloquée par un contournement gratuit.** (1) **WATCHDOG — cause PROUVÉE des tempêtes de relances** : `leo_watchdog.sh` sourçait `/opt/ros/noetic/setup.bash` **ligne 141**, alors que ses deux gardes de présence appellent `rosnode` **lignes 100 et 123**. Sous cron le PATH est minimal → `rosnode` introuvable → les deux vérifications échouaient CHAQUE MINUTE quoi que fasse le robot → `restart_stack.sh` en boucle. Mesuré : **6 relances en 14 min**, pile détruite toutes les ~2 min. Ces gardes n'avaient donc JAMAIS supervisé quoi que ce soit sous cron — ils ne faisaient que détruire. Le bug était invisible en ligne de commande (PATH déjà bon). **Conséquence rétroactive : toute mesure d'estimateur antérieure au correctif est suspecte**, openVINS n'ayant jamais eu plus de 2 min pour converger. Après correctif : 0 relance automatique en 18 h. (2) **GARDE-FOU VINS RETIRÉ — il aggravait ce qu'il devait corriger.** Écrit la veille pour geler la position d'openVINS en rotation. Mesure décisive, robot vérifié IMMOBILE (odométrie roues = 0,000 m sur 30 s) : openVINS brut = 16,99 m de chemin cumulé mais **0,021 m de déplacement NET** (il oscille, il ne dérive pas) ; sortie du garde = **5,914 m de dérive nette**. Le garde rejetait 12 % des incréments d'une oscillation SYMÉTRIQUE : retirer unilatéralement une fraction d'un bruit de moyenne nulle ne le supprime pas, ça le REDRESSE en dérive (87,10 m d'offset accumulé pour zéro mouvement réel). Le symptôme initial (ratio 30,2 en rotation) avait été mesuré PENDANT la tempête du point (1). Nœud retiré du launch, fichier conservé comme trace du raisonnement fautif. **Leçon : corriger les entrées, jamais les sorties d'un estimateur.** (3) **CAROLUS 6 DDL AU COCKPIT (exigence superviseur)** : le panneau « Beacon LED Reset » affichait X/Y/Yaw de la pose INTERNE du robot (`d.pose`) sous un titre de balise — 3 DDL sur 6, et pas la bonne source. Ajout d'un bloc distinct (accent violet, bordure) avec les 6 DDL, converti quaternion→RPY côté backend (clamp sur `asin` : une norme à 1,0000001 par arrondi lèverait un ValueError et tuerait la télémétrie) et **fraîcheur calculée côté backend** (PC et robot n'ont pas la même horloge). **Chaîne TF morte découverte** : `carolus_vicon_bridge` attend un TF `beacon_link` que RIEN ne publie (le `tf_bridge.py` que sa documentation cite est absent du dépôt) → `/mins/external_ref/carolus` toujours vide. Contournement gratuit : le détecteur publie sur **`/pose`** (`carolus_astrobee.cpp:362`) — le topic nommé par le superviseur — et le paramètre `~carolus_fix_topic` rendu configurable la veille permet d'y pointer sans une ligne de code. **Déclaré dans le launch et non par `rosparam set`** : le serveur de paramètres vit dans le master (sur le ROBOT), un redémarrage robot — justement prévu pour `enable_color` — l'aurait effacé en silence. (4) **MATLAB — erreur 5001** : « s'ouvre puis se ferme » élucidé par le journal (`Unable to launch MVM server: License Error`). Le processus étant volontairement DÉTACHÉ (double-fork), le backend ne pouvait pas récolter son code de sortie : ajout d'une surveillance du journal qui publie un verdict (`starting`/`running`/`failed`) en télémétrie, plus 4 garanties d'interface (verrouillage du bouton, notification par couleur, **repli téléchargement du .mat**, timeout 150 s). **NB : aucune licence node-locked n'a été installée** — `~/.matlab/R2025b_licenses/license.lic.save` fait 90 octets sans aucune ligne `INCREMENT`/`SERVER`/`DAEMON` et `licmode` vaut toujours `onlinelicensing`. MATLAB refonctionne parce que **le service MathWorks est revenu** (licence Academic Total Headcount 993588, valide jusqu'au 31/12/2026). L'erreur 5001 peut donc revenir ; le garde-fou la signalera. Vraie solution durable : licence RÉSEAU FlexLM du campus (`MLM_LICENSE_FILE=port@serveur`). (5) **EXPORTS HD + RAPPORT** : les 8 panneaux régénérés en 200 dpi polices agrandies (**~1620×1100 px** contre 730×425 pour les recadrages de secours), rapport recompilé — **351 pages, 0 erreur, 0 référence non résolue**. Contenu ajouté : §12.3.3 calibration inertielle (argument d'observabilité — l'information de Fisher de $K_g$ est identiquement nulle à l'arrêt, d'où `gyro_scale` laissé à 1,0), §12.13 alignement de trajectoire (identité $\varepsilon^2_{rig}-\varepsilon^2_{sim}=\sigma_p^2(s^\star-1)^2$, vérifiée sur données réelles), annexe D refondue (~35 pages) dont §D.7 architecture des launch files. (6) **CACHE — deux pièges** : `serve.py` n'envoyait `no-cache` que sur `.html/.json` ; le rapport PDF était figé 4 h par Cloudflare (règle du tableau de bord écrasant l'en-tête d'origine → estampille `?v=` ajoutée par `update_report.sh`), et les `.js` versionnés par `?v=rNN` étaient servis périmés car j'avais modifié `i18n.js`/`app.js` **sans incrémenter l'estampille** (clés i18n affichées en brut, `app.js` obsolète faisant croire que le bouton Export était cassé). Estampilles réalignées (elles avaient DÉRIVÉ entre pages : `app.js` en r28 sur 2 pages, r30 sur une autre) + `.js/.css` désormais revalidés. | ✅ En service |
| **v3.35** | 2026-07-31 | robot (`/var/ros/log`) + `report_latex/{Fusion_Campaign.tex,appendix.tex,Formatting.tex,main.tex}` + `JOURNAL_DE_BORD.md` | **Effondrement de la liaison série la veille au soir, récupération, puis remise à niveau documentaire complète et sécurisation du disque.** (1) **30/07 au soir — effondrement complet de la chaîne firmware** : `wheel_states`, `imu/data_raw` et `joint_states` tous à 0 Hz, 676 751 overruns UART, `serial_node` en boucle `Lost sync`, à 89,1 °C et 600 MHz avec une charge de 8,85 sur 4 cœurs et le nodelet couleur à 185 % de CPU. Manuel et auto tombent **ensemble** parce que `/serial_node` est l'unique abonné de `/cmd_vel` : cette simultanéité élimine la logique de mission et l'interface web avant tout test. Récupération par cycle `rosmon` ciblé (17,1 / 18,7 / 79,3 Hz rétablis), mais +553 overruns dans les 3 min suivantes : un reset, pas une réparation. **Enveloppe mesurée des deux côtés : la liaison tient à 86 °C et lâche à 89 °C, soit environ 3 °C de marge.** (2) **Correction d'une mesure antérieure fausse** : l'affirmation « couper la couleur ne donne aucun gain thermique » reposait sur un test invalide, la couleur étant déjà désactivée au niveau driver. Le nodelet coûte en réalité 185 % de CPU. (3) **Rapport LaTeX** : session du 28/07 rédigée (elle manquait entièrement, le chapitre sautait du 27 au 29), refonte de l'annexe D avec quatre sections de référence orphelines replacées, transport web à deux ports documenté, protocole « hit-and-run », ordre des opérations pour le lacet, procédure d'acquisition d'erreur absolue. 240 tirets cadratins convertis contextuellement, cinq figures TikZ ajoutées, 19 équations. Deux défauts préexistants révélés au passage : un fragment de phrase orphelin et cinq cellules de tableau vidées de leur sens. (4) **Disque robot sécurisé** : 27 répertoires de sessions roscore mortes supprimés dans `/var/ros/log`, **2 075 Mo libérés**, 98 % → 91 %. Piège évité : `latest` est un lien symbolique vers la session active et le motif `*/` l'inclut ; le supprimer aurait effacé les logs en cours. (5) **Point de rigueur** : le robot injoignable le 30 au soir l'était de façon transitoire (roaming WireGuard), pas par destruction. Et aucune donnée d'erreur absolue n'a encore été acquise. | ✅ |
| **v3.32** | 2026-07-21 | `leo_backend.py` + `web/app.js` + `report_latex/*.tex` | **Deux vraies pannes de terrain diagnostiquées ET corrigées EN DIRECT pendant que l'opérateur conduisait, sur sa demande explicite.** (1) MOUVEMENT SACCADÉ (« bouge 1 sec, s'arrête, rebouge ») : cause trouvée après avoir confirmé que l'opérateur pilote depuis un téléphone qui MARCHE avec le robot (pas depuis le PC fixe) — `MANUAL_DEADMAN` du backend (500 ms sans commande fraîche → arrêt moteur, sécurité « homme-mort ») était dimensionné pour l'époque Ethernet ; le téléphone de l'opérateur fait maintenant du roaming WiFi entre bornes campus exactement comme le robot lui-même, et la gigue normale dépasse régulièrement 500 ms sans aller jusqu'à une vraie déconnexion. Élargi à 1,0 s (toujours <50 cm de roue libre à 0,2 m/s) ; délai de reconnexion WebSocket du frontend raccourci 2,5 s→1,0 s pour les vraies coupures. Les deux déployés à chaud (kill du process supervisé + `respawn="true"` relance avec le code édité, timing process-start vs fichier-mtime vérifié avant chaque annonce « c'est en ligne »). (2) DÉRIVE GAUCHE : mesurée en direct, pas supposée — écouteur `/firmware/wheel_states` pendant 30 s de conduite réelle : les 4 roues tournent en parfaite symétrie de commande (ratio D/G=1,0000 sur 536 échantillons) → PAS un défaut moteur/PID. Corrélation avec `/imu/data_clean` sur les fenêtres à ω roues symétrique (<3 %, 289 échantillons) : biais gyro réel +0,0530 rad/s à ω≈3,22 rad/s, soit ~1,65 % d'asymétrie de rayon effectif droite/gauche — cohérent avec l'item 1 du rapport (jamais mesuré au réglet). Trim proportionnel ajouté au point de sortie unique des commandes (même endroit que la correction de polarité existante), initialisé en annulant exactement le biais mesuré (K=−0,264 rad/s par m/s). PREMIÈRE SURPRISE : ce trim plein a fait dériver le robot À DROITE sur DEUX essais de suite — une remesure automatisée PENDANT un de ces essais montrait pourtant un résidu quasi nul (+0,0085 rad/s, dans le bruit gyro 0,167 rad/s), en contradiction directe avec le ressenti opérateur. Explication retenue : l'opérateur qui sent une dérive la compense naturellement au pilotage, donc une mesure passive prise PENDANT qu'il conduit moyenne le biais physique ET sa propre compensation — ici, contrairement à la leçon habituelle du jour (log > hypothèse formée en direct), c'est le retour humain en temps réel qui était le signal fiable, pas la mesure automatisée. Convergé par bissection sur le seul retour qualitatif de l'opérateur en 4 itérations (K=−0,264→« trop » à droite ; −0,132→« un peu trop encore » à droite ; −0,075→« très très légèrement » à gauche, a franchi le zéro ; −0,087→« beaucoup mieux, restons comme ça », accepté) — moins de 10 minutes montre en main, conduite comprise. Valeur finale environ le tiers du trim de pleine annulation naïve : rappel explicite dans le code et le rapport que c'est un correctif logiciel provisoire, pas un substitut à la vraie mesure roll-out (à retirer une fois `config_wheel.yaml` corrigé). RAPPORT : deux nouvelles sous-sections dans `Fusion_Campaign.tex` (`sec:manualdeadman`, `sec:yawtrim`, avec table de convergence de la bissection), items 1 et 14 du registre mis à jour (item 14 : cause dominante du mouvement saccadé maintenant identifiée et corrigée, la disponibilité de pose et la saturation WiFi restent des contributeurs secondaires non distingués, surtout pour le symptôme séparé des obstacles manqués). Recompilé propre (0 erreur, 0 référence non résolue, 0 débordement), 233→235 pages. | Remplacé (voir v3.33) |
| **v3.31** | 2026-07-21 | WireGuard (PC+robot) + architecture réseau + `report_latex/*.tex` | **Panne WireGuard non résolue → pivot architectural (WiFi même sous-réseau) → tour du bâtiment complet → rapport complété avec 3 vrais bugs LaTeX trouvés en compilant.** (1) RÉSEAU : le tunnel WireGuard (solution v3.23) a d'abord dérivé une seconde fois (endpoint robot vu à `10.0.0.10:55458`, sous-réseau du hotspot du robot lui-même — comportement de roaming WireGuard formalisé $E_A(t)=\mathrm{src}(p^*)$, $p^*=\arg\max_{\tau(p)<t}\tau(p)$, aucun fichier de config n'était en cause) puis, après un premier correctif (stop/down/start propre des deux côtés — piège trouvé : `wg-quick down` manuel désynchronise systemd du noyau, `systemctl stop` avant `start` requis), a subi une SECONDE panne différente, résistant à tous les remèdes (relances des deux côtés, changement de port 51820↔51821, attente 4 min) alors qu'ICMP/SSH vers le robot restaient parfaitement sains tout du long — non résolue, abandonnée après un audit honnête plutôt que poursuivie en essai-erreur indéfini (odeur d'intervention réseau côté campus, hors de portée d'ici). Sur demande explicite de l'opérateur (« je veux absolument être connecté en wifi sur FLTech-Guest »), PIVOT : PC rejoint `FLTech-Guest` directement en WiFi (même sous-réseau `10.154.0.0/19` que le robot) plutôt que de rester sur Ethernet+tunnel — **découverte architecturale generalisable** : le pare-feu campus qui a motivé tout le chantier WireGuard de la veille n'agit qu'à la frontière du routeur (L3, inter-VLAN) ; deux hôtes du MÊME sous-réseau se résolvent par ARP/commutateur sans jamais atteindre ce routeur, donc la politique de blocage TCPROS ne s'applique tout simplement pas ici — validé en direct (`/firmware/wheel_odom` à 20,1 Hz, MINS initialisé et basculé automatiquement, caméra et télémétrie confirmées vivantes, zéro tunnel dans la boucle). Contrepartie assumée et documentée : le PC revient sur le dongle WiFi `rtl88x2bu` et son bug pilote connu (item 13, remis sur le chemin critique). (2) TOUR DU BÂTIMENT : divergence MINS EN DIRECT repérée par l'opérateur (« pourquoi il se décale... alors que le robot ne bouge pas ») pendant que le robot était réellement immobile — mesurée sur 9,9 s/684 échantillons (Δz=−99,78 m, dérive planaire 32,0 mm/s, très hors du critère T1.1 <1 mm/s), corrigée par `restart_stack.sh` déclenché AVANT le garde-fou automatique du watchdog (100 m) plutôt que d'attendre, ré-vérifiée propre immédiatement après (0,1 mm/s, 13 mm) — même mécanisme de divergence que l'incident historique (accélération résiduelle non modélisée intégrée en quadrature, $\delta a = 2\|\Delta z\|/\Delta t^2 \approx 2,04$ m/s², un cinquième de g, plausible). Le même garde-fou s'est ensuite déclenché seul, sans intervention, à deux autres reprises pendant le trajet (14:57:14, 15:00:12) — la correction de la veille tient sous exposition réelle répétée. (3) STATS RÉELLES DU TRAJET (analyse directe de `watchdog.log`, fenêtre 12:58–15:34, 9351 s) : 21 déclenchements zombie-guard, cadence moyenne ≈447 s (contre 60–180 s le 20/07 — allongement ×3–7, confirme la direction du modèle de débounce sans éliminer le problème, item 9 partiellement clos) ; 21 tentatives de réinit MINS, médiane 21 s mais queue longue (90 s, 260 s, et 2 dépassements du plafond dur de 300 s — `TIMEOUT init MINS`, sortie propre par conception) → borne basse de disponibilité de pose $A_{\text{pose}}\gtrsim 1-(811+600)/9351\approx0,85$ ; caméra : 11 récupérations « muette », ZÉRO escalade au reset matériel (seuil élargi à 4 tient sous conditions réelles) ; balise A4 détectée avec succès (`/tag_detections` actif 15:34:12, en l'absence d'un tag A3) ; Return-to-Base non tenté. Retours terrain opérateur corrélés aux logs plutôt que pris au mot : avancée saccadée par à-coups d'1 s + murs parfois non détectés comme obstacles + pertes de connexion → expliqués par le plancher de disponibilité de pose ci-dessus (~15 % du temps sans pose MINS valide, un trou tous les ~7 min en moyenne) ET par un risque déjà signalé mais jamais mesuré (saturation WiFi des topics image, cf. [[leo-diagnostics-2026-07]]) — aujourd'hui est le premier trajet 100 % WiFi sans repli Ethernet, donc le premier où ce risque peut vraiment se manifester ; les deux causes ne sont pas distinguées (nouvel item 14, mesure Hz directe sur le Pi requise) ; dérive légère vers la gauche en ligne droite rapportée par l'opérateur → rattachée à l'item 1 existant (rayon/voie des roues jamais mesuré) plutôt qu'un nouvel item. (4) RAPPORT : nouvelle section « Field session, July 21 » ajoutée à `Fusion_Campaign.tex` (5 sous-sections, 2 nouvelles équations, 1 nouveau schéma TikZ deux panneaux cross-subnet vs same-subnet), registre des points ouverts étendu de 9 à 14 entrées, `Conclusion.tex` repassé pour cohérence (le tableau E1-E5 et le paragraphe de synthèse citaient encore l'ancien compte « neuf entrées » et affirmaient le WiFi PC « plus sur le chemin critique » — les deux corrigés pour refléter la réalité du jour). **3 vrais bugs LaTeX trouvés et corrigés en compilant** (jamais seulement supposé que ça compile) : (a) nœud TikZ avec un `\\` mais sans `align=center` → cascade de ~15 erreurs en aval, corrigé par l'ajout de l'option manquante ; (b) le tableau des points ouverts (14 lignes désormais) débordait la hauteur de page (numéro de page « 195 » retrouvé imprimé au milieu du texte de l'item 9 lors de la vérification visuelle par rendu image) → converti de `table`+`tabular` vers `longtable` (déjà chargé dans `Formatting.tex`) pour une pagination propre ; (c) même bug réapparu dans le tableau E1-E5 de `Conclusion.tex` après l'avoir allongé pour la cohérence → même correctif `longtable` appliqué. Recompilé propre à chaque étape (0 erreur, 0 référence non résolue, 0 débordement de page après le correctif final), 224→233 pages. Leçon : une vérification visuelle systématique par rendu image de chaque nouveau schéma/tableau (pas seulement « ça compile sans erreur ») a trouvé 2 des 3 bugs — un tableau qui déborde silencieusement la page ne déclenche pas toujours un avertissement `Overfull \vbox` exploitable. | Remplacé (voir v3.32) |
| **v3.30** | 2026-07-21 | robot (`firmware_message_converter`) + `web/rosbridge_websocket_tunnel.py` (process) | **Vraie panne du matin trouvée — sans rapport avec le réseau/débounce d'hier soir.** Avant même le début du test terrain du jour, MINS échouait à s'initialiser en boucle (« TIMEOUT init MINS » x2, relances complètes du watchdog toutes les ~15-20 min malgré le débounce élargi de v3.26). Diagnostic par lecture directe du log MINS : `[IW-Init]: Waiting for collecting IMU(493<3) and wheel data(0<3)` — IMU affluait normalement, **données roues à zéro**. Remontée de la chaîne : `wheel_remap` (PC) n'avait rien à transmettre car `/joint_states` n'avait **aucun publisher** (`rostopic info` confirmé), alors que `/firmware_message_converter` (robot) restait pourtant enregistré comme vivant — un nœud registré mais silencieux, panne différente de tout ce qui a été vu jusqu'ici. Cyclé via `/rosmon/start_stop` (stop puis start) côté robot : `/joint_states` reparti à ~18 Hz, `/joint_states_mins` confirmé, MINS a publié en moins de 20 s, bascule manuelle réussie (`switched to MINS`). En parallèle, un second processus zombie sans rapport (`rosbridge_websocket_tunnel`, bloqué depuis 03h16 sans jamais lier son port malgré un process vivant — la logique « patience, ça démarre encore » du watchdog n'a pas de plafond de temps et attendait indéfiniment) a été tué manuellement puis relancé proprement. Leçon : le débounce élargi d'hier soir fonctionne comme prévu (`x6/6` observé, cohérent avec le modèle), mais il masque aussi de VRAIES pannes robot en les traitant identiquement aux faux positifs réseau — les deux causes doivent être distinguées au cas par cas via le log MINS lui-même (`wheel data(0<3)` = panne réelle, pas juste un aller-retour XML-RPC raté). Item ouvert à ajouter : (1) ajouter une sonde spécifique `/joint_states` sans publisher au watchdog plutôt que de compter sur le seul timeout MINS ; (2) plafonner la patience « en cours de démarrage » du healer web (actuellement infinie). | Remplacé (voir v3.31) |
| **v3.29** | 2026-07-21 | `report_latex/*.tex` | **Renforcement mathématique + nettoyage stylistique du rapport (demande explicite : « plus de mathématiques », « enlève les tirets qui font très IA »).** (1) Trois nouveaux développements mathématiques dans `Fusion_Campaign.tex` : preuve algébrique en 3 lignes de la continuité bit-à-bit du switch SE(3) (jusqu'ici affirmée, pas démontrée) ; argument de vraisemblance maximale sur le modèle de débounce — les 4 intervalles observés dans `watchdog.log` donnent $\mathcal{L}(p)=p^{12}$, MLE à la frontière $p\to1$, rapports de vraisemblance $\Lambda(0.9,0.5)\approx1157$, $\Lambda(0.99,0.9)\approx3.1$ ; quantification de l'overhead cryptographique WireGuard (32 B/paquet fixe, $\approx$2,3 % du débit utile, borne le coût ChaCha20-Poly1305 à 3 ordres de grandeur sous le budget réseau). (2) Suppression de tous les tirets cadratins (—/---) du rapport : 424 candidats identifiés, 254 étaient de faux positifs à ignorer (bordures ASCII de schémas dans `lstlisting`, séparateurs de commentaires LaTeX invisibles dans `Formatting.tex`) ; 167 vraies occurrences en prose corrigées une par une (contexte lu à chaque fois, remplacées par deux-points/virgule/point-virgule/parenthèses selon le rôle grammatical, jamais un remplacement aveugle) dans `Fusion_Campaign.tex` (158), `Research_Methodology.tex` (3), `Conclusion.tex` (10), `appendix.tex` (2) ; 4 exceptions volontairement conservées (commentaire LaTeX invisible, citation verbatim d'un message de log, deux tirets N/A dans une cellule de tableau — convention typographique standard, pas un tic stylistique). Bonus cohérence : un déséquilibre de parenthèses préexistant repéré et corrigé au passage. Recompilé proprement 3 fois de suite (0 erreur, 0 référence non résolue), 221→224 pages. | Remplacé (voir v3.30) |
| **v3.28** | 2026-07-21 | `report_latex/*.tex` | **Passage de complétion maximale du rapport, sur demande explicite (« complète au maximum, maximum de mathématiques, cohérence, schéma réseau ultra clair »).** (1) Nouveau schéma TikZ avant/après de la topologie réseau (`Fusion_Campaign.tex`, Fig. 12.4) — deux itérations, la première illisible (chevauchements), corrigée et vérifiée par rendu image. (2) Modèle mathématique de fiabilité du débounce watchdog : dérivation formelle du temps moyen entre faux déclenchements $E_0(n,p)=(1-p^n)/((1-p)p^n)$ pour une suite de Bernoulli, ajustée sur les horodatages réels de `watchdog.log` (estime $p\gtrsim0.9$ pendant le trajet), justifiant quantitativement le choix 3→6 ticks (Table 12.3) et rattachée honnêtement à l'item ouvert (n'élimine pas le problème, le dilue). (3) Argument quantitatif pour `PersistentKeepalive=25s` (marge sous les timeout NAT/pare-feu usuels, RFC 4787). (4) Passage de cohérence globale : figure de topologie obsolète dans `Research_Methodology.tex` (adresses `10.0.0.10` de l'architecture originale) — note de renvoi ajoutée plutôt que réécriture (exactitude historique préservée) ; glossaire technique (`appendix.tex`) complété avec tout le vocabulaire de la campagne (MSCKF, MINS, OpenVINS, WireGuard, TCPROS, XML-RPC, SE(3), etc. — absent jusqu'ici) et bug de parenthèses systématique corrigé sur toutes les entrées ; `Introduction.tex` (structure du rapport) et `appendix.tex` (specs matérielles) mis à jour pour mentionner le chapitre de campagne, absent des deux. (5) **Écart de cohérence majeur trouvé et corrigé** : `Conclusion.tex` ne mentionnait absolument rien de la campagne de fusion de juillet 2026 (ni MSCKF/MINS, ni réseau, ni validation terrain) — le chapitre le plus long et le plus récent du rapport était absent de sa propre conclusion. Ajout d'une section de synthèse dédiée (table de 5 objectifs d'extension E1-E5), extension des contributions techniques et des leçons apprises (nouvelle sous-section « Reliability Engineering Lessons » sur la leçon de la nuit précédente : ne jamais figer une hypothèse posée en direct sans la vérifier aux logs), réconciliation de la feuille de route (SLAM) avec l'architecture MSCKF actuelle. Recompilé proprement (0 erreur, 0 référence non résolue après repasse), 213→221 pages. | Remplacé (voir v3.29) |
| **v3.27** | 2026-07-21 | `tools/leo_watchdog.sh` | **Audit préventif des autres sondes réseau avant le test terrain de demain.** Suite à v3.26 (le débounce zombie insuffisant sous roaming WiFi), passage en revue systématique de toutes les sondes du watchdog basées sur un aller-retour réseau vers le robot, pour la même classe de faux positif. Deux autres corrigées : (1) l'échelle d'escalade caméra — le cycle STOP/START (tick 1, coût faible) reste immédiat, mais le **RESET MATÉRIEL D455** (35+ s sans vision, mode AUTO qui refuse de bouger pendant ce temps) ne se déclenchait qu'après 2 échecs de 6 s — repoussé à 4 échecs, probe élargie à 12 s ; (2) la sonde de famine IMU (cycle serial_node), 2→4 ticks. Non touchées après analyse : la sonde rosbridge (vérifie le port local `127.0.0.1:9090`, pas le robot — hors de cause), le ping robot (échec = attente silencieuse, déjà sûr par conception), le garde anti-divergence MINS (un timeout réseau donne une chaîne vide, ignorée par construction — déjà sûr), la purge des republish zombies (signature asymétrique déjà spécifique). Compromis assumé et à surveiller : une vraie panne caméra prolongée (batterie faible, item encore ouvert) mettra maintenant ~4 min avant le reset matériel au lieu de ~2 — le cycle léger tick 1 continue cependant de s'appliquer à chaque minute entre-temps. Syntaxe vérifiée (`bash -n`). Non testé en conditions réelles (aucun trajet ce soir) — à confirmer lors des essais de demain annoncés par l'opérateur. | Remplacé (voir v3.28) |
| **v3.26** | 2026-07-21 | `tools/leo_watchdog.sh` | **🎯 VRAIE CAUSE DES BASCULES MINS↔VINS TROUVÉE — le watchdog relançait toute la pile toutes les 1-3 min pendant TOUT le test couloir.** L'item ouvert v3.24 (bascules transitoires pose_source) attribué en direct à une hypothèse de « starvation de features » était FAUX — corrélation a posteriori de `watchdog.log` avec les horodatages des bascules observées : coïncidence exacte (18:01, 18:04, 18:05, 18:07, 18:10, 18:13...) avec le garde-fou « zombies post-reboot » (`rosnode list` ne trouve plus `/mins_subscribe` → relance complète de la stack). Le débounce (3 ticks/10s), dimensionné contre un raté isolé, ne tenait pas sous le roaming WiFi soutenu de `wlan_int` en déplacement. **Le test a réussi malgré des relances complètes de la pile toutes les 1-3 min, pas parce que MINS a tourné sans interruption** — masqué par le comportement idempotent de `restart_stack.sh` et la politique de respawn, invisible pour l'opérateur. Correctif appliqué : débounce élargi (3→6 ticks, 10s→20s par tentative) ; le vrai positif (redémarrage robot réel) reste détecté, seul le taux de faux positifs en mouvement doit baisser. Non revalidé en conditions réelles ce soir (pas de nouveau trajet) — item ouvert reporté (registre §9, corrigé). Rapport LaTeX (`Fusion_Campaign.tex` §Field validation + table des points ouverts) corrigé pour refléter la vraie cause, recompilé (213 pages). Leçon : une hypothèse posée en direct pendant un test doit être vérifiée contre les logs avant d'être écrite dans un rapport définitif. | Remplacé (voir v3.27) |
| **v1.0** | pré-session | `leo_tracking_map.py` | Architecture NASA Mission Control. Détection AprilTag (`cv2.aruco`) + croix 4 LED. Thread unique. Pas d'interface graphique PC. | Archivé |
| **v2.0** | 2026-06-24 | `leo_dashboard.py` | Refonte complète : interface Tkinter tout-en-un sur PC. Connexion ROS via SSH automatisé (paramiko). Caméra D455 lancée automatiquement. Thread vision unique. | Remplacé |
| **v2.1** | 2026-06-24 | `leo_dashboard.py` | **Fix freeze vidéo** : séparation `_decode_loop` (30 FPS garanti) + `_detect_loop` (lourd, isolé). `tcp_nodelay=True` sur le subscriber caméra. | Remplacé |
| **v2.2** | 2026-06-24 | `leo_dashboard.py` | **Basculement vers détection hybride** : remplacement AprilTag par damier (`findChessboardCornersSB`) + ROI LED autour du damier. Suppression AprilTag du pipeline (−5 à −10 ms/frame). | Remplacé |
| **v2.3** | 2026-06-25 | `leo_dashboard.py` | **Fix compteur balises bloqué à 0** : suppression de l'appel bloquant `_reset_srv()` dans le thread de détection. Incrément `beacon_count` local et immédiat. | Remplacé |
| **v2.4** | 2026-06-25 | `leo_dashboard.py` | **Centre de contrôle de mission** : graphiques matplotlib temps réel (vitesse lin/ang + distance), bandeau indicateurs numériques (Hz caméra/odom, chrono mission, compteur balises), bouton Reset manuel, mode `Cibler ∞ balises`. | Remplacé |
| **v2.5** | 2026-06-25 | `leo_dashboard.py` | **Ergonomie** : graphiques repositionnés haut-droite (au-dessus de la carte). Fenêtre redimensionnée à 922×1062 px (compatible écran 1152 px). Carte réduite à 250×250 px. | Remplacé |
| **v3.0** | 2026-07-02→04 | `leo_backend.py` + `web/` | **Plateforme web headless** : cockpit navigateur (rosbridge + MJPEG + tunnel Cloudflare), FSM 6 états, calibration Kalibr, déploiement MINS+OpenVINS avec pose_selector SE(3), sanitizer IMU, transport WiFi compressé, T1.1 A/B (σZ 257→50 mm). | Remplacé |
| **v3.25** | 2026-07-21 | `/etc/ros/robot.launch` (Pi) | **Persistance `laser_power=0` / `png_level=1` au boot.** Réglage préparé de longue date (drafté, jamais appliqué) : ces deux paramètres `dynamic_reconfigure` ne survivaient à aucun redémarrage de `leo.service`. Ajout de deux nœuds one-shot (`set_laser_power_boot` sur `/camera/stereo_module`, `set_png_level_boot` sur `/camera/depth/image_rect_raw/compressedDepth`), retardés de 8 s (`launch-prefix sleep`) pour laisser le driver realsense publier ses serveurs `dynamic_reconfigure` — non `required`, sans respawn : un délai insuffisant un jour donné ne casse que ce réglage, pas le reste de la pile (leçon de l'incident 10 Hz déjà documenté). Précaution avant d'y toucher : batterie non vérifiable depuis mon propre environnement (lecture ROS via SSH bloquée comme d'habitude), l'opérateur a validé « vas-y, j'assume le risque ». Sauvegarde de l'ancien `robot.launch` conservée sur le robot (`robot.launch.bak.<horodatage>`). Appliqué et vérifié : XML validé avant redémarrage, `leo.service` redémarré, les deux nœuds confirmés `exited with status 0` dans `rosmon`/`rosout.log` (preuve fiable, contrairement aux lectures ROS live qui échouent depuis mon environnement), caméra ré-enregistrée (`rosnode list`), puis `restart_stack.sh` côté PC pour éviter le piège des connexions figées après un redémarrage robot (MINS réinitialisé en 8 s, bascule automatique confirmée). | Remplacé (voir v3.26) |
| **v3.24** | 2026-07-20 | Terrain (bâtiment complet, couloir) + `report_latex/` | **🎯 VALIDATION TERRAIN — tour complet du bâtiment en mode manuel sur `FLTech-Guest`/WireGuard, sans coupure exploitante.** Premier test réel hors labo depuis la migration réseau (v3.22→v3.23.1) : opérateur au joystick, robot promené dans tout le bâtiment. Surveillance en direct (ping + télémétrie toutes les 4 s) : lien WireGuard resté up quasiment tout le trajet (une seule perte ping isolée, ~5 s, auto-récupérée sans intervention) ; plusieurs bascules transitoires `pose_source` MINS→VINS (~5-6 s chacune) toujours suivies d'un retour spontané à MINS — aucune ne s'est traduite par une perte de tracking ni un arrêt du selector ; un aléa isolé de lecture télémétrie (`?\|?`, 1 cycle) sans suite. **Résultat : objectif de la migration réseau atteint et validé en conditions réelles** (voir [[leo-reseau-wireguard]]). Rapport LaTeX complété en conséquence (nouvelle sous-section « Campus-scale connectivity », §Operations infrastructure de `Fusion_Campaign.tex`) et recompilé. Item ouvert noté au registre : caractériser la cause des bascules MINS↔VINS transitoires (débit/latence en mouvement vs starvation de features). | Remplacé (voir v3.25) |
| **v3.23.1** | 2026-07-20 | `~/bin/leo` | **`leo restart` fait maintenant vraiment tout, en une seule commande.** Question posée deux fois par l'opérateur (« si je fais leo restart ça va marcher directement ? ») : la réponse était non — `leo restart` relançait web/cockpit/backend mais jamais la pile MINS, qui exigeait un `restart_stack.sh` séparé. Corrigé : nouvelle étape `[nav]` appelant `restart_stack.sh` (idempotent — sûr même si déjà lancé), intégrée entre le backend et le serveur web. Testé en direct de bout en bout : `leo restart` seul → rosbridge + caméra + backend + **MINS initialisé (17 s) + basculé automatiquement** → télémétrie confirmée (`cam_alive=true`, `pose_source=MINS`, caméra 14 Hz, odométrie 19 Hz) via le chemin WireGuard. Une seule commande, tout fonctionne. | Remplacé (voir v3.24) |
| **v3.23** | 2026-07-20 | WireGuard (`wg0`, robot+PC) | **🎯🎯 SOLUTION DÉFINITIVE — `sshuttle` abandonné, remplacé par WireGuard.** Après validation initiale, `sshuttle` s'est montré non fiable en usage réel (échecs reproductibles des données robot→PC même après redémarrage complet du PC ET du robot, limite de descripteurs testée sans effet) — diagnostic final : un relais TCP Python en espace utilisateur n'est pas taillé pour le trafic soutenu et multi-connexions d'une pile ROS complète, contrairement à un VPN noyau. REMPLACÉ par WireGuard : tunnel point-à-point (`10.200.0.1` PC ↔ `10.200.0.2` robot), un seul port UDP fixe (51820) qui traverse le pare-feu campus (testé 0 % de perte), chiffrement/routage au niveau noyau — plus de relais applicatif. `ROS_IP` du robot et `~/.leo_network` du PC basculés sur les adresses WireGuard. RÉSULTAT MESURÉ, très supérieur à `sshuttle` : caméra à **15 Hz** (nominal complet, contre 1 Hz avec l'ancienne solution), odométrie 18 Hz, MINS initialisé en 13 s (contre jusqu'à 196 s), `cam_alive=true`, `pose_source=MINS` stable. Persisté des deux côtés (`systemctl enable wg-quick@wg0`) — survit à tout redémarrage. `leo-tunnel.service` (sshuttle) désactivé, gardé en place pour référence/secours. Le robot opère désormais sur `FLTech-Guest` (bâtiment complet) avec des performances quasi identiques au réseau local d'origine. | Remplacé |
| **v3.22.2** | 2026-07-20 | `/etc/systemd/system/leo-tunnel.service` | **🎯 MIGRATION TERMINÉE — service permanent validé de bout en bout.** `sshuttle` converti en service systemd (`Restart=always`, `WantedBy=multi-user.target` — survit à un redémarrage du PC et à la fermeture de tout terminal). Un faux problème rencontré en route : après la conversion, `/mission/telemetry` échouait malgré un tunnel sain — diagnostic par le journal détaillé (`/var/log/leo-tunnel.log`) a montré une centaine de cycles connexion/fermeture en boucle : MINS/le backend, restés actifs pendant les 3 transitions du tunnel (premier plan → service v1 → service v2), étaient bloqués dans une tentative de reconnexion perpétuelle sur des sockets périmés — pas un défaut du tunnel. Un `restart_stack.sh` propre après stabilisation du service a résolu ça immédiatement. VALIDATION FINALE : `cam_alive=true`, `pose_source=MINS`, télémétrie complète reçue via le service permanent. Le robot peut désormais opérer n'importe où dans le bâtiment sur `FLTech-Guest` ; le PC reste ancré sur Ethernet (fiable, immunisé contre le pilote WiFi défaillant). Item ouvert : caméra à 1 Hz à travers le tunnel (vs 15 Hz nominal) — suffisant pour supervision, à optimiser si un test nécessite de la vision réactive. | Remplacé |
| **v3.22.1** | 2026-07-20 | — (validation) | **Chaîne complète confirmée de bout en bout via le tunnel `sshuttle`.** `/mission/telemetry` reçue en direct : cam_alive=true, pose_source=MINS, health (cam/odom/batt/wheels) tous true. Réserve notée : caméra à 1 Hz à travers le tunnel (vs 15 Hz nominal — coût du relais SSH) — suffisant pour supervision/télémétrie, insuffisant pour navigation réactive par vision en l'état ; à optimiser si besoin. Prochaine étape : conversion de sshuttle en service persistant (actuellement dépendant d'une fenêtre de terminal restée ouverte — risque avant toute démonstration). | Remplacé (sshuttle abandonné, cf. v3.23) |
| **v3.22** | 2026-07-20 | tunnel `sshuttle` + robot | **🎯 MIGRATION RÉSEAU VALIDÉE : le robot opère maintenant sur `FLTech-Guest` (couverture bâtiment complet), contexte validation NASA.** Robot déjà connecté en double-radio (wlan_ext=hotspot LeoRover-e138 intact, wlan_int=client FLTech-Guest 5 GHz, 10.154.27.235). ROS_IP du robot (`/etc/ros/setup.bash`) basculé sur cette adresse. OBSTACLE MAJEUR : le dongle WiFi USB du PC a un bug RÉEL du pilote (`rtl88x2bu`, UBSAN invalid-load dans `phydm_ccx.c:696` lors du changement de canal, confirmé par dmesg — 6 déconnexions/heure) → jamais stable sur FLTech-Guest. SOLUTION RETENUE : tunnel `sshuttle` (proxy transparent via SSH, port 22 — seul port qui traverse le pare-feu campus entre le filaire et l'invité, contrairement aux ports dynamiques TCPROS de ROS) contournant complètement le dongle défaillant, via le port Ethernet du PC (déjà fiable). Détours avant la solution finale : tunnel SSH natif (`-w` tun/tap) rejeté par le sshd du Pi (« channel 0: open failed » — fonctionnalité non compilée) ; clé SSH dédiée générée (aucune n'existait) ; sshuttle installé (pip --user PUIS pip système pour que `sudo` le voie) ; bug d'auto-exclusion sshuttle identifié (la connexion de contrôle SSH se coupait elle-même en redirigeant tout le trafic vers sa propre destination) → corrigé avec `-x 10.154.27.235/32:22` (exclusion explicite du port de contrôle). VALIDÉ : vraies données IMU reçues via le tunnel (`rostopic echo /firmware/imu` fonctionnel). OPÉRATIONNEL : sshuttle doit tourner dans un terminal dédié laissé ouvert (`sudo sshuttle -r pi@10.154.27.235 10.154.0.0/19 -x 10.154.27.235/32:22 --ssh-cmd "ssh -i ~/.ssh/id_leo_tunnel -o StrictHostKeyChecking=no"`) — à convertir en service persistant avant toute démonstration (risque : fermeture accidentelle du terminal = coupure). `~/.leo_network` du PC pointe déjà sur `10.154.27.235`. | Remplacé |
| **v3.21** | 2026-07-20 | `tools/robot_env.sh` + 5 scripts | **Préparation migration réseau (FloridaTech-Guest, couverture bâtiment complet — contexte validation NASA).** Point de vérité réseau centralisé créé : `tools/robot_env.sh` (ROBOT_HOST/ROS_MASTER_URI/ROS_IP, override via `~/.leo_network`) — les 5 endroits qui codaient `10.0.0.1`/`10.0.0.10` en dur (`~/bin/leo`, `leo_watchdog.sh`, `restart_stack.sh`, `launch_mins.sh`, `start_web.sh`) le sourcent désormais. Changer de réseau = une ligne dans `~/.leo_network`, plus jamais 5 fichiers. Comportement par défaut vérifié IDENTIQUE (10.0.0.1, hotspot actuel intact — rien de destructif tant que le nouveau réseau n'est pas prouvé). Testé : syntaxe des 5 scripts, bit +x préservé, bascule simulée (`leo.local`), et test d'intégration réel du watchdog en conditions dégradées (robot injoignable) — sortie propre, message correctement dynamique. BLOQUÉ pour la suite : (1) dongle WiFi USB du PC en panne (NO-CARRIER, pas d'accès sudo pour le réinitialiser) — empêche le test décisif d'isolation client / portail captif sur FloridaTech-Guest ; (2) topologie WiFi du robot (un seul radio partagé AP+client, ou deux adaptateurs ?) inconnue — nécessaire avant de préparer les commandes de bascule côté Pi sans deviner. | Remplacé |
| **v3.20.2** | 2026-07-20 | `web/vendor/` + 4 pages HTML | **Les 3 derniers CDN sûrs auto-hébergés.** Suite de l'audit CDN (v3.20) : roslib, chart.js, lucide passés en local. PIÈGE ÉVITÉ : `roslib@2.1.0` (« latest » actuel) est livré en module ES (`export {...}`) — chargé en `<script>` classique, `window.ROSLIB` n'existe jamais et TOUT le cockpit casse (`new ROSLIB.Ros()` échoue). Auto-hébergé à la place un instantané EXACT de l'URL non-versionnée actuellement en service (v1.4.1, UMD, `window.ROSLIB` confirmé) — zéro changement de comportement, juste plus de dépendance externe. chart.js (4.5.1) et lucide (1.25.0, remplace le dangereux `@latest` non figé) vérifiés UMD sains (`window.Chart`, `window.lucide.createIcons`) avant déploiement. Tailwind Play CDN volontairement INTACT (nécessite un vrai pipeline de build, hors de portée d'un simple téléchargement — reste un CDN par nécessité, documenté). | Remplacé |
| **v3.20.1** | 2026-07-20 | `~/bin/leo` | **Dernier point non-idempotent trouvé et corrigé.** Capture opérateur d'un `leo start` : « port 8000 occupé (PID 1333) → libération » — le serveur web tournait déjà sain, tué et remplacé pour rien. `~/bin/leo` a sa PROPRE logique de lancement (dupliquée, jamais mise à jour depuis la règle idempotente posée dans `web/start_web.sh` en v3.15.1) : `_free_port 8000` inconditionnel à chaque `leo start`, coupant tout cockpit ouvert. Corrigé (même garde : `_port_open` avant tout kill). Testé en direct : PID du serveur identique avant/après `leo start` — confirmé idempotent. Au passage : Pose Source affichait VINS sur une capture alors que MINS tournait déjà (badge figé, pas un vrai retour à VINS) — pose_selector `default_source: VINS` au lancement du launch file, à re-switcher manuellement ou via restart_stack.sh après tout redémarrage de la stack nav. | Remplacé |
| **v3.20** | 2026-07-20 | `web/fonts/` + 4 pages HTML + `start_web.sh` | **Polices auto-hébergées + tunnel public réparé.** Question opérateur : le lien « api » (Google Fonts, fonts.googleapis.com) ralentit-il le site/robot ? Mesuré : robot NON affecté (aucun couplage), site OUI — CSS bloquant le rendu + 3,6 Mo de polices en 13 fichiers, chargés sur les 4 pages, et joignables via le hotspot du robot lui-même (donc en concurrence directe avec le trafic ROS sur le canal 1 déjà saturé, cf. diagnostic du jour). Corrigé : les 76 blocs @font-face du CSS Google filtrés aux sous-ensembles latin+latin-ext (accents FR/EN), 26 fichiers woff2 téléchargés une fois (1,1 Mo total, contre 3,6 Mo à chaque première visite), servis localement — plus aucune requête externe, plus de blocage de rendu. Audit complémentaire (non corrigé, signalé) : 4 autres CDN externes sur le cockpit (Tailwind Play CDN — explicitement déconseillé en production, roslib — essentiel, chart.js, lucide@latest — version non figée, risque de rupture silencieuse). En cours de route : tunnel public trouvé MORT (cloudflared introuvable sous cron — PATH minimal sans ~/.local/bin, même piège que l'incident watchdog du 08/07) → chemin absolu, tunnel restauré et vérifié (cockpit.leo-rover-gardon.dev 200 OK). | Remplacé |
| **v3.19** | 2026-07-20 | `leo_backend.py` + `web/ops.html` + `app.js` | **Bouton « Relancer la caméra » au cockpit.** La caméra était tombée ~12 h avant que le watchdog ne la guérisse seul (718 échecs comptés) — l'opérateur a demandé le même geste, mais déclenchable à la demande. Nouvelle commande backend `recover_camera` (thread dédié, cooldown 15 s) : cycle rosmon STOP/START (RESTART=3 disponible mais le cycle explicite log chaque étape), option reset matériel D455, purge des republish PC, réapplication laser 0 + png_level 1 — reprend exactement l'échelle du watchdog sans attendre son escalade (jusqu'à plusieurs minutes). Bouton ambre sous le panneau vidéo, désactivé + spinner pendant la relance (20 s). Testé de bout en bout : commande → 11 s → flux PC 0→15 Hz. app.js r24, i18n.js r9m. | Remplacé |
| **v3.18** | 2026-07-13 | `pose_selector.py` + configs + watchdog | **Le tracé ne peut plus « faire n'importe quoi ».** Mécanisme identifié : coupure caméra → MINS en inertie pure → biais accéléro intégré 2× → bonds de plusieurs m/s gravés dans la trace, et le chi2 rejette les roues (v=0) au moment où elles pourraient ancrer. Triple remède : (1) GARDE DE VRAISEMBLANCE dans pose_selector — toute pose impliquant >1,5 m/s (robot plafonné à 0,4) GÈLE la pose servie ; au retour au calme, ré-ancrage SE3 sur la dernière pose saine (l'excursion n'atteint jamais trace/carte/RTB, continuité préservée) ; (2) noise_v roues 0,5→0,2 (l'ancre v=0 agit AVANT l'emballement) ; (3) sonde FAMINE IMU au watchdog : serial_node rosserial dégradé (90 Hz produits, 3-8 Hz poussés, radio innocentée par comparaison camera_info) → cycle rosmon auto — le remède mesuré 3→79 Hz qui avait coûté 20 min de diagnostic. Épilogue soirée : antenne WiFi = cause racine de la journée réseau (RTT 2-422 ms → stabilisé après refixation). | Remplacé |
| **v3.17** | 2026-07-13 | backend + watchdog | **2ᵉ succès balise (9 s !) + leçons du run couloir.** (1) Reproductibilité : LOCK 19:14:24 → « Balise atteinte 0,69 m » 19:14:33, avec caméra à 3-4 Hz. Garde post-visite ajoutée (le robot re-verrouillait la balise visitée pendant sa rotation — cooldown 45 s dès le reset). (2) Collision chaise : AUCUN événement obstacle au log — piètements (bas) et assises (haut) des chaises sont HORS de la bande depth verticale 33-70 % ; chantier suivant : détection relative au plan de sol pour étendre la couverture sans faux positifs. (3) Run couloir : hors WiFi = arrêt sûr (deadman), mais découverte du mode « rosbridge NOYÉ » (port lié, handshakes gelés, 27 s d'arriéré — ni le PC ni le téléphone ne pouvaient se reconnecter) → sonde de handshake ajoutée au watchdog (purge + relance idempotente si pas de réponse en 8 s). Garde zombie durcie à 3 ticks (faux restart vécu). Réseau : scan fait, TP-Link_A552_5G du labo = meilleur candidat pour couper la laisse WiFi (checklist migration à générer le moment venu). | Remplacé |
| **v3.16.2** | 2026-07-13 | `tools/leo_watchdog.sh` | **Récupération post-reboot 100 % automatique.** La condition zombie (roslaunch PC vivant + mins_subscribe absent du master = roscore robot redémarré) est désormais détectée par le watchdog (2 ticks consécutifs, anti-faux-positif) → restart_stack automatique. C'était la DERNIÈRE étape manuelle de la chaîne : power-cycle du robot → tout revient seul en ~3 min (web intact, caméra ré-escaladée, laser 0 + png_level 1 réappliqués, stack relancée, MINS re-switché). | Remplacé |
| **v3.16.1** | 2026-07-13 | `web/app.js` | **Anti-gel du flux vidéo cockpit** : quand web_video_server redémarre (respawn supervision), la connexion MJPEG du <img> meurt SANS événement navigateur — l'image fige sur la dernière frame indéfiniment (constaté par l'opérateur pendant la séquence balise victorieuse ; la source publiait bien à 19,9 Hz). Aucun signal fiable n'existe côté navigateur → ré-armement du flux toutes les 45 s (re-handshake ~100 ms, imperceptible, onglet visible seulement). app.js r23. | Remplacé |
| **v3.16** | 2026-07-13 | mission complète | **🎯 SÉQUENCE BALISE VALIDÉE DE BOUT EN BOUT (18:55:48 → 18:55:59).** LOCK vision (4 LEDs, y=58 %) → centrage → approche GUIDÉE PAR LE DEPTH (sans damier ni tag) → « Balise atteinte (depth 0,67 m) — reset local — WAIT 5 s » : **11 secondes** de la détection au reset, monde préservé. La boucle mission complète (patrouille → obstacles → balise → reset) est validée terrain. Au passage ce soir : (1) moteurs verrouillés post-brown-out (commandes reçues, zéro courant moteur, tension stable — reset logiciel inopérant) → power-cycle = remède, diagnostic chaîne complète documenté ; (2) faux échappement à l'entrée AUTO corrigé (purge des historiques anti-coincement dans _start_patrol) ; (3) depth effondré à 1-3 Hz post-boot : compresseur PNG du Pi au niveau 9 d'usine → png_level 1 (8,9 Hz restaurés, réappliqué par le watchdog avec laser 0). Restent : tag A3 (portée), laser 0 + png_level à figer dans robot.launch côté Pi. | Remplacé |
| **v3.15.1** | 2026-07-13 | `web/start_web.sh` | **Fin des déconnexions cockpit en boucle.** Cause trouvée : start_web.sh repartait de zéro à chaque appel (free_port sur TOUT) ; or le watchdog l'appelle dès qu'UN port manque → un rosbridge tunnel 9443 en échec de démarrage (« no route to host », WiFi qui flappe pendant l'init) faisait tuer le rosbridge 9090 SAIN toutes les 60 s — le cockpit opérateur était déconnecté en boucle (« CRITICAL: WS DISCONNECTED » récurrent). Correctif : script IDEMPOTENT — chaque composant n'est (re)lancé que s'il est réellement mort (port lié = intact ; process présent sans port = démarrage en cours, on laisse tranquille : rosbridge peut mettre >1 min sous charge). | Remplacé |
| **v3.15** | 2026-07-13 | backend + watchdog + supervision | **LA BALISE EST VUE — diagnostic par les yeux du robot.** Capture directe de la vue annotée (web_video_server passé sous supervision roslaunch, respawn testé <10 s ; 3 lanceurs concurrents neutralisés — « plus jamais ce problème »). À 1 m, cécité totale expliquée canal par canal : (1) le PROJECTEUR IR mitraillait damier/LEDs/tags de points brillants ET dégradait le depth — mesure : remplissage 56 %→81 % SANS laser (stéréo passive sur tapis texturé) → laser 0 permanent (watchdog mis à jour ; penser à le figer dans robot.launch côté Pi) ; (2) MON seuil 240 éteignait la balise : rejeu du détecteur sur l'image réelle → 0/4 LEDs à 240, 3/4 à 220, 4/4 à 200 (croix parfaite à (357,290)) → seuil 200, le rejet plafonniers appartient à la garde y, PAS au seuil ; les LOCK sans convergence du test terrain étaient la VRAIE balise en détection intermittente ; (3) pas d'AprilTag sur la face avant (damier seul) — tag A3 toujours recommandé. VALIDÉ : LED RESET 4/4 en boucle face balise. + Trou du tout-en-un fermé : sans distance mesurée (tag absent, damier >1,5 m), l'approche est désormais guidée par le DEPTH du couloir (arrivée <0,7 m → reset, au-dessus du seuil obstacle 0,55 m — pas de conflit). | Remplacé |
| **v3.14.1** | 2026-07-13 | — (résultats terrain) | **VALIDATION TERRAIN AUTO (15:49-15:54, 5 min 25).** SUCCÈS : obstacles « super » (verdict opérateur) — 12+ bounces propres avec choix du côté libre, 3 bounces VISÉS vers grands chemins, 3 échappements (recul+pivot) dont la logique de fenêtres a correctement évité l'arrêt sécurisé ; attraction grand chemin permanente (couloirs 2,5-7,1 m) ; pivots mecanum 0,35 rad/s conformes (39° en 2,3 s mesuré) ; ZÉRO faux LOCK plafonnier (garde y + seuil 240 validés) ; ZÉRO perte heartbeat malgré changements d'onglets (piggyback validé) ; MINS stable tout du long (garde divergence silencieuse). ÉCHEC : balise jamais acquise — AprilTag 0 décodage en 5 min (budget pixel marginal : 2,2 px/module à 3 m — un tag A3 de ~35 cm porterait à 5-8 m) ; 2 LOCK vision au niveau sol (y=74/82 %) sans convergence en 12 s (reflet ou vraie balise à détection intermittente — indéterminé, traces réseau perdues par chute WiFi/alim du robot à ~15:55). PROCHAINE SESSION : test décisif de 5 min robot face balise à 2 m avec capture de la vue annotée (web_video_server à relancer côté leo start). | Remplacé |
| **v3.14** | 2026-07-13 | backend + launches | **Audit systèmes « mesurer d'abord » (space-grade).** Profil live : PC au plafond thermique (81°C/82, load 4.1 — la cause des lenteurs xmlrpc du jour). Budget dominé par MINS (64 %, légitime) + 2 consommateurs injustifiés : OpenVINS de secours (13-15 %, jamais utilisé, en nice -10 !) → priorité normale ; AprilTag (16 %, 15 Hz plein format pour une fenêtre backend de 0,7 s) → alimenté à 8 Hz (coût ÷2). Callback depth (boucle chaude sécurité, 15 Hz) profilé 4,04 ms → réécrit sur grille décimée ×2 : 1,68 ms (−58 %), médianes <0,5 % d'écart. Durcissement trié : le SEUL except silencieux dont l'échec désarme la chaîne obstacles (callback depth) valide désormais ses entrées et logge ses erreurs (1×/10 s) ; les 45 autres handlers volontairement intacts (réécrire de l'isolation d'erreur qui marche = risque de régression déguisé en rigueur) ; renommage cosmétique d'interfaces REFUSÉ (même raison). Concurrence déjà saine : vision en PROCESSUS séparé (immunité GIL par construction). Après : 60°C (+22° de marge, part transitoire), tags 6-8 Hz, depth sans erreur. | Remplacé |
| **v3.13.1** | 2026-07-13 | `tools/leo_watchdog.sh` | **Garde anti-divergence MINS.** Le Trajectory devenu fidèle a aussitôt révélé un vrai défaut : MINS divergé depuis des heures en silence (position à 886 km, 1123 m/s robot immobile — fuite d'intégration inertielle, déclencheur probable : chocs de collision du test). Relance propre → statique millimétrique en secondes. Leçon structurelle : un filtre divergé ne se plaint JAMAIS — le watchdog sonde désormais /mins/imu/odom chaque minute et relance automatiquement la stack si \|position\| > 100 m (impossible en intérieur) : tout épisode futur est borné à ~1 min. Anomalie chronique quantifiée au passage (item ouvert) : accéléro LeoCore \|a\| = 8,76 m/s² au repos vs g = 9,79 (−10,5 %, absorbé comme biais à l'init — à investiguer côté firmware). | Remplacé |
| **v3.13** | 2026-07-13 | backend + `web/trajectory.html` | **Rituel balise tout-en-un + Trajectory fluide.** (1) Chaîne unique en AUTO : acquisition (tag longue portée, vision en bonus) → approche 0,6 m → reset local IMMÉDIAT → marqueur → WAIT 5 s → 90°. Durcissement : le reset exige une distance MESURÉE (tag/damier) ≤ 0,6 m — l'ancienne règle « distance inconnue = reset immédiat » se déclenchait sur tout artefact centré ; le marqueur balise ne dépend plus du damier (distance du LOCK, cap centré). (2) LE refresh résiduel trouvé : trajectory.html se rechargeait (location.reload) 4 s après CHAQUE coupure WebSocket — remplacé par une reconnexion en place avec backoff (l'état trail/tags survit, la page ne se recharge plus jamais). (3) Fluidité : pose et viewport (centre+échelle) interpolés à chaque frame (fini les sauts à 8 Hz), bornes recalculées à 6 Hz au lieu de 60, trail regroupé en ≤12 chemins canvas par frame au lieu de 3000 strokes. | Remplacé |
| **v3.12** | 2026-07-13 | backend + `web/app.js` | **Premier run AUTO complet : 3 régressions corrigées.** (1) MANUAL toutes les 60 s pile : heartbeat = setInterval 200 ms, or les navigateurs étranglent les timers des onglets CACHÉS jusqu'à 1/min (l'opérateur regardait Trajectory) → heartbeat aussi émis à chaque RÉCEPTION de télémétrie (les événements WS ne sont pas étranglés) + timeout backend 1,5→12 s. (2) Approche LOCK AVEUGLE : la détection d'obstacle était limitée à PATROL (« ce qui est devant est la cible ») → un FAUX lock fonçait dans les meubles ; l'approche s'interrompt désormais sur contradiction (cible « à >0,6 m » mais depth <0,55 m devant) + contournement. (3) Plafonniers = balises, structurellement : en IR le détecteur de luminosité favorise EXACTEMENT les mauvais objets (plafonniers IR-brillants, LEDs visibles IR-sombres) ; tous les faux positifs à y=38-160 px → garde de plausibilité balise-au-sol (rejet au-dessus de 35 % de l'image), seuil 240 PERSISTANT, cooldown re-lock 45 s, source+hauteur loggées à chaque LOCK. Question terrain ouverte : les LEDs réelles passent-elles 240 en IR ? (tag+damier portent l'acquisition ; repli = flux couleur 424×240). | Remplacé |
| **v3.11** | 2026-07-13 | backend + configs MINS | **Rampe batterie saine (11,95 V) : théories corrigées, chaîne validée.** (1) Le « décrochage rouleaux » du 10/07 = affaissement moteur à 10,3 V : à batterie saine, 92-99 % d'autorité de 0,15 à 0,40 rad/s (zone morte < 0,12) → vitesses restaurées (pivots 0,35, fenêtres anti-coincement recalées). (2) FAUSSE PISTE du jour, corrigée : la rampe lisait un cap MINS au signe inversé → patch polarité angulaire + échange roues G/D… puis vérité terrain au GYRO BRUT : l'actionnement n'a jamais été inversé (remontage 180° = 2 inversions qui s'annulent en lacet) — c'était les 25 landmarks SLAM qui corrompaient le cap en pivot pur (profondeur inobservable : MINS +0,58 quand gyro −0,25, chi2 roues 6 %). Patches annulés, max_slam→0. VALIDATION finale : cmd +0,30 → gyro +0,249 → MINS +0,249, chi2 roues 100 % en rotation. Leçons : un cap ESTIMÉ n'est pas une référence pour diagnostiquer un signe d'actionnement (gyro brut seul) ; SLAM à garder sous garde d'excitation sur une plateforme riche en pivots. restart_stack.sh pérennisé dans tools/. | Remplacé |
| **v3.10** | 2026-07-13 | `leo_backend.py` + `tools/ramp_yaw.py` | **Adaptation FSM au régime mecanum** (décision : on GARDE les mecanum). Preuves du 10/07 : cmd 0,13 rad/s → 0,14 réel (accroche), cmd 0,4-0,5 → 0 (rouleaux décrochent). TOUTES les rotations passent à 0,18 rad/s : TURN_SPEED 0,6→0,18 et UTURN_SPEED 0,5→0,18 (bounces/escapes/U-turns — commandaient des rotations qui ne se produisaient PAS physiquement), LOCK_ALIGN 0,25→0,18, SEARCH_ANG, SPIRAL_ANG0 0,5→0,18. Pivots lents mais réels (90° ≈ 10 s) → fenêtres anti-coincement recalées : pivots répétés 20→90 s, déplacement 15→30 s (un pivot légitime = 10-16 s immobile, ne doit pas déclencher d'ESCAPE). Outil tools/ramp_yaw.py prêt pour situer précisément le genou de décrochage au retour du robot (hors ligne depuis le 10/07 au soir — la pile web a survécu seule, correctif v3.6.1 validé). | Remplacé (théorie corrigée en v3.11) |
| **v3.9.1** | 2026-07-10 | rapport + configs | **Identification : les roues sont des MECANUM** (FictionLab Leo Addon, Ø 128,8 mm, rouleaux polyuréthane 45°). Réinterprétation de la campagne : pas du patinage — les rouleaux libres absorbent le différentiel G/D par conception ; le firmware pilote en char (modèle différentiel structurellement faux pour du Mecanum : lacet réel ∝ 2(lx+ly) ≈ 2× la voie, et seulement si les rouleaux accrochent — quasi nul ici). r_eff 62,2 vs 64,4 géométrique = compression des rouleaux (3,4 %, typique). noise_w 0,8 = LA bonne réponse estimateur ; mesure de la voie au réglet devenue sans objet. Options matérielles au rapport : roues d'origine si les pivots comptent, OU contrôle par roue + odométrie mecanum pour débloquer le déplacement latéral. | Remplacé |
| **v3.9** | 2026-07-10 | configs MINS + `tools/calib_auto2.py` | **Campagne précision (carte blanche)** : calibration automatisée (le robot exécute lui-même ligne droite gardée par depth + arcs) → r = 62,2 mm ±0,5 (−0,5 % vs stock, moitié de l'item n°1 clos). DÉCOUVERTE : Δω roues 3,8 rad/s → 0,001 rad/s de lacet réel (gyro ET cap caméra d'accord) — les roues rigides patinent longitudinalement, le lacet différentiel est une FICTION sur ce sol (et a fait diverger MINS à 169 km) → noise_w 0,2→0,8 (lacet roues déconsidéré, vitesse conservée), b géométrique gardé. + 25 landmarks SLAM (MSCKF pur → hybride, confirmé live). VÉRIF : chi2 roues 100 % (408 mes.), T1.1-mini 60 s : σx=7,4 mm σy=1,0 mm pentes <0,5 mm/s, et le régime qui détruisait le filtre → errance max 9 mm. Seuil LED 240 appliqué en direct (4 faux resets plafonniers au log). Reste : voie (b) au réglet, grip roues (caoutchouc) si pivots importants. | Remplacé (roues = Mecanum, cf. v3.9.1) |
| **v3.8** | 2026-07-10 | `leo_backend.py` | **Audit perception caméra** (réponse : non, sous-ensemble minimal volontaire — infra1+infra2+depth 640×480@15, couleur/IMU D455 inutilisées). (1) ROI obstacle élargie au gabarit réel : ±8° (±6 cm à 45 cm !) → ±29° (±26 cm), plafonnée verticalement (sol n'entre qu'à 0,73 m — pas de faux positif) ; détection par tiers de couloir (p5 local, un pied de chaise ne se dilue plus). (2) Mesure A/B laser : remplissage depth IDENTIQUE à 60 et 150 (56 %, limité par la scène) — le compromis AprilTag ne coûte rien, laser 60 conservé. (3) Curseurs cockpit = commandes mortes (config figée au démarrage du process vision) → transmis LIVE à chaque frame ; « Luminosité min » pilote le vrai seuil (défaut 220), la teinte est un vestige sans effet en IR. | Remplacé |
| **v3.7** | 2026-07-10 | `leo_backend.py` + `wheel_remap.py` | **Anti-rotation-perpétuelle + 4 encodeurs** : en LOCK, une cible toujours visible mais jamais centrée (artefact solidaire de la caméra — reflet/plafonnier détecté comme LEDs à y=64) faisait tourner le robot sans fin à ω=0,25×err (0,13 rad/s observé) car LOCK_TIMEOUT n'arme que si la cible disparaît. Ajout : LOCK_STALL 12 s sans centrage → abandon + PATROL avec cooldown 25 s sans re-lock (le robot AVANCE) ; l'approche centrée compte comme progrès (pas d'abandon d'une approche longue légitime). MINS n'utilisait que 2 encodeurs sur 4 (FL/FR) : wheel_remap moyenne désormais les 2 roues de chaque côté (patinage skid-steer lissé). Outil `tools/calib_roues_rigides.py` créé (calibration passive r/b en 2 min de conduite — intrinsèques stock 0,0625/0,358 = roues d'origine, suspect n°1 des décalages MINS en rotation ; calib en ligne interdite, divergence du 6/07). | Remplacé |
| **v3.6.2** | 2026-07-10 | `navigation_supervision.launch` | **AprilTag supervisé** : le détecteur longue portée (lancé à la main le 8/07) disparaissait à chaque restart de stack (constaté au rallumage : /tag_detections sans publisher). Inclus dans la supervision (respawn) — règle : tout ce qui est lancé à la main en session terrain est déjà perdu. Validation live v3.6 au passage : secteurs depth mesurés à 13,6 Hz sur scène réelle (couloir 5,79 m dans l'axe correctement ignoré par l'hystérésis — pas d'inflexion inutile), MINS init 8 s, bascule MINS auto par restart_stack durci (attente rospy, timeout -k). | Remplacé |
| **v3.6.1** | 2026-07-10 | `tools/leo_watchdog.sh` | **Durcissement watchdog** (robot éteint 2 j) : `timeout -k 5` sur les 11 sondes (sans -k, un rostopic bloqué vers un master mort ignore le TERM → 14 runs zombies accumulés en 2 jours, tous purgés) ; guérison de la pile web déplacée AVANT la porte « robot injoignable » (le site ne dépend que du PC — il était resté mort 2 jours avec le robot). Leçon : auditer un superviseur sous la panne de ses propres sondes. | Remplacé |
| **v3.6** | 2026-07-08 | `leo_backend.py` | **Attraction « grand chemin »** : bande de profondeur découpée en 9 secteurs angulaires (~10° sur les 87° de FOV depth, médianes robustes — les pixels invalides ne comptent pas, inconnu ≠ libre). Un secteur ≥ 2,5 m ET ≥ 1,5× plus profond que l'axe = « grand chemin » : en PATROL le cap s'infléchit doucement vers lui (kp 0,8, saturé 0,25 rad/s — le robot suit les couloirs et portes ouvertes) ; sur obstacle, le bounce est VISÉ vers ce couloir s'il est à plus de 15° de l'axe (l'aléatoire reste le repli — composante ergodique conservée). Priorité balise et seuil obstacle 0,45 m inchangés. | **ACTUEL** (backend) |
| **v3.5** | 2026-07-08 | `leo_backend.py` | **Couverture type aspirateur autonome** : bounce aléatoire 45-135° sur obstacle, spirale d'exploration après 45 s sans balise (biais vers dernière balise aperçue, mémoire 2 min), anti-coincement 2 niveaux (pivots répétés / déplacement monde < 0,15 m sur 15 s) → échappement (recul 0,2 m — seule marche arrière autorisée, depth aveugle à l'arrière — + grand pivot aléatoire), arrêt sécurisé + alerte au 3ᵉ échappement/min. Chaîne sécurité inchangée. | Remplacé |
| **v3.4** | 2026-07-08 | `leo_backend.py` + `config_wheel.yaml` | **Inversion de traction (roues rigides)** : DRIVE_POLARITY au point de sortie unique des commandes (tous modes), cap odométrique basculé en repère caméra (+π, point d'entrée unique), T_imu_wheel = Rz(π) pour MINS. Validé : +0,200 m le long du cap caméra sur impulsion avant ; MINS accepte 100 % des mesures roues (chi²) — fin du rejet silencieux. LOCK avec approche jusqu'à 0,6 m (le tag longue portée déclenchait le rituel à distance). | Remplacé |
| **v3.3** | 2026-07-08 | `leo_backend.py` | **Refonte patrouille simple** : ligne droite permanente (plus de scan 360°), obstacle → pivot 90° unique vers le côté libre puis reprise tout droit, pause balise 30→5 s, rotation post-balise 180→90°. Conservés : reset local préservant le monde, marqueur balise + anti-doublon, priorité vision, repli 180° anti-enfermement. Verrouillage Carolus (/mins/external_ref/carolus + règle watchdog) et schémas d'isolation (rapport+site). | Remplacé |
| **v3.2** | 2026-07-08 | launch leo_navigation | **Audit d'architecture TF** : canaux TF des estimateurs isolés (/tf_mins, /tf_vins — le global→imu de MINS volait le frame imu de l'URDF ; conflit latent avec OpenVINS), frame laser redondant supprimé, rôle Carolus validé (slot VICON = correct). Coordination leo↔watchdog par drapeau maintenance. Correction incident fps 10 non supporté. | Remplacé |
| **v3.1** | 2026-07-06→07 | stack complète | **Campagne fiabilité + précision** : gardes anti-gel/monotonie IMU, chi2 roues 15→5, gravité locale 9,790, densité features 1500→300 (CAM 120→67 ms), AVOID (contournement), AprilTag 285 longue portée, dé-dup balises/obstacles, watchdog cron auto-réparant, page Trajectory, design system, **roues rigides** (rayon à re-mesurer !), rapport LaTeX 194 p. publié sur le site. | Remplacé |

### 2.2 Décisions d'architecture majeures

**Pourquoi abandonner AprilTag ?**  
La balise physique a été confirmée comme portant un **damier** et non un AprilTag. L'appel `cv2.aruco.detectMarkers` coûtait 5–10 ms par frame pour un résultat systématiquement nul. Supprimé en v2.2.

**Pourquoi le traitement sur le PC et non sur le Pi ?**  
Le Pi 4 sature rapidement avec `findChessboardCornersSB` (jusqu'à 250 ms sur image full-res). Le PC traite les images compressées reçues via Wi-Fi, décode localement, et analyse sans contrainte thermique.

**Pourquoi pas ROS 2 ?**  
`leo_bringup` (firmware odométrie roues) publie sur des topics `leo_msgs` non portés sur ROS 2 dans la version installée sur le robot.

---

## 3. ARCHITECTURE TECHNIQUE

### 3.1 Gestion des threads — Découplage Décodage / Détection / Affichage

**Problème initial** : un thread unique gérait décodage JPEG + détection damier + affichage Tkinter. `findChessboardCornersSB` pouvant prendre 250 ms, le flux vidéo se figeait complètement.

**Solution retenue** : pipeline 3-couches à séquences incrémentales :

```
Callback ROS     →   Thread DÉCODAGE    →   Thread DÉTECTION
(ultra-léger)        (_decode_loop)         (_detect_loop)
                      ~3 ms / frame          ~10–250 ms / frame

  _on_image()         Décode JPEG            Lit latest_bgr
  Stocke msg brut  →  Écrit latest_bgr   →   Appelle _analyse()
  Incrémente           Incrémente             Écrit self.det
  _cam_seq             _dec_seq               Appelle _validate_beacon()

Thread principal Tkinter (_tick @ 33 ms)
  Lit latest_bgr (déjà décodé) → Affichage direct (~30 FPS GARANTI)
  Lit self.det (déjà calculé)  → Overlay, carte, graphiques
```

**Garanties de cohérence** : un `threading.Lock` unique protège `latest_bgr`, `self.det`, `self.beacons`, `self.traj`, `self.origin`. Seul le thread Tkinter lit les `IntVar` (obligation Tkinter).

**Paramètres de performance** :
```python
TICK_MS      = 33      # cadence affichage ≈ 30 FPS
DETECT_PERIOD = 0.10   # cadence détection ≈ 10 Hz (throttlée)
CB_SCALE     = 0.5     # image réduite à 50% pour le damier (4× moins de pixels)
```

**Résultat mesuré** : 29 FPS d'affichage maintenus avec une détection de 250 ms en parallèle (phases de découverte `SB`).

### 3.2 Logique de détection hybride — Damier + 4 LED

La détection se fait en deux étapes complémentaires et hiérarchisées :

#### Étape 1 : Localisation du damier (ancre spatiale)

```
Algorithme _detect_checkerboard() :

  Image couleur 640×480
       ↓ conversion niveaux de gris
  Image grise  
       ↓ resize × CB_SCALE (0.5) → 320×240
  Image réduite

  ┌─ _cb_size connu ? ──YES──→ SB(connu) ──OK?→ return corners/CB_SCALE
  │                              ↓ NOK
  │                          FAST(connu) ──OK?→ return corners/CB_SCALE
  │
  └─ Découverte RAPIDE : FAST_CHECK sur CHECKERBOARD_SIZES (15 tailles)
         ↓ si trouvé → _lock_cb() → mémorise _cb_size → return
  
  └─ Découverte ROBUSTE : SB sur 6 premières tailles, throttlée @ 2 Hz
         ↓ si trouvé → _lock_cb() → return
  
  return None (damier absent)
```

**Optimisation clé** : une fois la taille verrouillée (`_cb_size`), le temps de détection passe de ~40 ms (découverte multi-tailles) à ~11 ms (test d'une seule taille).

**Tailles testées** (ordre de priorité) :
```python
CHECKERBOARD_SIZES = [(7,7),(6,6),(8,8),(5,5),(6,8),(8,6),(7,6),(6,7),
                      (7,9),(9,7),(5,7),(7,5),(9,6),(6,9),(4,4)]
```
*(en coins internes, soit cases−1)*

#### Étape 2 : Filtrage des LED dans la ROI du damier

```
Segmentation HSV → blob analysis → liste de candidats LED (toute l'image)

        ↓

ROI = rectangle autour du damier × LED_ROI_MARGIN (0.6)
      → rejette les reflets lointains et sources parasites

        ↓

_cluster_lights() :
  - tri par taille décroissante
  - sélection des N_LEDS (4) meilleures autour de la plus grosse
  - validation : au moins MIN_LIGHTS (3) LED retenues

        ↓

Validation balise :  damier trouvé  AND  len(cluster) ≥ 3
```

**Paramètres HSV LED bleues** (réglables en direct par curseurs) :
```python
LED_HUE_LOW  = 80    # → Teinte verte-cyan (80°)
LED_HUE_HIGH = 135   # → Teinte bleue (135°)
LED_V_MIN    = 200   # → Luminosité minimale (LED allumée = saturée)
LED_S_MIN    = 10    # → Saturation minimale
```

### 3.3 Système de reset des coordonnées

**Principe** : le reset est purement logiciel — aucun service ROS n'est appelé.

```python
# Dans _on_beacon_validated() :
self.beacon_count += 1           # (1) compteur INCRÉMENTÉ en premier
with self._lock:
    self.beacons = [(0.0, 0.0, ident)]  # balise = nouvelle origine
    self.origin  = None          # (2) RESET : prochain message odométrie
    self.traj    = []            #     devient la nouvelle origine → pose = 0,0,0
self._log(f"BALISE [{ident}] REPÉRÉE - RESET EFFECTUÉ")
```

**Dans `_on_odom()`** :
```python
if self.origin is None:
    self.origin = (px, py, yaw)  # fige l'origine au prochain message odom
# puis calcul relatif : pose = rotation(px-ox, py-oy) par angle -oyaw
```

**Pourquoi ne pas appeler `firmware/reset_odometry` ?**  
Ce service ROS peut bloquer indéfiniment si le firmware ne répond pas (réseau occupé, robot en mouvement). Avant correction (v2.3), ce blocage empêchait d'atteindre `beacon_count += 1` → le compteur restait à 0.

### 3.4 Machine d'états — Mode « Cibler ∞ balises »

```
                ┌─────────────────────────────────────────┐
                │                  SEEK                    │
                │  - Tourne @ SEARCH_ANG (0.4 rad/s)      │
                │  - Si balise visible → centre + approche │
                │  - Si beacon_count augmente → → RECUL   │
                └───────────────────────┬─────────────────┘
                                        │ balise validée
                                        ▼
                ┌─────────────────────────────────────────┐
                │                  RECUL                   │
                │  - lin = -RECUL_SPEED (-0.2 m/s)        │
                │  - Durée : RECUL_T (1.5 s)              │
                └───────────────────────┬─────────────────┘
                                        │ 1.5 s écoulées
                                        ▼
                ┌─────────────────────────────────────────┐
                │                TURN180                   │
                │  - ang = TURN_SPEED (0.6 rad/s)         │
                │  - Intègre Δyaw odométrique              │
                │  - Arrêt quand Σ|Δyaw| ≥ π rad          │
                └───────────────────────┬─────────────────┘
                                        │ 180° atteints
                                        ▼
                              retour → SEEK
```

**Robustesse du demi-tour** : l'accumulation de `|Δyaw|` via odométrie roues (non par chrono) garantit un vrai 180° indépendamment de la charge batterie ou du dérapage.

---

## 4. BILAN DES TESTS ET CORRECTIONS

### 4.1 Bugs majeurs — Analyse et correctifs

#### BUG-001 : Gel du flux vidéo lors de la détection
- **Symptôme** : l'image se figeait pendant 0,5 à 2 secondes lors de la recherche du damier
- **Cause racine** : `findChessboardCornersSB` appelé dans le même thread que le décodage JPEG et l'affichage Tkinter. Sur une image 640×480 avec plusieurs tailles à essayer, la durée pouvait atteindre 250 ms
- **Correctif** (v2.1) : introduction de `_decode_loop` (affichage, ~3 ms) et `_detect_loop` (détection, 10–250 ms) dans deux threads daemon séparés. Le thread principal Tkinter ne fait que lire `latest_bgr` déjà prêt
- **Validation** : 29 FPS mesurés en continu, même en phase de découverte de taille de damier

#### BUG-002 : Compteur de balises bloqué à 0
- **Symptôme** : le log affichait `BALISE VALIDE` et la carte se remettait à zéro, mais le compteur restait à 0 en permanence (visible sur capture d'écran utilisateur)
- **Cause racine** : dans `_on_beacon_validated()`, l'appel `self._reset_srv()` (service ROS `firmware/reset_odometry`) précédait `beacon_count += 1`. Si le service tardait ou bloquait, le thread de détection restait suspendu avant d'atteindre la ligne d'incrément
- **Correctif** (v2.3) : suppression totale de `_reset_srv()`. Le reset est désormais purement local (`origin = None`). `beacon_count += 1` est la première instruction exécutée
- **Validation** : le compteur monte immédiatement à 1, 2, 3... à chaque validation

#### BUG-003 : Carte affichant toujours 1 balise au lieu du cumul
- **Symptôme** : après plusieurs balises, le compteur sur la carte restait à 1
- **Cause racine** : le code affichait `len(self.beacons)` — or `self.beacons` est réinitialisé à `[(0.0, 0.0, ident)]` (1 élément) à chaque balise validée
- **Correctif** (v2.3) : affichage de `self.beacon_count` (compteur cumulatif) à la place
- **Validation** : compteur affiché sur la carte = vrai nombre de balises trouvées depuis le démarrage

#### BUG-004 : LED parasites détectées (reflets, sources éloignées)
- **Symptôme** : 6, 8 ou 12 LED détectées au lieu de 4 ; fausses validations possibles
- **Cause racine** : pas de filtre spatial — toute l'image était scrutée pour des blobs HSV correspondant aux LED
- **Correctif** (v2.2) : ROI calculée à partir des coins du damier + marge `LED_ROI_MARGIN = 0.6`. Seuls les blobs dans cette zone sont retenus. Puis `_cluster_lights()` ne conserve que les `N_LEDS = 4` plus grosses
- **Validation** : exactement 4 LED retenues sur la balise, 0 parasite dans les tests de bureau

#### BUG-005 : Crash matplotlib `labelcolor` sur mpl 3.1.2
- **Symptôme** : `TypeError: legend() got an unexpected keyword argument 'labelcolor'`
- **Cause racine** : `labelcolor` a été ajouté à matplotlib en version 3.3. La version installée avec ROS Noetic est 3.1.2
- **Correctif** (v2.4) : remplacement par une boucle `for _txt in leg.get_texts(): _txt.set_color(...)` — compatible toutes versions

#### BUG-006 : `tight_layout` provoquant des UserWarnings
- **Symptôme** : warnings répétés dans le terminal lors du redimensionnement des graphiques
- **Cause racine** : `tight_layout()` et `FigureCanvasTkAgg` interagissent mal sur mpl 3.1.2
- **Correctif** (v2.4) : remplacement par `fig.subplots_adjust(left=0.13, right=0.97, top=0.90, bottom=0.12, hspace=0.55)`

#### BUG-007 : Graphiques hors de l'écran (en bas de fenêtre)
- **Symptôme** : les graphiques matplotlib étaient invisibles (hors du bas de l'écran 1152 px)
- **Cause racine** : positionnés en `row=7` dans la grille Tkinter, après la télémétrie, le log et la carte
- **Correctif** (v2.5) : colonne droite restructurée en `rightcol = ttk.Frame` avec `_build_graphs(rightcol)` en premier (`pack()`), puis la carte dessous. Fenêtre réduite à 922×1062 px
- **Validation** : graphiques visibles et lisibles, capture d'écran utilisateur validée

### 4.2 Mesures de performance actuelles

| Métrique | Valeur mesurée | Cible | État |
|----------|---------------|-------|------|
| FPS affichage caméra | ~29–30 FPS | ≥ 25 FPS | ✅ |
| Hz caméra (Wi-Fi) | ~30 Hz | ≥ 25 Hz | ✅ |
| Hz odométrie | ~25 Hz | ≥ 20 Hz | ✅ |
| Latence détection (taille connue) | ~11 ms | < 50 ms | ✅ |
| Latence détection (découverte FAST) | ~40 ms | < 100 ms | ✅ |
| Latence détection (découverte SB) | ~80–250 ms | isolé (thread) | ✅ |
| Délai entre validation balise et incrément compteur | < 1 ms | < 50 ms | ✅ |
| Anti-rebond entre deux balises | 3,0 s (`BEACON_COOLDOWN`) | configurable | ✅ |
| Temps de verrouillage taille damier (cold start) | 1–5 frames | < 10 frames | ✅ |
| Précision reset X/Y/Z | 0,000 m à l'affichage | ≤ ±0,05 m terrain | À mesurer |
| Distance estimation (BEACON_WIDTH_M non calibré) | ±30% estimé | ±10% | ⚠️ À calibrer |

### 4.3 Résumé des corrections par version

```
v2.0  ──────────────────────────────────────────── Base tkinter + SSH
v2.1  ── BUG-001 (freeze vidéo)  → threads découplés
v2.2  ── BUG-004 (LED parasites) → ROI + N_LEDS=4
        + Basculement damier (suppression AprilTag)
v2.3  ── BUG-002 (compteur=0)    → incrément avant toute I/O réseau
        + BUG-003 (carte=1)      → beacon_count vs len(beacons)
v2.4  ── BUG-005 (mpl crash)     → get_texts() loop
        + BUG-006 (tight_layout) → subplots_adjust
        + Mode INFINI + graphiques + indicateurs
v2.5  ── BUG-007 (graphs off-screen) → haut-droite, pack()
```

---

## 5. PROCÉDURE D'UTILISATION (Standard Operating Procedure)

### 5.1 Prérequis (vérification une fois par session)

```bash
# Sur le PC — vérifier la connexion Wi-Fi au robot
ping 10.0.0.1 -c 3
# Attendu : 3 réponses < 5 ms

# Vérifier que ROS est accessible sur le robot
export ROS_MASTER_URI=http://10.0.0.1:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
rostopic list | head -5
# Attendu : liste de topics dont /firmware/wheel_odom
```

### 5.2 Lancement standard (quotidien)

**Étape 1 — Démarrer le dashboard sur le PC**
```bash
cd /home/lab272/TOUT
python3 leo_dashboard.py
```

**Étape 2 — Connecter au robot**
- Champ IP : `10.0.0.1` (pré-rempli)
- Champ SSH password : à saisir à la main (plus pré-rempli depuis l'audit sécurité du 2026-08-04, ce dépôt étant public ; demandez le mot de passe à l'opérateur)
- Cliquer **[Se connecter]**
- Attendre le voyant **`EN LIGNE ✓`** (≤ 5 s)
- La caméra est lancée **automatiquement** via SSH (`bash -ic` → charge `~/.bashrc`)

**Étape 3 — Vérifier les indicateurs (bandeau)**
```
Mission 00:00  |  Caméra 30 Hz  |  Odom 25 Hz  |  Balises 0
```
- Caméra < 8 Hz → alerte rouge : vérifier le Wi-Fi ou relancer la caméra
- Odom < 5 Hz → firmware LEO non démarré : `systemctl status leo` sur le robot

**Étape 4 — Calibration LED (si première utilisation ou changement de lumière)**
- Cocher **[Voir le masque]** dans le panneau gauche
- Ajuster les curseurs **Teinte min/max** (80–135 par défaut pour bleu)
- Ajuster **Luminosité min** (200 par défaut, baisser si LED paraissent sombres)
- Valider : les 4 LED bleues de la balise doivent apparaître en blanc sur le masque
- Décocher **[Voir le masque]**

### 5.3 Procédure de test — Détection balise

1. Placer la balise (damier + 4 LED allumées) à **1–2 m** face à la caméra
2. Observer le log : `DAMIER NxN verrouillé` → première détection
3. Après, la détection est instantanée (taille mémorisée)
4. Observer le log : `BALISE [DAMIER NxN] REPÉRÉE - RESET EFFECTUÉ`
5. Vérifier le bandeau : `Balises 1`
6. Vérifier la carte : position remise à 0,0,0 + point balise déposé

### 5.4 Procédure de test — Mode navigation autonome

| Mode | Bouton | Comportement attendu |
|------|--------|---------------------|
| AUTO | `[Mode : AUTO]` | Tourne 360° @ 0.4 rad/s, avance 1 s si rien, recommence |
| CIBLER | `[Cibler une balise]` | Se centre sur la balise visible et avance ; s'arrête à 0.6 m |
| INFINI | `[Cibler ∞ balises]` | CIBLER → valide → recul 1.5 s → demi-tour 180° → CIBLER |
| MANUEL | `[Mode : MANUEL]` | Flèches clavier ↑↓←→ |
| STOP | `[STOP]` | 5× Twist(0) → arrêt garanti |

### 5.5 Commandes SSH utiles (debug)

```bash
# Depuis le PC — vérifier les topics actifs
export ROS_MASTER_URI=http://10.0.0.1:11311
rostopic hz /camera/color/image_raw/compressed   # doit afficher ~30 Hz
rostopic hz /firmware/wheel_odom                 # doit afficher ~25 Hz
rostopic echo /firmware/battery --once           # tension batterie (>11.0 V)

# Depuis SSH sur le robot
ssh pi@10.0.0.1                      # mdp : demander a l'operateur (audit securite 2026-08-04)
systemctl status leo                 # état du firmware
rostopic list | grep firmware        # topics odométrie disponibles
```

### 5.6 Arrêt propre

1. Cliquer **[STOP]** (envoie 5× Twist zéro au robot)
2. Fermer la fenêtre du dashboard (le thread paramiko coupe la caméra SSH automatiquement)

---

## 6. PLANIFICATION DES FUTURS TESTS

### 6.0 SÉQUENCE DE REPRISE — prochaine session (2026-07-30)

**Bloquant matériel, à faire tournevis en main AVANT toute mesure.** Le
Raspberry Pi tourne à **86 °C**, bridé à **600 MHz** (`throttled=0xe0006`,
limite thermique douce atteinte). Ce seul fait invalide par avance toute
mesure d'estimateur : le bridage affame la chaîne série, d'où **13 à 21 trous
par 40 s** dans `/imu/data_clean`, d'où les aborts d'openVINS sur
`Propagator.cpp:101`. Le levier logiciel est **épuisé** — couper le flux
couleur a été mesuré sans le moindre gain thermique.

| Ordre | Étape | Critère de passage à l'étape suivante |
|-------|-------|----------------------------------------|
| **1** | **Installer le refroidissement actif** (ventilateur/dissipateur) | `vcgencmd measure_temp` < 70 °C **et** `get_throttled` = `0x0` **et** retour à 1500 MHz |
| **2** | **Vérification thermique sous charge** — pile complète + roulage 5 min | **0 trou IMU** sur 40 s (`tools/` sonde) et **0 abort** `Propagator.cpp:101` sur 10 min |
| **3** | **Activer la couleur** : `enable_color` → `true` dans `/etc/ros/robot.launch` (robot), puis `sudo systemctl restart leo` | `/pc/camera/color/image_raw` publie ; température **stable** sous 70 °C |
| **4** | **Lancer le détecteur** : `bash tools/launch_carolus.sh` | `/pose` publie ; le panneau **6 DDL du cockpit passe au violet** (déjà câblé, aucune action supplémentaire) |
| **5** | **Calibrer les intrinsèques COULEUR** (Kalibr) | Erreur de reprojection < 0,5 px |
| **6** | **Roulage final** avec balise en vue + `record_trajectories.sh` | `<base>_carolus.csv` généré ; `plot_trajectories.m` calcule l'**erreur absolue** |

> ⚠️ **L'étape 5 n'est pas optionnelle.** Les intrinsèques actuellement
> déclarées dans `tools/carolus_detect.launch` proviennent d'un capteur
> ~1280 px de large, appliquées à un flux **640×480**. Le launch le dit
> lui-même : « la pose 6-DOF sera APPROXIMATIVE ». Sauter l'étape 5
> produirait une pose **affichable mais non métrique** : la publier comme
> vérité-terrain dans le rapport serait scientifiquement indéfendable et
> remplacerait une divergence relative honnête par une erreur absolue fausse.

> ⚠️ **Antécédent de sécurité sur l'étape 3.** L'activation de la couleur a
> déjà provoqué une **perte de contrôle du robot** le 27/07 (Pi à 87 °C,
> surcharge CPU). Le throttle à 4 Hz de `carolus_detect.launch` est la
> mitigation ; ne pas franchir l'étape 3 tant que l'étape 1 n'est pas validée.

**Chantiers non bloquants, en parallèle** : (a) terminer la calibration
terrain — 3 passes par sens avec le protocole de
`docs/protocole_calibration_terrain.md`, pour trancher si l'écart de 7–9 %
en rotation vient des capteurs ou de la mesure au sol (deux instruments
INDÉPENDANTS, gyro et odométrie différentielle, s'accordent contre le
rapporteur → la référence est suspecte) ; (b) demander une **licence réseau
FlexLM** du campus à l'administrateur (n° 993588) pour rendre MATLAB
insensible aux incidents du service en ligne.

---

### 6.1 Tests terrain prioritaires

| Priorité | Réf. | Test | Méthode | Critère de succès |
|----------|------|------|---------|-------------------|
| HAUTE | T-01 | Charger la batterie | — | Tension > 12.0 V avant tout test |
| HAUTE | T-02 | Mode `Cibler ∞ balises` terrain | 3 balises posées à 2 m dans la pièce | Robot enchaîne les 3 sans intervention |
| HAUTE | T-03 | Précision reset odométrique | Déplacer 1 m après reset, mesurer X/Y affiché | Erreur < ±0,05 m sur 1 m |
| NORMALE | T-04 | Calibration `BEACON_WIDTH_M` | Mesurer écartement LED gauche-droite (règle) | Distance estimée ±10% vs mesure laser |
| NORMALE | T-05 | Test multi-balises (5+) | Poser 5 balises identiques | Compteur atteint 5, carte cohérente |
| NORMALE | T-06 | Dérive odométrique | Aller-retour 5 m × 3 fois, mesurer écart | Dérive < 0.2 m / 5 m |
| NORMALE | T-07 | Validation graphiques terrain | Piloter manuellement 30 s | Courbe vitesse lisible, non saturée |
| BASSE | T-08 | Dégradation Wi-Fi | Éloigner robot à 5 m+ | Alerte Hz caméra < 8 Hz déclenchée |
| BASSE | T-09 | Robustesse TURN180 | Effectuer 10 demi-tours consécutifs | Erreur angulaire < 15° à chaque fois |

### 6.2 Améliorations techniques identifiées

| Réf. | Amélioration | Complexité | Impact attendu |
|------|--------------|------------|----------------|
| A-01 | Calibration intrinsèque D455 (fx, fy réels) | Faible | Distance estimée ±5% au lieu de ±30% |
| A-02 | Persistance `_cb_size` entre sessions | Faible | Pas de phase découverte au démarrage |
| A-03 | Enregistrement CSV de la trajectoire | Faible | Analyse post-mission |
| A-04 | Affichage heading robot sur la carte (flèche) | Faible | Meilleure lecture de la pose |
| A-05 | Filtre de Kalman sur l'odométrie | Élevée | Réduction dérive long terme |
| A-06 | Détection multi-balises simultanées | Élevée | Mission complexe possible |

### 6.3 Paramètres à recalibrer en priorité

```python
# ⚠️ À MESURER SUR LA VRAIE BALISE avant tout test de distance :
BEACON_WIDTH_M = 0.165  # m — écartement LED gauche et droite (valeur Carolus supposée)
                        # → utiliser une règle, mesurer en mm, convertir

# ⚠️ À AJUSTER si la caméra D455 a été recalibrée :
CAM_FX = 384.65         # focale pixels (640×480, profil Color D455 standard)
                        # → lire via : rostopic echo /camera/color/camera_info --once
```

---

## 7. ANNEXES

### 7.1 Constantes logicielles (v2.5)

```python
# ── Réseau ────────────────────────────────────────────────
ROBOT_IP_DEFAULT  = "10.0.0.1"
CAM_TOPIC_DEFAULT = "/camera/color/image_raw/compressed"
ODOM_TOPIC        = "/firmware/wheel_odom"
CMD_TOPIC         = "/cmd_vel"

# ── Détection LED ─────────────────────────────────────────
LED_HUE_LOW    = 80       # teinte min (HSV) — vert-cyan
LED_HUE_HIGH   = 135      # teinte max (HSV) — bleu
LED_V_MIN      = 200      # luminosité min
LED_S_MIN      = 10       # saturation min
LED_MIN_AREA   = 8.0      # px² — taille min blob LED
LED_MAX_AREA   = 4000.0   # px² — taille max (rejette large reflet)
LED_MIN_CIRC   = 0.45     # circularité min (0=barre, 1=cercle)
LED_CLUSTER_PX = 320      # px — rayon regroupement LED
MIN_LIGHTS     = 3        # LED mini pour valider (tolère 1 panne)
N_LEDS         = 4        # LED à conserver (la balise en a 4)
LED_ROI_MARGIN = 0.6      # marge ROI autour du damier

# ── Détection damier ──────────────────────────────────────
CB_SCALE       = 0.5      # facteur resize avant détection (rapide)
DETECT_PERIOD  = 0.10     # s — cadence thread détection

# ── Navigation ────────────────────────────────────────────
SEARCH_ANG     = 0.4      # rad/s — vitesse rotation recherche
ADVANCE_LIN    = 0.2      # m/s  — avance entre tours 360°
TARGET_STOP_M  = 0.6      # m    — distance d'arrêt devant balise
BEACON_COOLDOWN= 3.0      # s    — anti-rebond entre deux balises
RECUL_SPEED    = 0.2      # m/s  — recul après balise trouvée
RECUL_T        = 1.5      # s    — durée du recul
TURN_SPEED     = 0.6      # rad/s — vitesse demi-tour
```

### 7.2 Structure des messages ROS utilisés

```
leo_msgs/WheelOdom  (/firmware/wheel_odom)
  float64 pose_x
  float64 pose_y
  float64 pose_yaw    (radians)
  float64 velocity_lin
  float64 velocity_ang

std_msgs/Float32  (/firmware/battery)
  float32 data        (tension en Volts)

leo_msgs/WheelStates  (/firmware/wheel_states)
  float64[] velocity  (4 roues : FL, RL, FR, RR)
  float64[] torque
  float64[] pwm_duty_cycle

sensor_msgs/CompressedImage  (/camera/color/image_raw/compressed)
  string format       ("jpeg")
  uint8[] data
```

### 7.3 Topologie réseau

```
PC (Ubuntu 20.04)                    Robot LEO (Raspberry Pi 4)
192.168.xxx.yyy  ←──── Wi-Fi ────►  10.0.0.1

PC : ROS_MASTER_URI=http://10.0.0.1:11311
     ROS_IP=192.168.xxx.yyy  (auto-détecté via socket UDP)

Robot : roscore + leo_bringup (systemd)
        + realsense2_camera (lancé par SSH via dashboard)
```

### 7.4 Fichiers du projet

```
/home/lab272/TOUT/
├── leo_dashboard.py        ← FICHIER PRINCIPAL (version active v2.5)
├── leo_tracking_map.py     ← Archive v1.0 (NASA Mission Control)
├── JOURNAL_DE_BORD.md      ← Ce document
├── GUIDE_DEMARRAGE_LEO.md  ← Guide pas-à-pas pour débutants
└── carolus_ws/             ← Workspace ROS de référence (blob detection)
    └── src/launch/carolusBlobDetection.launch

/home/lab272/.claude/projects/.../memory/
└── leo-rover-setup.md      ← Mémoire persistante inter-sessions
```

---

*Rapport généré le 25 juin 2026 — `leo_dashboard.py` v2.5*  
*Prochaine mise à jour : après session de tests terrain du 26 juin 2026*

# LEO Rover — Interface web (cockpit)

Backend Python sans tête + frontend HTML/JS statique. Aucun framework, aucune
étape de build : les pages sont servies telles quelles, Tailwind et Three.js
sont vendorisés dans `vendor/`.

**Ce document décrit l'état réel du système.** Si un point ci-dessous ne
correspond plus au code, c'est le code qui fait foi et ce fichier qui est à
corriger.

---

## 1. Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│ ROBOT LEO — 10.154.6.41                                              │
│   roscore · realsense2_camera · serial_node (CORE2, rosserial)       │
│   leo_bringup (odométrie)                                            │
└────────────────────────────────┬─────────────────────────────────────┘
                                 │ réseau ROS (WiFi FLTech-Guest, ou WireGuard)
┌────────────────────────────────┴─────────────────────────────────────┐
│ PC (Ubuntu 20.04 · ROS Noetic) — station sol                         │
│                                                                       │
│   ── lancé par start_web.sh ──────────────────────────────────────    │
│   rosbridge_server      ws://PC:9090    LAN direct (roslibjs)         │
│   rosbridge TLS         wss://PC:9443   via le tunnel Cloudflare      │
│   web_video_server      http://PC:8080  flux MJPEG                    │
│   serve.py              http://PC:8000  pages statiques (14)          │
│   sauvegarde repères    127.0.0.1:8010  écriture tf_validator, LOCAL  │
│   calibration_monitor                   /leo_vision/calibration_status│
│   cloudflared                           expose le tout publiquement   │
│                                                                       │
│   ── lancé SÉPARÉMENT par navigation_supervision.launch ──────────    │
│   leo_backend.py · pose_selector · imu_sanitizer · wheel_remap        │
│   MINS · openVINS · sqrtVINS · apriltag · republish caméra            │
└────────────────────────────────┬─────────────────────────────────────┘
                                 │ HTTP / WebSocket
┌────────────────────────────────┴─────────────────────────────────────┐
│ NAVIGATEUR (PC, tablette, ou public via Cloudflare)                  │
│   https://cockpit.leo-rover-gardon.dev/ops.html                      │
└──────────────────────────────────────────────────────────────────────┘
```

**Point de vérité réseau** : `tools/robot_env.sh` exporte `ROBOT_HOST`,
`ROS_MASTER_URI` et `ROS_IP`. Ne jamais coder l'IP en dur ailleurs.

---

## 2. Démarrage

```bash
cd /home/lab272/TOUT/web
./start_web.sh
```

Le script est **idempotent et non destructeur** : s'il trouve un service déjà
actif, il l'annonce `[= ] … INTACT` et n'y touche pas. C'est délibéré — une
version antérieure tuait puis relançait, et comme le WiFi peut flapper au
démarrage, elle tuait un rosbridge parfaitement sain toutes les deux minutes.
Un rosbridge peut mettre plus d'une minute à se lier sous charge : le laisser
tranquille est le comportement correct.

**Ce que `start_web.sh` ne lance PAS** : `leo_backend.py`, `pose_selector`,
`imu_sanitizer`, ni les estimateurs. Ils vivent dans
`navigation_supervision.launch`, avec `respawn` géré par roslaunch :

```bash
../catkin_ws/src/leo_navigation/launch_mins.sh navigation_master.launch
```

### Ports

| Port | Service | Exposition | Rôle |
|------|---------|-----------|------|
| 8000 | `serve.py` | **publique** (Cloudflare) | pages statiques |
| 8010 | sauvegarde repères | **127.0.0.1 uniquement** | écriture depuis `tf_validator.html` |
| 8080 | `web_video_server` | publique | flux MJPEG |
| 9090 | rosbridge | LAN | `ws://` direct |
| 9443 | rosbridge TLS | boucle locale → tunnel | `wss://` public |

Le **8010 est séparé du 8000 à dessein**. Le 8000 est publié sur internet ; y
ouvrir une route d'écriture l'ouvrirait à tout le monde. Un filtrage par IP
n'aiderait pas, puisque `cloudflared` se connecte lui-même en boucle locale.
Le 8010 écoute sur `127.0.0.1` seulement : la sauvegarde marche depuis le PC
et échoue à distance, ce qui est le comportement voulu.

Le tunnel a un interrupteur : créer `/tmp/leo_tunnel_off` empêche
`start_web.sh` de relancer `cloudflared`.

---

## 3. `serve.py` — et pourquoi ce n'est pas `python3 -m http.server`

`serve.py` remplace `http.server` pour **une** raison, mais elle est
structurante.

**Le problème.** Les documents qui changent à chaque mise à jour partaient
sans `Cache-Control`. Résultat : le navigateur servait des pages vieilles de
plusieurs versions. Pire, un étage au-dessus, le tunnel Cloudflare ne voyant
aucun `Cache-Control` sur un `.pdf` appliquait son défaut de 4 h et servait
publiquement un rapport de 300 pages alors que le disque en portait 309
(`cf-cache-status: HIT`, `age: 5256`). **Un rechargement forcé du navigateur
n'y pouvait rien : le cache était au bord du réseau, pas chez le client.**

**Le correctif.** `serve.py` envoie `Cache-Control: no-cache,
must-revalidate` sur `.html`, `.json`, `.js`, `.css` et le PDF du rapport.
`no-cache` ne désactive pas le cache : il force une **revalidation**, donc un
304 bon marché. Les assets versionnés (`?v=`) restent cachables normalement.

**Deuxième rôle, ajouté après l'audit du 2026-08-18.** `SimpleHTTPRequestHandler`
sert tout le répertoire, listings de dossiers compris — et ce répertoire est
publié sur internet. `serve.py` supprime ces listings.

### Les estampilles `?v=`

Chaque page charge les scripts partagés avec une estampille :
`i18n.js?v=r35`, `3d_engine.js?v=r3`. **Elles doivent rester identiques sur
toutes les pages.** Elles ont déjà dérivé deux fois (`app.js` en r28 sur deux
pages et r30 sur une troisième), et chaque page cachait alors une version
différente. Le `no-cache` de `serve.py` est le filet qui rend l'oubli
inoffensif, mais l'estampille reste le moyen d'invalidation immédiate.

Le PDF du rapport est estampillé automatiquement par `tools/update_report.sh`,
qui réécrit le lien sur les 11 pages qui le portent. **Sans le `?v=`,
Cloudflare sert une copie périmée.**

---

## 4. Les 14 pages

| Page | Rôle |
|------|------|
| `index.html` | Accueil, présentation de la plateforme |
| `ops.html` | **Cockpit** : pilotage, vidéo, télémétrie, FSM, commandes |
| `trajectory.html` | Trajectoires temps réel, comparaison des 3 estimateurs |
| `tf_validator.html` | Validation et édition des repères TF (écrit via :8010) ; vue 3D superposée des 3 extrinsèques |
| `mins_tuning.html` | Guide de réglage MINS, scène 3D des repères capteurs |
| `navigation_modes.html` | Modes de navigation, estimateurs, références amont ; scène 3D de la topologie de fusion |
| `robot_description.html` | Description matérielle du rover |
| `system.html` | Référence système et vocabulaire |
| `pid.html` | Régulation PID, réglage et courbes |
| `audit.html` | Audit d'ingénierie |
| `logbook.html` | Journal de bord (alimenté par `logbook_data.js`) |
| `hector_response.html` | Réponses techniques au Pr. Gutierrez (quaternion, sqrtVINS…) |
| `demo.html` | Page de démonstration, nav réduite |
| `preview-design.html` | Brouillon d'exploration visuelle, **non lié au site** |

`demo.html` et `preview-design.html` sont volontairement orphelines : elles
ne figurent dans aucune barre de navigation.

---

## 5. Sous le capot

### 5.1 Le système de design — `mission.css` (« Deep Space Control »)

Couche de jetons appliquée **par-dessus** Tailwind, sans toucher au markup
piloté par `app.js`. Une seule source de vérité visuelle.

```
--ms-bg       #0B0E14   fond mission        --ms-cyan    #22d3ee  données / VINS
--ms-bg-raise #10141c   panneaux            --ms-orange  #ff6b35  MINS / plasma
--ms-bg-high  #161b24   survols             --ms-alert   #FC3D21  critique
--ms-txt      #d4d9e1   texte               --ms-warn    #eab308  avertissement
--ms-dim      #6b7684   texte atténué       --ms-ok      #3fdc97  nominal
```

**La couleur porte du sens** : le cyan est la donnée et openVINS, l'orange est
MINS. Une nouvelle page qui parle de MINS s'accentue en orange, pas au hasard.

Parti pris central, et c'est le seul : **panneaux souples, commandes nettes.**
Les cartes gardent un rayon généreux (`.tile`, 1,25 rem) ; les boutons passent
à 6 px. Cette différence de traitement crée la hiérarchie — on distingue au
premier coup d'œil une surface d'un élément actionnable. L'amorti
`--ms-ease: cubic-bezier(.2,.7,.3,1)` est volontairement différent du défaut
Tailwind, qui donne à tous les sites la même inertie.

Chaque page redéclare la palette dans son bloc `tailwind.config` inline
(`space`, `accent`, `plasma`, `alert`, `ok`, `warn`). **Copier ce bloc tel
quel** dans toute nouvelle page.

### 5.2 Le moteur 3D — `3d_engine.js`

Three.js r128 + GSAP/ScrollTrigger, tous vendorisés. Cinq scènes exposées
sous `window.LEO3D` :

| Fonction | Utilisée par | Contenu |
|----------|-------------|---------|
| `initBackground(id, opts)` | index, ops, logbook, mins_tuning | globe filaire + anneaux, parallaxe souris |
| `initSatellite(id, opts)` | ops | satellite détaillé, rotation au défilement |
| `initCapsule(id, opts)` | logbook | capsule / module lunaire |
| `initPlanet(id, opts)` | ops | vignette orbitale (se dimensionne sur son canvas) |
| `initFrames(id, opts)` | mins_tuning | **repères capteurs orbitables à la souris** |
| `initTfCompare(id, opts)` | tf_validator | extrinsèques des 3 estimateurs superposées dans un repère IMU commun, orbitables |
| `initSensorFlow(id, opts)` | navigation_modes | topologie de fusion : seul MINS reçoit les roues |

Toutes renvoient `null` si WebGL est indisponible — **toujours garder l'appel
derrière `if (window.LEO3D && LEO3D.xxx)`** et concevoir la page pour rester
lisible sans la scène.

**La règle d'or, validée avec `initFrames`** : une scène 3D gagne sa place en
montrant un **mécanisme réel** du projet, pas en décorant. `initFrames` existe
parce qu'elle rend visible le `Rz(π)` entre le repère caméra et le repère
roues — un fait technique du rapport. Une scène sans contenu à expliquer n'a
pas sa place.

Densité : une page de lecture réduit le fond de moitié par rapport à
l'accueil (`rings: 2`, `nParticles: 750`) — le fond doit se faire oublier
derrière le texte.

`initTfCompare` reçoit ses données de la page (`setFrames(estimateurs,
couleurs)`) : le moteur ne lit pas `tf_frames.json` lui-même, il consomme ce
que la page a déjà chargé — les deux ne peuvent donc pas diverger. Les
marqueurs sont des sphères **filaires** de rayons décroissants : deux
estimateurs qui coïncident se lisent comme des cages emboîtées. Une sphère
pleine, même à 0,92 d'opacité, masque totalement celle du dessous (testé :
openVINS disparaissait entièrement sous MINS).

### 5.3 Le système bilingue — `i18n.js`

**725 clés en EN, 725 en FR**, strictement équilibrées. L'anglais est la
langue par défaut ; le choix est persisté dans `localStorage`
(`leo_lang_v2`).

Trois attributs :

| Attribut | Effet |
|----------|-------|
| `data-i18n="clé"` | remplace `textContent` (ou `placeholder` sur un `<input>`) |
| `data-i18n-html="clé"` | remplace `innerHTML` (valeur pouvant contenir des balises) |
| `data-i18n-aria="clé"` | pose `aria-label` — pour les contrôles à icône seule |

**Le piège à connaître** : `I18N.t()` renvoie **la clé elle-même** quand elle
est absente. Une clé oubliée n'échoue pas, elle s'affiche en brut à l'écran.
C'est arrivé (`traj_cmp3` affichait littéralement « traj_cmp3 » dans un
panneau). Après toute modification, vérifier :

```bash
# clés utilisées dans le HTML mais absentes de i18n.js
python3 - <<'PY'
import re, pathlib
s = pathlib.Path('i18n.js').read_text(encoding='utf-8')
a, b = s.index('    en: {'), s.index('    fr: {')
k = re.compile(r"^\s{6}([A-Za-z_][A-Za-z0-9_]*)\s*:", re.M)
EN, FR = set(k.findall(s[a:b])), set(k.findall(s[b:]))
used = set()
for p in pathlib.Path('.').glob('*.html'):
    used |= set(re.findall(r'data-i18n(?:-html|-aria)?="([^"]+)"',
                           p.read_text(encoding='utf-8', errors='replace')))
print("EN sans FR :", sorted(EN - FR) or "aucune")
print("FR sans EN :", sorted(FR - EN) or "aucune")
print("utilisées sans définition :", sorted(used - EN) or "aucune")
PY
```

Appeler `I18N.apply()` **après** le script de la page, jamais avant : les
éléments doivent exister quand `apply()` balaie le DOM.

---

## 6. Contrat ROS

| Topic | Type | Sens | Contenu |
|-------|------|------|---------|
| `/mission/telemetry` | `std_msgs/String` | backend → web | JSON état complet (~10 Hz) |
| `/mission/log` | `std_msgs/String` | backend → web | une ligne par évènement |
| `/mission/image_annotated` | `sensor_msgs/Image` | backend → web_video_server | BGR8 + overlay détection |
| `/mission/command` | `std_msgs/String` | web → backend | ordres JSON |
| `/cmd_vel` | `geometry_msgs/Twist` | backend → robot | consigne moteurs |

### Commandes JSON (`/mission/command`)

```json
{"action":"set_mode","mode":"AUTO"}
{"action":"target_beacon"}
{"action":"infinite_beacons"}
{"action":"stop"}
{"action":"reset"}
{"action":"clear_map"}
{"action":"set_params","hue_low":80,"hue_high":135,"v_min":200,"minled":3}
{"action":"manual","lin":0.2,"ang":0.0}
{"action":"set_view","mask":true}
```

---

## 7. Ajouter une page

1. **Partir d'une page existante proche**, pas d'une page blanche.
   `mins_tuning.html` est le gabarit le plus récent et le plus propre.
2. **Copier intégralement** : le `<head>` (i18n, `tailwind.config`, polices,
   `mission.css`), le `<header>` avec sa nav, le `<footer>`, et le bloc de
   scripts de fin.
3. **La nav existe en DEUX exemplaires** : desktop (`xl:flex`) et mobile
   (`#mobileMenu`). Ajouter le lien dans les deux, sur **toutes** les pages.
   Trois pages ont une nav simplifiée sans classes (`trajectory.html`,
   `tf_validator.html`) ou réduite (`demo.html`) — les traiter à part.
4. **Ajouter la clé de nav dans `i18n.js`**, en EN *et* en FR.
5. **Vérifier** : lancer le contrôle i18n de la §5.3, puis charger la page et
   confirmer qu'aucune clé brute ne s'affiche.

### Vérifier le site après modification

```bash
# structure, liens, ressources — sur le site servi
python3 - <<'PY'
import re, pathlib, urllib.request
BASE = "http://127.0.0.1:8000/"
urls = {}
for p in sorted(pathlib.Path('.').glob('*.html')):
    urls[p.name] = p.name
    s = p.read_text(encoding='utf-8', errors='replace')
    for u in re.findall(r'(?:src|href)="([^"]+)"', s):
        if not re.match(r'^(https?:|//|mailto:|data:|#|javascript:)', u) and u.strip():
            urls[u] = p.name
bad = []
for u, o in urls.items():
    try:
        if urllib.request.urlopen(BASE + u, timeout=15).status != 200:
            bad.append((u, o))
    except Exception as e:
        bad.append((u, f"{o} / {type(e).__name__}"))
print(f"{len(urls)} URL testées — en échec : {bad or 'aucune'}")
PY
```

Pour un rendu réel (et non une simple vérification de code de retour) :

```bash
firefox --headless --profile /tmp/ffprof --window-size 1440,900 \
        --screenshot /tmp/page.png "http://127.0.0.1:8000/ma_page.html"
```

**Puis regarder l'image.** Un CSS qui compile n'est pas un CSS qui s'affiche :
c'est ainsi qu'on a vu le fond 3D traverser le texte sur `mins_tuning.html`.
Attention aussi aux animations d'entrée (`animate-fade-up`) : une capture
prise trop tôt montre une page délavée qui n'a aucun défaut réel.

---

## 8. Sécurité

- **Homme-mort** : sans ordre reçu depuis 0,5 s en pilotage manuel, le robot
  s'arrête. Utile si la tablette se déconnecte.
- **Heartbeat** : `HEARTBEAT_TIMEOUT` 1,5 s + garde `HEARTBEAT_STALE_S` 5,0 s.
  La garde de péremption existe parce qu'une session navigateur fermée
  laissait un horodatage vieux de plusieurs minutes, et le failsafe se
  déclenchait à l'instant même où l'opérateur passait en AUTO.
- **Pas d'image → pas de mouvement** en mode automatique.
- **Arrêt d'urgence** : 5 Twist nuls + retour MANUEL.
- **Écriture confinée au 8010**, en boucle locale (voir §2).

---

## 9. Pièges déjà rencontrés

| Symptôme | Cause réelle |
|----------|-------------|
| Page ou PDF périmé malgré un rechargement forcé | cache Cloudflare au bord du réseau, pas chez le client — voir §3 |
| Une clé i18n s'affiche en brut | clé absente ; `I18N.t()` renvoie la clé, sans erreur |
| Cockpit affiche « LINK OK » sans aucune donnée | rosbridge orphelin ayant survécu à un redémarrage du master : le port accepte, rien ne circule |
| Le fond 3D traverse le texte | surface trop transparente pour une page de lecture — utiliser une tuile opaque, pas `.glass` |
| Un lien de nav manque sur certaines pages | la nav existe en deux exemplaires par page (desktop + mobile) |
| Le point d'accès WiFi tombe pendant un essai | flux caméra bruts souscrits à travers le WiFi — passer par les topics `/pc/camera/...` republiés localement |

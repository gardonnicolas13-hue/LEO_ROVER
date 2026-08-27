#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Endpoint d'écriture des repères édités depuis web/tf_validator.html.

POURQUOI UN SERVEUR SÉPARÉ, ET PAS UNE ROUTE DANS web/serve.py
--------------------------------------------------------------
`web/serve.py` écoute sur 0.0.0.0:8000 et **ce port est publié sur internet**
par le tunnel Cloudflare (https://cockpit.leo-rover-gardon.dev — voir l'audit
du 2026-08-18 dans l'en-tête de serve.py). Y ajouter une route POST qui écrit
dans les fichiers de calibration donnerait à n'importe qui le pouvoir de
réécrire la géométrie du robot.

Filtrer par adresse IP ne suffirait PAS : cloudflared tourne sur ce même PC et
se connecte en boucle locale, donc une requête venue d'internet arrive avec
127.0.0.1 comme adresse source. Elle serait indiscernable d'une requête locale.

Ce serveur écoute donc sur **127.0.0.1:8010**, un port que le tunnel ne relaie
pas. La conséquence est voulue et vaut d'être dite : la sauvegarde fonctionne
quand on ouvre le cockpit depuis le PC, et **échoue silencieusement à distance**.
C'est le comportement correct — on ne recalibre pas un robot depuis un
téléphone à l'autre bout du campus.

CE QUI EST ÉCRIT
----------------
  MINS      config_camera.yaml   T_imu_cam   (pose caméra dans le repère IMU)
  MINS      config_wheel.yaml    T_imu_wheel
  openVINS  kalibr_imucam_chain.yaml  T_cam_imu  — INVERSÉ avant écriture,
            puisque la page manipule partout la pose « capteur dans le repère
            IMU » et qu'openVINS stocke la convention opposée.
  sqrtVINS  config/leo_pc/kalibr_imucam_chain.yaml  T_cam_imu — INVERSÉ,
            même convention Kalibr qu'openVINS (charge_sqrtvins() dans
            export_tf_frames.py réutilise déjà charge_camchain_kalibr(),
            la même fonction que pour openVINS : mêmes maths ici).
            AJOUTÉ le 2026-08-27 — absent jusque-là (audit du 2026-08-26) :
            une édition sqrtVINS dans la page atteignait ce serveur, qui ne
            savait pas où l'écrire, et répondait « aucun repère marqué
            modifié » — un mensonge silencieux, l'opérateur en avait bien
            modifié un. Voir SRV_CHAIN ci-dessous.
            Seule leo_pc est écrite : c'est le fichier que charge
            l'instance PC. L'instance EMBARQUÉE (config/leo/, sur le Pi) a
            sa propre copie, délibérément séparée — voir tools/robot/sqrtvins/.

Le remplacement est CHIRURGICAL (mêmes expressions régulières que
fill_mins_camchain.py) : seules les quatre lignes de la matrice changent. Les
commentaires de ces fichiers portent l'essentiel de l'histoire du projet — un
yaml.dump() les effacerait tous.

TRAÇABILITÉ
-----------
  - sauvegarde horodatée `<fichier>.bak-tfval-<AAAAMMJJ_HHMMSS>` avant écriture ;
  - ligne ajoutée à calib_data/tf_edits.log (qui, quand, quelles valeurs).
Aucun commentaire n'est injecté DANS le YAML : il survivrait à une vraie
calibration ultérieure (fill_mins_camchain.py ne remplace que les matrices) et
finirait par affirmer « posé à la main » au-dessus d'une valeur Kalibr.

USAGE
    python3 tools/tf_save_server.py          # bloque, Ctrl-C pour arrêter
"""
import datetime
import json
import math
import os
import re
import shutil
import subprocess
import sys
import http.server

import numpy as np

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RACINE, 'catkin_ws', 'src', 'leo_navigation', 'scripts'))
sys.path.insert(0, os.path.join(RACINE, 'tools'))
from fill_mins_camchain import invert_se3, fmt_matrix        # noqa: E402

MINS_CAM = os.path.join(RACINE, 'LEO_Rover_Navigation_System', 'MINS-master',
                        'mins', 'config', 'leo', 'config_camera.yaml')
MINS_WHEEL = os.path.join(RACINE, 'LEO_Rover_Navigation_System', 'MINS-master',
                          'mins', 'config', 'leo', 'config_wheel.yaml')
OV_CHAIN = os.path.join(RACINE, 'catkin_ws', 'src', 'open_vins', 'config',
                        'leo', 'kalibr_imucam_chain.yaml')
# leo_pc, pas leo : c'est le fichier que charge l'instance PC de sqrtVINS
# (tools/launch_sqrtvins.sh), et c'est celui qu'export_tf_frames.py relit
# (charge_sqrtvins() -> SRV_CFG = .../config/leo_pc). Écrire ailleurs
# rendrait la page et le fichier réellement chargé incohérents entre eux.
SRV_CHAIN = os.path.join(RACINE, 'sqrtvins_ws', 'src', 'sqrtVINS', 'config',
                         'leo_pc', 'kalibr_imucam_chain.yaml')
JOURNAL = os.path.join(RACINE, 'calib_data', 'tf_edits.log')
PORT = 8010


def verifie_camera(nom, T):
    """Refuse une géométrie de caméra physiquement absurde sur ce rover.

    POURQUOI CE GARDE-FOU EXISTE (2026-08-20, incident réel)
    --------------------------------------------------------
    Le bouton « aligner l'orientation sur l'IMU » de tf_validator.html met la
    rotation à l'identité. Cliqué puis sauvegardé, il a écrit dans
    kalibr_imucam_chain.yaml une caméra dont l'axe optique pointait vers le
    HAUT au lieu de l'avant — sur les deux caméras à la fois. Rien n'a
    protesté : ni le YAML, ni openVINS au chargement suivant. Seul un `git
    status` l'a révélé.

    La règle physique invoquée ici est propre à CE robot : le D455 est boulonné
    à l'avant du châssis et regarde devant lui. L'axe optique Z, exprimé dans le
    repère IMU, doit donc être proche de +X. Une composante verticale dominante
    signale une erreur, pas un montage exotique.

    Fonction PURE : le serveur ET d'éventuels tests exercent le même code —
    même discipline que evaluer_recal() dans imu_sanitizer.py.

    Renvoie (accepte, motif). Le motif est TOUJOURS renseigné : un refus doit
    dire pourquoi, sinon l'opérateur ne peut que deviner.
    """
    R = np.asarray(T, dtype=float)[:3, :3]

    # 1. Matrice de rotation valide ? R·Rᵀ = I. Une matrice dégénérée
    #    (curseur bogué, charge utile tronquée) ne doit jamais atteindre le
    #    YAML : openVINS l'accepterait et divergerait sans message.
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
        return False, ("%s : la matrice n'est pas une rotation valide "
                       "(R·Rᵀ ≠ I). Écriture refusée." % nom)

    # 2. Rotation identité — le cas exact de l'incident du 2026-08-20.
    if np.allclose(R, np.eye(3), atol=1e-6):
        return False, (
            "%s : rotation IDENTITÉ refusée. L'axe optique Z de la caméra "
            "pointerait vers le HAUT (+Z de l'IMU) au lieu de l'AVANT. Sur ce "
            "rover, le D455 regarde devant : Z_optique doit valoir ≈ +X. "
            "Le bouton « aligner l'orientation sur l'IMU » sert à VOIR ce que "
            "donnerait un repère non tourné, jamais à l'écrire." % nom)

    # 3. Axe optique franchement vertical (> 45° hors de l'horizontale).
    #    Couvre toutes les autres façons d'arriver au même résultat — les
    #    curseurs, une copie depuis un mauvais volet, un collage manuel.
    z_opt = R[:, 2]
    if abs(float(z_opt[2])) > 0.7:
        direction = "le HAUT" if z_opt[2] > 0 else "le BAS"
        return False, (
            "%s : l'axe optique pointe vers %s (composante verticale %.2f). "
            "Sur ce rover la caméra regarde vers l'avant — Z_optique ≈ +X, "
            "composante verticale proche de 0. Écriture refusée." %
            (nom, direction, z_opt[2]))

    return True, "axe optique à %.0f° de l'horizontale, plausible" % (
        abs(math.degrees(math.asin(max(-1.0, min(1.0, float(z_opt[2])))))))


def sauvegarde(chemin):
    """Copie horodatée avant toute écriture. Le suffixe `.bak-` est celui que
    web/serve.py refuse de publier — une sauvegarde ne doit jamais fuiter."""
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    dest = '%s.bak-tfval-%s' % (chemin, stamp)
    shutil.copy2(chemin, dest)
    return dest


def remplace_matrice(txt, bloc, cle, T):
    """Remplace les 4 lignes de la matrice `cle` dans la section `bloc`.

    Même motif que fill_mins_camchain.py : on ancre sur `bloc:` puis sur la
    clé, et on ne touche QUE les quatre lignes `- [...]` qui suivent.
    Renvoie (texte, nb_remplacements) — 0 signale un ancrage manqué, jamais
    une écriture silencieusement perdue.
    """
    motif = re.compile(
        r"(^%s:\n(?:.*\n)*?\s*%s:\s*\n)"
        r"(?:\s*-\s*\[[^\]]*\]\s*\n){4}" % (re.escape(bloc), re.escape(cle)),
        re.MULTILINE)
    corps = fmt_matrix(np.asarray(T, dtype=float)) + "\n"
    return motif.subn(lambda m: m.group(1) + corps, txt)


def ecrit(payload):
    """Applique les repères reçus. Renvoie un rapport détaillé."""
    rapport = {'ecrits': [], 'ignores': [], 'sauvegardes': [], 'erreurs': []}
    est = payload.get('estimateurs', {})

    # ── GARDE-FOU, AVANT TOUTE ÉCRITURE ─────────────────────────────────────
    # Contrôlé ici et pas seulement dans la page : un test côté navigateur se
    # contourne (console, requête forgée, page en cache). C'est le serveur qui
    # tient le stylo, c'est donc lui qui doit refuser.
    # Tout ou rien : si UN repère est refusé, AUCUN n'est écrit. Un refus
    # partiel laisserait la configuration à moitié modifiée, état que personne
    # n'a demandé et que rien ne documenterait.
    #
    # FORÇAGE (`forcer: true`) : le garde-fou N'EST PAS retiré, il devient un
    # avertissement. Un clic accidentel reste bloqué ; seul un geste délibéré
    # passe outre. La distinction compte — c'est un incident de ce jour qui l'a
    # montré, une rotation identité ayant été écrite sans que rien ne proteste.
    # Un forçage est journalisé comme tel : la trace doit dire que la valeur a
    # été imposée contre l'avis du contrôle, pas qu'elle l'a satisfait.
    forcer = bool(payload.get('forcer'))
    refus = []
    for nom_est in ('MINS', 'openVINS', 'sqrtVINS'):
        for f in est.get(nom_est, {}).get('frames', []):
            if not f.get('modifie') or not re.match(r'cam\d+$', f.get('nom', '')):
                continue
            ok, motif = verifie_camera('%s %s' % (nom_est, f['nom']), f['T'])
            if not ok:
                refus.append(motif)
    if refus:
        if not forcer:
            rapport['erreurs'].extend(refus)
            rapport['ignores'].append(
                'AUCUNE écriture effectuée — les fichiers sont intacts')
            return rapport
        rapport['forces'] = refus
        rapport.setdefault('avertissements', []).append(
            'GARDE-FOU FORCÉ — %d contrôle(s) outrepassé(s) sur demande '
            'explicite. Les sauvegardes .bak-tfval-* permettent le retour.'
            % len(refus))

    # ── MINS : caméras (T_imu_cam) — la page tient déjà cette convention ────
    mins_frames = {f['nom']: f for f in est.get('MINS', {}).get('frames', [])}
    cams = {n: f for n, f in mins_frames.items()
            if re.match(r'cam\d+$', n) and f.get('modifie')}
    if cams:
        txt = open(MINS_CAM).read()
        bak = sauvegarde(MINS_CAM); rapport['sauvegardes'].append(bak)
        for nom, f in sorted(cams.items()):
            txt, n = remplace_matrice(txt, nom, 'T_imu_cam', f['T'])
            (rapport['ecrits'] if n else rapport['erreurs']).append(
                'MINS %s T_imu_cam' % nom + ('' if n else ' — ANCRAGE INTROUVABLE'))
        open(MINS_CAM, 'w').write(txt)

    # ── MINS : roues (T_imu_wheel) ──────────────────────────────────────────
    roues = mins_frames.get('roues')
    if roues and roues.get('modifie'):
        txt = open(MINS_WHEEL).read()
        bak = sauvegarde(MINS_WHEEL); rapport['sauvegardes'].append(bak)
        txt, n = remplace_matrice(txt, 'wheel', 'T_imu_wheel', roues['T'])
        (rapport['ecrits'] if n else rapport['erreurs']).append(
            'MINS T_imu_wheel' + ('' if n else ' — ANCRAGE INTROUVABLE'))
        open(MINS_WHEEL, 'w').write(txt)

    # ── openVINS : caméras (T_cam_imu) — INVERSION obligatoire ──────────────
    ov_frames = {f['nom']: f for f in est.get('openVINS', {}).get('frames', [])}
    ov_cams = {n: f for n, f in ov_frames.items()
               if re.match(r'cam\d+$', n) and f.get('modifie')}
    if ov_cams:
        txt = open(OV_CHAIN).read()
        bak = sauvegarde(OV_CHAIN); rapport['sauvegardes'].append(bak)
        for nom, f in sorted(ov_cams.items()):
            # La page manipule « caméra dans le repère IMU » ; openVINS veut
            # « IMU -> caméra ». Sans cette inversion la caméra part à l'envers
            # et rien ne le signale.
            T = invert_se3(np.asarray(f['T'], dtype=float))
            txt, n = remplace_matrice(txt, nom, 'T_cam_imu', T)
            (rapport['ecrits'] if n else rapport['erreurs']).append(
                'openVINS %s T_cam_imu (inversé)' % nom
                + ('' if n else ' — ANCRAGE INTROUVABLE'))
        open(OV_CHAIN, 'w').write(txt)

    # ── sqrtVINS : caméras (T_cam_imu) — INVERSION obligatoire ──────────────
    # Même convention Kalibr qu'openVINS (voir charge_sqrtvins() dans
    # export_tf_frames.py, qui appelle charge_camchain_kalibr() — la MÊME
    # fonction que pour openVINS) : même inversion, même structure d'écriture.
    # Seule l'instance PC (leo_pc) est écrite ici ; l'instance embarquée
    # (config/leo/, sur la carte SD du Pi) n'est pas atteinte par ce serveur
    # — voir SRV_CHAIN et tools/robot/sqrtvins/README.md.
    srv_frames = {f['nom']: f for f in est.get('sqrtVINS', {}).get('frames', [])}
    srv_cams = {n: f for n, f in srv_frames.items()
                if re.match(r'cam\d+$', n) and f.get('modifie')}
    if srv_cams:
        txt = open(SRV_CHAIN).read()
        bak = sauvegarde(SRV_CHAIN); rapport['sauvegardes'].append(bak)
        for nom, f in sorted(srv_cams.items()):
            T = invert_se3(np.asarray(f['T'], dtype=float))
            txt, n = remplace_matrice(txt, nom, 'T_cam_imu', T)
            (rapport['ecrits'] if n else rapport['erreurs']).append(
                'sqrtVINS %s T_cam_imu (inversé)' % nom
                + ('' if n else ' — ANCRAGE INTROUVABLE'))
        open(SRV_CHAIN, 'w').write(txt)

    if not rapport['ecrits'] and not rapport['erreurs']:
        rapport['ignores'].append('aucun repère marqué « modifié »')

    # ── RÉGÉNÉRATION + VÉRIFICATION ALLER-RETOUR ────────────────────────────
    # Sans ça, web/tf_frames.json reste figé à son ancien export : la page,
    # qui le relit à chaque chargement, réaffiche les VIEILLES valeurs alors
    # que le disque a changé. Bug constaté le 2026-08-20 — l'écriture
    # fonctionnait, mais rien ne le montrait, et on pouvait croire l'inverse.
    #
    # On régénère en RELISANT LES YAML, jamais depuis la charge utile reçue :
    # c'est ce qui transforme l'affichage en preuve. Si une écriture n'avait
    # pas mordu, la relecture rendrait l'ancienne valeur et l'écart
    # apparaîtrait ici plutôt que de passer inaperçu.
    if rapport['ecrits']:
        try:
            import importlib
            import export_tf_frames as X
            importlib.reload(X)
            X.main()
            relu = json.load(open(os.path.join(RACINE, 'web', 'tf_frames.json')))
            rapport['regenere'] = relu.get('genere')
            ecarts = []
            for e in ('MINS', 'openVINS', 'sqrtVINS'):
                voulus = {f['nom']: f for f in est.get(e, {}).get('frames', [])
                          if f.get('modifie')}
                sur_disque = {f['nom']: f
                              for f in relu['estimateurs'][e]['frames']}
                for nom, f in voulus.items():
                    d = sur_disque.get(nom)
                    if d is None:
                        ecarts.append('%s %s absent après relecture' % (e, nom))
                        continue
                    ecart = float(np.abs(np.array(f['T'], dtype=float)
                                         - np.array(d['T'], dtype=float)).max())
                    if ecart > 1e-6:
                        ecarts.append('%s %s écart %.2e après relecture'
                                      % (e, nom, ecart))
            rapport['verification'] = ('relecture conforme'
                                       if not ecarts else 'ÉCARTS : ' + ' ; '.join(ecarts))
            if ecarts:
                rapport['erreurs'].extend(ecarts)
        except Exception as exc:                   # noqa: BLE001
            rapport['erreurs'].append(
                'régénération de tf_frames.json ÉCHOUÉE (%s) — les YAML sont '
                'écrits mais la page continuera d\'afficher les anciennes '
                'valeurs : relancer python3 tools/export_tf_frames.py' % exc)

    if rapport['ecrits']:
        os.makedirs(os.path.dirname(JOURNAL), exist_ok=True)
        with open(JOURNAL, 'a') as fh:
            fh.write('[%s] tf_validator.html%s — %s\n' % (
                datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                '  *** GARDE-FOU FORCÉ ***' if rapport.get('forces') else '',
                ' ; '.join(rapport['ecrits'])))
            for motif in rapport.get('forces', []):
                fh.write('    FORCÉ malgré : %s\n' % motif.split('.')[0])
            for e in ('MINS', 'openVINS', 'sqrtVINS'):
                for f in est.get(e, {}).get('frames', []):
                    if f.get('modifie'):
                        fh.write('    %-9s %-6s T=%s\n' % (
                            e, f['nom'],
                            json.dumps([[round(v, 8) for v in r] for r in f['T']])))
    return rapport


def relance_pile():
    """Relance la pile de navigation via tools/restart_stack.sh.

    POURQUOI CET ENDPOINT EXISTE
    ----------------------------
    Les estimateurs ne lisent leur configuration QU AU DEMARRAGE. Ecrire une
    extrinseque depuis tf_validator.html ne changeait donc rien tant qu un
    humain n allait pas relancer la pile dans un terminal — le site ecrivait
    dans le vide, et rien ne le signalait. C est ce qui rendait l interface
    « passive » malgre son bouton de sauvegarde.

    restart_stack.sh est appele TEL QUEL : il porte deja les parametres
    d exploitation (LEO_ARGS) et l attente d initialisation MINS. Le
    dupliquer ici les ferait deriver l un de l autre.

    Detache via setsid : le script survit a la fin de cette requete HTTP.
    """
    chemin = os.path.join(RACINE, 'tools', 'restart_stack.sh')
    if not os.path.exists(chemin):
        return {'erreurs': ['restart_stack.sh introuvable : %s' % chemin]}
    journal = os.path.join(RACINE, 'logs', 'restart_stack.log')
    os.makedirs(os.path.dirname(journal), exist_ok=True)
    with open(journal, 'w') as fh:
        subprocess.Popen(['setsid', 'bash', chemin], stdout=fh, stderr=fh,
                         stdin=subprocess.DEVNULL, start_new_session=True)
    return {'relance': 'lancee',
            'note': ("La pile met ~15-30 s a revenir. Les estimateurs "
                     "reliront la configuration a ce moment-la, pas avant."),
            'journal': journal}


class Handler(http.server.BaseHTTPRequestHandler):
    def _cors(self):
        # La page est servie depuis :8000, ce serveur écoute sur :8010 —
        # origines distinctes, donc CORS obligatoire. Autorisation limitée à
        # la boucle locale : depuis internet, le navigateur ne peut de toute
        # façon pas joindre 127.0.0.1:8010.
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_POST(self):
        route = self.path.split('?')[0]
        if route not in ('/save_tf', '/reload_stack'):
            self.send_error(404); return
        try:
            n = int(self.headers.get('Content-Length', 0))
            brut = self.rfile.read(n).decode('utf-8') if n else '{}'
            payload = json.loads(brut) if brut.strip() else {}
            if route == '/reload_stack':
                rapport = relance_pile()
            else:
                rapport = ecrit(payload)
                # Ecrire sans relancer laisse les estimateurs sur l ancienne
                # configuration : on enchaine si le client le demande.
                if payload.get('relancer') and rapport.get('ecrits'):
                    rapport.update(relance_pile())
            corps = json.dumps(rapport, ensure_ascii=False).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self._cors()
            self.send_header('Content-Length', str(len(corps)))
            self.end_headers()
            self.wfile.write(corps)
        except Exception as e:                     # noqa: BLE001
            corps = json.dumps({'erreurs': [str(e)]}).encode('utf-8')
            self.send_response(500)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self._cors()
            self.send_header('Content-Length', str(len(corps)))
            self.end_headers()
            self.wfile.write(corps)

    def log_message(self, *a):
        pass


if __name__ == '__main__':
    print('tf_save_server : 127.0.0.1:%d  (boucle locale UNIQUEMENT — le '
          'tunnel Cloudflare ne relaie pas ce port)' % PORT)
    http.server.ThreadingHTTPServer(('127.0.0.1', PORT), Handler).serve_forever()

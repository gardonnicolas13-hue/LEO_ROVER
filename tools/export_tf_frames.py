#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Exporte les repères (TF) réellement configurés vers web/tf_frames.json.

POURQUOI CE SCRIPT EXISTE
-------------------------
Une matrice extrinsèque est illisible à l'œil : `[[0,-1,0,0],[0,0,-1,0.1],...]`
ne dit pas si l'axe Z de la caméra pointe vers l'avant ou vers le ciel. Une
inversion de convention, un axe retourné, une caméra placée derrière le
châssis — tout ça se lit en une seconde sur un dessin, et en dix minutes dans
un YAML. `web/tf_validator.html` fait ce dessin ; ce script lui fournit les
CHIFFRES RÉELS, lus dans les fichiers que les estimateurs chargent vraiment.

CE N'EST PAS UN OUTIL DE CALIBRATION. Rien ici ne modifie une extrinsèque, et
la page web ne propose aucun curseur d'ajustement — délibérément. Une
extrinsèque caméra-IMU relie le capteur inertiel enfoui sur la carte au CENTRE
OPTIQUE de la lentille, un point immatériel : ni l'un ni l'autre n'est visible
ni atteignable à la main, et openVINS exige une précision qu'aucun œil humain
n'atteint. Le calcul appartient à Kalibr (tools/calib_imucam_run.sh) ; cette
chaîne-ci sert à VALIDER son résultat, pas à le remplacer.

CONVENTIONS, ET POURQUOI ELLES PIÈGENT
--------------------------------------
Les deux estimateurs stockent la MÊME géométrie sous des conventions INVERSES :
  - MINS      config_camera.yaml : T_imu_cam  = caméra -> IMU
  - openVINS  kalibr_imucam_chain.yaml : T_cam_imu = IMU -> caméra
Confondre les deux retourne la caméra bout pour bout sans qu'aucun test
n'échoue. Ce script ramène TOUT dans un repère commun — la pose de chaque
capteur EXPRIMÉE DANS LE REPÈRE IMU — pour que la comparaison côte à côte ait
un sens. Vérifié le 2026-08-20 : les deux configs sont des inverses exacts
(écart max 0.0), donc cohérentes entre elles.

DÉTECTION « JAMAIS MESURÉ »
---------------------------
Chaque matrice est marquée `idealisee: true` si TOUS ses termes de rotation
valent exactement 0 ou ±1. Aucune calibration réelle ne produit ça : Kalibr
rend des 0.9998822, des -0.0153273. Une matrice parfaite est donc, sans
exception observée sur ce projet, une valeur posée à la main — placeholder,
mesure au mètre ruban, ou convention supposée. Le drapeau se met à jour tout
seul le jour où une vraie calibration remplace la valeur, sans qu'il faille
penser à éditer un commentaire.

USAGE
    python3 tools/export_tf_frames.py            # -> web/tf_frames.json
    python3 tools/export_tf_frames.py --stdout   # affiche sans écrire
"""
import json
import os
import sys

import math

import numpy as np

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RACINE, 'catkin_ws', 'src', 'leo_navigation', 'scripts'))

# Réutilisé, PAS réécrit : fill_mins_camchain.py sait déjà lire le format
# OpenCV FileStorage (« %YAML:1.0 » que PyYAML refuse) et inverser un SE(3).
# Une deuxième copie de ces deux fonctions dériverait de la première.
from fill_mins_camchain import load_kalibr, invert_se3      # noqa: E402

MINS_CFG = os.path.join(RACINE, 'LEO_Rover_Navigation_System', 'MINS-master',
                        'mins', 'config', 'leo')
OV_CFG = os.path.join(RACINE, 'catkin_ws', 'src', 'open_vins', 'config', 'leo')
# sqrtVINS (2026-08-21) : config PC, celle que le noeud « sqrtvins » charge
# reellement. La config soeur config/leo/ est celle du Pi -- meme geometrie,
# seuls les rostopic different -- et n'est donc pas relue ici.
SRV_CFG = os.path.join(RACINE, 'sqrtvins_ws', 'src', 'sqrtVINS', 'config', 'leo_pc')


def est_idealisee(T):
    """Vrai si la partie rotation ne contient que des 0 et des ±1 exacts.

    Voir DÉTECTION « JAMAIS MESURÉ » en en-tête : c'est la signature d'une
    valeur posée à la main, jamais d'une calibration.
    """
    R = np.asarray(T)[:3, :3]
    return bool(np.all(np.isclose(np.abs(R), np.round(np.abs(R)), atol=1e-9) &
                       (np.abs(R) <= 1.0 + 1e-9)))


def alignements(frames):
    """Contrôle les invariants GÉOMÉTRIQUES que ce montage doit respecter.

    POURQUOI CES RÈGLES-LÀ
    ----------------------
    Une matrice peut être une rotation parfaitement valide (R·Rᵀ = I) et
    décrire une géométrie physiquement impossible sur CE robot. Le garde-fou
    de tf_save_server.py n'attrape que le cas grossier (axe optique vertical).
    Ici on vérifie les invariants du MONTAGE : le D455 est un bloc rigide dont
    les deux imageurs sont sur une même barre horizontale, à la même hauteur,
    à la même profondeur, séparés d'une base connue.

    Chaque règle porte sa tolérance et sa justification. Une règle sans
    tolérance mesurable serait une opinion, pas un contrôle — et une tolérance
    trop serrée transformerait le bruit de calibration en fausse alerte.

    Vérifié le 2026-08-20 : la géométrie stéréo réellement mesurée par Kalibr
    (0,8913° entre les deux imageurs, 5 mm de décalage en z) PASSE ces règles.
    Elles rejettent l'absurde, pas le réel.

    Renvoie une liste de dicts {regle, valeur, tolerance, ok, explication}.
    """
    par_nom = {f['nom']: f for f in frames}
    c0, c1 = par_nom.get('cam0'), par_nom.get('cam1')
    R = []

    def ajoute(regle, val, unite, tol, ok, expl):
        R.append({'regle': regle, 'valeur': round(float(val), 5), 'unite': unite,
                  'tolerance': tol, 'ok': bool(ok), 'explication': expl})

    for nom, f in par_nom.items():
        if not nom.startswith('cam'):
            continue
        T = np.asarray(f['T'], dtype=float)
        z_opt = T[:3, 2]
        # Axe optique horizontal : le D455 est boulonné à plat, il regarde
        # devant. Tolérance 0.2 (~11°) : large, pour absorber une assiette
        # de châssis, mais très loin d'un axe qui viserait le plafond.
        ajoute('%s : axe optique horizontal' % nom, abs(z_opt[2]), '', 0.2,
               abs(z_opt[2]) <= 0.2,
               'composante verticale de l\'axe de visée ; ~0 = regarde devant')

    if c0 is not None and c1 is not None:
        T0 = np.asarray(c0['T'], dtype=float)
        T1 = np.asarray(c1['T'], dtype=float)
        p0, p1 = T0[:3, 3], T1[:3, 3]

        # 1. MÊME HAUTEUR — les deux imageurs sont sur la même barre.
        #    Tolérance 8 mm : Kalibr mesure 5 mm d'écart réel sur ce D455.
        dz = abs(p0[2] - p1[2])
        ajoute('caméras à la même hauteur', dz * 1000, 'mm', 8.0, dz <= 0.008,
               'écart vertical entre cam0 et cam1 ; elles partagent un support rigide')

        # 2. MÊME PROFONDEUR — même plan frontal.
        dx = abs(p0[0] - p1[0])
        ajoute('caméras à la même profondeur', dx * 1000, 'mm', 8.0, dx <= 0.008,
               'écart avant/arrière ; une différence signale un axe confondu')

        # 3. BASE STÉRÉO — spec D455 : 95 mm nominal, 90,2 mm mesuré par Kalibr.
        #    Tolérance 10 mm autour de 90 : rejette un facteur d\'échelle ou
        #    une unité fausse (cm/m), accepte la dispersion de fabrication.
        base = float(np.linalg.norm(p1 - p0))
        ajoute('base stéréo', base * 1000, 'mm', '90 ± 10',
               0.080 <= base <= 0.100,
               'distance entre les deux imageurs ; le D455 est à ~90 mm')

        # 4. AXES OPTIQUES QUASI PARALLÈLES — une paire stéréo n\'est pas
        #    convergente. Tolérance 3° : Kalibr mesure 0,89° de réel.
        ang = math.degrees(math.acos(max(-1.0, min(1.0,
              float(np.dot(T0[:3, 2], T1[:3, 2]))))))
        ajoute('axes optiques parallèles', ang, 'deg', 3.0, ang <= 3.0,
               'angle entre les deux axes de visée ; une paire stéréo est quasi parallèle')

        # 5. BASE PERPENDICULAIRE À LA VISÉE — la séparation est LATÉRALE.
        #    Une base alignée sur l\'axe optique rendrait la triangulation
        #    dégénérée (parallaxe nulle). Tolérance 10° autour de 90°.
        if base > 1e-6:
            u = (p1 - p0) / base
            perp = math.degrees(math.acos(max(-1.0, min(1.0,
                   abs(float(np.dot(u, T0[:3, 2])))))))
            ajoute('base perpendiculaire à la visée', perp, 'deg', '90 ± 10',
                   perp >= 80.0,
                   'la séparation doit être latérale ; alignée sur la visée, '
                   'la triangulation serait dégénérée')
    return R


def repere(nom, T, role, source, note=''):
    """Un repère prêt pour la page : matrice + provenance + verdict."""
    T = np.asarray(T, dtype=float)
    return {
        'nom': nom,
        'role': role,
        'T': [[round(float(v), 9) for v in ligne] for ligne in T],
        'position': [round(float(v), 6) for v in T[:3, 3]],
        'idealisee': est_idealisee(T),
        'source': source,
        'note': note,
    }


def charge_mins():
    """Repères MINS, tous ramenés dans le repère IMU."""
    cam = load_kalibr(os.path.join(MINS_CFG, 'config_camera.yaml'))
    roue = load_kalibr(os.path.join(MINS_CFG, 'config_wheel.yaml'))
    frames = [repere('imu', np.eye(4), 'centrale inertielle',
                     'repère de référence', 'origine de la comparaison')]

    # camN sont des clés de PREMIER NIVEAU (sœurs de « cam: », pas dedans) —
    # même disposition que le camchain openVINS, seule la convention change.
    for i in range(4):
        cle = 'cam%d' % i
        if cle not in cam or 'T_imu_cam' not in cam[cle]:
            continue
        # T_imu_cam est DÉJÀ caméra -> IMU : c'est la pose de la caméra
        # exprimée dans le repère IMU, aucune inversion nécessaire.
        frames.append(repere(cle, cam[cle]['T_imu_cam'],
                             'caméra infrarouge %d' % i,
                             'MINS config_camera.yaml (T_imu_cam)',
                             'convention caméra -> IMU'))

    bloc_r = roue.get('wheel', roue)
    if 'T_imu_wheel' in bloc_r:
        frames.append(repere('roues', bloc_r['T_imu_wheel'],
                             'odométrie roues (base_link)',
                             'MINS config_wheel.yaml (T_imu_wheel)',
                             'Rz(pi) : transmission remontée à 180° le 2026-07-08'))
    return frames


def charge_camchain_kalibr(dossier, nom_estimateur):
    """Repères d'un camchain de convention Kalibr, ramenés dans le repère IMU.

    Factorisé le 2026-08-21 : openVINS et sqrtVINS stockent tous deux
    T_cam_imu (IMU -> caméra) dans un kalibr_imucam_chain.yaml. Une seconde
    copie de cette fonction aurait dérivé de la première — c'est exactement
    ce qui est arrivé au calcul de bruit des biais dans sqrtVINS, où deux
    implémentations du même calcul avaient fini par diverger.
    """
    cfg = load_kalibr(os.path.join(dossier, 'kalibr_imucam_chain.yaml'))
    frames = [repere('imu', np.eye(4), 'centrale inertielle',
                     'repère de référence', 'origine de la comparaison')]
    for i in range(4):
        cle = 'cam%d' % i
        if cle not in cfg or 'T_cam_imu' not in cfg[cle]:
            continue
        # Convention Kalibr = IMU -> caméra : on INVERSE pour obtenir la pose
        # de la caméra dans le repère IMU, seule forme comparable à MINS.
        T = invert_se3(np.asarray(cfg[cle]['T_cam_imu'], dtype=float))
        frames.append(repere(cle, T, 'caméra infrarouge %d' % i,
                             '%s kalibr_imucam_chain.yaml (T_cam_imu, inversé)'
                             % nom_estimateur,
                             'convention IMU -> caméra, inversée ici'))
    return frames


def charge_openvins():
    """Repères openVINS, ramenés dans le repère IMU (donc INVERSÉS)."""
    return charge_camchain_kalibr(OV_CFG, 'openVINS')


def charge_sqrtvins():
    """Repères sqrtVINS, ramenés dans le repère IMU (donc INVERSÉS)."""
    return charge_camchain_kalibr(SRV_CFG, 'sqrtVINS')


def coherence(mins, autres):
    """Les estimateurs décrivent-ils tous la même géométrie ?

    Tous ont été ramenés dans le repère IMU, donc leurs matrices doivent
    coïncider avec celles de MINS, pris comme référence. Un écart signale une
    divergence de configuration RÉELLE — les estimateurs ne verraient alors
    pas la même caméra au même endroit, et toute comparaison de leurs sorties
    mesurerait ce désaccord de géométrie autant que leur mathématique.

    `autres` : dict {nom_estimateur: frames}. Étendu de 2 à N le 2026-08-21
    pour accueillir sqrtVINS.
    """
    resultats = []
    for nom, frames in autres.items():
        par_nom = {f['nom']: f for f in frames}
        for f in mins:
            if f['nom'] == 'imu' or f['nom'] not in par_nom:
                continue
            a = np.array(f['T']); b = np.array(par_nom[f['nom']]['T'])
            ecart = float(np.abs(a - b).max())
            resultats.append({
                'estimateur': nom,
                'contre': 'MINS',
                'repere': f['nom'],
                'ecart_max': round(ecart, 9),
                'coherent': ecart < 1e-6,
            })
    return resultats


def main():
    mins = charge_mins()
    ov = charge_openvins()
    try:
        srv = charge_sqrtvins()
    except Exception as exc:
        # sqrtvins_ws peut ne pas exister sur une machine qui n'a pas le
        # troisieme estimateur : la page doit alors montrer les DEUX autres
        # plutot que de ne rien montrer du tout.
        print('  sqrtVINS ignoré (%s)' % exc)
        srv = []
    sortie = {
        'genere': __import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'estimateurs': {
            'MINS': {'couleur': '#ff6b35', 'frames': mins},
            'openVINS': {'couleur': '#22d3ee', 'frames': ov},
            # Ambre : la couleur sqrtVINS partout ailleurs sur le site.
            'sqrtVINS': {'couleur': '#fbbf24', 'frames': srv},
        },
        'coherence': coherence(mins, {'openVINS': ov, 'sqrtVINS': srv}),
        'alignements': {'MINS': alignements(mins), 'openVINS': alignements(ov),
                        'sqrtVINS': alignements(srv)},
        'avertissement': ("Repères LUS dans les fichiers de configuration, pas "
                          "mesurés sur le robot. Une matrice marquée « idéalisée » "
                          "n'a jamais été calibrée."),
    }
    txt = json.dumps(sortie, indent=2, ensure_ascii=False)
    if '--stdout' in sys.argv:
        print(txt)
        return 0
    dest = os.path.join(RACINE, 'web', 'tf_frames.json')
    with open(dest, 'w') as fh:
        fh.write(txt + '\n')
    n_ideal = sum(1 for e in sortie['estimateurs'].values()
                  for f in e['frames'] if f['idealisee'] and f['nom'] != 'imu')
    print('  écrit : %s' % dest)
    print('  %d repères MINS, %d openVINS, %d sqrtVINS'
          % (len(mins), len(ov), len(srv)))
    print('  %d matrice(s) IDÉALISÉE(S) — jamais calibrées' % n_ideal)
    for c in sortie['coherence']:
        print('  %-6s écart %.2e  %s' % (c['repere'], c['ecart_max'],
                                         'cohérent' if c['coherent'] else 'INCOHÉRENT'))
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Génère une trace sqrtVINS SYNTHÉTIQUE, pour tester la chaîne d'analyse.

POURQUOI CE SCRIPT EXISTE
-------------------------
`compare_estimators.m` sait traiter trois estimateurs, mais n'a jamais pu être
exercé qu'avec deux : aucune capture ne contient de série sqrtVINS, parce que
`record_trajectories.sh` n'enregistrait pas `/ov_srvins/odomimu` (corrigé le
2026-08-18). Attendre la prochaine sortie terrain pour découvrir un défaut de
mise en page ou de calcul serait un mauvais échange. Ce script fabrique la
troisième série pour valider la chaîne AVANT que les vraies données arrivent.

CE QU'IL NE FAUT SURTOUT PAS EN FAIRE
-------------------------------------
Ces données sont FAUSSES. Elles ne mesurent rien, elles imitent une forme.
Tout ce projet repose sur la distinction entre ce qui est mesuré et ce qui est
supposé ; une trace synthétique qui se retrouverait analysée comme une vraie
contaminerait le résultat le plus visible du rapport. D'où trois garde-fous,
volontairement redondants :

  1. le nom de fichier porte `MOCK` en majuscules, impossible à survoler ;
  2. l'écriture dans `data/trajectories/` — où vivent les vraies captures —
     est REFUSÉE sauf `--force-danger`, et l'avertissement est explicite ;
  3. un fichier `<sortie>.SYNTHETIC.txt` est déposé à côté, qui dit en clair
     ce que le CSV contient et par quoi il a été produit.

FORME SIMULÉE
-------------
La divergence réelle observée le 13/08 n'était pas linéaire : le log de
sqrtVINS donne 19,3 m à l'image ~700, puis 128,8 / 275,4 / 6093,6 et enfin
43 742,6 m — le taux de croissance lui-même accélère. On reproduit ça : un
tour plausible au début, puis un emballement exponentiel. C'est la forme qui
compte pour éprouver l'échelle logarithmique et le cadrage, pas les valeurs.

USAGE
    python3 tools/make_mock_srv.py <prefixe_source> [-o <dossier_sortie>]

    <prefixe_source>  une capture existante dont on emprunte les horodatages,
                      p.ex. data/trajectories/sqrtvins_compare_20260813_175957
"""
import argparse
import csv
import math
import os
import sys

REELS = os.path.join('data', 'trajectories')


def lire_temps(path):
    """Emprunte la colonne t d'un CSV existant : la série synthétique doit
    partager la base de temps des vraies, sinon on ne teste pas le bon cas."""
    with open(path, newline='', encoding='utf8') as fh:
        r = csv.reader(fh)
        entete = next(r, None)
        if entete is None:
            raise SystemExit("ERREUR: %s est vide" % path)
        if len(entete) < 11:
            raise SystemExit("ERREUR: %s n'a que %d colonnes (11 attendues)"
                             % (path, len(entete)))
        return [float(row[0]) for row in r if row]


def trajectoire(ts):
    """Tour plausible, puis emballement exponentiel.

    Deux régimes, séparés à 45 % de la durée — proportion choisie pour coller
    au log réel, où le décrochage survient vers l'image 700 sur 6293."""
    if not ts:
        raise SystemExit("ERREUR: aucun horodatage lu")
    t0, t1 = ts[0], ts[-1]
    duree = max(t1 - t0, 1e-9)
    bascule = t0 + 0.45 * duree
    out = []
    for t in ts:
        u = (t - t0) / duree
        # Régime 1 : rectangle ~6 x 4,5 m parcouru une fois (ordre du vrai tour)
        ang = 2.0 * math.pi * min((t - t0) / (0.45 * duree), 1.0)
        x = 3.0 * math.sin(ang)
        y = 2.2 * (1.0 - math.cos(ang))
        if t > bascule:
            # Régime 2 : emballement. exp() sur la fraction écoulée depuis la
            # bascule ; le facteur 11 amène l'ordre de grandeur final autour de
            # 4e4 m, comme le log réel.
            v = (t - bascule) / max(t1 - bascule, 1e-9)
            g = math.exp(11.0 * v) - 1.0
            x += g * 2.3
            y -= g * 1.7
        lacet = math.atan2(y, x) if (x or y) else 0.0
        out.append((t, x, y, 0.0,
                    0.0, 0.0, math.sin(lacet / 2.0), math.cos(lacet / 2.0),
                    0.0, 0.0, lacet))
    return out


def main():
    ap = argparse.ArgumentParser(description="Trace sqrtVINS SYNTHETIQUE (test uniquement)")
    ap.add_argument('prefixe', help="prefixe d'une capture existante (sans _mins.csv)")
    ap.add_argument('-o', '--out-dir', default=None,
                    help="dossier de sortie (defaut : a cote, prefixe par MOCK_)")
    ap.add_argument('--force-danger', action='store_true',
                    help="autoriser l'ecriture dans data/trajectories/ (deconseille)")
    a = ap.parse_args()

    src = a.prefixe + '_mins.csv'
    if not os.path.isfile(src):
        raise SystemExit("ERREUR: source introuvable : %s" % src)
    ts = lire_temps(src)

    base = 'MOCK_' + os.path.basename(a.prefixe)
    out_dir = a.out_dir or os.path.dirname(a.prefixe) or '.'

    # Garde-fou 2 : refus d'ecrire parmi les vraies captures.
    norm = os.path.normpath(os.path.abspath(out_dir))
    if norm.endswith(os.path.normpath(REELS)) and not a.force_danger:
        raise SystemExit(
            "REFUS: %s heberge les captures REELLES.\n"
            "       Une trace synthetique n'y a rien a faire : elle finirait\n"
            "       par etre analysee comme une mesure. Donner -o vers un autre\n"
            "       dossier, ou --force-danger en connaissance de cause." % out_dir)

    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)
    cible = os.path.join(out_dir, base + '_srv.csv')

    lignes = trajectoire(ts)
    with open(cible, 'w', newline='', encoding='utf8') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
                    'roll_rad', 'pitch_rad', 'yaw_rad'])
        for r in lignes:
            w.writerow(['%.6f' % v for v in r])

    # Garde-fou 3 : un temoin lisible a cote du CSV.
    with open(cible.replace('_srv.csv', '.SYNTHETIC.txt'), 'w', encoding='utf8') as fh:
        fh.write(
            "DONNEES SYNTHETIQUES — NE MESURENT RIEN\n"
            "=======================================\n"
            "Fichier concerne : %s\n"
            "Produit par      : tools/make_mock_srv.py\n"
            "Horodatages      : empruntes a %s\n"
            "But              : exercer compare_estimators.m avec trois series\n"
            "                   avant que de vraies donnees sqrtVINS existent.\n"
            "Ne jamais citer ces chiffres comme un resultat.\n" % (cible, src))

    d = math.hypot(lignes[-1][1] - lignes[0][1], lignes[-1][2] - lignes[0][2])
    print("  ATTENTION : donnees SYNTHETIQUES, aucune valeur de mesure.")
    print("  ecrit    : %s" % cible)
    print("  points   : %d   duree %.1f s" % (len(lignes), ts[-1] - ts[0]))
    print("  ecart final au depart : %.0f m (imite l'emballement reel)" % d)


if __name__ == '__main__':
    sys.exit(main())

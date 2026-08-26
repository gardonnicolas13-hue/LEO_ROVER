#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Exposant de croissance de la dérive : d(t) ~ t^n, ajusté en log-log.

À QUOI SERT CE CHIFFRE
----------------------
L'exposant dit QUELLE erreur domine, ce qu'aucune distance finale ne dit :

  n ~ 1   dérive de vitesse (erreur de vitesse constante)
  n ~ 2   ACCÉLÉRATION constante -> biais/échelle ACCÉLÉROMÈTRE
  n ~ 3   accélération croissant linéairement -> erreur d'ATTITUDE croissante,
          c'est-à-dire un biais GYRO constant non compensé
  n ~ 4   le biais gyro DÉRIVE lui-même (thermique)

Mesuré le 2026-08-18 sur le roulage du 13/08 : MINS t^-0.01 (plat, ses roues
l'ancrent), openVINS t^3.25 et t^4.26 sur ses deux segments. C'est ce qui a
désigné le gyroscope plutôt que l'accéléromètre, et motivé la recalibration
périodique de imu_sanitizer.py.

USAGE ATTENDU APRÈS UN ESSAI
    python3 tools/drift_exponent.py <bag ou préfixe CSV>
Si la recalibration fait son travail, l'exposant d'openVINS doit descendre de
~3 vers ~2 : le biais gyro cesse de dominer, et il reste l'erreur d'échelle
accéléromètre en dessous. Un exposant inchangé signifie que la recalibration
n'a pas mordu — vérifier alors dans le journal du nœud combien de fois elle a
été APPLIQUÉE plutôt que refusée.

SEGMENTATION
    Un estimateur qui redémarre repart de son origine ; concaténer les deux
    portions donnerait un ajustement qui ne décrit rien. Les segments sont donc
    détectés (retour brutal vers 0 après éloignement) et ajustés séparément.

ENTRÉE
    <chemin>.bag        lit directement les topics d'odométrie du bag
    <préfixe>           lit <préfixe>_mins.csv / _vins.csv / _srv.csv

SORTIE MACHINE
    --json <fichier>    écrit les exposants au format JSON. C'est par là que
                        compare_estimators.m les récupère pour les joindre à
                        son tableau : le calcul reste ici, en un seul
                        exemplaire, plutôt que d'être réécrit en MATLAB où il
                        divergerait tôt ou tard de cette version.
"""
import csv
import json
import math
import os
import sys

# sqrtVINS apparait sous DEUX noms selon l'age du bag : /sqrtvins/odomimu
# depuis le renommage du noeud embarque (2026-08-25), /ov_srvins/odomimu
# avant. Les deux entrees portent la meme cle 'srv' ; un bag n'en contient
# jamais que l'une, donc la seconde reste simplement vide.
SERIES = [('MINS', 'mins', '/mins/imu/odom'),
          ('openVINS', 'vins', '/ov_msckf/odomimu'),
          ('sqrtVINS', 'srv', '/sqrtvins/odomimu'),
          ('sqrtVINS (ancien nom)', 'srv', '/ov_srvins/odomimu')]

RETOUR_ORIGINE = 0.5   # m — en deçà, on considère l'estimateur réinitialisé
ELOIGNE = 5.0          # m — au-delà, il s'était vraiment éloigné avant


def depuis_csv(prefixe, suffixe):
    p = '%s_%s.csv' % (prefixe, suffixe)
    if not os.path.isfile(p):
        return None
    with open(p, newline='', encoding='utf8') as fh:
        r = csv.reader(fh)
        entete = next(r, None)
        if not entete or len(entete) < 3:
            return None
        return [(float(a[0]), float(a[1]), float(a[2])) for a in r if a]


def depuis_bag(chemin, topic):
    try:
        import rosbag
    except ImportError:
        print("  (rosbag indisponible — sourcer ROS, ou passer un préfixe CSV)")
        return None
    pts = []
    try:
        with rosbag.Bag(chemin, 'r') as b:
            for _, msg, t in b.read_messages(topics=[topic]):
                p = msg.pose.pose.position
                pts.append((t.to_sec(), p.x, p.y))
    except Exception as e:
        print("  (lecture du bag impossible : %s)" % e)
        return None
    return pts or None


def segmenter(pts):
    """Découpe aux réinitialisations d'estimateur."""
    if not pts:
        return []
    x0, y0 = pts[0][1], pts[0][2]
    segs, cur = [], [pts[0]]
    for prev, now in zip(pts, pts[1:]):
        dp = math.hypot(prev[1] - x0, prev[2] - y0)
        dn = math.hypot(now[1] - x0, now[2] - y0)
        if dp > ELOIGNE and dn < RETOUR_ORIGINE:
            segs.append(cur)
            cur = [now]
            x0, y0 = now[1], now[2]
        else:
            cur.append(now)
    segs.append(cur)
    return segs


def exposant(seg):
    """Pente de log(d) vs log(t), et coefficient de détermination R².

    Le R² est renvoyé parce qu'une pente sans qualité d'ajustement peut
    tromper : une trajectoire qui plafonne puis explose donnera une pente
    moyenne dénuée de sens, et seul un R² faible le signale.
    """
    if len(seg) < 200:
        return None
    t0, x0, y0 = seg[0]
    pts = []
    for t, x, y in seg:
        dt = t - t0
        d = math.hypot(x - x0, y - y0)
        if dt > 5.0 and d > 0.5:      # hors bruit initial
            pts.append((math.log(dt), math.log(d)))
    if len(pts) < 100:
        return None
    n = float(len(pts))
    mx = sum(a for a, _ in pts) / n
    my = sum(b for _, b in pts) / n
    sxy = sum((a - mx) * (b - my) for a, b in pts)
    sxx = sum((a - mx) ** 2 for a, _ in pts)
    syy = sum((b - my) ** 2 for _, b in pts)
    if sxx <= 0 or syy <= 0:
        return None
    pente = sxy / sxx
    r2 = (sxy * sxy) / (sxx * syy)
    return pente, r2, len(pts)


def interprete(n):
    if n < 0.5:   return "plat — borné (capteur d'ancrage actif)"
    if n < 1.5:   return "n~1 : erreur de vitesse"
    if n < 2.5:   return "n~2 : ACCÉLÉROMÈTRE (biais/échelle)"
    if n < 3.5:   return "n~3 : GYRO, biais constant non compensé"
    return "n>=4 : GYRO dont le biais DÉRIVE"


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    arg = sys.argv[1]
    est_bag = arg.endswith('.bag')
    print("\n=== drift_exponent — %s ===" % arg)
    print("  d(t) ~ t^n, ajuste en log-log\n")
    sortie_json = None
    if '--json' in sys.argv:
        i = sys.argv.index('--json')
        if i + 1 >= len(sys.argv):
            print("  --json exige un chemin de fichier")
            return 2
        sortie_json = sys.argv[i + 1]
    resultats = {}

    vu = 0
    for nom, suf, topic in SERIES:
        pts = depuis_bag(arg, topic) if est_bag else depuis_csv(arg, suf)
        if not pts:
            print("  %-9s : absent" % nom)
            continue
        vu += 1
        segs = [s for s in segmenter(pts) if len(s) >= 200]
        print("  %-9s : %d point(s), %d segment(s)" % (nom, len(pts), len(segs)))
        for i, sg in enumerate(segs, 1):
            r = exposant(sg)
            if r is None:
                print("      segment %d : trop court ou trop peu de dérive" % i)
                continue
            pente, r2, npts = r
            duree = sg[-1][0] - sg[0][0]
            dfin = math.hypot(sg[-1][1] - sg[0][1], sg[-1][2] - sg[0][2])
            fiab = '' if r2 >= 0.9 else '   (R2 faible : ajustement peu fiable)'
            print("      segment %d : %5.1f s, d_final %9.1f m,  n = %.2f"
                  "  R2 = %.3f%s" % (i, duree, dfin, pente, r2, fiab))
            print("                  %s" % interprete(pente))
            resultats.setdefault(nom, []).append(
                {'segment': i, 'duration_s': duree, 'final_distance_m': dfin,
                 'exponent': pente, 'r_squared': r2, 'fit_points': npts,
                 'interpretation': interprete(pente)})
    if not vu:
        print("  aucune serie lue — verifier le chemin")
        return 1

    if sortie_json:
        # Le segment RETENU est le plus long : c'est celui qui porte le plus
        # d'information sur la derive. Les autres restent disponibles dans
        # `segments`, pour qu'un segment court et bruite ne devienne jamais LE
        # chiffre cite sans qu'on puisse le verifier.
        resume = {}
        for nom, segs in resultats.items():
            principal = max(segs, key=lambda x: x['duration_s'])
            resume[nom] = {'exponent': principal['exponent'],
                           'r_squared': principal['r_squared'],
                           'segment_used': principal['segment'],
                           'n_segments': len(segs),
                           'interpretation': principal['interpretation'],
                           'segments': segs}
        with open(sortie_json, 'w', encoding='utf8') as fh:
            json.dump({'source': arg, 'estimators': resume}, fh,
                      indent=2, ensure_ascii=False)
        print("  exposants JSON : %s" % sortie_json)
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())

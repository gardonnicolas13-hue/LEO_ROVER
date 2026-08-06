#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tag_drift.py — derive ABSOLUE mesuree sur une balise fixe (AprilTag).

POURQUOI (2026-08-05) : jusqu'ici la seule mesure de derive disponible sur
cette plateforme etait la FERMETURE DE BOUCLE (distance entre le point de
depart et le point d'arrivee). C'est une proxy : elle ne dit rien de ce qui
se passe EN COURS de tour, et un estimateur qui derive puis revient par
hasard pres du depart obtient une bonne note imméritee.

Une balise posee a un endroit FIXE leve cette limite. Le principe evite
volontairement toute connaissance des extrinseques camera :

    si la balise apparait a la MEME position dans le repere camera a deux
    instants differents, c'est que le robot est physiquement au MEME
    endroit. Les deux poses ESTIMEES a ces instants devraient donc
    coincider ; leur ecart est de la derive pure, en metres.

Aucune correction n'est appliquee nulle part : c'est une MESURE. Recaler la
pose sur la balise ferait au contraire disparaitre l'observable (la
trajectoire epouserait la reference par construction).

Usage :
    python3 tools/tag_drift.py <prefixe> [--source mins|vins] [--tol 0.05]

    <prefixe> designe les CSV produits par bag_to_csv.py, p.ex.
    'web/exports/test1' pour test1_tags.csv + test1_mins.csv.

    --tol : ecart maximal (m) entre deux vues pour les considerer prises du
            meme endroit. 0.05 = 5 cm, prudent.
"""
import argparse
import csv
import math
import os
import sys


def read_tags(path):
    """[(t, id, x, y, z)] — position de la balise dans le repere camera."""
    out = []
    if not os.path.isfile(path):
        return out
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                out.append((float(r["t"]), int(r["id"]),
                            float(r["x"]), float(r["y"]), float(r["z"])))
            except (KeyError, ValueError):
                continue
    return out


def read_pose(path):
    """[(t, x, y, yaw)] — pose estimee."""
    out = []
    if not os.path.isfile(path):
        return out
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                out.append((float(r["t"]), float(r["x"]), float(r["y"]),
                            float(r["yaw_rad"])))
            except (KeyError, ValueError):
                continue
    return out


def pose_at(poses, t):
    """Interpolation lineaire de la pose a l'instant t (None hors plage)."""
    if not poses or t < poses[0][0] or t > poses[-1][0]:
        return None
    lo, hi = 0, len(poses) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if poses[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, x0, y0, _ = poses[lo]
    t1, x1, y1, _ = poses[hi]
    if t1 == t0:
        return x0, y0
    a = (t - t0) / (t1 - t0)
    return x0 + a * (x1 - x0), y0 + a * (y1 - y0)


def main():
    ap = argparse.ArgumentParser(description="derive absolue via balise fixe")
    ap.add_argument("base", help="prefixe des CSV (sans _tags.csv)")
    ap.add_argument("--source", default="mins", choices=("mins", "vins", "fused"))
    ap.add_argument("--tol", type=float, default=0.05,
                    help="ecart max (m) entre deux vues jugees identiques")
    args = ap.parse_args()

    base = args.base
    for suf in ("_tags.csv", "_mins.csv", "_vins.csv", "_fused.csv"):
        if base.endswith(suf):
            base = base[: -len(suf)]
    tags = read_tags(base + "_tags.csv")
    poses = read_pose("%s_%s.csv" % (base, args.source))

    if not tags:
        sys.stderr.write(
            "aucune detection dans %s_tags.csv\n"
            "  -> la balise n'a jamais ete vue pendant l'essai, ou le bag est\n"
            "     anterieur a l'ajout de /tag_detections a l'enregistrement.\n"
            % base)
        return 1
    if not poses:
        sys.stderr.write("aucune pose dans %s_%s.csv\n" % (base, args.source))
        return 1

    ids = sorted({d[1] for d in tags})
    print("balises vues : %s   (%d detections, source %s)"
          % (ids, len(tags), args.source.upper()))
    print()

    any_pair = False
    for tid in ids:
        views = [d for d in tags if d[1] == tid]
        # Apparier les vues prises du MEME endroit (meme position relative de
        # la balise dans le repere camera, a --tol pres).
        pairs = []
        for i in range(len(views)):
            for j in range(i + 1, len(views)):
                a, b = views[i], views[j]
                if b[0] - a[0] < 5.0:      # au moins 5 s d'ecart : sinon on
                    continue               # compare un instant avec lui-meme
                d = math.dist(a[2:5], b[2:5])
                if d <= args.tol:
                    pairs.append((a, b, d))
        if not pairs:
            print("  balise %-4d : %3d vues, aucune paire vue du meme endroit"
                  " (tolerance %.0f cm)" % (tid, len(views), args.tol * 100))
            continue
        any_pair = True
        print("  balise %-4d : %3d vues, %d paires exploitables" % (tid, len(views), len(pairs)))
        drifts = []
        for a, b, d in pairs:
            pa = pose_at(poses, a[0])
            pb = pose_at(poses, b[0])
            if pa is None or pb is None:
                continue
            drift = math.dist(pa, pb)
            drifts.append((b[0] - a[0], drift))
        if not drifts:
            print("       (aucune pose estimee disponible a ces instants)")
            continue
        drifts.sort(key=lambda x: x[0])
        print("       ecart temps   DERIVE MESUREE")
        for dt, dr in drifts[:8]:
            print("       %8.1f s      %6.3f m" % (dt, dr))
        worst = max(d[1] for d in drifts)
        print("       -> derive maximale observee : %.3f m" % worst)
    print()
    if any_pair:
        print("Lecture : cette derive est ABSOLUE (le robot etait physiquement")
        print("au meme endroit aux deux instants). Contrairement a la fermeture")
        print("de boucle, elle ne peut pas etre flattee par un retour fortuit")
        print("pres du depart.")
    else:
        print("Aucune paire exploitable : il faut REPASSER devant la balise")
        print("depuis sensiblement le meme endroit (plusieurs tours), ou")
        print("augmenter --tol.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

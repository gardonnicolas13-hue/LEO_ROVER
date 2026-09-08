#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
decimate_stl.py — reduit un STL binaire par regroupement de sommets (voxel grid).

POURQUOI CET OUTIL EXISTE
    web/models/leo_rover_v1.9_v5.stl est un maillage CAO : 1 162 904 triangles,
    58 Mo. C'est acceptable sur robot_description.html, une page de
    visualisation dediee ou l'on accepte un chargement long. Ce ne l'est PAS
    sur trajectory.html, qui est une page OPERATIONNELLE : elle porte un flux
    rosbridge temps reel, et 58 Mo de geometrie y entrent en concurrence
    directe avec la telemetrie. Un rover affiche a ~60 px sur une carte n'a
    besoin que de quelques centaines de triangles pour que sa silhouette soit
    juste.

METHODE (regroupement par voxels, pas de dependance externe)
    Seuls numpy et scipy sont disponibles sur cette machine (ni trimesh, ni
    open3d, ni meshlab). Le regroupement par voxels est le compromis correct
    ici : simple, robuste, et il preserve la SILHOUETTE, qui est exactement ce
    qui compte pour une icone vue de dessus. Une decimation par erreur
    quadratique (QEM) preserverait mieux les details fins, mais ces details
    sont invisibles a la taille d'affichage visee.

    1. chaque sommet est quantifie sur une grille de pas `voxel`
    2. tous les sommets d'une meme cellule sont remplaces par leur centroide
    3. les triangles devenus degeneres (au moins deux sommets dans la meme
       cellule) sont supprimes
    4. les normales sont RECALCULEES depuis la geometrie reduite : garder
       celles d'origine donnerait un ombrage faux apres deplacement des
       sommets

USAGE
    python3 tools/decimate_stl.py <entree.stl> <sortie.stl> [--voxel MM]
    python3 tools/decimate_stl.py entree.stl sortie.stl --target 3000
"""
import argparse
import os
import struct
import sys

import numpy as np


def lire_stl_binaire(chemin):
    """Renvoie (n, 3, 3) float32 : les sommets de chaque triangle."""
    with open(chemin, "rb") as f:
        entete = f.read(84)
        if entete[:5].lower() == b"solid":
            raise SystemExit("STL ASCII non gere : ce script attend du binaire.")
        n = struct.unpack("<I", entete[80:84])[0]
        brut = np.frombuffer(f.read(n * 50), dtype=np.uint8)
    if brut.size != n * 50:
        raise SystemExit(f"fichier tronque : {brut.size} octets pour {n} triangles attendus")
    return brut.reshape(n, 50)[:, 12:48].copy().view("<f4").reshape(n, 3, 3)


def ecrire_stl_binaire(chemin, tris, entete=b"decimate_stl.py"):
    n = len(tris)
    # normales recalculees : les sommets ont bouge, les anciennes sont fausses
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    nor = np.cross(v1 - v0, v2 - v0)
    lon = np.linalg.norm(nor, axis=1, keepdims=True)
    nor = np.divide(nor, lon, out=np.zeros_like(nor), where=lon > 0)

    bloc = np.zeros((n, 50), dtype=np.uint8)
    bloc[:, 0:12] = nor.astype("<f4").view(np.uint8).reshape(n, 12)
    bloc[:, 12:48] = tris.astype("<f4").view(np.uint8).reshape(n, 36)
    with open(chemin, "wb") as f:
        f.write(entete.ljust(80, b"\0"))
        f.write(struct.pack("<I", n))
        f.write(bloc.tobytes())


def decimer(tris, voxel):
    """Regroupement par voxels. Renvoie les triangles conserves."""
    plats = tris.reshape(-1, 3)
    cles = np.floor(plats / voxel).astype(np.int64)

    # identifiant de cellule unique, puis centroide par cellule
    uniq, inverse = np.unique(cles, axis=0, return_inverse=True)
    nb = len(uniq)
    somme = np.zeros((nb, 3), dtype=np.float64)
    np.add.at(somme, inverse, plats)
    compte = np.bincount(inverse, minlength=nb).reshape(-1, 1)
    centroides = (somme / compte).astype(np.float32)

    idx = inverse.reshape(-1, 3)
    # un triangle dont deux sommets tombent dans la meme cellule est degenere
    garde = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])
    return centroides[idx[garde]]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("entree")
    ap.add_argument("sortie")
    ap.add_argument("--voxel", type=float, default=None,
                    help="pas de la grille, dans l'unite du STL (mm ici)")
    ap.add_argument("--target", type=int, default=None,
                    help="nombre de triangles vise ; ajuste le voxel automatiquement")
    a = ap.parse_args()

    tris = lire_stl_binaire(a.entree)
    mn, mx = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    diag = float(np.linalg.norm(mx - mn))
    print(f"entree : {len(tris)} triangles, "
          f"{os.path.getsize(a.entree)/1e6:.1f} Mo, diagonale {diag:.1f}")

    if a.target and not a.voxel:
        # recherche par dichotomie sur le pas : monotone, converge en ~12 tours
        bas, haut = diag / 2000.0, diag / 4.0
        for _ in range(14):
            mid = (bas + haut) / 2
            n = len(decimer(tris, mid))
            if n > a.target:
                bas = mid
            else:
                haut = mid
        voxel = haut
    else:
        voxel = a.voxel if a.voxel else diag / 120.0

    out = decimer(tris, voxel)
    ecrire_stl_binaire(a.sortie, out)
    print(f"sortie : {len(out)} triangles, "
          f"{os.path.getsize(a.sortie)/1e6:.3f} Mo, voxel {voxel:.2f}")
    print(f"reduction : {len(tris)/max(len(out),1):.0f}x en triangles, "
          f"{os.path.getsize(a.entree)/max(os.path.getsize(a.sortie),1):.0f}x en octets")


if __name__ == "__main__":
    main()

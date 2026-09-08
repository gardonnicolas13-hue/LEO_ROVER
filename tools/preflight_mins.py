#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
preflight_mins.py — contrôle avant essai de la chaîne MINS. Verdict GO / NO-GO.

POURQUOI CET OUTIL EXISTE
    Toutes les pannes MINS de ce projet ont un point commun : elles sont
    SILENCIEUSES. Le processus tourne, il est abonné, rien ne plante, aucune
    erreur ne s'affiche — et l'estimateur ne produit rien, ou produit faux.
    Un contrôle de présence de processus ne les voit pas. Exemples vécus :

      - imu_sanitizer verrouillé sur un horodatage daté de 2076 : 520 451
        mesures rejetées d'affilée, /imu/data_clean muet, MINS sans IMU
        (2026-09-04) ;
      - pose_selector conservant une correction figée : MINS parfait en brut
        (z ≈ 0), /robot_pose_fused décalé de 1,25 m (2026-09-04) ;
      - accel_scale périmé par la dérive du capteur : z à 1 196 m, robot
        immobile (2026-09-04) ;
      - MINS relancé pendant une dégradation réseau, verrouillé sur une
        initialisation corrompue (2026-09-03).

    Chacune aurait été prise en trente secondes par les contrôles ci-dessous.
    À lancer AVANT toute campagne de mesure, toute démonstration, et après
    chaque redémarrage de la pile.

USAGE
    python3 tools/preflight_mins.py            # robot supposé IMMOBILE
    python3 tools/preflight_mins.py --rapide   # saute le test de dérive 30 s

PRÉREQUIS : le robot doit être à l'arrêt et posé à plat. Le test de dérive
n'a aucun sens en mouvement, et le contrôle de |a| non plus.
"""
import argparse
import math
import re
import subprocess
import sys

ROBOT = "10.154.6.41"
MASTER = f"http://{ROBOT}:11311"
GRAVITE = 9.790          # gravité locale, Melbourne FL — cf. config_estimator.yaml
SEUIL_BATTERIE = 10.3    # V, seuil du projet (leo_backend.py)

VERT, ROUGE, JAUNE, GRIS, RAZ = "\033[92m", "\033[91m", "\033[93m", "\033[90m", "\033[0m"
resultats = []


def sh(cmd, timeout=20):
    """Exécute une commande ROS. Jamais de pipe vers grep/head : la sortie de
    rostopic est bufferisée quand elle n'est pas un TTY, et un timeout la tue
    avant qu'elle ne vide son tampon — le contrôle échoue alors qu'il n'y a
    aucun problème. Piège rencontré plusieurs fois sur ce projet."""
    env = f"source /opt/ros/noetic/setup.bash >/dev/null 2>&1; export ROS_MASTER_URI={MASTER}; "
    try:
        r = subprocess.run(["bash", "-c", env + cmd], capture_output=True,
                           text=True, timeout=timeout)
        return r.stdout
    except subprocess.TimeoutExpired:
        return ""


def note(nom, ok, detail, bloquant=True):
    resultats.append((nom, ok, detail, bloquant))
    tag = f"{VERT}  OK  {RAZ}" if ok else (f"{ROUGE}ÉCHEC {RAZ}" if bloquant
                                           else f"{JAUNE}ALERTE{RAZ}")
    print(f"  [{tag}] {nom:<34} {detail}")


def lire_xyz(txt):
    vals, cur = [], {}
    for l in txt.splitlines():
        m = re.match(r"^([xyz]): (-?[\d.eE+-]+)", l.strip())
        if m:
            cur[m.group(1)] = float(m.group(2))
            if len(cur) == 3:
                vals.append((cur["x"], cur["y"], cur["z"])); cur = {}
    return vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rapide", action="store_true",
                    help="saute le test de dérive statique (30 s)")
    a = ap.parse_args()

    print(f"\n{GRIS}{'='*72}{RAZ}")
    print("  CONTRÔLE AVANT ESSAI — chaîne MINS         (robot supposé IMMOBILE)")
    print(f"{GRIS}{'='*72}{RAZ}\n")

    # ── 1. le robot répond ────────────────────────────────────────────────
    p = subprocess.run(["ping", "-c", "3", "-W", "2", ROBOT],
                       capture_output=True, text=True)
    perte = 100
    m = re.search(r"(\d+)% packet loss", p.stdout)
    if m:
        perte = int(m.group(1))
    lat = re.search(r"= [\d.]+/([\d.]+)/", p.stdout)
    note("réseau robot", perte == 0,
         f"{perte}% de perte" + (f", {float(lat.group(1)):.1f} ms" if lat else ""))

    # ── 2. batterie ───────────────────────────────────────────────────────
    # Première cause vérifiée devant TOUT symptôme moteur : une batterie sous
    # le seuil imite une panne logicielle (télémétrie normale, moteurs muets).
    out = sh("timeout 8 rostopic echo -n1 /firmware/battery")
    mv = re.search(r"data:\s*([\d.]+)", out)
    if mv:
        v = float(mv.group(1))
        note("batterie", v >= SEUIL_BATTERIE, f"{v:.2f} V  (seuil {SEUIL_BATTERIE} V)")
    else:
        note("batterie", False, "illisible")

    # ── 3. IMU : débit ET justesse ────────────────────────────────────────
    out = sh("timeout 14 rostopic echo -n 300 /imu/data_clean/linear_acceleration", 25)
    A = lire_xyz(out)

    # Le flux vivant se déduit de cette MÊME capture : si 300 échantillons
    # sont arrivés, l'IMU publie, point. Une seconde requête pour le
    # redemander serait redondante et, mesurée, moins fiable que celle-ci.
    # C'est ce contrôle qui attrape le verrou de l'horodatage : quand le
    # sanitizer est verrouillé, /imu/data_clean est totalement muet.
    note("IMU — flux vivant", len(A) >= 100,
         f"{len(A)} échantillons reçus" if len(A) >= 100
         else f"seulement {len(A)} — sanitizer probablement verrouillé")

    if len(A) >= 100:
        n = [math.sqrt(x*x + y*y + z*z) for x, y, z in A]
        moy = sum(n) / len(n)
        ecart = moy - GRAVITE
        # 0,02 m/s² est le seuil retenu par le projet : au-delà, c'est
        # l'échelle accéléromètre qui a dérivé, pas les extrinsèques.
        note("IMU — |a| au repos", abs(ecart) <= 0.02,
             f"{moy:.4f} m/s²  écart {ecart:+.4f}  (cible {GRAVITE})")
    else:
        note("IMU — |a| au repos", False, "pas assez d'échantillons pour juger")

    # ── 5. caméras (seule contrainte sur z) ───────────────────────────────
    for t in ("infra1", "infra2"):
        out = sh(f"timeout 8 rostopic echo -n1 /pc/camera/{t}/image_rect_raw/height")
        # rostopic ajoute une ligne "---" après le message : isdigit() sur la
        # sortie brute échoue donc alors que la caméra publie. On cherche un
        # entier plausible (hauteur d'image) sur l'une des lignes.
        h = next((int(l) for l in out.splitlines()
                  if l.strip().isdigit() and 100 <= int(l) <= 4000), None)
        note(f"caméra {t}", h is not None,
             f"{h} px de haut" if h else "MUETTE")

    # ── 6. roues (ancrage x/y, et preuve d'immobilité) ────────────────────
    out = sh("timeout 8 rostopic echo -n1 /firmware/wheel_states/velocity")
    imm = "[0.0, 0.0, 0.0, 0.0]" in out
    note("roues — flux", bool(out.strip()), "publie" if out.strip() else "MUETTES")
    note("roues — robot immobile", imm,
         "à l'arrêt" if imm else "EN MOUVEMENT — le reste du test est invalide")

    # ── 7. MINS produit ───────────────────────────────────────────────────
    out = sh("timeout 10 rostopic echo -n1 /mins/imu/odom/pose/pose/position")
    brut = lire_xyz(out)
    note("MINS — publie", bool(brut), "oui" if brut else "MUET")

    # ── 8. pose_selector fidèle au brut ───────────────────────────────────
    # Attrape la correction figée : MINS peut être parfait et la pose servie
    # au cockpit décalée de plusieurs mètres, sans aucune alarme.
    out = sh("timeout 10 rostopic echo -n1 /robot_pose_fused/pose/pose/position")
    fus = lire_xyz(out)
    if brut and fus:
        d = math.dist(brut[0], fus[0])
        note("pose_selector — sans décalage", d < 0.05,
             f"écart brut/fusionné {d*1000:.1f} mm")
    else:
        note("pose_selector — sans décalage", False, "comparaison impossible")

    # ── 9. dérive statique ────────────────────────────────────────────────
    if not a.rapide and brut:
        print(f"\n{GRIS}  mesure de dérive sur 30 s…{RAZ}")
        out = sh("timeout 45 rostopic echo -n 2500 /mins/imu/odom/pose/pose/position", 60)
        P = lire_xyz(out)
        if len(P) >= 500:
            d = math.dist(P[0], P[-1]) * 1000
            amp = max(math.dist(P[0], p) for p in P) * 1000
            note("MINS — dérive statique", d < 30,
                 f"{d:.1f} mm sur {len(P)} éch.  (amplitude {amp:.1f} mm, critère < 30 mm)")
        else:
            note("MINS — dérive statique", False, f"seulement {len(P)} échantillons")

    # ── verdict ───────────────────────────────────────────────────────────
    durs = [r for r in resultats if r[3] and not r[1]]
    doux = [r for r in resultats if not r[3] and not r[1]]
    print(f"\n{GRIS}{'-'*72}{RAZ}")
    if durs:
        print(f"  {ROUGE}NO-GO{RAZ} — {len(durs)} contrôle(s) bloquant(s) en échec :")
        for n, _, d, _ in durs:
            print(f"      • {n} : {d}")
        print("\n  Ne pas lancer de campagne de mesure dans cet état.")
        sys.exit(1)
    print(f"  {VERT}GO{RAZ} — toute la chaîne MINS est conforme."
          + (f"  ({len(doux)} alerte(s) non bloquante(s))" if doux else ""))
    sys.exit(0)


if __name__ == "__main__":
    main()

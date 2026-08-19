#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tests des trois gardes de la recalibration du biais gyro.

CE QUI EST TESTÉ, ET POURQUOI CE N'EST PAS UNE COPIE
----------------------------------------------------
Ce script importe `evaluer_recal` DEPUIS `imu_sanitizer.py` — la fonction que
le nœud appelle réellement. Une version antérieure de ces tests réimplémentait
les règles ; elle ne prouvait alors que sa propre cohérence, et une divergence
entre le test et le code serait passée inaperçue. Ici, si quelqu'un modifie une
garde sans mettre les tests à jour, ils échouent.

`imu_sanitizer.py` importe `rospy` et `leo_msgs` au chargement. Ces modules ne
sont pas nécessaires à la fonction testée, mais leur absence empêcherait
l'import. On les remplace donc par des bouchons minimaux AVANT l'import — ce
qui permet de lancer ces tests sur n'importe quelle machine, sans ROS et sans
robot. C'est précisément l'intérêt : ces gardes protègent un chemin qui
alimente les deux estimateurs et le pilotage, elles doivent pouvoir être
vérifiées sans mobiliser le matériel.

USAGE
    python3 tools/test_gyro_recal_guards.py        # 0 si tout passe
"""
import math
import os
import sys
import types

# ── Bouchons ROS, posés AVANT l'import du module testé ───────────────────────
for nom in ('rospy', 'std_srvs', 'std_srvs.srv', 'leo_msgs', 'leo_msgs.msg',
            'sensor_msgs', 'sensor_msgs.msg'):
    if nom not in sys.modules:
        sys.modules[nom] = types.ModuleType(nom)
sys.modules['leo_msgs.msg'].Imu = object
sys.modules['leo_msgs.msg'].WheelStates = object
sys.modules['sensor_msgs.msg'].Imu = object
sys.modules['std_srvs.srv'].Trigger = object

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RACINE, 'catkin_ws', 'src', 'leo_navigation', 'scripts'))
from imu_sanitizer import evaluer_recal          # noqa: E402  (après les bouchons)

# Réglages par défaut du nœud, repris tels quels.
MAX_STD, MAX_DELTA, STILL = 0.01, 0.02, 3.0
BIAIS = [-0.04200, +0.02662, +0.04062]


def stats(ech):
    """Moyennes et écarts-types par axe, comme le fait le nœud."""
    n = float(len(ech))
    means = [sum(s[i] for s in ech) / n for i in range(3)]
    stds = [(sum((s[i] - means[i]) ** 2 for s in ech) / n) ** 0.5 for i in range(3)]
    return means, stds


def constant(v, n=400):
    return [tuple(v) for _ in range(n)]


def bruite(v, amp, n=400):
    """Signal centré sur v, oscillant d'amplitude amp sur l'axe X."""
    return [(v[0] + amp * math.sin(i / 7.0), v[1], v[2]) for i in range(n)]


# (nom, échantillons, immobile_depuis, acceptation attendue)
CAS = [
    ("dérive thermique lente, robot arrêté",
     constant([-0.0455, 0.0290, 0.0430]), 10.0, True),

    ("biais identique (rien n'a bougé)",
     constant(BIAIS), 10.0, True),

    ("robot EN MOUVEMENT (roues actives)",
     constant([-0.0455, 0.0290, 0.0430]), 0.5, False),

    ("arrêt trop récent (1 s < 3 s exigées)",
     constant([-0.0455, 0.0290, 0.0430]), 1.0, False),

    ("aucune donnée roue reçue",
     constant([-0.0455, 0.0290, 0.0430]), None, False),

    ("roues arrêtées mais robot soulevé / tourné",
     bruite([-0.0420, 0.0266, 0.0406], 0.05), 10.0, False),

    ("saut brutal du biais (mesure prise en manœuvre)",
     constant([-0.0420, 0.0266, 0.5000]), 10.0, False),

    ("écart juste SOUS la borne (0.019 < 0.02)",
     constant([BIAIS[0] + 0.019, BIAIS[1], BIAIS[2]]), 10.0, True),

    ("écart juste AU-DESSUS de la borne (0.021 > 0.02)",
     constant([BIAIS[0] + 0.021, BIAIS[1], BIAIS[2]]), 10.0, False),

    ("immobilité pile au seuil (3.0 s)",
     constant([-0.0455, 0.0290, 0.0430]), 3.0, True),
]


def main():
    print("  Gardes de recalibration gyro — %d cas\n" % len(CAS))
    echecs = 0
    for nom, ech, immo, attendu in CAS:
        means, stds = stats(ech)
        obtenu, motif = evaluer_recal(means, stds, BIAIS, immo,
                                      MAX_STD, MAX_DELTA, STILL)
        ok = (obtenu == attendu)
        if not ok:
            echecs += 1
        print("  %s  %-48s %s" % ('ok  ' if ok else 'ECHEC',
                                  nom,
                                  'ACCEPTE' if obtenu else 'refuse'))
        if not ok:
            print("        attendu %s, obtenu %s" %
                  ('ACCEPTE' if attendu else 'refuse',
                   'ACCEPTE' if obtenu else 'refuse'))
        print("        motif : %s" % motif)

    print()
    if echecs:
        print("  %d cas en ECHEC sur %d" % (echecs, len(CAS)))
        return 1
    print("  %d/%d cas conformes" % (len(CAS), len(CAS)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

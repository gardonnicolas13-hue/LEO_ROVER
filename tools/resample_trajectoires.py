#!/usr/bin/env python3
"""Rééchantillonne les trois estimateurs sur une base de temps COMMUNE, pour MATLAB.

POURQUOI CE SCRIPT EN PLUS DE export_matlab.py
----------------------------------------------
`export_matlab.py` capture les trois estimateurs en direct et les écrit tels
quels, chacun avec SA propre base de temps. C'est le bon choix pour archiver
une mesure : rien n'est transformé, rien n'est perdu.

Mais on ne peut pas comparer trois séries ligne à ligne quand elles ont des
horodatages différents. Les trois tournent à ~84-85 Hz, avec des dérives
indépendantes : mesuré sur ce robot, MINS 84,76 Hz, openVINS 85,02 Hz,
sqrtVINS 83,65 Hz. Sur 90 s cela fait plus d'une centaine d'échantillons
d'écart entre le premier et le dernier. Comparer `MINS(k)` à `sqrtVINS(k)`
reviendrait à comparer deux instants différents, et l'écart mesuré
contiendrait ce décalage autant que la différence d'estimateur.

Ce script produit donc une base de temps unique et y ramène les trois.

DEUX POINTS OÙ JE M'ÉCARTE DE LA SPÉCIFICATION, ET POURQUOI
-----------------------------------------------------------

1. LES QUATERNIONS NE SONT PAS INTERPOLÉS LINÉAIREMENT.
   Une interpolation linéaire composante par composante entre deux quaternions
   unitaires ne produit PAS un quaternion unitaire : le résultat quitte la
   sphère unité, et il faudrait le renormaliser — ce qui donne alors une
   rotation qui n'est pas celle du chemin le plus court, et dont la vitesse
   angulaire n'est pas constante. L'erreur est petite pour deux orientations
   proches (ici ~12 ms d'écart) mais elle est systématique, et sur un rapport
   d'ingénierie une erreur systématique est pire qu'une erreur aléatoire.
   On utilise donc SLERP (`scipy.spatial.transform.Slerp`), qui interpole le
   long de la géodésique de SO(3) et reste unitaire par construction.
   Les positions, elles, sont bien interpolées linéairement : c'est correct
   dans un espace vectoriel.

2. L'ALIGNEMENT D'ORIGINE EST UNE TRANSFORMATION SE(3) COMPLÈTE, PAS UNE
   SIMPLE TRANSLATION.
   Ramener les positions à (0,0,0) ne suffit pas : chaque estimateur définit
   son propre repère monde, et deux trajectoires partant du même point mais
   avec des lacets initiaux différents divergeront visuellement sans qu'aucun
   estimateur n'ait tort. On applique donc, pour chaque estimateur,
   l'inverse de sa pose initiale :

       p'(t) = R0^T (p(t) - p0)
       q'(t) = q0^-1 * q(t)

   Après quoi les trois partent de l'origine ET du même cap. C'est ce qui
   rend la superposition interprétable. `--sans-rotation` permet de ne faire
   que la translation si l'on veut conserver les caps d'origine.

USAGE
-----
    # depuis un bag
    python3 tools/resample_trajectoires.py --bag data/trajectories/xxx.bag

    # en direct, 90 secondes
    python3 tools/resample_trajectoires.py --live 90

    # cadence de sortie (défaut : 50 Hz, sous le ~84 Hz des sources)
    python3 tools/resample_trajectoires.py --bag xxx.bag --hz 50

Sortie dans data/matlab/ : un CSV par estimateur, colonnes
`time,x,y,z,qw,qx,qy,qz`, plus un CSV `_ecarts` donnant les distances
inter-estimateurs à chaque instant — c'est la grandeur qu'on veut réellement
tracer.
"""
import argparse
import os
import sys

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEST = os.path.join(RACINE, 'data', 'matlab')

TOPICS = {
    'mins':     '/mins/imu/odom',
    'openvins': '/ov_msckf/odomimu',
    'sqrtvins': '/sqrtvins/odomimu',
    # Instance embarquee sur le Raspberry Pi. Elle ne consomme PAS les memes
    # entrees que les trois autres : camera et IMU locales, sans le saut WiFi
    # ni la republication /pc/ que subissent les instances du PC. Un ecart
    # entre 'sqrtvins' et 'sqrtvins_pi' mesure donc la difference
    # d'ARCHITECTURE a code identique -- pas une difference d'estimateur.
    'sqrtvins_pi': '/ov_srvins/odomimu',
}


def lit_bag(chemin):
    """Extrait (t, p, q) de chaque topic d'un bag. q en (x,y,z,w), ordre scipy."""
    import numpy as np
    import rosbag
    brut = {k: {'t': [], 'p': [], 'q': []} for k in TOPICS}
    dispo = set()
    with rosbag.Bag(chemin) as bag:
        presents = set(bag.get_type_and_topic_info().topics.keys())
        voulus = {v: k for k, v in TOPICS.items() if v in presents}
        dispo = set(voulus.values())
        if not voulus:
            return {}, dispo
        for topic, msg, _ in bag.read_messages(topics=list(voulus.keys())):
            k = voulus[topic]
            p, o = msg.pose.pose.position, msg.pose.pose.orientation
            brut[k]['t'].append(msg.header.stamp.to_sec())
            brut[k]['p'].append([p.x, p.y, p.z])
            brut[k]['q'].append([o.x, o.y, o.z, o.w])
    return ({k: {c: np.asarray(v, dtype=float) for c, v in d.items()}
             for k, d in brut.items() if d['t']}, dispo)


def lit_live(duree):
    """Même structure, mais capturée en direct sur les topics ROS."""
    import socket
    socket.setdefaulttimeout(8)
    import numpy as np
    import rospy
    from nav_msgs.msg import Odometry
    brut = {k: {'t': [], 'p': [], 'q': []} for k in TOPICS}

    def cb(msg, cle):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        brut[cle]['t'].append(msg.header.stamp.to_sec())
        brut[cle]['p'].append([p.x, p.y, p.z])
        brut[cle]['q'].append([o.x, o.y, o.z, o.w])

    rospy.init_node('resample_trajectoires', anonymous=True)
    for cle, topic in TOPICS.items():
        rospy.Subscriber(topic, Odometry,
                         (lambda c: lambda m: cb(m, c))(cle), queue_size=200)
    print('Capture de %.0f s...' % duree)
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < duree and not rospy.is_shutdown():
        rospy.sleep(1.0)
    # Instantané cohérent : les callbacks écrivent encore pendant la lecture.
    fige = {}
    for k, d in brut.items():
        n = min(len(d['t']), len(d['p']), len(d['q']))
        if n:
            fige[k] = {'t': np.asarray(d['t'][:n], dtype=float),
                       'p': np.asarray(d['p'][:n], dtype=float),
                       'q': np.asarray(d['q'][:n], dtype=float)}
    return fige, set(fige.keys())


def aligne_origine(p, q, avec_rotation):
    """Applique l'inverse de la pose initiale : p' = R0^T (p - p0), q' = q0^-1 q."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    p0 = p[0].copy()
    p_al = p - p0
    if not avec_rotation:
        return p_al, q
    R0 = Rotation.from_quat(q[0])
    R0i = R0.inv()
    p_al = R0i.apply(p_al)
    q_al = (R0i * Rotation.from_quat(q)).as_quat()
    # q et -q representent la MEME rotation (double recouvrement de SU(2) sur
    # SO(3)). scipy peut renvoyer l'un ou l'autre, et un changement de signe
    # au milieu de la serie ne change rien a l'orientation mais fait sauter
    # toute courbe tracee naivement sur qw..qz -- artefact classique, pris
    # pour une discontinuite d'estimateur. On impose donc un signe continu,
    # en repartant du premier echantillon.
    for i in range(1, len(q_al)):
        if np.dot(q_al[i], q_al[i - 1]) < 0:
            q_al[i] = -q_al[i]
    return p_al, q_al


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--bag', help='fichier rosbag a lire')
    src.add_argument('--live', type=float, help='capture en direct, en secondes')
    ap.add_argument('--hz', type=float, default=50.0,
                    help='cadence de la base de temps commune (defaut 50)')
    ap.add_argument('--nom', default='resample', help='prefixe des fichiers')
    ap.add_argument('--sans-rotation', action='store_true',
                    help='aligner seulement la position, pas le cap initial')
    a = ap.parse_args()

    import numpy as np
    from scipy.interpolate import interp1d
    from scipy.spatial.transform import Rotation, Slerp

    brut, dispo = (lit_bag(a.bag) if a.bag else lit_live(a.live))
    if not brut:
        print('ECHEC : aucun des topics attendus trouve.')
        print('  attendus : %s' % ', '.join(TOPICS.values()))
        return 1

    manquants = set(TOPICS) - set(brut)
    if manquants:
        print('AVERTISSEMENT : absent(s) de la source -> %s' % ', '.join(sorted(manquants)))
        print('  la comparaison portera sur %d estimateur(s).' % len(brut))

    # ── Base de temps commune : l'INTERSECTION des fenetres ─────────────────
    # Prendre l'union obligerait a extrapoler la ou un estimateur n'a pas
    # encore publie -- une extrapolation n'est pas une mesure, et elle
    # apparaitrait dans le CSV sans se distinguer du reste.
    t_deb = max(d['t'][0] for d in brut.values())
    t_fin = min(d['t'][-1] for d in brut.values())
    if t_fin <= t_deb:
        print('ECHEC : aucun recouvrement temporel entre les series.')
        return 1
    n = int((t_fin - t_deb) * a.hz)
    if n < 2:
        print('ECHEC : recouvrement trop court (%.2f s a %.0f Hz).' % (t_fin - t_deb, a.hz))
        return 1
    t_commun = np.linspace(t_deb, t_fin, n)
    print('Base commune : %.2f s, %d echantillons a %.0f Hz' % (t_fin - t_deb, n, a.hz))

    os.makedirs(DEST, exist_ok=True)
    import time as _time
    horo = _time.strftime('%Y%m%d_%H%M%S')
    resultats = {}

    for cle, d in sorted(brut.items()):
        # Horodatages strictement croissants : un doublon ou un recul ferait
        # echouer interp1d, et un recul temporel n'a de toute facon pas de sens.
        ordre = np.argsort(d['t'], kind='stable')
        t, p, q = d['t'][ordre], d['p'][ordre], d['q'][ordre]
        garde = np.concatenate(([True], np.diff(t) > 0))
        t, p, q = t[garde], p[garde], q[garde]
        n_ecarte = len(d['t']) - len(t)

        # On reechantillonne D'ABORD, on aligne ENSUITE. L'ordre importe :
        # aligner sur le premier echantillon BRUT laisse un residu, parce que
        # la base commune demarre a max(premiers instants) et non au premier
        # echantillon de CETTE serie. Mesure du defaut avant correction :
        # origine a (-2.2e-4, -2.1e-5, 6.1e-4) au lieu de zero exact.
        # Aligner apres coup donne (0,0,0) par construction.
        p_r = np.column_stack([
            interp1d(t, p[:, i], kind='linear', bounds_error=False,
                     fill_value=(p[0, i], p[-1, i]))(t_commun)
            for i in range(3)])
        # Orientation : SLERP, pas lineaire -- voir l'en-tete du fichier.
        q_r = Slerp(t, Rotation.from_quat(q))(t_commun).as_quat()
        p_r, q_r = aligne_origine(p_r, q_r, not a.sans_rotation)

        resultats[cle] = (p_r, q_r)
        chemin = os.path.join(DEST, '%s_%s_%s.csv' % (a.nom, horo, cle))
        M = np.column_stack([t_commun - t_deb, p_r,
                             q_r[:, 3], q_r[:, 0], q_r[:, 1], q_r[:, 2]])
        np.savetxt(chemin, M, delimiter=',',
                   header='time,x,y,z,qw,qx,qy,qz', comments='', fmt='%.9g')
        print('  %-9s %6d ech source (%d ecarte(s)) -> %s'
              % (cle, len(t), n_ecarte, os.path.basename(chemin)))

    # ── Ecarts inter-estimateurs : la grandeur qu'on veut reellement tracer ──
    cles = sorted(resultats)
    if len(cles) >= 2:
        cols, noms = [t_commun - t_deb], ['time']
        for i in range(len(cles)):
            for j in range(i + 1, len(cles)):
                a_, b_ = cles[i], cles[j]
                cols.append(np.linalg.norm(resultats[a_][0] - resultats[b_][0], axis=1))
                noms.append('d_%s_%s' % (a_, b_))
        chemin = os.path.join(DEST, '%s_%s_ecarts.csv' % (a.nom, horo))
        np.savetxt(chemin, np.column_stack(cols), delimiter=',',
                   header=','.join(noms), comments='', fmt='%.9g')
        print('  %-9s -> %s' % ('ecarts', os.path.basename(chemin)))
        for k, nom in enumerate(noms[1:], start=1):
            v = cols[k]
            print('      %-24s median %7.3f m   max %8.3f m' % (nom, np.median(v), v.max()))

    print('')
    print("Sous MATLAB :  T = readtable('<fichier>.csv');")
    print("               plot(T.x, T.y);")
    return 0


if __name__ == '__main__':
    sys.exit(main())

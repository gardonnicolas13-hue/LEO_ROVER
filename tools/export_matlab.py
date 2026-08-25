#!/usr/bin/env python3
"""Export synchronisé des TROIS estimateurs vers MATLAB (.mat + CSV).

POURQUOI CE SCRIPT PLUTÔT QUE L'EXPORT DU SITE
----------------------------------------------
L'export du cockpit passe par le tampon de leo_backend, qui est VIDÉ à chaque
relance de la pile, et par rosbridge, qui peut désynchroniser (constaté le
2026-07-27, d'où tools/record_trajectories.sh). Celui-ci s'abonne directement
aux topics ROS : il ne dépend ni du navigateur, ni du backend, ni du tunnel.

CE QUI EST CAPTURÉ, ET POURQUOI CHAQUE CHAMP EST LÀ
---------------------------------------------------
Pour chacun des trois estimateurs (MINS, openVINS, sqrtVINS) :
  - t          : horodatage CAPTEUR (header.stamp), jamais l'heure d'arrivée.
                 L'heure d'arrivée mélange le jitter WiFi au jitter capteur
                 (mesuré : 63 % contre 40 %, cf. tools/audit_imu_jitter.py) et
                 seul le temps capteur entre dans les filtres.
  - p          : position (x, y, z) en mètres, repère monde de l'estimateur.
  - q          : quaternion (x, y, z, w), convention ROS.
  - v          : vitesse linéaire (x, y, z), repère CORPS (REP-103).
  - w          : vitesse angulaire (x, y, z), repère corps.
  - P_pose     : covariance 6x6 de la pose, APLATIE EN 36 COLONNES par ligne,
                 ordre ligne-major ROS. Reconstruction MATLAB :
                     P = reshape(P_pose(k,:), 6, 6)';   % NOTER la transposée
                 La transposée est indispensable : MATLAB lit en colonne-major,
                 ROS écrit en ligne-major. L'oublier donne une matrice
                 transposée qui reste symétrique en apparence si la vraie l'est,
                 donc l'erreur ne se voit PAS — d'où cet avertissement.
  - P_twist    : idem pour la covariance 6x6 des vitesses.
  - sigma3     : bornes 3-sigma sur x, y, z, extraites de la diagonale de
                 P_pose. Pré-calculées ici parce que c'est l'usage principal.

Pour openVINS et sqrtVINS en plus :
  - feat_t, feat_n : nombre de features réellement CONSOMMÉES par mise à jour
                 (largeur du nuage points_msckf). C'est la seule mesure honnête
                 de la santé du canal visuel : le tracé peut sembler correct
                 pendant que le filtre n'intègre rien (mesuré : 0,75 feature
                 pour 40 attendues, cf. leo-openvins-features-rejetees).

Et le témoin indépendant :
  - wheel_t, wheel_v : vitesses des 4 roues. Elles disent si le robot bougeait
                 VRAIMENT. Sans elles, impossible de distinguer une dérive
                 d'estimateur d'un déplacement réel — c'est ce qui a fait
                 prendre un problème d'affichage pour une divergence VINS.

USAGE
-----
    python3 tools/export_matlab.py 120 essai_nom
    python3 tools/export_matlab.py 120 essai_nom --csv-seulement

Sortie dans data/matlab/ : <nom>_<horodatage>.mat et/ou des CSV par estimateur.
"""
import os
import sys
import time

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEST = os.path.join(RACINE, 'data', 'matlab')

ESTIMATEURS = {
    'MINS':     '/mins/imu/odom',
    'openVINS': '/ov_msckf/odomimu',
    'sqrtVINS': '/sqrtvins/odomimu',
}
FEATURES = {
    'openVINS': '/ov_msckf/points_msckf',
    'sqrtVINS': '/sqrtvins/points_msckf',
}
ROUES = '/firmware/wheel_states'


def nom_matlab(s):
    """MATLAB refuse les noms de champ non alphanumériques ('sqrtVINS' passe,
    un nom avec tiret ou espace non). Normalisé ici plutôt qu'au chargement."""
    return ''.join(c if c.isalnum() else '_' for c in s)


def collecte(duree):
    import socket
    socket.setdefaulttimeout(8)
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from leo_msgs.msg import WheelStates

    donnees = {k: {'t': [], 'p': [], 'q': [], 'v': [], 'w': [],
                   'P_pose': [], 'P_twist': []} for k in ESTIMATEURS}
    feats = {k: {'t': [], 'n': []} for k in FEATURES}
    roues = {'t': [], 'v': []}

    def cb_odom(msg, cle):
        d = donnees[cle]
        d['t'].append(msg.header.stamp.to_sec())
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        vl = msg.twist.twist.linear
        va = msg.twist.twist.angular
        d['p'].append([p.x, p.y, p.z])
        d['q'].append([o.x, o.y, o.z, o.w])
        d['v'].append([vl.x, vl.y, vl.z])
        d['w'].append([va.x, va.y, va.z])
        d['P_pose'].append(list(msg.pose.covariance))
        d['P_twist'].append(list(msg.twist.covariance))

    def cb_feat(msg, cle):
        feats[cle]['t'].append(msg.header.stamp.to_sec())
        feats[cle]['n'].append(int(msg.width))

    def cb_roues(msg):
        roues['t'].append(msg.stamp.to_sec())
        roues['v'].append(list(msg.velocity))

    rospy.init_node('export_matlab', anonymous=True)
    for cle, topic in ESTIMATEURS.items():
        rospy.Subscriber(topic, Odometry,
                         (lambda c: lambda m: cb_odom(m, c))(cle), queue_size=200)
    for cle, topic in FEATURES.items():
        rospy.Subscriber(topic, PointCloud2,
                         (lambda c: lambda m: cb_feat(m, c))(cle), queue_size=100)
    rospy.Subscriber(ROUES, WheelStates, cb_roues, queue_size=100)

    print('Capture de %.0f s...' % duree)
    t0 = rospy.get_time()
    prochain = 10.0
    while rospy.get_time() - t0 < duree and not rospy.is_shutdown():
        rospy.sleep(0.5)
        ecoule = rospy.get_time() - t0
        if ecoule >= prochain:
            print('  %3.0f s | %s' % (ecoule, ' '.join(
                '%s:%d' % (k, len(donnees[k]['t'])) for k in ESTIMATEURS)))
            prochain += 10.0
    return donnees, feats, roues


def fige(d):
    """Instantané COHÉRENT d'un dict de listes alimentées par des callbacks.

    Les abonnements ROS continuent d'écrire pendant l'écriture des fichiers :
    lire `t` puis `P_pose` peut donner deux longueurs différentes (constaté
    en direct : 3413 contre 3412, np.hstack refuse). On tronque donc tout à
    la longueur du plus court AU MOMENT de l'appel. Les quelques
    échantillons écartés sont ceux arrivés après la fin de la fenêtre
    demandée — les perdre est correct, mélanger des lignes désalignées ne
    l'aurait pas été : la ligne k de P_pose ne correspondrait plus à
    l'instant t[k], et toute analyse de covariance en aval serait fausse
    sans que rien ne le signale.
    """
    n = min((len(v) for v in d.values() if isinstance(v, list)), default=0)
    return {k: (v[:n] if isinstance(v, list) else v) for k, v in d.items()}


def ecrit(donnees, feats, roues, nom, csv_seulement):
    import numpy as np
    # Figer AVANT toute lecture, et une seule fois.
    donnees = {k: fige(v) for k, v in donnees.items()}
    feats = {k: fige(v) for k, v in feats.items()}
    roues = fige(roues)
    os.makedirs(DEST, exist_ok=True)
    horo = time.strftime('%Y%m%d_%H%M%S')
    base = os.path.join(DEST, '%s_%s' % (nom, horo))
    ecrits = []

    paquet = {}
    for cle, d in donnees.items():
        if not d['t']:
            print('  %-9s AUCUNE donnée — estimateur muet, champ omis' % cle)
            continue
        P = np.asarray(d['P_pose'], dtype=float)
        # Diagonale de la 6x6 aplatie : indices 0, 7, 14 = var(x), var(y), var(z).
        sigma3 = 3.0 * np.sqrt(np.abs(P[:, [0, 7, 14]]))
        paquet[nom_matlab(cle)] = {
            't': np.asarray(d['t'], dtype=float),
            'p': np.asarray(d['p'], dtype=float),
            'q': np.asarray(d['q'], dtype=float),
            'v': np.asarray(d['v'], dtype=float),
            'w': np.asarray(d['w'], dtype=float),
            'P_pose': P,
            'P_twist': np.asarray(d['P_twist'], dtype=float),
            'sigma3': sigma3,
            'topic': ESTIMATEURS[cle],
        }
        print('  %-9s %6d échantillons' % (cle, len(d['t'])))

    for cle, f in feats.items():
        k = nom_matlab(cle)
        if f['t'] and k in paquet:
            paquet[k]['feat_t'] = np.asarray(f['t'], dtype=float)
            paquet[k]['feat_n'] = np.asarray(f['n'], dtype=float)
            print('  %-9s %6d mises à jour de features (moyenne %.2f)'
                  % (cle, len(f['n']), float(np.mean(f['n']))))

    paquet['roues'] = {
        't': np.asarray(roues['t'], dtype=float),
        'v': np.asarray(roues['v'], dtype=float) if roues['v'] else np.zeros((0, 4)),
        'topic': ROUES,
    }
    paquet['meta'] = {
        'genere': horo,
        'duree_s': float(max((d['t'][-1] - d['t'][0]) for d in donnees.values()
                             if len(d['t']) > 1) if any(len(d['t']) > 1
                                                        for d in donnees.values()) else 0.0),
        'note_covariance': ('P_pose et P_twist sont des 6x6 APLATIES en 36 colonnes, '
                            'ordre ligne-major ROS. Sous MATLAB : '
                            "P = reshape(P_pose(k,:), 6, 6)'  -- la transposée "
                            'est OBLIGATOIRE (MATLAB lit en colonne-major).'),
        'note_temps': ('t est l horodatage CAPTEUR (header.stamp), pas l heure '
                       'd arrivée : seul le temps capteur entre dans les filtres.'),
    }

    if not csv_seulement:
        try:
            from scipy.io import savemat
            savemat(base + '.mat', paquet, do_compression=True)
            ecrits.append(base + '.mat')
        except ImportError:
            print('  scipy absent -> pas de .mat, CSV seulement')

    # CSV : un fichier par estimateur. Colonnes nommées, en-tête unique ligne,
    # directement lisible par readtable() sans option.
    for cle, d in donnees.items():
        if not d['t']:
            continue
        import numpy as np
        chemin = '%s_%s.csv' % (base, nom_matlab(cle))
        P = np.asarray(d['P_pose'], dtype=float)
        Pt = np.asarray(d['P_twist'], dtype=float)
        cols = (['t', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw',
                 'vx', 'vy', 'vz', 'wx', 'wy', 'wz']
                + ['Ppose_%d' % i for i in range(36)]
                + ['Ptwist_%d' % i for i in range(36)]
                + ['sigma3x', 'sigma3y', 'sigma3z'])
        s3 = 3.0 * np.sqrt(np.abs(P[:, [0, 7, 14]]))
        M = np.hstack([np.asarray(d['t'])[:, None], np.asarray(d['p']),
                       np.asarray(d['q']), np.asarray(d['v']),
                       np.asarray(d['w']), P, Pt, s3])
        np.savetxt(chemin, M, delimiter=',', header=','.join(cols),
                   comments='', fmt='%.9g')
        ecrits.append(chemin)

    return ecrits


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    duree = float(args[0]) if args else 60.0
    nom = args[1] if len(args) > 1 else 'compare3'
    csv_seulement = '--csv-seulement' in sys.argv

    donnees, feats, roues = collecte(duree)
    if not any(d['t'] for d in donnees.values()):
        print('ÉCHEC : aucun estimateur n a publié. Pile lancée ?')
        return 1
    print('Écriture...')
    for f in ecrit(donnees, feats, roues, nom, csv_seulement):
        print('  écrit : %s' % f)
    print("\nSous MATLAB :  S = load('<fichier>.mat');")
    print("               P = reshape(S.MINS.P_pose(1,:), 6, 6)';  % transposée !")
    return 0


if __name__ == '__main__':
    sys.exit(main())

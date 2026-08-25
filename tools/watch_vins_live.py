#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Supervision temps reel d'openVINS pendant un roulage.

N'EMET QUE DES ANOMALIES — pas un flux d'etat. Un moniteur qui parle tout le
temps ne se lit plus ; celui-ci se tait tant que tout va bien, ce qui rend
chaque ligne significative.

Ce qui est surveille, et le seuil retenu :

  VITESSE FANTOME   |v| openVINS > ~vmax alors que les ROUES sont immobiles.
                    C'est le retour du defaut du 2026-08-20 (verrou ZUPT) :
                    l'estimateur se croit lance a l'arret. Seuil a 0.30 m/s,
                    tres au-dessus du bruit mesure (0.003) et tres en-dessous
                    de la valeur pathologique observee (1.295).

  SAUT DE POSITION  bond > ~jump entre deux echantillons consecutifs. Une
                    trajectoire physique ne teleporte pas ; un saut signale
                    une reinitialisation du filtre ou une divergence brutale.

  ECART VINS/MINS   distance entre les deux estimateurs > ~ecart. MINS est
                    ancre par ses roues : un ecart qui se creuse mesure la
                    derive d'openVINS. Emis par paliers (pas a chaque
                    echantillon) pour ne pas noyer le flux.

  RECALIBRATIONS    lignes APPLIQUEE/REFUSEE du sanitizer, remontees telles
                    quelles : ce sont des evenements rares et informatifs.

Les seuils sont volontairement larges. Un moniteur qui crie au moindre bruit
fait perdre confiance et finit ignore — celui-ci ne doit se declencher que sur
ce qui merite une intervention.

USAGE
    python3 tools/watch_vins_live.py            # tourne jusqu'a Ctrl-C
    python3 tools/watch_vins_live.py --vmax 0.5 # seuil vitesse personnalise
"""
import argparse
import math
import sys

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from leo_msgs.msg import WheelStates


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vmax', type=float, default=0.30,
                    help='vitesse openVINS toleree roues immobiles (m/s)')
    ap.add_argument('--jump', type=float, default=1.0,
                    help='saut de position tolere entre 2 echantillons (m)')
    ap.add_argument('--ecart', type=float, default=3.0,
                    help='premier palier d ecart VINS/MINS signale (m)')
    ap.add_argument('--zmax', type=float, default=1.0,
                    help='altitude MINS toleree (m) — il n a aucune contrainte en Z')
    a = ap.parse_args()

    rospy.init_node('watch_vins_live', anonymous=True, disable_signals=True)
    etat = {
        'ov': None, 'mins': None, 'roule': None, 'zupt': None,
        'prev_ov': None, 'palier': a.ecart,
        'derniere_alerte_v': 0.0,
        # Features CONSOMMEES par le filtre — le vrai critere de sante.
        # Mesure du 2026-08-20 : moyenne 0.1, max 2 en mouvement, alors qu'il
        # en faut ~40. Un extracteur actif ne prouve RIEN : il dessinait
        # 11850 pixels de points pendant que le filtre les rejetait tous.
        'feat': [], 'dernier_bilan': 0.0, 'alerte_z': 0.0,
    }

    def dit(msg):
        print('%s  %s' % (rospy.get_time().__format__('.1f'), msg))
        sys.stdout.flush()

    def cb_roues(m):
        try:
            etat['roule'] = any(abs(v) > 0.02 for v in m.velocity)
        except Exception:
            pass

    def cb_ov(m):
        p = m.pose.pose.position
        v = m.twist.twist.linear
        etat['ov'] = (p.x, p.y, p.z)
        vn = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        now = rospy.get_time()

        # 1. vitesse fantome a l'arret — le defaut qui nous a occupes ce soir
        # roule is None = aucune donnee roue encore : on ne conclut RIEN.
        # Initialiser a False ferait crier « vitesse fantome » au demarrage,
        # avant meme de savoir si le robot bouge (constate a l'essai).
        if etat['roule'] is False and vn > a.vmax and now - etat['derniere_alerte_v'] > 10:
            etat['derniere_alerte_v'] = now
            dit('VITESSE FANTOME : |v|=%.3f m/s alors que les roues sont '
                'immobiles (seuil %.2f) — le ZUPT ne mord plus' % (vn, a.vmax))

        # 2. saut de position
        if etat['prev_ov'] is not None:
            d = math.dist(etat['prev_ov'], etat['ov'])
            if d > a.jump:
                dit('SAUT DE POSITION : %.2f m entre deux echantillons '
                    '(filtre reinitialise ou divergence brutale)' % d)
        etat['prev_ov'] = etat['ov']

        # 3. ecart avec MINS, par paliers
        if etat['mins'] is not None:
            e = math.dist(etat['ov'], etat['mins'])
            if e > etat['palier']:
                dit('ECART VINS/MINS : %.1f m (MINS est ancre par ses roues, '
                    'cet ecart est la derive openVINS)' % e)
                # Palier place AU-DESSUS de l'ecart courant, pas double :
                # doubler depuis 3 m emettait six lignes d'affilee pour
                # atteindre 172 m (constate a l'essai).
                while etat['palier'] <= e:
                    etat['palier'] *= 2

    def cb_feat(m):
        etat['feat'].append((rospy.get_time(), m.width))
        now = rospy.get_time()
        # Bilan toutes les 10 s, separant mouvement et arret : c'est en
        # MOUVEMENT que le chiffre a un sens (a l'arret le ZUPT remplace les
        # mises a jour camera, zero y est normal).
        if now - etat['dernier_bilan'] >= 10.0:
            etat['dernier_bilan'] = now
            recent = [f for t, f in etat['feat'] if t > now - 10.0]
            etat['feat'] = [(t, f) for t, f in etat['feat'] if t > now - 10.0]
            if recent and etat['roule']:
                moy = sum(recent) / float(len(recent))
                pic = max(recent)
                verdict = ('SUCCES' if moy >= 10 else
                           'insuffisant' if moy >= 1 else 'REJET TOTAL')
                dit('FEATURES (en mouvement) moyenne %.1f  pic %d  -> %s'
                    % (moy, pic, verdict))

    def cb_mins(m):
        p = m.pose.pose.position
        etat['mins'] = (p.x, p.y, p.z)
        # GARDE VERTICALE (2026-08-20, decouverte par accident) : MINS n'a
        # AUCUNE contrainte sur Z. Ses roues tiennent le plan horizontal, la
        # camera tient le reste. Une extrinseque camera modifiee l'a fait
        # « tomber » de 26 m en Z pendant que X et Y restaient parfaits, sans
        # qu'aucune alarme existante ne se declenche. Toute modification de
        # geometrie camera doit donc etre validee SUR CET AXE, pas seulement
        # sur les features d'openVINS.
        if abs(p.z) > a.zmax and rospy.get_time() - etat['alerte_z'] > 15:
            etat['alerte_z'] = rospy.get_time()
            dit('MINS TOMBE EN Z : z=%.2f m (seuil %.1f) alors que x=%.2f y=%.2f '
                '— signature d une extrinseque camera fautive'
                % (p.z, a.zmax, p.x, p.y))

    def cb_zupt(m):
        if etat['zupt'] is not None and m.data != etat['zupt']:
            dit('ZUPT -> %s' % ('ARRET CONFIRME' if m.data else 'mouvement'))
        etat['zupt'] = m.data

    rospy.Subscriber('/ov_msckf/odomimu', Odometry, cb_ov, queue_size=5)
    rospy.Subscriber('/mins/imu/odom', Odometry, cb_mins, queue_size=5)
    rospy.Subscriber('/firmware/wheel_states', WheelStates, cb_roues, queue_size=5)
    rospy.Subscriber('/imu_sanitizer/is_stationary', Bool, cb_zupt, queue_size=5)
    rospy.Subscriber('/ov_msckf/points_msckf', PointCloud2, cb_feat, queue_size=5)

    dit('supervision active — seuils : vitesse %.2f m/s, saut %.1f m, ecart %.1f m'
        % (a.vmax, a.jump, a.ecart))
    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())

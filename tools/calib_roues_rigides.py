#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration PASSIVE des roues rigides (rayon effectif r, entraxe effectif b)
=============================================================================
Les intrinsèques MINS ([0.0625, 0.0625, 0.358]) sont celles des roues D'ORIGINE
(rosparam /firmware/diff_drive) ; les roues rigides ont un autre rayon et la
calibration en ligne de MINS a divergé le 2026-07-06 (rayons négatifs) — elle
est gelée. Ce script MESURE donc r et b à partir des données, sans rien
commander : lancez-le, conduisez en MANUEL, il imprime les estimations.

Protocole (2 minutes) :
  1. lancer :  python3 tools/calib_roues_rigides.py
  2. AVANCER DROIT ~3 m (cockpit, vitesse constante)      -> estime r
  3. TOURNER SUR PLACE ~2 tours complets (même sens)      -> estime b
  4. Ctrl-C : le script imprime r_gauche, r_droit, b et la ligne
     'intrinsics:' à coller dans config_wheel.yaml.

Méthode :
  r  : segments où |gyro_z| < 0.05 rad/s et |ω roues| > 1 rad/s (ligne droite) —
       r = déplacement_MINS / angle_roue_intégré (moyenne des 2 côtés, robuste
       aux patinages ponctuels par médiane sur fenêtres de 1 s).
  b  : segments en rotation pure (|gyro_z| > 0.2 rad/s, |v_lin| < 0.05 m/s) —
       le modèle différentiel donne yaw_rate = r*(ω_d - ω_g)/b, donc
       b = r*(ω_d - ω_g)/gyro_z (médiane des fenêtres).
  Le déplacement vient de /mins/imu/odom (à défaut /robot_pose_fused) : sur
  3 m la dérive VIO est négligeable devant l'incertitude visée (~1 %).
"""
import math, signal, sys, time
from collections import deque

import numpy as np
import rospy
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry

R_NOM = 0.0625   # rayon stock (référence d'affichage)
B_NOM = 0.358    # entraxe stock

samples = deque()   # (t, w_l, w_r)  vitesses angulaires roues (moyennes/côté)
gyro    = deque()   # (t, gz)
poses   = deque()   # (t, x, y)

def on_joints(m):
    try:
        i_fl = m.name.index("wheel_FL_joint"); i_fr = m.name.index("wheel_FR_joint")
        try: i_rl = m.name.index("wheel_RL_joint"); w_l = 0.5*(m.velocity[i_fl]+m.velocity[i_rl])
        except ValueError: w_l = m.velocity[i_fl]
        try: i_rr = m.name.index("wheel_RR_joint"); w_r = 0.5*(m.velocity[i_fr]+m.velocity[i_rr])
        except ValueError: w_r = m.velocity[i_fr]
        samples.append((m.header.stamp.to_sec() or time.time(), w_l, w_r))
    except (ValueError, IndexError):
        pass

def on_imu(m):
    gyro.append((m.header.stamp.to_sec(), m.angular_velocity.z))

def on_odom(m):
    poses.append((m.header.stamp.to_sec(),
                  m.pose.pose.position.x, m.pose.pose.position.y))

def interp(dq, t):
    arr = list(dq)
    for i in range(1, len(arr)):
        if arr[i][0] >= t:
            t0, *v0 = arr[i-1]; t1, *v1 = arr[i]
            if t1 == t0: return v0
            a = (t - t0) / (t1 - t0)
            return [x0 + a*(x1-x0) for x0, x1 in zip(v0, v1)]
    return None

def analyse(*_):
    print("\n=== ANALYSE ===")
    S = list(samples)
    if len(S) < 50:
        print("pas assez d'échantillons roues (%d)" % len(S)); sys.exit(1)
    r_est, b_est = [], []
    t_start, t_end = S[0][0], S[-1][0]
    t = t_start
    while t + 1.0 <= t_end:               # fenêtres de 1 s
        win = [s for s in S if t <= s[0] < t + 1.0]
        g   = [x for x in gyro if t <= x[0] < t + 1.0]
        t += 1.0
        if len(win) < 5 or len(g) < 5:
            continue
        wl = np.mean([w[1] for w in win]); wr = np.mean([w[2] for w in win])
        gz = np.mean([x[1] for x in g])
        p0 = interp(poses, win[0][0]); p1 = interp(poses, win[-1][0])
        if p0 is None or p1 is None:
            continue
        disp = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
        dt   = win[-1][0] - win[0][0]
        # ligne droite : roues rapides, gyro calme -> r = disp / (|w_moy|*dt)
        if abs(gz) < 0.05 and min(abs(wl), abs(wr)) > 1.0 and disp > 0.05:
            r_est.append(disp / (0.5*(abs(wl)+abs(wr)) * dt))
        # rotation pure : gyro net, déplacement faible -> b = r*(wr-wl)/gz
        if abs(gz) > 0.2 and disp < 0.05 * dt * 10 and abs(wr - wl) > 1.0:
            r_ref = np.median(r_est) if r_est else R_NOM
            b_est.append(r_ref * (wr - wl) / gz)
    print("fenêtres droites : %d   fenêtres rotation : %d" % (len(r_est), len(b_est)))
    if r_est:
        r = float(np.median(r_est))
        print("r effectif  = %.4f m   (stock %.4f, écart %+.1f %%)" % (r, R_NOM, 100*(r/R_NOM-1)))
    else:
        r = R_NOM; print("r : PAS de segment droit exploitable — refaire l'étape 2")
    if b_est:
        b = float(abs(np.median(b_est)))
        print("b effectif  = %.4f m   (stock %.4f, écart %+.1f %%)" % (b, B_NOM, 100*(b/B_NOM-1)))
    else:
        b = B_NOM; print("b : PAS de segment rotation exploitable — refaire l'étape 3")
    if r_est or b_est:
        print("\nà coller dans config_wheel.yaml :")
        print("  intrinsics: [%.4f, %.4f, %.4f]" % (r, r, b))
        print("(puis restart de la stack — et garder la mesure roll-out physique")
        print(" comme contre-vérification, protocole §roues du rapport)")
    sys.exit(0)

rospy.init_node("calib_roues", anonymous=True)
rospy.Subscriber("/joint_states", JointState, on_joints, queue_size=50)
rospy.Subscriber("/imu/data_clean", Imu, on_imu, queue_size=200)
rospy.Subscriber("/mins/imu/odom", Odometry, on_odom, queue_size=50)
signal.signal(signal.SIGINT, analyse)
print("Enregistrement… conduisez : ~3 m tout droit, puis ~2 tours sur place.")
print("Ctrl-C pour analyser.")
rospy.spin()

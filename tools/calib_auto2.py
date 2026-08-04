#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration automatisée v2 (leçons du 1er essai, 2026-07-10) :
  - LIGNE DROITE D'ABORD (r depuis le déplacement MINS, filtre frais/sain) ;
  - b par ARCS DOUX (lin 0.15 + ang 0.4) : le pivot sur place demande une
    friction skid-steer que ce sol ne fournit pas (0.04 rad/s réels pour 0.5
    commandés au 1er essai) ; b = r·(ω_d−ω_g)/gyro_z n'a pas besoin de MINS ;
  - intégration gyro par pointeur (plus de fenêtre glissante qui perd des
    échantillons) ; taux réels journalisés à chaque phase ;
  - garde de santé MINS (|pose| < 50 m) pour les fenêtres r.
"""
import json, math, signal, sys, time
from collections import deque

import numpy as np
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState, Imu, Image
from nav_msgs.msg import Odometry

R_NOM, B_NOM = 0.0625, 0.358
ROI = (0.20, 0.80, 0.333, 0.70)

wheels, gyro, poses = deque(), deque(), deque()
depth_min_mm = [8000]
front_free_m = [0.0]

def on_joints(m):
    try:
        v = {n: m.velocity[i] for i, n in enumerate(m.name)}
        wl = 0.5*(v["wheel_FL_joint"] + v.get("wheel_RL_joint", v["wheel_FL_joint"]))
        wr = 0.5*(v["wheel_FR_joint"] + v.get("wheel_RR_joint", v["wheel_FR_joint"]))
        wheels.append((m.header.stamp.to_sec() or time.time(), wl, wr))
    except (KeyError, IndexError):
        pass

def on_imu(m):
    gyro.append((m.header.stamp.to_sec(), m.angular_velocity.z))

def on_odom(m):
    poses.append((m.header.stamp.to_sec(),
                  m.pose.pose.position.x, m.pose.pose.position.y))

def on_depth(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    x0, x1 = int(msg.width*ROI[0]), int(msg.width*ROI[1])
    y0, y1 = int(msg.height*ROI[2]), int(msg.height*ROI[3])
    roi = arr[y0:y1, x0:x1]
    w3 = max(1, roi.shape[1]//3)
    p5s = []
    for k in range(3):
        sub = roi[:, k*w3:(k+1)*w3]
        v = sub[(sub > 100) & (sub < 8000)]
        if len(v) > 20:
            p5s.append(float(np.percentile(v, 5)))
    depth_min_mm[0] = min(p5s) if p5s else 8000
    mid = roi[:, w3:2*w3]
    v = mid[(mid > 100) & (mid < 8000)]
    front_free_m[0] = float(np.median(v))/1000.0 if len(v) > 30 else 0.0

rospy.init_node("calib_auto2")
cmd_pub = rospy.Publisher("/mission/command", String, queue_size=5)
rospy.Subscriber("/joint_states", JointState, on_joints, queue_size=50)
rospy.Subscriber("/imu/data_clean", Imu, on_imu, queue_size=200)
rospy.Subscriber("/mins/imu/odom", Odometry, on_odom, queue_size=50)
rospy.Subscriber("/pc/camera/depth/image_rect_raw", Image, on_depth, queue_size=1)

def send(lin, ang):
    cmd_pub.publish(String(data=json.dumps({"action": "manual", "lin": lin, "ang": ang})))

def stop_robot():
    for _ in range(8):
        send(0.0, 0.0); time.sleep(0.05)

def bail(sig=None, _=None):
    stop_robot(); print("\nARRÊT (signal)"); sys.exit(1)
signal.signal(signal.SIGINT, bail)

def mins_sane():
    if not poses: return False
    _, x, y = poses[-1]
    return abs(x) < 50 and abs(y) < 50

def drive(lin, ang, dur, guard=None, label=""):
    t0 = time.time(); n0w, n0g = len(wheels), len(gyro)
    aborted = ""
    while time.time() - t0 < dur and not rospy.is_shutdown():
        if guard is not None:
            g = guard()
            if g:
                aborted = g; break
        send(lin, ang); time.sleep(0.2)
    stop_robot()
    dt = time.time() - t0
    W = [w for w in list(wheels) if w[0] >= t0]
    G = [g for g in list(gyro) if g[0] >= t0]
    mwl = np.mean([w[1] for w in W]) if W else 0
    mwr = np.mean([w[2] for w in W]) if W else 0
    mgz = np.mean([g[1] for g in G]) if G else 0
    print("  [%s] %.1fs%s | roues G=%.2f D=%.2f rad/s | gyro=%.3f rad/s | MINS %s"
          % (label, dt, (" ABORT:"+aborted) if aborted else "",
             mwl, mwr, mgz, "sain" if mins_sane() else "DIVERGÉ"))
    return t0, time.time(), aborted

print("=== PRÉ-VOL ===")
time.sleep(4)
try:
    b0 = rospy.wait_for_message("/firmware/battery", Float32, timeout=10)
    print("batterie %.2f V" % b0.data)
except Exception:
    print("batterie muette — abandon"); sys.exit(1)
if not poses or not mins_sane():
    print("MINS absent/divergé — abandon"); sys.exit(1)
print("MINS sain, depth couloir %.2f m, centre libre %.2f m, roues %d msgs"
      % (depth_min_mm[0]/1000.0, front_free_m[0], len(wheels)))

# ── Orientation vers l'espace libre (arcs courts si nécessaire) ─────────────
tries = 0
while front_free_m[0] < 3.0 and tries < 3:
    tries += 1
    print("devant %.2f m < 3 m — arc de dégagement %d/3" % (front_free_m[0], tries))
    drive(0.12, 0.5, 6.0, lambda: "obst" if depth_min_mm[0] < 600 else "", "dégagement")
    time.sleep(1.5)
if front_free_m[0] < 3.0:
    print("pas de dégagement trouvé (%.2f m) — orientez le robot, relancez" % front_free_m[0])
    sys.exit(1)

# ── Phase 1 : LIGNE DROITE (r) — MINS frais ─────────────────────────────────
print("\n=== PHASE 1 : ligne droite (r) ===")
t_s0, t_s1, ab = drive(0.2, 0.0, 11.0,
                       lambda: "obst" if depth_min_mm[0] < 800 else
                               ("mins" if not mins_sane() else ""), "droite")
time.sleep(1.5)

# ── Phases 2-3 : ARCS (b) — gauche puis droite ──────────────────────────────
print("\n=== PHASES 2-3 : arcs doux (b) ===")
t_a = []
for sgn, lab in ((1.0, "arc gauche"), (-1.0, "arc droit")):
    a0, a1, ab2 = drive(0.15, sgn*0.4, 8.0,
                        lambda: "obst" if depth_min_mm[0] < 700 else "", lab)
    t_a.append((a0, a1))
    time.sleep(1.5)
stop_robot()

# ── Analyse ──────────────────────────────────────────────────────────────────
print("\n=== ANALYSE ===")
W, G, P = list(wheels), list(gyro), list(poses)
def wins(t0, t1, width=0.5):
    t = t0
    while t + width <= t1:
        yield t, t + width
        t += width
def mean_in(seq, t0, t1, idx, need=3):
    v = [s[idx] for s in seq if t0 <= s[0] < t1]
    return np.mean(v) if len(v) >= need else None
def pose_at(t):
    for i in range(1, len(P)):
        if P[i][0] >= t:
            ta, xa, ya = P[i-1]; tb, xb, yb = P[i]
            if tb == ta: return xa, ya
            k = (t-ta)/(tb-ta)
            return xa+k*(xb-xa), ya+k*(yb-ya)
    return None

r_est = []
for a, b in wins(t_s0+1.0, t_s1-0.3):
    wl, wr = mean_in(W, a, b, 1), mean_in(W, a, b, 2)
    gz = mean_in(G, a, b, 1, need=10)
    pa, pb = pose_at(a), pose_at(b)
    if None in (wl, wr, gz) or pa is None or pb is None: continue
    if abs(gz) > 0.06 or min(abs(wl), abs(wr)) < 1.0: continue
    if max(abs(pa[0]), abs(pa[1]), abs(pb[0]), abs(pb[1])) > 50: continue
    disp = math.hypot(pb[0]-pa[0], pb[1]-pa[1])
    r_est.append(disp / (0.5*(abs(wl)+abs(wr)) * (b-a)))
r = float(np.median(r_est)) if len(r_est) >= 4 else None

b_est = []
r_ref = r if r else R_NOM
for (a0, a1) in t_a:
    for a, b in wins(a0+1.0, a1-0.3):
        wl, wr = mean_in(W, a, b, 1), mean_in(W, a, b, 2)
        gz = mean_in(G, a, b, 1, need=10)
        if None in (wl, wr, gz) or abs(gz) < 0.12 or abs(wr-wl) < 0.8: continue
        b_est.append(r_ref * (wr - wl) / gz)
bb = float(abs(np.median(b_est))) if len(b_est) >= 4 else None

print("fenêtres : droites=%d arcs=%d" % (len(r_est), len(b_est)))
if r: print("r = %.4f m (stock %.4f, %+.1f %%)  IQR ±%.4f" % (
        r, R_NOM, 100*(r/R_NOM-1), float(np.subtract(*np.percentile(r_est,[75,25])))/2))
else: print("r : non mesuré (%d fenêtres)" % len(r_est))
if bb: print("b = %.4f m (stock %.4f, %+.1f %%)  IQR ±%.4f" % (
        bb, B_NOM, 100*(bb/B_NOM-1), float(np.subtract(*np.percentile(b_est,[75,25])))/2))
else: print("b : non mesuré (%d fenêtres)" % len(b_est))
print("MINS final : %s" % ("sain" if mins_sane() else "DIVERGÉ"))
print("RESULT_JSON " + json.dumps({"r": r, "b": bb,
    "n_r": len(r_est), "n_b": len(b_est), "mins_sane": mins_sane()}))

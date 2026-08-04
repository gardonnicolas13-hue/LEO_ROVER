#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test en rampe de l'autorité de lacet MECANUM : commande des pivots purs à
plusieurs vitesses et mesure le taux de lacet RÉEL (cap MINS). Sert à situer
le seuil de décrochage des rouleaux (mesuré grossièrement le 2026-07-10 :
cmd 0.13 rad/s -> 0.14 réel = accroche ; cmd 0.4-0.5 -> ~0 = décroche).
Les constantes FSM (TURN_SPEED & co, 0.18) sont à recaler sur le genou de
cette courbe. À lancer robot allumé, stack saine, ~1 m d'espace autour.

  python3 tools/ramp_yaw.py            # rampe 0.10 -> 0.40 rad/s
"""
import json, math, signal, sys, time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

CMDS = (0.10, 0.15, 0.20, 0.25, 0.30, 0.40)
HOLD_S = 8.0

yaws = []
def on_od(m):
    q = m.pose.pose.orientation
    yaws.append((time.time(),
                 math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))

rospy.init_node("ramp_yaw", anonymous=True)
rospy.Subscriber("/mins/imu/odom", Odometry, on_od, queue_size=50)
pub = rospy.Publisher("/mission/command", String, queue_size=5)

def send(lin, ang):
    pub.publish(String(data=json.dumps({"action": "manual", "lin": lin, "ang": ang})))
def stop():
    for _ in range(8):
        send(0.0, 0.0); time.sleep(0.05)
signal.signal(signal.SIGINT, lambda *_: (stop(), sys.exit(1)))

time.sleep(4)
if not yaws:
    print("MINS muet — abandon"); sys.exit(1)

def yaw_rate(t0, t1):
    pts = [(t, y) for t, y in yaws if t0 <= t <= t1]
    if len(pts) < 10: return None
    acc = 0.0
    for i in range(1, len(pts)):
        d = pts[i][1] - pts[i-1][1]
        acc += math.atan2(math.sin(d), math.cos(d))
    return acc / (pts[-1][0] - pts[0][0])

print("cmd (rad/s) | réel (rad/s) | autorité")
results = {}
for cmd in CMDS:
    t0 = time.time() + 1.5      # fenêtre après établissement
    te = time.time() + HOLD_S
    while time.time() < te:
        send(0.0, cmd); time.sleep(0.2)
    stop()
    r = yaw_rate(t0, te)
    results[cmd] = r
    print("   %.2f     |    %s    |  %s" % (
        cmd, ("%+.3f" % r) if r is not None else "  ?  ",
        ("%.0f %%" % (100*r/cmd)) if r is not None else "?"), flush=True)
    time.sleep(2)
print("\nRecaler TURN_SPEED/UTURN_SPEED/LOCK_ALIGN_SPEED/SEARCH_ANG/SPIRAL_ANG0")
print("sur la plus haute commande gardant >= 70 %% d'autorité.")
print("RESULT_JSON " + json.dumps({str(k): v for k, v in results.items()}))

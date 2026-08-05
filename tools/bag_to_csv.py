#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_to_csv.py — extrait les poses d'un rosbag (enregistré par
record_trajectories.sh) vers des CSV lisibles par plot_trajectories.m
(ou n'importe quel outil : Matlab, Python, tableur).

    python3 tools/bag_to_csv.py data/trajectories/traj_20260727_141530.bag
    python3 tools/bag_to_csv.py mon_essai.bag  --out /chemin/sortie/

Produit, à côté du bag (ou dans --out), un CSV par source :
    <base>_mins.csv       colonnes : t,x,y,z,qx,qy,qz,qw,yaw_rad
    <base>_vins.csv       (idem)
    <base>_carolus.csv    (idem — fix global balise, si présent)
    <base>_fused.csv      (source servie au robot)
    <base>_source.csv     colonnes : t,source     ("VINS"|"MINS" au fil du temps)

t est en SECONDES depuis le PREMIER message du bag (origine commune à toutes
les traces, pour un alignement temporel trivial côté Matlab).

N'exige PAS la ROS Toolbox de Matlab : le CSV est le format pivot. Le .m sait
aussi lire le bag directement si la toolbox est là — le CSV reste le chemin
recommandé (portable, inspectable).
"""
import argparse
import csv
import math
import os
import sys

try:
    import rosbag
except ImportError:
    sys.stderr.write(
        "erreur : 'rosbag' introuvable — source d'abord l'environnement ROS :\n"
        "  source /opt/ros/noetic/setup.bash\n")
    sys.exit(1)

# topic -> (suffixe de fichier, type de message : "odom" | "pose" | "source")
TOPIC_MAP = {
    "/mins/imu/odom":              ("mins",    "odom"),
    "/ov_msckf/odomimu":           ("vins",    "odom"),
    "/robot_pose_fused":           ("fused",   "odom"),
    # /pose = sortie REELLE de carolus_astrobee (carolus_astrobee.cpp:362,
    # geometry_msgs/PoseStamped). C'est aussi le topic nomme explicitement par
    # le superviseur dans ses exigences.
    "/pose":                       ("carolus", "pose"),
    # Conserve pour les bags ANTERIEURS : carolus_vicon_bridge devait alimenter
    # ce topic depuis un TF `beacon_link`, mais RIEN ne publie ce TF (le
    # composant tf_bridge.py que sa documentation attend est absent de
    # l'espace de travail). Le topic est donc toujours reste vide — d'ou
    # l'absence de tout _carolus.csv jusqu'au 29/07. Deux entrees peuvent
    # pointer sur "carolus" sans conflit : l'une des deux est toujours vide,
    # et un bag ne contient jamais les deux peuples.
    "/mins/external_ref/carolus":  ("carolus", "pose"),
    "/leo_navigation/pose_source": ("source",  "source"),
    # AprilTag posés aux coins du rectangle (2026-08-05). Enregistrés comme
    # OBSERVATION seule : rien n'est recalé dessus, sinon la trajectoire
    # épouserait le rectangle par construction et ne mesurerait plus rien.
    # Un message porte un TABLEAU de détections (souvent vide) : on écrit une
    # ligne PAR détection, d'où un format propre (t, id, x, y, z, distance)
    # différent des CSV de pose à 11 colonnes.
    "/tag_detections":             ("tags",    "tags"),
}


def _rpy_from_quat(qx, qy, qz, qw):
    """Roll/Pitch/Yaw (rad) depuis un quaternion — convention ZYX standard."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser(description="rosbag de poses -> CSV")
    ap.add_argument("bag", help="chemin du .bag")
    ap.add_argument("--out", default=None,
                    help="dossier de sortie (défaut : à côté du bag)")
    args = ap.parse_args()

    if not os.path.isfile(args.bag):
        sys.stderr.write("erreur : bag introuvable : %s\n" % args.bag)
        sys.exit(1)

    base = os.path.splitext(os.path.basename(args.bag))[0]
    outdir = args.out or os.path.dirname(os.path.abspath(args.bag))
    os.makedirs(outdir, exist_ok=True)

    # Un writer CSV par topic présent, ouvert paresseusement.
    writers, files, counts = {}, {}, {}
    t0 = None

    ODOM_HDR = ["t", "x", "y", "z", "qx", "qy", "qz", "qw",
                "roll_rad", "pitch_rad", "yaw_rad"]

    def _writer_for(topic):
        if topic in writers:
            return writers[topic]
        suffix, kind = TOPIC_MAP[topic]
        path = os.path.join(outdir, "%s_%s.csv" % (base, suffix))
        f = open(path, "w", newline="")
        w = csv.writer(f)
        if kind == "source":
            w.writerow(["t", "source"])
        elif kind == "tags":
            w.writerow(["t", "id", "x", "y", "z", "distance_m"])
        else:
            w.writerow(ODOM_HDR)
        writers[topic] = (w, kind)
        files[topic] = (f, path)
        counts[topic] = 0
        return writers[topic]

    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=list(TOPIC_MAP)):
            ts = t.to_sec()
            if t0 is None:
                t0 = ts
            rel = ts - t0
            w, kind = _writer_for(topic)

            if kind == "source":
                w.writerow(["%.6f" % rel, msg.data])
            elif kind == "tags":
                # Un message = un TABLEAU (souvent vide quand aucun tag n'est
                # dans le champ). On n'écrit que les détections réelles, une
                # ligne chacune ; les messages vides ne produisent rien, donc
                # le CSV ne contient que les instants où un coin a été VU.
                for det in msg.detections:
                    p = det.pose.pose.pose.position
                    tid = det.id[0] if len(det.id) else -1
                    dist = (p.x * p.x + p.y * p.y + p.z * p.z) ** 0.5
                    w.writerow(["%.6f" % rel, tid,
                                "%.6f" % p.x, "%.6f" % p.y, "%.6f" % p.z,
                                "%.6f" % dist])
                    counts[topic] += 1
                continue          # compteur déjà incrémenté par détection
            else:
                if kind == "odom":
                    p = msg.pose.pose.position
                    q = msg.pose.pose.orientation
                else:  # "pose" : geometry_msgs/PoseStamped
                    p = msg.pose.position
                    q = msg.pose.orientation
                roll, pitch, yaw = _rpy_from_quat(q.x, q.y, q.z, q.w)
                w.writerow(["%.6f" % rel,
                            "%.6f" % p.x, "%.6f" % p.y, "%.6f" % p.z,
                            "%.6f" % q.x, "%.6f" % q.y, "%.6f" % q.z, "%.6f" % q.w,
                            "%.6f" % roll, "%.6f" % pitch, "%.6f" % yaw])
            counts[topic] += 1

    if not writers:
        sys.stderr.write("aucun topic de pose trouvé dans le bag "
                         "(topics attendus : %s)\n" % ", ".join(TOPIC_MAP))
        sys.exit(2)

    for topic, (f, path) in files.items():
        f.close()
        print("  ✓ %-28s %6d msg  -> %s" % (topic, counts[topic], path))
    print("\n  Matlab :  plot_trajectories('%s')" %
          os.path.join(outdir, base))


if __name__ == "__main__":
    main()

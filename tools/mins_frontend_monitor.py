#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Moniteur temps réel du front-end MINS : features actives vs coût CPU.

Lit en continu le log de la stack (les lignes de timing que MINS émet déjà,
zéro instrumentation ajoutée) + la charge CPU réelle du process, et affiche
un tableau de bord glissant :

  features POOL / MSCKF | CAM ms/img | budget (période caméra) | marge | CPU%

Verdict temps réel : CAM ms doit rester < période caméra (66,7 ms @ 15 Hz).

Usage : python3 mins_frontend_monitor.py [durée_s=30]
"""
import re
import subprocess
import sys
import time

LOG = "/home/lab272/TOUT/logs/navigation_master.log"
CAM_PERIOD_MS = 1000.0 / 15.0     # flux infra actuel : 15 Hz

RE_TIME = re.compile(r"CAM:\s*(\d+)ms.*Total:\s*(\d+)ms")
RE_POOL = re.compile(r"POOL:(\d+), SLAM:(\d+), INIT:\d+, MSCKF:(\d+)")


def mins_pid():
    try:
        return int(subprocess.check_output(["pgrep", "-x", "subscribe"]).split()[0])
    except Exception:
        return None


def cpu_pct(pid):
    try:
        out = subprocess.check_output(
            ["ps", "-o", "%cpu=", "-p", str(pid)], text=True)
        return float(out.strip())
    except Exception:
        return float("nan")


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    pid = mins_pid()
    if pid is None:
        print("MINS (subscribe) introuvable")
        return
    cams, totals, pools, msckfs = [], [], [], []
    t0 = time.time()
    with open(LOG, "rb") as f:
        f.seek(0, 2)                      # fin du fichier : on ne lit que le neuf
        print("monitor %ds — budget %.1f ms/image (15 Hz)" % (dur, CAM_PERIOD_MS))
        while time.time() - t0 < dur:
            line = f.readline()
            if not line:
                time.sleep(0.1)
                continue
            s = line.decode("utf-8", "replace")
            m = RE_TIME.search(s)
            if m:
                cams.append(int(m.group(1)))
                totals.append(int(m.group(2)))
            m = RE_POOL.search(s)
            if m:
                pools.append(int(m.group(1)))
                msckfs.append(int(m.group(3)))

    def stat(v):
        return (min(v), sorted(v)[len(v) // 2], max(v)) if v else (0, 0, 0)

    cmin, cmed, cmax = stat(cams)
    pmin, pmed, pmax = stat(pools)
    print("\n=== FRONT-END MINS — fenêtre %.0fs ===" % dur)
    print("features POOL   : min %d  méd %d  max %d   (MSCKF utilisées: %s)"
          % (pmin, pmed, pmax, max(msckfs) if msckfs else "-"))
    print("CAM ms/image    : min %d  méd %d  max %d" % (cmin, cmed, cmax))
    print("budget 15 Hz    : %.1f ms  ->  marge médiane %+.1f ms  %s"
          % (CAM_PERIOD_MS, CAM_PERIOD_MS - cmed,
             "TEMPS RÉEL OK" if cmed < CAM_PERIOD_MS else "HORS BUDGET"))
    print("CPU process     : %.1f %% d'un cœur" % cpu_pct(pid))


if __name__ == "__main__":
    main()

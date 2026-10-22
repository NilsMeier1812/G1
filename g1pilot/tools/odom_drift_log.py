#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom_drift_log — Drift-Test fuer die LiDAR-Navigation, Stufe 0.

Schreibt mit, was der G1 selbst ueber seine Bewegung meldet (Onboard-
Odometrie aus Beinen + IMU), waehrend man ihn per PS4 eine vermessene
Strecke laufen laesst (z. B. 4 m geradeaus, dann 1 m nach rechts). Am Ende
steht die Endpose im START-Bezug neben dem, was das Massband sagt -> echte
Zahlen statt der Schaetzung im Plan (Abschnitt 8 im Bericht
analysen/lidar_autonav_real_g1.md).

Welches DDS-Topic die G1-Firmware fuer die Odometrie nutzt, ist ungeprueft.
Das Werkzeug probiert alle Kandidaten (SportModeState_) und zeigt, welche
Daten liefern. Die Gier (yaw) kommt aus dem Topic selbst, sonst aus der
IMU in rt/lowstate.

Abfrage OHNE Listener (Read() mit 20 Hz): ein lowstate-Listener mit 500 Hz
frisst in Python viel CPU (Messung 2026-10-21, siehe walk-ground-precheck).
Laeuft als eigener Prozess neben dem Real-Stack, beruehrt den Roboter nicht.

Aufruf (im Real-Container oder auf dem PC mit unitree_sdk2py):
  python3 tools/odom_drift_log.py --iface enp3s0           # Strg+C beendet
  python3 tools/odom_drift_log.py --iface enp3s0 --duration 60 --csv /tmp/drift.csv
"""
import argparse
import csv
import math
import os
import sys
import time

CANDIDATES = ("rt/odommodestate", "rt/lf/odommodestate",
              "rt/sportmodestate", "rt/lf/sportmodestate")


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def in_start_frame(start, pose):
    """(x, y, yaw) relativ zur Startpose: vor (+x), links (+y), Drehung."""
    x0, y0, a0 = start
    x, y, a = pose
    dx, dy = x - x0, y - y0
    c, s = math.cos(-a0), math.sin(-a0)
    return c * dx - s * dy, s * dx + c * dy, wrap(a - a0)


class Track:
    """Eine Quelle: Startpose, aktuelle Pose, gelaufene Strecke."""

    def __init__(self, name):
        self.name = name
        self.start = None
        self.last = None
        self.length = 0.0
        self.n = 0

    def add(self, x, y, yaw):
        p = (float(x), float(y), float(yaw))
        if self.start is None:
            self.start = p
        if self.last is not None:
            self.length += math.hypot(p[0] - self.last[0], p[1] - self.last[1])
        self.last = p
        self.n += 1

    def rel(self):
        if self.start is None:
            return None
        return in_start_frame(self.start, self.last)

    def line(self):
        r = self.rel()
        if r is None:
            return f"{self.name:<24} keine Daten"
        return (f"{self.name:<24} vor {r[0]:+6.3f} m  links {r[1]:+6.3f} m  "
                f"Drehung {math.degrees(r[2]):+6.1f} deg  Strecke {self.length:5.2f} m")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--iface", default=os.environ.get("ROBOT_INTERFACE", "eth0"))
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--duration", type=float, default=0.0, help="0 = bis Strg+C")
    ap.add_argument("--csv", default="")
    a = ap.parse_args(argv)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(a.domain, a.iface)
    subs = {}
    for t in CANDIDATES:
        s = ChannelSubscriber(t, SportModeState_)
        s.Init()
        subs[t] = s
    low = ChannelSubscriber("rt/lowstate", LowState_)
    low.Init()
    tracks = {t: Track(t) for t in CANDIDATES}
    imu_yaw = None

    path = a.csv or os.path.expanduser(time.strftime("~/g1_drift_%Y%m%d_%H%M%S.csv"))
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "topic", "x", "y", "yaw_topic", "yaw_imu"])
    print(f"Drift-Log auf {a.iface} -> {path}")
    print("Jetzt den G1 per PS4 die vermessene Strecke laufen lassen. Strg+C = Ende.\n")
    t0 = time.monotonic()
    next_print = t0 + 1.0
    try:
        while a.duration <= 0 or time.monotonic() - t0 < a.duration:
            m = low.Read(0.001)
            if m is not None:
                imu_yaw = float(m.imu_state.rpy[2])
            now = time.monotonic() - t0
            for t, s in subs.items():
                msg = s.Read(0.001)
                if msg is None:
                    continue
                yaw_t = float(msg.imu_state.rpy[2])
                # Manche Firmwares fuellen die Orientierung hier nicht -> IMU nehmen.
                yaw = yaw_t if (yaw_t != 0.0 or imu_yaw is None) else imu_yaw
                tracks[t].add(msg.position[0], msg.position[1], yaw)
                w.writerow([f"{now:.3f}", t, f"{msg.position[0]:.4f}", f"{msg.position[1]:.4f}",
                            f"{yaw_t:.4f}", "" if imu_yaw is None else f"{imu_yaw:.4f}"])
            if time.monotonic() >= next_print:
                next_print += 1.0
                live = [tr.line() for tr in tracks.values() if tr.n]
                print(f"[{now:6.1f} s] " + (" | ".join(live) if live else
                      "noch keine Odometrie-Daten (Interface/Domain pruefen)"))
            time.sleep(1.0 / a.rate)
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
    print("\n=== Ergebnis (Bezug: Startpose, vor = Blickrichtung beim Start) ===")
    for tr in tracks.values():
        print(tr.line())
    print("\nMit dem Massband vergleichen: Endpunkt vor/links + Drehung. "
          f"Rohdaten: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Ende-zu-Ende-Test der PS4-Oberkoerper-Steuerung in der laufenden MuJoCo-Sim.

Simuliert den Controller: publiziert Joy in der festen Belegung (ps4_layout)
auf /g1pilot/ps4/joy -- genau das, was ps4_joystick vom echten Controller
liefert -- und misst per TF, was die Haende tun. Prueft ausserdem, dass dabei
NICHTS das Laufen anstoesst (loco_cmd_vel, /g1pilot/joy, Basis bleibt stehen).

Voraussetzung: Sim-Stack laeuft mit G1_PS4_ARMS=1 (Controller muss nicht
angeschlossen sein). Im Container:
    docker exec -it g1pilot-g1pilot-sim-1 bash -c \
      "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/g1pilot/test_ps4_arm_sim.py"
Optionen: --no-estop (NOT-HALT am Ende auslassen; der setzt den Roboter in DAMP).
Exit-Code 0 = alle Pruefungen bestanden.
"""
import argparse
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from g1pilot.teleoperation import ps4_layout as L
from g1pilot.teleoperation.hand_jog import JOG_FRAME, JOG_TF, qangle

RATE = 50.0


class Probe(Node):
    def __init__(self):
        super().__init__("ps4_sim_probe")
        self.pub = self.create_publisher(Joy, "/g1pilot/ps4/joy", 10)
        self.pub_start = self.create_publisher(Bool, "/g1pilot/start", 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.walk_cmds = []        # nicht-null loco_cmd_vel
        self.joy_out = []          # nicht-null /g1pilot/joy
        self.start_walking = []
        self.estops = []
        self.hand_actions = []
        self.cancels = []
        self.status = {}
        self.fingers = {}
        self.create_subscription(Twist, "/g1pilot/loco_cmd_vel", self._cmd, 50)
        self.create_subscription(Joy, "/g1pilot/joy", self._joy, 50)
        self.create_subscription(Bool, "/g1pilot/start_walking",
                                 lambda m: m.data and self.start_walking.append(1), 10)
        self.create_subscription(Bool, "/g1pilot/emergency_stop",
                                 lambda m: m.data and self.estops.append(1), 10)
        self.create_subscription(Bool, "/g1pilot/pose_store/cancel",
                                 lambda m: m.data and self.cancels.append(1), 10)
        for s in ("left", "right"):
            self.create_subscription(String, f"/g1pilot/hand_action/{s}",
                                     lambda m, s=s: self.hand_actions.append((s, m.data)), 10)
        q = QoSProfile(depth=1)
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "/g1pilot/ps4/status", self._status, q)
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.arm_states = []       # /g1pilot/arm_command/status (state)
        self.create_subscription(String, "/g1pilot/arm_command/status",
                                 lambda m: self.arm_states.append(json.loads(m.data).get("state")), 10)
        self.goal_times = []       # Empfangszeiten /g1pilot/hand_goal/right
        self.create_subscription(PoseStamped, "/g1pilot/hand_goal/right",
                                 lambda m: self.goal_times.append(time.time()), 50)
        self.axes = [0.0] * L.NUM_AXES
        self.buttons = [0] * L.NUM_BUTTONS
        self.sending = True
        self.create_timer(1.0 / RATE, self._send)

    def _cmd(self, m):
        if abs(m.linear.x) + abs(m.linear.y) + abs(m.angular.z) > 1e-6:
            self.walk_cmds.append(m)

    def _joy(self, m):
        if any(abs(a) > 1e-6 for a in m.axes) or any(m.buttons):
            self.joy_out.append(m)

    def _status(self, m):
        self.status = json.loads(m.data)

    def _js(self, m):
        for n, p in zip(m.name, m.position):
            if "right" in n.lower() and ("index" in n.lower()):
                self.fingers[n] = p

    def _send(self):
        if not self.sending:
            return
        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.axes = list(self.axes)
        m.buttons = list(self.buttons)
        self.pub.publish(m)

    # ── Messen ──────────────────────────────────────────────────────────
    def hand(self, side):
        t = self.tf.lookup_transform(JOG_FRAME, JOG_TF[side], rclpy.time.Time())
        p, r = t.transform.translation, t.transform.rotation
        return (p.x, p.y, p.z), (r.x, r.y, r.z, r.w)

    def base(self):
        for parent, child in (("odom_unitree", "pelvis"), ("world", "pelvis")):
            try:
                t = self.tf.lookup_transform(parent, child, rclpy.time.Time())
                return (t.transform.translation.x, t.transform.translation.y)
            except Exception:   # noqa: BLE001
                continue
        return None


class Runner:
    def __init__(self, probe):
        self.p = probe
        self.results = []

    def sleep(self, s):
        time.sleep(s)

    def tap(self, btn, hold=0.1):
        self.p.buttons[btn] = 1
        self.sleep(hold)
        self.p.buttons[btn] = 0
        self.sleep(0.15)

    def stick(self, dur, **axes):
        for k, v in axes.items():
            self.p.axes[getattr(L, k)] = v
        self.sleep(dur)
        self.p.axes = [0.0] * L.NUM_AXES

    def check(self, name, ok, detail):
        self.results.append((name, bool(ok), detail))
        print(f"  [{'OK ' if ok else 'FAIL'}] {name}: {detail}", flush=True)

    def snap(self):
        return {s: self.p.hand(s) for s in ("left", "right")}

    def settle(self, tol=0.002, window=0.5, timeout=10.0):
        """Warten, bis beide Haende stehen (< tol in `window` s). Der Arm haengt
        dem Ziel je nach Rechner-Last hinterher (Ziel-Glaettung im
        arm_controller) -- gemessen wird erst, wenn die letzte Bewegung durch ist.
        -> Wartezeit in s (oder None, wenn sie nicht zur Ruhe kommen)."""
        t0 = time.time()
        prev = self.snap()
        while time.time() - t0 < timeout:
            self.sleep(window)
            cur = self.snap()
            if all(math.dist(prev[s][0], cur[s][0]) < tol for s in cur):
                return time.time() - t0
            prev = cur
        return None


def moved(r, name, a, b, axis, sign, min_m=0.02):
    """Bewegung entlang `axis` mit Vorzeichen `sign`: mindestens min_m und
    deutlich groesser als auf den anderen Achsen (Richtung stimmt)."""
    dv = [b[i] - a[i] for i in range(3)]
    main = dv[axis] * sign
    other = max(abs(dv[i]) for i in range(3) if i != axis)
    r.check(name, main >= min_m and main >= 2.0 * other,
            f"{'xyz'[axis]}: {dv[axis] * 100:+.1f} cm, quer max {other * 100:.1f} cm")


def d(a, b, i=None):
    if i is None:
        return math.dist(a, b)
    return b[i] - a[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-estop", action="store_true")
    args = ap.parse_args()
    rclpy.init()
    p = Probe()
    th = threading.Thread(target=rclpy.spin, args=(p,), daemon=True)
    th.start()
    r = Runner(p)

    print("Warte auf TF der Haende ...", flush=True)
    t0 = time.time()
    while True:
        try:
            r.snap()
            break
        except Exception:   # noqa: BLE001
            if time.time() - t0 > 30:
                print("FEHLER: keine TF pelvis->Hand. Laeuft der Sim-Stack?")
                return 2
            time.sleep(0.2)
    base0 = p.base()
    r.sleep(1.0)
    r.tap(L.DPAD_RIGHT)
    if p.status.get("mode") == "turn":
        r.tap(L.TRIANGLE)
    r.check("Status: rechte Hand, Verschieben",
            p.status.get("selection") == "right" and p.status.get("mode") == "move",
            p.status.get("label"))

    print("1) Options: Greifen-Modus / Arme uebernehmen", flush=True)
    r.tap(L.OPTIONS)
    r.sleep(2.0)

    def start_pose(sel_btn):
        """Gleiche Ausgangslage fuer jede Pruefung: Grundstellung (Share
        halten), dann beide Haende 2.5 s nach vorn (frei vom Koerper), dann
        Auswahl per Steuerkreuz."""
        r.tap(L.DPAD_UP)
        p.arm_states.clear()
        r.tap(L.SHARE, hold=1.3)
        # Grundstellung ist eine geplante Bewegung: bis zum Ende warten -- ein
        # Stick-Ausschlag vorher wuerde sie abbrechen (manueller Vorrang).
        t0 = time.time()
        while time.time() - t0 < 25 and not any(
                st in ("reached", "failed", "cancelled", "rejected") for st in p.arm_states):
            r.sleep(0.1)
        r.settle()
        r.stick(2.5, LY=1.0)
        r.settle()
        r.tap(sel_btn)
        r.settle()
        return r.snap()

    print("2) Rechte Hand vor (linker Stick oben, 2 s, Tempo Normal)", flush=True)
    a = start_pose(L.DPAD_RIGHT)
    r.stick(2.0, LY=1.0)
    b = r.snap()
    t_settle = r.settle()
    c = r.snap()
    moved(r, "rechts vor", a["right"][0], c["right"][0], 0, +1)
    r.check("links bleibt", d(a["left"][0], c["left"][0]) < 0.015,
            f"{d(a['left'][0], c['left'][0]) * 100:.1f} cm")
    r.check("steht nach Loslassen", t_settle is not None,
            f"Nachlauf {d(b['right'][0], c['right'][0]) * 100:.1f} cm, "
            f"steht nach {t_settle if t_settle is None else round(t_settle, 1)} s")

    print("3) Rechte Hand hoch (rechter Stick oben, 2 s)", flush=True)
    a = start_pose(L.DPAD_RIGHT)
    r.stick(2.0, RY=1.0)
    r.settle()
    b = r.snap()
    moved(r, "rechts hoch", a["right"][0], b["right"][0], 2, +1)

    print("4) Dreieck -> DREHEN, rechte Hand kippen (3 s)", flush=True)
    a = start_pose(L.DPAD_RIGHT)
    r.tap(L.TRIANGLE)
    r.check("Status Drehen", p.status.get("mode") == "turn", p.status.get("label"))
    r.stick(3.0, LY=1.0)
    r.settle()
    b = r.snap()
    ang = math.degrees(qangle(a["right"][1], b["right"][1]))
    r.check("rechts gedreht", ang > 8.0, f"{ang:.1f} Grad")
    r.check("Position beim Drehen fest", d(a["right"][0], b["right"][0]) < 0.03,
            f"{d(a['right'][0], b['right'][0]) * 100:.1f} cm")
    r.tap(L.TRIANGLE)

    print("5) Steuerkreuz links -> linke Hand nach links (2 s)", flush=True)
    a = start_pose(L.DPAD_LEFT)
    r.check("Status links", p.status.get("selection") == "left", p.status.get("label"))
    r.stick(2.0, LX=1.0)
    r.settle()
    b = r.snap()
    moved(r, "links nach links", a["left"][0], b["left"][0], 1, +1)
    r.check("rechts bleibt", d(a["right"][0], b["right"][0]) < 0.015,
            f"{d(a['right'][0], b['right'][0]) * 100:.1f} cm")

    print("6) Steuerkreuz oben -> beide parallel hoch (2.5 s)", flush=True)
    a = start_pose(L.DPAD_UP)
    r.stick(2.5, RY=1.0)
    r.settle()
    b = r.snap()
    moved(r, "beide hoch: links", a["left"][0], b["left"][0], 2, +1)
    moved(r, "beide hoch: rechts", a["right"][0], b["right"][0], 2, +1)

    print("7) Steuerkreuz unten -> gespiegelt: Stick rechts = Haende auseinander (2 s)", flush=True)
    a = start_pose(L.DPAD_DOWN)
    r.stick(2.0, LX=-1.0)
    r.settle()
    b = r.snap()
    moved(r, "gespiegelt: rechts nach rechts", a["right"][0], b["right"][0], 1, -1)
    moved(r, "gespiegelt: links nach links", a["left"][0], b["left"][0], 1, +1)

    print("8) Kreis/Viereck -> rechte Hand auf/zu", flush=True)
    r.tap(L.DPAD_RIGHT)
    p.hand_actions.clear()
    f0 = dict(p.fingers)
    r.tap(L.SQUARE)
    r.sleep(1.5)
    f1 = dict(p.fingers)
    r.tap(L.CIRCLE)
    r.sleep(1.5)
    f2 = dict(p.fingers)
    r.check("hand_action", p.hand_actions == [("right", "close"), ("right", "open")],
            str(p.hand_actions))
    if f0 and f1 and f2:
        k = sorted(f1)[0]
        r.check("Finger bewegen sich", abs(f1[k] - f2[k]) > 0.3,
                f"{k}: zu {f1[k]:.2f} / auf {f2[k]:.2f} rad")
    else:
        print("  (keine Finger-Gelenke in /joint_states -- ohne Inspire-Haende)")

    print("9) Controller-Abriss: Stick voll, dann keine Daten mehr", flush=True)
    r.settle()
    p.axes[L.LY] = -1.0
    r.sleep(0.6)
    p.sending = False
    t_cut = time.time()
    r.sleep(1.5)
    late = [t - t_cut for t in p.goal_times if t - t_cut > 0.8]
    t_settle = r.settle()
    p.axes = [0.0] * L.NUM_AXES
    p.sending = True
    r.check("keine Ziele mehr nach Abriss", not late,
            f"{len(late)} Ziele spaeter als 0.8 s nach dem letzten Joy")
    r.check("Hand kommt zur Ruhe", t_settle is not None, f"nach {t_settle} s")
    r.sleep(0.5)

    print("10) Share kurz -> nichts; Share 1.2 s -> Grundstellung; Kreuz bricht ab", flush=True)
    r.settle()
    a = r.snap()
    r.tap(L.SHARE, hold=0.3)
    r.sleep(1.0)
    b = r.snap()
    r.check("Share kurz: keine Grundstellung", d(a["right"][0], b["right"][0]) < 0.01,
            f"{d(a['right'][0], b['right'][0]) * 100:.1f} cm")
    p.cancels.clear()
    r.tap(L.SHARE, hold=1.2)
    r.sleep(0.6)
    r.tap(L.CROSS)
    c = r.snap()
    r.sleep(1.5)
    e = r.snap()
    r.check("Kreuz sendet Abbruch", len(p.cancels) >= 1, f"{len(p.cancels)}x cancel")
    r.check("Hand steht nach Kreuz", d(c["right"][0], e["right"][0]) < 0.03,
            f"{d(c['right'][0], e['right'][0]) * 100:.1f} cm")

    base1 = p.base()
    if base0 and base1:
        r.check("Basis bleibt stehen", math.dist(base0, base1) < 0.05,
                f"{math.dist(base0, base1) * 100:.1f} cm")
    r.check("kein Laufbefehl (loco_cmd_vel)", not p.walk_cmds, f"{len(p.walk_cmds)} Befehle")
    r.check("nichts auf /g1pilot/joy", not p.joy_out, f"{len(p.joy_out)} Nachrichten")
    r.check("kein START WALKING", not p.start_walking, f"{len(p.start_walking)}")

    if not args.no_estop:
        print("11) PS -> NOT-HALT, dann START quittiert", flush=True)
        r.tap(L.PS)
        r.sleep(0.5)
        r.check("NOT-HALT gesendet", len(p.estops) >= 1 and p.status.get("estop") is True,
                f"{len(p.estops)}x, status estop={p.status.get('estop')}")
        p.pub_start.publish(Bool(data=True))
        r.sleep(0.5)
        r.check("START quittiert", p.status.get("estop") is False, p.status.get("label"))

    n_fail = sum(1 for _, ok, _ in r.results if not ok)
    print(f"\n{len(r.results) - n_fail}/{len(r.results)} Pruefungen bestanden.")
    p.destroy_node()
    rclpy.shutdown()
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())

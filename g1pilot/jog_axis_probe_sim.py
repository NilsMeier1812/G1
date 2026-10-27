#!/usr/bin/env python3
"""Messung in der laufenden MuJoCo-Sim: wie stark weicht die Hand beim
Verfahren (PS4-Jog, Tempo Normal) in den NICHT bewegten Achsen ab?

Je Startpose (Gelenkwinkel per /g1pilot/arm_command) und Richtung:
Stick 1 s voll, loslassen, warten bis die Hand steht. Gemessen per TF
pelvis -> <side>_hand_point_contact (50 Hz):
  weg      Fahrweg entlang der Sollrichtung [cm]
  quer     Versatz senkrecht dazu [cm] (max waehrend / nach der Fahrt)
  ori      Drehung der Hand gegenueber dem Start [deg] (max / Ende)
  ziel_ori Drehung des gesendeten Jog-Ziels (hand_goal) gegenueber dem Start [deg]
Ausserdem "Tippen": 4x 0.4 s vor + 4x 0.4 s zurueck (Ratschen-Effekt).
--mirror: beide Haende aus spiegelbildlicher Startlage, Auswahl "BEIDE
gespiegelt", dieselben Fahrten; gemessen wird, wie weit die Fahrwege beider
Haende vom Spiegelbild abweichen (spiegel [cm]) und wie stark sie sich drehen.
--reference command|start|tf setzt vorher jog_reference am ps4_arm_teleop.

Voraussetzung: Sim-Stack mit G1_PS4_ARMS=1. Im Container:
  python3 /ros2_ws/src/g1pilot/jog_axis_probe_sim.py [--json out.json] [--quick]
"""
import argparse
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from g1pilot.teleoperation import ps4_layout as L
from g1pilot.teleoperation.hand_jog import JOG_FRAME, JOG_TF, qangle, qmul, qconj

POSES = {
    "P1_brust": [-0.3, -0.1, 0.0, -0.3, 0.0, 0.0, 0.0],
    "P2_tisch": [-0.2, -0.15, 0.0, 0.1, 0.0, 0.4, 0.0],
    "P3_hoch":  [-0.9, -0.1, 0.0, -0.2, 0.0, 0.0, 0.0],
    "P4_seit":  [-0.4, -0.4, 0.3, -0.2, 0.3, 0.0, 0.0],
    "P5_innen": [-0.4, 0.0, -0.3, -0.5, -0.2, 0.2, 0.2],
}
LEFT_PARK = [0.3, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0]
DIRS = {"+x": ((1, 0, 0), {"LY": 1.0}), "-x": ((-1, 0, 0), {"LY": -1.0}),
        "+y": ((0, 1, 0), {"LX": 1.0}), "-y": ((0, -1, 0), {"LX": -1.0}),
        "+z": ((0, 0, 1), {"RY": 1.0}), "-z": ((0, 0, -1), {"RY": -1.0})}
RATE = 50.0


def rotvec(q):
    """Quaternion (x,y,z,w) -> Drehvektor [rad]."""
    x, y, z, w = q
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-12:
        return np.zeros(3)
    ang = 2.0 * math.atan2(s, w)
    return np.array([x, y, z]) / s * ang


class Probe(Node):
    def __init__(self, side):
        super().__init__("jog_axis_probe")
        from tf2_ros import Buffer, TransformListener
        self.side = side
        self.pub = self.create_publisher(Joy, "/g1pilot/ps4/joy", 10)
        self.pub_cmd = self.create_publisher(String, "/g1pilot/arm_command", 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.status = {}
        self.cmd_states = []
        q = QoSProfile(depth=1)
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "/g1pilot/ps4/status",
                                 lambda m: self.status.update(json.loads(m.data)), q)
        self.create_subscription(String, "/g1pilot/arm_command/status",
                                 lambda m: self.cmd_states.append(json.loads(m.data)), 20)
        self.goals = []
        self.create_subscription(PoseStamped, f"/g1pilot/hand_goal/{side}", self._goal, 50)
        self.axes = [0.0] * L.NUM_AXES
        self.buttons = [0] * L.NUM_BUTTONS
        self.rec = None
        self.rec2 = None
        self.create_timer(1.0 / RATE, self._tick)

    def _goal(self, m):
        o, p = m.pose.orientation, m.pose.position
        if self.rec is not None:
            self.goals.append((time.time(), (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))

    def hand(self, side=None):
        t = self.tf.lookup_transform(JOG_FRAME, JOG_TF[side or self.side], rclpy.time.Time())
        p, r = t.transform.translation, t.transform.rotation
        return (p.x, p.y, p.z), (r.x, r.y, r.z, r.w)

    def _tick(self):
        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.axes = list(self.axes)
        m.buttons = list(self.buttons)
        self.pub.publish(m)
        if self.rec is not None:
            try:
                self.rec.append((time.time(),) + self.hand())
            except Exception:   # noqa: BLE001
                pass
        if self.rec2 is not None:
            try:
                other = "left" if self.side == "right" else "right"
                self.rec2.append((time.time(),) + self.hand(other))
            except Exception:   # noqa: BLE001
                pass


class Runner:
    def __init__(self, p):
        self.p = p

    def tap(self, btn, hold=0.1):
        self.p.buttons[btn] = 1
        time.sleep(hold)
        self.p.buttons[btn] = 0
        time.sleep(0.15)

    def stick(self, dur, axes):
        for k, v in axes.items():
            self.p.axes[getattr(L, k)] = v
        time.sleep(dur)
        self.p.axes = [0.0] * L.NUM_AXES

    def stick_until(self, axes, u, dist, timeout):
        """Stick halten, bis die Hand `dist` entlang u gefahren ist."""
        p0 = np.array(self.p.hand()[0])
        for k, v in axes.items():
            self.p.axes[getattr(L, k)] = v
        t0 = time.time()
        while time.time() - t0 < timeout:
            if float((np.array(self.p.hand()[0]) - p0) @ np.array(u, float)) >= dist:
                break
            time.sleep(0.02)
        self.p.axes = [0.0] * L.NUM_AXES

    def settle(self, tol=0.0015, window=0.4, timeout=12.0):
        t0 = time.time()
        prev = self.p.hand()
        while time.time() - t0 < timeout:
            time.sleep(window)
            cur = self.p.hand()
            if math.dist(prev[0], cur[0]) < tol and qangle(prev[1], cur[1]) < math.radians(0.3):
                return time.time() - t0
            prev = cur
        return None

    def goto(self, right, left=None, timeout=30.0):
        self.p.cmd_states.clear()
        rid = f"probe{int(time.time() * 1000) % 10**8}"
        cmd = {"type": "joints", "id": rid}
        cmd[self.p.side] = right
        if left is not None:
            cmd["left" if self.p.side == "right" else "right"] = left
        self.p.pub_cmd.publish(String(data=json.dumps(cmd)))
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = [s.get("state") for s in self.p.cmd_states if s.get("id") == rid]
            if any(s in ("reached", "failed", "rejected", "cancelled") for s in st):
                break
            time.sleep(0.1)
        self.settle()
        time.sleep(0.5)
        return st[-1] if st else None


def analyse(rec, goals, u):
    t0, p0, q0 = rec[0]
    p0 = np.array(p0)
    u = np.array(u, float) if u is not None else None
    lat, ori, along = [], [], []
    for (_, p, q) in rec:
        d = np.array(p) - p0
        if u is not None:
            a = float(d @ u)
            along.append(a)
            lat.append(float(np.linalg.norm(d - a * u)))
        ori.append(math.degrees(qangle(q0, q)))
    gori = [math.degrees(qangle(q0, g[2])) for g in goals] or [0.0]
    # Drehachse der Endabweichung im pelvis-Frame (welche Richtung kippt die Hand?)
    dq = qmul(rec[-1][2], qconj(q0))
    rv = np.degrees(rotvec(dq))
    out = dict(ori_max=max(ori), ori_end=ori[-1], goal_ori_max=max(gori),
               goal_ori_end=gori[-1], rot_end_xyz=[round(float(c), 1) for c in rv])
    if u is not None:
        out.update(along=along[-1] * 100, lat_max=max(lat) * 100, lat_end=lat[-1] * 100)
    return out


def mirror_pose(q):
    """Gelenkwinkel des einen Arms -> spiegelbildliche des anderen (roll/yaw-Achsen)."""
    return [c * s for c, s in zip(q, (1, -1, -1, 1, -1, 1, -1))]


def set_reference(mode):
    import subprocess
    r = subprocess.run(["ros2", "param", "set", "/ps4_arm_teleop", "jog_reference", mode],
                       capture_output=True, text=True, timeout=30)
    print("jog_reference:", mode, (r.stdout or r.stderr).strip(), flush=True)


def run_mirror(p, r, poses, args):
    """Gespiegelt: Fahrweg rechts vs. gespiegelter Fahrweg links."""
    r.tap(L.DPAD_DOWN)
    time.sleep(1.0)
    seqs = {"einzeln": [[d] for d in ("+y", "-y", "+z", "-z", "+x", "-x")],
            "folge": [["+y", "+z", "+x", "-y", "-z"]]}
    results = []
    other = "left" if p.side == "right" else "right"
    for pn, q in poses.items():
        for sn, seq_list in seqs.items():
            for seq in seq_list:
                r.goto(q, mirror_pose(q))
                p.rec, p.rec2 = [], []
                time.sleep(0.3)
                for dn in seq:
                    r.stick(args.dur, DIRS[dn][1])
                    r.settle(timeout=6.0)
                time.sleep(0.3)
                a, b = p.rec, p.rec2
                p.rec = p.rec2 = None
                da = np.array(a[-1][1]) - np.array(a[0][1])
                db = np.array(b[-1][1]) - np.array(b[0][1])
                mir = float(np.linalg.norm(da - db * np.array([1.0, -1.0, 1.0]))) * 100
                ori = max(math.degrees(qangle(a[0][2], a[-1][2])),
                          math.degrees(qangle(b[0][2], b[-1][2])))
                m = dict(pose=pn, seq="".join(seq), mirror_cm=mir, ori_end=ori,
                         way_cm=float(np.linalg.norm(da)) * 100)
                results.append(m)
                print(f"{pn:8s} {m['seq']:12s}: Weg {m['way_cm']:5.1f} cm | spiegel "
                      f"{mir:4.2f} cm | ori end {ori:4.1f} deg ({p.side}/{other})", flush=True)
    v = sorted(x["mirror_cm"] for x in results)
    o = sorted(x["ori_end"] for x in results)
    print(f"ZUSAMMENFASSUNG spiegel median {v[len(v) // 2]:.2f} max {v[-1]:.2f} cm | "
          f"ori_end median {o[len(o) // 2]:.1f} max {o[-1]:.1f} deg")
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", default="right")
    ap.add_argument("--json", default="")
    ap.add_argument("--quick", action="store_true", help="nur P1/P2, +x/-z/+y")
    ap.add_argument("--dur", type=float, default=1.0)
    ap.add_argument("--poses", default="")
    ap.add_argument("--dirs", default="", help="z.B. +y,+z")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--no-tip", action="store_true")
    ap.add_argument("--dist", type=float, default=0.0,
                    help="> 0: Stick halten, bis die Hand so weit [m] gefahren ist (max. --dur)")
    ap.add_argument("--mirror", action="store_true", help="beide Haende gespiegelt")
    ap.add_argument("--reference", default="", help="jog_reference vorher setzen")
    args = ap.parse_args()
    if args.reference:
        set_reference(args.reference)
    rclpy.init()
    p = Probe(args.side)
    ex = SingleThreadedExecutor()
    ex.add_node(p)
    threading.Thread(target=ex.spin, daemon=True).start()
    r = Runner(p)
    t0 = time.time()
    while True:
        try:
            p.hand()
            break
        except Exception:   # noqa: BLE001
            if time.time() - t0 > 60:
                print("keine TF")
                return 2
            time.sleep(0.3)
    time.sleep(1.5)
    # Auswahl/Modus/Tempo definiert setzen
    r.tap(L.DPAD_RIGHT if args.side == "right" else L.DPAD_LEFT)
    time.sleep(1.2)
    if p.status.get("mode") == "turn":
        r.tap(L.TRIANGLE)
    for _ in range(2):
        r.tap(L.L1)
    r.tap(L.R1)
    r.tap(L.R2)
    time.sleep(2.0)
    # Grundstellung (geplant), danach sind beide Arme frei vom Koerper
    p.cmd_states.clear()
    r.tap(L.SHARE, hold=1.3)
    time.sleep(8.0)
    r.settle()
    print("Status:", p.status.get("label"), flush=True)

    poses = dict(POSES)
    if args.poses:
        poses = {k: v for k, v in POSES.items() if k in args.poses.split(",")}
    dirs = dict(DIRS)
    if args.quick:
        poses = {k: POSES[k] for k in ("P1_brust", "P2_tisch")}
        dirs = {k: DIRS[k] for k in ("+x", "-z", "+y")}
    if args.dirs:
        dirs = {k: DIRS[k] for k in args.dirs.split(",")}
    if args.mirror:
        results = run_mirror(p, r, poses, args)
        if args.json:
            with open(args.json, "w") as f:
                json.dump(results, f, indent=1)
        rclpy.shutdown()
        return 0
    results = []
    for pn, q in poses.items():
        for dn, (u, axes) in [d for d in dirs.items() for _ in range(args.repeat)]:
            st = r.goto(q)
            p.rec, p.goals = [], []
            time.sleep(0.2)
            if args.dist > 0:
                r.stick_until(axes, u, args.dist, args.dur)
            else:
                r.stick(args.dur, axes)
            r.settle()
            time.sleep(0.3)
            rec, goals = p.rec, p.goals
            p.rec = None
            m = analyse(rec, goals, u)
            m.update(pose=pn, dir=dn, goto=st)
            results.append(m)
            print(f"{pn:8s} {dn}: weg {m['along']:5.1f} cm | quer max {m['lat_max']:4.1f} "
                  f"end {m['lat_end']:4.1f} cm | ori max {m['ori_max']:5.1f} end "
                  f"{m['ori_end']:5.1f} deg | ziel_ori end {m['goal_ori_end']:4.1f} | "
                  f"rot {m['rot_end_xyz']}", flush=True)
        if args.no_tip:
            continue
        # Tippen vor/zurueck (Ratsche)
        st = r.goto(q)
        p.rec, p.goals = [], []
        time.sleep(0.2)
        for axes in ([{"LY": 1.0}] * 4 + [{"LY": -1.0}] * 4):
            r.stick(0.4, axes)
            r.settle(timeout=4.0)
        time.sleep(0.3)
        rec, goals = p.rec, p.goals
        p.rec = None
        m = analyse(rec, goals, None)
        m.update(pose=pn, dir="tip", goto=st,
                 pos_end=math.dist(rec[0][1], rec[-1][1]) * 100)
        results.append(m)
        print(f"{pn:8s} tip: Rueckkehrfehler {m['pos_end']:4.1f} cm | ori max {m['ori_max']:5.1f} "
              f"end {m['ori_end']:5.1f} deg | ziel_ori end {m['goal_ori_end']:4.1f} | "
              f"rot {m['rot_end_xyz']}", flush=True)

    mv = [x for x in results if x["dir"] != "tip"]
    def stat(k, rows):
        v = sorted(x[k] for x in rows)
        return f"median {v[len(v) // 2]:.1f} p90 {v[int(0.9 * (len(v) - 1))]:.1f} max {v[-1]:.1f}"
    print("ZUSAMMENFASSUNG")
    for k in ("lat_max", "lat_end", "ori_max", "ori_end", "goal_ori_end"):
        print(f"  {k:12s} {stat(k, mv)}")
    tips = [x for x in results if x["dir"] == "tip"]
    if tips:
        for k in ("pos_end", "ori_end", "goal_ori_end"):
            print(f"  tip {k:8s} {stat(k, tips)}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=1)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

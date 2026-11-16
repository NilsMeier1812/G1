#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v8_nothalt.py -- V8: Not-Halt, Ausfall eines Bedienwegs und Zielsprung.

Ziele (--ziel):
  attrappe  arm_controller im Realbetrieb-Modus (G1_SIM_MODE=false, Real-
            Parameter aus bringup_real) gegen die Roboter-Attrappe FakeG1 aus
            test/test_arm_controller_real.py -- alles in einem Prozess, eigene
            ROS-Domain (G1_EVAL_DOMAIN, Standard 87, nur localhost). Kein
            Roboter, kein DDS nach aussen. "Herstellerschnittstelle" = der
            Write() auf rt/arm_sdk, den die Attrappe mit Zeitstempel mitschreibt.
  sim       laufender Sim-Stack (am besten HOLD_BASE_MODE=weld, sonst faellt
            der G1 beim Not-Halt, weil loco_sim die Beine daempft; Controller-
            Faelle brauchen G1_PS4_ARMS=1). "Herstellerschnittstelle" = rt/arm_sdk
            (mitgelesen). Nur Simulation -- das Werkzeug sendet Befehle.

Faelle (--faelle, Standard alle passenden):
  nothalt_topic    NOT-HALT als Bool auf /g1pilot/emergency_stop -- so senden
                   ihn die Demo-GUI (Knopf) und das Handy (ueber die GUI).
  nothalt_ps       PS-Taste: Joy-Nachricht an ps4_arm_teleop (/g1pilot/ps4/joy).
  vollnotaus       R1+L1+PS gehalten (2 s): brutto ab Druck, netto = brutto -
                   Haltezeit; zusaetzlich Zeit bis /g1pilot/full_emergency_stop
                   und (Attrappe) bis die Taille schlaff ist.
  ausfall_joy      Joy-Strom reisst waehrend eines Jogs ab (Funk/Akku):
                   Zeit vom letzten Joy bis die Armkommandos stehen.
  ausfall_ziel     Strom von Hand-Zielen (/g1pilot/hand_goal, RViz-Marker/
                   Fremdprogramm) reisst ab: Zeit bis die Kommandos stehen.
  ausfall_zustand  (nur Attrappe) rt/lowstate liefert waehrend einer geplanten
                   Fahrt nichts Neues mehr (Attrappe eingefroren = Read liefert
                   die letzte Nachricht): haelt der Arm an, und wann?
  zielsprung       Hand-Ziel springt um --sprung m (zufaellige Richtung):
                   groesste Handgeschwindigkeit (Vorwaertskinematik der
                   kommandierten Gelenkwinkel) gegen ee_velocity_limit.

Messgroessen ohne Glaettung: Ausloesen = Zeitpunkt des publish()/Ausfalls
(time.monotonic), Antwort = Zeitstempel des ersten passenden Kommandos.
Je Fall --wdh Wiederholungen (Standard 100); Verteilung n, Median, P95, Max.

Aufruf:
    python3 tools/eval/v8_nothalt.py --ziel attrappe [--faelle nothalt_topic ...] [--wdh 100]
    python3 tools/eval/v8_nothalt.py --ziel sim      (im g1pilot-sim-Container)
"""
import argparse
import importlib.util
import math
import os
import random
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_common as ec  # noqa: E402

VERSUCH = "v8_nothalt"
ARM_IDX = list(range(15, 29))
WAIST_IDX = [12, 13, 14]
FAELLE = ("nothalt_topic", "nothalt_ps", "vollnotaus", "ausfall_joy", "ausfall_ziel",
          "ausfall_zustand", "zielsprung")
NUR_ATTRAPPE = ("ausfall_zustand",)
# Joy-Belegung (ps4_layout.py)
L1, R1, PS = 4, 5, 10
LY = 1
NUM_BUTTONS, NUM_AXES = 17, 8
# Attrappe: Parameter wie bringup_real.launch.py (manipulation_launcher)
REAL_PARAMS = {"use_robot": True, "arm_velocity_limit": 1.5, "ee_velocity_limit": 0.25,
               "self_collision_gate": True, "max_tracking_error": 0.3, "arm_accel_limit": 3.0,
               "ik_async": True, "kp_low": 60.0, "kd_low": 1.5, "kp_wrist": 40.0, "kd_wrist": 1.5,
               "waist_hold": True, "kp_waist": 150.0, "kd_waist": 4.0}
STILL_EPS = 1e-4          # rad je Kommando: darunter "steht"
STILL_WIN = 0.2           # s ohne Bewegung = Halt
# Ausgangslage vor jedem Bewegungsfall (Hand vor dem Koerper, frei): ohne sie
# wandert die Hand ueber die Wiederholungen ab, bis das Kollisions-Gate haelt.
START_POSE = {"left": [-0.4, 0.25, 0.0, 0.9, 0.0, 0.0, 0.0],
              "right": [-0.4, -0.25, 0.0, 0.9, 0.0, 0.0, 0.0]}


# ══════════════════════════════════════════════════════════════════════
# Kommando-Mitschnitt
# ══════════════════════════════════════════════════════════════════════
class Writes:
    """(t, w, q14, kp14, kp_waist3) je Kommando an die Herstellerschnittstelle."""

    def __init__(self):
        self.lock = threading.Lock()
        self.rows = []

    def add(self, t, msg):
        w = float(msg.motor_cmd[29].q)
        q = [float(msg.motor_cmd[j].q) for j in ARM_IDX]
        kp = [float(msg.motor_cmd[j].kp) for j in ARM_IDX]
        kw = [float(msg.motor_cmd[j].kp) for j in WAIST_IDX]
        with self.lock:
            self.rows.append((t, w, q, kp, kw))
            if len(self.rows) > 200000:
                del self.rows[:100000]

    def since(self, t0):
        with self.lock:
            return [r for r in self.rows if r[0] >= t0]

    def last(self):
        with self.lock:
            return self.rows[-1] if self.rows else None


def first_slack(rows):
    """Erstes Kommando mit Gewicht > 0 und allen Arm-kp = 0 (NOT-HALT-Slack)."""
    for r in rows:
        if r[1] > 0.5 and max(r[3]) == 0.0:
            return r[0]
    return None


def first_waist_passive(rows):
    for r in rows:
        if r[1] > 0.5 and max(r[3]) == 0.0 and max(r[4]) == 0.0:
            return r[0]
    return None


def standstill(rows, eps=STILL_EPS, win=STILL_WIN):
    """Beginn der ersten Spanne >= win ohne Kommando-Aenderung > eps; None = keine."""
    if len(rows) < 2:
        return None
    start = None
    for a, b in zip(rows, rows[1:]):
        moved = max(abs(x - y) for x, y in zip(a[2], b[2])) > eps
        if moved:
            start = None
            continue
        if start is None:
            start = a[0]
        if b[0] - start >= win:
            return start
    return None


def moved_since(rows, eps=STILL_EPS):
    return any(max(abs(x - y) for x, y in zip(a[2], b[2])) > eps for a, b in zip(rows, rows[1:]))


# ══════════════════════════════════════════════════════════════════════
# Vorwaertskinematik (gleiches Modell wie der arm_controller)
# ══════════════════════════════════════════════════════════════════════
class HandFK:
    def __init__(self, urdf):
        from v6_armplaner import Model
        import numpy as np
        import pinocchio as pin
        self.np, self.pin = np, pin
        self.ik = Model(urdf).ik
        self.data = self.ik.model.createData()
        self.idx = [(i, self.ik._name_to_q_index[n]) for i, n in enumerate(self.ik._ros_joint_names)
                    if n in self.ik._name_to_q_index]

    def pos(self, q14, side, waist=None):
        np, pin = self.np, self.pin
        full = np.zeros(29)
        full[15:29] = q14
        if waist is not None:
            full[12:15] = waist
        q = pin.neutral(self.ik.model)
        for i, k in self.idx:
            if i < 29:
                q[k] = full[i]
        pin.forwardKinematics(self.ik.model, self.data, q)
        pin.updateFramePlacements(self.ik.model, self.data)
        fid = self.ik._fid_left if side == "left" else self.ik._fid_right
        return np.array(self.data.oMf[fid].translation)


def hand_speeds(fk, rows, side, window=0.02):
    """-> (v_max je Kommando [m/s], v_max ueber 20-ms-Fenster [m/s])."""
    import numpy as np
    if len(rows) < 2:
        return None, None
    t = np.array([r[0] for r in rows])
    p = np.array([fk.pos(np.array(r[2]), side) for r in rows])
    dt = np.diff(t)
    ok = dt > 1e-6
    v_raw = (np.linalg.norm(np.diff(p, axis=0), axis=1)[ok] / dt[ok]).max() if ok.any() else None
    v_win, j = 0.0, 0
    for i in range(len(t)):
        while j < len(t) and t[j] - t[i] < window:
            j += 1
        if j >= len(t):
            break
        v_win = max(v_win, float(np.linalg.norm(p[j] - p[i]) / (t[j] - t[i])))
    return (float(v_raw) if v_raw is not None else None), v_win


# ══════════════════════════════════════════════════════════════════════
# Ziele
# ══════════════════════════════════════════════════════════════════════
class Base:
    sim = False

    def __init__(self, a):
        self.a = a
        self.writes = Writes()
        self.cmd_pose = {}
        self.full_estop_t = []
        self.teleop = None

    # ROS-Seite (gemeinsam)
    def _ros(self):
        from std_msgs.msg import Bool
        from sensor_msgs.msg import Joy
        from geometry_msgs.msg import PoseStamped
        self.Bool, self.Joy, self.PoseStamped = Bool, Joy, PoseStamped
        n = self.inj
        self.p_estop = n.create_publisher(Bool, "/g1pilot/emergency_stop", 10)
        self.p_full = n.create_publisher(Bool, "/g1pilot/full_emergency_stop", 10)
        self.p_start = n.create_publisher(Bool, "/g1pilot/start", 10)
        self.p_enable = n.create_publisher(Bool, "/g1pilot/arms/enabled", 10)
        self.p_balance = n.create_publisher(Bool, "/g1pilot/start_balancing", 10)
        self.p_joy = n.create_publisher(Joy, "/g1pilot/ps4/joy", 10)
        from std_msgs.msg import String
        self.String = String
        self.p_cmd = n.create_publisher(String, "/g1pilot/arm_command", 10)
        self.status = []
        n.create_subscription(String, "/g1pilot/arm_command/status",
                              lambda m: self.status.append((time.monotonic(), m.data)), 50)
        self.p_goal = {s: n.create_publisher(PoseStamped, f"/g1pilot/hand_goal/{s}", 10)
                       for s in ("left", "right")}
        for s in ("left", "right"):
            n.create_subscription(PoseStamped, f"/g1pilot/hand_cmd_pose/{s}",
                                  lambda m, s=s: self.cmd_pose.__setitem__(s, (time.monotonic(), m)), 10)
        n.create_subscription(Bool, "/g1pilot/full_emergency_stop",
                              lambda m: self.full_estop_t.append(time.monotonic()) if m.data else None, 10)

    def pub_bool(self, pub, v=True):
        t = time.monotonic()
        pub.publish(self.Bool(data=bool(v)))
        return t

    def joy(self, buttons=(), ly=0.0):
        m = self.Joy()
        m.header.stamp = self.inj.get_clock().now().to_msg()
        m.axes = [0.0] * NUM_AXES
        m.axes[LY] = float(ly)
        m.buttons = [0] * NUM_BUTTONS
        for b in buttons:
            m.buttons[b] = 1
        t = time.monotonic()
        self.p_joy.publish(m)
        return t

    def joy_stream(self, seconds, buttons=(), ly=0.0, hz=50.0, stop=None):
        t_end = time.monotonic() + seconds
        t_last = None
        while time.monotonic() < t_end:
            t_last = self.joy(buttons, ly)
            if stop is not None and stop():
                break
            time.sleep(1.0 / hz)
        return t_last

    def goal(self, side, pos, quat):
        m = self.PoseStamped()
        m.header.frame_id = "pelvis"
        m.header.stamp = self.inj.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(x) for x in pos)
        o = m.pose.orientation
        o.x, o.y, o.z, o.w = (float(x) for x in quat)
        t = time.monotonic()
        self.p_goal[side].publish(m)
        return t

    def is_ready(self):
        r = self.writes.since(time.monotonic() - 0.3)
        return len(r) > 10 and all(x[1] >= 0.999 and max(x[3]) > 0.0 for x in r)

    def wait_ready(self, timeout=30.0):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if self.is_ready():
                return True
            time.sleep(0.05)
        return False

    def reset(self, timeout=30.0):
        """NOT-HALT quittieren (START) und Arme wieder uebernehmen."""
        self.pub_bool(self.p_start)
        time.sleep(0.1)
        self.enable()
        t_end = time.monotonic() + timeout
        t_retry = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            if self.is_ready():
                return True
            time.sleep(0.1)
            if time.monotonic() > t_retry:      # Befehl verloren? erneut
                t_retry = time.monotonic() + 3.0
                self.pub_bool(self.p_start)
                self.enable()
        return False

    def enable(self):
        self.pub_bool(self.p_enable)

    def current_pose(self, side, max_age=1.0):
        v = self.cmd_pose.get(side)
        if v is None or time.monotonic() - v[0] > max_age:
            return None
        p, o = v[1].pose.position, v[1].pose.orientation
        return [p.x, p.y, p.z], [o.x, o.y, o.z, o.w]

    def start_teleop(self, env):
        if self.teleop is not None:
            return
        cmd = [sys.executable, "-c",
               "import sys; sys.path.insert(0, %r); "
               "from g1pilot.teleoperation.ps4_arm_teleop import main; main()" % ec.PKG_ROOT]
        self.teleop_log = tempfile.NamedTemporaryFile("w+", suffix="_teleop.log", delete=False)
        self.teleop = subprocess.Popen(cmd, env=env, stdout=self.teleop_log, stderr=subprocess.STDOUT)
        time.sleep(4.0)

    def close(self):
        if self.teleop is not None:
            self.teleop.terminate()
            try:
                self.teleop.wait(5)
            except Exception:
                self.teleop.kill()


class Attrappe(Base):
    """arm_controller (Realbetrieb-Code) gegen FakeG1, alles in diesem Prozess."""

    def __init__(self, a):
        super().__init__(a)
        os.environ["G1_SIM_MODE"] = "false"
        os.environ["ROS_DOMAIN_ID"] = str(a.domain)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["G1_POSE_STORE"] = os.path.join(tempfile.mkdtemp(prefix="v8_"), "poses.json")
        os.environ["ROS_PACKAGE_PATH"] = os.path.dirname(ec.PKG_ROOT)
        sys.path.insert(0, ec.PKG_ROOT)
        spec = importlib.util.spec_from_file_location(
            "v8_fake_g1", os.path.join(ec.PKG_ROOT, "test", "test_arm_controller_real.py"))
        tm = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tm)
        tm._install_unitree_stubs()
        self.tm = tm
        import ament_index_python.packages as aip
        aip.get_package_share_directory = lambda name: ec.PKG_ROOT
        from g1pilot.manipulation import arm_controller as ac
        ac.init_dds = lambda *x, **k: None
        orig = tm.ROBOT.write

        def write(msg):
            self.writes.add(time.monotonic(), msg)
            orig(msg)
        tm.ROBOT.write = write
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        args = ["--ros-args"] + sum((["-p", f"{k}:={str(v).lower() if isinstance(v, bool) else v}"]
                                     for k, v in REAL_PARAMS.items()), [])
        rclpy.init(args=args)
        self.node = ac.ArmController()           # eigener 250-Hz-Timer bleibt
        self.inj = rclpy.create_node("eval_v8_injector")
        self._ros()
        self.ex = MultiThreadedExecutor(num_threads=4)
        self.ex.add_node(self.node)
        self.ex.add_node(self.inj)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        print("[v8] Attrappe: warte auf Planer-/IK-Prozess ...", flush=True)
        t_end = time.monotonic() + 180
        while not self.node._planner.ready and time.monotonic() < t_end:
            time.sleep(0.2)
        self.env = dict(os.environ)

    def calibrate(self, side):
        """Erstes Hand-Ziel = aktuelle Hand (wie der RViz-Marker), damit
        hand_cmd_pose kommt. Liest die Pose ueber das Modell des Knotens."""
        M = self.node._fk_current_ee(side)
        if M is None:
            return False
        import pinocchio as pin
        q = pin.Quaternion(M.rotation)
        self.goal(side, M.translation, [q.x, q.y, q.z, q.w])
        return True

    def freeze(self, on):
        self.tm.ROBOT.freeze = bool(on)

    def close(self):
        super().close()
        try:
            self.ex.shutdown()
            self.node.destroy_node()
            self.inj.destroy_node()
            import rclpy
            rclpy.shutdown()
        except Exception:
            pass


class Sim(Base):
    sim = True

    def __init__(self, a):
        super().__init__(a)
        import eval_ros as er
        er.assert_sim("v8_nothalt.py --ziel sim")
        self.tap = er.DdsTap(a.interface)
        self.tap.subscribe("rt/arm_sdk", lambda t, m: self.writes.add(t, m), 1000)
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        if not rclpy.ok():
            rclpy.init()
        self.inj = rclpy.create_node("eval_v8_injector")
        self._ros()
        from tf2_ros import Buffer, TransformListener
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self.inj)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.inj)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        self.env = None

    def enable(self):
        self.pub_bool(self.p_enable)
        self.pub_bool(self.p_balance)

    def calibrate(self, side):
        import rclpy
        try:
            tr = self.tf.lookup_transform("pelvis", f"{side}_hand_point_contact", rclpy.time.Time())
        except Exception:
            return False
        p, o = tr.transform.translation, tr.transform.rotation
        self.goal(side, [p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
        return True

    def close(self):
        super().close()
        self.tap.close()


# ══════════════════════════════════════════════════════════════════════
# Faelle (je Wiederholung ein dict)
# ══════════════════════════════════════════════════════════════════════
def go_start(tg, timeout=20.0):
    """Beide Arme geplant in START_POSE (arm_command joints), warten bis 'reached'."""
    import json
    rid = f"v8_start_{time.monotonic():.3f}"
    t0 = time.monotonic()
    tg.p_cmd.publish(tg.String(data=json.dumps({"id": rid, "type": "joints", **START_POSE})))
    t_end = t0 + timeout
    while time.monotonic() < t_end:
        for t, d in list(tg.status):
            if t >= t0 and rid in d:
                st = json.loads(d).get("state")
                if st in ("reached", "failed", "rejected", "cancelled"):
                    time.sleep(0.3)
                    return st == "reached"
        time.sleep(0.05)
    return False


def ensure_pose(tg, side="left", timeout=5.0):
    """Aktuelle kommandierte Handpose (nach Kalibrier-Ziel)."""
    p = tg.current_pose(side)
    if p is not None:
        return p
    tg.calibrate(side)
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        p = tg.current_pose(side)
        if p is not None:
            return p
        time.sleep(0.05)
    return None


def case_nothalt_topic(tg, rng, a):
    time.sleep(rng.uniform(0.0, 0.02))          # zufaellige Phase zum 250-Hz-Takt
    t0 = tg.pub_bool(tg.p_estop)
    time.sleep(0.3)
    t1 = first_slack(tg.writes.since(t0))
    return {"ausgeloest_s": t0, "latenz_ms": None if t1 is None else (t1 - t0) * 1000.0}


def case_nothalt_ps(tg, rng, a):
    tg.joy_stream(0.2)                             # Controller "verbunden", neutral
    time.sleep(rng.uniform(0.0, 0.02))
    t0 = tg.joy((PS,))
    tg.joy_stream(0.1, (PS,))
    tg.joy_stream(0.1)
    time.sleep(0.2)
    t1 = first_slack(tg.writes.since(t0))
    return {"ausgeloest_s": t0, "latenz_ms": None if t1 is None else (t1 - t0) * 1000.0}


def case_vollnotaus(tg, rng, a):
    tg.joy_stream(0.2)
    n_full = len(tg.full_estop_t)
    time.sleep(rng.uniform(0.0, 0.02))
    t0 = tg.joy((R1, L1, PS))
    tg.joy_stream(a.vollnotaus_halten + 0.6, (R1, L1, PS))
    tg.joy_stream(0.1)
    time.sleep(0.2)
    rows = tg.writes.since(t0)
    t_slack = first_slack(rows)
    t_waist = first_waist_passive(rows)
    t_topic = tg.full_estop_t[n_full] if len(tg.full_estop_t) > n_full else None
    hold = a.vollnotaus_halten
    ms = lambda t: None if t is None else (t - t0) * 1000.0   # noqa: E731
    return {"ausgeloest_s": t0, "latenz_ms": ms(t_slack),
            # Netto = Voll-Notaus-Wirkung (Taille schlaff, Attrappe) bzw. Topic (Sim) minus Haltezeit
            "netto_ms": (None if (t_waist or t_topic) is None
                         else ms(t_waist if t_waist is not None else t_topic) - hold * 1000.0),
            "bis_topic_ms": ms(t_topic), "bis_taille_schlaff_ms": ms(t_waist)}


def case_ausfall_joy(tg, rng, a):
    # Jog-Bezug = kommandierte Hand (hand_cmd_pose) -- fuer beide Seiten, die
    # Hand-Auswahl des Controllers (Standard: rechts) bleibt unveraendert.
    if ensure_pose(tg, "left") is None or ensure_pose(tg, "right") is None:
        return {"fehler": "keine Handpose (hand_cmd_pose)"}
    tg.joy_stream(0.2)
    t_s = time.monotonic()
    t_last = tg.joy_stream(a.jog_dauer + rng.uniform(0.0, 0.1), ly=1.0)   # linker Stick vor
    time.sleep(max(3.0, a.halt_timeout))
    rows = tg.writes.since(t_s)
    moved = moved_since([r for r in rows if r[0] <= t_last])
    t_halt = standstill([r for r in rows if r[0] >= t_last])
    res = {"ausgeloest_s": t_last, "bewegt_vor_ausfall": moved,
           "latenz_ms": None if (t_halt is None or not moved) else (t_halt - t_last) * 1000.0}
    # Stick zurueck in die Mitte (sonst laeuft der Jog beim naechsten Joy weiter)
    tg.joy_stream(0.3)
    return res


def case_ausfall_ziel(tg, rng, a):
    import numpy as np
    pose = ensure_pose(tg)
    if pose is None:
        return {"fehler": "keine Handpose (hand_cmd_pose)"}
    p0, quat = np.array(pose[0]), pose[1]
    d = np.array([0.0, rng.choice([-1.0, 1.0]), 0.0]) * a.ziel_tempo   # seitlich hin/her
    t_s = time.monotonic()
    hz, n = 30.0, int((a.jog_dauer + rng.uniform(0.0, 0.1)) * 30.0)
    t_last = None
    for i in range(1, n + 1):
        t_last = tg.goal("left", p0 + d * (i / hz), quat)
        time.sleep(1.0 / hz)
    time.sleep(max(3.0, a.halt_timeout))
    rows = tg.writes.since(t_s)
    moved = moved_since([r for r in rows if r[0] <= t_last])
    t_halt = standstill([r for r in rows if r[0] >= t_last])
    return {"ausgeloest_s": t_last, "bewegt_vor_ausfall": moved,
            "latenz_ms": None if (t_halt is None or not moved) else (t_halt - t_last) * 1000.0}


def case_ausfall_zustand(tg, rng, a):
    import json
    from std_msgs.msg import String
    side = rng.choice(["left", "right"])
    goal = {"left": [-0.6, 0.3, 0.0, 0.7, 0.0, 0.0, 0.0],
            "right": [-0.6, -0.3, 0.0, 0.7, 0.0, 0.0, 0.0]}
    home = {"left": [0.1, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0], "right": [0.1, -0.3, 0.0, 1.0, 0.0, 0.0, 0.0]}
    last = tg.writes.last()
    q_now = last[2] if last else None
    sl = slice(0, 7) if side == "left" else slice(7, 14)
    tgt = goal if (q_now is None or abs(q_now[sl][0] - goal[side][0]) > 0.3) else home
    cmd = {"id": f"v8_{time.monotonic():.3f}", "type": "joints", side: tgt[side]}
    tg.node._on_arm_command(String(data=json.dumps(cmd)))
    # warten, bis die Fahrt laeuft
    t_end = time.monotonic() + 10.0
    while time.monotonic() < t_end and not tg.node._planned_motion_active:
        time.sleep(0.01)
    if not tg.node._planned_motion_active:
        return {"fehler": "geplante Fahrt startete nicht"}
    time.sleep(rng.uniform(0.2, 0.6))
    t0 = time.monotonic()
    tg.freeze(True)
    time.sleep(max(8.0, a.halt_timeout))
    rows = tg.writes.since(t0)
    t_halt = standstill(rows)
    tg.freeze(False)
    # Fahrt zu Ende kommen lassen
    t_end = time.monotonic() + 15.0
    while time.monotonic() < t_end and tg.node._planned_motion_active:
        time.sleep(0.05)
    return {"ausgeloest_s": t0, "latenz_ms": None if t_halt is None else (t_halt - t0) * 1000.0,
            "hinweis": "Halt = Kommandos stehen (auch Fahrtende oder Schleppfehler-Klemme)"}


def case_zielsprung(tg, rng, a, fk=None):
    import numpy as np
    pose = ensure_pose(tg)
    if pose is None:
        return {"fehler": "keine Handpose (hand_cmd_pose)"}
    p0, quat = np.array(pose[0]), pose[1]
    v = np.array([rng.gauss(0, 1) for _ in range(3)])
    v /= np.linalg.norm(v)
    tgt = p0 + v * a.sprung
    t0 = tg.goal("left", tgt, quat)
    time.sleep(a.sprung_dauer)
    rows = tg.writes.since(t0)
    tg.goal("left", p0, quat)                 # zurueck an den Ausgangspunkt
    time.sleep(a.sprung_dauer)
    v_raw, v_win = hand_speeds(fk, rows, "left")
    reached = None
    if rows:
        reached = float(np.linalg.norm(fk.pos(np.array(rows[-1][2]), "left")
                                       - fk.pos(np.array(rows[0][2]), "left")))
    return {"ausgeloest_s": t0, "richtung": [float(x) for x in v],
            "v_hand_max_mps": v_raw, "v_hand_max_20ms_mps": v_win,
            "weg_hand_m": reached, "n_kommandos": len(rows)}


CASES = {"nothalt_topic": case_nothalt_topic, "nothalt_ps": case_nothalt_ps,
         "vollnotaus": case_vollnotaus, "ausfall_joy": case_ausfall_joy,
         "ausfall_ziel": case_ausfall_ziel, "ausfall_zustand": case_ausfall_zustand,
         "zielsprung": case_zielsprung}
NEEDS_RESET = ("nothalt_topic", "nothalt_ps", "vollnotaus")
NEEDS_TELEOP = ("nothalt_ps", "vollnotaus", "ausfall_joy")
NEEDS_START = ("ausfall_joy", "ausfall_ziel", "zielsprung")


def main(argv=None):
    ap = argparse.ArgumentParser(description="V8: Not-Halt, Ausfall und Zielsprung (Attrappe / Sim).")
    ap.add_argument("--ziel", choices=("attrappe", "sim"), required=True)
    ap.add_argument("--faelle", nargs="+", choices=FAELLE, default=None)
    ap.add_argument("--wdh", type=int, default=100, help="Wiederholungen je Fall (Default 100)")
    ap.add_argument("--seed", type=int, default=8)
    ap.add_argument("--domain", type=int, default=int(os.getenv("G1_EVAL_DOMAIN", "87")),
                    help="ROS_DOMAIN_ID der Attrappe (abgeschottet, nur localhost)")
    ap.add_argument("--interface", default=os.getenv("ROBOT_INTERFACE", "lo"))
    ap.add_argument("--vollnotaus-halten", type=float, default=2.0,
                    help="Haltezeit R1+L1+PS laut ps4_arm_teleop (full_estop_hold_s)")
    ap.add_argument("--jog-dauer", type=float, default=1.0, help="Bewegung vor dem Ausfall [s]")
    ap.add_argument("--ziel-tempo", type=float, default=0.10, help="Tempo des Ziel-Stroms [m/s]")
    ap.add_argument("--halt-timeout", type=float, default=3.0, help="so lange auf Halt warten [s]")
    ap.add_argument("--sprung", type=float, default=0.15, help="Zielsprung [m]")
    ap.add_argument("--sprung-dauer", type=float, default=3.0, help="Beobachtung je Sprung [s]")
    ap.add_argument("--urdf", default=os.path.join(ec.PKG_ROOT, "description_files", "urdf",
                                                   "g1_29dof_inspire_ftp.urdf"))
    ap.add_argument("--no-plot", action="store_true")
    ec.add_common_args(ap, VERSUCH)
    a = ap.parse_args(argv)
    faelle = a.faelle or [f for f in FAELLE if a.ziel == "attrappe" or f not in NUR_ATTRAPPE]
    bad = [f for f in faelle if a.ziel == "sim" and f in NUR_ATTRAPPE]
    if bad:
        ap.error(f"{bad} nur mit --ziel attrappe")
    if a.ziel == "sim" and ec.env_mode() != ec.MODE_SIM:
        ap.error("--ziel sim nur in der Simulation (G1_SIM_MODE=true)")

    mode = ec.MODE_FAKE if a.ziel == "attrappe" else ec.MODE_SIM
    params = {"ziel": a.ziel, "faelle": faelle, "wdh": a.wdh, "seed": a.seed,
              "vollnotaus_halten_s": a.vollnotaus_halten, "jog_dauer_s": a.jog_dauer,
              "ziel_tempo_mps": a.ziel_tempo, "sprung_m": a.sprung, "sprung_dauer_s": a.sprung_dauer,
              "halt_kriterium": {"eps_rad_je_kommando": STILL_EPS, "fenster_s": STILL_WIN},
              "urdf": os.path.relpath(a.urdf, ec.PKG_ROOT)}
    if a.ziel == "attrappe":
        params["arm_controller_parameter"] = REAL_PARAMS
        params["ros_domain"] = a.domain
    else:
        params["umgebung"] = {k: os.getenv(k, "") for k in ("HOLD_BASE_MODE", "SIM_LOCKSTEP",
                                                           "G1_PS4_ARMS", "G1_HANDS")}
    run = ec.Run("v8_nothalt.py", VERSUCH, mode, params, out_root=a.out)
    rng = random.Random(a.seed)
    tg = Attrappe(a) if a.ziel == "attrappe" else Sim(a)
    fk = HandFK(a.urdf) if "zielsprung" in faelle else None
    rows, summary = [], {"faelle": {}}
    try:
        if a.ziel == "attrappe" and any(f in NEEDS_TELEOP for f in faelle):
            tg.start_teleop(tg.env)
        if not tg.reset(60.0):
            raise SystemExit("Arme lassen sich nicht uebernehmen (kein Gewicht 1 / kp > 0).")
        for fall in faelle:
            print(f"[v8] {fall}: {a.wdh} Wiederholungen ...", flush=True)
            res = []
            for i in range(a.wdh):
                if not tg.is_ready() and not tg.reset(30.0):
                    res.append({"fehler": "nicht bereit (Arme nicht uebernommen)"})
                    continue
                if fall in NEEDS_START and not go_start(tg):
                    res.append({"fehler": "Ausgangslage nicht erreicht"})
                    continue
                time.sleep(0.2)
                fn = CASES[fall]
                r = fn(tg, rng, a, fk) if fall == "zielsprung" else fn(tg, rng, a)
                res.append(r)
                if fall in NEEDS_RESET:
                    tg.reset(30.0)
                if (i + 1) % 10 == 0:
                    print(f"[v8] {fall}: {i + 1}/{a.wdh}", flush=True)
            keys = sorted({k for r in res for k in r if k not in ("ausgeloest_s", "richtung")})
            for i, r in enumerate(res):
                rows.append([fall, i + 1] + [r.get(k) for k in
                                             ("latenz_ms", "netto_ms", "bis_topic_ms", "bis_taille_schlaff_ms",
                                              "v_hand_max_mps", "v_hand_max_20ms_mps", "weg_hand_m",
                                              "bewegt_vor_ausfall", "fehler")])
            fs = {"n": len(res), "fehler": sum(1 for r in res if r.get("fehler")),
                  "ohne_antwort": sum(1 for r in res if not r.get("fehler") and "latenz_ms" in r
                                      and r["latenz_ms"] is None)}
            for k in ("latenz_ms", "netto_ms", "bis_topic_ms", "bis_taille_schlaff_ms",
                      "v_hand_max_mps", "v_hand_max_20ms_mps", "weg_hand_m"):
                vals = [r[k] for r in res if r.get(k) is not None]
                if vals:
                    fs[k] = ec.describe(vals)
            if fall == "zielsprung":
                lim = REAL_PARAMS["ee_velocity_limit"] if a.ziel == "attrappe" else None
                fs["ee_velocity_limit_mps"] = lim
                if lim:
                    fs["anteil_ueber_grenze_20ms"] = (sum(1 for r in res if (r.get("v_hand_max_20ms_mps") or 0) > lim)
                                                      / max(1, len(res)))
            summary["faelle"][fall] = fs
            med = fs.get("latenz_ms", fs.get("v_hand_max_20ms_mps", {})).get("median")
            print(f"[v8] {fall}: Median {med}", flush=True)
    finally:
        tg.close()
    run.write_csv("wiederholungen.csv",
                  ["fall", "wdh", "latenz_ms", "netto_ms", "bis_topic_ms", "bis_taille_schlaff_ms",
                   "v_hand_max_mps", "v_hand_max_20ms_mps", "weg_hand_m", "bewegt_vor_ausfall", "fehler"],
                  rows, "je Wiederholung; latenz = Ausloesen/Ausfall bis passendes Kommando an rt/arm_sdk")
    summary["hinweise"] = [
        "Handy und Demo-GUI senden denselben Bool auf /g1pilot/emergency_stop (nothalt_topic); "
        "die Strecke Handy->Bluetooth->GUI ist nicht enthalten.",
        "Voll-Notaus: Beine (Damp ueber loco_client bzw. loco_sim) sind nicht Teil der Messung, "
        "nur rt/arm_sdk und das Topic.",
        "Attrappe: Antwortzeit enthaelt ROS-Zustellung im selben Rechner und den 250-Hz-Takt "
        "des arm_controller, nicht die Strecke zum Roboter.",
    ]
    if not a.no_plot:
        try:
            plot(run, rows)
        except Exception as e:      # noqa: BLE001
            summary["diagramm_fehler"] = str(e)
    run.finish(summary)
    return 0


def plot(run, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    by = {}
    for r in rows:
        if r[2] is not None:
            by.setdefault(r[0], []).append(r[2])
    sp = [r[7] for r in rows if r[0] == "zielsprung" and r[7] is not None]
    with PdfPages(run.path("diagramme.pdf")) as pdf:
        if by:
            fig, ax = plt.subplots(figsize=(10, 4.5))
            labels = list(by)
            ax.boxplot([by[k] for k in labels], whis=(0, 100))
            ax.set_xticks(range(1, len(labels) + 1))
            ax.set_xticklabels(labels, rotation=15)
            ax.set_ylabel("Antwortzeit [ms] (Whisker = Min/Max)")
            ax.set_title("V8 Not-Halt / Ausfall bis geaendertes Kommando")
            fig.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)
        if sp:
            fig, ax = plt.subplots(figsize=(8, 4))
            ax.hist(sp, bins=30)
            ax.set_xlabel("groesste Handgeschwindigkeit (20-ms-Fenster) [m/s]")
            ax.set_title("V8 Zielsprung")
            fig.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)
    run.files.append("diagramme.pdf")


if __name__ == "__main__":
    sys.exit(main())

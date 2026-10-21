#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Controller-Jog im ECHT-Pfad: Bezug kommandierte statt gemessene Hand.

Nils (2026-10-21): mit PS4/Xbox kommt fast nie die gewollte Bewegung raus --
hoch/runter dreht die Hand mit, nach innen drehen sich die Haende, gespiegelte
Bewegungen aus der Grundstellung enden nicht an der gleichen Stelle. Der
RViz-Marker dagegen faehrt sauber in einer Achse.

Unterschied: der Marker gibt ein Ziel vor, der arm_controller bringt die
KOMMANDIERTE Hand genau dorthin. Der Jog bezog sich real (jog_reference=tf)
auf die GEMESSENE Hand: jede Fahrt startete dort (Durchhang/Reibung wurden
als neue Soll-Lage uebernommen -> Ratsche), Vorlauf und Anhalten hingen am
Schleppfehler des jeweiligen Arms (-> links/rechts verschieden).

Hier laeuft der echte arm_controller (G1_SIM_MODE=false) gegen den Fake-G1
aus test_arm_controller_real, diesmal MIT bleibendem Regelfehler je Gelenk
(Durchhang) und Haftreibung, links staerker als rechts. Davor sitzt dieselbe
Jog-Rechnung wie in ps4_arm_teleop._tick/_hold (hand_jog: JogReference,
jog_step, hold_pose, HoldStreamer); die kommandierte Hand kommt vom echten
/g1pilot/hand_cmd_pose-Publisher des Nodes, die gemessene per FK der Messung
(wie TF). Verglichen werden jog_reference "tf" (alter Real-Stand) und
"command" (neu). Ohne Hardware -- am Roboter ungetestet.
"""
import math
import time

import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
rclpy = pytest.importorskip("rclpy")

from test_arm_controller_real import (  # noqa: E402,F401
    ac_module, _make_node, _close, _reset_robot, _bool, ROBOT, DT)
from g1pilot.teleoperation import hand_jog as hj  # noqa: E402

RATE = 30.0                       # Takt von ps4_arm_teleop
# Regelfehler (rad) je Gelenk [sh_pitch, sh_roll, sh_yaw, elbow, wr_roll, wr_pitch, wr_yaw]
SAG_R = np.array([0.02, 0.0, 0.0, 0.03, 0.0, 0.06, 0.0])
SAG_L = np.array([0.03, 0.01, 0.0, 0.05, 0.02, 0.09, 0.01])
FRIC_R = np.array([0.006, 0.006, 0.006, 0.006, 0.01, 0.01, 0.01])
FRIC_L = 1.8 * FRIC_R
# Arbeitslage: Unterarme nach vorn, Haende vor der Brust (Reichweite ~77 %).
# NICHT die Onboard-/Lauf-Pose des Fake-G1: dort haengen die Arme fast
# gestreckt an der Reichweitengrenze (IK klemmt das Ziel auf die Kugel um die
# Schulter) -- das waere ein anderer Effekt als der hier gepruefte.
WORK_R = [-0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
WORK_L = [-0.4, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0]


def _wait_ik(node, timeout=120.0):
    t_end = time.monotonic() + timeout
    while not (node._ik_proc is not None and node._ik_proc.ready) and time.monotonic() < t_end:
        time.sleep(0.1)
    assert node._ik_proc is not None and node._ik_proc.ready, "IK-Prozess nicht bereit"


class Jogger:
    """ps4_arm_teleop._tick/_hold ohne ROS, gegen den Node im selben Prozess."""

    def __init__(self, node, mode):
        self.node = node
        self.ref = hj.JogReference(mode)
        self.cmd = hj.CommandedPoses()
        self.holds = hj.HoldStreamer()
        self.targets, self.twist = {}, {}
        self.data = pin.Data(node.ik_solver.model)
        v, w = hj.JOG_SPEEDS["Normal"]
        self.lead = hj.lead_limits(v, w)
        for side in ("left", "right"):
            pub = node.pub_hand_cmd_pose[side]
            pub.publish = lambda m, s=side: self._on_cmd(s, m)

    def _on_cmd(self, side, m):
        p, q = m.pose.position, m.pose.orientation
        self.cmd.update(side, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), time.monotonic())

    def measured(self, side):
        """Gemessene Hand wie TF (robot_state: FK der gemessenen Gelenke)."""
        node, ik = self.node, self.node.ik_solver
        full = node._full_config_with_arms(ROBOT.measured())
        q = ik._q_full_from(None, full)
        M = ik._frame_placement(side, q, self.data)
        qq = pin.Quaternion(M.rotation)
        return list(M.translation), (qq.x, qq.y, qq.z, qq.w)

    def _goal(self, side, pose):
        from geometry_msgs.msg import PoseStamped
        ps = PoseStamped()
        ps.header.frame_id = "pelvis"
        ps.header.stamp = self.node.get_clock().now().to_msg()
        p, q = pose
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
        o = ps.pose.orientation
        o.x, o.y, o.z, o.w = map(float, q)
        (self.node._left_goal_callback if side == "left"
         else self.node._right_goal_callback)(ps)

    def _pose(self, side, new):
        return self.ref.pose(side, self.cmd.get(side, time.monotonic()),
                             self.measured(side), new)

    def run(self, seconds, moves=None):
        """moves: {side: lin} solange gehalten, danach loslassen + halten."""
        moves = moves or {}
        t_end = time.monotonic() + seconds
        t_tel = 0.0
        while time.monotonic() < t_end:
            now = time.monotonic()
            if now >= t_tel:
                t_tel = now + 1.0 / RATE
                for side in [s for s in self.targets if s not in moves]:
                    _, hand = self._pose(side, False)
                    self.ref.end(side)
                    pose = hj.hold_pose(self.targets.pop(side), hand,
                                        self.twist.pop(side), (0.0, 0.0, 0.0))
                    self.holds.hold(side, pose, now)
                for side, pose in self.holds.due(now).items():
                    if side not in moves:
                        self._goal(side, pose)
                for side, lin in moves.items():
                    self.holds.cancel(side)
                    start, hand = self._pose(side, side not in self.targets)
                    tgt = hj.jog_step(self.targets.get(side), hand, lin, (0.0, 0.0, 0.0),
                                      1.0 / RATE, *self.lead, start=start)
                    self.targets[side], self.twist[side] = tgt, lin
                    self._goal(side, tgt)
            self.node.main_loop()
            time.sleep(DT)

    def tap(self, moves, move_s=0.4, rest_s=1.4):
        self.run(move_s, moves)
        self.run(rest_s)


def _start(ac_module, mode):
    _reset_robot()
    from test_arm_controller_real import ARM_IDX
    for k, j in enumerate(ARM_IDX):
        ROBOT.state.motor_state[j].q = float((WORK_L + WORK_R)[k])
    node = _make_node(ac_module)
    _wait_ik(node)
    node._arms_controlled_callback(_bool(True))
    jog = Jogger(node, mode)
    jog.run(2.4)
    ROBOT.sag = np.concatenate([SAG_L, SAG_R])
    ROBOT.friction = np.concatenate([FRIC_L, FRIC_R])
    jog.run(1.0)
    # Erstes Ziel = gemessene Hand (wie der erste Controller-Kontakt): die
    # Auto-Kalibrierung des arm_controller laeuft, danach gibt es hand_cmd_pose.
    for side in ("left", "right"):
        jog.tap({side: (0.0, 0.0, 0.0)}, move_s=0.1, rest_s=0.1)
    jog.run(1.2)
    return node, jog


def _ang(q0, q1):
    return math.degrees(hj.qangle(q0, q1))


def _taps(ac_module, mode):
    node, jog = _start(ac_module, mode)
    try:
        m0 = jog.measured("right")
        for _ in range(4):
            jog.tap({"right": (0.0, 0.0, 0.15)})
            jog.tap({"right": (0.0, 0.0, -0.15)})
        m1 = jog.measured("right")
        return _ang(m0[1], m1[1]), math.dist(m0[0], m1[0])
    finally:
        _close(node)


def test_taps_up_down_no_tilt_ratchet(ac_module):
    """4x hoch + 4x runter tippen (je 0.4 s, Normal): kommandiert bezogen
    kehrt die Hand ohne Kippen zurueck; an der gemessenen Hand bezogen
    uebernimmt jede Fahrt den Durchhang erneut."""
    ori_cmd, pos_cmd = _taps(ac_module, "command")
    ori_tf, pos_tf = _taps(ac_module, "tf")
    print(f"\nTippen 8x: command Kippen {ori_cmd:.1f} deg / Rueckkehr {pos_cmd * 100:.1f} cm"
          f" | tf {ori_tf:.1f} deg / {pos_tf * 100:.1f} cm")
    assert ori_cmd < 1.5 and pos_cmd < 0.012
    assert ori_tf > 3.0 * max(ori_cmd, 1.0)


def _mirror(ac_module, mode):
    node, jog = _start(ac_module, mode)
    try:
        c0 = {s: jog.cmd.get(s, time.monotonic()) for s in ("left", "right")}
        m0 = {s: jog.measured(s) for s in ("left", "right")}
        for lin in ((0.0, 0.15, 0.0), (0.0, 0.0, 0.15), (0.15, 0.0, 0.0),
                    (0.0, -0.15, 0.0), (0.0, 0.0, -0.15)):
            # Referenz rechts, links gespiegelt (ArmTeleopLogic.twist "mirror")
            jog.tap({"right": lin, "left": (lin[0], -lin[1], lin[2])})
        out = {}
        for name, a0, a1 in (("cmd", c0, {s: jog.cmd.get(s, time.monotonic())
                                          for s in ("left", "right")}),
                             ("meas", m0, {s: jog.measured(s) for s in ("left", "right")})):
            dr = np.subtract(a1["right"][0], a0["right"][0])
            dl = np.subtract(a1["left"][0], a0["left"][0])
            out[name] = float(np.linalg.norm(dr - dl * np.array([1.0, -1.0, 1.0])))
            out[name + "_ori"] = max(_ang(a0[s][1], a1[s][1]) for s in ("left", "right"))
        return out
    finally:
        _close(node)


def test_mirrored_moves_end_symmetric(ac_module):
    """Gespiegelt (innen, hoch, vor, aussen, runter) mit verschieden stark
    durchhaengenden/reibenden Armen: der Fahrweg beider Haende ist
    spiegelbildlich (kommandiert exakt, gemessen bis auf den bleibenden
    Regelfehler) und keine Hand dreht sich."""
    new = _mirror(ac_module, "command")
    old = _mirror(ac_module, "tf")
    print(f"\nSpiegel: command Wegdifferenz kommandiert {new['cmd'] * 100:.2f} cm, gemessen "
          f"{new['meas'] * 100:.2f} cm, Drehung {new['cmd_ori']:.1f}/{new['meas_ori']:.1f} deg | tf "
          f"{old['cmd'] * 100:.2f} / {old['meas'] * 100:.2f} cm, {old['meas_ori']:.1f} deg")
    assert new["cmd"] < 0.005
    assert new["meas"] < 0.012
    # Kommandiert dreht keine Hand; gemessen bleibt nur, was Reibung und
    # Durchhang der Fake-Arme (links bis 0.09 rad je Gelenk) in der neuen
    # Lage bewirken -- frueher kam das bei jeder Fahrt obendrauf.
    assert new["cmd_ori"] < 1.0 and new["meas_ori"] < 4.0
    assert old["meas"] > 5 * new["meas"] and old["meas_ori"] > 5 * new["meas_ori"]


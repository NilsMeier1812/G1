#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Uebergaben des arm_controller im ECHT-Modus -- ohne Hardware.

Der arm_controller laeuft hier mit G1_SIM_MODE=false gegen einen Fake-G1:
die Unitree-DDS-Klassen (rt/lowstate, rt/arm_sdk) sind durch Attrappen
ersetzt, ein Mini-Modell mischt wie der echte Roboter das arm_sdk-Kommando
mit der Onboard-Armpose nach dem Gewicht (motor_cmd[29].q):

    Soll = w * arm_sdk + (1 - w) * Onboard-Pose,   Messung folgt dem Soll (PD)

Geprueft wird, was Nils fuer den Echtbetrieb verlangt: kein Sprung beim
Uebernehmen (ENABLE), sanftes Anfahren, Rueckgabe an den Onboard-Regler erst
NACH der Rueckfahrt in dessen Pose (DISABLE und Programmende), E-Stop waehrend
der Rueckgabe, Schleppfehler-Begrenzung, Real-Grenzen fuer Gains/Rampen und
die "ersetzt"-Meldung an den RViz-Marker. Braucht rclpy + pinocchio.
"""
import json
import math
import os
import sys
import threading
import time
import types

import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
rclpy = pytest.importorskip("rclpy")

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ARM_IDX = list(range(15, 29))          # 7 links + 7 rechts (G1_29_JointArmIndex)
ONBOARD = np.array([0.10, 0.30, 0.0, 1.00, 0.0, 0.0, 0.0,
                    0.10, -0.30, 0.0, 1.00, 0.0, 0.0, 0.0])
RAISED = {"left": [-0.60, 0.30, 0.0, 0.70, 0.0, 0.0, 0.0],
          "right": [-0.60, -0.30, 0.0, 0.70, 0.0, 0.0, 0.0]}
DT = 0.004
WAIST_IDX = [12, 13, 14]
ONBOARD_WAIST = np.array([0.02, 0.0, 0.03])   # so haelt der Onboard-Regler die Taille
ONBOARD_WAIST_KP = 200.0
# Oberkoerper = umgekehrtes Pendel ueber der Taille: unterhalb dieser effektiven
# Steifigkeit [Nm/rad] kippt er (grobe Schaetzung m*g*l ~ 15 kg * 9.81 * 0.18 m).
PENDULUM_K = 26.0


# ── Attrappen fuer unitree_sdk2py ────────────────────────────────────────
class _Motor:
    def __init__(self):
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.mode = 0


class _LowCmd:
    def __init__(self):
        self.motor_cmd = [_Motor() for _ in range(35)]
        self.mode_pr = 0
        self.mode_machine = 0
        self.crc = 0


class FakeG1:
    """Mischt arm_sdk und Onboard-Pose nach dem Gewicht; Messung folgt (PD)."""

    def __init__(self):
        self.state = types.SimpleNamespace(
            motor_state=[_Motor() for _ in range(35)], mode_machine=5)
        for k, j in enumerate(ARM_IDX):
            self.state.motor_state[j].q = float(ONBOARD[k])
        for k, j in enumerate(WAIST_IDX):
            self.state.motor_state[j].q = float(ONBOARD_WAIST[k])
        self.writes = []            # (weight, q14, kp14) je Write
        self.waist_writes = []      # (weight, q3, kp3, kd3) je Write
        self.freeze = False         # True: Arm festgehalten (Messung steht)
        # Optional (test_jog_reference_real): bleibender Regelfehler je Gelenk
        # (Durchhang, die Messung steht um `sag` neben dem Soll) und Haftreibung
        # (die Messung folgt erst, wenn der Fehler `friction` uebersteigt).
        self.sag = np.zeros(14)
        self.friction = np.zeros(14)

    def waist(self):
        return np.array([self.state.motor_state[j].q for j in WAIST_IDX])

    def _waist_write(self, msg, w):
        """Gewicht blendet auch die Taille (wie beim echten G1): effektive
        Steifigkeit (1-w)*Onboard + w*arm_sdk; zu weich -> Oberkoerper kippt."""
        q = np.array([msg.motor_cmd[j].q for j in WAIST_IDX])
        kp = np.array([msg.motor_cmd[j].kp for j in WAIST_IDX])
        kd = np.array([msg.motor_cmd[j].kd for j in WAIST_IDX])
        self.waist_writes.append((w, q, kp, kd))
        meas = self.waist()
        k_eff = (1.0 - w) * ONBOARD_WAIST_KP + w * kp
        for k, j in enumerate(WAIST_IDX):
            if k_eff[k] < PENDULUM_K:
                if j == 14:                      # Pitch: kippt nach vorn
                    meas[k] = min(0.52, meas[k] + 0.01)
                continue
            soll = ((1.0 - w) * ONBOARD_WAIST_KP * ONBOARD_WAIST[k]
                    + w * kp[k] * q[k]) / k_eff[k]
            meas[k] += 0.5 * (soll - meas[k])
        for k, j in enumerate(WAIST_IDX):
            self.state.motor_state[j].q = float(meas[k])

    def measured(self):
        return np.array([self.state.motor_state[j].q for j in ARM_IDX])

    def write(self, msg):
        w = float(msg.motor_cmd[29].q)
        q = np.array([msg.motor_cmd[j].q for j in ARM_IDX])
        kp = np.array([msg.motor_cmd[j].kp for j in ARM_IDX])
        self.writes.append((w, q, kp))
        self._waist_write(msg, w)
        if self.freeze:
            return
        soll = w * q + (1.0 - w) * ONBOARD
        if w > 0.0 and float(np.max(kp)) == 0.0:      # E-Stop-Slack: Arme sacken
            soll = self.measured()
        meas = self.measured()
        err = soll + self.sag - meas
        meas += 0.5 * (err - np.clip(err, -self.friction, self.friction))
        for k, j in enumerate(ARM_IDX):
            self.state.motor_state[j].q = float(meas[k])


ROBOT = FakeG1()


def _install_unitree_stubs():
    class ChannelSubscriber:
        def __init__(self, *a, **k):
            pass

        def Init(self, *a, **k):
            pass

        def Read(self, *a, **k):
            return ROBOT.state

    class ChannelPublisher:
        def __init__(self, *a, **k):
            pass

        def Init(self, *a, **k):
            pass

        def Write(self, msg):
            ROBOT.write(msg)

    class CRC:
        def Crc(self, msg):
            return 0

    mods = {
        "unitree_sdk2py": types.ModuleType("unitree_sdk2py"),
        "unitree_sdk2py.core": types.ModuleType("unitree_sdk2py.core"),
        "unitree_sdk2py.core.channel": types.ModuleType("unitree_sdk2py.core.channel"),
        "unitree_sdk2py.idl": types.ModuleType("unitree_sdk2py.idl"),
        "unitree_sdk2py.idl.unitree_hg": types.ModuleType("unitree_sdk2py.idl.unitree_hg"),
        "unitree_sdk2py.idl.unitree_hg.msg": types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg"),
        "unitree_sdk2py.idl.unitree_hg.msg.dds_":
            types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_"),
        "unitree_sdk2py.idl.default": types.ModuleType("unitree_sdk2py.idl.default"),
        "unitree_sdk2py.utils": types.ModuleType("unitree_sdk2py.utils"),
        "unitree_sdk2py.utils.crc": types.ModuleType("unitree_sdk2py.utils.crc"),
    }
    mods["unitree_sdk2py.core.channel"].ChannelPublisher = ChannelPublisher
    mods["unitree_sdk2py.core.channel"].ChannelSubscriber = ChannelSubscriber
    mods["unitree_sdk2py.core.channel"].ChannelFactoryInitialize = lambda *a, **k: None
    mods["unitree_sdk2py.idl.unitree_hg.msg.dds_"].LowCmd_ = _LowCmd
    mods["unitree_sdk2py.idl.unitree_hg.msg.dds_"].LowState_ = object
    mods["unitree_sdk2py.idl.default"].unitree_hg_msg_dds__LowCmd_ = _LowCmd
    mods["unitree_sdk2py.utils.crc"].CRC = CRC
    for name, m in mods.items():
        sys.modules.setdefault(name, m)


@pytest.fixture(scope="module")
def ac_module(tmp_path_factory):
    os.environ["G1_SIM_MODE"] = "false"
    os.environ["G1_POSE_STORE"] = str(tmp_path_factory.mktemp("poses") / "poses.json")
    # package://g1pilot/... (Meshes) loest pinocchio ueber ROS_PACKAGE_PATH auf.
    os.environ["ROS_PACKAGE_PATH"] = os.path.dirname(PKG)
    _install_unitree_stubs()
    import ament_index_python.packages as aip
    aip.get_package_share_directory = lambda name: PKG
    from g1pilot.manipulation import arm_controller as ac
    ac.init_dds = lambda *a, **k: None
    yield ac


def _make_node(ac, **params):
    base = {"kp_low": 60.0, "kd_low": 1.5, "kp_wrist": 40.0, "kd_wrist": 1.5,
            "arm_accel_limit": 3.0, "max_tracking_error": 0.3}
    base.update(params)
    args = ["--ros-args"] + sum((["-p", f"{k}:={v}"] for k, v in base.items()), [])
    rclpy.init(args=args)
    node = ac.ArmController()
    node.destroy_timer(node.timer)      # main_loop treibt der Test selbst
    return node


def _close(node):
    try:
        node.destroy_node()
    finally:
        rclpy.shutdown()


def _wait_planner(node, timeout=120.0):
    t_end = time.monotonic() + timeout
    while not node._planner.ready and time.monotonic() < t_end:
        time.sleep(0.1)
    assert node._planner.ready, "Armplaner-Prozess nicht bereit"


def _run(node, seconds, until=None):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        node.main_loop()
        if until is not None and until():
            return True
        time.sleep(DT)
    return until() if until is not None else True


def _reset_robot():
    ROBOT.writes.clear()
    ROBOT.waist_writes.clear()
    ROBOT.freeze = False
    ROBOT.sag = np.zeros(14)
    ROBOT.friction = np.zeros(14)
    for k, j in enumerate(ARM_IDX):
        ROBOT.state.motor_state[j].q = float(ONBOARD[k])
    for k, j in enumerate(WAIST_IDX):
        ROBOT.state.motor_state[j].q = float(ONBOARD_WAIST[k])


def _max_step(writes, vel=1.5, slack=1.6):
    """Groesster Kommando-Sprung je Write relativ zur erlaubten Gelenkgeschw."""
    worst = 0.0
    for (_, q0, _), (_, q1, _) in zip(writes, writes[1:]):
        worst = max(worst, float(np.max(np.abs(q1 - q0))))
    return worst


def _bool(v):
    from std_msgs.msg import Bool
    return Bool(data=v)


def test_enable_move_disable_returns_before_handover(ac_module):
    """ENABLE ohne Sprung, sanft anfahren, DISABLE: erst zurueck in die
    Onboard-Pose, DANN Gewicht 1->0 -- die Ueberblendung hat (fast) keinen
    Positionsunterschied mehr."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        node._arms_controlled_callback(_bool(True))
        assert node._onboard_q is not None
        assert np.allclose(node._onboard_q, ONBOARD, atol=1e-6)
        _run(node, 2.4)
        assert node._weight_state == "on"
        # Uebernehmen ohne Sprung: Kommando = gemessene Pose, Messung bewegt sich nicht.
        assert float(np.max(np.abs(ROBOT.measured() - ONBOARD))) < 0.01
        assert all(float(np.max(np.abs(q - ONBOARD))) < 0.01 for _, q, _ in ROBOT.writes)

        # Weit entferntes Ziel (Arme vor) -> geplante Fahrt.
        from std_msgs.msg import String
        ROBOT.writes.clear()
        node._on_arm_command(String(data=json.dumps(
            {"id": "vor", "type": "joints", **RAISED})))
        goal = np.array(RAISED["left"] + RAISED["right"])
        assert _run(node, 20.0, until=lambda: (not node._planned_motion_active
                                               and float(np.max(np.abs(node._last_q_target - goal))) < 0.03))
        # Anfahrrampe: Geschwindigkeit steigt je Tick hoechstens um a*dt.
        speeds = [float(np.max(np.abs(q1 - q0))) for (_, q0, _), (_, q1, _)
                  in zip(ROBOT.writes, ROBOT.writes[1:])]
        assert max(speeds) < 1.5 * 0.1          # nie mehr als das Gelenklimit je Tick (dt <= 0.1)

        # DISABLE -> Rueckfahrt, dann Rampe.
        ROBOT.writes.clear()
        node._arms_controlled_callback(_bool(False))
        assert node._releasing, "DISABLE muss erst zurueckfahren"
        assert node.arms_enabled, "waehrend der Rueckfahrt behaelt arm_sdk die Arme"
        assert _run(node, 25.0, until=lambda: node._weight_state == "off")
        assert not node._releasing and not node.arms_enabled
        ws = ROBOT.writes
        # Solange das Gewicht noch voll ist, faehrt der Arm zur Onboard-Pose ...
        first_ramp = next(i for i, (w, _, _) in enumerate(ws) if w < 0.999)
        q_at_ramp = ws[first_ramp][1]
        assert float(np.max(np.abs(q_at_ramp - ONBOARD))) <= 0.06
        # ... und waehrend der Rampe steht das Kommando (keine Bewegung mehr).
        ramp_q = np.array([q for w, q, _ in ws[first_ramp:]])
        assert float(np.max(np.abs(ramp_q - q_at_ramp))) < 1e-9
        assert ws[-1][0] == 0.0
        # Die Messung landet ohne Schwung in der Onboard-Pose.
        assert float(np.max(np.abs(ROBOT.measured() - ONBOARD))) < 0.07
    finally:
        _close(node)


def test_inputs_ignored_and_estop_during_release(ac_module):
    _reset_robot()
    node = _make_node(ac_module)
    try:
        from std_msgs.msg import String
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.3)
        node._on_arm_command(String(data=json.dumps({"id": "vor", "type": "joints", **RAISED})))
        goal = np.array(RAISED["left"] + RAISED["right"])
        assert _run(node, 20.0, until=lambda: (not node._planned_motion_active
                                               and float(np.max(np.abs(node._last_q_target - goal))) < 0.03))
        node._arms_controlled_callback(_bool(False))
        assert node._releasing
        # Waehrend der Rueckgabe: neue Ziele werden abgelehnt.
        assert node._arms_not_ready_reason() is not None
        node._on_arm_command(String(data=json.dumps({"id": "x", "type": "joints", **RAISED})))
        assert node._releasing
        _run(node, 1.0)
        # E-Stop mitten in der Rueckgabe: sofort schlaff, Gewicht bleibt 1.
        ROBOT.writes.clear()
        node._on_emergency_stop(_bool(True))
        assert not node._releasing and node._arm_slack
        _run(node, 0.2)
        w, _, kp = ROBOT.writes[-1]
        assert w == 1.0 and float(np.max(kp)) == 0.0
    finally:
        _close(node)


def test_enable_during_release_holds(ac_module):
    _reset_robot()
    node = _make_node(ac_module)
    try:
        from std_msgs.msg import String
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.3)
        node._on_arm_command(String(data=json.dumps({"id": "vor", "type": "joints", **RAISED})))
        goal = np.array(RAISED["left"] + RAISED["right"])
        assert _run(node, 20.0, until=lambda: (not node._planned_motion_active
                                               and float(np.max(np.abs(node._last_q_target - goal))) < 0.03))
        node._arms_controlled_callback(_bool(False))
        _run(node, 0.5)
        node._arms_controlled_callback(_bool(True))
        assert not node._releasing and node.arms_enabled
        q_hold = node._last_q_target.copy()
        _run(node, 1.0)
        assert float(np.max(np.abs(node._last_q_target - q_hold))) < 0.02
        assert node._weight_state == "on"
    finally:
        _close(node)


def test_tracking_error_is_bounded(ac_module):
    """Arm festgehalten: das Kommando laeuft nicht weiter als 0.3 rad weg."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        from std_msgs.msg import String
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.3)
        _wait_planner(node)
        ROBOT.freeze = True
        node._on_arm_command(String(data=json.dumps({"id": "vor", "type": "joints", **RAISED})))
        peak = 0.0
        t_end = time.monotonic() + 6.0
        while time.monotonic() < t_end:
            node.main_loop()
            peak = max(peak, float(np.max(np.abs(node._last_q_target - ROBOT.measured()))))
            time.sleep(DT)
        assert node._planned_motion_active or peak > 0.25, "Fahrt muss gestartet sein"
        assert peak <= 0.3 + 1e-6
    finally:
        _close(node)


def test_shutdown_release_returns_then_ramps(ac_module):
    """Strg+C / docker stop: Arme fahren Richtung Onboard-Pose, Gewicht -> 0."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        from std_msgs.msg import String
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.3)
        node._on_arm_command(String(data=json.dumps({"id": "vor", "type": "joints", **RAISED})))
        goal = np.array(RAISED["left"] + RAISED["right"])
        assert _run(node, 20.0, until=lambda: (not node._planned_motion_active
                                               and float(np.max(np.abs(node._last_q_target - goal))) < 0.03))
        ROBOT.writes.clear()
        node.shutdown_release()
        ws = ROBOT.writes
        assert ws[-1][0] == 0.0
        # Erst Bewegung bei vollem Gewicht, Schritte klein (<= 0.8 rad/s * 4 ms).
        steps = [float(np.max(np.abs(q1 - q0))) for (_, q0, _), (_, q1, _) in zip(ws, ws[1:])]
        assert max(steps) <= 0.8 * DT + 1e-6
        first_ramp = next(i for i, (w, _, _) in enumerate(ws) if w < 0.999)
        moved = float(np.max(np.abs(ws[first_ramp][1] - goal)))
        assert moved > 0.5, "vor der Rampe Richtung Onboard-Pose gefahren"
    finally:
        import signal
        signal.signal(signal.SIGINT, signal.default_int_handler)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        _close(node)


def test_real_limits_clamp_sim_gains(ac_module):
    """Real: Sim-Gains und Rampe 0 aus einem falschen Launchfile werden begrenzt."""
    _reset_robot()
    node = _make_node(ac_module, kp_low=150.0, kd_low=12.0,
                      arm_weight_ramp_up_s=0.0, arm_weight_ramp_down_s=0.0,
                      ee_velocity_limit=0.5, arm_velocity_limit=5.0,
                      kp_waist=400.0, kd_waist=12.0)
    try:
        assert node.kp_low <= 80.0 and node.kd_low <= 3.0
        assert node.kp_waist <= 250.0 and node.kd_waist <= 6.0
        assert node._waist.kp <= 250.0 and node._waist.kd <= 6.0
        assert float(node.get_parameter("arm_weight_ramp_up_s").value) >= 1.0
        assert float(node.get_parameter("arm_weight_ramp_down_s").value) >= 1.0
        assert node.ee_velocity_limit <= 0.25 and node.arm_velocity_limit <= 1.5
    finally:
        _close(node)


def test_new_goal_supersedes_marker_goal(ac_module):
    """Zweiter Eingang setzt ein Ziel: das alte Marker-Ziel bekommt 'cancelled'."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        from geometry_msgs.msg import PoseStamped
        got = []
        node._publish_marker_goal_status = (
            lambda side, stamp, state, **kw: got.append((side, stamp, state)))
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.3)
        T = node._fk_current_ee("right")
        for sec in (1, 2):
            ps = PoseStamped()
            ps.header.frame_id = "pelvis"
            ps.header.stamp.sec = sec
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
                float(T.translation[0]), float(T.translation[1]), float(T.translation[2]) + 0.02)
            ps.pose.orientation.w = 1.0
            node._right_goal_callback(ps)
        assert ("right", (1, 0), "cancelled") in got
    finally:
        _close(node)


_SIGTERM_SCRIPT = r'''
import os, sys, threading, time
sys.path.insert(0, os.environ["G1_TEST_DIR"])
import numpy as np
import test_arm_controller_real as T
T._install_unitree_stubs()
import ament_index_python.packages as aip
aip.get_package_share_directory = lambda name: T.PKG
from g1pilot.manipulation import arm_controller as ac
ac.init_dds = lambda *a, **k: None
from std_msgs.msg import Bool
orig = ac.ArmController.__init__
def init(self, *a, **k):
    orig(self, *a, **k)
    said = []
    def tick():
        if not self.arms_enabled:
            self._arms_controlled_callback(Bool(data=True))
        elif self._weight_state == "on" and not said:
            said.append(1)
            print("WEIGHT_ON", flush=True)
    self.create_timer(0.5, tick)
ac.ArmController.__init__ = init
def report():
    ws = T.ROBOT.writes
    print("WRITES", len(ws), "LASTW", ws[-1][0] if ws else None, flush=True)
import atexit
atexit.register(report)
ac.main(args=["--ros-args", "-p", "kp_low:=60.0", "-p", "kd_low:=1.5",
              "-p", "kd_wrist:=1.5"])
'''


def test_sigterm_hands_arms_back(tmp_path):
    """docker stop / launch-Eskalation (SIGTERM): Prozess gibt die Arme mit
    Gewichtsrampe 1->0 ab, statt den arm_sdk-Strom abzureissen."""
    import signal
    import subprocess
    script = tmp_path / "run_ac.py"
    script.write_text(_SIGTERM_SCRIPT)
    env = dict(os.environ, G1_SIM_MODE="false", G1_TEST_DIR=os.path.dirname(__file__),
               G1_POSE_STORE=str(tmp_path / "p.json"),
               ROS_PACKAGE_PATH=os.path.dirname(PKG))
    p = subprocess.Popen([sys.executable, str(script)], env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    lines = []
    reader = threading.Thread(target=lambda: lines.extend(iter(p.stdout.readline, "")),
                              daemon=True)
    reader.start()
    try:
        # Warten, bis der Controller geladen hat und die Arme fuehrt (Gewicht 1).
        # Unter Last (volle Testsuite, Planer-Prozess startet parallel) dauert
        # das Laden deutlich laenger als allein -- feste Wartezeiten flaken.
        t_end = time.monotonic() + 180.0
        while time.monotonic() < t_end and not any("WEIGHT_ON" in ln for ln in lines):
            assert p.poll() is None, "".join(lines)[-3000:]
            time.sleep(0.2)
        assert any("WEIGHT_ON" in ln for ln in lines), "".join(lines)[-3000:]
        p.send_signal(signal.SIGTERM)
        p.wait(timeout=30)
        reader.join(timeout=5)
    finally:
        if p.poll() is None:
            p.kill()
    out = "".join(lines)
    assert "WRITES" in out, out[-3000:]
    last_w = float(out.split("LASTW")[-1].split()[0])
    assert last_w == 0.0, out[-3000:]
    assert "Arme an den Onboard-Regler uebergeben" in out, out[-3000:]
    assert "Gewicht 1.00->0" in out or "Programmende: Arme fahren" in out, out[-3000:]


def test_planner_loads_parallel_to_controller(ac_module, monkeypatch):
    """Der Planer-Prozess startet VOR dem Modell-Laden des Controllers (beide
    laden parallel). Nacheinander war er ~20 s nach dem Controller bereit und
    das erste WALK wartete so lange auf die Planung der Lauf-Pose."""
    order = []
    orig_start, orig_ik = ac_module.PlannerProcess.start_async, ac_module.G1IKSolver

    def start_async(self):
        order.append(self)
        return orig_start(self)

    def ik(*a, **k):
        # Der Arbeiterprozess muss schon LAUFEN (nicht nur ein Thread, der ihn
        # erst startet, wenn das Modell-Laden hier das GIL freigibt).
        proc = order[0]._proc if order else None
        order.append(("ik", proc is not None and proc.is_alive()))
        return orig_ik(*a, **k)

    monkeypatch.setattr(ac_module.PlannerProcess, "start_async", start_async)
    monkeypatch.setattr(ac_module, "G1IKSolver", ik)
    _reset_robot()
    node = _make_node(ac_module)
    try:
        assert len(order) >= 2 and order[1] == ("ik", True), order
    finally:
        _close(node)


def test_walk_without_enable_reports_ready_at_once(ac_module):
    """Real direkt nach START -> WALK (Arme nicht aktiviert): die Arme fuehrt
    der Onboard-Regler, nichts aufzuraeumen -> walk_ready sofort (vorher kam
    es nie, die Demo-GUI hing bis zu ihrem 16-s-Timeout in "Arme werden
    eingeklappt"). Kein arm_sdk-Kommando dabei."""
    _reset_robot()
    node = _make_node(ac_module)
    sent = []
    orig_pub = node.walk_ready_publisher.publish
    node.walk_ready_publisher.publish = lambda m: (sent.append(m.data), orig_pub(m))
    try:
        node._on_walk_mode(_bool(True))
        assert sent == [True]
        _run(node, 0.5)
        assert not node.arms_enabled
        assert all(w == 0.0 for w, _, _ in ROBOT.writes), "Arme duerfen nicht uebernommen werden"
        # Zurueck nach GREIFEN nimmt die Meldung wieder zurueck.
        node._on_balance_mode(_bool(True))
        assert sent == [True, False] and not node._walk_ready_sent
    finally:
        _close(node)


# ── Taille (Waist 12-14) ────────────────────────────────────────────────
# Befund vom Real-Test 2026-10-14: sobald die Arme gesteuert werden, ist die
# Taille kraftlos und der Oberkoerper kippt. Ursache: das arm_sdk-Gewicht
# schaltet auch die Taille vom Onboard-Regler weg; wir schrieben dort kp=0.


def test_waist_limp_without_hold_regression(ac_module):
    """Gegenprobe (altes Verhalten, waist_hold=false): bei Gewicht 1 kippt
    der Oberkoerper im Fake-G1 -- genau das Bild vom Real-Test."""
    _reset_robot()
    node = _make_node(ac_module, waist_hold=False)
    try:
        assert node._waist is None
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.6)
        assert node._weight_state == "on"
        assert ROBOT.waist()[2] - ONBOARD_WAIST[2] > 0.2, "Taille muesste kippen"
    finally:
        _close(node)


def test_waist_held_while_arms_controlled(ac_module):
    """ENABLE + Armfahrt: Taille bleibt in der Onboard-Stellung, ohne Sprung
    beim Uebernehmen; kp/kd wie konfiguriert."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        from std_msgs.msg import String
        node._arms_controlled_callback(_bool(True))
        assert np.allclose(node._waist.target, ONBOARD_WAIST, atol=1e-9)
        _run(node, 2.4)
        assert node._weight_state == "on"
        node._on_arm_command(String(data=json.dumps({"id": "vor", "type": "joints", **RAISED})))
        goal = np.array(RAISED["left"] + RAISED["right"])
        assert _run(node, 20.0, until=lambda: (not node._planned_motion_active
                                               and float(np.max(np.abs(node._last_q_target - goal))) < 0.03))
        ww = ROBOT.waist_writes
        assert len(ww) > 50
        for w, q, kp, kd in ww:
            assert np.allclose(q, ONBOARD_WAIST, atol=1e-6), "Taillen-Soll = Onboard-Stellung"
            assert np.allclose(kp, 150.0) and np.allclose(kd, 4.0)
        assert float(np.max(np.abs(ROBOT.waist() - ONBOARD_WAIST))) < 0.01
    finally:
        _close(node)


def test_waist_handover_on_disable(ac_module):
    """DISABLE: waehrend der Gewichtsrampe 1->0 haelt arm_sdk die Taille
    unveraendert in der Onboard-Stellung -> weiche Rueckgabe."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        ROBOT.waist_writes.clear()
        node._arms_controlled_callback(_bool(False))
        assert _run(node, 25.0, until=lambda: node._weight_state == "off")
        ww = ROBOT.waist_writes
        assert ww[-1][0] == 0.0
        assert any(0.0 < w < 1.0 for w, _, _, _ in ww), "Rampe muss gesendet werden"
        for w, q, kp, _ in ww:
            assert np.allclose(q, ONBOARD_WAIST, atol=1e-6) and float(np.min(kp)) > 0.0
        assert float(np.max(np.abs(ROBOT.waist() - ONBOARD_WAIST))) < 0.01
    finally:
        _close(node)


def test_waist_estop_passive_then_start_reengages(ac_module):
    """E-Stop mit estop_waist_hold=false (altes Verhalten): Taille passiv wie
    die Arme (kp=0, Daempfung). START: weich wieder halten -- ab der gemessenen
    Stellung, kp rampt, Kommando faehrt langsam zurueck in die Onboard-Stellung
    (Arme bleiben schlaff)."""
    _reset_robot()
    node = _make_node(ac_module, waist_engage_s=1.0, waist_max_vel=0.3,
                      estop_waist_hold=False)
    try:
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        node._on_emergency_stop(_bool(True))
        _run(node, 0.3)
        w, q, kp, kd = ROBOT.waist_writes[-1]
        assert w == 1.0 and float(np.max(kp)) == 0.0 and np.allclose(kd, 3.0)
        # Roboter ist zusammengesackt: Taille steht 0.25 rad vorgekippt.
        bent = ONBOARD_WAIST + np.array([0.0, 0.0, 0.25])
        for k, j in enumerate(WAIST_IDX):
            ROBOT.state.motor_state[j].q = float(bent[k])
        ROBOT.waist_writes.clear()
        node._on_start(_bool(True))
        assert not node.estop_active and node._arm_slack
        _run(node, 2.5)
        ww = ROBOT.waist_writes
        # Erstes Kommando an der Messung, kp klein (Rampe), kein Sprung.
        assert float(np.max(np.abs(ww[0][1] - bent))) <= 0.3 * 0.1 + 1e-9
        assert float(np.max(ww[0][2])) < 30.0
        steps = [float(np.max(np.abs(b[1] - a[1]))) for a, b in zip(ww, ww[1:])]
        assert max(steps) <= 0.3 * 0.1 + 1e-9
        kps = [float(np.max(x[2])) for x in ww]
        assert all(b >= a - 1e-9 for a, b in zip(kps, kps[1:])), "kp rampt monoton hoch"
        assert np.isclose(kps[-1], 150.0)
        assert np.allclose(ww[-1][1], ONBOARD_WAIST, atol=1e-6)
        # Arme bleiben schlaff.
        assert float(np.max(ROBOT.writes[-1][2])) == 0.0
    finally:
        _close(node)


def test_waist_estop_keeps_holding_by_default(ac_module):
    """Real-Test 2026-10-15 (Walk-Modus): der E-Stop darf die Taille NICHT
    schlaff schalten -- nur Arme/Haende. Der Walker balanciert weiter und
    braucht einen steifen Oberkoerper."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        assert node.get_parameter("estop_waist_hold").value is True
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        node._on_emergency_stop(_bool(True))
        ROBOT.waist_writes.clear()
        _run(node, 1.0)
        assert node.estop_active and node._arm_slack and not node._waist.passive
        ww = ROBOT.waist_writes
        assert len(ww) > 50
        for w, q, kp, kd in ww:
            assert w == 1.0 and float(np.min(kp)) == 150.0 and np.allclose(kd, 4.0)
            assert np.allclose(q, ONBOARD_WAIST, atol=1e-6)
        assert float(np.max(np.abs(ROBOT.waist() - ONBOARD_WAIST))) < 0.01, "Oberkoerper kippt"
        # Arme: weiter schlaff (kp=0, Gewicht 1 = kein Onboard-Snap).
        assert float(np.max(ROBOT.writes[-1][2])) == 0.0 and ROBOT.writes[-1][0] == 1.0
        # START quittiert, Taille war nie passiv -> kein Neu-Zupacken noetig.
        node._on_start(_bool(True))
        _run(node, 0.3)
        assert float(np.max(np.abs(ROBOT.waist() - ONBOARD_WAIST))) < 0.01
    finally:
        _close(node)


def test_waist_estop_from_onboard_walker_holds_measured_pose(ac_module):
    """Der Befund selbst: Walk OHNE ENABLE (Onboard-Regler fuehrt Arme UND
    Taille, arm_sdk-Gewicht 0), dann E-Stop. arm_sdk uebernimmt (Gewicht 1);
    die Taille wird an der GEMESSENEN Stellung gehalten -- sie kippt nicht und
    springt nicht. Gegenprobe estop_waist_hold=false: Oberkoerper kippt."""
    for hold in (True, False):
        _reset_robot()
        bent = ONBOARD_WAIST + np.array([0.0, 0.05, 0.04])   # Walker steht leicht schraeg
        for k, j in enumerate(WAIST_IDX):
            ROBOT.state.motor_state[j].q = float(bent[k])
        node = _make_node(ac_module, estop_waist_hold=hold)
        try:
            node._on_emergency_stop(_bool(True))
            ROBOT.waist_writes.clear()
            _run(node, 1.0)
            if hold:
                assert float(np.max(np.abs(ROBOT.waist() - bent))) < 0.01, "Taille driftet"
                for w, q, kp, kd in ROBOT.waist_writes:
                    assert np.allclose(q, bent, atol=1e-6) and float(np.min(kp)) > 100.0
            else:
                assert ROBOT.waist()[2] - bent[2] > 0.2, "alte Variante muesste kippen"
        finally:
            _close(node)


def test_waist_hold_off_in_sim_mode(ac_module):
    """Sim: Taille bleibt unangetastet (MuJoCo-Bridge/Policy unveraendert)."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        os.environ["G1_SIM_MODE"] = "true"
        node._init_waist_hold()
        assert node._waist is None
    finally:
        os.environ["G1_SIM_MODE"] = "false"
        _close(node)


# ── LAUFEN mit aktiven Armen (Nils 2026-10-19) ──────────────────────────
# Real (bringup_real, G1_WALK_PARK_ARMS=false): die Arme bleiben beim Wechsel
# nach LAUFEN in ihrer Pose, statt in die Lauf-Pose zu fahren. arm_sdk behaelt
# Gewicht 1, Arme und Taille werden gehalten.


def _enable_and_raise(ac, node):
    from std_msgs.msg import String
    _wait_planner(node)
    node._arms_controlled_callback(_bool(True))
    _run(node, 2.4)
    assert node._weight_state == "on"
    node._on_arm_command(String(data=json.dumps({"id": "vor", "type": "joints", **RAISED})))
    goal = np.array(RAISED["left"] + RAISED["right"])
    assert _run(node, 20.0, until=lambda: (not node._planned_motion_active
                                           and float(np.max(np.abs(node._last_q_target - goal))) < 0.03))
    _run(node, 0.5)
    return goal


def test_real_walk_keeps_arms_in_pose(ac_module):
    """GREIFEN (Arme vorn) -> LAUFEN: walk_ready sofort, Arme bleiben exakt
    stehen (kein Parken, kein Sprung), Gewicht 1, Taille gehalten.
    Zurueck nach GREIFEN: Arme stehen weiter, walk_ready zurueckgenommen."""
    _reset_robot()
    node = _make_node(ac_module, walk_park_arms=False)
    sent = []
    orig_pub = node.walk_ready_publisher.publish
    node.walk_ready_publisher.publish = lambda m: (sent.append(m.data), orig_pub(m))
    try:
        goal = _enable_and_raise(ac_module, node)
        q_before = node._last_q_target.copy()
        meas_before = ROBOT.measured().copy()
        ROBOT.writes.clear()
        ROBOT.waist_writes.clear()
        node._on_walk_mode(_bool(True))
        assert sent == [True], "walk_ready muss sofort kommen"
        assert not node.walk_mode and not node._walk_transition
        assert not node._planned_motion_active
        _run(node, 2.0)
        assert len(ROBOT.writes) > 50
        for w, q, kp in ROBOT.writes:
            assert w == 1.0 and float(np.min(kp)) > 0.0
            assert float(np.max(np.abs(q - q_before))) < 1e-6, "Arm-Soll darf sich nicht bewegen"
        assert float(np.max(np.abs(ROBOT.measured() - meas_before))) < 0.01
        assert float(np.max(np.abs(ROBOT.measured() - goal))) < 0.05
        for w, q, kp, kd in ROBOT.waist_writes:
            assert w == 1.0 and np.allclose(q, ONBOARD_WAIST, atol=1e-6) and float(np.min(kp)) > 100.0
        assert float(np.max(np.abs(ROBOT.waist() - ONBOARD_WAIST))) < 0.01
        # Zurueck nach GREIFEN: Arme bleiben, Meldung zurueckgenommen.
        node._on_balance_mode(_bool(True))
        assert sent == [True, False]
        _run(node, 0.5)
        assert float(np.max(np.abs(node._last_q_target - q_before))) < 1e-6
    finally:
        _close(node)


def test_real_walk_stops_running_motion_in_place(ac_module):
    """LAUFEN waehrend einer geplanten Armfahrt: die Fahrt endet, die Arme
    bleiben, wo sie gerade sind -- ohne Sprung zum alten Ziel oder zur
    Lauf-Pose."""
    from std_msgs.msg import String
    _reset_robot()
    node = _make_node(ac_module, walk_park_arms=False)
    try:
        _wait_planner(node)
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        node._on_arm_command(String(data=json.dumps({"id": "vor", "type": "joints", **RAISED})))
        assert _run(node, 20.0, until=lambda: (node._planned_motion_active and
                                               float(np.max(np.abs(node._last_q_target - ONBOARD))) > 0.2))
        node._on_walk_mode(_bool(True))
        assert not node._planned_motion_active
        q_hold = node._last_q_target.copy()
        ROBOT.writes.clear()
        _run(node, 2.0)
        qs = np.array([q for _, q, _ in ROBOT.writes])
        # Bremsen mit der Beschleunigungsgrenze ist erlaubt, danach Stillstand.
        assert float(np.max(np.abs(qs - q_hold))) < 0.15
        assert float(np.max(np.abs(qs[-1] - qs[-20]))) < 1e-6, "Arme muessen stehen bleiben"
        goal = np.array(RAISED["left"] + RAISED["right"])
        assert float(np.max(np.abs(qs[-1] - goal))) > 0.1, "darf nicht zum alten Ziel weiterfahren"
        walk = np.concatenate((node.walk_left, node.walk_right))
        assert float(np.max(np.abs(qs[-1] - walk))) > 0.1, "darf nicht in die Lauf-Pose fahren"
        assert _max_step(ROBOT.writes) < 0.02
    finally:
        _close(node)


def test_real_walk_estop_unchanged(ac_module):
    """NOT-HALT im LAUFEN mit gehaltenen Armen: wie bisher Arme schlaff
    (kp 0, Gewicht 1), Taille gehalten."""
    _reset_robot()
    node = _make_node(ac_module, walk_park_arms=False)
    try:
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        node._on_walk_mode(_bool(True))
        _run(node, 0.3)
        node._on_emergency_stop(_bool(True))
        ROBOT.waist_writes.clear()
        _run(node, 1.0)
        assert node.estop_active and node._arm_slack
        assert float(np.max(ROBOT.writes[-1][2])) == 0.0 and ROBOT.writes[-1][0] == 1.0
        for w, q, kp, kd in ROBOT.waist_writes:
            assert w == 1.0 and float(np.min(kp)) > 100.0
        assert float(np.max(np.abs(ROBOT.waist() - ONBOARD_WAIST))) < 0.01
    finally:
        _close(node)


def test_real_walk_park_arms_true_keeps_old_parking(ac_module):
    """G1_WALK_PARK_ARMS=true: wie frueher -- geplante Fahrt in die Lauf-Pose,
    walk_ready erst dort."""
    _reset_robot()
    node = _make_node(ac_module, walk_park_arms=True)
    sent = []
    orig_pub = node.walk_ready_publisher.publish
    node.walk_ready_publisher.publish = lambda m: (sent.append(m.data), orig_pub(m))
    try:
        _wait_planner(node)
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        node._on_walk_mode(_bool(True))
        assert node.walk_mode and sent == []
        walk = np.concatenate((node.walk_left, node.walk_right))
        assert _run(node, 25.0, until=lambda: sent == [True])
        assert float(np.max(np.abs(node._last_q_target - walk))) < 0.06
    finally:
        _close(node)


def test_bringup_real_defaults_to_holding_arms():
    """bringup_real reicht G1_WALK_PARK_ARMS (Default false) an walk_park_arms."""
    src = open(os.path.join(PKG, "launch", "bringup_real.launch.py")).read()
    assert "'walk_park_arms': os.environ.get('G1_WALK_PARK_ARMS', 'false')" in src


def test_full_estop_makes_waist_passive_too(ac_module):
    """Voll-Notaus (PS4 R1+L1+PS): Arme schlaff wie beim E-Stop UND die Taille
    passiv (kp=0, gedaempft), auch mit estop_waist_hold=true. arm_sdk behaelt
    Gewicht 1 (kein Onboard-Snap, falls der Damp-RPC ausfaellt). Auch nach
    einem vorherigen normalen E-Stop (PS zuerst gedrueckt)."""
    for first_estop in (False, True):
        _reset_robot()
        node = _make_node(ac_module)
        try:
            node._arms_controlled_callback(_bool(True))
            _run(node, 2.4)
            if first_estop:
                node._on_emergency_stop(_bool(True))
                _run(node, 0.2)
                assert not node._waist.passive
            node._on_full_emergency_stop(_bool(True))
            assert node.estop_active and node._arm_slack and node._waist.passive
            ROBOT.waist_writes.clear()
            _run(node, 0.5)
            for w, q, kp, kd in ROBOT.waist_writes:
                assert w == 1.0 and float(np.max(kp)) == 0.0 and np.allclose(kd, 3.0)
            assert float(np.max(ROBOT.writes[-1][2])) == 0.0 and ROBOT.writes[-1][0] == 1.0
            # ENABLE gesperrt bis START
            node._arms_controlled_callback(_bool(True))
            assert not node.arms_enabled
            node._on_start(_bool(True))
            assert not node.estop_active
        finally:
            _close(node)


# ── Hueftdrehung (Taillen-Yaw per Controller, Nils 2026-10-21) ──────────
# Sollwert ueber WaistHold (Gelenk 12), nur mit uebernommenen Armen. Mitte =
# Yaw, den der Onboard-Regler beim ENABLE hielt (ONBOARD_WAIST[0]).
HOME_YAW = float(ONBOARD_WAIST[0])


def _rate(node, r):
    from std_msgs.msg import Float32
    node._on_waist_rate(Float32(data=float(r)))


def _turn(node, rate, seconds):
    """Stick halten: Rate-Strom mit ~30 Hz wie ps4_arm_teleop."""
    t_end = time.monotonic() + seconds
    t_next = 0.0
    while time.monotonic() < t_end:
        if time.monotonic() >= t_next:
            _rate(node, rate)
            t_next = time.monotonic() + 1.0 / 30.0
        node.main_loop()
        time.sleep(DT)


def _state(node):
    sent = []
    orig = node.pub_waist_state.publish
    node.pub_waist_state.publish = lambda m: (sent.append(json.loads(m.data)), orig(m))
    return sent


def test_waist_yaw_turns_with_limit_arms_hold_joints(ac_module):
    _reset_robot()
    node = _make_node(ac_module, waist_yaw_limit_deg=15.0)
    st = _state(node)
    try:
        node._arms_controlled_callback(_bool(True))
        _rate(node, 1.0)                                       # noch keine Uebernahme
        _run(node, 0.3)
        assert node._wyaw.target == pytest.approx(HOME_YAW, abs=1e-6)
        _run(node, 2.4)
        assert node._weight_state == "on"
        q_arm = node._last_q_target.copy()
        ROBOT.writes.clear()
        _turn(node, 1.0, 0.6)                                 # ~15 deg/s
        assert math.degrees(node._wyaw.target - HOME_YAW) == pytest.approx(9.0, abs=3.0)
        _turn(node, 1.0, 1.5)
        assert math.degrees(node._wyaw.target - HOME_YAW) == pytest.approx(15.0, abs=1e-6)
        _run(node, 1.5)
        assert ROBOT.waist()[0] == pytest.approx(HOME_YAW + math.radians(15), abs=0.01)
        # Roll/Pitch unveraendert, Arme exakt gehalten
        assert np.allclose(ROBOT.waist()[1:], ONBOARD_WAIST[1:], atol=0.01)
        for _, q, _ in ROBOT.writes:
            assert float(np.max(np.abs(q - q_arm))) < 1e-6
        # Totmann: ohne Strom bleibt sie stehen
        yaw = node._wyaw.target
        _run(node, 0.5)
        assert node._wyaw.target == yaw
        assert st[-1]["owner"] and st[-1]["available"] and not st[-1]["walk_ok"]
        assert st[-1]["limit_deg"] == pytest.approx(15.0)
    finally:
        _close(node)


def test_waist_yaw_ignored_without_arms_or_in_walk(ac_module):
    _reset_robot()
    node = _make_node(ac_module)
    try:
        _rate(node, 1.0)
        _run(node, 0.5)                                       # Arme nicht uebernommen
        assert node._wyaw.target is None or node._wyaw.target == pytest.approx(HOME_YAW, abs=1e-6)
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        node._on_walk_mode(_bool(True))                       # GEHEN (Arme bleiben)
        _turn(node, 1.0, 0.5)
        assert node._wyaw.target == pytest.approx(HOME_YAW, abs=1e-6)
        node._on_balance_mode(_bool(True))
        _turn(node, 1.0, 0.5)
        assert node._wyaw.target > HOME_YAW + math.radians(3)
    finally:
        _close(node)


def test_waist_yaw_walk_waits_for_center(ac_module):
    """Huefte gedreht -> GEHEN: erst zurueck zur Mitte, walk_ready erst danach;
    walk_ok im Zustand erst in der Mitte (loco_client sperrt Move() bis dahin)."""
    _reset_robot()
    node = _make_node(ac_module, walk_park_arms=False)
    sent = []
    orig_pub = node.walk_ready_publisher.publish
    node.walk_ready_publisher.publish = lambda m: (sent.append(m.data), orig_pub(m))
    st = _state(node)
    try:
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        _turn(node, -1.0, 1.0)                                # nach rechts
        assert node._wyaw.offset < -math.radians(10)
        _run(node, 0.5)
        node._on_walk_mode(_bool(True))
        assert sent == [] and node._wyaw.centering
        assert _run(node, 0.3) and st[-1]["walk_ok"] is False and st[-1]["pending"] == "walk"
        _rate(node, 1.0)                                      # Stick waehrend der Mitte-Fahrt: ignoriert
        assert _run(node, 6.0, until=lambda: sent == [True])
        assert abs(ROBOT.waist()[0] - HOME_YAW) < math.radians(3.1)
        assert node._wyaw.target == pytest.approx(HOME_YAW)
        _run(node, 0.2)
        assert st[-1]["walk_ok"] is True
    finally:
        _close(node)


def test_waist_yaw_disable_centers_before_handover(ac_module):
    _reset_robot()
    node = _make_node(ac_module)
    try:
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        _turn(node, 1.0, 1.0)
        _run(node, 0.5)
        ROBOT.waist_writes.clear()
        node._arms_controlled_callback(_bool(False))
        assert node.arms_enabled and not node._releasing      # erst Mitte-Fahrt
        assert _run(node, 30.0, until=lambda: node._weight_state == "off")
        ww = ROBOT.waist_writes
        for w, q, _, _ in ww:
            if w < 1.0:                                       # Abrampen erst in der Mitte
                assert abs(q[0] - HOME_YAW) < math.radians(2.1)
        assert abs(ROBOT.waist()[0] - HOME_YAW) < 0.01
    finally:
        _close(node)


def test_waist_yaw_estop_freezes_no_auto_return(ac_module):
    _reset_robot()
    node = _make_node(ac_module)
    try:
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        _rate(node, 1.0)
        _run(node, 0.4)
        node._on_emergency_stop(_bool(True))
        yaw = node._wyaw.target
        _run(node, 1.0)
        assert node._wyaw.target == yaw and not node._wyaw.moving(time.monotonic())
        assert ROBOT.waist_writes[-1][1][0] == pytest.approx(yaw, abs=1e-6)
        node._on_waist_center(_bool(True))                    # im NOT-HALT: nichts
        assert not node._wyaw.centering
        node._on_start(_bool(True))
        _run(node, 1.0)
        assert node._wyaw.target == yaw, "nach START kein automatisches Zurueckdrehen"
        node._on_waist_center(_bool(True))                    # Mitte auf Wunsch
        assert _run(node, 4.0, until=lambda: node._wyaw.at_home)
    finally:
        _close(node)


def test_waist_yaw_full_estop_then_start_keeps_measured(ac_module):
    _reset_robot()
    node = _make_node(ac_module)
    try:
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        _turn(node, 1.0, 1.0)
        node._on_full_emergency_stop(_bool(True))
        ROBOT.state.motor_state[12].q = HOME_YAW + 0.05       # schlaff weggedreht
        _run(node, 0.3)
        node._on_start(_bool(True))
        _run(node, 0.5)
        assert node._wyaw.target == pytest.approx(HOME_YAW + 0.05, abs=0.01)
        assert node._waist.target[0] == pytest.approx(node._wyaw.target)
    finally:
        _close(node)


def test_waist_yaw_turn_aborts_motion_and_blocks_goals(ac_module):
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    _reset_robot()
    node = _make_node(ac_module)
    st = []
    orig = node.pub_marker_goal_status.publish
    node.pub_marker_goal_status.publish = lambda m: (st.append(json.loads(m.data)), orig(m))
    try:
        _wait_planner(node)
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        node._on_arm_command(String(data=json.dumps({"id": "vor", "type": "joints", **RAISED})))
        _run(node, 0.3, until=lambda: node._planned_motion_active)
        _rate(node, 1.0)
        node.main_loop()
        assert not node._planned_motion_active, "Hueftdrehung bricht die geplante Fahrt ab"
        q_arm = node._last_q_target.copy()
        g = PoseStamped()
        g.header.frame_id = "pelvis"
        g.pose.position.x, g.pose.position.z = 0.3, 0.1
        g.pose.orientation.w = 1.0
        node._right_goal_callback(g)
        assert st and st[-1].get("state") == "ignored"
        _turn(node, 1.0, 0.5)
        assert float(np.max(np.abs(node._last_q_target - q_arm))) < 1e-6
        _rate(node, 1.0)
        node.main_loop()
        assert "Huefte" in (node._arms_not_ready_reason() or "")
    finally:
        _close(node)


def test_waist_yaw_other_hand_stays_after_turn(ac_module):
    """Bug Nils 2026-10-22: rechte Hand gewaehlt, Huefte ~30 deg gedreht,
    danach ein Jog-Ziel nur fuer rechts -> die LINKE Hand fuhr zur Pose von vor
    der Drehung zurueck (IK-Halteziel noch im alten Becken-Bezug). Jetzt:
    nach dem Nachlauf neue Halteziele mit gedrehter Taille -> links bleibt."""
    from geometry_msgs.msg import PoseStamped
    _reset_robot()
    node = _make_node(ac_module)
    try:
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        assert node._weight_state == "on"
        _turn(node, -1.0, 2.0)                                # ~30 deg rechts
        assert math.degrees(node._wyaw.target - HOME_YAW) == pytest.approx(-30.0, abs=5.0)
        _run(node, 2.5)                                       # Nachlauf
        assert node._wyaw_settle_t0 is None and not node._waist_busy()
        q_left = node._last_q_target[0:7].copy()
        q_right = node._last_q_target[7:14].copy()
        g = PoseStamped()
        g.header.frame_id = "pelvis"
        g.pose.position.x, g.pose.position.y, g.pose.position.z = 0.3, -0.25, 0.1
        g.pose.orientation.w = 1.0
        for _ in range(40):                                   # Jog-Strom rechts
            node._right_goal_callback(g)
            node.main_loop()
            time.sleep(DT)
        assert float(np.max(np.abs(node._last_q_target[7:14] - q_right))) > 0.01, \
            "rechts folgt dem Ziel"
        assert float(np.max(np.abs(node._last_q_target[0:7] - q_left))) < 1e-3, \
            "linke Hand bleibt nach der Hueftdrehung stehen"
    finally:
        _close(node)


def test_waist_yaw_home_waits_for_center(ac_module):
    _reset_robot()
    node = _make_node(ac_module)
    try:
        _wait_planner(node)
        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        _turn(node, 1.0, 1.0)
        _run(node, 0.3)
        node._homming_callback(_bool(True))
        assert not node._planned_motion_active and node._wyaw_pending[0] == "home"
        assert _run(node, 6.0, until=lambda: node._wyaw_pending is None)
        assert node._wyaw.at_home
        assert _run(node, 2.0, until=lambda: node._planned_motion_active
                    or node._planning_thread is not None)
    finally:
        _close(node)


def test_waist_yaw_sim_publishes_target_for_loco_sim(ac_module):
    _reset_robot()
    node = _make_node(ac_module)
    st = _state(node)
    try:
        os.environ["G1_SIM_MODE"] = "true"
        node._waist = None                                    # Sim: kein WaistHold
        node._wyaw.target = None
        node._arms_controlled_callback(_bool(True))
        _run(node, 0.2)
        assert node._wyaw.home == 0.0
        _turn(node, 1.0, 0.5)
        assert st[-1]["sim"] and st[-1]["target_rad"] > HOME_YAW + math.radians(3)
        assert st[-1]["available"]
    finally:
        os.environ["G1_SIM_MODE"] = "false"
        _close(node)


def test_bringup_real_limits_waist_yaw():
    src = open(os.path.join(PKG, "launch", "bringup_real.launch.py")).read()
    assert "'waist_yaw_limit_deg': os.environ.get('G1_WAIST_YAW_LIMIT_DEG', '105.0')" in src


def test_mirror_command_real(ac_module):
    """Arm spiegeln (Controller Steuerkreuz + Kreuz -> arm_command type=mirror):
    ohne ENABLE abgelehnt (real UND ohne Sim-Auto-Enable), danach geplant in
    die Spiegelpose des KOMMANDIERTEN anderen Arms (Roll/Yaw gespiegelt),
    Status accepted -> executing -> reached; der Quell-Arm bleibt stehen.
    Im E-Stop abgelehnt."""
    from std_msgs.msg import String
    from g1pilot.manipulation import arm_command as acmd
    _reset_robot()
    node = _make_node(ac_module)
    states = []
    orig = node._publish_cmd_status

    def rec(state, **kw):
        states.append((kw.get("req_id") if kw.get("req_id") is not None
                       else node._cmd_req_id, state, kw.get("reason", "")))
        orig(state, **kw)
    node._publish_cmd_status = rec
    mirror = lambda rid, side: node._on_arm_command(String(data=json.dumps(  # noqa: E731
        {"id": rid, "type": "mirror", "side": side})))
    try:
        mirror("m0", "left")
        assert states[-1][:2] == ("m0", "rejected") and "ENABLE" in states[-1][2]
        assert not node.arms_enabled and not node._planned_motion_active

        node._arms_controlled_callback(_bool(True))
        _run(node, 2.4)
        _wait_planner(node)
        right = [-0.60, -0.35, 0.30, 0.80, 0.25, -0.20, 0.30]
        node._on_arm_command(String(data=json.dumps(
            {"id": "r", "type": "joints", "right": right})))
        assert _run(node, 25.0, until=lambda: (
            not node._planned_motion_active
            and float(np.max(np.abs(node._last_q_target[7:14] - right))) < 0.03))
        q_right = node._last_q_target[7:14].copy()

        mirror("m1", "left")
        goal = np.array(acmd.mirror_arm_joints(q_right))
        assert np.allclose(goal[[1, 2, 4, 6]], -q_right[[1, 2, 4, 6]])
        assert _run(node, 25.0, until=lambda: ("m1", "reached") in [s[:2] for s in states])
        # Ende jeder geplanten Fahrt (auch Pose anfahren): das Kommando bleibt
        # um die Restglaettung (ik_alpha) vor dem Ziel stehen, hier ~1 mrad
        # (0.06 Grad, < 1 mm an der Hand) -- das Ziel selbst ist exakt gespiegelt.
        _run(node, 1.0)
        assert float(np.max(np.abs(node._last_q_target[0:7] - goal))) < 2e-3
        assert float(np.max(np.abs(node._last_q_target[7:14] - q_right))) < 1e-6
        seq = [s[1] for s in states if s[0] == "m1"]
        assert seq[0] == "accepted" and "executing" in seq and seq[-1] == "reached"
        # Messung folgt (Fake-G1) ohne Sprung.
        assert float(np.max(np.abs(ROBOT.measured()[0:7] - goal))) < 0.05

        node._on_emergency_stop(_bool(True))
        mirror("m2", "right")
        assert states[-1][:2] == ("m2", "rejected") and "E-Stop" in states[-1][2]
    finally:
        _close(node)


# Messung liegt bleibend neben dem Soll: Schulter-Pitch nach vorn/oben,
# Ellbogen etwas gebeugt (wie Nils es real sah: Haende wandern hoch/vor).
TRACK_BIAS = np.array([-0.04, 0.0, 0.0, -0.03, 0.0, 0.0, 0.0,
                       -0.04, 0.0, 0.0, -0.03, 0.0, 0.0, 0.0])


def test_walk_grasp_toggle_does_not_creep(ac_module):
    """Regression (real 2026-10-22): am Controller L2 (GEHEN) / R2 (GREIFEN)
    im Wechsel -- R2 sendet start_balancing UND arms/enabled. Das wiederholte
    ENABLE darf das Soll nicht auf die Messung setzen, sonst wandern die Arme
    je Wechsel um den Regelfehler weiter (6 Wechsel = 6x Fehler)."""
    _reset_robot()
    node = _make_node(ac_module, walk_park_arms=False)
    try:
        _enable_and_raise(ac_module, node)
        ROBOT.sag = TRACK_BIAS.copy()
        _run(node, 0.5)
        q_cmd = node._last_q_target.copy()
        for _ in range(6):
            node._on_walk_mode(_bool(True))
            _run(node, 0.3)
            node._on_balance_mode(_bool(True))
            node._arms_controlled_callback(_bool(True))
            assert node._weight_state == "on" and node.arms_enabled
            _run(node, 0.3)
        assert float(np.max(np.abs(node._last_q_target - q_cmd))) < 1e-6, \
            "Soll ist beim Wechsel GEHEN -> GREIFEN gewandert"
        assert float(np.max(np.abs(ROBOT.measured() - (q_cmd + TRACK_BIAS)))) < 0.01
        assert float(np.max(np.abs(ROBOT.writes[-1][1] - q_cmd))) < 1e-6
    finally:
        _close(node)


def test_first_jog_after_mode_change_starts_at_command(ac_module):
    """Nach GREIFEN (IK-Halten) setzt das erste neue Ziel (Jog, Marker) am
    bisherigen Soll an -- nicht an der Messung (sonst springt das Soll um den
    Regelfehler und die Hand ratscht bei jedem Moduswechsel + Jog weiter)."""
    from geometry_msgs.msg import PoseStamped
    _reset_robot()
    node = _make_node(ac_module, walk_park_arms=False)
    try:
        _enable_and_raise(ac_module, node)
        ROBOT.sag = TRACK_BIAS.copy()
        _run(node, 0.5)
        q_cmd = node._last_q_target.copy()
        node._on_walk_mode(_bool(True))
        node._on_balance_mode(_bool(True))
        node._arms_controlled_callback(_bool(True))
        assert node._reset_after_home
        T = node._fk_current_ee("right")      # Ziel egal: geprueft wird der Ansatz
        ps = PoseStamped()
        ps.header.frame_id = "pelvis"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(
            float, T.translation)
        ps.pose.orientation.w = 1.0
        node._right_goal_callback(ps)
        assert not node._reset_after_home
        assert float(np.max(np.abs(node._last_q_target - q_cmd))) < 1e-6
    finally:
        _close(node)


def test_enable_after_estop_still_takes_measured_pose(ac_module):
    """Nach NOT-HALT (Arme schlaff) gilt weiter: ENABLE haelt an der MESSUNG
    (die Arme sind abgesackt, ein altes Soll waere ein Sprung)."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        _enable_and_raise(ac_module, node)
        node._on_emergency_stop(_bool(True))
        _run(node, 0.3)
        sag = np.array([0.15, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0] * 2)
        for k, j in enumerate(ARM_IDX):
            ROBOT.state.motor_state[j].q += float(sag[k])
        node._on_start(_bool(True))
        node._arms_controlled_callback(_bool(True))
        assert float(np.max(np.abs(node._last_q_target - ROBOT.measured()))) < 1e-6
    finally:
        _close(node)

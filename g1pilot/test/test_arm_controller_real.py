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
        self.writes = []            # (weight, q14, kp14) je Write
        self.freeze = False         # True: Arm festgehalten (Messung steht)

    def measured(self):
        return np.array([self.state.motor_state[j].q for j in ARM_IDX])

    def write(self, msg):
        w = float(msg.motor_cmd[29].q)
        q = np.array([msg.motor_cmd[j].q for j in ARM_IDX])
        kp = np.array([msg.motor_cmd[j].kp for j in ARM_IDX])
        self.writes.append((w, q, kp))
        if self.freeze:
            return
        soll = w * q + (1.0 - w) * ONBOARD
        if w > 0.0 and float(np.max(kp)) == 0.0:      # E-Stop-Slack: Arme sacken
            soll = self.measured()
        meas = self.measured()
        meas += 0.5 * (soll - meas)
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
    ROBOT.freeze = False
    for k, j in enumerate(ARM_IDX):
        ROBOT.state.motor_state[j].q = float(ONBOARD[k])


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
                      ee_velocity_limit=0.5, arm_velocity_limit=5.0)
    try:
        assert node.kp_low <= 80.0 and node.kd_low <= 3.0
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

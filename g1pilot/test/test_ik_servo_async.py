#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Marker-/Jog-IK im eigenen Prozess (ik_servo_process.py) -- ohne Hardware.

Hintergrund (Real-Test 2026-10-16): beim Joggen mit PS4/Demo-GUI-Joystick
ratterte der Arm. Ursache: die IK lief im 250-Hz-Regeltakt und kostete ~45 ms,
die Senderate an rt/arm_sdk fiel auf ~20 Hz (Sollwert-Treppe). Geprueft wird:
  * solve_servo (thread-/prozesssicher) rechnet dasselbe wie solve(),
  * das fruehe Ende (servo_step_tol) aendert das Ergebnis kaum,
  * der Prozess liefert dieselbe Loesung,
  * real: waehrend eines Jog-Ziel-Stroms bleibt main_loop kurz (volle
    Senderate), der Arm folgt, die Kommando-Schritte bleiben klein,
  * Ergebnisse einer alten IK-Generation werden nach ENABLE/Home verworfen.
"""
import os
import time

import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")

from g1pilot.utils.ik_solver import G1IKSolver  # noqa: E402

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
URDF = os.path.join(PKG, "description_files", "urdf", "g1_29dof_inspire_ftp.urdf")
MESHES = os.path.dirname(PKG)
HOME_L = np.array([0.3, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0])
HOME_R = np.array([0.3, -0.2, 0.0, 0.5, 0.0, 0.0, 0.0])
Q_R = np.array([-0.2, -0.25, 0.1, 0.9, 0.05, 0.1, 0.0])


@pytest.fixture(scope="module")
def ik():
    os.environ.setdefault("ROS_PACKAGE_PATH", MESHES)
    s = G1IKSolver(urdf_path=URDF, mesh_dir=MESHES, debug=False)
    s.set_rest_posture("left", HOME_L)
    s.set_rest_posture("right", HOME_R)
    return s


def _full(ik, q_right):
    from g1pilot.utils.joints_names import RIGHT_JOINT_INDICES_LIST, LEFT_JOINT_INDICES_LIST
    full = np.zeros(29)
    for i, j in enumerate(LEFT_JOINT_INDICES_LIST):
        full[j] = HOME_L[i]
    for i, j in enumerate(RIGHT_JOINT_INDICES_LIST):
        full[j] = q_right[i]
    return full


def _goal(ik, q_right, dp):
    full = _full(ik, q_right)
    q = ik._q_full_from(None, full)
    M = ik._frame_placement("right", q, pin.Data(ik.model))
    return pin.SE3(M.rotation.copy(), M.translation + np.asarray(dp))


def test_solve_servo_equals_solve(ik):
    goal = _goal(ik, Q_R, [0.03, 0.0, 0.02])
    full = _full(ik, Q_R)
    ik.servo_step_tol = None
    ik._goal_right = goal
    q_sync = ik.solve("right", None, full)
    q_srv, diverged = ik.solve_servo("right", goal, full, pin.Data(ik.model))
    assert not diverged
    assert np.allclose(q_sync, q_srv, atol=1e-12)


def test_step_tol_keeps_result_and_saves_iterations(ik):
    goal = _goal(ik, Q_R, [0.02, -0.01, 0.02])
    full = _full(ik, Q_R)
    data = pin.Data(ik.model)
    ik.servo_step_tol = None
    q_full_iter, _ = ik.solve_servo("right", goal, full, data)
    n_full = ik.last_iters
    ik.servo_step_tol = 2e-4
    q_early, _ = ik.solve_servo("right", goal, full, data)
    n_early = ik.last_iters
    ik.servo_step_tol = None
    assert n_full == ik.max_iter
    assert n_early < n_full / 2
    assert float(np.max(np.abs(q_early - q_full_iter))) < 0.01


def test_process_matches_inline(ik):
    from g1pilot.manipulation.ik_servo_process import IkServoProcess
    proc = IkServoProcess(URDF, MESHES, rest={"left": HOME_L, "right": HOME_R})
    proc.start_async()
    try:
        t_end = time.monotonic() + 120
        while not proc.ready and time.monotonic() < t_end:
            time.sleep(0.1)
        assert proc.ready, proc.error
        goal = _goal(ik, Q_R, [0.03, 0.01, 0.0])
        full = _full(ik, Q_R)
        proc.submit(7, {"right": goal}, full, {"null_space_gain": ik.null_space_gain,
                                               "servo_step_tol": 0.0})
        t_end = time.monotonic() + 10
        while proc.result() is None and time.monotonic() < t_end:
            time.sleep(0.005)
        gen, out = proc.result()
        assert gen == 7
        ik.servo_step_tol = None
        q_inline, _ = ik.solve_servo("right", goal, full, pin.Data(ik.model))
        assert np.allclose(out["right"], q_inline, atol=1e-9)
    finally:
        proc.shutdown()


# ── Real-Pfad gegen den Fake-G1 (wie test_arm_controller_real) ─────────────
rclpy = pytest.importorskip("rclpy")
from test_arm_controller_real import (  # noqa: E402,F401
    ac_module, _make_node, _close, _reset_robot, _bool, ROBOT, DT)


def _wait_ik(node, timeout=120.0):
    t_end = time.monotonic() + timeout
    while not (node._ik_proc is not None and node._ik_proc.ready) and time.monotonic() < t_end:
        time.sleep(0.1)
    assert node._ik_proc is not None and node._ik_proc.ready, "IK-Prozess nicht bereit"


def _goal_msg(p, q):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.header.frame_id = "pelvis"
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
    o = ps.pose.orientation
    o.x, o.y, o.z, o.w = q
    return ps


def test_jog_stream_keeps_full_send_rate(ac_module):
    """Jog-Ziel-Strom (30 Hz, 4 cm/s vorwaerts): main_loop bleibt kurz (die IK
    rechnet im Prozess), jeder Takt schreibt ein Kommando, der Arm folgt mit
    kleinen Schritten. Vorher kostete jeder Takt die volle IK (~45 ms)."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        _wait_ik(node)
        node._arms_controlled_callback(_bool(True))
        t_end = time.monotonic() + 2.4
        while time.monotonic() < t_end:
            node.main_loop()
            time.sleep(DT)
        M0 = node._fk_current_ee("right")
        qq = pin.Quaternion(M0.rotation)
        quat = (qq.x, qq.y, qq.z, qq.w)
        p0 = M0.translation.copy()
        ROBOT.writes.clear()
        durs = []
        t0 = time.monotonic()
        last_goal = -1.0
        while time.monotonic() - t0 < 1.5:
            now = time.monotonic() - t0
            if now - last_goal >= 1.0 / 30.0:
                last_goal = now
                node._right_goal_callback(_goal_msg(p0 + [min(now, 1.0) * 0.04, 0.0, 0.0], quat))
            t = time.perf_counter()
            node.main_loop()
            durs.append(time.perf_counter() - t)
            time.sleep(DT)
        assert node._ik_proc.result() is not None
        durs = np.array(durs[25:])
        assert float(np.percentile(durs, 95)) < 0.015, f"main_loop p95 {np.percentile(durs, 95)*1e3:.1f} ms"
        n = len(ROBOT.writes)
        assert n >= 0.9 * (len(durs) + 25)            # jeder Takt schreibt
        steps = [float(np.max(np.abs(q1 - q0))) for (_, q0, _), (_, q1, _)
                 in zip(ROBOT.writes, ROBOT.writes[1:])]
        assert max(steps) < 0.01                       # keine Sollwert-Treppe
        x = node._fk_current_ee("right").translation[0] - p0[0]
        assert x > 0.01, f"Hand nur {x * 100:.1f} cm vor"
    finally:
        _close(node)


def test_stale_ik_generation_is_ignored(ac_module):
    """Nach ENABLE (neue IK-Generation) gilt ein altes Ergebnis nicht mehr."""
    _reset_robot()
    node = _make_node(ac_module)
    try:
        _wait_ik(node)
        node._arms_controlled_callback(_bool(True))
        old = node._ik_gen
        node._ik_proc._result = (old, {"right": np.zeros(7), "left": np.zeros(7)})
        node._arms_controlled_callback(_bool(True))
        assert node._ik_gen != old
        held = {"left": False, "right": False}
        out = node._ik_async_targets(held, node.get_current_motor_q())
        assert out == {}
    finally:
        _close(node)

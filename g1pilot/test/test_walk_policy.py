#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit-Tests der Lauf-Policy-Anbindung (walk_policy) und der WALK->STAND-Uebergabe
(stand_balancer). Ohne ROS und ohne MuJoCo (numpy, onnxruntime, PyYAML, pytest).

Das Laufen selbst prueft der Headless-Sim-Test g1pilot/test_agile_walk_sim.py.
"""
import os

import numpy as np
import onnx
import pytest
import yaml

from g1pilot.navigation.stand_balancer import (
    BalancerParams, SettleGate, SettleParams, StandBalancer, LEG_STAND_POSE)
from g1pilot.navigation.walk_policy import (
    AgileHistoryPolicy, MjlabVelocityPolicy, load_walk_policy)
from g1pilot.utils.joints_names import JOINT_NAMES_ROS

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AGILE_DIR = os.path.join(PKG, "policies", "agile_velocity_g1")
LEGACY_DIR = os.path.join(PKG, "policies", "g1_wholebody")

# Quelle laut export_onnx.py (sha256 der AGILE-TorchScript-Datei).
SOURCE_SHA256 = "f58db6f61c7546941fe814ac6f571a11e894260e9e638f99eadd9125a4e8ab12"

# Referenz-Ausgaben der Original-TorchScript-Policy fuer feste Eingaben
# (x0 = 0, x1 = N(0,1)*0.5, x2 = N(0,1); numpy default_rng(42), nacheinander gezogen).
GOLDEN = np.array([
    [-0.172755, -0.160335, 0.138816, -0.135567, -0.003712, -0.028596, 0.023771,
     -0.075241, 0.27455, 0.266612, -0.432008, -0.420416, -0.309137, 0.305055],
    [-0.415453, -0.320342, -0.114605, 0.128013, -0.129316, 0.074258, 0.752312,
     -0.194874, -0.556943, 1.63833, -0.801728, 0.128705, -0.662477, 0.792587],
    [-0.724532, -0.474245, -0.581359, -0.697533, 0.174046, 0.22435, -0.541976,
     0.117518, -1.514602, 0.938754, 0.958457, 0.147841, -3.694241, -0.431809],
], dtype=np.float32)


@pytest.fixture(scope="module")
def dep():
    with open(os.path.join(AGILE_DIR, "deploy.yaml")) as f:
        return yaml.safe_load(f)


@pytest.fixture()
def policy():
    return load_walk_policy(AGILE_DIR)


# ── Deploy-Config + ONNX ─────────────────────────────────────────────────────

def test_deploy_yaml_consistent(dep):
    names = dep["joint_names"]
    assert len(names) == len(set(names)) == 14
    assert all(n in JOINT_NAMES_ROS.values() for n in names)
    assert "waist_yaw_joint" not in names and "waist_yaw_joint" in dep["held_joints"]
    for key in ("default_joint_pos", "stiffness", "damping"):
        assert len(dep[key]) == 14
    frame = sum(t["dim"] for t in dep["observations"])
    assert frame * dep["history_length"] == 255


def test_onnx_provenance_and_shapes():
    model = onnx.load(os.path.join(AGILE_DIR, "policy.onnx"))
    meta = {p.key: p.value for p in model.metadata_props}
    assert meta["source_sha256"] == SOURCE_SHA256
    assert meta["task"] == "Velocity-G1-History-v0"
    (inp,), (out,) = model.graph.input, model.graph.output
    assert inp.name == "obs" and inp.type.tensor_type.shape.dim[1].dim_value == 255
    assert out.name == "actions" and out.type.tensor_type.shape.dim[1].dim_value == 14


def test_onnx_matches_torchscript_reference():
    import onnxruntime as ort
    sess = ort.InferenceSession(os.path.join(AGILE_DIR, "policy.onnx"),
                                providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(42)
    x = np.zeros((3, 255), np.float32)
    x[1] = rng.normal(size=255).astype(np.float32) * 0.5
    x[2] = rng.normal(size=255).astype(np.float32)
    y = np.concatenate([sess.run(None, {"obs": x[i:i + 1]})[0] for i in range(3)])
    np.testing.assert_allclose(y, GOLDEN, atol=1e-4)


# ── Obs-Aufbau ───────────────────────────────────────────────────────────────

def _inputs(policy, k):
    """Zustand, dessen Terme sich eindeutig erkennen lassen (Wert haengt von k ab)."""
    q = np.zeros(29, np.float32)
    q[policy.idx] = policy.default + 0.01 * k
    dq = np.zeros(29, np.float32)
    dq[policy.idx] = 1.0 * k
    gyro = np.full(3, 2.0 * k, np.float32)
    grav = np.array([0.0, 0.0, -1.0], np.float32)
    cmd = np.array([0.3, 0.0, 0.0], np.float32)
    return q, dq, gyro, grav, cmd


def test_obs_layout_term_major_oldest_first(policy):
    obs = None
    for k in range(1, 4):
        obs = policy.build_obs(*_inputs(policy, k)).copy()
    o = obs[0]
    # Term 0: base_ang_vel * 0.2, 5 Zeitschritte x 3. Nach dem ersten Wert ist die
    # History damit gefuellt (k=1 in Slot 0..2), dann k=2, k=3 (neuester zuletzt).
    ang = o[0:15].reshape(5, 3)[:, 0]
    np.testing.assert_allclose(ang, 0.2 * 2.0 * np.array([1, 1, 1, 2, 3]), rtol=1e-6)
    # Term 1: projected_gravity, 5 x 3
    np.testing.assert_allclose(o[15:30].reshape(5, 3), np.tile([0, 0, -1], (5, 1)))
    # Term 2: velocity_commands, 5 x 3
    np.testing.assert_allclose(o[30:45].reshape(5, 3), np.tile([0.3, 0, 0], (5, 1)), rtol=1e-6)
    # Term 3: joint_pos_rel, 5 x 14 (q - default)
    pos = o[45:115].reshape(5, 14)
    np.testing.assert_allclose(pos[:, 0], 0.01 * np.array([1, 1, 1, 2, 3]), atol=1e-6)
    # Term 4: joint_vel_rel * 0.05, 5 x 14
    vel = o[115:185].reshape(5, 14)
    np.testing.assert_allclose(vel[:, 0], 0.05 * np.array([1, 1, 1, 2, 3]), rtol=1e-6)
    # Term 5: last_action (vor dem ersten step() noch 0)
    np.testing.assert_array_equal(o[185:255], 0.0)


def test_small_command_is_zeroed(policy):
    np.testing.assert_array_equal(policy.command_for_policy([0.05, 0.05, 0.0]), 0.0)
    np.testing.assert_allclose(policy.command_for_policy([0.08, 0.0, 0.08]), [0.08, 0.0, 0.08])


def test_step_targets_and_motor_mapping(policy):
    q, dq, gyro, grav, cmd = _inputs(policy, 0)
    t = policy.step(q, dq, gyro, grav, cmd)
    names = [JOINT_NAMES_ROS[int(i)] for i in t.idx]
    assert names[:14] == policy.joint_names and names[14] == "waist_yaw_joint"
    # Unitree-Motorindizes: Beine 0..11, Taille yaw/roll/pitch 12..14, keine Arme.
    assert sorted(t.idx) == list(range(15))
    np.testing.assert_allclose(t.q[:14], policy.default + 0.5 * policy.last_action, rtol=1e-6)
    assert t.q[14] == 0.0 and t.kp[14] == 300.0
    # last_action fliesst in die naechste Obs ein
    obs = policy.build_obs(q, dq, gyro, grav, cmd)
    np.testing.assert_allclose(obs[0, 185 + 4 * 14:255], policy.last_action, rtol=1e-6)


def test_reset_clears_history(policy):
    policy.step(*_inputs(policy, 5))
    policy.reset()
    assert not policy.last_action.any()
    o = policy.build_obs(*_inputs(policy, 1))[0]
    np.testing.assert_allclose(o[0:15].reshape(5, 3)[:, 0], 0.4)


def test_loader_dispatch():
    assert isinstance(load_walk_policy(AGILE_DIR), AgileHistoryPolicy)
    assert load_walk_policy(AGILE_DIR).needs_arm_pose is False
    legacy = load_walk_policy(LEGACY_DIR)
    assert isinstance(legacy, MjlabVelocityPolicy) and legacy.needs_arm_pose is True


# ── WALK -> STAND ────────────────────────────────────────────────────────────

def test_settle_gate_waits_for_quiet():
    p = SettleParams(min_s=0.6, quiet_s=0.3, gyro_max=0.3, dq_max=0.5, timeout_s=3.0)
    g = SettleGate()
    g.start(0.0)
    still, moving = np.zeros(3), np.array([0.0, 0.5, 0.0])
    t = 0.0
    while t < 1.0:                       # wackelt noch -> keine Uebergabe
        assert g.update(t, moving, np.zeros(12), p) is None
        t += 0.02
    res = None
    while res is None and t < 2.0:
        res = g.update(t, still, np.zeros(12), p)
        t += 0.02
    assert res == "ruhig" and 1.29 < t < 1.36 and not g.active


def test_settle_gate_timeout_and_cancel():
    p = SettleParams(timeout_s=1.0)
    g = SettleGate()
    g.start(0.0)
    assert g.update(0.5, np.ones(3), np.zeros(12), p) is None
    assert g.update(1.0, np.ones(3), np.zeros(12), p) == "timeout"
    g.start(2.0)
    g.cancel()
    assert not g.active and g.update(5.0, np.zeros(3), np.zeros(12), p) is None


def test_balancer_hold_pose_keeps_entry_pose():
    crouch = np.array([-0.47, 0.16, -0.02, 0.83, -0.38, -0.13,
                       -0.44, -0.16, 0.02, 0.84, -0.41, 0.13], np.float32)
    p = BalancerParams()
    up = np.array([0.0, 0.0, -1.0])
    for hold, goal in ((True, crouch), (False, LEG_STAND_POSE)):
        b = StandBalancer(0.02)
        b.enter(hold_pose=hold)
        b.step(crouch, up, np.zeros(3), 0.0, p)
        c = b.step(crouch, up, np.zeros(3), 5.0, p)        # Rampe vorbei
        np.testing.assert_allclose(c.leg_q, goal, atol=1e-6)

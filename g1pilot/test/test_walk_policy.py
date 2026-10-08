#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit-Tests der Lauf-Policy-Anbindung (walk_policy), des Stand-Balancers
(stand_balancer: WALK->STAND-Uebergabe, Schwerpunkt-Fuehrung, Kipp-Erkennung) und
des Schwerpunkt-Modells (com_model). Ohne ROS (numpy, onnxruntime, PyYAML, pytest;
fuer die Modell-Tests zusaetzlich mujoco + pinocchio, sonst uebersprungen).

Das Laufen/Stehen selbst prueft der Headless-Sim-Test g1pilot/test_agile_walk_sim.py.
"""
import math
import os

import numpy as np
import onnx
import pytest
import yaml

from g1pilot.navigation.stand_balancer import (
    BalancerParams, SettleGate, SettleParams, StandBalancer, TipDetector, TipParams,
    foot_tilt, LEG_STAND_POSE)
from g1pilot.navigation.walk_policy import (
    AgileHistoryPolicy, MjlabVelocityPolicy, load_walk_policy)
from g1pilot.utils.joints_names import JOINT_NAMES_ROS

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AGILE_DIR = os.path.join(PKG, "policies", "agile_velocity_g1")
VH_DIR = os.path.join(PKG, "policies", "agile_velocity_height_g1")
LEGACY_DIR = os.path.join(PKG, "policies", "g1_wholebody")
G1_MJCF_DIR = os.path.join(os.path.dirname(PKG), "unitree_mujoco", "unitree_robots", "g1")

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


# Velocity-Height: sha256 der Quelle und Gelenk-Sollwerte des offiziellen LEAPP-Exports
# (agile/data/policy/velocity_height_g1/leapp, ONNX MIT eingebauter Obs-Verarbeitung)
# fuer die Zustandsfolge aus _vh_states() mit cmd (0.3, -0.1, 0.4), Hoehe 0.72; History
# beim Start wie nach unserem Reset (erster Messwert in allen Slots, last_action 0).
# Zeilen: Schritte 0, 1, 5, 11; Spalten in Policy-Reihenfolge (joint_names).
VH_SOURCE_SHA256 = "240a5ce0b121837eba2f886a523d284a2a263dceb78d3132639eaf74ad7650f2"
VH_GOLDEN_STEPS = (0, 1, 5, 11)
VH_GOLDEN = np.array([
    [-0.100355, -0.124496, 0.355736, -0.182847, -0.116640, 0.032075,
     0.341258, 0.592636, 0.682159, 0.203977, -0.068031, 0.685486],
    [-0.194713, -0.218778, 0.148193, 0.161302, -0.153555, -0.058706,
     0.418095, 0.652873, 0.132189, 0.081751, -0.106180, -0.008398],
    [-0.204136, -0.304631, 0.205264, 0.066978, -0.234623, -0.077577,
     0.285383, 0.792328, 1.067904, -0.054477, -0.791941, 0.147921],
    [-0.503712, -0.089088, 0.179239, -0.135192, -0.079549, 0.124522,
     0.596612, 0.631218, 0.039025, 0.672070, -0.412682, -0.038520],
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
    # Stand-Pose: 12 Beinwerte, links/rechts gespiegelt, Pitch-Kette summiert zu 0
    # (Fuesse flach bei aufrechtem Becken)
    sp = np.array(dep["stand_pose_legs"])
    assert sp.shape == (12,)
    np.testing.assert_allclose(sp[[0, 3, 4]], sp[[6, 9, 10]])
    np.testing.assert_allclose(sp[[1, 2, 5]], -sp[[7, 8, 11]])
    assert abs(sp[0] + sp[3] + sp[4]) < 1e-6


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
    agile = load_walk_policy(AGILE_DIR)
    assert isinstance(agile, AgileHistoryPolicy) and agile.needs_arm_pose is False
    assert agile.stand_leg_pose.shape == (12,)
    vh = load_walk_policy(VH_DIR)
    assert isinstance(vh, AgileHistoryPolicy) and vh.needs_arm_pose is False
    assert vh.stand_leg_pose.shape == (12,) and vh.num_obs == 400
    assert vh.rescue_policy_name == "agile_velocity_g1"
    assert agile.rescue_policy_name is None
    legacy = load_walk_policy(LEGACY_DIR)
    assert isinstance(legacy, MjlabVelocityPolicy) and legacy.needs_arm_pose is True
    assert legacy.stand_leg_pose is None        # -> gerade Standpose des Balancers


# ── Velocity-Height-Policy ───────────────────────────────────────────────────

def test_vh_deploy_yaml_consistent():
    with open(os.path.join(VH_DIR, "deploy.yaml")) as f:
        d = yaml.safe_load(f)
    names = d["joint_names"]
    assert len(names) == len(set(names)) == 12 and all("waist" not in n for n in names)
    for key in ("default_joint_pos", "stiffness", "damping", "action_scale"):
        assert len(d[key]) == 12
    obs_names = d["obs_joint_names"]
    assert sorted(obs_names) == sorted(JOINT_NAMES_ROS[i] for i in range(29))
    assert len(d["obs_default_joint_pos"]) == 29
    assert set(d["held_joints"]) == {"waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"}
    frame = sum(t["dim"] for t in d["observations"])
    assert frame * d["history_length"] == 400
    # PD-Stand-Pose = Hocke der AGILE-Velocity-Policy, die auch abfaengt
    with open(os.path.join(AGILE_DIR, "deploy.yaml")) as f:
        agile = yaml.safe_load(f)
    np.testing.assert_allclose(d["stand_pose_legs"], agile["stand_pose_legs"])
    assert d["rescue_policy"] == "agile_velocity_g1"


def test_vh_onnx_provenance_and_shapes():
    model = onnx.load(os.path.join(VH_DIR, "policy.onnx"))
    meta = {p.key: p.value for p in model.metadata_props}
    assert meta["source_sha256"] == VH_SOURCE_SHA256
    assert meta["task"] == "Velocity-Height-G1-History-v0"
    (inp,), (out,) = model.graph.input, model.graph.output
    assert inp.type.tensor_type.shape.dim[1].dim_value == 400
    assert out.type.tensor_type.shape.dim[1].dim_value == 12


def _vh_states(n=12, seed=7):
    """Zufaellige, aber feste Zustaende (Unitree-Motorreihenfolge, Quaternion wxyz)."""
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        q = rng.normal(0, 0.3, 29).astype(np.float32)
        dq = rng.normal(0, 2.0, 29).astype(np.float32)
        ax = rng.normal(0, 0.15, 3)
        ang = float(np.linalg.norm(ax))
        ax /= ang
        quat = np.array([np.cos(ang / 2), *(np.sin(ang / 2) * ax)])
        gyro = rng.normal(0, 0.5, 3).astype(np.float32)
        out.append((q, dq, quat, gyro))
    return out


def test_vh_matches_official_leapp_export():
    """Ganze Kette (Obs aus 29 Gelenken, History, Netz, Aktions-Skalierung je Gelenk,
    Knoechel-Skalierung 1.0, Gelenk-Zuordnung) gegen den NVIDIA-Export."""
    from g1pilot.navigation.walk_policy import get_gravity_orientation
    vh = load_walk_policy(VH_DIR)
    assert vh.base_height == pytest.approx(0.72)
    vh.reset()
    got = []
    for q, dq, quat, gyro in _vh_states():
        t = vh.step(q, dq, gyro, get_gravity_orientation(quat), [0.3, -0.1, 0.4])
        got.append(t.q[:12].copy())
        # Taille gehalten wie im Training
        np.testing.assert_allclose(t.q[12:], 0.0)
        np.testing.assert_allclose(t.kp[12:], 300.0)
    np.testing.assert_allclose(np.array(got)[list(VH_GOLDEN_STEPS)], VH_GOLDEN, atol=2e-4)
    # Gelenk-Zuordnung: Ausgabe i gehoert zu Motor JOINT_NAMES_ROS-Index von joint_names[i]
    for k, name in enumerate(vh.joint_names):
        assert JOINT_NAMES_ROS[int(t.idx[k])] == name


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


CROUCH = np.array([-0.47, 0.16, -0.02, 0.83, -0.38, -0.13,
                   -0.44, -0.16, 0.02, 0.84, -0.41, 0.13], np.float32)
UP = np.array([0.0, 0.0, -1.0])
Z12 = np.zeros(12, np.float32)


def test_balancer_hold_vs_goal():
    p = BalancerParams()
    # ohne Ziel (Uebergabe aus WALK): vorgefundene Pose halten
    b = StandBalancer(0.02)
    b.enter()
    b.step(CROUCH, Z12, UP, np.zeros(3), 0.0, p)
    c = b.step(CROUCH, Z12, UP, np.zeros(3), 5.0, p)        # Rampe vorbei
    np.testing.assert_allclose(c.leg_q, CROUCH, atol=1e-6)
    # mit Ziel (aus HOLD): Ziel schon im ersten Befehl (die Bridge stellt dort auf)
    b = StandBalancer(0.02)
    b.enter(LEG_STAND_POSE)
    c = b.step(CROUCH, Z12, UP, np.zeros(3), 0.0, p)
    np.testing.assert_allclose(c.leg_q, LEG_STAND_POSE, atol=1e-6)


def test_balancer_com_shift_direction_and_limit():
    p = BalancerParams()
    b = StandBalancer(0.02)
    b.enter(CROUCH)
    t = 0.0
    for _ in range(200):                # Schwerpunkt 3 cm hinter dem Soll
        c = b.step(CROUCH, Z12, UP, np.zeros(3), t, p, com_x=p.com_target_m - 0.03)
        t += 0.02
    # Becken nach vorn: Knoechel-Pitch kleiner, Hueft-Pitch groesser, Rest gleich
    assert 0.0 < b.shift <= p.com_limit + 1e-9
    np.testing.assert_allclose(c.leg_q[[4, 10]], CROUCH[[4, 10]] - b.shift, atol=1e-6)
    np.testing.assert_allclose(c.leg_q[[0, 6]], CROUCH[[0, 6]] + b.shift, atol=1e-6)
    np.testing.assert_allclose(c.leg_q[[1, 2, 3, 5]], CROUCH[[1, 2, 3, 5]], atol=1e-6)
    for _ in range(500):                # dauerhaft weit daneben -> Begrenzung
        b.step(CROUCH, Z12, UP, np.zeros(3), t, p, com_x=-1.0)
        t += 0.02
    assert b.shift == pytest.approx(p.com_limit)
    # ohne Modell-Schwerpunkt keine Verschiebung
    b2 = StandBalancer(0.02)
    b2.enter(CROUCH)
    for k in range(50):
        c = b2.step(CROUCH, Z12, UP, np.zeros(3), 0.02 * k, p, com_x=None)
    assert b2.shift == 0.0
    np.testing.assert_allclose(c.leg_q, CROUCH, atol=1e-6)


def _quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def test_foot_tilt_flat_stances_and_pelvis_tilt():
    stand = load_walk_policy(AGILE_DIR).stand_leg_pose
    # Die gemessene Policy-Hocke rollt die Sohlen um ~1.1 Grad nach aussen
    # (Hueft-Roll 0.16 vs. Knoechel-Roll 0.13); die gerade Pose ist exakt flach.
    for pose, tol in ((LEG_STAND_POSE, 1e-6), (stand, 0.02)):
        assert np.all(foot_tilt([1, 0, 0, 0], pose) < tol)
        # Becken gedreht um die Hochachse: Sohlen-Neigung unveraendert
        np.testing.assert_allclose(foot_tilt(_quat_from_euler(0, 0, 1.0), pose),
                                   foot_tilt([1, 0, 0, 0], pose), atol=1e-9)
    # Ganzer Roboter steif 0.1 rad nach vorn gekippt -> Sohlen 0.1 rad schief
    np.testing.assert_allclose(foot_tilt(_quat_from_euler(0, 0.1, 0), LEG_STAND_POSE), 0.1,
                               atol=1e-6)


def test_foot_tilt_matches_mujoco():
    mujoco = pytest.importorskip("mujoco")
    m = mujoco.MjModel.from_xml_path(os.path.join(G1_MJCF_DIR, "g1_29dof_inspire_ftp.xml"))
    d = mujoco.MjData(m)
    feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{s}_ankle_roll_link")
            for s in ("left", "right")]
    rng = np.random.default_rng(3)
    for _ in range(50):
        legs = CROUCH + rng.normal(0, 0.15, 12)
        quat = _quat_from_euler(*rng.normal(0, 0.2, 3))
        d.qpos[:] = m.qpos0
        d.qpos[3:7] = quat
        d.qpos[7:19] = legs
        mujoco.mj_kinematics(m, d)
        ref = [math.acos(min(1.0, d.xmat[b][8])) for b in feet]
        np.testing.assert_allclose(foot_tilt(quat, legs), ref, atol=1e-3)


def test_tip_detector():
    p = TipParams(foot_tilt_deg=6.0, tilt_max=0.2, debounce_s=0.04)
    stand = load_walk_policy(AGILE_DIR).stand_leg_pose
    td = TipDetector()
    flat, tipped = [1, 0, 0, 0], _quat_from_euler(0, math.radians(8), 0)
    g_up = np.array([0.0, 0.0, -1.0])
    assert not td.update(0.00, flat, stand, g_up, p)
    assert not td.update(0.02, tipped, stand, g_up, p)     # entprellt
    assert td.update(0.06, tipped, stand, g_up, p)
    assert not td.update(0.08, flat, stand, g_up, p)       # wieder flach -> zurueck
    # starke Becken-Neigung allein reicht auch
    assert not td.update(1.00, flat, stand, np.array([0.25, 0.0, -0.97]), p)
    assert td.update(1.05, flat, stand, np.array([0.25, 0.0, -0.97]), p)


def test_com_model_matches_mujoco():
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("pinocchio")
    from g1pilot.navigation.com_model import ComModel
    for name in ("g1_29dof.xml", "g1_29dof_inspire_ftp.xml"):
        path = os.path.join(G1_MJCF_DIR, name)
        cm = ComModel(path)
        m = mujoco.MjModel.from_xml_path(path)
        d = mujoco.MjData(m)
        assert cm.mass == pytest.approx(float(m.body_subtreemass[1]), rel=1e-6)
        feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{s}_ankle_roll_link")
                for s in ("left", "right")]
        adr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, JOINT_NAMES_ROS[i])]
               for i in range(29)]
        rng = np.random.default_rng(5)
        for _ in range(20):
            q29 = rng.normal(0, 0.3, 29)
            d.qpos[:] = m.qpos0
            d.qpos[adr] = q29           # Inspire-MJCF: Fingergelenke liegen dazwischen
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            mid = d.xpos[feet].mean(axis=0)
            ref = d.xmat[feet[0]].reshape(3, 3).T @ (d.subtree_com[1] - mid)
            np.testing.assert_allclose(cm.com_in_foot(q29), ref, atol=1e-6)

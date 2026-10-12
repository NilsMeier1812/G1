#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tests des Armplaners (arm_geometry + arm_planner) ohne ROS.

Braucht pinocchio (+ coal fuer die Mesh-Selbstkollision); fehlt es, werden
die Tests uebersprungen. Szene: Tisch vor dem Roboter (Oberkante 2 cm ueber
dem Becken) mit Greif-Box -- entspricht Arbeitsplatz 1.
"""
import os
import time

import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")

from g1pilot.manipulation.arm_geometry import ArmGeometry, Environment, HAND_LINK_MIN  # noqa: E402
from g1pilot.manipulation.arm_planner import plan_arms, joint_indices_for  # noqa: E402
from g1pilot.utils.ik_solver import G1IKSolver  # noqa: E402
from g1pilot.utils.joints_names import JOINT_LIMITS_RAD  # noqa: E402

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
URDF = os.path.join(PKG, "description_files", "urdf", "g1_29dof_inspire_ftp.urdf")
# package://g1pilot/... wird relativ zum Elternordner des Pakets aufgeloest.
MESHES = os.path.dirname(PKG)

SICHER_L = np.array([1.449, 0.426, 0.641, -0.374, -0.154, -1.008, -0.086])
UEBER_TISCH_L = np.array([-0.8633, 0.0742, 0.1242, 0.365, -0.0604, 0.4799, 0.0992])
ABSENKEN_L = np.array([-0.8663, 0.0374, 0.1888, 1.295, 0.0216, -0.3539, 0.0691])
WALK = {"left": np.array([0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0]),
        "right": np.array([0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0])}
TOP = 0.02


def _box(name, c, h, cls="obstacle"):
    return {"name": name, "cls": cls, "pos": list(c), "half_extents": list(h),
            "quat": [1.0, 0.0, 0.0, 0.0]}


def _table(top=TOP):
    return [_box("tisch", (0.62, 0.0, top - 0.4), (0.4, 0.8, 0.4)),
            _box("box", (0.43, 0.13, top + 0.05), (0.05, 0.05, 0.05), "grasp")]


@pytest.fixture(scope="module")
def models():
    if not os.path.exists(URDF):
        pytest.skip("URDF fehlt")
    ik = G1IKSolver(urdf_path=URDF, mesh_dir=MESHES, debug=False)
    geom = ArmGeometry(ik.model, ik.collision_model)
    return ik, geom, ik.make_scratch_buffers()


def _plan(models, scene, cur, goal, params=None):
    ik, geom, scratch = models
    sides = [s for s in ("left", "right") if s in goal]
    full = np.zeros(29)
    for s in ("left", "right"):
        full[joint_indices_for([s])] = cur[s]
    geom.set_base(full)
    geom.set_environment(Environment(scene))
    idx = joint_indices_for(sides)

    def self_check(qN):
        f = full.copy()
        f[idx] = qN
        return ik.arm_command_in_collision(f, hard=True, data=scratch["data"],
                                           cdata=scratch["cdata_hard"])

    return plan_arms(geom, sides, np.concatenate([cur[s] for s in sides]),
                     np.concatenate([goal[s] for s in sides]),
                     [JOINT_LIMITS_RAD[j] for j in idx], params,
                     self_check=self_check, walk_pose=WALK)


def _hand_table_dist(geom, Q, side="left"):
    """Abstand der Hand-Kugeln zu Nicht-Greif-Objekten je Bahnpunkt + TCP."""
    fk = geom.fk(Q, [side])[side]
    ei = geom.env_idx(side)
    d, _ = geom.env_dist(fk["c"][:, ei], geom.radii(side)[ei])
    hand = geom.links(side)[ei] >= HAND_LINK_MIN
    d = np.where(geom.env.is_grasp[None, None, :], np.inf, d[:, hand])
    return d.min(axis=(1, 2)), fk["tcp"]


def _hand_on_table(models):
    """IK-Konfiguration, in der die linke Hand auf der Tischplatte aufliegt."""
    ik, geom, _ = models
    full = np.zeros(29)
    full[joint_indices_for(["left"])] = UEBER_TISCH_L
    full[joint_indices_for(["right"])] = WALK["right"]
    geom.set_base(full)
    geom.set_environment(Environment(_table()))
    R = ik._frame_placement("left", ik._q_full_from(None, full), pin.Data(ik.model)).rotation
    for z in np.arange(0.20, 0.0, -0.005):
        res = ik.solve_pose("left", pin.SE3(R, np.array([0.30, 0.22, z])), full)
        if res["q"] is None or res["pos_err_m"] > 0.01:
            continue
        d, _ = _hand_table_dist(geom, np.asarray(res["q"])[None])
        if d[0] <= 0.0:
            return np.asarray(res["q"])
    pytest.skip("keine Handpose auf dem Tisch gefunden")


def test_fk_matches_pinocchio(models):
    ik, geom, _ = models
    rng = np.random.default_rng(0)
    full = np.zeros(29)
    geom.set_base(full)
    data = pin.Data(ik.model)
    for _ in range(5):
        q7 = rng.uniform(-1.0, 1.0, 7)
        full[joint_indices_for(["left"])] = q7
        M = ik._frame_placement("left", ik._q_full_from(None, full), data)
        fk = geom.fk(q7[None], ["left"])["left"]
        # wrist_yaw-Frame-Orientierung stimmt mit pinocchio ueberein
        assert np.allclose(fk["R6"][0], M.rotation, atol=1e-6)


def test_distance_gradient(models):
    _, geom, _ = models
    geom.set_environment(Environment(_table()))
    rng = np.random.default_rng(1)
    C = rng.uniform([0.1, -0.3, -0.2], [0.6, 0.3, 0.3], (20, 3))
    r = np.full(20, 0.02)
    d, n = geom.env_dist(C, r)
    eps = 1e-6
    for k in range(3):
        dC = np.zeros(3)
        dC[k] = eps
        d2, _ = geom.env_dist(C + dC, r)
        assert np.allclose((d2 - d) / eps, n[..., k], atol=1e-4)


def test_hand_may_rest_on_table(models):
    """Hand auf den Tisch legen und wieder abheben ist erlaubt und schnell."""
    on = _hand_on_table(models)
    for cur, goal in (({"left": UEBER_TISCH_L, "right": WALK["right"]}, {"left": on}),
                      ({"left": on, "right": WALK["right"]}, {"left": UEBER_TISCH_L})):
        r = _plan(models, _table(), cur, goal)
        assert r["ok"], r["reason"]
        assert r["plan_time_s"] < 2.5


def test_mid_motion_keeps_clearance(models):
    """Von der sicheren Pose auf den Tisch: in der Mitte der Bahn (TCP weiter
    als relax_radius von Start und Ziel) haelt die Hand den vollen Abstand,
    am Ziel liegt sie auf."""
    _, geom, _ = models
    on = _hand_on_table(models)
    r = _plan(models, _table(), {"left": SICHER_L, "right": WALK["right"]}, {"left": on})
    assert r["ok"], r["reason"]
    d, tcp = _hand_table_dist(geom, r["q"])
    far = ((np.linalg.norm(tcp - tcp[0], axis=1) > 0.15)
           & (np.linalg.norm(tcp - tcp[-1], axis=1) > 0.15))
    assert far.any()
    assert d[far].min() >= r["clearance"] - 0.006
    assert d[-1] <= 0.005
    # Keine Umwege: TCP-Weg hoechstens 1.6x Luftlinie
    path = np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum()
    assert path < 1.6 * np.linalg.norm(tcp[-1] - tcp[0])


def test_start_inside_table_is_not_stuck(models):
    """Steckt die Hand am Start 1 cm im Tisch (Roboter naeher gekommen),
    kommt der Planer trotzdem heraus."""
    on = _hand_on_table(models)
    sunk = _table(top=TOP + 0.01)
    r = _plan(models, sunk, {"left": on, "right": WALK["right"]}, {"left": UEBER_TISCH_L})
    assert r["ok"], r["reason"]


def test_goal_behind_wall_is_rejected_quickly(models):
    wall = _table() + [_box("wand", (0.30, 0.2, 0.2), (0.01, 0.5, 0.5))]
    t0 = time.monotonic()
    r = _plan(models, wall, {"left": SICHER_L, "right": WALK["right"]}, {"left": UEBER_TISCH_L})
    assert not r["ok"]
    assert r["reason"] == "goal_in_collision"
    assert time.monotonic() - t0 < 1.0


def test_grasp_object_may_be_touched(models):
    """Absenken an die Greif-Box: die Hand darf die Box beruehren."""
    r = _plan(models, _table(), {"left": UEBER_TISCH_L, "right": WALK["right"]},
              {"left": ABSENKEN_L})
    assert r["ok"], r["reason"]


def test_trajectory_respects_limits(models):
    r = _plan(models, _table(), {"left": SICHER_L, "right": WALK["right"]},
              {"left": UEBER_TISCH_L}, {"vel_limit": 1.0})
    assert r["ok"], r["reason"]
    t, q = r["t"], r["q"]
    v = np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]
    assert v.max() <= 1.0 * 1.05
    assert np.allclose(q[0], SICHER_L) and np.allclose(q[-1], UEBER_TISCH_L)


def test_reachable_drops_far_objects(models):
    _, geom, _ = models
    geom.set_base(np.zeros(29))
    env = Environment(_table() + [_box("fern", (1.4, 0.0, 0.0), (0.1, 0.1, 0.1))])
    assert env.n == 3
    near = geom.reachable(env, ["left", "right"])
    assert near.n == 2 and "fern" not in near.names


def test_cancel(models):
    _, geom, _ = models
    full = np.zeros(29)
    geom.set_base(full)
    post = _table() + [_box("pfosten", (0.28, 0.22, 0.25), (0.02, 0.02, 0.25))]
    geom.set_environment(Environment(post))
    t0 = time.monotonic()
    r = plan_arms(geom, ["left"], SICHER_L, UEBER_TISCH_L, [JOINT_LIMITS_RAD[j] for j in
                                                             joint_indices_for(["left"])],
                  should_stop=lambda: time.monotonic() - t0 > 0.05)
    assert r["reason"] in ("cancelled", "direct", "optimized")
    assert time.monotonic() - t0 < 1.0

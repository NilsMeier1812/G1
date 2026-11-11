#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hand-Profile (eigene starre Haende in der Sim, z. B. die Box-Haken).

Prueft die Auswahl (G1_HANDS / G1_INSPIRE_HANDS, real unberuehrt), dass die
committeten Modelle zu hand.yaml/STL passen, und dass MuJoCo-Modell und URDF
(IK, Kollisions-Gate, Armplaner) dieselbe Hand am selben Ort haben.
"""
import os
import subprocess
import sys

import numpy as np
import pytest

from g1pilot.utils import hand_profile as hp

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REPO = os.path.dirname(PKG)
G1_DIR = os.path.join(REPO, "unitree_mujoco", "unitree_robots", "g1")
TOOLS = os.path.join(G1_DIR, "tools")
GEN = os.path.join(TOOLS, "gen_hand_profile.py")
URDF_DIR = os.path.join(PKG, "description_files", "urdf")
PROFILE = "haken"


# ── Auswahl ──────────────────────────────────────────────────────────────
@pytest.mark.parametrize("env,profile,urdf", [
    ({}, "rubber", "g1_29dof_inspire_ftp.urdf"),
    ({"G1_INSPIRE_HANDS": "1"}, "inspire", "g1_29dof_inspire_ftp.urdf"),
    ({"G1_INSPIRE_HANDS": "0"}, "rubber", "g1_29dof_inspire_ftp.urdf"),
    ({"G1_HANDS": "inspire"}, "inspire", "g1_29dof_inspire_ftp.urdf"),
    ({"G1_HANDS": "rubber", "G1_INSPIRE_HANDS": "1"}, "rubber", "g1_29dof_inspire_ftp.urdf"),
    ({"G1_HANDS": "haken", "G1_SIM_MODE": "true", "G1_INSPIRE_HANDS": "1"},
     "haken", "g1_29dof_hand_haken.urdf"),
    ({"G1_HANDS": " Haken ", "G1_SIM_MODE": "true"}, "haken", "g1_29dof_hand_haken.urdf"),
    # Real-Pfad: eigene Profile werden ignoriert -> wie bisher Inspire-URDF
    ({"G1_HANDS": "haken", "G1_SIM_MODE": "false", "G1_INSPIRE_HANDS": "1"},
     "inspire", "g1_29dof_inspire_ftp.urdf"),
    ({"G1_HANDS": "haken"}, "rubber", "g1_29dof_inspire_ftp.urdf"),
    # Unsinn im Namen -> kein Pfad-Basteln
    ({"G1_HANDS": "../x", "G1_SIM_MODE": "true"}, "rubber", "g1_29dof_inspire_ftp.urdf"),
])
def test_profile_selection(env, profile, urdf):
    assert hp.hand_profile(env) == profile
    assert hp.urdf_file_name(env) == urdf
    assert hp.inspire_active(env) == (profile == "inspire")


def test_mjcf_names():
    assert hp.mjcf_file_name({"G1_INSPIRE_HANDS": "1"}) == "g1_29dof_inspire_ftp.xml"
    assert hp.mjcf_file_name({}) == "g1_29dof.xml"
    assert hp.mjcf_file_name({"G1_HANDS": "haken", "G1_SIM_MODE": "true"}) == \
        "g1_29dof_hand_haken.xml"


def test_urdf_path_fallback(tmp_path):
    (tmp_path / "g1_29dof_inspire_ftp.urdf").write_text("<robot/>")
    msgs = []
    p = hp.urdf_path(str(tmp_path), {"G1_HANDS": "gibtsnicht", "G1_SIM_MODE": "true"},
                     log=msgs.append)
    assert p.endswith("g1_29dof_inspire_ftp.urdf") and msgs
    p = hp.urdf_path(URDF_DIR, {"G1_HANDS": PROFILE, "G1_SIM_MODE": "true"})
    assert p.endswith(f"g1_29dof_hand_{PROFILE}.urdf")


def _sim_config(env):
    code = ("import config; print(config.ROBOT_SCENE); print(config._INSPIRE); "
            "print(config.GRASP_BOX)")
    e = {k: v for k, v in os.environ.items() if not k.startswith("G1_")}
    e.update(env)
    out = subprocess.run([sys.executable, "-c", code], env=e, capture_output=True, text=True,
                         cwd=os.path.join(REPO, "unitree_mujoco", "simulate_python"), check=True)
    return out.stdout.split()


@pytest.mark.parametrize("env,scene,inspire", [
    ({}, "scene.xml", "False"),
    ({"G1_INSPIRE_HANDS": "1"}, "scene_inspire_ftp.xml", "True"),
    ({"G1_HANDS": "haken", "G1_INSPIRE_HANDS": "1"}, "scene_hand_haken.xml", "False"),
    ({"G1_HANDS": "inspire"}, "scene_inspire_ftp.xml", "True"),
    ({"G1_HANDS": "haken", "G1_ENV": "scene_A5"}, "scene_env_scene_A5.xml", "False"),
])
def test_sim_config_scene(env, scene, inspire):
    path, insp, grasp_box = _sim_config(env)
    assert path.endswith("/" + scene)
    assert insp == inspire
    assert grasp_box == inspire          # Greif-Testbox nur mit Inspire-Handflaeche


# ── Generator ───────────────────────────────────────────────────────────
def test_generated_files_current():
    """Committete Modelle passen zu hand.yaml + STL (sonst: hand_tool.sh build)."""
    r = subprocess.run([sys.executable, GEN, "--check", PROFILE], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_list_profiles():
    r = subprocess.run([sys.executable, GEN, "--list"], capture_output=True, text=True, check=True)
    assert any(line.startswith(f"{PROFILE}|") for line in r.stdout.splitlines())


def test_side_transform_symmetric():
    """Links ist das Spiegelbild von rechts (an der Sagittalebene, y -> -y)."""
    pytest.importorskip("yaml")
    sys.path.insert(0, TOOLS)
    import gen_hand_profile as gen
    p = gen.load_profile(PROFILE)
    Al, bl = gen.side_transform(p, "left")
    Ar, br = gen.side_transform(p, "right")
    S = np.diag([1.0, -1.0, 1.0])
    assert np.allclose(Al, S @ Ar, atol=1e-9)
    assert np.allclose(bl, S @ br, atol=1e-9)
    # Montagepunkt liegt auf der Flansch-Mitte
    assert np.allclose(Al @ np.asarray(p["origin"]) + bl, [gen.FLANGE_X, 0, 0], atol=1e-6)


# ── MuJoCo ──────────────────────────────────────────────────────────────
@pytest.fixture(scope="module")
def mj():
    mujoco = pytest.importorskip("mujoco")
    m = mujoco.MjModel.from_xml_path(os.path.join(G1_DIR, f"scene_hand_{PROFILE}.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return mujoco, m, d


def test_mujoco_model(mj):
    mujoco, m, d = mj
    base = mujoco.MjModel.from_xml_path(os.path.join(G1_DIR, "scene.xml"))
    assert m.nu == base.nu == 29                       # keine Finger-Aktoren
    for side in ("left", "right"):
        b = m.body(f"{side}_hand_tool_link")
        assert m.body(b.parentid[0]).name == f"{side}_wrist_yaw_link"
        cols = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b.id and m.geom_contype[g]]
        assert len(cols) >= 2
        for g in cols:                                     # nur Bit 2: Objekte, nicht Koerper/Boden
            assert m.geom_contype[g] == 2 and m.geom_conaffinity[g] == 2
    # Rubber-Hand weg, Haken-Masse dazu
    assert not any("rubber" in m.mesh(i).name for i in range(m.nmesh))
    sys.path.insert(0, TOOLS)
    import json
    info = json.load(open(os.path.join(G1_DIR, "hands", PROFILE, "generated", "info.json")))
    assert np.isclose(sum(m.body_mass), sum(base.body_mass) + 2 * info["mass_kg"], atol=1e-6)


def test_mujoco_hook_settles_without_self_contact(mj):
    """Die Haken beruehren im Stand weder Koerper noch einander."""
    mujoco, m, d = mj
    tool = {m.body(f"{s}_hand_tool_link").id for s in ("left", "right")}
    for i in range(d.ncon):
        c = d.contact[i]
        assert m.geom_bodyid[c.geom1] not in tool and m.geom_bodyid[c.geom2] not in tool


# ── URDF / IK / Planer ──────────────────────────────────────────────────
@pytest.fixture(scope="module")
def ik():
    pytest.importorskip("pinocchio")
    from g1pilot.utils.ik_solver import G1IKSolver
    urdf = os.path.join(URDF_DIR, f"g1_29dof_hand_{PROFILE}.urdf")
    return G1IKSolver(urdf_path=urdf, mesh_dir=REPO, debug=False)


def test_urdf_tcp_matches_mujoco(ik, mj):
    """TCP (*_hand_point_contact) im URDF = Site *_hand_tcp im MuJoCo-Modell,
    jeweils relativ zum wrist_yaw_link."""
    import pinocchio as pin
    mujoco, m, d = mj
    q = pin.neutral(ik.model)
    pin.forwardKinematics(ik.model, ik.data, q)
    pin.updateFramePlacements(ik.model, ik.data)
    for side in ("left", "right"):
        wf = ik.model.getFrameId(f"{side}_wrist_yaw_link")
        tf = ik.model.getFrameId(f"{side}_hand_point_contact")
        t_urdf = (ik.data.oMf[wf].inverse() * ik.data.oMf[tf]).translation
        wb = m.body(f"{side}_wrist_yaw_link").id
        s = m.site(f"{side}_hand_tcp").id
        R = d.xmat[wb].reshape(3, 3)
        t_mj = R.T @ (d.site_xpos[s] - d.xpos[wb])
        assert np.allclose(t_urdf, t_mj, atol=1e-5), (side, t_urdf, t_mj)
    # Hakenspitze weiter vorn als die alte Rubber/Inspire-TCP (0.0915 m)
    assert t_urdf[0] > 0.12


def test_urdf_has_no_inspire_fingers(ik):
    names = list(ik.model.names)
    assert not any("thumb" in n or "index_" in n for n in names)
    assert "left_hand_tool_joint" not in names                 # fixed -> Frame, kein Gelenk
    assert ik.model.existFrame("left_hand_tool_link")
    assert ik.model.existFrame("right_hand_flange")


def test_collision_gate_covers_hook(ik):
    cm = ik.collision_model
    pairs = {(cm.geometryObjects[p.first].name, cm.geometryObjects[p.second].name)
             for p in cm.collisionPairs}
    hook = [a for a, b in pairs if "hand_tool" in a] + [b for a, b in pairs if "hand_tool" in b]
    assert hook, "Haken-Geometrie fehlt im Selbstkollisions-Gate"
    # Grundstellung ist kollisionsfrei
    assert not ik.arm_command_in_collision(np.zeros(29), hard=True)


def test_planner_spheres_cover_hook(ik):
    from g1pilot.manipulation.arm_geometry import ArmGeometry, HAND_LINK_MIN
    geom = ArmGeometry(ik.model, ik.collision_model)
    for side in ("left", "right"):
        sp = geom.spheres[side]
        hand = sp["link"] >= HAND_LINK_MIN
        reach = np.max(sp["c"][hand, 0] + sp["r"][hand])      # im wrist_yaw-Frame
        assert reach > 0.18, reach                             # Haken reicht bis ~0.20 m
        assert np.isclose(sp["tcp"][0], geom.spheres[side]["tcp"][0])


def test_planner_spheres_cover_hook_surfaces(ik):
    """Die V-HACD-Teile des Hakens haben wenige, weit auseinanderliegende
    Ecken; die Kugeln muessen trotzdem die FLAECHEN umschliessen (<= 2 mm
    Ueberstand), sonst fuehrt der Planer den Haken enger an Objekte als
    clearance/floor vorgeben."""
    sys.path.insert(0, os.path.dirname(__file__))
    from sphere_coverage import hand_surface_gap
    from g1pilot.manipulation.arm_geometry import ArmGeometry, HAND_LINK_MIN
    geom = ArmGeometry(ik.model, ik.collision_model)
    for side in ("left", "right"):
        gaps = hand_surface_gap(ik.model, ik.collision_model, geom, side, HAND_LINK_MIN)
        assert any("hand_tool" in n for n in gaps)
        worst = max(gaps, key=gaps.get)
        assert gaps[worst] <= 0.002, (side, worst, gaps[worst])


def test_com_model_loads():
    pytest.importorskip("pinocchio")
    from g1pilot.navigation.com_model import ComModel
    cm = ComModel(os.path.join(G1_DIR, f"g1_29dof_hand_{PROFILE}.xml"))
    base = ComModel(os.path.join(G1_DIR, "g1_29dof.xml"))
    assert cm.mass > base.mass


def test_mujoco_hook_carries_object(mj):
    """Physik-Probe: ein kleiner Wuerfel faellt auf die Lippe des rechten Hakens
    und bleibt liegen (Kollisions-Zerlegung traegt, Haken ist kein leerer Klotz)."""
    mujoco, m0, d0 = mj
    spec = mujoco.MjSpec.from_file(os.path.join(G1_DIR, f"scene_hand_{PROFILE}.xml"))
    tcp = d0.site_xpos[m0.site("right_hand_tcp").id].copy()
    R = d0.xmat[m0.body("right_wrist_yaw_link").id].reshape(3, 3)
    for b in spec.bodies:
        b.gravcomp = 1                                        # Roboter schwebt
    for j in spec.joints:
        j.damping = [50, 0, 0]
    for j in list(spec.body("pelvis").joints):               # Becken fest
        spec.delete(j)
    box = spec.worldbody.add_body(name="testbox",
                                  pos=list(tcp + [0, 0, 0.03] + R[:, 1] * 0.01))
    box.add_freejoint()
    box.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.008] * 3, mass=0.05,
                 contype=3, conaffinity=3)
    m = spec.compile()
    d = mujoco.MjData(m)
    for _ in range(int(1.5 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    z = d.xpos[m.body("testbox").id][2]
    assert z > tcp[2] - 0.01, f"Wuerfel durchgefallen (z={z:.3f}, Lippe {tcp[2]:.3f})"


@pytest.mark.parametrize("mm,deg", [([0, 0, 0], [0, 0, 0]), ([5, -3, 2], [0, 0, 0]),
                                    ([0, 0, 0], [10, -20, 30]), ([12.5, 4, -7], [-15, 5, 90])])
def test_adjust_live_equals_saved(mm, deg):
    """Justier-Tool: die Live-Verschiebung im Viewer ergibt nach "Speichern"
    (compose -> hand.yaml -> Generator) genau dieselbe Lage, beide Seiten."""
    pytest.importorskip("yaml")
    sys.path.insert(0, TOOLS)
    import gen_hand_profile as gen
    import hand_adjust as adj
    p = gen.load_profile(PROFILE)
    off, rot = adj.compose(p["offset_mm"], p["rotate_deg"], mm, deg)
    p2 = dict(p, offset_mm=off, rotate_deg=rot)
    for side in ("left", "right"):
        A, b = gen.side_transform(p, side)
        pos, R = adj.delta_pose(side, mm, deg)
        A2, b2 = gen.side_transform(p2, side)
        assert np.allclose(R @ A, A2, atol=1e-5), side
        assert np.allclose(R @ b + pos, b2, atol=2e-5), side


def test_adjust_write_mount_keeps_comments(tmp_path):
    sys.path.insert(0, TOOLS)
    import hand_adjust as adj
    f = tmp_path / "hand.yaml"
    f.write_text("mount:\n  # Kommentar\n  offset_mm: [0, 0, 0]   # mm\n  rotate_deg: [0.0, 0.0, 0.0]\n")
    adj.write_mount(f, [1.5, -2, 0], [0, 90, 0])
    t = f.read_text()
    assert "offset_mm: [1.5, -2, 0]   # mm" in t and "rotate_deg: [0, 90, 0]" in t
    assert "# Kommentar" in t

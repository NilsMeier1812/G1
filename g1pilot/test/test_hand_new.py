#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Neue Hand anlegen: automatische Montage-Erkennung (hand_detect.py) und das
Anlegen eines Profils aus einer 3D-Datei (hand_new.py).

Prueft, dass die Erkennung bei den gedruckten Box-Haken die von Hand
eingepasste Montage findet, bei einem kuenstlichen Werkzeug (auch schraeg im
Raum) Steckplatte, Richtung und Handflaeche trifft, und dass das angelegte
Profil vom Generator gelesen wird. Schreibt nichts ins Repo (tmp-Ordner).
"""
import os
import sys

import numpy as np
import pytest

trimesh = pytest.importorskip("trimesh")
pytest.importorskip("yaml")

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REPO = os.path.dirname(PKG)
TOOLS = os.path.join(REPO, "unitree_mujoco", "unitree_robots", "g1", "tools")
HAKEN_STL = os.path.join(TOOLS, "..", "hands", "haken", "G1_Box_Hand_1.stl")
sys.path.insert(0, TOOLS)

import gen_hand_profile as gen  # noqa: E402
import hand_detect as det  # noqa: E402
import hand_new  # noqa: E402


def _close(a, b, tol):
    return float(np.linalg.norm(np.asarray(a, float) - np.asarray(b, float))) <= tol


def _tool_mesh():
    """Kuenstliches Werkzeug (mm): Steckplatte 20x20x12 am Arm-Ende, Koerper
    40 lang x 30 breit x 10 dick entlang +x, Kralle an der Spitze nach +z."""
    plate = trimesh.creation.box((12, 20, 20))
    plate.apply_translation((6, 0, 0))                      # x 0..12
    body = trimesh.creation.box((100, 30, 10))
    body.apply_translation((12 + 50, 0, 0))                 # x 12..112
    claw = trimesh.creation.box((10, 30, 25))
    claw.apply_translation((107, 0, 5 + 12.5))              # an der Spitze, nach +z
    return trimesh.util.concatenate([plate, body, claw])


# ── Erkennung ────────────────────────────────────────────────────────────
def test_haken_mount_found():
    mesh = det.load_mesh(HAKEN_STL)
    assert det.guess_unit(mesh) == "mm"
    sug = det.suggest_mounts(mesh, 4, "mm")
    assert len(sug) == 4
    # wahrscheinlichster: aus dem Arm entlang -y, Steckplatte (y 150..162.5) im Arm
    top = sug[0]
    assert _close(top["out"], [0, -1, 0], 1e-6)
    assert _close(top["origin"], [25.0, 150.0, 14.0], 2.5), top["origin"]
    # die von Hand eingepasste Montage (palm +z) ist unter den Vorschlaegen
    assert any(_close(s["out"], [0, -1, 0], 1e-6) and _close(s["palm"], [0, 0, 1], 1e-6)
               for s in sug[:2])


def test_synthetic_tool_axis_aligned():
    sug = det.suggest_mounts(_tool_mesh(), 4)
    top = sug[0]
    assert _close(top["out"], [1, 0, 0], 1e-6)
    assert _close(top["palm"], [0, 0, 1], 1e-6)              # Kralle = Handflaeche
    assert _close(top["origin"], [12, 0, 0], 3.5), top["origin"]   # Absatz der Platte
    tcp = np.asarray(top["tcp"])
    assert 80 < tcp[0] < 110 and tcp[2] > 0                 # vorn, auf der Kralle-Seite


@pytest.mark.parametrize("seed", [1, 7])
def test_synthetic_tool_rotated(seed):
    """Schraeg exportiert: Erkennung arbeitet in den Hauptachsen des Teils."""
    rng = np.random.default_rng(seed)
    R = trimesh.transformations.random_rotation_matrix(rng.random(3))
    T = R.copy()
    T[:3, 3] = (30, -40, 15)
    mesh = _tool_mesh()
    mesh.apply_transform(T)
    top = det.suggest_mounts(mesh, 4)[0]
    assert _close(top["out"], R[:3, :3] @ [1, 0, 0], 0.03)
    assert _close(top["palm"], R[:3, :3] @ [0, 0, 1], 0.03)
    assert _close(top["origin"], R[:3, :3] @ [12, 0, 0] + T[:3, 3], 4.0)


def test_haken_rotated():
    """Die Haken schraeg exportiert: gleiche Montage, nur mitgedreht."""
    R = trimesh.transformations.rotation_matrix(0.6, (1, 2, 0.5))
    mesh = det.load_mesh(HAKEN_STL)
    mesh.apply_transform(R)
    Rr = R[:3, :3]
    sug = det.suggest_mounts(mesh, 4, "mm")
    hit = [s for s in sug[:2] if _close(s["out"], Rr @ [0, -1, 0], 0.02)
           and _close(s["palm"], Rr @ [0, 0, 1], 0.02)]
    assert hit, sug
    assert _close(hit[0]["origin"], Rr @ [25.0, 150.0, 14.0], 3.0), hit[0]["origin"]


def test_unit_guess_meters():
    mesh = _tool_mesh()
    mesh.apply_scale(0.001)
    assert det.guess_unit(mesh) == "m"


def test_suggestions_are_orthonormal():
    for s in det.suggest_mounts(det.load_mesh(HAKEN_STL), 6):
        o, p = np.asarray(s["out"]), np.asarray(s["palm"])
        assert abs(np.linalg.norm(o) - 1) < 1e-6 and abs(np.linalg.norm(p) - 1) < 1e-6
        assert abs(float(o @ p)) < 1e-6


# ── Anlegen ──────────────────────────────────────────────────────────────
@pytest.fixture
def hands_tmp(tmp_path, monkeypatch):
    monkeypatch.setattr(gen, "HANDS_DIR", tmp_path)
    return tmp_path


def test_slugify():
    assert hand_new.slugify("Große Kralle (v2)") == "grosse_kralle_v2"
    assert hand_new.slugify("  ") == "hand"


def test_create_profile_from_stl(hands_tmp):
    res = hand_new.analyse(HAKEN_STL, None, render=False)
    s = res["suggestions"][1]
    slug = hand_new.create_profile(HAKEN_STL, "Box-Haken Kopie", s, unit=res["unit"],
                                   pick=2, unit_guess=res["unit_guess"])
    assert slug == "box_haken_kopie"
    p = gen.load_profile(slug)
    assert p["title"] == "Box-Haken Kopie"
    assert p["scale"] == 0.001
    assert _close(p["origin"], s["origin"], 0.01)
    assert _close(p["out"], s["out"], 1e-4) and _close(p["palm"], s["palm"], 1e-4)
    assert _close(p["tcp"], s["tcp"], 0.01)
    assert p["mirror"] == {"left": True, "right": False}
    assert 0.1 < p["mass"] < 0.2                     # 238 cm3 PLA, 50 % gefuellt
    assert p["offset_mm"] == [0, 0, 0] and p["rotate_deg"] == [0, 0, 0]
    # zweites Mal gleicher Name -> eigener Ordner
    assert hand_new.free_slug(slug) == f"{slug}_2"
    with pytest.raises(ValueError):
        hand_new.create_profile(HAKEN_STL, "x", s, unit="mm", slug=slug)


def test_create_profile_from_obj(hands_tmp, tmp_path_factory):
    src = tmp_path_factory.mktemp("src") / "kralle.obj"
    _tool_mesh().export(str(src))
    res = hand_new.analyse(str(src), None, render=False)
    slug = hand_new.create_profile(src, "Kralle", res["suggestions"][0], unit=res["unit"],
                                   mass=0.2)
    p = gen.load_profile(slug)
    assert p["mesh"].suffix == ".stl" and p["mesh"].is_file()
    assert p["mass"] == 0.2
    # Datei als STL umgeschrieben, Geometrie gleich
    assert np.allclose(trimesh.load(str(p["mesh"])).extents, _tool_mesh().extents, atol=1e-6)


def test_create_profile_bad_file_leaves_nothing(hands_tmp, tmp_path_factory):
    bad = tmp_path_factory.mktemp("src") / "leer.stl"
    bad.write_bytes(b"solid x\nendsolid x\n")
    with pytest.raises(Exception):
        hand_new.create_profile(bad, "Leer", {"origin": [0, 0, 0], "out": [1, 0, 0],
                                              "palm": [0, 1, 0], "tcp": [0, 0, 0]}, unit="mm")
    assert not any(hands_tmp.iterdir())


def test_preview_model_places_hand_like_generator():
    """Vorschau (im Speicher) setzt die Hand dorthin, wo der Generator sie hinlegt."""
    pytest.importorskip("mujoco")
    mesh = det.load_mesh(HAKEN_STL)
    s = det.suggest_mounts(mesh, 2, "mm")[1]
    m, d = hand_new.preview_model(mesh, s, "mm", True)
    p = hand_new._profile_dict(s, "mm", True)
    for side in gen.SIDES:
        A, b = gen.side_transform(p, side)
        want = gen.transform_mesh(mesh, A, b)
        gid = m.geom(f"{side}_new_hand").id
        body = m.geom_bodyid[gid]
        assert m.body(body).name == f"{side}_wrist_yaw_link"
        # Mesh-Punkte im Koerper-Frame (MuJoCo zentriert/dreht Meshes intern)
        mid = m.geom_dataid[gid]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        q = m.geom_quat[gid]
        Rg = np.zeros(9)
        import mujoco
        mujoco.mju_quat2Mat(Rg, q)
        pts = v @ Rg.reshape(3, 3).T + m.geom_pos[gid]
        assert np.allclose(pts.min(axis=0), want.bounds[0], atol=1e-4)
        assert np.allclose(pts.max(axis=0), want.bounds[1], atol=1e-4)

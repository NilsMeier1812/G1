#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Greif-Objekte aus Netzen (KLT-Kisten) kollidieren ueber Quader statt ueber
die konvexe Huelle (scene_editor/build_env_scene.py, add_grasp_collision).

Prueft an der echten Szene A5 (Haken-Hand):
  * die kleine KLT hat viele Kollisionsteile im selben freien Koerper, das
    Netz ist reine Optik, Kollisionsbits wie bei allen Umgebungs-Objekten,
  * Masse und Traegheit sind bitgleich zur Szene ohne Zerlegung,
  * scene_objects.py meldet die Objekte unveraendert (RViz/Planer/Nav),
  * Innenraum und Grifftasche sind frei, die 1-mm-Stirnwand ist zu,
  * alle Objekte liegen nach 2 s simulierter Zeit ruhig,
  * zwei Haken (Kollisionsteile der Haken-Hand) greifen unter die Schuerze und
    heben die Kiste -- mit der alten Huellen-Kollision geht das nicht (die
    frueheren Behelfs-Rippen der kleinen KLT sind darum entfallen),
  * Bauzeit der Szene mit leerem und mit gefuelltem Cache (wird ausgegeben).

Laeuft ohne ROS: braucht mujoco, numpy, trimesh (wie scene_editor/.venv).
    python3 -m pytest -q -s test/test_grasp_collision.py
"""
import glob
import importlib.util
import sys
import time
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("trimesh")

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parent
MJ_ROOT = REPO / "unitree_mujoco"
G1_DIR = MJ_ROOT / "unitree_robots" / "g1"
EDITOR = MJ_ROOT / "scene_editor"
SCENE_A5 = EDITOR / "scenes" / "scene_A5.xml"
ROBOT = "g1_29dof_hand_haken.xml"
KLT = "grasp_asset_KLT_300x200_m_stl_0010"        # kleine KLT 300x200 (blau)
KLT_BIG = "grasp_asset_KLT_400x300_m_stl_0009"    # KLT 400x300 (schwarz)
SIM_DT = 0.001                                    # wie config.SIMULATE_DT


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def bes():
    return _load("build_env_scene_t", EDITOR / "build_env_scene.py")


@pytest.fixture(scope="module")
def so():
    sys.path.insert(0, str(MJ_ROOT / "simulate_python"))
    return _load("scene_objects_t", MJ_ROOT / "simulate_python" / "scene_objects.py")


def _build(bes, tag, cache=None, decompose=True):
    """A5 in eine eigene (gitignorierte) Datei neben dem Robotermodell bauen
    -- das <include> des Roboters ist relativ."""
    out = G1_DIR / f"scene_env_pytest_{tag}.xml"
    old_cache, old_fn = bes.COLLISION_CACHE, bes._grasp_boxes_cached
    try:
        if cache is not None:
            bes.COLLISION_CACHE = Path(cache)
        if not decompose:
            bes._grasp_boxes_cached = lambda mesh_abs: []
        bes.build_scene(SCENE_A5, ROBOT, out)
    finally:
        bes.COLLISION_CACHE, bes._grasp_boxes_cached = old_cache, old_fn
    return out


@pytest.fixture(scope="module")
def built(bes, tmp_path_factory):
    """Szene mit leerem Cache (kalt), nochmal mit gefuelltem Cache (warm) und
    ohne Zerlegung (Vergleich) bauen; Zeiten ausgeben."""
    cache = tmp_path_factory.mktemp("collision_cache")
    grasp_t = []
    orig = bes._grasp_boxes_cached

    def timed(mesh_abs):
        t = time.perf_counter()
        r = orig(mesh_abs)
        grasp_t.append(time.perf_counter() - t)
        return r

    bes._grasp_boxes_cached = timed
    try:
        t0 = time.perf_counter()
        _build(bes, "kalt", cache)
        t_cold, g_cold = time.perf_counter() - t0, sum(grasp_t)
        grasp_t.clear()
        t0 = time.perf_counter()
        path = _build(bes, "a5", cache)
        t_warm, g_warm = time.perf_counter() - t0, sum(grasp_t)
    finally:
        bes._grasp_boxes_cached = orig
    plain = _build(bes, "ohne", cache, decompose=False)
    print(f"\n[Bauzeit A5] ohne Cache {t_cold:.1f} s (davon Greif-Zerlegung {g_cold:.1f} s), "
          f"mit Cache {t_warm:.2f} s (Greif-Zerlegung {g_warm:.3f} s)")
    yield {"a5": path, "ohne": plain, "t_cold": t_cold, "t_warm": t_warm,
           "g_cold": g_cold, "g_warm": g_warm}
    for f in G1_DIR.glob("scene_env_pytest_*.xml"):
        f.unlink()


def _model(path):
    m = mujoco.MjModel.from_xml_path(str(path))
    m.opt.timestep = SIM_DT
    return m


def _hull_geoms(m, body):
    bid = m.body(body).id
    return [g for g in range(m.ngeom) if m.geom_bodyid[g] == bid
            and "__hull" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")]


# ── Aufbau ──────────────────────────────────────────────────────────────
def test_klt_has_many_collision_parts(built):
    m = _model(built["a5"])
    for body in (KLT, KLT_BIG):
        parts = _hull_geoms(m, body)
        assert len(parts) > 1, body
        bid = m.body(body).id
        assert m.jnt_type[m.body_jntadr[bid]] == mujoco.mjtJoint.mjJNT_FREE
        # Netz = reine Optik, Teile kollidieren wie alle Umgebungs-Objekte (3)
        vis = m.geom(body).id
        assert m.geom_contype[vis] == 0 and m.geom_conaffinity[vis] == 0
        assert all(m.geom_contype[g] == 3 and m.geom_conaffinity[g] == 3 for g in parts)
        assert all(m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX for g in parts)
    print(f"\n[A5] Kollisionsteile kleine KLT {len(_hull_geoms(m, KLT))}, "
          f"grosse KLT {len(_hull_geoms(m, KLT_BIG))}")


def test_mass_and_inertia_unchanged(built):
    a, b = _model(built["a5"]), _model(built["ohne"])
    for body in (KLT, KLT_BIG, "grasp_asset_sample_ramp_stl_0007"):
        i, j = a.body(body).id, b.body(body).id
        assert a.body_mass[i] == pytest.approx(b.body_mass[j], abs=1e-12)
        np.testing.assert_allclose(a.body_inertia[i], b.body_inertia[j], atol=1e-12)
        np.testing.assert_allclose(a.body_ipos[i], b.body_ipos[j], atol=1e-12)
    assert a.body_mass[a.body(KLT).id] == pytest.approx(0.63)     # Masse aus der Szene
    assert a.body_mass[a.body(KLT_BIG).id] == pytest.approx(1.0)


def test_convex_grasp_mesh_untouched(built):
    """Die Rampe ist konvex -> bleibt ein Netz-Geom, wird nicht zerlegt."""
    m = _model(built["a5"])
    ramp = "grasp_asset_sample_ramp_stl_0007"
    assert not _hull_geoms(m, ramp)
    assert m.geom_contype[m.geom(ramp).id] == 3


def test_scene_objects_report_unchanged(built, so):
    """RViz/Planer/Nav sehen die Kisten wie bisher als EIN Greif-Objekt mit der
    Bounding-Box des Netzes (Schnittstelle unveraendert)."""
    new = so.parse_scene_objects(str(built["a5"]))
    old = so.parse_scene_objects(str(built["ohne"]))
    assert new == old
    names = {s["name"]: s for s in new}
    for body in (KLT, KLT_BIG):
        assert names[body]["class"] == "grasp"
        assert names[body]["aabb_half"] is not None
    assert not any("__hull" in s["name"] for s in new)


def test_interior_and_handle_open(built):
    """Punkte im Innenraum und in der Grifftasche liegen in keinem Teil, die
    1 mm duenne Stirnwand ist ueberall zu (Hand kann nicht durch die Wand)."""
    m = _model(built["a5"])
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    bid = m.body(KLT).id
    boxes = [(d.geom_xpos[g] - d.xpos[bid], m.geom_size[g]) for g in _hull_geoms(m, KLT)]

    def inside(p):
        return any(np.all(np.abs(np.asarray(p) - c) <= s + 1e-9) for c, s in boxes)

    # Mesh-Koordinaten (Ursprung Mitte Unterkante, Koerper ungedreht):
    # Stirnwand y = +-0.1296..0.1307, Schuerze y = +-0.1477, Tasche dazwischen
    # 17 mm breit (wie die echte KLT 3115), von z ~0.083 (Unterkante Schuerze)
    # bis 0.134 (Rand). Gitterebenen liegen auf den Wandflaechen: die Tasche
    # ist auf 0,2 mm genau frei.
    for p in ([0, 0, 0.07], [0.05, 0.08, 0.12], [-0.06, -0.1, 0.03]):
        assert not inside(p), f"Innenraum zu bei {p}"
    for sgn in (1, -1):
        for x in (-0.03, 0.0, 0.03):
            for z in (0.09, 0.11, 0.125):
                for y in (0.1309, 0.139, 0.1475):
                    assert not inside([x, sgn * y, z]), f"Grifftasche zu bei {x, sgn * y, z}"
        for x in np.linspace(-0.08, 0.08, 9):
            for z in np.linspace(0.01, 0.13, 13):
                assert inside([x, sgn * 0.1302, z]), f"Loch in der Stirnwand bei {x:.2f}, {z:.2f}"


# ── Physik ──────────────────────────────────────────────────────────────
def _free_bodies(m):
    out = []
    for b in range(m.nbody):
        if m.body_jntnum[b] == 1 and m.jnt_type[m.body_jntadr[b]] == mujoco.mjtJoint.mjJNT_FREE:
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
            if name != "pelvis":
                out.append(b)
    return out


def test_objects_rest_after_2s(built):
    """Nach dem Laden 2 s simulieren: keine Spruenge, kein Durchrutschen."""
    m = _model(built["a5"])
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    bodies = _free_bodies(m)
    assert len(bodies) >= 6
    p0 = {b: d.xpos[b].copy() for b in bodies}
    q0 = {b: d.xquat[b].copy() for b in bodies}
    max_jump = {b: 0.0 for b in bodies}
    t = time.perf_counter()
    n = int(2.0 / SIM_DT)
    for _ in range(n):
        mujoco.mj_step(m, d)
        for b in bodies:
            max_jump[b] = max(max_jump[b], float(np.linalg.norm(d.xpos[b] - p0[b])))
    ms = (time.perf_counter() - t) / n * 1e3
    print(f"\n[Ruhe 2 s] {ms:.3f} ms je Sim-Schritt (dt {SIM_DT * 1e3:.0f} ms, ganze Szene)")
    for b in bodies:
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        dof = m.jnt_dofadr[m.body_jntadr[b]]
        v = float(np.linalg.norm(d.qvel[dof:dof + 3]))
        dz = float(d.xpos[b][2] - p0[b][2])
        tilt = np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(d.xquat[b], q0[b]))))))
        print(f"  {name:40s} max {max_jump[b] * 1e3:5.2f} mm  dz {dz * 1e3:+6.2f} mm  "
              f"kipp {tilt:4.2f} Grad  v {v:.1e} m/s")
        if "ramp" in name:
            continue      # 240-kg-Rampe steht schraeg auf dem Boden (unveraendert, ~12 mm)
        assert max_jump[b] < 0.003, name
        assert dz > -0.003, name
        assert tilt < 1.0, name
        assert v < 1e-3, name


def _hook_probe(path, klt, half_y, wall_y, lift=0.08):
    """Zwei Haken (Kollisionsteile der Haken-Hand, Mocap) fahren unter die
    Schuerze an beiden Stirnseiten und heben 8 cm. Rueckgabe: (Hub der Kiste,
    Kippung, tiefste Durchdringung Haken/Kiste)."""
    spec = mujoco.MjSpec.from_file(str(path))
    m0 = spec.compile()
    d0 = mujoco.MjData(m0)
    mujoco.mj_forward(m0, d0)
    P = d0.xpos[m0.body(klt).id].copy()
    hooks = []
    for side, sgn in (("right", 1), ("left", -1)):
        # Haken-Frame 180 Grad um z gedreht: Finger -x, Handflaeche zur Kiste,
        # Lippe (hand z -0.05..-0.035) unten
        b = spec.worldbody.add_body(name=f"probe_{side}", mocap=True, quat=[0, 0, 0, 1])
        for f in sorted(glob.glob(str(G1_DIR / "hands" / "haken" / "generated" / f"{side}_col_*.stl"))):
            mn = f"probe_{side}_{Path(f).stem}"
            spec.add_mesh(name=mn, file=f)
            b.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mn, contype=2, conaffinity=2,
                       friction=[1.0, 0.01, 0.001])
        hooks.append((side, sgn))
    m = spec.compile()
    m.opt.timestep = 0.002
    d = mujoco.MjData(m)
    kb = m.body(klt).id
    mids = [m.body(f"probe_{s}").mocapid[0] for s, _ in hooks]
    hook_ids = {m.body(f"probe_{s}").id for s, _ in hooks}
    z_lip = 0.082 - 0.010            # Lippen-Oberkante 1 cm unter der Schuerze

    def place(tip_y, zt):
        for (_, sgn), mid in zip(hooks, mids):
            # Lippe: hand x 0.1316..0.1914, Spitze |y| 0.034, Oberkante z -0.035
            d.mocap_pos[mid] = [P[0] + 0.1615, P[1] + sgn * (tip_y + 0.034), P[2] + zt + 0.035]

    def run(T, f):
        n = int(T / m.opt.timestep)
        for i in range(n):
            f(i / n)
            mujoco.mj_step(m, d)

    out_y, in_y = half_y + 0.03, wall_y + 0.002
    place(out_y, z_lip)
    run(0.5, lambda a: None)
    z0 = d.xpos[kb][2]
    run(1.0, lambda a: place(out_y + (in_y - out_y) * a, z_lip))     # hineinfahren
    run(0.3, lambda a: place(in_y, z_lip))
    run(1.5, lambda a: place(in_y, z_lip + lift * a))               # anheben
    run(1.0, lambda a: place(in_y, z_lip + lift))                   # halten
    pen = [d.contact[k].dist for k in range(d.ncon)
           if {m.geom_bodyid[d.contact[k].geom1], m.geom_bodyid[d.contact[k].geom2]} & hook_ids
           and kb in {m.geom_bodyid[d.contact[k].geom1], m.geom_bodyid[d.contact[k].geom2]}]
    R = d.xmat[kb].reshape(3, 3)
    tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
    return float(d.xpos[kb][2] - z0), tilt, (-min(pen) if pen else 0.0)


@pytest.mark.parametrize("klt,half_y,wall_y", [(KLT, 0.15, 0.1307), (KLT_BIG, 0.20, 0.1781)])
def test_hook_lifts_klt_by_handle(built, klt, half_y, wall_y):
    lift, tilt, pen = _hook_probe(built["a5"], klt, half_y, wall_y)
    print(f"\n[Haken {klt}] Hub {lift * 100:.1f} cm, Kippung {tilt:.1f} Grad, "
          f"Durchdringung {pen * 1e3:.1f} mm")
    assert lift > 0.06            # Haken 8 cm, Lippe startet 1 cm unter der Schuerze
    assert tilt < 5.0
    assert pen < 0.003            # sauber eingehakt, nicht in die Kiste gedrueckt


def _hook_probe_rotated(path, klt, wall_y, lip_top, squeeze=0.002, lift=0.06):
    """Haken so, wie Nils ihn benutzt: Hand 90 Grad nach innen gedreht, Paddel
    waagrecht unter der Schuerze, die 15-mm-Lippe steht nach oben. Ablauf:
    Lippe nahe der Stirnwand von unten in die Tasche, beide Haken nach innen
    zusammendruecken, dann heben. Rueckgabe: (Hub beim Einfahren, Hub der
    Kiste, Kippung, tiefste Durchdringung)."""
    spec = mujoco.MjSpec.from_file(str(path))
    m0 = spec.compile()
    d0 = mujoco.MjData(m0)
    mujoco.mj_forward(m0, d0)
    P = d0.xpos[m0.body(klt).id].copy()

    def quat(cols):
        R = np.array(cols, float).T
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, R.ravel())
        return q

    # Spalten = Hand-Achsen in Weltkoordinaten: Finger -x, Lippen-Hoehe (hand y) nach oben
    cfg = {"right": (1, quat([(-1, 0, 0), (0, 0, 1), (0, 1, 0)])),
           "left": (-1, quat([(-1, 0, 0), (0, 0, -1), (0, -1, 0)]))}
    for side, (_, q) in cfg.items():
        b = spec.worldbody.add_body(name=f"rot_{side}", mocap=True, quat=list(q))
        for f in sorted(glob.glob(str(G1_DIR / "hands" / "haken" / "generated" / f"{side}_col_*.stl"))):
            mn = f"rot_{side}_{Path(f).stem}"
            spec.add_mesh(name=mn, file=f)
            b.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mn, contype=2, conaffinity=2,
                       friction=[1.0, 0.01, 0.001])
    m = spec.compile()
    m.opt.timestep = 0.002
    d = mujoco.MjData(m)
    kb = m.body(klt).id
    mids = {s: m.body(f"rot_{s}").mocapid[0] for s in cfg}
    hook_ids = {m.body(f"rot_{s}").id for s in cfg}
    lip_y = wall_y + 0.0075 + 0.0005        # Lippe (15 mm) 0,5 mm vor der Stirnwand

    def place(dy_in, top):
        for s, (sgn, _) in cfg.items():
            # Lippenmitte = hand z -0.0425, Lippen-Oberkante = |hand y| 0.034
            d.mocap_pos[mids[s]] = [P[0] + 0.1615, P[1] + sgn * (lip_y - dy_in + 0.0425),
                                    P[2] + top - 0.034]

    def run(T, f):
        n = int(T / m.opt.timestep)
        for i in range(n):
            f(i / n)
            mujoco.mj_step(m, d)

    z_start = 0.060
    place(0, z_start)
    run(0.5, lambda a: None)
    z0 = d.xpos[kb][2]
    run(1.0, lambda a: place(0, z_start + (lip_top - z_start) * a))   # von unten einfahren
    z_in = d.xpos[kb][2] - z0
    run(0.5, lambda a: place(squeeze * a, lip_top))                   # zusammendruecken
    run(1.5, lambda a: place(squeeze, lip_top + lift * a))            # heben
    run(1.0, lambda a: None)
    pen = [d.contact[k].dist for k in range(d.ncon)
           if {m.geom_bodyid[d.contact[k].geom1], m.geom_bodyid[d.contact[k].geom2]} & hook_ids
           and kb in {m.geom_bodyid[d.contact[k].geom1], m.geom_bodyid[d.contact[k].geom2]}]
    R = d.xmat[kb].reshape(3, 3)
    tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
    return float(z_in), float(d.xpos[kb][2] - z0), tilt, (-min(pen) if pen else 0.0)


@pytest.mark.parametrize("klt,wall_y", [(KLT, 0.1307), (KLT_BIG, 0.1781)])
def test_rotated_hook_enters_pocket_and_lifts(built, klt, wall_y):
    """Gedrehter Haken: die Lippe faehrt frei in die Tasche (Kiste bewegt sich
    beim Einfahren nicht), Paddel liegt unter der Schuerze, Kiste hebt gerade."""
    # Lippe 24 mm ueber dem Paddel; Paddel stoppt knapp unter der Schuerze
    # (Unterkante ~80 mm) -> Lippen-Oberkante 100 mm ueber dem Kistenboden
    z_in, lift, tilt, pen = _hook_probe_rotated(built["a5"], klt, wall_y, lip_top=0.100)
    print(f"\n[Haken gedreht {klt}] Einfahren {z_in * 1e3:.1f} mm, Hub {lift * 100:.1f} cm, "
          f"Kippung {tilt:.1f} Grad, Durchdringung {pen * 1e3:.1f} mm")
    assert abs(z_in) < 0.003          # nicht geklemmt, Kiste nicht mitgeschoben
    assert lift > 0.05                # Haken heben 6 cm
    assert tilt < 5.0
    assert pen < 0.003


def test_hook_cannot_lift_convex_hull(built):
    """Gegenprobe: mit der alten Kollision (konvexe Huelle = voller Block, ohne
    die frueheren Behelfs-Rippen) gibt es keine Tasche -- die Haken drueckt es
    zur Seite, die Kiste bleibt (fast) liegen."""
    lift, tilt, pen = _hook_probe(built["ohne"], KLT, 0.15, 0.1307)
    print(f"\n[Haken alte Huelle] Hub {lift * 100:.1f} cm, Kippung {tilt:.1f} Grad, "
          f"Durchdringung {pen * 1e3:.1f} mm")
    assert lift < 0.02


def test_build_time_with_cache(built):
    """Mit Cache darf die Greif-Zerlegung den Szenenstart nicht spuerbar
    bremsen (Messwerte stehen oben in der Ausgabe)."""
    assert built["g_warm"] < 0.5
    assert built["t_warm"] < 5.0

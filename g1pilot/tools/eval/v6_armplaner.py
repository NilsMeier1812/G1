#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v6_armplaner.py -- V6: Messreihe Armplaner (offline, ohne Roboter/Sim).

Ruft den UNVERAENDERTEN Planer (manipulation/arm_planner.plan_arms) mit
demselben Robotermodell (URDF), derselben exakten Mesh-Selbstkollision und
denselben Parametern auf wie der Planer-Prozess des arm_controller
(arm_planner_process.py), auf einem festen Satz von Testfaellen plus
zufaelligen Start-Ziel-Paaren mit festem Seed. Je Fall --reps Wiederholungen
(Default 10; der Planer hat einen unge-seedeten RRT und ein CPU-Zeit-Budget,
die Wiederholungen zeigen also echte Streuung).

Zeit je Stufe: die Funktionen des Planers werden NUR IN DIESEM PROZESS mit
Stoppuhren umhuellt (_Problem.optimize/validate, _rrt_seed,
time_parameterize) -- am Planer-Code aendert sich nichts. Zuordnung der
Versuche zu Stufen ueber stats["tries"] des Planers:
  direkt               Pruefung der Geraden im Gelenkraum
  optimierer           Optimierer aus Gerade/Anheben/Zurueckziehen, voller Abstand
  rrt                  RRT-Connect-Rueckfall (Vorlage + Glaettung + Rohpruefung)
  reduzierter_abstand  Optimierer mit stufenweise reduziertem Abstand
  lauf_pose            Startbahn ueber die Lauf-Pose
  zeitparametrierung   time_parameterize der gefundenen Bahn
  sonstiges            Rest (Aufbau des Problems, Objekt-Auswahl, Zielpruefung)

Aufruf (im Sim-/Real-Container oder mit pip pin numpy matplotlib):
    python3 tools/eval/v6_armplaner.py [--reps 10] [--random 30] [--seed 12345]
                                      [--only GRUPPE ...] [--param clearance=0.05]
"""
import argparse
import contextlib
import math
import os
import sys
import time
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_common as ec  # noqa: E402

VERSUCH = "v6_armplaner"

# ── feste Posen (linker Arm, rad) -- bewusst HIER eingefroren und nicht aus
#    data/arm_poses.json gelesen, damit der Fallsatz bei geaenderten
#    gespeicherten Posen gleich bleibt. Quelle: arm_poses.json Stand 2026-11 +
#    Controller-Defaults (home_left, walk_left).
POSES = {
    "home": [0.3, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0],
    "lauf": [0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0],
    "sicher": [1.449, 0.426, 0.641, -0.374, -0.154, -1.008, -0.086],
    "ueber_tisch": [-0.8633, 0.0742, 0.1242, 0.365, -0.0604, 0.4799, 0.0992],
    "absenken": [-0.8663, 0.0374, 0.1888, 1.295, 0.0216, -0.3539, 0.0691],
    "anheben": [-0.783, 0.024, 0.158, 0.831, 0.012, -0.061, 0.065],
    "neben_klt": [0.12, 0.314, 0.223, 0.353, -0.345, -0.35, -0.359],
    "zudruecken": [0.21, 0.01, -0.035, 0.098, -0.011, -0.278, -0.027],
    "klt_anheben": [0.25, 0.027, -0.054, -0.367, -0.022, 0.141, -0.007],
    "vor_tisch": [1.064, 0.171, -0.055, -0.5, -0.215, -0.549, -0.055],
    "neben_box": [0.267, 0.78, 0.066, -0.249, 0.806, 0.022, -0.027],
    "linker_arm": [-0.815, 0.275, 0.054, 0.333, -0.054, 0.509, 0.028],
    "box_halten": [0.13, 0.388, -0.404, -0.084, 1.21, 0.351, 0.139],
}
MIRROR = [1, -1, -1, 1, -1, 1, -1]          # = arm_command.MIRROR_SIGNS
TABLE_TOP = 0.02                            # Tischoberkante ueber dem Becken [m]
GROUPS = ("frei", "tisch", "hindernis_mitte", "ziel_nah_tisch", "hand_auf_tisch",
          "beidarmig", "unerreichbar", "zufall")


def mirror(q):
    return [s * v for s, v in zip(MIRROR, q)]


def box(name, c, h, cls="obstacle"):
    return {"name": name, "cls": cls, "pos": [float(x) for x in c],
            "half_extents": [float(x) for x in h], "quat": [1.0, 0.0, 0.0, 0.0]}


def scene_table(top=TABLE_TOP):
    """Arbeitsplatz 1 wie test_arm_planner: Tisch + Greif-Box."""
    return [box("tisch", (0.62, 0.0, top - 0.4), (0.4, 0.8, 0.4)),
            box("greif_box", (0.43, 0.13, top + 0.05), (0.05, 0.05, 0.05), "grasp")]


# ══════════════════════════════════════════════════════════════════════
# Modell laden
# ══════════════════════════════════════════════════════════════════════
def _ament_stub():
    """ik_solver importiert ament_index_python; ausserhalb von ROS fehlt es.
    Stub NUR in diesem Prozess (das Werkzeug gibt URDF/Meshes explizit an)."""
    try:
        import ament_index_python.packages  # noqa: F401
        return False
    except Exception:
        pkg = types.ModuleType("ament_index_python")
        sub = types.ModuleType("ament_index_python.packages")
        sub.get_package_share_directory = lambda name: ec.PKG_ROOT
        pkg.packages = sub
        sys.modules["ament_index_python"] = pkg
        sys.modules["ament_index_python.packages"] = sub
        return True


class Model:
    def __init__(self, urdf):
        stub = _ament_stub()
        sys.path.insert(0, ec.PKG_ROOT)
        os.environ.setdefault("ROS_PACKAGE_PATH", os.path.dirname(ec.PKG_ROOT))
        import numpy as np
        import pinocchio as pin
        from g1pilot.utils.ik_solver import G1IKSolver
        from g1pilot.manipulation import arm_planner as ap
        from g1pilot.manipulation.arm_geometry import ArmGeometry, Environment, HAND_LINK_MIN
        from g1pilot.utils.joints_names import JOINT_LIMITS_RAD
        self.np, self.pin, self.ap = np, pin, ap
        self.Environment, self.HAND_LINK_MIN = Environment, HAND_LINK_MIN
        self.limits_all = JOINT_LIMITS_RAD
        t0 = time.monotonic()
        self.ik = G1IKSolver(urdf_path=urdf, mesh_dir=os.path.dirname(ec.PKG_ROOT), debug=False)
        self.geom = ArmGeometry(self.ik.model, self.ik.collision_model)
        self.scratch = self.ik.make_scratch_buffers()
        self.load_s = time.monotonic() - t0
        self.ament_stub = stub
        self.urdf = urdf

    def full(self, cur):
        f = self.np.zeros(29)
        for s in ("left", "right"):
            f[self.ap.joint_indices_for([s])] = cur[s]
        return f

    def self_check_fn(self, full, sides):
        idx = self.ap.joint_indices_for(sides)

        def check(qN):
            f = full.copy()
            f[idx] = qN
            return self.ik.arm_command_in_collision(f, hard=True, data=self.scratch["data"],
                                                    cdata=self.scratch["cdata_hard"])
        return check

    def limits(self, sides):
        return [self.limits_all[j] for j in self.ap.joint_indices_for(sides)]


# ══════════════════════════════════════════════════════════════════════
# Testfaelle
# ══════════════════════════════════════════════════════════════════════
def _case(cid, group, scene, cur, goal, sides, note, ik_target=None, expect=None):
    return {"id": cid, "gruppe": group, "szene": scene, "cur": cur, "goal": goal,
            "sides": sides, "notiz": note, "ik_target": ik_target, "erwartung": expect}


def _arm_pair(model, scene_objs, side, q_side, other):
    np = model.np
    cur = {"left": np.array(other["left"], float), "right": np.array(other["right"], float)}
    cur[side] = np.array(q_side, float)
    return cur


def _env_min_dist(model, scene_objs, full, sides=("left", "right")):
    """Kleinster Abstand der Arm-/Hand-Kugeln zu Nicht-Greif-Objekten (m)."""
    np = model.np
    g = model.geom
    g.set_base(full)
    g.set_environment(model.Environment(scene_objs))
    if not g.env.n:
        return float("inf")
    best = float("inf")
    for s in sides:
        q = full[model.ap.joint_indices_for([s])][None]
        fk = g.fk(q, [s])[s]
        ei = g.env_idx(s)
        d, _ = g.env_dist(fk["c"][:, ei], g.radii(s)[ei])
        d = np.where(g.env.is_grasp[None, None, :], np.inf, d)
        best = min(best, float(d.min()))
    return best


def _tcp(model, full, side):
    g = model.geom
    g.set_base(full)
    q = full[model.ap.joint_indices_for([side])][None]
    return g.fk(q, [side])[side]["tcp"][0]


def _ik_on_table(model, start_q, xy, gap, side="left"):
    """IK: Hand in der Orientierung von start_q ueber (x, y) so tief, dass der
    kleinste Abstand der Arm-/Hand-Kugeln zum Tisch gerade <= gap ist
    (gap <= 0: Hand beruehrt/liegt auf). Abwaerts in 2,5-mm-Schritten.
    Rueckgabe (q, tatsaechlicher Abstand) oder (None, None)."""
    np, pin = model.np, model.pin
    full = model.full({"left": np.array(POSES["ueber_tisch"]), "right": np.array(mirror(POSES["lauf"]))}
                      if side == "left" else
                      {"left": np.array(POSES["lauf"]), "right": np.array(mirror(POSES["ueber_tisch"]))})
    full[model.ap.joint_indices_for([side])] = start_q
    R = model.ik._frame_placement(side, model.ik._q_full_from(None, full),
                                  pin.Data(model.ik.model)).rotation
    sign = 1.0 if side == "left" else -1.0
    for z in np.arange(0.30, -0.05, -0.0025):
        res = model.ik.solve_pose(side, pin.SE3(R, np.array([xy[0], sign * xy[1], TABLE_TOP + z])), full)
        if res["q"] is None or res["pos_err_m"] > 0.01:
            continue
        q = np.asarray(res["q"], float)
        f2 = full.copy()
        f2[model.ap.joint_indices_for([side])] = q
        d = _env_min_dist(model, scene_table(), f2, (side,))
        if d <= max(gap, 0.0):
            return q, d
    return None, None


def _self_collision_goal(model):
    """Linker Arm quer durch den Rumpf (feste Kandidaten, erster mit harter
    Mesh-Selbstkollision)."""
    np = model.np
    for q in ([0.0, -1.2, 0.0, 1.6, 0.0, 0.0, 0.0], [0.3, -1.5, 0.5, 1.8, 0.0, 0.0, 0.0],
              [-0.3, -1.4, 0.0, 2.0, 0.0, 0.0, 0.0], [0.6, -1.5, -0.8, 1.2, 0.0, 0.0, 0.0]):
        q = np.array(q)
        lim = np.array(model.limits(["left"]))
        q = np.clip(q, lim[:, 0], lim[:, 1])
        full = model.full({"left": q, "right": np.array(mirror(POSES["lauf"]))})
        if model.ik.arm_command_in_collision(full, hard=True, data=model.scratch["data"],
                                             cdata=model.scratch["cdata_hard"]):
            return q
    return None


def build_cases(model, rng_seed, n_random):
    np = model.np
    P = {k: np.array(v, float) for k, v in POSES.items()}
    PR = {k: np.array(mirror(v), float) for k, v in POSES.items()}
    lauf = {"left": P["lauf"], "right": PR["lauf"]}
    cases = []

    def one(cid, group, scene, side, a, b, note, expect=None, other=lauf):
        cur = {"left": other["left"].copy(), "right": other["right"].copy()}
        src = P if side == "left" else PR
        cur[side] = src[a].copy() if isinstance(a, str) else np.asarray(a, float)
        goal = {side: src[b].copy() if isinstance(b, str) else np.asarray(b, float)}
        cases.append(_case(cid, group, scene, cur, goal, [side], note, expect=expect))

    # frei (6)
    one("F1", "frei", "frei", "left", "home", "sicher", "home -> sichere Pose")
    one("F2", "frei", "frei", "left", "sicher", "home", "sichere Pose -> home")
    one("F3", "frei", "frei", "left", "lauf", "ueber_tisch", "Lauf-Pose -> ueber Tisch (ohne Tisch)")
    one("F4", "frei", "frei", "left", "home", "box_halten", "home -> Box halten")
    one("F5", "frei", "frei", "left", "sicher", "neben_box", "sichere Pose -> neben Box")
    one("F6", "frei", "frei", "right", "home", "sicher", "rechts: home -> sichere Pose")
    # tisch (6), Arbeitsplatz 1
    one("T1", "tisch", "tisch", "left", "sicher", "ueber_tisch", "AP1: sichere Pose -> ueber Tisch")
    one("T2", "tisch", "tisch", "left", "ueber_tisch", "absenken", "AP1: absenken an die Greif-Box")
    one("T3", "tisch", "tisch", "left", "absenken", "anheben", "AP1: anheben")
    one("T4", "tisch", "tisch", "left", "anheben", "sicher", "AP1: zurueck in die sichere Pose")
    one("T5", "tisch", "tisch", "left", "lauf", "ueber_tisch", "Lauf-Pose -> ueber Tisch")
    one("T6", "tisch", "tisch", "right", "sicher", "ueber_tisch", "rechts: sichere Pose -> ueber Tisch")
    # hindernis_mitte (5): Quader auf der Mitte der geraden TCP-Bahn
    for cid, side, a, b in (("H1", "left", "sicher", "ueber_tisch"), ("H2", "left", "lauf", "sicher"),
                            ("H3", "left", "home", "neben_box"), ("H4", "left", "neben_box", "ueber_tisch"),
                            ("H5", "right", "lauf", "sicher")):
        one(cid, "hindernis_mitte", "hindernis", side, a, b, f"{a} -> {b}, Quader mitten im Weg")
    # ziel_nah_tisch (4): Ziel 1/2/3 cm ueber der Platte (IK)
    for cid, h, start in (("N1", 0.01, "ueber_tisch"), ("N2", 0.02, "sicher"),
                          ("N3", 0.03, "sicher"), ("N4", 0.015, "lauf")):
        q, d = _ik_on_table(model, P["ueber_tisch"], (0.30, 0.22), h)
        if q is None:
            continue
        one(cid, "ziel_nah_tisch", "tisch", "left", start, q,
            f"{start} -> Hand {h * 100:.1f} cm ueber dem Tisch (IK, Kugelabstand {d * 100:.1f} cm)")
    # hand_auf_tisch (4)
    on, d_on = _ik_on_table(model, P["ueber_tisch"], (0.30, 0.22), 0.0)
    if on is not None:
        one("A1", "hand_auf_tisch", "tisch", "left", "ueber_tisch", on, "ueber Tisch -> Hand auf dem Tisch")
        one("A2", "hand_auf_tisch", "tisch", "left", "sicher", on, "sichere Pose -> Hand auf dem Tisch")
        one("A3", "hand_auf_tisch", "tisch", "left", on, "ueber_tisch", "Hand auf dem Tisch -> ueber Tisch")
        one("A4", "hand_auf_tisch", "tisch", "left", on, "sicher", "Hand auf dem Tisch -> sichere Pose")
    # beidarmig (5)
    for cid, scene, a, b in (("B1", "frei", "sicher", "neben_klt"), ("B2", "frei", "neben_klt", "zudruecken"),
                             ("B3", "frei", "zudruecken", "klt_anheben"), ("B4", "frei", "home", "box_halten"),
                             ("B5", "tisch", "vor_tisch", "neben_box")):
        cur = {"left": P[a].copy(), "right": PR[a].copy()}
        goal = {"left": P[b].copy(), "right": PR[b].copy()}
        cases.append(_case(cid, "beidarmig", scene, cur, goal, ["left", "right"],
                           f"beide Arme: {a} -> {b}"))
    # unerreichbar (4)
    cur = {"left": P["sicher"].copy(), "right": PR["lauf"].copy()}
    cases.append(_case("U1", "unerreichbar", "tisch", cur, None, ["left"],
                       "kartesisches Ziel 1,2 m vor dem Becken (IK)",
                       ik_target=[1.2, 0.2, 0.1], expect="ik_failed"))
    cases.append(_case("U2", "unerreichbar", "wand", {k: v.copy() for k, v in cur.items()},
                       {"left": P["ueber_tisch"].copy()}, ["left"],
                       "Ziel hinter einer Wand (Ziel im Objekt)", expect="goal_in_collision"))
    q_in, _ = _ik_on_table(model, P["ueber_tisch"], (0.30, 0.22), 0.0)
    if q_in is not None:
        cases.append(_case("U3", "unerreichbar", "tisch_hoch", {k: v.copy() for k, v in cur.items()},
                           {"left": q_in}, ["left"],
                           "Hand-auf-Tisch-Ziel, Tisch 5 cm hoeher (Ziel 5 cm im Tisch)",
                           expect="goal_in_collision"))
    q_self = _self_collision_goal(model)
    if q_self is not None:
        cases.append(_case("U4", "unerreichbar", "frei", {k: v.copy() for k, v in cur.items()},
                           {"left": q_self}, ["left"],
                           "Ziel mit Arm im eigenen Rumpf (Selbstkollision)",
                           expect="goal_in_collision"))
    # zufall
    rng = np.random.default_rng(rng_seed)
    lim = np.array(model.limits(["left"]))
    n_try = 0
    k = 0
    while k < n_random and n_try < 200 * max(1, n_random):
        n_try += 1
        qa = lim[:, 0] + rng.random(7) * (lim[:, 1] - lim[:, 0])
        qb = lim[:, 0] + rng.random(7) * (lim[:, 1] - lim[:, 0])
        ok = True
        for q in (qa, qb):
            full = model.full({"left": q, "right": PR["lauf"]})
            if model.ik.arm_command_in_collision(full, hard=True, data=model.scratch["data"],
                                                 cdata=model.scratch["cdata_hard"]):
                ok = False
                break
            if _env_min_dist(model, scene_table(), full, ("left",)) < 0.0:
                ok = False
                break
        if not ok:
            continue
        k += 1
        cases.append(_case(f"Z{k:02d}", "zufall", "tisch", {"left": qa, "right": PR["lauf"].copy()},
                           {"left": qb}, ["left"], f"zufaelliges Paar (Seed {rng_seed})"))
    return cases, n_try


def scene_objects(model, case):
    """Szene eines Falls ('hindernis' wird aus der Geometrie des Falls gebaut)."""
    np = model.np
    s = case["szene"]
    if s == "frei":
        return []
    if s == "tisch":
        return scene_table()
    if s == "tisch_hoch":
        return scene_table(TABLE_TOP + 0.05)
    if s == "wand":
        return scene_table() + [box("wand", (0.30, 0.2, 0.2), (0.01, 0.5, 0.5))]
    side = case["sides"][0]
    full_a = model.full(case["cur"])
    goal_full = full_a.copy()
    goal_full[model.ap.joint_indices_for([side])] = case["goal"][side]
    pa, pb = _tcp(model, full_a, side), _tcp(model, goal_full, side)
    if s == "hindernis":
        # Quader auf der Mitte der Geraden im GELENKRAUM (dort faehrt die Hand
        # ohne Planer lang); Groesse so, dass Start und Ziel frei bleiben.
        mid = model.full(case["cur"])
        mid[model.ap.joint_indices_for([side])] = 0.5 * (case["cur"][side] + case["goal"][side])
        c = _tcp(model, mid, side)
        for h in (0.06, 0.05, 0.04, 0.03, 0.02):
            objs = [box("hindernis", c, (h, h, h))]
            if (_env_min_dist(model, objs, full_a, (side,)) > 0.06
                    and _env_min_dist(model, objs, goal_full, (side,)) > 0.06):
                return objs
        return [box("hindernis", c, (0.02, 0.02, 0.02))]
    raise ValueError(s)


# ══════════════════════════════════════════════════════════════════════
# Instrumentierung (nur in diesem Prozess)
# ══════════════════════════════════════════════════════════════════════
class Recorder:
    def __init__(self):
        self.events = []

    def wrap(self, kind, fn):
        rec = self

        def wrapper(*a, **k):
            t0 = time.monotonic()
            c0 = time.process_time()
            try:
                return fn(*a, **k)
            finally:
                rec.events.append((kind, t0, time.monotonic() - t0, time.process_time() - c0))
        wrapper.__wrapped__ = fn
        return wrapper


@contextlib.contextmanager
def instrumented(ap, rec):
    P = ap._Problem
    orig = {"opt": P.optimize, "val": P.validate, "rrt": ap._rrt_seed, "tp": ap.time_parameterize}
    P.optimize = rec.wrap("optimize", orig["opt"])
    P.validate = rec.wrap("validate", orig["val"])
    ap._rrt_seed = rec.wrap("rrt_seed", orig["rrt"])
    ap.time_parameterize = rec.wrap("time_param", orig["tp"])
    try:
        yield
    finally:
        P.optimize, P.validate = orig["opt"], orig["val"]
        ap._rrt_seed, ap.time_parameterize = orig["rrt"], orig["tp"]


STAGES = ("direkt", "optimierer", "rrt", "reduzierter_abstand", "lauf_pose",
          "zeitparametrierung", "sonstiges")


def stage_times(events, tries, clearance0, total_wall):
    """Ereignisse + stats['tries'] -> Wandzeit je Stufe [s]."""
    out = {s: 0.0 for s in STAGES}
    attempts = list(tries[1:]) if tries else []
    ai = 0
    pending_opt = None
    first_validate = True
    for kind, _t0, dt, _cpu in events:
        if kind == "time_param":
            out["zeitparametrierung"] += dt
        elif kind == "rrt_seed":
            out["rrt"] += dt
        elif kind == "optimize":
            pending_opt = dt
        elif kind == "validate":
            if first_validate and pending_opt is None:
                out["direkt"] += dt
                first_validate = False
                continue
            first_validate = False
            if pending_opt is None:            # Rohpruefung der RRT-Bahn
                out["rrt"] += dt
                continue
            name, clear = (attempts[ai][0], attempts[ai][1]) if ai < len(attempts) else ("?", 0.0)
            ai += 1
            if name == "rrt":
                st = "rrt"
            elif name == "via_walk":
                st = "lauf_pose"
            elif clear < clearance0 - 1e-9:
                st = "reduzierter_abstand"
            else:
                st = "optimierer"
            out[st] += pending_opt + dt
            pending_opt = None
    out["sonstiges"] = max(0.0, total_wall - sum(v for k, v in out.items() if k != "sonstiges"))
    return out


# ══════════════════════════════════════════════════════════════════════
# Bahn-Kennzahlen
# ══════════════════════════════════════════════════════════════════════
def path_metrics(model, case, scene_objs, res, prm, full_start):
    np = model.np
    g = model.geom
    sides = case["sides"]
    t = np.asarray(res["t"], float)
    Q = np.asarray(res["q"], float)
    g.set_base(full_start)
    g.set_environment(model.Environment(scene_objs))
    fk = g.fk(Q, sides)
    m = {}
    # Abstand zu Objekten (Hand ignoriert Greif-Objekte wie der Planer)
    dmin = np.full(len(Q), np.inf)
    mid_mask = np.zeros(len(Q), bool)
    mid_def = "relax"
    for s in sides:
        tcp = fk[s]["tcp"]
        far = ((np.linalg.norm(tcp - tcp[0], axis=1) > prm["relax_radius"])
               & (np.linalg.norm(tcp - tcp[-1], axis=1) > prm["relax_radius"]))
        mid_mask |= far
        if g.env.n:
            ei = g.env_idx(s)
            d, _ = g.env_dist(fk[s]["c"][:, ei], g.radii(s)[ei])
            hand = g.links(s)[ei] >= model.HAND_LINK_MIN
            grasp = g.env.is_grasp[None, None, :]
            d = np.where(grasp & hand[None, :, None], np.inf, d)
            dmin = np.minimum(dmin, d.min(axis=(1, 2)))
    if not mid_mask.any():
        mid_def = "bogen_25_75"
        seg = np.linalg.norm(np.diff(Q, axis=0), axis=1)
        sarc = np.concatenate([[0.0], np.cumsum(seg)])
        sarc = sarc / sarc[-1] if sarc[-1] > 0 else sarc
        mid_mask = (sarc >= 0.25) & (sarc <= 0.75)
    m["abstand_min_mitte_m"] = float(dmin[mid_mask].min()) if mid_mask.any() and g.env.n else float("inf")
    m["abstand_min_gesamt_m"] = float(dmin.min()) if g.env.n else float("inf")
    m["abstand_ende_m"] = float(dmin[-1]) if g.env.n else float("inf")
    m["mitte_definition"] = mid_def
    m["mitte_anteil"] = float(mid_mask.mean())
    # Selbstkollision dicht (Zwischenschritte <= 0.02 rad)
    check = model.self_check_fn(full_start, sides)
    n_hit, started_free, n_checked = 0, not check(Q[0]), 1
    for a, b in zip(Q[:-1], Q[1:]):
        n = max(1, int(math.ceil(np.abs(b - a).max() / 0.02)))
        for k in range(1, n + 1):
            q = a + (b - a) * (k / n)
            hit = check(q)
            n_checked += 1
            if hit and started_free:
                n_hit += 1
            if not hit:
                started_free = True
    m["selbstkollision_frei"] = n_hit == 0
    m["selbstkollision_treffer"] = n_hit
    m["selbstkollision_geprueft"] = n_checked
    # Weglaengen
    m["weg_gelenkraum_rad"] = float(np.linalg.norm(np.diff(Q, axis=0), axis=1).sum())
    m["gerade_gelenkraum_rad"] = float(np.linalg.norm(Q[-1] - Q[0]))
    hand_len, hand_lin = 0.0, 0.0
    v_tcp, a_tcp, v_sph = 0.0, 0.0, 0.0
    dt = np.diff(t)
    ok_dt = dt > 1e-9
    for s in sides:
        tcp = fk[s]["tcp"]
        hand_len += float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum())
        hand_lin += float(np.linalg.norm(tcp[-1] - tcp[0]))
        if len(t) > 1:
            vv = np.linalg.norm(np.diff(tcp, axis=0), axis=1)[ok_dt] / dt[ok_dt]
            v_tcp = max(v_tcp, float(vv.max()) if vv.size else 0.0)
            pts = np.concatenate([fk[s]["c"], tcp[:, None]], axis=1)
            vs = np.linalg.norm(np.diff(pts, axis=0), axis=-1).max(axis=1)[ok_dt] / dt[ok_dt]
            v_sph = max(v_sph, float(vs.max()) if vs.size else 0.0)
            vel = np.diff(tcp, axis=0)[ok_dt] / dt[ok_dt][:, None]
            tm = 0.5 * (t[1:] + t[:-1])[ok_dt]
            if len(tm) > 1:
                acc = np.linalg.norm(np.diff(vel, axis=0), axis=1) / np.diff(tm)
                a_tcp = max(a_tcp, float(acc.max()))
    m["weg_hand_m"] = hand_len
    m["gerade_hand_m"] = hand_lin
    m["umweg_hand"] = hand_len / hand_lin if hand_lin > 1e-6 else float("nan")
    m["v_hand_max_mps"] = v_tcp
    m["v_kugel_max_mps"] = v_sph
    m["a_hand_max_mps2"] = a_tcp
    # Gelenk-Tempo, -Beschleunigung, -Ruck (Differenzen der ausgegebenen Abtastung)
    vmax = amax = jmax = 0.0
    if len(t) > 2:
        V = np.diff(Q, axis=0)[ok_dt] / dt[ok_dt][:, None]
        tv = 0.5 * (t[1:] + t[:-1])[ok_dt]
        vmax = float(np.abs(V).max())
        if len(tv) > 1:
            A = np.diff(V, axis=0) / np.diff(tv)[:, None]
            ta = 0.5 * (tv[1:] + tv[:-1])
            amax = float(np.abs(A).max())
            if len(ta) > 1:
                J = np.diff(A, axis=0) / np.diff(ta)[:, None]
                jmax = float(np.abs(J).max())
    m["v_gelenk_max_radps"] = vmax
    m["a_gelenk_max_radps2"] = amax
    m["ruck_gelenk_max_radps3"] = jmax
    lim = np.array(model.limits(sides))
    margin = np.minimum(Q - lim[:, 0][None], lim[:, 1][None] - Q)
    m["gelenkgrenze_reserve_min_rad"] = float(margin.min())
    m["gelenkgrenzen_ok"] = bool(margin.min() >= -1e-6)
    m["tempo_ok"] = bool(vmax <= prm["vel_limit"] * (1 + 1e-6))
    m["beschl_ok"] = bool(amax <= prm["acc_limit"] * (1 + 1e-6))
    m["kart_tempo_ok"] = bool(v_sph <= prm["cart_vel_limit"] * (1 + 1e-6))
    m["kart_beschl_ok"] = bool(a_tcp <= prm["cart_acc_limit"] * (1 + 1e-6))
    m["dauer_s"] = float(t[-1]) if len(t) else 0.0
    m["n_abtastungen"] = int(len(t))
    return m


# ══════════════════════════════════════════════════════════════════════
# Lauf
# ══════════════════════════════════════════════════════════════════════
def run_case(model, case, prm, rec, walk_pose):
    np = model.np
    ap = model.ap
    sides = case["sides"]
    full = model.full(case["cur"])
    objs = scene_objects(model, case)
    model.geom.set_base(full)
    model.geom.set_environment(model.Environment(objs))
    check = model.self_check_fn(full, sides)
    rec.events = []
    out = {"ik_s": 0.0}
    goal = case["goal"]
    if case.get("ik_target") is not None:
        side = sides[0]
        R = model.ik._frame_placement(side, model.ik._q_full_from(None, full),
                                      model.pin.Data(model.ik.model)).rotation
        t0 = time.monotonic()
        res = model.ik.solve_pose(side, model.pin.SE3(R, np.array(case["ik_target"], float)), full)
        out["ik_s"] = time.monotonic() - t0
        out["ik_pos_err_m"] = float(res["pos_err_m"])
        if res["q"] is None or not res["converged"]:
            return {"ok": False, "reason": "ik_failed", "plan_time_s": out["ik_s"],
                    "wall_s": out["ik_s"], "cpu_s": out["ik_s"], "stats": {"tries": []}}, objs, out
        goal = {side: np.asarray(res["q"], float)}
    q_start = np.concatenate([case["cur"][s] for s in sides])
    q_goal = np.concatenate([goal[s] for s in sides])
    c0 = time.process_time()
    t0 = time.monotonic()
    with instrumented(ap, rec):
        res = ap.plan_arms(model.geom, sides, q_start, q_goal, model.limits(sides), prm,
                           self_check=check, walk_pose=walk_pose)
    res["wall_s"] = time.monotonic() - t0 + out["ik_s"]
    res["cpu_s"] = time.process_time() - c0
    return res, objs, out


def main(argv=None):
    ap_ = argparse.ArgumentParser(description="V6: Messreihe Armplaner (offline).")
    ap_.add_argument("--reps", type=int, default=10, help="Wiederholungen je Fall (Default 10)")
    ap_.add_argument("--random", type=int, default=30, help="zufaellige Start-Ziel-Paare (Default 30)")
    ap_.add_argument("--seed", type=int, default=12345, help="Seed der Zufallspaare")
    ap_.add_argument("--only", nargs="*", default=None, choices=GROUPS, help="nur diese Gruppen")
    ap_.add_argument("--param", action="append", default=[],
                     help="Planer-Parameter ueberschreiben, z. B. --param clearance=0.04")
    ap_.add_argument("--urdf", default=os.path.join(ec.PKG_ROOT, "description_files", "urdf",
                                                    "g1_29dof_inspire_ftp.urdf"),
                     help="Robotermodell (Default: Inspire-URDF wie im Realbetrieb)")
    ap_.add_argument("--no-plot", action="store_true", help="keine PDF-Diagramme")
    ec.add_common_args(ap_, VERSUCH)
    a = ap_.parse_args(argv)

    model = Model(a.urdf)
    np = model.np
    prm = dict(model.ap.DEFAULTS)
    # wie arm_controller._planner_params (Sim und Real: arm_velocity_limit 1.5,
    # ee_velocity_limit 0.5 Sim / 0.25 Real -> beide ergeben die Planer-Defaults)
    for kv in a.param:
        k, _, v = kv.partition("=")
        if k not in prm:
            ap_.error(f"unbekannter Parameter {k}")
        prm[k] = type(prm[k])(float(v)) if not isinstance(prm[k], int) else int(float(v))
    walk = {"left": np.array(POSES["lauf"]), "right": np.array(mirror(POSES["lauf"]))}

    cases, n_try = build_cases(model, a.seed, a.random if (not a.only or "zufall" in a.only) else 0)
    if a.only:
        cases = [c for c in cases if c["gruppe"] in a.only]
    run = ec.Run("v6_armplaner.py", VERSUCH, ec.MODE_OFFLINE,
                 {"reps": a.reps, "random": a.random, "seed": a.seed, "only": a.only,
                  "urdf": os.path.relpath(a.urdf, ec.PKG_ROOT), "planer_parameter": prm,
                  "lauf_pose": POSES["lauf"], "modell_laden_s": round(model.load_s, 2),
                  "ament_stub": model.ament_stub, "zufall_versuche": n_try},
                 out_root=a.out)
    # Fallliste
    crow = []
    for c in cases:
        crow.append({"id": c["id"], "gruppe": c["gruppe"], "szene": c["szene"],
                     "seiten": "+".join(c["sides"]), "notiz": c["notiz"],
                     "erwartung": c["erwartung"] or "erfolg",
                     "start": " ".join(f"{x:.4f}" for s in c["sides"] for x in c["cur"][s]),
                     "ziel": ("IK " + " ".join(map(str, c["ik_target"]))) if c["ik_target"] else
                     " ".join(f"{x:.4f}" for s in c["sides"] for x in c["goal"][s])})
    run.write_csv("faelle.csv", list(crow[0].keys()), crow, "fester Fallsatz + Zufallspaare")

    rec = Recorder()
    rows = []
    cols = None
    n_total = len(cases) * a.reps
    i_run = 0
    t_all = time.monotonic()
    for c in cases:
        for rep in range(a.reps):
            i_run += 1
            res, objs, extra = run_case(model, c, prm, rec, walk)
            st = stage_times(rec.events, res.get("stats", {}).get("tries"), prm["clearance"],
                             res.get("wall_s", 0.0))
            row = {"fall": c["id"], "gruppe": c["gruppe"], "wiederholung": rep + 1,
                   "seiten": "+".join(c["sides"]), "erfolg": bool(res["ok"]), "grund": res["reason"],
                   "erwartung_erfuellt": (res["reason"] == c["erwartung"]) if c["erwartung"]
                   else bool(res["ok"]),
                   "planzeit_wand_s": res.get("wall_s"), "planzeit_cpu_s": res.get("cpu_s"),
                   "planzeit_planer_s": res.get("plan_time_s"), "ik_s": extra.get("ik_s", 0.0),
                   "abstand_eingehalten_m": res.get("clearance") if res["ok"] else "",
                   "restverletzung_m": res.get("violation") if res["ok"] else "",
                   "versuche": len(res.get("stats", {}).get("tries", [])),
                   "versuchsfolge": " ".join(str(x[0]) for x in res.get("stats", {}).get("tries", []))}
            for s in STAGES:
                row["t_" + s + "_s"] = st[s]
            if res["ok"]:
                row.update(path_metrics(model, c, objs, res, prm, model.full(c["cur"])))
            if cols is None or len(row) > len(cols):
                cols = list(row.keys()) if cols is None else cols + [k for k in row if k not in cols]
            rows.append(row)
            if i_run % 20 == 0 or i_run == n_total:
                print(f"[v6] {i_run}/{n_total} Laeufe ({time.monotonic() - t_all:.0f} s)", flush=True)
    run.write_csv("laeufe.csv", cols, rows, "ein Lauf je Zeile")

    # Auswertung
    summary = {"laeufe": len(rows), "faelle": len(cases), "dauer_gesamt_s": time.monotonic() - t_all,
               "gruppen": {}}
    for gname in GROUPS:
        gr = [r for r in rows if r["gruppe"] == gname]
        if not gr:
            continue
        ok = [r for r in gr if r["erfolg"]]
        reasons = {}
        for r in gr:
            reasons[r["grund"]] = reasons.get(r["grund"], 0) + 1
        pt = [r["planzeit_wand_s"] for r in gr]
        d = {"laeufe": len(gr), "erfolge": len(ok), "erfolgsquote": len(ok) / len(gr),
             "erwartung_erfuellt": sum(1 for r in gr if r["erwartung_erfuellt"]) / len(gr),
             "gruende": reasons,
             "planzeit_wand_s": ec.describe(pt),
             "planzeit_spannweite_s": (max(pt) - min(pt)) if pt else None,
             "stufen_s": {s: ec.describe([r["t_" + s + "_s"] for r in gr]) for s in STAGES}}
        if ok:
            for key in ("abstand_min_mitte_m", "dauer_s", "weg_gelenkraum_rad", "weg_hand_m",
                        "umweg_hand", "a_gelenk_max_radps2", "ruck_gelenk_max_radps3",
                        "v_gelenk_max_radps", "v_kugel_max_mps"):
                d[key] = ec.describe([r[key] for r in ok if r.get(key) not in ("", None)
                                      and not (isinstance(r[key], float) and math.isinf(r[key]))])
            for key in ("selbstkollision_frei", "gelenkgrenzen_ok", "tempo_ok", "beschl_ok",
                        "kart_tempo_ok", "kart_beschl_ok"):
                d[key + "_anteil"] = sum(1 for r in ok if r.get(key)) / len(ok)
        summary["gruppen"][gname] = d
    allpt = [r["planzeit_wand_s"] for r in rows]
    summary["gesamt"] = {"erfolgsquote": sum(1 for r in rows if r["erfolg"]) / max(1, len(rows)),
                         "planzeit_wand_s": ec.describe(allpt)}
    summary["hinweise"] = [
        "Planzeit = Wandzeit um plan_arms (inkl. IK bei kartesischen Zielen); der Planer selbst "
        "rechnet mit einem CPU-Zeit-Budget (time_budget_s) und einer Wandzeit-Obergrenze 3x Budget.",
        "Beschleunigung/Ruck aus Differenzen der ausgegebenen 20-ms-Abtastung (keine Glaettung); "
        "der Ruck ist dadurch an Knickstellen der stueckweise linearen Interpolation hoch.",
        "Abstaende im Kugelmodell des Planers (arm_geometry), Hand ignoriert Greif-Objekte.",
    ]
    if not a.no_plot:
        try:
            plot(run, rows, summary)
        except Exception as e:      # noqa: BLE001
            summary["diagramm_fehler"] = str(e)
            print(f"[v6] Diagramme nicht erzeugt: {e}")
    run.finish(summary)
    for gname, d in summary["gruppen"].items():
        print(f"[v6] {gname:16s} Erfolg {d['erfolgsquote'] * 100:5.1f} %  "
              f"Planzeit Median {d['planzeit_wand_s'].get('median', 0):.3f} s  "
              f"max {d['planzeit_wand_s'].get('max', 0):.3f} s  Gruende {d['gruende']}")
    return 0


def plot(run, rows, summary):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    groups = [g for g in GROUPS if g in summary["gruppen"]]
    with PdfPages(run.path("diagramme.pdf")) as pdf:
        fig, ax = plt.subplots(figsize=(10, 5))
        data = [[r["planzeit_wand_s"] for r in rows if r["gruppe"] == g] for g in groups]
        ax.boxplot(data, showfliers=True)
        ax.set_xticks(range(1, len(groups) + 1))
        ax.set_xticklabels(groups)
        ax.set_yscale("log")
        ax.set_ylabel("Planungszeit (Wand) [s]")
        ax.set_title("V6 Planungszeit je Fallgruppe (alle Laeufe)")
        ax.tick_params(axis="x", rotation=30)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(10, 5))
        q = [summary["gruppen"][g]["erfolgsquote"] * 100 for g in groups]
        ax.bar(groups, q, color="#4c78a8")
        for i, v in enumerate(q):
            ax.text(i, v + 1, f"{v:.0f} %", ha="center")
        ax.set_ylim(0, 110)
        ax.set_ylabel("Erfolgsquote [%]")
        ax.set_title("V6 Erfolgsquote je Fallgruppe")
        ax.tick_params(axis="x", rotation=30)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(10, 5))
        data = [[r["t_" + s + "_s"] for r in rows if r["t_" + s + "_s"] > 0] for s in STAGES]
        ax.boxplot([d if d else [float("nan")] for d in data])
        ax.set_xticks(range(1, len(STAGES) + 1))
        ax.set_xticklabels(STAGES)
        ax.set_yscale("log")
        ax.set_ylabel("Zeit je Lauf [s] (nur Laeufe mit Anteil > 0)")
        ax.set_title("V6 Zeit je Planer-Stufe")
        ax.tick_params(axis="x", rotation=30)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        ok = [r for r in rows if r["erfolg"]]
        for key, lab in (("abstand_min_mitte_m", "kleinster Abstand Bahnmitte [m]"),
                         ("dauer_s", "Dauer der Bahn [s]"),
                         ("umweg_hand", "Handweg / Luftlinie [-]"),
                         ("a_gelenk_max_radps2", "groesste Gelenkbeschleunigung [rad/s^2]")):
            vals = [r[key] for r in ok if r.get(key) not in ("", None)
                    and not (isinstance(r[key], float) and (math.isinf(r[key]) or math.isnan(r[key])))]
            if not vals:
                continue
            fig, ax = plt.subplots(figsize=(8, 4.5))
            ax.hist(vals, bins=30, color="#59a14f")
            ax.set_xlabel(lab)
            ax.set_ylabel("Anzahl Laeufe")
            ax.set_title(f"V6 Verteilung: {lab} (n={len(vals)})")
            fig.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)
    run.files.append("diagramme.pdf")


if __name__ == "__main__":
    sys.exit(main())

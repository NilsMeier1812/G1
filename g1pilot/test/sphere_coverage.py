# -*- coding: utf-8 -*-
"""Hilfsfunktion fuer die Planer-Tests: wie weit ragen die FLAECHEN der Hand-
Kollisionsteile aus den Umgebungs-Kugeln des Armplaners heraus?

Bewusst unabhaengig vom Abtaster in arm_geometry.py: eigene, flaechen-
gewichtete Zufallspunkte auf den Dreiecken (plus alle Eckpunkte)."""
import numpy as np


def _triangles(g):
    n = int(g.num_polygons)
    return np.array([[g.polygons(i)[j] for j in range(3)] for i in range(n)], dtype=int)


def hand_surface_gap(model, collision_model, geom, side, hand_link_min, n_per_part=6000,
                     seed=0):
    """Je Hand-Kollisionsteil: groesster Abstand (m) eines Flaechenpunkts
    ausserhalb der naechsten Umgebungs-Kugel desselben Kettenglieds.
    Rueckgabe {geometry_name: max_aussen}."""
    import pinocchio as pin
    rng = np.random.default_rng(seed)
    data = pin.Data(model)
    pin.forwardKinematics(model, data, pin.neutral(model))
    jids = geom.chain[side]["jids"]
    link_of = {}
    for k, jid in enumerate(jids):
        for j in model.subtrees[jid]:
            link_of[j] = k
    sp = geom.spheres[side]
    env = sp["env_idx"]
    gaps = {}
    for g in collision_model.geometryObjects:
        k = link_of.get(g.parentJoint)
        if k is None or k < hand_link_min:
            continue
        P = np.asarray(g.geometry.points(), dtype=float)
        T = _triangles(g.geometry)
        A, B, C = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(B - A, C - A), axis=1)
        tri = rng.choice(len(T), n_per_part, p=area / area.sum())
        u, v = rng.random((2, n_per_part))
        flip = u + v > 1.0
        u[flip], v[flip] = 1.0 - u[flip], 1.0 - v[flip]
        pts = np.concatenate([P, A[tri] + u[:, None] * (B[tri] - A[tri])
                              + v[:, None] * (C[tri] - A[tri])])
        M = data.oMi[jids[k]].inverse() * data.oMi[g.parentJoint] * g.placement
        pts = pts @ np.array(M.rotation).T + np.array(M.translation)
        sel = env[sp["link"][env] == k]
        d = np.linalg.norm(pts[:, None] - sp["c"][sel][None], axis=2) - sp["r"][sel][None]
        gaps[g.name] = float(d.min(axis=1).max())
    return gaps

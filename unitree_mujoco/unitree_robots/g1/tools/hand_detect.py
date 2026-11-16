#!/usr/bin/env python3
"""Montage-Vorschlaege fuer eine neue Hand: wo sitzt die STL am Handgelenk-Flansch?

Rät aus der Form der STL, was in hand.yaml unter mount (origin, out, palm) und
tcp stehen sollte, und gibt mehrere Vorschlaege nach Wahrscheinlichkeit sortiert
zurueck. Feinjustage danach im Bearbeitungstool (hand_adjust.py).

Annahmen (passen zu typischen Werkzeugen wie den gedruckten Box-Haken):
  * Das Werkzeug ragt entlang seiner LAENGSTEN Achse aus dem Arm (out).
  * Am Arm-Ende sitzt der Adapter: dort ist das Teil schmaler/kompakter als am
    anderen Ende. Hat das Ende einen Absatz (Steckplatte -> breiterer Koerper),
    steckt die Platte im Arm und die Flansch-Flaeche liegt am Absatz.
  * Die Handflaeche (palm) ist die Seite, zu der die Spitze ueber den
    Grundkoerper hinausragt (z. B. eine Lippe/Kralle). Sonst: die flache Seite.
  * Greifpunkt (tcp) = auf der Handflaechen-Seite, 80 % der Laenge vom Flansch.

Alle Rechnungen auf einem gefuellten Voxel-Gitter (nur numpy + trimesh).

  python3 hand_detect.py datei.stl          Vorschlaege als Text
  python3 hand_detect.py datei.stl --json   als JSON (fuer den Assistenten)
"""
import argparse
import json
import math
import sys

N_CELLS = 64            # Voxel entlang der laengsten Achse
END_FRAC = 0.08         # Anteil der Laenge, der als "Ende" zaehlt
SHOULDER_MAX = 0.40     # Absatz (Steckplatte) nur in den ersten 40 % der Laenge
SHOULDER_GROW = 1.10    # Querschnitt waechst dauerhaft um >= 10 % -> Absatz
SHAPE_TOL = 0.15        # ... oder der Umriss aendert sich um >= 15 % (und >= 2 Voxel)
TCP_FRAC = 0.80
MESH_TYPES = (".stl", ".obj", ".ply", ".off", ".glb", ".gltf", ".3mf")


def load_mesh(path):
    import trimesh
    m = trimesh.load(str(path), force="mesh")
    if not isinstance(m, trimesh.Trimesh) or len(m.faces) == 0:
        raise ValueError(f"{path}: keine Dreiecke gefunden.")
    return m


def guess_unit(mesh) -> str:
    """mm oder m: eine Hand ist 3 cm .. 1 m gross."""
    return "m" if float(max(mesh.extents)) < 1.0 else "mm"


def voxel_points(mesh, n=N_CELLS):
    import numpy as np
    pitch = float(max(mesh.extents)) / n
    vox = mesh.voxelized(pitch)
    pts = None
    # "holes" braucht scipy, "base" nicht (das venv des Szenen-Editors hat kein scipy)
    for method in ("holes", "base"):
        try:
            pts = vox.copy().fill(method=method).points
            break
        except Exception:
            continue
    if pts is None:                         # nur Oberflaeche
        pts = vox.points
    if len(pts) < 10:
        pts = np.asarray(mesh.vertices)
    return np.asarray(pts, dtype=float), pitch


def volume_estimate(mesh) -> float:
    """Volumen in Mesh-Einheiten^3 (Voxel, falls das Mesh nicht dicht ist)."""
    if mesh.is_watertight and mesh.volume > 0:
        return float(mesh.volume)
    pts, pitch = voxel_points(mesh)
    return float(len(pts) * pitch ** 3)


def _face_axes(mesh):
    """Achsen aus den grossen ebenen Flaechen (CAD-Teile): Normale mit der meisten
    Flaeche, dann die meiste Flaeche senkrecht dazu. None, wenn das Teil kaum
    ebene, rechtwinklige Flaechen hat (organische Form)."""
    import numpy as np
    n = np.asarray(mesh.face_normals, dtype=float)
    a = np.asarray(mesh.area_faces, dtype=float)
    if len(n) == 0 or a.sum() <= 0:
        return None

    def strongest(mask):
        best, best_a = None, 0.0
        cand = np.flatnonzero(mask & (a > 0))
        order = cand[np.argsort(-a[cand])][:200]        # die groessten Dreiecke als Keime
        for k in order:
            share = a[np.abs(n @ n[k]) > 0.999].sum()
            if share > best_a:
                best, best_a = n[k], share
        return best, best_a
    n1, a1 = strongest(np.ones(len(n), bool))
    if n1 is None:
        return None
    n2, a2 = strongest(np.abs(n @ n1) < 0.01)
    if n2 is None or (a1 + a2) < 0.3 * a.sum():
        return None
    n2 = n2 - (n2 @ n1) * n1
    n2 /= np.linalg.norm(n2)
    return np.column_stack([n1, n2, np.cross(n1, n2)])


def _frames(mesh, pts):
    """Achsensystem fuer die Suche: die STL-Achsen, ausser das Teil liegt schraeg
    im Raum (dann seine eigenen Achsen: ebene Flaechen, sonst Hauptachsen)."""
    import numpy as np
    c = pts - pts.mean(axis=0)
    F = _face_axes(mesh)
    if F is None:
        _, vecs = np.linalg.eigh(c.T @ c)
        F = vecs[:, ::-1].copy()
    if np.linalg.det(F) < 0:
        F[:, 2] *= -1

    def box(B):
        return float(np.prod(np.ptp(c @ B, axis=0)))
    # CAD-Teile liegen meist achsparallel. Die eigenen Achsen nur nehmen, wenn sie
    # schraeg liegen UND das Teil darin deutlich kompakter umschliessen.
    tilted = min(np.max(np.abs(F[:, i])) for i in range(3)) < math.cos(math.radians(10))
    if tilted and box(F) < 0.8 * box(np.eye(3)):
        return [F]
    return [np.eye(3)]


def _bbox(a):
    return (float(a.min()), float(a.max())) if len(a) else (0.0, 0.0)


def _clean(v):
    out = [round(float(x), 6) for x in v]
    return [0.0 if abs(x) < 1e-9 else x for x in out]


def _analyse_out(pts, pitch, d, u, v):
    """Kennzahlen fuer 'out = d' (u, v = die beiden Querachsen)."""
    import numpy as np
    s, pu, pv = pts @ d, pts @ u, pts @ v
    lo, hi = float(s.min()), float(s.max())
    L = hi - lo
    nb = max(4, int(round(L / pitch)))
    edges = np.linspace(lo, hi, nb + 1)
    idx = np.clip(np.digitize(s, edges) - 1, 0, nb - 1)

    def region(a, b):                         # Anteile der Laenge [a, b)
        m = (s >= lo + a * L) & (s <= lo + b * L)
        return pu[m], pv[m]

    def area(bu, bv):
        return max(bu[1] - bu[0], pitch) * max(bv[1] - bv[0], pitch)

    eu, ev = region(0.0, END_FRAC)
    end_u, end_v = _bbox(eu), _bbox(ev)
    a_end = area(end_u, end_v)
    a_all = area(_bbox(pu), _bbox(pv))
    tu, tv = region(1.0 - 0.25, 1.0)
    a_tip = area(_bbox(tu), _bbox(tv))

    # Ende der Steckplatte, ab dem Arm-Ende gesucht (erste 40 % der Laenge):
    # Absatz = der Querschnitt aendert sich deutlich und dauerhaft gegenueber dem
    # Arm-Ende: mehr Material (Voxel je Scheibe) oder ein anderer Umriss
    # (breiter/schmaler in einer Querrichtung), z. B. Platte -> Koerper.
    counts = np.bincount(idx, minlength=nb).astype(float)
    ext = np.zeros((nb, 2))
    for k in range(nb):
        m = idx == k
        if m.any():
            ext[k] = (np.ptp(pu[m]), np.ptp(pv[m]))
    ref_c = float(np.median(counts[1:4]))           # Scheibe 0 ist oft angeschnitten
    ref_e = np.median(ext[1:4], axis=0)

    def changed(k):
        if counts[k] > SHOULDER_GROW * ref_c:
            return True
        tol = np.maximum(2.0 * pitch, SHAPE_TOL * ref_e)
        return bool(np.any(np.abs(ext[k] - ref_e) > tol))
    shoulder = None
    for k in range(4, nb - 2):
        if edges[k] - lo > SHOULDER_MAX * L:
            break
        if all(changed(j) for j in (k, k + 1, k + 2)):
            shoulder = float(edges[k])
            break
    origin_s = shoulder if shoulder is not None else lo
    origin = d * origin_s + u * (sum(end_u) / 2) + v * (sum(end_v) / 2)

    # Grundkoerper auf der Arm-Seite (hinter der Steckplatte)
    base0 = 0.10 if shoulder is None else min(0.45, (shoulder - lo) / L + 0.03)
    mu, mv = region(base0, 0.50)
    return {"lo": lo, "L": L, "a_end": a_end, "a_all": a_all, "a_tip": a_tip,
            "shoulder": None if shoulder is None else shoulder - lo,
            "origin": origin, "mid": (_bbox(mu), _bbox(mv)),
            "tip": (_bbox(tu), _bbox(tv)), "region": region}


def suggest_mounts(mesh, n=4, unit=""):
    """Vorschlaege [{origin, out, palm, tcp, score, why}] in Mesh-Einheiten."""
    import numpy as np
    us = f" {unit}" if unit else ""
    pts, pitch = voxel_points(mesh)
    cands = []
    for F in _frames(mesh, pts):
        max_ext = float(max(np.ptp(pts @ F, axis=0))) or 1.0
        for i in range(3):
            for sign in (1.0, -1.0):
                d = sign * F[:, i]
                u, v = F[:, (i + 1) % 3], F[:, (i + 2) % 3]
                a = _analyse_out(pts, pitch, d, u, v)
                s_len = a["L"] / max_ext                      # laengste Achse
                s_end = 1.0 - a["a_end"] / a["a_all"]           # schmales Arm-Ende
                s_tip = max(0.0, min(1.0, (a["a_tip"] - a["a_end"]) / a["a_all"]))
                s_out = 2.0 * s_len + 0.6 * s_end + 0.4 * s_tip
                (mu, mv), (tu, tv) = a["mid"], a["tip"]
                thin = (mu[1] - mu[0]) <= (mv[1] - mv[0])       # u ist die duenne Achse
                for axis, (mid, tip), is_thin in ((u, (mu, tu), thin), (v, (mv, tv), not thin)):
                    for ps in (1.0, -1.0):
                        palm = ps * axis
                        prot = (tip[1] - mid[1]) if ps > 0 else (mid[0] - tip[0])
                        s_palm = max(0.0, prot) / max(a["L"], 1e-9)
                        flat = 0.04 if is_thin else 0.0
                        score = s_out + 3.0 * s_palm + flat
                        # Greifpunkt: Handflaechen-Seite des Grundkoerpers, 80 % der Laenge
                        ms = a["lo"] + TCP_FRAC * a["L"]
                        cu, cv = a["region"](TCP_FRAC - 0.1, TCP_FRAC + 0.1)
                        cross = v if axis is u else u
                        cr = _bbox(cv if axis is u else cu)
                        surf = (mid[1] if ps > 0 else mid[0])
                        tcp = d * ms + axis * surf + cross * (sum(cr) / 2)
                        why = []
                        if s_len > 0.99:
                            why.append("längste Achse")
                        if a["shoulder"] is not None:
                            why.append(f"Steckplatte {a['shoulder']:.3g}{us} tief im Arm")
                        else:
                            why.append("Flansch am Ende")
                        if s_palm > 0.02:
                            why.append(f"Spitze ragt {max(0.0, prot):.3g}{us} zur Handfläche")
                        elif is_thin:
                            why.append("flache Seite = Handfläche")
                        cands.append({"origin": _clean(a["origin"]), "out": _clean(d),
                                      "palm": _clean(palm), "tcp": _clean(tcp),
                                      "score": round(score, 4), "why": ", ".join(why)})
    cands.sort(key=lambda c: -c["score"])
    out, seen = [], set()
    for c in cands:
        key = (tuple(np.round(c["out"], 3)), tuple(np.round(c["palm"], 3)))
        if key in seen:
            continue
        seen.add(key)
        out.append(c)
        if len(out) >= n:
            break
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Montage-Vorschlaege fuer eine Hand-STL")
    ap.add_argument("mesh")
    ap.add_argument("-n", type=int, default=4)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    mesh = load_mesh(a.mesh)
    unit = guess_unit(mesh)
    res = {"unit": unit, "extents": _clean(mesh.extents),
           "volume": volume_estimate(mesh), "suggestions": suggest_mounts(mesh, a.n, unit)}
    if a.json:
        print(json.dumps(res))
        return 0
    print(f"Einheit (geraten): {res['unit']}, Groesse {res['extents']}")
    for i, s in enumerate(res["suggestions"], 1):
        print(f"{i}) Punkte {s['score']:.2f}: origin {s['origin']} out {s['out']} "
              f"palm {s['palm']} tcp {s['tcp']}  ({s['why']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())

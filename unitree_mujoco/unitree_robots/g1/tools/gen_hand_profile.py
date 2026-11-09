#!/usr/bin/env python3
"""Hand-Profile fuer die Sim: eigene starre Haende (z. B. gedruckte Haken) statt
Rubber-/Inspire-Hand.

Ein Profil ist ein Ordner  unitree_robots/g1/hands/<profil>/  mit  hand.yaml  und
der STL aus dem CAD. Dieses Skript erzeugt daraus (alles wird committet, zur
Laufzeit braucht niemand trimesh):

  MuJoCo  unitree_robots/g1/g1_29dof_hand_<profil>.xml     Roboter (Basis g1_29dof.xml)
          unitree_robots/g1/scene_hand_<profil>.xml        Szene (wie scene.xml)
          unitree_robots/g1/hands/<profil>/generated/*.stl Meshes im Handgelenk-Frame (m)
  ROS     g1pilot/description_files/urdf/g1_29dof_hand_<profil>.urdf
          g1pilot/description_files/meshes/hand_<profil>_*.STL
          (IK, Armplaner, Kollisions-Gate, RViz; TCP *_hand_point_contact = tcp)
  CAD     hands/flansch_referenz_{left,right}.stl          Flansch im Flansch-Frame (mm)
          hands/<profil>/generated/flansch_in_stl_{left,right}.stl
                                                           Flansch in DEINEN STL-Koordinaten

Frames:
  Flansch-Frame  = *_wrist_yaw_link + (0.0415, 0, 0), Achsen wie wrist_yaw_link
                   (x zeigt vom Flansch weg; dort sitzen Rubber- und Inspire-Hand).
  Hand-Frame     = x: out, y: palm (links -y, rechts +y des wrist_yaw_link), z: x * y.
                   Versatz/Drehung aus hand.yaml wirken hier -> beide Seiten gleich.

Aufruf:
  python3 gen_hand_profile.py haken            erzeugen
  python3 gen_hand_profile.py --check haken    nur pruefen, ob die Ausgaben aktuell sind
  python3 gen_hand_profile.py --list           Profile auflisten
"""
import argparse
import copy
import hashlib
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
G1_DIR = HERE.parent                                    # unitree_robots/g1
REPO = HERE.parents[3]
HANDS_DIR = G1_DIR / "hands"
BASE_MJCF = G1_DIR / "g1_29dof.xml"
SCENE_TEMPLATE = G1_DIR / "scene.xml"
URDF_DIR = REPO / "g1pilot" / "description_files" / "urdf"
URDF_MESH_DIR = REPO / "g1pilot" / "description_files" / "meshes"
BASE_URDF = URDF_DIR / "g1_29dof_inspire_ftp.urdf"
WRIST_MESH = G1_DIR / "meshes" / "{side}_wrist_yaw_link.STL"

GEN_VERSION = "1"             # erhoehen, wenn sich die Ausgabe des Generators aendert
FLANGE_X = 0.0415             # Flansch-Flaeche im *_wrist_yaw_link-Frame (m)
SIDES = ("left", "right")
NAME_RE = re.compile(r"^[a-z0-9_]+$")
# Kollisions-Bits wie die Inspire-Finger: nur greifbare Objekte (Bit 2) und
# Umgebung (Bits 1+2), NICHT Boden/Roboterkoerper -> keine Selbstkollision.
COL_BITS = "2"


# ══════════════════════════════════════════════════════════════════════
# Profil lesen
# ══════════════════════════════════════════════════════════════════════
def list_profiles():
    if not HANDS_DIR.is_dir():
        return []
    return sorted(p.name for p in HANDS_DIR.iterdir()
                  if (p / "hand.yaml").is_file() and NAME_RE.match(p.name))


def profile_digest(name: str) -> str:
    """Fingerabdruck aller Eingaben (ohne yaml-Parser -> auch im System-python3)."""
    pdir = HANDS_DIR / name
    h = hashlib.sha1(GEN_VERSION.encode())
    raw = (pdir / "hand.yaml").read_bytes()
    h.update(raw)
    m = re.search(rb"^mesh:\s*([^\s#]+)", raw, re.M)
    if m:
        mp = pdir / m.group(1).decode()
        if mp.is_file():
            h.update(mp.read_bytes())
    h.update(Path(__file__).read_bytes())
    return h.hexdigest()[:16]


def _vec(v, n=3, what="Wert"):
    try:
        out = [float(x) for x in v]
    except Exception:
        raise ValueError(f"{what}: Liste mit {n} Zahlen erwartet, bekommen {v!r}")
    if len(out) != n:
        raise ValueError(f"{what}: Liste mit {n} Zahlen erwartet, bekommen {v!r}")
    return out


def load_profile(name: str) -> dict:
    import yaml
    if not NAME_RE.match(name):
        raise ValueError(f"Profilname '{name}': nur a-z, 0-9 und _ erlaubt.")
    pdir = HANDS_DIR / name
    cfg = yaml.safe_load((pdir / "hand.yaml").read_text(encoding="utf-8")) or {}
    mount = cfg.get("mount") or {}
    unit = str(cfg.get("unit", "mm")).strip().lower()
    if unit not in ("mm", "m"):
        raise ValueError("unit: nur 'mm' oder 'm'.")
    mesh = pdir / str(cfg.get("mesh", ""))
    if not mesh.is_file():
        raise ValueError(f"mesh: Datei nicht gefunden: {mesh}")
    mirror = cfg.get("mirror") or {}
    p = {
        "name": name,
        "title": str(cfg.get("name", name)),
        "dir": pdir,
        "mesh": mesh,
        "scale": 0.001 if unit == "mm" else 1.0,
        "mass": float(cfg.get("mass_kg", 0.15)),
        "color": _vec(cfg.get("color", [0.95, 0.55, 0.1, 1.0]), 4, "color"),
        "origin": _vec(mount.get("origin", [0, 0, 0]), 3, "mount.origin"),
        "out": _vec(mount.get("out", [1, 0, 0]), 3, "mount.out"),
        "palm": _vec(mount.get("palm", [0, -1, 0]), 3, "mount.palm"),
        "offset_mm": _vec(mount.get("offset_mm", [0, 0, 0]), 3, "mount.offset_mm"),
        "rotate_deg": _vec(mount.get("rotate_deg", [0, 0, 0]), 3, "mount.rotate_deg"),
        "mirror": {"left": bool(mirror.get("left", False)),
                   "right": bool(mirror.get("right", False))},
        "tcp": _vec(cfg.get("tcp", mount.get("origin", [0, 0, 0])), 3, "tcp"),
        "max_hulls": int((cfg.get("collision") or {}).get("max_hulls", 12)),
    }
    if p["mass"] <= 0:
        raise ValueError("mass_kg muss > 0 sein.")
    return p


# ══════════════════════════════════════════════════════════════════════
# Geometrie: STL-Koordinaten <-> wrist_yaw_link-Frame
# ══════════════════════════════════════════════════════════════════════
def _np():
    import numpy as np
    return np


def _unit(v, what):
    np = _np()
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError(f"{what}: Richtung darf nicht 0 sein.")
    return v / n


def _rpy(r, p, y):
    np = _np()
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def side_transform(p: dict, side: str):
    """Affine Abbildung STL-Punkt -> wrist_yaw_link-Frame (m):  p_w = A @ p_s + b.
    A enthaelt Skalierung, Montage-Drehung, Feindrehung und ggf. Spiegelung."""
    np = _np()
    out_s = _unit(p["out"], "mount.out")
    palm_s = _unit(p["palm"], "mount.palm")
    if abs(float(out_s @ palm_s)) > 1e-3:
        raise ValueError("mount.out und mount.palm muessen senkrecht zueinander stehen.")
    Q = np.column_stack([out_s, palm_s, np.cross(out_s, palm_s)])     # Hand -> STL
    palm_w = np.array([0.0, -1.0, 0.0]) if side == "left" else np.array([0.0, 1.0, 0.0])
    x_w = np.array([1.0, 0.0, 0.0])
    H = np.column_stack([x_w, palm_w, np.cross(x_w, palm_w)])         # Hand -> wrist
    M = np.eye(3)
    if p["mirror"][side]:
        n = Q[:, 2]
        M = np.eye(3) - 2.0 * np.outer(n, n)                          # Spiegelung in STL
    R_adj = _rpy(*[math.radians(a) for a in p["rotate_deg"]])
    A = H @ R_adj @ Q.T @ M * p["scale"]
    o = np.asarray(p["origin"], dtype=float)
    b = np.array([FLANGE_X, 0.0, 0.0]) + H @ (np.asarray(p["offset_mm"]) * 0.001) - A @ o
    return A, b


def transform_mesh(mesh, A, b):
    """Neues trimesh mit p' = A p + b; bei Spiegelung (det<0) Normalen umdrehen."""
    import trimesh
    np = _np()
    v = mesh.vertices @ A.T + b
    f = mesh.faces.copy()
    if np.linalg.det(A) < 0:
        f = f[:, ::-1]
    return trimesh.Trimesh(v, f, process=False)


def decompose(p: dict, cache_dir: Path):
    """Konvexe Zerlegung (V-HACD) der STL, gecacht nach Inhalt + max_hulls."""
    import trimesh
    raw = p["mesh"].read_bytes()
    key = hashlib.sha1(raw + str(p["max_hulls"]).encode()).hexdigest()[:10]
    cdir = cache_dir / f"hulls-{key}"
    files = sorted(cdir.glob("hull_*.stl"))
    if not files:
        mesh = trimesh.load(str(p["mesh"]), force="mesh")
        parts = trimesh.decomposition.convex_decomposition(
            mesh, maxConvexHulls=p["max_hulls"], resolution=200000, maxRecursionDepth=12)
        if isinstance(parts, dict):
            parts = [parts]
        cdir.mkdir(parents=True, exist_ok=True)
        for i, part in enumerate(parts):
            trimesh.Trimesh(part["vertices"], part["faces"]).export(str(cdir / f"hull_{i:02d}.stl"))
        files = sorted(cdir.glob("hull_*.stl"))
    return [trimesh.load(str(f), force="mesh") for f in files]


def _fmt(*v):
    return " ".join(f"{x:.7g}" for x in v)


def _inertia(mesh, mass):
    """(com, 3x3-Traegheit um com) bei gleichmaessiger Dichte, Masse = mass."""
    np = _np()
    vol = float(abs(mesh.volume))
    if not mesh.is_volume or vol < 1e-12:
        # nicht geschlossen -> Quader aus der Bounding-Box als Naeherung
        ext = mesh.extents
        com = mesh.bounds.mean(axis=0)
        I = np.diag([ext[1] ** 2 + ext[2] ** 2, ext[0] ** 2 + ext[2] ** 2,
                     ext[0] ** 2 + ext[1] ** 2]) * mass / 12.0
        return com, I
    com = np.asarray(mesh.center_mass)
    # moment_inertia gilt fuer mesh.density (Default 1) -> auf die echte Masse skalieren
    I = np.asarray(mesh.moment_inertia) * (mass / (vol * mesh.density))
    return com, I


# ══════════════════════════════════════════════════════════════════════
# Erzeugen
# ══════════════════════════════════════════════════════════════════════
def build(name: str, verbose=True) -> dict:
    import trimesh
    np = _np()
    p = load_profile(name)
    gen = p["dir"] / "generated"
    gen.mkdir(exist_ok=True)
    for f in gen.glob("*.stl"):
        f.unlink()
    for f in URDF_MESH_DIR.glob(f"hand_{name}_*.STL"):
        f.unlink()

    src = trimesh.load(str(p["mesh"]), force="mesh")
    hulls = decompose(p, HANDS_DIR / ".cache")

    info = {"profile": name, "title": p["title"], "mass_kg": p["mass"],
            "digest": profile_digest(name), "sides": {}}
    side_data = {}
    for side in SIDES:
        A, b = side_transform(p, side)
        vis = transform_mesh(src, A, b)
        cols = [transform_mesh(h, A, b) for h in hulls]
        vis.export(str(gen / f"{side}_visual.stl"))
        for i, c in enumerate(cols):
            c.export(str(gen / f"{side}_col_{i:02d}.stl"))
        vis.export(str(URDF_MESH_DIR / f"hand_{name}_{side}_visual.STL"))
        for i, c in enumerate(cols):
            c.export(str(URDF_MESH_DIR / f"hand_{name}_{side}_col_{i:02d}.STL"))
        com, I = _inertia(vis, p["mass"])
        tcp = A @ np.asarray(p["tcp"]) + b
        # Flansch in DEINEN STL-Koordinaten (zum Gegenpruefen im CAD), mm bzw. unit
        wrist = trimesh.load(str(WRIST_MESH).format(side=side), force="mesh")
        Ainv = np.linalg.inv(A)
        transform_mesh(wrist, Ainv, -Ainv @ b).export(str(gen / f"flansch_in_stl_{side}.stl"))
        side_data[side] = {"n_col": len(cols), "com": com, "I": I, "tcp": tcp}
        lo, hi = vis.bounds
        info["sides"][side] = {
            "tcp_wrist_m": [round(float(x), 5) for x in tcp],
            "com_wrist_m": [round(float(x), 5) for x in com],
            "bounds_wrist_m": [[round(float(x), 4) for x in lo], [round(float(x), 4) for x in hi]],
            "n_collision_hulls": len(cols),
            "mirrored": p["mirror"][side],
        }

    # Profil-unabhaengige Referenz: Flansch im Flansch-Frame, mm (fuers CAD)
    for side in SIDES:
        wrist = trimesh.load(str(WRIST_MESH).format(side=side), force="mesh")
        transform_mesh(wrist, np.eye(3) * 1000.0, np.array([-FLANGE_X * 1000.0, 0, 0])).export(
            str(HANDS_DIR / f"flansch_referenz_{side}.stl"))

    write_mjcf(p, side_data)
    write_urdf(p, side_data)
    (gen / "info.json").write_text(json.dumps(info, indent=2, ensure_ascii=False) + "\n")
    if verbose:
        print(f"OK: Profil '{name}' ({p['title']}) erzeugt.")
        for side in SIDES:
            s = info["sides"][side]
            print(f"  {side:5s}: {s['n_collision_hulls']} Kollisions-Teile, "
                  f"TCP {s['tcp_wrist_m']} m, Spitze bis x={s['bounds_wrist_m'][1][0]:.3f} m")
        print(f"  MuJoCo: {mjcf_path(name).relative_to(REPO)}, {scene_path(name).relative_to(REPO)}")
        print(f"  URDF  : {urdf_path(name).relative_to(REPO)}")
    return info


def mjcf_path(name):
    return G1_DIR / f"g1_29dof_hand_{name}.xml"


def scene_path(name):
    return G1_DIR / f"scene_hand_{name}.xml"


def urdf_path(name):
    return URDF_DIR / f"g1_29dof_hand_{name}.urdf"


def write_mjcf(p, side_data):
    name = p["name"]
    t = ET.parse(BASE_MJCF)
    root = t.getroot()
    root.set("model", f"g1_29dof_hand_{name}")
    asset = root.find("asset")
    rel = f"../hands/{name}/generated"             # relativ zu meshdir="meshes"
    for side in SIDES:
        wb = next(b for b in root.iter("body") if b.get("name") == f"{side}_wrist_yaw_link")
        for g in list(wb.findall("geom")):
            if g.get("mesh") == f"{side}_rubber_hand":
                wb.remove(g)
        for a in list(asset.findall("mesh")):
            if a.get("name") == f"{side}_rubber_hand":
                asset.remove(a)
        sd = side_data[side]
        ET.SubElement(asset, "mesh", {"name": f"hand_{name}_{side}_visual",
                                      "file": f"{rel}/{side}_visual.stl"})
        for i in range(sd["n_col"]):
            ET.SubElement(asset, "mesh", {"name": f"hand_{name}_{side}_col_{i:02d}",
                                          "file": f"{rel}/{side}_col_{i:02d}.stl"})
        body = ET.SubElement(wb, "body", {"name": f"{side}_hand_tool_link"})
        I = sd["I"]
        ET.SubElement(body, "inertial", {
            "pos": _fmt(*sd["com"]), "mass": f"{p['mass']:.6g}",
            "fullinertia": _fmt(I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])})
        ET.SubElement(body, "geom", {
            "name": f"{side}_hand_tool_visual", "type": "mesh",
            "mesh": f"hand_{name}_{side}_visual", "contype": "0", "conaffinity": "0",
            "group": "1", "density": "0", "rgba": _fmt(*p["color"])})
        for i in range(sd["n_col"]):
            ET.SubElement(body, "geom", {
                "name": f"{side}_hand_tool_col_{i:02d}", "type": "mesh",
                "mesh": f"hand_{name}_{side}_col_{i:02d}", "contype": COL_BITS,
                "conaffinity": COL_BITS, "group": "3", "density": "0",
                "friction": "1.0 0.01 0.001", "rgba": "0.9 0.3 0.3 0.4"})
        ET.SubElement(body, "site", {"name": f"{side}_hand_flange", "pos": _fmt(FLANGE_X, 0, 0),
                                     "size": "0.004", "group": "4", "rgba": "0.2 0.4 1 0.8"})
        ET.SubElement(body, "site", {"name": f"{side}_hand_tcp", "pos": _fmt(*sd["tcp"]),
                                     "size": "0.006", "group": "4", "rgba": "0.1 0.9 0.2 0.8"})
    ET.indent(t, space="  ")
    header = (f"<!-- ERZEUGT von tools/gen_hand_profile.py aus hands/{name}/hand.yaml. "
              "Nicht von Hand aendern. -->\n")
    mjcf_path(name).write_text(header + ET.tostring(root, encoding="unicode") + "\n")

    scene = SCENE_TEMPLATE.read_text()
    scene = scene.replace('<include file="g1_29dof.xml"/>', f'<include file="{mjcf_path(name).name}"/>', 1)
    scene = re.sub(r'<mujoco model="[^"]*">', f'<mujoco model="g1_29dof_hand_{name} scene">', scene, count=1)
    if mjcf_path(name).name not in scene:
        raise RuntimeError("scene.xml: <include file=\"g1_29dof.xml\"/> nicht gefunden.")
    scene_path(name).write_text(scene)


def write_urdf(p, side_data):
    name = p["name"]
    t = ET.parse(BASE_URDF)
    root = t.getroot()
    root.set("name", f"g1_29dof_hand_{name}")
    joints = root.findall("joint")
    links = {l.get("name"): l for l in root.findall("link")}
    children = {}
    for j in joints:
        children.setdefault(j.find("parent").get("link"), []).append(j)
    for side in SIDES:
        # Inspire-Baum unter *_base_joint entfernen (Handbasis, Finger, Sensoren)
        base_j = next(j for j in joints if j.get("name") == f"{side}_base_joint")
        stack = [base_j]
        while stack:
            j = stack.pop()
            child = j.find("child").get("link")
            stack.extend(children.get(child, []))
            root.remove(j)
            if child in links:
                root.remove(links.pop(child))
        sd = side_data[side]
        I = sd["I"]
        link = ET.SubElement(root, "link", {"name": f"{side}_hand_tool_link"})
        ine = ET.SubElement(link, "inertial")
        ET.SubElement(ine, "origin", {"xyz": _fmt(*sd["com"]), "rpy": "0 0 0"})
        ET.SubElement(ine, "mass", {"value": f"{p['mass']:.6g}"})
        ET.SubElement(ine, "inertia", {"ixx": f"{I[0, 0]:.7g}", "iyy": f"{I[1, 1]:.7g}",
                                       "izz": f"{I[2, 2]:.7g}", "ixy": f"{I[0, 1]:.7g}",
                                       "ixz": f"{I[0, 2]:.7g}", "iyz": f"{I[1, 2]:.7g}"})
        vis = ET.SubElement(link, "visual")
        ET.SubElement(vis, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geo = ET.SubElement(vis, "geometry")
        ET.SubElement(geo, "mesh", {
            "filename": f"package://g1pilot/description_files/meshes/hand_{name}_{side}_visual.STL"})
        mat = ET.SubElement(vis, "material", {"name": f"hand_{name}"})
        ET.SubElement(mat, "color", {"rgba": _fmt(*p["color"])})
        for i in range(sd["n_col"]):
            col = ET.SubElement(link, "collision")
            ET.SubElement(col, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            geo = ET.SubElement(col, "geometry")
            ET.SubElement(geo, "mesh", {
                "filename": f"package://g1pilot/description_files/meshes/hand_{name}_{side}_col_{i:02d}.STL"})
        j = ET.SubElement(root, "joint", {"name": f"{side}_hand_tool_joint", "type": "fixed"})
        ET.SubElement(j, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(j, "parent", {"link": f"{side}_wrist_yaw_link"})
        ET.SubElement(j, "child", {"link": f"{side}_hand_tool_link"})
        # Flansch-Frame (nur zur Anzeige/Kontrolle in RViz)
        ET.SubElement(root, "link", {"name": f"{side}_hand_flange"})
        j = ET.SubElement(root, "joint", {"name": f"{side}_hand_flange_joint", "type": "fixed"})
        ET.SubElement(j, "origin", {"xyz": _fmt(FLANGE_X, 0, 0), "rpy": "0 0 0"})
        ET.SubElement(j, "parent", {"link": f"{side}_wrist_yaw_link"})
        ET.SubElement(j, "child", {"link": f"{side}_hand_flange"})
        # TCP auf den Greifpunkt des Profils legen (Orientierung = wrist_yaw_link)
        tj = next(j for j in root.findall("joint")
                  if j.get("name") == f"{side}_hand_point_contact_fix_joint")
        tj.find("origin").set("xyz", _fmt(*sd["tcp"]))
        tj.find("origin").set("rpy", "0 0 0")
    ET.indent(t, space="  ")
    header = (f"<?xml version=\"1.0\"?>\n<!-- ERZEUGT von unitree_mujoco/unitree_robots/g1/tools/"
              f"gen_hand_profile.py aus hands/{name}/hand.yaml (Basis: "
              f"g1_29dof_inspire_ftp.urdf ohne Inspire-Haende). Nicht von Hand aendern. -->\n")
    urdf_path(name).write_text(header + ET.tostring(root, encoding="unicode") + "\n")


def check(name: str) -> bool:
    """True, wenn alle Ausgaben existieren und zu hand.yaml/STL passen."""
    info_f = HANDS_DIR / name / "generated" / "info.json"
    if not (info_f.is_file() and mjcf_path(name).is_file() and scene_path(name).is_file()
            and urdf_path(name).is_file()):
        return False
    try:
        return json.loads(info_f.read_text()).get("digest") == profile_digest(name)
    except Exception:
        return False


def _reexec_in_editor_venv():
    """Wie build_env_scene.py: fehlt trimesh im System-python3, einmal mit dem
    Python aus scene_editor/.venv (./setup.sh) neu starten."""
    try:
        import trimesh, yaml  # noqa: F401
        return
    except ImportError:
        pass
    venv = REPO / "unitree_mujoco" / "scene_editor" / ".venv"
    py = venv / "bin" / "python"
    if py.is_file() and Path(sys.prefix).resolve() != venv.resolve():
        os.execv(str(py), [str(py), str(Path(sys.argv[0]).resolve()), *sys.argv[1:]])


def main(argv=None):
    ap = argparse.ArgumentParser(description="Hand-Profil (hands/<name>/hand.yaml) erzeugen")
    ap.add_argument("profile", nargs="?", help="Profilname (Ordner unter hands/)")
    ap.add_argument("--check", action="store_true", help="nur pruefen, ob aktuell (Exit 0/1)")
    ap.add_argument("--list", action="store_true", help="Profile mit Anzeigenamen auflisten")
    a = ap.parse_args(argv)
    if a.list:
        for n in list_profiles():
            raw = (HANDS_DIR / n / "hand.yaml").read_text(encoding="utf-8")
            m = re.search(r"^name:\s*(.+?)\s*(#.*)?$", raw, re.M)
            print(f"{n}|{m.group(1) if m else n}")
        return 0
    if not a.profile:
        ap.error("Profilname fehlt (verfuegbar: " + ", ".join(list_profiles()) + ")")
    if a.profile not in list_profiles():
        print(f"Profil '{a.profile}' nicht gefunden (verfuegbar: {', '.join(list_profiles())}).",
              file=sys.stderr)
        return 2
    if a.check:
        ok = check(a.profile)
        print("aktuell" if ok else "veraltet")
        return 0 if ok else 1
    _reexec_in_editor_venv()
    try:
        build(a.profile)
    except ImportError as e:
        print(f"FEHLER: {e}. Bitte einmal unitree_mujoco/scene_editor/setup.sh ausfuehren "
              "(installiert trimesh + vhacdx).", file=sys.stderr)
        return 1
    except ValueError as e:
        print(f"FEHLER in hands/{a.profile}/hand.yaml: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

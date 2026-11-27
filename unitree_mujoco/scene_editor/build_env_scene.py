#!/usr/bin/env python3
"""
Kombiniert die feste G1-BASIS mit einer UMGEBUNG zu einer lauffaehigen Szene.

Idee des Systems:
  * BASIS (kommt IMMER automatisch dazu, hier fest verdrahtet):
      - der G1 (per <include>, Hand-Variante je nach --inspire)
      - Lichtquelle
      - Boden + Skybox/Groundplane
      - der Weld "hold_base_weld" (haelt den G1 am Anfang an torso_link fest;
        wird vom Sim per Name gesteuert, siehe hold_base.py)
      - visual/statistic-Grundeinstellungen
  * UMGEBUNG (scene_editor/scenes/<name>.xml): enthaelt NUR Hindernisse bzw.
    Objekte zum Interagieren/Greifen - also nur <asset> (eigene Meshes) und
    die Objekte im <worldbody>. KEIN Roboter, KEIN Boden, KEIN Licht, KEIN Weld.

Dieses Skript erzeugt die kombinierte Szene  unitree_robots/g1/scene_env_<name>.xml
= BASIS + Objekte der Umgebung. Mesh-Pfade der Umgebung werden relativ
umgerechnet, damit sie auf dem Host UND im read-only gemounteten Docker-Container
stimmen.

Hindernis vs. Greif-Objekt (fuer Nav/IK, siehe g1pilot/docs/31-T-Navigation.md):
Klassifikation per Namenskonvention -- ein Objekt-Name, der mit "grasp_"
beginnt, wird HIER automatisch von einem statischen <geom> in einen FREIEN
Koerper umgewandelt (<body><freejoint/><geom/></body>): nur so kann MuJoCo es
beim Anfassen/Greifen bewegen (Masse/Traegheit werden von MuJoCo automatisch
aus Geometrie + Default-Dichte berechnet). Alle anderen Objekte bleiben
statische Hindernisse. Dieselbe Namenskonvention wird von
unitree_mujoco/simulate_python/scene_objects.py (Sim-Seite) und
g1pilot/g1pilot/navigation/scene_bridge.py (ROS-Seite) verwendet.

Aufruf:
    python3 build_env_scene.py --env scenes/warehouse.xml
    python3 build_env_scene.py --env scenes/warehouse.xml --inspire 1
    python3 build_env_scene.py --env scenes/warehouse.xml --hands haken
"""
import argparse
import hashlib
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

GRASP_PREFIX_RE = re.compile(r"^grasp_", re.IGNORECASE)
# Startpunkt-Markierung: ein Objekt, dessen Name mit "g1_spawn" beginnt, legt
# fest, WO der G1 startet (Position x/y + Blickrichtung aus der Drehung um z).
# Es wird beim Kombinieren entfernt und als <custom><numeric name="g1_spawn"
# data="x y yaw"/> abgelegt; unitree_mujoco.py setzt den Roboter dorthin.
SPAWN_PREFIX_RE = re.compile(r"^g1_spawn", re.IGNORECASE)
# Stations-Markierungen: Objekte "station_<Name>" (beliebig viele) = Ziele fuer
# die Stations-Knoepfe der Demo-GUI (AUTO NAV). Position = Ziel, Drehung um z =
# Blickrichtung am Ziel. Werden wie g1_spawn entfernt (kein Hindernis) und als
# <custom><numeric name="station_<Name>" data="x y yaw"/> abgelegt; der
# Szenen-Publisher schickt sie mit nach ROS (/scene_markers, ns g1scene:station).
STATION_PREFIX_RE = re.compile(r"^station_", re.IGNORECASE)
# Anbauteile an Greif-Objekten: ein Geom "<grasp-Name>__<beliebig>" (z.B.
# "grasp_klt__rippe_links") wird beim Kombinieren in den freien Koerper seines
# Greif-Objekts gehaengt (Pose relativ umgerechnet) -- so lassen sich z.B. Rippen/
# Griffleisten an ein Mesh anbauen, ohne das Mesh zu aendern (dessen Kollision
# ist die konvexe Huelle, Vorspruenge im Mesh wuerden dort verschwinden).
ATTACH_SEP = "__"

HERE = Path(__file__).resolve().parent          # .../unitree_mujoco/scene_editor
MJ_ROOT = HERE.parent                            # .../unitree_mujoco
G1_DIR = MJ_ROOT / "unitree_robots" / "g1"
G1_MESHDIR = G1_DIR / "meshes"                    # meshdir des Robotermodells

# Kollisions-Zerlegung statischer Mesh-Hindernisse (siehe add_collision_hulls).
# Liegt unter unitree_mujoco/ -> auch im read-only gemounteten Container sichtbar.
COLLISION_CACHE = HERE / "meshes" / ".collision"
# Namens-Suffix des erzeugten Kollisions-Bodys. scene_objects.py (Sim -> RViz)
# erkennt ihn daran, blendet ihn aus und behandelt das zugehoerige Optik-Mesh
# (contype/conaffinity 0) trotzdem als Hindernis. Beide Stellen muessen passen.
COLLISION_BODY_SUFFIX = "__collision"
# contype/conaffinity fuer Umgebungs-Objekte: Bit 1 (G1-Koerper) + Bit 2 (Haende).
ENV_COLLISION_BITS = "3"
EDITOR_VENV = HERE / ".venv"
ORIENT_ATTRS = ("quat", "euler", "axisangle", "xyaxes", "zaxis")
CONTACT_ATTRS = ("friction", "solref", "solimp", "condim", "priority", "margin")

# Roboter-Basismodell je nach Hand-Variante (wie config.py)
ROBOT_FILES = {
    "0": "g1_29dof.xml",
    "1": "g1_29dof_inspire_ftp.xml",
}


def rel_to_meshdir(mesh_abs: Path) -> str:
    """Pfad relativ zu g1/meshes, plattform-neutral (mit '/')."""
    return os.path.relpath(mesh_abs, G1_MESHDIR).replace(os.sep, "/")


def build_base(robot_file: str, model_name: str):
    """Baut die feste Basis (G1 + Licht + Boden + Weld + Technik).

    Gibt (mujoco_root, asset_element, worldbody_element) zurueck, damit die
    Objekte der Umgebung anschliessend in asset/worldbody eingemischt werden.
    """
    mj = ET.Element("mujoco", {"model": model_name})
    ET.SubElement(mj, "include", {"file": robot_file})
    ET.SubElement(mj, "statistic", {"center": "0 0 0.5", "extent": "2.0"})

    vis = ET.SubElement(mj, "visual")
    ET.SubElement(vis, "headlight",
                  {"diffuse": "0.6 0.6 0.6", "ambient": "0.3 0.3 0.3", "specular": "0 0 0"})
    ET.SubElement(vis, "rgba", {"haze": "0.15 0.25 0.35 1"})
    ET.SubElement(vis, "global", {"azimuth": "-130", "elevation": "-20"})

    asset = ET.SubElement(mj, "asset")
    ET.SubElement(asset, "texture",
                  {"type": "skybox", "builtin": "gradient",
                   "rgb1": "0.3 0.5 0.7", "rgb2": "0 0 0", "width": "512", "height": "3072"})
    ET.SubElement(asset, "texture",
                  {"type": "2d", "name": "groundplane", "builtin": "checker", "mark": "edge",
                   "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3", "markrgb": "0.8 0.8 0.8",
                   "width": "300", "height": "300"})
    ET.SubElement(asset, "material",
                  {"name": "groundplane", "texture": "groundplane", "texuniform": "true",
                   "texrepeat": "5 5", "reflectance": "0.2"})

    wb = ET.SubElement(mj, "worldbody")
    # Lichtquelle (Basis)
    ET.SubElement(wb, "light", {"pos": "0 0 1.5", "dir": "0 0 -1", "directional": "true"})
    # Boden (Basis)
    ET.SubElement(wb, "geom",
                  {"name": "floor", "size": "0 0 0.05", "type": "plane", "material": "groundplane"})

    # Weld, der den G1 am Anfang festhaelt (per Name vom Sim gesteuert).
    eq = ET.SubElement(mj, "equality")
    ET.SubElement(eq, "weld",
                  {"name": "hold_base_weld", "body1": "torso_link",
                   "solref": "0.01 1", "solimp": "0.99 0.999 0.001 0.5 2"})

    return mj, asset, wb


def _wrap_grasp_geom(geom_el):
    """Wandelt ein statisches <geom name="grasp_..."/> in einen freien Koerper
    um (<body><freejoint/><geom/></body>), damit MuJoCo es beim Greifen/
    Anfassen bewegen kann. Body UND Geom tragen denselben Namen (in MuJoCo
    erlaubt -- Bodies und Geoms haben getrennte Namensraeume); das ist wichtig,
    weil die Sim-Seite (scene_objects.py) die LIVE-Pose ueber den BODY-Namen
    aufloest. Body erbt Pose (pos/quat) vom Geom, das Geom selbst wird auf
    Identity relativ zum Body gesetzt (pos/quat entfernt).
    """
    name = geom_el.get("name")
    pos = geom_el.get("pos", "0 0 0")
    quat = geom_el.get("quat")

    body = ET.Element("body", {"name": name, "pos": pos})
    if quat:
        body.set("quat", quat)
    ET.SubElement(body, "freejoint")

    inner = ET.Element("geom", dict(geom_el.attrib))
    inner.attrib.pop("pos", None)
    inner.attrib.pop("quat", None)
    # Der Scene-Editor exportiert Meshes als statische Geoms mit mass="0" --
    # an einem freien Koerper verweigert MuJoCo das ("mass and inertia of
    # moving bodies must be larger than mjMINVAL"). Dann Masse aus Geometrie
    # und Default-Dichte berechnen lassen.
    try:
        if float(inner.get("mass", "1")) <= 0.0:
            inner.attrib.pop("mass")
    except ValueError:
        pass
    body.append(inner)
    return body


def _is_grasp_geom(el) -> bool:
    return (el.tag == "geom" and bool(GRASP_PREFIX_RE.match(el.get("name") or ""))
            and ATTACH_SEP not in (el.get("name") or ""))


def _attach_parts(body_el, parts, warnings):
    """Anbauteile (Welt-Pose) als Geoms relativ zum Greif-Koerper anhaengen."""
    bpos = _floats(body_el.get("pos"), (0.0, 0.0, 0.0))
    bq = _floats(body_el.get("quat"), (1.0, 0.0, 0.0, 0.0))
    bq_inv = [bq[0], -bq[1], -bq[2], -bq[3]]
    for g in parts:
        gpos = _floats(g.get("pos"), (0.0, 0.0, 0.0))
        gq = _floats(g.get("quat"), (1.0, 0.0, 0.0, 0.0))
        new = ET.Element("geom", dict(g.attrib))
        new.set("pos", _fmt(_quat_rotate(bq_inv, [a - b for a, b in zip(gpos, bpos)])))
        new.set("quat", _fmt(_quat_mul(bq_inv, gq)))
        try:   # Masse 0 (statischer Editor-Export) an bewegtem Koerper -> Dichte
            if float(new.get("mass", "1")) <= 0.0:
                new.attrib.pop("mass")
        except ValueError:
            pass
        body_el.append(new)
        warnings.append(f"  i Anbauteil '{g.get('name')}' an '{body_el.get('name')}' gehaengt.")


def _floats(s, default):
    return [float(x) for x in s.split()] if s else list(default)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw]


def _quat_rotate(q, v):
    w, x, y, z = _quat_mul(_quat_mul(q, [0.0, *v]), [q[0], -q[1], -q[2], -q[3]])
    return [x, y, z]


def _fmt(vals):
    return " ".join(f"{v:.6g}" for v in vals)


def _normalize_free_body(body_el) -> None:
    """Freier Koerper aus dem Scene-Editor: Pose vom Geom auf den Body ziehen.

    Der Editor exportiert bewegliche Objekte als <body> im Ursprung mit einem
    verschobenen Geom darin (<body><joint type="free"/><geom pos="x y z"/>).
    MuJoCo simuliert das korrekt, aber der Body-Ursprung (xpos) liegt dann bei
    0,0,0 -- und genau den meldet scene_state_publisher als Live-Pose an RViz
    (Box erscheint im Ursprung statt auf dem Tisch). Hier: Body-Pose = Welt-Pose
    des Geoms, Geom liegt identisch im Body. Nur fuer den einfachen Fall (ein
    Geom, keine Kind-Bodies, Orientierung nur per quat).
    """
    has_free = body_el.find("freejoint") is not None or any(
        j.get("type") == "free" for j in body_el.findall("joint"))
    geoms = body_el.findall("geom")
    if not has_free or len(geoms) != 1 or body_el.find("body") is not None:
        return
    geom = geoms[0]
    other_orient = [a for a in ORIENT_ATTRS if a != "quat"]
    if any(geom.get(a) or body_el.get(a) for a in other_orient):
        return
    b_pos = _floats(body_el.get("pos"), (0.0, 0.0, 0.0))
    b_quat = _floats(body_el.get("quat"), (1.0, 0.0, 0.0, 0.0))
    g_pos = _floats(geom.get("pos"), (0.0, 0.0, 0.0))
    g_quat = _floats(geom.get("quat"), (1.0, 0.0, 0.0, 0.0))
    rotated = _quat_rotate(b_quat, g_pos)
    body_el.set("pos", _fmt([b + r for b, r in zip(b_pos, rotated)]))
    body_el.set("quat", _fmt(_quat_mul(b_quat, g_quat)))
    geom.attrib.pop("pos", None)
    geom.attrib.pop("quat", None)


# V-HACD arbeitet auf einem Voxelgitter mit VHACD_RESOLUTION Voxeln ueber die
# Bounding-Box -- bei einem 5 m breiten Arbeitsplatz ist ein Voxel ~4 cm gross.
# Die Teil-Huellen stehen darum bis zu einem Voxel UEBER der echten Tischplatte
# und haben schraege Oberseiten (gemessen 1-2.5 Grad): abgelegte Objekte liegen
# dann auf einer Rampe und kriechen ueber die Zeit vom Tisch. Abhilfe in
# _flatten_support_plates: grosse, ebene, waagrechte Rechtecke im Original-Mesh
# (Tischplatten, Regalboeden) werden erkannt, die Huellen dort knapp oberhalb
# der Platte ausgeschnitten und die Platte selbst als exakte Box ergaenzt.
VHACD_RESOLUTION = 400000
# Version der Nachbearbeitung -- hochzaehlen, wenn sich das Verfahren aendert
# (eigener Cache-Unterordner, die teure V-HACD-Zerlegung bleibt erhalten).
FLATTEN_VERSION = "flat_v1"
PLATE_MIN_AREA = 0.05      # m^2: kleinere waagrechte Flaechen sind keine Ablage
PLATE_MIN_HEIGHT = 0.10    # m ueber Mesh-Unterkante: Fussplatten/Boden ignorieren
PLATE_BOX_THICKNESS = 0.02  # m: Dicke der exakten Kollisions-Box unter der Platte
# m: Huellen enden so weit UNTER der Platte -- sonst liegen ihre (jetzt ebenen)
# Oberseiten deckungsgleich mit der Box, die doppelten Kontakte lassen
# abgelegte Objekte zittern und langsam wandern.
PLATE_HULL_GAP = 0.002


def _decompose_cached(mesh_abs: Path):
    """Konvexe Zerlegung (V-HACD) eines Meshes, gecacht nach Datei-Inhalt,
    mit begradigten Ablageflaechen (siehe _flatten_support_plates).

    Gibt (Teil-Huellen als STL-Pfade, Ablageflaechen) zurueck; (None, []),
    wenn trimesh/vhacdx fehlen (dann bleibt das Mesh wie bisher).
    """
    digest = hashlib.sha1(mesh_abs.read_bytes()).hexdigest()[:10]
    out_dir = COLLISION_CACHE / f"{mesh_abs.stem}-{digest}"
    flat_dir = out_dir / FLATTEN_VERSION
    pieces = sorted(flat_dir.glob("part_*.stl"))
    if pieces and (flat_dir / "plates.json").is_file():
        return pieces, json.loads((flat_dir / "plates.json").read_text())
    try:
        import trimesh
    except ImportError:
        return None, []
    mesh = trimesh.load(str(mesh_abs), force="mesh")
    hulls = sorted(out_dir.glob("hull_*.stl"))
    if not hulls:
        parts = trimesh.decomposition.convex_decomposition(
            mesh, maxConvexHulls=64, resolution=VHACD_RESOLUTION, maxRecursionDepth=12)
        if isinstance(parts, dict):
            parts = [parts]
        out_dir.mkdir(parents=True, exist_ok=True)
        for i, part in enumerate(parts):
            trimesh.Trimesh(part["vertices"], part["faces"]).export(
                str(out_dir / f"hull_{i:02d}.stl"))
        hulls = sorted(out_dir.glob("hull_*.stl"))
    return _flatten_support_plates(mesh, hulls, flat_dir)


def _support_plates(mesh):
    """Ebene, waagrechte, nach oben zeigende Rechtecke im Mesh (Tischplatten,
    Regalboeden): Liste [x0, x1, y0, y1, z] im Mesh-Koordinatensystem."""
    import numpy as np
    tri = mesh.triangles
    up = (mesh.face_normals[:, 2] > 0.9999) & (np.ptp(tri[:, :, 2], axis=1) < 5e-4)
    z_of = np.round(tri[up, :, 2].mean(axis=1), 3)
    area = mesh.area_faces[up]
    z_min = mesh.bounds[0][2]
    plates = []
    for z in np.unique(z_of):
        sel = z_of == z
        a = float(area[sel].sum())
        if a < PLATE_MIN_AREA or z < z_min + PLATE_MIN_HEIGHT:
            continue
        pts = tri[up][sel].reshape(-1, 3)
        (x0, y0), (x1, y1) = pts[:, :2].min(axis=0), pts[:, :2].max(axis=0)
        # nur (fast) volle Rechtecke -- sonst wuerde der Ausschnitt ueber
        # Aussparungen hinweg echte Geometrie entfernen
        if a < 0.95 * (x1 - x0) * (y1 - y0):
            continue
        plates.append([float(x0), float(x1), float(y0), float(y1), float(pts[:, 2].max())])
    return plates


def _clip_convex(faces, n, c, eps=1e-9):
    """Konvexes Polyeder (Liste von Polygonen) auf den Halbraum n.p <= c
    beschneiden; die Schnittflaeche wird als neues Polygon ergaenzt."""
    import numpy as np
    out, cap, cut = [], [], False
    for poly in faces:
        res = []
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            da, db = float(n @ a) - c, float(n @ b) - c
            if da <= eps:
                res.append(a)
            else:
                cut = True
            if (da < -eps < eps < db) or (db < -eps < eps < da):
                res.append(a + (b - a) * (da / (da - db)))
        cap.extend(p for p in res if abs(float(n @ p) - c) <= 1e-7)
        if len(res) >= 3:
            out.append(np.array(res))
    if cut and out:
        pts = np.unique(np.round(np.array(cap), 9), axis=0) if cap else np.zeros((0, 3))
        if len(pts) >= 3:
            ctr = pts.mean(axis=0)
            u = np.cross(n, [1.0, 0.0, 0.0] if abs(n[0]) < 0.9 else [0.0, 1.0, 0.0])
            u /= np.linalg.norm(u)
            v = np.cross(n, u)
            ang = np.arctan2((pts - ctr) @ v, (pts - ctr) @ u)
            out.append(pts[np.argsort(ang)])
    return out


def _convex_mesh(faces):
    """Polygone -> (Vertices, Dreiecke nach aussen orientiert, Volumen)."""
    import numpy as np
    verts, tris = [], []
    for poly in faces:
        base = len(verts)
        verts.extend(poly)
        tris.extend([base, base + k, base + k + 1] for k in range(1, len(poly) - 1))
    verts, tris = np.array(verts), np.array(tris, dtype=int)
    inner = verts.mean(axis=0)
    a, b, c = (verts[tris[:, k]] for k in range(3))
    det = np.einsum("ij,ij->i", np.cross(b - a, c - a), a - inner)
    tris[det < 0] = tris[det < 0][:, ::-1]
    return verts, tris, float(np.abs(det).sum() / 6.0)


def _carve_plate(faces, plate, band):
    """Den Quader [x0,x1]x[y0,y1]x[z, z+band] aus einem konvexen Teil
    entfernen (unten PLATE_HULL_GAP tiefer, siehe dort). Ergebnis: bis zu 6
    konvexe Teile (darunter, darueber, 4 Seiten)."""
    import numpy as np
    x0, x1, y0, y1, z = plate
    ex, ey, ez = np.eye(3)
    clip = _clip_convex
    z -= PLATE_HULL_GAP
    band += PLATE_HULL_GAP
    mid = clip(clip(faces, -ez, -z), ez, z + band)
    inner_x = clip(clip(mid, -ex, -x0), ex, x1)
    return [clip(faces, ez, z), clip(faces, -ez, -(z + band)),
            clip(mid, ex, x0), clip(mid, -ex, -x1),
            clip(inner_x, ey, y0), clip(inner_x, -ey, -y1)]


def _flatten_support_plates(mesh, hulls, flat_dir: Path):
    """Teil-Huellen ueber Ablageflaechen ausschneiden (siehe VHACD_RESOLUTION).

    Ueber jeder erkannten Platte wird ein Band von ~2 Voxeln Hoehe aus allen
    Huellen entfernt (dort steht nur Ueberhang der Voxelisierung, keine echte
    Geometrie); die Platte selbst liefert add_collision_hulls als exakte Box.
    """
    import numpy as np
    import trimesh
    plates = _support_plates(mesh)
    voxel = (float(np.prod(mesh.extents)) / VHACD_RESOLUTION) ** (1.0 / 3.0)
    band = 2.0 * voxel + 0.01
    pieces = []
    for hull_path in hulls:
        h = trimesh.load(str(hull_path), force="mesh")
        parts = [list(h.triangles)]
        for x0, x1, y0, y1, z in plates:
            nxt = []
            for faces in parts:
                v = np.concatenate(faces)
                lo, hi = v.min(axis=0), v.max(axis=0)
                if (hi[0] <= x0 or lo[0] >= x1 or hi[1] <= y0 or lo[1] >= y1
                        or hi[2] <= z - PLATE_HULL_GAP or lo[2] >= z + band):
                    nxt.append(faces)
                    continue
                nxt.extend(f for f in _carve_plate(faces, (x0, x1, y0, y1, z), band)
                           if len(f) >= 4)
            parts = nxt
        for faces in parts:
            if len(faces) < 4:
                continue
            verts, tris, vol = _convex_mesh(faces)
            if vol > 1e-5:   # Splitter < 10 cm^3 weglassen (UDP-Grenze, siehe scene_state_publisher)
                pieces.append((verts, tris))
    flat_dir.mkdir(parents=True, exist_ok=True)
    for old in flat_dir.glob("part_*.stl"):
        old.unlink()
    for i, (verts, tris) in enumerate(pieces):
        trimesh.Trimesh(verts, tris, process=True).export(str(flat_dir / f"part_{i:03d}.stl"))
    (flat_dir / "plates.json").write_text(json.dumps(plates))
    return sorted(flat_dir.glob("part_*.stl")), plates


def add_collision_hulls(geom_el, mesh_abs: Path, mesh_scale, asset, wb, warnings) -> None:
    """Statisches Mesh-Hindernis: Kollision ueber konvexe Teil-Huellen.

    MuJoCo kollidiert ein Mesh-Geom nur mit seiner EINEN konvexen Huelle. Bei
    einem Arbeitsplatz/Regal "fuellt" die alles auf: Objekte auf dem Tisch
    starten IN der Huelle und werden weggeschossen, auf der Tischplatte liegen
    bleibt nichts. Darum: das Original-Mesh nur noch als Optik (contype/
    conaffinity 0), die Kollision uebernehmen die Teile der Zerlegung (group 3
    = im Viewer standardmaessig ausgeblendet) in einem Body mit derselben Pose.
    Ablageflaechen (Tischplatten, Regalboeden) kommen als exakte, waagrechte
    Boxen dazu, damit abgelegte Objekte nicht auf schraegen Huellen-Flaechen
    liegen und langsam herunterrutschen.
    """
    hulls, plates = _decompose_cached(mesh_abs)
    if not hulls:
        warnings.append(f"  ! Mesh '{geom_el.get('mesh')}': keine Kollisions-Zerlegung "
                        "(trimesh fehlt) -> Kollision nur ueber konvexe Huelle.")
        return

    geom_el.set("contype", "0")
    geom_el.set("conaffinity", "0")

    base = geom_el.get("name") or geom_el.get("mesh")
    body = ET.SubElement(wb, "body", {"name": f"{base}{COLLISION_BODY_SUFFIX}",
                                      "pos": geom_el.get("pos", "0 0 0")})
    for attr in ORIENT_ATTRS:
        if geom_el.get(attr):
            body.set(attr, geom_el.get(attr))
    contact = {attr: geom_el.get(attr) for attr in CONTACT_ATTRS if geom_el.get(attr)}
    for i, hull in enumerate(hulls):
        mesh_name = f"{geom_el.get('mesh')}_hull{i:02d}"
        mesh_attrs = {"name": mesh_name, "file": rel_to_meshdir(hull)}
        if mesh_scale:
            mesh_attrs["scale"] = mesh_scale
        ET.SubElement(asset, "mesh", mesh_attrs)
        ET.SubElement(body, "geom", {"type": "mesh", "mesh": mesh_name, "group": "3",
                                     "rgba": "0.9 0.3 0.3 0.4", **contact})
    sx, sy, sz = _floats(mesh_scale, (1.0, 1.0, 1.0))
    t = PLATE_BOX_THICKNESS
    for k, (x0, x1, y0, y1, z) in enumerate(plates):
        ET.SubElement(body, "geom", {
            "name": f"{base}__platte{k}", "type": "box", "group": "3",
            "rgba": "0.3 0.9 0.3 0.4",
            "pos": _fmt((sx * (x0 + x1) / 2, sy * (y0 + y1) / 2, sz * z - t / 2)),
            "size": _fmt((abs(sx * (x1 - x0)) / 2, abs(sy * (y1 - y0)) / 2, t / 2)),
            **contact})
    warnings.append(f"  i Mesh '{geom_el.get('mesh')}': Kollision ueber {len(hulls)} "
                    f"konvexe Teile + {len(plates)} Ablageflaeche(n) "
                    "(Viewer: Gruppe 3 einblenden zum Ansehen).")


# ── Greif-Objekte aus Netzen (z. B. KLT-Kisten) ──────────────────────────
# Auch ein Greif-Objekt aus EINEM Netz-Geom kollidiert in MuJoCo nur mit seiner
# konvexen Huelle: eine KLT-Kiste ist dann ein voller Block, Innenraum und
# Grifftaschen sind zu. Darum bekommt jedes nicht-konvexe Greif-Netz eine
# Kollision aus Quadern (siehe _grasp_boxes):
#   * Gitter mit Zellen bis GRASP_VOXEL, dessen Ebenen genau auf den
#     achsparallelen Wandflaechen des Netzes liegen (_wall_planes, ab
#     GRASP_PLANE_MIN_AREA). Punkte einer solchen Wand kommen in die Zelle auf
#     der Materialseite (_material_side, per Strahl, weil die Wicklung der
#     Netze nicht stimmt). Gerade Waende und Taschen sind dadurch exakt: die
#     1-mm-Stirnwand bleibt 1 mm, die 17-mm-Grifftasche 17 mm.
#   * Oberflaeche ins Gitter legen, rundum geschlossene Hohlraeume fuellen,
#     Spalten bis 2*GRASP_CLOSE_VOXELS Zellen schliessen (Treppenstufen
#     schraeger Flaechen), belegte Zellen gierig zu moeglichst grossen
#     achsparallelen Quadern zusammenfassen.
#   * Die unterste Lage wird als eine flache Auflage zusammengefasst
#     (Loecher in der Grundflaeche gefuellt): auf dem Tisch beruehren nur ~3
#     Quader die Platte statt ~20 -- jeder beruehrende Quader kostet MuJoCo pro
#     Schritt eine Box-Box-Rechnung.
#   * Warum keine V-HACD wie bei den festen Hindernissen? Gemessen an der
#     kleinen KLT (Stirnwand 1 mm, Grifftasche 14 mm): V-HACD verliert selbst
#     mit 4 Mio. Voxeln Teile der 1-mm-Stirnwand (Hand faehrt durch die Wand in
#     die Kiste), CoACD haelt die Wand, braucht aber ~190 s und macht die Tasche
#     mit schraegen Keilen enger. Ein reines 3-mm-Raster (grasp_boxes_v1)
#     machte die Tasche 2 mm enger -- zu eng fuer die 15-mm-Haken-Lippe, die
#     gedreht von unten in die echte 17-mm-Tasche faehrt. Bau ~2-3 s je Kiste,
#     danach aus dem Cache.
#   * Rechenzeit (A5, zwei KLT, 4-Kern-Cloud-Rechner, je Sim-Schritt): reines
#     3-mm-Raster 377 Quader 0,35 ms; Wand-Gitter 3 mm 652 Quader 0,50 ms,
#     4 mm 468 Quader 0,38 ms. 4 mm reicht, weil die Wandebenen die Genauigkeit
#     liefern; nur schraege/runde Flaechen werden zur Treppe (Stufe <= 4 mm).
#   * Grenze: Spalten unter 2 Zellen gehen zu; konvexe Netze (z. B.
#     sample_ramp) brauchen nichts und bleiben wie bisher.
# Die Quader haengen im freien Koerper des Objekts (Geom-Namen
# "<objekt>__hull000" ..., Gruppe 3 = im Viewer ausgeblendet) und tragen KEINE
# Masse; das Netz bleibt Optik (contype/conaffinity 0) und traegt wie bisher
# Masse und Traegheit. scene_objects.py erkennt die Teile am Namen.
GRASP_HULL_INFIX = "__hull"
GRASP_VOXEL = 0.004           # m: groesste Zellkante (schraege Flaechen)
GRASP_CLOSE_VOXELS = 1        # Spalten bis 2*N Voxel werden geschlossen
GRASP_MAX_CELLS = 8_000_000   # groessere Objekte -> Voxel wird groesser
GRASP_PLANE_MIN_AREA = 1e-3   # m^2: achsparallele Wand ab 10 cm^2 -> Gitterebene
GRASP_PLANE_MERGE = 3e-4      # m: Ebenen naeher als 0,3 mm = eine Wand
GRASP_WALL_MAX = 0.006        # m: dickere Wand -> Seite per Gegenflaeche unsicher
# Version des Verfahrens -- hochzaehlen, wenn sich etwas an _grasp_boxes
# aendert (eigener Cache-Unterordner, alte Ergebnisse werden nicht benutzt).
GRASP_DECOMP_VERSION = "grasp_boxes_v2"


def _surface_points(vertices, faces, step):
    """Punkte auf allen Dreiecken, Abstand <= step (baryzentrisches Raster,
    je Dreieck so fein wie seine laengste Kante es verlangt)."""
    import numpy as np
    tri = vertices[faces]                                     # (F,3,3)
    edge = np.max(np.linalg.norm(tri - np.roll(tri, 1, axis=1), axis=2), axis=1)
    n_div = np.maximum(1, np.ceil(edge / step)).astype(int)
    out = [vertices]
    for n in np.unique(n_div):
        sel = tri[n_div == n]
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
        keep = (i + j) <= n
        a, b = i[keep] / n, j[keep] / n                       # (K,)
        w = np.stack([1.0 - a - b, a, b], axis=1)             # (K,3)
        out.append(np.einsum("kv,fvd->fkd", w, sel).reshape(-1, 3))
    return np.concatenate(out)


def _fill_enclosed(occ):
    """Rundum geschlossene Hohlraeume eines Voxelgitters (beliebige Dimension)
    fuellen, wie scipy.ndimage.binary_fill_holes -- scipy ist im Editor-venv
    nicht sicher da. Offene Raeume (Kisten-Innenraum, Grifftaschen) bleiben
    frei."""
    import numpy as np
    free = ~occ
    outside = np.zeros_like(occ)
    for ax in range(occ.ndim):
        for end in (0, -1):
            sl = [slice(None)] * occ.ndim
            sl[ax] = end
            outside[tuple(sl)] = free[tuple(sl)]
    while True:
        grown = outside | _shift_or(outside)
        grown &= free
        if np.array_equal(grown, outside):
            return ~outside
        outside = grown


def _shift_or(a):
    """Jede Zelle ODER ihre 2*ndim direkten Nachbarn (Kreuz-Nachbarschaft)."""
    out = a.copy()
    for ax in range(a.ndim):
        lo = [slice(None)] * a.ndim
        hi = [slice(None)] * a.ndim
        lo[ax], hi[ax] = slice(1, None), slice(None, -1)
        out[tuple(lo)] |= a[tuple(hi)]
        out[tuple(hi)] |= a[tuple(lo)]
    return out


def _close(occ, r):
    """Morphologisches Schliessen mit einem Wuerfel (2r+1)^3: fuellt Spalten
    bis 2r Voxel Breite, breitere Oeffnungen behalten ihre Form."""
    import numpy as np
    if r <= 0:
        return occ.copy()
    pad = r + 1
    a = np.pad(occ, pad)

    def sweep(x, op):
        # Wuerfel = Linie je Achse nacheinander (separierbar)
        for ax in range(x.ndim):
            y = x.copy()
            for k in range(1, r + 1):
                lo = [slice(None)] * x.ndim
                hi = [slice(None)] * x.ndim
                lo[ax], hi[ax] = slice(k, None), slice(None, -k)
                if op == "or":
                    y[tuple(lo)] |= x[tuple(hi)]
                    y[tuple(hi)] |= x[tuple(lo)]
                else:
                    y[tuple(lo)] &= x[tuple(hi)]
                    y[tuple(hi)] &= x[tuple(lo)]
                    # Rand: ausserhalb gilt als frei
                    edge_lo = [slice(None)] * x.ndim
                    edge_hi = [slice(None)] * x.ndim
                    edge_lo[ax], edge_hi[ax] = slice(None, k), slice(-k, None)
                    y[tuple(edge_lo)] = False
                    y[tuple(edge_hi)] = False
            x = y
        return x

    closed = sweep(sweep(a, "or"), "and")
    return closed[(slice(pad, -pad),) * occ.ndim] | occ


def _greedy_boxes(occ):
    """Belegte Voxel gierig zu Quadern zusammenfassen. Jeder Quader waechst
    von seinem Startvoxel aus in alle sechs Richtungen, solange die neue
    Schicht voll belegt ist; Quader duerfen sich ueberlappen (weniger Teile).
    Probiert alle Achsen-Reihenfolgen, nimmt die mit den wenigsten Quadern.
    Ergebnis: Liste (lo, hi) in Voxel-Indizes (hi exklusiv)."""
    import itertools
    import numpy as np
    best = None
    for order in itertools.permutations(range(occ.ndim)):
        occ_t = np.transpose(occ, order)
        todo = occ_t.copy()
        n0, n1, n2 = occ_t.shape
        boxes = []
        for i, j, k in np.argwhere(occ_t):
            if not todo[i, j, k]:
                continue
            i2 = i + 1
            while i2 < n0 and occ_t[i2, j, k]:
                i2 += 1
            j2 = j + 1
            while j2 < n1 and occ_t[i:i2, j2, k].all():
                j2 += 1
            k2 = k + 1
            while k2 < n2 and occ_t[i:i2, j:j2, k2].all():
                k2 += 1
            i0, j0, k0 = i, j, k
            while i0 > 0 and occ_t[i0 - 1, j:j2, k:k2].all():
                i0 -= 1
            while j0 > 0 and occ_t[i0:i2, j0 - 1, k:k2].all():
                j0 -= 1
            while k0 > 0 and occ_t[i0:i2, j0:j2, k0 - 1].all():
                k0 -= 1
            todo[i0:i2, j0:j2, k0:k2] = False
            lo, hi = [0, 0, 0], [0, 0, 0]
            for t, ax in enumerate(order):
                lo[ax], hi[ax] = (i0, j0, k0)[t], (i2, j2, k2)[t]
            boxes.append((lo, hi))
        if best is None or len(boxes) < len(best):
            best = boxes
    return best


def _wall_planes(mesh, axis, min_area=GRASP_PLANE_MIN_AREA, merge=GRASP_PLANE_MERGE):
    """Achsparallele Flaechen eines Netzes entlang einer Achse: Liste
    (Koordinate, Flaechen-Indizes) fuer jede Ebene mit mindestens min_area
    Flaeche. Ebenen naeher als merge werden zusammengefasst (flaechengewichtet)."""
    import numpy as np
    n = mesh.face_normals[:, axis]
    sel = np.nonzero(np.abs(n) > 0.999)[0]
    if not len(sel):
        return []
    coord = mesh.triangles_center[sel, axis]
    area = mesh.area_faces[sel]
    order = np.argsort(coord)
    groups, cur = [], [order[0]]
    for a, b in zip(order[:-1], order[1:]):
        if coord[b] - coord[a] > merge:
            groups.append(cur)
            cur = []
        cur.append(b)
    groups.append(cur)
    out = []
    for g in groups:
        g = np.array(g)
        if area[g].sum() >= min_area:
            out.append((float(np.average(coord[g], weights=area[g])), sel[g]))
    return out


def _material_side(mesh, faces, axis, max_wall=GRASP_WALL_MAX):
    """Auf welcher Seite (+1/-1 entlang axis) liegt bei achsparallelen Flaechen
    das Material? Die Wicklung der Netze ist nicht verlaesslich, darum per
    Strahl: Material ist die Seite, auf der die Gegenflaeche der Wand innerhalb
    max_wall liegt (duennwandige Teile). Eine Flaeche ohne nahe Gegenflaeche,
    die zu einer Seite ins Freie zeigt, ist eine Haut ohne Dicke (KLT-Schuerze):
    Material dann zur freien Seite. 0 = unklar (beide Nachbarzellen)."""
    import numpy as np
    c = mesh.triangles_center[faces]
    side = np.zeros(len(faces), dtype=int)
    dist = {}
    for sg in (1, -1):
        d = np.zeros(3)
        d[axis] = sg
        org = c + d * 1e-5
        loc, ray, _ = mesh.ray.intersects_location(org, np.tile(d, (len(c), 1)),
                                                   multiple_hits=True)
        best = np.full(len(c), np.inf)
        if len(ray):
            t = (loc[:, axis] - org[ray, axis]) * sg
            ok = t > 1e-5
            np.minimum.at(best, ray[ok], t[ok])
        dist[sg] = best
    near = np.minimum(dist[1], dist[-1]) <= max_wall
    side[near & (dist[1] < dist[-1])] = 1
    side[near & (dist[-1] < dist[1])] = -1
    # Einzelne Flaeche ohne Gegenflaeche (z. B. Schuerze als duenne Haut):
    # Material zur offenen Seite hin, damit die Tasche davor frei bleibt
    side[~near & np.isinf(dist[1]) & np.isfinite(dist[-1])] = 1
    side[~near & np.isinf(dist[-1]) & np.isfinite(dist[1])] = -1
    return side


def _grasp_boxes(mesh, voxel=GRASP_VOXEL, close=GRASP_CLOSE_VOXELS):
    """Quader-Zerlegung eines Netzes (siehe GRASP_HULL_INFIX).

    Gitter: je Achse ein Raster von etwa voxel Kantenlaenge, aber die
    Gitterebenen liegen genau auf den achsparallelen Wandflaechen des Netzes
    (Raster-Linien naeher als voxel/2 an einer Wand fallen weg). Achsparallele
    Waende, Taschen und Aussenmasse stimmen dadurch exakt (1-mm-Wand bleibt
    1 mm, Tasche behaelt ihre Breite); nur schraege/runde Flaechen werden zur
    Treppe mit Stufen bis ~1,5 voxel.
    Rueckgabe: Liste [cx, cy, cz, hx, hy, hz] in Netz-Koordinaten.
    """
    import numpy as np
    lo = mesh.bounds[0].astype(float)
    hi = mesh.bounds[1].astype(float)
    ext = np.maximum(hi - lo, 1e-6)
    while True:
        n = np.maximum(1, np.round(ext / voxel)).astype(int)
        if int(np.prod(n)) <= GRASP_MAX_CELLS:
            break
        voxel *= 1.25
    verts = np.asarray(mesh.vertices, float)
    faces = np.asarray(mesh.faces)
    edges, snap = [], []          # snap: je Achse (Flaechen, Ebene, Seite)
    for ax in range(3):
        planes = _wall_planes(mesh, ax)
        e = np.unique(np.clip([lo[ax], hi[ax]] + [p for p, _ in planes], lo[ax], hi[ax]))
        e = e[np.concatenate([[True], np.diff(e) > 1e-4])]    # keine Null-Zellen
        e[-1] = hi[ax]
        # jedes Stueck zwischen zwei Ebenen gleichmaessig auf <= voxel teilen
        e = np.concatenate([np.linspace(a, b, max(1, int(np.ceil((b - a) / voxel - 1e-9))) + 1)[:-1]
                            for a, b in zip(e[:-1], e[1:])] + [e[-1:]])
        edges.append(e)
        sn = []
        for p, fidx in planes:
            sn.append((fidx, p, _material_side(mesh, fidx, ax)))
        snap.append(sn)
    shape = tuple(len(e) - 1 for e in edges)
    step = 0.5 * min(float(np.diff(e).min()) for e in edges)
    step = max(step, 0.25 * voxel)
    # Punkte je Dreieck, leicht zur Dreiecksmitte gezogen (Kantenpunkte liegen
    # sonst genau auf Nachbar-Ebenen und landen in der falschen Zelle)
    tri = verts[faces]
    cen = tri.mean(axis=1, keepdims=True)
    tri = cen + (tri - cen) * (1.0 - 1e-4)
    cells = np.zeros(shape, dtype=bool)
    fid_all = np.arange(len(faces))
    edge_len = np.max(np.linalg.norm(tri - np.roll(tri, 1, axis=1), axis=2), axis=1)
    n_div_f = np.maximum(1, np.ceil(edge_len / step)).astype(int)
    # Achsparallele Wandflaechen: Koordinate genau auf die Ebene, dann ein
    # Hauch zur Materialseite (unklar: in beide Nachbarzellen)
    fix = [dict() for _ in range(3)]
    for ax in range(3):
        for fidx, p, side in snap[ax]:
            for f, sg in zip(fidx, side):
                fix[ax][int(f)] = (p, int(sg))
    for nd in np.unique(n_div_f):
        fsel = fid_all[n_div_f == nd]
        i, j = np.meshgrid(np.arange(nd + 1), np.arange(nd + 1), indexing="ij")
        keep = (i + j) <= nd
        a, b = i[keep] / nd, j[keep] / nd
        w = np.stack([1.0 - a - b, a, b], axis=1)
        pts = np.einsum("kv,fvd->fkd", w, tri[fsel])           # (F,K,3)
        fids = fsel
        for ax in range(3):
            rows, plane, sgn = [], [], []
            for r, f in enumerate(fids):
                hit = fix[ax].get(int(f))
                if hit is not None:
                    rows.append(r)
                    plane.append(hit[0])
                    sgn.append(hit[1])
            if not rows:
                continue
            rows, plane, sgn = np.array(rows), np.array(plane), np.array(sgn)
            pts[rows, :, ax] = (plane + np.where(sgn == 0, 1, sgn) * 1e-7)[:, None]
            dup = rows[sgn == 0]
            if len(dup):
                extra = pts[dup].copy()
                extra[:, :, ax] -= 2e-7
                pts = np.concatenate([pts, extra])
                fids = np.concatenate([fids, fids[dup]])
        variants = [pts]
        for P in variants:
            P = P.reshape(-1, 3)
            idx = [np.clip(np.searchsorted(edges[ax], P[:, ax], side="right") - 1,
                           0, shape[ax] - 1) for ax in range(3)]
            cells[idx[0], idx[1], idx[2]] = True
    occ = _close(_fill_enclosed(cells), close)
    # Auflage: unterste Lage fuer sich (Loecher gefuellt), Rest darueber
    base = np.zeros_like(occ)
    base[:, :, 0] = _fill_enclosed(occ[:, :, 0])
    upper = occ.copy()
    upper[:, :, 0] = False
    out = []
    for part in (base, upper):
        for b_lo, b_hi in _greedy_boxes(part):
            p0 = np.array([edges[ax][b_lo[ax]] for ax in range(3)])
            p1 = np.array([edges[ax][b_hi[ax]] for ax in range(3)])
            out.append([float(v) for v in (*((p0 + p1) / 2), *((p1 - p0) / 2))])
    return out


def _grasp_boxes_cached(mesh_abs: Path):
    """Quader-Zerlegung eines Greif-Netzes, gecacht nach Datei-Inhalt (wie
    _decompose_cached). Rueckgabe: Liste der Quader, [] wenn das Netz schon
    konvex ist (dann reicht MuJoCos Huelle), None ohne trimesh."""
    digest = hashlib.sha1(mesh_abs.read_bytes()).hexdigest()[:10]
    cache = COLLISION_CACHE / f"{mesh_abs.stem}-{digest}" / GRASP_DECOMP_VERSION / "boxes.json"
    if cache.is_file():
        try:
            return json.loads(cache.read_text())
        except (OSError, ValueError):
            pass
    try:
        import trimesh
    except ImportError:
        return None
    mesh = trimesh.load(str(mesh_abs), force="mesh")
    boxes = [] if (mesh.is_watertight and mesh.is_convex) else _grasp_boxes(mesh)
    cache.parent.mkdir(parents=True, exist_ok=True)
    tmp = cache.with_suffix(".tmp")
    tmp.write_text(json.dumps(boxes))
    tmp.replace(cache)
    return boxes


def add_grasp_collision(body_el, geom_el, mesh_abs: Path, mesh_scale, warnings) -> None:
    """Greif-Objekt aus einem Netz: Kollision ueber Quader im selben freien
    Koerper, das Netz selbst wird reine Optik und behaelt Masse/Traegheit."""
    boxes = _grasp_boxes_cached(mesh_abs)
    name = geom_el.get("name")
    if boxes is None:
        warnings.append(f"  ! Greif-Objekt '{name}': keine Kollisions-Zerlegung "
                        "(trimesh fehlt) -> Kollision nur ueber konvexe Huelle.")
        return
    if not boxes:
        return   # Netz ist konvex: MuJoCos Huelle ist exakt
    geom_el.set("contype", "0")
    geom_el.set("conaffinity", "0")
    scale = _floats(mesh_scale, (1.0, 1.0, 1.0))
    sx, sy, sz = scale * 3 if len(scale) == 1 else scale[:3]
    contact = {attr: geom_el.get(attr) for attr in CONTACT_ATTRS if geom_el.get(attr)}
    for i, (cx, cy, cz, hx, hy, hz) in enumerate(boxes):
        ET.SubElement(body_el, "geom", {
            "name": f"{name}{GRASP_HULL_INFIX}{i:03d}", "type": "box", "group": "3",
            "mass": "0", "rgba": "0.9 0.3 0.3 0.4",
            "pos": _fmt((sx * cx, sy * cy, sz * cz)),
            "size": _fmt((abs(sx * hx), abs(sy * hy), abs(sz * hz))),
            **contact})
    warnings.append(f"  i Greif-Objekt '{name}': Kollision ueber {len(boxes)} Quader "
                    "(Innenraum/Griffe offen; Viewer: Gruppe 3 einblenden).")


def merge_environment(env_root, asset, wb, env_dir, warnings):
    """Mischt NUR die Objekte der Umgebung in die Basis ein.

    - <asset>: eigene Meshes/Texturen/Materialien der Objekte (Mesh-Pfade
      werden umgeschrieben). Doppelte Namen und ein zweiter Skybox werden
      uebersprungen.
    - <worldbody>: alle Objekte (geoms/bodies). Ein evtl. mitgespeicherter
      Boden (<geom type="plane">) und Lichtquellen werden weggelassen - die
      kommen aus der Basis.
    """
    used_asset_names = {el.get("name") for el in asset if el.get("name")}
    env_meshes = {}  # Mesh-Name -> (absoluter Pfad, scale) fuer add_collision_hulls
    n_base = len(wb)  # alles danach in <worldbody> stammt aus der Umgebung

    for env_asset in env_root.findall("asset"):
        for el in list(env_asset):
            if el.tag == "texture" and el.get("type") == "skybox":
                continue  # nur ein Skybox erlaubt (Basis hat schon einen)
            nm = el.get("name")
            if nm and nm in used_asset_names:
                warnings.append(f"  ! Asset-Name '{nm}' schon in der Basis -> uebersprungen")
                continue
            if el.tag == "mesh" and el.get("file"):
                p = Path(el.get("file"))
                mesh_abs = p if p.is_absolute() else (env_dir / p).resolve()
                if not mesh_abs.is_file():
                    warnings.append(f"  ! Mesh nicht gefunden: {el.get('file')}  ({mesh_abs})")
                else:
                    try:
                        mesh_abs.relative_to(MJ_ROOT)
                    except ValueError:
                        warnings.append(
                            "  ! Mesh liegt ausserhalb von unitree_mujoco/ (im Docker-"
                            f"Container evtl. nicht sichtbar): {mesh_abs}")
                    if nm:
                        env_meshes[nm] = (mesh_abs, el.get("scale"))
                el.set("file", rel_to_meshdir(mesh_abs))
            asset.append(el)
            if nm:
                used_asset_names.add(nm)

    # Anbauteile einsammeln (gehoeren in den Koerper ihres Greif-Objekts)
    attach = {}
    for env_wb in env_root.findall("worldbody"):
        for el in list(env_wb):
            nm = el.get("name") or ""
            if el.tag == "geom" and ATTACH_SEP in nm and GRASP_PREFIX_RE.match(nm):
                attach.setdefault(nm.split(ATTACH_SEP)[0], []).append(el)
                env_wb.remove(el)

    for env_wb in env_root.findall("worldbody"):
        for el in list(env_wb):
            if el.tag == "light":
                continue  # Licht kommt aus der Basis
            if el.tag == "geom" and el.get("type") == "plane":
                continue  # Boden kommt aus der Basis
            if _is_grasp_geom(el):
                body = _wrap_grasp_geom(el)
                _attach_parts(body, attach.pop(el.get("name"), []), warnings)
                if el.get("type") == "mesh" and el.get("mesh") in env_meshes:
                    mesh_abs, mesh_scale = env_meshes[el.get("mesh")]
                    add_grasp_collision(body, body.find("geom"), mesh_abs, mesh_scale, warnings)
                wb.append(body)
                continue
            if el.tag == "body":
                _normalize_free_body(el)
            wb.append(el)
            if el.tag == "geom" and el.get("type") == "mesh" and el.get("mesh") in env_meshes:
                mesh_abs, mesh_scale = env_meshes[el.get("mesh")]
                add_collision_hulls(el, mesh_abs, mesh_scale, asset, wb, warnings)

    for parent, parts in attach.items():
        warnings.append(f"  ! Anbauteile ohne Greif-Objekt '{parent}' ignoriert: "
                        + ", ".join(g.get("name") for g in parts))

    # Kollisions-Bits der Umgebung: Koerper-Geoms des G1 kollidieren auf Bit 1,
    # die Haende/Finger (Inspire) nur auf Bit 2 (damit sie nicht am eigenen
    # Koerper haengen bleiben). Umgebungs-Objekte bekommen darum BEIDE Bits (3),
    # sonst greifen die Haende durch Box und Tisch hindurch. Explizit gesetzte
    # Werte (z.B. contype=0 = reine Optik) bleiben unangetastet.
    for el in list(wb)[n_base:]:
        for g in el.iter("geom"):
            if g.get("contype") is None and g.get("conaffinity") is None:
                g.set("contype", ENV_COLLISION_BITS)
                g.set("conaffinity", ENV_COLLISION_BITS)

    # Auf Abschnitte hinweisen, die eine reine Objekt-Umgebung normalerweise
    # nicht enthalten sollte (werden bewusst NICHT uebernommen).
    for tag in ("equality", "actuator", "default", "contact", "tendon", "sensor"):
        if env_root.find(tag) is not None:
            warnings.append(f"  ! <{tag}> in der Umgebung wird ignoriert "
                            "(Umgebungen sollen nur Objekte enthalten).")


def _reexec_in_editor_venv() -> None:
    """start.sh ruft das Skript mit dem System-python3 auf, dem trimesh/vhacdx
    (fuer die Kollisions-Zerlegung) meist fehlen. Dann einmalig mit dem Python
    aus scene_editor/.venv neu starten, falls vorhanden."""
    try:
        import trimesh  # noqa: F401
        return
    except ImportError:
        pass
    venv_py = EDITOR_VENV / "bin" / "python"
    if venv_py.is_file() and Path(sys.prefix).resolve() != EDITOR_VENV.resolve():
        os.execv(str(venv_py), [str(venv_py), str(Path(__file__).resolve()), *sys.argv[1:]])


def _marker_xy_yaw(el):
    """Position + Blickrichtung (Drehung um z) einer Markierung (geom oder body)."""
    if el.tag == "body":
        _normalize_free_body(el)
        pos = _floats(el.get("pos"), (0.0, 0.0, 0.0))
        quat = _floats(el.get("quat"), (1.0, 0.0, 0.0, 0.0))
        inner = el.find("geom")
        if inner is not None and inner.get("pos"):
            pos = [a + b for a, b in zip(pos, _quat_rotate(quat, _floats(inner.get("pos"), (0, 0, 0))))]
    else:
        pos = _floats(el.get("pos"), (0.0, 0.0, 0.0))
        quat = _floats(el.get("quat"), (1.0, 0.0, 0.0, 0.0))
    w, x, y, z = quat
    return pos, math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def extract_stations(env_root, warnings):
    """Stations-Markierungen (station_...) aus der Umgebung entfernen und als
    Liste [(name, x, y, yaw)] zurueckgeben (Reihenfolge wie in der Datei)."""
    out = []
    for env_wb in env_root.findall("worldbody"):
        for el in list(env_wb):
            names = [el.get("name") or ""] + [g.get("name") or "" for g in el.findall("geom")]
            name = next((n for n in names if STATION_PREFIX_RE.match(n)), None)
            if el.tag not in ("geom", "body") or name is None:
                continue
            env_wb.remove(el)
            pos, yaw = _marker_xy_yaw(el)
            out.append((name, pos[0], pos[1], yaw))
            warnings.append(f"  i Station '{name}': x={pos[0]:.2f} y={pos[1]:.2f}, "
                            f"Blickrichtung {math.degrees(yaw):.0f} Grad.")
    return out


def extract_spawn(env_root, warnings):
    """Startpunkt-Markierung (g1_spawn...) aus der Umgebung entfernen und
    (x, y, yaw) zurueckgeben -- oder None, wenn keine da ist (Start im Ursprung)."""
    for env_wb in env_root.findall("worldbody"):
        for el in list(env_wb):
            names = [el.get("name") or ""] + [g.get("name") or "" for g in el.findall("geom")]
            if el.tag not in ("geom", "body") or not any(SPAWN_PREFIX_RE.match(n) for n in names):
                continue
            env_wb.remove(el)
            pos, yaw = _marker_xy_yaw(el)
            warnings.append(f"  i Startpunkt '{names[0] or names[-1]}': G1 startet bei "
                            f"x={pos[0]:.2f} y={pos[1]:.2f}, Blickrichtung {math.degrees(yaw):.0f} Grad.")
            return pos[0], pos[1], yaw
    return None


def build_scene(env_path: Path, robot_file: str, out_path: Path) -> list:
    """Basis + Umgebung -> out_path schreiben. Gibt die Hinweise zurueck."""
    env_dir = env_path.parent
    # Umgebung einlesen (Kommentare vorher strippen: MuJoCos Parser ist tolerant,
    # ElementTree strikt - z.B. sind '--'-Folgen in XML-Kommentaren ungueltig).
    raw = env_path.read_text(encoding="utf-8")
    cleaned = re.sub(r"<!--.*?-->", "", raw, flags=re.DOTALL)
    env_root = ET.fromstring(cleaned)

    warnings = []
    mj, asset, wb = build_base(robot_file, f"g1_env_{env_path.stem}")
    spawn = extract_spawn(env_root, warnings)
    stations = extract_stations(env_root, warnings)
    merge_environment(env_root, asset, wb, env_dir, warnings)
    if spawn is not None or stations:
        custom = ET.SubElement(mj, "custom")
        if spawn is not None:
            ET.SubElement(custom, "numeric", {"name": "g1_spawn", "data": _fmt(spawn)})
        for name, x, y, yaw in stations:
            ET.SubElement(custom, "numeric", {"name": name, "data": _fmt((x, y, yaw))})

    ET.indent(mj, space="  ")
    header = (
        "<!-- AUTO-GENERIERT von scene_editor/build_env_scene.py.\n"
        f"     Basis (G1 + Licht + Boden + Weld) + Umgebung: {env_path.name}\n"
        f"     Roboter: {robot_file}\n"
        "     NICHT von Hand editieren - wird bei jeder Umgebungs-Auswahl neu erzeugt. -->\n"
    )
    out_path.write_text(header + ET.tostring(mj, encoding="unicode") + "\n", encoding="utf-8")
    return warnings


def main() -> None:
    _reexec_in_editor_venv()
    ap = argparse.ArgumentParser(description="G1-Basis + Umgebung zu lauffaehiger Szene kombinieren")
    ap.add_argument("--env", required=True,
                    help="Umgebungs-XML (nur Objekte), z.B. scenes/warehouse.xml")
    ap.add_argument("--inspire", default="0", choices=["0", "1"],
                    help="0 = Rubber-Hand-G1, 1 = Inspire-FTP-G1 (Default 0)")
    ap.add_argument("--hands", default="",
                    help="Hand-Profil (inspire | rubber | Ordner unter unitree_robots/g1/hands/, "
                         "z. B. haken). Gewinnt vor --inspire, leer = --inspire entscheidet.")
    ap.add_argument("--out",
                    help="Zieldatei (Default: unitree_robots/g1/scene_env_<name>.xml)")
    args = ap.parse_args()

    env_path = Path(args.env)
    if not env_path.is_absolute():
        env_path = (Path.cwd() / env_path).resolve()
    if not env_path.is_file():
        sys.exit(f"[build_env_scene] Umgebung nicht gefunden: {env_path}")

    hands = args.hands.strip().lower()
    if hands in ("inspire", "rubber"):
        robot_file = ROBOT_FILES["1" if hands == "inspire" else "0"]
    elif hands:
        robot_file = f"g1_29dof_hand_{hands}.xml"   # tools/gen_hand_profile.py
    else:
        robot_file = ROBOT_FILES[args.inspire]
    if not (G1_DIR / robot_file).is_file():
        sys.exit(f"[build_env_scene] Robotermodell fehlt: {G1_DIR / robot_file}")

    name = env_path.stem
    out_path = Path(args.out).resolve() if args.out else (G1_DIR / f"scene_env_{name}.xml")
    for w in build_scene(env_path, robot_file, out_path):
        print(w, file=sys.stderr)
    print(out_path)


if __name__ == "__main__":
    main()

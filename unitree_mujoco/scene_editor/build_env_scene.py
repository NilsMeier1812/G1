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

Hindernis vs. Greif-Objekt (fuer Nav/IK, siehe g1pilot/docs/51_navigation_technik.md):
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
"""
import argparse
import hashlib
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
    return el.tag == "geom" and bool(GRASP_PREFIX_RE.match(el.get("name") or ""))


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


def _decompose_cached(mesh_abs: Path):
    """Konvexe Zerlegung (V-HACD) eines Meshes, gecacht nach Datei-Inhalt.

    Gibt die Liste der Teil-Huellen (STL-Pfade) zurueck; None, wenn trimesh/
    vhacdx fehlen (dann bleibt das Mesh wie bisher).
    """
    digest = hashlib.sha1(mesh_abs.read_bytes()).hexdigest()[:10]
    out_dir = COLLISION_CACHE / f"{mesh_abs.stem}-{digest}"
    hulls = sorted(out_dir.glob("hull_*.stl"))
    if hulls:
        return hulls
    try:
        import trimesh
    except ImportError:
        return None
    mesh = trimesh.load(str(mesh_abs), force="mesh")
    parts = trimesh.decomposition.convex_decomposition(
        mesh, maxConvexHulls=64, resolution=400000, maxRecursionDepth=12)
    if isinstance(parts, dict):
        parts = [parts]
    out_dir.mkdir(parents=True, exist_ok=True)
    for i, part in enumerate(parts):
        trimesh.Trimesh(part["vertices"], part["faces"]).export(
            str(out_dir / f"hull_{i:02d}.stl"))
    return sorted(out_dir.glob("hull_*.stl"))


def add_collision_hulls(geom_el, mesh_abs: Path, mesh_scale, asset, wb, warnings) -> None:
    """Statisches Mesh-Hindernis: Kollision ueber konvexe Teil-Huellen.

    MuJoCo kollidiert ein Mesh-Geom nur mit seiner EINEN konvexen Huelle. Bei
    einem Arbeitsplatz/Regal "fuellt" die alles auf: Objekte auf dem Tisch
    starten IN der Huelle und werden weggeschossen, auf der Tischplatte liegen
    bleibt nichts. Darum: das Original-Mesh nur noch als Optik (contype/
    conaffinity 0), die Kollision uebernehmen die Teile der Zerlegung (group 3
    = im Viewer standardmaessig ausgeblendet) in einem Body mit derselben Pose.
    """
    hulls = _decompose_cached(mesh_abs)
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
    for i, hull in enumerate(hulls):
        mesh_name = f"{geom_el.get('mesh')}_hull{i:02d}"
        mesh_attrs = {"name": mesh_name, "file": rel_to_meshdir(hull)}
        if mesh_scale:
            mesh_attrs["scale"] = mesh_scale
        ET.SubElement(asset, "mesh", mesh_attrs)
        hull_attrs = {"type": "mesh", "mesh": mesh_name, "group": "3",
                      "rgba": "0.9 0.3 0.3 0.4"}
        for attr in CONTACT_ATTRS:
            if geom_el.get(attr):
                hull_attrs[attr] = geom_el.get(attr)
        ET.SubElement(body, "geom", hull_attrs)
    warnings.append(f"  i Mesh '{geom_el.get('mesh')}': Kollision ueber {len(hulls)} "
                    "konvexe Teile (Viewer: Gruppe 3 einblenden zum Ansehen).")


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

    for env_wb in env_root.findall("worldbody"):
        for el in list(env_wb):
            if el.tag == "light":
                continue  # Licht kommt aus der Basis
            if el.tag == "geom" and el.get("type") == "plane":
                continue  # Boden kommt aus der Basis
            if _is_grasp_geom(el):
                wb.append(_wrap_grasp_geom(el))
                continue
            if el.tag == "body":
                _normalize_free_body(el)
            wb.append(el)
            if el.tag == "geom" and el.get("type") == "mesh" and el.get("mesh") in env_meshes:
                mesh_abs, mesh_scale = env_meshes[el.get("mesh")]
                add_collision_hulls(el, mesh_abs, mesh_scale, asset, wb, warnings)

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


def main() -> None:
    _reexec_in_editor_venv()
    ap = argparse.ArgumentParser(description="G1-Basis + Umgebung zu lauffaehiger Szene kombinieren")
    ap.add_argument("--env", required=True,
                    help="Umgebungs-XML (nur Objekte), z.B. scenes/warehouse.xml")
    ap.add_argument("--inspire", default="0", choices=["0", "1"],
                    help="0 = Rubber-Hand-G1, 1 = Inspire-FTP-G1 (Default 0)")
    ap.add_argument("--out",
                    help="Zieldatei (Default: unitree_robots/g1/scene_env_<name>.xml)")
    args = ap.parse_args()

    env_path = Path(args.env)
    if not env_path.is_absolute():
        env_path = (Path.cwd() / env_path).resolve()
    if not env_path.is_file():
        sys.exit(f"[build_env_scene] Umgebung nicht gefunden: {env_path}")
    env_dir = env_path.parent

    robot_file = ROBOT_FILES[args.inspire]
    if not (G1_DIR / robot_file).is_file():
        sys.exit(f"[build_env_scene] Robotermodell fehlt: {G1_DIR / robot_file}")

    name = env_path.stem
    out_path = Path(args.out).resolve() if args.out else (G1_DIR / f"scene_env_{name}.xml")

    # Umgebung einlesen (Kommentare vorher strippen: MuJoCos Parser ist tolerant,
    # ElementTree strikt - z.B. sind '--'-Folgen in XML-Kommentaren ungueltig).
    raw = env_path.read_text(encoding="utf-8")
    cleaned = re.sub(r"<!--.*?-->", "", raw, flags=re.DOTALL)
    env_root = ET.fromstring(cleaned)

    warnings = []
    mj, asset, wb = build_base(robot_file, f"g1_env_{name}")
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

    for w in warnings:
        print(w, file=sys.stderr)
    print(out_path)


if __name__ == "__main__":
    main()

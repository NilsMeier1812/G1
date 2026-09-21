#!/usr/bin/env python
"""
Wrapper um die mujoco-scene-editor CLI.

Zweck (zwei fest verdrahtete Pfade, damit der Editor out-of-the-box passt):

1. EXPORT-Pfad auf  scene_editor/scenes/  vorbelegen. Beim Speichern muss nur
   noch der Name angepasst werden (z.B. scene.xml -> kueche.xml) + "Export
   scene"; die Datei landet automatisch unter scenes/.

2. ASSET-/MESH-Ordner auf  scene_editor/meshes/  vorbelegen. Der Mesh-Import
   des Editors ("Add Assets from File") scannt ein Verzeichnis nach
   STL/OBJ/... - mit diesem Default tauchen die STLs aus meshes/ sofort in der
   Auswahl auf (Ordner aufklappen -> "Scan assets" -> auswaehlen -> "Add
   asset"). Der eingebaute Default (~/temp/ArmarXObjects) existiert sonst nicht,
   dann ist die Liste leer und es wirkt, als gaebe es keinen Import.

Aufruf wie die normale CLI:
    python run_editor.py new
    python run_editor.py edit scenes/environment_starter.xml
    python run_editor.py prompt "eine Kueche"
"""
import json
import os
import re
import sys
from pathlib import Path

# Haengender cachier-Eintrag (~/.cachier) darf den Editor nicht blockieren:
# Wird ein Editor beendet, WAEHREND er beim ersten Start den Objaverse-Katalog
# laedt, bleibt der Eintrag fuer immer als "in Berechnung" markiert. Jeder
# spaetere Start wartet dann endlos darauf -- die GUI erscheint zwar, aber die
# Knoepfe (z.B. "Add asset") sind nie verdrahtet und tun nichts. Nach 30 s
# Warten rechnet cachier jetzt selbst neu, statt ewig zu haengen.
import cachier
cachier.set_global_params(wait_for_calc_timeout=30)

HERE = Path(__file__).resolve().parent
# Arbeitsverzeichnis fest auf scene_editor/: relative Mesh-Pfade in den Szenen
# (siehe _relativize_scene_files) gelten dann egal, von wo der Editor startet.
os.chdir(HERE)

# Ziel-Ordner fuer exportierte Szenen (fest verdrahtet, neben diesem Skript)
SCENES_DIR = HERE / "scenes"
SCENES_DIR.mkdir(parents=True, exist_ok=True)
DEFAULT_TARGET = str(SCENES_DIR / "scene.xml")

# Ordner, aus dem der Editor eigene Meshes (STL/OBJ/...) importiert.
MESHES_DIR = HERE / "meshes"
MESHES_DIR.mkdir(parents=True, exist_ok=True)
DEFAULT_ASSET_DIR = str(MESHES_DIR)

# Die vorbelegten Pfade in allen Modulen setzen, die sie beim Aufbauen der GUI
# lesen. Wir patchen die schon importierten Modul-Globals, damit es unabhaengig
# von der Importreihenfolge wirkt.
import mujoco_scene_editor.constants as _constants
from mujoco_scene_editor.constants import NO_SELECTION
_constants.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET
_constants.DEFAULT_ASSET_DIR = DEFAULT_ASSET_DIR

import mujoco_scene_editor.layout as _layout
_layout.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET
_layout.DEFAULT_ASSET_DIR = DEFAULT_ASSET_DIR

import mujoco_scene_editor.cli.editor_cli as _editor_cli
_editor_cli.DEFAULT_EXPORT_TARGET = DEFAULT_TARGET


# ---------------------------------------------------------------------------
# Asset-Scan ohne Cache. Der Editor cached die Ordner-Liste per cachier 30 Tage
# lang in ~/.cachier -- neu nach meshes/ kopierte STLs fehlen dann im Dropdown
# "Items", obwohl sie im Ordner liegen. Der Scan ist ein simples rglob und
# braucht keinen Cache, also immer frisch scannen.
# ---------------------------------------------------------------------------
import mujoco_scene_editor.inventory.local_assets as _local_assets


def _list_assets_uncached(self, root, **_cachier_kwargs):
    root = Path(root).expanduser().resolve()
    if not root.is_dir():
        return []
    items = [
        _local_assets.ObjectModel(name=p.stem, path=p)
        for p in _local_assets._iter_asset_files(root, _local_assets.ASSET_EXTS)
    ]
    items.sort(key=lambda m: m.name.lower())
    return items


_local_assets.Inventory.list = _list_assets_uncached


# ---------------------------------------------------------------------------
# Export: Objekte, die an einem Mesh/Shape "haengen", auf Weltebene ziehen.
# Wird ein Objekt hinzugefuegt, waehrend oben unter "Elements" ein Mesh/Shape
# gewaehlt ist, legt der Editor es als KIND davon an (/mesh_0000/box_0001).
# Der XML-Exporter (robits) kann das nicht (AttributeError 'joints') -- dann
# entsteht nur die .json, die .xml fehlt und die Umgebung taucht nirgends auf.
# Hier: solche Kinder vor dem XML-Export mit Welt-Pose (Eltern-Pose * eigene
# Pose) auf die oberste Ebene holen. Die .json bleibt unveraendert.
# ---------------------------------------------------------------------------
from dataclasses import replace as _replace

import numpy as _np
from robits.sim.blueprints import GeomBlueprint, MeshBlueprint, Pose
import robits.sim.converters.mujoco_exporter as _mj_exporter


def _flatten_leaf_children(blueprints):
    by_path = {bp.path: bp for bp in blueprints}

    def world_matrix(bp):
        parent = by_path.get(bp.path.rsplit("/", 1)[0])
        own = _np.asarray(bp.pose.matrix)
        if isinstance(parent, (GeomBlueprint, MeshBlueprint)):
            return world_matrix(parent) @ own
        return own

    flat = []
    for bp in blueprints:
        parent = by_path.get(bp.path.rsplit("/", 1)[0])
        if isinstance(parent, (GeomBlueprint, MeshBlueprint)):
            bp = _replace(bp, path="/" + bp.path.rsplit("/", 1)[1],
                          pose=Pose(matrix=world_matrix(bp)))
        flat.append(bp)
    return flat


_orig_export_scene = _mj_exporter.MujocoXMLExporter.export_scene


def _export_scene_flat(self, out_path, blueprints):
    _orig_export_scene(self, out_path, _flatten_leaf_children(list(blueprints)))
    # Der Exporter entpackt zusaetzlich eine Kopie "<Modellname>.xml" (Default
    # "MuJoCo Model.xml") neben die Zieldatei -- in scenes/ taucht die sonst
    # als eigene Umgebung "MuJoCo Model" auf.
    stray = Path(out_path).parent / "MuJoCo Model.xml"
    if stray.resolve() != Path(out_path).resolve():
        stray.unlink(missing_ok=True)
    _relativize_scene_files(Path(out_path))


def _rel_if_inside(p: str, start: Path) -> str:
    """Mesh-Pfad relativ zu `start`, wenn die Datei in scene_editor/ liegt;
    sonst unveraendert (Datei ausserhalb des Repos -- dann bewusst absolut)."""
    path = Path(p)
    absolute = path if path.is_absolute() else (HERE / path)
    absolute = absolute.resolve()
    try:
        absolute.relative_to(HERE)
    except ValueError:
        return p
    return os.path.relpath(absolute, start).replace(os.sep, "/")


def _relativize_scene_files(xml_path: Path) -> None:
    """Absolute Mesh-Pfade (/home/<user>/...) in <szene>.xml und <szene>.json
    relativ machen, damit die Szenen im Git auf jedem Rechner laufen.
      .xml : relativ zur Szenen-Datei (../meshes/x.stl) -- so loesen MuJoCo,
             der Editor-Import und build_env_scene.py sie auf.
      .json: relativ zu scene_editor/ (meshes/x.stl) -- der Editor laedt sie
             aus seinem Arbeitsverzeichnis (os.chdir(HERE) oben)."""
    xml_path = Path(xml_path).resolve()
    try:
        xml_path.relative_to(HERE)
    except ValueError:
        return   # Export ausserhalb von scene_editor/: absolute Pfade sind dort robuster
    if xml_path.is_file():
        text = xml_path.read_text(encoding="utf-8")
        text = re.sub(r'(<mesh\b[^>]*\bfile=")([^"]+)(")',
                      lambda m: m.group(1) + _rel_if_inside(m.group(2), xml_path.parent) + m.group(3),
                      text)
        xml_path.write_text(text, encoding="utf-8")
    json_path = xml_path.with_suffix(".json")
    if json_path.is_file():
        doc = json.loads(json_path.read_text(encoding="utf-8"))
        for bp in doc.get("blueprints", []):
            if isinstance(bp, dict) and bp.get("mesh_path"):
                bp["mesh_path"] = _rel_if_inside(bp["mesh_path"], HERE)
        json_path.write_text(json.dumps(doc, indent=3), encoding="utf-8")


_mj_exporter.MujocoXMLExporter.export_scene = _export_scene_flat


# ---------------------------------------------------------------------------
# Zusaetzlicher Upload-Button: echter Datei-Dialog des Browsers.
# Der eingebaute Import ("Add Assets from File") scannt nur einen Ordner. Fuer
# "Datei aus beliebigem Ordner auswaehlen" haengen wir per viser-Upload-Button
# einen zweiten Weg an: ausgewaehlte Datei wird nach meshes/ gespeichert und
# direkt in die Szene eingefuegt. Umgesetzt ohne Aenderung am Fremdpaket, indem
# wir die Editor-Fabrik umschliessen.
# ---------------------------------------------------------------------------
_UPLOAD_EXTS = ".stl,.obj,.ply,.glb,.gltf,.STL,.OBJ,.PLY,.GLB,.GLTF"


def _install_upload_button(editor) -> None:
    server = editor.layout.server
    try:
        with server.gui.add_folder("Eigene Datei hochladen", expand_by_default=True):
            up = server.gui.add_upload_button(
                "STL/OBJ waehlen ...", mime_type=_UPLOAD_EXTS,
                hint="Datei aus beliebigem Ordner waehlen; wird nach meshes/ "
                     "kopiert und in die Szene eingefuegt.",
            )
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Upload-Button nicht verfuegbar: {exc}", file=sys.stderr)
        return

    @up.on_upload
    def _on_upload(event) -> None:
        f = up.value
        if not f or not f.name:
            return
        dest = MESHES_DIR / Path(f.name).name
        try:
            dest.write_bytes(f.content)
            editor.controller.create_mesh(editor.get_selected_parent(), dest.resolve())
        except Exception as exc:  # pragma: no cover - Laufzeit
            print(f"[run_editor] Upload fehlgeschlagen: {exc}", file=sys.stderr)
            return
        try:
            event.client.add_notification(
                title="Mesh eingefuegt",
                body=f"{dest.name} nach meshes/ gespeichert und in die Szene gelegt.",
                loading=False,
            )
        except Exception:
            pass


def _notify(event, title, body):
    try:
        event.client.add_notification(title=title, body=body, loading=False)
    except Exception:
        pass


def _install_mesh_scale_control(editor) -> None:
    """Skalier-Control fuer Meshes (fehlt im eingebauten Editor).

    Der Properties-Panel des Editors kann nur Box/Zylinder/Kugel-Masse aendern,
    aber importierte Meshes/STLs nicht skalieren. Hier: gewaehltes Mesh oben
    unter "Elements" waehlen, Faktor eingeben, anwenden. Loest auch mm->m
    (CAD-STL in mm -> Faktor 0.001).
    """
    from robits.sim.blueprints import MeshBlueprint

    server = editor.layout.server
    ctrl = editor.controller
    try:
        with server.gui.add_folder("Mesh skalieren", expand_by_default=True):
            num = server.gui.add_number(
                "Faktor", initial_value=1.0, min=0.0001, max=10000.0, step=0.01,
                hint="Skaliert das oben gewaehlte Mesh. CAD-STL in mm -> 0.001.")
            btn = server.gui.add_button("Auf gewaehltes Mesh anwenden")
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Skalier-Control nicht verfuegbar: {exc}", file=sys.stderr)
        return

    # Beim Auswaehlen eines Meshes den aktuellen Faktor ins Feld holen (viser
    # haengt zusaetzliche on_update-Callbacks an, ersetzt die vorhandenen nicht).
    @editor.layout.element_list.on_update
    def _sync_scale(_evt) -> None:
        bp = ctrl.state.blueprints.get(editor.layout.element_list.value)
        if isinstance(bp, MeshBlueprint):
            try:
                num.value = float(getattr(bp, "scale", 1.0) or 1.0)
            except Exception:
                pass

    @btn.on_click
    def _apply_scale(event) -> None:
        name = editor.layout.element_list.value
        bp = ctrl.state.blueprints.get(name)
        if not isinstance(bp, MeshBlueprint):
            _notify(event, "Kein Mesh gewaehlt",
                    "Bitte oben unter 'Elements' ein importiertes Mesh auswaehlen.")
            return
        factor = float(num.value)
        if factor <= 0:
            _notify(event, "Ungueltiger Faktor", "Faktor muss > 0 sein.")
            return
        try:
            ctrl.state.update(name, scale=factor)
            ctrl.renderer.render_from_state(list(ctrl.state.blueprints.values()))
            editor.layout.element_list.value = name  # Auswahl/Gizmo wiederherstellen
            ctrl.select(name)
        except Exception as exc:  # pragma: no cover - Laufzeit
            print(f"[run_editor] Skalieren fehlgeschlagen: {exc}", file=sys.stderr)
            return
        _notify(event, "Mesh skaliert", f"Faktor {factor} angewendet.")


_GRASP_PREFIX = "grasp_"


def _rename_element(ctrl, old_path: str, new_base: str) -> str:
    """Element (samt angehaengter Kinder) umbenennen. Gibt den neuen Pfad zurueck."""
    parent = old_path.rsplit("/", 1)[0]
    new_path = f"{parent}/{new_base}"
    if new_path == old_path:
        return new_path
    if new_path in ctrl.state.blueprints:
        raise ValueError(f"Name '{new_base}' gibt es schon.")
    ctrl.state.push_state_to_history()  # Undo moeglich
    renamed = {}
    for path, bp in ctrl.state.blueprints.items():
        if path == old_path or path.startswith(old_path + "/"):
            path = new_path + path[len(old_path):]
            bp = _replace(bp, path=path)
        renamed[path] = bp
    ctrl.state.blueprints = renamed
    ctrl.renderer.render_from_state(list(renamed.values()))
    return new_path


def _install_grasp_control(editor) -> None:
    """Umbenennen + Greif-Objekt markieren (fehlt im eingebauten Editor).

    Hindernis vs. Greif-Objekt wird per Namenspraefix "grasp_" unterschieden
    (siehe README, build_env_scene.py, scene_objects.py). Der eingebaute Editor
    kann Elemente aber nicht umbenennen -- das ergaenzt dieser Ordner.
    """
    server = editor.layout.server
    ctrl = editor.controller
    elements = editor.layout.element_list
    try:
        with server.gui.add_folder("Greif-Objekt / Name", expand_by_default=True):
            txt = server.gui.add_text(
                "Name", initial_value="",
                hint="Name des oben gewaehlten Elements. 'grasp_' vorne = Greif-Objekt.")
            btn_rename = server.gui.add_button("Umbenennen")
            btn_grasp = server.gui.add_button("Greif-Objekt an/aus (grasp_)")
    except Exception as exc:  # pragma: no cover - GUI-Aufbau
        print(f"[run_editor] Greif-Control nicht verfuegbar: {exc}", file=sys.stderr)
        return

    @elements.on_update
    def _sync_name(_evt) -> None:
        sel = elements.value
        txt.value = "" if sel == NO_SELECTION else sel.rsplit("/", 1)[-1]

    def _apply(event, new_base: str) -> None:
        old = elements.value
        if old not in ctrl.state.blueprints:
            _notify(event, "Nichts gewaehlt", "Bitte oben unter 'Elements' ein Objekt waehlen.")
            return
        new_base = re.sub(r"[^A-Za-z0-9_\-]", "_", new_base.strip())
        if not new_base:
            _notify(event, "Ungueltiger Name", "Name darf nicht leer sein.")
            return
        try:
            new_path = _rename_element(ctrl, old, new_base)
            elements.value = new_path  # Auswahl/Gizmo wiederherstellen
            ctrl.select(new_path)
        except Exception as exc:
            _notify(event, "Umbenennen fehlgeschlagen", str(exc))
            return
        kind = ("Greif-Objekt (beweglich, die Hand darf ran)"
                if new_base.lower().startswith(_GRASP_PREFIX) else "Hindernis")
        _notify(event, "Umbenannt", f"{new_base} -> {kind}. Danach 'Export scene' nicht vergessen.")

    @btn_rename.on_click
    def _on_rename(event) -> None:
        _apply(event, txt.value)

    @btn_grasp.on_click
    def _on_toggle_grasp(event) -> None:
        base = elements.value.rsplit("/", 1)[-1]
        if base.lower().startswith(_GRASP_PREFIX):
            _apply(event, base[len(_GRASP_PREFIX):])
        else:
            _apply(event, _GRASP_PREFIX + base)


_orig_get_scene_editor = _editor_cli.get_scene_editor


def _get_scene_editor_with_extras(blueprints=None):
    editor = _orig_get_scene_editor(blueprints)
    _install_upload_button(editor)
    _install_mesh_scale_control(editor)
    _install_grasp_control(editor)
    return editor


_editor_cli.get_scene_editor = _get_scene_editor_with_extras


if __name__ == "__main__":
    _editor_cli.cli()

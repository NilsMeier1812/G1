#!/usr/bin/env python3
"""Neue Hand anlegen: 3D-Datei waehlen, Montage-Vorschlaege ansehen, uebernehmen.

Ablauf im Fenster:
  1. STL (oder OBJ/PLY/3MF/...) waehlen, Namen geben.
  2. Das Tool raet die Montage (hand_detect.py) und zeigt mehrere Vorschlaege als
     Bild: rechts die rechte Hand von nah, links beide Haende von vorne.
  3. Vorschlag waehlen -> "Anlegen": Ordner hands/<profil>/ mit Datei + hand.yaml,
     Modell erzeugen (gen_hand_profile.py), danach oeffnet das Bearbeitungstool
     (hand_adjust.py) zum Feinjustieren.

Fenster mit PyQt6 im Aussehen des Startmenues (hand_qt.py). Die Vorschau-Bilder
rendert ein eigener Prozess (MuJoCo-OpenGL und die GUI vertragen sich unter X11
nicht in einem Prozess, siehe hand_adjust.py).

  python3 hand_new.py [datei]                         Fenster
  python3 hand_new.py --create datei --name "Titel" [--pick 1] [--slug x] [--no-build]
                                                      ohne Fenster (Vorschlag Nr. pick)
"""
import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import unicodedata
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_hand_profile as gen  # noqa: E402
import hand_detect as det  # noqa: E402

PLA_DENSITY = 1240.0          # kg/m^3
INFILL = 0.5                  # grobe Annahme fuer gedruckte Teile
THUMB = (300, 225)            # Pixel je Ansicht
ARM_LINKS = ("wrist_yaw_link", "wrist_pitch_link", "wrist_roll_link", "elbow_link")


# ══════════════════════════════════════════════════════════════════════
# Profil anlegen
# ══════════════════════════════════════════════════════════════════════
def slugify(title: str) -> str:
    t = title.lower()
    for a, b in (("ä", "ae"), ("ö", "oe"), ("ü", "ue"), ("ß", "ss")):
        t = t.replace(a, b)
    t = unicodedata.normalize("NFKD", t).encode("ascii", "ignore").decode()
    t = re.sub(r"[^a-z0-9]+", "_", t).strip("_")
    return t[:32] or "hand"


def free_slug(slug: str) -> str:
    """slug, oder slug_2, slug_3 ..., falls der Ordner schon existiert."""
    if not (gen.HANDS_DIR / slug).exists():
        return slug
    i = 2
    while (gen.HANDS_DIR / f"{slug}_{i}").exists():
        i += 1
    return f"{slug}_{i}"


def mass_estimate(mesh, unit: str) -> float:
    scale = 0.001 if unit == "mm" else 1.0
    vol = det.volume_estimate(mesh) * scale ** 3
    return max(0.02, round(vol * PLA_DENSITY * INFILL, 2))


def _num(x, unit):
    nd = 2 if unit == "mm" else 5
    v = round(float(x), nd)
    return 0.0 if v == 0 else v


def _vec(v, unit=None, nd=4):
    if unit is not None:
        return "[" + ", ".join(f"{_num(x, unit)}" for x in v) + "]"
    return "[" + ", ".join(f"{(round(float(x), nd) or 0.0)}" for x in v) + "]"


HAND_YAML = """\
# Hand-Profil "{slug}": angelegt mit "Neue Hand anlegen" (tools/hand_new.py).
# Montage = Vorschlag {pick} der automatischen Erkennung: {why}.
#
# Feinjustage:  unitree_mujoco/unitree_robots/g1/tools/hand_tool.sh adjust {slug}
# Neu erzeugen: unitree_mujoco/unitree_robots/g1/tools/hand_tool.sh build {slug}
# Erklaerung aller Felder: unitree_mujoco/unitree_robots/g1/hands/README.md

name: {title}

mesh: {mesh}
unit: {unit}                          # Einheit der Datei: mm | m (geraten: {unit_guess})

# Masse JE Hand in kg. Geschaetzt aus dem Volumen ({vol_cm3:.0f} cm3) als PLA mit
# {infill:.0f} % Fuellung. Bitte wiegen!
mass_kg: {mass}
color: [0.95, 0.55, 0.10, 1.0]    # RGBA in der Sim

mount:
  # Punkt der Datei, der auf der Flansch-Mitte liegt
  origin: {origin}
  # Richtung in der Datei, die vom Flansch WEG zeigt (Verlaengerung des Unterarms)
  out: {out}
  # Richtung in der Datei zur HANDFLAECHE (dahin, wo die Inspire-Finger schliessen)
  palm: {palm}
  # Feinjustage im Hand-Frame (x = out, y = palm, z = quer): mm bzw. Grad
  offset_mm: [0.0, 0.0, 0.0]
  rotate_deg: [0.0, 0.0, 0.0]

# Eine Datei fuer beide Seiten: eine Seite ist das Spiegelbild der anderen.
mirror:
  left: {mirror_left}
  right: false

# Greifpunkt (TCP) in Datei-Koordinaten: darauf zielen IK, Jog und Planer.
tcp: {tcp}

collision:
  max_hulls: 12
"""


def create_profile(src: Path, title: str, suggestion: dict, *, unit: str, slug: str = "",
                   mirror_left: bool = True, mass: float = None, pick: int = 1,
                   unit_guess: str = "") -> str:
    """Ordner hands/<slug>/ mit Datei + hand.yaml anlegen. Gibt den slug zurueck."""
    src = Path(src)
    slug = slug or free_slug(slugify(title))
    if not gen.NAME_RE.match(slug):
        raise ValueError(f"Ordnername '{slug}': nur a-z, 0-9 und _ erlaubt.")
    pdir = gen.HANDS_DIR / slug
    if pdir.exists():
        raise ValueError(f"hands/{slug} gibt es schon.")
    mesh = det.load_mesh(src)
    pdir.mkdir(parents=True)
    try:
        name = slugify(src.stem) or "hand"
        if src.suffix.lower() == ".stl":
            dst = pdir / f"{name}.stl"
            shutil.copyfile(src, dst)
        else:                                   # andere Formate als STL ablegen
            dst = pdir / f"{name}.stl"
            mesh.export(str(dst))
        scale = 0.001 if unit == "mm" else 1.0
        vol = det.volume_estimate(mesh) * scale ** 3
        if mass is None:
            mass = mass_estimate(mesh, unit)
        title_q = json.dumps(title, ensure_ascii=False)
        (pdir / "hand.yaml").write_text(HAND_YAML.format(
            slug=slug, pick=pick, why=suggestion.get("why", "-"), title=title_q,
            mesh=dst.name, unit=unit, unit_guess=unit_guess or unit,
            vol_cm3=vol * 1e6, infill=INFILL * 100, mass=round(float(mass), 3),
            origin=_vec(suggestion["origin"], unit), out=_vec(suggestion["out"]),
            palm=_vec(suggestion["palm"]), tcp=_vec(suggestion["tcp"], unit),
            mirror_left="true" if mirror_left else "false"), encoding="utf-8")
        gen.load_profile(slug)                  # Plausibilitaet (wirft bei Fehlern)
    except Exception:
        shutil.rmtree(pdir, ignore_errors=True)
        raise
    return slug


# ══════════════════════════════════════════════════════════════════════
# Vorschau-Bilder (eigener Prozess: --analyse)
# ══════════════════════════════════════════════════════════════════════
def _profile_dict(s, unit, mirror_left):
    return {"origin": s["origin"], "out": s["out"], "palm": s["palm"],
            "offset_mm": [0, 0, 0], "rotate_deg": [0, 0, 0],
            "mirror": {"left": mirror_left, "right": False},
            "scale": 0.001 if unit == "mm" else 1.0}


def preview_model(mesh, s, unit, mirror_left):
    """G1 mit der Hand nach Vorschlag s (nur Optik, im Speicher gebaut)."""
    import mujoco
    import numpy as np
    spec = mujoco.MjSpec.from_file(str(gen.BASE_MJCF))
    spec.option.gravity = [0, 0, 0]
    p = _profile_dict(s, unit, mirror_left)
    for side in gen.SIDES:
        body = spec.body(f"{side}_wrist_yaw_link")
        for g in list(body.geoms):
            if g.meshname == f"{side}_rubber_hand":
                spec.delete(g)
        A, b = gen.side_transform(p, side)
        tm = gen.transform_mesh(mesh, A, b)
        me = spec.add_mesh()
        me.name = f"new_hand_{side}"
        me.uservert = np.asarray(tm.vertices, dtype=float).flatten().tolist()
        me.userface = np.asarray(tm.faces, dtype=int).flatten().tolist()
        g = body.add_geom()
        g.type = mujoco.mjtGeom.mjGEOM_MESH
        g.meshname = me.name
        g.name = f"{side}_new_hand"
        g.contype = g.conaffinity = 0
        g.density = 0
        g.rgba = [0.95, 0.55, 0.10, 1.0]
        # Flansch-Flaeche als blaue Scheibe, Greifpunkt als gruene Kugel
        f = body.add_geom()
        f.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        f.size = [0.024, 0.0008, 0]
        f.pos = [gen.FLANGE_X, 0, 0]
        f.quat = [math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0]
        f.contype = f.conaffinity = 0
        f.density = 0
        f.rgba = [0.2, 0.45, 1.0, 0.9]
        t = body.add_geom()
        t.type = mujoco.mjtGeom.mjGEOM_SPHERE
        t.size = [0.006, 0, 0]
        t.pos = (A @ np.asarray(s["tcp"], dtype=float) + b).tolist()
        t.contype = t.conaffinity = 0
        t.density = 0
        t.rgba = [0.1, 0.9, 0.2, 1.0]
    tex = spec.add_texture()
    tex.name, tex.type, tex.builtin = ("sky", mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                                       mujoco.mjtBuiltin.mjBUILTIN_GRADIENT)
    tex.rgb1, tex.rgb2, tex.width, tex.height = [0.35, 0.5, 0.65], [0.05, 0.05, 0.08], 256, 1536
    spec.worldbody.add_light(pos=[0, 0, 3], dir=[0, 0, -1],
                             type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    m = spec.compile()
    arm = {m.body(f"{sd}_{lk}").id for sd in gen.SIDES for lk in ARM_LINKS}
    for gi in range(m.ngeom):
        if m.geom_bodyid[gi] in arm:
            m.geom_rgba[gi, 3] = 0.3
            m.geom_matid[gi] = -1
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def render_suggestion(mesh, s, unit, mirror_left, out_png):
    import mujoco
    import numpy as np
    m, d = preview_model(mesh, s, unit, mirror_left)
    w, h = THUMB
    r = mujoco.Renderer(m, h, w)
    size = float(max(mesh.extents)) * (0.001 if unit == "mm" else 1.0)
    gr = m.geom("right_new_hand").id
    gl = m.geom("left_new_hand").id
    c_r = d.geom_xpos[gr]                   # Mesh-Mitte (MuJoCo zentriert Meshes)
    views = []
    cam = mujoco.MjvCamera()
    cam.lookat[:] = c_r
    cam.distance, cam.azimuth, cam.elevation = max(0.3, 3.2 * size), 120, -20
    r.update_scene(d, cam)
    views.append(r.render())
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (d.geom_xpos[gr] + d.geom_xpos[gl]) / 2
    cam.distance, cam.azimuth, cam.elevation = 0.4 + 2.0 * size, 180, -25
    r.update_scene(d, cam)
    views.insert(0, r.render())
    r.close()
    img = np.concatenate(views, axis=1)
    try:
        from PIL import Image
        Image.fromarray(img).save(out_png)
    except ImportError:                    # ohne Pillow: PPM, kann Qt auch
        out_png = str(out_png)[:-4] + ".ppm"
        with open(out_png, "wb") as fh:
            fh.write(f"P6 {img.shape[1]} {img.shape[0]} 255\n".encode() + img.tobytes())
    return str(out_png)


def analyse(path, out_dir, unit=None, mirror_left=True, n=4, render=True):
    mesh = det.load_mesh(path)
    guess = det.guess_unit(mesh)
    unit = unit or guess
    res = {"unit": unit, "unit_guess": guess, "extents": det._clean(mesh.extents),
           "mass": mass_estimate(mesh, unit),
           "suggestions": det.suggest_mounts(mesh, n, unit)}
    if render:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        for i, s in enumerate(res["suggestions"]):
            try:
                s["image"] = render_suggestion(mesh, s, unit, mirror_left,
                                               out_dir / f"vorschlag_{i + 1}.png")
            except Exception as e:                       # kein OpenGL -> ohne Bild
                s["image"] = ""
                s["image_error"] = str(e)
    return res


# ══════════════════════════════════════════════════════════════════════
# Fenster (PyQt6, Aussehen wie das Startmenue: hand_qt.py)
# ══════════════════════════════════════════════════════════════════════
FILE_FILTER = ("3D-Modelle (" + " ".join(f"*{e} *{e.upper()}" for e in det.MESH_TYPES)
               + ");;Alle Dateien (*)")


def build_wizard(initial=None):
    """Fenster bauen (ohne exec) -> Widget. Nach dem Schliessen steht in
    w.next_slug das neue Profil (dann weiter im Bearbeitungstool)."""
    from PyQt6.QtCore import QObject, QSize, Qt, QTimer, pyqtSignal
    from PyQt6.QtGui import QColor, QFont, QPainter, QPixmap
    from PyQt6.QtWidgets import (QButtonGroup, QFileDialog, QFrame, QHBoxLayout, QLabel,
                                 QLineEdit, QPushButton, QScrollArea,
                                 QSizePolicy, QVBoxLayout, QWidget)

    from g1pilot.utils import msgbox
    from start_gui import theme as T
    from start_gui import widgets as W

    class Bridge(QObject):
        analysed = pyqtSignal(object)
        built = pyqtSignal(str, int, str)

    class SuggestionCard(QPushButton):
        """Vorschlag: Bild (beide Haende | rechte Hand), Titel, Begruendung."""

        IMG_H = 190

        def __init__(self, i, s):
            super().__init__()
            self.index = i
            self.setObjectName("choice")
            self.setCheckable(True)
            self.setCursor(Qt.CursorShape.PointingHandCursor)
            self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            img = s.get("image") or ""
            pm = QPixmap(img) if img else QPixmap()
            self._pix = None if pm.isNull() else pm
            self._err = s.get("image_error", "")
            lay = QVBoxLayout(self)
            lay.setContentsMargins(2, 2, 2, 10)
            lay.setSpacing(2)
            self._img = QLabel()
            self._img.setFixedHeight(self.IMG_H)
            self._img.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lay.addWidget(self._img)
            t = W.label(f"Vorschlag {i + 1}" + ("  ·  wahrscheinlichster" if i == 0 else ""))
            f = t.font()
            f.setWeight(QFont.Weight.DemiBold)
            t.setFont(f)
            t.setContentsMargins(12, 6, 12, 0)
            lay.addWidget(t)
            why = W.label(s.get("why", ""), "hint", wrap=True)
            why.setContentsMargins(12, 0, 12, 0)
            lay.addWidget(why)
            for w in self.findChildren(QLabel):
                w.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
            self.toggled.connect(lambda _on: self._render())

        def sizeHint(self):
            return QSize(420, self.IMG_H + 80)

        def _render(self):
            w = max(120, self.width() - 4)
            out = W.rounded_top(None, w, self.IMG_H)
            p = QPainter(out)
            p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
            if self._pix is not None:
                sc = self._pix.scaled(w, self.IMG_H, Qt.AspectRatioMode.KeepAspectRatio,
                                      Qt.TransformationMode.SmoothTransformation)
                p.drawPixmap((w - sc.width()) // 2, (self.IMG_H - sc.height()) // 2, sc)
            else:
                p.setPen(QColor(T.MUTED))
                p.drawText(out.rect().adjusted(12, 0, -12, 0),
                           Qt.AlignmentFlag.AlignCenter | Qt.TextFlag.TextWordWrap,
                           "Kein Vorschaubild" + (f": {self._err[:90]}" if self._err else ""))
            if self.isChecked():                 # Haken oben rechts wie im Startmenue
                W.paint_check(p, out.width() - 34, 10)
            p.end()
            self._img.setPixmap(out)

        def resizeEvent(self, e):
            super().resizeEvent(e)
            self._render()


    class Wizard(QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Neue Hand anlegen")
            self.resize(980, 860)
            self.res = None
            self.busy = False
            self.again = False
            self.unit = ""                        # "" = raten lassen
            self.mass_auto = None
            self.next_slug = None
            self.tmp = tempfile.mkdtemp(prefix="hand_new_")
            self.bridge = Bridge()
            self.bridge.analysed.connect(self._done)
            self.bridge.built.connect(self._built)

            root = QVBoxLayout(self)
            root.setContentsMargins(0, 0, 0, 0)
            root.setSpacing(0)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QFrame.Shape.NoFrame)
            body = QWidget()
            lay = QVBoxLayout(body)
            lay.setContentsMargins(28, 22, 28, 18)
            lay.setSpacing(8)
            scroll.setWidget(body)
            root.addWidget(scroll, 1)

            head = QHBoxLayout()
            head.setSpacing(14)
            head.addWidget(W.icon_label("hand", T.FG, 26, box=48, bg=T.CARD))
            col = QVBoxLayout()
            col.setSpacing(2)
            col.addWidget(W.label("Neue Hand anlegen", "h1"))
            col.addWidget(W.label("3D-Datei wählen, Montage-Vorschlag aussuchen, danach "
                                  "im Bearbeitungstool fein einpassen.", "sub", wrap=True))
            head.addLayout(col, 1)
            lay.addLayout(head)

            lay.addWidget(W.section_title("Datei"))
            rows = W.RowList()
            fbox = QWidget()
            fl = QHBoxLayout(fbox)
            fl.setContentsMargins(0, 0, 0, 0)
            fl.setSpacing(8)
            self.file_lbl = W.label("noch keine Datei", "muted")
            self.file_lbl.setMinimumWidth(260)
            self.file_lbl.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            fl.addWidget(self.file_lbl, 1)
            fl.addWidget(W.button("Datei wählen …", "", self.choose, icon="folder-open"))
            self.file_path = str(initial or "")
            rows.add(W.FieldRow("3D-Datei", fbox, "STL, OBJ, PLY, 3MF …"))
            self.ed_title = QLineEdit()
            self.ed_title.setPlaceholderText("z. B. Box-Haken")
            self.ed_title.setMinimumWidth(320)
            self.ed_title.textChanged.connect(self._sync_slug)
            self.name_row = W.FieldRow("Name", self.ed_title, "Ordner: –")
            self.slug_lbl = self.name_row.findChildren(QLabel)[1]   # Hinweiszeile
            rows.add(self.name_row)
            seg = QWidget()
            sl = QHBoxLayout(seg)
            sl.setContentsMargins(0, 0, 0, 0)
            sl.setSpacing(6)
            self.unit_group = QButtonGroup(self)
            self.unit_btns = {}
            for u in ("mm", "m"):
                b = QPushButton(u)
                b.setObjectName("seg")
                b.setCheckable(True)
                b.setCursor(Qt.CursorShape.PointingHandCursor)
                b.clicked.connect(lambda _=False, uu=u: self._set_unit(uu))
                self.unit_group.addButton(b)
                sl.addWidget(b)
                self.unit_btns[u] = b
            self.unit_btns["mm"].setChecked(True)
            rows.add(W.FieldRow("Einheit der Datei", seg, "wird beim Laden geraten"))
            self.ed_mass = QLineEdit()
            self.ed_mass.setFixedWidth(90)
            self.ed_mass.setAlignment(Qt.AlignmentFlag.AlignRight)
            rows.add(W.FieldRow("Masse je Hand", self.ed_mass,
                                f"geschätzt aus dem Volumen als PLA mit {INFILL * 100:.0f} % "
                                "Füllung, bitte wiegen", "kg"))
            self.mirror = W.ToggleRow("Linke Hand gespiegelt",
                                      "Eine Datei für beide Seiten: links ist das Spiegelbild.",
                                      True)
            self.mirror.toggled.connect(lambda _on: self.start_analyse())
            rows.add(self.mirror)
            lay.addWidget(rows)

            lay.addWidget(W.section_title("Montage-Vorschläge"))
            lay.addWidget(W.label("Links beide Hände von vorn, rechts die rechte Hand von nah. "
                                  "Blaue Scheibe = Flansch am Handgelenk, grüner Punkt = "
                                  "Greifpunkt.", "hint", wrap=True))
            self.empty = W.label("Erst eine 3D-Datei wählen.", "muted")
            self.empty.setContentsMargins(0, 12, 0, 12)
            lay.addWidget(self.empty)
            self.grid = W.TileGrid(min_w=380)
            lay.addWidget(self.grid)
            self.pick_group = QButtonGroup(self)
            self.pick_group.setExclusive(True)
            self.cards = []
            lay.addStretch(1)

            bar = QFrame()
            bar.setObjectName("bar")
            bl = QHBoxLayout(bar)
            bl.setContentsMargins(24, 12, 24, 12)
            bl.setSpacing(10)
            self.status = W.label("Datei wählen.", "muted", wrap=True)
            bl.addWidget(self.status, 1)
            bl.addWidget(W.button("Abbrechen", "ghost", self.close))
            self.btn_ok = W.button("Anlegen und einpassen", "primary", self.create,
                                   icon="check")
            self.btn_ok.setEnabled(False)
            bl.addWidget(self.btn_ok)
            root.addWidget(W.hline("barline"))
            root.addWidget(bar)

            if self.file_path:
                self._show_file()
                self.ed_title.setText(Path(self.file_path).stem.replace("_", " "))
                QTimer.singleShot(100, self.start_analyse)

        # ── Eingaben ────────────────────────────────────────────────────
        def _show_file(self):
            self.file_lbl.setText(self.file_path or "noch keine Datei")
            self.file_lbl.setProperty("role", "" if self.file_path else "muted")
            self.file_lbl.style().unpolish(self.file_lbl)
            self.file_lbl.style().polish(self.file_lbl)

        def _sync_slug(self, text):
            t = text.strip()
            self.slug_lbl.setText(
                f"Ordner: hands/{free_slug(slugify(t))}/" if t else "Ordner: –")

        def _set_unit(self, u):
            self.unit = u
            self.start_analyse()

        def choose(self):
            f, _ = QFileDialog.getOpenFileName(self, "3D-Datei der Hand",
                                               str(Path(self.file_path).parent)
                                               if self.file_path else str(Path.home()),
                                               FILE_FILTER)
            if not f:
                return
            self.file_path = f
            self.unit = ""
            self._show_file()
            if not self.ed_title.text().strip():
                self.ed_title.setText(Path(f).stem.replace("_", " "))
            self.start_analyse()

        # ── Analyse (eigener Prozess: MuJoCo-Render) ────────────────────
        def start_analyse(self):
            if not self.file_path:
                return
            if self.busy:
                self.again = True                 # nach dem laufenden Durchgang neu
                return
            self.busy = True
            self.btn_ok.setEnabled(False)
            self.status.setText("Erkenne Montage und rendere Vorschläge …")
            cmd = [sys.executable, str(Path(__file__).resolve()), "--analyse", self.file_path,
                   "--out", self.tmp, "--mirror-left", "1" if self.mirror.isChecked() else "0"]
            if self.unit:
                cmd += ["--unit", self.unit]

            def work():
                try:
                    pr = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
                    res = json.loads(pr.stdout.strip().splitlines()[-1]) if pr.returncode == 0 \
                        else {"error": (pr.stderr or pr.stdout).strip()[-800:]}
                except Exception as e:
                    res = {"error": str(e)}
                self.bridge.analysed.emit(res)
            threading.Thread(target=work, daemon=True).start()

        def _done(self, res):
            self.busy = False
            if self.again:
                self.again = False
                self.start_analyse()
                return
            if "error" in res:
                self.status.setText("Fehler: " + res["error"])
                return
            self.show_result(res)

        def show_result(self, res):
            self.res = res
            self.unit = res["unit"]
            self.unit_btns[res["unit"]].setChecked(True)
            if not self.ed_mass.text().strip() or self.mass_auto == self.ed_mass.text():
                self.ed_mass.setText(str(res["mass"]))
                self.mass_auto = self.ed_mass.text()
            old = self.pick()
            for c in self.cards:
                self.pick_group.removeButton(c)
            self.cards = [SuggestionCard(i, s) for i, s in enumerate(res["suggestions"])]
            for c in self.cards:
                self.pick_group.addButton(c)
            if self.cards:
                self.cards[old if old < len(self.cards) else 0].setChecked(True)
            self.grid.set_items(self.cards)
            self.empty.setVisible(not self.cards)
            ex = " × ".join(f"{x:g}" for x in res["extents"])
            self.status.setText(f"Größe {ex} {res['unit']}. Vorschlag wählen; danach im "
                                "Bearbeitungstool fein einpassen.")
            self.btn_ok.setEnabled(bool(self.cards))

        def pick(self):
            for c in self.cards:
                if c.isChecked():
                    return c.index
            return 0

        # ── Anlegen ─────────────────────────────────────────────────────
        def create(self):
            res = self.res
            title = self.ed_title.text().strip()
            if not res or not title:
                msgbox.warn(self, "Neue Hand", "Bitte Datei wählen und einen Namen geben.")
                return
            try:
                mass = float(self.ed_mass.text().replace(",", "."))
                assert mass > 0
            except Exception:
                msgbox.warn(self, "Neue Hand", "Masse: Zahl > 0 in kg.")
                return
            i = self.pick()
            try:
                slug = create_profile(Path(self.file_path), title, res["suggestions"][i],
                                      unit=res["unit"], mirror_left=self.mirror.isChecked(),
                                      mass=mass, pick=i + 1, unit_guess=res["unit_guess"])
            except Exception as e:
                msgbox.warn(self, "Neue Hand", str(e))
                return
            self.btn_ok.setEnabled(False)
            self.status.setText(f"hands/{slug}/ angelegt. Erzeuge Modell (Kollisions-Zerlegung, "
                                "dauert etwas) …")

            def work():
                # ueber hand_tool.sh: nimmt das Szenen-Editor-venv (vhacdx fuer die Zerlegung)
                pr = subprocess.run(["bash", str(HERE / "hand_tool.sh"), "build", slug],
                                    capture_output=True, text=True)
                self.bridge.built.emit(slug, pr.returncode, (pr.stderr or pr.stdout))
                if pr.returncode == 0:
                    print(pr.stdout, flush=True)
            threading.Thread(target=work, daemon=True).start()

        def _built(self, slug, rc, out):
            if rc != 0:
                self.status.setText("Erzeugen fehlgeschlagen: " + out[-600:])
                return
            print(f"Neue Hand: hands/{slug}/ -> Bearbeitungstool", flush=True)
            self.next_slug = slug
            self.close()

        def closeEvent(self, e):
            shutil.rmtree(self.tmp, ignore_errors=True)
            super().closeEvent(e)

    return Wizard()


def run_wizard(initial=None):
    import hand_qt
    app = hand_qt.make_app("g1-hand-tool")
    w = build_wizard(initial)
    w.show()
    app.exec()
    if w.next_slug:
        # gleich weiter im Bearbeitungstool (selber Prozess -> die Konsole der
        # Start-GUI zeigt es weiter an, "Stoppen" schliesst es)
        adj = str(HERE / "hand_adjust.py")
        os.execv(sys.executable, [sys.executable, adj, w.next_slug])
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Neue Hand anlegen (Datei + Montage-Vorschlag)")
    ap.add_argument("file", nargs="?", help="3D-Datei (STL, OBJ, PLY, 3MF, ...)")
    ap.add_argument("--analyse", metavar="DATEI", help=argparse.SUPPRESS)
    ap.add_argument("--out", help=argparse.SUPPRESS)
    ap.add_argument("--unit", choices=("mm", "m"))
    ap.add_argument("--mirror-left", default="1", choices=("0", "1"))
    ap.add_argument("--create", metavar="DATEI", help="ohne Fenster anlegen")
    ap.add_argument("--name", help="Anzeigename (mit --create)")
    ap.add_argument("--slug", default="", help="Ordnername (Standard: aus dem Namen)")
    ap.add_argument("--pick", type=int, default=1, help="Vorschlag Nr. (mit --create)")
    ap.add_argument("--mass", type=float)
    ap.add_argument("--no-build", action="store_true")
    a = ap.parse_args(argv)
    mirror = a.mirror_left == "1"
    if a.analyse:
        print(json.dumps(analyse(a.analyse, a.out or tempfile.mkdtemp(), a.unit, mirror)))
        sys.stdout.flush()
        os._exit(0)                     # GL-Kontext nicht aufraeumen (GLFW-Absturz)
    if a.create:
        res = analyse(a.create, None, a.unit, mirror, render=False)
        sug = res["suggestions"]
        if not 1 <= a.pick <= len(sug):
            sys.exit(f"--pick: 1..{len(sug)}")
        slug = create_profile(Path(a.create), a.name or Path(a.create).stem, sug[a.pick - 1],
                              unit=res["unit"], slug=a.slug, mirror_left=mirror,
                              mass=a.mass, pick=a.pick, unit_guess=res["unit_guess"])
        print(f"Angelegt: hands/{slug}/hand.yaml  ({sug[a.pick - 1]['why']})")
        if not a.no_build:
            gen.build(slug)
        return 0
    import hand_qt
    if not hand_qt.have_qt():
        sys.exit(hand_qt.QT_MISSING.format(py=sys.executable)
                 + " Ohne Fenster: hand_new.py --create <datei> --name <name>")
    return run_wizard(a.file)


if __name__ == "__main__":
    sys.exit(main())

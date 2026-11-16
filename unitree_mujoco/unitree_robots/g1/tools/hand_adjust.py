#!/usr/bin/env python3
"""Justier-Werkzeug: eigene Hand interaktiv an den Arm setzen.

Oeffnet den MuJoCo-Viewer (G1 schwebend, Arme vorne) und daneben ein kleines
Fenster mit Schiebern. Versatz (mm) und Drehung (Grad) wirken SOFORT auf beide
Haende (links gespiegelt), ohne Neuerzeugen. "Speichern" schreibt die Werte in
hand.yaml (mount.offset_mm / mount.rotate_deg), erzeugt das Modell neu und
laedt die Ansicht neu.

Achsen (Hand-Frame, gleich fuer beide Seiten):
  x = vom Flansch weg (Unterarm-Verlaengerung)  -> positiv = Hand weiter raus
  y = Richtung Handflaeche (zur Kiste)
  z = quer (x * y)
Drehungen wirken um die Flansch-Mitte.

Hilfen im Fenster: Unterarm durchsichtig (sieht man, wie tief ein Adapter im
Arm steckt), Kollisions-Teile zeigen. Die Schieber (PyQt6, Aussehen wie das Startmenue)
laufen in einem eigenen Prozess (Qt/Tk und der MuJoCo-Viewer vertragen sich
unter X11 nicht in einem).

Aufruf:  tools/hand_tool.sh adjust haken
"""
import argparse
import math
import os
import re
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_hand_profile as gen  # noqa: E402

SIDES = gen.SIDES
# Unterarm-Glieder, die "durchsichtig" werden (Adapter im Arm sichtbar machen)
ARM_LINKS = ("wrist_yaw_link", "wrist_pitch_link", "wrist_roll_link", "elbow_link")
SLIDERS = (("Versatz x (raus)", "mm", 0, 60.0, 0.5),
           ("Versatz y (Handflaeche)", "mm", 1, 60.0, 0.5),
           ("Versatz z (quer)", "mm", 2, 60.0, 0.5),
           ("Drehung um x", "Grad", 0, 180.0, 1.0),
           ("Drehung um y", "Grad", 1, 180.0, 1.0),
           ("Drehung um z", "Grad", 2, 180.0, 1.0))


# ══════════════════════════════════════════════════════════════════════
# Mathe (ohne GUI, getestet in g1pilot/test/test_hand_profile.py)
# ══════════════════════════════════════════════════════════════════════
def hand_frame(side):
    """Spalten (out, palm, quer) des Hand-Frames im wrist_yaw_link-Frame."""
    x = np.array([1.0, 0.0, 0.0])
    palm = np.array([0.0, -1.0, 0.0]) if side == "left" else np.array([0.0, 1.0, 0.0])
    return np.column_stack([x, palm, np.cross(x, palm)])


def mat_to_rpy(R):
    """Umkehrung von gen._rpy (R = Rz(y) Ry(p) Rx(r)) -> (r, p, y) in rad."""
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    else:                                    # Gimbal-Lock: Gier auf 0
        r = math.atan2(-R[1, 2], R[1, 1])
        y = 0.0
    return r, p, y


def delta_pose(side, mm, deg):
    """Zusaetzliche Lage des Werkzeug-Bodys im wrist_yaw_link-Frame fuer eine
    Korrektur (mm, Grad) im Hand-Frame, gedreht um die Flansch-Mitte.
    Rueckgabe (pos, R): p' = R p + pos."""
    H = hand_frame(side)
    Radj = gen._rpy(*[math.radians(a) for a in deg])
    Rd = H @ Radj @ H.T
    F = np.array([gen.FLANGE_X, 0.0, 0.0])
    pos = F + H @ (np.asarray(mm, dtype=float) * 0.001) - Rd @ F
    return pos, Rd


def compose(offset_mm, rotate_deg, mm, deg):
    """Neue (offset_mm, rotate_deg) fuer hand.yaml, wenn auf die gespeicherte
    Montage die Korrektur (mm, deg) gesetzt wird."""
    R0 = gen._rpy(*[math.radians(a) for a in rotate_deg])
    Radj = gen._rpy(*[math.radians(a) for a in deg])
    off = np.asarray(mm, dtype=float) + Radj @ np.asarray(offset_mm, dtype=float)
    rot = [math.degrees(a) for a in mat_to_rpy(Radj @ R0)]
    # + 0.0 macht aus -0.0 eine 0.0 (sonst steht "-0" in hand.yaml)
    return [round(float(v), 2) + 0.0 for v in off], [round(float(v), 2) + 0.0 for v in rot]


def write_mount(yaml_path: Path, offset_mm, rotate_deg):
    """offset_mm/rotate_deg in hand.yaml ersetzen, Kommentare bleiben stehen."""
    txt = yaml_path.read_text(encoding="utf-8")

    def fmt(v):
        return "[" + ", ".join(f"{x:g}" for x in v) + "]"

    for key, val in (("offset_mm", offset_mm), ("rotate_deg", rotate_deg)):
        pat = re.compile(rf"^(\s*{key}:\s*)\[[^\]]*\]", re.M)
        if pat.search(txt):
            txt = pat.sub(lambda m: m.group(1) + fmt(val), txt, count=1)
        else:                                   # Feld fehlt -> unter "mount:" anhaengen
            txt = re.sub(r"^(mount:\s*\n)", rf"\g<1>  {key}: {fmt(val)}\n", txt, count=1, flags=re.M)
    yaml_path.write_text(txt, encoding="utf-8")


# ══════════════════════════════════════════════════════════════════════
# Viewer-Prozess  <->  Schieber-Fenster (eigener Prozess)
# ══════════════════════════════════════════════════════════════════════
# MuJoCo-Viewer (GLFW, eigener Render-Thread) und die GUI im selben Prozess greifen
# aus zwei Threads auf X11 zu -> "X Error" bzw. eingefrorenes Bild. Darum laeuft
# das Schieber-Fenster als eigener Prozess und spricht per JSON-Zeilen ueber
# stdin/stdout mit dem Viewer-Prozess.

def build_panel(name, send):
    """Schieber-Fenster (PyQt6, Aussehen wie das Startmenue). send(dict) geht an
    den Viewer-Prozess. Rueckgabe: Widget mit .on_message(dict)."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtGui import QFont
    from PyQt6.QtWidgets import (QDoubleSpinBox, QHBoxLayout, QLabel, QSlider,
                                 QVBoxLayout, QWidget)

    from start_gui import theme as T
    from start_gui import widgets as W

    class SliderRow(QWidget):
        """Titel | Schieber | Zahlenfeld | Einheit; Schieber und Feld gekoppelt."""

        def __init__(self, title, unit, rng, res, on_change):
            super().__init__()
            self.scale = round(1.0 / res)
            lay = QHBoxLayout(self)
            lay.setContentsMargins(0, 6, 0, 6)
            lay.setSpacing(12)
            t = W.label(title)
            t.setFixedWidth(150)
            lay.addWidget(t)
            self.slider = QSlider(Qt.Orientation.Horizontal)
            self.slider.setRange(int(-rng * self.scale), int(rng * self.scale))
            self.slider.setMinimumWidth(240)
            lay.addWidget(self.slider, 1)
            self.spin = QDoubleSpinBox()
            self.spin.setRange(-rng, rng)
            self.spin.setSingleStep(res)
            self.spin.setDecimals(1)
            self.spin.setFixedWidth(92)
            self.spin.setAlignment(Qt.AlignmentFlag.AlignRight)
            self.spin.setKeyboardTracking(False)
            self.spin.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
            lay.addWidget(self.spin)
            u = W.label(unit, "muted")
            u.setFixedWidth(34)
            lay.addWidget(u)
            self.slider.valueChanged.connect(
                lambda v: self.spin.setValue(v / self.scale))
            self.spin.valueChanged.connect(self._from_spin)
            self.spin.valueChanged.connect(lambda _v: on_change())

        def _from_spin(self, v):
            self.slider.blockSignals(True)
            self.slider.setValue(int(round(v * self.scale)))
            self.slider.blockSignals(False)

        def value(self):
            return float(self.spin.value())

        def set_value(self, v):
            self.spin.setValue(v)

    class Panel(QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle(f"Hand einpassen: {name}")
            self.resize(620, 720)
            self._last = None
            self._loading = True
            root = QVBoxLayout(self)
            root.setContentsMargins(24, 20, 24, 18)
            root.setSpacing(8)

            head = QHBoxLayout()
            head.setSpacing(14)
            head.addWidget(W.icon_label("move", T.FG, 24, box=46, bg=T.CARD))
            col = QVBoxLayout()
            col.setSpacing(2)
            col.addWidget(W.label(f"Hand einpassen: {name}", "h1"))
            col.addWidget(W.label("Korrektur relativ zur gespeicherten Lage. Wirkt sofort im "
                                  "MuJoCo-Fenster, links gespiegelt.", "sub", wrap=True))
            head.addLayout(col, 1)
            root.addLayout(head)

            self.rows = []
            for title, group in (("Versatz", SLIDERS[:3]), ("Drehung um die Flansch-Mitte",
                                                             SLIDERS[3:])):
                root.addWidget(W.section_title(title))
                lst = W.RowList()
                for label, unit, _i, rng, res in group:
                    text = (label.replace("Versatz ", "").replace("Drehung um ", "um ")
                            .replace("Handflaeche", "Handfläche"))
                    r = SliderRow(text,
                                  "Grad" if unit == "Grad" else unit, rng, res, self.push)
                    self.rows.append(r)
                    lst.add(r)
                root.addWidget(lst)

            root.addWidget(W.section_title("Ansicht"))
            lst = W.RowList()
            self.glass = W.ToggleRow("Unterarm durchsichtig",
                                     "Zeigt, wie tief ein Adapter im Arm steckt.", True)
            self.col = W.ToggleRow("Kollisions-Teile zeigen",
                                   "Konvexe Teile, mit denen Sim und Planer rechnen.", False)
            for t in (self.glass, self.col):
                t.toggled.connect(lambda _on: self.push())
                lst.add(t)
            root.addWidget(lst)

            btns = QHBoxLayout()
            btns.setSpacing(10)
            btns.addWidget(W.button("Korrektur auf 0", "ghost", self.reset, icon="rotate-ccw"))
            btns.addStretch(1)
            self.btn_save = W.button("Speichern (hand.yaml)", "primary",
                                     lambda: send({"cmd": "save"}), icon="save")
            btns.addWidget(self.btn_save)
            root.addSpacing(6)
            root.addLayout(btns)

            self.status = QLabel("Viewer startet …")
            self.status.setObjectName("statusbox")
            self.status.setWordWrap(True)
            self.status.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            f = QFont("monospace")
            f.setStyleHint(QFont.StyleHint.Monospace)
            self.status.setFont(f)
            self.status.setStyleSheet(f"QLabel#statusbox{{background:{T.CONSOLE_BG};"
                                      f"border:1px solid {T.LINE};border-radius:10px;"
                                      f"padding:10px;color:#d7dbe0;}}")
            root.addWidget(self.status)
            root.addStretch(1)
            self._loading = False

        def state(self):
            v = [r.value() for r in self.rows]
            return {"mm": v[:3], "deg": v[3:], "glass": self.glass.isChecked(),
                    "col": self.col.isChecked()}

        def push(self):
            if self._loading:
                return
            st = self.state()
            if st != self._last:
                self._last = st
                send(st)

        def reset(self):
            self._loading = True
            for r in self.rows:
                r.set_value(0.0)
            self._loading = False
            self.push()

        def on_message(self, msg):
            if msg.get("reset"):
                self.reset()
            if "status" in msg:
                self.status.setText(msg["status"])

        def closeEvent(self, e):
            send({"cmd": "quit"})
            super().closeEvent(e)

    return Panel()


def run_panel(name):
    """Eigener Prozess: Schieber-Fenster, JSON-Zeilen ueber stdin/stdout."""
    import json
    import threading

    import hand_qt
    from PyQt6.QtCore import QObject, QTimer, pyqtSignal

    app = hand_qt.make_app("g1-hand-tool")

    class Inbox(QObject):
        line = pyqtSignal(object)

    def send(msg):
        try:
            sys.stdout.write(json.dumps(msg) + "\n")
            sys.stdout.flush()
        except (BrokenPipeError, OSError):
            app.quit()

    panel = build_panel(name, send)
    inbox = Inbox()

    def on_line(msg):
        if msg is None:                        # Viewer-Prozess weg
            app.quit()
        else:
            panel.on_message(msg)
    inbox.line.connect(on_line)

    def reader():
        for line in sys.stdin:
            try:
                inbox.line.emit(json.loads(line))
            except ValueError:
                pass
        inbox.line.emit(None)

    threading.Thread(target=reader, daemon=True).start()
    # oben rechts, neben dem MuJoCo-Fenster
    scr = app.primaryScreen().availableGeometry() if app.primaryScreen() else None
    if scr is not None:
        panel.move(scr.right() - panel.width() - 20, scr.top() + 40)
    panel.show()
    QTimer.singleShot(100, panel.push)
    app.exec()


class Adjuster:
    """Viewer-Prozess: haelt Modell + passiven MuJoCo-Viewer und setzt die
    Werkzeug-Bodies nach den Werten aus dem Schieber-Fenster."""

    def __init__(self, name):
        import json
        import queue
        import subprocess
        import threading
        self.json = json
        self.name = name
        self.viewer = None
        self.cam = None
        self.state = {"mm": [0.0] * 3, "deg": [0.0] * 3, "glass": True, "col": False}
        self.inbox = queue.Queue()
        self.panel = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--panel", name],
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                                      bufsize=1)

        def reader():
            for line in self.panel.stdout:
                self.inbox.put(line)
            self.inbox.put(None)

        threading.Thread(target=reader, daemon=True).start()
        self._load()

    # ── Modell / Viewer ─────────────────────────────────────────────────
    def _load(self):
        import mujoco
        import mujoco.viewer
        import hand_preview
        self.mujoco = mujoco
        p = gen.load_profile(self.name)
        self.saved = (list(p["offset_mm"]), list(p["rotate_deg"]))
        if self.viewer is not None:
            self.cam = (self.viewer.cam.lookat.copy(), self.viewer.cam.distance,
                        self.viewer.cam.azimuth, self.viewer.cam.elevation)
            self._close_viewer()
        self.m, self.d = hand_preview._load(self.name)
        m = self.m
        self.tool = {s: m.body(f"{s}_hand_tool_link").id for s in SIDES}
        self.base_pos = {s: m.body_pos[b].copy() for s, b in self.tool.items()}
        arm_bodies = {m.body(f"{s}_{l}").id for s in SIDES for l in ARM_LINKS}
        self.arm_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in arm_bodies]
        self.arm_rgba = m.geom_rgba[self.arm_geoms].copy()
        self.viewer = mujoco.viewer.launch_passive(self.m, self.d, show_left_ui=False,
                                                   show_right_ui=False)
        with self.viewer.lock():
            hand_preview._setup_opt(self.viewer.opt)
        # Kamera erst setzen, wenn der Viewer-Thread laeuft (sonst ueberschreibt er
        # sie mit seiner Startansicht): beim ersten Mal auf die rechte Hand, nach
        # dem Speichern die vorherige Ansicht.
        self._cam_due = time.monotonic() + 0.8
        self._apply()

    def _close_viewer(self):
        """Viewer schliessen und warten, bis sein Render-Thread weg ist."""
        if self.viewer is None:
            return
        self.viewer.close()
        t0 = time.monotonic()
        while self.viewer.is_running() and time.monotonic() - t0 < 3.0:
            time.sleep(0.02)

    def _set_cam(self):
        with self.viewer.lock():
            if self.cam:
                (self.viewer.cam.lookat[:], self.viewer.cam.distance,
                 self.viewer.cam.azimuth, self.viewer.cam.elevation) = self.cam
            else:
                c = self.d.xpos[self.tool["right"]]
                self.viewer.cam.lookat[:] = c + np.array([0.12, 0.0, 0.0])
                self.viewer.cam.distance, self.viewer.cam.azimuth, self.viewer.cam.elevation = \
                    0.45, 120, -20

    def _apply(self):
        """Zustand (Schieber + Anzeige-Haken) aufs Modell legen."""
        mm, deg = self.state["mm"], self.state["deg"]
        with self.viewer.lock():
            for s, b in self.tool.items():
                pos, R = delta_pose(s, mm, deg)
                q = np.zeros(4)
                self.mujoco.mju_mat2Quat(q, R.flatten())
                self.m.body_pos[b] = pos + R @ self.base_pos[s]
                self.m.body_quat[b] = q
            rgba = self.arm_rgba.copy()
            if self.state["glass"]:
                rgba[:, 3] = 0.25
            self.m.geom_rgba[self.arm_geoms] = rgba
            self.viewer.opt.geomgroup[3] = int(self.state["col"])
            self.mujoco.mj_forward(self.m, self.d)
        self._send_status()

    def _status_text(self, extra=""):
        off, rot = compose(*self.saved, self.state["mm"], self.state["deg"])
        tcp = self.d.site_xpos[self.m.site("right_hand_tcp").id]
        return (f"Ergebnis fuer hand.yaml:\n  offset_mm:  {off}\n  rotate_deg: {rot}\n"
                f"Gespeichert: offset_mm {self.saved[0]}, rotate_deg {self.saved[1]}\n"
                f"TCP rechts (Welt): {np.round(tcp, 3).tolist()}" + extra)

    def _send(self, msg):
        try:
            self.panel.stdin.write(self.json.dumps(msg) + "\n")
            self.panel.stdin.flush()
        except (BrokenPipeError, OSError, ValueError):
            pass

    def _send_status(self, extra=""):
        self._send({"status": self._status_text(extra)})

    def save(self):
        off, rot = compose(*self.saved, self.state["mm"], self.state["deg"])
        write_mount(gen.HANDS_DIR / self.name / "hand.yaml", off, rot)
        self._send({"status": "Speichere und erzeuge neu ..."})
        try:
            gen.build(self.name)
        except Exception as e:                                   # noqa: BLE001
            self._send_status(f"\nFEHLER beim Erzeugen: {e}")
            return
        self.state["mm"], self.state["deg"] = [0.0] * 3, [0.0] * 3
        self._send({"reset": True})
        self._load()
        self._send_status("\nGespeichert und neu erzeugt.")

    def handle(self, line):
        msg = self.json.loads(line)
        cmd = msg.get("cmd")
        if cmd == "save":
            self.save()
        elif cmd == "quit":
            return False
        else:
            self.state.update({k: msg[k] for k in ("mm", "deg", "glass", "col") if k in msg})
            self._apply()
        return True

    def run(self):
        import queue
        try:
            while self.viewer.is_running():
                if self._cam_due and time.monotonic() > self._cam_due:
                    self._set_cam()
                    self._cam_due = None
                try:
                    while True:
                        line = self.inbox.get_nowait()
                        if line is None or not self.handle(line):
                            return
                except queue.Empty:
                    pass
                self.viewer.sync()
                time.sleep(0.02)
        finally:
            self._close_viewer()
            if self.panel.poll() is None:
                self.panel.terminate()


def main():
    ap = argparse.ArgumentParser(description="Hand-Profil interaktiv einpassen")
    ap.add_argument("profile")
    ap.add_argument("--panel", action="store_true", help=argparse.SUPPRESS)
    a = ap.parse_args()
    if a.profile not in gen.list_profiles():
        sys.exit(f"Profil '{a.profile}' nicht gefunden (verfuegbar: "
                 f"{', '.join(gen.list_profiles())}).")
    import hand_qt
    if not hand_qt.have_qt():
        sys.exit(hand_qt.QT_MISSING.format(py=sys.executable)
                 + " Ohne Schieber: hand_tool.sh preview <profil>")
    if a.panel:
        run_panel(a.profile)
        return 0
    if not gen.check(a.profile):
        gen.build(a.profile)
    Adjuster(a.profile).run()
    # Der passive Viewer haengt an GLFW; ein harter Ausgang vermeidet Abstuerze
    # beim Aufraeumen des Interpreters (alles ist gespeichert).
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())

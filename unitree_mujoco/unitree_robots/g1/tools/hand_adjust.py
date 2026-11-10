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
Arm steckt), Kollisions-Teile zeigen. Die Schieber laufen in einem eigenen
Prozess (Tk und der MuJoCo-Viewer vertragen sich unter X11 nicht in einem).

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
# MuJoCo-Viewer (GLFW, eigener Render-Thread) und Tk im selben Prozess greifen
# aus zwei Threads auf X11 zu -> "X Error" bzw. eingefrorenes Bild. Darum laeuft
# das Schieber-Fenster als eigener Prozess und spricht per JSON-Zeilen ueber
# stdin/stdout mit dem Viewer-Prozess.

def run_panel(name):
    """Schieber-Fenster. Schickt bei jeder Aenderung den Zustand, liest Status."""
    import json
    import queue
    import threading
    import tkinter as tk

    root = tk.Tk()
    root.title(f"Hand einpassen: {name}")
    root.geometry("-20+40")          # oben rechts, neben dem MuJoCo-Fenster
    vars_ = [tk.DoubleVar(value=0.0) for _ in SLIDERS]
    v_glass = tk.BooleanVar(value=True)
    v_col = tk.BooleanVar(value=False)
    status = tk.StringVar(value="Viewer startet ...")
    inbox = queue.Queue()

    def send(msg):
        try:
            sys.stdout.write(json.dumps(msg) + "\n")
            sys.stdout.flush()
        except (BrokenPipeError, OSError):
            root.destroy()

    def values():
        out = []
        for v in vars_:
            try:
                out.append(float(v.get()))
            except (ValueError, tk.TclError):
                out.append(0.0)
        return out

    last = [None]

    def push_state(*_):
        st = {"mm": values()[:3], "deg": values()[3:], "glass": v_glass.get(), "col": v_col.get()}
        if st != last[0]:
            last[0] = st
            send(st)

    f = tk.Frame(root, padx=10, pady=8)
    f.pack(fill="both", expand=True)
    tk.Label(f, text="Korrektur relativ zur gespeicherten Lage\n"
                     "(wirkt sofort im MuJoCo-Fenster, links gespiegelt)",
             justify="left").pack(anchor="w")
    for var, (label, unit, _i, rng, res) in zip(vars_, SLIDERS):
        row = tk.Frame(f)
        row.pack(fill="x", pady=1)
        tk.Label(row, text=f"{label} [{unit}]", width=24, anchor="w").pack(side="left")
        tk.Scale(row, variable=var, from_=-rng, to=rng, resolution=res, orient="horizontal",
                 length=260, showvalue=False).pack(side="left")
        tk.Spinbox(row, textvariable=var, from_=-rng, to=rng, increment=res,
                   width=7).pack(side="left", padx=4)
        var.trace_add("write", push_state)
    opts = tk.Frame(f)
    opts.pack(fill="x", pady=(6, 2))
    tk.Checkbutton(opts, text="Unterarm durchsichtig", variable=v_glass,
                   command=push_state).pack(side="left")
    tk.Checkbutton(opts, text="Kollisions-Teile zeigen", variable=v_col,
                   command=push_state).pack(side="left", padx=8)
    btns = tk.Frame(f)
    btns.pack(fill="x", pady=6)
    tk.Button(btns, text="Speichern (hand.yaml)", command=lambda: send({"cmd": "save"})).pack(side="left")
    tk.Button(btns, text="Korrektur auf 0",
              command=lambda: [v.set(0.0) for v in vars_]).pack(side="left", padx=6)
    tk.Label(f, textvariable=status, justify="left", fg="#555").pack(anchor="w")

    def reader():
        for line in sys.stdin:
            inbox.put(line)
        inbox.put(None)

    threading.Thread(target=reader, daemon=True).start()

    def poll():
        try:
            while True:
                line = inbox.get_nowait()
                if line is None:                       # Viewer-Prozess weg
                    root.destroy()
                    return
                msg = json.loads(line)
                if msg.get("reset"):
                    for v in vars_:
                        v.set(0.0)
                if "status" in msg:
                    status.set(msg["status"])
        except queue.Empty:
            pass
        root.after(50, poll)

    root.protocol("WM_DELETE_WINDOW", lambda: (send({"cmd": "quit"}), root.destroy()))
    root.after(50, poll)
    root.after(100, push_state)
    root.mainloop()


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
    try:
        import tkinter  # noqa: F401
    except ImportError:
        v = f"{sys.version_info.major}.{sys.version_info.minor}"
        sys.exit(f"tkinter fehlt fuer Python {v}: sudo apt install python{v}-tk "
                 "(oder ohne Schieber: hand_tool.sh preview <profil>)")
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

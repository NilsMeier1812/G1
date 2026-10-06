#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
#  g1_gui.py — Grafisches Start-Menue fuer den G1-Stack (Sim / Real / Szenen).
#
#  Ziel: Im normalen Betrieb muss man KEIN cmd-Fenster mehr anfassen. Dieses
#  Tkinter-Fenster ersetzt die Text-Menues aus:
#     - g1pilot/start.sh                 (Sim / Real starten)
#     - unitree_mujoco/scene_editor/launch.sh   (Umgebungen bearbeiten)
#
#  Es implementiert die Orchestrierung NICHT neu, sondern ruft die bewaehrten
#  Skripte env-getrieben auf:
#     Sim / Real  ->  start.sh --yes           (alle Optionen als Env-Vars)
#     Szenen      ->  scene_editor/launch.sh <cmd> [datei]
#  Die Ausgabe der Skripte (Docker/Compose/Viewer) landet live in einer
#  In-App-Log-View mit Stop-Button — kein Terminal noetig.
#
#  EIN Fenster: Menue, Options-Views und Log-Views werden im selben Fenster
#  ausgetauscht (Router). Laufende Prozesse bleiben in der Registry — man geht
#  per '‹ Menue' zurueck (Prozess laeuft weiter) und ueber 'Laufende Prozesse'
#  wieder hin. Schliessen = ein einziges Fenster schliessen.
#
#  Aufbau (Sim und Real gleich, in der Reihenfolge der Entscheidungen):
#     Umgebung (nur Sim) bzw. Verbindung (nur Real) -> Bedienoberflaeche
#     (Demo-GUI / Streamdeck) -> Ausstattung (Haende, Navigation, RViz)
#     -> eingeklapptes 'Erweitert'. Real zusaetzlich Geh-Limits und das
#     Sicherheits-Gate. Hand-Oberflaechen im Browser werden nur beim
#     Streamdeck angeboten -- die Demo-GUI hat die Handsteuerung eingebaut.
#  Die letzte Auswahl wird gemerkt (~/.config/g1pilot/launcher.json); die
#  Real-Sicherheitsbestaetigung und "Images neu bauen" bewusst nicht.
#
#  Start:  python3 g1_gui.py        (start.sh startet das automatisch)
#
#  Faellt Tkinter/DISPLAY aus, benutzt start.sh weiter das Text-Menue.
# ════════════════════════════════════════════════════════════════════════
from __future__ import annotations

import json
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from pathlib import Path

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, simpledialog
except Exception as exc:  # pragma: no cover - nur wenn tkinter fehlt
    sys.stderr.write(
        "[g1_gui] Tkinter ist nicht verfuegbar (%s).\n"
        "         Bitte das klassische Text-Menue nutzen: ./start.sh --menu\n" % exc
    )
    sys.exit(2)


# ── Pfade (alles relativ zu diesem Skript, damit der Aufrufort egal ist) ──
HERE = Path(__file__).resolve().parent                       # .../g1pilot
REPO_ROOT = HERE.parent                                      # Repo-Wurzel
START_SH = HERE / "start.sh"
SCENE_DIR = REPO_ROOT / "unitree_mujoco" / "scene_editor"
SCENES_DIR = SCENE_DIR / "scenes"
LAUNCH_SH = SCENE_DIR / "launch.sh"
SETUP_SH = SCENE_DIR / "setup.sh"
SCENE_VENV = SCENE_DIR / ".venv" / "bin" / "python"

# Dokumente, die aus dem Menue heraus geoeffnet werden koennen. Vollstaendige
# Uebersicht: docs/README.md. Jedes Thema hat eine Anleitung (Anwender) und
# ein Technik-Dokument (Entwickler) -- siehe g1pilot/docs/.
DOCS_DIR = HERE / "docs"
DOCS = [
    ("Dokumentation — Uebersicht", DOCS_DIR / "README.md"),
    ("README (G1Pilot)", HERE / "README.md"),
    ("Installation & Ersteinrichtung", DOCS_DIR / "01_installation.md"),
    ("Architektur (Entwickler)", DOCS_DIR / "02_architektur.md"),
    ("Arm-Manipulation — Anleitung", DOCS_DIR / "10_arm_manipulation_anleitung.md"),
    ("Arm-Manipulation — Technik", DOCS_DIR / "11_arm_manipulation_technik.md"),
    ("Arm-API — Anleitung", DOCS_DIR / "20_arm_api_anleitung.md"),
    ("Arm-API — Technik", DOCS_DIR / "21_arm_api_technik.md"),
    ("Locomotion — Anleitung", DOCS_DIR / "30_loco_anleitung.md"),
    ("Locomotion — Technik", DOCS_DIR / "31_loco_technik.md"),
    ("Teleoperation — Anleitung", DOCS_DIR / "40_teleoperation_anleitung.md"),
    ("Teleoperation — Technik", DOCS_DIR / "41_teleoperation_technik.md"),
    ("Navigation — Anleitung", DOCS_DIR / "50_navigation_anleitung.md"),
    ("Navigation — Technik", DOCS_DIR / "51_navigation_technik.md"),
    ("Inspire-FTP-Haende — Anleitung", DOCS_DIR / "60_inspire_haende_anleitung.md"),
    ("Inspire-FTP-Haende — Technik", DOCS_DIR / "61_inspire_haende_technik.md"),
    ("Echter Roboter — Sicherheit & Ablauf", DOCS_DIR / "70_echtroboter_anleitung.md"),
]

# ── Farb-/Stilkonstanten ────────────────────────────────────────────────
BG = "#1f232b"
CARD = "#2a2f3a"
FG = "#e8eaed"
MUTED = "#9aa0aa"
ACCENT = "#4c8bf5"
GREEN = "#3ecf7a"
RED = "#e5484d"
AMBER = "#e0a13b"


# ════════════════════════════════════════════════════════════════════════
#  Hilfsfunktionen (Host-Umgebung abfragen)
# ════════════════════════════════════════════════════════════════════════
def have(cmd: str) -> bool:
    return shutil.which(cmd) is not None


def docker_ready() -> bool:
    """True, wenn 'docker' existiert und der Daemon erreichbar ist."""
    if not have("docker"):
        return False
    try:
        r = subprocess.run(
            ["docker", "info"], capture_output=True, timeout=6
        )
        return r.returncode == 0
    except Exception:
        return False


def list_scenes() -> list[Path]:
    """Alle Umgebungen aus scene_editor/scenes/*.xml (sortiert)."""
    if not SCENES_DIR.is_dir():
        return []
    return sorted(p for p in SCENES_DIR.glob("*.xml") if p.is_file())


def scene_editor_ready() -> bool:
    """True, wenn das scene_editor-virtualenv eingerichtet ist."""
    return SCENE_VENV.exists()


def detect_nics() -> list[tuple[str, str]]:
    """Physische Netzwerk-Interfaces + IPv4 ermitteln (wie start.sh, in Python).

    Rueckgabe: Liste (name, ip). Bei fehlendem 'ip'-Tool leere Liste.
    """
    nics: list[tuple[str, str]] = []
    if not have("ip"):
        return nics
    try:
        out = subprocess.run(
            ["ip", "-o", "link", "show"], capture_output=True, text=True, timeout=5
        ).stdout
    except Exception:
        return nics
    for line in out.splitlines():
        # Format: "2: eth0: <BROADCAST,...> ..."
        parts = line.split(": ")
        if len(parts) < 2:
            continue
        name = parts[1].split("@")[0].strip()
        if not name or name == "lo":
            continue
        if name.startswith(("docker", "veth", "br-", "virbr")):
            continue
        ip = ""
        try:
            addr = subprocess.run(
                ["ip", "-4", "-o", "addr", "show", name],
                capture_output=True, text=True, timeout=5,
            ).stdout
            for al in addr.splitlines():
                toks = al.split()
                if "inet" in toks:
                    ip = toks[toks.index("inet") + 1]
                    break
        except Exception:
            pass
        nics.append((name, ip))
    return nics


def open_path(path: Path) -> None:
    """Datei mit dem System-Standard oeffnen (Doku etc.)."""
    try:
        if sys.platform == "darwin":
            subprocess.Popen(["open", str(path)])
        elif os.name == "nt":
            os.startfile(str(path))  # type: ignore[attr-defined]
        else:
            subprocess.Popen(["xdg-open", str(path)])
    except Exception as exc:
        messagebox.showerror("Oeffnen fehlgeschlagen", f"{path}\n\n{exc}")


def open_url(url: str) -> bool:
    """Eine URL im System-Browser oeffnen. True bei Erfolg.

    Der Scene-Editor oeffnet den Browser nur bei einem TTY selbst — unter der
    GUI laeuft er als Subprozess (Pipe, kein TTY), daher uebernimmt das die GUI.
    Deckt Linux/Mac/Windows und WSL2 ab.
    """
    try:
        if webbrowser.open(url):
            return True
    except Exception:
        pass
    # WSL2/Windows/Linux-Fallbacks (analog start.sh)
    for exe in ("wslview", "xdg-open", "sensible-browser", "x-www-browser",
                "google-chrome", "chromium", "firefox"):
        if have(exe):
            try:
                subprocess.Popen([exe, url],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return True
            except Exception:
                pass
    if os.name == "nt":
        try:
            os.startfile(url)  # type: ignore[attr-defined]
            return True
        except Exception:
            pass
    return False


# ════════════════════════════════════════════════════════════════════════
#  Wiederverwendbare Widgets
# ════════════════════════════════════════════════════════════════════════
def section(parent, text: str) -> tk.Frame:
    """Ein abgesetzter 'Card'-Rahmen mit Ueberschrift."""
    outer = tk.Frame(parent, bg=CARD, bd=0, highlightthickness=0)
    outer.pack(fill="x", padx=14, pady=6)
    tk.Label(outer, text=text, bg=CARD, fg=ACCENT,
             font=("TkDefaultFont", 10, "bold")).pack(anchor="w", padx=12, pady=(8, 2))
    inner = tk.Frame(outer, bg=CARD)
    inner.pack(fill="x", padx=12, pady=(0, 10))
    return inner


def toggle_row(parent, label: str, var: tk.BooleanVar, hint: str = "") -> tk.Frame:
    row = tk.Frame(parent, bg=CARD)
    row.pack(fill="x", pady=2)
    cb = tk.Checkbutton(row, text=label, variable=var, bg=CARD, fg=FG,
                        selectcolor=BG, activebackground=CARD, activeforeground=FG,
                        anchor="w", font=("TkDefaultFont", 10))
    cb.pack(side="left")
    if hint:
        tk.Label(row, text=hint, bg=CARD, fg=MUTED,
                 font=("TkDefaultFont", 9)).pack(side="left", padx=8)
    return row


def field_row(parent, label: str, var: tk.StringVar, width: int = 18) -> tk.Entry:
    row = tk.Frame(parent, bg=CARD)
    row.pack(fill="x", pady=2)
    tk.Label(row, text=label, bg=CARD, fg=FG, width=22, anchor="w",
             font=("TkDefaultFont", 10)).pack(side="left")
    ent = tk.Entry(row, textvariable=var, width=width, bg=BG, fg=FG,
                   insertbackground=FG, relief="flat")
    ent.pack(side="left", padx=4, ipady=3)
    return ent


def primary_button(parent, text: str, cmd, color: str = ACCENT) -> tk.Button:
    return tk.Button(parent, text=text, command=cmd, bg=color, fg="white",
                     activebackground=color, relief="flat",
                     font=("TkDefaultFont", 11, "bold"), padx=18, pady=8)


def hint(parent, text: str) -> tk.Label:
    lbl = tk.Label(parent, text=text, bg=CARD, fg=MUTED, font=("TkDefaultFont", 9),
                   wraplength=620, justify="left")
    lbl.pack(anchor="w", pady=(2, 0))
    return lbl


def choice_rows(parent, var: tk.StringVar, choices) -> None:
    """Einzelauswahl als Radiobuttons mit Erklaerung: [(wert, titel, text)]."""
    for value, title, text in choices:
        row = tk.Frame(parent, bg=CARD)
        row.pack(fill="x", pady=1)
        tk.Radiobutton(row, text=title, value=value, variable=var, bg=CARD, fg=FG,
                       selectcolor=BG, activebackground=CARD, activeforeground=FG,
                       width=14, anchor="w",
                       font=("TkDefaultFont", 10, "bold")).pack(side="left")
        tk.Label(row, text=text, bg=CARD, fg=MUTED,
                 font=("TkDefaultFont", 9)).pack(side="left", padx=4)


def collapsible(parent, title: str) -> tk.Frame:
    """Eingeklappte Card ('Erweitert ▸'). Gibt den Inhalts-Frame zurueck."""
    outer = tk.Frame(parent, bg=CARD)
    outer.pack(fill="x", padx=14, pady=6)
    inner = tk.Frame(outer, bg=CARD)
    btn = tk.Button(outer, text=f"{title}  ▸", bg=CARD, fg=ACCENT, relief="flat", bd=0,
                    activebackground=CARD, activeforeground=ACCENT, anchor="w",
                    font=("TkDefaultFont", 10, "bold"))
    btn.pack(fill="x", padx=8, pady=(6, 6))

    def toggle():
        if inner.winfo_ismapped():
            inner.pack_forget()
            btn.configure(text=f"{title}  ▸")
        else:
            inner.pack(fill="x", padx=12, pady=(0, 10))
            btn.configure(text=f"{title}  ▾")

    btn.configure(command=toggle)
    return inner


def set_row_state(widget, enabled: bool) -> None:
    """Alle Eingabe-Widgets unterhalb von widget (de)aktivieren."""
    for child in widget.winfo_children():
        try:
            child.configure(state="normal" if enabled else "disabled")
        except tk.TclError:
            pass
        set_row_state(child, enabled)


def show_row(widget, visible: bool) -> None:
    """Zeile ein-/ausblenden. widget liegt allein in einem eigenen Container
    (siehe slot()), damit es beim Wiedereinblenden an seinem Platz landet."""
    if visible and not widget.winfo_manager():
        widget.pack(fill="x", pady=2)
    elif not visible and widget.winfo_manager():
        widget.pack_forget()


def slot(parent) -> tk.Frame:
    """Fester Platzhalter fuer eine ein-/ausblendbare Zeile."""
    f = tk.Frame(parent, bg=CARD)
    f.pack(fill="x")
    return f


# ── Zuletzt gewaehlte Optionen merken (pro Ansicht) ─────────────────────
SETTINGS_FILE = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) \
    / "g1pilot" / "launcher.json"


def load_settings(key: str) -> dict:
    try:
        return json.loads(SETTINGS_FILE.read_text()).get(key, {})
    except Exception:
        return {}


def save_settings(key: str, values: dict) -> None:
    try:
        data = json.loads(SETTINGS_FILE.read_text()) if SETTINGS_FILE.exists() else {}
    except Exception:
        data = {}
    data[key] = values
    try:
        SETTINGS_FILE.parent.mkdir(parents=True, exist_ok=True)
        SETTINGS_FILE.write_text(json.dumps(data, indent=2))
    except Exception:
        pass   # Merken ist Komfort -- Start darf daran nie scheitern


def big_button(parent, title, subtitle, color, cmd):
    """Grosse anklickbare Karte fuers Hauptmenue."""
    card = tk.Frame(parent, bg=CARD, cursor="hand2")
    card.pack(fill="x", pady=7)
    bar = tk.Frame(card, bg=color, width=6)
    bar.pack(side="left", fill="y")
    inner = tk.Frame(card, bg=CARD)
    inner.pack(side="left", fill="both", expand=True, padx=14, pady=12)
    tk.Label(inner, text=title, bg=CARD, fg=FG,
             font=("TkDefaultFont", 14, "bold")).pack(anchor="w")
    tk.Label(inner, text=subtitle, bg=CARD, fg=MUTED,
             font=("TkDefaultFont", 10)).pack(anchor="w")
    for w in (card, inner, bar, *inner.winfo_children()):
        w.bind("<Button-1>", lambda _e, c=cmd: c())
    return card


def nav_header(frame, app, title, subtitle, color=FG):
    """Kopfzeile einer Unteransicht mit '‹ Menue'-Zurueck-Button."""
    head = tk.Frame(frame, bg=BG)
    head.pack(fill="x", padx=16, pady=(12, 2))
    top = tk.Frame(head, bg=BG)
    top.pack(fill="x")
    tk.Button(top, text="‹  Menü", command=app.show_menu, bg=CARD, fg=FG,
              relief="flat", padx=10, pady=4).pack(side="left")
    tk.Label(top, text=title, bg=BG, fg=color,
             font=("TkDefaultFont", 15, "bold")).pack(side="left", padx=10)
    tk.Label(head, text=subtitle, bg=BG, fg=MUTED).pack(anchor="w", pady=(2, 6))


class ScrollableFrame(tk.Frame):
    """Ein vertikal scrollbarer Bereich. Inhalt kommt in '.body'."""

    def __init__(self, parent):
        super().__init__(parent, bg=BG)
        self._canvas = tk.Canvas(self, bg=BG, highlightthickness=0)
        vsb = ttk.Scrollbar(self, orient="vertical", command=self._canvas.yview)
        self._canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        self._canvas.pack(side="left", fill="both", expand=True)
        self.body = tk.Frame(self._canvas, bg=BG)
        self._win = self._canvas.create_window((0, 0), window=self.body, anchor="nw")
        self.body.bind(
            "<Configure>",
            lambda _e: self._canvas.configure(scrollregion=self._canvas.bbox("all")))
        self._canvas.bind(
            "<Configure>",
            lambda e: self._canvas.itemconfigure(self._win, width=e.width))
        # Mausrad nur aktiv, solange der Zeiger ueber diesem Bereich ist.
        self.bind("<Enter>", self._bind_wheel)
        self.bind("<Leave>", self._unbind_wheel)
        self.bind("<Destroy>", self._unbind_wheel)

    def _bind_wheel(self, _e=None):
        self._canvas.bind_all("<MouseWheel>", self._on_wheel)
        self._canvas.bind_all("<Button-4>", self._on_wheel)
        self._canvas.bind_all("<Button-5>", self._on_wheel)

    def _unbind_wheel(self, _e=None):
        try:
            self._canvas.unbind_all("<MouseWheel>")
            self._canvas.unbind_all("<Button-4>")
            self._canvas.unbind_all("<Button-5>")
        except tk.TclError:
            pass

    def _on_wheel(self, e):
        if e.num == 4 or getattr(e, "delta", 0) > 0:
            self._canvas.yview_scroll(-1, "units")
        elif e.num == 5 or getattr(e, "delta", 0) < 0:
            self._canvas.yview_scroll(1, "units")


# Queue-Sentinels (Kontrollmarker, unterscheidbar von Log-Zeilen/Strings):
_EOF = object()   # Reader: Prozess-Ausgabe zu Ende (Prozess beendet)
_DONE = object()  # Stop-Worker: Aufraeumen (docker down) abgeschlossen -> finalisieren
# viser-Banner des Editors ("HTTP │ http://127.0.0.1:8081"). Ist 8080 belegt
# (z.B. alter Editor), weicht viser auf 8081, 8082 ... aus -- die echte URL
# steht nur in dieser Ausgabezeile.
_EDITOR_URL_RE = re.compile(r"http://(?:127\.0\.0\.1|localhost):\d+")


# ════════════════════════════════════════════════════════════════════════
#  Konsole-View: startet einen Subprozess und zeigt dessen Ausgabe live
# ════════════════════════════════════════════════════════════════════════
class ConsoleFrame(tk.Frame):
    """View, die einen Subprozess ausfuehrt und dessen Ausgabe streamt.

    - Bleibt in der App-Registry, auch wenn man zurueck ins Menue navigiert:
      der Prozess laeuft weiter, das Log wird weiter mitgeschrieben, und man
      kann jederzeit ueber 'Laufende Prozesse' im Menue zurueckkehren.
    - 'Stoppen' beendet den Prozess (SIGTERM) + optionales Aufraeum-Kommando.
    - Thread-sicher via Queue + after()-Polling (Tkinter ist nicht threadsafe).
    """

    ephemeral = False  # nicht zerstoeren, wenn eine andere View gezeigt wird

    def __init__(self, parent, app, title: str, argv: list[str], *,
                 cwd: Path, env: dict | None = None,
                 stop_cmd: list[str] | None = None,
                 stop_note: str = "",
                 browser_url: str | None = None):
        super().__init__(parent, bg=BG)
        self.app = app
        self.view_title = title
        self._title = title
        self._argv = argv
        self._cwd = cwd
        self._env = env
        self._stop_cmd = stop_cmd
        self._stop_note = stop_note
        self._browser_url = browser_url
        self._server_url: str | None = None  # echte Editor-URL aus der Ausgabe
        self._proc: subprocess.Popen | None = None
        self._queue: queue.Queue = queue.Queue()  # Log-Strings + _EOF/_DONE-Marker
        self._stopping = False
        self._finished = False
        self._alive = True
        self._after_id = None

        # Kopfzeile: Zurueck + Titel + Status.
        head = tk.Frame(self, bg=CARD)
        head.pack(fill="x")
        tk.Button(head, text="‹  Menü", command=app.show_menu, bg=CARD, fg=FG,
                  relief="flat", padx=10, pady=6).pack(side="left", padx=6, pady=6)
        tk.Label(head, text=title, bg=CARD, fg=FG,
                 font=("TkDefaultFont", 11, "bold")).pack(side="left", padx=6)
        self._status = tk.Label(head, text="läuft…", bg=CARD, fg=AMBER,
                                font=("TkDefaultFont", 10, "bold"))
        self._status.pack(side="right", padx=12)

        # Log-Textfeld.
        wrap = tk.Frame(self, bg=BG)
        wrap.pack(fill="both", expand=True, padx=10, pady=(8, 4))
        self._text = tk.Text(wrap, bg="#12151b", fg="#d7dbe0", insertbackground=FG,
                             wrap="none", relief="flat", font=("TkFixedFont", 9),
                             state="disabled")
        yscroll = ttk.Scrollbar(wrap, orient="vertical", command=self._text.yview)
        xscroll = ttk.Scrollbar(self, orient="horizontal", command=self._text.xview)
        self._text.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self._text.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        wrap.rowconfigure(0, weight=1)
        wrap.columnconfigure(0, weight=1)
        xscroll.pack(fill="x", padx=10)

        # Fussleiste.
        foot = tk.Frame(self, bg=BG)
        foot.pack(fill="x", padx=10, pady=8)
        self._stop_btn = tk.Button(foot, text="Stoppen", command=self.stop,
                                   bg=RED, fg="white", activebackground="#c93b3f",
                                   relief="flat", font=("TkDefaultFont", 10, "bold"),
                                   padx=14, pady=6)
        self._stop_btn.pack(side="left")
        tk.Label(foot, text="»Menü« lässt den Prozess im Hintergrund weiterlaufen.",
                 bg=BG, fg=MUTED, font=("TkDefaultFont", 9)).pack(side="left", padx=12)

        self._append(f"$ {' '.join(argv)}\n\n")
        self._start()
        self._after_id = self.after(80, self._drain)
        if self._browser_url and self._proc is not None:
            threading.Thread(target=self._browser_waiter, daemon=True).start()

    # ── Status fuer die Menue-Liste ───────────────────────────────────────
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def state(self) -> str:
        if self.is_running():
            return "stoppe…" if self._stopping else "läuft"
        return "gestoppt" if self._stopping else "beendet"

    # ── intern ────────────────────────────────────────────────────────────
    def _start(self) -> None:
        try:
            self._proc = subprocess.Popen(
                self._argv, cwd=str(self._cwd), env=self._env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                bufsize=1, text=True,
            )
        except Exception as exc:
            self._append(f"[Fehler beim Start] {exc}\n")
            self._finalize(-1)
            return
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        assert self._proc and self._proc.stdout
        for line in self._proc.stdout:
            if self._browser_url and self._server_url is None:
                m = _EDITOR_URL_RE.search(line)
                if m:
                    self._server_url = m.group(0)
            self._queue.put(line)
        self._proc.wait()
        self._queue.put(_EOF)  # Prozess-Ausgabe zu Ende

    def _browser_waiter(self) -> None:
        """Wartet, bis der Editor-Webserver lauscht, und oeffnet dann den Browser.

        Bewusst eine echte HTTP-Anfrage (wie ein Browser) statt eines rohen
        TCP-Connects: der Editor-Server (viser) bedient HTTP UND WebSocket auf
        demselben Port; ein sofort geschlossener TCP-Connect saehe fuer ihn wie
        ein abgebrochener WebSocket-Handshake aus und wuerde einen (harmlosen)
        Fehler-Traceback loggen.

        Die URL kommt aus der Editor-Ausgabe (_reader), NICHT fest 8080: sonst
        oeffnet der Browser einen alten, evtl. haengenden Editor auf 8080,
        waehrend der neue auf 8081 laeuft.
        """
        deadline = time.monotonic() + 180  # max ~3 min (deckt ersten Start ab)
        while self._alive and not self._stopping and time.monotonic() < deadline:
            url = self._server_url
            if url and self._server_responds(url):
                if self._alive:
                    self._queue.put(f"\n[GUI] Oeffne Editor im Browser: {url}\n")
                    open_url(url)
                return
            if self._proc is not None and self._proc.poll() is not None:
                return  # Prozess beendet, ohne je zu lauschen
            time.sleep(0.5)

    @staticmethod
    def _server_responds(url: str) -> bool:
        """True, sobald der HTTP-Server irgendeine Antwort liefert."""
        try:
            urllib.request.urlopen(url, timeout=1).close()
            return True
        except urllib.error.HTTPError:
            return True  # Server antwortet (nur mit Fehlerstatus) -> laeuft
        except (urllib.error.URLError, OSError):
            return False  # noch nicht erreichbar

    def _drain(self) -> None:
        if not self._alive:
            return
        try:
            while True:
                item = self._queue.get_nowait()
                if item is _EOF:
                    # Prozess-Ausgabe endet. Bei benutzerinitiiertem Stop NICHT
                    # sofort finalisieren — erst wartet noch das Aufraeum-Kommando
                    # (docker down); der Stop-Worker meldet danach _DONE. So gilt
                    # "beendet" wirklich erst, wenn MuJoCo/Container zu sind.
                    if not self._stopping:
                        self._finalize()
                        return
                elif item is _DONE:
                    self._finalize()
                    return
                else:
                    self._append(item)
        except queue.Empty:
            pass
        except tk.TclError:
            return  # View wurde zwischenzeitlich zerstoert
        self._after_id = self.after(80, self._drain)

    def _append(self, text: str) -> None:
        self._text.configure(state="normal")
        self._text.insert("end", text)
        # Speicher begrenzen: hoechstens ~4000 Zeilen behalten.
        last = int(self._text.index("end-1c").split(".")[0])
        if last > 4000:
            self._text.delete("1.0", f"{last - 4000}.0")
        self._text.see("end")
        self._text.configure(state="disabled")

    def _finalize(self, rc: int | None = None) -> None:
        self._finished = True
        if rc is None:
            rc = self._proc.returncode if self._proc else -1
        user_stopped = self._stopping
        if user_stopped:
            self._status.configure(text="gestoppt", fg=MUTED)
        elif rc == 0:
            self._status.configure(text="beendet (ok)", fg=GREEN)
        else:
            self._status.configure(text=f"beendet (Code {rc})", fg=RED)
        self._stop_btn.configure(state="disabled")
        # Sauber (ab)geschlossen -> App entscheidet ueber Auto-Rueckkehr/Refresh.
        self.app.notify_finished(self, user_stopped=user_stopped)

    def terminate_editor(self) -> None:
        """Beim Schliessen des Launchers: Web-Editor mitbeenden.

        Sonst laeuft er verwaist weiter (Ausgabe-Pipe ohne Leser -> haengt),
        blockiert Port 8080 und der naechste Editor weicht auf 8081 aus.
        Docker-Stacks sind davon nicht betroffen (die laufen bewusst weiter).
        """
        if self._browser_url and self.is_running():
            try:
                self._proc.terminate()
                self._proc.wait(timeout=3)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass

    def stop(self) -> None:
        if self._stopping or not self.is_running():
            return
        self._stopping = True
        self._status.configure(text="stoppe…", fg=AMBER)
        self._stop_btn.configure(state="disabled")
        self._append("\n[Stoppen angefordert]\n")

        def worker():
            if self._proc and self._proc.poll() is None:
                try:
                    self._proc.terminate()
                    self._proc.wait(timeout=12)
                except Exception:
                    try:
                        self._proc.kill()
                    except Exception:
                        pass
            if self._stop_cmd:
                self._queue.put(f"\n$ {' '.join(self._stop_cmd)}\n")
                try:
                    r = subprocess.run(self._stop_cmd, cwd=str(self._cwd),
                                       capture_output=True, text=True, timeout=90)
                    self._queue.put(r.stdout + r.stderr)
                except Exception as exc:
                    self._queue.put(f"[down-Fehler] {exc}\n")
            self._queue.put(_DONE)

        threading.Thread(target=worker, daemon=True).start()

    def destroy(self) -> None:
        self._alive = False
        if self._after_id is not None:
            try:
                self.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None
        super().destroy()


# ════════════════════════════════════════════════════════════════════════
#  Gemeinsame Bausteine der beiden Start-Ansichten
# ════════════════════════════════════════════════════════════════════════
GUI_CHOICES = [
    ("demo", "Demo-GUI",
     "Vorführung: Gehen / Greifen, Arme + Hände, Stationen anfahren"),
    ("streamdeck", "Streamdeck",
     "Entwicklung: alle Einzelfunktionen als Kacheln"),
]


def gui_section(parent, var: tk.StringVar) -> None:
    s = section(parent, "Bedienoberfläche")
    choice_rows(s, var, GUI_CHOICES)


def hand_gui_rows(parent, var: tk.BooleanVar):
    """Browser-Oberflaechen der Hand-Bridge (nur fuer den Streamdeck relevant)
    bzw. Hinweis, dass die Demo-GUI die Handsteuerung eingebaut hat.
    -> (zeile_browser, zeile_hinweis), beide per show_row() umschaltbar."""
    s = slot(parent)
    row = toggle_row(s, "Hand-Oberflächen im Browser öffnen", var,
                     "Controller + Taktil-Viewer der Hand-Bridge")
    s2 = slot(parent)
    note = tk.Label(s2, text="Handsteuerung ist in der Demo-GUI eingebaut (Greifen → Hände).",
                    bg=CARD, fg=MUTED, font=("TkDefaultFont", 9), anchor="w")
    return row, note


def start_footer(frame, app, text: str, cmd, color: str) -> tk.Button:
    foot = tk.Frame(frame, bg=BG)
    foot.pack(fill="x", side="bottom", padx=16, pady=12)
    btn = primary_button(foot, text, cmd, color)
    btn.pack(side="left")
    tk.Button(foot, text="Abbrechen", command=app.show_menu, bg=CARD, fg=FG,
              relief="flat", padx=12, pady=8).pack(side="right")
    return btn


# ════════════════════════════════════════════════════════════════════════
#  View: Simulation starten
# ════════════════════════════════════════════════════════════════════════
class SimFrame(tk.Frame):
    """Sim-Start. Aufbau in der Reihenfolge der Entscheidungen:
    Umgebung -> Bedienoberflaeche -> Ausstattung -> Erweitert.
    Die Auswahl wird gemerkt (settings), Defaults siehe start.sh (Sim-Zweig)."""

    ephemeral = True
    view_title = "Simulation"
    DEFAULT_ENV = "Standard (scene.xml)"

    def __init__(self, parent, app):
        super().__init__(parent, bg=BG)
        self.app = app
        nav_header(self, app, "Simulation",
                   "MuJoCo + Whole-Body-Policy — gefahrlos testen.")
        start_footer(self, app, "Simulation starten", self._start, GREEN)

        body = ScrollableFrame(self)
        body.pack(fill="both", expand=True)
        b = body.body

        st = load_settings("sim")
        scenes = [p.stem for p in list_scenes()]
        env = st.get("env", "")
        self.v_env = tk.StringVar(value=env if env in scenes else self.DEFAULT_ENV)
        self.v_gui = tk.StringVar(value=st.get("gui", "demo"))
        self.v_hands = tk.BooleanVar(value=st.get("hands", True))
        self.v_nav = tk.BooleanVar(value=st.get("nav", False))
        self.v_rviz = tk.BooleanVar(value=st.get("rviz", False))
        self._rviz_own = None   # eigene RViz-Wahl, solange Navigation RViz erzwingt
        self.v_open_guis = tk.BooleanVar(value=st.get("open_guis", False))
        self.v_rt = tk.StringVar(value=st.get("rt", "1.0"))
        self.v_rebuild = tk.BooleanVar(value=False)   # bewusst nie gemerkt

        # 1) Umgebung
        s = section(b, "Umgebung")
        ttk.Combobox(s, textvariable=self.v_env, values=[self.DEFAULT_ENV, *scenes],
                     state="readonly", width=44).pack(anchor="w", pady=2)
        hint(s, "Der G1 bleibt gleich, nur die Welt wechselt. Anlegen und bearbeiten: "
                "Startmenü → Umgebungen.")

        # 2) Bedienoberflaeche
        gui_section(b, self.v_gui)

        # 3) Ausstattung
        s = section(b, "Ausstattung")
        toggle_row(s, "Inspire-Hände", self.v_hands,
                   "Finger steuerbar, Kraftsensoren; sonst starre Hände")
        hand_box = tk.Frame(s, bg=CARD)
        hand_box.pack(fill="x", padx=(24, 0))
        self._open_row, self._hand_hint = hand_gui_rows(hand_box, self.v_open_guis)
        toggle_row(s, "Navigation", self.v_nav,
                   "Planer + Stationen / AUTO NAV in der Demo-GUI")
        self._rviz_row = toggle_row(s, "RViz", self.v_rviz,
                                    "Zusatzfenster mit TF/Markern; MuJoCo-Fenster kommt immer")
        for v in (self.v_hands, self.v_gui, self.v_nav):
            v.trace_add("write", lambda *_: self._sync())

        # 4) Erweitert
        s = collapsible(b, "Erweitert")
        field_row(s, "Sim-Tempo (1.0 = Echtzeit)", self.v_rt, width=8)
        hint(s, "Kleiner als 1.0 = Zeitlupe (Obergrenze; langsame PCs laufen ohnehin langsamer).")
        toggle_row(s, "Docker-Images neu bauen", self.v_rebuild,
                   "nur nach Änderungen an Dockerfiles/Abhängigkeiten")
        self._sync()

    def _sync(self) -> None:
        hands = self.v_hands.get()
        streamdeck = self.v_gui.get() == "streamdeck"
        # Browser-Oberflaechen nur fuer den Streamdeck -- die Demo-GUI hat sie eingebaut.
        show_row(self._open_row, hands and streamdeck)
        show_row(self._hand_hint, hands and not streamdeck)
        # Navigation braucht RViz (Karte/Ziel-Werkzeug leben dort) -> erzwungen;
        # ohne Navigation wieder die eigene RViz-Wahl.
        nav = self.v_nav.get()
        set_row_state(self._rviz_row, not nav)
        if nav and self._rviz_own is None:
            self._rviz_own = self.v_rviz.get()
            self.v_rviz.set(True)
        elif not nav and self._rviz_own is not None:
            self.v_rviz.set(self._rviz_own)
            self._rviz_own = None

    def _start(self) -> None:
        rt = self.v_rt.get().strip() or "1.0"
        try:
            float(rt)
        except ValueError:
            messagebox.showerror("Ungültig", "Sim-Tempo muss eine Zahl sein (z.B. 1.0).",
                                 parent=self)
            return
        env_name = self.v_env.get()
        env_name = "" if env_name == self.DEFAULT_ENV else env_name
        hands = self.v_hands.get()
        gui = self.v_gui.get()
        save_settings("sim", {
            "env": env_name, "gui": gui, "hands": hands, "nav": self.v_nav.get(),
            "rviz": self.v_rviz.get() if self._rviz_own is None else self._rviz_own,
            "open_guis": self.v_open_guis.get(), "rt": rt,
        })

        env = os.environ.copy()
        env["G1_MODE"] = "sim"
        env["G1_ENV"] = env_name
        env["G1_GUI"] = gui
        env["G1_INSPIRE_HANDS"] = "1" if hands else "0"
        env["OPEN_GUIS"] = "true" if (hands and gui == "streamdeck"
                                      and self.v_open_guis.get()) else "false"
        env["G1_ENABLE_NAV"] = "1" if self.v_nav.get() else "0"
        env["USE_RVIZ"] = "true" if self.v_rviz.get() else "false"
        env["SIM_REALTIME_FACTOR"] = rt

        argv = ["bash", str(START_SH), "--yes"]
        if self.v_rebuild.get():
            argv.append("--build")
        stop_cmd = ["docker", "compose", "--profile", "sim", "down", "--remove-orphans"]
        self.app.start_process("Simulation", argv, cwd=HERE, env=env,
                               stop_cmd=stop_cmd,
                               stop_note="Stoppt den Sim-Stack (docker compose down).")


# ════════════════════════════════════════════════════════════════════════
#  View: Echten Roboter starten
# ════════════════════════════════════════════════════════════════════════
class RealFrame(tk.Frame):
    """Real-Start: Verbindung -> Bedienoberflaeche -> Ausstattung -> Geh-Limits
    -> Erweitert -> Sicherheits-Bestaetigung. Die Bestaetigung wird NIE gemerkt."""

    ephemeral = True
    view_title = "Echter Roboter"

    def __init__(self, parent, app):
        super().__init__(parent, bg=BG)
        self.app = app
        nav_header(self, app, "Echter Roboter",
                   "G1 per LAN: Unitree-Loco + Arme + Hände. Der Roboter bewegt sich!", RED)
        self.start_btn = start_footer(self, app, "Echten Roboter starten", self._start, RED)
        self.start_btn.configure(state="disabled")

        body = ScrollableFrame(self)
        body.pack(fill="both", expand=True)
        b = body.body

        st = load_settings("real")
        self.v_gui = tk.StringVar(value=st.get("gui", "demo"))
        self.v_hands = tk.BooleanVar(value=st.get("hands", True))
        self.v_left = tk.StringVar(value=st.get("left", "192.168.123.210"))
        self.v_right = tk.StringVar(value=st.get("right", "192.168.123.211"))
        self.v_open_guis = tk.BooleanVar(value=st.get("open_guis", False))
        self.v_rviz = tk.BooleanVar(value=st.get("rviz", True))
        self.v_vx = tk.StringVar(value=st.get("vx", "0.4"))
        self.v_vy = tk.StringVar(value=st.get("vy", "0.3"))
        self.v_vyaw = tk.StringVar(value=st.get("vyaw", "0.4"))
        self.v_port = tk.StringVar(value=st.get("port", "6000"))
        self.v_joy = tk.StringVar(value=st.get("joy", "Wireless Controller"))
        self.v_lidar = tk.BooleanVar(value=False)
        self.v_rebuild = tk.BooleanVar(value=False)
        self.v_confirm = tk.BooleanVar(value=False)

        # 1) Verbindung
        s = section(b, "Verbindung zum G1")
        nics = detect_nics()
        values = []
        default = ""
        self._iface_map: dict[str, str] = {}
        for name, ip in nics:
            label = f"{name}  ({ip or 'keine IPv4'})"
            values.append(label)
            self._iface_map[label] = name
            if ip.startswith("192.168.123.") or (not default and name == st.get("iface")):
                default = label
        self.v_iface = tk.StringVar(value=default or (values[0] if values else st.get("iface", "")))
        row = tk.Frame(s, bg=CARD)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Netzwerk-Interface", bg=CARD, fg=FG, width=22, anchor="w",
                 font=("TkDefaultFont", 10)).pack(side="left")
        ttk.Combobox(row, textvariable=self.v_iface, values=values, width=34).pack(side="left")
        self._lan = tk.Label(s, text="", bg=CARD, font=("TkDefaultFont", 9))
        self._lan.pack(anchor="w", pady=(4, 0))
        self.v_iface.trace_add("write", lambda *_: self._sync_lan(nics))
        self._sync_lan(nics)

        # 2) Bedienoberflaeche
        gui_section(b, self.v_gui)

        # 3) Ausstattung
        s = section(b, "Ausstattung")
        toggle_row(s, "Inspire-Hände (Modbus TCP im Roboter-LAN)", self.v_hands)
        self._hand_box = tk.Frame(s, bg=CARD)
        self._hand_box.pack(fill="x", padx=(24, 0))
        field_row(self._hand_box, "IP linke Hand", self.v_left)
        field_row(self._hand_box, "IP rechte Hand", self.v_right)
        self._open_row, self._hand_hint = hand_gui_rows(self._hand_box, self.v_open_guis)
        toggle_row(s, "RViz", self.v_rviz, "empfohlen: Arm-Marker + Zustand sichtbar")
        for v in (self.v_hands, self.v_gui):
            v.trace_add("write", lambda *_: self._sync())

        # 4) Geh-Limits
        s = section(b, "Geh-Limits (konservativ für die ersten Tests)")
        row = tk.Frame(s, bg=CARD)
        row.pack(fill="x", pady=2)
        for lbl, var, unit in (("vorwärts", self.v_vx, "m/s"), ("seitlich", self.v_vy, "m/s"),
                               ("drehen", self.v_vyaw, "rad/s")):
            tk.Label(row, text=lbl, bg=CARD, fg=FG).pack(side="left", padx=(0, 4))
            tk.Entry(row, textvariable=var, width=6, bg=BG, fg=FG,
                     insertbackground=FG, relief="flat").pack(side="left", ipady=2)
            tk.Label(row, text=unit, bg=CARD, fg=MUTED).pack(side="left", padx=(2, 16))

        # 5) Erweitert
        s = collapsible(b, "Erweitert")
        field_row(s, "Modbus-Port der Hände", self.v_port, width=8)
        field_row(s, "Gamepad-Name (evdev)", self.v_joy, width=24)
        toggle_row(s, "LiDAR + Navigation", self.v_lidar,
                   "experimentell: braucht Livox MID360 und das volle Image (make real-full)")
        toggle_row(s, "Docker-Images neu bauen", self.v_rebuild,
                   "nur nach Änderungen an Dockerfiles/Abhängigkeiten")

        # 6) Sicherheit
        gate = tk.Frame(b, bg="#3a2326", bd=0)
        gate.pack(fill="x", padx=14, pady=8)
        tk.Label(gate, text="ACHTUNG: ECHTER ROBOTER", bg="#3a2326", fg=RED,
                 font=("TkDefaultFont", 11, "bold")).pack(anchor="w", padx=12, pady=(8, 0))
        tk.Label(gate,
                 text=("Kein Auto-Start: der Roboter bewegt sich erst, wenn er in der\n"
                       "Bedienoberfläche gestartet wird (Demo-GUI: »Roboter starten«,\n"
                       "Streamdeck: START). NOT-HALT = Damp = Roboter sackt zusammen (sichern!).\n"
                       "Checkliste vorher: g1pilot/docs/70_echtroboter_anleitung.md"),
                 bg="#3a2326", fg=FG, justify="left",
                 font=("TkDefaultFont", 9)).pack(anchor="w", padx=12, pady=(2, 6))
        tk.Checkbutton(gate,
                       text="Ich habe die Sicherheitshinweise gelesen und der Bereich ist frei.",
                       variable=self.v_confirm, bg="#3a2326", fg=FG, selectcolor=BG,
                       activebackground="#3a2326", activeforeground=FG,
                       command=self._sync_confirm).pack(anchor="w", padx=12, pady=(0, 8))
        self._sync()

    def _iface(self) -> str:
        label = self.v_iface.get().strip()
        return self._iface_map.get(label, label.split()[0] if label else "")

    def _sync_lan(self, nics) -> None:
        ip = dict(nics).get(self._iface(), "")
        if ip.startswith("192.168.123."):
            self._lan.configure(text=f"Im Roboter-LAN ({ip}).", fg=GREEN)
        else:
            self._lan.configure(text="Keine 192.168.123.x-Adresse auf diesem Interface — "
                                     "Kabel/IP prüfen (Roboter-LAN).", fg=AMBER)

    def _sync(self) -> None:
        hands = self.v_hands.get()
        streamdeck = self.v_gui.get() == "streamdeck"
        set_row_state(self._hand_box, hands)
        show_row(self._open_row, streamdeck)
        show_row(self._hand_hint, not streamdeck)

    def _sync_confirm(self) -> None:
        self.start_btn.configure(state="normal" if self.v_confirm.get() else "disabled")

    def _start(self) -> None:
        if not self.v_confirm.get():
            return
        iface = self._iface()
        if not iface:
            messagebox.showerror("Interface fehlt",
                                 "Bitte ein Netzwerk-Interface wählen oder eintippen.",
                                 parent=self)
            return
        hands = self.v_hands.get()
        gui = self.v_gui.get()
        vals = {
            "iface": iface, "gui": gui, "hands": hands,
            "left": self.v_left.get().strip() or "192.168.123.210",
            "right": self.v_right.get().strip() or "192.168.123.211",
            "port": self.v_port.get().strip() or "6000",
            "open_guis": self.v_open_guis.get(), "rviz": self.v_rviz.get(),
            "vx": self.v_vx.get().strip() or "0.4",
            "vy": self.v_vy.get().strip() or "0.3",
            "vyaw": self.v_vyaw.get().strip() or "0.4",
            "joy": self.v_joy.get().strip() or "Wireless Controller",
        }
        save_settings("real", vals)

        env = os.environ.copy()
        env["G1_MODE"] = "real"
        env["G1_REAL_CONFIRM"] = "1"  # Sicherheits-Gate von start.sh im --yes-Modus
        env["ROBOT_INTERFACE"] = iface
        env["G1_GUI"] = gui
        env["G1_INSPIRE_HANDS"] = "1" if hands else "0"
        env["OPEN_GUIS"] = "true" if (hands and gui == "streamdeck"
                                      and vals["open_guis"]) else "false"
        env["G1_HAND_LEFT_HOST"] = vals["left"]
        env["G1_HAND_RIGHT_HOST"] = vals["right"]
        env["G1_HAND_PORT"] = vals["port"]
        env["USE_RVIZ"] = "true" if vals["rviz"] else "false"
        env["G1_ENABLE_LIDAR"] = "1" if self.v_lidar.get() else "0"
        env["JOYSTICK_NAME"] = vals["joy"]
        env["G1_MAX_VX"] = vals["vx"]
        env["G1_MAX_VY"] = vals["vy"]
        env["G1_MAX_VYAW"] = vals["vyaw"]

        argv = ["bash", str(START_SH), "--yes"]
        if self.v_rebuild.get():
            argv.append("--build")
        stop_cmd = ["docker", "compose", "--profile", "real", "down", "--remove-orphans"]
        self.app.start_process("Echter Roboter", argv, cwd=HERE, env=env,
                               stop_cmd=stop_cmd,
                               stop_note="Stoppt den Real-Stack (docker compose down).")


# ════════════════════════════════════════════════════════════════════════
#  View: Umgebungen bearbeiten (scene_editor)
# ════════════════════════════════════════════════════════════════════════
class SceneFrame(tk.Frame):
    ephemeral = True
    view_title = "Umgebungen"

    def __init__(self, parent, app):
        super().__init__(parent, bg=BG)
        self.app = app
        nav_header(self, app, "Umgebungen",
                   "Szenen aus scene_editor/scenes/ — dieselben, die beim Sim-Start "
                   "wählbar sind.")
        body = ScrollableFrame(self)
        body.pack(fill="both", expand=True)
        self._body = body.body
        # Wie beim Sim-Start (gleiche Voreinstellung bzw. letzte Sim-Auswahl).
        self.v_hands = tk.BooleanVar(value=load_settings("sim").get("hands", True))
        if not scene_editor_ready():
            self._show_setup_needed()
            return
        self._build_editor_ui()

    def _show_setup_needed(self) -> None:
        box = section(self._body, "Einmaliges Setup nötig")
        tk.Label(box, text="Der Scene-Editor braucht ein virtualenv (scene_editor/.venv).\n"
                           "Das wird einmalig eingerichtet (Internet nötig, danach gecacht).",
                 bg=CARD, fg=FG, justify="left").pack(anchor="w", pady=(0, 8))
        primary_button(box, "Setup jetzt ausführen", self._run_setup, ACCENT).pack(anchor="w")

    def _run_setup(self) -> None:
        self.app.start_process("Scene-Editor Setup", ["bash", str(SETUP_SH)],
                               cwd=SCENE_DIR, env=os.environ.copy(),
                               stop_note="Bricht das Setup ab.")

    def _build_editor_ui(self) -> None:
        s = section(self._body, "Vorhandene Umgebungen")
        listwrap = tk.Frame(s, bg=CARD)
        listwrap.pack(fill="both", expand=True)
        self.listbox = tk.Listbox(listwrap, height=8, bg=BG, fg=FG,
                                  selectbackground=ACCENT, relief="flat",
                                  activestyle="none", exportselection=False)
        sb = ttk.Scrollbar(listwrap, orient="vertical", command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=sb.set)
        self.listbox.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self._reload_scenes()
        tk.Button(s, text="Liste aktualisieren", command=self._reload_scenes,
                  bg=BG, fg=FG, relief="flat", padx=10, pady=4).pack(anchor="w", pady=(6, 0))

        act = section(self._body, "Aktion für die gewählte Umgebung")
        grid = tk.Frame(act, bg=CARD)
        grid.pack(fill="x")
        for col, (text, cmd, color) in enumerate((
                ("Im Editor bearbeiten", "edit", ACCENT),
                ("Mit G1 ansehen", "with-g1", GREEN),
                ("Nur Umgebung ansehen", "view", BG))):
            primary_button(grid, text, lambda c=cmd: self._run_scene(c), color).grid(
                row=0, column=col, padx=4, pady=4, sticky="ew")
            grid.columnconfigure(col, weight=1)
        toggle_row(act, "Bei »Mit G1 ansehen« die Inspire-Hände laden", self.v_hands)

        new = section(self._body, "Neue Umgebung")
        newrow = tk.Frame(new, bg=CARD)
        newrow.pack(fill="x")
        primary_button(newrow, "Leere Umgebung im Editor",
                       lambda: self._run_cmd(["new"], "Neue Umgebung", browser=True),
                       ACCENT).pack(side="left", padx=(0, 6))
        primary_button(newrow, "Aus Text-Prompt generieren", self._run_prompt,
                       BG).pack(side="left")
        tk.Label(new, text="Der Editor öffnet einen lokalen Webserver (http://127.0.0.1:8080) "
                           "im Browser. Export landet automatisch in scenes/.",
                 bg=CARD, fg=MUTED, font=("TkDefaultFont", 9),
                 wraplength=560, justify="left").pack(anchor="w", pady=(6, 0))

    def _reload_scenes(self) -> None:
        self.listbox.delete(0, "end")
        self._scenes = list_scenes()
        if not self._scenes:
            self.listbox.insert("end", "(noch keine Umgebungen — 'Leere Umgebung' anlegen)")
            self.listbox.configure(state="disabled")
            return
        self.listbox.configure(state="normal")
        for p in self._scenes:
            self.listbox.insert("end", p.stem)
        self.listbox.selection_set(0)

    def _selected_scene(self):
        if not getattr(self, "_scenes", None):
            return None
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showinfo("Keine Auswahl", "Bitte zuerst eine Umgebung wählen.",
                                parent=self)
            return None
        return self._scenes[sel[0]]

    def _run_scene(self, cmd: str) -> None:
        scene = self._selected_scene()
        if scene is None:
            return
        env = os.environ.copy()
        env["G1_INSPIRE_HANDS"] = "1" if self.v_hands.get() else "0"
        title = {"edit": "Editor", "view": "Viewer", "with-g1": "Viewer + G1"}[cmd]
        # 'edit' startet den Web-Editor -> Browser oeffnen; view/with-g1 sind
        # native MuJoCo-Fenster.
        self._run_cmd([cmd, str(scene)], f"{title}: {scene.stem}", env=env,
                      browser=(cmd == "edit"))

    def _run_prompt(self) -> None:
        text = simpledialog.askstring(
            "Umgebung aus Text",
            "Beschreibe die Umgebung (Englisch funktioniert am besten):",
            parent=self)
        if not text:
            return
        self._run_cmd(["prompt", text], "Umgebung generieren", browser=True)

    def _run_cmd(self, args: list[str], title: str, env: dict | None = None,
                 browser: bool = False) -> None:
        self.app.start_process(f"Scene-Editor — {title}",
                               ["bash", str(LAUNCH_SH), *args],
                               cwd=SCENE_DIR, env=env or os.environ.copy(),
                               stop_note="Beendet den Editor/Viewer-Prozess.",
                               browser_url="http://127.0.0.1:8080" if browser else None)


# ════════════════════════════════════════════════════════════════════════
#  View: Hauptmenue
# ════════════════════════════════════════════════════════════════════════
class MenuFrame(tk.Frame):
    ephemeral = True
    view_title = "Startmenü"

    def __init__(self, parent, app):
        super().__init__(parent, bg=BG)
        self.app = app

        head = tk.Frame(self, bg=BG)
        head.pack(fill="x", padx=24, pady=(20, 4))
        tk.Label(head, text="G1 Robot Control", bg=BG, fg=FG,
                 font=("TkDefaultFont", 20, "bold")).pack(anchor="w")
        tk.Label(head, text="Was soll gestartet werden?",
                 bg=BG, fg=MUTED, font=("TkDefaultFont", 11)).pack(anchor="w")

        body = ScrollableFrame(self)
        body.pack(fill="both", expand=True)
        b = body.body

        wrap = tk.Frame(b, bg=BG)
        wrap.pack(fill="x", padx=24, pady=8)
        big_button(wrap, "Simulation",
                   "MuJoCo + Whole-Body-Policy — gefahrlos testen", GREEN, app.open_sim)
        big_button(wrap, "Echter Roboter",
                   "G1 per LAN — Loco + Arme + Hände (der Roboter bewegt sich!)", RED,
                   app.open_real)
        big_button(wrap, "Umgebungen",
                   "Szenen anlegen, bearbeiten und mit dem G1 ansehen", ACCENT, app.open_scenes)

        # Laufende / letzte Prozesse (damit nichts 'verloren' geht).
        if app.consoles:
            s = section(b, "Prozesse")
            for c in app.consoles:
                row = tk.Frame(s, bg=CARD)
                row.pack(fill="x", pady=2)
                dot = GREEN if c.is_running() else MUTED
                tk.Label(row, text="●", bg=CARD, fg=dot).pack(side="left", padx=(0, 6))
                tk.Label(row, text=f"{c.view_title}  ({c.state()})", bg=CARD, fg=FG,
                         anchor="w").pack(side="left")
                tk.Button(row, text="Ansehen", command=lambda x=c: app.show_console(x),
                          bg=BG, fg=FG, relief="flat", padx=10, pady=3).pack(side="right", padx=4)
                if c.is_running():
                    tk.Button(row, text="Stoppen", command=lambda x=c: self._stop(x),
                              bg=RED, fg="white", relief="flat", padx=10,
                              pady=3).pack(side="right", padx=4)
                else:
                    tk.Button(row, text="Entfernen", command=lambda x=c: app.remove_console(x),
                              bg=BG, fg=MUTED, relief="flat", padx=10,
                              pady=3).pack(side="right", padx=4)

        # Sekundaerzeile.
        sec = tk.Frame(b, bg=BG)
        sec.pack(fill="x", padx=24, pady=(6, 12))
        tk.Button(sec, text="Alle Stacks stoppen", command=app.stop_all,
                  bg=CARD, fg=FG, relief="flat", padx=12, pady=6).pack(side="left")
        docs_btn = tk.Menubutton(sec, text="Dokumentation  ▾", bg=CARD, fg=FG,
                                 activebackground=CARD, activeforeground=FG,
                                 relief="flat", padx=12, pady=6)
        docmenu = tk.Menu(docs_btn, tearoff=0, bg=CARD, fg=FG,
                          activebackground=ACCENT, activeforeground="white")
        for label, path in DOCS:
            docmenu.add_command(label=label, command=lambda p=path: open_path(p))
        docs_btn["menu"] = docmenu
        docs_btn.pack(side="left", padx=8)
        tk.Button(sec, text="Beenden", command=app._quit, bg=CARD, fg=FG,
                  relief="flat", padx=12, pady=6).pack(side="right")

    def _stop(self, console) -> None:
        console.stop()
        # Menue neu aufbauen, damit der Status aktualisiert wird.
        self.app.show_menu()


# ════════════════════════════════════════════════════════════════════════
#  App: ein Fenster, Views werden im Inhalt ausgetauscht (Router)
# ════════════════════════════════════════════════════════════════════════
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("G1 — Startmenü")
        self.configure(bg=BG)
        self.geometry("900x760")
        self.minsize(700, 600)
        # Kein heller Fokus-Rahmen um Knoepfe/Kaestchen (passt nicht zum dunklen Stil).
        self.option_add("*highlightThickness", 0)

        self._quitting = False
        self._current = None
        self.consoles: list[ConsoleFrame] = []

        # Statuszeile unten (bleibt ueber allen Views sichtbar).
        self.status = tk.Label(self, text="", bg="#171a20", fg=MUTED, anchor="w",
                               font=("TkFixedFont", 9))
        self.status.pack(fill="x", side="bottom")

        # Container fuellt den Rest; hier werden die Views ein-/ausgeblendet.
        self.container = tk.Frame(self, bg=BG)
        self.container.pack(fill="both", expand=True)

        self.protocol("WM_DELETE_WINDOW", self._quit)
        self.show_menu()
        self.after(200, self.refresh_status)

    # ── Router ────────────────────────────────────────────────────────────
    def show(self, frame) -> None:
        if self._current is frame:
            return
        if self._current is not None:
            self._current.pack_forget()
            if getattr(self._current, "ephemeral", False):
                self._current.destroy()
        self._current = frame
        frame.pack(fill="both", expand=True)
        self.title(f"G1 — {getattr(frame, 'view_title', 'Startmenü')}")

    def show_menu(self) -> None:
        self.show(MenuFrame(self.container, self))

    def open_sim(self) -> None:
        if self._docker_guard():
            self.show(SimFrame(self.container, self))

    def open_real(self) -> None:
        if self._docker_guard():
            self.show(RealFrame(self.container, self))

    def open_scenes(self) -> None:
        self.show(SceneFrame(self.container, self))

    def show_console(self, console: ConsoleFrame) -> None:
        if console not in self.consoles:
            self.consoles.append(console)
        self.show(console)

    def notify_finished(self, console: ConsoleFrame, *, user_stopped: bool) -> None:
        """Ein Prozess ist fertig (sauber gestoppt oder von selbst beendet).

        - Wird er gerade angesehen UND wurde vom Nutzer gestoppt (MuJoCo/Container
          sind dann sauber zu): automatisch zurueck ins Startmenue.
        - Ist gerade das Menue offen: nur die Prozessliste auffrischen.
        - Sonst (andere View sichtbar): nichts tun, nicht wegreissen. Von selbst
          beendete/abgestuerzte Prozesse bleiben in ihrer View, damit Fehler/Log
          lesbar bleiben.
        """
        if self._current is console:
            if user_stopped:
                self.show_menu()
        elif isinstance(self._current, MenuFrame):
            self.show_menu()

    def start_process(self, title, argv, *, cwd, env=None, stop_cmd=None,
                      stop_note="", browser_url=None) -> ConsoleFrame:
        console = ConsoleFrame(self.container, self, title, argv, cwd=cwd, env=env,
                               stop_cmd=stop_cmd, stop_note=stop_note,
                               browser_url=browser_url)
        self.consoles.append(console)
        self.show(console)
        return console

    def remove_console(self, console: ConsoleFrame) -> None:
        if console in self.consoles:
            self.consoles.remove(console)
        if self._current is console:
            self._current = None
        console.destroy()
        self.show_menu()

    # ── Aktionen ──────────────────────────────────────────────────────────
    def _docker_guard(self) -> bool:
        if docker_ready():
            return True
        return messagebox.askyesno(
            "Docker nicht erreichbar",
            "Docker scheint nicht zu laufen oder ist nicht installiert.\n"
            "Trotzdem fortfahren?")

    def stop_all(self) -> None:
        if not messagebox.askyesno("Stoppen",
                                   "Sim- UND Real-Stack stoppen (docker compose down)?"):
            return
        cmd = ["bash", "-c",
               "docker compose --profile sim down --remove-orphans; "
               "docker compose --profile real down --remove-orphans"]
        self.start_process("Stacks stoppen", cmd, cwd=HERE, env=os.environ.copy())

    def _quit(self) -> None:
        # Ein Fenster, ein Schliessen: beendet den Launcher komplett. Bereits
        # gestartete Stacks laufen unabhaengig im Docker weiter (bewusst).
        # Nur der Web-Editor wird mitbeendet (siehe ConsoleFrame.terminate_editor).
        self._quitting = True
        for console in list(self.consoles):
            console.terminate_editor()
        try:
            self.destroy()
        finally:
            # Garantiert beenden, auch wenn noch Hintergrund-Threads/Subprozesse
            # (Log-Reader, docker compose) haengen.
            os._exit(0)

    def refresh_status(self) -> None:
        editor = "Scene-Editor: bereit" if scene_editor_ready() else "Scene-Editor: Setup nötig"
        scenes = f"Umgebungen: {len(list_scenes())}"
        self.status.configure(text=f"  Docker: prüfe…   |   {editor}   |   {scenes}")

        def worker():
            docker = "Docker: ok" if docker_ready() else "Docker: nicht erreichbar"
            text = f"  {docker}   |   {editor}   |   {scenes}"
            if self._quitting:
                return
            try:
                self.after(0, lambda: self.status.configure(text=text))
            except (tk.TclError, RuntimeError):
                pass

        threading.Thread(target=worker, daemon=True).start()


def main() -> int:
    app = App()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

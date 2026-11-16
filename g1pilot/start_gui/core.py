"""Logik des Startmenues ohne GUI: Pfade, Host-Abfragen, gemerkte Auswahl,
Umgebungsvariablen fuer start.sh. Bewusst ohne Qt-Import, damit die Tests die
Start-Logik (welche Env-Variablen bei welcher Auswahl) direkt pruefen koennen.

Laeuft auf dem HOST (nicht im Container), daher nur Standardbibliothek.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import webbrowser
from dataclasses import dataclass
from pathlib import Path

# ── Pfade (alles relativ zu diesem Paket, damit der Aufrufort egal ist) ──
HERE = Path(__file__).resolve().parent.parent                # .../g1pilot
REPO_ROOT = HERE.parent                                      # Repo-Wurzel
START_SH = HERE / "start.sh"
ICON_DIR = HERE / "resource" / "icons"
START_ICON = ICON_DIR / "g1_start.png"
SCENE_DIR = REPO_ROOT / "unitree_mujoco" / "scene_editor"
SCENES_DIR = SCENE_DIR / "scenes"
THUMBS_DIR = SCENES_DIR / "thumbs"
THUMB_SCRIPT = SCENE_DIR / "render_thumbs.py"
LAUNCH_SH = SCENE_DIR / "launch.sh"
SETUP_SH = SCENE_DIR / "setup.sh"
SCENE_VENV = SCENE_DIR / ".venv" / "bin" / "python"
STANDARD_SCENE_XML = REPO_ROOT / "unitree_mujoco" / "unitree_robots" / "g1" / "scene.xml"
# Hand-Profile (eigene starre Haende fuer die Sim, z. B. die Box-Haken)
HANDS_DIR = REPO_ROOT / "unitree_mujoco" / "unitree_robots" / "g1" / "hands"
HAND_TOOL_SH = HANDS_DIR.parent / "tools" / "hand_tool.sh"
DOCS_DIR = HERE / "docs"

# Name des Vorschaubilds der Standard-Umgebung (scene.xml des G1).
STANDARD_THUMB = "_standard"

# ── Auswahl-Listen: (wert, titel, kurztext) ─────────────────────────────
# Lauf-Policy von loco_sim (nur Sim; Ordner unter policies/, -> G1_WALK_POLICY).
WALK_POLICY_CHOICES = [
    ("g1_wholebody", "Alt (Standard)",
     "natürlicher Gang; Arme beim Laufen in der Lauf-Pose geparkt"),
    ("agile_velocity_g1", "AGILE Velocity",
     "sehr robust, Gang mit Fangschritten; Arme frei"),
    ("agile_velocity_height_g1", "AGILE Velocity-Height",
     "gleichmäßiger Gang, bis 1 m/s; Arme frei"),
]
DEFAULT_WALK_POLICY = WALK_POLICY_CHOICES[0][0]

# Art der Sim-Haende; bei "custom" kommt die Auswahl des eigenen Hand-Profils
# dazu (Ordner unter HANDS_DIR).
HAND_KIND_CHOICES = [
    ("rubber", "Rubber", "starre Standard-Hand"),
    ("inspire", "Inspire", "Finger steuerbar, Kraftsensoren"),
    ("custom", "Custom", "eigene starre Hände aus dem CAD, z. B. Haken"),
]


# ════════════════════════════════════════════════════════════════════════
#  Host-Abfragen
# ════════════════════════════════════════════════════════════════════════
def have(cmd: str) -> bool:
    return shutil.which(cmd) is not None


def docker_ready() -> bool:
    """True, wenn 'docker' existiert und der Daemon erreichbar ist."""
    if not have("docker"):
        return False
    try:
        r = subprocess.run(["docker", "info"], capture_output=True, timeout=6)
        return r.returncode == 0
    except Exception:
        return False


def list_hand_profiles(hands_dir: Path = HANDS_DIR) -> list[tuple[str, str]]:
    """Eigene Hand-Profile [(ordner, anzeigename)] aus unitree_robots/g1/hands/."""
    out = []
    if not hands_dir.is_dir():
        return out
    for d in sorted(hands_dir.iterdir()):
        f = d / "hand.yaml"
        if not f.is_file() or not re.match(r"^[a-z0-9_]+$", d.name):
            continue
        try:
            m = re.search(r"^name:\s*(.+?)\s*(#.*)?$", f.read_text(encoding="utf-8"), re.M)
        except OSError:
            m = None
        out.append((d.name, m.group(1) if m else d.name))
    return out


def list_scenes(scenes_dir: Path = SCENES_DIR) -> list[Path]:
    """Alle Umgebungen aus scene_editor/scenes/*.xml (sortiert)."""
    if not scenes_dir.is_dir():
        return []
    return sorted(p for p in scenes_dir.glob("*.xml") if p.is_file())


def scene_summary(xml: Path) -> str:
    """Kurzbeschreibung fuer die Kachel: Stationen, Greif-Objekte, Hindernisse.

    Liest nur die Namenskonventionen von build_env_scene/scene_editor
    (station_*, grasp_asset_*, asset_*) -- kein MuJoCo noetig.
    """
    try:
        text = xml.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    names = re.findall(r'<geom\b[^>]*\bname="([^"]+)"', text)
    ngeoms = len(re.findall(r"<geom\b", text))
    stations = sum(1 for n in names if n.startswith("station_"))
    grasp = sum(1 for n in names if n.startswith("grasp_asset_") and "__" not in n)
    static = sum(1 for n in names if n.startswith("asset_"))
    bodies = len(re.findall(r"<joint\b[^>]*type=\"free\"", text))
    objs = grasp + bodies
    parts = []
    if static:
        parts.append(f"{static} Möbel")
    if objs:
        parts.append(f"{objs} Objekt" + ("" if objs == 1 else "e"))
    if stations:
        parts.append(f"{stations} Station" + ("" if stations == 1 else "en"))
    if not parts and ngeoms:
        parts.append(f"{ngeoms} Körper")
    return " · ".join(parts) or "leer"


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
        out = subprocess.run(["ip", "-o", "link", "show"], capture_output=True,
                             text=True, timeout=5).stdout
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
            addr = subprocess.run(["ip", "-4", "-o", "addr", "show", name],
                                  capture_output=True, text=True, timeout=5).stdout
            for al in addr.splitlines():
                toks = al.split()
                if "inet" in toks:
                    ip = toks[toks.index("inet") + 1]
                    break
        except Exception:
            pass
        nics.append((name, ip))
    return nics


def open_path(path: Path) -> str:
    """Datei mit dem System-Standard oeffnen. Rueckgabe: Fehlertext oder ''."""
    try:
        if sys.platform == "darwin":
            subprocess.Popen(["open", str(path)])
        elif os.name == "nt":
            os.startfile(str(path))  # type: ignore[attr-defined]
        else:
            subprocess.Popen(["xdg-open", str(path)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return ""
    except Exception as exc:
        return str(exc)


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
#  Zuletzt gewaehlte Optionen merken (pro Ansicht)
# ════════════════════════════════════════════════════════════════════════
def settings_file() -> Path:
    base = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    return base / "g1pilot" / "launcher.json"


def load_settings(key: str) -> dict:
    try:
        return json.loads(settings_file().read_text()).get(key, {})
    except Exception:
        return {}


def save_settings(key: str, values: dict) -> None:
    f = settings_file()
    try:
        data = json.loads(f.read_text()) if f.exists() else {}
    except Exception:
        data = {}
    data[key] = values
    try:
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_text(json.dumps(data, indent=2))
    except Exception:
        pass   # Merken ist Komfort -- Start darf daran nie scheitern


# ════════════════════════════════════════════════════════════════════════
#  Start-Kommandos (Env-Variablen fuer start.sh --yes)
# ════════════════════════════════════════════════════════════════════════
@dataclass
class Launch:
    """Alles, was die GUI zum Starten eines Stacks braucht."""
    title: str
    argv: list
    env: dict
    stop_cmd: list
    stop_note: str


def sim_launch(*, env_name: str, policy: str, hand_profile: str, nav: bool,
               ps4: bool, rviz: bool, rt: str, rebuild: bool,
               base_env: dict | None = None) -> Launch:
    """Sim-Start (wie bisher SimFrame._start). env_name '' = Standard-Szene."""
    env = dict(os.environ if base_env is None else base_env)
    env["G1_MODE"] = "sim"
    env["G1_ENV"] = env_name
    env["G1_WALK_POLICY"] = policy
    env["G1_INSPIRE_HANDS"] = "1" if hand_profile == "inspire" else "0"
    env["G1_HANDS"] = hand_profile
    env["G1_ENABLE_NAV"] = "1" if nav else "0"
    env["G1_PS4_ARMS"] = "1" if ps4 else "0"
    # Handy-Zugang gehoert immer zur Demo-GUI; Bluetooth startet erst beim Klick
    # auf »Handy« (kein Haken -> ohne Dongle keine Fehlermeldung beim Start).
    env["G1_PHONE"] = "1"
    # BlueZ ueber den System-D-Bus geht nur ohne Dockers AppArmor-Profil.
    env["G1_SIM_APPARMOR"] = "unconfined" if env["G1_PHONE"] == "1" else "docker-default"
    env["USE_RVIZ"] = "true" if rviz else "false"
    env["SIM_REALTIME_FACTOR"] = rt
    argv = ["bash", str(START_SH), "--yes"]
    if rebuild:
        argv.append("--build")
    return Launch("Simulation", argv, env,
                  ["docker", "compose", "--profile", "sim", "down", "--remove-orphans"],
                  "Stoppt den Sim-Stack (docker compose down).")


def real_launch(vals: dict, *, rebuild: bool, base_env: dict | None = None) -> Launch:
    """Real-Start (wie bisher RealFrame._start). vals wie save_settings('real')."""
    env = dict(os.environ if base_env is None else base_env)
    env["G1_MODE"] = "real"
    env["G1_REAL_CONFIRM"] = "1"  # Sicherheits-Gate von start.sh im --yes-Modus
    env["ROBOT_INTERFACE"] = vals["iface"]
    env["G1_INSPIRE_HANDS"] = "1" if vals["hands"] else "0"
    env["G1_HAND_LEFT_HOST"] = vals["left"]
    env["G1_HAND_RIGHT_HOST"] = vals["right"]
    env["G1_HAND_PORT"] = vals["port"]
    env["USE_RVIZ"] = "true" if vals["rviz"] else "false"
    env["JOYSTICK_NAME"] = vals["joy"]
    env["G1_PS4_ARMS"] = "1" if vals["ps4"] else "0"
    env["G1_PHONE"] = "1"   # Bluetooth erst beim Klick auf »Handy«
    env["G1_MAX_VX"] = vals["vx"]
    env["G1_MAX_VY"] = vals["vy"]
    env["G1_MAX_VYAW"] = vals["vyaw"]
    argv = ["bash", str(START_SH), "--yes"]
    if rebuild:
        argv.append("--build")
    return Launch("Echter Roboter", argv, env,
                  ["docker", "compose", "--profile", "real", "down", "--remove-orphans"],
                  "Stoppt den Real-Stack (docker compose down).")


# ════════════════════════════════════════════════════════════════════════
#  Doku-Verzeichnis (fuer den eingebauten Viewer)
# ════════════════════════════════════════════════════════════════════════
@dataclass
class DocEntry:
    path: Path
    title: str            # erste '# '-Ueberschrift, sonst Dateiname
    label: str            # Anzeige im Baum, z. B. "25 Eigene Hände — Anleitung"
    kind: str             # "readme" | "group" | "A" | "T"


@dataclass
class DocGroup:
    number: str
    entry: DocEntry | None
    topics: list


_GROUP_RE = re.compile(r"^(\d)-(.+)\.md$")
_TOPIC_RE = re.compile(r"^(\d)(\d)-([AT])-(.+)\.md$")


def doc_title(path: Path) -> str:
    try:
        with path.open(encoding="utf-8") as fh:
            for line in fh:
                if line.startswith("# "):
                    return line[2:].strip()
    except OSError:
        pass
    return path.stem


def docs_index(docs_dir: Path = DOCS_DIR) -> tuple[list[DocEntry], list[DocGroup]]:
    """Doku nach dem Schema GT-X-Name.md gruppieren (siehe docs/README.md).

    Rueckgabe: (Kopf-Eintraege [docs/README, README], Gruppen 0..9 mit
    Uebersicht + Themen). Neue Dateien tauchen automatisch auf.
    """
    head = []
    for p, label in ((docs_dir / "README.md", "Dokumentation — Übersicht"),
                     (docs_dir.parent / "README.md", "README (G1Pilot)")):
        if p.is_file():
            head.append(DocEntry(p, doc_title(p), label, "readme"))
    groups: dict[str, DocGroup] = {}
    if docs_dir.is_dir():
        for p in sorted(docs_dir.glob("*.md")):
            m = _GROUP_RE.match(p.name)
            if m:
                g = groups.setdefault(m.group(1), DocGroup(m.group(1), None, []))
                title = doc_title(p)
                label = title if title[:1].isdigit() else f"{m.group(1)} · {title}"
                g.entry = DocEntry(p, title, label, "group")
                continue
            m = _TOPIC_RE.match(p.name)
            if m:
                g = groups.setdefault(m.group(1), DocGroup(m.group(1), None, []))
                title = doc_title(p)
                kind = m.group(3)
                g.topics.append(DocEntry(p, title, f"{m.group(1)}{m.group(2)} {title}", kind))
    return head, [groups[k] for k in sorted(groups)]


def github_slug(text: str) -> str:
    """Anker wie GitHub: klein, Satzzeichen weg, Leerzeichen -> '-'."""
    s = text.strip().lower()
    s = re.sub(r"[^\w\- ]", "", s, flags=re.UNICODE)
    return s.replace(" ", "-")


# ════════════════════════════════════════════════════════════════════════
#  Vorschaubilder der Umgebungen (erzeugt von scene_editor/render_thumbs.py)
# ════════════════════════════════════════════════════════════════════════
def thumb_path(name: str, thumbs_dir: Path = THUMBS_DIR) -> Path | None:
    """PNG der Umgebung (name = Datei-Stamm, STANDARD_THUMB = Standard) oder None."""
    p = thumbs_dir / f"{name}.png"
    return p if p.is_file() else None


def _xml_hash(path: Path) -> str:
    import hashlib
    return hashlib.sha1(path.read_bytes()).hexdigest()[:16]


def stale_thumbs(scenes_dir: Path = SCENES_DIR, thumbs_dir: Path = THUMBS_DIR,
                 standard_xml: Path = STANDARD_SCENE_XML) -> list[str]:
    """Namen der Umgebungen, deren Vorschaubild fehlt oder nicht zur XML passt.

    Gleiche Regel wie render_thumbs.py (Hash in thumbs/index.json), damit die GUI
    den Renderer nur startet, wenn es wirklich etwas zu tun gibt.
    """
    try:
        idx = json.loads((thumbs_dir / "index.json").read_text())
    except Exception:
        idx = {}
    cand = [(STANDARD_THUMB, standard_xml)] + [(p.stem, p) for p in list_scenes(scenes_dir)]
    out = []
    for name, xml in cand:
        if not xml.is_file():
            continue
        try:
            h = _xml_hash(xml)
        except OSError:
            continue
        if idx.get(name) != h or not (thumbs_dir / f"{name}.png").is_file():
            out.append(name)
    return out

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
eval_common.py -- gemeinsame Basis der Messwerkzeuge (Bachelorarbeit, Evaluation).

Jedes Werkzeug unter g1pilot/tools/eval/ legt seine Ausgaben so ab:

    data/eval/<versuch>/<JJJJ-MM-TT_hh-mm-ss>/
        *.csv          Rohdaten, Kopfzeilen beginnen mit '# '
        summary.json   Zusammenfassung, Schluessel "meta" = derselbe Kopf

Der Kopf jeder Datei enthaelt: Werkzeug, Versuch, Zeitpunkt, Git-Commit,
Betriebsart (Simulation / Realbetrieb / offline / statisch), Rechner (CPU,
RAM, GPU, OS, Container) und die Parameter des Laufs.

Nur Standardbibliothek (+ numpy, wenn vorhanden, fuer die Statistik ist es
nicht noetig) -- die Werkzeuge laufen damit auch im schlanken Real-Image.
Messwerte werden NIE geglaettet oder gefiltert: was hier ankommt, wird so
geschrieben.
"""
import csv
import datetime as _dt
import json
import math
import os
import platform
import re
import shutil
import subprocess
import sys

# g1pilot/tools/eval/eval_common.py -> g1pilot/ (Paketwurzel) und Repo-Wurzel
EVAL_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(EVAL_DIR, "..", ".."))
REPO_ROOT = os.path.abspath(os.path.join(PKG_ROOT, ".."))
DEFAULT_OUT = os.path.join(PKG_ROOT, "data", "eval")

MODE_SIM = "Simulation"
MODE_REAL = "Realbetrieb"
MODE_OFFLINE = "offline (ohne Roboter und ohne Simulation)"
MODE_STATIC = "statisch (Code-Analyse)"
MODE_FAKE = "Attrappe (FakeG1, G1_SIM_MODE=false ohne Hardware)"


# ══════════════════════════════════════════════════════════════════════
# Betriebsart
# ══════════════════════════════════════════════════════════════════════
def env_mode():
    """Betriebsart wie im System selbst: G1_SIM_MODE (utils/common.py).
    Nicht gesetzt = Realbetrieb (gleicher Default wie is_sim_mode())."""
    return MODE_SIM if os.getenv("G1_SIM_MODE", "false").lower() == "true" else MODE_REAL


# ══════════════════════════════════════════════════════════════════════
# Git
# ══════════════════════════════════════════════════════════════════════
def _run(cmd, cwd=None, timeout=5.0):
    try:
        out = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             timeout=timeout, check=False)
        if out.returncode != 0:
            return None
        return out.stdout.decode("utf-8", "replace").strip()
    except Exception:
        return None


def git_info(root=REPO_ROOT):
    """Commit, Branch und ob der Arbeitsbaum Aenderungen hat. Im Container
    ist meist nur g1pilot/ gemountet (kein .git) -> Fallback auf die Variable
    G1_EVAL_GIT_COMMIT (beim Start mitgeben, siehe docs/7-Evaluation.md)."""
    commit = _run(["git", "rev-parse", "HEAD"], cwd=root)
    if commit:
        status = _run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=root)
        return {
            "commit": commit,
            "branch": _run(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=root) or "",
            "geaendert": bool(status),
            "quelle": "git",
        }
    env = os.getenv("G1_EVAL_GIT_COMMIT", "").strip()
    if env:
        return {"commit": env, "branch": os.getenv("G1_EVAL_GIT_BRANCH", ""),
                "geaendert": None, "quelle": "G1_EVAL_GIT_COMMIT"}
    return {"commit": "unbekannt", "branch": "", "geaendert": None,
            "quelle": "kein .git erreichbar und G1_EVAL_GIT_COMMIT nicht gesetzt"}


# ══════════════════════════════════════════════════════════════════════
# Rechner
# ══════════════════════════════════════════════════════════════════════
def _read(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except Exception:
        return ""


def cpu_info():
    txt = _read("/proc/cpuinfo")
    m = re.search(r"^model name\s*:\s*(.+)$", txt, re.M)
    model = m.group(1).strip() if m else (platform.processor() or platform.machine())
    return {"modell": model, "logische_kerne": os.cpu_count(),
            "physische_kerne": len(set(re.findall(r"^core id\s*:\s*(\d+)$", txt, re.M))) or None}


def ram_info():
    txt = _read("/proc/meminfo")
    m = re.search(r"^MemTotal:\s*(\d+)\s*kB", txt, re.M)
    return {"gesamt_gib": round(int(m.group(1)) / 1024.0 / 1024.0, 2) if m else None}


def gpu_info():
    """nvidia-smi, sonst lspci (VGA/3D), sonst unbekannt. Nur lesend."""
    if shutil.which("nvidia-smi"):
        out = _run(["nvidia-smi", "--query-gpu=name,driver_version,memory.total",
                    "--format=csv,noheader"])
        if out:
            return {"quelle": "nvidia-smi", "geraete": [s.strip() for s in out.splitlines()]}
    if shutil.which("lspci"):
        out = _run(["lspci"])
        if out:
            dev = [l.split(": ", 1)[-1] for l in out.splitlines()
                   if re.search(r"VGA|3D controller|Display", l)]
            if dev:
                return {"quelle": "lspci", "geraete": dev}
    drm = []
    base = "/sys/class/drm"
    if os.path.isdir(base):
        for d in sorted(os.listdir(base)):
            vendor = _read(os.path.join(base, d, "device", "vendor")).strip()
            if vendor and re.match(r"card\d+$", d):
                drm.append(f"{d} vendor={vendor}")
    if drm:
        return {"quelle": "/sys/class/drm", "geraete": drm}
    return {"quelle": "keine", "geraete": ["keine GPU gefunden (oder nicht sichtbar im Container)"]}


def os_info():
    rel = _read("/etc/os-release")
    m = re.search(r'^PRETTY_NAME="?([^"\n]+)"?', rel, re.M)
    return {"name": m.group(1) if m else platform.system(),
            "kernel": platform.release(),
            "python": platform.python_version(),
            "container": os.path.exists("/.dockerenv") or "docker" in _read("/proc/1/cgroup")}


def host_info():
    return {"hostname": platform.node(), "cpu": cpu_info(), "ram": ram_info(),
            "gpu": gpu_info(), "os": os_info()}


# ══════════════════════════════════════════════════════════════════════
# Kopf + Ausgabeordner
# ══════════════════════════════════════════════════════════════════════
def _jsonable(v):
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return None
    if hasattr(v, "tolist"):            # numpy
        return _jsonable(v.tolist())
    if isinstance(v, (str, int, float, bool)) or v is None:
        return v
    return str(v)


class Run:
    """Ein Messlauf: Ausgabeordner + Kopf. Dateien darin mit write_csv()/
    write_json() schreiben -- beide setzen den Kopf automatisch."""

    def __init__(self, tool, versuch, mode, params, out_root=None, stamp=None):
        self.tool = tool
        self.versuch = versuch
        self.mode = mode
        self.params = dict(params)
        now = stamp or _dt.datetime.now()
        self.stamp = now.strftime("%Y-%m-%d_%H-%M-%S")
        root = out_root or os.getenv("G1_EVAL_OUT", "") or DEFAULT_OUT
        self.dir = os.path.join(root, versuch, self.stamp)
        n = 1
        while os.path.exists(self.dir):           # zwei Laeufe in derselben Sekunde
            n += 1
            self.dir = os.path.join(root, versuch, f"{self.stamp}_{n}")
        os.makedirs(self.dir)
        self.meta = {
            "werkzeug": tool,
            "versuch": versuch,
            "zeitpunkt": now.isoformat(timespec="seconds"),
            "git": git_info(),
            "betriebsart": mode,
            "rechner": host_info(),
            "parameter": _jsonable(self.params),
            "aufruf": " ".join([os.path.basename(sys.argv[0])] + sys.argv[1:]),
        }
        self.files = []

    def path(self, name):
        return os.path.join(self.dir, name)

    def header_lines(self, extra=None):
        """Kopfzeilen fuer CSV: '# schluessel: wert' (verschachtelt als JSON)."""
        m = dict(self.meta)
        if extra:
            m["datei"] = extra
        lines = []
        for k, v in m.items():
            if isinstance(v, (dict, list)):
                v = json.dumps(_jsonable(v), ensure_ascii=False, sort_keys=True)
            lines.append(f"# {k}: {v}")
        return lines

    def write_csv(self, name, columns, rows, beschreibung=""):
        """rows: Liste von Listen/Tupeln oder Dicts (Schluessel = columns)."""
        p = self.path(name)
        with open(p, "w", newline="", encoding="utf-8") as f:
            for line in self.header_lines(beschreibung or name):
                f.write(line + "\n")
            w = csv.writer(f)
            w.writerow(columns)
            for r in rows:
                if isinstance(r, dict):
                    r = [r.get(c, "") for c in columns]
                w.writerow([_fmt(x) for x in r])
        self.files.append(name)
        return p

    def write_json(self, name, data):
        p = self.path(name)
        out = {"meta": self.meta}
        out.update(_jsonable(data))
        with open(p, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2, sort_keys=False)
            f.write("\n")
        self.files.append(name)
        return p

    def finish(self, summary):
        """summary.json schreiben (enthaelt die Dateiliste) und Pfad ausgeben."""
        summary = dict(summary)
        summary["dateien"] = sorted(set(self.files + ["summary.json"]))
        p = self.write_json("summary.json", summary)
        print(f"[{self.tool}] Ausgabe: {self.dir}", flush=True)
        return p


def _fmt(x):
    if isinstance(x, float):
        if math.isnan(x):
            return "nan"
        if math.isinf(x):
            return "inf" if x > 0 else "-inf"
        return repr(x)
    if isinstance(x, bool):
        return "1" if x else "0"
    if x is None:
        return ""
    if hasattr(x, "item"):              # numpy-Skalar
        return _fmt(x.item())
    return x


def read_csv(path):
    """CSV mit '#'-Kopf lesen -> (meta dict, Spalten, Zeilen als Dicts mit str)."""
    meta, rows, cols = {}, [], None
    with open(path, "r", encoding="utf-8") as f:
        body = []
        for line in f:
            if cols is None and line.startswith("# "):
                k, _, v = line[2:].rstrip("\n").partition(": ")
                try:
                    meta[k] = json.loads(v)
                except Exception:
                    meta[k] = v
                continue
            body.append(line)
    r = csv.reader(body)
    for i, row in enumerate(r):
        if i == 0:
            cols = row
            continue
        if row:
            rows.append(dict(zip(cols, row)))
    return meta, cols or [], rows


# ══════════════════════════════════════════════════════════════════════
# Statistik (ohne Glaettung, ohne Ausreisser-Entfernung)
# ══════════════════════════════════════════════════════════════════════
def percentile(values, p):
    """Lineare Interpolation (wie numpy.percentile, method='linear')."""
    v = sorted(x for x in values if x is not None and not (isinstance(x, float) and math.isnan(x)))
    if not v:
        return None
    if len(v) == 1:
        return float(v[0])
    k = (len(v) - 1) * (p / 100.0)
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return float(v[lo])
    return float(v[lo] + (v[hi] - v[lo]) * (k - lo))


def describe(values):
    """n, Mittel, Std (Stichprobe), Min, P5, Median, P95, P99, Max."""
    v = [float(x) for x in values if x is not None and not (isinstance(x, float) and math.isnan(x))]
    n = len(v)
    if n == 0:
        return {"n": 0}
    mean = sum(v) / n
    std = math.sqrt(sum((x - mean) ** 2 for x in v) / (n - 1)) if n > 1 else 0.0
    return {"n": n, "mittel": mean, "std": std, "min": min(v),
            "p5": percentile(v, 5), "median": percentile(v, 50),
            "p95": percentile(v, 95), "p99": percentile(v, 99), "max": max(v)}


def add_common_args(ap, versuch):
    ap.add_argument("--out", default=None,
                    help="Wurzel der Ausgabe (Default: g1pilot/data/eval bzw. $G1_EVAL_OUT). "
                         f"Darunter entsteht {versuch}/<datum-uhrzeit>/.")
    return ap

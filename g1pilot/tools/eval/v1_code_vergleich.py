#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v1_code_vergleich.py -- V1: Code-Vergleich Simulation / Realbetrieb.

Findet statisch (ohne etwas zu starten) alle Stellen, an denen der Code die
Betriebsart abfragt, und stellt die Unterschiede der Container-Images und
Konfigurationen fuer Simulation und Realbetrieb gegenueber:

  1. Python (g1pilot/ inkl. launch/, Werkzeuge, Tests): AST-Analyse
     - direkte Abfragen: is_sim_mode(), resolve_dds(), init_dds(), die
       Konstanten SIM_DDS_*/REAL_DDS_*, Lesen von "G1_SIM_MODE",
     - Abfrage-Funktionen: Funktionen, deren Rueckgabewert die Betriebsart
       ist (z. B. hand_profile._sim_mode) -- ihre Aufrufe zaehlen wie direkte,
     - abgeleitete Namen: Variablen/Attribute, denen eine Abfrage zugewiesen
       wird (z. B. DemoGUI.sim_mode) -- jede Verwendung zaehlt.
     Je Fundstelle: Datei, Zeile, Funktion, Art der Abfrage, Wirkung
     (Sim-Zweig / Real-Zweig / Wertwahl / Parameter-Default / ...).
  2. Shell/Compose/Make/Docker: jede Zeile mit G1_SIM_MODE.
  3. Launch: bringup_sim.launch.py gegen bringup_real.launch.py (Nodes,
     Includes und deren Argumente/Parameter).
  4. Images: docker/Dockerfile.sim gegen docker/Dockerfile.real (Basis,
     apt-/pip-Pakete, ENV, Quellen) + Dockerfile.mujoco (nur Sim).
  5. Konfiguration: docker-compose.yml Dienst g1pilot-sim gegen g1pilot-real
     (environment, volumes, Rechte, Befehl) + Sim-only-Dienste/Overrides,
     start.sh: exportierte Variablen je Zweig.

Ausgabe nach data/eval/v1_code_vergleich/<zeit>/: abfragen.csv,
indirekt.csv, launch.csv, images.csv, konfiguration.csv, tabelle.md
(alles als lesbare Tabelle) und summary.json.

Aufruf (aus g1pilot/ oder von ueberall):
    python3 tools/eval/v1_code_vergleich.py [--include-tests] [--out DIR]
"""
import argparse
import ast
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_common as ec  # noqa: E402

VERSUCH = "v1_code_vergleich"
QUERY_FUNCS = {"is_sim_mode": "Abfrage is_sim_mode()",
               "resolve_dds": "DDS-Auswahl resolve_dds()",
               "init_dds": "DDS-Start init_dds()"}
QUERY_CONSTS = ("SIM_DDS_DOMAIN", "REAL_DDS_DOMAIN", "SIM_DDS_INTERFACE")
ENV_NAME = "G1_SIM_MODE"
COMMON_MOD = "g1pilot.utils.common"

# Bekannte Funktionen: was die Abfrage dort bewirkt (Datei, Qualname) -> Text.
# Fehlt ein Eintrag, steht in der Tabelle "nicht katalogisiert" -- dann die
# Stelle von Hand ansehen und hier nachtragen.
CATALOG = {
    ("g1pilot/utils/common.py", "is_sim_mode"):
        "Definition des Schalters: G1_SIM_MODE=true -> Simulation, sonst Realbetrieb",
    ("g1pilot/utils/common.py", "resolve_dds"):
        "DDS: Sim Domain 1 auf lo erzwungen, Real Domain 0 auf dem Roboter-NIC",
    ("g1pilot/utils/common.py", "init_dds"):
        "DDS-ChannelFactory mit der Auswahl aus resolve_dds starten",
    ("g1pilot/utils/hand_profile.py", "_sim_mode"):
        "Hand-Profil: eigene Profile (z. B. haken) nur in der Sim, real immer Inspire-URDF",
    ("g1pilot/utils/hand_profile.py", "hand_profile"):
        "Hand-Profil: eigenes Profil nur in der Sim gueltig",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._init_waist_hold"):
        "Nur real: Taillen-Halten (WaistHold) bei arm_sdk-Gewicht > 0; Sim: Taille fuehrt loco_sim",
    ("g1pilot/manipulation/arm_controller.py", "ArmController.shutdown_release"):
        "Nur real: bei Programmende Arme zurueckfahren und arm_sdk-Gewicht 1->0 rampen",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._on_emergency_stop"):
        "NOT-HALT: real Taillen-Yaw auf das kommandierte Ziel einfrieren, Sim auf den Messwert",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._waist_yaw_owner"):
        "Hueftdrehung: Sim kommandiert immer (loco_sim), real nur mit aktivem WaistHold",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._waist_yaw_block_reason"):
        "Hueftdrehung: real zusaetzlich gesperrt, solange die Arme noch uebernommen werden",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._waist_yaw_can_center"):
        "Hueftdrehung zur Mitte: Sim immer, real nur solange arm_sdk die Taille haelt",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._apply_waist_yaw_target"):
        "Hueftdrehung: nur real in das Halteziel der Taille schreiben",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._waist_yaw_tick"):
        "Hueftdrehung: Bezug 'Mitte' real = Halteziel, Sim = 0",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._on_walk_mode"):
        "WALK: real bleiben die Arme exakt in ihrer Stellung (laufende Fahrt endet)",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._report_walk_ready_onboard"):
        "WALK ohne Arm-Uebernahme: real sofort walk_ready, Sim nicht (Bridge haelt Null-Pose)",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._sim_auto_enable_arms"):
        "Nur Sim: Arme automatisch uebernehmen, wenn WALK/POSE ANFAHREN sie braucht",
    ("g1pilot/teleoperation/ps4_arm_teleop.py", "main.Ps4ArmTeleop.__init__"):
        "Controller: Moduswechsel per Trigger real nur bei gemeldetem Onboard-Walk",
    ("g1pilot/teleoperation/demo_gui.py", "DemoGUI.__init__"):
        "Demo-GUI: Sim-/Real-Oberflaeche (Titel, Startleiste, Temperatur/Akku, Sim-Knoepfe)",
    ("g1pilot/teleoperation/demo_gui.py", "main"):
        "Demo-GUI: Fenster-Icon SIM blau / REAL rot",
    ("g1pilot/teleoperation/demo_gui.py", "DemoGUI._build"):
        "Demo-GUI: Startleiste/Onboard-Info/Temperatur nur real; Szene-/Roboter-Reset und Schubs nur Sim",
    ("g1pilot/teleoperation/demo_gui.py", "DemoGUI._refresh_info"):
        "Demo-GUI: Onboard-Zustand nur real; Controller-Statustext je Betriebsart",
    ("g1pilot/teleoperation/demo_gui.py", "DemoGUI._refresh_walk_hint"):
        "Demo-GUI: Hinweis auf der Kachel GEHEN (Onboard-Walk, Bodenkontakt) nur real",
    ("g1pilot/teleoperation/demo_gui.py", "DemoGUI.request_mode"):
        "Moduswahl: Sim schaltet Balancieren/Gehen selbst; real nur bei Onboard-Walk und Bodenkontakt",
    ("g1pilot/teleoperation/demo_gui.py", "DemoGUI._enter_estop"):
        "NOT-HALT: Sim quittieren per START, real erneut ueber die Startleiste starten",
    ("g1pilot/teleoperation/demo_gui.py", "DemoGUI._quit_clicked"):
        "Beenden: real mit Rueckgabe der Arme an den Onboard-Regler (_quit_real_begin)",
    ("g1pilot/teleoperation/phone/link.py", "PhoneLink.__init__"):
        "Handy: Bluetooth-Name G1 SIM / G1 REAL",
    ("g1pilot/teleoperation/phone/link.py", "PhoneLink._do_ack"):
        "Handy: Quittieren nach NOT-HALT -- Sim per START, real ueber die Startleiste",
    ("g1pilot/teleoperation/phone/link.py", "PhoneLink._do_loco"):
        "Handy: Startleiste (Onboard-Loco-Schritte) nur real",
    ("g1pilot/teleoperation/phone/link.py", "PhoneLink.snapshot"):
        "Handy: Statusbild mit Onboard-Info/Startleiste nur real",
    ("g1pilot/teleoperation/phone/link.py", "PhoneLink._pad_status"):
        "Handy: Controller-Statustext je Betriebsart",
    ("g1pilot/teleoperation/phone/link.py", "PhoneLink._temp_status"):
        "Handy: Temperaturen nur real",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._publish_waist_state"):
        "Hueftdrehung: Flag sim im Status-JSON (Anzeige)",
    ("g1pilot/manipulation/inspire_ftp/backends.py", "MujocoContactBackend.__init__"):
        "Inspire-Haende (Sim-Backend): DDS zur MuJoCo-Bridge",
    ("g1pilot/navigation/loco_client.py", "G1LocoClient.__init__"):
        "Loco-Client (real): DDS zum Onboard-Rechner",
    ("g1pilot/navigation/loco_sim.py", "LocoSim.__init__"):
        "loco_sim (nur Sim gestartet): DDS zur MuJoCo-Bridge",
    ("g1pilot/navigation/sim_localization.py", "SimLocalization.__init__"):
        "Sim-Lokalisierung (nur Sim gestartet): DDS zur MuJoCo-Bridge",
    ("g1pilot/state/robot_state.py", "RobotState.__init__"):
        "Roboterzustand: DDS (Sim lo/1, real NIC/0)",
    ("g1pilot/manipulation/arm_controller.py", "ArmController._init_robot_interface") :
        "DDS + nur real: Gains, Rampen und Tempo auf sichere Real-Grenzen begrenzen",
    ("g1pilot/tools/sim_arm_monitor.py", "main"): "Werkzeug: DDS-Auswahl je Betriebsart (Log)",
    ("g1pilot/tools/sim_arm_wiggle.py", "main"): "Werkzeug: DDS-Auswahl je Betriebsart (Log)",
    ("g1pilot/tools/sim_dds_check.py", "main"): "Werkzeug: DDS-Auswahl je Betriebsart (Log)",
    ("g1pilot/tools/sim_leg_hold.py", "main"): "Werkzeug: DDS-Auswahl je Betriebsart (Log)",
}


# ══════════════════════════════════════════════════════════════════════
# 1. Python-Analyse
# ══════════════════════════════════════════════════════════════════════
def _rel(path):
    return os.path.relpath(path, ec.PKG_ROOT).replace(os.sep, "/")


def _bereich(rel):
    if rel.startswith("launch/"):
        return "Launch"
    if rel.startswith("test/"):
        return "Test"
    if rel.startswith("tools/") or rel.startswith("g1pilot/tools/"):
        return "Werkzeug"
    if rel.startswith("g1pilot/"):
        return "Laufzeit"
    return "Skript"


def python_files(include_tests):
    out = []
    for dirpath, dirnames, filenames in os.walk(ec.PKG_ROOT):
        rel_dir = os.path.relpath(dirpath, ec.PKG_ROOT).replace(os.sep, "/")
        dirnames[:] = [d for d in dirnames
                       if d not in ("build", "install", "log", "__pycache__", "data", "web")
                       and not d.startswith(".")]
        if rel_dir.startswith("tools/eval"):
            continue
        if rel_dir.startswith("test") and not include_tests:
            continue
        for fn in filenames:
            if fn.endswith(".py"):
                out.append(os.path.join(dirpath, fn))
    return sorted(out)


class _Module:
    def __init__(self, path):
        self.path = path
        self.rel = _rel(path)
        with open(path, "r", encoding="utf-8") as f:
            self.src = f.read()
        self.lines = self.src.splitlines()
        self.tree = ast.parse(self.src, filename=path)
        self.modname = self._modname()
        self.parents = {}
        for node in ast.walk(self.tree):
            for ch in ast.iter_child_nodes(node):
                self.parents[ch] = node
        # Importe: lokaler Name -> (Modul, Name) bzw. Modul-Alias -> Modul
        self.from_imports = {}
        self.mod_alias = {}
        for node in ast.walk(self.tree):
            if isinstance(node, ast.ImportFrom) and node.module:
                for a in node.names:
                    self.from_imports[a.asname or a.name] = (node.module, a.name)
            elif isinstance(node, ast.Import):
                for a in node.names:
                    self.mod_alias[a.asname or a.name.split(".")[0]] = a.name

    def _modname(self):
        rel = self.rel[:-3]
        if rel.endswith("/__init__"):
            rel = rel[: -len("/__init__")]
        return rel.replace("/", ".")

    def qualname(self, node):
        names = []
        p = self.parents.get(node)
        while p is not None:
            if isinstance(p, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                names.append(p.name)
            p = self.parents.get(p)
        return ".".join(reversed(names)) or "<modul>"

    def enclosing_func(self, node):
        p = self.parents.get(node)
        while p is not None:
            if isinstance(p, (ast.FunctionDef, ast.AsyncFunctionDef)):
                return p
            p = self.parents.get(p)
        return None

    def line(self, n):
        return self.lines[n - 1].strip() if 0 < n <= len(self.lines) else ""


def _call_name(func):
    if isinstance(func, ast.Name):
        return None, func.id
    if isinstance(func, ast.Attribute):
        base = func.value
        if isinstance(base, ast.Name):
            return base.id, func.attr
        return "?", func.attr
    return None, None


def _is_query_call(mod, node, query_funcs):
    """node: ast.Call -> (Art, Zielname) oder None."""
    base, name = _call_name(node.func)
    if name is None:
        return None
    if base is None:
        src = mod.from_imports.get(name)
        if src and src[0] == COMMON_MOD and src[1] in QUERY_FUNCS:
            return QUERY_FUNCS[src[1]], src[1]
        if src and (src[0], src[1]) in query_funcs:
            return f"Abfrage-Funktion {src[1]}()", src[1]
        if (mod.modname, name) in query_funcs:
            return f"Abfrage-Funktion {name}()", name
        if mod.modname == COMMON_MOD.replace("g1pilot.", "g1pilot.") and name in QUERY_FUNCS \
                and mod.rel == "g1pilot/utils/common.py":
            return QUERY_FUNCS[name], name
        return None
    if base == "self":
        return None
    target = mod.mod_alias.get(base) or (".".join(mod.from_imports[base])
                                         if base in mod.from_imports else None)
    if target == COMMON_MOD and name in QUERY_FUNCS:
        return QUERY_FUNCS[name], name
    if target and (target, name) in query_funcs:
        return f"Abfrage-Funktion {name}()", name
    return None


def _tainted_use(mod, n, tainted):
    if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Load) and n.attr in tainted["attr"]:
        return True
    if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load):
        if (mod.rel, n.id) in tainted["global"]:
            return True
        f = mod.enclosing_func(n)
        if f is not None and (mod.rel, id(f), n.id) in tainted["local"]:
            return True
    return False


def find_query_funcs(mods):
    """Funktionen, deren RUECKGABE die Betriebsart ist (Wrapper/Praedikate).
    Fixpunkt ueber alle Module (Wrapper von Wrappern)."""
    qf = set()
    changed = True
    while changed:
        changed = False
        for mod in mods:
            for node in ast.walk(mod.tree):
                if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    continue
                if mod.parents.get(node) is not mod.tree:
                    continue           # nur Modulfunktionen (Methoden: siehe tainted attr)
                key = (mod.modname, node.name)
                if key in qf or (mod.rel == "g1pilot/utils/common.py" and node.name in QUERY_FUNCS):
                    continue
                for r in ast.walk(node):
                    if isinstance(r, ast.Return) and r.value is not None:
                        if _mode_flag(mod, r.value, qf, {"attr": set(), "global": set(),
                                                         "local": set()}):
                            qf.add(key)
                            changed = True
                            break
    return qf


def _mode_flag(mod, expr, qf, tainted):
    """Ist expr selbst ein Betriebsart-Wert (nicht nur davon abhaengig)?
    Abfrage, abgeleiteter Name, not/and/or davon, bool(...), Vergleich mit
    dem Inhalt von G1_SIM_MODE."""
    if isinstance(expr, ast.Call):
        if _is_query_call(mod, expr, qf):
            return True
        _, name = _call_name(expr.func)
        if name == "bool" and expr.args:
            return _mode_flag(mod, expr.args[0], qf, tainted)
        return False
    if isinstance(expr, (ast.Name, ast.Attribute)):
        return _tainted_use(mod, expr, tainted)
    if isinstance(expr, ast.UnaryOp) and isinstance(expr.op, ast.Not):
        return _mode_flag(mod, expr.operand, qf, tainted)
    if isinstance(expr, ast.BoolOp):
        return any(_mode_flag(mod, v, qf, tainted) for v in expr.values)
    if isinstance(expr, ast.Compare):
        return any(isinstance(n, ast.Constant) and n.value == ENV_NAME for n in ast.walk(expr))
    return False


def find_tainted(mods, qf):
    """Namen, die eine Betriebsart-Kopie sind: ALLE Zuweisungen an den Namen
    sind selbst Betriebsart-Werte (z. B. DemoGUI.sim_mode = is_sim_mode()).
    Zustandsvariablen, die nur mit der Betriebsart VORBELEGT werden und
    spaeter andere Werte bekommen (z. B. started), zaehlen nicht."""
    assigns = {}
    for mod in mods:
        for node in ast.walk(mod.tree):
            if isinstance(node, ast.Assign):
                targets, value = node.targets, node.value
            elif isinstance(node, (ast.AnnAssign, ast.AugAssign)) and node.value is not None:
                targets, value = [node.target], node.value
            else:
                continue
            for tg in targets:
                if isinstance(tg, ast.Attribute):
                    key = ("attr", tg.attr)
                elif isinstance(tg, ast.Name):
                    f = mod.enclosing_func(node)
                    key = ("global", (mod.rel, tg.id)) if f is None else \
                        ("local", (mod.rel, id(f), tg.id))
                else:
                    continue
                assigns.setdefault(key, []).append((mod, node, value))
    t = {"attr": {}, "global": {}, "local": {}}
    changed = True
    while changed:
        changed = False
        view = {"attr": set(t["attr"]), "global": set(t["global"]), "local": set(t["local"])}
        for (bucket, name), lst in assigns.items():
            if name in t[bucket]:
                continue
            if all(_mode_flag(m, v, qf, view) for m, _n, v in lst):
                m, n, _v = lst[0]
                t[bucket][name] = f"{m.rel}:{n.lineno}"
                changed = True
    return t


def _effect(mod, node):
    """Wirkung der Fundstelle aus dem Kontext (Zweig, Wertwahl, ...)."""
    child, p = node, mod.parents.get(node)
    negated = False
    while p is not None:
        if isinstance(p, ast.UnaryOp) and isinstance(p.op, ast.Not):
            negated = not negated
        elif isinstance(p, ast.BoolOp):
            pass
        elif isinstance(p, (ast.If, ast.While)) and child is p.test:
            sim_branch = not negated
            kind = "Sim-Zweig" if sim_branch else "Real-Zweig"
            body = p.body[0] if p.body else None
            first = mod.line(body.lineno) if body is not None else ""
            other = " (mit else)" if p.orelse else ""
            return f"Verzweigung: {kind}{other} -> {first[:90]}"
        elif isinstance(p, ast.IfExp) and child is p.test:
            a = ast.unparse(p.body) if hasattr(ast, "unparse") else "..."
            b = ast.unparse(p.orelse) if hasattr(ast, "unparse") else "..."
            if negated:
                a, b = b, a
            return f"Wertwahl: Sim {a[:50]} / Real {b[:50]}"
        elif isinstance(p, ast.Call):
            base, name = _call_name(p.func)
            if name == "declare_parameter":
                return "Parameter-Default je Betriebsart"
            if name in ("setVisible", "setEnabled"):
                return f"Oberflaeche: {name}"
            if child in p.args or any(child is k.value for k in p.keywords):
                return f"Argument an {name}()"
        elif isinstance(p, (ast.Assign, ast.AnnAssign)):
            tg = p.targets[0] if isinstance(p, ast.Assign) else p.target
            return f"Zuweisung an {ast.unparse(tg) if hasattr(ast, 'unparse') else '?'}"
        elif isinstance(p, ast.Return):
            return "Rueckgabewert"
        elif isinstance(p, (ast.Expr,)):
            return "Aufruf (Seiteneffekt)"
        elif isinstance(p, (ast.stmt,)):
            break
        child, p = p, mod.parents.get(p)
    return "sonstige Verwendung"


def _doc1(func):
    if func is None:
        return ""
    d = ast.get_docstring(func) or ""
    return d.strip().splitlines()[0][:100] if d.strip() else ""


def scan_python(include_tests):
    mods = []
    errors = []
    for p in python_files(include_tests):
        try:
            mods.append(_Module(p))
        except SyntaxError as e:
            errors.append(f"{_rel(p)}: {e}")
    qf = find_query_funcs(mods)
    tainted = find_tainted(mods, qf)
    tview = {"attr": set(tainted["attr"]), "global": set(tainted["global"]),
             "local": set(tainted["local"])}
    rows = []
    seen = set()
    funcs_with_query = {}
    for mod in mods:
        for node in ast.walk(mod.tree):
            kind, detail = None, ""
            if isinstance(node, ast.Call):
                q = _is_query_call(mod, node, qf)
                if q:
                    kind, detail = q
            elif isinstance(node, ast.Constant) and node.value == ENV_NAME:
                p = mod.parents.get(node)
                ctx = "Umgebungsvariable gelesen"
                if isinstance(p, ast.Call):
                    _, name = _call_name(p.func)
                    if name in ("setenv",):
                        ctx = "Umgebungsvariable gesetzt (Test)"
                    elif name == "pop":
                        ctx = "Umgebungsvariable entfernt"
                elif isinstance(p, ast.Subscript) and isinstance(mod.parents.get(p), ast.Assign) \
                        and p in getattr(mod.parents.get(p), "targets", []):
                    ctx = "Umgebungsvariable gesetzt"
                elif isinstance(p, ast.keyword) or isinstance(p, ast.Dict):
                    ctx = "Umgebungsvariable gesetzt (Kindprozess)"
                kind, detail = ctx, ENV_NAME
            elif isinstance(node, ast.Name) and node.id in QUERY_CONSTS \
                    and isinstance(node.ctx, ast.Load):
                kind, detail = "DDS-Konstante", node.id
            elif isinstance(node, ast.Attribute) and node.attr in ("init_dds", "resolve_dds",
                                                                  "is_sim_mode") \
                    and isinstance(node.ctx, ast.Store):
                kind, detail = "Ersetzung der Funktion (Attrappe)", node.attr
            elif _tainted_use(mod, node, tview):
                name = node.attr if isinstance(node, ast.Attribute) else node.id
                src = (tainted["attr"].get(name) if isinstance(node, ast.Attribute)
                       else tainted["global"].get((mod.rel, name))
                       or next((w for (r, _f, n), w in tainted["local"].items()
                                if r == mod.rel and n == name), ""))
                kind, detail = f"abgeleitet ueber '{name}'", f"gesetzt in {src}"
            if kind is None:
                continue
            line = getattr(node, "lineno", 0)
            key = (mod.rel, line, kind, detail)
            if key in seen:
                continue
            seen.add(key)
            qn = mod.qualname(node)
            func = mod.enclosing_func(node)
            desc = CATALOG.get((mod.rel, qn))
            rows.append({
                "datei": mod.rel, "zeile": line, "bereich": _bereich(mod.rel),
                "funktion": qn, "art_der_abfrage": kind, "detail": detail,
                "wirkung": _effect(mod, node),
                "unterschied": desc or ("(Test)" if _bereich(mod.rel) == "Test"
                                        else "nicht katalogisiert -- Kontext pruefen"),
                "katalog": "ja" if desc else "nein",
                "docstring": _doc1(func), "code": mod.line(line)[:140],
            })
            if func is not None and _bereich(mod.rel) != "Test":
                funcs_with_query.setdefault((mod.modname, mod.qualname(func) if False else
                                             _qual_of(mod, func)), (mod, func))
    rows.sort(key=lambda r: (r["datei"], int(r["zeile"])))
    indirect = find_indirect(mods, funcs_with_query, qf)
    return rows, indirect, sorted(f"{m}.{n}" for m, n in qf), \
        {k: v for k, v in tainted["attr"].items()}, errors


def _qual_of(mod, func):
    names = [func.name]
    p = mod.parents.get(func)
    while p is not None:
        if isinstance(p, (ast.FunctionDef, ast.ClassDef, ast.AsyncFunctionDef)):
            names.append(p.name)
        p = mod.parents.get(p)
    return ".".join(reversed(names))


def find_indirect(mods, funcs_with_query, qf):
    """Eine Ebene: Aufrufe von Funktionen/Methoden, die selbst die
    Betriebsart abfragen (aber nicht nur zurueckgeben)."""
    by_mod_func = {}
    for (modname, qual), (mod, func) in funcs_with_query.items():
        if (modname, qual) in qf or "." in qual and qual.split(".")[-1] == "__init__":
            continue
        by_mod_func[(modname, qual)] = (mod, func)
    rows = []
    for mod in mods:
        for node in ast.walk(mod.tree):
            if not isinstance(node, ast.Call):
                continue
            base, name = _call_name(node.func)
            if name is None:
                continue
            hit = None
            if base == "self":
                cls = mod.qualname(node).split(".")[0]
                hit = by_mod_func.get((mod.modname, f"{cls}.{name}"))
            elif base is None:
                hit = by_mod_func.get((mod.modname, name))
                src = mod.from_imports.get(name)
                if hit is None and src:
                    hit = by_mod_func.get((src[0], src[1]))
            if hit is None:
                continue
            tmod, tfunc = hit
            if mod is tmod and mod.enclosing_func(node) is tfunc:
                continue                       # Rekursion
            rows.append({
                "datei": mod.rel, "zeile": node.lineno, "bereich": _bereich(mod.rel),
                "funktion": mod.qualname(node),
                "ruft": f"{tmod.rel}::{_qual_of(tmod, tfunc)}",
                "code": mod.line(node.lineno)[:140],
            })
    rows.sort(key=lambda r: (r["datei"], int(r["zeile"])))
    return rows


# ══════════════════════════════════════════════════════════════════════
# 2. Textdateien (Shell, Compose, Make, Docker)
# ══════════════════════════════════════════════════════════════════════
TEXT_GLOBS = (".sh", ".yml", ".yaml", "Makefile", "Dockerfile.sim", "Dockerfile.real",
              "Dockerfile.mujoco")


def scan_text():
    rows = []
    for dirpath, dirnames, filenames in os.walk(ec.PKG_ROOT):
        dirnames[:] = [d for d in dirnames if d not in ("build", "install", "log", "data")
                       and not d.startswith(".")]
        for fn in filenames:
            if not (fn.endswith((".sh", ".yml", ".yaml")) or fn in ("Makefile",)
                    or fn.startswith("Dockerfile")):
                continue
            p = os.path.join(dirpath, fn)
            rel = _rel(p)
            if rel.startswith("tools/eval"):
                continue
            with open(p, "r", encoding="utf-8", errors="replace") as f:
                for i, line in enumerate(f, 1):
                    if ENV_NAME in line and not line.lstrip().startswith("#"):
                        m = re.search(ENV_NAME + r"\s*[=:]\s*\$?\{?([A-Za-z0-9_:\-]+)", line)
                        rows.append({"datei": rel, "zeile": i, "wert": m.group(1) if m else "",
                                     "code": line.strip()[:140]})
    rows.sort(key=lambda r: (r["datei"], r["zeile"]))
    return rows


# ══════════════════════════════════════════════════════════════════════
# 3. Launch: bringup_sim gegen bringup_real
# ══════════════════════════════════════════════════════════════════════
def _src(node):
    try:
        return ast.unparse(node)
    except Exception:
        return "?"


def _kw(call, name):
    for k in call.keywords:
        if k.arg == name:
            return k.value
    return None


def _dict_items(node):
    """dict-Literal (auch {...}.items()) -> {schluessel: quelltext}."""
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
            and node.func.attr == "items":
        node = node.func.value
    if isinstance(node, ast.IfExp):
        node = node.body
    out = {}
    if isinstance(node, ast.Dict):
        for k, v in zip(node.keys, node.values):
            if isinstance(k, ast.Constant):
                out[str(k.value)] = _src(v)
    return out


def launch_blocks(path):
    """Nodes/Includes einer Launch-Datei -> {Schluessel: {param: wert}}."""
    with open(path, "r", encoding="utf-8") as f:
        tree = ast.parse(f.read())
    parents = {}
    for n in ast.walk(tree):
        for ch in ast.iter_child_nodes(n):
            parents[ch] = n

    def guard(node):
        """Bedingung eines umschliessenden if-Blocks (z. B. if enable_nav)."""
        conds, child, p = [], node, parents.get(node)
        while p is not None:
            if isinstance(p, ast.If):
                txt = _src(p.test)
                conds.append(txt if child not in p.orelse else f"not ({txt})")
            child, p = p, parents.get(p)
        return " and ".join(reversed(conds))

    blocks = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        _, name = _call_name(node.func)
        if name == "Node":
            exe = _kw(node, "executable")
            nm = _kw(node, "name")
            key = "Node " + (nm.value if isinstance(nm, ast.Constant) else
                             (exe.value if isinstance(exe, ast.Constant) else _src(exe)))
            params = {}
            pv = _kw(node, "parameters")
            if isinstance(pv, ast.List):
                for el in pv.elts:
                    params.update(_dict_items(el))
            args = _kw(node, "arguments")
            if args is not None:
                params["<arguments>"] = _src(args)
            cond = _kw(node, "condition")
            if cond is not None:
                params["<condition>"] = _src(cond)
            g = guard(node)
            if g:
                params["<nur wenn>"] = g
            blocks[key] = params
        elif name == "IncludeLaunchDescription":
            target = "?"
            for n in ast.walk(node.args[0]) if node.args else []:
                if isinstance(n, ast.Constant) and isinstance(n.value, str) \
                        and n.value.endswith(".launch.py"):
                    target = n.value
            la = _kw(node, "launch_arguments")
            blocks["Include " + target] = _dict_items(la) if la is not None else {}
    return blocks


def compare_launch():
    sim = launch_blocks(os.path.join(ec.PKG_ROOT, "launch", "bringup_sim.launch.py"))
    real = launch_blocks(os.path.join(ec.PKG_ROOT, "launch", "bringup_real.launch.py"))
    rows = []
    for key in sorted(set(sim) | set(real)):
        if key not in real:
            g = sim[key].get("<nur wenn>", "")
            rows.append({"baustein": key, "schluessel": "", "sim": "gestartet"
                         + (f" (nur wenn {g})" if g else ""),
                         "real": "-", "unterschied": "nur Sim"})
            continue
        if key not in sim:
            g = real[key].get("<nur wenn>", "")
            rows.append({"baustein": key, "schluessel": "", "sim": "-",
                         "real": "gestartet" + (f" (nur wenn {g})" if g else ""),
                         "unterschied": "nur Real"})
            continue
        a, b = sim[key], real[key]
        for p in sorted(set(a) | set(b)):
            va, vb = a.get(p, "(Default)"), b.get(p, "(Default)")
            rows.append({"baustein": key, "schluessel": p, "sim": va, "real": vb,
                         "unterschied": "gleich" if va == vb else
                         ("nur in Sim gesetzt" if vb == "(Default)" else
                          "nur in Real gesetzt" if va == "(Default)" else "verschieden")})
    return rows


# ══════════════════════════════════════════════════════════════════════
# 4. Dockerfiles
# ══════════════════════════════════════════════════════════════════════
def dockerfile(path):
    with open(path, "r", encoding="utf-8") as f:
        raw = f.read()
    raw = re.sub(r"\\\s*\n", " ", raw)              # Fortsetzungszeilen
    info = {"FROM": [], "ENV": {}, "apt": set(), "pip": set(), "git": [], "COPY": [], "RUN": []}
    for line in raw.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        instr, _, rest = s.partition(" ")
        instr = instr.upper()
        if instr == "FROM":
            info["FROM"].append(rest.strip())
        elif instr == "ENV":
            for m in re.finditer(r'(\w+)=("[^"]*"|\S+)', rest):
                info["ENV"][m.group(1)] = m.group(2).strip('"')
            if "=" not in rest:
                k, _, v = rest.partition(" ")
                info["ENV"][k] = v.strip()
        elif instr == "COPY":
            info["COPY"].append(rest.strip())
        elif instr == "RUN":
            info["RUN"].append(rest.strip())
            for part in re.split(r"&&|;", rest):
                toks = part.split()
                if "apt-get" in toks and "install" in toks:
                    for t in toks[toks.index("install") + 1:]:
                        if not t.startswith("-"):
                            info["apt"].add(t.replace("${ROS_DISTRO}", "humble"))
                if ("pip3" in toks or "pip" in toks) and "install" in toks:
                    i = toks.index("install")
                    for t in toks[i + 1:]:
                        if t.startswith("-") or t == ".":
                            continue
                        info["pip"].add(t.strip('"'))
                if "git" in toks and "clone" in toks:
                    info["git"].append(toks[toks.index("clone") + 1])
                if "git" in toks and "checkout" in toks:
                    info["git"].append("checkout " + toks[toks.index("checkout") + 1])
    return info


def _pkgname(p):
    return re.split(r"[<>=!~]", p.strip('"'), maxsplit=1)[0].lower()


def compare_images():
    d = os.path.join(ec.PKG_ROOT, "docker")
    sim = dockerfile(os.path.join(d, "Dockerfile.sim"))
    real = dockerfile(os.path.join(d, "Dockerfile.real"))
    muj = dockerfile(os.path.join(d, "Dockerfile.mujoco"))
    rows = []

    def add(kat, key, a, b):
        rows.append({"kategorie": kat, "eintrag": key, "sim": a, "real": b,
                     "unterschied": "gleich" if a == b else
                     ("nur Sim" if b == "-" else "nur Real" if a == "-" else "verschieden")})

    add("Basis", "FROM", " | ".join(sim["FROM"]), " | ".join(real["FROM"]))
    for kat, ka in (("apt", "apt"),):
        for pkg in sorted(sim[ka] | real[ka]):
            add(kat, pkg, "ja" if pkg in sim[ka] else "-", "ja" if pkg in real[ka] else "-")
    ps = {_pkgname(p): p for p in sim["pip"]}
    pr = {_pkgname(p): p for p in real["pip"]}
    for name in sorted(set(ps) | set(pr)):
        add("pip", name, ps.get(name, "-"), pr.get(name, "-"))
    for k in sorted(set(sim["ENV"]) | set(real["ENV"])):
        add("ENV", k, sim["ENV"].get(k, "-"), real["ENV"].get(k, "-"))
    gs, gr = set(sim["git"]), set(real["git"])
    for g in sorted(gs | gr):
        add("Quelle (git)", g, "ja" if g in gs else "-", "ja" if g in gr else "-")
    cs, cr = set(sim["COPY"]), set(real["COPY"])
    for c in sorted(cs | cr):
        add("COPY", c, "ja" if c in cs else "-", "ja" if c in cr else "-")
    # Zweites Sim-Image (Physik): komplett "nur Sim"
    add("Image mujoco (nur Sim)", "FROM", " | ".join(muj["FROM"]), "-")
    for pkg in sorted(muj["apt"]):
        add("Image mujoco (nur Sim)", "apt " + pkg, "ja", "-")
    for p in sorted(muj["pip"]):
        add("Image mujoco (nur Sim)", "pip " + p, p, "-")
    for k, v in sorted(muj["ENV"].items()):
        add("Image mujoco (nur Sim)", "ENV " + k, v, "-")
    return rows


# ══════════════════════════════════════════════════════════════════════
# 5. Konfiguration (Compose, start.sh)
# ══════════════════════════════════════════════════════════════════════
def _env_list(env):
    out = {}
    if isinstance(env, dict):
        return {str(k): str(v) for k, v in env.items()}
    for e in env or []:
        k, _, v = str(e).partition("=")
        out[k] = v
    return out


def compare_compose():
    try:
        import yaml
    except ImportError:
        return [{"bereich": "compose", "schluessel": "(PyYAML fehlt -- pip install pyyaml)",
                 "sim": "", "real": "", "unterschied": "nicht geprueft"}]
    with open(os.path.join(ec.PKG_ROOT, "docker-compose.yml"), "r", encoding="utf-8") as f:
        dc = yaml.safe_load(f)
    svc = dc.get("services", {})
    s, r = svc.get("g1pilot-sim", {}), svc.get("g1pilot-real", {})
    rows = []

    def add(bereich, key, a, b):
        a = "-" if a is None else a
        b = "-" if b is None else b
        rows.append({"bereich": bereich, "schluessel": key, "sim": a, "real": b,
                     "unterschied": "gleich" if a == b else
                     ("nur Sim" if b == "-" else "nur Real" if a == "-" else "verschieden")})

    es, er = _env_list(s.get("environment")), _env_list(r.get("environment"))
    for k in sorted(set(es) | set(er)):
        add("compose environment", k, es.get(k), er.get(k))
    vs, vr = set(map(str, s.get("volumes") or [])), set(map(str, r.get("volumes") or []))
    for v in sorted(vs | vr):
        add("compose volumes", v, "ja" if v in vs else None, "ja" if v in vr else None)
    for key in ("image", "privileged", "security_opt", "devices", "device_cgroup_rules",
                "stop_grace_period", "depends_on", "profiles"):
        a, b = s.get(key), r.get(key)
        if a is None and b is None:
            continue
        add("compose " + key, key, None if a is None else str(a), None if b is None else str(b))
    cs = " ".join(str(s.get("command", "")).split())
    cr = " ".join(str(r.get("command", "")).split())
    add("compose command", "command", cs, cr)
    for name, d in sorted(svc.items()):
        if name in ("g1pilot-sim", "g1pilot-real"):
            continue
        prof = d.get("profiles") or []
        if "sim" in prof and "real" not in prof:
            add("compose Dienst", name, "Dienst laeuft (" + str(d.get("image", "")) + ")", None)
        elif "real" in prof and "sim" not in prof:
            add("compose Dienst", name, None, "Dienst laeuft (" + str(d.get("image", "")) + ")")
    # Overrides (nur fuer einen Dienst)
    for fn in sorted(os.listdir(ec.PKG_ROOT)):
        if not (fn.startswith("docker-compose.") and fn.endswith(".yml")) or fn == "docker-compose.yml":
            continue
        with open(os.path.join(ec.PKG_ROOT, fn), "r", encoding="utf-8") as f:
            ov = yaml.safe_load(f) or {}
        for name in (ov.get("services") or {}):
            add("compose Override", f"{fn} -> {name}",
                "ja" if name in ("g1pilot-sim", "mujoco-sim") else None,
                "ja" if name == "g1pilot-real" else None)
    rows.extend(compare_start_sh())
    return rows


def compare_start_sh():
    p = os.path.join(ec.PKG_ROOT, "start.sh")
    if not os.path.exists(p):
        return []
    with open(p, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()
    start = next((i for i, l in enumerate(lines)
                  if re.match(r'^if \[ "\$G1_MODE" = "real" \]', l)), None)
    if start is None:
        return [{"bereich": "start.sh", "schluessel": "(Real/Sim-Verzweigung nicht gefunden)",
                 "sim": "", "real": "", "unterschied": "nicht geprueft"}]
    els = next((i for i in range(start + 1, len(lines)) if lines[i].startswith("else")), None)
    end = next((i for i in range((els or start) + 1, len(lines)) if lines[i].startswith("fi")), None)

    def exports(a, b):
        out = {}
        for l in lines[a:b]:
            m = re.match(r"\s*(?:if .*then )?export (\w+)=(\S*)", l)
            if m:
                out.setdefault(m.group(1), []).append(m.group(2).rstrip(";"))
        return {k: " | ".join(dict.fromkeys(v)) for k, v in out.items()}

    real = exports(start, els)
    sim = exports(els, end)
    rows = []
    for k in sorted(set(sim) | set(real)):
        a, b = sim.get(k, "-"), real.get(k, "-")
        rows.append({"bereich": "start.sh export", "schluessel": k, "sim": a, "real": b,
                     "unterschied": "gleich" if a == b else
                     ("nur Sim" if b == "-" else "nur Real" if a == "-" else "verschieden")})
    return rows


# ══════════════════════════════════════════════════════════════════════
# Ausgabe
# ══════════════════════════════════════════════════════════════════════
def _md_table(cols, rows, titles=None):
    titles = titles or cols
    out = ["| " + " | ".join(titles) + " |", "|" + "---|" * len(cols)]
    for r in rows:
        out.append("| " + " | ".join(str(r.get(c, "")).replace("|", "\\|").replace("\n", " ")
                                     for c in cols) + " |")
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="V1: Code-Vergleich Simulation/Realbetrieb (statisch, startet nichts).")
    ap.add_argument("--include-tests", action="store_true",
                    help="auch test/ durchsuchen (Tests setzen/ersetzen den Schalter)")
    ec.add_common_args(ap, VERSUCH)
    a = ap.parse_args(argv)

    run = ec.Run("v1_code_vergleich.py", VERSUCH, ec.MODE_STATIC,
                 {"include_tests": a.include_tests, "paket": ec.PKG_ROOT}, out_root=a.out)
    q, ind, qf, tattr, errors = scan_python(a.include_tests)
    txt = scan_text()
    launch = compare_launch()
    images = compare_images()
    conf = compare_compose()

    qcols = ["datei", "zeile", "bereich", "funktion", "art_der_abfrage", "detail", "wirkung",
             "unterschied", "katalog", "docstring", "code"]
    run.write_csv("abfragen.csv", qcols, q, "Stellen, an denen der Code die Betriebsart abfragt")
    run.write_csv("indirekt.csv", ["datei", "zeile", "bereich", "funktion", "ruft", "code"], ind,
                  "Aufrufe von Funktionen, die intern die Betriebsart abfragen (eine Ebene)")
    run.write_csv("text_fundstellen.csv", ["datei", "zeile", "wert", "code"], txt,
                  "G1_SIM_MODE in Shell/Compose/Make/Docker")
    lcols = ["baustein", "schluessel", "sim", "real", "unterschied"]
    run.write_csv("launch.csv", lcols, launch, "bringup_sim.launch.py gegen bringup_real.launch.py")
    icols = ["kategorie", "eintrag", "sim", "real", "unterschied"]
    run.write_csv("images.csv", icols, images, "Dockerfile.sim gegen Dockerfile.real (+ mujoco)")
    ccols = ["bereich", "schluessel", "sim", "real", "unterschied"]
    run.write_csv("konfiguration.csv", ccols, conf, "docker-compose + start.sh je Betriebsart")

    def count(rows, key="unterschied"):
        c = {}
        for r in rows:
            c[r[key]] = c.get(r[key], 0) + 1
        return c

    by_file = {}
    for r in q:
        if r["bereich"] != "Test":
            by_file[r["datei"]] = by_file.get(r["datei"], 0) + 1
    summary = {
        "abfragen_gesamt": len(q),
        "abfragen_ohne_tests": sum(1 for r in q if r["bereich"] != "Test"),
        "abfragen_je_bereich": count(q, "bereich"),
        "abfragen_je_art": count(q, "art_der_abfrage"),
        "abfragen_je_datei": by_file,
        "nicht_katalogisiert": [f"{r['datei']}:{r['zeile']} {r['funktion']}" for r in q
                                if r["katalog"] == "nein" and r["bereich"] != "Test"],
        "abfrage_funktionen": qf,
        "abgeleitete_attribute": tattr,
        "indirekte_aufrufe": len(ind),
        "text_fundstellen": len(txt),
        "launch": count(launch), "images": count(images), "konfiguration": count(conf),
        "parse_fehler": errors,
        "grenzen": [
            "statische Analyse: dynamische Zugriffe (getattr, eval, Strings) werden nicht erkannt",
            "abgeleitete Attribute werden ueber den NAMEN verfolgt (jedes .sim_mode zaehlt), "
            "auch wenn ein anderes Objekt gleichnamige Attribute hat",
            "indirekt.csv nur eine Aufrufebene",
            "unitree_mujoco/ fragt G1_SIM_MODE nicht ab (laeuft nur in der Sim) und wird nicht durchsucht",
        ],
    }
    md = ["# V1 Code-Vergleich Simulation / Realbetrieb", "",
          f"Commit `{run.meta['git']['commit']}`, erzeugt {run.meta['zeitpunkt']}.", "",
          "## Abfragen der Betriebsart (ohne Tests)", "",
          _md_table(["datei", "zeile", "funktion", "art_der_abfrage", "wirkung", "unterschied"],
                    [r for r in q if r["bereich"] != "Test"],
                    ["Datei", "Zeile", "Funktion", "Art der Abfrage", "Wirkung", "Unterschied"]),
          "", "## G1_SIM_MODE in Shell/Compose/Make/Docker", "",
          _md_table(["datei", "zeile", "wert", "code"], txt, ["Datei", "Zeile", "Wert", "Zeile im Code"]),
          "", "## Launch (bringup_sim gegen bringup_real), nur Unterschiede", "",
          _md_table(lcols, [r for r in launch if r["unterschied"] != "gleich"],
                    ["Baustein", "Schluessel", "Sim", "Real", "Unterschied"]),
          "", "## Container-Images, nur Unterschiede", "",
          _md_table(icols, [r for r in images if r["unterschied"] != "gleich"],
                    ["Kategorie", "Eintrag", "Sim", "Real", "Unterschied"]),
          "", "## Konfiguration (Compose, start.sh), nur Unterschiede", "",
          _md_table(ccols, [r for r in conf if r["unterschied"] != "gleich"],
                    ["Bereich", "Schluessel", "Sim", "Real", "Unterschied"]), ""]
    with open(run.path("tabelle.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md))
    run.files.append("tabelle.md")
    run.finish(summary)
    print(f"[v1] {summary['abfragen_ohne_tests']} Abfragen (ohne Tests) in "
          f"{len(by_file)} Dateien, {len(summary['nicht_katalogisiert'])} nicht katalogisiert; "
          f"Launch-Unterschiede {sum(v for k, v in summary['launch'].items() if k != 'gleich')}, "
          f"Image-Unterschiede {sum(v for k, v in summary['images'].items() if k != 'gleich')}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Node-/Topic-Diagramme des g1pilot-Stacks (docs/images/ros_graph_*.svg).

Die Tabellen unten sind von Hand aus dem Code abgeleitet (create_publisher /
create_subscription / ChannelPublisher / ChannelSubscriber in g1pilot/g1pilot,
Launch-Bedingungen in launch/). Das Skript prueft sie gegen den Code:

  * jedes eingetragene ROS-Topic muss als String im Code vorkommen,
  * jedes Topic-Literal aus einem create_publisher/create_subscription-Aufruf
    im Code muss hier eingetragen sein (sonst Warnung + Exit-Code 1).

Aufruf (braucht Graphviz `dot`, nur zum Zeichnen; --check geht ohne):

    python3 tools/ros_graph.py            # pruefen + SVGs schreiben
    python3 tools/ros_graph.py --check    # nur pruefen

Beschreibung und Lesehilfe: docs/05-T-Nodes-und-Topics.md
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

PKG = pathlib.Path(__file__).resolve().parents[1]
SRC = PKG / "g1pilot"
OUT_DIR = PKG / "docs" / "images"

# ── Kategorien (Farben wie in der Vorlage) ──────────────────────────────────
CATS = {
    "teleop": ("Teleoperation", "#fdf3b0", "#b8a23a"),
    "gui":    ("Bedien-GUI",    "#e6dcf7", "#8a6cc2"),
    "nav":    ("Navigation / Fortbewegung", "#bff0d4", "#3fa36b"),
    "manip":  ("Manipulation",  "#fbd8b8", "#c98142"),
    "state":  ("Zustand",       "#c9def7", "#4f7fbf"),
    "ext":    ("Extern (Roboter, Sim, Hardware)", "#e4e4e4", "#7a7a7a"),
}
SCOPE_TAG = {"sim": "nur Sim", "real": "nur Real"}
SCOPE_COLOR = {"sim": "#1f6fb2", "real": "#c0392b", "demo": "#8a6cc2"}

# id: (Anzeigename, Kategorie, Scope, Zusatzzeile)
#   Scope: "both" | "sim" | "real". Zusatzzeile: Startbedingung o. ae.
NODES = {
    "joystick":       ("joystick", "teleop", "both", "ohne G1_PS4_ARMS"),
    "ps4_joystick":   ("ps4_joystick", "teleop", "both", "G1_PS4_ARMS=1"),
    "ps4_arm_teleop": ("ps4_arm_teleop", "teleop", "both", "G1_PS4_ARMS=1"),
    "ps4_walk":       ("ps4_walk", "teleop", "both", "G1_PS4_ARMS=1 + G1_PS4_WALK"),
    "joy_mux":        ("joy_mux", "teleop", "both", ""),
    "gui":            ("ui_interface | demo_gui", "gui", "both",
                       "G1_GUI=streamdeck | demo"),
    "robot_state":    ("robot_state", "state", "both", ""),
    "arm_controller": ("arm_controller", "manip", "both", ""),
    "interactive_marker": ("interactive_marker", "manip", "both", ""),
    "arm_api":        ("arm_api", "manip", "both", "HTTP :8770"),
    "inspire":        ("inspire_ftp_bridge", "manip", "both", "G1_INSPIRE_HANDS=1"),
    "loco_sim":       ("loco_sim", "nav", "sim", "RL-Lauf-Policy"),
    "loco_client":    ("loco_client", "nav", "real", "Unitree LocoClient"),
    "scene_bridge":   ("scene_bridge", "nav", "sim", ""),
    "create_map":     ("create_map", "nav", "sim", "G1_ENABLE_NAV=1"),
    "sim_localization": ("sim_localization", "nav", "sim", "G1_ENABLE_NAV=1"),
    "joy_to_cmdvel":  ("joy_to_cmdvel", "nav", "sim", "G1_ENABLE_NAV=1"),
    "mola_fixed":     ("mola_fixed", "nav", "real", "G1_ENABLE_LIDAR=1"),
    "dijkstra":       ("dijkstra_planner", "nav", "both",
                       "Sim: G1_ENABLE_NAV, Real: G1_ENABLE_LIDAR"),
    "nav2point":      ("nav2point", "nav", "both",
                       "Sim: G1_ENABLE_NAV, Real: G1_ENABLE_LIDAR"),
    # Extern
    "mujoco":   ("MuJoCo-Simulation", "ext", "sim", "unitree_sdk2py_bridge"),
    "g1":       ("G1 onboard (Unitree)", "ext", "real", "Motoren, Loco-Dienst"),
    "hands_hw": ("Inspire-Haende", "ext", "real", "Modbus TCP"),
    "mola":     ("Livox MID360 + MOLA", "ext", "real", "LiDAR-Odometrie"),
    "rsp":      ("robot_state_publisher", "ext", "both", "URDF -> /tf"),
    "phone":    ("Handy (Browser)", "ext", "both", "Web Bluetooth -> Demo-GUI"),
    "http":     ("HTTP-Client", "ext", "both", "Skripte, Fremdprogramme"),
}

# Topics: name -> (Typ, Publisher, Subscriber, Art)
#   Publisher/Subscriber: Liste von Node-ids; Suffix ":sim"/":real" = Kante gilt
#   nur dort, ":demo" = nur in der Demo-GUI (nicht im Streamdeck).
#   Art: "ros" | "dds" (Unitree-DDS, kein ROS-Topic)
#   {left,right} steht fuer zwei Topics mit gleichem Aufbau.
TOPICS = {
    # ── Unitree-DDS ──
    "rt/lowstate": ("LowState_", ["mujoco:sim", "g1:real"],
                    ["robot_state", "arm_controller", "loco_sim:sim",
                     "sim_localization:sim", "loco_client:real"], "dds"),
    "rt/lowcmd": ("LowCmd_ (Beine)", ["loco_sim:sim"], ["mujoco:sim"], "dds"),
    "rt/arm_sdk": ("LowCmd_ (Arme, Taille)", ["arm_controller"],
                   ["mujoco:sim", "g1:real"], "dds"),
    "rt/sportmodestate": ("SportModeState_", ["mujoco:sim"],
                          ["sim_localization:sim"], "dds"),
    "rt/inspire/cmd": ("LowCmd_ (Finger)", ["inspire:sim"], ["mujoco:sim"], "dds"),
    "rt/inspire/state": ("LowState_ (Finger)", ["mujoco:sim"], ["inspire:sim"], "dds"),
    "rt/api/sport/request": ("Loco-RPC: SetFsmId, Move", ["loco_client:real"],
                             ["g1:real"], "dds"),
    # ── Eingabe / Joystick ──
    "/g1pilot/joy_manual": ("Joy", ["joystick"], ["joy_mux"], "ros"),
    "/g1pilot/auto_enable": ("Bool", ["joystick", "gui"], ["joy_mux", "nav2point"], "ros"),
    "/g1pilot/auto_joy": ("Joy", ["nav2point"], ["joy_mux"], "ros"),
    "/g1pilot/joy": ("Joy", ["joy_mux"], ["loco_client:real", "joy_to_cmdvel:sim"], "ros"),
    "/g1pilot/ps4/joy": ("Joy", ["ps4_joystick"], ["ps4_arm_teleop", "ps4_walk"], "ros"),
    "/g1pilot/ps4/feedback": ("String", ["ps4_arm_teleop", "ps4_walk"], ["ps4_joystick"], "ros"),
    "/g1pilot/ps4/device": ("String", ["ps4_joystick"], ["gui:demo"], "ros"),
    "/g1pilot/ps4/status": ("String", ["ps4_arm_teleop"], ["ps4_walk", "gui:demo"], "ros"),
    "/g1pilot/ps4/walk_status": ("String", ["ps4_walk"], ["gui:demo"], "ros"),
    "/g1pilot/ps4/walk_cmd": ("Twist", ["ps4_walk"], ["loco_sim:sim", "loco_client:real"], "ros"),
    "/g1pilot/ps4/markers": ("MarkerArray", ["ps4_arm_teleop"], [], "ros"),
    "/g1pilot/teleop_state": ("String", ["gui:demo", "ps4_arm_teleop", "ps4_walk"],
                              ["gui:demo", "ps4_arm_teleop", "ps4_walk"], "ros"),
    "/g1pilot/sequence/next": ("Bool", ["ps4_arm_teleop"], ["gui:demo"], "ros"),
    "/g1pilot/sequence/state": ("String", ["gui:demo"], ["ps4_arm_teleop"], "ros"),
    # ── Modus / Notaus ──
    "/g1pilot/start": ("Bool", ["gui"],
                       ["arm_controller", "loco_sim:sim", "loco_client:real",
                        "ps4_arm_teleop", "ps4_walk"], "ros"),
    "/g1pilot/start_balancing": ("Bool", ["gui", "ps4_arm_teleop"],
                                 ["arm_controller", "interactive_marker", "loco_sim:sim",
                                  "loco_client:real", "ps4_arm_teleop"], "ros"),
    "/g1pilot/start_walking": ("Bool", ["gui", "ps4_arm_teleop"],
                               ["arm_controller", "interactive_marker", "loco_sim:sim",
                                "loco_client:real", "ps4_arm_teleop"], "ros"),
    "/g1pilot/emergency_stop": ("Bool", ["gui", "ps4_arm_teleop"],
                                ["arm_controller", "loco_sim:sim", "loco_client:real",
                                 "inspire", "ps4_walk", "ps4_arm_teleop", "gui"], "ros"),
    "/g1pilot/full_emergency_stop": ("Bool", ["ps4_arm_teleop"],
                                     ["arm_controller", "loco_client:real"], "ros"),
    "/g1pilot/loco_cmd_vel": ("Twist", ["gui", "joy_to_cmdvel:sim"],
                              ["loco_sim:sim", "loco_client:real"], "ros"),
    "/g1pilot/loco_request": ("String", ["gui:demo"], ["loco_client:real"], "ros"),
    "/g1pilot/onboard_state": ("String", ["loco_client:real"],
                               ["ps4_walk", "ps4_arm_teleop", "gui:demo"], "ros"),
    "/g1pilot/loco_state": ("String", ["loco_sim:sim"], ["ps4_walk", "gui"], "ros"),
    # ── Arme ──
    "/g1pilot/arms/enabled": ("Bool", ["gui", "ps4_arm_teleop", "loco_sim:sim",
                                       "loco_client:real"], ["arm_controller"], "ros"),
    "/g1pilot/arms/home": ("Bool", ["gui", "ps4_arm_teleop", "loco_client:real"],
                           ["arm_controller"], "ros"),
    "/g1pilot/arms/state": ("Bool", ["arm_controller"], ["ps4_arm_teleop", "gui"], "ros"),
    "/g1pilot/arms/walk_ready": ("Bool", ["arm_controller"], ["loco_sim:sim", "gui:demo"], "ros"),
    "/g1pilot/hand_goal/{left,right}": ("PoseStamped",
                                        ["interactive_marker", "ps4_arm_teleop", "gui:demo"],
                                        ["arm_controller"], "ros"),
    "/g1pilot/hand_goal_status": ("String", ["arm_controller"], ["interactive_marker"], "ros"),
    "/g1pilot/marker_follow_ee": ("Bool", ["gui"], ["interactive_marker"], "ros"),
    "/g1pilot/hand_cmd_pose/{left,right}": ("PoseStamped", ["arm_controller"],
                                            ["ps4_arm_teleop", "gui:demo"], "ros"),
    "/g1pilot/arm_command": ("String (JSON)", ["arm_api", "ps4_arm_teleop", "gui:demo"],
                             ["arm_controller"], "ros"),
    "/g1pilot/arm_command/cancel": ("Bool", ["arm_api"], ["arm_controller"], "ros"),
    "/g1pilot/arm_command/status": ("String (JSON)", ["arm_controller"],
                                    ["arm_api", "ps4_arm_teleop", "gui:demo"], "ros"),
    "/g1pilot/pose_store/save": ("String", ["gui", "arm_api"], ["arm_controller"], "ros"),
    "/g1pilot/pose_store/save/status": ("String (JSON)", ["arm_controller"],
                                        ["arm_api", "gui:demo"], "ros"),
    "/g1pilot/pose_store/goto": ("String", ["gui"], ["arm_controller"], "ros"),
    "/g1pilot/pose_store/cancel": ("Bool", ["gui", "ps4_arm_teleop"],
                                   ["arm_controller", "gui:demo"], "ros"),
    "/g1pilot/waist/yaw_rate": ("Float32", ["ps4_arm_teleop"], ["arm_controller"], "ros"),
    "/g1pilot/waist/center": ("Bool", ["ps4_arm_teleop", "gui:demo"], ["arm_controller"], "ros"),
    "/g1pilot/waist/state": ("String (JSON)", ["arm_controller"],
                             ["loco_sim:sim", "loco_client:real", "ps4_arm_teleop",
                              "gui:demo"], "ros"),
    "/g1pilot/workspace/{left,right}": ("Marker", ["arm_controller"], [], "ros"),
    # ── Haende ──
    "/g1pilot/hand_action/{left,right}": ("String", ["gui", "ps4_arm_teleop",
                                                     "loco_client:real"], ["inspire"], "ros"),
    "/g1pilot/hand_state/{left,right}": ("Float32MultiArray", ["inspire"],
                                         ["arm_controller"], "ros"),
    "/g1pilot/finger_goal/{left,right}": ("Float32MultiArray", ["arm_controller"],
                                          ["inspire"], "ros"),
    "/g1pilot/hand_cmd": ("String (JSON)", ["gui:demo"], ["inspire"], "ros"),
    "/g1pilot/hand_status": ("String (JSON)", ["inspire"], ["gui:demo"], "ros"),
    # ── Zustand ──
    "/joint_states": ("JointState", ["robot_state", "inspire"], ["arm_api", "rsp"], "ros"),
    "/tf": ("TFMessage", ["robot_state", "rsp"], ["gui:demo"], "ros"),
    "/g1pilot/imu": ("Imu", ["robot_state"], [], "ros"),
    "/g1pilot/motor_state": ("MotorStateList", ["robot_state"], [], "ros"),
    "/g1pilot/temperatures": ("String (JSON)", ["robot_state"], ["gui:demo"], "ros"),
    # ── Szene (Sim) ──
    "/scene_markers": ("MarkerArray", ["scene_bridge:sim"],
                       ["arm_controller", "create_map:sim", "gui:demo"], "ros"),
    "/g1pilot/scene_reset": ("Bool", ["gui"], ["scene_bridge:sim"], "ros"),
    "/g1pilot/push": ("Bool", ["gui"], ["loco_sim:sim"], "ros"),
    "/g1pilot/push_charge": ("Float32", ["gui"], ["loco_sim:sim"], "ros"),
    "/g1pilot/grasp_box": ("Bool", ["gui"], ["loco_sim:sim"], "ros"),
    "/g1pilot/catch_falls": ("Bool", ["gui"], [], "ros"),
    # ── Navigation ──
    "/map": ("OccupancyGrid", ["create_map:sim"], ["dijkstra"], "ros"),
    "/g1pilot/goal": ("PoseStamped", ["gui:demo"], ["dijkstra", "nav2point"], "ros"),
    "/g1pilot/path": ("Path", ["dijkstra"], ["nav2point", "joy_mux"], "ros"),
    "/lidar_odometry/pose": ("Odometry", ["mola:real"], ["mola_fixed:real"], "ros"),
    "/lidar_odometry/pose_fixed": ("Odometry", ["mola_fixed:real", "sim_localization:sim"],
                                   ["nav2point", "dijkstra", "joy_mux"], "ros"),
    "/g1pilot/waypoint_marker": ("Marker", ["nav2point"], [], "ros"),
    "/g1pilot/goal_marker": ("Marker", ["nav2point"], [], "ros"),
    "/g1pilot/nav_status": ("String", ["nav2point"], ["gui:demo"], "ros"),
}

# Direkte Verbindungen ohne Topic: (von, nach, Beschriftung, Scope)
LINKS = [
    ("inspire", "hands_hw", "Modbus TCP", "real"),
    ("phone", "gui", "Bluetooth LE", "demo"),
    ("http", "arm_api", "HTTP JSON", "both"),
]

# Topics, die der Code nennt, die aber bewusst nicht im Diagramm stehen.
IGNORED_IN_CODE = {
    "/g1pilot/hand_goal",      # Praefix der {left,right}-Topics
}


# ── Pruefung gegen den Code ─────────────────────────────────────────────────

def _expand(name):
    m = re.search(r"\{([^}]*)\}", name)
    if not m:
        return [name]
    return [name[:m.start()] + v + name[m.end():] for v in m.group(1).split(",")]


def _constants(text):
    """NAME = "/..." auf Modulebene (WAIST_STATE_TOPIC, TOPIC, ...)."""
    return dict(re.findall(r'^([A-Z_]+)\s*=\s*["\'](/[^"\']+)["\']', text, re.M))


def check():
    files = [p for p in SRC.rglob("*.py") if "tools" not in p.parts]
    code = {p: p.read_text(encoding="utf-8") for p in files}
    allcode = "\n".join(code.values()).replace("{self.ns}", "/g1pilot")
    consts = {}
    for t in code.values():
        consts.update(_constants(t))
    problems = []

    # 1) Jedes eingetragene ROS-Topic steht im Code.
    for name, (_, _, _, kind) in TOPICS.items():
        for n in _expand(name):
            needle = n.replace("/left", "/").replace("/right", "/") if "{" in name else n
            if kind == "dds":
                needle = n
            if n in ("/tf", "rt/api/sport/request"):
                continue   # TransformBroadcaster bzw. RPC im Unitree-SDK
            if needle not in allcode and n not in allcode:
                problems.append(f"im Diagramm, aber nicht im Code: {n}")

    # 2) Jedes Topic-Literal in create_publisher/create_subscription steht im Diagramm.
    known = set()
    for name in TOPICS:
        known.update(_expand(name))
    call = re.compile(r"create_(?:publisher|subscription)\(\s*\w+\s*,\s*([^,]+),", re.S)
    for p, text in code.items():
        for arg in call.findall(text):
            arg = arg.strip()
            m = re.match(r'f?["\'](/[^"\']+)["\']', arg)
            if m:
                topic = m.group(1)
            elif arg in consts:
                topic = consts[arg]
            elif "." in arg and arg.split(".")[-1] in consts:
                topic = consts[arg.split(".")[-1]]
            else:
                continue   # Parameter (get_parameter) / Variable: Defaults per Hand
            topic = topic.replace("{self.ns}", "/g1pilot")
            for side in ("{s}", "{side}"):
                if side in topic:
                    topic = topic.replace(side, "left")
            topic = re.sub(r"\{side=\w+\}", "left", topic)
            if topic in known or topic in IGNORED_IN_CODE:
                continue
            problems.append(f"im Code, aber nicht im Diagramm: {topic} "
                            f"({p.relative_to(PKG)})")
    return problems


# ── Zeichnen ────────────────────────────────────────────────────────────────

def _esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _split(ref):
    node, _, scope = ref.partition(":")
    return node, (scope or "both")


def _node_scope(nid):
    return NODES[nid][2]


def _edge_attrs(scope):
    if scope in SCOPE_COLOR:
        style = "dashed" if scope == "demo" else "solid"
        return f'color="{SCOPE_COLOR[scope]}", style={style}, penwidth=1.1'
    return 'color="#4a4a4a", penwidth=0.9'


def _tid(name):
    return "t_" + re.sub(r"[^A-Za-z0-9]", "_", name)


# Externe Eingaenge, die in jeder Ansicht mit ihrem Ziel-Node erscheinen.
EXT_INPUTS = {"phone", "http"}

# Ansichten: das Gesamtbild hat zu viele Kanten, um es zu lesen. Jede Ansicht
# zeigt nur ihre Topics und die Nodes, die daran haengen.
VIEWS = {
    "modus": ("Modus und Notaus", [
        "/g1pilot/start", "/g1pilot/start_balancing", "/g1pilot/start_walking",
        "/g1pilot/emergency_stop", "/g1pilot/full_emergency_stop",
        "/g1pilot/arms/enabled", "/g1pilot/arms/home"]),
    "gehen": ("Gehen und Fortbewegung", [
        "/g1pilot/ps4/joy", "/g1pilot/ps4/walk_cmd", "/g1pilot/loco_cmd_vel",
        "/g1pilot/loco_request", "/g1pilot/onboard_state", "/g1pilot/loco_state",
        "/g1pilot/arms/walk_ready", "/g1pilot/waist/state", "rt/lowcmd",
        "rt/api/sport/request", "/g1pilot/push", "/g1pilot/push_charge",
        "/g1pilot/grasp_box", "/g1pilot/catch_falls"]),
    "navigation": ("Navigation", [
        "/g1pilot/joy_manual", "/g1pilot/auto_enable", "/g1pilot/auto_joy",
        "/g1pilot/joy", "/g1pilot/loco_cmd_vel", "/map", "/g1pilot/goal",
        "/g1pilot/path", "/lidar_odometry/pose", "/lidar_odometry/pose_fixed",
        "/g1pilot/waypoint_marker", "/g1pilot/goal_marker", "/g1pilot/nav_status",
        "/scene_markers", "/g1pilot/scene_reset", "rt/sportmodestate"]),
    "arme": ("Arme, Huefte und Haende", [
        "/g1pilot/hand_goal/{left,right}", "/g1pilot/hand_goal_status",
        "/g1pilot/marker_follow_ee", "/g1pilot/hand_cmd_pose/{left,right}",
        "/g1pilot/arm_command", "/g1pilot/arm_command/cancel",
        "/g1pilot/arm_command/status", "/g1pilot/pose_store/save",
        "/g1pilot/pose_store/save/status", "/g1pilot/pose_store/goto",
        "/g1pilot/pose_store/cancel", "/g1pilot/waist/yaw_rate",
        "/g1pilot/waist/center", "/g1pilot/workspace/{left,right}",
        "/g1pilot/hand_action/{left,right}", "/g1pilot/hand_state/{left,right}",
        "/g1pilot/finger_goal/{left,right}", "/g1pilot/hand_cmd",
        "/g1pilot/hand_status", "rt/arm_sdk", "rt/inspire/cmd", "rt/inspire/state"]),
    "zustand": ("Roboterzustand und Bedien-Rueckmeldung", [
        "rt/lowstate", "/joint_states", "/tf", "/g1pilot/imu",
        "/g1pilot/motor_state", "/g1pilot/temperatures", "/g1pilot/arms/state",
        "/g1pilot/ps4/feedback", "/g1pilot/ps4/device", "/g1pilot/ps4/status",
        "/g1pilot/ps4/walk_status", "/g1pilot/ps4/markers",
        "/g1pilot/teleop_state", "/g1pilot/sequence/next",
        "/g1pilot/sequence/state"]),
}


SPLINES = "spline"


def build_dot(title, topics, dense=False):
    """DOT fuer eine Ansicht: nur `topics` und die daran haengenden Nodes."""
    out = [
        "digraph g1pilot {",
        f'  graph [rankdir=LR, splines={SPLINES}, '
        'nodesep=0.22, ranksep=1.1, '
        'fontname="DejaVu Sans", bgcolor="white", pad=0.4, '
        f'label=<<b>g1pilot — {_esc(title)}</b>  '
        '<font point-size="11">(ROS-Nodes und Topics, aus dem Code ermittelt '
        'mit tools/ros_graph.py)</font>>, labelloc=t, labeljust=l, fontsize=20];',
        '  node [fontname="DejaVu Sans", fontsize=10, margin="0.10,0.04"];',
        '  edge [arrowsize=0.55, arrowhead=vee];',
    ]
    used = set()
    for name in topics:
        _, pubs, subs, _ = TOPICS[name]
        used.update(_split(r)[0] for r in pubs + subs)
    links = [l for l in LINKS if l[0] in used and l[1] in used
             or l[0] in EXT_INPUTS and l[1] in used]
    for a, b, _, _ in links:
        used.update((a, b))
    # Nodes
    for nid, (label, cat, scope, extra) in NODES.items():
        if nid not in used:
            continue
        _, fill, border = CATS[cat]
        rows = [f'<b>{_esc(label)}</b>']
        if scope in SCOPE_TAG:
            rows.append(f'<font point-size="8" color="{SCOPE_COLOR[scope]}">'
                        f'<b>{SCOPE_TAG[scope].upper()}</b></font>')
        if extra:
            rows.append(f'<font point-size="8" color="#555555">{_esc(extra)}</font>')
        shape = "box3d" if cat == "ext" else "box"
        style = "filled" if cat == "ext" else "rounded,filled"
        if scope in ("sim", "real"):
            style += ",dashed"
        out.append(f'  {nid} [shape={shape}, style="{style}", fillcolor="{fill}", '
                   f'color="{border}", penwidth=1.6, fontsize=11, '
                   f'label=<{"<br/>".join(rows)}>];')
    # Topics
    for name in topics:
        typ, _, _, kind = TOPICS[name]
        fill = "#f2f2f2" if kind == "dds" else "white"
        tag = '<font point-size="7" color="#777777">Unitree-DDS</font><br/>' \
            if kind == "dds" else ""
        out.append(f'  {_tid(name)} [shape=box, style=filled, fillcolor="{fill}", '
                   f'color="#666666", penwidth=0.8, fontsize=9, '
                   f'label=<{tag}{_esc(name)}<br/><b>{_esc(typ)}</b>>];')
    # Kanten
    for name in topics:
        _, pubs, subs, _ = TOPICS[name]
        t = _tid(name)
        both = {_split(r)[0] for r in pubs} & {_split(r)[0] for r in subs}
        for ref in pubs:
            n, scope = _split(ref)
            if scope == "both":
                scope = _node_scope(n)
            if n in both:
                out.append(f"  {n} -> {t} [dir=both, {_edge_attrs(scope)}];")
            else:
                out.append(f"  {n} -> {t} [{_edge_attrs(scope)}];")
        for ref in subs:
            n, scope = _split(ref)
            if n in both:
                continue
            if scope == "both":
                scope = _node_scope(n)
            out.append(f"  {t} -> {n} [{_edge_attrs(scope)}];")
    for a, b, label, scope in links:
        if scope == "both":
            scope = "x"
        out.append(f'  {a} -> {b} [label=<<font point-size="8">{label}</font>>, '
                   f'{_edge_attrs(scope)}];')
    # Hardware/Sim ganz rechts, Legende ganz links
    hw = [n for n in ("mujoco", "g1", "hands_hw") if n in used]
    if hw:
        out.append("  { rank=sink; " + "; ".join(hw) + "; }")
    out.append("  { rank=source; legend; }")
    out.append(_legend())
    out.append("}")
    return "\n".join(out)


def render(dot, path):
    svg = subprocess.run(["dot", "-Tsvg"], input=dot.encode(), check=True,
                         capture_output=True).stdout
    pathlib.Path(path).write_bytes(svg)
    print("geschrieben:", path)


def _legend():
    rows = ['<tr><td colspan="2" align="left"><b>Legende</b></td></tr>']
    for _, (label, fill, border) in CATS.items():
        rows.append(f'<tr><td bgcolor="{fill}" border="1" color="{border}" '
                    f'width="34" height="16"></td><td align="left">{label} Nodes</td></tr>')
    rows += [
        '<tr><td border="1" bgcolor="white" width="34">/topic<br/><b>Typ</b></td>'
        '<td align="left">ROS-2-Topic: Name und Nachrichtentyp</td></tr>',
        '<tr><td border="1" bgcolor="#f2f2f2" width="34">rt/…</td>'
        '<td align="left">Unitree-DDS-Kanal (unitree_sdk2py, kein ROS)</td></tr>',
        '<tr><td><font color="#4a4a4a"><b>——▶</b></font></td>'
        '<td align="left">Publisher → Topic → Subscriber (Sim und Real)</td></tr>',
        f'<tr><td><font color="{SCOPE_COLOR["sim"]}"><b>——▶</b></font></td>'
        '<td align="left">nur Simulation (Node gestrichelt umrandet: läuft nur in Sim)</td></tr>',
        f'<tr><td><font color="{SCOPE_COLOR["real"]}"><b>——▶</b></font></td>'
        '<td align="left">nur echter Roboter (Node gestrichelt: läuft nur real)</td></tr>',
        f'<tr><td><font color="{SCOPE_COLOR["demo"]}"><b>- - ▶</b></font></td>'
        '<td align="left">nur Demo-GUI (nicht Streamdeck-GUI)</td></tr>',
        '<tr><td><b>◀——▶</b></td>'
        '<td align="left">Node publiziert und abonniert dasselbe Topic</td></tr>',
        '<tr><td colspan="2" align="left"><font point-size="8" color="#555555">'
        'Topics ohne Subscriber: Anzeige in RViz. {left,right} = zwei Topics je Seite.'
        '</font></td></tr>',
    ]
    return ('  legend [shape=plaintext, fontsize=10, label=<<table border="1" '
            'color="#999999" cellborder="0" cellspacing="3" cellpadding="3" '
            f'bgcolor="#fbfbfb">{"".join(rows)}</table>>];')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="nur gegen den Code pruefen")
    ap.add_argument("-o", "--out-dir", default=str(OUT_DIR), help="Zielordner der SVGs")
    args = ap.parse_args()

    problems = check()
    for p in problems:
        print("WARNUNG:", p)
    print(f"{len(NODES)} Nodes, {len(TOPICS)} Topic-Kaesten, "
          f"{len(problems)} Abweichung(en) zum Code")
    in_views = {t for _, ts in VIEWS.values() for t in ts}
    for t in TOPICS:
        if t not in in_views:
            print("WARNUNG: Topic in keiner Ansicht:", t)
            problems.append(t)
    if args.check:
        return 1 if problems else 0

    if not shutil.which("dot"):
        print("Graphviz 'dot' fehlt (apt install graphviz)", file=sys.stderr)
        return 2
    out_dir = pathlib.Path(args.out_dir)
    for key, (title, topics) in VIEWS.items():
        render(build_dot(title, topics), out_dir / f"ros_graph_{key}.svg")
    render(build_dot("Gesamtbild (alle Topics)", list(TOPICS), dense=True),
           out_dir / "ros_graph_gesamt.svg")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())

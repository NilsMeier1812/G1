#!/usr/bin/env python3
"""Node-/Topic-Diagramme des g1pilot-Stacks (docs/images/ros_graph_*.svg).

Die Tabellen unten sind von Hand aus dem Code abgeleitet (create_publisher /
create_subscription / ChannelPublisher / ChannelSubscriber in g1pilot/g1pilot,
Launch-Bedingungen in launch/). Das Skript prueft sie gegen den Code:

  * jedes eingetragene ROS-Topic muss als String im Code vorkommen,
  * jedes Topic-Literal aus einem create_publisher/create_subscription-Aufruf
    im Code muss hier eingetragen sein (sonst Warnung + Exit-Code 1).

Gezeichnet wird ohne Graphviz mit einem eigenen Layout: feste Spalten (Eingaben
links, Roboter/Sim rechts, Topics dazwischen) und rechtwinklige Linien. Pillow
misst die Textbreiten, falls vorhanden, sonst wird geschaetzt.

Aufruf:

    python3 tools/ros_graph.py            # pruefen + SVGs schreiben
    python3 tools/ros_graph.py --check    # nur pruefen

Beschreibung und Lesehilfe: docs/05-T-Nodes-und-Topics.md
"""

import argparse
import pathlib
import re
import sys

PKG = pathlib.Path(__file__).resolve().parents[1]
SRC = PKG / "g1pilot"
OUT_DIR = PKG / "docs" / "images"
DRAWIO = OUT_DIR / "ros_graph.drawio"

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


# ── Zeichnen: eigenes Layout (Spalten, rechtwinklige Linien) ───────────────

def _esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _split(ref):
    node, _, scope = ref.partition(":")
    return node, (scope or "both")


def _node_scope(nid):
    return NODES[nid][2]


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


# Stufe (Spalte) je Node: links Eingaben, rechts Roboter und Simulation. Eine
# Ansicht nutzt nur die Stufen ihrer Nodes, Topics stehen zwischen den Stufen
# (rechts neben dem Publisher oder links vor dem Subscriber, je nachdem, wo
# weniger Linien rueckwaerts laufen).
TIER = {
    "phone": 0, "http": 0, "joystick": 0, "ps4_joystick": 0, "mola": 0,
    "gui": 1, "ps4_arm_teleop": 1, "arm_api": 1, "scene_bridge": 1,
    "robot_state": 1, "sim_localization": 1, "mola_fixed": 1,
    "ps4_walk": 2, "interactive_marker": 2, "create_map": 2,
    "dijkstra": 3, "nav2point": 4, "joy_mux": 5, "joy_to_cmdvel": 6,
    "arm_controller": 6, "loco_sim": 7, "loco_client": 7,
    "inspire": 8, "rsp": 8,
    "mujoco": 9, "g1": 9, "hands_hw": 9,
}

FONT = "DejaVu Sans, Verdana, Arial, sans-serif"
EDGE_COLOR = "#4a4a4a"
PAD_X, PAD_Y = 10, 5          # Innenabstand der Kaesten
PORT = 9                      # Mindestabstand der Anschluesse an einer Kante
V_GAP = 14                    # Abstand der Kaesten in einer Spalte
LANE = 10                     # Abstand paralleler senkrechter Linien
MIN_GAP = 46                  # Mindestbreite zwischen zwei Spalten
TRACK = 9                     # Abstand der Umweg-Linien oben/unten
SEP = 8                       # Mindestabstand waagerechter Linien


def _text_width():
    """Textbreite in px; mit Pillow genau, sonst geschaetzt."""
    try:
        from PIL import ImageFont
        reg = ImageFont.truetype("DejaVuSans.ttf", 100)
        bold = ImageFont.truetype("DejaVuSans-Bold.ttf", 100)
        return lambda t, size, b=False: (bold if b else reg).getlength(t) * size / 100.0
    except Exception:
        return lambda t, size, b=False: len(t) * size * (0.68 if b else 0.62)


TEXT_W = _text_width()


class Box:
    def __init__(self, bid, kind, lines, style):
        self.id, self.kind, self.lines, self.style = bid, kind, lines, style
        self.w = max(TEXT_W(t, s, b) for t, s, b, _ in lines) + 2 * PAD_X
        self.text_h = sum(s * 1.3 for _, s, _, _ in lines) + 2 * PAD_Y
        self.h = self.text_h
        self.col = 0
        self.y = 0.0
        self.x = 0.0
        self.ports = {"L": [], "R": []}   # Trunk-Schluessel je Seite
        self.port_y = {}                  # (Seite, Schluessel) -> y

    @property
    def cy(self):
        return self.y + self.h / 2


def _node_box(nid):
    label, cat, scope, extra = NODES[nid]
    _, fill, border = CATS[cat]
    lines = [(label, 12, True, "#1e1e1e")]
    if scope in SCOPE_TAG:
        lines.append((SCOPE_TAG[scope].upper(), 9, True, SCOPE_COLOR[scope]))
    if extra:
        lines.append((extra, 9, False, "#555555"))
    style = dict(fill=fill, stroke=border, dashed=scope in ("sim", "real"),
                 ext=cat == "ext")
    return Box(nid, "node", lines, style)


def _topic_box(name):
    typ, _, _, kind = TOPICS[name]
    lines = []
    if kind == "dds":
        lines.append(("Unitree-DDS", 8, False, "#777777"))
    lines += [(name, 10, False, "#1e1e1e"), (typ, 10, True, "#1e1e1e")]
    style = dict(fill="#f2f2f2" if kind == "dds" else "#ffffff", stroke="#666666",
                 dashed=False, ext=False)
    return Box(_tid(name), "topic", lines, style)


def _tid(name):
    return "t_" + re.sub(r"[^A-Za-z0-9]", "_", name)


class Edge:
    def __init__(self, a, b, scope, both=False, label=None):
        self.a, self.b, self.scope, self.both, self.label = a, b, scope, both, label
        self.kind = None      # "adj" | "long" | "back"
        self.trunks = []      # [(Schluessel, Luecke)]
        self.track = None     # y der waagerechten Linie (long/back)
        self.back_key = None


def _collect(topics):
    boxes, edges = {}, []
    used = set()
    for name in topics:
        _, pubs, subs, _ = TOPICS[name]
        used.update(_split(r)[0] for r in pubs + subs)
    for a, b, _, _ in LINKS:
        if (a in used and b in used) or (a in EXT_INPUTS and b in used):
            used.update((a, b))
    for nid in NODES:
        if nid in used:
            boxes[nid] = _node_box(nid)
    for name in topics:
        _, pubs, subs, _ = TOPICS[name]
        t = _tid(name)
        boxes[t] = _topic_box(name)
        both = {_split(r)[0] for r in pubs} & {_split(r)[0] for r in subs}
        for ref in pubs:
            n, scope = _split(ref)
            edges.append(Edge(n, t, scope if scope != "both" else NODES[n][2], n in both))
        for ref in subs:
            n, scope = _split(ref)
            if n not in both:
                edges.append(Edge(t, n, scope if scope != "both" else NODES[n][2]))
    for a, b, label, scope in LINKS:
        if a in boxes and b in boxes:
            edges.append(Edge(a, b, scope if scope != "both" else "x", label=label))
    return boxes, edges


def _edge_cost(ca, cb):
    """Kosten einer Kante zwischen zwei Spalten: rueckwaerts teuer, lang etwas."""
    if cb > ca:
        return cb - ca - 1
    return 12 + (ca - cb)


def _topic_col(name, col):
    """Spalte eines Topics bei gegebenen Node-Spalten (gerade Zahlen)."""
    _, pubs, subs, _ = TOPICS[name]
    both = {_split(r)[0] for r in pubs} & {_split(r)[0] for r in subs}
    pc = [col[_split(r)[0]] for r in pubs if _split(r)[0] not in both]
    sc = [col[_split(r)[0]] for r in subs if _split(r)[0] not in both]
    bc = [col[n] for n in both]
    cands = {c + d for c in pc + bc for d in (1, -1)} | {c - 1 for c in sc}
    best = None
    for c in sorted(cands, reverse=True):
        cost = (sum(_edge_cost(p, c) for p in pc) + sum(_edge_cost(c, x) for x in sc)
                + sum(_edge_cost(min(c, x), max(c, x)) for x in bc))
        if best is None or cost < best[0]:
            best = (cost, c)
    return best


def _assign_columns(boxes, edges, topics):
    """Stufen der Nodes so waehlen, dass moeglichst wenig Linien rueckwaerts laufen.

    Start ist TIER; Eingaben (Stufe 0) bleiben links, Roboter/Sim (Stufe 9)
    rechts, die uebrigen Nodes werden so lange einzeln verschoben, wie das die
    Summe der Kantenkosten senkt.
    """
    nodes = [b for b in boxes if b in NODES]
    tier = {n: TIER[n] for n in nodes}
    links = [(e.a, e.b) for e in edges if e.a in NODES and e.b in NODES]

    def total(t):
        col = {n: 2 * v for n, v in t.items()}
        cost = sum(_topic_col(name, col)[0] for name in topics)
        return cost + sum(_edge_cost(col[a], col[b]) for a, b in links)

    best = total(tier)
    movable = [n for n in nodes if TIER[n] not in (0, 9)]
    for _ in range(6):
        improved = False
        for n in movable:
            for v in range(1, 9):
                if v == tier[n]:
                    continue
                old = tier[n]
                tier[n] = v
                c = total(tier)
                if c < best - 1e-9:
                    best, improved = c, True
                else:
                    tier[n] = old
        if not improved:
            break
    col = {n: 2 * v for n, v in tier.items()}
    for name in topics:
        col[_tid(name)] = _topic_col(name, col)[1]
    used = sorted(set(col.values()))
    remap = {c: i for i, c in enumerate(used)}
    for b, c in col.items():
        boxes[b].col = remap[c]
    return len(used)


def _plan_edges(boxes, edges):
    """Trunks (senkrechte Sammellinien) je Kante festlegen, Anschluesse zaehlen."""
    fan_s, fan_d = {}, {}
    for e in edges:
        # Doppelpfeile immer von links nach rechts fuehren
        if e.both and boxes[e.a].col > boxes[e.b].col:
            e.a, e.b = e.b, e.a
        ca, cb = boxes[e.a].col, boxes[e.b].col
        e.kind = "adj" if cb == ca + 1 else ("long" if cb > ca else "back")
        if e.kind == "adj" and not e.both:
            fan_s[e.a] = fan_s.get(e.a, 0) + 1
            fan_d[e.b] = fan_d.get(e.b, 0) + 1
    for i, e in enumerate(edges):
        ca, cb = boxes[e.a].col, boxes[e.b].col
        ks, kd = ("s", e.a), ("d", e.b)
        if e.both or e.label:
            ks, kd = ("es", i), ("ed", i)
        if e.kind == "adj":
            k = ks if (e.both or e.label or fan_s.get(e.a, 0) >= fan_d.get(e.b, 0)) else kd
            e.trunks = [(k, ca)]
            src_port, dst_port = k, k
        else:
            e.trunks = [(ks, ca), (kd, cb - 1)]
            src_port, dst_port = ks, kd
        if src_port not in boxes[e.a].ports["R"]:
            boxes[e.a].ports["R"].append(src_port)
        if dst_port not in boxes[e.b].ports["L"]:
            boxes[e.b].ports["L"].append(dst_port)
        e.src_port, e.dst_port = src_port, dst_port
    for bx in boxes.values():
        n = max(len(bx.ports["L"]), len(bx.ports["R"]))
        bx.h = max(bx.text_h, (n + 1) * PORT)


def _place_rows(boxes, edges, ncols):
    cols = [[] for _ in range(ncols)]
    for bx in boxes.values():
        cols[bx.col].append(bx)
    nb = {b: [] for b in boxes}
    for e in edges:
        nb[e.a].append(e.b)
        nb[e.b].append(e.a)

    def stack(col_boxes, desired):
        order = sorted(col_boxes, key=lambda b: desired[b.id])
        y = -1e9
        for b in order:
            b.y = max(desired[b.id] - b.h / 2, y)
            y = b.y + b.h + V_GAP
        # Spalte so verschieben, dass sie im Mittel an ihren Wunschplaetzen sitzt
        shift = sum(desired[b.id] - b.cy for b in order) / len(order)
        for b in order:
            b.y += shift

    for c in cols:
        y = 0.0
        for b in c:
            b.y = y
            y += b.h + V_GAP
    order = list(range(ncols))
    for sweep in range(24):
        for ci in (order if sweep % 2 == 0 else order[::-1]):
            desired = {}
            for b in cols[ci]:
                ys = [boxes[n].cy for n in nb[b.id] if boxes[n].col != ci]
                desired[b.id] = sum(ys) / len(ys) if ys else b.cy
            stack(cols[ci], desired)
    top = min(b.y for b in boxes.values())
    for b in boxes.values():
        b.y -= top
    return cols


def _free_y(y0, ca, cb, cols, hsegs, ylimits):
    """Freie Hoehe fuer eine waagerechte Linie ueber die Spalten ca+1..cb-1."""
    blocked = [(b.y - 5, b.y + b.h + 5) for c in range(ca + 1, cb) for b in cols[c]]
    lo, hi = ylimits
    for k in range(0, 400):
        for y in ((y0,) if k == 0 else (y0 + k * SEP, y0 - k * SEP)):
            if not lo <= y <= hi:
                continue
            if any(a <= y <= b for a, b in blocked):
                continue
            if any(abs(y - hy) < SEP and not (c1 < ca or c0 > cb) for hy, c0, c1 in hsegs):
                continue
            return y
    return y0


def _route(boxes, edges, cols):
    ymax = max(b.y + b.h for b in boxes.values())
    mid = ymax / 2
    # Anschluss-Reihenfolge: nach der Hoehe der Gegenstelle
    trunk_ends = {}
    for e in edges:
        for k, _ in e.trunks:
            trunk_ends.setdefault(k, []).extend([boxes[e.a].cy, boxes[e.b].cy])
    for bx in boxes.values():
        for side in ("L", "R"):
            ks = sorted(bx.ports[side], key=lambda k: sum(trunk_ends[k]) / len(trunk_ends[k]))
            n = len(ks)
            for i, k in enumerate(ks):
                bx.port_y[(side, k)] = bx.y + (i + 1) * bx.h / (n + 1)
    # Waagerechte Linien fuer lange Kanten (frei durch die Zwischenspalten)
    hsegs = []
    for e in edges:
        if e.kind != "long":
            continue
        a, b = boxes[e.a], boxes[e.b]
        ya, yb = a.port_y[("R", e.src_port)], b.port_y[("L", e.dst_port)]
        e.track = _free_y(yb, a.col, b.col, cols, hsegs, (-60, ymax + 60))
        if abs(e.track - yb) > 1 and abs(ya - yb) > 1:
            alt = _free_y(ya, a.col, b.col, cols, hsegs, (-60, ymax + 60))
            if abs(alt - ya) < abs(e.track - yb):
                e.track = alt
        hsegs.append((e.track, a.col, b.col))
    # Rueckwaerts-Kanten: Umweg oben oder unten, eine Spur je Quelle und Seite
    groups = {}
    for e in edges:
        if e.kind != "back":
            continue
        a, b = boxes[e.a], boxes[e.b]
        side = "top" if (a.cy + b.cy) / 2 < mid else "bottom"
        key = ("back", e.a, side, e.both or bool(e.label), id(e) if e.both else 0)
        g = groups.setdefault(key, {"side": side, "lo": b.col - 1, "hi": a.col, "edges": []})
        g["lo"] = min(g["lo"], b.col - 1)
        g["hi"] = max(g["hi"], a.col)
        g["edges"].append(e)
    lanes = {"top": [], "bottom": []}
    for key, g in sorted(groups.items(), key=lambda kv: kv[1]["hi"] - kv[1]["lo"]):
        used = lanes[g["side"]]
        lane = 0
        while any(l == lane and not (hi < g["lo"] or lo > g["hi"]) for l, lo, hi in used):
            lane += 1
        used.append((lane, g["lo"], g["hi"]))
        y = (-24 - lane * TRACK) if g["side"] == "top" else (ymax + 24 + lane * TRACK)
        for e in g["edges"]:
            e.track = y
    # Trunk-Spannen (fuer die Spuren in den Luecken)
    spans = {}
    for e in edges:
        a, b = boxes[e.a], boxes[e.b]
        ya, yb = a.port_y[("R", e.src_port)], b.port_y[("L", e.dst_port)]
        if e.kind == "adj":
            (k, g), = e.trunks
            spans.setdefault((k, g), []).extend([ya, yb])
        else:
            (ks, gs), (kd, gd) = e.trunks
            spans.setdefault((ks, gs), []).extend([ya, e.track])
            spans.setdefault((kd, gd), []).extend([e.track, yb])
    # Spuren je Luecke (Intervall-Faerbung)
    gap_lanes, lane_of = {}, {}
    for (k, g), ys in sorted(spans.items(), key=lambda kv: (kv[0][1], min(kv[1]))):
        lo, hi = min(ys), max(ys)
        used = gap_lanes.setdefault(g, [])
        lane = 0
        while any(l == lane and not (h + 4 < lo or l0 - 4 > hi) for l, l0, h in used):
            lane += 1
        used.append((lane, lo, hi))
        lane_of[(k, g)] = lane
    return gap_lanes, lane_of


def _layout_x(cols, gap_lanes, x0, min_gap):
    widths = [max(b.w for b in c) for c in cols]
    nl = lambda g: 1 + max([l for l, _, _ in gap_lanes.get(g, [])], default=-1)  # noqa: E731
    gap_w = {g: max(MIN_GAP, min_gap.get(g, 0), (nl(g) + 1) * LANE)
             for g in range(-1, len(cols))}
    gap_x = {}
    x = x0
    gap_x[-1] = x
    x += gap_w[-1] if gap_lanes.get(-1) else 0
    for i, c in enumerate(cols):
        for b in c:
            b.x = x + (widths[i] - b.w) / 2
        x += widths[i]
        gap_x[i] = x
        x += gap_w[i]
    return gap_x, x


def _svg_text(x, y, t, size, bold, color, anchor="middle"):
    w = ' font-weight="bold"' if bold else ""
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}"{w} fill="{color}" '
            f'text-anchor="{anchor}">{_esc(t)}</text>')


def _svg_box(b):
    s = b.style
    out = []
    dash = ' stroke-dasharray="5 3"' if s["dashed"] else ""
    if s["ext"]:
        out.append(f'<rect x="{b.x + 4:.1f}" y="{b.y - 4:.1f}" width="{b.w:.1f}" '
                   f'height="{b.h:.1f}" fill="#c8c8c8" stroke="{s["stroke"]}"/>')
        out.append(f'<rect x="{b.x:.1f}" y="{b.y:.1f}" width="{b.w:.1f}" height="{b.h:.1f}" '
                   f'fill="{s["fill"]}" stroke="{s["stroke"]}" stroke-width="1.4"{dash}/>')
    elif b.kind == "node":
        out.append(f'<rect x="{b.x:.1f}" y="{b.y:.1f}" width="{b.w:.1f}" height="{b.h:.1f}" '
                   f'rx="7" fill="{s["fill"]}" stroke="{s["stroke"]}" stroke-width="1.6"{dash}/>')
    else:
        out.append(f'<rect x="{b.x:.1f}" y="{b.y:.1f}" width="{b.w:.1f}" height="{b.h:.1f}" '
                   f'fill="{s["fill"]}" stroke="{s["stroke"]}" stroke-width="0.9"/>')
    th = sum(sz * 1.3 for _, sz, _, _ in b.lines)
    y = b.cy - th / 2
    for t, sz, bold, color in b.lines:
        y += sz * 1.3
        out.append(_svg_text(b.x + b.w / 2, y - sz * 0.3, t, sz, bold, color))
    return out


def _edge_style(scope):
    color = SCOPE_COLOR.get(scope, EDGE_COLOR)
    dash = ' stroke-dasharray="6 3"' if scope == "demo" else ""
    width = 1.3 if scope in SCOPE_COLOR else 1.1
    return color, f'stroke="{color}" stroke-width="{width}"{dash}'


def _arrow(x, y, direction, color):
    d = 7 if direction == "right" else -7
    return (f'<path d="M{x - d:.1f},{y - 3.5:.1f} L{x:.1f},{y:.1f} L{x - d:.1f},{y + 3.5:.1f} z" '
            f'fill="{color}"/>')


def _edge_points(e, boxes, gap_x, lane_of):
    """Eckpunkte einer Kante: Anschluss rechts an a, Sammellinie(n), links an b."""
    a, b = boxes[e.a], boxes[e.b]
    ya, yb = a.port_y[("R", e.src_port)], b.port_y[("L", e.dst_port)]
    xs = [gap_x[g] + (lane_of[(k, g)] + 1) * LANE for k, g in e.trunks]
    xa, xb = a.x + a.w, b.x
    if e.kind == "adj":
        return [(xa, ya), (xs[0], ya), (xs[0], yb), (xb, yb)]
    return [(xa, ya), (xs[0], ya), (xs[0], e.track), (xs[1], e.track),
            (xs[1], yb), (xb, yb)]


def _svg_edges(boxes, edges, gap_x, lane_of):
    out = []
    order = {"demo": 0, "sim": 1, "real": 2}
    for e in sorted(edges, key=lambda e: order.get(e.scope, 3)):
        pts = _edge_points(e, boxes, gap_x, lane_of)
        (xa, ya), (xb, yb) = pts[0], pts[-1]
        color, attrs = _edge_style(e.scope)
        d = " ".join(f"{'M' if i == 0 else 'L'}{x:.1f},{y:.1f}" for i, (x, y) in enumerate(pts))
        out.append(f'<path d="{d}" fill="none" {attrs} stroke-linejoin="round"/>')
        out.append(_arrow(xb, yb, "right", color))
        if e.both:
            out.append(_arrow(xa, ya, "left", color))
        if e.label:
            segs = [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)
                    if abs(pts[i][1] - pts[i + 1][1]) < 0.5]
            (x0, y0), (x1, _) = segs[-1] if e.kind == "adj" else max(
                segs, key=lambda s: abs(s[1][0] - s[0][0]))
            out.append(_svg_text((x0 + x1) / 2, y0 - 4, e.label, 9, False, color))
    return out


def _svg_legend(x, y):
    out, rows = [], []
    for _, (label, fill, border) in CATS.items():
        rows.append(("box", fill, border, f"{label} Nodes"))
    rows += [
        ("topic", "#ffffff", "#666666", "ROS-2-Topic: Name und Nachrichtentyp"),
        ("topic", "#f2f2f2", "#666666", "Unitree-DDS-Kanal (unitree_sdk2py, kein ROS)"),
        ("line", "x", "", "Publisher → Topic → Subscriber (Sim und Real)"),
        ("line", "sim", "", "nur Simulation (Node gestrichelt: läuft nur in Sim)"),
        ("line", "real", "", "nur echter Roboter (Node gestrichelt: läuft nur real)"),
        ("line", "demo", "", "nur Demo-GUI (nicht Streamdeck-GUI)"),
        ("both", "x", "", "Node publiziert und abonniert dasselbe Topic"),
    ]
    w, rh = 360, 22
    h = 34 + rh * len(rows) + 26
    out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="#fbfbfb" '
               f'stroke="#999999"/>')
    out.append(_svg_text(x + 12, y + 22, "Legende", 12, True, "#1e1e1e", "start"))
    yy = y + 34
    for kind, a, bcol, text in rows:
        cy = yy + rh / 2
        if kind == "box":
            out.append(f'<rect x="{x + 12}" y="{cy - 8}" width="44" height="16" rx="4" '
                       f'fill="{a}" stroke="{bcol}" stroke-width="1.4"/>')
        elif kind == "topic":
            out.append(f'<rect x="{x + 12}" y="{cy - 8}" width="44" height="16" '
                       f'fill="{a}" stroke="{bcol}"/>')
        else:
            color, attrs = _edge_style(a)
            out.append(f'<path d="M{x + 12},{cy} L{x + 56},{cy}" {attrs}/>')
            out.append(_arrow(x + 56, cy, "right", color))
            if kind == "both":
                out.append(_arrow(x + 12, cy, "left", color))
        out.append(_svg_text(x + 66, cy + 4, text, 10, False, "#1e1e1e", "start"))
        yy += rh
    out.append(_svg_text(x + 12, yy + 16, "Topics ohne Subscriber: Anzeige in RViz. "
                         "{left,right} = zwei Topics.", 9, False, "#555555", "start"))
    return out, w, h


class Layout:
    """Fertig angeordnete Ansicht (fuer SVG und draw.io)."""

    def __init__(self, title, topics, legend_w, legend_h):
        self.title = title
        self.boxes, self.edges = _collect(topics)
        ncols = _assign_columns(self.boxes, self.edges, topics)
        _plan_edges(self.boxes, self.edges)
        cols = _place_rows(self.boxes, self.edges, ncols)
        gap_lanes, self.lane_of = _route(self.boxes, self.edges, cols)
        min_gap = {}
        for e in self.edges:
            if e.label:
                g = e.trunks[0][1]
                min_gap[g] = max(min_gap.get(g, 0), TEXT_W(e.label, 9) + 4 * LANE)
        self.gap_x, self.width = _layout_x(cols, gap_lanes, legend_w + 30, min_gap)
        tracks = [e.track for e in self.edges if e.track is not None]
        ys = [b.y for b in self.boxes.values()] + tracks
        ye = [b.y + b.h for b in self.boxes.values()] + tracks
        self.top, self.bottom = min(ys) - 20, max(max(ye), legend_h) + 20

    def points(self, e):
        return _edge_points(e, self.boxes, self.gap_x, self.lane_of)


HEAD = 56
SUBTITLE = "ROS-Nodes und Topics, aus dem Code ermittelt mit tools/ros_graph.py"


def build_svg(title, topics):
    """SVG fuer eine Ansicht: nur `topics` und die daran haengenden Nodes."""
    legend, lw, lh = _svg_legend(0, 0)
    lay = Layout(title, topics, lw, lh)
    body = _svg_edges(lay.boxes, lay.edges, lay.gap_x, lay.lane_of)
    for b in lay.boxes.values():
        body += _svg_box(b)
    w, h = lay.width + 20, lay.bottom - lay.top + HEAD
    oy = HEAD - lay.top
    return "\n".join([
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{h:.0f}" '
        f'viewBox="0 0 {w:.0f} {h:.0f}" font-family="{FONT}">',
        f'<rect width="{w:.0f}" height="{h:.0f}" fill="#ffffff"/>',
        _svg_text(20, 34, f"g1pilot — {title}", 20, True, "#1e1e1e", "start"),
        _svg_text(20 + TEXT_W(f"g1pilot — {title}", 20, True) + 14, 34,
                  SUBTITLE, 11, False, "#555555", "start"),
        f'<g transform="translate(20,{oy + max(0, lay.top):.1f})">', *legend, "</g>",
        f'<g transform="translate(20,{oy:.1f})">', *body, "</g>",
        "</svg>", ""])


# ── draw.io-Export: dieselbe Anordnung, Kaesten frei verschiebbar ───────────

def _xa(s):
    """Text fuer ein XML-Attribut (draw.io-Labels sind HTML)."""
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
            .replace('"', "&quot;").replace("\n", "&#10;"))


def _dio_label(b):
    parts = []
    for t, size, bold, color in b.lines:
        t = _esc(t)
        if bold:
            t = f"<b>{t}</b>"
        parts.append(f'<font style="font-size:{size}px" color="{color}">{t}</font>')
    return "<br>".join(parts)


def _dio_box_style(b):
    s = b.style
    style = ["whiteSpace=wrap", "html=1", f"fillColor={s['fill']}",
             f"strokeColor={s['stroke']}", "fontFamily=DejaVu Sans", "verticalAlign=middle"]
    if s["ext"]:
        style += ["rounded=0", "shadow=1", "strokeWidth=1.4"]
    elif b.kind == "node":
        style += ["rounded=1", "arcSize=18", "strokeWidth=1.6"]
    else:
        style += ["rounded=0", "strokeWidth=0.9"]
    if s["dashed"]:
        style += ["dashed=1", "dashPattern=5 3"]
    return ";".join(style) + ";"


def _dio_edge_style(e, pts, a, b):
    color = SCOPE_COLOR.get(e.scope, EDGE_COLOR)
    style = ["edgeStyle=orthogonalEdgeStyle", "rounded=0", "html=1", "jumpStyle=none",
             f"strokeColor={color}", "strokeWidth=1.2", "endArrow=block", "endFill=1",
             "endSize=5", "exitX=1", f"exitY={(pts[0][1] - a.y) / a.h:.3f}", "exitDx=0",
             "exitDy=0", "entryX=0", f"entryY={(pts[-1][1] - b.y) / b.h:.3f}",
             "entryDx=0", "entryDy=0", "fontSize=9", f"fontColor={color}"]
    if e.both:
        style += ["startArrow=block", "startFill=1", "startSize=5"]
    else:
        style += ["startArrow=none"]
    if e.scope == "demo":
        style += ["dashed=1", "dashPattern=6 3"]
    return ";".join(style) + ";"


def _dio_legend(x, y):
    """Legende als eigene Zellen (ein Rahmen + Muster + Texte)."""
    cells = []
    rows = [("box", fill, border, f"{label} Nodes") for _, (label, fill, border) in CATS.items()]
    rows += [
        ("topic", "#ffffff", "#666666", "ROS-2-Topic: Name und Nachrichtentyp"),
        ("topic", "#f2f2f2", "#666666", "Unitree-DDS-Kanal (unitree_sdk2py, kein ROS)"),
        ("line", "x", "", "Publisher → Topic → Subscriber (Sim und Real)"),
        ("line", "sim", "", "nur Simulation (Node gestrichelt: läuft nur in Sim)"),
        ("line", "real", "", "nur echter Roboter (Node gestrichelt: läuft nur real)"),
        ("line", "demo", "", "nur Demo-GUI (nicht Streamdeck-GUI)"),
        ("both", "x", "", "Node publiziert und abonniert dasselbe Topic"),
    ]
    rh, w = 22, 360
    h = 34 + rh * len(rows) + 26
    cells.append(("legend", "<b>Legende</b>",
                  "rounded=0;whiteSpace=wrap;html=1;fillColor=#fbfbfb;strokeColor=#999999;"
                  "verticalAlign=top;align=left;spacingLeft=10;spacingTop=4;fontSize=12;"
                  "container=1;collapsible=0;", 0, 0, w, h, None))
    yy = 34
    for i, (kind, a, bcol, text) in enumerate(rows):
        cy = yy + rh / 2
        if kind in ("box", "topic"):
            rnd = "rounded=1;arcSize=25;" if kind == "box" else "rounded=0;"
            cells.append((f"legend_s{i}", "", f"{rnd}html=1;fillColor={a};strokeColor={bcol};",
                          12, cy - 8, 44, 16, "legend"))
        else:
            color = SCOPE_COLOR.get(a, EDGE_COLOR)
            dash = "dashed=1;dashPattern=6 3;" if a == "demo" else ""
            start = "startArrow=block;startFill=1;startSize=5;" if kind == "both" else ""
            cells.append((f"legend_s{i}", None,
                          f"endArrow=block;endFill=1;endSize=5;{start}html=1;"
                          f"strokeColor={color};strokeWidth=1.2;{dash}",
                          12, cy, 56, cy, "legend"))
        cells.append((f"legend_t{i}", _esc(text),
                      "text;html=1;align=left;verticalAlign=middle;fontSize=10;",
                      66, cy - 10, 290, 20, "legend"))
        yy += rh
    cells.append(("legend_note", "Topics ohne Subscriber: Anzeige in RViz. "
                  "{left,right} = zwei Topics.",
                  "text;html=1;align=left;verticalAlign=middle;fontSize=9;fontColor=#555555;",
                  12, yy + 6, 340, 18, "legend"))
    return cells, w, h


def build_drawio_page(key, title, topics):
    legend, lw, lh = _dio_legend(0, 0)
    lay = Layout(title, topics, lw, lh)
    oy = HEAD - lay.top
    out = [f'<diagram id="{key}" name="{_xa(title)}"><mxGraphModel dx="1600" dy="900" '
           'grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" '
           'fold="1" page="0" pageScale="1" math="0" shadow="0"><root>',
           '<mxCell id="0"/><mxCell id="1" parent="0"/>',
           f'<mxCell id="title" value="{_xa(f"<b>g1pilot — {_esc(title)}</b>")}" '
           'style="text;html=1;align=left;verticalAlign=middle;fontSize=20;" vertex="1" '
           'parent="1"><mxGeometry x="20" y="14" width="900" height="30" as="geometry"/>'
           '</mxCell>',
           f'<mxCell id="subtitle" value="{_xa(SUBTITLE)}" style="text;html=1;align=left;'
           'verticalAlign=middle;fontSize=11;fontColor=#555555;" vertex="1" parent="1">'
           f'<mxGeometry x="20" y="40" width="600" height="18" as="geometry"/></mxCell>']
    ly = oy + max(0, lay.top)
    for cid, value, style, x, y, w, h, parent in legend:
        par = parent or "1"
        if parent is None:
            x, y = x + 20, y + ly
        if value is None:   # Linie in der Legende
            out.append(f'<mxCell id="{cid}" style="{_xa(style)}" edge="1" parent="{par}">'
                       '<mxGeometry relative="1" as="geometry">'
                       f'<mxPoint x="{x}" y="{y:.1f}" as="sourcePoint"/>'
                       f'<mxPoint x="{w}" y="{h:.1f}" as="targetPoint"/></mxGeometry></mxCell>')
        else:
            out.append(f'<mxCell id="{cid}" value="{_xa(value)}" style="{_xa(style)}" '
                       f'vertex="1" parent="{par}"><mxGeometry x="{x:.1f}" y="{y:.1f}" '
                       f'width="{w:.1f}" height="{h:.1f}" as="geometry"/></mxCell>')
    for b in lay.boxes.values():
        out.append(f'<mxCell id="{b.id}" value="{_xa(_dio_label(b))}" '
                   f'style="{_xa(_dio_box_style(b))}" vertex="1" parent="1">'
                   f'<mxGeometry x="{b.x + 20:.1f}" y="{b.y + oy:.1f}" width="{b.w:.1f}" '
                   f'height="{b.h:.1f}" as="geometry"/></mxCell>')
    order = {"demo": 0, "sim": 1, "real": 2}
    for i, e in enumerate(sorted(lay.edges, key=lambda e: order.get(e.scope, 3))):
        a, b = lay.boxes[e.a], lay.boxes[e.b]
        pts = lay.points(e)
        inner = "".join(f'<mxPoint x="{x + 20:.1f}" y="{y + oy:.1f}"/>' for x, y in pts[1:-1])
        value = _xa(_esc(e.label)) if e.label else ""
        out.append(f'<mxCell id="e{i}" value="{value}" '
                   f'style="{_xa(_dio_edge_style(e, pts, a, b))}" edge="1" parent="1" '
                   f'source="{e.a}" target="{e.b}"><mxGeometry relative="1" as="geometry">'
                   f'<Array as="points">{inner}</Array></mxGeometry></mxCell>')
    out.append("</root></mxGraphModel></diagram>")
    return "".join(out)


def build_drawio(views):
    pages = [build_drawio_page(k, t, ts) for k, (t, ts) in views.items()]
    return ('<mxfile host="g1pilot-ros_graph" type="device">\n'
            + "\n".join(pages) + "\n</mxfile>\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="nur gegen den Code pruefen")
    ap.add_argument("-o", "--out-dir", default=str(OUT_DIR), help="Zielordner der SVGs")
    ap.add_argument("--drawio", metavar="DATEI", nargs="?", const=str(DRAWIO),
                    help="zusaetzlich alle Ansichten als draw.io-Datei schreiben "
                         f"(Default {DRAWIO.relative_to(PKG)}; ueberschreibt Handarbeit!)")
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

    out_dir = pathlib.Path(args.out_dir)
    views = dict(VIEWS)
    views["gesamt"] = ("Gesamtbild (alle Topics)", list(TOPICS))
    for key, (title, topics) in views.items():
        path = out_dir / f"ros_graph_{key}.svg"
        path.write_text(build_svg(title, topics), encoding="utf-8")
        print("geschrieben:", path)
    if args.drawio:
        pathlib.Path(args.drawio).write_text(build_drawio(views), encoding="utf-8")
        print("geschrieben:", args.drawio)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())

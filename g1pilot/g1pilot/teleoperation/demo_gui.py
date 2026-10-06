#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo_gui — vereinfachte Bedienoberflaeche fuer Vorfuehrungen (KONZEPT).

Gedacht fuer Besucher/Demo-Bediener, die den Stack nicht kennen. Statt 25
Streamdeck-Kacheln gibt es drei klar getrennte Bereiche:

  1) MODUS (oben)      : GEHEN  <->  GREIFEN. Genau einer ist aktiv und farbig
                         hervorgehoben; der andere ist ausgegraut. Waehrend des
                         Wechsels (Arme raeumen auf) blinkt "wechselt ...".
  2) STEUERUNG (Mitte) : zeigt NUR die Bedienelemente des aktiven Modus.
                         GEHEN   -> Joystick-Knopf + Pfeiltasten + Drehen.
                         GREIFEN -> Umschalter ARME | HAENDE:
                                    ARME   = Beispielbewegungen (Pose-Store-
                                             Kategorie "Demo"), Grundstellung und
                                             eingeklappter "Erweitert"-Bereich.
                                    HAENDE = ganze Hand auf/zu, Griffkraft,
                                             Kraftzonen + einzelne Finger
                                             (hand_panel.py, ersetzt die
                                             Browser-GUIs der Hand-Bridge).
  3) STATUS (unten)    : was der Roboter gerade tut + immer sichtbarer NOT-HALT.

Die GUI publiziert auf DIESELBEN Topics wie ui_interface.py (Streamdeck) --
die Logik bleibt in loco_sim/loco_client/arm_controller. Beide GUIs NICHT
gleichzeitig starten (doppelter Auto-Start). Auswahl ueber G1_GUI=demo,
siehe launch/teleoperation_launcher.launch.py und docs/42_demo_gui_konzept.md.
"""

import json
import math
import os
import re
import sys

from PyQt6.QtWidgets import (
    QApplication, QWidget, QGridLayout, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QStackedWidget, QFrame, QDialog, QMessageBox, QSizePolicy,
    QGraphicsOpacityEffect,
)
from PyQt6.QtCore import QTimer, Qt

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from g1pilot.navigation import scene_markers as sm

from g1pilot.utils.window_style import DarkStyle
from g1pilot.utils.common import is_sim_mode
from g1pilot.teleoperation.ui_interface import (
    StreamDeck, VirtualJoystick, PoseSaveDialog, PoseLoadDialog,
)
from g1pilot.teleoperation.hand_panel import HandPanel

WALK = "walk"
MANIP = "manip"

# Farbcode je Modus -- zieht sich durch Kachel, Steuerbereich-Rahmen und Status.
MODE_COLOR = {WALK: "#1e88e5", MANIP: "#fb8c00"}
MODE_TITLE = {WALK: "GEHEN", MANIP: "GREIFEN"}

# Geschwindigkeitsstufen (normiert, loco_sim skaliert mit max_cmd ~0.8 m/s).
# Fuer Vorfuehrungen bewusst gedeckelt -- "Normal" ist nicht Vollgas.
SPEEDS = {"Langsam": 0.3, "Normal": 0.6}

# Pose-Store-Kategorie, deren Posen als Beispielbewegungen erscheinen.
DEMO_CATEGORY = os.environ.get("G1_DEMO_CATEGORY", "Demo")
# Ablaeufe: jede Kategorie "Ablauf <Name>" = eine Sequenz (Posen in Namens-
# reihenfolge, z.B. AP1_01_..., AP1_02_...) -> ein Knopf "<Name>".
SEQUENCE_PREFIX = "Ablauf "
# Knopf "Grundstellung" faehrt diese gespeicherte Pose an.
HOME_POSE = os.environ.get("G1_HOME_POSE", "Sichere_Pose")
MAX_DEMO_BUTTONS = 8
# WALK gilt als "angekommen", wenn arms/walk_ready kommt -- spaetestens nach
# diesem Timeout (loco_sim laeuft dann ohnehin selbst los).
WALK_SWITCH_TIMEOUT_MS = 16000   # > loco_sim walk_arm_timeout_s (15 s)

TERMINAL = ("reached", "failed", "rejected", "cancelled")


def _envflag(name):
    return os.environ.get(name, '0').strip().lower() in ('1', 'true', 'yes', 'on')


# AUTO NAV nur, wenn der Nav-Stack laeuft (Sim: G1_ENABLE_NAV, Real:
# G1_ENABLE_LIDAR) -- sonst publiziert der Knopf an niemanden.
NAV_AVAILABLE = _envflag('G1_ENABLE_NAV') or _envflag('G1_ENABLE_LIDAR')


def pretty_pose_name(name: str) -> str:
    """'Demo_2_Winken' -> 'Winken' (Reihenfolge-Praefix ausblenden)."""
    short = re.sub(r"^[^_]+_\d+_", "", name)
    return short.replace("_", " ")


class DemoNode(StreamDeck):
    """Publisher wie der Streamdeck + Rueckkanaele fuer die Statusanzeige."""

    def __init__(self):
        super().__init__()
        self.on_walk_ready = None
        self.on_arm_status = None
        self.on_hand_status = None
        self.create_subscription(Bool, "/g1pilot/arms/walk_ready",
                                 lambda m: self.on_walk_ready and self.on_walk_ready(m.data), 10)
        self.create_subscription(String, "/g1pilot/arm_command/status",
                                 self._arm_status, 10)
        # Hand-Bridge (inspire_ftp/bridge.py): Einzelbefehle + Zustand.
        self.pub_hand_cmd = self.create_publisher(String, "/g1pilot/hand_cmd", 10)
        self.create_subscription(String, "/g1pilot/hand_status", self._hand_status, 10)
        # Stations-Ziele aus der Szene (station_<Name>, siehe scene_bridge.py):
        # name -> (x, y, yaw). on_stations wird nur bei Aenderung gerufen.
        self.stations = {}
        self.on_stations = None
        self.pub_goal = self.create_publisher(PoseStamped, "/g1pilot/goal", 10)
        qos_scene = QoSProfile(depth=1)
        qos_scene.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(MarkerArray, "/scene_markers", self._scene_markers, qos_scene)

    def _scene_markers(self, msg: MarkerArray):
        stations = {}
        for m in msg.markers:
            if m.ns != sm.NS_STATION or m.type != Marker.ARROW or m.action != Marker.ADD:
                continue
            q = m.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            stations[m.text] = (m.pose.position.x, m.pose.position.y, yaw)
        if stations != self.stations:
            self.stations = stations
            if self.on_stations:
                self.on_stations(sorted(stations))

    def publish_goal(self, x, y, yaw):
        """Ziel fuer dijkstra_planner/nav2point (wie »2D Goal Pose« in RViz)."""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(msg)

    def _hand_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if self.on_hand_status:
            self.on_hand_status(data)

    def _arm_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if self.on_arm_status:
            self.on_arm_status(data)


# ── Bereich 1: Modus-Kachel ──────────────────────────────────────────────
class ModeTile(QPushButton):
    """Grosse Kachel mit drei sichtbaren Zustaenden: aktiv / wechselt / inaktiv."""

    def __init__(self, mode):
        super().__init__()
        self.mode = mode
        self.setMinimumHeight(120)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.set_state("inactive")

    def set_state(self, state):
        color = MODE_COLOR[self.mode]
        title = MODE_TITLE[self.mode]
        if state == "active":
            sub, bg, fg, border = "● AKTIV", color, "white", "4px solid white"
        elif state == "pending":
            sub, bg, fg, border = "wechselt …", "#2a2a2a", color, f"4px dashed {color}"
        else:
            sub, bg, fg, border = "antippen zum Wechseln", "#1c1c1c", "#777", "2px solid #333"
        self.setText(f"{title}\n{sub}")
        self.setStyleSheet(f"""
            QPushButton {{ background:{bg}; color:{fg}; border:{border};
                           border-radius:18px; font-size:26px; font-weight:800; }}
            QPushButton:hover:!disabled {{ border-color:{color}; }}
            QPushButton:disabled {{ background:#151515; color:#444; border:2px solid #222; }}
        """)


def big_button(text, color="#2d2d2d", font=20, height=80):
    b = QPushButton(text)
    b.setMinimumHeight(height)
    b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
    b.setStyleSheet(f"""
        QPushButton {{ background:{color}; color:white; font-size:{font}px; font-weight:700;
                       border:1px solid #555; border-radius:14px; padding:8px; }}
        QPushButton:hover {{ border:2px solid #aaa; }}
        QPushButton:pressed {{ background:#555; }}
        QPushButton:checked {{ background:#4CAF50; border:2px solid #80ff80; }}
        QPushButton:disabled {{ background:#1c1c1c; color:#555; border:1px solid #2a2a2a; }}
    """)
    return b


# ── Bereich 2a: Gehen ────────────────────────────────────────────────────
class WalkPanel(QWidget):
    """Joystick-Knopf ODER Pfeiltasten (gedrueckt halten = laufen, loslassen =
    stehen). Ausgabe: velocity() -> (vx, vy, yaw), bereits mit Stufe skaliert."""

    def __init__(self):
        super().__init__()
        self.held = set()        # gerade gehaltene Pfeiltasten
        self.speed = SPEEDS["Langsam"]

        outer = QVBoxLayout(self)
        outer.setSpacing(14)
        # Manuelle Steuerung in einem Container -> bei AUTO NAV komplett sperren.
        self.manual = QWidget()
        lay = QHBoxLayout(self.manual)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(30)
        outer.addWidget(self.manual, 1)

        # Knopf
        left = QVBoxLayout()
        left.addWidget(self._caption("Knopf ziehen"))
        self.joystick = VirtualJoystick(260)
        left.addWidget(self.joystick, alignment=Qt.AlignmentFlag.AlignCenter)
        left.addStretch(1)
        lay.addLayout(left, 1)

        # Pfeile + Drehen
        mid = QVBoxLayout()
        mid.addWidget(self._caption("…oder Pfeil gedrückt halten"))
        pad = QGridLayout()
        pad.setSpacing(10)
        arrows = {
            "fwd": ("▲", 0, 1), "back": ("▼", 2, 1),
            "left": ("◀", 1, 0), "right": ("▶", 1, 2),
            "turn_l": ("⟲\ndrehen", 0, 0), "turn_r": ("⟳\ndrehen", 0, 2),
        }
        for key, (txt, r, c) in arrows.items():
            b = big_button(txt, font=30 if len(txt) == 1 else 18, height=90)
            b.setFixedWidth(110)
            b.pressed.connect(lambda k=key: self.held.add(k))
            b.released.connect(lambda k=key: self.held.discard(k))
            pad.addWidget(b, r, c)
        mid.addLayout(pad)
        mid.addStretch(1)
        lay.addLayout(mid, 1)

        # Tempo
        right = QVBoxLayout()
        right.addWidget(self._caption("Tempo"))
        self.speed_btns = {}
        for name, val in SPEEDS.items():
            b = big_button(name, height=70)
            b.setCheckable(True)
            b.clicked.connect(lambda _, n=name: self.set_speed(n))
            right.addWidget(b)
            self.speed_btns[name] = b
        right.addStretch(1)
        lay.addLayout(right)
        self.set_speed("Langsam")

        # AUTO NAV: Roboter faehrt selbststaendig zum in RViz gesetzten Ziel
        # (/g1pilot/auto_enable -> joy_mux gibt den Nav-Joy weiter).
        nav = QHBoxLayout()
        nav.setSpacing(16)
        self.btn_auto_nav = big_button("", "#283593", height=70)
        self.btn_auto_nav.setCheckable(True)
        self.btn_auto_nav.setFixedWidth(360)
        self.btn_auto_nav.setEnabled(NAV_AVAILABLE)
        nav.addWidget(self.btn_auto_nav)
        self.nav_hint = self._caption("")
        self.nav_hint.setWordWrap(True)
        nav.addWidget(self.nav_hint, 1)
        outer.addLayout(nav)
        # Stations-Knoepfe: setzen das Ziel + schalten AUTO NAV ein. Werden aus
        # den station_<Name>-Markierungen der geladenen Szene gebaut.
        self.station_row = QHBoxLayout()
        self.station_row.setSpacing(10)
        self.station_row.addWidget(self._caption("Zur Station:"))
        self.station_btns = []
        self.on_station = None
        self.station_row.addStretch(1)
        outer.addLayout(self.station_row)
        self.set_stations([])
        self.set_auto_nav_view(False)

    @staticmethod
    def _caption(text):
        lbl = QLabel(text)
        lbl.setStyleSheet("color:#aaa; font-size:15px;")
        return lbl

    def set_stations(self, names):
        for b in self.station_btns:
            self.station_row.removeWidget(b)
            b.deleteLater()
        self.station_btns = []
        if not names:
            b = QLabel("keine — in der Szene Objekte »station_<Name>« anlegen")
            b.setStyleSheet("color:#666; font-size:14px;")
            self.station_row.insertWidget(1, b)
            self.station_btns.append(b)
            return
        for i, name in enumerate(names):
            b = big_button(sm.station_label(name), "#2e7d32", font=15, height=48)
            b.setMaximumWidth(240)
            b.setEnabled(NAV_AVAILABLE)
            b.clicked.connect(lambda _, n=name: self.on_station and self.on_station(n))
            self.station_row.insertWidget(1 + i, b)
            self.station_btns.append(b)

    def set_speed(self, name):
        self.speed = SPEEDS[name]
        for n, b in self.speed_btns.items():
            b.setChecked(n == name)

    def velocity(self):
        h = self.held
        vx = self.joystick.vx + (1 if "fwd" in h else 0) - (1 if "back" in h else 0)
        vy = self.joystick.vy + (1 if "left" in h else 0) - (1 if "right" in h else 0)
        yaw = (1 if "turn_l" in h else 0) - (1 if "turn_r" in h else 0)
        clamp = lambda v: max(-1.0, min(1.0, v))   # noqa: E731
        return clamp(vx) * self.speed, clamp(vy) * self.speed, clamp(yaw) * self.speed

    def set_auto_nav_view(self, on):
        """Nur Anzeige: Knopf-Text/Haken, Hinweis, manuelle Steuerung sperren."""
        self.btn_auto_nav.setChecked(on)
        self.manual.setEnabled(not on)
        # Gesperrt sichtbar machen (der gezeichnete Joystick kennt kein :disabled).
        fade = QGraphicsOpacityEffect(self.manual)
        fade.setOpacity(0.3 if on else 1.0)
        self.manual.setGraphicsEffect(fade)
        if on:
            self.reset()
            self.btn_auto_nav.setText("AUTO NAV  ● AN\nantippen zum Stoppen")
            self.nav_hint.setText("Navigiert selbstständig zum Ziel. Ziel in RViz setzen "
                                  "(»2D Goal Pose«). Manuelle Steuerung ist gesperrt.")
        elif NAV_AVAILABLE:
            self.btn_auto_nav.setText("AUTO NAV\nselbst zum Ziel laufen")
            self.nav_hint.setText("Erst Ziel in RViz setzen, dann AUTO NAV einschalten.")
        else:
            self.btn_auto_nav.setText("AUTO NAV")
            self.nav_hint.setText("Navigation ist nicht gestartet — im Startmenü "
                                  "Ausstattung → »Navigation« aktivieren.")

    def reset(self):
        self.held.clear()
        self.joystick.mouseReleaseEvent(None)   # Knopf zentrieren -> vx=vy=0


# ── Bereich 2b: Greifen ──────────────────────────────────────────────────
class ManipPanel(QWidget):
    """Umschalter ARME | HAENDE, darunter jeweils eine Seite.
    ARME: Beispielbewegungen als grosse Knoepfe + 'Ganze Demo abspielen'.
    Einzelfunktionen (Pose anfahren/speichern, Marker folgen) liegen
    eingeklappt unter 'Erweitert' -- fuer den Betreuer, nicht fuer Besucher.
    HAENDE: HandPanel (ganze Hand + umschaltbar einzelne Finger)."""

    def __init__(self, gui):
        super().__init__()
        self.gui = gui
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(8)

        seg = QHBoxLayout()
        seg.setSpacing(0)
        self.view_btns = {}
        for key, text, radius in (("arms", "ARME", "14px 0 0 14px"),
                                  ("hands", "HÄNDE", "0 14px 14px 0")):
            b = QPushButton(text)
            b.setCheckable(True)
            b.setMinimumHeight(42)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            b.setStyleSheet(f"""
                QPushButton {{ background:#1c1c1c; color:#888; font-size:17px; font-weight:800;
                               border:2px solid #333; border-radius:0; padding:6px;
                               border-top-left-radius:{radius.split()[0]};
                               border-top-right-radius:{radius.split()[1]};
                               border-bottom-right-radius:{radius.split()[2]};
                               border-bottom-left-radius:{radius.split()[3]}; }}
                QPushButton:checked {{ background:{MODE_COLOR[MANIP]}; color:white;
                                       border-color:white; }}
                QPushButton:hover:!checked {{ border-color:{MODE_COLOR[MANIP]}; }}
            """)
            b.clicked.connect(lambda _, k=key: self.show_view(k))
            seg.addWidget(b)
            self.view_btns[key] = b
        seg_wrap = QHBoxLayout()
        seg_wrap.addStretch(1)
        seg_wrap.addLayout(seg, 4)
        seg_wrap.addStretch(1)
        outer.addLayout(seg_wrap)

        self.views = QStackedWidget()
        outer.addWidget(self.views, 1)
        arms = QWidget()
        self.views.addWidget(arms)
        self.hand_panel = HandPanel(
            gui.node, status_cb=lambda t: gui._status(t, MODE_COLOR[MANIP]))
        self.views.addWidget(self.hand_panel)
        self.view_pages = {"arms": arms, "hands": self.hand_panel}

        lay = QVBoxLayout(arms)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(14)

        head = QHBoxLayout()
        head.addWidget(WalkPanel._caption("Beispielbewegungen"))
        head.addStretch(1)
        self.btn_play = big_button("Ganze Demo abspielen", "#2e7d32", height=60)
        self.btn_play.setFixedWidth(320)
        head.addWidget(self.btn_play)
        lay.addLayout(head)

        self.seq_row = QHBoxLayout()
        self.seq_row.setSpacing(10)
        lay.addLayout(self.seq_row)
        self.demo_grid = QGridLayout()
        self.demo_grid.setSpacing(10)
        lay.addLayout(self.demo_grid)
        self.demo_hint = WalkPanel._caption("")
        lay.addWidget(self.demo_hint)

        row = QHBoxLayout()
        self.btn_open = big_button("Hände öffnen")
        self.btn_close = big_button("Hände schließen")
        self.btn_home = big_button("Grundstellung")
        self.btn_cancel = big_button("Bewegung stoppen", "#5d4037")
        for b in (self.btn_open, self.btn_close, self.btn_home, self.btn_cancel):
            row.addWidget(b)
        lay.addLayout(row)

        # Erweitert (eingeklappt)
        self.btn_expert = QPushButton("Erweitert ▸")
        self.btn_expert.setCheckable(True)
        self.btn_expert.setStyleSheet("QPushButton{background:transparent;color:#888;"
                                      "border:none;font-size:14px;text-align:left;}")
        lay.addWidget(self.btn_expert)
        self.expert = QFrame()
        ex = QHBoxLayout(self.expert)
        ex.setContentsMargins(0, 0, 0, 0)
        self.btn_goto = big_button("Pose anfahren …", font=15, height=50)
        self.btn_save = big_button("Pose speichern …", font=15, height=50)
        self.btn_marker = big_button("Marker folgen", font=15, height=50)
        self.btn_marker.setCheckable(True)
        self.btn_marker.setChecked(True)   # Default des interactive_marker-Node
        for b in (self.btn_goto, self.btn_save, self.btn_marker):
            ex.addWidget(b)
        self.expert.setVisible(False)
        self.btn_expert.toggled.connect(self._toggle_expert)
        lay.addWidget(self.expert)
        lay.addStretch(1)
        self.show_view("arms")

    def show_view(self, key):
        self.views.setCurrentWidget(self.view_pages[key])
        for k, b in self.view_btns.items():
            b.setChecked(k == key)

    def _toggle_expert(self, on):
        self.expert.setVisible(on)
        self.btn_expert.setText("Erweitert ▾" if on else "Erweitert ▸")

    def fill_sequence_buttons(self, sequences):
        """sequences: [(Anzeigename, [Posennamen ...]), ...]"""
        while self.seq_row.count():
            w = self.seq_row.takeAt(0).widget()
            if w is not None:
                w.deleteLater()
        if not sequences:
            return
        self.seq_row.addWidget(WalkPanel._caption("Abläufe:"))
        for label, names in sequences:
            b = big_button(label, "#1565c0", font=17, height=64)
            b.setToolTip(" → ".join(pretty_pose_name(n) for n in names))
            b.clicked.connect(lambda _, ns=list(names), lb=label: self.gui.run_poses(ns, lb))
            self.seq_row.addWidget(b)

    def fill_demo_buttons(self, names, note=""):
        while self.demo_grid.count():
            self.demo_grid.takeAt(0).widget().deleteLater()
        for i, name in enumerate(names[:MAX_DEMO_BUTTONS]):
            b = big_button(pretty_pose_name(name), "#37474f", height=90)
            b.clicked.connect(lambda _, n=name: self.gui.run_poses([n]))
            self.demo_grid.addWidget(b, i // 4, i % 4)
        self.demo_hint.setText(note)
        self.demo_hint.setVisible(bool(note))
        self.btn_play.setEnabled(bool(names))


# ── Hauptfenster ─────────────────────────────────────────────────────────
class DemoGUI(QWidget):

    def __init__(self, node: DemoNode):
        super().__init__()
        self.node = node
        self.sim_mode = is_sim_mode()
        self.mode = None            # aktiver Modus (WALK/MANIP) oder None
        self.pending = None         # angeforderter, noch nicht bestaetigter Modus
        self.started = self.sim_mode
        self._estop_unacked = False   # Sim: NOT-HALT noch nicht per START quittiert
        self.queue = []             # verbleibende Posen einer laufenden Sequenz
        self.current_pose = None
        self._walk_req = 0          # verwirft veraltete WALK-Timeouts
        self.auto_nav = False

        node.on_walk_ready = self._on_walk_ready
        node.on_arm_status = self._on_arm_status

        self.setWindowTitle("G1 Demo — " + ("SIM" if self.sim_mode else "REAL"))
        self.setStyleSheet("QWidget { background:#111; }")
        self.resize(1200, 860)
        self._build()
        self._wire()
        self._refresh()

        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._publish_cmd_vel)
        self.cmd_timer.start(33)

        # Wie ui_interface: Auto-Start NUR in der Sim (Arme an + BALANCING).
        if self.sim_mode:
            QTimer.singleShot(3000, lambda: self.request_mode(MANIP))
        else:
            self._status("REAL: zuerst »Roboter starten« drücken.", "#ffb300")

    # ── Aufbau ──────────────────────────────────────────────────────────
    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 16, 20, 16)
        root.setSpacing(14)

        # Bereich 1: Modus
        root.addWidget(self._section_label("1  ·  MODUS"))
        top = QHBoxLayout()
        top.setSpacing(16)
        self.tiles = {WALK: ModeTile(WALK), MANIP: ModeTile(MANIP)}
        top.addWidget(self.tiles[WALK])
        top.addWidget(self.tiles[MANIP])
        root.addLayout(top)
        self.btn_start = big_button("Roboter starten", "#455a64", height=60)
        self.btn_start.setVisible(not self.sim_mode)
        root.addWidget(self.btn_start)

        # Bereich 2: Steuerung des aktiven Modus
        root.addWidget(self._section_label("2  ·  STEUERUNG"))
        self.panel_frame = QFrame()
        self.panel_frame.setObjectName("panel")
        pf = QVBoxLayout(self.panel_frame)
        pf.setContentsMargins(18, 14, 18, 14)
        self.panel_title = QLabel()
        pf.addWidget(self.panel_title)
        self.stack = QStackedWidget()
        self.idle_panel = QLabel("Bitte oben einen Modus wählen.")
        self.idle_panel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.idle_panel.setStyleSheet("color:#666; font-size:22px;")
        self.walk_panel = WalkPanel()
        self.manip_panel = ManipPanel(self)
        for w in (self.idle_panel, self.walk_panel, self.manip_panel):
            self.stack.addWidget(w)
        pf.addWidget(self.stack, 1)
        root.addWidget(self.panel_frame, 1)

        # Bereich 3: Status + Sicherheit
        bottom = QHBoxLayout()
        bottom.setSpacing(14)
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(70)
        bottom.addWidget(self.status, 1)
        self.btn_reset_scene = big_button("Szene\nzurücksetzen", font=15, height=70)
        self.btn_reset_scene.setFixedWidth(170)
        self.btn_reset_scene.setVisible(self.sim_mode)
        bottom.addWidget(self.btn_reset_scene)
        # Nur Sim: Roboter nach NOT-HALT/Sturz zurueck an den Startpunkt (HOLD,
        # gehalten wie beim Start) -> danach Modus waehlen, steht sanft auf.
        self.btn_reset_robot = big_button("Roboter\nzurücksetzen", font=15, height=70)
        self.btn_reset_robot.setFixedWidth(170)
        self.btn_reset_robot.setVisible(self.sim_mode)
        bottom.addWidget(self.btn_reset_robot)
        # Stoer-Test (nur Sim): schubst den Roboter -> zeigt, dass er sich faengt.
        self.btn_push = big_button("Roboter\nschubsen", font=15, height=70)
        self.btn_push.setFixedWidth(170)
        self.btn_push.setVisible(self.sim_mode)
        bottom.addWidget(self.btn_push)
        self.btn_estop = QPushButton("NOT-HALT")
        self.btn_estop.setFixedSize(220, 90)
        self.btn_estop.setStyleSheet("""
            QPushButton { background:#c62828; color:white; font-size:26px; font-weight:900;
                          border:3px solid #ff8a80; border-radius:45px; }
            QPushButton:pressed { background:#ff1744; }
        """)
        bottom.addWidget(self.btn_estop)
        root.addWidget(self._section_label("3  ·  STATUS"))
        root.addLayout(bottom)
        self._status("Startet …", "#aaa")

    @staticmethod
    def _section_label(text):
        lbl = QLabel(text)
        lbl.setStyleSheet("color:#555; font-size:12px; font-weight:700; letter-spacing:2px;")
        return lbl

    def _wire(self):
        n = self.node
        mp = self.manip_panel
        self.walk_panel.btn_auto_nav.clicked.connect(lambda on: self.set_auto_nav(on))
        self.tiles[WALK].clicked.connect(lambda: self.request_mode(WALK))
        self.tiles[MANIP].clicked.connect(lambda: self.request_mode(MANIP))
        self.btn_start.clicked.connect(self._start_robot)
        self.btn_estop.clicked.connect(self.emergency_stop)
        self.btn_reset_scene.clicked.connect(lambda: self._pulse(n.pub_scene_reset))
        self.btn_push.clicked.connect(self._push)
        self.btn_reset_robot.clicked.connect(self._reset_robot)
        self.walk_panel.on_station = self.go_to_station
        n.on_stations = self.walk_panel.set_stations
        if n.stations:
            self.walk_panel.set_stations(sorted(n.stations))

        mp.btn_play.clicked.connect(lambda: self.run_poses(self._demo_names()[0]))
        mp.btn_open.clicked.connect(lambda: mp.hand_panel.hand_action(None, "open"))
        mp.btn_close.clicked.connect(lambda: mp.hand_panel.hand_action(None, "close"))
        n.on_hand_status = mp.hand_panel.on_status
        mp.btn_home.clicked.connect(self._home)
        mp.btn_cancel.clicked.connect(self._cancel)
        mp.btn_goto.clicked.connect(self._pose_goto_dialog)
        mp.btn_save.clicked.connect(self._pose_save_dialog)
        mp.btn_marker.toggled.connect(lambda on: n.publish_bool(n.pub_marker_follow, on))

    # ── Modus ───────────────────────────────────────────────────────────
    def _start_robot(self):
        """REAL: START (Standby). Danach sind die Modus-Kacheln freigegeben."""
        self.node.publish_bool(self.node.pub_start, True)
        self.started = True
        self.btn_start.setVisible(False)
        self._estop_unacked = False
        self._status("Gestartet. Jetzt einen Modus wählen.", "#aaa")
        self._refresh()

    def request_mode(self, mode):
        if not self.started or mode == self.mode or mode == self.pending:
            return
        self._cancel_sequence()
        self.set_auto_nav(False)
        self.walk_panel.reset()
        n = self.node
        if self._estop_unacked:
            # Sim nach NOT-HALT: START quittiert den E-Stop-Latch im
            # arm_controller -- sonst ignoriert er jedes ENABLE und die Arme
            # bleiben dauerhaft schlaff (Real: Knopf "Roboter starten").
            # Getrennte Topics -> keine Reihenfolge-Garantie: Modus (ENABLE)
            # erst kurz danach senden, sonst kaeme er evtl. vor dem START an.
            n.publish_bool(n.pub_start, True)
            self._estop_unacked = False
            self._status("NOT-HALT quittiert …", "#aaa")
            QTimer.singleShot(300, lambda: self.request_mode(mode))
            return
        if mode == MANIP:
            # BALANCING: Fuesse geplant, Arme frei -> sofort bestaetigt.
            n.publish_bool(n.pub_arms_enabled, True)
            n.publish_bool(n.pub_start_balancing, True)
            self.mode, self.pending = MANIP, None
            self._status("Greifen bereit — Bewegung auswählen.", MODE_COLOR[MANIP])
            self._reload_demo_buttons()
        else:
            # WALK: Arme fahren erst in die Lauf-Pose -> "wechselt", bis
            # arms/walk_ready kommt (oder Timeout).
            n.publish_bool(n.pub_start_walking, True)
            self.pending = WALK
            self._status("Arme werden eingeklappt …", MODE_COLOR[WALK])
            self._walk_req += 1
            req = self._walk_req
            QTimer.singleShot(WALK_SWITCH_TIMEOUT_MS,
                              lambda: req == self._walk_req and self._on_walk_ready(True))
        self._refresh()

    def _on_walk_ready(self, ready):
        if ready and self.pending == WALK:
            self.mode, self.pending = WALK, None
            self._status("Gehen bereit — Knopf ziehen oder Pfeil halten.", MODE_COLOR[WALK])
            self._refresh()

    def _refresh(self):
        for m, tile in self.tiles.items():
            tile.setEnabled(self.started)
            tile.set_state("active" if m == self.mode
                           else "pending" if m == self.pending else "inactive")
        shown = self.pending or self.mode
        color = MODE_COLOR.get(shown, "#333")
        self.panel_frame.setStyleSheet(
            f"QFrame#panel {{ border:3px solid {color}; border-radius:18px; }}")
        if self.mode is None:
            self.stack.setCurrentWidget(self.idle_panel)
            self.panel_title.setText("")
        else:
            self.stack.setCurrentWidget(self.walk_panel if self.mode == WALK else self.manip_panel)
            self.panel_title.setText(MODE_TITLE[self.mode])
        self.panel_title.setStyleSheet(f"color:{color}; font-size:20px; font-weight:800;")

    def set_auto_nav(self, on):
        """AUTO NAV an/aus. Aus sendet joy_mux einmal einen neutralen Stop."""
        on = bool(on) and NAV_AVAILABLE and self.mode == WALK
        if on == self.auto_nav:
            self.walk_panel.set_auto_nav_view(on)
            return
        self.auto_nav = on
        self.node.publish_bool(self.node.pub_auto_enable, on)
        self.walk_panel.set_auto_nav_view(on)
        if on:
            self._status("AUTO NAV: navigiert selbstständig zum Ziel …", MODE_COLOR[WALK])
        elif self.mode == WALK:
            self._status("AUTO NAV aus — manuelle Steuerung frei.", MODE_COLOR[WALK])

    def go_to_station(self, name):
        """Stations-Knopf: Ziel setzen -> Planer plant -> AUTO NAV laeuft hin."""
        st = self.node.stations.get(name)
        if st is None or self.mode != WALK:
            return
        self.node.publish_goal(*st)
        self.set_auto_nav(True)
        self._status(f"AUTO NAV: läuft zur Station »{sm.station_label(name)}« …",
                     MODE_COLOR[WALK])

    def _publish_cmd_vel(self):
        # Dauerhaft senden (wie ui_interface): ausserhalb von GEHEN immer 0.
        # Bei AUTO NAV GAR NICHT senden: in der Sim faehrt Nav ueber dasselbe
        # Topic (joy_to_cmdvel) -- unsere Nullen wuerden den Roboter staendig bremsen.
        if self.auto_nav:
            return
        vx, vy, yaw = self.walk_panel.velocity() if self.mode == WALK else (0.0, 0.0, 0.0)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(yaw)
        self.node.pub_cmd_vel.publish(msg)

    # ── Greifen ─────────────────────────────────────────────────────────
    def _pose_store(self):
        try:
            from g1pilot.manipulation.pose_store import PoseStore
            return PoseStore()
        except Exception as e:   # noqa: BLE001
            self.node.get_logger().warn(f"Pose-Store nicht lesbar: {e}")
            return None

    def _demo_names(self):
        """-> (Posennamen, Hinweis). Kategorie DEMO_CATEGORY in Namensreihenfolge
        (wie demo_sequence.py); fehlt sie, alle Posen als Fallback."""
        store = self._pose_store()
        if store is None:
            return [], "Pose-Store nicht lesbar."
        grouped = store.list_grouped()
        names = sorted(grouped.get(DEMO_CATEGORY, []), key=str.lower)
        if names:
            return names, ""
        all_names = sorted((n for cat, ns in grouped.items()
                            if not cat.startswith(SEQUENCE_PREFIX) for n in ns), key=str.lower)
        if not all_names:
            return [], "Noch keine Posen gespeichert (Erweitert → Pose speichern)."
        return all_names, (f"Keine Kategorie »{DEMO_CATEGORY}« — zeige alle Posen. "
                           f"Posen in »{DEMO_CATEGORY}« speichern, um die Auswahl festzulegen.")

    def _sequences(self):
        store = self._pose_store()
        if store is None:
            return []
        return [(cat[len(SEQUENCE_PREFIX):].strip(), sorted(ns, key=str.lower))
                for cat, ns in sorted(store.list_grouped().items())
                if cat.startswith(SEQUENCE_PREFIX) and ns]

    def _reload_demo_buttons(self):
        self.manip_panel.fill_demo_buttons(*self._demo_names())
        self.manip_panel.fill_sequence_buttons(self._sequences())

    def run_poses(self, names, label=None):
        """Eine oder mehrere Posen nacheinander anfahren; der naechste Schritt
        startet, wenn arm_command/status 'reached' meldet."""
        if self.mode != MANIP or not names:
            return
        self.queue = list(names)
        self._seq_label = label
        self._seq_total = len(names)
        self._next_pose()

    def _next_pose(self):
        if not self.queue:
            self.current_pose = None
            self._status("Fertig.", "#66bb6a")
            return
        self.current_pose = self.queue.pop(0)
        self.node.publish_str(self.node.pub_pose_goto, self.current_pose)
        step = ""
        if getattr(self, "_seq_label", None):
            step = f"{self._seq_label} · Schritt {self._seq_total - len(self.queue)}/{self._seq_total}: "
        self._status(f"{step}{pretty_pose_name(self.current_pose)} …", MODE_COLOR[MANIP])

    def _on_arm_status(self, data):
        if self.current_pose is None:
            return
        state = data.get("state")
        if state not in TERMINAL:
            return
        if state == "reached":
            QTimer.singleShot(300, self._next_pose)
        else:
            self.queue.clear()
            self.current_pose = None
            reason = data.get("reason", "")
            self._status(f"Abgebrochen ({state}) {reason}", "#ef5350")

    def _cancel_sequence(self):
        self.queue.clear()
        self.current_pose = None

    def _cancel(self):
        self._cancel_sequence()
        self.node.publish_bool(self.node.pub_pose_cancel, True)
        self._status("Bewegung gestoppt.", "#aaa")

    def _home(self):
        """Grundstellung = Sichere Pose (Ellbogen hinten, Haende seitlich ueber
        Tischhoehe) -- geplant angefahren wie jede Pose. Fehlt sie im Speicher,
        das alte Homing des arm_controller."""
        self._cancel_sequence()
        store = self._pose_store()
        if store is not None and store.get(HOME_POSE) is not None:
            self.run_poses([HOME_POSE], "Grundstellung")
            return
        self._pulse(self.node.pub_arms_home)
        self._status("Arme fahren in Grundstellung …", MODE_COLOR[MANIP])

    def _pose_goto_dialog(self):
        store = self._pose_store()
        grouped = store.list_grouped() if store is not None else {}
        if not any(grouped.values()):
            QMessageBox.information(self, "Keine Posen", "Noch keine Pose gespeichert.")
            return
        dlg = PoseLoadDialog(self, grouped, components_of=store.components)
        if dlg.exec() == QDialog.DialogCode.Accepted and dlg.selected_name():
            self.run_poses([dlg.selected_name()])

    def _pose_save_dialog(self):
        store = self._pose_store()
        cats = store.list_categories() if store is not None else [DEMO_CATEGORY]
        if DEMO_CATEGORY not in cats:
            cats = [DEMO_CATEGORY] + cats
        dlg = PoseSaveDialog(self, store=store, categories=cats)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        self.node.publish_str(self.node.pub_pose_save, json.dumps(dlg.result_data()))
        self._status("Pose gespeichert.", "#66bb6a")
        # arm_controller schreibt die Datei asynchron -> kurz warten, dann neu laden.
        QTimer.singleShot(1500, self._reload_demo_buttons)

    # ── Sicherheit / Status ─────────────────────────────────────────────
    def _pulse(self, pub, duration=1000):
        """Bool-Impuls True -> False (wie flash_button im Streamdeck)."""
        self.node.publish_bool(pub, True)
        QTimer.singleShot(duration, lambda: self.node.publish_bool(pub, False))

    def _push(self):
        """Wie PUSH ROBOT im Streamdeck: 400-ms-Impuls auf /g1pilot/push."""
        self._pulse(self.node.pub_push, duration=400)
        self._status("Roboter wird geschubst …", "#ffb300")

    def _reset_robot(self):
        """Nur Sim: wie beim Start -> loco_sim HOLD; die Bridge stellt den Roboter
        an den Startpunkt (bzw. g1_spawn) und haelt ihn (Weld). Beim naechsten
        Modus-Klick wird er in die Stand-Pose gestellt und freigegeben."""
        n = self.node
        self._cancel_sequence()
        self.set_auto_nav(False)
        self.walk_panel.reset()
        for pub in (n.pub_start_balancing, n.pub_start_walking, n.pub_arms_enabled):
            n.publish_bool(pub, False)
        n.publish_bool(n.pub_start, True)
        self._estop_unacked = False
        self.mode = self.pending = None
        self.started = True
        self._refresh()
        self._status("Roboter steht wieder am Startpunkt (gehalten) — "
                     "oben einen Modus wählen.", "#aaa")

    def emergency_stop(self):
        n = self.node
        self._cancel_sequence()
        self.set_auto_nav(False)
        self.walk_panel.reset()
        for pub in (n.pub_start, n.pub_start_balancing, n.pub_start_walking,
                    n.pub_arms_enabled, n.pub_arms_home):
            n.publish_bool(pub, False)
        n.publish_bool(n.pub_emergency_stop, True)
        self._estop_unacked = self.sim_mode
        self.mode = self.pending = None
        # REAL: nach NOT-HALT wieder bewusst ueber START gehen.
        self.started = self.sim_mode
        self.btn_start.setVisible(not self.sim_mode)
        self._refresh()
        self._status("NOT-HALT aktiv — Roboter ist weich geschaltet. "
                     "Zum Fortsetzen oben einen Modus wählen.", "#ef5350")

    def _status(self, text, color):
        self.status.setText(text)
        self.status.setStyleSheet(
            f"color:{color}; font-size:20px; font-weight:700; padding:10px 16px;"
            f"background:#1a1a1a; border-left:6px solid {color}; border-radius:8px;")


def main():
    rclpy.init()
    node = DemoNode()
    app = QApplication(sys.argv)
    DarkStyle(app)
    gui = DemoGUI(node)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

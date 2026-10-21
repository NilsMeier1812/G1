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
                                    ARME   = Ablaeufe (Pose-Store-Kategorien
                                             "Ablauf <Name>"), Grundstellung und
                                             eingeklappter "Erweitert"-Bereich.
                                    HAENDE = ganze Hand auf/zu, Griffkraft,
                                             Kraftzonen + einzelne Finger
                                             (hand_panel.py, ersetzt die
                                             Browser-GUIs der Hand-Bridge).
  3) STATUS (unten)    : was der Roboter gerade tut + immer sichtbarer NOT-HALT.

Controller (PS4/Xbox) und GUI teilen EINEN Bedienzustand (shared_state.py,
/g1pilot/teleop_state): Modus-Kacheln, Hand-Auswahl, Tempo »Hand bewegen«
und Geh-Tempo folgen dem Controller und umgekehrt. Die Info-Zeile zeigt
dazu, was am Controller gerade passiert (Gehen mit L2, Arm-Sperre,
NOT-HALT, welcher Controller verbunden ist).

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
import time

from PyQt6.QtWidgets import (
    QApplication, QWidget, QGridLayout, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QStackedWidget, QFrame, QDialog, QMessageBox, QSizePolicy,
    QGraphicsOpacityEffect, QTableWidget, QTableWidgetItem, QScrollArea,
)
from PyQt6.QtCore import QTimer, Qt, QRectF, QPointF, QEvent
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QKeySequence, QShortcut

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

from g1pilot.navigation import scene_markers as sm

from g1pilot.utils.window_style import DarkStyle, set_app_identity
from g1pilot.utils.common import is_sim_mode
from g1pilot.utils import g1_temperature as g1t
from g1pilot.utils import push_charge
from g1pilot.utils.push_charge import PushCharger
from g1pilot.teleoperation.sim_autostart import SimAutoStart
from g1pilot.teleoperation.ui_interface import (
    StreamDeck, VirtualJoystick, PoseSaveDialog, PoseLoadDialog,
)
from g1pilot.teleoperation.hand_panel import HandPanel
from g1pilot.teleoperation import status_line
from g1pilot.teleoperation import shared_state
from g1pilot.teleoperation.ps4_arm_teleop import mirror_twist
from g1pilot.teleoperation.ps4_joystick import DEVICE_TOPIC
from g1pilot.teleoperation.hand_jog import (
    JOG_FRAME, JOG_TF, JOG_SPEEDS, JOG_CMD_TOPIC, CommandedPoses, HoldStreamer, hold_pose,
    JogReference, jog_reference_mode, lead_limits, jog_step,
)

WALK = "walk"
MANIP = "manip"

# Farbcode je Modus -- zieht sich durch Kachel, Steuerbereich-Rahmen und Status.
MODE_COLOR = {WALK: "#1e88e5", MANIP: "#fb8c00"}
MODE_TITLE = {WALK: "GEHEN", MANIP: "GREIFEN"}

# Geschwindigkeitsstufen (normiert, loco_sim skaliert mit max_cmd ~0.8 m/s).
# Fuer Vorfuehrungen bewusst gedeckelt -- "Normal" ist nicht Vollgas.
SPEEDS = {"Langsam": 0.3, "Normal": 0.6}
# Geh-Tempo ist mit dem Controller geteilt (Langsam/Normal/Schnell). »Schnell«
# gibt es nur am Controller (L2 + ▲): die GUI zeigt es an, geht selbst aber
# hoechstens mit »Normal«.
WALK_GUI_MAX = "Normal"

# Vorgeschlagene Kategorie im Dialog »Pose speichern« (auch demo_sequence.py).
DEMO_CATEGORY = os.environ.get("G1_DEMO_CATEGORY", "Demo")
# Ablaeufe: jede Kategorie "Ablauf <Name>" = eine Sequenz (Posen in Namens-
# reihenfolge, z.B. AP1_01_..., AP1_02_...) -> ein Knopf "<Name>".
SEQUENCE_PREFIX = "Ablauf "
# Knopf "Sichere Pose" faehrt diese gespeicherte Pose an.
SAFE_POSE = os.environ.get("G1_SAFE_POSE", "Sichere_Pose")
# WALK gilt als "angekommen", wenn arms/walk_ready kommt -- spaetestens nach
# diesem Timeout (loco_sim laeuft dann ohnehin selbst los).
WALK_SWITCH_TIMEOUT_MS = 16000   # > loco_sim walk_arm_timeout_s (15 s)

TERMINAL = ("reached", "failed", "rejected", "cancelled")

# AUTO-NAV-Anzeige: aus | an, wartet auf Ziel | laeuft (gestrichelt) | angekommen.
NAV_OFF, NAV_WAIT, NAV_MOVING, NAV_ARRIVED = "off", "waiting", "moving", "arrived"
# Stations-Knopf: kommt so lange kein Pfad (nav_status moving/no_path), gilt
# das Ziel als unerreichbar (z.B. Planer laeuft nicht) -> AUTO NAV wieder aus.
NAV_PLAN_TIMEOUT_MS = 5000
NAV_DONE_COLOR = "#66bb6a"

# »Hand bewegen« (Joystick + Hoch/Runter-Schieber): kartesische Hand-Ziele im
# pelvis-Frame auf /g1pilot/hand_goal/<side> -- dieselbe Schnittstelle wie der
# RViz-Marker; der arm_controller loest per IK (inkl. Kollisions-Gate). Tempo,
# Vorlauf und Jog-Mathematik teilt sich die GUI mit dem PS4-Controller
# (teleoperation/hand_jog.py).
# »Gespiegelt« wie Steuerkreuz ▼ am Controller: beide Haende, die linke
# spiegelbildlich zur rechten (ps4_arm_teleop.mirror_twist).
JOG_SIDES = {"Links": ("left",), "Rechts": ("right",), "Beide": ("left", "right"),
             "Gespiegelt": ("left", "right")}
# GUI-Name <-> Auswahl im gemeinsamen Zustand (= ps4_arm_teleop.SELECTIONS)
JOG_SIDE_KEY = {"Links": "left", "Rechts": "right", "Beide": "both", "Gespiegelt": "mirror"}
JOG_SIDE_NAME = {v: k for k, v in JOG_SIDE_KEY.items()}

# »Sim beenden«: Trigger-Datei im bind-gemounteten Repo (docker-compose:
# .:/ros2_ws/src/g1pilot). Der Host-Watcher docker/sim_shutdown_watcher.sh
# (gestartet von start.sh / make sim) stoppt daraufhin den ganzen Sim-Stack.
SIM_SHUTDOWN_TRIGGER = "/ros2_ws/src/g1pilot/.sim_shutdown_request"
QUIT_CONFIRM_MS = 4000      # so lange gilt der erste Klick als »scharf«
QUIT_ACK_TIMEOUT_MS = 3000  # Watcher loescht die Datei -> sonst Hinweis
# »Real beenden«: erst Laufen stoppen und die Arme sicher an den Onboard-Regler
# zurueckgeben (DISABLE: Rueckfahrt in die Pose vom ENABLE, dann Gewichtsrampe),
# DANN den Stack stoppen. Host-Watcher: docker/real_shutdown_watcher.sh
# (sanftes docker compose stop -> SIGTERM -> arm_controller.shutdown_release).
REAL_SHUTDOWN_TRIGGER = "/ros2_ws/src/g1pilot/.real_shutdown_request"
REAL_RELEASE_TIMEOUT_MS = 20000   # > release_timeout_s (12 s) + Rampe
REAL_RAMP_WAIT_MS = 2500          # nach arms/state False: Gewichtsrampe 1->0 (2 s real)


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
        # Motor-/IMU-Temperaturen des echten G1 (robot_state, reiner Lesepfad).
        self.temps = None
        self.temps_t = None
        self.create_subscription(String, "/g1pilot/temperatures", self._temperatures, 10)
        # Info-Zeile (nur Anzeige): Onboard-Modus des echten G1 (loco_client)
        # und Auswahl/Tempo des PS4-Controllers (ps4_arm_teleop). Beide latched.
        self.onboard = self.onboard_t = None
        self.ps4 = self.ps4_t = None
        qos_latched = QoSProfile(depth=1)
        qos_latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_loco_request = self.create_publisher(String, "/g1pilot/loco_request", 10)
        self.create_subscription(String, "/g1pilot/onboard_state",
                                 lambda m: self._latched_json("onboard", m), qos_latched)
        self.create_subscription(String, "/g1pilot/ps4/status",
                                 lambda m: self._latched_json("ps4", m), qos_latched)
        # Gehen am Controller (ps4_walk: L2 gehalten, Geh-Tempo) und welcher
        # Controller verbunden ist (ps4_joystick). Beide nur bei Aenderung.
        self.ps4walk = self.ps4walk_t = None
        self.ps4dev = self.ps4dev_t = None
        self.create_subscription(String, "/g1pilot/ps4/walk_status",
                                 lambda m: self._latched_json("ps4walk", m), qos_latched)
        self.create_subscription(String, DEVICE_TOPIC,
                                 lambda m: self._latched_json("ps4dev", m), qos_latched)
        # Gemeinsamer Bedienzustand mit dem Controller (shared_state.py).
        # on_shared(Aenderungen) wird fuer Wechsel von aussen gerufen.
        self.t_start_wall = time.time()
        self.shared = shared_state.SharedState("demo_gui")
        self.on_shared = None
        self.pub_shared = self.create_publisher(String, shared_state.TOPIC, qos_latched)
        self.create_subscription(String, shared_state.TOPIC, self._on_shared, qos_latched)
        # Stations-Ziele aus der Szene (station_<Name>, siehe scene_bridge.py):
        # name -> (x, y, yaw). on_stations wird nur bei Aenderung gerufen.
        self.stations = {}
        self.on_stations = None
        self.pub_goal = self.create_publisher(PoseStamped, "/g1pilot/goal", 10)
        qos_scene = QoSProfile(depth=1)
        qos_scene.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(MarkerArray, "/scene_markers", self._scene_markers, qos_scene)
        # Fortschritt der Navigation (nav2point): idle | moving | arrived | no_path.
        # TRANSIENT_LOCAL wie bei nav2point -> letzter Stand auch nach GUI-Start.
        self.nav_status = "idle"
        self.on_nav_status = None
        self.create_subscription(String, "/g1pilot/nav_status", self._nav_status, qos_scene)
        # Hand bewegen: aktuelle Hand-Pose per TF, Ziele wie der RViz-Marker.
        # /tf kommt mit hoher Rate -> eigener Node mit eigenem Spin-Thread, damit
        # die GUI-Schleife (spin_once im Qt-Timer) nicht von TF-Nachrichten
        # verstopft wird. Der Buffer ist thread-sicher.
        self.tf_node = rclpy.create_node("demo_gui_tf")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)
        self.pub_hand_goal = {
            side: self.create_publisher(PoseStamped, f"/g1pilot/hand_goal/{side}", 10)
            for side in ("left", "right")}
        # Jog-Bezug wie am PS4-Controller (hand_jog.JogReference, Env
        # G1_JOG_REFERENCE): Default Start, Vorlauf und Anhalten an der
        # kommandierten Hand (sim + real), "tf" = gemessene Hand wie frueher.
        self.jog_ref = JogReference(jog_reference_mode())
        self.jog_from_command = self.jog_ref.mode != "tf"
        self.cmd_poses = CommandedPoses()
        if self.jog_from_command:
            for side in ("left", "right"):
                self.create_subscription(
                    PoseStamped, JOG_CMD_TOPIC.format(side=side),
                    lambda m, s=side: self._on_cmd_pose(s, m), 10)

    def _on_cmd_pose(self, side, msg):
        if msg.header.frame_id and msg.header.frame_id != JOG_FRAME:
            return
        p, q = msg.pose.position, msg.pose.orientation
        self.cmd_poses.update(side, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), time.monotonic())

    def jog_pose(self, side, new):
        """-> (start, ref) fuer einen »Hand bewegen«-Schritt (JogReference):
        Startpose eines neuen Ziels (None = an ref) und Bezugshand fuer
        Vorlauf/Anhalten (None = unbekannt)."""
        cmd = self.cmd_poses.get(side, time.monotonic()) if self.jog_from_command else None
        return self.jog_ref.pose(side, cmd, self.hand_pose(side), new)

    def jog_end(self, side):
        self.jog_ref.end(side)

    def hand_pose(self, side):
        """-> ([x, y, z], (qx, qy, qz, qw)) der Hand im JOG_FRAME oder None (kein TF)."""
        try:
            t = self.tf_buffer.lookup_transform(JOG_FRAME, JOG_TF[side], rclpy.time.Time())
        except Exception:   # noqa: BLE001 -- Lookup/Connectivity/Extrapolation
            return None
        tr, r = t.transform.translation, t.transform.rotation
        return [tr.x, tr.y, tr.z], (r.x, r.y, r.z, r.w)

    def publish_hand_goal(self, side, pos, rot):
        msg = PoseStamped()
        msg.header.frame_id = JOG_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pos)
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = map(float, rot)
        self.pub_hand_goal[side].publish(msg)

    def _nav_status(self, msg: String):
        self.nav_status = msg.data
        if self.on_nav_status:
            self.on_nav_status(msg.data)

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

    def share(self, field, value, stamp=None):
        """Eigene Einstellung in den gemeinsamen Zustand (Controller zieht nach)."""
        if self.shared.set(field, value, stamp):
            self.pub_shared.publish(String(data=self.shared.to_json()))

    def _on_shared(self, msg: String):
        ch = self.shared.merge(msg.data)
        if ch and self.on_shared:
            self.on_shared(ch)

    def _latched_json(self, key, msg: String):
        data = status_line.parse(msg.data)
        if data is not None:
            setattr(self, key, data)
            setattr(self, key + "_t", time.monotonic())

    def _temperatures(self, msg: String):
        data = g1t.parse_payload(msg.data)
        if data is not None:
            self.temps, self.temps_t = data, time.monotonic()

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
class TempPanel(QFrame):
    """Real: Temperaturen des G1. Links der heisseste Motor, daneben je Gruppe
    (Bein L/R, Taille, Arm L/R) das Maximum, farbig nach Schwelle. Klick =
    Tabelle aller 29 Motoren (beide Sensoren) + IMU."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.warn, self.crit = g1t.thresholds()
        self.setObjectName("temp")
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setStyleSheet("QFrame#temp { background:#1b1b1b; border:1px solid #333;"
                           " border-radius:8px; } QLabel { background:transparent; }")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 6, 10, 6)
        lay.setSpacing(8)
        cap = QLabel("TEMP.\nMOTOREN")
        cap.setStyleSheet("color:#777; font-size:11px; font-weight:700;")
        lay.addWidget(cap)
        self.tiles = {}
        for key, title in (("max", "Max"),) + tuple((k, t) for k, t, _ in g1t.GROUPS):
            tile = QLabel()
            tile.setAlignment(Qt.AlignmentFlag.AlignCenter)
            tile.setMinimumWidth(96 if key == "max" else 78)
            tile.setFixedHeight(56)
            lay.addWidget(tile)
            self.tiles[key] = tile
            self._paint(tile, title, "–", "", "none")
        self.dialog = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)
        self.refresh()

    @staticmethod
    def _paint(tile, title, value, sub, lvl):
        tile.setText(f"<div style='font-size:10px; color:#ddd'>{title}</div>"
                     f"<div style='font-size:17px; font-weight:800'>{value}</div>"
                     f"<div style='font-size:9px; color:#eee'>{sub}</div>")
        tile.setStyleSheet(f"background:{g1t.LEVEL_COLOR[lvl]}; color:white;"
                           " border-radius:6px;")

    def refresh(self):
        data = self.node.temps
        if data is None or g1t.is_stale(self.node.temps_t, time.monotonic()):
            for key, tile in self.tiles.items():
                title = "Max" if key == "max" else dict((k, t) for k, t, _ in g1t.GROUPS)[key]
                self._paint(tile, title, "–", "keine Daten", "none")
                tile.setToolTip("Keine Temperaturdaten vom Roboter (robot_state).")
            return
        idx, top = g1t.hottest(data)
        self._paint(self.tiles["max"], "Max", f"{top:.0f} °C", g1t.motor_label(idx),
                    g1t.level(top, self.warn, self.crit))
        self.tiles["max"].setToolTip(f"Heissester Motor: {g1t.motor_label(idx)}, {top:.0f} °C")
        for g in g1t.group_summary(data):
            tile = self.tiles[g["key"]]
            self._paint(tile, g["title"], f"{g['max']:.0f} °C", g["label"],
                        g1t.level(g["max"], self.warn, self.crit))
            tile.setToolTip("\n".join(
                f"{g1t.motor_label(i)}: {t:.0f} °C" for i, t in g["temps"].items()))
        if self.dialog is not None and self.dialog.isVisible():
            self._fill_dialog(data)

    # ── Detailtabelle ───────────────────────────────────────────────
    def mousePressEvent(self, e):
        data = self.node.temps
        if data is None:
            return
        if self.dialog is None:
            self.dialog = QDialog(self)
            self.dialog.setWindowTitle("Motortemperaturen")
            self.dialog.setStyleSheet("QDialog { background:#111; } QLabel { color:#ddd; }"
                                      " QTableWidget { background:#1b1b1b; color:white;"
                                      " gridline-color:#333; }"
                                      " QHeaderView::section { background:#222; color:#aaa; }")
            v = QVBoxLayout(self.dialog)
            self.table = QTableWidget(g1t.N_MOTORS + 1, 4)
            self.table.setHorizontalHeaderLabels(["Motor", "Sensor 1 [°C]", "Sensor 2 [°C]", "Spannung [V]"])
            self.table.verticalHeader().setVisible(False)
            self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
            self.table.setColumnWidth(0, 200)
            for c in (1, 2, 3):
                self.table.setColumnWidth(c, 110)
            v.addWidget(self.table)
            v.addWidget(QLabel(f"Gelb ab {self.warn:.0f} °C, rot ab {self.crit:.0f} °C "
                               "(Annahme, anpassbar: G1_TEMP_WARN / G1_TEMP_CRIT)."))
            self.dialog.resize(600, 760)
        self._fill_dialog(data)
        self.dialog.show()
        self.dialog.raise_()

    def _fill_dialog(self, data):
        def cell(r, c, text, lvl=None):
            it = QTableWidgetItem(text)
            if lvl:
                it.setBackground(QColor(g1t.LEVEL_COLOR[lvl]))
            self.table.setItem(r, c, it)
        for i in range(g1t.N_MOTORS):
            cell(i, 0, g1t.motor_label(i))
            for c in (0, 1):
                t = data["t"][i][c]
                cell(i, c + 1, f"{t:.0f}", g1t.level(t, self.warn, self.crit))
            cell(i, 3, f"{data['v'][i]:.1f}" if i < len(data["v"]) else "–")
        r = g1t.N_MOTORS
        cell(r, 0, "IMU")
        cell(r, 1, f"{data['imu']:.0f}", g1t.level(data["imu"], self.warn, self.crit))
        cell(r, 2, "–")
        cell(r, 3, "–")


class ModeTile(QPushButton):
    """Grosse Kachel mit drei sichtbaren Zustaenden: aktiv / wechselt / inaktiv."""

    def __init__(self, mode):
        super().__init__()
        self.mode = mode
        self.hint = None        # Untertitel statt »antippen zum Wechseln« (inaktiv)
        self.state = "inactive"
        self.setMinimumHeight(100)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.set_state("inactive")

    def set_hint(self, hint):
        """Vorab-Hinweis (real, GEHEN): z. B. »gesperrt · kein Bodenkontakt«."""
        if hint != self.hint:
            self.hint = hint
            self.set_state(self.state)

    def set_state(self, state):
        self.state = state
        color = MODE_COLOR[self.mode]
        title = MODE_TITLE[self.mode]
        if state == "active":
            sub, bg, fg, border = "● AKTIV", color, "white", "4px solid white"
        elif state == "pending":
            sub, bg, fg, border = "wechselt …", "#2a2a2a", color, f"4px dashed {color}"
        else:
            sub, bg, fg, border = "antippen zum Wechseln", "#1c1c1c", "#777", "2px solid #333"
            if self.hint:
                sub = self.hint
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


def state_css(bg, fg, border, font):
    """Knopf-Stil fuer Zustaende (z.B. gestrichelt = laeuft, wie ModeTile)."""
    return f"""
        QPushButton {{ background:{bg}; color:{fg}; font-size:{font}px; font-weight:700;
                       border:{border}; border-radius:14px; padding:8px; }}
        QPushButton:disabled {{ background:#1c1c1c; color:#555; border:1px solid #2a2a2a; }}
    """


class VerticalRocker(QWidget):
    """Senkrechter Schieber, der zurueckfedert (Gegenstueck zum VirtualJoystick):
    nach oben ziehen -> value = +1 (hoch), nach unten -> -1, loslassen -> 0."""

    def __init__(self, height=190, width=74, labels=("▲", "▼")):
        super().__init__()
        self.setFixedSize(width, height)
        self._labels = labels
        self._travel = height / 2 - 24     # Knopf-Weg je Richtung (Pixel)
        self._knob = 0.0                   # Versatz vom Zentrum, + = nach unten
        self.value = 0.0

    def _set_from_y(self, y):
        self._knob = max(-self._travel, min(self._travel, y - self.height() / 2))
        self.value = -self._knob / self._travel
        self.update()

    def mousePressEvent(self, e):
        self._set_from_y(e.position().y())

    def mouseMoveEvent(self, e):
        self._set_from_y(e.position().y())

    def mouseReleaseEvent(self, e):
        self._knob = 0.0
        self.value = 0.0
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        p.setBrush(QBrush(QColor("#1e1e1e")))
        p.setPen(QPen(QColor("#444"), 2))
        p.drawRoundedRect(QRectF(cx - 26, 4, 52, h - 8), 26, 26)
        p.setPen(QPen(QColor("#333"), 1))
        p.drawLine(int(cx - 18), int(cy), int(cx + 18), int(cy))
        p.setPen(QPen(QColor("#666"), 1))
        p.drawText(QRectF(0, 8, w, 20), Qt.AlignmentFlag.AlignCenter, self._labels[0])
        p.drawText(QRectF(0, h - 28, w, 20), Qt.AlignmentFlag.AlignCenter, self._labels[1])
        active = abs(self.value) > 1e-3
        p.setBrush(QBrush(QColor("#4CAF50") if active else QColor("#3c3c3c")))
        p.setPen(QPen(QColor("#80ff80") if active else QColor("#666"), 2))
        p.drawEllipse(QPointF(cx, cy + self._knob), 15, 15)


class ArmJogPanel(QWidget):
    """Hand bewegen, zwei Gruppen (beide gleichzeitig nutzbar), Richtungen aus
    Sicht des Roboters (JOG_FRAME), Drehung um die Hand selbst:
      VERSCHIEBEN  Joystick = vor/zurueck + links/rechts, Schieber = hoch/runter
      DREHEN       Joystick = kippen (Finger hoch/runter) + schwenken
                   (links/rechts), Schieber = rollen (um die Vorwaertsachse)
    Gedrueckt halten = Hand faehrt, loslassen = Hand steht.
    twist() -> ((vx, vy, vz) m/s, (wx, wy, wz) rad/s) im JOG_FRAME."""

    STICK = 180

    def __init__(self):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(18)

        opts = QVBoxLayout()
        opts.setSpacing(6)
        opts.addWidget(WalkPanel._caption("Arm"))
        # on_change(feld, wert): Auswahl in der GUI geaendert -> gemeinsamer Zustand
        self.on_change = None
        self.side_btns = self._choice_row(opts, list(JOG_SIDES)[:2], self.set_side)
        self.side_btns.update(self._choice_row(opts, list(JOG_SIDES)[2:], self.set_side))
        opts.addSpacing(8)
        opts.addWidget(WalkPanel._caption("Tempo"))
        self.speed_btns = self._choice_row(opts, JOG_SPEEDS, self.set_speed)
        opts.addStretch(1)
        lay.addLayout(opts)

        self.move_stick, self.move_rocker = self._group(
            lay, "VERSCHIEBEN", "vor / zurück · links / rechts", "hoch / runter", ("▲", "▼"))
        self.turn_stick, self.turn_rocker = self._group(
            lay, "DREHEN", "kippen · schwenken", "rollen", ("⟲", "⟳"))
        lay.addStretch(1)

        self.set_side("Rechts")
        self.set_speed("Normal")

    def _group(self, parent, title, stick_cap, rocker_cap, rocker_labels):
        box = QVBoxLayout()
        box.setSpacing(4)
        head = QLabel(title)
        head.setStyleSheet(f"color:{MODE_COLOR[MANIP]}; font-size:13px; font-weight:800;"
                           "letter-spacing:1px;")
        box.addWidget(head)
        row = QHBoxLayout()
        row.setSpacing(12)
        col = QVBoxLayout()
        col.addWidget(WalkPanel._caption(stick_cap), alignment=Qt.AlignmentFlag.AlignCenter)
        stick = VirtualJoystick(self.STICK)
        col.addWidget(stick, alignment=Qt.AlignmentFlag.AlignCenter)
        row.addLayout(col)
        col = QVBoxLayout()
        col.addWidget(WalkPanel._caption(rocker_cap), alignment=Qt.AlignmentFlag.AlignCenter)
        rocker = VerticalRocker(self.STICK, labels=rocker_labels)
        col.addWidget(rocker, alignment=Qt.AlignmentFlag.AlignCenter)
        row.addLayout(col)
        box.addLayout(row)
        parent.addLayout(box)
        return stick, rocker

    @staticmethod
    def _choice_row(parent, choices, on_pick):
        row = QHBoxLayout()
        row.setSpacing(6)
        btns = {}
        for name in choices:
            b = big_button(name, font=15, height=44)
            b.setCheckable(True)
            b.setFixedWidth(118)
            b.clicked.connect(lambda _, n=name: on_pick(n))
            row.addWidget(b)
            btns[name] = b
        row.addStretch(1)
        parent.addLayout(row)
        return btns

    def set_side(self, name, notify=True):
        self.side = name
        for n, b in self.side_btns.items():
            b.setChecked(n == name)
        if notify and self.on_change:
            self.on_change("arm_side", JOG_SIDE_KEY[name])

    def set_speed(self, name, notify=True):
        self.speed_name = name
        self.lin_speed, self.ang_speed = JOG_SPEEDS[name]
        for n, b in self.speed_btns.items():
            b.setChecked(n == name)
        if notify and self.on_change:
            self.on_change("arm_speed", name)

    def sides(self):
        return JOG_SIDES[self.side]

    def side_twist(self, side, lin, ang):
        """Sollbewegung je Hand: bei »Gespiegelt« die linke spiegelbildlich."""
        if self.side == "Gespiegelt" and side == "left":
            return mirror_twist(lin, ang)
        return lin, ang

    def twist(self):
        m, t = self.move_stick, self.turn_stick
        lin = (m.vx * self.lin_speed, m.vy * self.lin_speed,
               self.move_rocker.value * self.lin_speed)
        # Joystick oben = Finger hoch: das ist eine NEGATIVE Drehung um y
        # (positive dreht x nach -z). Links = positive Drehung um z.
        ang = (self.turn_rocker.value * self.ang_speed,
               -t.vx * self.ang_speed,
               t.vy * self.ang_speed)
        return lin, ang

    def reset(self):
        for w in (self.move_stick, self.move_rocker, self.turn_stick, self.turn_rocker):
            w.mouseReleaseEvent(None)


# ── Bereich 2a: Gehen ────────────────────────────────────────────────────
class WalkPanel(QWidget):
    """Joystick-Knopf ODER Pfeiltasten (gedrueckt halten = laufen, loslassen =
    stehen). Ausgabe: velocity() -> (vx, vy, yaw), bereits mit Stufe skaliert."""

    def __init__(self):
        super().__init__()
        self.held = set()        # gerade gehaltene Pfeiltasten
        self.speed = SPEEDS["Langsam"]
        self.speed_name = "Langsam"
        self.on_change = None    # on_change("walk_speed", stufe) -> gemeinsamer Zustand

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
        for name in shared_state.SPEED_ORDER:
            gui = name in SPEEDS
            b = big_button(name if gui else f"{name}\n(Controller)", font=20 if gui else 15,
                           height=70)
            b.setCheckable(True)
            if gui:
                b.clicked.connect(lambda _, n=name: self.set_speed(n))
            else:
                # Nur Anzeige: am Controller gewaehlt (L2 + ▲), in der GUI gesperrt.
                b.setEnabled(False)
                b.setToolTip("Nur am Controller wählbar (L2 halten + ▲). "
                             f"Die GUI selbst geht dann mit »{WALK_GUI_MAX}«.")
                b.setStyleSheet(b.styleSheet() + """
                    QPushButton:checked:disabled { background:#1b3a1c; color:#ccc;
                                                   border:2px dashed #80ff80; }""")
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
        self._auto_nav_css = self.btn_auto_nav.styleSheet()
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
        self.station_by_name = {}
        self.nav_station = None   # Station, zu der gerade gelaufen wird
        self.on_station = None
        self.station_row.addStretch(1)
        outer.addLayout(self.station_row)
        self.set_stations([])
        self.set_nav_view(NAV_OFF)

    @staticmethod
    def _caption(text):
        lbl = QLabel(text)
        lbl.setStyleSheet("color:#aaa; font-size:15px;")
        return lbl

    def set_stations(self, names):
        for b in self.station_btns:
            self.station_row.removeWidget(b)
            b.hide()            # deleteLater allein laesst es bis zum Loeschen sichtbar
            b.deleteLater()
        self.station_btns = []
        self.station_by_name = {}
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
            b.default_css = b.styleSheet()
            self.station_row.insertWidget(1 + i, b)
            self.station_btns.append(b)
            self.station_by_name[name] = b
        self._style_stations()

    def _style_stations(self):
        """Station, zu der gerade gelaufen wird: gestrichelt (wie »wechselt«)."""
        for name, b in self.station_by_name.items():
            if name == self.nav_station:
                b.setStyleSheet(state_css("#2a2a2a", NAV_DONE_COLOR,
                                          f"4px dashed {NAV_DONE_COLOR}", 15))
            else:
                b.setStyleSheet(b.default_css)

    def set_speed(self, name, notify=True):
        """Stufe Langsam/Normal/Schnell; die GUI selbst geht hoechstens WALK_GUI_MAX."""
        if name not in self.speed_btns:
            return
        self.speed_name = name
        self.speed = SPEEDS[shared_state.capped(name, WALK_GUI_MAX)]
        for n, b in self.speed_btns.items():
            b.setChecked(n == name)
        if notify and self.on_change:
            self.on_change("walk_speed", name)

    def velocity(self):
        h = self.held
        vx = self.joystick.vx + (1 if "fwd" in h else 0) - (1 if "back" in h else 0)
        vy = self.joystick.vy + (1 if "left" in h else 0) - (1 if "right" in h else 0)
        yaw = (1 if "turn_l" in h else 0) - (1 if "turn_r" in h else 0)
        clamp = lambda v: max(-1.0, min(1.0, v))   # noqa: E731
        return clamp(vx) * self.speed, clamp(vy) * self.speed, clamp(yaw) * self.speed

    def set_nav_view(self, phase, station=None):
        """Nur Anzeige: AUTO-NAV-Knopf + Hinweis je Phase, Stations-Knopf
        gestrichelt waehrend des Laufens, manuelle Steuerung sperren."""
        on = phase != NAV_OFF
        self.btn_auto_nav.setChecked(on)
        self.manual.setEnabled(not on)
        # Gesperrt sichtbar machen (der gezeichnete Joystick kennt kein :disabled).
        fade = QGraphicsOpacityEffect(self.manual)
        fade.setOpacity(0.3 if on else 1.0)
        self.manual.setGraphicsEffect(fade)
        self.nav_station = station if phase == NAV_MOVING else None
        self._style_stations()
        if on:
            self.reset()
        walk = MODE_COLOR[WALK]
        to = f"zur Station »{sm.station_label(station)}«" if station else "zum Ziel"
        if phase == NAV_MOVING:
            self.btn_auto_nav.setText("AUTO NAV  ● läuft …\nantippen zum Stoppen")
            self.btn_auto_nav.setStyleSheet(
                state_css("#2a2a2a", walk, f"4px dashed {walk}", 20))
            self.nav_hint.setText(f"Läuft selbstständig {to}. "
                                  "Manuelle Steuerung ist gesperrt.")
        elif phase == NAV_ARRIVED:
            self.btn_auto_nav.setText("AUTO NAV  ● AN\nZiel erreicht")
            self.btn_auto_nav.setStyleSheet(
                state_css("#2e7d32", "white", "4px solid #80ff80", 20))
            self.nav_hint.setText("Am Ziel angekommen. AUTO NAV bleibt an — nächstes "
                                  "Ziel in RViz setzen oder antippen zum Stoppen.")
        elif phase == NAV_WAIT:
            self.btn_auto_nav.setStyleSheet(self._auto_nav_css)
            self.btn_auto_nav.setText("AUTO NAV  ● AN\nwartet auf Ziel")
            self.nav_hint.setText("Ziel in RViz setzen (»2D Goal Pose«) oder Station "
                                  "wählen. Manuelle Steuerung ist gesperrt.")
        elif NAV_AVAILABLE:
            self.btn_auto_nav.setStyleSheet(self._auto_nav_css)
            self.btn_auto_nav.setText("AUTO NAV\nselbst zum Ziel laufen")
            self.nav_hint.setText("Erst Ziel in RViz setzen, dann AUTO NAV einschalten.")
        else:
            self.btn_auto_nav.setStyleSheet(self._auto_nav_css)
            self.btn_auto_nav.setText("AUTO NAV")
            self.nav_hint.setText("Navigation ist nicht gestartet — im Startmenü "
                                  "Ausstattung → »Navigation« aktivieren.")

    def reset(self):
        self.held.clear()
        self.joystick.mouseReleaseEvent(None)   # Knopf zentrieren -> vx=vy=0


# ── Bereich 2b: Greifen ──────────────────────────────────────────────────
class ManipPanel(QWidget):
    """Umschalter ARME | HAENDE, darunter jeweils eine Seite.
    ARME: Ablaeufe als grosse Knoepfe, Haende auf/zu, Grundstellung.
    Einzelfunktionen (Pose anfahren/speichern, Marker folgen) liegen
    eingeklappt unter 'Erweitert' -- fuer den Betreuer, nicht fuer Besucher.
    HAENDE: HandPanel (ganze Hand + umschaltbar einzelne Finger)."""

    def __init__(self, gui):
        super().__init__()
        self.gui = gui
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(8)

        # Umschalter ARME | HAENDE: eigenes Widget -> DemoGUI setzt es in die
        # Titelzeile neben »GREIFEN« (spart eine Zeile).
        self.view_switch = QWidget()
        self.view_switch.setFixedWidth(520)
        seg = QHBoxLayout(self.view_switch)
        seg.setContentsMargins(0, 0, 0, 0)
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

        self.seq_row = QHBoxLayout()
        self.seq_row.setSpacing(10)
        lay.addLayout(self.seq_row)

        row = QHBoxLayout()
        # Haende auf/zu gibt es auf der Seite HAENDE.
        self.btn_safe = big_button("Sichere Pose", font=18, height=50)
        self.btn_home = big_button("Grundstellung", font=18, height=50)
        self.btn_cancel = big_button("Bewegung stoppen", "#5d4037", font=18, height=50)
        for b in (self.btn_safe, self.btn_home, self.btn_cancel):
            row.addWidget(b)
        lay.addLayout(row)

        # Hand bewegen (Joystick + Hoch/Runter) -- statt Marker in RViz.
        lay.addSpacing(4)
        lay.addWidget(WalkPanel._caption("Hand bewegen  —  gedrückt halten, loslassen = stehen"))
        self.jog = ArmJogPanel()
        lay.addWidget(self.jog)
        self.arms_page = arms

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
            self.seq_row.addWidget(WalkPanel._caption(
                f"Noch keine Abläufe — Posen in einer Kategorie »{SEQUENCE_PREFIX}<Name>« "
                "speichern (Erweitert → Pose speichern)."))
            return
        self.seq_row.addWidget(WalkPanel._caption("Abläufe:"))
        for label, names in sequences:
            b = big_button(label, "#1565c0", font=17, height=64)
            b.setToolTip(" → ".join(pretty_pose_name(n) for n in names))
            b.clicked.connect(lambda _, ns=list(names), lb=label: self.gui.run_poses(ns, lb))
            self.seq_row.addWidget(b)


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
        self._estop_shown = False     # NOT-HALT angezeigt, noch nicht quittiert
        self.queue = []             # verbleibende Posen einer laufenden Sequenz
        self.current_pose = None
        self._walk_req = 0          # verwirft veraltete WALK-Timeouts
        self.auto_nav = False
        self.nav_phase = NAV_OFF
        self.nav_station = None     # Quickbefehl-Ziel (Station) oder None = RViz-Ziel
        self._nav_planning = False  # Station geklickt, Pfad noch nicht da
        self._nav_req = 0           # verwirft veraltete Planungs-Timeouts
        self._jog_targets = {}      # side -> [pos, rot]: laufendes Hand-Ziel
        self._jog_twist = {}        # side -> letzte Sollbewegung (lin, ang)
        self._jog_holds = HoldStreamer()   # Halte-Ziele nach dem Loslassen
        self._jog_t = time.monotonic()

        node.on_walk_ready = self._on_walk_ready
        node.on_arm_status = self._on_arm_status
        node.on_nav_status = self._on_nav_status
        node.on_emergency_stop = self._external_estop
        node.on_shared = self._on_shared

        self.setWindowTitle("G1 Demo — " + ("SIM" if self.sim_mode else "REAL"))
        self.setStyleSheet("QWidget { background:#111; }")
        self.resize(1200, 860)
        self._build()
        self._wire()
        self._refresh()
        # Startwerte der GUI nur als Vorschlag (DEFAULT_STAMP): laeuft der
        # Controller schon bzw. startet er, gelten seine Werte (real: Langsam).
        jog = self.manip_panel.jog
        for field, value in (("arm_side", JOG_SIDE_KEY[jog.side]),
                             ("arm_speed", jog.speed_name),
                             ("walk_speed", self.walk_panel.speed_name)):
            node.share(field, value, stamp=shared_state.DEFAULT_STAMP)

        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self._publish_cmd_vel)
        self.cmd_timer.start(33)
        self.jog_timer = QTimer(self)
        self.jog_timer.timeout.connect(self._jog_tick)
        self.jog_timer.start(33)
        self.info_timer = QTimer(self)
        self.info_timer.timeout.connect(self._refresh_info)
        self.info_timer.start(500)
        self._refresh_info()

        # Wie ui_interface: Auto-Start NUR in der Sim (Arme an + BALANCING),
        # wiederholt bis arm_controller und loco_sim quittieren (sim_autostart.py;
        # frueher EIN Versuch nach 3 s -> ging verloren, wenn die Regler noch
        # luden: GREIFEN angezeigt, Arme aber aus).
        self.autostart = SimAutoStart(
            lambda: node.publish_bool(node.pub_arms_enabled, True),
            lambda: node.publish_bool(node.pub_start_balancing, True),
            log=lambda t: node.get_logger().info(t),
            on_done=self._autostart_done)
        self.autostart_timer = QTimer(self)
        self.autostart_timer.timeout.connect(
            lambda: self.autostart.tick(node.arms_state, node.loco_state))
        self.autostart_timer.start(100)
        if self.sim_mode:
            QTimer.singleShot(3000, self._auto_start)
        else:
            self._status("REAL: zuerst »Start« in der Startleiste drücken.", "#ffb300")

    # ── Aufbau ──────────────────────────────────────────────────────────
    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 12, 20, 12)
        root.setSpacing(10)

        # Bereich 1: Modus. Rechts in der Kopfzeile: Controller-Belegung (nur
        # wenn ein Controller aktiv ist) und Vollbild.
        head1 = QHBoxLayout()
        head1.setSpacing(10)
        head1.addWidget(self._section_label("1  ·  MODUS"))
        head1.addStretch(1)
        self.btn_pad_help = self._small_button("🎮  Belegung")
        self.btn_pad_help.setVisible(False)
        head1.addWidget(self.btn_pad_help)
        # Vollbild immer aus der GUI erreichbar (am Roboter-Rechner fehlt oft die
        # Titelleiste des Fenstermanagers). Tasten: F11 umschalten, Esc verlassen.
        self.btn_fullscreen = self._small_button("")
        head1.addWidget(self.btn_fullscreen)
        root.addLayout(head1)
        top = QHBoxLayout()
        top.setSpacing(16)
        self.tiles = {WALK: ModeTile(WALK), MANIP: ModeTile(MANIP)}
        top.addWidget(self.tiles[WALK])
        top.addWidget(self.tiles[MANIP])
        root.addLayout(top)
        # Startleiste (nur real), eine Zeile: Start | Damp | Preparation | Walk |
        # Rueckmeldung des loco_client (Onboard-Modus, wie in der Handy-App).
        # Start = /g1pilot/start: gibt die Modus-Kacheln frei und quittiert
        # einen NOT-HALT (arm_controller, loco_client, Controller). Freigabe der
        # Onboard-Schritte je nach Onboard-Modus (status_line.start_buttons),
        # Damp aus dem Stehen und Walk brauchen einen zweiten Klick.
        self.start_bar = QWidget()
        bar = QHBoxLayout(self.start_bar)
        bar.setContentsMargins(0, 0, 0, 0)
        bar.setSpacing(10)
        self.btn_start = big_button("Start", "#e65100", font=18, height=50)
        bar.addWidget(self.btn_start, 1)
        self.start_btns = {}
        for step in status_line.START_STEPS:
            b = big_button(status_line.START_LABEL[step], "#37474f", font=18, height=50)
            b.clicked.connect(lambda _=False, s=step: self._start_step(s))
            bar.addWidget(b, 1)
            self.start_btns[step] = b
        self._start_armed = None      # Schritt, der auf den zweiten Klick wartet
        self._start_armed_n = 0
        self._last_note = None
        self.info_onboard = QLabel()
        self.info_onboard.setWordWrap(True)
        self.info_onboard.setSizePolicy(QSizePolicy.Policy.Expanding,
                                        QSizePolicy.Policy.Preferred)
        self.info_onboard.setVisible(not self.sim_mode)
        bar.addWidget(self.info_onboard, 3)
        self.start_bar.setVisible(not self.sim_mode)
        root.addWidget(self.start_bar)
        self._update_start_button()

        # Bereich 2: Steuerung des aktiven Modus
        root.addWidget(self._section_label("2  ·  STEUERUNG"))
        self.panel_frame = QFrame()
        self.panel_frame.setObjectName("panel")
        pf = QVBoxLayout(self.panel_frame)
        pf.setContentsMargins(18, 10, 18, 10)
        self.walk_panel = WalkPanel()
        self.manip_panel = ManipPanel(self)
        # Titelzeile: Modus-Titel + (nur GREIFEN) Umschalter ARME | HAENDE,
        # zentriert im Platz rechts vom Titel. Darunter eine Trennlinie.
        head = QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        self.panel_title = QLabel()
        head.addWidget(self.panel_title)
        head.addStretch(1)
        head.addWidget(self.manip_panel.view_switch)
        head.addStretch(1)
        pf.addLayout(head)
        self.panel_rule = QFrame()
        self.panel_rule.setFixedHeight(2)
        pf.addSpacing(4)
        pf.addWidget(self.panel_rule)
        pf.addSpacing(6)
        self.stack = QStackedWidget()
        self.idle_panel = QLabel("Bitte oben einen Modus wählen.")
        self.idle_panel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.idle_panel.setStyleSheet("color:#666; font-size:22px;")
        for w in (self.idle_panel, self.walk_panel, self.manip_panel):
            self.stack.addWidget(w)
        # Scrollbereich als Reserve: die Mindesthoehe des Fensters haengt so
        # nicht mehr am groessten Panel. Frueher lag sie real bei ~1160 px --
        # hoeher als ein 1080er-Schirm, darum bot der Fenstermanager kein
        # Maximieren an und auch Vollbild blieb groesser als der Schirm.
        self.panel_scroll = QScrollArea()
        self.panel_scroll.setWidgetResizable(True)
        self.panel_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.panel_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.panel_scroll.setWidget(self.stack)
        pf.addWidget(self.panel_scroll, 1)
        root.addWidget(self.panel_frame, 1)

        # Bereich 3: Status + Sicherheit
        bottom = QHBoxLayout()
        bottom.setSpacing(14)
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(48)
        # Real: Temperaturen des G1 (links; in der Sim ausgeblendet, kein Messwert).
        self.temp_panel = TempPanel(self.node)
        self.temp_panel.setVisible(not self.sim_mode)
        if self.sim_mode:
            self.temp_panel.timer.stop()
        bottom.addWidget(self.temp_panel)
        bottom.addStretch(1)
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
        # Ganzen Stack beenden (alle Fenster zu). Zweiter Klick bestaetigt.
        # Real: vorher Laufen stoppen und Arme sicher abgeben (_quit_real_begin).
        self._quit_label = "Sim\nbeenden" if self.sim_mode else "Real\nbeenden"
        self.btn_quit = big_button(self._quit_label, "#37474f", font=15, height=70)
        self.btn_quit.setFixedWidth(170)
        self._quit_css = self.btn_quit.styleSheet()
        self._quit_armed = 0        # >0: erster Klick erfolgt (Zaehler gegen alte Timer)
        self._real_quit = 0         # >0: »Real beenden« laeuft (Zaehler gegen alte Timer)
        bottom.addWidget(self.btn_quit)
        self.btn_estop = QPushButton("NOT-HALT")
        self.btn_estop.setFixedSize(220, 90)
        self.btn_estop.setStyleSheet("""
            QPushButton { background:#c62828; color:white; font-size:26px; font-weight:900;
                          border:3px solid #ff8a80; border-radius:45px; }
            QPushButton:pressed { background:#ff1744; }
        """)
        bottom.addWidget(self.btn_estop)
        root.addWidget(self._section_label("3  ·  STATUS"))
        # Status ueber volle Breite (lange Meldungen), Knoepfe darunter.
        root.addWidget(self.status)
        root.addLayout(bottom)
        self._status("Startet …", "#aaa")

    # ── Vollbild ────────────────────────────────────────────────────────
    def set_fullscreen(self, on):
        """Vollbild an/aus; aus kehrt zum vorigen Zustand zurueck (maximiert
        bzw. Fenster)."""
        if on == self.isFullScreen():
            return
        if on:
            self._max_before_fs = self.isMaximized()
            self.showFullScreen()
        elif getattr(self, "_max_before_fs", False):
            self.showMaximized()
        else:
            self.showNormal()
        self._update_fullscreen_button()

    def toggle_fullscreen(self):
        self.set_fullscreen(not self.isFullScreen())

    def _update_fullscreen_button(self):
        if self.isFullScreen():
            self.btn_fullscreen.setText("✕  Vollbild beenden")
            self.btn_fullscreen.setToolTip("Vollbild verlassen (Esc oder F11)")
        else:
            self.btn_fullscreen.setText("⛶  Vollbild")
            self.btn_fullscreen.setToolTip("Vollbild (F11), verlassen mit Esc")

    def changeEvent(self, event):
        # Auch wenn der Fenstermanager das Vollbild umschaltet, Text nachziehen.
        if event.type() == QEvent.Type.WindowStateChange:
            self._update_fullscreen_button()
        super().changeEvent(event)

    @staticmethod
    def _info_css(color):
        return (f"color:{color}; font-size:16px; font-weight:700; padding:4px 10px;"
                f"background:#1a1a1a; border-left:5px solid {color}; border-radius:6px;")

    def _refresh_info(self):
        """Info-Zeile aus den zuletzt empfangenen Zustaenden (2 Hz)."""
        n, now = self.node, time.monotonic()
        if not self.sim_mode:
            age = None if n.onboard_t is None else now - n.onboard_t
            walk_req = WALK in (self.mode, self.pending)
            text, color = status_line.onboard_text(n.onboard, age, walk_requested=walk_req)
            self.info_onboard.setText(text)
            self.info_onboard.setStyleSheet(self._info_css(color))
            self._refresh_start_bar()
            self._refresh_walk_hint()
        age = None if n.ps4_t is None else now - n.ps4_t
        # Controller: Zustand (Modus, Hand, Tempo) zeigt die GUI selbst; hier
        # nur der Knopf zur Belegung, solange ein Controller aktiv ist.
        res = status_line.ps4_text(n.ps4, age, sim=self.sim_mode, walk=n.ps4walk,
                                   device=n.ps4dev)
        self.btn_pad_help.setVisible(res is not None)
        if res is not None:
            name = (n.ps4dev or {}).get("name")
            label = status_line.controller_label(n.ps4dev)
            self.btn_pad_help.setToolTip((f"{name}\n" if name else "")
                                         + f"Belegung des {label}-Controllers")

    def _refresh_start_bar(self):
        n, now = self.node, time.monotonic()
        age = None if n.onboard_t is None else now - n.onboard_t
        allowed = status_line.start_buttons(n.onboard, age)
        if self._start_armed and not allowed[self._start_armed][0]:
            self._start_disarm()
        for step, b in self.start_btns.items():
            b.setEnabled(allowed[step][0])
            if step != self._start_armed:
                b.setText(self._start_label(step))
        note = (n.onboard or {}).get("note") if age is not None else None
        if note and note != self._last_note and self._last_note is not None:
            self._status(note, "#ef5350" if "abgelehnt" in note else "#aaa")
        if note:
            self._last_note = note
        elif self._last_note is None:
            self._last_note = ""

    def _start_step(self, step):
        """Startleiste: Anforderung an den loco_client (der prueft erneut)."""
        n, now = self.node, time.monotonic()
        age = None if n.onboard_t is None else now - n.onboard_t
        enabled, confirm = status_line.start_buttons(n.onboard, age)[step]
        if not enabled:
            return
        if confirm and self._start_armed != step:
            self._start_disarm()
            self._start_armed = step
            self._start_armed_n += 1
            armed = self._start_armed_n
            b = self.start_btns[step]
            if step != "walk":
                b.setText("Wirklich Damp?\nnochmal tippen")
            elif status_line.walk_switch(n.onboard):
                b.setText("Walk-Modus wechseln?\nnochmal tippen")
            else:
                b.setText("Kran gelockert?\nnochmal tippen")
            QTimer.singleShot(QUIT_CONFIRM_MS,
                              lambda: armed == self._start_armed_n and self._start_disarm())
            return
        self._start_disarm()
        if step != "walk":
            # Damp/Preparation beenden einen GUI-Modus (Gehen/Greifen).
            self._cancel_sequence()
            self.set_auto_nav(False)
            self.walk_panel.reset()
            if self.mode == WALK or self.pending == WALK:
                n.publish_bool(n.pub_start_walking, False)
                self.mode = self.pending = None
                self._refresh()
        n.pub_loco_request.publish(String(data=step))
        self._status(f"{status_line.START_LABEL[step]} angefordert …", "#aaa")

    def _start_label(self, step):
        """»Walk« zeigt die Bodenkontakt-Vorabpruefung schon vor dem Klick."""
        if step != "walk":
            return status_line.START_LABEL[step]
        n = self.node
        age = None if n.onboard_t is None else time.monotonic() - n.onboard_t
        return status_line.walk_button_label(n.onboard, age)

    def _refresh_walk_hint(self):
        """Kachel GEHEN (real): vorab zeigen, ob GEHEN jetzt geht."""
        if self.sim_mode:
            return
        n = self.node
        age = None if n.onboard_t is None else time.monotonic() - n.onboard_t
        self.tiles[WALK].set_hint(status_line.walk_tile_hint(n.onboard, age))

    def _start_disarm(self):
        if self._start_armed:
            self.start_btns[self._start_armed].setText(self._start_label(self._start_armed))
        self._start_armed = None
        self._start_armed_n += 1

    def _ps4_help(self):
        rows = "".join(f"<tr><td style='padding:3px 14px 3px 0'><b>{k}</b></td><td>{v}</td></tr>"
                       for k, v in status_line.ps4_help(self.node.ps4))
        label = status_line.controller_label(self.node.ps4dev)
        box = QMessageBox(self)
        box.setWindowTitle(f"{label}-Controller: Belegung")
        box.setTextFormat(Qt.TextFormat.RichText)
        note = ("<p>Xbox: Tasten nach Position wie PS4 (A = Kreuz, B = Kreis, X = Viereck, "
                "Y = Dreieck, LB/RB = L1/R1, LT/RT = L2/R2, View = Share, Menu = Options, "
                "Xbox-Taste = PS).</p>" if label == "Xbox" else "")
        box.setText(f"<table>{rows}</table>{note}<p>Modus, Hand-Auswahl und Tempo sind "
                    "mit dieser GUI gekoppelt: was am Controller gewählt wird, zeigt die "
                    "GUI und umgekehrt.</p>")
        box.open()

    @staticmethod
    def _small_button(text):
        """Flacher Knopf fuer die Kopfzeile (Vollbild, Controller-Belegung)."""
        b = QPushButton(text)
        b.setFixedHeight(30)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.setStyleSheet("""
            QPushButton { background:#263238; color:#ddd; font-size:14px; font-weight:700;
                          border:1px solid #555; border-radius:8px; padding:2px 12px; }
            QPushButton:hover { border:2px solid #aaa; }
            QPushButton:pressed { background:#555; }
        """)
        return b

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
        # Halten = Schubs aufladen (Ladegrad im Button), Loslassen = schubsen.
        self._push_charger = PushCharger()
        self._push_timer = QTimer(self)
        self._push_timer.setInterval(50)
        self._push_timer.timeout.connect(self._push_refresh)
        self.btn_push.pressed.connect(self._push_press)
        self.btn_push.released.connect(self._push_release)
        self.btn_reset_robot.clicked.connect(self._reset_robot)
        self.btn_quit.clicked.connect(self._quit_clicked)
        self.btn_fullscreen.clicked.connect(self.toggle_fullscreen)
        self.btn_pad_help.clicked.connect(self._ps4_help)
        self._sc_fullscreen = QShortcut(QKeySequence(Qt.Key.Key_F11), self)
        self._sc_fullscreen.activated.connect(self.toggle_fullscreen)
        self._sc_leave_fullscreen = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        self._sc_leave_fullscreen.activated.connect(lambda: self.set_fullscreen(False))
        self._update_fullscreen_button()
        self.walk_panel.on_station = self.go_to_station
        n.on_stations = self.walk_panel.set_stations
        if n.stations:
            self.walk_panel.set_stations(sorted(n.stations))

        mp.btn_safe.clicked.connect(self._safe_pose)
        n.on_hand_status = mp.hand_panel.on_status
        mp.btn_home.clicked.connect(self._home)
        mp.btn_cancel.clicked.connect(self._cancel)
        mp.btn_goto.clicked.connect(self._pose_goto_dialog)
        mp.btn_save.clicked.connect(self._pose_save_dialog)
        mp.btn_marker.toggled.connect(lambda on: n.publish_bool(n.pub_marker_follow, on))
        # Einstellungen gehen in den gemeinsamen Zustand (Controller zieht nach).
        mp.jog.on_change = n.share
        self.walk_panel.on_change = n.share

    # ── Modus ───────────────────────────────────────────────────────────
    def _start_robot(self):
        """REAL: START (Standby). Danach sind die Modus-Kacheln freigegeben."""
        if self.started:
            return
        self.node.publish_bool(self.node.pub_start, True)
        self.started = True
        self._update_start_button()
        self._estop_unacked = False
        self._estop_shown = False
        self._status("Gestartet. Jetzt einen Modus wählen.", "#aaa")
        self._refresh()

    def _update_start_button(self):
        """Start-Knopf in der Startleiste: orange und frei, solange START fehlt
        (nach dem Hochfahren oder nach NOT-HALT), sonst grau »Gestartet ✓«."""
        need = not self.started
        self.btn_start.setEnabled(need)
        if not need:
            self.btn_start.setText("Gestartet ✓")
            self.btn_start.setToolTip("START ist erfolgt. Nach einem NOT-HALT "
                                      "hier erneut quittieren.")
        elif self._estop_shown:
            self.btn_start.setText("NOT-HALT\nquittieren")
            self.btn_start.setToolTip("Quittiert den NOT-HALT (Arme, Gehen, Controller) "
                                      "und gibt die Modus-Kacheln wieder frei.")
        else:
            self.btn_start.setText("Start")
            self.btn_start.setToolTip("Gibt die Modus-Kacheln frei (START an loco_client "
                                      "und Arm-Regler). Liegt der G1 noch in Damp, stellt "
                                      "der loco_client ihn dabei auf Preparation.")

    def _auto_start(self):
        if self.started and self.mode is None and self.pending is None:
            self.request_mode(MANIP, auto=True)

    def _autostart_done(self, ok):
        if self.mode != MANIP:
            return
        if ok:
            self._status("Greifen bereit — Bewegung auswählen.", MODE_COLOR[MANIP])
        else:
            # Kein Regler-Quittung: GREIFEN nicht als aktiv vortaeuschen, damit
            # ein Antippen die Anforderung erneut sendet.
            self.mode = None
            self._refresh()
            self._status("Arme reagieren nicht (keine Rückmeldung vom Arm-Regler) — "
                         "GREIFEN antippen.", "#ef5350")

    def request_mode(self, mode, auto=False):
        if (not self.started or self._real_quit or mode == self.mode
                or mode == self.pending):
            return
        if not auto:
            self.autostart.cancel("Modus gewählt")
        self._cancel_sequence()
        self.set_auto_nav(False)
        self.walk_panel.reset()
        self.manip_panel.jog.reset()
        n = self.node
        if self._estop_unacked:
            # Sim nach NOT-HALT: START quittiert den E-Stop-Latch im
            # arm_controller -- sonst ignoriert er jedes ENABLE und die Arme
            # bleiben dauerhaft schlaff (Real: Knopf »Start« in der Startleiste).
            # Getrennte Topics -> keine Reihenfolge-Garantie: Modus (ENABLE)
            # erst kurz danach senden, sonst kaeme er evtl. vor dem START an.
            n.publish_bool(n.pub_start, True)
            self._estop_unacked = False
            self._estop_shown = False
            self._status("NOT-HALT quittiert …", "#aaa")
            QTimer.singleShot(300, lambda: self.request_mode(mode))
            return
        if mode == MANIP and auto:
            # Sim-Auto-Start: senden + wiederholen, bis quittiert (_autostart_done).
            self.autostart.start()
            n.share("mode", "grasp")
            self.mode, self.pending = MANIP, None
            self._status("Greifen wird eingeschaltet — warte auf Arm-Regler …",
                         MODE_COLOR[MANIP])
            self._reload_sequence_buttons()
        elif mode == MANIP:
            # BALANCING: Fuesse geplant, Arme frei -> sofort bestaetigt.
            # REAL: nur Arme, solange der G1 nicht im Walk steht (z.B. am Kran
            # in Preparation) -- kein implizites Umschalten auf Walk.
            n.publish_bool(n.pub_arms_enabled, True)
            if self.sim_mode or self._onboard_walk():
                n.publish_bool(n.pub_start_balancing, True)
            n.share("mode", "grasp")
            self.mode, self.pending = MANIP, None
            self._status("Greifen bereit — Bewegung auswählen.", MODE_COLOR[MANIP])
            self._reload_sequence_buttons()
        elif not self.sim_mode and not self._onboard_walk():
            # REAL: GEHEN stellt den Onboard-Modus nicht mehr selbst um --
            # das macht die Startleiste (Walk, mit Rueckfrage zum Kran).
            if status_line.ground_contact(n.onboard) is False:
                self._status("Gehen gesperrt: kein Bodenkontakt (Füße ohne Last) — "
                             "Kran lockern, dann Startleiste »Walk«.", "#ef5350")
            else:
                self._status("Gehen gesperrt: G1 steht nicht im Walk-Modus — "
                             "erst Startleiste Damp → Preparation → Walk.", "#ef5350")
            return
        else:
            # WALK: Arme fahren erst in die Lauf-Pose -> "wechselt", bis
            # arms/walk_ready kommt (oder Timeout).
            n.publish_bool(n.pub_start_walking, True)
            n.share("mode", "walk")
            self._await_walk("Arme werden eingeklappt …")
        self._refresh()

    def _await_walk(self, text):
        """WALK: Arme fahren erst in die Lauf-Pose -> "wechselt", bis
        arms/walk_ready kommt (oder Timeout)."""
        self.pending = WALK
        self._status(text, MODE_COLOR[WALK])
        self._walk_req += 1
        req = self._walk_req
        QTimer.singleShot(WALK_SWITCH_TIMEOUT_MS,
                          lambda: req == self._walk_req and self._on_walk_ready(True))

    # ── Gemeinsamer Zustand mit dem Controller ──────────────────────────
    def _on_shared(self, ch):
        """Wechsel am Controller (oder einer anderen Oberflaeche) nachziehen.
        Nur Anzeige/Einstellung: es wird NICHTS erneut gesendet."""
        jog = self.manip_panel.jog
        if "arm_side" in ch and ch["arm_side"] in JOG_SIDE_NAME:
            if self._jog_targets:
                self._jog_stop()
            jog.set_side(JOG_SIDE_NAME[ch["arm_side"]], notify=False)
        if "arm_speed" in ch:
            jog.set_speed(ch["arm_speed"], notify=False)
        if "walk_speed" in ch:
            self.walk_panel.set_speed(ch["walk_speed"], notify=False)
        # Modus nur, wenn er nach dem GUI-Start gewechselt wurde: ein alter
        # latched Stand beschreibt den Roboter nicht mehr.
        if "mode" in ch and self.node.shared.stamps.get("mode", 0.0) > self.node.t_start_wall:
            self._adopt_mode(WALK if ch["mode"] == "walk" else MANIP)

    def _adopt_mode(self, mode):
        """Modus wurde am Controller gewechselt (L2 / R2 / Options): Kacheln und
        Steuerbereich folgen. start_walking/start_balancing/arms/enabled hat
        der Controller schon gesendet (real nur, wenn der G1 im Walk steht)."""
        if self._real_quit or mode == self.mode or mode == self.pending:
            return
        if not self.started:
            self._status(f"Controller hat {MODE_TITLE[mode]} gewählt — "
                         "erst »Start« in der Startleiste drücken.", "#ffb300")
            return
        self.autostart.cancel("Modus am Controller gewählt")
        self._cancel_sequence()
        self.set_auto_nav(False)
        self.walk_panel.reset()
        self.manip_panel.jog.reset()
        if mode == MANIP:
            self.mode, self.pending = MANIP, None
            self._status("GREIFEN — am Controller gewählt.", MODE_COLOR[MANIP])
            self._reload_sequence_buttons()
        else:
            self._await_walk("GEHEN — am Controller gewählt, Arme werden eingeklappt …")
        self._refresh()

    def _onboard_walk(self):
        n = self.node
        if n.onboard is None or n.onboard_t is None:
            return False
        if time.monotonic() - n.onboard_t > status_line.STALE_S:
            return False
        return n.onboard.get("fsm") in status_line.WALK_FSM

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
        self.manip_panel.view_switch.setVisible(self.mode == MANIP)
        self.panel_rule.setVisible(self.mode is not None)
        self.panel_rule.setStyleSheet(f"background:{color}; border:none; border-radius:1px;")
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
            self._show_nav()
            return
        self.auto_nav = on
        self.node.publish_bool(self.node.pub_auto_enable, on)
        self._nav_req += 1
        self._nav_planning = False
        self.nav_station = None
        if on:
            # Liegt schon ein Pfad an (Ziel vorher in RViz gesetzt), laeuft er sofort.
            moving = self.node.nav_status == "moving"
            self.nav_phase = NAV_MOVING if moving else NAV_WAIT
            self._show_nav()
            if moving:
                self._status("AUTO NAV: läuft zum Ziel …", MODE_COLOR[WALK])
            else:
                self._status("AUTO NAV an — Ziel in RViz setzen (»2D Goal Pose«).",
                             MODE_COLOR[WALK])
        else:
            self.nav_phase = NAV_OFF
            self._show_nav()
            if self.mode == WALK:
                self._status("AUTO NAV aus — manuelle Steuerung frei.", MODE_COLOR[WALK])

    def _show_nav(self):
        self.walk_panel.set_nav_view(self.nav_phase, self.nav_station)

    def go_to_station(self, name):
        """Quickbefehl: Ziel setzen -> Planer plant -> AUTO NAV laeuft hin und
        schaltet sich nach dem Ankommen wieder aus."""
        st = self.node.stations.get(name)
        if st is None or self.mode != WALK:
            return
        self.node.publish_goal(*st)
        self.set_auto_nav(True)
        self.nav_station = name
        self._nav_planning = True   # 'arrived' eines alten Ziels ignorieren
        self.nav_phase = NAV_MOVING
        self._show_nav()
        self._status(f"AUTO NAV: läuft zur Station »{sm.station_label(name)}« …",
                     MODE_COLOR[WALK])
        req = self._nav_req
        QTimer.singleShot(NAV_PLAN_TIMEOUT_MS,
                          lambda: req == self._nav_req and self._nav_planning
                          and self._on_nav_status("no_path"))

    def _on_nav_status(self, state):
        """nav2point meldet moving / arrived / no_path -> Anzeige + Status."""
        if not self.auto_nav:
            return
        station = self.nav_station
        label = f"Station »{sm.station_label(station)}«" if station else "Ziel"
        to = f"zur {label}" if station else "zum Ziel"
        if state == "moving":
            self._nav_planning = False
            self.nav_phase = NAV_MOVING
            self._show_nav()
            self._status(f"AUTO NAV: läuft {to} …", MODE_COLOR[WALK])
        elif state == "arrived":
            if self._nav_planning:
                return      # gehoert noch zum vorherigen Ziel
            if station:
                self.set_auto_nav(False)
                self._status(f"Angekommen an {label}. AUTO NAV ist wieder aus.",
                             NAV_DONE_COLOR)
            else:
                self.nav_phase = NAV_ARRIVED
                self._show_nav()
                self._status("Am Ziel angekommen. AUTO NAV bleibt an.", NAV_DONE_COLOR)
        elif state == "no_path":
            self._nav_planning = False
            if station:
                self.set_auto_nav(False)
                self._status(f"Kein Weg {to} gefunden. AUTO NAV ist wieder aus.",
                             "#ffb300")
            else:
                self.nav_phase = NAV_WAIT
                self._show_nav()
                self._status("AUTO NAV: kein Weg zum Ziel gefunden — anderes Ziel "
                             "in RViz setzen.", "#ffb300")

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

    def _sequences(self):
        store = self._pose_store()
        if store is None:
            return []
        return [(cat[len(SEQUENCE_PREFIX):].strip(), sorted(ns, key=str.lower))
                for cat, ns in sorted(store.list_grouped().items())
                if cat.startswith(SEQUENCE_PREFIX) and ns]

    def _reload_sequence_buttons(self):
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
        self._jog_holds.clear()     # ein Halte-Ziel braeche die Fahrt ab
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

    def _safe_pose(self):
        """Gespeicherte »Sichere Pose« (SAFE_POSE) geplant anfahren."""
        self._cancel_sequence()
        store = self._pose_store()
        if store is None or store.get(SAFE_POSE) is None:
            self._status(f"Pose »{pretty_pose_name(SAFE_POSE)}« ist nicht gespeichert "
                         "(Erweitert → Pose speichern).", "#ffb300")
            return
        self.run_poses([SAFE_POSE], "Sichere Pose")

    def _home(self):
        """Grundstellung = Homing des arm_controller (/g1pilot/arms/home): Lauf-
        Pose, Arme neben dem Koerper -- geplant angefahren. Die »Sichere Pose«
        (Arme hoch, Ellbogen hinten) hat ihren eigenen Knopf."""
        self._cancel_sequence()
        self._jog_holds.clear()     # ein Halte-Ziel braeche die Fahrt ab
        self._pulse(self.node.pub_arms_home)
        self._status("Arme fahren in Grundstellung …", MODE_COLOR[MANIP])

    # ── Hand bewegen ────────────────────────────────────────────────────
    def _jog_tick(self):
        """~30 Hz: solange ein Bedienelement ausgelenkt ist, das Hand-Ziel mit
        dem eingestellten Tempo verschieben/drehen (Drehung um die Hand, Achsen
        im JOG_FRAME). Startpunkt ist die kommandierte Hand (JogReference, sonst TF)."""
        now = time.monotonic()
        dt = min(0.1, now - self._jog_t)
        self._jog_t = now
        mp = self.manip_panel
        jog = mp.jog
        lin, ang = jog.twist()
        active = (self.mode == MANIP and self.pending is None
                  and mp.views.currentWidget() is mp.arms_page
                  and max(abs(c) for c in lin + ang) > 1e-4)
        if active:
            for side in jog.sides():
                self._jog_holds.cancel(side)
        for side, pose in self._jog_holds.due(now).items():
            self.node.publish_hand_goal(side, *pose)
        if not active:
            if self._jog_targets:
                self._jog_stop()
            return
        if not self._jog_targets:
            self._cancel_sequence()     # laufenden Ablauf beenden (Marker-Vorrang)
            self._status(f"Hand bewegen: {jog.side.lower()} …", MODE_COLOR[MANIP])
        lead_m, lead_rad = lead_limits(jog.lin_speed, jog.ang_speed)
        missing = []
        for side in jog.sides():
            start, hand = self.node.jog_pose(side, side not in self._jog_targets)
            if hand is None:
                missing.append(side)
                continue
            lin_s, ang_s = jog.side_twist(side, lin, ang)
            p, q = jog_step(self._jog_targets.get(side), hand, lin_s, ang_s, dt,
                            lead_m, lead_rad, start=start)
            self._jog_targets[side] = [p, q]
            self._jog_twist[side] = (lin_s, ang_s)
            self.node.publish_hand_goal(side, p, q)
        if missing:
            self._status("Hand bewegen: Handposition unbekannt (TF fehlt) — "
                         "läuft der Roboter-Zustand?", "#ffb300")

    def _jog_stop(self):
        """Losgelassen: Ziel auf die aktuelle Hand setzen -> Arm bleibt sofort
        stehen statt den Vorlauf noch abzufahren (hold_pose: entlang der
        Bewegung an der Hand, quer auf der Ziel-Linie). Das Halte-Ziel wird eine
        Weile wiederholt (HoldStreamer, Ziel-Glaettung im arm_controller)."""
        now = time.monotonic()
        for side, target in self._jog_targets.items():
            _, hand = self.node.jog_pose(side, False)
            self.node.jog_end(side)
            if hand is not None:
                pose = hold_pose(target, hand,
                                 *self._jog_twist.get(side, ((0.0,) * 3, (0.0,) * 3)))
                self._jog_holds.hold(side, pose, now)
                self.node.publish_hand_goal(side, *pose)
        self._jog_targets = {}
        self._jog_twist = {}
        if self.mode == MANIP:
            self._status("Hand steht.", MODE_COLOR[MANIP])

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
        QTimer.singleShot(1500, self._reload_sequence_buttons)

    # ── Sicherheit / Status ─────────────────────────────────────────────
    def _pulse(self, pub, duration=1000):
        """Bool-Impuls True -> False (wie flash_button im Streamdeck)."""
        self.node.publish_bool(pub, True)
        QTimer.singleShot(duration, lambda: self.node.publish_bool(pub, False))

    def _push_press(self):
        """PUSH-Button gedrueckt: Aufladen beginnt (Anzeige im Button + Statuszeile)."""
        self._push_charger.press()
        self._push_timer.start()

    def _push_refresh(self):
        lvl = self._push_charger.level()
        if lvl > 0.0:
            f = push_charge.force_for_level(lvl)
            self.btn_push.setText(f"Schubs\n{lvl*100:.0f} % · {f:.0f} N")
            self._status(f"Schubs aufgeladen: {lvl*100:.0f} % ({f:.0f} N) – loslassen zum Schubsen",
                         "#ffb300")

    def _push_release(self):
        """Loslassen: schubst mit der aufgeladenen Kraft (kurzer Klick = Standard-Schubs)."""
        if not self._push_charger.holding:
            return
        self._push_timer.stop()
        lvl = self._push_charger.release()
        self.node.publish_push_charge(lvl)
        self.btn_push.setText("Roboter\nschubsen")
        f = push_charge.force_for_level(lvl)
        self._status(f"Roboter wird geschubst … ({f:.0f} N)", "#ffb300")

    def _reset_robot(self):
        """Nur Sim: wie beim Start -> loco_sim HOLD; die Bridge stellt den Roboter
        an den Startpunkt (bzw. g1_spawn) und haelt ihn (Weld). Beim naechsten
        Modus-Klick wird er in die Stand-Pose gestellt und freigegeben."""
        n = self.node
        self.autostart.cancel("Roboter zurueckgesetzt")
        self._cancel_sequence()
        self.set_auto_nav(False)
        self.walk_panel.reset()
        self.manip_panel.jog.reset()
        for pub in (n.pub_start_balancing, n.pub_start_walking, n.pub_arms_enabled):
            n.publish_bool(pub, False)
        n.publish_bool(n.pub_start, True)
        self._estop_unacked = False
        self._estop_shown = False
        self.mode = self.pending = None
        self.started = True
        self._refresh()
        self._status("Roboter steht wieder am Startpunkt (gehalten) — "
                     "oben einen Modus wählen.", "#aaa")

    def emergency_stop(self):
        n = self.node
        for pub in (n.pub_start, n.pub_start_balancing, n.pub_start_walking,
                    n.pub_arms_enabled, n.pub_arms_home):
            n.publish_bool(pub, False)
        n.publish_bool(n.pub_emergency_stop, True)
        self._enter_estop()

    def _external_estop(self):
        """NOT-HALT von aussen (PS-Taste am PS4-Controller): GUI wechselt in
        denselben Zustand wie beim eigenen Knopf, ohne erneut zu publizieren --
        sonst bliebe sie im alten Modus haengen und koennte nicht quittieren
        (Real: »Start« bliebe gesperrt). Der eigene NOT-HALT kommt
        hier ebenfalls an und ist dann schon angezeigt."""
        if not self._estop_shown:
            self._enter_estop()

    def _enter_estop(self):
        self._estop_shown = True
        if self._real_quit:
            # NOT-HALT hat Vorrang: »Real beenden« abbrechen, nichts stoppen.
            self._real_quit = 0
            self._disarm_quit()
            self.btn_quit.setEnabled(True)
        # Auch NOT-HALT von aussen beendet einen laufenden Sim-Auto-Start.
        self.autostart.cancel("NOT-HALT")
        self._jog_holds.clear()
        self._cancel_sequence()
        self.set_auto_nav(False)
        self.walk_panel.reset()
        self.manip_panel.jog.reset()
        self._estop_unacked = self.sim_mode
        self.mode = self.pending = None
        # REAL: nach NOT-HALT wieder bewusst ueber START gehen.
        self.started = self.sim_mode
        self._update_start_button()
        self._refresh()
        self._status("NOT-HALT aktiv — Roboter ist weich geschaltet. "
                     "Zum Fortsetzen oben einen Modus wählen.", "#ef5350")

    def _quit_clicked(self):
        """Erster Klick: scharf schalten (Rueckfrage im Knopf). Zweiter Klick
        innerhalb QUIT_CONFIRM_MS: Sim -> Host-Watcher stoppt den Stack sofort;
        Real -> erst Laufen stoppen und Arme abgeben (_quit_real_begin)."""
        if self._real_quit:
            return
        if not self._quit_armed:
            self._quit_armed += 1
            armed = self._quit_armed
            self.btn_quit.setText("Wirklich?\nnochmal tippen")
            self.btn_quit.setStyleSheet(state_css("#2a2a2a", "#ffb300",
                                                  "4px dashed #ffb300", 15))
            QTimer.singleShot(QUIT_CONFIRM_MS,
                              lambda: armed == self._quit_armed and self._disarm_quit())
            return
        self._quit_armed = 0
        self.btn_quit.setEnabled(False)
        if not self.sim_mode:
            self._quit_real_begin()
            return
        self.btn_quit.setText("Sim wird\nbeendet …")
        try:
            with open(SIM_SHUTDOWN_TRIGGER, "w") as f:
                f.write("quit")
        except OSError as e:
            self._sim_quit_unavailable(f"Trigger-Datei nicht schreibbar ({e}).")
            return
        self._status("Sim wird beendet — alle Fenster schließen sich gleich.", "#ffb300")
        QTimer.singleShot(QUIT_ACK_TIMEOUT_MS, self._check_quit_ack)

    def _disarm_quit(self):
        self._quit_armed = 0
        self.btn_quit.setText(self._quit_label)
        self.btn_quit.setStyleSheet(self._quit_css)

    def _check_quit_ack(self):
        # Der Watcher loescht die Datei sofort -> existiert sie noch, laeuft keiner
        # (z.B. per »docker compose up« oder »make sim-bg« gestartet).
        if os.path.exists(SIM_SHUTDOWN_TRIGGER):
            try:
                os.remove(SIM_SHUTDOWN_TRIGGER)
            except OSError:
                pass
            self._sim_quit_unavailable("Kein Host-Watcher aktiv.")

    def _sim_quit_unavailable(self, reason):
        self._disarm_quit()
        self.btn_quit.setEnabled(True)
        self._status(f"Sim beenden geht nur bei Start über ./start.sh oder make sim. "
                     f"{reason} Im Terminal: make stop", "#ef5350")

    # ── »Real beenden« ──────────────────────────────────────────────────
    def _quit_real_begin(self):
        """Real: 1) alles anhalten (Ablauf, AUTO NAV, Jog, WALK -> StopMove;
        BALANCING wird NICHT neu gestartet, der Roboter bleibt im Onboard-
        Zustand stehen), 2) Arme abgeben (ENABLE aus -> arm_controller faehrt
        sie in die Pose vom ENABLE zurueck, dann Gewicht 1->0), 3) Stack per
        Host-Watcher sanft stoppen. Beine: kein Damp -- ablegen per
        Unitree-Fernbedienung."""
        n = self.node
        self._real_quit += 1
        token = self._real_quit
        self.btn_quit.setText("Arme werden\nabgegeben …")
        self.autostart.cancel("Real beenden")
        self._cancel_sequence()
        self.set_auto_nav(False)
        self.walk_panel.reset()
        self.manip_panel.jog.reset()
        self._jog_holds.clear()
        # Laufen beenden: start_walking False -> loco_client StopMove. (Kein
        # start_balancing: der Roboter steht schon; ein nicht balancierter
        # Roboter wuerde sonst die Hoehenrampe starten.)
        n.publish_bool(n.pub_start_walking, False)
        self.mode = self.pending = None
        self._refresh()
        if n.arms_state:
            n.publish_bool(n.pub_arms_enabled, False)
            self._status("Real beenden: Arme fahren langsam in die Ausgangspose und "
                         "werden an den Roboter übergeben …", "#ffb300")
            self._real_quit_t0 = time.monotonic()
            self._real_quit_poll(token)
        else:
            self._quit_real_shutdown(token)

    def _real_quit_poll(self, token):
        if token != self._real_quit:
            return                      # abgebrochen (NOT-HALT)
        if self.node.arms_state is False:
            # arm_controller hat die Rueckfahrt beendet; jetzt laeuft noch die
            # Gewichtsrampe 1->0 -> abwarten, dann den Stack stoppen.
            QTimer.singleShot(REAL_RAMP_WAIT_MS, lambda: self._quit_real_shutdown(token))
            return
        if (time.monotonic() - self._real_quit_t0) * 1000.0 > REAL_RELEASE_TIMEOUT_MS:
            # Keine Quittung: trotzdem beenden. Beim Stoppen (SIGTERM) gibt der
            # arm_controller die Arme selbst langsam ab (shutdown_release).
            self._status("Keine Rückmeldung vom Arm-Regler — beende trotzdem "
                         "(er gibt die Arme beim Beenden selbst ab).", "#ffb300")
            self._quit_real_shutdown(token)
            return
        QTimer.singleShot(200, lambda: self._real_quit_poll(token))

    def _quit_real_shutdown(self, token):
        if token != self._real_quit:
            return
        self.btn_quit.setText("Real wird\nbeendet …")
        try:
            with open(REAL_SHUTDOWN_TRIGGER, "w") as f:
                f.write("quit")
        except OSError as e:
            self._real_quit_unavailable(f"Trigger-Datei nicht schreibbar ({e}).")
            return
        self._status("Arme übergeben — Stack wird beendet. Der Roboter bleibt im "
                     "Onboard-Modus stehen: per Fernbedienung ablegen.", "#ffb300")
        QTimer.singleShot(QUIT_ACK_TIMEOUT_MS, lambda: self._check_real_quit_ack(token))

    def _check_real_quit_ack(self, token):
        if token != self._real_quit:
            return
        if os.path.exists(REAL_SHUTDOWN_TRIGGER):
            try:
                os.remove(REAL_SHUTDOWN_TRIGGER)
            except OSError:
                pass
            self._real_quit_unavailable("Kein Host-Watcher aktiv.")

    def _real_quit_unavailable(self, reason):
        self._real_quit = 0
        self._disarm_quit()
        self.btn_quit.setEnabled(True)
        self._status(f"Arme sind abgegeben. Stack-Stopp geht nur bei Start über "
                     f"./start.sh oder make real. {reason} Im Terminal: Strg+C "
                     f"oder make stop", "#ef5350")

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
    # Eigenes Dock-Icon, SIM blau / REAL rot (statt Zahnrad), vor dem ersten show().
    if is_sim_mode():
        set_app_identity(app, "g1-demo-sim", "g1_demo_sim")
    else:
        set_app_identity(app, "g1-demo-real", "g1_demo_real")
    gui = DemoGUI(node)
    # G1_GUI_FULLSCREEN=1: direkt im Vollbild starten. Standard: maximiert
    # (bildschirmfuellend mit Titelleiste, falls der Fenstermanager eine hat).
    if os.environ.get("G1_GUI_FULLSCREEN", "").strip().lower() in ("1", "true", "yes", "on"):
        gui.showFullScreen()
    else:
        gui.showMaximized()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec()
    node.tf_node.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo_gui — vereinfachte Bedienoberflaeche fuer Vorfuehrungen (KONZEPT).

Gedacht fuer Besucher/Demo-Bediener, die den Stack nicht kennen. Statt 25
Einzel-Kacheln (frueher die Streamdeck-GUI) gibt es drei klar getrennte Bereiche:

  1) MODUS (oben)      : GEHEN  <->  GREIFEN. Genau einer ist aktiv und farbig
                         hervorgehoben; der andere ist ausgegraut. Waehrend des
                         Wechsels (Arme raeumen auf) blinkt "wechselt ...".
  2) STEUERUNG (Mitte) : zeigt NUR die Bedienelemente des aktiven Modus.
                         GEHEN   -> Joystick-Knopf + Pfeiltasten + Drehen.
                         GREIFEN -> Umschalter ARME | HAENDE:
                                    ARME   = Ablaeufe (pose_store, Fenster
                                             »Posen & Abläufe«), Am Stück /
                                             Schritt für Schritt + »Weiter«
                                             (Controller: L3), Grundstellung
                                             und eingeklappter "Erweitert"-Bereich.
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

Die GUI publiziert nur Bedien-Topics (gui_base.GuiNode) -- die Logik bleibt
in loco_sim/loco_client/arm_controller. Sie ist die einzige Bedienoberflaeche
(die alte Streamdeck-GUI ui_interface.py wurde 2026-10-30 entfernt), siehe
launch/teleoperation_launcher.launch.py und docs/10-A-Demo-GUI.md.
"""

import json
import math
import os
import sys
import time
from collections import deque

from PyQt6.QtWidgets import (
    QApplication, QWidget, QGridLayout, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QFrame, QDialog, QSizePolicy, QGraphicsOpacityEffect, QScrollArea,
)
from PyQt6.QtCore import QTimer, Qt, QEvent
from PyQt6.QtGui import QKeySequence, QShortcut, QIcon

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

from g1pilot.navigation import scene_markers as sm

from g1pilot.utils import msgbox
from g1pilot.utils import ui_theme as T
from g1pilot.utils.window_style import DarkStyle, set_app_identity
from g1pilot.utils import ui_icons
from g1pilot.utils.common import is_sim_mode
from g1pilot.utils.hand_profile import hand_profile, inspire_active
from g1pilot.utils import g1_temperature as g1t
from g1pilot.utils import g1_battery as g1b
from g1pilot.utils import push_charge
from g1pilot.utils.push_charge import PushCharger
from g1pilot.teleoperation.sim_autostart import SimAutoStart
from g1pilot.teleoperation.gui_base import (
    GuiNode, VirtualJoystick, PoseSaveDialog, PoseLoadDialog,
)
from g1pilot.teleoperation.hand_panel import HandPanel
from g1pilot.teleoperation import demo_ui as UI
from g1pilot.teleoperation.demo_ui import ModeTile, VerticalRocker, TempPanel, state_css
from g1pilot.teleoperation import status_line
from g1pilot.teleoperation import shared_state
from g1pilot.teleoperation import sequence_runner as sr
from g1pilot.teleoperation.sequence_editor import SequenceEditor
from g1pilot.teleoperation.phone import link as phone_link
from g1pilot.teleoperation.ps4_arm_teleop import mirror_twist
from g1pilot.manipulation.waist_yaw import (
    WAIST_CENTER_TOPIC, WAIST_RATE_TOPIC, WAIST_STATE_TOPIC, STATE_STALE_S, rotate_twist_yaw,
)
from g1pilot.teleoperation.ps4_joystick import DEVICE_TOPIC
from g1pilot.teleoperation.hand_jog import (
    JOG_FRAME, JOG_TF, JOG_SPEEDS, JOG_CMD_TOPIC, CommandedPoses, HoldStreamer, hold_pose,
    JogReference, jog_reference_mode, lead_limits, jog_step,
)

# Modi und Farbcode (Modus-Umschalter, Linie unter der Kopfleiste, Karten).
WALK, MANIP = UI.WALK, UI.MANIP
MODE_COLOR = UI.MODE_COLOR
MODE_TITLE = UI.MODE_TITLE

# Geschwindigkeitsstufen (normiert, loco_sim skaliert mit max_cmd ~0.8 m/s).
# Fuer Vorfuehrungen bewusst gedeckelt -- "Normal" ist nicht Vollgas.
SPEEDS = {"Langsam": 0.3, "Normal": 0.6}
# Geh-Tempo ist mit dem Controller geteilt (Langsam/Normal/Schnell). »Schnell«
# gibt es nur am Controller (L2 + ▲): die GUI zeigt es an, geht selbst aber
# hoechstens mit »Normal«.
WALK_GUI_MAX = "Normal"

# Vorgeschlagene Kategorie im Dialog »Pose speichern« (auch demo_sequence.py).
DEMO_CATEGORY = os.environ.get("G1_DEMO_CATEGORY", "Demo")
# Ablaeufe stehen im pose_store (Liste von Pose-Verweisen, Fenster »Posen &
# Abläufe«). Speichern in eine Kategorie "Ablauf <Name>" haengt die Pose dort an.
SEQUENCE_PREFIX = "Ablauf "
# Ablauf-Modus beim Start: real Schritt fuer Schritt (jeder Schritt bestaetigt),
# Sim am Stueck. G1_SEQUENCE_MODE=auto|step ueberschreibt.
SEQUENCE_MODE_ENV = "G1_SEQUENCE_MODE"
# Knopf "Sichere Pose" faehrt diese gespeicherte Pose an.
SAFE_POSE = os.environ.get("G1_SAFE_POSE", "Sichere_Pose")
# WALK gilt als "angekommen", wenn arms/walk_ready kommt -- spaetestens nach
# diesem Timeout (loco_sim laeuft dann ohnehin selbst los).
WALK_SWITCH_TIMEOUT_MS = 16000   # > loco_sim walk_arm_timeout_s (15 s)


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
# Sim-Knopf »Roboter schubsen« (Text waehrend des Aufladens: Ladegrad).
PUSH_LABEL = "Roboter schubsen"
# So viele Meldungen merkt sich die Statusleiste (Knopf »Verlauf«).
HISTORY_LEN = 200

SIM_SHUTDOWN_TRIGGER = "/ros2_ws/src/g1pilot/.sim_shutdown_request"
QUIT_CONFIRM_MS = 4000      # so lange gilt der erste Klick als »scharf«
# VOLL-NOTAUS-Knopf: so lange ununterbrochen halten (Controller: 2 s fuer
# R1 + L1 + PS). Kuerzer, weil der Druck selbst schon den NOT-HALT ausloest.
FULL_ESTOP_HOLD_S = 1.5
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


# AUTO NAV nur, wenn der Nav-Stack laeuft (nur Sim: G1_ENABLE_NAV) -- sonst
# publiziert der Knopf an niemanden.
NAV_AVAILABLE = _envflag('G1_ENABLE_NAV')


pretty_pose_name = sr.pretty_pose_name   # 'Demo_2_Winken' -> 'Winken'


def relative_move_command(move, hand_reference, yaw=0.0):
    """Relative Bewegung {"dx","dy","dz","sides"} -> (arm_command dict, None)
    oder (None, Fehlertext). Verschiebung im Oberkoerper-Bezug (um den
    Taillen-Yaw gedreht, wie der Jog), Orientierung bleibt. Ziel = aktuelle
    (kommandierte) Hand + Verschiebung, als EIN Kommando fuer alle Seiten."""
    d, _ = rotate_twist_yaw((move["dx"], move["dy"], move["dz"]), (0.0, 0.0, 0.0), yaw)
    cmd = {"type": "pose"}
    for side in move["sides"]:
        ref = hand_reference(side)
        if ref is None:
            return None, (f"Handposition {'links' if side == 'left' else 'rechts'} "
                          "unbekannt (TF fehlt).")
        pos, quat = ref
        cmd[side] = {"position": [float(pos[i]) + d[i] for i in range(3)],
                     "orientation": [float(q) for q in quat], "frame": JOG_FRAME}
    return cmd, None


def default_sequence_mode(sim):
    env = (os.environ.get(SEQUENCE_MODE_ENV) or "").strip().lower()
    if env in sr.MODE_TITLE:
        return env
    return sr.MODE_AUTO if sim else sr.MODE_STEP


class DemoNode(GuiNode):
    """Bedien-Publisher (gui_base.GuiNode) + Rueckkanaele fuer die Statusanzeige."""

    def __init__(self):
        super().__init__()
        self.on_walk_ready = None
        self.on_arm_status = None
        self.on_hand_status = None
        self.create_subscription(Bool, "/g1pilot/arms/walk_ready",
                                 lambda m: self.on_walk_ready and self.on_walk_ready(m.data), 10)
        self.create_subscription(String, "/g1pilot/arm_command/status",
                                 self._arm_status, 10)
        # Ablauf: »Weiter« vom Controller (L3), Stopp vom Controller
        # (pose_store/cancel beendet auch einen wartenden Ablauf), Speicher-
        # Quittung (Schritt anhaengen) und eigener Stand (latched, Controller
        # vibriert beim Warten; Handy/Logs).
        self.on_seq_next = None
        self.on_pose_cancel = None
        self.on_save_status = None
        self.create_subscription(Bool, sr.NEXT_TOPIC,
                                 lambda m: m.data and self.on_seq_next and self.on_seq_next(), 10)
        self.create_subscription(Bool, "/g1pilot/pose_store/cancel",
                                 lambda m: m.data and self.on_pose_cancel and self.on_pose_cancel(),
                                 10)
        self.create_subscription(String, "/g1pilot/pose_store/save/status",
                                 self._save_status, 10)
        # Relative Bewegungen (Ablauf-Schritt {"move"}): kartesisches Ziel,
        # geplant wie jede Pose (arm_command type=pose, Status wie pose_store).
        self.pub_arm_command = self.create_publisher(String, "/g1pilot/arm_command", 10)
        seq_qos = QoSProfile(depth=1)
        seq_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_seq_state = self.create_publisher(String, sr.STATE_TOPIC, seq_qos)
        # Hand-Bridge (inspire_ftp/bridge.py): Einzelbefehle + Zustand.
        self.pub_hand_cmd = self.create_publisher(String, "/g1pilot/hand_cmd", 10)
        self.create_subscription(String, "/g1pilot/hand_status", self._hand_status, 10)
        # Motor-/IMU-Temperaturen des echten G1 (robot_state, reiner Lesepfad).
        self.temps = None
        self.temps_t = None
        self.create_subscription(String, "/g1pilot/temperatures", self._temperatures, 10)
        # Akku des echten G1 (robot_state aus rt/lf/bmsstate, reiner Lesepfad).
        self.battery = None
        self.battery_t = None
        self.create_subscription(String, g1b.TOPIC_ROS, self._battery, 10)
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
        # Hueftdrehung (arm_controller, 10 Hz): Anzeige, Jog-Bezug, Knopf »Mitte«.
        self.waist = self.waist_t = None
        self.create_subscription(String, WAIST_STATE_TOPIC,
                                 lambda m: self._latched_json("waist", m), 10)
        self.pub_waist_center = self.create_publisher(Bool, WAIST_CENTER_TOPIC, 10)
        # Huefte drehen aus der GUI (Schieber in »Hand bewegen«): wie der
        # Controller ein Rate-Strom -1..1 (+ = links), ~30 Hz solange gezogen,
        # beim Loslassen einmal 0. Der arm_controller haelt ohne Strom an
        # (Totmann) und prueft selbst, ob er die Taille drehen darf.
        self.pub_waist_rate = self.create_publisher(Float32, WAIST_RATE_TOPIC, 10)
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

    def torso_yaw(self):
        """Gemessener Taillen-Yaw [rad] (Jog im Oberkoerper-Bezug), 0 ohne Daten."""
        if self.waist is None or self.waist_t is None or \
                time.monotonic() - self.waist_t > STATE_STALE_S:
            return 0.0
        try:
            return float(self.waist.get("yaw_rad") or 0.0)
        except (TypeError, ValueError):
            return 0.0

    def hand_reference(self, side):
        """-> ([x, y, z], (qx, qy, qz, qw)) der Hand im JOG_FRAME fuer relative
        Bewegungen: kommandierte Hand (wie Jog, ohne Durchhang), sonst TF."""
        cmd = self.cmd_poses.get(side, time.monotonic()) if self.jog_from_command else None
        return cmd if cmd is not None else self.hand_pose(side)

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

    def _battery(self, msg: String):
        data = g1b.parse_payload(msg.data)
        if data is not None:
            self.battery, self.battery_t = data, time.monotonic()

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

    def _save_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, dict) and self.on_save_status:
            self.on_save_status(data)


# ── Bausteine (demo_ui.py) ───────────────────────────────────────────────
def hands_controllable(env=None):
    """HAENDE-Seite nutzbar? Real immer (Inspire), Sim nur mit Inspire-Profil."""
    env = os.environ if env is None else env
    if str(env.get("G1_SIM_MODE", "false")).strip().lower() != "true":
        return True
    return inspire_active(env)


# ── Hand bewegen ─────────────────────────────────────────────────────────
class ArmJogPanel(QWidget):
    """Hand bewegen, zwei Gruppen (beide gleichzeitig nutzbar), Richtungen aus
    Sicht des Roboters (JOG_FRAME), Drehung um die Hand selbst:
      VERSCHIEBEN  Joystick = vor/zurueck + links/rechts, Schieber = hoch/runter
      DREHEN       Joystick = kippen (Finger hoch/runter) + schwenken
                   (links/rechts), Schieber = rollen (um die Vorwaertsachse)
    Gedrueckt halten = Hand faehrt, loslassen = Hand steht.
    twist() -> ((vx, vy, vz) m/s, (wx, wy, wz) rad/s) im JOG_FRAME."""

    STICK = 170        # Startgroesse, danach nach Breite
    STICK_MIN, STICK_MAX = 140, 220

    def __init__(self):
        super().__init__()
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(12)
        # on_change(feld, wert): Auswahl in der GUI geaendert -> gemeinsamer Zustand
        self.on_change = None
        opts = QHBoxLayout()
        opts.setSpacing(16)
        col = QVBoxLayout()
        col.setSpacing(4)
        col.addWidget(UI.caption("Arm"))
        w, self.side_btns = UI.segmented(list(JOG_SIDES), self.set_side, font=14, height=40)
        col.addWidget(w)
        opts.addLayout(col, 4)
        col = QVBoxLayout()
        col.setSpacing(4)
        col.addWidget(UI.caption("Tempo"))
        w, self.speed_btns = UI.segmented(list(JOG_SPEEDS), self.set_speed, font=14, height=40)
        col.addWidget(w)
        opts.addLayout(col, 3)
        outer.addLayout(opts)

        outer.addStretch(1)
        sticks = QHBoxLayout()
        sticks.setSpacing(28)
        sticks.addStretch(1)
        self.move_stick, self.move_rocker = self._group(
            sticks, "VERSCHIEBEN", "vor / zurück · links / rechts", "hoch / runter",
            ("chevron-up", "chevron-down"))
        self.turn_stick, self.turn_rocker = self._group(
            sticks, "DREHEN", "kippen · schwenken", "rollen", ("rotate-ccw", "rotate-cw"))
        sticks.addStretch(1)
        outer.addLayout(sticks)
        outer.addStretch(1)

        self.set_side("Rechts")
        self.set_speed("Normal")
        # Mindestbreite mit den kleinsten Joysticks (sonst schrumpften sie nie).
        self.setMinimumWidth(max(opts.minimumSize().width(),
                                 2 * (self.STICK_MIN + 70) + 28 + 40))

    def _group(self, parent, title, stick_cap, rocker_cap, rocker_labels):
        box = QVBoxLayout()
        box.setSpacing(4)
        head = QLabel(title)
        head.setStyleSheet(f"color:{MODE_COLOR[MANIP]}; font-size:13px; font-weight:800;"
                           "letter-spacing:1px; background:transparent;")
        box.addWidget(head)
        row = QHBoxLayout()
        row.setSpacing(10)
        col = QVBoxLayout()
        col.addWidget(UI.caption(stick_cap, size=12), alignment=Qt.AlignmentFlag.AlignCenter)
        stick = VirtualJoystick(self.STICK)
        col.addWidget(stick, alignment=Qt.AlignmentFlag.AlignCenter)
        row.addLayout(col)
        col = QVBoxLayout()
        col.addWidget(UI.caption(rocker_cap, size=12), alignment=Qt.AlignmentFlag.AlignCenter)
        rocker = VerticalRocker(self.STICK, labels=rocker_labels)
        col.addWidget(rocker, alignment=Qt.AlignmentFlag.AlignCenter)
        row.addLayout(col)
        box.addLayout(row)
        box.addStretch(1)
        parent.addLayout(box)
        return stick, rocker

    def resizeEvent(self, e):
        """Joysticks + Schieber wachsen mit der Kartenbreite (STICK_MIN..STICK_MAX)."""
        free = e.size().width() - 2 * (60 + 10) - 28 - 40
        size = max(self.STICK_MIN, min(self.STICK_MAX, free // 2))
        if size != self.move_stick.width() and not self._jog_held():
            for st in (self.move_stick, self.turn_stick):
                st.set_size(size)
            for ro in (self.move_rocker, self.turn_rocker):
                ro.set_length(size)
        super().resizeEvent(e)

    def _jog_held(self):
        lin, ang = self.twist()
        return max(abs(c) for c in lin + ang) > 1e-4

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


# ── GEHEN ────────────────────────────────────────────────────────────────
class WalkPanel(QWidget):
    """Zwei Karten: links »Manuell gehen« (Joystick-Knopf ODER Pfeiltasten,
    gedrueckt halten = laufen, loslassen = stehen; Tempo), rechts
    »Auto-Navigation« (AUTO NAV + Stationen als Raster).
    velocity() -> (vx, vy, yaw), bereits mit Stufe skaliert."""

    def __init__(self):
        super().__init__()
        self.held = set()        # gerade gehaltene Pfeiltasten
        self.speed = SPEEDS["Langsam"]
        self.speed_name = "Langsam"
        self.on_change = None    # on_change("walk_speed", stufe) -> gemeinsamer Zustand

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        self.row = UI.ResponsiveRow(breakpoint=900)
        outer.addWidget(self.row, 1)

        # Karte 1: manuell. Der Inhalt liegt in self.manual -> bei AUTO NAV sperren.
        self.manual_card = UI.Card("Manuell gehen", "footprints", MODE_COLOR[WALK],
                                   "gedrückt halten · loslassen = stehen")
        self.manual = QWidget()
        mlay = QVBoxLayout(self.manual)
        mlay.setContentsMargins(0, 0, 0, 0)
        mlay.setSpacing(12)
        mlay.addStretch(1)
        ctl = QHBoxLayout()
        ctl.setSpacing(30)
        ctl.addStretch(1)
        left = QVBoxLayout()
        left.addWidget(UI.caption("Knopf ziehen"), alignment=Qt.AlignmentFlag.AlignCenter)
        self.joystick = VirtualJoystick(240)
        left.addWidget(self.joystick, alignment=Qt.AlignmentFlag.AlignCenter)
        left.addStretch(1)
        ctl.addLayout(left)
        mid = QVBoxLayout()
        mid.addWidget(UI.caption("…oder Pfeil gedrückt halten"),
                      alignment=Qt.AlignmentFlag.AlignCenter)
        pad = QGridLayout()
        pad.setSpacing(10)
        arrows = {"fwd": ("arrow-up", 0, 1), "back": ("arrow-down", 2, 1),
                  "left": ("arrow-left", 1, 0), "right": ("arrow-right", 1, 2),
                  "turn_l": ("rotate-ccw", 0, 0), "turn_r": ("rotate-cw", 0, 2)}
        tips = {"fwd": "vorwärts", "back": "rückwärts", "left": "seitlich links",
                "right": "seitlich rechts", "turn_l": "links drehen", "turn_r": "rechts drehen"}
        self.arrow_btns = {}
        for key, (ic, r, c) in arrows.items():
            b = UI.big_button("", height=72)
            ui_icons.set_icon(b, ic, "#ffffff", 34)
            b.setToolTip(f"{tips[key]} (gedrückt halten)")
            b.setFixedWidth(86)
            b.pressed.connect(lambda k=key: self.held.add(k))
            b.released.connect(lambda k=key: self.held.discard(k))
            pad.addWidget(b, r, c)
            self.arrow_btns[key] = b
        mid.addLayout(pad)
        mid.addStretch(1)
        ctl.addLayout(mid)
        ctl.addStretch(1)
        mlay.addLayout(ctl)
        mlay.addStretch(1)
        tempo = QHBoxLayout()
        tempo.setSpacing(10)
        tempo.addWidget(UI.caption("Tempo"))
        names = [n if n in SPEEDS else f"{n} (Controller)" for n in shared_state.SPEED_ORDER]
        w, btns = UI.segmented(names, None, color=MODE_COLOR[WALK], font=15, height=42)
        self.speed_btns = {}
        for name, label in zip(shared_state.SPEED_ORDER, names):
            b = btns[label]
            if name in SPEEDS:
                b.clicked.connect(lambda _=False, n=name: self.set_speed(n))
            else:
                # Nur Anzeige: am Controller gewaehlt (R1 bzw. L2 + Steuerkreuz hoch).
                b.setEnabled(False)
                b.setToolTip("Nur am Controller wählbar (im GEHEN R1, oder L2 halten + "
                             f"Steuerkreuz hoch). Die GUI selbst geht dann mit »{WALK_GUI_MAX}«.")
            self.speed_btns[name] = b
        w.setMaximumWidth(520)
        tempo.addWidget(w, 1)
        tempo.addStretch(1)
        mlay.addLayout(tempo)
        self.manual_card.body.addWidget(self.manual, 1)
        self.row.add(self.manual_card, 3)
        self.set_speed("Langsam")
        # Controller-Ansicht: statt Joystick/Pfeilen Zustand + Belegung (Gehen).
        self.ctrl_col = QWidget()
        cl = QVBoxLayout(self.ctrl_col)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(14)
        self.ctrl_card = UI.ControllerCard(MODE_COLOR[WALK], (
            ("state", "Gehen"), ("speed", "Geh-Tempo")))
        cl.addWidget(self.ctrl_card)
        self.legend_card = UI.LegendCard()
        cl.addWidget(self.legend_card, 1)
        self.ctrl_col.hide()
        self.row.add(self.ctrl_col, 3)
        self.controller_view = False

        # Karte 2: AUTO NAV -- Roboter faehrt selbststaendig zum in RViz gesetzten
        # Ziel (/g1pilot/auto_enable -> joy_mux gibt den Nav-Joy weiter).
        self.nav_card = UI.Card("Auto-Navigation", "anchor", NAV_DONE_COLOR)
        self.btn_auto_nav = UI.big_button("", "#283593", font=17, height=64)
        self.btn_auto_nav.setCheckable(True)
        self.btn_auto_nav.setEnabled(NAV_AVAILABLE)
        self._auto_nav_css = self.btn_auto_nav.styleSheet()
        self.nav_card.body.addWidget(self.btn_auto_nav)
        self.nav_hint = UI.caption("")
        self.nav_hint.setWordWrap(True)
        self.nav_card.body.addWidget(self.nav_hint)
        self.nav_card.body.addWidget(UI.section("Stationen"))
        # Stations-Knoepfe: setzen das Ziel + schalten AUTO NAV ein. Werden aus
        # den station_<Name>-Markierungen der geladenen Szene gebaut.
        self.station_grid = UI.TileGrid(tile_w=150, placeholder="Station suchen …")
        self.nav_card.body.addWidget(self.station_grid, 1)
        self.row.add(self.nav_card, 2)
        self.station_btns = []
        self.station_by_name = {}
        self.nav_station = None   # Station, zu der gerade gelaufen wird
        self.on_station = None
        self.set_stations([])
        self.set_nav_view(NAV_OFF)

    @staticmethod
    def _caption(text):
        return UI.caption(text, size=15)

    def set_stations(self, names):
        self.station_btns = []
        self.station_by_name = {}
        for name in names:
            b = UI.big_button(sm.station_label(name), UI.STATION_COLOR, font=15, height=52)
            b.setEnabled(NAV_AVAILABLE)
            b.setToolTip(f"Ziel »{sm.station_label(name)}« setzen und selbst hinlaufen")
            b.clicked.connect(lambda _, n=name: self.on_station and self.on_station(n))
            b.default_css = b.styleSheet()
            self.station_btns.append(b)
            self.station_by_name[name] = b
        self.station_grid.set_widgets(
            self.station_btns, "Keine Stationen — in der Szene Objekte »station_<Name>« anlegen.")
        self.nav_card.set_hint(f"{len(names)} Stationen" if names else "")
        self._style_stations()

    def _style_stations(self):
        """Station, zu der gerade gelaufen wird: gestrichelt (wie »wechselt«)."""
        for name, b in self.station_by_name.items():
            if name == self.nav_station:
                b.setStyleSheet(UI.state_css(T.CARD_HI, NAV_DONE_COLOR,
                                             f"3px dashed {NAV_DONE_COLOR}", 15))
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
        self.manual_card.set_hint("gesperrt: AUTO NAV ist an" if on
                                  else "gedrückt halten · loslassen = stehen")
        self.nav_station = station if phase == NAV_MOVING else None
        self._style_stations()
        if on:
            self.reset()
        walk = MODE_COLOR[WALK]
        to = f"zur Station »{sm.station_label(station)}«" if station else "zum Ziel"
        if phase == NAV_MOVING:
            self.btn_auto_nav.setText("AUTO NAV  läuft …  ·  antippen zum Stoppen")
            self.btn_auto_nav.setStyleSheet(
                UI.state_css(T.CARD_HI, walk, f"3px dashed {walk}", 17))
            self.nav_hint.setText(f"Läuft selbstständig {to}. "
                                  "Manuelle Steuerung ist gesperrt.")
        elif phase == NAV_ARRIVED:
            self.btn_auto_nav.setText("AUTO NAV  AN  ·  Ziel erreicht")
            self.btn_auto_nav.setStyleSheet(
                UI.state_css("#2e7d32", "white", "3px solid #8db4fa", 17))
            self.nav_hint.setText("Am Ziel angekommen. AUTO NAV bleibt an — nächstes "
                                  "Ziel in RViz setzen oder antippen zum Stoppen.")
        elif phase == NAV_WAIT:
            self.btn_auto_nav.setStyleSheet(self._auto_nav_css)
            self.btn_auto_nav.setText("AUTO NAV  AN  ·  wartet auf Ziel")
            self.nav_hint.setText("Ziel in RViz setzen (»2D Goal Pose«) oder Station "
                                  "wählen. Manuelle Steuerung ist gesperrt.")
        elif NAV_AVAILABLE:
            self.btn_auto_nav.setStyleSheet(self._auto_nav_css)
            self.btn_auto_nav.setText("AUTO NAV  ·  selbst zum Ziel laufen")
            self.nav_hint.setText("Ziel in RViz setzen (»2D Goal Pose«) und AUTO NAV "
                                  "einschalten, oder direkt eine Station wählen.")
        else:
            self.btn_auto_nav.setStyleSheet(self._auto_nav_css)
            self.btn_auto_nav.setText("AUTO NAV")
            self.nav_hint.setText("Navigation ist nicht gestartet — im Startmenü "
                                  "Ausstattung → »Navigation« aktivieren.")

    def reset(self):
        self.held.clear()
        self.joystick.mouseReleaseEvent(None)   # Knopf zentrieren -> vx=vy=0

    def set_controller_view(self, on):
        """Controller-Ansicht: »Manuell gehen« weg (L2 halten am Controller),
        dafuer Zustand + Belegung. Auto-Navigation und Stationen bleiben."""
        self.controller_view = bool(on)
        self.reset()
        self.manual_card.setVisible(not on)
        self.ctrl_col.setVisible(bool(on))
        self.updateGeometry()


# ── GREIFEN ──────────────────────────────────────────────────────────────
class ManipPanel(QWidget):
    """Kopfzeile: Umschalter ARME | HAENDE und »Bewegung stoppen«.
    ARME: drei Karten -- »Hand bewegen« (Joysticks + Hueftdrehung),
          »Abläufe« (Raster mit Suche, Am Stück / Schritt, »Weiter«) und
          »Posen« (Sichere Pose, Grundstellung, Pose anfahren/speichern,
          RViz-Marker folgen).
    HAENDE: HandPanel (ganze Hand + einzelne Finger, Kraftzonen)."""

    def __init__(self, gui):
        super().__init__()
        self.gui = gui
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(12)

        head = QHBoxLayout()
        head.setSpacing(12)
        self.view_switch, btns = UI.segmented(["  ARME", "  HÄNDE"], None, color=MODE_COLOR[MANIP],
                                              font=16, height=44, min_w=150)
        self.view_btns = {"arms": btns["  ARME"], "hands": btns["  HÄNDE"]}
        ui_icons.set_icon(self.view_btns["arms"], "bot", "#ffffff", 18)
        ui_icons.set_icon(self.view_btns["hands"], "hand", "#ffffff", 18)
        for key, b in self.view_btns.items():
            b.clicked.connect(lambda _=False, k=key: self.show_view(k))
        head.addWidget(self.view_switch)
        head.addStretch(1)
        self.btn_cancel = UI.big_button("Bewegung stoppen", "#5d4037", font=16, height=44,
                                        icon="circle-stop")
        self.btn_cancel.setToolTip("Laufende Arm-Bewegung oder Ablauf anhalten "
                                   "(Controller: Kreuz)")
        self.btn_cancel.setFixedWidth(240)
        head.addWidget(self.btn_cancel)
        outer.addLayout(head)

        self.views = UI.CurrentStack()
        outer.addWidget(self.views, 1)
        arms = QWidget()
        self.views.addWidget(arms)
        self.hand_panel = HandPanel(
            gui.node, status_cb=lambda t: gui._status(t, MODE_COLOR[MANIP]))
        self.views.addWidget(self.hand_panel)
        self.view_pages = {"arms": arms, "hands": self.hand_panel}
        self.arms_page = arms

        lay = QVBoxLayout(arms)
        lay.setContentsMargins(0, 0, 0, 0)
        self.row = UI.ResponsiveRow(breakpoint=940)
        lay.addWidget(self.row, 1)

        # Karte »Hand bewegen« (Joystick + Hoch/Runter) -- statt Marker in RViz,
        # unten die Hueftdrehung (wie am Controller R2 + rechter Stick).
        self.jog_card = UI.Card("Hand bewegen", "move", MODE_COLOR[MANIP],
                                "gedrückt halten · loslassen = stehen")
        self.jog = ArmJogPanel()
        self.jog_card.body.addWidget(self.jog, 1)
        rule = QFrame()
        rule.setFixedHeight(1)
        rule.setStyleSheet(f"background:{T.LINE}; border:none;")
        self.jog_card.body.addWidget(rule)
        waist = QHBoxLayout()
        waist.setSpacing(12)
        wcol = QVBoxLayout()
        wcol.setSpacing(2)
        self.waist_label = QLabel("Hüfte: –")
        self.waist_label.setStyleSheet(f"color:{UI.WAIST_COLOR}; font-size:15px; font-weight:700;")
        wcol.addWidget(self.waist_label)
        whint = UI.caption("Knopf ziehen = drehen, loslassen = steht "
                           "(Controller: R2 + rechter Stick)", size=12)
        whint.setWordWrap(True)
        wcol.addWidget(whint)
        waist.addLayout(wcol, 2)
        self.waist_rocker = UI.HorizontalRocker(170)
        self.waist_rocker.setToolTip("Oberkörper drehen: nach links/rechts ziehen, "
                                     "loslassen = Hüfte steht")
        waist.addWidget(self.waist_rocker, 2)
        self.btn_waist_center = UI.big_button("Hüfte Mitte", font=15, height=44)
        self.btn_waist_center.setFixedWidth(140)
        waist.addWidget(self.btn_waist_center)
        self.jog_card.body.addLayout(waist)
        self.row.add(self.jog_card, 5)

        # Controller-Ansicht (UI.VIEW_CONTROLLER): statt »Hand bewegen« der
        # Zustand des Controllers + Belegung; »Posen« rutscht darunter.
        self.ctrl_col = QWidget()
        self.ctrl_lay = QVBoxLayout(self.ctrl_col)
        self.ctrl_lay.setContentsMargins(0, 0, 0, 0)
        self.ctrl_lay.setSpacing(14)
        self.ctrl_card = UI.ControllerCard(MODE_COLOR[MANIP], (
            ("hand", "Hand"), ("motion", "Stick bewegt"), ("speed", "Tempo"),
            ("waist", "Hüfte")))
        self.ctrl_lay.addWidget(self.ctrl_card)
        self.legend_card = UI.LegendCard()
        self.ctrl_lay.addWidget(self.legend_card, 1)
        self.row.add(self.ctrl_col, 4)
        self.ctrl_col.hide()

        right = QWidget()
        rcol = QVBoxLayout(right)
        rcol.setContentsMargins(0, 0, 0, 0)
        rcol.setSpacing(14)
        self.rcol = rcol
        # Pose-Schnellwahl (nur Controller-Ansicht): die 16 Plaetze R2 +
        # Steuerkreuz 2x, Wahl am Controller leuchtet, Klick faehrt an.
        self.quick_card = UI.Card("Pose-Schnellwahl", "layout-grid", UI.WAIST_COLOR,
                                  "R2 halten · Steuerkreuz 2× · R2 los")
        self.quick_grid = UI.QuickGrid(on_pick=lambda name: self.gui.run_poses([name]))
        self.quick_card.body.addWidget(self.quick_grid)
        self.btn_quick_edit = UI.big_button("Plätze belegen …", font=14, height=38,
                                            icon="square-pen")
        self.quick_card.body.addWidget(self.btn_quick_edit)
        rcol.addWidget(self.quick_card)
        self.quick_card.hide()
        # Karte »Abläufe«: Am Stueck | Schritt fuer Schritt, »Weiter« (nur wenn
        # ein Ablauf auf Bestaetigung wartet; Controller: L3), Raster mit Suche.
        self.seq_card = UI.Card("Abläufe", "play", T.ACCENT)
        ctl = QHBoxLayout()
        ctl.setSpacing(10)
        w, self.seq_mode_btns = UI.segmented([sr.MODE_AUTO, sr.MODE_STEP], None, font=14,
                                             height=42)
        for key, b in self.seq_mode_btns.items():
            b.setText(sr.MODE_TITLE[key])
            b.clicked.connect(lambda _=False, k=key: self.gui.set_sequence_mode(k))
        ctl.addWidget(w, 3)
        self.btn_next = UI.big_button("Weiter", "#2e7d32", font=16, height=42)
        ui_icons.set_icon(self.btn_next, "chevron-right", "#ffffff", 20)
        self.btn_next.setLayoutDirection(Qt.LayoutDirection.RightToLeft)   # Pfeil rechts
        self.btn_next.setToolTip("Nächsten Schritt des Ablaufs anfahren (Controller: L3)")
        ctl.addWidget(self.btn_next, 2)
        self.seq_card.body.addLayout(ctl)
        self.seq_grid = UI.TileGrid(tile_w=170, placeholder="Ablauf suchen …")
        self.seq_row = self.seq_grid.grid     # Kompatibilitaet: Knoepfe der Ablaeufe
        self.seq_card.body.addWidget(self.seq_grid, 1)
        self.btn_sequences = UI.big_button("Posen && Abläufe bearbeiten …", font=14, height=40,
                                           icon="square-pen")
        self.seq_card.body.addWidget(self.btn_sequences)
        rcol.addWidget(self.seq_card, 1)

        # Karte »Posen«
        self.pose_card = UI.Card("Posen", "bot", MODE_COLOR[MANIP])
        grid = QGridLayout()
        grid.setSpacing(8)
        self.btn_safe = UI.big_button("Sichere Pose", font=14, height=42, icon="shield-check")
        self.btn_home = UI.big_button("Grundstellung", font=14, height=42, icon="anchor")
        self.btn_goto = UI.big_button("Pose anfahren …", font=14, height=42, icon="folder-open")
        self.btn_save = UI.big_button("Pose speichern …", font=14, height=42, icon="save")
        for i, b in enumerate((self.btn_safe, self.btn_home, self.btn_goto, self.btn_save)):
            grid.addWidget(b, i // 2, i % 2)
        self.pose_card.body.addLayout(grid)
        self.btn_marker = QPushButton("RViz-Marker folgen")
        self.btn_marker.setCheckable(True)
        self.btn_marker.setChecked(True)   # Default des interactive_marker-Node
        self.btn_marker.setToolTip("Arme folgen dem interaktiven Marker in RViz")
        self.btn_marker.setStyleSheet(f"""
            QPushButton {{ background:transparent; color:{T.MUTED}; border:none;
                           font-size:14px; text-align:left; padding:2px; }}
            QPushButton:checked {{ color:{T.FG}; }}""")
        self.btn_marker.toggled.connect(self._marker_icon)
        self._marker_icon(True)
        self.pose_card.body.addWidget(self.btn_marker)
        rcol.addWidget(self.pose_card)
        self.row.add(right, 4)
        self.controller_view = False

        self.show_view("arms")
        # Fingersteuerung gibt es nur mit Inspire-Haenden. In der Sim mit Rubber-
        # oder eigenem starren Profil (G1_HANDS) ist HAENDE ausgegraut; real hat
        # der G1 immer Inspire-Haende.
        self.hands_available = hands_controllable()
        hb = self.view_btns["hands"]
        hb.setEnabled(self.hands_available)
        if not self.hands_available:
            hb.setToolTip(f"Starre Hände ({hand_profile()}): keine Finger zum Bewegen. "
                          "Fingersteuerung nur mit Inspire-Händen (Startmenü → Hände).")

    def set_controller_view(self, on):
        """Controller-Ansicht: Joysticks/Huefte-Schieber und HAENDE-Seite weg
        (macht man am Controller), dafuer Controller-Zustand, Belegung und
        Pose-Schnellwahl. Abläufe, Posen und »Bewegung stoppen« bleiben."""
        on = bool(on)
        self.controller_view = on
        self.jog.reset()
        self.waist_rocker.mouseReleaseEvent(None)     # Schieber los -> Huefte steht
        if on:
            self.show_view("arms")
            self.ctrl_lay.insertWidget(1, self.pose_card)
        else:
            self.rcol.addWidget(self.pose_card)
        self.view_switch.setVisible(not on)
        self.jog_card.setVisible(not on)
        self.ctrl_col.setVisible(on)
        self.quick_card.setVisible(on)
        self.updateGeometry()

    def _marker_icon(self, on):
        ui_icons.set_icon(self.btn_marker, "check" if on else "square",
                          T.ACCENT if on else T.MUTED, 18)

    def show_view(self, key):
        if key == "hands" and not getattr(self, "hands_available", True):
            key = "arms"
        self.views.setCurrentWidget(self.view_pages[key])
        self.views.updateGeometry()
        for k, b in self.view_btns.items():
            b.setChecked(k == key)

    def fill_sequence_buttons(self, sequences):
        """sequences: [(Ablauf, [Posennamen ...]), ...] (leere Ablaeufe ohne Knopf)."""
        sequences = [(label, names) for label, names in sequences if names]
        btns = []
        for label, names in sequences:
            b = UI.big_button(label, UI.SEQ_COLOR, font=15, height=56)
            b.setToolTip(f"{len(names)} Schritte: "
                         + " → ".join(pretty_pose_name(n) for n in names))
            b.clicked.connect(lambda _, lb=label: self.gui.run_sequence(lb))
            b.default_css = b.styleSheet()
            btns.append(b)
        self.seq_buttons = {label: b for (label, _), b in zip(sequences, btns)}
        self.seq_grid.set_widgets(
            btns, "Noch keine Abläufe — »Posen & Abläufe bearbeiten …« öffnen, Ablauf "
                  "anlegen und Posen hinzufügen.")
        self.seq_card.set_hint(f"{len(btns)} gespeichert" if btns else "")
        self._style_sequences(getattr(self, "_running", None))

    def _style_sequences(self, running):
        """Laufender Ablauf: gestrichelt (wie »wechselt«)."""
        self._running = running
        for label, b in getattr(self, "seq_buttons", {}).items():
            if label == running:
                b.setStyleSheet(UI.state_css(UI.SEQ_COLOR, "white", "3px dashed white", 15))
            else:
                b.setStyleSheet(b.default_css)

    def show_sequence(self, runner):
        """Modus-Knoepfe, »Weiter« und laufender Ablauf nach dem Runner-Stand."""
        for k, b in self.seq_mode_btns.items():
            b.setChecked(k == runner.mode)
        waiting = runner.state == sr.WAITING
        self.btn_next.setEnabled(waiting)
        if waiting:
            self.btn_next.setText(f"Weiter  ({runner.index + 2}/{runner.total})")
        elif runner.active and runner.label:
            self.btn_next.setText(f"Schritt {runner.index + 1}/{runner.total} läuft …")
        else:
            self.btn_next.setText("Weiter")
        self._style_sequences(runner.label if runner.active else None)

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
        self._full_estop_shown = False  # davon VOLL-NOTAUS (ganzer G1 Damp)
        # Ablauf/Einzelpose (sequence_runner.py): wann welcher Schritt losfaehrt.
        self.seq = sr.SequenceRunner(self._seq_goto, self._on_seq_change,
                                     mode=default_sequence_mode(self.sim_mode))
        self.seq_editor = None
        self._move_n = 0
        self._save_wait = {}        # Speicher-ID -> Ablauf, an den angehaengt wird
        self._save_n = 0
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
        self._waist_sending = False   # Hueft-Rate-Strom laeuft (0 beim Loslassen senden)

        node.on_walk_ready = self._on_walk_ready
        node.on_arm_status = self._on_arm_status
        node.on_nav_status = self._on_nav_status
        node.on_emergency_stop = self._external_estop
        node.on_full_emergency_stop = self._enter_full_estop
        node.on_shared = self._on_shared
        node.on_seq_next = self.sequence_next
        node.on_pose_cancel = self._on_pose_cancel
        node.on_save_status = self._on_save_status

        self.setWindowTitle("G1 Demo — " + ("SIM" if self.sim_mode else "REAL"))
        self.setObjectName("demo")
        self.setStyleSheet(f"QWidget#demo {{ background:{T.BG}; }}")
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
        # Controller-Ansicht: Zustand 10 Hz (Schnellwahl leuchtet sofort).
        self.view = UI.VIEW_TOUCH
        self.ctrl_timer = QTimer(self)
        self.ctrl_timer.timeout.connect(self._refresh_controller_view)
        self.set_view(UI.default_view())
        self.seq_timer = QTimer(self)
        self.seq_timer.timeout.connect(self.seq.tick)
        self.seq_timer.start(100)
        self._on_seq_change(self.seq, "init", "")
        self._refresh_info()
        # Handy-Bedienung: ruft dieselben Methoden wie die Knoepfe. Bluetooth
        # startet erst beim Klick auf »📱 Handy« (G1_PHONE=0 = ganz aus).
        self.phone = phone_link.attach(self)

        # Auto-Start NUR in der Sim (Arme an + BALANCING),
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
        """Aufbau (demo_ui.py):
          Kopfleiste    SIM/REAL | Modus GEHEN/GREIFEN | Controller, Handy, Vollbild
          Mitte         Arbeitsflaeche des aktiven Modus (Karten, scrollt)
          rechts        Seitenleiste: NOT-HALT, Roboter (real) bzw. Simulation, Beenden
          unten         Statusleiste mit Verlauf"""
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Kopfleiste ──────────────────────────────────────────────────
        bar = QFrame()
        bar.setObjectName("topbar")
        bar.setStyleSheet(f"QFrame#topbar {{ background:{T.SURFACE}; border:none; }}")
        head = QHBoxLayout(bar)
        head.setContentsMargins(18, 8, 18, 8)
        head.setSpacing(12)
        badge = QLabel("G1 · SIM" if self.sim_mode else "G1 · REAL")
        badge.setStyleSheet(
            "font-size:15px; font-weight:800; letter-spacing:1px; padding:6px 12px;"
            " border-radius:8px; "
            + ("background:#16304f; color:#8db4fa; border:1px solid #24497a;" if self.sim_mode
               else "background:#3a1d20; color:#ff8a80; border:1px solid #6a2a30;"))
        head.addWidget(badge)
        head.addSpacing(6)
        self.tiles = {WALK: ModeTile(WALK), MANIP: ModeTile(MANIP)}
        modes = QHBoxLayout()
        modes.setSpacing(8)
        modes.addWidget(self.tiles[WALK])
        modes.addWidget(self.tiles[MANIP])
        head.addLayout(modes, 3)
        head.addStretch(1)
        # Ansicht: Touch (alles per Klick) | Controller (Joysticks weg, dafuer
        # Controller-Zustand, Belegung, Pose-Schnellwahl). Start: G1_DEMO_VIEW.
        keys = (UI.VIEW_TOUCH, UI.VIEW_CONTROLLER)
        self.view_toggle, vbtns = UI.segmented(["  " + UI.VIEW_TITLE[k] for k in keys], None,
                                               font=14, height=40)
        self.view_btns = {k: vbtns["  " + UI.VIEW_TITLE[k]] for k in keys}
        for k, b in self.view_btns.items():
            ui_icons.set_icon(b, UI.VIEW_ICON[k], "#ffffff", 16)
            b.clicked.connect(lambda _=False, v=k: self.set_view(v))
        self.view_btns[UI.VIEW_TOUCH].setToolTip(
            "Touch-Ansicht: alles per Klick (Joysticks, Pfeiltasten, Hüfte, Hände)")
        self.view_btns[UI.VIEW_CONTROLLER].setToolTip(
            "Controller-Ansicht: was man am Controller macht, fällt weg -- dafür "
            "Controller-Zustand, Belegung und Pose-Schnellwahl. Klicken geht weiter.")
        head.addWidget(self.view_toggle)
        # Rechts: Controller-Belegung (nur wenn ein Controller aktiv ist), Handy
        # (ausser G1_PHONE=0, phone/link.py) und Vollbild -- am Roboter-Rechner
        # fehlt oft die Titelleiste. Tasten: F11 umschalten, Esc verlassen.
        self.btn_pad_help = UI.chip_button("Belegung", "gamepad-2")
        self.btn_pad_help.setVisible(False)
        head.addWidget(self.btn_pad_help)
        self.btn_phone = UI.chip_button("Handy", "smartphone")
        self.btn_phone.setVisible(False)
        head.addWidget(self.btn_phone)
        self.btn_fullscreen = UI.chip_button("")
        head.addWidget(self.btn_fullscreen)
        root.addWidget(bar)
        # Farblinie des aktiven Modus unter der Kopfleiste.
        self.mode_line = QFrame()
        self.mode_line.setFixedHeight(3)
        root.addWidget(self.mode_line)

        main = QHBoxLayout()
        main.setContentsMargins(0, 0, 0, 0)
        main.setSpacing(0)
        root.addLayout(main, 1)

        # ── Arbeitsflaeche ──────────────────────────────────────────────
        work = QVBoxLayout()
        work.setContentsMargins(0, 0, 0, 0)
        work.setSpacing(0)
        main.addLayout(work, 1)
        self.walk_panel = WalkPanel()
        self.manip_panel = ManipPanel(self)
        self.stack = UI.CurrentStack()
        self.idle_panel = QLabel()
        self.idle_panel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.idle_panel.setWordWrap(True)
        self.idle_panel.setStyleSheet(f"color:{T.MUTED}; font-size:22px;")
        for w in (self.idle_panel, self.walk_panel, self.manip_panel):
            self.stack.addWidget(w)
        # Scrollbereich: die Mindesthoehe des Fensters haengt so nicht am
        # groessten Panel (frueher real ~1160 px -> kein Maximieren am 1080er-Schirm).
        holder = QWidget()
        hl = QVBoxLayout(holder)
        hl.setContentsMargins(18, 14, 18, 14)
        hl.addWidget(self.stack)
        self.panel_scroll = QScrollArea()
        self.panel_scroll.setWidgetResizable(True)
        self.panel_scroll.setFrameShape(QFrame.Shape.NoFrame)
        # Waagrecht nur als Notbehelf (sehr schmales Fenster, Hand-Seite).
        self.panel_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.panel_scroll.setWidget(holder)
        work.addWidget(self.panel_scroll, 1)

        # ── Statusleiste ────────────────────────────────────────────────
        sbar = QFrame()
        sbar.setObjectName("statusbar")
        sbar.setStyleSheet(f"QFrame#statusbar {{ background:{T.SURFACE}; border:none;"
                           f" border-top:1px solid {T.LINE}; }}")
        sl = QHBoxLayout(sbar)
        sl.setContentsMargins(18, 8, 18, 8)
        sl.setSpacing(12)
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(40)
        self.status.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        sl.addWidget(self.status, 1)
        self.btn_history = UI.chip_button("Verlauf", "book-open")
        self.btn_history.setToolTip("Die letzten Meldungen anzeigen")
        sl.addWidget(self.btn_history, 0, Qt.AlignmentFlag.AlignVCenter)
        work.addWidget(sbar)

        # ── Seitenleiste ────────────────────────────────────────────────
        side = QFrame()
        side.setObjectName("sidebar")
        side.setFixedWidth(UI.SIDEBAR_W)
        side.setStyleSheet(f"QFrame#sidebar {{ background:{T.SURFACE}; border:none;"
                           f" border-left:1px solid {T.LINE}; }} QLabel {{ background:transparent; }}")
        sv = QVBoxLayout(side)
        sv.setContentsMargins(14, 14, 14, 14)
        sv.setSpacing(12)
        main.addWidget(side)
        # NOT-HALT: immer oben, immer sichtbar (nicht im Scrollbereich).
        self.btn_estop = QPushButton("  NOT-HALT")
        self.btn_estop.setFixedHeight(96)
        self.btn_estop.setStyleSheet("""
            QPushButton { background:#c62828; color:white; font-size:28px; font-weight:900;
                          letter-spacing:1px; border:4px solid #ff8a80; border-radius:48px; }
            QPushButton:hover { background:#d32f2f; }
            QPushButton:pressed { background:#ff1744; }
        """)
        ui_icons.set_icon(self.btn_estop, "octagon-alert", "#ffffff", 30)
        self.btn_estop.setToolTip("Alles weich schalten (Arme, Gehen). Danach neu starten.")
        sv.addWidget(self.btn_estop)
        # VOLL-NOTAUS: ganzer G1 in Damp (auch die Beine) -- derselbe Pfad wie
        # R1 + L1 + PS am Controller. Schutz gegen Antippen: FULL_ESTOP_HOLD_S
        # halten (Knopf fuellt sich). Schon der Druck loest den normalen
        # NOT-HALT aus, das Halten eskaliert nur noch auf die Beine.
        self.btn_full_estop = UI.HoldButton(
            f"  VOLL-NOTAUS\n{FULL_ESTOP_HOLD_S:.1f} s halten".replace(".", ","),
            hold_s=FULL_ESTOP_HOLD_S)
        ui_icons.set_icon(self.btn_full_estop, "triangle-alert", "#ffffff", 24)
        self.btn_full_estop.setToolTip(
            "Gedrückt halten: ganzer G1 in Damp, auch die Beine "
            "(wie R1 + L1 + PS am Controller). Der G1 sackt in den Kran bzw. fällt. "
            "Schon das Drücken löst den normalen NOT-HALT aus; loslassen vor Ablauf "
            "bricht nur die Eskalation ab.")
        self.btn_full_estop.armed.connect(self._full_estop_armed)
        self.btn_full_estop.fired.connect(self.full_emergency_stop)
        self.btn_full_estop.aborted.connect(self._full_estop_aborted)
        sv.addWidget(self.btn_full_estop)
        side_scroll = QScrollArea()
        side_scroll.setWidgetResizable(True)
        side_scroll.setFrameShape(QFrame.Shape.NoFrame)
        side_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        side_scroll.setStyleSheet(f"QScrollArea {{ background:{T.SURFACE}; }}")
        side_body = QWidget()
        side_body.setStyleSheet(f"background:{T.SURFACE};")
        sb = QVBoxLayout(side_body)
        sb.setContentsMargins(0, 0, 4, 0)
        sb.setSpacing(8)
        side_scroll.setWidget(side_body)
        sv.addWidget(side_scroll, 1)

        # Real: Startleiste Start | Damp | Preparation | Walk + Rueckmeldung des
        # loco_client (Onboard-Modus). Start = /g1pilot/start: gibt den Modus-
        # Umschalter frei und quittiert einen NOT-HALT (arm_controller,
        # loco_client, Controller). Freigabe der Onboard-Schritte je nach
        # Onboard-Modus (status_line.start_buttons), Damp aus dem Stehen und
        # Walk brauchen einen zweiten Klick.
        self.start_bar = QWidget()
        bl = QVBoxLayout(self.start_bar)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.setSpacing(8)
        bl.addWidget(UI.section("Startleiste · Onboard"))
        self.info_onboard = QLabel()
        self.info_onboard.setWordWrap(True)
        self.info_onboard.setVisible(not self.sim_mode)
        bl.addWidget(self.info_onboard)
        self.btn_start = UI.big_button("Start", "#e65100", font=17, height=52)
        bl.addWidget(self.btn_start)
        self.start_btns = {}
        for i, step in enumerate(status_line.START_STEPS):
            b = UI.big_button(status_line.START_LABEL[step], "#37474f", font=15, height=46)
            b.setToolTip(f"Schritt {i + 1} von {len(status_line.START_STEPS)}: "
                         f"{status_line.START_LABEL[step]} am Roboter anfordern")
            b.clicked.connect(lambda _=False, s=step: self._start_step(s))
            bl.addWidget(b)
            self.start_btns[step] = b
        self._start_armed = None      # Schritt, der auf den zweiten Klick wartet
        self._start_armed_n = 0
        self._last_note = None
        self.start_bar.setVisible(not self.sim_mode)
        sb.addWidget(self.start_bar)
        self._update_start_button()

        # Real: Akku + Temperaturen (in der Sim kein Messwert -> aus).
        self.temp_box = QWidget()
        tl = QVBoxLayout(self.temp_box)
        tl.setContentsMargins(0, 8, 0, 0)
        tl.setSpacing(8)
        tl.addWidget(UI.section("Akku · Temperaturen"))
        self.temp_panel = TempPanel(self.node)
        tl.addWidget(self.temp_panel)
        self.temp_box.setVisible(not self.sim_mode)
        self.temp_panel.setVisible(not self.sim_mode)
        if self.sim_mode:
            self.temp_panel.timer.stop()
        sb.addWidget(self.temp_box)

        # Sim: Werkzeuge rund um den simulierten Roboter.
        self.sim_box = QWidget()
        xl = QVBoxLayout(self.sim_box)
        xl.setContentsMargins(0, 0, 0, 0)
        xl.setSpacing(8)
        xl.addWidget(UI.section("Simulation"))
        # Roboter nach NOT-HALT/Sturz zurueck an den Startpunkt (HOLD, gehalten
        # wie beim Start) -> danach Modus waehlen, steht sanft auf.
        self.btn_reset_robot = UI.big_button("Roboter zurücksetzen", font=15, height=46,
                                             icon="refresh-cw")
        self.btn_reset_robot.setToolTip("Roboter zurück an den Startpunkt (gehalten); "
                                        "danach Modus wählen")
        # Stoer-Test: schubst den Roboter -> zeigt, dass er sich faengt.
        self.btn_push = UI.big_button(PUSH_LABEL, font=15, height=46, icon="move")
        self.btn_push.setToolTip("Kurz tippen = Standard-Schubs, halten = stärker")
        self.btn_reset_scene = UI.big_button("Szene zurücksetzen", font=15, height=46, icon="box")
        self.btn_reset_scene.setToolTip("Objekte der Szene an ihre Startplätze")
        for b in (self.btn_reset_robot, self.btn_push, self.btn_reset_scene):
            b.setVisible(self.sim_mode)
            xl.addWidget(b)
        self.sim_box.setVisible(self.sim_mode)
        sb.addWidget(self.sim_box)
        sb.addStretch(1)

        # Ganzen Stack beenden (alle Fenster zu). Zweiter Klick bestaetigt.
        # Real: vorher Laufen stoppen und Arme sicher abgeben (_quit_real_begin).
        self._quit_label = "Sim beenden" if self.sim_mode else "Real beenden"
        self.btn_quit = UI.big_button(self._quit_label, font=15, height=48)
        self.btn_quit.setStyleSheet(UI.button_css("transparent", "#ff8a80", 15,
                                                  border="#5a2a30"))
        ui_icons.set_icon(self.btn_quit, "power", "#ff8a80", 18)
        self._quit_css = self.btn_quit.styleSheet()
        self._quit_armed = 0        # >0: erster Klick erfolgt (Zaehler gegen alte Timer)
        self._real_quit = 0         # >0: »Real beenden« laeuft (Zaehler gegen alte Timer)
        sv.addWidget(self.btn_quit)
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
            self.btn_fullscreen.setText("Vollbild beenden")
            ui_icons.set_icon(self.btn_fullscreen, "minimize-2", "#e8eaed", 16)
            self.btn_fullscreen.setToolTip("Vollbild verlassen (Esc oder F11)")
        else:
            self.btn_fullscreen.setText("Vollbild")
            ui_icons.set_icon(self.btn_fullscreen, "maximize", "#e8eaed", 16)
            self.btn_fullscreen.setToolTip("Vollbild (F11), verlassen mit Esc")

    def changeEvent(self, event):
        # Auch wenn der Fenstermanager das Vollbild umschaltet, Text nachziehen.
        if event.type() == QEvent.Type.WindowStateChange:
            self._update_fullscreen_button()
        super().changeEvent(event)

    @staticmethod
    def _info_css(color):
        return (f"color:{color}; font-size:14px; font-weight:700; padding:6px 10px;"
                f"background:{T.BG}; border-left:4px solid {color}; border-radius:6px;")

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
        wt = status_line.waist_text(n.waist, None if n.waist_t is None else now - n.waist_t)
        mp = self.manip_panel
        mp.waist_label.setText(wt[0] if wt else "Hüfte: keine Daten")
        mp.waist_label.setStyleSheet(f"color:{wt[1] if wt else '#8f97a3'}; font-size:15px; "
                                     "font-weight:700;")
        mp.btn_waist_center.setEnabled(bool(wt) and not wt[0].startswith("Hüfte Mitte"))
        age = None if n.ps4_t is None else now - n.ps4_t
        # Controller: Zustand (Modus, Hand, Tempo) zeigt die GUI selbst; hier
        # nur der Knopf zur Belegung, solange ein Controller aktiv ist.
        res = status_line.ps4_text(n.ps4, age, sim=self.sim_mode, walk=n.ps4walk,
                                   device=n.ps4dev)
        # In der Controller-Ansicht steht die Belegung schon auf der Flaeche.
        self.btn_pad_help.setVisible(res is not None
                                     and getattr(self, "view", None) != UI.VIEW_CONTROLLER)
        self._show_quickslot(res)
        if res is not None:
            name = (n.ps4dev or {}).get("name")
            label = status_line.controller_label(n.ps4dev)
            self.btn_pad_help.setToolTip((f"{name}\n" if name else "")
                                         + f"Belegung des {label}-Controllers")

    # ── Ansicht Touch | Controller ─────────────────────────────────────
    def set_view(self, view):
        """Touch- oder Controller-Ansicht (nur Oberflaeche: Modus, Topics und
        Sicherheit bleiben gleich; alle Knoepfe der Ansicht sind klickbar)."""
        view = UI.VIEW_CONTROLLER if view == UI.VIEW_CONTROLLER else UI.VIEW_TOUCH
        on = view == UI.VIEW_CONTROLLER
        self.view = view
        for k, b in self.view_btns.items():
            b.setChecked(k == view)
        self.manip_panel.set_controller_view(on)
        self.walk_panel.set_controller_view(on)
        if on:
            self.btn_pad_help.setVisible(False)
            self._reload_quick_slots()
            self._refresh_controller_view()
            self.ctrl_timer.start(100)
        else:
            self.ctrl_timer.stop()
        self.stack.updateGeometry()

    def _reload_quick_slots(self):
        store = self._pose_store()
        try:
            slots = store.list_quickslots() if store is not None else {}
        except Exception:   # noqa: BLE001 -- Anzeige, Datei evtl. gerade im Schreiben
            return
        self.manip_panel.quick_grid.set_slots(slots)
        self._quick_reload_t = time.monotonic()

    def _refresh_controller_view(self):
        """Controller-Ansicht (10 Hz): Zustand, Werte, Belegung, Schnellwahl."""
        n, now = self.node, time.monotonic()
        age = None if n.ps4_t is None else now - n.ps4_t
        res = status_line.ps4_text(n.ps4, age, sim=self.sim_mode, walk=n.ps4walk,
                                   device=n.ps4dev)
        fresh = res is not None and age is not None and age <= status_line.STALE_S
        st = (n.ps4 or {}) if fresh else {}
        walk = (n.ps4walk or {}) if fresh else {}
        colors = {"manip": MODE_COLOR[MANIP], "walk": MODE_COLOR[WALK],
                  "waist": UI.WAIST_COLOR, "seq": "#66bb6a", "safety": "#ef5350",
                  "other": T.FG}
        groups = status_line.ps4_help_groups(n.ps4)
        mp, wp = self.manip_panel, self.walk_panel
        if self.mode == MANIP or self.mode is None:
            c = mp.ctrl_card
            c.set_line(res)
            sel = st.get("selection")
            c.set_fact("hand", status_line.SELECTION_LABEL.get(sel, "–") if sel else "–",
                       status_line._hex(status_line.COLORS[sel])
                       if sel in status_line.COLORS else T.FG)
            c.set_fact("motion", status_line.MODE_LABEL.get(st.get("mode"), "–"))
            speed = st.get("speed") or "–"
            if st.get("speed_req") not in (None, st.get("speed")):
                speed += " (max.)"
            c.set_fact("speed", speed)
            wt = status_line.waist_text(n.waist, None if n.waist_t is None else now - n.waist_t)
            c.set_fact("waist", wt[0].replace("Hüfte ", "", 1) if wt else "–",
                       wt[1] if wt else T.FG)
            mp.legend_card.set_groups(
                [g for g in groups if g[2] in ("manip", "waist", "seq", "safety")], colors)
            quick = st.get("quick") if st.get("waist_mode") else None
            mp.quick_grid.highlight(quick)
            if now - getattr(self, "_quick_reload_t", 0.0) > 2.0:
                self._reload_quick_slots()      # Zuordnung im Editor geaendert
        if self.mode == WALK:
            c = wp.ctrl_card
            c.set_line(res)
            if walk.get("active") and not walk.get("estop"):
                c.set_fact("state", "läuft (L2 gehalten)", MODE_COLOR[WALK])
            elif fresh:
                c.set_fact("state", "steht · L2 halten = laufen")
            else:
                c.set_fact("state", "–")
            c.set_fact("speed", walk.get("speed") or wp.speed_name)
            wp.legend_card.set_groups([g for g in groups if g[2] in ("walk", "safety")], colors)

    def _show_quickslot(self, res):
        """Pose-Schnellwahl am Controller (R2 + Steuerkreuz 2x): Wahl in der
        Statuszeile zeigen (nur bei Aenderung, ueberschreibt sonst nichts)
        und das Feld im Fenster »Posen & Abläufe« aufleuchten lassen."""
        st = self.node.ps4 or {}
        quick = st.get("quick") if res is not None and st.get("waist_mode") else None
        busy = res is not None and st.get("qs_busy")
        key = (json.dumps(quick, sort_keys=True) if quick else "", bool(busy))
        if key != getattr(self, "_quick_shown", ("", False)):
            self._quick_shown = key
            if quick or busy:
                self._status(res[0], res[1])
        ed = self.seq_editor
        if ed is not None and ed.isVisible():
            ed.quick.highlight(quick)

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

    def _start_step(self, step, confirmed=False):
        """Startleiste: Anforderung an den loco_client (der prueft erneut).
        confirmed=True: Rueckfrage schon anderswo bestaetigt (Handy-Schieber)."""
        n, now = self.node, time.monotonic()
        age = None if n.onboard_t is None else now - n.onboard_t
        enabled, confirm = status_line.start_buttons(n.onboard, age)[step]
        if not enabled:
            return
        if confirm and not confirmed and self._start_armed != step:
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
        """Fenster »Controller-Belegung«: Tasten als Chips, nach Bereichen in Karten."""
        from g1pilot.teleoperation import dialog_style as DS
        label = status_line.controller_label(self.node.ps4dev)
        colors = {"manip": MODE_COLOR[MANIP], "walk": MODE_COLOR[WALK], "waist": "#ab47bc",
                  "seq": "#66bb6a", "safety": "#ef5350", "other": DS.FG}
        dlg = QDialog(self)
        dlg.setWindowTitle(f"{label}-Controller: Belegung")
        dlg.setStyleSheet(DS.DIALOG_CSS)
        dlg.resize(1080, 760)
        outer = QVBoxLayout(dlg)
        outer.setContentsMargins(20, 18, 20, 16)
        outer.setSpacing(12)
        head = QHBoxLayout()
        head.setSpacing(14)
        head.addWidget(DS.icon_box("gamepad-2", DS.ACCENT, 24, 46))
        col = QVBoxLayout()
        col.setSpacing(2)
        t = QLabel(f"{label}-Controller: Belegung")
        t.setStyleSheet("font-size:20px; font-weight:800;")
        col.addWidget(t)
        name = (self.node.ps4dev or {}).get("name", "")
        sub = QLabel((f"{name} · " if name else "") + "Modus, Hand-Auswahl und Tempo sind mit "
                     "dieser GUI gekoppelt: was am Controller gewählt wird, zeigt die GUI "
                     "und umgekehrt.")
        sub.setWordWrap(True)
        sub.setStyleSheet(f"color:{DS.MUTED}; font-size:13px;")
        col.addWidget(sub)
        head.addLayout(col, 1)
        outer.addLayout(head)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        body.setObjectName("dbody")
        grid = QGridLayout(body)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(12)
        groups = status_line.ps4_help_groups(self.node.ps4)
        cols = [[], []]
        h = [0, 0]
        for g in groups:                     # in die kuerzere Spalte
            i = 0 if h[0] <= h[1] else 1
            cols[i].append(g)
            h[i] += len(g[3]) + 2
        for c, gs in enumerate(cols):
            vb = QVBoxLayout()
            vb.setSpacing(12)
            for title, icon, ck, rows in gs:
                box, cl = DS.card(title, icon, colors.get(ck, DS.FG))
                rg = QGridLayout()
                rg.setHorizontalSpacing(14)
                rg.setVerticalSpacing(8)
                for r, (key, what) in enumerate(rows):
                    rg.addWidget(DS.key_chip(key, colors.get(ck, DS.ACCENT)), r, 0,
                                 Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
                    w = QLabel(what)
                    w.setWordWrap(True)
                    w.setStyleSheet("font-size:14px;")
                    rg.addWidget(w, r, 1)
                rg.setColumnStretch(1, 1)
                cl.addLayout(rg)
                vb.addWidget(box)
            vb.addStretch(1)
            grid.addLayout(vb, 0, c)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        scroll.setWidget(body)
        outer.addWidget(scroll, 1)
        if label == "Xbox":
            note = QLabel("Xbox: Tasten nach Position wie PS4 (A = Kreuz, B = Kreis, X = Viereck, "
                          "Y = Dreieck, LB/RB = L1/R1, LT/RT = L2/R2, View = Share, "
                          "Menu = Options, Xbox-Taste = PS).")
            note.setWordWrap(True)
            note.setStyleSheet(f"color:{DS.MUTED}; font-size:13px;")
            outer.addWidget(note)
        row = QHBoxLayout()
        row.addStretch(1)
        close = QPushButton("Schließen")
        close.clicked.connect(dlg.close)
        row.addWidget(close)
        outer.addLayout(row)
        self._pad_dialog = dlg               # Referenz halten (nicht modal)
        dlg.show()

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
        self.btn_history.clicked.connect(self.show_history)
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
        mp.btn_waist_center.clicked.connect(
            lambda: n.publish_bool(n.pub_waist_center, True))
        mp.btn_goto.clicked.connect(self._pose_goto_dialog)
        mp.btn_save.clicked.connect(lambda: self.save_current_pose())
        mp.btn_next.clicked.connect(self.sequence_next)
        mp.btn_sequences.clicked.connect(self.open_sequence_editor)
        mp.btn_quick_edit.clicked.connect(lambda: self.open_sequence_editor(tab=1))
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
        full = self._full_estop_shown
        self._estop_unacked = False
        self._estop_shown = False
        self._full_estop_shown = False
        self._update_start_button()
        if full:
            # loco_client quittiert den Voll-Notaus nur (kein Aufstehen).
            self._status("VOLL-NOTAUS quittiert — der G1 bleibt in Damp. Aufstehen: "
                         "Startleiste Preparation, dann Walk.", "#ffb300")
        else:
            self._status("Gestartet. Jetzt einen Modus wählen.", "#aaa")
        self._refresh()

    def _update_start_button(self):
        """Start-Knopf in der Startleiste: orange und frei, solange START fehlt
        (nach dem Hochfahren oder nach NOT-HALT), sonst grau »Gestartet« mit Haken."""
        need = not self.started
        self.btn_start.setEnabled(need)
        if not need:
            self.btn_start.setText("Gestartet")
            ui_icons.set_icon(self.btn_start, "check", "#9e9e9e", 22)
            self.btn_start.setToolTip("START ist erfolgt. Nach einem NOT-HALT "
                                      "hier erneut quittieren.")
        elif self._full_estop_shown:
            self.btn_start.setIcon(QIcon())
            self.btn_start.setText("VOLL-NOTAUS\nquittieren")
            self.btn_start.setToolTip("Quittiert den VOLL-NOTAUS. Der G1 bleibt in Damp; "
                                      "Aufstehen danach über Preparation, dann Walk.")
        elif self._estop_shown:
            self.btn_start.setIcon(QIcon())
            self.btn_start.setText("NOT-HALT\nquittieren")
            self.btn_start.setToolTip("Quittiert den NOT-HALT (Arme, Gehen, Controller) "
                                      "und gibt die Modus-Kacheln wieder frei.")
        else:
            self.btn_start.setIcon(QIcon())
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
            self._full_estop_shown = False
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
        color = MODE_COLOR.get(shown, T.LINE)
        self.mode_line.setStyleSheet(f"background:{color}; border:none;")
        if self.mode is None:
            self.stack.setCurrentWidget(self.idle_panel)
            self.idle_panel.setText(self._idle_text())
        else:
            self.stack.setCurrentWidget(self.walk_panel if self.mode == WALK else self.manip_panel)
        self.stack.updateGeometry()

    def _idle_text(self):
        """Arbeitsflaeche ohne Modus: was als Naechstes zu tun ist."""
        if self.pending is not None:
            return f"{MODE_TITLE[self.pending]} wird eingeschaltet …"
        if not self.started:
            if self._full_estop_shown:
                return ("VOLL-NOTAUS aktiv — der G1 ist in Damp.\nRechts »VOLL-NOTAUS quittieren«, "
                        "dann Preparation und Walk in der Startleiste.")
            if self._estop_shown:
                return "NOT-HALT aktiv.\nRechts »NOT-HALT quittieren«, dann oben einen Modus wählen."
            return "Zuerst rechts »Start« drücken,\ndann oben GEHEN oder GREIFEN wählen."
        return "Oben einen Modus wählen: GEHEN oder GREIFEN."

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
        # Dauerhaft senden: ausserhalb von GEHEN immer 0.
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
        """-> [(Ablauf, [Posennamen])] aus dem pose_store (fuer Knoepfe/Handy)."""
        store = self._pose_store()
        if store is None:
            return []
        try:
            return [(name, [sr.step_name(st) for st in steps])
                    for name, steps in store.list_sequences()]
        except Exception as e:   # noqa: BLE001
            self.node.get_logger().warn(f"Ablaeufe nicht lesbar: {e}")
            return []

    def _reload_sequence_buttons(self):
        self.manip_panel.fill_sequence_buttons(self._sequences())
        self._reload_quick_slots()
        if self.seq_editor is not None and self.seq_editor.isVisible():
            self.seq_editor.reload()

    # Kompatibilitaet (Handy, Tests): laufende Pose / verbleibende Schritte.
    @property
    def current_pose(self):
        return self.seq.current_pose

    @property
    def queue(self):
        if not self.seq.active:
            return []
        return [sr.step_name(st) for st in self.seq.steps[self.seq.index + 1:]]

    def run_sequence(self, name):
        """Ablauf `name` aus dem pose_store starten (Knopf, Editor, Handy).
        -> True, wenn er losfaehrt."""
        store = self._pose_store()
        steps = store.get_sequence(name) if store is not None else None
        if not steps:
            self._status(f"Ablauf »{name}« ist leer oder fehlt.", "#ffb300")
            return False
        missing = store.missing_poses(steps)
        if missing:
            self._status(f"Ablauf »{name}«: Pose »{missing[0]}« fehlt — in »Posen & "
                         "Abläufe« ersetzen oder neu speichern.", "#ef5350")
            return False
        return self.run_poses(steps, name)

    def run_poses(self, names, label=None):
        """Eine oder mehrere Posen nacheinander anfahren (Schritte als Namen oder
        {"pose", "wait_s"}). Weiter je nach Modus von selbst oder auf »Weiter«.
        -> True, wenn der erste Schritt losgeschickt wurde."""
        if self.mode != MANIP or self.pending is not None or not names:
            if names and self.mode != MANIP:
                self._status("Posen und Abläufe nur im Modus GREIFEN.", "#ffb300")
            return False
        return self.seq.start(names, label)

    def set_sequence_mode(self, mode):
        """Am Stueck / Schritt fuer Schritt (auch waehrend eines Ablaufs)."""
        self.seq.set_mode(mode)
        self.manip_panel.show_sequence(self.seq)

    def sequence_next(self):
        """»Weiter« (GUI, Handy, Controller L3). -> True, wenn es weiterging."""
        if self.mode != MANIP or self.pending is not None:
            return False
        return self.seq.confirm()

    def _seq_goto(self, step):
        """Schritt losschicken. -> None oder Fehlertext (Ablauf endet dann)."""
        self._jog_holds.clear()     # ein Halte-Ziel braeche die Fahrt ab
        if "move" not in step:
            self.node.publish_str(self.node.pub_pose_goto, step["pose"])
            return None
        store = self._pose_store()
        move = store.get_move(step["move"]) if store is not None else None
        if move is None:
            return f"Bewegung »{step['move']}« fehlt."
        cmd, err = relative_move_command(move, self.node.hand_reference,
                                         self.node.torso_yaw())
        if err:
            return err
        self._move_n += 1
        cmd["id"] = f"demo_gui_move_{self._move_n}"
        self.node.publish_str(self.node.pub_arm_command, json.dumps(cmd))
        return None

    def _on_seq_change(self, runner, event, text):
        color = {"done": "#66bb6a", "aborted": "#ef5350", "waiting": "#66bb6a"}.get(
            event, MODE_COLOR[MANIP])
        if text:
            self._status(text, color)
        self.manip_panel.show_sequence(runner)
        try:
            snap = runner.snapshot()
            snap["event"] = event
            self.node.pub_seq_state.publish(String(data=json.dumps(snap, ensure_ascii=False)))
        except Exception:   # noqa: BLE001 -- Anzeige darf nie den Ablauf stoeren
            pass

    def _on_arm_status(self, data):
        self.seq.on_status(data)

    def _on_pose_cancel(self):
        """Stopp von aussen (Controller Kreuz, eigener Knopf):
        auch ein auf »Weiter« wartender Ablauf endet."""
        self.seq.cancel("Stopp")

    def _cancel_sequence(self, reason=""):
        self.seq.cancel(reason)

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
                         "(Karte »Posen« → Pose speichern …).", "#ffb300")
            return
        self.run_poses([SAFE_POSE])

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
        self._waist_tick()
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
        if not self._jog_targets and self.seq.moving:
            # Fahrt laeuft: Jog hat Vorrang (arm_controller bricht sie ab) ->
            # Ablauf beenden. Wartet er auf »Weiter«, darf man nachkorrigieren.
            self._cancel_sequence("Hand bewegt")
            self._status(f"Hand bewegen: {jog.side.lower()} …", MODE_COLOR[MANIP])
        lead_m, lead_rad = lead_limits(jog.lin_speed, jog.ang_speed)
        missing = []
        for side in jog.sides():
            start, hand = self.node.jog_pose(side, side not in self._jog_targets)
            if hand is None:
                missing.append(side)
                continue
            lin_s, ang_s = jog.side_twist(side, lin, ang)
            # Oberkoerper-Bezug: "vor" = Richtung Brust, auch bei gedrehter Huefte.
            lin_s, ang_s = rotate_twist_yaw(lin_s, ang_s, self.node.torso_yaw())
            p, q = jog_step(self._jog_targets.get(side), hand, lin_s, ang_s, dt,
                            lead_m, lead_rad, start=start)
            self._jog_targets[side] = [p, q]
            self._jog_twist[side] = (lin_s, ang_s)
            self.node.publish_hand_goal(side, p, q)
        if missing:
            self._status("Hand bewegen: Handposition unbekannt (TF fehlt) — "
                         "läuft der Roboter-Zustand?", "#ffb300")

    def _waist_tick(self):
        """Huefte drehen (Schieber in »Hand bewegen«), ~30 Hz wie der Controller:
        solange gezogen Rate -1..1 senden (+ = links), beim Loslassen oder
        Verlassen von GREIFEN/ARME einmal 0. Ohne Strom haelt der arm_controller
        die Huefte von selbst an (Totmann)."""
        mp = self.manip_panel
        rate = mp.waist_rocker.value
        allowed = (self.mode == MANIP and self.pending is None
                   and mp.views.currentWidget() is mp.arms_page and not mp.controller_view)
        if not allowed and abs(rate) > 1e-3:
            mp.waist_rocker.mouseReleaseEvent(None)
            rate = 0.0
        n = self.node
        if abs(rate) > 1e-3:
            if not self._waist_sending:
                # Wie am Controller: keine Halte-Ziele (Becken-Bezug) -- sie
                # zoegen die Arme nach dem Drehen an die alte Stelle zurueck.
                self._jog_holds.clear()
                self._waist_sending = True
                self._status("Hüfte dreht …", UI.WAIST_COLOR)
            n.pub_waist_rate.publish(Float32(data=float(max(-1.0, min(1.0, rate)))))
        elif self._waist_sending:
            self._waist_sending = False
            n.pub_waist_rate.publish(Float32(data=0.0))
            if self.mode == MANIP:
                self._status("Hüfte steht.", UI.WAIST_COLOR)

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
            msgbox.info(self, "Keine Posen", "Noch keine Pose gespeichert.")
            return
        dlg = PoseLoadDialog(self, grouped, components_of=store.components)
        if dlg.exec() == QDialog.DialogCode.Accepted and dlg.selected_name():
            self.run_poses([dlg.selected_name()])

    def open_sequence_editor(self, tab=None):
        """Fenster »Posen & Abläufe« (nicht modal: nebenher Hand bewegen).
        tab=1: gleich den Reiter »Controller-Schnellwahl« zeigen."""
        if self.seq_editor is None:
            self.seq_editor = SequenceEditor(self, parent=self)
        self.seq_editor.reload()
        if tab is not None:
            self.seq_editor.tabs.setCurrentIndex(tab)
        self.seq_editor.show()
        self.seq_editor.raise_()
        self.seq_editor.activateWindow()

    def save_current_pose(self, sequence=None):
        """Aktuelle Stellung speichern (Dialog). `sequence`: Ablauf, an den die
        Pose angehaengt wird (vorausgewaehlt, im Dialog aenderbar). Der
        arm_controller speichert; angehaengt wird erst nach seiner Quittung."""
        store = self._pose_store()
        cats = store.list_categories() if store is not None else [DEMO_CATEGORY]
        if DEMO_CATEGORY not in cats:
            cats = [DEMO_CATEGORY] + cats
        seqs = [n for n, _ in store.list_sequences()] if store is not None else []
        parent = self.seq_editor if (self.seq_editor is not None
                                     and self.seq_editor.isVisible()) else self
        dlg = PoseSaveDialog(parent, store=store, categories=cats,
                             sequences=seqs, sequence=sequence)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        self.request_pose_save(dlg.result_data())

    def request_pose_save(self, data):
        """Speichern beim arm_controller anfordern (Dialog, Handy). data =
        {"name", "category", "components", optional "sequence"}; angehaengt an
        den Ablauf wird erst nach der Quittung (_on_save_status)."""
        data = dict(data)
        target = data.pop("sequence", None)
        self._save_n += 1
        data["id"] = f"demo_gui_save_{self._save_n}"
        self._save_wait[data["id"]] = (data["name"], target)
        self.node.publish_str(self.node.pub_pose_save, json.dumps(data))
        self._status(f"Pose »{data['name']}« wird gespeichert …", MODE_COLOR[MANIP])
        # Ohne Quittung (alter arm_controller, Arme aus): trotzdem neu laden.
        QTimer.singleShot(1500, self._reload_sequence_buttons)

    def _on_save_status(self, data):
        """Quittung des arm_controller (/g1pilot/pose_store/save/status)."""
        entry = self._save_wait.pop(data.get("id", ""), None)
        if entry is None:
            return
        name, target = entry
        if data.get("state") != "saved":
            self._status(f"Pose »{name}« NICHT gespeichert: "
                         f"{data.get('reason', data.get('state', '?'))}", "#ef5350")
            return
        text = f"Pose »{name}« gespeichert."
        if target:
            store = self._pose_store()
            try:
                steps = store.get_sequence(target) or []
                # Kategorie "Ablauf <X>" haengt schon im pose_store an -- nicht doppelt.
                if not (steps and steps[-1].get("pose") == name):
                    store.append_step(target, name)
                text = f"Pose »{name}« gespeichert und an Ablauf »{target}« angehängt."
            except Exception as e:   # noqa: BLE001
                text = f"Pose »{name}« gespeichert, Anhängen an »{target}« fehlgeschlagen: {e}"
        self._status(text, "#66bb6a")
        self._reload_sequence_buttons()

    # ── Sicherheit / Status ─────────────────────────────────────────────
    def _pulse(self, pub, duration=1000):
        """Bool-Impuls True -> False (Taster-Verhalten)."""
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
            self.btn_push.setText(f"Schubs {lvl*100:.0f} % · {f:.0f} N")
            self._status(f"Schubs aufgeladen: {lvl*100:.0f} % ({f:.0f} N) – loslassen zum Schubsen",
                         "#ffb300")

    def _push_release(self):
        """Loslassen: schubst mit der aufgeladenen Kraft (kurzer Klick = Standard-Schubs)."""
        if not self._push_charger.holding:
            return
        self._push_timer.stop()
        lvl = self._push_charger.release()
        self.node.publish_push_charge(lvl)
        self.btn_push.setText(PUSH_LABEL)
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
        self._full_estop_shown = False
        self.mode = self.pending = None
        self.started = True
        self._refresh()
        self._status("Roboter steht wieder am Startpunkt (gehalten) — "
                     "oben einen Modus wählen.", "#8f97a3")

    def emergency_stop(self):
        n = self.node
        for pub in (n.pub_start, n.pub_start_balancing, n.pub_start_walking,
                    n.pub_arms_enabled, n.pub_arms_home):
            n.publish_bool(pub, False)
        n.publish_bool(n.pub_emergency_stop, True)
        self._enter_estop()

    def full_emergency_stop(self):
        """VOLL-NOTAUS (Halte-Knopf): ganzer G1 in Damp. Reihenfolge wie am
        Controller (ps4_arm_teleop »full_estop«): zuerst /g1pilot/full_emergency_stop
        (loco_client: Damp vor StopMove; arm_controller: Taille schlaff), dann
        der normale NOT-HALT (Arme, Controller, andere GUIs sperren). Sim:
        loco_sim daempft beim NOT-HALT ohnehin den ganzen Roboter."""
        n = self.node
        n.publish_bool(n.pub_full_emergency_stop, True)
        for pub in (n.pub_start, n.pub_start_balancing, n.pub_start_walking,
                    n.pub_arms_enabled, n.pub_arms_home):
            n.publish_bool(pub, False)
        n.publish_bool(n.pub_emergency_stop, True)
        self._enter_full_estop()

    def _full_estop_armed(self):
        """Halte-Knopf gedrueckt: sofort normaler NOT-HALT (falls noch nicht
        aktiv), das Halten eskaliert danach auf den ganzen G1."""
        if not self._estop_shown:
            self.emergency_stop()
        if not self._full_estop_shown:
            self._status(f"VOLL-NOTAUS: weiter halten ({FULL_ESTOP_HOLD_S:.1f} s) — "
                         "loslassen bricht ab, NOT-HALT bleibt.".replace(".", ",", 1),
                         "#ef5350")

    def _full_estop_aborted(self):
        if not self._full_estop_shown:
            self._status("VOLL-NOTAUS abgebrochen (zu früh losgelassen) — "
                         "der normale NOT-HALT ist aktiv.", "#ef5350")

    def _enter_full_estop(self):
        """VOLL-NOTAUS anzeigen (eigener Knopf oder R1 + L1 + PS am Controller).
        Ein normaler NOT-HALT kommt zusaetzlich (_enter_estop); hier nur die
        strengere Anzeige: Quittieren laesst den G1 in Damp."""
        if self._full_estop_shown:
            return
        self._full_estop_shown = True
        if not self._estop_shown:
            self._enter_estop()
        self.btn_full_estop.cancel()
        self._update_start_button()
        self._refresh()
        if self.sim_mode:
            self._status("VOLL-NOTAUS aktiv — ganzer Roboter in Damp. Danach "
                         "»Roboter zurücksetzen«.", "#ef5350")
        else:
            self._status("VOLL-NOTAUS aktiv — ganzer G1 in Damp (auch die Beine). "
                         "Quittieren rechts, Aufstehen nur über Preparation → Walk.",
                         "#ef5350")

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
            self.btn_quit.setText("Wirklich? nochmal tippen")
            self.btn_quit.setStyleSheet(state_css(T.CARD_HI, "#ffb300",
                                                  "3px dashed #ffb300", 15))
            QTimer.singleShot(QUIT_CONFIRM_MS,
                              lambda: armed == self._quit_armed and self._disarm_quit())
            return
        self._quit_armed = 0
        self.btn_quit.setEnabled(False)
        if not self.sim_mode:
            self._quit_real_begin()
            return
        self.btn_quit.setText("Sim wird beendet …")
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
        self.btn_quit.setText("Arme werden abgegeben …")
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
        self.btn_quit.setText("Real wird beendet …")
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
        """Meldung in der Statusleiste (und im Verlauf, Wiederholungen nur einmal)."""
        self._status_color = color
        self.status.setText(text)
        self.status.setStyleSheet(
            f"color:{color}; font-size:18px; font-weight:700; padding:4px 14px;"
            f"background:transparent; border-left:5px solid {color};")
        hist = self.__dict__.setdefault("_history", deque(maxlen=HISTORY_LEN))
        if not hist or hist[-1][1] != text:
            hist.append((time.strftime("%H:%M:%S"), text, color))
        dlg = getattr(self, "_history_dialog", None)
        if dlg is not None and dlg.isVisible():
            dlg.fill(hist)

    def show_history(self):
        """Fenster »Meldungen« (die letzten HISTORY_LEN Meldungen)."""
        hist = self.__dict__.setdefault("_history", deque(maxlen=HISTORY_LEN))
        if getattr(self, "_history_dialog", None) is None:
            self._history_dialog = UI.HistoryDialog(self, hist)
        self._history_dialog.fill(hist)
        self._history_dialog.show()
        self._history_dialog.raise_()


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
    if gui.phone is not None:
        gui.phone.stop()
    node.tf_node.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

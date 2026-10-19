#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import unitree_sdk2py.g1.loco.g1_loco_client as _loco_mod
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)

from g1pilot.utils.common import init_dds


# Onboard-FSM-IDs (Unitree G1): 4 = Ready/"Preparation" (Standby), 500/501 = Walk
# (Hauptbetrieb). 500 = Start() im aktuellen Unitree-SDK (Walk, Taille fest),
# 501 = "Walk with waist" (Nils, Real-Test 2026-10-19). Welche ID der Walk-Start
# setzt: Parameter start_fsm_id (Default 501); beide gelten als Walk.
FSM_READY = 4
DEFAULT_WALK_FSM_IDS = (500, 501)
DEFAULT_START_FSM_ID = 501
# Klartext fuer die GUI-Anzeige (/g1pilot/onboard_state). Begriffe wie in der
# Unitree-Handy-App, soweit belegt; unbekannte IDs zeigt die GUI als Zahl.
FSM_NAMES = {0: "Zero Torque", 1: "Damp", 2: "Hocke", 3: "Sitzen",
             4: "Preparation", 500: "Walk", 501: "Walk mit Taille", 702: "Aufstehen",
             706: "Aufstehen"}
ONBOARD_STATE_TOPIC = "/g1pilot/onboard_state"
LOCO_REQUEST_TOPIC = "/g1pilot/loco_request"


# RPC-Fehlercodes (unitree_sdk2py/rpc/internal.py) als Klartext fuer Log/GUI.
RPC_ERR_TEXT = {3102: "Senden fehlgeschlagen", 3103: "API nicht registriert",
                3104: "Zeitueberschreitung, Roboter antwortet nicht",
                3105: "API-Version passt nicht", 3106: "Antwort unlesbar",
                3107: "Lease ungueltig", 3201: "Server-Sendefehler",
                3202: "Server-interner Fehler", 3203: "API vom Roboter nicht unterstuetzt",
                3204: "Parameter abgelehnt", 3205: "Lease verweigert (anderer Client hat Vorrang?)"}


def _rpc_get(client, api_id):
    """-> (Wert oder None, RPC-Code; -1 = Ausnahme/leere Antwort)."""
    try:
        code, data = client._Call(api_id, "{}")
        if code == 0 and data:
            return json.loads(data).get("data"), 0
        return None, int(code) if code else -1
    except Exception:
        return None, -1


def _rpc_get_int(client, api_id):
    return _rpc_get(client, api_id)[0]


class G1LocoClient(Node):
    def __init__(self):
        super().__init__("loco_client")
        self.robot_stopped = False
        self.balanced = False
        self.prev_buttons = {}
        self.prev_axis_last = None
        self.control_arms = False

        # Streamdeck-Walk-Zustand (WALK-Button + virtueller Joystick der UI).
        # walk_enabled wirkt als Radio zu START BALANCING: nur wenn True darf
        # der cmd_vel-Pfad Move() rufen. _move_active merkt sich, ob zuletzt
        # ein Move lief, damit StopMove genau EINMAL gesendet wird (kein Spam).
        self.walk_enabled = False
        self._move_active = False
        self._joy_deadman = False        # PS4-Deadman (Button 8) gedrueckt?
        # RPC-Serialisierung: der E-Stop-Callback laeuft in einer EIGENEN
        # Callback-Gruppe (MultiThreadedExecutor) und kann damit auch WAEHREND
        # eines laufenden entering_balancing feuern. Damit Damp() nicht mit
        # einem in-flight RPC kollidiert, laeuft jeder Roboter-RPC unter
        # diesem Lock (gehalten pro Einzel-Call, nie ueber Schleifen).
        self._rpc_lock = threading.Lock()
        self._estop_group = MutuallyExclusiveCallbackGroup()
        self._sync_group = MutuallyExclusiveCallbackGroup()
        self._joy_stamp = 0.0            # time.monotonic() des letzten Joy-Pakets
        # Abgleich mit dem ONBOARD-Zustand (Handy-App/Fernbedienung schalten den
        # G1 ausserhalb dieses Nodes um): letzte gelesene FSM-ID, Entprellung.
        self._onboard_fsm = None
        self._rpc_err = None             # letzter Lesefehler (Code) oder None
        self._state_group = MutuallyExclusiveCallbackGroup()
        self._balanced_at = 0.0
        self._nonwalk_polls = 0
        self._entering = False
        self._estop_legs_kept = False    # letzter E-Stop liess die Beine stehen
        self._cmd_lock = threading.Lock()
        self._cmd_vel = (0.0, 0.0, 0.0)  # normiert [-1,1] aus /g1pilot/loco_cmd_vel
        self._cmd_stamp = None           # time.monotonic() des letzten Empfangs

        self.declare_parameter('use_robot', True)
        self.use_robot = bool(self.get_parameter('use_robot').value)

        self.declare_parameter('interface', 'eth0')
        interface = self.get_parameter('interface').get_parameter_value().string_value
        self.declare_parameter('arm_controlled', 'both')
        self.arm_controlled = self.get_parameter('arm_controlled').get_parameter_value().string_value
        self.declare_parameter('enable_arm_ui', True)
        self.enable_arm_ui = self.get_parameter('enable_arm_ui').get_parameter_value().bool_value

        self.declare_parameter('ik_use_waist', False)
        self.declare_parameter('ik_alpha', 0.2)
        self.declare_parameter('ik_max_dq_step', 0.05)
        self.declare_parameter('arm_velocity_limit', 2.0)

        # Sicherheit/Gehen (Streamdeck- und PS4-Pfad teilen sich die Limits):
        # loco_cmd_vel kommt normiert [-1,1] von der UI -> Skalierung auf m/s bzw.
        # rad/s passiert HIER. cmd_vel_timeout ist der Deadman: bleibt der Stream
        # aus (GUI zu, Verbindung weg), stoppt der Roboter von selbst.
        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_vyaw', 0.4)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('cmd_vel_rate_hz', 20.0)
        # Beim Node-Start NICHT in den Roboter-Zustand eingreifen: ein stehender
        # G1 wuerde bei Damp() sofort zusammensacken. Nur fuer Bankett-/Haenge-
        # Tests explizit auf true setzen.
        self.declare_parameter('damp_on_init', False)
        # E-Stop: Beine. False (Default) = StopMove, die Beine bleiben unter der
        # Onboard-Regelung (nur Arme/Haende/-> arm_controller werden schlaff).
        # True = zusaetzlich Damp() (alter Stand: der Roboter sackt zusammen).
        self.declare_parameter('estop_damp_legs', False)
        # Move() nur fuer diese Zeit: wir erneuern es mit 20 Hz. Stirbt dieser
        # Node, stoppt der Onboard-Regler selbst (frueher: 864000 s "continous").
        self.declare_parameter('move_duration_s', 1.0)
        # FSM-IDs, in denen der Onboard-Regler gehen kann (Handy "Walk").
        self.declare_parameter('walk_fsm_ids', list(DEFAULT_WALK_FSM_IDS))
        self.declare_parameter('state_sync_hz', 2.0)
        # SetBalanceMode beim Umschalten auf Walk (entering_balancing). Laut
        # Unitree-SDK: 0 = BalanceStand (steht still), 1 = ContinuousGait (tritt
        # dauernd auf der Stelle). 1 ist der alte g1pilot-Wert -- auf Hardware
        # pruefen (docs/70_echtroboter_anleitung.md), dann ggf. 0.
        self.declare_parameter('balance_mode', 1)
        # Preparation -> Walk: false (Default, 2026-10-16) = direkt BalanceStand +
        # Start() wie die Handy-App, nur mit belasteten Fuessen (FSM-Mode 0).
        # true = alte Hoehenrampe aus dem Haenge-Boot (SetStandHeight 0.02..0.2).
        self.declare_parameter('walk_height_ramp', False)
        # RPC-Dienst des Onboard-Reglers. Unitree hat ihn am 2025-06-23 von
        # "loco" in "sport" umbenannt ("to support ai_sport >= 8.2.0.0",
        # unitree_sdk2_python 60e8358). Das Real-Image installiert einen
        # aelteren SDK-Stand mit "loco" -> der G1 hat auf rt/api/loco/request
        # keinen Leser -> jeder RPC endete mit 3102. Hier fest vorgegeben,
        # unabhaengig vom installierten SDK.
        self.declare_parameter('loco_service', 'sport')
        # FSM-ID fuer "Start" (Walk). Altes SDK: Start() = 200, aktuelles
        # (unitree_sdk2_python 82d7dde, 2026-04-20): 500 (Walk, Taille fest).
        # Default 501 = Walk with waist (Nils 2026-10-19). Explizit statt Start().
        self.declare_parameter('start_fsm_id', DEFAULT_START_FSM_ID)

        ik_use_waist = self.get_parameter('ik_use_waist').get_parameter_value().bool_value
        ik_alpha = float(self.get_parameter('ik_alpha').value)
        ik_max_dq_step = float(self.get_parameter('ik_max_dq_step').value)
        arm_vel_lim = float(self.get_parameter('arm_velocity_limit').value)

        if self.use_robot:
            init_dds(interface, self.get_logger())
            service = str(self.get_parameter('loco_service').value)
            sdk_service = getattr(_loco_mod, 'LOCO_SERVICE_NAME', '?')
            # LocoClient.__init__ liest den Namen aus dem Modul -> vor dem
            # Erzeugen setzen (gilt nur fuer diesen Prozess).
            _loco_mod.LOCO_SERVICE_NAME = service
            self.robot = LocoClient()
            self.get_logger().info(
                f"Loco-RPC-Dienst: '{service}' (rt/api/{service}/request); "
                f"installiertes SDK haette '{sdk_service}'.")
            self.robot.SetTimeout(10.0)
            self.robot.Init()
            if bool(self.get_parameter('damp_on_init').value):
                self.robot.SetFsmId(4)
                self.robot.Damp()
                self.get_logger().warn("damp_on_init: Roboter wurde in Damp versetzt.")
            self.current_id = self.get_fsm_id()
            self.current_mode = self.get_fsm_mode()
            self.get_logger().info(f"Current FSM ID: {self.current_id}, Mode: {self.current_mode}")
        else:
            self.robot = None
            self.current_id = 4
            self.current_mode = 0
            self.get_logger().info("use_robot:=false -> Not connecting to robot.")

        # E-Stop in eigener Callback-Gruppe: feuert dank MultiThreadedExecutor
        # auch, waehrend entering_balancing (blockierend) laeuft. Vorher teilte
        # sich alles einen Thread -> der E-Stop musste WARTEN, bis die
        # Balancing-Schleife fertig war.
        self.create_subscription(Bool, '/g1pilot/emergency_stop',
                                 self.emergency_callback, 10,
                                 callback_group=self._estop_group)
        self.create_subscription(Bool, '/g1pilot/start', self.start_callback, 10)
        self.create_subscription(Bool, '/g1pilot/start_balancing', self.start_balancing_callback, 10)
        self.create_subscription(Joy, '/g1pilot/joy', self.joystick_callback, 10)
        # Streamdeck-Walk: dieselben Topics, die in der Sim loco_sim bedient.
        self.create_subscription(Bool, '/g1pilot/start_walking', self.start_walking_callback, 10)
        self.create_subscription(Twist, '/g1pilot/loco_cmd_vel', self.cmd_vel_callback, 10)
        # Startleiste der Demo-GUI: Onboard-Modus ausdruecklich anfordern
        # ("damp" | "preparation" | "walk") -- damit der Start ohne Handy-App geht.
        self.create_subscription(String, LOCO_REQUEST_TOPIC, self.mode_request_callback, 10)
        self._note = ""                  # letzte Rueckmeldung fuer die GUI
        rate_hz = max(1.0, float(self.get_parameter('cmd_vel_rate_hz').value))
        self.create_timer(1.0 / rate_hz, self._cmd_vel_tick)
        sync_hz = float(self.get_parameter('state_sync_hz').value)
        if self.use_robot and sync_hz > 0.0:
            self.create_timer(1.0 / sync_hz, self._sync_onboard_state,
                              callback_group=self._sync_group)

        self.publisher_arms_controlled = self.create_publisher(Bool, '/g1pilot/arms/enabled', 1)
        # Reiner Lesepfad fuer die GUIs: Onboard-Modus (FSM) + Zustand dieses
        # Nodes. TRANSIENT_LOCAL -> eine spaeter gestartete GUI sieht ihn sofort.
        qos_state = QoSProfile(depth=1)
        qos_state.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(String, ONBOARD_STATE_TOPIC, qos_state)
        # Eigene Gruppe: haengt ein RPC (bis SetTimeout), meldet die Anzeige
        # trotzdem weiter -- sonst zeigte die GUI "keine Daten vom loco_client".
        self.create_timer(0.5, self._publish_state, callback_group=self._state_group)
        self.right_gripper_pub = self.create_publisher(String, '/g1pilot/hand_action/right', 1)
        self.left_gripper_pub = self.create_publisher(String, '/g1pilot/hand_action/left', 1)
        self.publisher_homming_arms = self.create_publisher(Bool, '/g1pilot/arms/home', 1)

    def _log_once(self, level, msg, key):
        if getattr(self, key, False):
            return
        setattr(self, key, True)

        logger = self.get_logger()
        if level == "info":
            logger.info(msg)
        elif level == "warn":
            logger.warn(msg)
        elif level == "error":
            logger.error(msg)
        elif level == "debug":
            logger.debug(msg)
        else:
            logger.info(msg)


    def _clear_once(self, key):
        if hasattr(self, key):
            delattr(self, key)

    def _btn_rising(self, msg, idx):
        prev = self.prev_buttons.get(idx, 0)
        return msg.buttons[idx] == 1 and prev == 0

    def _btn_falling(self, msg, idx):
        prev = self.prev_buttons.get(idx, 0)
        return msg.buttons[idx] == 0 and prev == 1

    def _axis_edge(self, cur, prev, val):
        return cur == val and prev != val, cur != val and prev == val

    def _robot_rpc(self, method, *args, **kwargs):
        """Roboter-RPC unter self._rpc_lock. None, wenn kein Roboter."""
        if not self.use_robot or self.robot is None:
            return None
        with self._rpc_lock:
            return getattr(self.robot, method)(*args, **kwargs)

    def _robot_rpc_urgent(self, method, *args, **kwargs):
        """Wie _robot_rpc, aber fuer den Notfall: wartet hoechstens 0.5 s auf einen
        laufenden RPC und sendet dann trotzdem (ein haengender Call darf den
        E-Stop nicht blockieren). Fehler werden gemeldet, nie verschluckt."""
        if not self.use_robot or self.robot is None:
            return None
        got = self._rpc_lock.acquire(timeout=0.5)
        try:
            return getattr(self.robot, method)(*args, **kwargs)
        except Exception as e:
            self.get_logger().error(f"E-STOP-RPC {method} fehlgeschlagen: {e}")
            return None
        finally:
            if got:
                self._rpc_lock.release()

    def _walk_ids(self):
        """Walk-IDs fuer den Abgleich; die eigene Start-ID zaehlt immer dazu
        (sonst gaebe das Polling einen gerade gestarteten Walk wieder auf)."""
        try:
            ids = {int(i) for i in self.get_parameter('walk_fsm_ids').value}
        except Exception:
            ids = set(DEFAULT_WALK_FSM_IDS)
        return ids | {self._start_fsm_id()}

    def _move_robot(self, vx, vy, vyaw):
        """Move() mit begrenzter Dauer (move_duration_s), vom 20-Hz-Takt erneuert."""
        dur = max(0.2, float(self.get_parameter('move_duration_s').value))
        self._robot_rpc("SetVelocity", vx, vy, vyaw, dur)

    def _halt_legs(self, why):
        """Notfall fuer die Beine. StopMove IMMER (auch wenn _move_active schon
        False ist -- sonst lief ein zuletzt kommandiertes Move weiter), Damp nur
        mit estop_damp_legs. Antwort-Codes werden geloggt."""
        self._move_active = False
        code_stop = self._robot_rpc_urgent("SetVelocity", 0.0, 0.0, 0.0, 1.0)
        damp = bool(self.get_parameter('estop_damp_legs').value)
        code_damp = None
        if damp:
            code_damp = self._robot_rpc_urgent("SetFsmId", 1)
        self._estop_legs_kept = not damp
        if damp:
            self.get_logger().warn(f"{why}: StopMove (Code {code_stop}) + Damp (Code {code_damp}).")
        else:
            self.get_logger().warn(
                f"{why}: StopMove (Code {code_stop}); Beine bleiben unter der "
                "Onboard-Regelung, KEIN Damp (estop_damp_legs=false).")

    def _sync_onboard_state(self, force=False):
        """FSM-Zustand des Onboard-Reglers lesen und mit balanced/walk_enabled
        abgleichen. Hintergrund: Handy-App/Fernbedienung stellen den G1 auf
        Preparation/Walk, ohne dass dieser Node es merkt -> balanced blieb False
        und die GUI konnte den G1 nicht bewegen. Umgekehrt: verlaesst der G1 den
        Walk-Zustand (Damp per Fernbedienung), sperrt das hier sofort Move()."""
        if not self.use_robot or self.robot is None or self._entering:
            return None
        if not self._rpc_lock.acquire(timeout=0.3 if force else 0.0):
            return None                 # RPC laeuft -> naechster Takt
        try:
            fsm, code = _rpc_get(self.robot, ROBOT_API_ID_LOCO_GET_FSM_ID)
        finally:
            self._rpc_lock.release()
        if fsm is None:
            if code != self._rpc_err:
                self.get_logger().warn(
                    f"FSM-ID nicht lesbar: RPC-Code {code} "
                    f"({RPC_ERR_TEXT.get(code, 'unbekannt')}).")
            self._rpc_err = code
            return None
        if self._rpc_err is not None:
            self.get_logger().info(f"FSM-ID wieder lesbar (FSM {fsm}).")
        self._rpc_err = None
        self._onboard_fsm = fsm
        walking = fsm in self._walk_ids()
        if walking:
            self._nonwalk_polls = 0
            if not self.balanced and not self.robot_stopped:
                self.balanced = True
                self._balanced_at = time.monotonic()
                self.get_logger().info(
                    f"Onboard-Modus WALK erkannt (FSM {fsm}) -- als balanciert uebernommen.")
            return fsm
        # Kein Walk-Zustand: erst nach 2 Polls und 3 s Schonfrist umschalten
        # (Start() braucht einen Moment, bis die FSM-ID 500/501 meldet).
        self._nonwalk_polls += 1
        if (self.balanced and self._nonwalk_polls >= 2
                and time.monotonic() - self._balanced_at > 3.0):
            self.get_logger().warn(
                f"Onboard-Modus verlassen (FSM {fsm}) -- Gehen gesperrt, "
                "StopMove. Neu: START BALANCING / LAUFEN.")
            self.balanced = False
            self.walk_enabled = False
            self._stop_move()
        return fsm

    def _publish_state(self):
        """Onboard-Modus + Node-Zustand als JSON (nur Anzeige, keine Logik)."""
        fsm = self._onboard_fsm
        st = {"fsm": fsm, "name": FSM_NAMES.get(fsm, "") if fsm is not None else "",
              "balanced": bool(self.balanced), "walk": bool(self.walk_enabled),
              "estop": bool(self.robot_stopped), "robot": bool(self.use_robot),
              "note": self._note, "rpc_err": self._rpc_err,
              "walk_fsm": self._start_fsm_id()}
        self.state_pub.publish(String(data=json.dumps(st, sort_keys=True)))

    def get_fsm_id(self):
        if not self.use_robot or self.robot is None:
            return 4
        with self._rpc_lock:
            return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_ID)

    def get_fsm_mode(self):
        if not self.use_robot or self.robot is None:
            return 0
        with self._rpc_lock:
            return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_MODE)

    def emergency_callback(self, msg: Bool):
        if msg.data:
            self._log_once("warn", "EMERGENCY STOP ACTIVATED!", "_e_stop_activated_logged")
            # Flags ZUERST: eine ggf. laufende entering_balancing-Schleife
            # sieht robot_stopped und bricht ab, bevor Damp() durch ist.
            self.robot_stopped = True
            self.balanced = False
            self.walk_enabled = False
            self._halt_legs("EMERGENCY STOP")
            if self.control_arms:
                self.control_arms = False
                self.publisher_arms_controlled.publish(Bool(data=False))
        else:
            self._clear_once("_e_stop_activated_logged")

    def start_callback(self, msg: Bool):
        if self.use_robot and self.robot is not None and msg.data:
            self.walk_enabled = False
            self._stop_move("START/Standby")
            legs_kept = self.robot_stopped and self._estop_legs_kept
            self._estop_legs_kept = False
            self.robot_stopped = False
            self.balanced = False
            # Aktuellen Onboard-Modus lesen (Handy-App/Fernbedienung koennen den
            # G1 schon aufgestellt haben). Steht er bereits (Walk/Preparation),
            # schaltet START NICHT um: frueher sprang ein per App auf Walk
            # gestellter G1 bei »Roboter starten« zurueck nach FSM 4 (Modus-
            # sprung im Stand, Balance aus). Gilt auch beim Quittieren eines
            # E-Stops ohne Damp und fuer Steuerkreuz-oben am alten Joystick.
            fsm = self._sync_onboard_state(force=True)
            if legs_kept:
                self._log_once("info", "E-Stop quittiert: Beine blieben stehen, "
                               "Onboard-Modus unveraendert.", "_estop_ack_logged")
                self._clear_once("_estop_ack_logged")
                return
            if fsm in self._walk_ids():
                self.get_logger().info(f"START: G1 steht schon im Walk-Modus (FSM {fsm}) "
                                       "-- uebernommen, kein Umschalten nach FSM 4.")
                return
            if fsm == FSM_READY:
                self.get_logger().info("START: G1 steht schon in Preparation (FSM 4).")
                return
            if fsm is None:
                self.get_logger().warn("START: Onboard-Modus nicht lesbar -- schalte NICHT "
                                       "blind um (erneut START bzw. Fernbedienung).")
                return
            code = self._robot_rpc("SetFsmId", FSM_READY)
            self.get_logger().info(f"START: FSM {fsm} -> 4 (Preparation/Standby), Code {code}.")

    def start_balancing_callback(self, msg: Bool):
        if not msg.data:
            return
        # Radio-Semantik wie am Streamdeck: BALANCING gewaehlt -> Gehen aus,
        # Roboter bleibt balanciert stehen.
        self.walk_enabled = False
        self._stop_move("START BALANCING")
        if self.balanced:
            self._log_once("info", "Already balanced, no action taken.", "_already_balanced_notice_logged")
            return
        if not self._may_enter_balancing("START BALANCING"):
            return
        self._log_once("info", "Starting balancing procedure...", "_start_balance_req_logged")
        self.entering_balancing(max_height=0.5, step=0.02)
        self._log_once("info", "Balancing procedure completed.", "_balance_completed_logged")

    def _may_enter_balancing(self, what):
        """Hoehenrampe + Start() nur aus Preparation (FSM 4) und ohne E-Stop.
        Frueher startete START BALANCING (= GREIFEN der Demo-GUI) den Walk-
        Regler auch aus Damp/Zero-Torque/Sitzen -- LAUFEN war schon gesperrt."""
        if not self.use_robot or self.robot is None:
            return True
        if self.robot_stopped:
            self.get_logger().warn(f"{what} ignoriert: E-Stop aktiv (erst START).")
            return False
        fsm = self._sync_onboard_state(force=True)
        if self.balanced:
            return False                # per App/Fernbedienung schon auf Walk
        if fsm != FSM_READY:
            self.get_logger().warn(
                f"{what} ignoriert: G1 steht nicht in Preparation (FSM {fsm}) -- erst "
                "START bzw. per Fernbedienung/Handy aufstellen.")
            return False
        return True

    # ── Streamdeck-Walk (WALK-Button + virtueller Joystick der UI) ───────────

    def start_walking_callback(self, msg: Bool):
        if not msg.data:
            if self.walk_enabled:
                self.walk_enabled = False
                self._stop_move("WALK deaktiviert")
            return
        if self.robot_stopped:
            self._log_once("warn", "WALK ignoriert: E-Stop aktiv (erst START).",
                           "_walk_not_balanced_logged")
            return
        if not self.balanced:
            # Hat die Handy-App/Fernbedienung den G1 schon auf Walk gestellt?
            self._sync_onboard_state(force=True)
        if not self.balanced:
            # LAUFEN in der GUI stellt den ECHTEN Modus um: aus Preparation
            # (FSM 4) wie bei GREIFEN per Hoehenrampe + Start() in den Walk-
            # Zustand. Aus Damp/Zero-Torque/Sitzen NIE von selbst aufstehen.
            fsm = self._onboard_fsm if self.use_robot else FSM_READY
            if fsm != FSM_READY:
                self._log_once("warn",
                               f"WALK ignoriert: G1 steht nicht in Preparation/Walk "
                               f"(FSM {fsm}) -- erst START bzw. per Fernbedienung "
                               "aufstellen.", "_walk_not_balanced_logged")
                return
            self.get_logger().info("LAUFEN: stelle den G1 von Preparation auf Walk um ...")
            self.entering_balancing(max_height=0.5, step=0.02)
            if not self.balanced:
                return
        self._clear_once("_walk_not_balanced_logged")
        if not self.walk_enabled:
            self.walk_enabled = True
            self.get_logger().info(
                "WALK aktiviert: loco_cmd_vel steuert jetzt Move() "
                f"(Limits vx={float(self.get_parameter('max_vx').value):.2f} "
                f"vy={float(self.get_parameter('max_vy').value):.2f} "
                f"vyaw={float(self.get_parameter('max_vyaw').value):.2f}).")

    def cmd_vel_callback(self, msg: Twist):
        # Normiert [-1,1] (Konvention der Streamdeck-UI, identisch zur Sim).
        nx = max(-1.0, min(1.0, float(msg.linear.x)))
        ny = max(-1.0, min(1.0, float(msg.linear.y)))
        nz = max(-1.0, min(1.0, float(msg.angular.z)))
        with self._cmd_lock:
            self._cmd_vel = (nx, ny, nz)
            self._cmd_stamp = time.monotonic()

    def _stop_move(self, reason=""):
        """StopMove genau einmal senden (kein Dauerfeuer an die RPC-Schnittstelle)."""
        if not self._move_active:
            return
        self._move_active = False
        if self.use_robot and self.robot is not None:
            try:
                self._robot_rpc("StopMove")
            except Exception as e:
                self.get_logger().error(f"StopMove fehlgeschlagen: {e}")
        if reason:
            self.get_logger().info(f"StopMove ({reason})")

    def _cmd_vel_tick(self):
        """20-Hz-Sicherheitsschleife des Streamdeck-Walk-Pfads.

        Move() laeuft nur solange ALLE Bedingungen gelten (balanciert, nicht
        gestoppt, WALK aktiv, frischer cmd_vel-Stream). Faellt eine weg --
        insbesondere der Deadman-Timeout, wenn die GUI stirbt -- wird einmalig
        StopMove() gesendet. Der PS4-Deadman (Button 8) hat Vorrang, solange
        er gedrueckt ist.
        """
        if self._joy_deadman:
            # PS4-/Nav-Joy: bleibt der Joy-Strom aus (Node tot, Bluetooth weg),
            # gilt der Deadman als losgelassen -> StopMove (frueher lief das
            # letzte Move() weiter).
            timeout = float(self.get_parameter('cmd_vel_timeout').value)
            if time.monotonic() - self._joy_stamp <= timeout:
                return
            self._joy_deadman = False
            self._stop_move("Joy-Timeout")
        if not (self.balanced and not self.robot_stopped and self.walk_enabled):
            self._stop_move()
            return
        with self._cmd_lock:
            (nx, ny, nz), stamp = self._cmd_vel, self._cmd_stamp
        timeout = float(self.get_parameter('cmd_vel_timeout').value)
        stale = stamp is None or (time.monotonic() - stamp) > timeout
        if stale or (abs(nx) < 0.03 and abs(ny) < 0.03 and abs(nz) < 0.03):
            self._stop_move("Deadman-Timeout" if stale and self._move_active else "")
            return
        vx = nx * float(self.get_parameter('max_vx').value)
        vy = ny * float(self.get_parameter('max_vy').value)
        vyaw = nz * float(self.get_parameter('max_vyaw').value)
        if self.use_robot and self.robot is not None:
            try:
                self._move_robot(vx, vy, vyaw)
            except Exception as e:
                self.get_logger().error(f"Move fehlgeschlagen: {e} -> StopMove")
                try:
                    self._halt_legs("Move-Fehler")
                except Exception:
                    pass
                self.robot_stopped = True
                self.balanced = False
                self.walk_enabled = False
                self._move_active = False
                return
        self._move_active = True

    def joystick_callback(self, msg: Joy):
        self._joy_stamp = time.monotonic()
        try:
            if not self.prev_buttons:
                self.prev_buttons = {i: 0 for i in range(len(msg.buttons))}

            if not self.balanced:
                self._log_once("warn", "Robot is not balanced, cannot move.", "_warn_not_balanced_logged")
            else:
                self._clear_once("_warn_not_balanced_logged")

            if self.robot_stopped:
                self._log_once("warn", "Robot is stopped, cannot move.", "_warn_robot_stopped_logged")
            else:
                self._clear_once("_warn_robot_stopped_logged")

            axis_last = msg.axes[-1] if len(msg.axes) else 0.0
            if self.prev_axis_last is None:
                self.prev_axis_last = axis_last

            up_on, up_off = self._axis_edge(axis_last, self.prev_axis_last, -1.0)
            if up_on:
                # Wie START in den GUIs (kein Umschalten, wenn der G1 schon steht).
                self.start_callback(Bool(data=True))

            if self._btn_rising(msg, 0):
                self.control_arms = not self.control_arms
                if self.control_arms:
                    self._log_once("info", "Enabling arm control mode.", "_enable_arm_control_logged")
                    self.publisher_arms_controlled.publish(Bool(data=True))
                else:
                    self._log_once("info", "Disabling arm control mode.", "_disable_arm_control_logged")
                    self.publisher_arms_controlled.publish(Bool(data=False))

            if self._btn_rising(msg, 1):
                if self.control_arms:
                    self._log_once("info", "Moving arms to home position.", "_move_arms_home_logged")
                    self.publisher_homming_arms.publish(Bool(data=True))
                else:
                    self._log_once("warn", "Cannot move arms to home, arm control mode is disabled.", "_warn_move_home_no_control_logged")

            if self._btn_rising(msg, 5):
                self._log_once("warn", "Emergency stop button pressed!", "_e_stop_button_pressed_logged")
                self.robot_stopped = True
                self.balanced = False
                self.walk_enabled = False
                self._halt_legs("EMERGENCY STOP (PS4)")
                self.control_arms = False
                self.publisher_arms_controlled.publish(Bool(data=False))
            if self._btn_falling(msg, 5):
                self._clear_once("_e_stop_button_pressed_logged")

            # Gripper controls
            if msg.axes[4] ==1.0 and self._btn_rising(msg, 3):
                self._log_once("info", "Open right gripper.", "_open_right_gripper_logged")
                self.right_gripper_pub.publish(String(data="open"))
            if msg.axes[4] ==1.0 and self._btn_falling(msg, 3):
                self._log_once("info", "Close right gripper.", "_close_right_gripper_logged")
                self.right_gripper_pub.publish(String(data="close"))

            if msg.axes[4] ==-1.0 and self._btn_rising(msg, 3):
                self._log_once("info", "Open left gripper.", "_open_left_gripper_logged")
                self.left_gripper_pub.publish(String(data="open"))
            if msg.axes[4] ==-1.0 and self._btn_falling(msg, 3):
                self._log_once("info", "Close left gripper.", "_close_left_gripper_logged")
                self.left_gripper_pub.publish(String(data="close"))

            if self._btn_rising(msg, 6):
                # Wie START BALANCING (nur aus Preparation, nicht im E-Stop).
                self.start_balancing_callback(Bool(data=True))

            # PS4-Deadman (Button 8): solange gedrueckt steuert der Controller,
            # der Streamdeck-cmd_vel-Pfad pausiert (_cmd_vel_tick prueft das).
            # StopMove nur auf der fallenden Flanke -- frueher wurde es bei
            # losgelassenem Button in JEDEM Joy-Paket gesendet und haette damit
            # auch ein per Streamdeck kommandiertes Gehen dauerhaft abgewuergt.
            self._joy_deadman = (msg.buttons[8] == 1)
            if self._btn_falling(msg, 8) and not self.robot_stopped and self.balanced:
                self._move_active = True   # sicherstellen, dass _stop_move sendet
                self._stop_move("PS4-Deadman losgelassen")

            if msg.buttons[8] == 1 and not self.robot_stopped and self.balanced:
                max_vx = float(self.get_parameter('max_vx').value)
                max_vy = float(self.get_parameter('max_vy').value)
                max_vyaw = float(self.get_parameter('max_vyaw').value)
                vx = round(msg.axes[1] * -max_vx, 2)
                vy = round(msg.axes[0] * -max_vy, 2)
                yaw = round(msg.axes[2] * -max_vyaw, 2)
                self._log_once("info", f"Moving with vx: {vx}, vy: {vy}, yaw: {yaw}", "_moving_logged")
                if self.use_robot and self.robot is not None:
                    if abs(vx) < 0.03 and abs(vy) < 0.03 and abs(yaw) < 0.03:
                        self._stop_move()
                    else:
                        self._move_robot(vx, vy, yaw)
                        self._move_active = True

            self.prev_buttons = {i: msg.buttons[i] for i in range(len(msg.buttons))}
            self.prev_axis_last = axis_last

        except Exception as e:
            self.get_logger().error(f"Error in joystick_callback: {e}")
            if self.use_robot and self.robot is not None:
                self._halt_legs("Joystick-Fehler")
            self.robot_stopped = True
            self.balanced = False
            self.walk_enabled = False
            self._move_active = False

    def _set_note(self, text, level="info"):
        self._note = text
        # rclpy: je Aufrufstelle nur EINE Log-Stufe -> getrennte Aufrufe.
        log = self.get_logger()
        if level == "error":
            log.error(text)
        elif level == "warn":
            log.warn(text)
        else:
            log.info(text)

    def mode_request_callback(self, msg: String):
        """Startleiste: damp | preparation | walk (wie die Handy-App).

        damp        immer erlaubt (Richtung sicher; am Kran haengend gedacht).
        preparation nur aus Damp oder Walk, nicht im E-Stop -> FSM 4.
        walk        nur aus Preparation, nicht im E-Stop, nur mit belasteten
                    Fuessen -> BalanceStand + Start() (siehe walk_height_ramp).
                    Steht der G1 schon in einem ANDEREN Walk-Modus (z. B. 500
                    per Handy-App), nur Umschalten auf start_fsm_id (501)."""
        req = (msg.data or "").strip().lower()
        if not self.use_robot or self.robot is None:
            return
        was_walk = self.walk_enabled
        self.walk_enabled = False
        self._stop_move(f"Startleiste: {req}")
        if req == "damp":
            self.balanced = False
            code = self._robot_rpc_urgent("SetFsmId", 1)
            self._set_note(f"Damp angefordert (Code {code}).", "warn")
            return
        if self.robot_stopped:
            self._set_note(f"{req}: abgelehnt -- NOT-HALT aktiv, erst quittieren.", "warn")
            return
        fsm = self._sync_onboard_state(force=True)
        if req == "preparation":
            if fsm == FSM_READY:
                self._set_note("Preparation: G1 steht schon in FSM 4.")
                return
            if fsm is None:
                self._set_note("Preparation: Modus nicht lesbar -- nichts umgeschaltet.", "warn")
                return
            if fsm != 1 and fsm not in self._walk_ids():
                # Ablauf am Kran: Einschalten -> Damp -> Preparation. Aus Zero
                # Torque (schlaff) o.ae. nicht direkt hochfahren.
                self._set_note(f"Preparation: abgelehnt -- nur aus Damp oder Walk "
                               f"(FSM {fsm}). Erst Damp.", "warn")
                return
            self.balanced = False
            code = self._robot_rpc("SetFsmId", FSM_READY)
            self._set_note(f"Preparation: FSM {fsm} -> 4 (Code {code}).")
        elif req == "walk":
            target = self._start_fsm_id()
            if fsm in self._walk_ids() and fsm != target:
                # Steht schon (z. B. 500 per Handy-App): nur den Walk-Modus
                # wechseln, StopMove ist oben schon raus. Ein aktives GEHEN der
                # GUI bleibt aktiv (sonst haengt die GUI in LAUFEN ohne Wirkung).
                code = self._start_walk()
                self.walk_enabled = was_walk
                self.balanced = True
                self._balanced_at = time.monotonic()
                self._nonwalk_polls = 0
                self._set_note(f"Walk: FSM {fsm} -> {target} umgeschaltet (Code {code}).")
                return
            if self.balanced:
                self._set_note(f"Walk: G1 ist schon im Walk-Modus (FSM {fsm}).")
                return
            if fsm != FSM_READY:
                self._set_note(f"Walk: abgelehnt -- nur aus Preparation (FSM {fsm}).", "warn")
                return
            self.entering_balancing()
        else:
            self.get_logger().warn(f"Startleiste: unbekannte Anforderung '{req}'.")

    def entering_balancing(self, max_height=0.5, step=0.02):
        if not self.use_robot or self.robot is None:
            self.balanced = True
            self.get_logger().info("Sim balancing done (use_robot:=false).")
            return
        self._entering = True
        try:
            if bool(self.get_parameter('walk_height_ramp').value):
                self._enter_balancing_inner(max_height, step)
            else:
                self._enter_walk_direct()
        finally:
            self._entering = False

    def _start_fsm_id(self):
        try:
            return int(self.get_parameter('start_fsm_id').value)
        except Exception:
            return DEFAULT_START_FSM_ID

    def _start_walk(self):
        """Statt LocoClient.Start(): dessen FSM-ID haengt vom SDK-Stand ab
        (200 alt, 500 aktuell) und es liefert keinen Antwort-Code. Hier
        start_fsm_id (Default 501 = Walk with waist)."""
        return self._robot_rpc("SetFsmId", self._start_fsm_id())

    def _enter_walk_direct(self):
        """Preparation -> Walk wie in der Handy-App: ohne Hoehenrampe. Nur wenn
        die Fuesse Last haben (FSM-Mode 0); haengt der G1 noch straff am Kran
        (Mode != 0) oder kommt keine Antwort: kein Start()."""
        mode = self.get_fsm_mode()
        if mode != 0:
            self._set_note(f"Walk: abgelehnt -- Fuesse ohne Last (FSM-Mode {mode}). "
                           "Kran lockern, dann erneut.", "error")
            return
        if self.robot_stopped:
            return
        self._robot_rpc("BalanceStand", int(self.get_parameter('balance_mode').value))
        code = self._start_walk()
        self.balanced = True
        self._balanced_at = time.monotonic()
        self._nonwalk_polls = 0
        self._set_note(f"Walk: BalanceStand + Start() gesendet (Code {code}).")

    def _enter_balancing_inner(self, max_height, step):
        height = 0.0
        problem = None
        while height < max_height and not self.robot_stopped:
            height += step
            self._robot_rpc("SetStandHeight", height)
            mode = self.get_fsm_mode()
            if mode == 0 and height >= 0.2:
                self._log_once("info", f"Reached max height: {height}", "_balance_reach_height_logged")
                break
            elif mode != 0:
                # mode != 0: Fuesse nicht belastet (haengt?), None: RPC-Fehler.
                problem = mode
                break
        # E-Stop waehrend der Rampe? Dann NICHT BalanceStand/Start nachschieben
        # (Damp() ist bereits raus, der Roboter haengt/sackt kontrolliert).
        if self.robot_stopped:
            self.get_logger().warn("Balancing abgebrochen (robot_stopped/E-Stop).")
            return
        if problem is not None or height < 0.2 - 1e-6:
            # Frueher kam hier trotzdem BalanceStand + Start(): der Walk-Regler
            # lief dann z.B. mit Fuessen in der Luft an. Jetzt: kein Start.
            self.get_logger().error(
                f"Balancing abgebrochen: FSM-Mode {problem} bei Hoehe {height:.2f} "
                "(Fuesse nicht belastet oder keine Antwort) -- KEIN Start(), G1 bleibt "
                "in Preparation. Aufstellen per Fernbedienung/Handy oder Gantry pruefen.")
            return
        self._robot_rpc("BalanceStand", int(self.get_parameter('balance_mode').value))
        self._robot_rpc("SetStandHeight", height)
        self._start_walk()
        self.balanced = True
        self._balanced_at = time.monotonic()
        self._nonwalk_polls = 0


def main(args=None):
    rclpy.init(args=args)
    node = G1LocoClient()
    # MultiThreadedExecutor: der E-Stop (eigene Callback-Gruppe) muss auch
    # waehrend eines blockierenden Callbacks (entering_balancing) durchkommen.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)

from g1pilot.utils.common import init_dds


# Onboard-FSM-IDs (Unitree G1): 4 = Ready/"Preparation" (Standby), 500/501 = Walk
# (Hauptbetrieb). 500 stammt aus LocoClient.Start(); 501 ist aus dem Repo NICHT
# belegt -> per Parameter walk_fsm_ids anpassbar.
FSM_READY = 4
DEFAULT_WALK_FSM_IDS = (500, 501)


def _rpc_get_int(client, api_id):
    try:
        code, data = client._Call(api_id, "{}")
        if code == 0 and data:
            return json.loads(data).get("data")
    except Exception:
        pass
    return None


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

        ik_use_waist = self.get_parameter('ik_use_waist').get_parameter_value().bool_value
        ik_alpha = float(self.get_parameter('ik_alpha').value)
        ik_max_dq_step = float(self.get_parameter('ik_max_dq_step').value)
        arm_vel_lim = float(self.get_parameter('arm_velocity_limit').value)

        if self.use_robot:
            init_dds(interface, self.get_logger())
            self.robot = LocoClient()
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
        rate_hz = max(1.0, float(self.get_parameter('cmd_vel_rate_hz').value))
        self.create_timer(1.0 / rate_hz, self._cmd_vel_tick)
        sync_hz = float(self.get_parameter('state_sync_hz').value)
        if self.use_robot and sync_hz > 0.0:
            self.create_timer(1.0 / sync_hz, self._sync_onboard_state,
                              callback_group=self._sync_group)

        self.publisher_arms_controlled = self.create_publisher(Bool, '/g1pilot/arms/enabled', 1)
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
        try:
            return {int(i) for i in self.get_parameter('walk_fsm_ids').value}
        except Exception:
            return set(DEFAULT_WALK_FSM_IDS)

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
            return
        if not self._rpc_lock.acquire(timeout=0.3 if force else 0.0):
            return                      # RPC laeuft -> naechster Takt
        try:
            fsm = _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_ID)
        finally:
            self._rpc_lock.release()
        if fsm is None:
            return
        self._onboard_fsm = fsm
        walking = fsm in self._walk_ids()
        if walking:
            self._nonwalk_polls = 0
            if not self.balanced and not self.robot_stopped:
                self.balanced = True
                self._balanced_at = time.monotonic()
                self.get_logger().info(
                    f"Onboard-Modus WALK erkannt (FSM {fsm}) -- als balanciert uebernommen.")
            return
        # Kein Walk-Zustand: erst nach 2 Polls und 3 s Schonfrist umschalten
        # (Start() braucht einen Moment, bis die FSM-ID 500 meldet).
        self._nonwalk_polls += 1
        if (self.balanced and self._nonwalk_polls >= 2
                and time.monotonic() - self._balanced_at > 3.0):
            self.get_logger().warn(
                f"Onboard-Modus verlassen (FSM {fsm}) -- Gehen gesperrt, "
                "StopMove. Neu: START BALANCING / LAUFEN.")
            self.balanced = False
            self.walk_enabled = False
            self._stop_move()

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
            if self.robot_stopped and self._estop_legs_kept:
                # E-Stop ohne Damp: der G1 steht noch unter der Onboard-Regelung.
                # Quittieren soll ihn NICHT in FSM 4 umschalten (Modusspruenge im
                # Stand vermeiden) -- Latch loesen, aktuellen Modus uebernehmen.
                self._estop_legs_kept = False
                self.robot_stopped = False
                self.balanced = False
                self._log_once("info", "E-Stop quittiert: Beine blieben stehen, "
                               "Onboard-Modus unveraendert.", "_estop_ack_logged")
                self._clear_once("_estop_ack_logged")
                self._sync_onboard_state(force=True)
                return
            self._robot_rpc("SetFsmId", 4)
            self._log_once("info", "Switched to FSM ID 4 (Standby)", "_switch_fsm_id_4_logged")
            self.robot_stopped = False
            self.balanced = False

    def start_balancing_callback(self, msg: Bool):
        if not msg.data:
            return
        # Radio-Semantik wie am Streamdeck: BALANCING gewaehlt -> Gehen aus,
        # Roboter bleibt balanciert stehen.
        self.walk_enabled = False
        self._stop_move("START BALANCING")
        if not self.balanced:
            self._log_once("info", "Starting balancing procedure...", "_start_balance_req_logged")
            self.entering_balancing(max_height=0.5, step=0.02)
            self._log_once("info", "Balancing procedure completed.", "_balance_completed_logged")
        else:
            self._log_once("info", "Already balanced, no action taken.", "_already_balanced_notice_logged")

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
            fsm = self._onboard_fsm
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
                self._robot_rpc("SetFsmId", 4)
                self._log_once("info", "Switched to FSM ID 4 (Standby)", "_switch_fsm_id_4_logged")
                self.robot_stopped = False
                self.balanced = False
            if up_off:
                self._clear_once("_switch_fsm_id_4_logged")

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
                if not self.balanced:
                    self._log_once("info", "Starting balancing procedure...", "_start_balance_r1_logged")
                    self.entering_balancing(max_height=0.5, step=0.02)
                    self._log_once("info", "Balancing procedure completed.", "_balance_completed_r1_logged")
                else:
                    self._log_once("info", "Already balanced, no action taken.", "_already_balanced_notice_r1_logged")
            if self._btn_falling(msg, 6):
                self._clear_once("_start_balance_r1_logged")
                self._clear_once("_balance_completed_r1_logged")
                self._clear_once("_already_balanced_notice_r1_logged")

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

    def entering_balancing(self, max_height=0.5, step=0.02):
        if not self.use_robot or self.robot is None:
            self.balanced = True
            self.get_logger().info("Sim balancing done (use_robot:=false).")
            return
        self._entering = True
        try:
            self._enter_balancing_inner(max_height, step)
        finally:
            self._entering = False

    def _enter_balancing_inner(self, max_height, step):
        height = 0.0
        while height < max_height and not self.robot_stopped:
            height += step
            self._robot_rpc("SetStandHeight", height)
            if self.get_fsm_mode() == 0 and height >= 0.2:
                self._log_once("info", f"Reached max height: {height}", "_balance_reach_height_logged")
                break
            elif self.get_fsm_mode() != 0:
                self._log_once("warn", "Problems during balancing, stopping...", "_balance_problem_stop_logged")
                break
        # E-Stop waehrend der Rampe? Dann NICHT BalanceStand/Start nachschieben
        # (Damp() ist bereits raus, der Roboter haengt/sackt kontrolliert).
        if self.robot_stopped:
            self.get_logger().warn("Balancing abgebrochen (robot_stopped/E-Stop).")
            return
        self._robot_rpc("BalanceStand", 1)
        self._robot_rpc("SetStandHeight", height)
        self._robot_rpc("Start")
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loco_sim — Hybrid-Loco-/Balance-Controller fuer die MuJoCo-Sim.

Ersetzt im Sim das Unitree-Onboard-High-Level (LocoClient.BalanceStand/Move), das
es in MuJoCo nicht gibt. KOMBINIERT zwei Regler, je nach Aufgabe:

  * STAND (am Platz): modellbasierter Knoechel-/Hueft-PD (stand_balancer.py). Haelt
    die FUESSE GEPLANT, richtet die per IMU gemessene Neigung aktiv auf und
    schiebt das Becken so, dass der Schwerpunkt ueber der Fussmitte bleibt
    (Schwerpunkt aus den Gelenkwinkeln, com_model.py) -> Arm-Bewegungen und Lasten
    in den Haenden werden ohne Schritte abgefangen. Braucht IMU + Encoder.
    Steht der Roboter in der Stand-Pose der Lauf-Policy (AGILE: breite Hocke).
  * WALK (laufen): velocity-konditionierte ONNX-Lauf-Policy (walk_policy.py).
    Default: NVIDIA WBC-AGILE "Velocity-G1-History-v0"
    (policies/agile_velocity_g1, Apache-2.0). Sie steuert nur Beine + Taille
    roll/pitch und wurde mit staendig bewegten Armen trainiert -> die Arme
    bleiben beim Laufen frei (arm_controller/Marker), keine Lauf-Pose noetig.
    Legacy-Alternative: policy:=g1_wholebody (unitree_rl_mjlab, braucht die
    Arme in der Lauf-Pose -> dann arm_controller walk_park_arms:=true).

WARUM HYBRID: Eine Lauf-Policy balanciert Stoerungen im Stand per Schritt und
driftet unter dauernder Arm-Bewegung langsam weg. Der PD haelt dagegen die Fuesse
fest. Also: Policy fuers Laufen+Bremsen, PD fuers stationaere Stehen. Nur wenn
der PD es nicht mehr halten kann (Fuss hebt ab, Becken kippt stark; z.B. ein
Stoss), faengt die Policy mit Schritten ab und gibt danach wieder an den PD.

Uebergaenge (per Nutzer-Button; automatisch nur das Abfangen, s.u.):
  * START BALANCING -> STAND (PD, Fuesse geplant, wirklich stationaer).
    Aus HOLD/DAMP: die Bridge stellt den Roboter in die Stand-Pose der Policy
    (loco_sim kommandiert sie im umschaltenden Befehl, die Bridge uebernimmt sie).
    Aus WALK heraus: erst bremst die Policy mit cmd=0 aus, bis der Roboter ruhig
    steht (settle_*), dann uebernimmt der PD und HAELT die vorgefundene Beinpose
    (kein Hochziehen in die Standpose, das kippt mit Armen vorn; siehe
    stand_balancer.py).
  * START WALKING   -> WALK (Policy). Joystick=0 -> die Policy steht am Platz;
    zum geplanten Stehen wechselt man bewusst zu BALANCING. Weil der PD in der
    Stand-Pose der Policy steht, laeuft sie ohne Anlauf-Satz los.
  * Kippen im STAND (rescue_*) -> Policy mit cmd = 0 faengt mit Schritten ab,
    danach wie bei START BALANCING im Laufen zurueck in den PD.
  * Sturz erkannt (IMU-Neigung > Schwelle) -> DAMP (limp), kein Gezappel mehr.

ARME: loco_sim regelt sie NICHT. Sie gehoeren komplett dem arm_controller
(rt/arm_sdk, via rviz/Marker). loco_sim schreibt nur Beine (0..11) + Taille
(12..14) -> kein Arm-"Teleport" bei Zustandswechseln.

FSM (per Streamdeck-Topics); der Zustand wird auch der Bridge gemeldet (Weld):
  HOLD   : Standby. Bridge haelt die Basis (Weld an), Gelenke auf Default-Pose.
  STAND  : PD-Balancer (Basis frei, aufgestellt). Fuesse geplant.
  WALK   : ONNX-Policy (Basis frei). Geschwindigkeit via loco_cmd_vel.
  DAMP   : Emergency / Sturz. kp=0, kd=damp -> weich.
  /g1pilot/start_balancing(True) : -> STAND.
  /g1pilot/start_walking(True)   : -> WALK.
  /g1pilot/loco_cmd_vel (Twist)  : normierte [-1,1] Velocity -> phys. Sollwert.
  /g1pilot/emergency_stop(True)  : -> DAMP + Arme aus.
  /g1pilot/start(True)           : -> HOLD.

Aufruf:  ros2 run g1pilot loco_sim --ros-args -p interface:=lo
"""
import os
import socket
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.common import init_dds
from g1pilot.navigation.walk_policy import get_gravity_orientation, load_walk_policy
from g1pilot.navigation.stand_balancer import (
    BalancerParams, SettleGate, SettleParams, StandBalancer, TipDetector, TipParams,
    LEG_IDX, LEG_KP, LEG_KD, LEG_STAND_POSE, WAIST_IDX, WAIST_KP, WAIST_KD, WAIST_TARGET)


# FSM-Zustaende. Zustands-Code an die Bridge (rt/lowcmd motor_cmd[STATE_IDX].q):
# 0=HOLD (Basis gehalten), 1=aktiv (Basis frei + aufgestellt; STAND oder WALK),
# 2=DAMP (Basis frei, kein Reset). Der Bridge ist nur wichtig OB balanciert wird.
HOLD = "hold"
STAND = "stand"    # PD-Balancer, Fuesse geplant (stationaer)
WALK = "walk"      # ONNX-Policy (laufen/bremsen)
DAMP = "damp"      # Emergency / Sturz: limp

STATE_IDX = 29
LOCO_CODE = {HOLD: 0.0, STAND: 1.0, WALK: 1.0, DAMP: 2.0}

NJ = 29            # G1: 29 Gelenke
# Die ARME (15..28) regelt loco_sim BEWUSST NICHT -- sie gehoeren komplett dem
# arm_controller (rt/arm_sdk, via rviz/Marker). loco_sim schreibt nur Beine (0..11)
# und Taille (12..14). So koennen die Arme nie von loco_sim "teleportiert" werden.


class LocoSim(Node):
    def __init__(self):
        super().__init__("loco_sim")

        self.declare_parameter("interface", "lo")
        # Lauf-Policy (Ordner unter share/g1pilot/policies). Legacy: g1_wholebody.
        self.declare_parameter("policy", "agile_velocity_g1")
        self.declare_parameter("damp_kd", 8.0)
        # WALK->STAND (START BALANCING im Laufen): die Policy bremst mit cmd=0 aus;
        # der PD uebernimmt fruehestens nach settle_s, wenn der Roboter settle_quiet_s
        # lang ruhig ist (|Gyro| < settle_gyro_max, Bein-|dq| < settle_dq_max),
        # spaetestens nach settle_timeout_s. Headless validiert (test_agile_walk_sim).
        self.declare_parameter("settle_s", 0.6)
        self.declare_parameter("settle_quiet_s", 0.3)
        self.declare_parameter("settle_gyro_max", 0.3)
        self.declare_parameter("settle_dq_max", 0.5)
        self.declare_parameter("settle_timeout_s", 3.0)
        # Abfangen: Im STAND gibt loco_sim an die Policy (cmd=0, Schritte), wenn eine
        # Sohle mehr als rescue_foot_tilt_deg kippt (Fuss hebt ab; normal < 2 Grad)
        # oder das Becken mehr als rescue_tilt_max (|proj. Gravitation xy|) kippt,
        # rescue_debounce_s lang. Danach wie START BALANCING im Laufen zurueck in den PD.
        self.declare_parameter("rescue_enable", True)
        self.declare_parameter("rescue_foot_tilt_deg", 6.0)
        self.declare_parameter("rescue_tilt_max", 0.2)
        self.declare_parameter("rescue_debounce_s", 0.04)
        # Policy fuers Abfangen (policies/<name>). Leer -> wie in der deploy.yaml der
        # Lauf-Policy (rescue_policy), sonst die Lauf-Policy selbst.
        self.declare_parameter("rescue_policy", "")
        # Sim-Modell fuer die Schwerpunkt-Fuehrung (com_model.py). Leer -> aus
        # G1_INSPIRE_HANDS abgeleitet (/unitree_mujoco/... im g1pilot-sim-Container).
        # Nicht ladbar -> Balancer ohne Schwerpunkt-Fuehrung (Warnung).
        self.declare_parameter("robot_mjcf", "")
        # Sturz-Erkennung: aufrecht ist proj.grav z=-1; > fall_gz (Neigung ~>60 Grad)
        # ueber fall_debounce_s -> DAMP.
        self.declare_parameter("fall_gz", -0.5)
        self.declare_parameter("fall_debounce_s", 0.3)
        # HOLD: Daempfungs-Faktor auf die Bein-kd. Standard-kd ist fuer das aktive
        # Balancieren ausgelegt und im (verschweissten) HOLD viel zu schwach -> die
        # frei haengenden Beine schwingen hin und her (Fuesse erreichen knapp nicht
        # den Boden). Hoehere kd haelt sie ruhig auf der Default-Standpose, OHNE sie
        # in einer ausgelenkten Schwungpose festzuhalten -> sauberer Balance-Eintritt.
        self.declare_parameter("hold_kd_scale", 6.0)
        # Nur fuer Policies, die die Arme in ihrer Trainings-Pose brauchen
        # (needs_arm_pose, z.B. g1_wholebody): WALK erst freigeben, wenn der
        # arm_controller die Arme in die Lauf-Pose aufgeraeumt hat
        # (/g1pilot/arms/walk_ready). Bis dahin bleibt loco_sim im stationaeren
        # PD-STAND. Fallback nach walk_arm_timeout_s, falls keine Meldung kommt.
        # Die AGILE-Policy (Default) braucht das nicht -> sofort WALK.
        self.declare_parameter("walk_arm_wait", True)
        # Grosszuegig: aus der Sicheren Pose (Ellbogen hinten) brauchen die Arme mit
        # dem kartesischen Speedlimit (0.25 m/s) deutlich laenger als 4 s.
        self.declare_parameter("walk_arm_timeout_s", 15.0)

        # PD-Balancer-Gains (live tunebar via ros2 param set). Bewaehrte Defaults.
        self.declare_parameter("bal_kp_scale", 10.0)         # Posture-Steifigkeit (Haupthebel)
        self.declare_parameter("bal_ramp_s", 0.4)            # weicher Eintritt Policy->PD
        # Integral-Trim [Nm/(rad*s)] auf den Knoechel-Pitch: gleicht STATISCHE
        # CoM-Versaetze aus (z.B. die schwereren Inspire-FTP-Haende verschieben
        # den CoM ~15 mm nach vorn). Ohne Trim bleibt eine Dauerneigung stehen,
        # die den reinen Proportional-Balancer nahe an die Zehenkante bringt.
        # Headless validiert: gx-Restneigung 0.03 -> ~0.00. 0 = aus.
        self.declare_parameter("bal_ki_pitch", 80.0)
        self.declare_parameter("bal_i_limit", 25.0)          # Anti-Windup [Nm]
        self.declare_parameter("bal_ankle_kp_pitch", 150.0)
        self.declare_parameter("bal_ankle_kd_pitch", 40.0)
        # Roll halb so steif wie Pitch (in der breiten Hocke schaukelte 150/40 bzw.
        # 200/40 den Inspire-Roboter seitlich auf, siehe stand_balancer.py).
        self.declare_parameter("bal_ankle_kp_roll", 75.0)
        self.declare_parameter("bal_ankle_kd_roll", 20.0)
        self.declare_parameter("bal_ankle_tau_limit", 50.0)
        self.declare_parameter("bal_hip_kp_pitch", 200.0)
        self.declare_parameter("bal_hip_kd_pitch", 40.0)
        self.declare_parameter("bal_hip_kp_roll", 100.0)
        self.declare_parameter("bal_hip_kd_roll", 20.0)
        self.declare_parameter("bal_hip_tau_limit", 80.0)
        self.declare_parameter("bal_yaw_kd", 30.0)
        # Schwerpunkt-Fuehrung (stand_balancer.BalancerParams.com_*), com_tau_s 0 = aus.
        self.declare_parameter("bal_com_target_m", 0.035)
        self.declare_parameter("bal_com_tau_s", 0.2)
        self.declare_parameter("bal_com_lead_s", 0.25)
        self.declare_parameter("bal_com_limit", 0.2)
        self.declare_parameter("bal_bias_tau_s", 2.0)

        interface = self.get_parameter("interface").get_parameter_value().string_value
        policy_name = self.get_parameter("policy").get_parameter_value().string_value
        self.damp_kd = float(self.get_parameter("damp_kd").value)
        self.fall_gz = float(self.get_parameter("fall_gz").value)
        self.fall_debounce_s = float(self.get_parameter("fall_debounce_s").value)

        self.lockstep = str(os.environ.get("SIM_LOCKSTEP", "")).strip().lower() in (
            "1", "true", "yes", "on")
        self._state_seq = 0

        self._load_policy(policy_name,
                          self.get_parameter("rescue_policy").get_parameter_value().string_value)
        self._load_com_model(self.get_parameter("robot_mjcf").get_parameter_value().string_value)

        init_dds(interface, self.get_logger())
        self.low_state = None
        self.mode_machine = 0
        self._lock = threading.Lock()

        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self._on_lowstate, 10)
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()

        self.crc = CRC()
        self.cmd_msg = unitree_hg_msg_dds__LowCmd_()
        for i in range(len(self.cmd_msg.motor_cmd)):
            self.cmd_msg.motor_cmd[i].mode = 1

        self.get_logger().info("Warte auf rt/lowstate von MuJoCo ...")
        t_wait = time.time()
        while self.low_state is None and rclpy.ok():
            if time.time() - t_wait > 10.0:
                self.get_logger().error("Keine rt/lowstate. MuJoCo laeuft? Domain/Interface?")
                break
            time.sleep(0.02)
        if self.low_state is not None:
            self.get_logger().info(f"Verbunden. mode_machine={self.mode_machine}")

        # Laufzeit-Status. Start = HOLD.
        self.state = HOLD
        self.cmd = np.zeros(3, dtype=np.float32)
        self._fall_since = None            # Sturz-Debounce
        self.balancer = StandBalancer(self.control_dt)
        self.settle = SettleGate()         # WALK->STAND: erst ausbremsen lassen
        self.tip = TipDetector()           # STAND: kippt er trotz PD? -> Policy faengt ab
        self._rescue = False               # WALK nur zum Abfangen (cmd = 0, dann zurueck)
        self._walk_after_rescue = False    # START WALKING waehrend des Abfangens
        self._fresh_walk = False           # WALK aus HOLD/DAMP: erst Stand-Pose senden
        self._walk_pending = False         # WALK angefordert, warte auf Arme
        self._walk_pending_t0 = 0.0
        self._dbg_grav = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self._dbg_gyro = np.zeros(3, dtype=np.float32)

        self.create_subscription(Bool, "/g1pilot/start_balancing", self._on_start_balancing, 10)
        self.create_subscription(Bool, "/g1pilot/start_walking", self._on_start_walking, 10)
        self.create_subscription(Bool, "/g1pilot/emergency_stop", self._on_emergency, 10)
        self.create_subscription(Bool, "/g1pilot/start", self._on_start, 10)
        self.create_subscription(Twist, "/g1pilot/loco_cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/g1pilot/arms/walk_ready", self._on_arms_walk_ready, 10)
        self.arms_enabled_pub = self.create_publisher(Bool, "/g1pilot/arms/enabled", 1)

        self._push_port = int(os.environ.get("SIM_PUSH_PORT", "47900"))
        self._push_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(Bool, "/g1pilot/push", self._on_push, 10)

        # GRASP BOX: Streamdeck-Toggle -> UDP an die Sim (greifbare Test-Kugel an/aus).
        self._grasp_port = int(os.environ.get("SIM_GRASP_PORT", "47901"))
        self._grasp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(Bool, "/g1pilot/grasp_box", self._on_grasp_box, 10)

        self._run_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._run_thread.start()
        self.get_logger().info(
            f"loco_sim bereit (HOLD). policy='{policy_name}', control_dt={self.control_dt:.3f}s. "
            f"START BALANCING -> STAND (PD, Fuesse geplant). "
            f"START WALKING / loco_cmd_vel -> WALK (Policy).")

    # ── Setup ────────────────────────────────────────────────────────────────
    def _load_policy(self, policy_name, rescue_name):
        share = get_package_share_directory("g1pilot")
        pdir = os.path.join(share, "policies", policy_name)
        self.walk_policy = load_walk_policy(pdir)
        self.control_dt = self.walk_policy.step_dt
        # Abfang-Policy: eigene Instanz nur, wenn sie sich von der Lauf-Policy
        # unterscheidet (beide im selben Regeltakt).
        rescue_name = rescue_name or self.walk_policy.rescue_policy_name
        if not rescue_name or rescue_name == policy_name:
            self.rescue_policy = self.walk_policy
        else:
            self.rescue_policy = load_walk_policy(os.path.join(share, "policies", rescue_name))
            if abs(self.rescue_policy.step_dt - self.control_dt) > 1e-9:
                raise ValueError(f"rescue_policy '{rescue_name}' hat step_dt "
                                 f"{self.rescue_policy.step_dt}, Lauf-Policy {self.control_dt}")
            self.get_logger().info(f"Abfang-Policy: {self.rescue_policy.policy_path}")
        # Stand-Pose des PD aus HOLD: die der Policy (AGILE: breite Hocke), sonst
        # die gerade Standpose. Dort startet die Policy ohne Anlauf-Satz.
        sp = self.walk_policy.stand_leg_pose
        self.stand_pose = LEG_STAND_POSE.copy() if sp is None else sp.copy()
        self.get_logger().info(
            f"Lauf-Policy geladen: {self.walk_policy.policy_path} "
            f"({type(self.walk_policy).__name__}, Arm-Pose noetig: "
            f"{self.walk_policy.needs_arm_pose}, Stand-Pose: "
            f"{'Policy' if sp is not None else 'gerade'})")

    def _load_com_model(self, path):
        if not path:
            inspire = os.environ.get("G1_INSPIRE_HANDS", "0").strip().lower() in (
                "1", "true", "yes", "on")
            path = os.path.join("/unitree_mujoco/unitree_robots/g1",
                                "g1_29dof_inspire_ftp.xml" if inspire else "g1_29dof.xml")
        self.com_model = None
        try:
            from g1pilot.navigation.com_model import ComModel
            self.com_model = ComModel(path)
            self.get_logger().info(
                f"Schwerpunkt-Modell: {path} ({self.com_model.mass:.1f} kg).")
        except Exception as e:
            self.get_logger().warn(
                f"Schwerpunkt-Modell nicht geladen ({path}: {e}). Stand-Balancer ohne "
                f"Schwerpunkt-Fuehrung: Arm-Bewegungen/Lasten koennen ihn kippen.")

    # ── DDS / ROS Callbacks ──────────────────────────────────────────────────
    def _on_lowstate(self, msg: LowState_):
        with self._lock:
            self.low_state = msg
            self.mode_machine = int(getattr(msg, "mode_machine", 0))
            self._state_seq += 1

    def _enter_stand(self, reason, hold_pose=False):
        """hold_pose=False: aus HOLD/DAMP, die Bridge stellt den Roboter in
        self.stand_pose. hold_pose=True: aus WALK, die vorgefundene Pose halten."""
        if self.low_state is None:
            self.get_logger().warn(f"Kann nicht STAND ({reason}): keine rt/lowstate.")
            return
        if self.state == STAND:
            # Schon im Stand: die Eintritts-Rampe NICHT neu starten. Sonst faellt
            # kp_scale_eff schlagartig von voller (10x) auf 1x zurueck und das
            # aufrichtende Feedforward-Moment (tau*ramp) auf 0 -> der Balancer wird
            # fuer bal_ramp_s "weich" -> Ruck/Sturz. Ein zweiter BALANCING-Befehl darf
            # den laufenden Stand nicht stoeren -> nur bestaetigen.
            self._walk_pending = False
            self.get_logger().info(f"{reason}: bereits im STAND, balanciere weiter.")
            return
        with self._lock:
            self.cmd = np.zeros(3, dtype=np.float32)
        # Aus HOLD/DAMP: Stand-Pose kommandieren (die Bridge stellt den Roboter beim
        # Wechsel genau dorthin). Aus WALK (hold_pose): die Pose halten, in der die
        # Policy den Roboter abgestellt hat. Trim/Schwerpunkt neu lernen.
        self.balancer.enter(None if hold_pose else self.stand_pose)
        self.settle.cancel()
        self.tip.reset()
        self._rescue = self._walk_after_rescue = False
        self._fall_since = None
        self._walk_pending = False          # ein STAND bricht eine WALK-Anforderung ab
        self.state = STAND
        self.get_logger().info(f"{reason} -> STAND (PD, Fuesse geplant"
                               + (", haelt die Lauf-Endpose)." if hold_pose else ")."))

    def _enter_walk(self, reason):
        if self.low_state is None:
            self.get_logger().warn(f"Kann nicht WALK ({reason}): keine rt/lowstate.")
            return
        if self.state == WALK:
            if self._rescue and self.rescue_policy is not self.walk_policy:
                # Abfangen laeuft mit einer anderen Policy: nicht mitten im Fangschritt
                # wechseln. Danach direkt weiter nach WALK statt in den PD.
                self._walk_after_rescue = True
                self.get_logger().info(f"{reason}: Abfangen laeuft, danach WALK.")
                return
            # Laeuft schon: Policy-History NICHT zuruecksetzen (waere ein Ruck), nur
            # eine evtl. laufende STAND-Uebergabe abbrechen.
            if self.settle.active:
                self.settle.cancel()
                self.get_logger().info(f"{reason}: STAND-Uebergabe abgebrochen, laufe weiter.")
            self._rescue = False
            self._walk_pending = False
            return
        # Aus HOLD/DAMP stellt die Bridge den Roboter beim Wechsel neu auf: erst
        # einen Takt die Stand-Pose kommandieren (_send_policy), damit sie genau
        # dort aufstellt, wo die Policy ruhig steht.
        self._fresh_walk = self.state in (HOLD, DAMP)
        self.walk_policy.reset()
        self.settle.cancel()
        self._rescue = False
        self._walk_after_rescue = False
        self._fall_since = None
        self._walk_pending = False
        self.state = WALK
        self.get_logger().info(f"{reason} -> WALK (Policy).")

    def _enter_rescue(self):
        """STAND -> Policy faengt mit Schritten ab (cmd = 0), danach zurueck in den
        PD ueber die SettleGate (wie START BALANCING im Laufen)."""
        self.rescue_policy.reset()
        self._rescue = True
        self._walk_after_rescue = False
        self._fresh_walk = False
        self._fall_since = None
        self.state = WALK
        self.settle.start(time.perf_counter())
        self.get_logger().warn(
            "STAND: Roboter kippt (Fuss hebt ab) -> Policy faengt mit Schritten ab, "
            "danach wieder PD.")

    def _on_start_balancing(self, msg: Bool):
        if not msg.data:
            return
        if self.state == WALK:
            # Nicht direkt umschalten: erst ausbremsen lassen (_send_policy), der PD
            # uebernimmt, sobald der Roboter ruhig steht.
            if not self.settle.active:
                self.settle.start(time.perf_counter())
                self.get_logger().info(
                    "START BALANCING -> Policy bremst aus, PD uebernimmt, sobald der Roboter ruhig steht ...")
            return
        self._enter_stand("START BALANCING")

    def _on_start_walking(self, msg: Bool):
        if not msg.data:
            return
        # Braucht die Policy die Arme in ihrer Trainings-Pose (Legacy g1_wholebody),
        # erst die Arme aufraeumen lassen (der arm_controller faehrt sie auf dieselbe
        # start_walking-Nachricht hin in die Lauf-Pose). Solange im stationaeren
        # PD-STAND bleiben. Sobald /g1pilot/arms/walk_ready True ist (oder der
        # Timeout greift), geht es nach WALK. Die AGILE-Policy laeuft sofort los.
        if not (self.walk_policy.needs_arm_pose
                and bool(self.get_parameter("walk_arm_wait").value)):
            self._enter_walk("START WALKING")
            return
        # Stationaer stehen bleiben (Fuesse geplant), waehrend die Arme aufraeumen.
        # _enter_stand zuerst (loescht u.a. ein altes pending-Flag), DANACH scharf
        # schalten, damit walk_ready/Timeout den Wechsel nach WALK ausloesen.
        if self.state != STAND:
            self._enter_stand("START WALKING (warte auf Arme)", hold_pose=(self.state == WALK))
        self._walk_pending = True
        self._walk_pending_t0 = time.perf_counter()
        self.get_logger().info("START WALKING -> warte, bis die Arme aufgeraeumt sind ...")

    def _on_arms_walk_ready(self, msg: Bool):
        if msg.data and self._walk_pending:
            self._enter_walk("Arme aufgeraeumt")

    def _on_emergency(self, msg: Bool):
        if msg.data:
            self.state = DAMP
            self._walk_pending = False
            self._rescue = self._walk_after_rescue = False
            self.settle.cancel()
            self.arms_enabled_pub.publish(Bool(data=False))
            self.get_logger().warn("EMERGENCY STOP -> DAMP + Arme aus.")

    def _on_start(self, msg: Bool):
        if msg.data:
            self.state = HOLD
            self._walk_pending = False
            self._rescue = self._walk_after_rescue = False
            self.settle.cancel()
            self.get_logger().info("START -> Standby (HOLD).")

    def _on_push(self, msg: Bool):
        if not msg.data:
            return
        try:
            self._push_sock.sendto(b"push", ("127.0.0.1", self._push_port))
            self.get_logger().info("PUSH -> Stoer-Impuls an die Sim.")
        except OSError as e:
            self.get_logger().warn(f"PUSH nicht gesendet: {e}")

    def _on_grasp_box(self, msg: Bool):
        """Streamdeck-Toggle -> greifbare Test-Kugel in der Sim an/aus."""
        payload = b"on" if msg.data else b"off"
        try:
            self._grasp_sock.sendto(payload, ("127.0.0.1", self._grasp_port))
            self.get_logger().info(f"GRASP BOX -> {'AN' if msg.data else 'AUS'} an die Sim.")
        except OSError as e:
            self.get_logger().warn(f"GRASP BOX nicht gesendet: {e}")

    def _on_cmd_vel(self, msg: Twist):
        if self.state not in (STAND, WALK):
            return
        cmd = self.walk_policy.scale_command(msg.linear.x, msg.linear.y, msg.angular.z)
        with self._lock:
            self.cmd = cmd
        # KEIN Auto-Umschalten: im STAND (PD) ignoriert der Roboter den Joystick und
        # bleibt wirklich stationaer. Laufen startet nur per START WALKING-Button.

    # ── Sturz-Erkennung (IMU-only, gilt fuer STAND und WALK) ─────────────────
    def _fallen(self, gravity):
        if float(gravity[2]) > self.fall_gz:        # zu stark geneigt
            if self._fall_since is None:
                self._fall_since = time.perf_counter()
            elif time.perf_counter() - self._fall_since > self.fall_debounce_s:
                return True
        else:
            self._fall_since = None
        return False

    # ── Regelschleife ────────────────────────────────────────────────────────
    def _control_loop(self):
        diag_n = 0
        diag_t0 = time.perf_counter()
        while rclpy.ok():
            t0 = time.perf_counter()
            try:
                if self.low_state is None:
                    pass
                elif self.state == HOLD:
                    self._send_hold()
                elif self.state == DAMP:
                    self._send_damp()
                elif self.state == STAND:
                    self._send_balance_pd()
                elif self.state == WALK:
                    self._send_policy()
            except Exception as e:
                self.get_logger().error(f"Regelschleife: {e}")
                self.state = DAMP
            busy = time.perf_counter() - t0

            # WALK-Anforderung haengt? Falls keine arms/walk_ready-Meldung kommt
            # (arm_controller aus/anderer Modus), nach Timeout trotzdem loslaufen.
            if self._walk_pending:
                if (time.perf_counter() - self._walk_pending_t0
                        > float(self.get_parameter("walk_arm_timeout_s").value)):
                    self.get_logger().warn(
                        "WALK: keine arms/walk_ready-Meldung -> Timeout, laufe trotzdem los.")
                    self._enter_walk("Arme-Timeout")

            if self.state in (STAND, WALK):
                diag_n += 1
                if diag_n >= 100:
                    span = time.perf_counter() - diag_t0
                    hz = diag_n / span if span > 0 else 0.0
                    eff = (1.0 / self.control_dt) if self.lockstep else hz
                    g = self._dbg_grav
                    self.get_logger().info(
                        f"[{self.state}] eff={eff:.1f}Hz (soll 50) "
                        f"grav=[{g[0]:+.2f} {g[1]:+.2f} {g[2]:+.2f}] "
                        f"cmd=[{self.cmd[0]:+.2f} {self.cmd[1]:+.2f} {self.cmd[2]:+.2f}]"
                        + ("  <-- ZU LANGSAM!" if eff < 45 else ""))
                    diag_n = 0
                    diag_t0 = time.perf_counter()
            else:
                diag_n = 0
                diag_t0 = time.perf_counter()

            if self.lockstep:
                target_seq = self._state_seq + 1
                t_wait = time.perf_counter()
                while (self._state_seq < target_seq and rclpy.ok()
                       and (time.perf_counter() - t_wait) < 0.5):
                    time.sleep(0.0002)
            else:
                dt = self.control_dt - busy
                if dt > 0:
                    time.sleep(dt)

    def _write(self):
        self.cmd_msg.mode_pr = 0
        self.cmd_msg.mode_machine = self.mode_machine
        self.cmd_msg.motor_cmd[STATE_IDX].q = LOCO_CODE.get(self.state, 0.0)
        self.cmd_msg.crc = self.crc.Crc(self.cmd_msg)
        self.lowcmd_pub.Write(self.cmd_msg)

    def _set_motors(self, idx, q, kp, kd, tau=None):
        """PD-Sollwerte fuer die Motoren idx in die naechste rt/lowcmd schreiben."""
        for k, i in enumerate(idx):
            mc = self.cmd_msg.motor_cmd[int(i)]
            mc.mode = 1
            mc.q = float(q[k])
            mc.dq = 0.0
            mc.tau = float(tau[k]) if tau is not None else 0.0
            mc.kp = float(kp[k])
            mc.kd = float(kd[k])

    def _send_hold(self):
        # Steifer Stand: Beine auf Default-Standpose, Taille gehalten. Arme: frei.
        # Erhoehte Daempfung (hold_kd_scale), damit die im (verschweissten) HOLD frei
        # haengenden Beine NICHT hin- und herschwingen, sondern ruhig auf der Default-
        # Pose sitzen. Ziel bleibt bewusst die natuerliche Standpose (default), nicht
        # die zufaellige Auslenkung -> der Balance-Eintritt startet aus der Ruhe.
        kd_scale = float(self.get_parameter("hold_kd_scale").value)
        self._set_motors(LEG_IDX, LEG_STAND_POSE, LEG_KP, LEG_KD * kd_scale)
        self._set_motors(WAIST_IDX, WAIST_TARGET, WAIST_KP, WAIST_KD)
        self._write()

    def _send_damp(self):
        for i in range(len(self.cmd_msg.motor_cmd)):
            mc = self.cmd_msg.motor_cmd[i]
            mc.q = 0.0
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = 0.0
            mc.kd = self.damp_kd
        self._write()

    def _balancer_params(self):
        """Aktuelle Balancer-Gains aus den (live tunebaren) ROS-Parametern."""
        def g(name):
            return float(self.get_parameter(name).value)
        return BalancerParams(
            kp_scale=g("bal_kp_scale"), ramp_s=g("bal_ramp_s"),
            ki_pitch=g("bal_ki_pitch"), i_limit=g("bal_i_limit"),
            ankle_kp_pitch=g("bal_ankle_kp_pitch"), ankle_kd_pitch=g("bal_ankle_kd_pitch"),
            ankle_kp_roll=g("bal_ankle_kp_roll"), ankle_kd_roll=g("bal_ankle_kd_roll"),
            ankle_tau_limit=g("bal_ankle_tau_limit"),
            hip_kp_pitch=g("bal_hip_kp_pitch"), hip_kd_pitch=g("bal_hip_kd_pitch"),
            hip_kp_roll=g("bal_hip_kp_roll"), hip_kd_roll=g("bal_hip_kd_roll"),
            hip_tau_limit=g("bal_hip_tau_limit"), yaw_kd=g("bal_yaw_kd"),
            com_target_m=g("bal_com_target_m"), com_tau_s=g("bal_com_tau_s"),
            com_lead_s=g("bal_com_lead_s"), com_limit=g("bal_com_limit"),
            bias_tau_s=g("bal_bias_tau_s"),
            mass_kg=self.com_model.mass if self.com_model is not None else 35.0)

    def _tip_params(self):
        def g(name):
            return float(self.get_parameter(name).value)
        return TipParams(foot_tilt_deg=g("rescue_foot_tilt_deg"),
                         tilt_max=g("rescue_tilt_max"), debounce_s=g("rescue_debounce_s"))

    def _settle_params(self):
        def g(name):
            return float(self.get_parameter(name).value)
        return SettleParams(min_s=g("settle_s"), quiet_s=g("settle_quiet_s"),
                            gyro_max=g("settle_gyro_max"), dq_max=g("settle_dq_max"),
                            timeout_s=g("settle_timeout_s"))

    def _send_balance_pd(self):
        """Modellbasierter Knoechel-/Hueft-Balancer (Fuesse geplant, stationaer),
        Regelgesetz in stand_balancer.StandBalancer."""
        ls = self.low_state
        gyro = np.array(ls.imu_state.gyroscope, dtype=np.float32)
        g = get_gravity_orientation(ls.imu_state.quaternion)
        self._dbg_grav = g.copy()
        self._dbg_gyro = gyro.copy()

        if self._fallen(g):
            self.state = DAMP
            self.arms_enabled_pub.publish(Bool(data=False))
            self.get_logger().warn("STURZ erkannt (Stand) -> DAMP + Arme aus.")
            return self._send_damp()

        now = time.perf_counter()
        q = np.array([ls.motor_state[i].q for i in range(NJ)], dtype=np.float32)
        dq_legs = np.array([ls.motor_state[i].dq for i in LEG_IDX], dtype=np.float32)
        quat = np.array(ls.imu_state.quaternion, dtype=np.float64)

        # Kann der PD das noch halten? Hebt ein Fuss ab, faengt die Policy ab.
        if (bool(self.get_parameter("rescue_enable").value)
                and self.tip.update(now, quat, q[LEG_IDX], g, self._tip_params())):
            self._enter_rescue()
            return self._send_policy()

        com_x = (float(self.com_model.com_in_foot(q)[0])
                 if self.com_model is not None else None)
        c = self.balancer.step(q[LEG_IDX], dq_legs, g, gyro, now,
                               self._balancer_params(), com_x)
        self._set_motors(LEG_IDX, c.leg_q, c.leg_kp, c.leg_kd, tau=c.leg_tau)
        self._set_motors(WAIST_IDX, c.waist_q, c.waist_kp, c.waist_kd)
        self._write()

    def _send_policy(self):
        ls = self.low_state
        q = np.array([ls.motor_state[i].q for i in range(NJ)], dtype=np.float32)
        dq = np.array([ls.motor_state[i].dq for i in range(NJ)], dtype=np.float32)
        gyro = np.array(ls.imu_state.gyroscope, dtype=np.float32)
        gravity = get_gravity_orientation(ls.imu_state.quaternion)
        self._dbg_grav = gravity.copy()
        self._dbg_gyro = gyro.copy()
        with self._lock:
            cmd = self.cmd.copy()

        if self._fallen(gravity):
            self.state = DAMP
            self.settle.cancel()
            self._rescue = self._walk_after_rescue = False
            self.arms_enabled_pub.publish(Bool(data=False))
            self.get_logger().warn("STURZ erkannt (Walk) -> DAMP + Arme aus.")
            return self._send_damp()

        if self._fresh_walk:
            # Erster Takt aus HOLD/DAMP: Stand-Pose kommandieren. Die Bridge stellt
            # den Roboter beim Wechsel in genau diese Pose; ab dem naechsten Takt
            # laeuft die Policy (History frisch aus dem aufgestellten Zustand).
            self._fresh_walk = False
            self.walk_policy.reset()
            self._set_motors(LEG_IDX, self.stand_pose, LEG_KP, LEG_KD)
            self._set_motors(WAIST_IDX, WAIST_TARGET, WAIST_KP, WAIST_KD)
            return self._write()

        if self._rescue:
            cmd = np.zeros(3, dtype=np.float32)    # Abfangen: am Platz, Joystick egal

        if self.settle.active:
            # START BALANCING angefordert: mit cmd=0 ausbremsen, bis ruhig.
            cmd = np.zeros(3, dtype=np.float32)
            done = self.settle.update(time.perf_counter(), gyro, dq[LEG_IDX],
                                      self._settle_params())
            if done is not None:
                if done == "timeout":
                    self.get_logger().warn("STAND-Uebergabe: nicht ganz ruhig geworden "
                                           "-> Timeout, PD uebernimmt trotzdem.")
                if self._rescue and self._walk_after_rescue:
                    # START WALKING kam waehrend des Abfangens: jetzt mit der
                    # Lauf-Policy weiter (frische History aus dem ruhigen Stand).
                    self.walk_policy.reset()
                    self._rescue = self._walk_after_rescue = False
                    self.get_logger().info("Abgefangen -> WALK (Policy).")
                else:
                    self._enter_stand("Abgefangen" if self._rescue else "START BALANCING",
                                      hold_pose=True)
                    return self._send_balance_pd()

        # KEIN Auto-Umschalten: bei cmd=0 STEHT die Policy einfach am Platz. Zu
        # wirklich stationaer (Fuesse geplant) wechselt nur der Nutzer per
        # START BALANCING. Die Policy aktuiert nur Beine + Taille (0..14); die
        # Arme (15..28) bleiben dem arm_controller (rt/arm_sdk).
        policy = self.rescue_policy if self._rescue else self.walk_policy
        t = policy.step(q, dq, gyro, gravity, cmd)
        self._set_motors(t.idx, t.q, t.kp, t.kd)
        self._write()


def main(args=None):
    rclpy.init(args=args)
    node = LocoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

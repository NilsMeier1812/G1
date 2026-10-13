#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, threading, math, json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, ColorRGBA, String, Float32MultiArray
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
import pinocchio as pin
from pinocchio import SE3



from g1pilot.utils.joints_names import (
    JOINT_NAMES_ROS,
    JOINT_LIMITS_RAD,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
)

from g1pilot.utils.ik_solver import G1IKSolver
from g1pilot.navigation import scene_markers as sm
from g1pilot.manipulation.pose_store import PoseStore
from g1pilot.manipulation import arm_command as ac
from g1pilot.manipulation.arm_geometry import ArmGeometry, Environment, HAND_LINK_MIN
from g1pilot.manipulation.arm_planner_process import PlannerProcess
from g1pilot.manipulation.arm_planner import DEFAULTS as PLANNER_DEFAULTS

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ , LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.common import (
    is_sim_mode,
    MotorState,
    G1_29_JointArmIndex,
    G1_29_JointWristIndex,
    G1_29_JointIndex,
    DataBuffer,
    init_dds,
)

WORKSPACE = {
    "frame": 'pelvis',
    "left_arm": {
        "left_bottom_front": [0.33, 0.24, 0.02],
        "right_bottom_front": [0.33, 0.07,  0.02],
        "left_bottom_back":   [0.16, 0.24,  0.02],
        "right_bottom_back":  [0.16, 0.07,  0.02],
        "right_top_back":    [0.07, 0.20,  0.20],
        "left_top_back":     [0.07, 0.47,  0.20],
        "right_top_front":  [0.45, 0.11,  0.20],
        "left_top_front":   [0.41, 0.30,  0.20],
    },

    "right_arm": {
        "left_bottom_front": [0.33, -0.24, 0.02],
        "right_bottom_front": [0.33, -0.07,  0.02],
        "left_bottom_back":   [0.16, -0.24,  0.02],
        "right_bottom_back":  [0.16, -0.07,  0.02],
        "right_top_back":    [0.07, -0.20,  0.20],
        "left_top_back":     [0.07, -0.47,  0.20],
        "right_top_front":  [0.45, -0.11,  0.20],
        "left_top_front":   [0.41, -0.30,  0.20],
    },
}


def _yaw_from_R(R: np.ndarray) -> float:
    """Yaw (Z) desde matriz de rotación."""
    return math.atan2(R[1, 0], R[0, 0])


def _mat_to_quat_wxyz(R: np.ndarray):
    q = pin.Quaternion(R)
    return np.array([q.w, q.x, q.y, q.z])


def _quat_wxyz_to_matrix(qwxyz):
    w, x, y, z = qwxyz
    return pin.Quaternion(w, x, y, z).matrix()


class ArmController(Node):
    """
    ROS 2 node controlling G1 arms with external IK (G1IKSolver) and Unitree DDS.

    This node manages the high-level control of both G1-29 arms by combining
    inverse kinematics (Pinocchio-based), filtered goal tracking, end-effector
    auto-calibration, and direct low-level command publishing via Unitree's DDS
    LowCmd interface. It also supports simulation mode through /joint_states
    publishing when `use_robot=False`.

    Parameters
    ----------
    use_robot : bool
        Enables physical robot control (DDS) if True; simulation otherwise.
    interface : str
        Ethernet interface for DDS communication.
    arm_velocity_limit : float
        Maximum allowed joint velocity.
    rate_hz : float
        Main loop update frequency.
    ik_world_frame : str
        Reference frame for IK computation.
    ik_alpha : float
        Exponential smoothing coefficient for joint updates.
    ik_goal_filter_alpha : float
        Low-pass filter coefficient for goal smoothing.
    ik_orientation_mode : str
        Orientation mode for IK ('full', 'yaw-only', etc.).
    ik_max_ori_step_rad : float
        Maximum allowed orientation step in radians per iteration.
    ee_auto_calibrate : bool
        Enables end-effector offset auto-calibration.
    auto_reissue_goals : bool
        Das letzte Marker-Ziel periodisch erneut durch den Ziel-Filter schicken,
        damit der gefilterte Sollwert wirklich am Marker ankommt.
    goal_pos_tol : float
        Tolerance (m) for position convergence check.
    goal_ori_tol_deg : float
        Tolerance (deg) for orientation convergence check.
    ee_offset_right_xyz : list[float]
        Static XYZ offset for right-hand calibration.
    ee_offset_right_rpy_deg : list[float]
        Static RPY offset (deg) for right-hand calibration.
    ee_offset_left_xyz : list[float]
        Static XYZ offset for left-hand calibration.
    ee_offset_left_rpy_deg : list[float]
        Static RPY offset (deg) for left-hand calibration.
    """



    def __init__(self):
        super().__init__("arm_controller")
        self.get_logger().info("Arm Controller Node started.")

        self.declare_parameter("use_robot", True)
        self.declare_parameter("interface", "eth0")
        # Gelenk-Geschwindigkeitslimit [rad/s]. Frueher 5.0 -- viel zu schnell
        # fuer echte Hardware (und der Launcher hat seinen 2.0-Default nie an
        # den Node uebergeben, d.h. es galt immer 5.0).
        self.declare_parameter("arm_velocity_limit", 1.5)
        # Kartesisches Speedlimit [m/s] fuer Hand-TCP UND Ellbogen. 0.25 m/s =
        # "reduced speed" nach ISO 10218-1 / ISO TS 15066 (Industriewert fuer
        # Betrieb mit Personen im Schutzraum). Faengt den Fall ab, dass der
        # IK-Solver eine wirre Konfiguration vorschlaegt: der Arm darf da
        # hinfahren, aber nur mit dieser Geschwindigkeit. 0.0 = aus.
        self.declare_parameter("ee_velocity_limit", 0.25)
        # Selbstkollisions-Gate: kommandierte Konfiguration wird VOR dem Senden
        # gegen Selbstkollision geprueft (Marge s. ik_solver.collision_margin);
        # bei Verletzung haelt der Arm an, statt in den Koerper zu fahren.
        self.declare_parameter("self_collision_gate", True)
        # Umgebungs-Kollisions-Gate: dieselbe Pruefung, aber gegen die Objekte
        # aus /scene_markers, ueber das Kugelmodell (arm_geometry.py): Hand darf
        # aufsetzen, Hand ignoriert Greif-Objekte, ein Armpunkt unter seinem
        # Mindestabstand darf sich nicht weiter annaehern. Eigener
        # Schalter, unabhaengig von self_collision_gate (siehe g1pilot/
        # g1pilot/docs/11_arm_manipulation_technik.md (Umgebungs-Kollisionsgate)).
        self.declare_parameter("environment_collision_gate", True)
        # Toleranz [rad], ab der das Ende einer GEPLANTEN Bewegung (siehe
        # g1pilot/docs/11_arm_manipulation_technik.md (Armplaner)) als erreicht gilt.
        self.declare_parameter("planned_motion_tolerance", 0.02)
        # Armplaner (manipulation/arm_planner.py): Abstand zu Objekten in der
        # MITTE einer Bewegung [m]; nahe Start und Ziel klingt er auf 0 ab
        # (Hand darf am Ende aufliegen). Geht er nicht, wird bis
        # planner_min_clearance reduziert.
        self.declare_parameter("planner_clearance", float(PLANNER_DEFAULTS["clearance"]))
        self.declare_parameter("planner_min_clearance", float(PLANNER_DEFAULTS["min_clearance"]))
        # Abstand [m] von Start-/Zielposition eines Armpunkts, ab dem der volle
        # Abstand gilt (Groesse des "Trichters" um Start und Ziel).
        self.declare_parameter("planner_relax_radius", float(PLANNER_DEFAULTS["relax_radius"]))
        self.declare_parameter("planner_time_budget_s", float(PLANNER_DEFAULTS["time_budget_s"]))
        # Tempo der geplanten Fahrt (Gelenk rad/s, kartesisch m/s je Armpunkt).
        self.declare_parameter("planner_vel_limit", float(PLANNER_DEFAULTS["vel_limit"]))
        self.declare_parameter("planner_cart_vel_limit", float(PLANNER_DEFAULTS["cart_vel_limit"]))
        # Laufzeit-Gate gegen Objekte (Kugelmodell): naeher als das [m] darf ein
        # Armpunkt nur, wenn er sich dabei nicht weiter annaehert. Hand 0 =
        # aufsetzen erlaubt; Arm (Ellbogen/Unterarm) etwas Abstand.
        self.declare_parameter("gate_hand_floor", 0.0)
        self.declare_parameter("gate_arm_floor", 0.01)
        # Marker-Steuerung: unter diesem Abstand [m] wird die Annaeherung an ein
        # Objekt verlangsamt (bis 25 %), damit die Hand sanft aufsetzt.
        self.declare_parameter("gate_slow_dist", 0.05)
        # Reine Hand-Posen (nur left_hand/right_hand): so lange nach dem Senden
        # warten, bis "reached" gemeldet wird (Inspire schliesst voll in ~1.3 s).
        self.declare_parameter("hand_only_settle_s", 1.8)
        # Glaettung (wie ik_alpha) NUR fuer geplante Pose-Fahrten. Der Pfad des
        # Planers ist schon glatt; ik_alpha=0.2 macht die Fahrt zaeh (effektiv
        # 20 % der erlaubten Gelenkgeschwindigkeit + langer Kriech-Schwanz).
        # Default = ik_alpha (Real unveraendert), bringup_sim setzt 1.0.
        self.declare_parameter("planned_motion_alpha", -1.0)
        # walk_ready erst, wenn die GEMESSENEN Armgelenke so nah an der Lauf-Pose
        # sind (Norm ueber 14 Gelenke, rad).
        self.declare_parameter("walk_ready_measured_tolerance", 0.25)
        # Daempfung [kd] der Arm-Gelenke im E-Stop/Slack-Zustand. kp=0/tau=0 ->
        # keine Positionshaltung; kleines kd -> die Arme sacken GEDAEMPFT statt
        # frei zu fallen (an Seilen abgefangen). 0.0 = voellig frei (harter Fall).
        self.declare_parameter("estop_arm_kd", 3.0)
        self.declare_parameter("rate_hz", 250.0)
        self.declare_parameter("ik_world_frame", "pelvis")
        self.declare_parameter("ik_alpha", 0.2)
        self.declare_parameter("ik_goal_filter_alpha", 0.25)
        # Glaettung pro Takt (ik_alpha + Ziel-Filter im IK-Solver) gilt fuer
        # EINEN Takt -- wie stark sie bremst, haengt also von der erreichten
        # Regelrate ab (Soll 250 Hz, real mit IK ~30-50 Hz, Sim auf schwachem
        # PC ~10-15 Hz -> Arm haengt 1-2 s hinterher). > 0: die Alphas gelten
        # fuer diese Bezugsrate und werden auf den echten Zeitschritt
        # umgerechnet (gleiches Zeitverhalten unabhaengig von der Rate).
        # 0 = pro Takt wie bisher (Real-Default). bringup_sim setzt 50.
        self.declare_parameter("smoothing_ref_hz", 0.0)
        self.declare_parameter("ik_orientation_mode", "full")
        self.declare_parameter("ik_max_ori_step_rad", 0.35)
        # Staerke der IK-Nullraum-Regularisierung zur Ruhe-Pose (0 = aus). Verhindert,
        # dass der redundante Arm beim Marker-Ziehen in den zum Koerper gefalteten
        # Zweig faellt. Live tunebar.
        self.declare_parameter("ik_null_space_gain", 0.15)
        # Schwerkraft-Feedforward [Nm] auf die Arm-Gelenke (aus dem Pinocchio-
        # Modell). Ohne haengt der Arm PD-bedingt ~0.05 rad unter dem Sollwert
        # (mit den schwereren Inspire-FTP-Haenden ~2-4 cm an der Hand) -> die
        # Hand "haelt nicht stabil an der Stelle". Live abschaltbar.
        self.declare_parameter("gravity_comp", True)
        self.declare_parameter("ee_auto_calibrate", True)
        # arm_sdk-Gewichts-Rampe (motor_cmd[29].q): 0=Roboter-eigene Steuerung,
        # 1=arm_sdk uebernimmt. Auf dem echten G1 muss das Gewicht beim Enable
        # sanft 0->1 und beim Disable 1->0 gefahren werden, sonst reisst die
        # Uebergabe an den Armen (Unitree-Konvention, s. g1_arm7_sdk-Beispiel).
        # 0.0 = sofort umschalten (Sim-Verhalten, wird vom Sim-Bringup gesetzt).
        self.declare_parameter("arm_weight_ramp_up_s", 2.0)
        self.declare_parameter("arm_weight_ramp_down_s", 2.0)
        # Rueckgabe an den Onboard-Regler (DISABLE / Programmende, nur mit
        # Gewichts-Rampe, also real): ERST die Arme geplant in die Pose fahren,
        # in der der Onboard-Regler sie beim ENABLE hielt, DANN das Gewicht
        # 1->0 rampen. Ohne die Rueckfahrt blendet die Rampe in 2 s von der
        # aktuellen Stellung in die Onboard-Pose ueber -- bei hochgehobenen
        # Armen ein kraeftiger Schwung auf geradem Gelenkweg (ggf. durch den
        # Tisch). Sim (Rampe 0) unveraendert.
        self.declare_parameter("release_to_onboard_pose", True)
        self.declare_parameter("release_tolerance", 0.05)      # rad, max je Gelenk
        self.declare_parameter("release_timeout_s", 12.0)      # inkl. Planung
        # Programmende (Strg+C / docker stop): direkte, langsame Rueckfahrt
        # hoechstens so lange [s], danach Rampe (launch eskaliert nach ~5 s).
        self.declare_parameter("shutdown_return_s", 2.5)
        # Schleppfehler-Begrenzung [rad] (0 = aus, real 0.3): das Kommando
        # entfernt sich nie weiter als das von der GEMESSENEN Stellung. Haelt
        # jemand/etwas den Arm fest, laeuft der Sollwert nicht davon -- sonst
        # baut der PD-Regler Kraft auf und der Arm schnellt beim Loslassen
        # zum weit entfernten Sollwert.
        self.declare_parameter("max_tracking_error", 0.0)
        # Anfahrrampe [rad/s^2] (0 = aus, real 3.0): die Gelenkgeschwindigkeit
        # des Kommandos darf pro Sekunde hoechstens um so viel STEIGEN. Ein
        # neues, weit entferntes Ziel (anderer Eingang, Marker-Klick) faehrt
        # damit sanft an statt im ersten Tick mit voller Geschwindigkeit.
        # Bremsen bleibt unbegrenzt (Anhalten ist nie das Risiko).
        self.declare_parameter("arm_accel_limit", 0.0)


        self.declare_parameter("ee_offset_right_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_right_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_left_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_left_rpy_deg", [0.0, 0.0, 0.0])

        self.use_robot = bool(self.get_parameter("use_robot").value)
        self.interface = str(self.get_parameter("interface").value)
        self.arm_velocity_limit = float(self.get_parameter("arm_velocity_limit").value)
        self.ee_velocity_limit = float(self.get_parameter("ee_velocity_limit").value)
        self.self_collision_gate = bool(self.get_parameter("self_collision_gate").value)
        self.environment_collision_gate = bool(self.get_parameter("environment_collision_gate").value)
        self.planned_motion_tolerance = float(self.get_parameter("planned_motion_tolerance").value)
        self.estop_arm_kd = float(self.get_parameter("estop_arm_kd").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.frame = str(self.get_parameter("ik_world_frame").value)
        self.ik_alpha = float(self.get_parameter("ik_alpha").value)
        self.ik_goal_filter_alpha = float(self.get_parameter("ik_goal_filter_alpha").value)
        self.ik_orientation_mode = str(self.get_parameter("ik_orientation_mode").value).lower()
        self.ik_max_ori_step_rad = float(self.get_parameter("ik_max_ori_step_rad").value)
        self.ee_auto_calibrate = bool(self.get_parameter("ee_auto_calibrate").value)

        self.declare_parameter("auto_reissue_goals", True)
        self.declare_parameter("goal_pos_tol", 0.01)
        self.declare_parameter("goal_ori_tol_deg", 3.0)

        self.auto_reissue_goals = bool(self.get_parameter("auto_reissue_goals").value)
        self.goal_pos_tol = float(self.get_parameter("goal_pos_tol").value)
        self.goal_ori_tol_deg = float(self.get_parameter("goal_ori_tol_deg").value)

        # Marker-Ziel-Rueckmeldung (/g1pilot/hand_goal_status): je Seite wird das
        # zuletzt per Marker gesetzte Ziel verfolgt, bis es erreicht ist
        # (goal_pos_tol / goal_ori_tol_deg) oder als unerreichbar gilt -- d.h.
        # der kommandierte Arm steht laenger als stall_time_s still (IK am
        # Reichweitenrand, Kollisions-Gate, Gelenklimit), ohne angekommen zu
        # sein. Der RViz-Marker bleibt so lange auf dem Ziel stehen.
        self.declare_parameter("marker_goal_check_hz", 25.0)
        self.declare_parameter("marker_goal_stall_time_s", 0.8)
        self.declare_parameter("marker_goal_stall_tol_rad", 0.01)
        self._marker_check_dt = 1.0 / max(1.0, float(self.get_parameter("marker_goal_check_hz").value))
        self._marker_stall_time = float(self.get_parameter("marker_goal_stall_time_s").value)
        self._marker_stall_tol = float(self.get_parameter("marker_goal_stall_tol_rad").value)
        self._marker_check_last = 0.0
        # side -> {"stamp": (sec, nanosec), "T": SE3 Endziel (mit Offsets),
        #          "q_ref": 7 DOF, "t_ref": monotonic}  bzw. None
        self._marker_goal = {"left": None, "right": None}

        def _pvec(name):
            v = self.get_parameter(name).value
            return np.array(v, dtype=float)

        self._ee_off_right_xyz = _pvec("ee_offset_right_xyz")
        self._ee_off_right_rpy_deg = _pvec("ee_offset_right_rpy_deg")
        self._ee_off_left_xyz = _pvec("ee_offset_left_xyz")
        self._ee_off_left_rpy_deg = _pvec("ee_offset_left_rpy_deg")

        self.motor_state = [MotorState() for _ in range(35)]
        self.lowstate_buffer = DataBuffer()
        self._last_q_target = np.zeros(14, dtype=float)
        self.arms_enabled = False
        # arm_sdk-Gewicht + Rampenzustand: off -> ramp_up -> on -> ramp_down -> off.
        # Waehrend ramp_down publiziert main_loop weiter (letzte Sollpose), damit
        # die Uebergabe an die Roboter-eigene Steuerung weich ist.
        self._arm_sdk_weight = 0.0
        self._weight_state = "off"
        # E-Stop-Latch: nach /g1pilot/emergency_stop=true verstummt der Node
        # SOFORT (eine letzte Nachricht mit Weight=0 + kp/kd=0 -> Arme schlaff,
        # Damp() faengt den Rest). Quittiert wird ueber /g1pilot/start --
        # dieselbe Semantik wie robot_stopped im loco_client.
        self.estop_active = False
        # Slack-Latch: Arme werden gedaempft-drehmomentfrei gehalten (Weight=1,
        # kp=0), OHNE Rueckgabe an den Onboard-Regler. Bleibt nach dem
        # Quittieren (START) aktiv, bis ENABLE MANIPULATION die Kontrolle
        # bewusst zurueckholt.
        self._arm_slack = False
        # Rueckgabe an den Onboard-Regler (siehe release_to_onboard_pose):
        # _onboard_q = gemessene Armstellung (14) im Moment, als der Onboard-
        # Regler die Arme noch allein fuehrte (ENABLE bzw. E-Stop bei Gewicht 0).
        self._onboard_q = None
        self._releasing = False          # Rueckfahrt laeuft (danach Rampe 1->0)
        self._release_direct = False     # Planung gescheitert -> direkt (Gate aktiv)
        self._release_t0 = 0.0
        self._release_best = (np.inf, 0.0)   # (kleinster Restfehler, Zeitpunkt)
        self._last_cmd_speed = 0.0       # rad/s, fuer die Anfahrrampe
        self.homing_active = False
        self.homing_reached = False
        self.homing_tolerance = 0.02
        self._last_left_goal_raw = None
        self._last_right_goal_raw = None
        self._goal_left_filt = None
        self._goal_right_filt = None
        self._ik_hold_goal = {}          # Halteziele aus _align_ik_to_config
        self._reset_after_home = False
        self._initialized = False

        # Positionsspeicher (plan-execute, siehe g1pilot/docs/11_arm_manipulation_technik.md
        # (Armplaner)): Endpunkt wird gespeichert, die Bahn dorthin JEDES MAL
        # neu geplant (arm_planner.plan_arms im Planer-Prozess), weil Startpose und
        # Umgebung sich seit dem Speichern geaendert haben koennen.
        # Ablage liegt im bind-gemounteten Repo -> ueberlebt Container-Neustarts
        # (siehe pose_store.default_store_path); Pfad loggen, damit man beim
        # Backup/Umzug weiss, wo die Posen liegen.
        # Live-Schnittstelle: Vorgangs-ID/Quelle der AKTUELL laufenden Bewegung
        # (nur eine gleichzeitig -- durchgesetzt ueber _arms_not_ready_reason),
        # damit Status-Meldungen dem richtigen Aufrufer zuordenbar sind.
        self._cmd_req_id = ""
        self._cmd_source = "pose_store"
        self._cmd_ik_detail = None      # IK-Restfehler der laufenden Bewegung
        self._pose_store = PoseStore()
        self.get_logger().info(f"Positionsspeicher: {self._pose_store.path}")
        self._planned_motion_active = False
        # Gleichzeitige Ausfuehrung: EINE zeitparametrierte Trajektorie ueber
        # die geplanten Arme (7 DOF je Seite, in _planned_sides-Reihenfolge
        # konkateniert), abgetastet nach Zeit (siehe main_loop) -> beide Arme
        # fahren synchron. _planned_sides == [] bedeutet: keine Arm-Bewegung.
        self._planned_sides = []                      # z.B. ["left","right"] oder ["right"]
        self._planned_traj_t = None                   # (M,) Zeitstempel [s]
        self._planned_traj_q = None                   # (M, 7*len(sides))
        self._planned_time = 0.0                      # Fortschritt auf der Trajektorie [s]
        self._planned_stall_s = 0.0                   # wie lange die Uhr schon steht
        self._planned_last_tick = None                # time.monotonic() des letzten Ticks
        self._planned_stats = {"t0": 0.0, "ticks": 0, "held_s": 0.0}
        self._planned_pose_name = None
        self._planning_thread = None
        # Zuletzt empfangener Hand-Ist-Zustand (6 Inspire-DOF je Hand, 0..1000)
        # von der inspire-Bridge (siehe manipulation/inspire_ftp/bridge.py) --
        # nur zum SPEICHERN der Handposition. None = Bridge liefert (noch) nichts.
        self._hand_state = {"left": None, "right": None}
        # Hand-Ziele, die beim START der geplanten Armbewegung an die Bridge
        # gehen (damit Finger und Arme gemeinsam losfahren). Bei reiner
        # Handposition (ohne Arme) werden sie sofort gesendet.
        self._pending_hand_goals = {}
        self._plan_result = DataBuffer()               # Thread-sicherer Handoff Planer -> main_loop
        self._plan_result_seen = None
        # Generations-Zaehler: jeder POSE ANFAHREN erhoeht ihn, jeder Abbruch
        # (E-Stop/DISABLE/Marker/HOMING/WALK/ABBRECHEN) ebenfalls. Ein Planungs-
        # Ergebnis wird NUR aktiviert, wenn seine Generation noch die aktuelle
        # ist -- so kann ein Marker-Griff (oder E-Stop) WAEHREND einer noch
        # laufenden Planung deren spaeteres Ergebnis zuverlaessig entwerten,
        # ohne dass es schon vorliegen muesste (race-frei).
        self._plan_gen = 0

        self._T_off_right_static = self._mk_static_T(self._ee_off_right_xyz, self._ee_off_right_rpy_deg)
        self._T_off_left_static = self._mk_static_T(self._ee_off_left_xyz, self._ee_off_left_rpy_deg)
        self._T_off_right_auto = None
        self._T_off_left_auto = None
        self._auto_done_right = False
        self._auto_done_left = False

        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('g1pilot')
        urdf_path = os.path.join(pkg_share, 'description_files', 'urdf', 'g1_29dof_inspire_ftp.urdf')
        mesh_dir = os.path.join(pkg_share, 'description_files', 'meshes')
        self.ik_solver = G1IKSolver(urdf_path=urdf_path, mesh_dir=mesh_dir, debug=False)
        # Kugelmodell der Arme (Abstaende zu Objekten, siehe arm_geometry.py) fuer
        # das Laufzeit-Gate; der Planer-Prozess baut sich dasselbe Modell.
        self.arm_geom = ArmGeometry(self.ik_solver.model, self.ik_solver.collision_model)
        self._env_objects = []
        # Waehrend einer geplanten Fahrt: Start- und Zielkonfiguration (2,14) --
        # dort darf das Gate so tief wie die Enden selbst (gleiche Regel wie
        # der Planer). None = Marker-Betrieb.
        self._gate_end_q14 = None
        self._env_near_cache = (None, np.inf, None)
        self._gate_end_cache = (None, {})
        self.smoothing_ref_hz = float(self.get_parameter("smoothing_ref_hz").value)
        self._ik_goal_alpha_base = float(self.ik_solver.goal_filter_alpha)
        self._last_dt = 1.0 / float(self.get_parameter("rate_hz").value)
        if hasattr(self.ik_solver, "set_orientation_mode"):
            self.ik_solver.set_orientation_mode(self.ik_orientation_mode)
        # Eigener FK-Buffer fuer die Marker-Ziel-Pruefung (stoert self.data nicht).
        self._marker_fk_data = pin.Data(self.ik_solver.model)

        # Kartesisches Speedlimit: ueberwachte Punkte = Hand-TCPs + Ellbogen
        # (der Ellbogen kann bei reiner Schulterdrehung schneller sein als die
        # Hand). Eigenes pin.Data, damit die FK hier den Solver-Zustand nicht
        # anfasst.
        self._speed_data = pin.Data(self.ik_solver.model)
        self._speed_fids = [f for f in (self.ik_solver._fid_left,
                                        self.ik_solver._fid_right) if f is not None]
        for link in ("left_elbow_link", "right_elbow_link"):
            try:
                self._speed_fids.append(self.ik_solver.model.getFrameId(link))
            except Exception:
                pass
        if self.self_collision_gate and not getattr(self.ik_solver, "_gate_ready", False):
            self.get_logger().warn(
                "Selbstkollisions-Gate NICHT verfuegbar (s. IK-Log) -- "
                "Arm-Kommandos werden ungeprueft gesendet!")
        self._gate_last_warn = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Home-Pose je Arm (7 DOF, Reihenfolge:
        #   [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
        #    wrist_roll, wrist_pitch, wrist_yaw]).
        # Natuerliche Ruhepose: Arme leicht vor (pitch +), leicht seitlich aus
        # (roll links + / rechts -, gespiegelt!), Ellbogen leicht gebeugt (elbow +).
        # Als Parameter -> live tunebar via --ros-args -p home_left:="[...]".
        self.declare_parameter("home_left",  [0.3,  0.2, 0.0, 0.5, 0.0, 0.0, 0.0])
        self.declare_parameter("home_right", [0.3, -0.2, 0.0, 0.5, 0.0, 0.0, 0.0])
        self.home_left  = np.array(self.get_parameter("home_left").value,  dtype=float)
        self.home_right = np.array(self.get_parameter("home_right").value, dtype=float)
        # Ziel des HOMING-ARMS-Buttons (Streamdeck) bzw. »Grundstellung« (Demo-GUI):
        # die Lauf-Pose walk_left/right weiter unten (Arme neben dem Koerper).
        # Frueher Nullpose (Unterarm waagerecht nach vorn) -- siehe _homing_goals.

        # Ruhe-Pose fuer die IK-Nullraum-Regularisierung = Home-Pose. Haelt den
        # redundanten (7. DOF) Ellbogen-Swivel natuerlich -> der Arm faellt beim
        # Marker-Ziehen nicht in den zum Koerper gefalteten IK-Zweig.
        if hasattr(self.ik_solver, "set_rest_posture"):
            self.ik_solver.set_rest_posture("left",  self.home_left)
            self.ik_solver.set_rest_posture("right", self.home_right)
        self.ik_solver.null_space_gain = float(self.get_parameter("ik_null_space_gain").value)

        # Armplaner als EIGENER Prozess (hielte im Regelprozess das GIL und
        # braechte den 250-Hz-Takt aus dem Tritt). Startet im Hintergrund.
        self._planner = PlannerProcess(
            urdf_path, mesh_dir,
            rest={"left": self.home_left, "right": self.home_right},
            null_space_gain=self.ik_solver.null_space_gain, logger=self.get_logger())
        self._planner.start_async()

        # WALK-Pose: waehrend WALK haelt der arm_controller die Arme HIER (= Policy-
        # Default-Armpose des g1_wholebody-Policy). Headless validiert: die Lauf-Policy
        # laeuft mit fixen Armen NUR stabil, wenn sie nahe dieser Pose sind (Arme unten
        # am Koerper -> Sturz). Bei BALANCING sind die Arme wieder frei (rviz/Marker).
        self.declare_parameter("walk_left",  [0.35,  0.18, 0.0, 0.87, 0.0, 0.0, 0.0])
        self.declare_parameter("walk_right", [0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0])
        self.walk_left  = np.array(self.get_parameter("walk_left").value,  dtype=float)
        self.walk_right = np.array(self.get_parameter("walk_right").value, dtype=float)
        self.walk_mode = False
        # walk_park_arms=false (Sim mit der AGILE-Lauf-Policy, siehe bringup_sim):
        # Arme beim Laufen NICHT in die Lauf-Pose parken. Die Policy wurde mit
        # staendig bewegten Armen trainiert und braucht keine bestimmte Armpose ->
        # die Arme halten ihre Stellung und bleiben per Marker/Kommando bedienbar
        # (z.B. eine Box tragen). walk_ready kommt dann sofort.
        # NUR SIM: Real fuehrt beim Laufen Unitrees Onboard-Regler -- dort bleibt
        # es immer beim Parken, egal was der Parameter sagt.
        self.declare_parameter("walk_park_arms", True)
        self.walk_park_arms = bool(self.get_parameter("walk_park_arms").value)
        if not self.walk_park_arms and not is_sim_mode():
            self.get_logger().warn("walk_park_arms=false gilt nur in der Sim -> real: Arme parken.")
            self.walk_park_arms = True
        # Uebergang in die Lauf-Pose laeuft noch als GEPLANTE Bewegung (um Tisch/
        # Hindernisse herum, wie HOMING). Erst danach haelt der WALK-Zweig in
        # main_loop die Lauf-Pose und meldet walk_ready.
        self._walk_transition = False
        # Sobald die Arme die Lauf-Pose erreicht haben, meldet der Controller das per
        # /g1pilot/arms/walk_ready -> loco_sim laeuft erst DANN los (Arme aufgeraeumt).
        # Toleranz etwas lockerer als die Homing-Toleranz (die Haltepose muss nicht
        # exakt sitzen, nur "im Wesentlichen aufgeraeumt").
        self.declare_parameter("walk_ready_tolerance", 0.05)
        self.walk_ready_tolerance = float(self.get_parameter("walk_ready_tolerance").value)
        self._walk_ready_sent = False

        self.left_workspace_publisher = self.create_publisher(Marker, '/g1pilot/workspace/left', 10)
        self.right_workspace_publisher = self.create_publisher(Marker, '/g1pilot/workspace/right', 10)
        self.walk_ready_publisher = self.create_publisher(Bool, '/g1pilot/arms/walk_ready', 1)


        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        # Tiefe 1: nur das NEUESTE Hand-Ziel zaehlt. Der Executor bedient die
        # Subscription nur einmal pro main_loop-Takt; kommen Ziele schneller als
        # der Takt (PS4-Teleop 30 Hz, Sim-Takt 10-20 Hz), stauten sich mit Tiefe 10
        # bis zu 10 alte Ziele -> 0,3-0,8 s Verzug, bevor die Hand reagiert.
        self.create_subscription(PoseStamped, "/g1pilot/hand_goal/right", self._right_goal_callback, 1)
        self.create_subscription(PoseStamped, "/g1pilot/hand_goal/left", self._left_goal_callback, 1)
        self.create_subscription(Bool, "/g1pilot/arms/enabled", self._arms_controlled_callback, 10)
        self.create_subscription(Bool, "/g1pilot/arms/home", self._homming_callback, 10)
        # Loco-Zustand: bei WALK Arme in die Lauf-Pose, bei BALANCING wieder frei.
        self.create_subscription(Bool, "/g1pilot/start_walking", self._on_walk_mode, 10)
        self.create_subscription(Bool, "/g1pilot/start_balancing", self._on_balance_mode, 10)
        # E-Stop: sofort verstummen + Arme schlaff schalten (Weight=0, kp/kd=0).
        # /g1pilot/start quittiert den Latch (wie robot_stopped im loco_client).
        self.create_subscription(Bool, "/g1pilot/emergency_stop", self._on_emergency_stop, 10)
        self.create_subscription(Bool, "/g1pilot/start", self._on_start, 10)

        # Umgebungs-Objekte (Hindernisse + Greif-Objekte), siehe scene_bridge.py.
        # TRANSIENT_LOCAL, damit ein spaeter gestarteter arm_controller den
        # zuletzt veroeffentlichten Stand sofort bekommt (statt bis zum
        # naechsten scene_bridge-Publish-Tick auf leere Umgebung zu laufen).
        qos_scene = QoSProfile(depth=1)
        qos_scene.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(MarkerArray, "/scene_markers", self._on_scene_markers, qos_scene)

        # Positionsspeicher (Streamdeck-Buttons, siehe teleoperation/ui_interface.py).
        self.create_subscription(String, "/g1pilot/pose_store/save", self._on_pose_save, 10)
        self.create_subscription(String, "/g1pilot/pose_store/goto", self._on_pose_goto, 10)
        self.create_subscription(Bool, "/g1pilot/pose_store/cancel", self._on_pose_cancel, 10)

        # Live-Pose-Schnittstelle (siehe g1pilot/docs/21_arm_api_technik.md): fremde Projekte
        # spielen ein Ziel ein, das DIREKT ausgefuehrt wird -- nichts wird
        # gespeichert. JSON-Wire-Format in manipulation/arm_command.py; die
        # HTTP-Bruecke (arm_api.py) ist nur ein Adapter auf genau dieses Topic.
        self.create_subscription(String, "/g1pilot/arm_command", self._on_arm_command, 10)
        # Abbruch-Alias: gleiche Wirkung wie pose_store/cancel, aber ohne dass
        # ein API-Nutzer den Positionsspeicher-Namensraum kennen muss.
        self.create_subscription(Bool, "/g1pilot/arm_command/cancel", self._on_pose_cancel, 10)
        self.pub_cmd_status = self.create_publisher(String, "/g1pilot/arm_command/status", 10)
        # Rueckmeldung zu Marker-Zielen (reached/unreachable/cancelled/ignored),
        # damit der RViz-Marker weiss, wann er wieder der Hand folgen darf.
        self.pub_marker_goal_status = self.create_publisher(String, "/g1pilot/hand_goal_status", 10)
        # Rueckkanal des Speicherns (POST /arm/save bzw. Streamdeck-Dialog):
        # sagt, WAS unter welchem Namen/Kategorie wirklich in der Datei landete.
        self.pub_save_status = self.create_publisher(
            String, "/g1pilot/pose_store/save/status", 10)

        # Hand-Ist-Zustand von der inspire-Bridge (nur zum SPEICHERN der
        # Handposition) und Finger-Ziel zurueck (zum Wiederherstellen). Eigenes
        # Topic finger_goal: /g1pilot/hand_goal/<side> ist das PoseStamped-
        # Armziel (Marker), ein zweiter Nachrichtentyp darauf kollidiert. Laeuft die
        # Bridge nicht, bleibt _hand_state None -> Handposition wird uebersprungen.
        self.create_subscription(
            Float32MultiArray, "/g1pilot/hand_state/left",
            lambda m: self._on_hand_state("left", m), 10)
        self.create_subscription(
            Float32MultiArray, "/g1pilot/hand_state/right",
            lambda m: self._on_hand_state("right", m), 10)
        self.pub_finger_goal = {
            "left": self.create_publisher(Float32MultiArray, "/g1pilot/finger_goal/left", 10),
            "right": self.create_publisher(Float32MultiArray, "/g1pilot/finger_goal/right", 10),
        }

        self._init_robot_interface()

        self._last_tick_time = None
        self.timer = self.create_timer(1.0 / self.rate_hz, self.main_loop)

    def _arm_gravity_tau(self, q14):
        """Schwerkraft-Drehmomente [Nm] fuer die 14 Arm-Gelenke bei der
        Konfiguration q14 (7 links + 7 rechts) aus dem Pinocchio-Modell."""
        tau = np.zeros(14, dtype=float)
        if not bool(self.get_parameter("gravity_comp").value):
            return tau
        try:
            ik = self.ik_solver
            q_full = pin.neutral(ik.model)
            for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                q_full[ik._name_to_q_index[ik._ros_joint_names[arm_i]]] = float(q14[i])
            for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                q_full[ik._name_to_q_index[ik._ros_joint_names[arm_i]]] = float(q14[7 + i])
            g = pin.computeGeneralizedGravity(ik.model, ik.data, q_full)
            for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                tau[i] = float(g[ik._name_to_v_index[ik._ros_joint_names[arm_i]]])
            for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                tau[7 + i] = float(g[ik._name_to_v_index[ik._ros_joint_names[arm_i]]])
            np.clip(tau, -20.0, 20.0, out=tau)
        except Exception:
            tau[:] = 0.0
        return tau

    def _mk_static_T(self, xyz, rpy_deg):
        """
        Create a fixed SE3 transform from XYZ translation and RPY rotation in degrees.

        Parameters
        ----------
        xyz : array-like of float
            Translation vector [x, y, z] in meters.
        rpy_deg : array-like of float
            Roll, pitch, yaw angles in degrees.

        Returns
        -------
        pinocchio.SE3
            Homogeneous transform combining translation and rotation.
        """

        rpy = np.radians(np.array(rpy_deg, dtype=float))
        R = pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2])
        return SE3(R, np.array(xyz, dtype=float))
    
    def _goal_error(self, side: str, T_goal: SE3):
        """
        Compute position and orientation error between current and target end-effector pose.

        Parameters
        ----------
        side : str
            Arm identifier ('left' or 'right').
        T_goal : SE3
            Desired end-effector target pose.

        Returns
        -------
        tuple(float, float)
            (position_error_m, orientation_error_rad)
        """

        M_cur = self._fk_current_ee(side)
        if M_cur is None or T_goal is None:
            return None, None
        dp = float(np.linalg.norm(T_goal.translation - M_cur.translation))
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal.rotation)
        ang = 2.0 * math.atan2(
            math.sqrt(dq.x*dq.x + dq.y*dq.y + dq.z*dq.z),
            abs(dq.w)
        )
        return dp, ang

    def _lowpass_goal(self, T_prev: SE3, T_new: SE3, alpha: float) -> SE3:
        """
        Apply exponential smoothing between previous and new SE3 goals.

        Parameters
        ----------
        T_prev : SE3
            Previous goal pose.
        T_new : SE3
            New goal pose.
        alpha : float
            Low-pass coefficient (0.0–1.0).

        Returns
        -------
        SE3
            Smoothed goal transform.
        """

        if T_prev is None:
            return T_new
        p = (1.0 - alpha) * T_prev.translation + alpha * T_new.translation
        q0 = _mat_to_quat_wxyz(T_prev.rotation)
        q1 = _mat_to_quat_wxyz(T_new.rotation)
        qf = (1 - alpha) * q0 + alpha * q1
        qf = qf / np.linalg.norm(qf)
        Rf = _quat_wxyz_to_matrix(qf)
        return SE3(Rf, p)

    def _limit_ori_step(self, R_cur: np.ndarray, R_des: np.ndarray, max_step: float) -> np.ndarray:
        """
        Limit the angular step between current and desired rotation matrices.

        Parameters
        ----------
        R_cur : np.ndarray
            Current 3×3 rotation matrix.
        R_des : np.ndarray
            Desired 3×3 rotation matrix.
        max_step : float
            Maximum angular step (rad).

        Returns
        -------
        np.ndarray
            Limited rotation matrix.
        """

        R_err = R_cur.T @ R_des
        aa = pin.log3(R_err)
        nrm = float(np.linalg.norm(aa))
        if nrm <= 1e-12 or nrm <= max_step:
            return R_des
        aa_lim = aa * (max_step / nrm)
        return R_cur @ pin.exp3(aa_lim)

    def _fk_current_ee(self, side: str):
        """
        Compute the current end-effector pose (SE3) for the given arm using FK.

        Parameters
        ----------
        side : str
            'left' or 'right'.

        Returns
        -------
        SE3 or None
            Forward kinematics result for the end-effector.
        """

        try:
            q_full = pin.neutral(self.ik_solver.model)
            cur_all = self.get_current_motor_q() if self.use_robot else self._assemble_full_from_last()
            for jid_idx, ros_name in enumerate(self.ik_solver._ros_joint_names):
                if ros_name in self.ik_solver._name_to_q_index:
                    q_full[self.ik_solver._name_to_q_index[ros_name]] = float(cur_all[jid_idx])
            pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
            pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)
            fid = self.ik_solver._fid_right if side == 'right' else self.ik_solver._fid_left
            if fid is None:
                return None
            return self.ik_solver.data.oMf[fid]
        except Exception:
            return None

    def _gate_auto_calibration(self, T_goal_in: SE3, side: str):
        """
        Check if the current end-effector pose is close enough to the incoming goal
        to perform automatic end-effector calibration.

        Parameters
        ----------
        T_goal_in : SE3
            Incoming target pose before any static offset is applied.
        side : str
            Arm identifier ('left' or 'right').

        Returns
        -------
        SE3 or None
            Current SE3 pose if within calibration threshold, otherwise None.
        """

        M_cur = self._fk_current_ee(side)
        if M_cur is None:
            return None
        dp = np.linalg.norm(T_goal_in.translation - M_cur.translation)
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal_in.rotation)
        ang = 2 * math.atan2(np.linalg.norm([dq.x, dq.y, dq.z]), abs(dq.w))
        if dp < 0.05 and ang < math.radians(12.0):
            return M_cur
        return None

    def _apply_offsets_and_filters(self, side: str, T_goal_input: SE3):
        """Offsets anwenden und filtern (siehe _goal_with_offsets/_filter_goal)."""
        return self._filter_goal(side, self._goal_with_offsets(side, T_goal_input))

    def _goal_with_offsets(self, side: str, T_goal_input: SE3):
        """
        Apply static and auto-calibrated offsets to an incoming goal.

        This function handles end-effector calibration (static + automatic);
        smoothing and orientation step limitation happen in _filter_goal.

        Parameters
        ----------
        side : str
            Arm identifier ('left' or 'right').
        T_goal_input : SE3
            Raw goal transform.

        Returns
        -------
        SE3
            Goal with offsets applied (unfiltered).
        """

        T_static = self._T_off_right_static if side == 'right' else self._T_off_left_static
        T_auto = self._T_off_right_auto if side == 'right' else self._T_off_left_auto
        auto_done = self._auto_done_right if side == 'right' else self._auto_done_left

        if self.ee_auto_calibrate and not auto_done:
            M_cur_ok = self._gate_auto_calibration(T_goal_input, side)
            if M_cur_ok is not None:
                T_pre = T_goal_input * T_static
                T_auto_new = T_pre.inverse() * M_cur_ok
                if side == 'right':
                    self._T_off_right_auto = T_auto_new; self._auto_done_right = True
                    t = T_auto_new.translation
                    self.get_logger().info(f"[IK] auto-calibrated right: d=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")
                else:
                    self._T_off_left_auto = T_auto_new; self._auto_done_left = True
                    t = T_auto_new.translation
                    self.get_logger().info(f"[IK] auto-calibrated left: d=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")
                T_auto = T_auto_new

        return T_goal_input * T_static * (T_auto if T_auto is not None else SE3.Identity())

    def _filter_goal(self, side: str, T_raw: SE3):
        """Ziel (mit Offsets) tiefpassfiltern und den Orientierungsschritt
        gegenueber der aktuellen Hand begrenzen. Der Filter macht pro Aufruf nur
        einen Schritt -- deshalb ruft _update_marker_goals ihn periodisch mit
        dem Endziel erneut auf, sonst bliebe der Sollwert vor dem Marker stehen."""
        if side == 'right':
            self._goal_right_filt = self._lowpass_goal(self._goal_right_filt, T_raw, self.ik_goal_filter_alpha)
            T_use = self._goal_right_filt
        else:
            self._goal_left_filt = self._lowpass_goal(self._goal_left_filt, T_raw, self.ik_goal_filter_alpha)
            T_use = self._goal_left_filt

        M_cur = self._fk_current_ee(side)
        if (M_cur is not None) and (T_use is not None):
            R_lim = self._limit_ori_step(M_cur.rotation, T_use.rotation, self.ik_max_ori_step_rad)
            T_use = SE3(R_lim, T_use.translation.copy())

        return T_use

    def _init_robot_interface(self):
        """Initialize DDS interface for robot communication."""

        init_dds(self.interface, self.get_logger())

        self.lowstate_subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        self.lowstate_subscriber.Init()

        self.subscribe_thread = threading.Thread(
            target=self._subscribe_motor_state, daemon=True
        )
        self.subscribe_thread.start()

        self.lowcmd_publisher = ChannelPublisher('rt/arm_sdk', LowCmd_)
        self.lowcmd_publisher.Init()

        while not self.lowstate_buffer.GetData():
            self.get_logger().info("Waiting for LowState data from DDS...")
            time.sleep(0.01)

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()
        self.all_motor_q = self.get_current_motor_q()

        # PD-Gains. Defaults = Sim-Tuning (kd hoch, weil die MuJoCo-Gelenke
        # sonst unterdaempft um die Sollpose schwingen). Das REAL-Bringup
        # (bringup_real -> manipulation_launcher) ueberschreibt auf die im
        # offiziellen Unitree-g1_arm7_sdk-Beispiel erprobten Werte kp=60,
        # kd=1.5 -- NICHT die Sim-Werte auf echte Motoren loslassen.
        def _gain(name, default):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            return float(self.get_parameter(name).value)

        self.kp_low   = _gain("kp_low",   150.0); self.kd_low   = _gain("kd_low",   12.0)
        self.kp_wrist = _gain("kp_wrist",  40.0); self.kd_wrist = _gain("kd_wrist",  4.0)
        if not is_sim_mode():
            self._apply_real_safety_limits()
        self._accel_limit = float(self.get_parameter("arm_accel_limit").value)
        self._max_track_err = float(self.get_parameter("max_tracking_error").value)

        # NUR die 14 Arm-Gelenke (15..28) + Weight (29) werden je beschrieben --
        # exakt wie im offiziellen Unitree-g1_arm7_sdk-Beispiel. Alle anderen
        # Eintraege werden hier einmalig explizit inert gesetzt und danach NIE
        # angefasst: Beine gehoeren nicht in rt/arm_sdk, und die Taille (12-14)
        # ist via arm_sdk real ansteuerbar -- ein Kommando dort wuerde gegen
        # den Unitree-Loco-Controller kaempfen. (Frueher schrieb
        # _hold_non_arm_joints Beine+Taille mit kp=300 in die Message.)
        arm_vals = {m.value for m in G1_29_JointArmIndex}
        for jid in G1_29_JointIndex:
            if jid.value in arm_vals:
                continue
            mc = self.msg.motor_cmd[jid]
            mc.mode = 0
            mc.q = 0.0; mc.dq = 0.0; mc.tau = 0.0; mc.kp = 0.0; mc.kd = 0.0

        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in wrist_vals:
                self.msg.motor_cmd[jid].kp = self.kp_wrist
                self.msg.motor_cmd[jid].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low
            self.msg.motor_cmd[jid].q = float(self.all_motor_q[jid.value])

        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)
        self._initialized = True

    def _subscribe_motor_state(self):
        """
        Thread loop continuously reading motor state messages from DDS.

        Runs as a daemon thread to asynchronously update internal motor states
        and store the latest LowState message in `lowstate_buffer`.

        Notes
        -----
        - This method blocks indefinitely while ROS 2 is running.
        - Updates both joint positions and velocities for each motor.
        """

        while rclpy.ok():
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self.lowstate_buffer.SetData(msg)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = msg.motor_state[i].q
                    self.motor_state[i].dq = msg.motor_state[i].dq
            time.sleep(0.001)

    def get_mode_machine(self) -> int:
        """
        Get the current robot mode from the latest LowState message.

        Returns
        -------
        int
            Current machine mode identifier, or 0 if no data available.
        """

        msg = self.lowstate_buffer.GetData()
        return getattr(msg, "mode_machine", 0) if msg is not None else 0

    def get_current_motor_q(self) -> np.ndarray:
        """
        Retrieve the current joint positions (q) for all robot joints.

        Returns
        -------
        np.ndarray
            Array of joint positions (rad) ordered according to `G1_29_JointIndex`.
        """

        msg = self.lowstate_buffer.GetData()
        return np.array([msg.motor_state[id].q for id in G1_29_JointIndex], dtype=float)

    # --------------------------------------------------------
    # Echt-Roboter-Absicherung: Grenzen, Rueckgabe, Programmende
    # --------------------------------------------------------

    # Obergrenzen auf dem ECHTEN G1, egal welches Launchfile startet (frueher
    # bekam z.B. bringup_launcher / `make real-full` die Sim-Gains kp=150/kd=12).
    REAL_MAX_GAINS = {"kp_low": 80.0, "kd_low": 3.0, "kp_wrist": 50.0, "kd_wrist": 3.0}
    REAL_MIN_RAMP_S = 1.0
    REAL_MAX_ARM_VEL = 1.5      # rad/s
    REAL_MAX_EE_VEL = 0.25      # m/s

    def _apply_real_safety_limits(self) -> None:
        """Nur real: Gains, Rampen und Speedlimits auf sichere Werte begrenzen.
        Ein falsches Launchfile darf die Arme nicht hart/schnell machen."""
        from rclpy.parameter import Parameter
        log = self.get_logger()
        for name, cap in self.REAL_MAX_GAINS.items():
            if getattr(self, name) > cap:
                log.error(f"REAL: {name}={getattr(self, name):.1f} ist zu hart -> {cap:.1f} "
                          "(Sim-Gains auf echten Motoren?).")
                setattr(self, name, cap)
        fixes = []
        for name in ("arm_weight_ramp_up_s", "arm_weight_ramp_down_s"):
            if float(self.get_parameter(name).value) < self.REAL_MIN_RAMP_S:
                fixes.append(Parameter(name, Parameter.Type.DOUBLE, 2.0))
        if not (0.0 < self.arm_velocity_limit <= self.REAL_MAX_ARM_VEL):
            self.arm_velocity_limit = self.REAL_MAX_ARM_VEL
            fixes.append(Parameter("arm_velocity_limit", Parameter.Type.DOUBLE,
                                   self.REAL_MAX_ARM_VEL))
        if not (0.0 < self.ee_velocity_limit <= self.REAL_MAX_EE_VEL):
            self.ee_velocity_limit = self.REAL_MAX_EE_VEL
            fixes.append(Parameter("ee_velocity_limit", Parameter.Type.DOUBLE,
                                   self.REAL_MAX_EE_VEL))
        if fixes:
            self.set_parameters(fixes)
            log.error("REAL: unsichere Parameter korrigiert: "
                      + ", ".join(f"{p.name}={p.value}" for p in fixes))

    def _measured_arm_q(self) -> np.ndarray:
        """Gemessene Armstellung (7 links + 7 rechts)."""
        cur = self.get_current_motor_q()
        return np.array([cur[j] for j in LEFT_JOINT_INDICES_LIST + RIGHT_JOINT_INDICES_LIST],
                        dtype=float)

    def _record_onboard_pose(self, why: str) -> None:
        """Armstellung merken, solange der Onboard-Regler die Arme noch ALLEIN
        fuehrt (Gewicht 0, kein Slack) -- dorthin faehrt die Rueckgabe zurueck."""
        if (not self.use_robot or self._arm_slack or self._weight_state != "off"
                or self._arm_sdk_weight > 0.0):
            return
        try:
            q = self._measured_arm_q()
        except Exception:
            return
        lim = [JOINT_LIMITS_RAD[j] for j in LEFT_JOINT_INDICES_LIST + RIGHT_JOINT_INDICES_LIST]
        lo = np.array([a for a, _ in lim]); hi = np.array([b for _, b in lim])
        self._onboard_q = np.clip(q, lo, hi)
        self.get_logger().info(f"Onboard-Armpose gemerkt ({why}) -- Ziel der spaeteren Rueckgabe.")

    def _release_possible(self) -> bool:
        ramp = float(self.get_parameter("arm_weight_ramp_down_s").value)
        return bool(self.use_robot and self.arms_enabled and not self.estop_active
                    and not self._arm_slack and ramp > 0.0
                    and bool(self.get_parameter("release_to_onboard_pose").value)
                    and self._weight_state in ("ramp_up", "on") and self._arm_sdk_weight > 0.0)

    def _start_release(self) -> bool:
        """DISABLE: Rueckfahrt in die Onboard-Pose starten (geplant). -> True,
        wenn die Rueckgabe uebernommen wurde (Aufrufer macht dann nichts mehr)."""
        if not self._release_possible():
            return False
        if self._onboard_q is None:
            self.get_logger().warn("DISABLE: Onboard-Armpose unbekannt -- weiche Rampe ohne "
                                   "Rueckfahrt (Arme blenden in 2 s in die Onboard-Pose).")
            return False
        self._cancel_marker_goals("Rueckgabe an den Onboard-Regler")
        self._abort_planned_motion("DISABLE")
        self.homing_active = False
        self.homing_reached = False
        # Real: beim Laufen fuehrt danach der Onboard-Regler die Arme selbst.
        self.walk_mode = False
        self._walk_transition = False
        tol = float(self.get_parameter("release_tolerance").value)
        if float(np.max(np.abs(self._last_q_target - self._onboard_q))) <= tol:
            self._finish_release("Arme stehen schon in der Onboard-Pose")
            return True
        self._releasing = True
        self._release_direct = False
        self._release_t0 = time.monotonic()
        self._release_best = (np.inf, self._release_t0)
        self.get_logger().info("DISABLE: Arme fahren geplant in die Onboard-Pose zurueck, "
                               "danach weiche Uebergabe (Gewicht 1->0).")
        self._start_planned_motion(
            "Rueckgabe an den Onboard-Regler", ["left", "right"],
            {"left": self._onboard_q[0:7].copy(), "right": self._onboard_q[7:14].copy()},
            {}, source="release")
        return True

    def _finish_release(self, why: str) -> None:
        """Rueckfahrt beendet (oder abgebrochen): Arme abgeben, Gewicht 1->0."""
        self._releasing = False
        self._release_direct = False
        if self._planned_motion_active or (
                self._planning_thread is not None and self._planning_thread.is_alive()):
            self._abort_planned_motion("Rueckgabe")
        self.arms_enabled = False
        self._cancel_marker_goals("Manipulation deaktiviert")
        if self._weight_state in ("ramp_up", "on") and self._arm_sdk_weight > 0.0:
            self._weight_state = "ramp_down"
        self.get_logger().info(f"Rueckgabe: {why} -- Gewicht 1->0, Onboard-Regler uebernimmt.")

    def _limit_accel(self, q_prev: np.ndarray, q_new: np.ndarray, dt: float) -> np.ndarray:
        """Anfahrrampe: Gelenkgeschwindigkeit (max ueber die Gelenke) darf je
        Tick nur um arm_accel_limit*dt steigen. Richtungserhaltend; Bremsen
        und Anhalten bleiben unbegrenzt."""
        a = self._accel_limit
        if a <= 0.0 or dt <= 0.0:
            return q_new
        step = q_new - q_prev
        s = float(np.max(np.abs(step)))
        if s < 1e-12:
            return q_new
        v = s / dt
        v_ok = self._last_cmd_speed + a * dt
        if v <= v_ok:
            return q_new
        return q_prev + step * (v_ok / v)

    def _limit_tracking_error(self, q_new: np.ndarray) -> np.ndarray:
        """Kommando auf +-max_tracking_error um die GEMESSENE Stellung klemmen
        (nur bei voller arm_sdk-Autoritaet). Weicht der Arm aus (festgehalten,
        angestossen), gibt der Sollwert nach statt Kraft aufzubauen."""
        e = self._max_track_err
        if e <= 0.0 or not self.use_robot or self._weight_state != "on":
            return q_new
        try:
            meas = self._measured_arm_q()
        except Exception:
            return q_new
        return np.clip(q_new, meas - e, meas + e)

    def _write_arm_cmd(self, q14: np.ndarray, weight: float) -> None:
        """Arm-Kommando (14 Gelenke, PD + Schwerkraft-Feedforward) mit dem
        gegebenen arm_sdk-Gewicht auf rt/arm_sdk schreiben."""
        self.msg.mode_machine = self.get_mode_machine()
        # mode_pr NICHT setzen (bleibt 0 wie im Unitree-arm_sdk-Beispiel;
        # betrifft nur die Knoechel-Interpretation, die uns nichts angeht).
        try:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = float(weight)
        except Exception:
            pass
        # Schwerkraft-Feedforward gegen den PD-Durchhang (s. Parameter).
        tau_g = self._arm_gravity_tau(q14)
        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for idx, jid in enumerate(G1_29_JointArmIndex):
            mc = self.msg.motor_cmd[jid]
            mc.mode = 1
            mc.q = float(q14[idx])
            mc.dq = 0.0
            mc.tau = float(tau_g[idx])
            if jid.value in wrist_vals:
                mc.kp = self.kp_wrist
                mc.kd = self.kd_wrist
            else:
                mc.kp = self.kp_low
                mc.kd = self.kd_low
        self.msg.crc = self.crc.Crc(self.msg)
        self.lowcmd_publisher.Write(self.msg)

    def shutdown_release(self) -> None:
        """Programmende (Strg+C, docker stop, Absturz) NUR REAL: die Arme nicht
        einfach fallen lassen. Haelt arm_sdk die Arme, fahren sie langsam
        (direkt, Gates aktiv) Richtung Onboard-Pose und das Gewicht rampt dann
        1->0 -- wie bei DISABLE, nur ohne Planer (der Prozess endet gerade).
        Im E-Stop/Slack: nur das Gewicht abrampen (kp bleibt 0)."""
        if not self.use_robot or is_sim_mode() or not getattr(self, "_initialized", False):
            return
        import signal
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, signal.SIG_IGN)   # Rueckgabe nicht abwuergen
            except Exception:
                pass
        w = float(self._arm_sdk_weight)
        if w <= 0.0 and self._weight_state == "off":
            return
        rate = 250.0
        dt = 1.0 / rate

        def _param(name, default):
            try:
                return float(self.get_parameter(name).value)
            except Exception:
                return default

        ramp = max(self.REAL_MIN_RAMP_S, _param("arm_weight_ramp_down_s", 2.0))
        if self._arm_slack and not self.arms_enabled:
            print("[arm_controller] Programmende im E-Stop: Gewicht 1->0 (Arme bleiben schlaff).",
                  flush=True)
            while w > 0.0:
                w = max(0.0, w - dt / ramp)
                self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = float(w)
                self.msg.crc = self.crc.Crc(self.msg)
                self.lowcmd_publisher.Write(self.msg)
                time.sleep(dt)
            return
        q = self._last_q_target.copy()
        target = self._onboard_q
        if (target is not None and self._weight_state != "ramp_down"
                and bool(self.get_parameter("release_to_onboard_pose").value)):
            print("[arm_controller] Programmende: Arme fahren langsam in die Onboard-Pose, "
                  "dann Uebergabe.", flush=True)
            tol = _param("release_tolerance", 0.05)
            vmax = min(self.arm_velocity_limit, 0.8)
            t_end = time.monotonic() + _param("shutdown_return_s", 2.5)
            v_prev = 0.0
            while time.monotonic() < t_end:
                err = target - q
                m = float(np.max(np.abs(err)))
                if m <= tol:
                    break
                v = min(vmax, v_prev + 3.0 * dt)       # sanft anfahren
                q_new = q + err * min(1.0, v * dt / m)
                q_new = self._limit_cartesian_speed(q, q_new, dt)
                q_new = self._apply_collision_gate(q, q_new)
                moved = float(np.max(np.abs(q_new - q)))
                if moved < 1e-7:
                    break                               # Gate haelt -> von hier abgeben
                v_prev = moved / dt
                q = q_new
                self._write_arm_cmd(q, w)
                time.sleep(dt)
        print(f"[arm_controller] Programmende: arm_sdk-Gewicht {w:.2f}->0 in {ramp:.1f} s.",
              flush=True)
        while w > 0.0:
            w = max(0.0, w - dt / ramp)
            self._write_arm_cmd(q, w)
            time.sleep(dt)
        self._arm_sdk_weight = 0.0
        self._weight_state = "off"
        print("[arm_controller] Arme an den Onboard-Regler uebergeben.", flush=True)

    def _assemble_full_from_last(self) -> np.ndarray:
        """
        Build a complete 29-DOF joint configuration vector from the latest arm targets.

        Combines left and right 7-DOF joint targets into a full-body vector.

        Returns
        -------
        np.ndarray
            Full 29-element joint configuration array.
        """

        full = np.zeros(29, dtype=float)
        for i, jidx in enumerate(LEFT_JOINT_INDICES_LIST):
            full[jidx] = self._last_q_target[i]
        for i, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
            full[jidx] = self._last_q_target[7 + i]
        return full

    def _full_config_with_arms(self, q14: np.ndarray) -> np.ndarray:
        """29-DOF-Konfiguration: Arme aus q14, Rest (Beine/Taille) aus der
        Messung (real) bzw. neutral (Sim)."""
        if self.use_robot:
            try:
                full = self.get_current_motor_q()
            except Exception:
                full = np.zeros(29, dtype=float)
        else:
            full = np.zeros(29, dtype=float)
        for i, jidx in enumerate(LEFT_JOINT_INDICES_LIST):
            full[jidx] = q14[i]
        for i, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
            full[jidx] = q14[7 + i]
        return full

    def _arm_points(self, q14: np.ndarray) -> np.ndarray:
        """Positionen der ueberwachten Punkte (Hand-TCPs + Ellbogen) fuer die
        Arm-Konfiguration q14 (Nicht-Arm-Gelenke neutral -- fuer die
        RELATIVE Verschiebung zwischen zwei Ticks ausreichend)."""
        ik = self.ik_solver
        q_full = pin.neutral(ik.model)
        for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
            q_full[ik._name_to_q_index[ik._ros_joint_names[arm_i]]] = float(q14[i])
        for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
            q_full[ik._name_to_q_index[ik._ros_joint_names[arm_i]]] = float(q14[7 + i])
        pin.forwardKinematics(ik.model, self._speed_data, q_full)
        pin.updateFramePlacements(ik.model, self._speed_data)
        return np.array([self._speed_data.oMf[f].translation for f in self._speed_fids])

    def _limit_cartesian_speed(self, q_prev: np.ndarray, q_new: np.ndarray,
                               dt: float) -> np.ndarray:
        """Schritt q_prev->q_new so skalieren, dass kein ueberwachter Punkt
        schneller als ee_velocity_limit [m/s] faehrt (ISO 10218-1 reduced
        speed). Skaliert den GESAMTEN Gelenkschritt richtungserhaltend."""
        if self.ee_velocity_limit <= 0.0 or not self._speed_fids:
            return q_new
        dq = q_new - q_prev
        if float(np.max(np.abs(dq))) < 1e-9:
            return q_new
        try:
            disp = float(np.max(np.linalg.norm(
                self._arm_points(q_new) - self._arm_points(q_prev), axis=1)))
        except Exception:
            return q_new
        lim = self.ee_velocity_limit * dt
        if disp <= lim or disp < 1e-12:
            return q_new
        return q_prev + dq * (lim / disp)

    def _self_in_collision(self, full: np.ndarray, hard: bool = False) -> bool:
        """Selbstkollision (exakte Mesh-Pruefung, coal) der 29-DOF-Konfiguration.
        hard=False: unter der Sicherheitsmarge (ik_solver.collision_margin)."""
        return bool(self.self_collision_gate
                    and self.ik_solver.arm_command_in_collision(full, hard=hard))

    def _env_pairs(self, q14_rows: np.ndarray):
        """Kugel-Objekt-Abstaende beider Arme fuer die Zeilen von q14_rows.
        -> dict side -> (d (R,S,N), n (R,S,N,3), fk) oder {} ohne Objekte."""
        g = self.arm_geom
        if not self.environment_collision_gate or g.env.n == 0:
            return {}
        fk = g.fk(q14_rows, ["left", "right"])
        out = {}
        for side in ("left", "right"):
            ei = g.env_idx(side)
            d, n = g.env_dist(fk[side]["c"][:, ei], g.radii(side)[ei])
            out[side] = (d, n, fk[side])
        return out

    def _env_floor(self, side: str) -> np.ndarray:
        """Untergrenze je (Kugel, Objekt): Hand gate_hand_floor (aufsetzen
        erlaubt), Arm gate_arm_floor; Hand gegen Greif-Objekte unbegrenzt
        (ACM). Waehrend einer geplanten Fahrt zusaetzlich so tief wie Start
        bzw. Ziel der Bahn (dieselbe Regel wie im Planer)."""
        g = self.arm_geom
        ei = g.env_idx(side)
        hand = g.links(side)[ei] >= HAND_LINK_MIN
        fl = np.where(hand, float(self.get_parameter("gate_hand_floor").value),
                      float(self.get_parameter("gate_arm_floor").value))[:, None]
        fl = np.repeat(fl, g.env.n, axis=1)
        fl[hand[:, None] & g.env.is_grasp[None, :]] = -np.inf
        ends = self._gate_end_q14
        if ends is not None:
            key = (id(g.env), id(ends))
            if self._gate_end_cache[0] != key:
                pairs = self._env_pairs(ends)
                self._gate_end_cache = (key, {sd: p[0].min(axis=0) for sd, p in pairs.items()})
            d_end = self._gate_end_cache[1].get(side)
            if d_end is not None and d_end.shape == fl.shape:
                fl = np.minimum(fl, d_end - 0.003)
        return fl

    def _env_near(self, q_prev: np.ndarray, q_new: np.ndarray) -> bool:
        """Schneller Vorab-Test fuer den 250-Hz-Tick: ist irgendein Armpunkt
        so nah an seiner Untergrenze, dass dieser Schritt sie erreichen (oder
        die Verlangsamung greifen) koennte? Sonst entfaellt die volle Pruefung.
        Abstaende an q_prev werden gecacht (q_prev = Ergebnis des letzten Ticks)."""
        key = (q_prev.tobytes(), id(self._gate_end_q14))
        if self._env_near_cache[0] != key or self._env_near_cache[2] is not self.arm_geom.env:
            pairs = self._env_pairs(q_prev[None])
            gap = min((float(np.min(d[0] - self._env_floor(sd))) for sd, (d, _, _) in pairs.items()
                       if d.size), default=np.inf)
            self._env_near_cache = (key, gap, self.arm_geom.env)
        gap = self._env_near_cache[1]
        # Obere Schranke fuer die Verschiebung eines Armpunkts in diesem Schritt
        # (7 Gelenke, Hebel <= Armreichweite).
        step = 7.0 * float(np.max(np.abs(q_new - q_prev))) * 0.75
        return gap < float(self.get_parameter("gate_slow_dist").value) + step + 0.01

    def _env_gate(self, q_prev: np.ndarray, q_new: np.ndarray):
        """Gate gegen Umgebungs-Objekte auf Abstandsbasis (Kugelmodell).

        Ein Paar (Armkugel, Objekt) unter seiner Untergrenze darf sich nicht
        weiter annaehern -- alles andere ist erlaubt. Damit blockiert ein
        Start dicht am oder minimal im Tisch NICHT mehr jede Bewegung: weg
        vom Tisch und entlang der Platte geht immer, nur tiefer hinein nicht.
        Wuerde der Schritt ein Paar verletzen, wird er auf die erlaubte
        Richtung projiziert (die Hand gleitet an der Oberflaeche entlang,
        statt einzufrieren). -> (q, gehalten?)"""
        pairs = self._env_pairs(np.stack([q_prev, q_new]))
        if not pairs:
            return q_new, False
        dq = q_new - q_prev
        blocked = []          # (side, Gradient dd/dq14) je verletzendem Paar
        g = self.arm_geom
        for k, side in enumerate(("left", "right")):
            d, n, fk = pairs[side]
            fl = self._env_floor(side)
            bad = (d[1] < fl) & (d[1] < d[0] - 1e-5)
            if not np.any(bad):
                continue
            ei = g.env_idx(side)
            links = g.links(side)
            fk0 = {kk: v[0] for kk, v in fk.items()}
            for si, oi in zip(*np.nonzero(bad)):
                F = np.zeros((len(links), 3))
                F[ei[si]] = n[0, si, oi]
                grad7 = g.forces_to_q(fk0, links, F)
                grad = np.zeros(14)
                grad[7 * k:7 * k + 7] = grad7
                blocked.append(grad)
        if not blocked:
            return q_new, False
        # Annaehernde Anteile entfernen (zwei Durchlaeufe genuegen praktisch).
        for _ in range(2):
            for gr in blocked:
                gg = float(gr @ gr)
                proj = float(gr @ dq)
                if gg > 1e-12 and proj < 0.0:
                    dq = dq - (proj / gg) * gr
        q_slide = q_prev + dq
        pairs = self._env_pairs(np.stack([q_prev, q_slide]))
        for side in ("left", "right"):
            d = pairs[side][0]
            if np.any((d[1] < self._env_floor(side)) & (d[1] < d[0] - 1e-5)):
                return q_prev, True
        return q_slide, False

    def _env_slowdown(self, q_prev: np.ndarray, q_new: np.ndarray) -> np.ndarray:
        """Marker-Steuerung: naehert sich ein Armpunkt einem Objekt auf weniger
        als gate_slow_dist, wird der Schritt verkleinert (bis 25 %) -> die Hand
        setzt sanft auf, statt mit vollem Tempo anzuschlagen."""
        slow = float(self.get_parameter("gate_slow_dist").value)
        if slow <= 0.0:
            return q_new
        pairs = self._env_pairs(np.stack([q_prev, q_new]))
        factor = 1.0
        for side, (d, _, _) in pairs.items():
            fl = self._env_floor(side)
            gap = d - np.where(np.isfinite(fl), fl, -np.inf)[None]
            approaching = (d[1] < d[0]) & np.isfinite(gap[0]) & (gap[0] < slow)
            if np.any(approaching):
                factor = min(factor, max(0.25, float(gap[0][approaching].min()) / slow))
        return q_prev + (q_new - q_prev) * factor

    def _apply_collision_gate(self, q_prev: np.ndarray, q_new: np.ndarray) -> np.ndarray:
        """Kollisions-Gate auf dem KOMMANDIERTEN Schritt, pro Tick, fuer JEDEN
        Sollwert (Marker-Servoing UND geplante Bahn -- auch eine geplante Bahn
        bekommt so ein Sicherheitsnetz, falls sich die Umgebung bewegt hat).

        Selbstkollision (exakte Meshes, coal):
          * frei -> ok; HART -> halten.
          * im Margenband: geplante Bahn ok (vom Planer geprueft), sonst nur,
            wenn der aktuelle Zustand auch schon im Band ist (Herausfahren).
        Umgebung (Kugelmodell, Abstaende): siehe _env_gate."""
        if float(np.max(np.abs(q_new - q_prev))) < 1e-9:
            return q_new
        try:
            if (self.environment_collision_gate and self.arm_geom.env.n
                    and self._env_near(q_prev, q_new)):
                if not self._planned_motion_active:
                    q_new = self._env_slowdown(q_prev, q_new)
                q_new, held = self._env_gate(q_prev, q_new)
                if held:
                    self._gate_warn("Umgebung")
                    return q_prev
            full_new = self._full_config_with_arms(q_new)
            if self._self_in_collision(full_new, hard=True):
                self._gate_warn("Koerper")
                return q_prev
            if (self._self_in_collision(full_new) and not self._planned_motion_active
                    and not self._self_in_collision(self._full_config_with_arms(q_prev))):
                self._gate_warn("Koerper")
                return q_prev
            return q_new
        except Exception as e:
            self.get_logger().warn(f"Kollisions-Gate-Fehler: {e} -- halte Pose.")
            return q_prev

    def _gate_warn(self, what: str) -> None:
        now = time.time()
        if now - self._gate_last_warn > 1.0:
            self._gate_last_warn = now
            self.get_logger().warn(
                f"Kollisions-Gate ({what}): Zielbewegung wuerde in "
                f"{'Koerper' if what == 'Koerper' else 'ein Objekt'} fahren -- "
                "Arm haelt an (Marker zurueckziehen).")

    def _advance_arm_weight(self, dt: float) -> float:
        """arm_sdk-Gewicht (motor_cmd[29].q) einen Tick weiterfahren.

        ramp_up: 0->1 in arm_weight_ramp_up_s (0.0 = sofort 1.0, Sim-Verhalten).
        ramp_down: 1->0 in arm_weight_ramp_down_s, danach Zustand off.
        """
        if self._weight_state == "ramp_up":
            ramp = float(self.get_parameter("arm_weight_ramp_up_s").value)
            if ramp <= 0.0:
                self._arm_sdk_weight = 1.0
            else:
                self._arm_sdk_weight = min(1.0, self._arm_sdk_weight + dt / ramp)
            if self._arm_sdk_weight >= 1.0:
                self._weight_state = "on"
        elif self._weight_state == "ramp_down":
            ramp = float(self.get_parameter("arm_weight_ramp_down_s").value)
            if ramp <= 0.0:
                self._arm_sdk_weight = 0.0
            else:
                self._arm_sdk_weight = max(0.0, self._arm_sdk_weight - dt / ramp)
            if self._arm_sdk_weight <= 0.0:
                self._weight_state = "off"
        elif self._weight_state == "on":
            self._arm_sdk_weight = 1.0
        else:
            self._arm_sdk_weight = 0.0
        return self._arm_sdk_weight

    def _publish_weight_ramp_down(self):
        """Nach dem Disable: letzte Arm-Sollwerte weiter publizieren, waehrend das
        arm_sdk-Gewicht 1->0 faehrt. self.msg enthaelt die zuletzt geschriebenen
        Arm-Kommandos noch -- nur Gewicht + CRC aktualisieren."""
        if not self.use_robot:
            self._weight_state = "off"
            self._arm_sdk_weight = 0.0
            return
        w = self._advance_arm_weight(self._compute_dt())
        try:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = float(w)
        except Exception:
            pass
        self.msg.crc = self.crc.Crc(self.msg)
        self.lowcmd_publisher.Write(self.msg)
        if self._weight_state == "off":
            self.get_logger().info("arm_sdk-Gewicht auf 0 -- Arme an die "
                                   "Roboter-Steuerung uebergeben.")

    def _publish_arm_slack(self):
        """Arme drehmomentfrei / leicht gedaempft halten -- OHNE die Kontrolle
        an den Unitree-Onboard-Regler zurueckzugeben.

        KRITISCH: das arm_sdk-Gewicht bleibt hier auf 1. Setzt man es auf 0
        (frueheres E-Stop-Verhalten!), uebernimmt der Onboard-Regler die Arme
        und faehrt sie AKTIV mit voller Geschwindigkeit in seine Default-Pose
        (sieht aus wie ein Sprung in die Home-Pose). Mit Gewicht=1 behaelt
        arm_sdk die Autoritaet: kp=0 -> keine Positionshaltung, tau=0 -> kein
        Feedforward, kd klein -> die Arme sacken GEDAEMPFT (an Seilen
        abgefangen). Muss WEITER gesendet werden, sonst greift der arm_sdk-
        Watchdog und der Onboard-Regler schnappt doch noch zu."""
        if not self.use_robot or not getattr(self, "_initialized", False):
            return
        try:
            cur = self.get_current_motor_q()
        except Exception:
            cur = None
        try:
            self.msg.mode_machine = self.get_mode_machine()
            for jid in G1_29_JointArmIndex:
                mc = self.msg.motor_cmd[jid]
                mc.mode = 1
                mc.kp = 0.0
                mc.kd = self.estop_arm_kd
                mc.tau = 0.0
                mc.dq = 0.0
                if cur is not None:
                    mc.q = float(cur[jid.value])   # bei kp=0 ohne Wirkung, sauber
            # Weight = 1: arm_sdk behaelt die Autoritaet (KEIN Onboard-Snap).
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
        except Exception as e:
            self.get_logger().error(f"Arm-Slack-Nachricht fehlgeschlagen: {e}")

    def _on_emergency_stop(self, msg: Bool):
        """E-STOP: Arme drehmomentfrei/gedaempft, OHNE Onboard-Uebergabe.

        Der Slack-Zustand wird ab jetzt in jedem Tick weiter gesendet
        (_publish_arm_slack) -- kein Handover-Fenster, in dem der Onboard-
        Regler die Arme in die Home-Pose reissen koennte. Der Latch
        (estop_active) blockiert ENABLE, bis via /g1pilot/start quittiert
        wird; die Arme bleiben schlaff, bis ENABLE die Kontrolle bewusst
        zurueckholt. Beine/Koerper werden separat von loco_client gedaempft
        (Damp)."""
        if not msg.data or self.estop_active:
            return
        self._record_onboard_pose("E-Stop")   # nur wenn der Onboard-Regler sie fuehrte
        self.estop_active = True
        self._releasing = False
        self._release_direct = False
        self._arm_slack = True
        self.arms_enabled = False
        self.homing_active = False
        self.homing_reached = False
        self.walk_mode = False
        self._abort_planned_motion()   # keine Ueberraschungsbewegung nach dem Quittieren
        self._cancel_marker_goals("E-Stop")
        self._weight_state = "off"
        self._arm_sdk_weight = 1.0     # Autoritaet behalten (kein Onboard-Snap)
        self._publish_arm_slack()
        self.get_logger().warn("EMERGENCY STOP: Arme drehmomentfrei/gedaempft "
                               "(arm_sdk behaelt Autoritaet, KEIN Sprung in "
                               "Home). Quittieren: START.")

    def _on_start(self, msg: Bool):
        """START (Streamdeck) quittiert den E-Stop-Latch. Die Arme bleiben
        schlaff (_arm_slack) und werden weiter gedaempft gehalten, bis ENABLE
        MANIPULATION die Kontrolle bewusst zurueckholt -- NICHT an den
        Onboard-Regler zurueckgeben (der wuerde sie posieren)."""
        if msg.data and self.estop_active:
            self.estop_active = False
            self.get_logger().info("E-Stop quittiert (START) -- Arme bleiben "
                                   "gedaempft schlaff bis ENABLE MANIPULATION.")

    def _arms_controlled_callback(self, msg: Bool):
        """
        ROS 2 callback to enable or disable arm control.

        Parameters
        ----------
        msg : std_msgs.msg.Bool
            True to enable arm control, False to disable.

        Behavior
        --------
        - On enable: stores current joint state as last target.
        - On disable: stops sending active motion commands.
        """

        if msg.data and self.estop_active:
            self.get_logger().warn("ENABLE ignoriert: E-Stop aktiv -- erst mit "
                                   "START quittieren.")
            return
        if not msg.data:
            if self._releasing:
                return                   # Rueckgabe laeuft bereits
            # Real: erst zurueck in die Onboard-Pose fahren, dann Rampe.
            if self._start_release():
                return
        elif self._releasing:
            # ENABLE waehrend der Rueckfahrt: abbrechen, Arme HALTEN (unten).
            self._releasing = False
            self._release_direct = False
            self.get_logger().info("Rueckgabe abgebrochen (ENABLE) -- Arme halten.")
        if msg.data:
            self._record_onboard_pose("ENABLE")
            self._last_cmd_speed = 0.0
        self.arms_enabled = msg.data
        self._cancel_marker_goals("Manipulation %s" % ("aktiviert" if msg.data else "deaktiviert"))
        # Reste einer geplanten Bewegung IMMER verwerfen (Enable UND Disable):
        # sonst feuerte ein waehrend disabled fertig gewordener Plan beim
        # naechsten Enable als Ueberraschung los -- gleiche Vorsicht wie beim
        # Homing unten. (Auch beim Disable, damit ein noch laufender Planungs-
        # Thread sein Ergebnis nicht spaeter aktiviert.)
        self._abort_planned_motion()
        if self.arms_enabled:
            # Slack-Latch aufheben: ENABLE holt die Kontrolle bewusst zurueck.
            self._arm_slack = False
            try:
                cur = self.get_current_motor_q()
                left = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right= [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                pass
            # Reste einer frueheren Session verwerfen: alte IK-Ziele wuerden den
            # Arm beim Re-Enable SOFORT auf ein altes Marker-Ziel losfahren
            # lassen, ein altes homing_reached wuerde ihn zur Home-Pose treiben.
            # Stattdessen: HALTEN an der aktuellen Stellung, bis der Nutzer
            # einen Marker zieht.
            self.homing_active = False
            self.homing_reached = False
            self._align_ik_to_config(self._last_q_target[0:7],
                                     self._last_q_target[7:14])
            # Gewicht sanft hochfahren (bzw. Abwaertsrampe nahtlos umkehren).
            if self._weight_state != "on":
                self._weight_state = "ramp_up"
            self.get_logger().info("Arm ENABLED.")
        else:
            ramp_down = float(self.get_parameter("arm_weight_ramp_down_s").value)
            if (ramp_down > 0.0
                    and self._weight_state in ("ramp_up", "on")
                    and self._arm_sdk_weight > 0.0):
                self._weight_state = "ramp_down"
            else:
                # ramp_down<=0 (Sim): sofort verstummen wie bisher -- KEINE
                # w=0-Nachricht senden (die MuJoCo-Bridge haelt die Arme mit
                # dem zuletzt empfangenen Kommando; eine w=0-Nachricht wuerde
                # sie dort schlaff schalten = Verhaltensaenderung).
                self._weight_state = "off"
                self._arm_sdk_weight = 0.0
            self.get_logger().info("Arm DISABLED")

    def _homming_callback(self, msg: Bool):
        """
        ROS 2 callback that triggers the homing sequence for both arms.

        Parameters
        ----------
        msg : std_msgs.msg.Bool
            If True, initiates homing to predefined joint targets.

        Notes
        -----
        - Clears IK goals before starting the homing motion.
        - Sets internal flags for homing management.
        """

        if msg.data:
            if self._releasing:
                self.get_logger().warn("HOMING ignoriert: Arme werden gerade an den "
                                       "Onboard-Regler zurueckgegeben.")
                return
            # Nur bei aktiver Manipulation annehmen. Frueher wurde ein Homing-
            # Klick im disabled-Zustand GESPEICHERT und feuerte dann beim
            # naechsten Enable als Ueberraschungsbewegung. (Sim: automatisch
            # aktivieren, siehe _sim_auto_enable_arms.)
            if not self.walk_mode:
                self._sim_auto_enable_arms("HOMING")
            if not self.arms_enabled or self.estop_active:
                self.get_logger().warn("HOMING ignoriert: Manipulation nicht "
                                       "aktiv (erst ENABLE MANIPULATION).")
                return
            if self.walk_mode:
                # VOR dem Abbruch pruefen: sonst wuerde ein ignoriertes Homing
                # den geplanten Weg in die Lauf-Pose abwuergen.
                self.get_logger().warn("HOMING ignoriert: WALK aktiv (Arme in Lauf-Pose).")
                return
            self._abort_planned_motion("HOMING gestartet")
            # Home-Pose GEPLANT anfahren (wie POSE ANFAHREN): um Tisch/Hindernisse
            # und den eigenen Koerper herum. Die fruehere direkte Gelenkfahrt lief
            # stur auf geradem Weg -- steht der Roboter am Tisch, fuhr sie die
            # Haende hinein (Gate haelt nur an) und stiess den Roboter um.
            self.homing_reached = False
            self._reset_after_home = False
            goals, label = self._homing_goals()
            self._start_planned_motion(label, ["left", "right"], goals, {}, source="homing")

    def _homing_goals(self):
        """Ziel fuers Homing (Grundstellung): immer die Lauf-Pose -- Arme neben
        dem Koerper, Haende auf Hueft-Hoehe. Liegt nah am Koerper und kollidiert
        daher auch dicht am Tisch nicht (die fruehere Nullpose mit Unterarmen
        nach vorn musste dort auf die Lauf-Pose ausweichen)."""
        return {"left": self.walk_left.copy(), "right": self.walk_right.copy()}, "HOME (Lauf-Pose)"

    def _align_ik_to_config(self, left_q, right_q):
        """IK-Ziele auf die gegebene Arm-Konfiguration setzen -> die Arme HALTEN dort,
        bis der Nutzer einen Marker zieht (kein Sprung auf ein altes Ziel)."""
        left_q = np.asarray(left_q, dtype=float); right_q = np.asarray(right_q, dtype=float)
        try:
            if hasattr(self.ik_solver, "clear_goals"):
                self.ik_solver.clear_goals()
            self.ik_solver.set_current_configuration({"left": left_q.copy(), "right": right_q.copy()})
            q_full = pin.neutral(self.ik_solver.model)
            for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = left_q[i]
            for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = right_q[i]
            pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
            pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)
            T_left  = self.ik_solver.data.oMf[self.ik_solver._fid_left]
            T_right = self.ik_solver.data.oMf[self.ik_solver._fid_right]
            self._goal_left_filt  = T_left.copy()
            self._goal_right_filt = T_right.copy()
            # Solange diese Ziele unveraendert sind, HAELT die Hauptschleife die
            # Gelenke exakt (keine IK) -- sonst zieht der Nullraum-Term die
            # Arme beim Uebernehmen (ENABLE) in die Ruhe-Konfiguration: Hand
            # bleibt, Ellbogen dreht bis ~0.25 rad weg.
            self._ik_hold_goal = {"left": T_left.copy(), "right": T_right.copy()}
            if hasattr(self.ik_solver, "set_goal"):
                self.ik_solver.set_goal("left",  T_left.copy())
                self.ik_solver.set_goal("right", T_right.copy())
            self._reset_after_home = True
        except Exception as e:
            self.get_logger().warning(f"IK-Align fehlgeschlagen: {e}")

    def _ik_side_held(self, side: str) -> bool:
        """True, wenn das IK-Ziel dieser Seite noch das per _align_ik_to_config
        gesetzte Halteziel ist (kein neues Ziel seitdem)."""
        hold = self._ik_hold_goal.get(side)
        goal = self._goal_left_filt if side == "left" else self._goal_right_filt
        if hold is None or goal is None:
            return False
        return bool(np.allclose(goal.homogeneous, hold.homogeneous, atol=1e-9))

    def _on_walk_mode(self, msg: Bool):
        """WALK: Arme sanft in die Lauf-Pose fahren und dort halten (Marker
        werden waehrend WALK ignoriert), damit die Lauf-Policy stabil bleibt."""
        if not msg.data or self.walk_mode:
            return
        if self._releasing:
            # Die Arme gehen ohnehin gerade an den Onboard-Regler, der sie beim
            # Laufen selbst fuehrt -- nicht in die Lauf-Pose umplanen.
            self.get_logger().info("WALK: Rueckgabe an den Onboard-Regler laeuft weiter.")
            return
        if not self.walk_park_arms:
            # Arme frei (Sim, AGILE-Policy): nichts parken, Marker/Kommandos wirken
            # weiter. Arme aktiv halten (sonst haelt die Bridge sie in der Null-
            # Pose) und sofort walk_ready melden (Demo-GUI wartet darauf).
            self._sim_auto_enable_arms("WALK")
            self._walk_ready_sent = True
            self.walk_ready_publisher.publish(Bool(data=True))
            self.get_logger().info("WALK-Modus: Arme bleiben frei (walk_park_arms=false).")
            return
        self.walk_mode = True
        self.homing_active = False
        self.homing_reached = False
        self._abort_planned_motion("WALK gestartet")
        self._cancel_marker_goals("WALK gestartet")
        self._walk_ready_sent = False    # erst melden, wenn die Lauf-Pose erreicht ist
        self.get_logger().info("WALK-Modus: Arme in Lauf-Pose aufraeumen, dann walk_ready.")
        # NUR SIM: Ohne ENABLE MANIPULATION ueberspringt main_loop den WALK-Zweig
        # (arms_enabled False) -> Arme kommen nie in die Lauf-Pose, kein walk_ready,
        # loco_sim laeuft nach Timeout trotzdem los und die Bridge haelt die Arme
        # in der Null-Pose -> die Policy kippt nach ~2 s (headless reproduziert).
        # Also fuer WALK die Arme selbst uebernehmen. Real bleibt unveraendert:
        # dort fuehrt Unitrees Onboard-Regler die Arme beim Laufen.
        self._sim_auto_enable_arms("WALK")
        # Den Weg in die Lauf-Pose PLANEN (um Tisch/Hindernisse herum). Die
        # fruehere direkte Gelenkfahrt lief auf gerader Linie -- steht der
        # Roboter am Tisch, fuhren die Finger in die Platte und das Gate hielt
        # die Arme nur an (walk_ready kam dann erst per loco_sim-Timeout).
        if self.arms_enabled and not self.estop_active:
            self._walk_transition = True
            self._start_planned_motion(
                "Lauf-Pose (WALK)", ["left", "right"],
                {"left": self.walk_left.copy(), "right": self.walk_right.copy()},
                {}, source="walk")

    def _sim_auto_enable_arms(self, what: str) -> None:
        """NUR SIM: Arme automatisch aktivieren, wenn eine Aktion sie braucht
        (WALK: Lauf-Pose, POSE ANFAHREN: gespeicherte Pose), statt sie still zu
        ignorieren. Real bleibt ENABLE MANIPULATION eine bewusste Operator-
        Entscheidung. Ein aktiver E-Stop wird nie uebergangen."""
        if is_sim_mode() and not self.arms_enabled and not self.estop_active:
            self.get_logger().info(f"{what} (Sim): Arme nicht aktiviert -> automatisch aktivieren.")
            self._arms_controlled_callback(Bool(data=True))

    def _on_balance_mode(self, msg: Bool):
        """BALANCING: Arme wieder freigeben (rviz/Marker). Sie HALTEN ihre aktuelle
        Stellung, bis der Nutzer einen Marker zieht."""
        if not msg.data:
            return
        if not self.walk_mode:
            # Arme waren beim Laufen frei (walk_park_arms=false): nur walk_ready
            # zuruecknehmen, damit das naechste START WALKING wieder neu meldet.
            if self._walk_ready_sent:
                self._walk_ready_sent = False
                self.walk_ready_publisher.publish(Bool(data=False))
            return
        self.walk_mode = False
        if self._walk_transition:
            self._abort_planned_motion("BALANCING gestartet")
        self.homing_active = False
        self.homing_reached = False
        if self._walk_ready_sent:
            self._walk_ready_sent = False
            self.walk_ready_publisher.publish(Bool(data=False))
        self._cancel_marker_goals("BALANCING gestartet")
        self._align_ik_to_config(self._last_q_target[0:7], self._last_q_target[7:14])
        self.get_logger().info("BALANCING-Modus: Arme frei (rviz/Marker).")

    def _transform_pose_to_world(self, ps: PoseStamped) -> PoseStamped:
        """
        Transform an incoming pose message into the world (IK) reference frame.

        Parameters
        ----------
        ps : geometry_msgs.msg.PoseStamped
            Input pose message with arbitrary frame_id.

        Returns
        -------
        geometry_msgs.msg.PoseStamped
            Pose transformed into `self.frame` if possible; original if TF lookup fails.
        """

        if not ps.header.frame_id or ps.header.frame_id == self.frame:
            return ps
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, ps.header.frame_id, Time(), timeout=Duration(seconds=0.2))
            return do_transform_pose_stamped(ps, tf)
        except Exception as e:
            self.get_logger().warning(f"[IK] TF {ps.header.frame_id}->{self.frame} failed: {e}")
            return ps

    def _on_scene_markers(self, msg: MarkerArray):
        """Umgebungs-Objekte (/scene_markers, i.d.R. Frame 'map') EINMAL pro
        Update in den IK-World-Frame (self.frame, Default 'pelvis') transformieren
        und ins Kugelmodell (Gate) bzw. an den Planer geben. Nur EIN TF-Lookup
        pro Update (nicht pro Marker) -- alle Marker teilen sich denselben
        Quell-Frame, do_transform_pose_stamped() je Marker ist reine Pose-Arithmetik,
        keine erneute TF-Baum-Abfrage."""
        add_markers = [m for m in msg.markers
                       if m.action == Marker.ADD and sm.is_collision_marker(m)]
        if not add_markers:
            self._set_environment([])
            return
        frame_id = add_markers[0].header.frame_id or "map"
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame, frame_id, Time(), timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(
                f"[scene] TF {frame_id}->{self.frame} fehlgeschlagen: {e} -- "
                f"Umgebungs-Objekte fuer dieses Update uebersprungen.")
            return

        objects = []
        for marker in add_markers:
            try:
                ps = PoseStamped()
                ps.header = marker.header
                ps.pose = marker.pose
                ps_t = do_transform_pose_stamped(ps, tf)
                half = sm.local_half_extents_from_marker(marker)
                name, _ = sm.decode_text(marker.text)
                o, p = ps_t.pose.orientation, ps_t.pose.position
                objects.append({
                    "name": name or f"marker_{marker.ns}_{marker.id}",
                    "cls": sm.class_from_ns(marker.ns),
                    "half_extents": half,
                    "pos": [p.x, p.y, p.z],
                    "quat": [o.w, o.x, o.y, o.z],
                })
            except Exception as e:
                self.get_logger().warn(f"[scene] Marker uebersprungen: {e}")
        self._set_environment(objects)

    def _set_environment(self, objects: list) -> None:
        """Umgebungs-Objekte (Posen im IK-World-Frame) uebernehmen: Gate
        (Kugelmodell) sofort, der Planer bekommt sie mit jeder Anfrage."""
        self._env_objects = list(objects)
        # Taille/Beine aus der Messung (Arm-Ketten haengen an der Taille).
        try:
            if self.use_robot:
                self.arm_geom.set_base(self.get_current_motor_q())
        except Exception:
            pass
        # Gate: nur, was ein Arm ueberhaupt erreichen kann (spart Rechenzeit
        # im 250-Hz-Tick bei grossen Szenen).
        self.arm_geom.set_environment(
            self.arm_geom.reachable(Environment(self._env_objects), ["left", "right"], margin=0.05))

    # --------------------------------------------------------
    # Positionsspeicher (plan-execute), siehe g1pilot/docs/11_arm_manipulation_technik.md (Positionsspeicher)
    # --------------------------------------------------------

    def _on_hand_state(self, side: str, msg: Float32MultiArray):
        """Aktuellen Fingerzustand (6 Inspire-DOF, 0..1000) von der Bridge
        merken -- ausschliesslich, um ihn beim Speichern der Handposition
        mitnehmen zu koennen."""
        vals = list(msg.data)
        if len(vals) == 6:
            self._hand_state[side] = [float(v) for v in vals]

    def _publish_hand_goals(self, hand_goals: dict) -> None:
        """Gespeicherte Handstellungen an die inspire-Bridge senden (setzt dort
        alle 6 DOF je Hand, siehe bridge.py /g1pilot/finger_goal)."""
        for side, vals in hand_goals.items():
            if vals is None:
                continue
            m = Float32MultiArray()
            m.data = [float(v) for v in vals]
            self.pub_finger_goal[side].publish(m)
            self.get_logger().info(f"Handposition ({side}) wiederhergestellt.")

    def _publish_save_status(self, state: str, *, req_id: str = "", **kw) -> None:
        """Ergebnis eines Speicher-Vorgangs veroeffentlichen
        (/g1pilot/pose_store/save/status). Ohne diesen Rueckkanal wuesste ein
        Aufrufer der HTTP-Schnittstelle nicht, ob wirklich gespeichert wurde --
        die GUI sieht es am Log, ein fremdes Programm nicht."""
        self.pub_save_status.publish(
            String(data=ac.save_status_message(req_id, state, **kw)))

    def _on_pose_save(self, msg: String):
        """Speichert die AKTUELLE Stellung unter einem Namen, mit Auswahl der
        Komponenten und einer Kategorie ("Ordner"). Quellen: der Streamdeck-
        Dialog und der HTTP-Endpunkt POST /arm/save (siehe g1pilot/docs/21_arm_api_technik.md) -- beide
        schicken dasselbe JSON `{id?, name, category?, components?}` auf dieses
        Topic; ein reiner Name (Altform) gilt weiter als "beide Arme".

        Gespeichert wird das zuletzt KOMMANDIERTE Gelenkziel (nicht die Messung,
        die rauscht) bzw. der letzte gemeldete Fingerzustand."""
        try:
            cmd = ac.parse_save_command(msg.data)
        except ac.CommandError as e:
            self._publish_save_status(ac.ST_REJECTED, reason=str(e))
            self.get_logger().warn(f"Pose speichern abgelehnt: {e}")
            return
        req_id, name = cmd["id"], cmd["name"]
        category, components = cmd["category"], cmd["components"]

        kwargs, skipped = {}, []
        if "left_arm" in components:
            kwargs["left_arm"] = self._last_q_target[0:7].copy()
        if "right_arm" in components:
            kwargs["right_arm"] = self._last_q_target[7:14].copy()
        # Haende: nur speicherbar, wenn die inspire-Bridge einen Ist-Zustand
        # gemeldet hat. Fehlt er, wird die Seite ausgelassen UND das im Status
        # als 'skipped' zurueckgegeben -- stillschweigend weniger zu speichern
        # als angefordert waere die schlechtere Antwort.
        for side in ("left", "right"):
            if f"{side}_hand" not in components:
                continue
            st = self._hand_state[side]
            if st is None:
                skipped.append(f"{side}_hand")
            else:
                kwargs[f"{side}_hand"] = list(st)
        if skipped:
            self.get_logger().warn(
                f"Kein Hand-Zustand fuer {', '.join(skipped)} (inspire-Bridge "
                f"nicht aktiv?) -- ausgelassen.")

        if not kwargs:
            why = ("Nichts Speicherbares: " + ", ".join(skipped) + " nicht verfuegbar."
                   if skipped else "Nichts Speicherbares ausgewaehlt.")
            self._publish_save_status(ac.ST_FAILED, req_id=req_id, name=name,
                                      reason=why, skipped=skipped)
            self.get_logger().warn(f"Pose '{name}' nicht gespeichert: {why}")
            return
        try:
            self._pose_store.save(name, category=category, **kwargs)
            saved_in = self._pose_store.category_of(name)
            self._publish_save_status(
                ac.ST_SAVED, req_id=req_id, name=name, category=saved_in,
                components=[c for c in ac.COMPONENTS if c in kwargs], skipped=skipped)
            self.get_logger().info(
                f"Pose '{name}' in Kategorie '{saved_in}' "
                f"gespeichert ({', '.join(sorted(kwargs))}).")
        except Exception as e:  # noqa: BLE001 -- Schreibfehler darf den Node nie killen
            self._publish_save_status(ac.ST_FAILED, req_id=req_id, name=name,
                                      reason=f"Schreiben fehlgeschlagen: {e}")
            self.get_logger().error(f"Pose '{name}' konnte nicht gespeichert werden: {e}")

    def _on_pose_cancel(self, msg: Bool):
        if not msg.data:
            return
        if self._releasing:
            self.get_logger().info("POSE ABBRECHEN ignoriert: Rueckgabe an den "
                                   "Onboard-Regler laeuft (Abbruch: ENABLE oder E-Stop).")
            return
        if self.walk_mode:
            # Ein Abbruch liesse die Arme auf GERADEM Weg in die Lauf-Pose fahren
            # (WALK braucht sie dort) -- schlechter als der geplante Weg.
            self.get_logger().info("POSE ABBRECHEN ignoriert: WALK aktiv "
                                   "(Arme fahren geplant in die Lauf-Pose).")
            return
        self._abort_planned_motion("POSE ABBRECHEN")

    def _arms_not_ready_reason(self):
        """-> Klartext-Grund, warum jetzt KEINE Bewegung starten darf, oder None.
        Eine Stelle fuer alle Eingaenge (Positionsspeicher UND eingespielte
        Kommandos), damit die Schranken nicht auseinanderlaufen."""
        if self.estop_active:
            return "E-Stop aktiv (mit START quittieren)."
        if self._releasing:
            return "Arme werden an den Onboard-Regler zurueckgegeben."
        if not self.arms_enabled:
            return "Arme nicht aktiviert (ENABLE MANIPULATION)."
        if self.homing_active:
            return "Homing laeuft."
        if self.walk_mode:
            return "WALK aktiv -- Arme haengen in der Lauf-Pose."
        if self._planning_thread is not None and self._planning_thread.is_alive():
            return "Es laeuft bereits eine Planung."
        return None

    def _start_planned_motion(self, label: str, sides: list, goals: dict,
                              hand_goals: dict, *, req_id: str = "",
                              source: str = "pose_store", ik_targets: dict = None):
        """Gemeinsamer Einstieg in die geplante Bewegung -- benutzt von
        POSE ANFAHREN und von der Live-Schnittstelle (arm_command). `goals` sind
        Gelenkziele je Seite; sind statt dessen `ik_targets` (SE3 je Seite)
        gesetzt, loest der Worker sie ZUERST per IK auf. Die Planung laeuft immer
        im Hintergrund-Thread, damit der 250-Hz-Tick nie blockiert."""
        # Faehrt noch eine aeltere Bewegung, wird sie SAUBER abgebrochen (mit
        # Status 'cancelled' unter ihrer alten ID -- deshalb VOR dem Umsetzen von
        # _cmd_req_id). Sonst uebernaehme die neue Bahn stillschweigend und ein
        # wartender Aufrufer (arm_api) liefe in seinen Timeout.
        if self._planned_motion_active:
            self._abort_planned_motion("neues Ziel angefordert")
        self._cancel_marker_goals(f"geplante Bewegung: {label}")
        self._cmd_req_id = req_id
        self._cmd_source = source
        self._cmd_ik_detail = None
        # 'accepted' erst HIER (nach dem Abbruch der alten Bewegung), damit die
        # Statusfolge fuer einen Mitleser chronologisch stimmt: erst wird die
        # alte Bewegung abgeraeumt, dann die neue angenommen.
        self._publish_cmd_status(ac.ST_ACCEPTED, sides=sides)
        # Hand-Ziele fahren beim START der Armbewegung mit los (gemeinsam).
        self._pending_hand_goals = hand_goals or {}
        self._plan_gen += 1                    # neue Planungs-Generation
        gen = self._plan_gen
        self.get_logger().info(f"Plane Weg zu {label} ({'+'.join(sides)}) ...")
        self._planning_thread = threading.Thread(
            target=self._plan_pose_worker, args=(label, gen, sides, goals),
            kwargs={"ik_targets": ik_targets}, daemon=True)
        self._planning_thread.start()

    def _publish_cmd_status(self, state: str, *, reason: str = "", sides=None,
                            detail=None, req_id=None, source=None) -> None:
        """Status einer eingespielten/geplanten Bewegung veroeffentlichen
        (/g1pilot/arm_command/status). Auch fuer Positionsspeicher-Bewegungen --
        ein externer Beobachter sieht damit, was der Arm gerade tut. Wird auch
        aus dem Planungs-Thread aufgerufen (rclpy-publish ist thread-sicher)."""
        self.pub_cmd_status.publish(String(data=ac.status_message(
            req_id if req_id is not None else self._cmd_req_id,
            state, reason=reason, sides=sides, detail=detail,
            source=source or self._cmd_source)))

    def _on_arm_command(self, msg: String):
        """Live eingespielte Zielpose DIREKT ausfuehren (nichts speichern) --
        siehe g1pilot/docs/21_arm_api_technik.md. Zwei Varianten: Gelenkwinkel ('joints') oder
        kartesische Hand-Pose ('pose', wird per IK aufgeloest).

        Bewusst dieselben Schranken wie POSE ANFAHREN: E-Stop, ENABLE
        MANIPULATION, kein Homing/WALK, Gelenklimits, Kollisionscheck in der
        Planung, Marker-Vorrang, abbrechbar. Ein fremdes Projekt darf sich hier
        NICHT an den Sicherheitsgates vorbeischreiben koennen."""
        try:
            cmd = ac.parse_command(msg.data)
        except ac.CommandError as e:
            # Ohne gueltiges JSON gibt es keine ID -- Status trotzdem senden,
            # damit die HTTP-Bruecke den Grund weitergeben kann.
            self._publish_cmd_status(ac.ST_REJECTED, reason=str(e), req_id="",
                                     source="arm_command")
            self.get_logger().warn(f"arm_command abgelehnt: {e}")
            return

        req_id, sides = cmd["id"], cmd["sides"]
        def reject(why):
            self._publish_cmd_status(ac.ST_REJECTED, reason=why, sides=sides,
                                     req_id=req_id, source="arm_command")
            self.get_logger().warn(f"arm_command '{req_id}' abgelehnt: {why}")

        if not self.walk_mode:
            self._sim_auto_enable_arms(f"arm_command '{req_id}'")
        not_ready = self._arms_not_ready_reason()
        if not_ready:
            reject(not_ready)
            return

        goals, ik_targets = {}, None
        if cmd["type"] == ac.TYPE_JOINTS:
            for side in sides:
                q7 = np.asarray(cmd["targets"][side], dtype=float)
                bad = self._joint_limit_violation(side, q7)
                if bad:
                    reject(bad)
                    return
                goals[side] = q7
        else:
            # Kartesisch: SE3 je Seite bauen (TF -> IK-World-Frame, optional
            # dieselbe EE-Offset-Konvention wie der RViz-Marker). Die IK selbst
            # laeuft im Planungs-Thread -- sie kostet zu viel fuer diesen
            # Callback, der im 250-Hz-Executor liegt.
            try:
                ik_targets = {s: self._command_pose_to_se3(s, cmd["targets"][s],
                                                           cmd["apply_ee_offset"])
                              for s in sides}
            except ValueError as e:
                reject(str(e))
                return

        label = f"eingespielter Pose '{req_id}' ({cmd['type']})"
        self._start_planned_motion(label, sides, goals, cmd["hands"],
                                   req_id=req_id, source="arm_command",
                                   ik_targets=ik_targets)

    def _joint_limit_violation(self, side: str, q7: np.ndarray):
        """-> Klartext, welches Gelenk ausserhalb seines Limits liegt, sonst None.
        Ein eingespieltes Ziel jenseits der Limits wuerde die Planung ohnehin
        scheitern lassen -- hier gibt es dafuer einen brauchbaren Grund."""
        idx_list = LEFT_JOINT_INDICES_LIST if side == "left" else RIGHT_JOINT_INDICES_LIST
        for i, jidx in enumerate(idx_list):
            lo, hi = JOINT_LIMITS_RAD[jidx]
            if not (lo <= float(q7[i]) <= hi):
                return (f"{side} Gelenk {i} ({JOINT_NAMES_ROS[jidx]}) = "
                        f"{float(q7[i]):.3f} rad liegt ausserhalb "
                        f"[{lo:.3f}, {hi:.3f}].")
        return None

    def _command_pose_to_se3(self, side: str, target: dict, apply_offset: bool) -> SE3:
        """Kartesisches Ziel eines eingespielten Kommandos -> SE3 im IK-World-
        Frame. `frame` leer = schon im IK-World-Frame (Default 'pelvis'), sonst
        wird per TF transformiert (schlaegt der Lookup fehl, wird abgelehnt --
        stillschweigend im falschen Frame zu fahren waere gefaehrlich)."""
        ps = PoseStamped()
        ps.header.frame_id = target["frame"] or self.frame
        p = target["position"]
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p
        qx, qy, qz, qw = target["quat_xyzw"]
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        if ps.header.frame_id != self.frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame, ps.header.frame_id, Time(), timeout=Duration(seconds=0.2))
                ps = do_transform_pose_stamped(ps, tf)
            except Exception as e:  # noqa: BLE001
                raise ValueError(
                    f"{side}: TF {ps.header.frame_id} -> {self.frame} nicht "
                    f"verfuegbar ({e}). Ziel bitte in '{self.frame}' angeben.")
        o, p = ps.pose.orientation, ps.pose.position
        T = SE3(pin.Quaternion(o.w, o.x, o.y, o.z).matrix(),
                np.array([p.x, p.y, p.z]))
        if apply_offset:
            # Gleiche Konvention wie der RViz-Marker: das Ziel beschreibt die
            # Marker-/Greifpunkt-Pose, der statische EE-Offset rechnet auf den
            # IK-Frame um. Die AUTO-Kalibrierung bleibt bewusst aussen vor --
            # die richtet den Marker auf die Ist-Hand aus und wuerde ein absolut
            # gemeintes Ziel verschieben.
            T = T * (self._T_off_right_static if side == "right"
                     else self._T_off_left_static)
        return T

    def _on_pose_goto(self, msg: String):
        name = (msg.data or "").strip()
        if not name:
            return
        if not self.walk_mode:
            self._sim_auto_enable_arms(f"POSE ANFAHREN '{name}'")
        not_ready = self._arms_not_ready_reason()
        if not_ready:
            self.get_logger().warn(f"POSE ANFAHREN '{name}' ignoriert: {not_ready}")
            return
        entry = self._pose_store.get(name)
        if entry is None:
            self.get_logger().warn(f"Pose '{name}' nicht gefunden.")
            # Endzustand melden -- sonst wartet ein Ablauf (Demo-GUI) ewig.
            self._publish_cmd_status(ac.ST_REJECTED, reason=f"Pose '{name}' nicht gefunden.",
                                     req_id="", source="pose_store")
            return

        # Welche Arme sind gespeichert? Reihenfolge fix (links, dann rechts) --
        # das ist die Spaltenreihenfolge im gemeinsamen Gelenkvektor.
        sides = [s for s in ("left", "right") if f"{s}_arm" in entry]
        goals = {s: np.asarray(entry[f"{s}_arm"], dtype=float) for s in sides}
        hand_goals = {s: entry.get(f"{s}_hand") for s in ("left", "right")
                      if f"{s}_hand" in entry}

        if not sides and not hand_goals:
            self.get_logger().warn(f"Pose '{name}' enthaelt nichts zum Anfahren.")
            return

        if not sides:
            # Nur Handposition -- keine Armplanung noetig, sofort senden. "reached"
            # erst, wenn die Finger Zeit zum Schliessen/Oeffnen hatten: Abläufe
            # (Demo-GUI) warten auf diesen Endzustand, bevor der naechste Schritt
            # (z.B. Anheben) losfaehrt.
            self._publish_hand_goals(hand_goals)
            self.get_logger().info(f"Pose '{name}': nur Handposition wiederhergestellt.")
            self._publish_cmd_status(ac.ST_EXECUTING, req_id="", source="pose_store")
            settle = float(self.get_parameter("hand_only_settle_s").value)
            timer = None

            def _done():
                timer.cancel()
                self._publish_cmd_status(ac.ST_REACHED, req_id="", source="pose_store")
            timer = self.create_timer(max(settle, 0.05), _done)
            return

        self._start_planned_motion(f"Pose '{name}'", sides, goals, hand_goals,
                                   req_id="", source="pose_store")

    def _planner_params(self) -> dict:
        gp = lambda n: float(self.get_parameter(n).value)   # noqa: E731
        return {
            "clearance": gp("planner_clearance"),
            "min_clearance": gp("planner_min_clearance"),
            "relax_radius": gp("planner_relax_radius"),
            "time_budget_s": gp("planner_time_budget_s"),
            "vel_limit": min(gp("planner_vel_limit"), 0.95 * self.arm_velocity_limit),
            "cart_vel_limit": (gp("planner_cart_vel_limit") if self.ee_velocity_limit <= 0.0
                               else min(gp("planner_cart_vel_limit"),
                                        0.95 * self.ee_velocity_limit)),
        }

    def _plan_pose_worker(self, name: str, gen: int, sides: list, goals: dict,
                          ik_targets: dict = None):
        """Laeuft in einem Hintergrund-Thread und wartet dort auf den Planer-
        PROZESS (arm_planner_process.py) -- der Regelkreis bleibt unberuehrt.
        Die gewaehlten Arme werden GEMEINSAM geplant (7 DOF je Seite -> bei
        beiden Armen ein 14-DOF-Problem: Arm-zu-Arm ist an jedem Zwischen-
        zustand geprueft, beide fahren synchron). Kartesische Ziele
        (`ik_targets`, SE3 je Seite) loest der Planer-Prozess zuerst per IK.
        Ergebnis (mit Generation `gen`) geht per DataBuffer an main_loop."""
        try:
            base_current_all = (self.get_current_motor_q() if self.use_robot
                                else self._assemble_full_from_last())
            base_current_all = base_current_all.copy()
            q_start, limits = [], []
            for side in sides:
                sl = slice(0, 7) if side == "left" else slice(7, 14)
                idx_list = LEFT_JOINT_INDICES_LIST if side == "left" else RIGHT_JOINT_INDICES_LIST
                q_start.append(self._last_q_target[sl].copy())
                limits += [JOINT_LIMITS_RAD[i] for i in idx_list]
            # Nicht geplante Gelenke: Arme auf dem kommandierten Stand (der
            # stehende Arm ist statisches Hindernis), Beine/Taille gemessen.
            for i, jidx in enumerate(LEFT_JOINT_INDICES_LIST):
                base_current_all[jidx] = self._last_q_target[i]
            for i, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
                base_current_all[jidx] = self._last_q_target[7 + i]
            request = {
                "sides": list(sides),
                "base_full": base_current_all,
                "q_start": np.concatenate(q_start),
                "q_goal": (np.concatenate([goals[s] for s in sides]) if not ik_targets else None),
                "ik_targets": ({s: np.array(ik_targets[s].homogeneous) for s in sides}
                               if ik_targets else None),
                "limits": limits,
                "env": list(self._env_objects) if self.environment_collision_gate else [],
                "self_collision": self.self_collision_gate,
                "params": self._planner_params(),
                "walk_pose": {"left": self.walk_left, "right": self.walk_right},
            }
            budget = request["params"]["time_budget_s"]
            res = self._planner.plan(request, timeout_s=budget + 10.0)

            if res.get("ik_detail"):
                self._cmd_ik_detail = res["ik_detail"]
            if res.get("reason") == "ik_failed":
                why = ("Ziel-Pose fuer " + "/".join(res.get("failed_sides", []))
                       + " nicht erreichbar (IK konvergiert nicht -- Restfehler siehe detail).")
                self._plan_result.SetData(("failed", name, gen, sides, why))
                self._publish_cmd_status(ac.ST_FAILED, reason=why, sides=sides,
                                         detail=res["ik_detail"])
                self.get_logger().warn(f"IK fuer {name} fehlgeschlagen: {why}")
                return
            if res.get("ik_detail"):
                self.get_logger().info(
                    f"IK fuer {name} geloest: "
                    + ", ".join(f"{s}: {d['pos_err_m'] * 1000:.1f} mm / "
                                f"{d['ori_err_deg']:.1f} deg" for s, d in res["ik_detail"].items()))
            if not res.get("ok"):
                reason = res.get("reason", "unbekannt")
                if gen != self._plan_gen:
                    return      # abgebrochen -- Status kam schon von _abort_planned_motion
                self._plan_result.SetData(("failed", name, gen, sides, reason))
                self._publish_cmd_status(ac.ST_FAILED, reason=f"Planung: {reason}", sides=sides)
                self.get_logger().warn(
                    f"Planung zu {name} ({'+'.join(sides)}) fehlgeschlagen: {reason} "
                    f"({res.get('plan_time_s', '?')} s)")
                if res.get("traceback"):
                    self.get_logger().error(res["traceback"])
                return
            traj = {"t": np.asarray(res["t"], dtype=float), "q": np.asarray(res["q"], dtype=float),
                    "q_start": request["q_start"], "q_goal": np.asarray(res["q_goal"], dtype=float)}
            self._plan_result.SetData(("ok", name, gen, sides, traj))
            self.get_logger().info(
                f"Planung zu {name} ({'+'.join(sides)}): {res['reason']} in "
                f"{res['plan_time_s']:.2f} s, Abstand Mitte {res['clearance'] * 100:.1f} cm, "
                f"Fahrzeit {traj['t'][-1]:.1f} s.")
        except Exception as e:
            self._plan_result.SetData(("failed", name, gen, sides, str(e)))
            self._publish_cmd_status(ac.ST_FAILED, reason=str(e), sides=sides)
            self.get_logger().error(f"Planung zu {name} abgestuerzt: {e}")

    def _poll_plan_result(self):
        """Pro Tick billig pruefen, ob eine Hintergrund-Planung fertig wurde
        (neues Objekt im DataBuffer -- Identitaetsvergleich reicht, SetData
        ersetzt das Objekt bei jedem Aufruf). Aktiviert nur Ergebnisse der
        AKTUELLEN Generation (result[2]); veraltete (durch Abbruch/neue Planung
        invalidierte) werden verworfen."""
        result = self._plan_result.GetData()
        if result is None or result is self._plan_result_seen:
            return
        self._plan_result_seen = result
        if result[2] != self._plan_gen:
            return   # veraltet: abgebrochen oder von einer neueren Planung ueberholt.
        if result[0] != "ok":
            if self._releasing:
                # Rueckgabe: ohne Plan direkt zurueck (Gates aktiv, Stillstand
                # oder Zeitlimit -> Uebergabe von dort).
                self._release_direct = True
                self.get_logger().warn("Rueckgabe: kein geplanter Weg -- fahre direkt "
                                       "(Kollisions-Gate haelt bei Beruehrung an).")
                return
            # Fehlschlag wurde schon vom Worker geloggt -- Arm haelt einfach.
            # Ausnahme WALK: die Lauf-Pose muss trotzdem angefahren werden
            # (sonst kein walk_ready) -> wie frueher direkt, das Laufzeit-Gate
            # haelt bei Kollision weiterhin an.
            if self._walk_transition:
                self._walk_transition = False
                self.get_logger().warn(
                    "WALK: kein kollisionsfreier Weg in die Lauf-Pose gefunden -- "
                    "fahre direkt (Kollisions-Gate haelt bei Beruehrung an).")
            return
        # Defense-in-depth: sind die Arme seit dem Planungsstart gesperrt worden
        # (E-Stop, DISABLE, Homing, WALK)? Dann verwerfen. Normalerweise hat ein
        # solcher Wechsel _plan_gen bereits erhoeht (Abbruch); diese Pruefung
        # faengt Restfaelle ab.
        if (self.estop_active or not self.arms_enabled
                or self.homing_active
                or (self.walk_mode and not self._walk_transition)):
            self._publish_cmd_status(
                ac.ST_CANCELLED, reason="Arme waehrend der Planung gesperrt "
                                        "(E-Stop/DISABLE/Homing/WALK).",
                sides=result[3])
            self.get_logger().info(
                f"Geplante Bewegung zu {result[1]} verworfen "
                f"(Arme nicht mehr bereit -- erneut anfordern).")
            return
        _, name, _gen, sides, traj = result
        self._planned_pose_name = name
        self._planned_sides = list(sides)
        self._planned_traj_t = traj["t"]
        self._planned_traj_q = traj["q"]
        self._planned_time = 0.0
        self._planned_stall_s = 0.0
        self._planned_last_tick = None
        self._planned_stats = {"t0": time.monotonic(), "ticks": 0, "held_s": 0.0}
        # Laufzeit-Gate: an Start und Ziel der Bahn darf der Arm so nah an
        # Objekte wie dort geplant (Hand auf dem Tisch) -- Endpunkt-Boeden.
        ends = []
        for qend in (traj["q"][0], traj["q"][-1]):
            q14 = self._last_q_target.copy()
            for k, side in enumerate(sides):
                q14[slice(0, 7) if side == "left" else slice(7, 14)] = qend[7 * k:7 * k + 7]
            ends.append(q14)
        self._gate_end_q14 = ends
        self._planned_motion_active = True
        self.homing_active = False
        self.homing_reached = False
        # Finger fahren gemeinsam mit den Armen los.
        if self._pending_hand_goals:
            self._publish_hand_goals(self._pending_hand_goals)
            self._pending_hand_goals = {}
        self._publish_cmd_status(ac.ST_EXECUTING, sides=sides,
                                 detail=self._cmd_ik_detail or None)
        self.get_logger().info(f"Fahre geplante Bewegung zu {name} ab.")

    def destroy_node(self):
        try:
            self._planner.shutdown()
        except Exception:
            pass
        return super().destroy_node()

    def _traj_sample(self, t: float) -> np.ndarray:
        """Geplante Trajektorie zur Zeit t (lineare Interpolation zwischen den
        0.02-s-Stuetzstellen)."""
        tt, qq = self._planned_traj_t, self._planned_traj_q
        if t <= tt[0]:
            return qq[0].copy()
        if t >= tt[-1]:
            return qq[-1].copy()
        i = int(np.searchsorted(tt, t)) - 1
        u = (t - tt[i]) / max(1e-9, tt[i + 1] - tt[i])
        return (1.0 - u) * qq[i] + u * qq[i + 1]

    def _abort_planned_motion(self, reason: str = "") -> None:
        """Laufende geplante Bewegung abbrechen UND jede noch laufende Planung
        entwerten: durch Erhoehen von _plan_gen wird ein spaeter eintreffendes
        Ergebnis in _poll_plan_result verworfen (race-frei -- das Ergebnis muss
        beim Abbruch noch nicht vorliegen). Wird bei E-Stop, DISABLE, ENABLE,
        HOMING, WALK, Marker-Griff und POSE ABBRECHEN aufgerufen."""
        was_active = self._planned_motion_active
        was_planning = (self._planning_thread is not None
                        and self._planning_thread.is_alive())
        self._planned_motion_active = False
        self._walk_transition = False   # WALK faellt auf die direkte Lauf-Pose zurueck
        self._planned_sides = []
        self._planned_traj_t = None
        self._planned_traj_q = None
        self._gate_end_q14 = None
        self._pending_hand_goals = {}
        self._plan_gen += 1     # invalidiert jedes in-flight/pending Planungs-Ergebnis
        if was_planning:
            self._planner.cancel()   # Planer-Prozess bricht ab (reason 'cancelled')
        if was_active or was_planning:
            # Wartet ein Aufrufer auf das Ergebnis (arm_api), muss er den Abbruch
            # erfahren -- sonst laeuft er in seinen Timeout.
            self._publish_cmd_status(ac.ST_CANCELLED, reason=reason or "abgebrochen")
        if was_active and reason:
            self.get_logger().info(f"Geplante Bewegung abgebrochen ({reason}).")

    def _right_goal_callback(self, msg: PoseStamped):
        """
        ROS 2 callback for the right-hand end-effector goal.

        Parameters
        ----------
        msg : geometry_msgs.msg.PoseStamped
            Desired right-hand pose (can be in any TF frame).

        Behavior
        --------
        - Applies TF transformation to the world frame.
        - Handles homing reset alignment if needed.
        - Applies static and auto-calibration offsets.
        - Updates the IK solver's right-hand goal.
        """

        if self.homing_active:
            self._publish_marker_goal_status('right', msg.header.stamp, "ignored",
                                             reason="Homing laeuft")
            return

        if not self.arms_enabled:
            self._publish_marker_goal_status('right', msg.header.stamp, "ignored",
                                             reason="Manipulation nicht aktiv")
            return

        if self._releasing:
            self._publish_marker_goal_status('right', msg.header.stamp, "ignored",
                                             reason="Rueckgabe an den Onboard-Regler")
            return

        # Mode-Mux (siehe g1pilot/docs/11_arm_manipulation_technik.md (Mode-Mux Marker vs. Positionsspeicher)): manueller
        # Marker hat IMMER Vorrang -- ein laufender ODER gerade geplanter Plan
        # (Positionsspeicher) wird sofort verworfen, kein Kaempfen zweier
        # Geschwindigkeitsquellen.
        # Waehrend WALK ignoriert main_loop die Marker ohnehin -- den Uebergang
        # in die Lauf-Pose also nicht abbrechen.
        if not self.walk_mode and (self._planned_motion_active or (
                self._planning_thread is not None and self._planning_thread.is_alive())):
            self._abort_planned_motion("Marker bewegt -- manueller Vorrang")

        if self._reset_after_home:
            self._reset_after_home = False
            self.homing_reached = False
            try:
                cur = self.get_current_motor_q()
                left  = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right = [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                self._last_q_target = np.concatenate((self.home_left, self.home_right)).copy()

            self._goal_left_filt  = None
            self._goal_right_filt = None

            self.ik_solver.set_current_configuration({
                "left":  self._last_q_target[0:7].copy(),
                "right": self._last_q_target[7:14].copy()
            })

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal_in = SE3(q.matrix(), np.array([p.x, p.y, p.z]))

        self._last_right_goal_raw = T_goal_in
        T_target = self._goal_with_offsets('right', T_goal_in)
        T_goal_use = self._filter_goal('right', T_target)
        if T_goal_use is not None:
            self.ik_solver.set_goal("right", T_goal_use)
        self._track_marker_goal('right', msg.header.stamp, T_target)


    def _left_goal_callback(self, msg: PoseStamped):
        """
        ROS 2 callback for the left-hand end-effector goal.

        Parameters
        ----------
        msg : geometry_msgs.msg.PoseStamped
            Desired left-hand pose (can be in any TF frame).

        Behavior
        --------
        - Applies TF transformation to the world frame.
        - Handles homing reset alignment if needed.
        - Applies static and auto-calibration offsets.
        - Updates the IK solver's left-hand goal.
        """

        if self.homing_active:
            self._publish_marker_goal_status('left', msg.header.stamp, "ignored",
                                             reason="Homing laeuft")
            return

        if not self.arms_enabled:
            self._publish_marker_goal_status('left', msg.header.stamp, "ignored",
                                             reason="Manipulation nicht aktiv")
            return

        if self._releasing:
            self._publish_marker_goal_status('left', msg.header.stamp, "ignored",
                                             reason="Rueckgabe an den Onboard-Regler")
            return

        # Mode-Mux (siehe g1pilot/docs/11_arm_manipulation_technik.md (Mode-Mux Marker vs. Positionsspeicher)): manueller
        # Marker hat IMMER Vorrang -- ein laufender ODER gerade geplanter Plan
        # (Positionsspeicher) wird sofort verworfen, kein Kaempfen zweier
        # Geschwindigkeitsquellen.
        # Waehrend WALK ignoriert main_loop die Marker ohnehin -- den Uebergang
        # in die Lauf-Pose also nicht abbrechen.
        if not self.walk_mode and (self._planned_motion_active or (
                self._planning_thread is not None and self._planning_thread.is_alive())):
            self._abort_planned_motion("Marker bewegt -- manueller Vorrang")

        if self._reset_after_home:
            self._reset_after_home = False
            self.homing_reached = False
            try:
                cur = self.get_current_motor_q()
                left  = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right = [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                self._last_q_target = np.concatenate((self.home_left, self.home_right)).copy()

            self._goal_left_filt  = None
            self._goal_right_filt = None

            self.ik_solver.set_current_configuration({
                "left":  self._last_q_target[0:7].copy(),
                "right": self._last_q_target[7:14].copy()
            })

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal_in = SE3(q.matrix(), np.array([p.x, p.y, p.z]))

        self._last_left_goal_raw = T_goal_in
        T_target = self._goal_with_offsets('left', T_goal_in)
        T_goal_use = self._filter_goal('left', T_target)
        if T_goal_use is not None:
            self.ik_solver.set_goal("left", T_goal_use)
        self._track_marker_goal('left', msg.header.stamp, T_target)


    # --------------------------------------------------------
    # Marker-Ziele: erreicht / unerreichbar zurueckmelden
    # --------------------------------------------------------

    @staticmethod
    def _stamp_tuple(stamp):
        return (int(stamp.sec), int(stamp.nanosec))

    def _publish_marker_goal_status(self, side: str, stamp, state: str, *,
                                    pos_err=None, ori_err_deg=None, reason: str = ""):
        """Status eines Marker-Ziels auf /g1pilot/hand_goal_status (JSON).
        `goal_stamp` ist der header.stamp der Ziel-Nachricht -- daran erkennt
        der Marker, ob die Meldung zu SEINEM letzten Ziel gehoert."""
        sec, nsec = stamp if isinstance(stamp, tuple) else self._stamp_tuple(stamp)
        payload = {"side": side, "state": state,
                   "goal_stamp": {"sec": sec, "nanosec": nsec}}
        if pos_err is not None:
            payload["pos_err_m"] = round(float(pos_err), 4)
        if ori_err_deg is not None:
            payload["ori_err_deg"] = round(float(ori_err_deg), 2)
        if reason:
            payload["reason"] = reason
        self.pub_marker_goal_status.publish(String(data=json.dumps(payload)))

    def _track_marker_goal(self, side: str, stamp, T_target: SE3):
        """Neues Marker-Ziel fuer `side` merken. Ein neues Ziel ersetzt das alte
        einfach (Marker erneut gezogen) -- der Stillstand-Zaehler beginnt neu."""
        q7 = self._last_q_target[0:7] if side == "left" else self._last_q_target[7:14]
        stamp_t = self._stamp_tuple(stamp)
        prev = self._marker_goal.get(side)
        if prev is not None and prev["stamp"] != stamp_t:
            # Ein ANDERER Eingang (PS4, Demo-GUI, zweiter Marker-Zug) hat ein
            # neues Ziel gesetzt: dem alten Absender sagen, dass sein Ziel
            # erledigt ist -- sonst bleibt z.B. der RViz-Marker auf dem alten
            # Ziel stehen, und ein spaeterer Klick darauf zoege den Arm dorthin
            # zurueck.
            self._publish_marker_goal_status(side, prev["stamp"], "cancelled",
                                             reason="durch neues Ziel ersetzt")
        self._marker_goal[side] = {
            "stamp": stamp_t,
            "T": T_target,
            "q_ref": np.array(q7, dtype=float),
            "t_ref": time.monotonic(),
        }

    def _cancel_marker_goals(self, reason: str):
        """Laufende Marker-Ziele beider Seiten abbrechen (Homing, WALK, E-Stop,
        geplante Bewegung, Enable/Disable) und das dem Marker melden."""
        for side in ("left", "right"):
            g = self._marker_goal.get(side)
            if g is None:
                continue
            self._marker_goal[side] = None
            self._publish_marker_goal_status(side, g["stamp"], "cancelled", reason=reason)

    def _update_marker_goals(self):
        """Mit marker_goal_check_hz: offene Marker-Ziele nachfuehren und pruefen.

        * Ziel-Filter erneut mit dem Endziel fuettern (auto_reissue_goals),
          damit der gefilterte Sollwert wirklich am Marker ankommt.
        * erreicht: FK der KOMMANDIERTEN Armstellung innerhalb goal_pos_tol /
          goal_ori_tol_deg.
        * unerreichbar: kommandierte Armstellung bewegt sich seit
          marker_goal_stall_time_s um weniger als marker_goal_stall_tol_rad."""
        if self._marker_goal["left"] is None and self._marker_goal["right"] is None:
            return
        now = time.monotonic()
        if now - self._marker_check_last < self._marker_check_dt:
            return
        self._marker_check_last = now

        full = self._full_config_with_arms(self._last_q_target)
        q_full = pin.neutral(self.ik_solver.model)
        for jid_idx, ros_name in enumerate(self.ik_solver._ros_joint_names):
            if ros_name in self.ik_solver._name_to_q_index:
                q_full[self.ik_solver._name_to_q_index[ros_name]] = float(full[jid_idx])
        data = self._marker_fk_data
        pin.forwardKinematics(self.ik_solver.model, data, q_full)
        pin.updateFramePlacements(self.ik_solver.model, data)

        for side in ("left", "right"):
            g = self._marker_goal[side]
            if g is None:
                continue
            if self.auto_reissue_goals:
                self._filter_goal(side, g["T"])
            fid = self.ik_solver._fid_left if side == "left" else self.ik_solver._fid_right
            if fid is None:
                continue
            M = data.oMf[fid]
            pos_err = float(np.linalg.norm(g["T"].translation - M.translation))
            ori_err = math.degrees(float(np.linalg.norm(pin.log3(M.rotation.T @ g["T"].rotation))))
            if pos_err <= self.goal_pos_tol and ori_err <= self.goal_ori_tol_deg:
                self._marker_goal[side] = None
                # Rest-Versatz des Filters schliessen: Sollwert exakt aufs Ziel.
                if side == "left":
                    self._goal_left_filt = g["T"]
                else:
                    self._goal_right_filt = g["T"]
                self._publish_marker_goal_status(side, g["stamp"], "reached",
                                                 pos_err=pos_err, ori_err_deg=ori_err)
                continue
            q7 = self._last_q_target[0:7] if side == "left" else self._last_q_target[7:14]
            if np.linalg.norm(q7 - g["q_ref"]) > self._marker_stall_tol:
                g["q_ref"] = np.array(q7, dtype=float)
                g["t_ref"] = now
            elif now - g["t_ref"] >= self._marker_stall_time:
                self._marker_goal[side] = None
                self._publish_marker_goal_status(side, g["stamp"], "unreachable",
                                                 pos_err=pos_err, ori_err_deg=ori_err)
                self.get_logger().warn(
                    f"[marker] {side}: Ziel nicht erreichbar "
                    f"(Rest {pos_err * 100:.1f} cm / {ori_err:.1f} deg).")

    def _alpha_dt(self, alpha: float, dt: float) -> float:
        """Glaettungs-Alpha (gilt pro Takt bei smoothing_ref_hz) auf den echten
        Zeitschritt umrechnen: 1 - (1 - a)^(dt * ref). smoothing_ref_hz <= 0
        oder alpha >= 1 -> unveraendert (pro Takt, bisheriges Verhalten)."""
        if self.smoothing_ref_hz <= 0.0 or alpha >= 1.0 or alpha <= 0.0:
            return alpha
        return 1.0 - (1.0 - alpha) ** (dt * self.smoothing_ref_hz)

    def _compute_dt(self) -> float:
        """
        Compute the elapsed time (Δt) between consecutive main loop cycles.

        Returns
        -------
        float
            Time difference in seconds (clamped to [1e-4, 0.1]).
        """

        now = time.time()
        if self._last_tick_time is None:
            dt = 1.0 / self.rate_hz
        else:
            dt = max(1e-4, min(0.1, now - self._last_tick_time))
        self._last_tick_time = now
        return dt
    
    def _publish_workspace(self, arm):
        marker = Marker()
        marker.header.frame_id = WORKSPACE["frame"]
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.005  
        marker.color = ColorRGBA(r=0.1, g=1.0, b=0.3, a=0.9)

        points = WORKSPACE[arm]

        pts = {k: Point(x=v[0], y=v[1], z=v[2]) for k, v in points.items()}

        edges = [
            # Bottom rectangle
            ("left_bottom_front", "right_bottom_front"),
            ("right_bottom_front", "right_bottom_back"),
            ("right_bottom_back", "left_bottom_back"),
            ("left_bottom_back", "left_bottom_front"),

            # Top rectangle
            ("left_top_front", "right_top_front"),
            ("right_top_front", "right_top_back"),
            ("right_top_back", "left_top_back"),
            ("left_top_back", "left_top_front"),

            # Vertical edges
            ("left_bottom_front", "left_top_front"),
            ("right_bottom_front", "right_top_front"),
            ("left_bottom_back", "left_top_back"),
            ("right_bottom_back", "right_top_back"),
        ]

        for a, b in edges:
            marker.points.append(pts[a])
            marker.points.append(pts[b])

        if arm == "left_arm":
            self.left_workspace_publisher.publish(marker)
        else:
            self.right_workspace_publisher.publish(marker)


    def main_loop(self):
        """
        Main control loop executed at `rate_hz` frequency.

        Core Responsibilities
        ---------------------
        - Update LowState data from DDS.
        - Hold non-arm joints when arms are disabled.
        - Execute homing sequence if active.
        - Update IK solver configuration and compute joint targets.
        - Apply velocity and smoothing limits.
        - Publish joint targets via DDS or /joint_states (simulation).

        Notes
        -----
        - The loop manages both autonomous IK motion and homing control.
        - It automatically synchronizes the IK goals when returning to home.
        """

        self._publish_workspace("left_arm")
        self._publish_workspace("right_arm")

        if not getattr(self, "_initialized", False):
            return

        # Hintergrund-Planung (Positionsspeicher) fertig? Billig (Lock+Read).
        self._poll_plan_result()

        if self._releasing and (time.monotonic() - self._release_t0
                                > float(self.get_parameter("release_timeout_s").value)):
            self._finish_release("Zeitlimit der Rueckfahrt")

        # E-STOP / Slack: Arme drehmomentfrei/gedaempft halten und WEITER
        # senden -- ohne die Kontrolle an den Onboard-Regler zurueckzugeben
        # (der wuerde die Arme in seine Default-Pose reissen). Laeuft, bis
        # ENABLE MANIPULATION den Slack-Latch aufhebt.
        if self._arm_slack and not self.arms_enabled:
            self._publish_arm_slack()
            return

        # LowState kommt aus _subscribe_motor_state (eigener Thread ->
        # lowstate_buffer/motor_state). Frueher stand hier zusaetzlich ein
        # BLOCKIERENDES lowstate_subscriber.Read(): der Tick wartete auf die
        # naechste LowState-Nachricht und konkurrierte dabei mit dem Thread um
        # jede Probe. Real (500 Hz) kostete das ~2 ms je Tick, in der Sim
        # (Lockstep, ~30-50 Hz) halbierte es die Regelrate -> Arm zaeh, 1-2 s
        # Verzug am PS4-Controller/Marker.

        if not self.arms_enabled:
            if self._weight_state == "ramp_down":
                # Weiche Uebergabe: Gewicht 1->0 fahren, solange weiter senden.
                self._publish_weight_ramp_down()
            # Sonst: STILL sein. (Frueher wurden hier Beine+Taille mit kp=300
            # in self.msg geschrieben -- die gehoeren nicht in rt/arm_sdk.)
            return

        if self.walk_mode and not self._walk_transition:
            # WALK: Arme auf die Lauf-Pose halten (Marker ignoriert). Die
            # Geschwindigkeitsbegrenzung unten faehrt sie sanft dorthin (kein Teleport).
            q_target = np.concatenate((self.walk_left, self.walk_right))
            # Sobald die Arme die Lauf-Pose erreicht haben: einmalig walk_ready melden,
            # damit loco_sim erst dann mit dem Laufen beginnt (Arme aufgeraeumt).
            # Erst melden, wenn die Arme WIRKLICH unten sind (gemessen), nicht nur
            # das Kommando -- sonst laeuft loco_sim los, waehrend die Arme noch
            # aus der Sicheren Pose herunterschwingen (Laufen dann "komisch").
            arms_there = np.linalg.norm(q_target - self._last_q_target) < self.walk_ready_tolerance
            if arms_there and self.use_robot:
                try:
                    cur = self.get_current_motor_q()
                    meas = np.array([cur[j] for j in LEFT_JOINT_INDICES_LIST + RIGHT_JOINT_INDICES_LIST])
                    arms_there = np.linalg.norm(q_target - meas) < float(
                        self.get_parameter("walk_ready_measured_tolerance").value)
                except Exception:
                    pass
            if not self._walk_ready_sent and arms_there:
                self._walk_ready_sent = True
                self.walk_ready_publisher.publish(Bool(data=True))
                self.get_logger().info("Lauf-Pose erreicht -> walk_ready (Arme aufgeraeumt).")

        elif self.homing_active:
            q_target = np.concatenate((self.home_left, self.home_right))
            if np.linalg.norm(q_target - self._last_q_target) < self.homing_tolerance:

                self.homing_active = False
                self.homing_reached = True
                self._last_q_target = q_target.copy()

                if hasattr(self.ik_solver, "clear_goals"):
                    self.ik_solver.clear_goals()

                self.ik_solver.set_current_configuration({
                    "left":  self.home_left.copy(),
                    "right": self.home_right.copy()
                })

                try:
                    q_full = pin.neutral(self.ik_solver.model)
                    for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                        q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = self.home_left[i]
                    for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                        q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = self.home_right[i]

                    pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
                    pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)

                    T_left  = self.ik_solver.data.oMf[self.ik_solver._fid_left]
                    T_right = self.ik_solver.data.oMf[self.ik_solver._fid_right]

                    self._goal_left_filt  = T_left.copy()
                    self._goal_right_filt = T_right.copy()
                    if hasattr(self.ik_solver, "set_goal"):
                        self.ik_solver.set_goal("left",  T_left.copy())
                        self.ik_solver.set_goal("right", T_right.copy())

                    self._reset_after_home = True

                    self.get_logger().info("IK solver goals aligned with home pose.")
                except Exception as e:
                    self.get_logger().warning(f"Failed to align IK goals with home: {e}")

                self.get_logger().info("Home position reached.")

        elif self.homing_reached:
            q_target = np.concatenate((self.home_left, self.home_right))

        elif self._planned_motion_active:
            # Geplante Bahn abfahren: der Planer liefert eine ZEITPARAMETRIERTE
            # Trajektorie (t, q) fuer die gewaehlten Arme (7 DOF je Seite, in
            # _planned_sides-Reihenfolge). Beide Arme laufen ueber dieselbe Uhr
            # -> synchron. Fortschritts-Waechter: die Uhr laeuft nur weiter,
            # solange das Kommando der Bahn folgt (haelt das Laufzeit-Gate an,
            # bleibt auch die Bahn stehen statt davonzulaufen). Marker-Griff
            # bricht ueber _abort_planned_motion ab.
            q_target = self._last_q_target.copy()
            sides = self._planned_sides
            tt, qq = self._planned_traj_t, self._planned_traj_q
            if not sides or tt is None:
                self._planned_motion_active = False
            else:
                slices = [(slice(7 * k, 7 * k + 7),
                           slice(0, 7) if side == "left" else slice(7, 14))
                          for k, side in enumerate(sides)]
                q_now = np.concatenate([self._last_q_target[tgt] for _, tgt in slices])
                q_ref = self._traj_sample(self._planned_time)
                lag = float(np.max(np.abs(q_ref - q_now)))
                # Echte Zeit seit dem letzten Tick (der Regelkreis haelt seine
                # Rate unter Last nicht immer), begrenzt gegen Spruenge.
                now = time.monotonic()
                tick = (1.0 / self.rate_hz if self._planned_last_tick is None
                        else min(0.05, max(0.0, now - self._planned_last_tick)))
                self._planned_last_tick = now
                st = self._planned_stats
                st["ticks"] += 1
                if lag < 0.05:
                    self._planned_time = min(tt[-1], self._planned_time + tick)
                    self._planned_stall_s = 0.0
                else:
                    self._planned_stall_s += tick
                    st["held_s"] += tick
                q_ref = self._traj_sample(self._planned_time)
                for col, tgt in slices:
                    q_target[tgt] = q_ref[col]
                done = (self._planned_time >= tt[-1]
                        and float(np.max(np.abs(qq[-1] - q_now))) < self.planned_motion_tolerance)
                if done:
                    self._planned_motion_active = False
                    self._walk_transition = False   # WALK-Zweig haelt ab jetzt
                    self._gate_end_q14 = None
                    self._align_ik_to_config(q_target[0:7], q_target[7:14])
                    self._publish_cmd_status(ac.ST_REACHED, sides=sides,
                                             detail=self._cmd_ik_detail or None)
                    if self._releasing:
                        self._finish_release("Onboard-Pose erreicht")
                    wall = time.monotonic() - st["t0"]
                    self.get_logger().info(
                        f"Geplante Bewegung zu {self._planned_pose_name} abgeschlossen "
                        f"(Bahn {tt[-1]:.1f} s, gefahren {wall:.1f} s, davon angehalten "
                        f"{st['held_s']:.1f} s, Regeltakt {st['ticks'] / max(wall, 1e-3):.0f} Hz).")
                elif self._planned_stall_s > 3.0:
                    # Gate haelt dauerhaft an (Szene hat sich geaendert?) ->
                    # stehen bleiben und melden statt ewig zu warten.
                    name = self._planned_pose_name
                    self._planned_motion_active = False   # -> kein 'cancelled' zusaetzlich
                    self._abort_planned_motion("")
                    self._publish_cmd_status(
                        ac.ST_FAILED, reason="Bewegung blockiert (Kollisions-Gate haelt an) "
                                             "-- Szene geaendert? Neu anfordern.", sides=sides)
                    self.get_logger().warn(
                        f"Geplante Bewegung zu {name} blockiert -- abgebrochen.")
                    q_target = self._last_q_target.copy()
                    if self._releasing:
                        self._finish_release("Rueckfahrt blockiert (Gate)")

        elif self._releasing:
            # Rueckgabe: Planung laeuft -> stillhalten; ohne Plan direkt zur
            # Onboard-Pose (Speed-/Kollisions-Limits unten wirken wie immer).
            q_target = self._last_q_target.copy()
            if self._release_direct and self._onboard_q is not None:
                q_target = self._onboard_q.copy()
                err = float(np.max(np.abs(q_target - self._last_q_target)))
                now = time.monotonic()
                if err <= float(self.get_parameter("release_tolerance").value):
                    self._finish_release("Onboard-Pose erreicht (direkt)")
                elif err < self._release_best[0] - 1e-3:
                    self._release_best = (err, now)
                elif now - self._release_best[1] > 1.5:
                    self._finish_release("Rueckfahrt blockiert (Gate)")

        elif self._walk_transition:
            # WALK: Weg in die Lauf-Pose wird noch geplant -> stillhalten.
            q_target = self._last_q_target.copy()

        else:
            # IK-Seed: Arm-Gelenke aus dem LETZTEN ZIEL (deterministisch), nicht
            # aus der Messung. Wuerde man aus der Messung seeden, fuehrt Mess-/
            # Sim-Rauschen zu leicht anderen IK-Loesungen -> minimal anderes
            # Kommando -> Arm zittert dauerhaft, ohne je einzurasten. Die Nicht-
            # Arm-Gelenke (Taille) bleiben aus der Messung, damit die FK-Basis
            # stimmt.
            if self.use_robot:
                current_all = self.get_current_motor_q()
                for i, jidx in enumerate(LEFT_JOINT_INDICES_LIST):
                    current_all[jidx] = self._last_q_target[i]
                for i, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
                    current_all[jidx] = self._last_q_target[7 + i]
            else:
                current_all = self._assemble_full_from_last()

            try:
                self.ik_solver.set_current_configuration({
                    "left":  self._last_q_target[0:7].copy(),
                    "right": self._last_q_target[7:14].copy()
                })
            except Exception:
                pass

            # Ziel-Filter des Solvers zeitbasiert (smoothing_ref_hz), nur fuer
            # diesen Takt-Aufruf; Callbacks behalten ihr Alpha pro Nachricht.
            self.ik_solver.goal_filter_alpha = self._alpha_dt(
                self._ik_goal_alpha_base, self._last_dt)
            if self._goal_left_filt is not None:
                self.ik_solver.set_goal("left", self._goal_left_filt)
            if self._goal_right_filt is not None:
                self.ik_solver.set_goal("right", self._goal_right_filt)
            self.ik_solver.goal_filter_alpha = self._ik_goal_alpha_base

            held = {side: self._ik_side_held(side) for side in ("left", "right")}
            q_dict = ({} if held["left"] and held["right"]
                      else self.ik_solver.get_joint_targets(current_all))
            for side in ("left", "right"):
                if held[side]:
                    q_dict.pop(side, None)      # Gelenke exakt halten
            # Default: HALTEN. Ohne gesetztes IK-Ziel (kein Marker gezogen)
            # liefert get_joint_targets ein leeres Dict – dann muss der Arm in
            # seiner letzten Position bleiben. Frueher stand hier zeros(14), was
            # beide Arme beim Enable mit voller Geschwindigkeit in die
            # Null-Konfiguration (Gelenklimits/Selbstkollision) trieb -> Zappeln.
            q_target = self._last_q_target.copy()
            if "left" in q_dict:
                q_target[0:7] = q_dict["left"]
            if "right" in q_dict:
                q_target[7:14] = q_dict["right"]

        dt = self._compute_dt()
        max_step = self.arm_velocity_limit * dt
        # Richtungserhaltend skalieren statt je Gelenk zu clippen: sonst laufen
        # Gelenke mit kleinem Weg frueh fertig und der Arm faehrt NICHT die vom
        # Planer gepruefte Gerade, sondern einen Knick -- z.B. Sichere_Pose ->
        # ueber den Tisch: Hand 6 cm in die Tischkante statt 12 cm darueber.
        dq = q_target - self._last_q_target
        dq_max = float(np.max(np.abs(dq)))
        if dq_max > max_step:
            dq = dq * (max_step / dq_max)

        q_unsmoothed = self._last_q_target + dq
        alpha = self.ik_alpha
        if self._planned_motion_active:
            pa = float(self.get_parameter("planned_motion_alpha").value)
            alpha = pa if pa > 0.0 else self.ik_alpha
        alpha = self._alpha_dt(alpha, dt)
        self._last_dt = dt
        q_smooth = (1.0 - alpha) * self._last_q_target + alpha * q_unsmoothed
        # Kartesisches Speedlimit (TCP + Ellbogen, ISO 10218-1 reduced speed):
        # begrenzt auch die Faelle, in denen ein kleiner Gelenkschritt eine
        # grosse Handbewegung erzeugt (gestreckter Arm, Schulterdrehung).
        q_smooth = self._limit_cartesian_speed(self._last_q_target, q_smooth, dt)
        # Anfahrrampe (real): kein Sprung von 0 auf volle Geschwindigkeit.
        q_smooth = self._limit_accel(self._last_q_target, q_smooth, dt)
        # Selbstkollisions-Gate: nie in den eigenen Koerper fahren.
        q_smooth = self._apply_collision_gate(self._last_q_target, q_smooth)
        self._last_cmd_speed = float(np.max(np.abs(q_smooth - self._last_q_target))) / max(dt, 1e-4)
        # Schleppfehler-Begrenzung (real): Sollwert nie weit weg von der Messung.
        q_smooth = self._limit_tracking_error(q_smooth)
        self._last_q_target = q_smooth.copy()

        if not (self.walk_mode or self.homing_active or self.homing_reached
                or self._planned_motion_active or self._releasing):
            self._update_marker_goals()

        if self.use_robot:
            self._write_arm_cmd(q_smooth, self._advance_arm_weight(dt))
        else:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [JOINT_NAMES_ROS[i] for i in sorted(JOINT_NAMES_ROS.keys())]
            js.position = [0.0] * len(js.name)
            for idx, joint_idx in enumerate(LEFT_JOINT_INDICES_LIST):
                js.position[joint_idx] = float(q_smooth[idx])
            for idx, joint_idx in enumerate(RIGHT_JOINT_INDICES_LIST):
                js.position[joint_idx] = float(q_smooth[7 + idx])
            self.joint_pub.publish(js)


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    import signal
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:  # aeltere rclpy
        ExternalShutdownException = KeyboardInterrupt
    rclpy.init(args=args)
    node = ArmController()
    # SIGTERM (docker stop, launch-Eskalation) wie Strg+C behandeln: raus aus
    # spin und die Arme sauber an den Onboard-Regler zurueckgeben, statt den
    # arm_sdk-Strom mitten in einer Bewegung abreissen zu lassen.
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:  # noqa: BLE001 -- auch bei Absturz die Arme abgeben
        node.get_logger().error(f"arm_controller abgestuerzt: {e!r} -- gebe Arme ab.")
    finally:
        try:
            node.shutdown_release()
        except Exception as e:  # noqa: BLE001
            print(f"[arm_controller] Rueckgabe beim Beenden fehlgeschlagen: {e!r}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

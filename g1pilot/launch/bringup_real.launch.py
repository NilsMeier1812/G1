"""
bringup_real.launch.py
Schlankes Bringup fuer den ECHTEN G1 (erster Hardware-Betrieb):
Arm-Manipulation (IK/rviz-Marker) + Inspire-FTP-Haende (Modbus TCP) +
Unitree-eigener Loco-Controller (Balance/Gehen via loco_client).

Aufbau:
  - KEIN LiDAR/Navigation (Livox/MOLA wurde 2026-10 verworfen und entfernt)
  - loco_client direkt als Node (mit Walk-Safety-Parametern) statt ueber
    navigation_launcher (der auch nav2point/dijkstra hochziehen wuerde)
  - komplett env-getrieben (vom start.sh-Real-Zweig gesetzt), keine
    Launch-Argumente, die verloren gehen koennen

DDS-Konvention (utils/common.py): G1_SIM_MODE=false -> Unitree-DDS auf
Domain 0 ueber ROBOT_INTERFACE; der ROS-Graph laeuft daneben auf
ROS_DOMAIN_ID=1 (loopback, cyclonedds.xml) -- sonst kollidieren beide
CycloneDDS-Instanzen im selben Prozess ("create domain error").
"""
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os


def _env_bool(name, default='0'):
    return os.environ.get(name, default).strip().lower() in ('1', 'true', 'yes', 'on')


def generate_launch_description():
    pkg_share = FindPackageShare('g1pilot').find('g1pilot')

    # ── Konfiguration ueber Umgebungsvariablen (start.sh-Real-Zweig) ─────
    interface = os.environ.get('ROBOT_INTERFACE', 'eth0').strip()
    use_rviz = os.environ.get('USE_RVIZ', 'true').strip().lower()
    inspire_hands = _env_bool('G1_INSPIRE_HANDS')
    left_host = os.environ.get('G1_HAND_LEFT_HOST', '192.168.123.210').strip()
    right_host = os.environ.get('G1_HAND_RIGHT_HOST', '192.168.123.211').strip()
    hand_port = int(os.environ.get('G1_HAND_PORT', '6000'))
    joystick_name = os.environ.get('JOYSTICK_NAME', 'Wireless Controller').strip()
    # PS4-Controller fuer den OBERKOERPER (Arme + Haende) und Gehen mit L2 als
    # Deadman (ps4_walk -> /g1pilot/ps4/walk_cmd -> loco_client). Der alte
    # joystick-Node startet dann nicht. Real vorsichtiger als Sim:
    # Start-Tempo Langsam, hoechstens Normal (15 cm/s < EE-Limit 0.25 m/s),
    # Options uebernimmt nur die Arme (kein start_balancing -> der Controller
    # aendert den Zustand der Beine nie; PS = NOT-HALT wie die GUIs).
    # Siehe g1pilot/docs/11-A-Controller.md.
    ps4_arms = _env_bool('G1_PS4_ARMS')
    ps4_speed = os.environ.get('G1_PS4_SPEED', 'Langsam').strip()
    ps4_max_speed = os.environ.get('G1_PS4_MAX_SPEED', 'Normal').strip()
    ps4_ps_estop = 'true' if _env_bool('G1_PS4_PS_ESTOP', '1') else 'false'
    # Gehen per PS4 (L2 halten + Sticks, loco_client Move() nur im Walk-Modus).
    # G1_PS4_WALK=0 -> L2 ohne Wirkung (Stand vor 2026-10-19).
    ps4_walk = 'true' if _env_bool('G1_PS4_WALK', '1') else 'false'
    ps4_walk_speed = os.environ.get('G1_PS4_WALK_SPEED', 'Langsam').strip()
    ps4_walk_max_speed = os.environ.get('G1_PS4_WALK_MAX_SPEED', 'Schnell').strip()
    # Konservative Geh-Limits fuer die ersten Tests (normierte UI-Kommandos
    # werden in loco_client damit skaliert; gilt auch fuer den PS4-Pfad).
    max_vx = os.environ.get('G1_MAX_VX', '0.4')
    max_vy = os.environ.get('G1_MAX_VY', '0.3')
    max_vyaw = os.environ.get('G1_MAX_VYAW', '0.4')

    nodes = [

        # ── 1. robot_state: rt/lowstate -> /joint_states + IMU + TFs + RViz ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'robot_state_launcher.launch.py')
            ),
            launch_arguments={
                'use_robot':            'true',
                'interface':            interface,
                'publish_joint_states': 'true',
                # Finger gibt robot_state nur ab, wenn die Inspire-Bridge sie uebernimmt.
                'publish_hand_joints':  'false' if inspire_hands else 'true',
                # RViz default AN: der rviz-Marker ist auf real das Interface
                # fuer die Arm-Manipulation.
                'use_rviz':             use_rviz,
            }.items()
        ),

        # Statische TF: odom_unitree -> base_link (Identity; real gibt es keine
        # bewegte odom-Quelle -- RViz braucht den Frame trotzdem).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_real_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom_unitree', 'base_link']
        ),

        # ── 2. arm_controller + interactive_marker (rt/arm_sdk) ─────────────
        #    Gewichts-Rampe AKTIV (Defaults 2.0 s) -- weiche Uebergabe zwischen
        #    Roboter-eigener Armsteuerung und arm_sdk. (Sim setzt hier 0.0.)
        #    PD-Gains: die auf echter Hardware erprobten Werte aus dem
        #    offiziellen Unitree-g1_arm7_sdk-Beispiel (kp=60, kd=1.5) statt der
        #    Sim-Gains (150/12). Speedlimits: 0.25 m/s TCP+Ellbogen (ISO
        #    10218-1 reduced speed) + 1.5 rad/s Gelenk; Selbstkollisions-Gate an.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'manipulation_launcher.launch.py')
            ),
            launch_arguments={
                'use_robot': 'true',
                'interface': interface,
                'arm_velocity_limit': os.environ.get('G1_ARM_VEL_LIMIT', '1.5'),
                'ee_velocity_limit':  os.environ.get('G1_EE_VEL_LIMIT', '0.25'),
                'self_collision_gate': 'true',
                # Schleppfehler-Begrenzung + Anfahrrampe (siehe 50-A-Echter-Roboter.md).
                'max_tracking_error': os.environ.get('G1_ARM_MAX_TRACKING_ERROR', '0.3'),
                'arm_accel_limit':    os.environ.get('G1_ARM_ACCEL_LIMIT', '3.0'),
                # Marker-/Jog-IK im eigenen Prozess (Arm ratterte beim Joggen,
                # Real-Test 2026-10-16). G1_IK_ASYNC=0 = altes Verhalten.
                'ik_async': 'false' if os.environ.get('G1_IK_ASYNC', '1') == '0' else 'true',
                'kp_low':   '60.0',
                'kd_low':   '1.5',
                'kp_wrist': '40.0',
                'kd_wrist': '1.5',
                # Taille halten, solange arm_sdk Gewicht hat (sonst kraftlos ->
                # Oberkoerper kippt). Gains ungeprueft; per Env anpassbar.
                'waist_hold': os.environ.get('G1_WAIST_HOLD', 'true'),
                'kp_waist': os.environ.get('G1_WAIST_KP', '150.0'),
                'kd_waist': os.environ.get('G1_WAIST_KD', '4.0'),
                # E-Stop laesst die Taille gehalten (Real-Test 2026-10-15).
                'estop_waist_hold': os.environ.get('G1_ESTOP_WAIST_HOLD', 'true'),
                # Hueftdrehung (R2 halten + rechter Stick): real zuerst nur +-15 deg
                # (Taillen-Gains am G1 ungeprueft). G1_WAIST_YAW=0 schaltet sie ab.
                'waist_yaw_control': 'false' if os.environ.get('G1_WAIST_YAW', '1') == '0' else 'true',
                'waist_yaw_limit_deg': os.environ.get('G1_WAIST_YAW_LIMIT_DEG', '105.0'),
                # LAUFEN: Arme bleiben in ihrer Pose (Nils 2026-10-19). Mit
                # G1_WALK_PARK_ARMS=true wie frueher in die Lauf-Pose parken.
                'walk_park_arms': os.environ.get('G1_WALK_PARK_ARMS', 'false'),
            }.items()
        ),

        # ── 3. Teleoperation: Demo-GUI + PS4-Controller (evdev) ────────
        #    G1_PS4_ARMS=0: alter joystick-Node -> joy_mux -> loco_client (Gehen
        #    mit Deadman). G1_PS4_ARMS=1: ps4_joystick + ps4_arm_teleop (Arme +
        #    Haende) + ps4_walk (L2 halten + Sticks = Gehen, G1_PS4_WALK=0 aus).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'teleoperation_launcher.launch.py')
            ),
            launch_arguments=({
                'joystick_name': joystick_name,
                'ps4_arms': 'true',
                'ps4_speed': ps4_speed,
                'ps4_max_speed': ps4_max_speed,
                'ps4_options_balancing': 'false',
                'ps4_ps_estop': ps4_ps_estop,
                'ps4_walk': ps4_walk,
                'ps4_walk_speed': ps4_walk_speed,
                'ps4_walk_max_speed': ps4_walk_max_speed,
            } if ps4_arms else {
                'joystick_name': joystick_name,
            }).items()
        ),

        # ── 4. loco_client: Unitree-Onboard-High-Level (Balance/Gehen) ──────
        #    START -> FSM 4, START BALANCING -> BalanceStand, WALK/loco_cmd_vel
        #    bzw. PS4-Deadman -> Move(). damp_on_init bleibt false: der Node
        #    greift beim Start NICHT in den Roboter-Zustand ein.
        Node(
            package='g1pilot',
            executable='loco_client',
            name='loco_client',
            output='screen',
            parameters=[{
                'interface':       interface,
                'use_robot':       True,
                'max_vx':          float(max_vx),
                'max_vy':          float(max_vy),
                'max_vyaw':        float(max_vyaw),
                'cmd_vel_timeout': 0.5,
                'damp_on_init':    False,
                # RPC-Dienst des Onboard-Reglers (aktuelle Firmware: 'sport').
                'loco_service':    os.environ.get('G1_LOCO_SERVICE', 'sport'),
                # Walk-Modus beim Start: 501 = Walk with waist, 500 = Walk (Taille fest).
                'start_fsm_id':    int(os.environ.get('G1_WALK_FSM_ID', '501')),
                # E-Stop: Beine nur StopMove (kein Damp). G1_ESTOP_DAMP_LEGS=1 = alt.
                'estop_damp_legs': _env_bool('G1_ESTOP_DAMP_LEGS'),
                # Bodenkontakt vor Walk: knee_load (Knie-Last aus rt/lowstate),
                # fsm_mode (alt, am Kran wirkungslos) oder none. Schwelle je Knie.
                'ground_check':     os.environ.get('G1_GROUND_CHECK', 'knee_load'),
                'knee_load_min_nm': float(os.environ.get('G1_KNEE_LOAD_MIN_NM', '10.0')),
            }]
        ),
    ]

    # ── 5. Inspire-FTP-Hand-Bridge: ECHTE Haende via Modbus TCP ─────────────
    #    GUIs identisch zur Sim (Controller :8766 / Viewer :8765 / HTTP :8767);
    #    Ist-Winkel/Kraefte/Taktil kommen von der Hardware, RViz spiegelt die
    #    echten Finger ueber /joint_states.
    if inspire_hands:
        nodes.append(Node(
            package='g1pilot',
            executable='inspire_hand',
            name='inspire_ftp_bridge',
            output='screen',
            parameters=[{
                'ws_host':         '0.0.0.0',
                'controller_port': 8766,
                'viewer_port':     8765,
                'update_rate_hz':  50.0,
                'backend':         'modbus',
                'left_host':       left_host,
                'right_host':      right_host,
                'modbus_port':     hand_port,
            }]
        ))

    return LaunchDescription(nodes)

# Architektur — Technik

Richtet sich an: Entwickler, die sich einen Überblick verschaffen wollen,
bevor sie an einer bestimmten Stelle etwas ändern. Für die Details der
einzelnen Subsysteme siehe die jeweiligen `*-T-*.md`-Dokumente
(Übersicht in [README.md](README.md)).

## Gesamtbild

G1Pilot ist ein ROS-2-Package (`g1pilot`), das über die Unitree-DDS-Schnittstelle
(`unitree_sdk2py`) sowohl mit dem echten Unitree G1 als auch mit einer
MuJoCo-Simulation spricht — **derselbe Code, dieselbe Nachrichten-API**, nur
die DDS-Domain/das Interface unterscheiden sich. Der Roboter selbst hat 29
Freiheitsgrade (Beine, Taille, zwei 7-DOF-Arme).

```
.
├── g1pilot/                     ROS-2-Package + Docker/Compose/Makefile (Hauptprojekt)
│   ├── g1pilot/
│   │   ├── state/                Roboterzustand -> /joint_states, /tf, IMU, Temperaturen
│   │   ├── manipulation/         IK, Arm-Regelung, Planer, Positionsspeicher, Arm-API, Hüfte, Hände
│   │   ├── navigation/           Locomotion (Sim), Loco-Client (Real), Bodenkontakt, autonome Navigation
│   │   ├── teleoperation/        Demo-GUI, Streamdeck, PS4/Xbox-Controller, Handy (phone/), Abläufe
│   │   ├── tools/                Diagnose- und Sim-Werkzeuge (dds_check, sim_*), Beispiel-Abläufe
│   │   └── utils/                IK-Solver, Joint-Tabellen, Temperaturen, Fenster-Stil, Hilfsfunktionen
│   ├── launch/                   ROS-2-Launchdateien (Sim-/Real-Bringup + Bausteine)
│   ├── docker/, docker-compose*.yml  Container-Definitionen, Shutdown-Watcher
│   ├── policies/                 RL-Lauf-Policies (ONNX) + deploy.yaml: g1_wholebody (Standard), agile_velocity_g1, agile_velocity_height_g1
│   ├── data/arm_poses.json       gespeicherte Posen und Abläufe (versioniert)
│   ├── test/                     Unit- und Integrationstests (run_tests.sh)
│   ├── start.sh, g1_gui.py       Startmenü (Text / grafisch)
│   └── docs/                     diese Dokumentation
├── unitree_mujoco/               MuJoCo-Simulation (Unitree-Original, angepasst)
│   └── simulate_python/
│   │   ├── unitree_mujoco.py        Sim-Loop + Viewer
│   │   └── unitree_sdk2py_bridge.py DDS <-> MuJoCo, Befehls-Merge
│   └── scene_editor/             Umgebungs-Editor und -Builder (eigener Code)
├── unitree_ros2/                 Unitree-ROS-2-Abhängigkeiten
└── unitree_sdk2_python/          Unitree-SDK (Python-Bindings für die DDS-Nachrichten)
```

## Laufzeit-Topologie (Simulation)

Zwei Docker-Container, beide mit `network_mode: host`:

```
┌─ HOST (network_mode: host) ────────────────────────────────────────────┐
│                                                                        │
│  Service: mujoco-sim                   Service: g1pilot-sim            │
│  ┌──────────────────────────┐          ┌────────────────────────────┐│
│  │ MuJoCo-Physik (1 kHz)    │          │ ROS 2 Humble               ││
│  │ unitree_sdk2py_bridge     │          │  robot_state · arm_ctrl    ││
│  │   (DDS <-> MuJoCo)        │          │  interactive_marker        ││
│  └────────────┬─────────────┘          │  loco_sim · teleop · rviz  ││
│               │                        └─────────────┬──────────────┘│
│               │     DDS Domain 1 / lo (CycloneDDS)   │                │
│               └──────────────────────────────────────┘                │
└────────────────────────────────────────────────────────────────────────┘
```

Auf dem echten Roboter entfällt der `mujoco-sim`-Container; stattdessen
spricht der Container `g1pilot-real` (bzw. `g1pilot-real-full` mit
LiDAR/Navigation) über das physische Netzwerkinterface direkt mit dem
G1-Onboard-Rechner. Die Inspire-Hände hängen real per Modbus TCP am selben
Netz (`192.168.123.210`/`.211`).

## Zwei Kommunikationsebenen

- **Unitree-DDS (CycloneDDS)** — zwischen dem g1pilot-Code und dem Roboter
  (Sim: Bridge; Real: G1-Onboard-Rechner). Identische Nachrichtentypen wie am
  echten Roboter (`unitree_hg`-IDL). Wichtige Topics:

  | Topic | Typ | Richtung | Inhalt |
  |---|---|---|---|
  | `rt/lowstate` | `LowState_` | Roboter → Stack | Gelenk-q/dq/τ, IMU; (Sim) Fußkraft + Basis-v in `reserve[]` |
  | `rt/lowcmd` | `LowCmd_` | Stack → Roboter | Bein-/Taillen-Befehle (`loco_sim`/`loco_client`) |
  | `rt/arm_sdk` | `LowCmd_` | Stack → Roboter | Arm-Befehle (`arm_controller`), gewichtet gemischt mit dem Onboard-Regler |
  | `rt/sportmodestate` | `SportModeState_` | Roboter → Stack | Odometrie (Sim: Ground Truth) |
  | `rt/wirelesscontroller` | `WirelessController_` | Roboter → Stack | Gamepad-Zustand |
  | `rt/inspire/cmd`\|`state` | intern | beidseitig | Inspire-Hand-Finger (nur Sim-Backend `mujoco`) |

  ```
  LowCmd_.motor_cmd[i]:  q, dq, tau, kp, kd  ->  ctrl = tau + kp*(q-q_ist) + kd*(dq-dq_ist)
  LowState_.motor_state[i]: q, dq, tau_est
  LowState_.imu_state: quaternion[w,x,y,z], gyroscope[xyz], accelerometer[xyz]
  ```

- **ROS 2 (rclpy, CycloneDDS als RMW)** — innerhalb des g1pilot-Containers:
  `/joint_states`, `/tf`, Hand-Ziele, RViz, Teleoperation, der ganze
  g1pilot-eigene Node-Graph. Muss auf einer **anderen** DDS-Domain laufen als
  die Unitree-Schnittstelle, sonst kollidieren beide CycloneDDS-Instanzen im
  selben Prozess ("create domain error").

## Sim ↔ Real: Ein Schalter

Der zentrale Unterschied zwischen Simulation und echtem Roboter ist eine
einzige Umgebungsvariable, `G1_SIM_MODE`, ausgewertet in
`g1pilot/utils/common.py`:

```python
def resolve_dds(interface):
    if is_sim_mode():
        return SIM_DDS_DOMAIN, SIM_DDS_INTERFACE   # 1, "lo"
    return REAL_DDS_DOMAIN, interface               # 0, <physisches NIC>
```

| Aspekt | Simulation | Echter Roboter |
|---|---|---|
| `G1_SIM_MODE` | `true` | `false` |
| Unitree-DDS-Domain | `1` | `0` (Roboter-Firmware fest) |
| `ROS_DOMAIN_ID` | `0` | `1` |
| DDS-Interface | `lo` (Loopback) | physisches Ethernet-NIC |
| Locomotion | `loco_sim` (RL-Policy + PD-Balancer) | `loco_client` (Unitree-Onboard-High-Level) |
| Start | `make sim` / `./start.sh` → Simulation | `make real ROBOT_INTERFACE=<nic>` / `./start.sh` → Echter Roboter |
| Compose-Profil | `sim` | `real` / `real-full` |
| Launch-Datei | `bringup_sim.launch.py` | `bringup_real.launch.py` / `bringup_launcher.launch.py` |

Jeder Node, der die DDS-Verbindung öffnet, ruft dafür
`g1pilot.utils.common.init_dds(interface, logger)` auf — das ist die
**einzige** Stelle, an der Sim/Real unterschieden wird. Alle anderen Module
(Arm-IK, Positionsspeicher, Teleoperation, Navigation) sind
modus-unabhängig.

## Joint-Layout (29 DOF)

| Index | Gruppe | Gelenke |
|---|---|---|
| 0–5 | Bein links | hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll |
| 6–11 | Bein rechts | (analog) |
| 12–14 | Taille | waist_yaw, waist_roll, waist_pitch |
| 15–21 | Arm links | shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw |
| 22–28 | Arm rechts | (analog) |

Reine Drehmoment-Aktuatoren, jedes Gelenk drehmomentbegrenzt (z. B. Knöchel
±50 Nm). Die Zuständigkeit für die drei Gruppen ist zwischen den Nodes
strikt aufgeteilt:

- **Beine + Taille (0–14):** `loco_sim` (Sim) bzw. der Unitree-Onboard-Regler
  über `loco_client` (Real), via `rt/lowcmd`.
- **Arme (15–28):** ausschließlich `arm_controller`, via `rt/arm_sdk`.

Auf der Sim-Seite mischt `unitree_sdk2py_bridge.py` (in `unitree_mujoco`)
beide Befehlsquellen pro Motor: Beine/Taille aus `rt/lowcmd`, Arme gewichtet
aus `rt/arm_sdk` (`motor_cmd[29].q` ist das Blend-Gewicht 0..1). Pro
Physik-Schritt wird angewendet: `ctrl = tau + kp*(q-q_ist) + kd*(dq-dq_ist)`.
Auf dem echten Roboter übernimmt dieselbe Logik die G1-Firmware.

## Projektstruktur — Node-Übersicht

| Node (`ros2 run g1pilot ...`) | Datei | Aufgabe |
|---|---|---|
| `robot_state` | `state/robot_state.py` | `rt/lowstate` → `/joint_states`, IMU, TF, Motor-Telemetrie |
| `arm_controller` | `manipulation/arm_controller.py` | Arm-IK, Sicherheitsgates, Positionsspeicher, `rt/arm_sdk` |
| `arm_api` | `manipulation/arm_api.py` | HTTP/JSON-Brücke für externe Projekte |
| `interactive_marker` | `manipulation/interactive_marker.py` | RViz-Marker für Arm-Zielposen |
| `inspire_hand` | `manipulation/inspire_ftp/bridge.py` | Inspire-FTP-Hand-Bridge (GUIs + DDS/Modbus) |
| `loco_sim` | `navigation/loco_sim.py` | Sim-Stellvertreter für Stehen/Laufen (nur Sim) |
| `loco_client` | `navigation/loco_client.py` | Unitree-Onboard-Locomotion ansprechen (nur Real) |
| `dijkstra_planner`, `nav2point`, `create_map` | `navigation/*.py` | autonome Punkt-zu-Punkt-Navigation |
| `scene_bridge` | `navigation/scene_bridge.py` | Umgebungsobjekte (Sim) → `/scene_markers` |
| `sim_localization` | `navigation/sim_localization.py` | Sim-Ersatz für MOLA-Lokalisierung |
| `joy_to_cmdvel` | `navigation/joy_to_cmdvel.py` | Sim: `/g1pilot/joy` → `/g1pilot/loco_cmd_vel` (Navigation) |
| `mola_fixed` | `navigation/fix_mola_odometry.py` | Real: Orientierungs-Fix der MOLA-Odometrie |
| `demo_gui` | `teleoperation/demo_gui.py` | Demo-GUI (Standard-Oberfläche, inkl. Handy-Gateway) |
| `ui_interface` | `teleoperation/ui_interface.py` | Streamdeck-GUI (alle Einzelfunktionen) |
| `ps4_joystick`, `ps4_arm_teleop`, `ps4_walk` | `teleoperation/ps4_*.py` | PS4-/Xbox-Controller: Eingabe, Arme/Hände/Hüfte, Gehen mit L2 |
| `joystick`, `joy_mux` | `teleoperation/*.py` | alter Joystick-Modus, Auto/Manuell-Mux |
| `dds_check`, `sim_dds_check` | `tools/sim_dds_check.py` | Verbindungstest zur Unitree-DDS (real bzw. Sim) |
| `sim_arm_wiggle`, `sim_goal_probe`, `sim_arm_monitor`, `sim_leg_hold` | `tools/sim_*.py` | Sim-Diagnose |

Welcher Node welches Topic schreibt und liest, zeigen die Diagramme in
[05-T-Nodes-und-Topics.md](05-T-Nodes-und-Topics.md).

Werkzeuge außerhalb des Node-Graphen: [61-A-Werkzeuge.md](61-A-Werkzeuge.md).

## Weiterführend

- [04-T-Umgebungsvariablen.md](04-T-Umgebungsvariablen.md) — alle `G1_*`-Schalter
- [05-T-Nodes-und-Topics.md](05-T-Nodes-und-Topics.md) — Node-/Topic-Diagramme
- [10-T-Demo-GUI.md](10-T-Demo-GUI.md) — Demo-GUI, Topics
- [11-T-Controller.md](11-T-Controller.md) — PS4/Xbox-Controller
- [12-T-Handy.md](12-T-Handy.md) — Handy-Gateway (Bluetooth LE)
- [13-T-Streamdeck.md](13-T-Streamdeck.md) — Streamdeck, alter Joystick, Joy-Mux
- [20-T-Arm-Manipulation.md](20-T-Arm-Manipulation.md) — IK, Planer, Sicherheitsgates
- [21-T-Arm-API.md](21-T-Arm-API.md) — HTTP-Brücke
- [22-T-Inspire-Haende.md](22-T-Inspire-Haende.md) — Hand-Bridge
- [23-T-Posen-und-Ablaeufe.md](23-T-Posen-und-Ablaeufe.md) — Positionsspeicher, Abläufe
- [24-T-Hueftdrehung.md](24-T-Hueftdrehung.md) — Hüftdrehung, Taille halten
- [30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md) — Stehen/Laufen in Sim und Real
- [31-T-Navigation.md](31-T-Navigation.md) — autonome Navigation
- [40-T-Simulation.md](40-T-Simulation.md) — MuJoCo, Bridge, Umgebungen
- [51-T-Notaus.md](51-T-Notaus.md) — Halt-Wege im Code
- [60-A-Tests.md](60-A-Tests.md) — Testsuite

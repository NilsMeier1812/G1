# G1Pilot

ROS-2-Package für den Unitree G1 (29 DOF). Steuert Arme (Manipulation),
Locomotion, Teleoperation und optionale autonome Navigation — unverändert
gegen eine MuJoCo-Simulation oder den echten Roboter.

Projektüberblick: [../README.md](../README.md). Vollständige Dokumentation:
[docs/README.md](docs/README.md).

## Voraussetzungen

**Docker muss installiert sein**, bevor `make build-sim` funktioniert
(sonst: `make: docker: No such file or directory`). Unter Ubuntu:

```bash
# Docker Engine installieren
curl -fsSL https://get.docker.com | sudo sh

# Docker ohne sudo nutzen (danach neu einloggen oder `newgrp docker`)
sudo usermod -aG docker $USER

# Prüfen
docker --version
docker run hello-world

# Host-Pakete: Tkinter (grafisches Startmenü von ./start.sh) und venv
# (Umgebungs-Builder / Scene-Editor)
sudo apt install -y python3-tk python3-venv
python3 -c 'import tkinter; print(tkinter.TkVersion)'

# Umgebungs-Builder einrichten (trimesh + vhacdx für build_env_scene.py)
cd ../unitree_mujoco/scene_editor && ./setup.sh && cd -
```

Hinweise:

- Snap-Docker (`sudo snap remove docker`) bitte vorher entfernen — er macht
  häufig Pfad- und Rechteprobleme.
- Tkinter (`python3-tk`) ist nicht im Docker-Image enthalten, sondern wird
  auf dem Host gebraucht. Fehlt es, startet `./start.sh` ohne Fehlermeldung
  das Text-Menü statt der GUI. Das gilt auch unter WSL2: Das Display kommt
  dort über WSLg, Tkinter muss aber trotzdem installiert werden.
- Ohne `scene_editor/.venv` mit `trimesh`/`vhacdx` baut `start.sh`
  Umgebungen ohne Fehlermeldung mit groben Kollisionshüllen (Greif-Objekte
  fliegen weg). Nach dem Verschieben des Repos das `.venv` neu anlegen:
  `rm -rf .venv && ./setup.sh`.
- **WSL2 + VPN (z. B. eduVPN):** Kommt WSL nicht ins Internet (`apt` hängt
  bei `Ign:`, `curl` läuft in einen Timeout), Mirrored Networking aktivieren.
  Anleitung: [WSL2-Netzwerk](docs/01_installation.md#wsl2-netzwerk-vpn-uni-wlan-roboter).
- Für GPU-Betrieb zusätzlich das NVIDIA Container Toolkit installieren
  (siehe `docker-compose.nvidia.yml` / `docker-compose.gpu.yml`).

## Schnellstart

```bash
cd g1pilot
make build-sim
make sim
```

Details, Voraussetzungen und Windows/WSL2-Anleitung:
[docs/01_installation.md](docs/01_installation.md).

Alltäglicher Einstieg ist `./start.sh` — öffnet ein grafisches Startmenü
(Simulation / Echter Roboter / Umgebungen), von
dem aus auch die gesamte Dokumentation erreichbar ist.

## Nodes

| Node | Datei | Aufgabe |
|---|---|---|
| `robot_state` | `g1pilot/state/robot_state.py` | Roboterzustand → `/joint_states`, `/tf`, IMU, Motor-Telemetrie |
| `arm_controller` | `g1pilot/manipulation/arm_controller.py` | Arm-IK, Sicherheitsgates, Positionsspeicher |
| `arm_api` | `g1pilot/manipulation/arm_api.py` | HTTP/JSON-Brücke für externe Projekte |
| `interactive_marker` | `g1pilot/manipulation/interactive_marker.py` | RViz-Marker für Arm-Zielposen |
| `inspire_hand` | `g1pilot/manipulation/inspire_ftp/bridge.py` | Inspire-FTP-Hand-Bridge (GUIs + DDS/Modbus) |
| `loco_sim` | `g1pilot/navigation/loco_sim.py` | Stehen/Laufen in der Simulation |
| `loco_client` | `g1pilot/navigation/loco_client.py` | Unitree-Onboard-Locomotion (echter Roboter) |
| `dijkstra_planner`, `nav2point`, `create_map` | `g1pilot/navigation/*.py` | autonome Punkt-zu-Punkt-Navigation |
| `scene_bridge` | `g1pilot/navigation/scene_bridge.py` | Umgebungsobjekte (Sim) → `/scene_markers` |
| `joystick`, `joy_mux` | `g1pilot/teleoperation/*.py` | physischer Controller, Auto/Manuell-Mux |
| `ui_interface` | `g1pilot/teleoperation/ui_interface.py` | Streamdeck-Bedienoberfläche |

Ausführliche Beschreibung jedes Bereichs in den jeweiligen
`docs/*_technik.md`-Dokumenten (siehe [docs/README.md](docs/README.md)).

## Konfiguration

Zentrale Einstellungen liegen in `config/config.yaml`. Der Sim/Real-Umschalter
läuft über die Umgebungsvariable `G1_SIM_MODE` (siehe
`g1pilot/utils/common.py` und
[docs/02_architektur.md](docs/02_architektur.md)):

| Modus | `G1_SIM_MODE` | Unitree-DDS-Domain | `ROS_DOMAIN_ID` | Interface | Entry Point |
|---|---|---|---|---|---|
| Simulation | `true` | 1 | 0 | `lo` | `bringup_sim.launch.py` |
| Echter Roboter | `false` | 0 | 1 | `${ROBOT_INTERFACE}` | `bringup_real.launch.py` |
| Echter Roboter (voller Nav-Stack) | `false` | 0 | 1 | `${ROBOT_INTERFACE}` | `bringup_launcher.launch.py` |

## Manueller Betrieb ohne `start.sh`

```bash
colcon build
source install/setup.bash
ros2 launch g1pilot bringup_launcher.launch.py
```

Empfohlener konsolidierter Docker-Einstiegspunkt:

```bash
# Simulation: MuJoCo-G1 + g1pilot (robot_state, Arme, RViz, Teleop)
G1_SIM_MODE=true docker compose --profile sim up

# Echter Roboter (schlank: Arme + Hände + Unitree-Loco)
ROBOT_INTERFACE=<iface> docker compose --profile real up

# Echter Roboter mit Livox/MOLA/Navigation (großes Image, MID360 nötig)
ROBOT_INTERFACE=<iface> docker compose --profile real-full up
```

**Vor dem ersten Start auf echter Hardware:**
[docs/70_echtroboter_anleitung.md](docs/70_echtroboter_anleitung.md) lesen
(Sicherheitscheckliste + Ablauf).

Einzelne Bausteine lassen sich auch getrennt starten, z. B.:

```bash
ros2 launch g1pilot livox_launcher.launch.py
ros2 launch g1pilot mola_launcher.launch.py
ros2 launch g1pilot navigation_launcher.launch.py
ros2 launch g1pilot manipulation_launcher.launch.py
ros2 launch g1pilot teleoperation_launcher.launch.py
```

## Contributing

Beiträge sind willkommen: Repository forken, Branch für die Änderung
anlegen, Commits mit aussagekräftigen Nachrichten, Pull Request mit
Beschreibung der Änderung öffnen.

## Maintainer

**Clemente Donoso**
E-Mail: [clemente.donoso@inria.fr](mailto:clemente.donoso@inria.fr) ·
GitHub: [CDonosoK](https://github.com/CDonosoK)

## Lizenz

BSD-3-Clause. Siehe [LICENSE](LICENSE).

# Herkunft und Lizenzen

## g1pilot (Basis dieses Pakets)

Dieses ROS-2-Paket ist aus **[hucebot/g1pilot](https://github.com/hucebot/g1pilot)**
hervorgegangen (HuCeBot-Team, Inria/Loria; Autor Clemente Donoso, weitere
Beiträge von Hippolyte Henry und Jean-Baptiste Mouret). Abgezweigt wurde der
Upstream-Stand `455e287` / `efc5e7c` (Oktober/November 2025). Lizenz:
BSD-3-Clause, Copyright (c) 2024 HuCeBot Inria/Loria team, siehe [LICENSE](LICENSE).

Seitdem ist das Paket zum größten Teil neu geschrieben. Stand 2026-10-30 stammt
nur noch etwa ein Zehntel der Zeilen aus dem Original. Dateien, die noch
nennenswert Original-Code enthalten (Rest jeweils eigene Erweiterung):

| Datei | Original-Anteil (ca.) |
|---|---|
| `g1pilot/manipulation/arm_controller.py` | Grundgerüst (~20 %) |
| `g1pilot/manipulation/interactive_marker.py` | ~50 % |
| `g1pilot/utils/ik_solver.py` | ~30 % |
| `g1pilot/utils/common.py`, `joints_names.py`, `helpers.py`, `window_style.py` | 55–100 % |
| `g1pilot/navigation/loco_client.py` | Grundgerüst (~20 %) |
| `g1pilot/navigation/dijkstra_planner.py`, `nav2point.py`, `create_map.py` | 35–80 % |
| `g1pilot/state/robot_state.py` | ~80 % |
| `g1pilot/teleoperation/joy_mux.py`, `joystick.py`, `gui_base.py` | 25–80 % |
| `launch/*_launcher.launch.py` (robot_state, manipulation, navigation, teleoperation) | 20–75 % |
| `config/29dof.rviz`, `setup.py`, `package.xml`, `.gitignore` | 45–98 % |

Alle übrigen Dateien (u. a. Demo-GUI, Arm-Planer, `loco_sim` mit Lauf-Policies,
Inspire-Hände, PS4/Xbox-Teleop, Arm-API, Handy-Zugang, Tests, Doku) sind
eigene Arbeit des G1-Projekts. Entfernt wurden aus dem Original u. a. die
Dex3-Hand, die Streamdeck-GUI, Livox-/MOLA-LiDAR-Anbindung und das große
Docker-Image.

## Weitere Fremdbestandteile

| Bestandteil | Quelle | Lizenz |
|---|---|---|
| Roboter-Meshes `description_files/meshes/` (G1) | Unitree Robotics (über g1pilot) | BSD-3-Clause |
| `unitree_mujoco/`, `unitree_sdk2_python/`, `unitree_ros2/` (Repo-Wurzel) | Unitree Robotics | BSD-3-Clause, je eigene LICENSE |
| `policies/g1_wholebody/` | Unitree `unitree_rl_mjlab` | Apache-2.0, siehe `ATTRIBUTION.md` dort |
| `policies/agile_velocity_*` | NVIDIA WBC-AGILE | siehe `LICENSE`/`ATTRIBUTION` im Ordner |

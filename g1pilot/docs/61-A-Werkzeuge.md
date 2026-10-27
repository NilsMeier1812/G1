# Werkzeuge — Anleitung

Richtet sich an Entwickler und Betreuer. Hilfsprogramme außerhalb der
eigentlichen Bedienung: Make-Ziele, Startskripte, Diagnose-Nodes,
Posen-Werkzeuge und Messskripte. Alle Pfade relativ zu `g1pilot/`.

## Make-Ziele

| Ziel | Wirkung |
|---|---|
| `make build-sim` | Sim-Images bauen (~8 min) |
| `make build-real` | Real-Image bauen (schlank, ~10 min, ohne Livox/MOLA). **Vor** dem Wechsel ins Roboter-LAN, braucht Internet |
| `make build-real-full` | Real-Image mit Livox/MOLA (~45 min) |
| `make sim` / `make sim-bg` | Simulation starten (Vordergrund / Hintergrund; `sim-bg` ohne Shutdown-Watcher) |
| `make real` / `make real-full` | echter Roboter (schlank / mit LiDAR und Navigation), `ROBOT_INTERFACE=<nic>` |
| `make stop` | alles stoppen |
| `make logs`, `make status` | Logs bzw. laufende Container |
| `make shell-sim`, `make shell-mujoco`, `make shell-real` | Shell im jeweiligen Container |
| `make gpu-check` | GPU-Zugriff der Sim-Container prüfen (`docker/check_gpu.sh`) |
| `make phone-web` | Handy-Seite per USB aufs Handy ([12-A-Handy.md](12-A-Handy.md)) |
| `make clean` | aufräumen |

GPU-Dateien (`docker-compose.gpu.yml`, `docker-compose.nvidia.yml`) nehmen
`make` und `start.sh` automatisch dazu, wenn `/dev/dri` bzw. eine NVIDIA mit
Container Toolkit da ist. Der Alltagseinstieg ist trotzdem `./start.sh`
([03-A-Startmenue.md](03-A-Startmenue.md)).

## Startskripte (Sim)

| Skript | Wirkung |
|---|---|
| `run_sim.sh [--build]` | X11 freigeben, alte Container stoppen, Sim starten |
| `run_sim_loco.sh` | wie `run_sim.sh`, aber Basis frei (`HOLD_BASE_MODE=off`) für Loco-Tests |
| `stop_sim.sh` | Sim-Stack stoppen, verwaiste Container entfernen |

## Verbindung prüfen (DDS)

| Aufruf | Prüft |
|---|---|
| `ros2 run g1pilot sim_dds_check` | kommt `rt/lowstate` von MuJoCo an (Rate, Arm-Winkel)? Ohne ROS-Graph, nur Unitree-DDS |
| `ros2 run g1pilot dds_check --interface <nic>` | dasselbe gegen den echten G1 (`G1_SIM_MODE=false`). Nur im **laufenden** Real-Container per `exec`, siehe [50-A-Echter-Roboter.md](50-A-Echter-Roboter.md) |
| `python3 -m g1pilot.tools.dds_api_check <nic>` | welche RPC-Dienste (`rt/api/<dienst>/request`) der G1 anbietet, dazu eine `GetFsmId`-Abfrage. Hilft bei RPC-Fehler 3102 |

## Sim-Diagnose (Arme und Beine)

| Aufruf | Zweck |
|---|---|
| `ros2 run g1pilot sim_arm_wiggle -- --joint 22 --amp 0.4` | ein Armgelenk sinusförmig bewegen, Soll gegen Ist (ohne ROS-Graph, IK, Marker) |
| `ros2 run g1pilot sim_arm_monitor` | mitschneiden, was `arm_controller` auf `rt/arm_sdk` sendet und was zurückkommt (Zappeln einordnen) |
| `ros2 run g1pilot sim_goal_probe` | Hand-Ziel ohne Marker an den arm_controller schicken |
| `ros2 run g1pilot sim_leg_hold` | Beine per PD in einer Standpose halten (Bridge-Merge prüfen, Basis frei mit `run_sim_loco.sh`) |

## Posen und Abläufe

| Aufruf | Zweck |
|---|---|
| `python3 pose_tool.py list\|delete\|rename\|move …` | gespeicherte Posen aufräumen (Host, nur Standardbibliothek) |
| `python3 demo_sequence.py --category Demo` | Posen einer Kategorie über die Arm-API abfahren |
| `python3 -m g1pilot.tools.install_example_sequences` | Beispiel-Abläufe für die Szene A5 |
| `python3 examples/arm_api/arm_cli.py …` | Kommandozeilen-Client der Arm-API ([21-A-Arm-API.md](21-A-Arm-API.md)) |

Details: [23-A-Posen-und-Ablaeufe.md](23-A-Posen-und-Ablaeufe.md).

## Messskripte

| Skript | Misst |
|---|---|
| `jog_axis_probe_sim.py` | Nebenachsen und Kippen beim Controller-Jog in der Sim ([11-T-Controller.md](11-T-Controller.md#präzise-achsen-2026-10-21)) |
| `test/teleop_jitter_probe.py` | Befehlsstrom des arm_controllers (Real-Pfad) bei Jog gegen geplante Fahrt, gegen einen Fake-G1 mit Gelenkdynamik |
| `tools/dds_lowstate_load_robot.py` + `tools/dds_lowstate_load_client.py` | Last-Test: `rt/lowstate`-Listener gegen Abfrage ohne Listener (Grundlage der Bodenkontakt-Abfrage in `loco_client`) |

## Doku-Links prüfen

```bash
python3 tools/check_doc_links.py
```

Prüft alle Markdown-Links im Repo (Datei und Anker, wie GitHub sie
bildet) und alle Verweise `docs/<Name>.md` bzw. `NN-X-Name.md` in Code,
Skripten und Launch-Dateien. Exit-Code 1 bei toten Zielen. Nach jedem
Umbenennen oder Verschieben eines Abschnitts laufen lassen.

## Beispiele aus dem Unitree-SDK

Diese Skripte gehören nicht zum Stack und sprechen den echten G1 **direkt**
an, an GUI, NOT-HALT-Latch und Bodenkontakt-Prüfung vorbei. Nur am Kran
und mit Fernbedienung in der Hand benutzen.

| Datei | Wirkung |
|---|---|
| `g1pilot/state/lights.py <nic>` | LED-Leiste per `AudioClient` (Farbwechsel) |
| `g1pilot/state/voice.py <nic>` | Lautstärke, Sprachausgabe (chinesische Beispielsätze) und **`WaveHand()` — der Arm bewegt sich** |
| `g1pilot/tools/test.py` | **Hochfahr-Sequenz am Kran** (fest `eth0`): `Damp` → FSM 4 → `BalanceStand(0)` → `Start()`, gibt FSM-ID/-Mode aus |

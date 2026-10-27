# Tests — Anleitung

Richtet sich an Entwickler, die eine Änderung absichern wollen. Welche
Tests zu welchem Bereich gehören, steht in den Technik-Dokumenten der
Themen; hier stehen Aufruf, Gruppen und Regeln.

## Regel: gezielt testen

- Nur die Tests laufen lassen, die zur Änderung passen (betroffene Module
  plus direkte Abhängigkeiten). Bei kleinen GUI-/Anzeige-Änderungen reichen
  die GUI-Tests.
- Die **ganze Suite** nur bei sicherheitsrelevanten Änderungen (Notaus,
  Walk/Modus-Wechsel, Arm-Controller/Planer, Loco-Client) oder vor einem
  Merge nach `main`, wenn viele Bereiche betroffen sind.
- Im Bericht nennen, was gelaufen ist und was bewusst nicht.
- Reine Doku-Änderungen: `python3 tools/check_doc_links.py` (tote Links,
  siehe [61-A-Werkzeuge.md](61-A-Werkzeuge.md#doku-links-prüfen)).

## Unit- und Integrationstests (`test/`)

Rund 550 Testfunktionen in 43 Dateien, alle **ohne Roboter und ohne laufende
Sim**. Der echte Real-Pfad (`arm_controller`, `loco_client`) läuft dabei
gegen einen Fake-G1 mit Unitree-SDK-Attrappen, GUIs laufen offscreen.

```bash
cd g1pilot
test/run_tests.sh <gruppe> [weitere pytest-Argumente]
```

| Gruppe | Inhalt | Dauer (ca.) |
|---|---|---|
| `schnell` | alles außer den drei langsamen Fake-G1-Dateien | 2 min |
| `sicherheit` | Notaus, Totmann, Walk/Modus, Bodenkontakt, Arm-Controller real, Planer, Handy-Gateway | 10 min |
| `voll` | komplette Suite | 14 min |
| `arm` | arm_controller real, IK-Prozess, Jog-Bezug, Taille, Hüfte, Spiegeln | |
| `planer` | Armplaner, arm_controller real | |
| `loco` | loco_client real, Controller-Gehen, Bodenkontakt, Joy-Mux, Startleiste | |
| `controller` | PS4/Xbox, Gehen, gemeinsamer Zustand, Spiegeln, Hüfte | |
| `gui` | Demo-GUI (Vollbild, Sync, Temperaturen, Abläufe, Hüfte, Real beenden), Fenster-Icons | |
| `haende` | Inspire-Modbus-Backend | |
| `sim` | Lauf-Policy, loco_sim-Zustand, Schubs, Auto-Start | |
| `handy` | Gateway, BLE gegen nachgebautes BlueZ, `proto.js`, Handy-Seite in Chromium | |

Die drei langsamen Dateien sind `test_arm_controller_real.py`,
`test_jog_reference_real.py` und `test_ik_servo_async.py` (zusammen etwa
11 der 14 Minuten).

**Voraussetzungen:** Python mit `pytest`, rclpy (ROS 2 Humble) samt
Nachrichtenpaketen und tf2, Pinocchio (für `com_model` Version 3), PyQt6,
`mujoco`, `onnxruntime`, das Unitree-SDK aus `unitree_sdk2_python` mit
`cyclonedds`, für die Handy-Tests zusätzlich `dbus-next`, Node.js und
Playwright/Chromium. Das Paket muss importierbar sein
(`PYTHONPATH=<repo>/g1pilot:$PYTHONPATH`, vorhandenen Pfad nicht
überschreiben). Die Laufzeit-Images enthalten nicht alles davon (z. B.
kein pytest); bewährt haben sich ein eigenes Testimage auf `ros:humble`
oder eine micromamba-Umgebung (robostack, Python 3.11). GUI-Tests setzen
`QT_QPA_PLATFORM=offscreen` selbst.

`-rs` zeigt übersprungene Tests mit Grund. **Achtung:** Fehlt Pinocchio
oder das Unitree-SDK, überspringen ganze Sicherheitsdateien still — dann
ist der Lauf kein Nachweis.

## Ende-zu-Ende gegen die laufende Sim

Diese Skripte liegen in der Paketwurzel und brauchen einen laufenden
Sim-Stack (`./start.sh` bzw. `make sim`). Sie laufen im Container
`g1pilot-sim`:

```bash
docker exec -it g1pilot-g1pilot-sim-1 bash -c \
  "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/g1pilot/<skript>"
```

| Skript | Prüft | Voraussetzung |
|---|---|---|
| `test_ps4_arm_sim.py` | Controller: Richtung je Stick, Hand-Auswahl, Drehen, Spiegeln, Hände, Abriss, NOT-HALT (simuliertes Joy) | `G1_PS4_ARMS=1` |
| `test_waist_yaw_sim.py` | Hüftdrehung: R2 + Stick, Grenze, R2 + R3 zur Mitte | `G1_PS4_ARMS=1` |
| `test_sim_startup_grasp.py` | GREIFEN direkt nach dem Auto-Start bedienbar | frisch gestarteter Stack |
| `run_startup_grasp_loop.sh [N] [streamdeck\|demo]` | startet den Stack N-mal und ruft jedes Mal `test_sim_startup_grasp.py` | Images gebaut, ggf. `Xvfb` |
| `jog_axis_probe_sim.py [--mirror] [--reference …]` | Messung: Nebenachsen und Kippen beim Jog | `G1_PS4_ARMS=1` |

## Headless-MuJoCo (ohne ROS)

| Skript | Prüft |
|---|---|
| `test_agile_walk_sim.py [--inspire] [--policy NAME]` | Laufen, PD-Stand, Übergaben, Stöße mit `walk_policy` + `stand_balancer` (Ergebnisse in [30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md#testergebnisse-headless-mujoco)) |
| `test_wholebody_policy.py` | Standard-Policy `g1_wholebody`: steht, läuft, kommt zum Stehen |

## Was nicht abgedeckt ist

- Echter G1 (Hardware): erste Läufe nach [50-A-Echter-Roboter.md](50-A-Echter-Roboter.md).
- Echtes Bluetooth (BlueZ + Android-Chrome) für das Handy.
- LED und Vibration echter Controller.

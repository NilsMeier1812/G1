# Demo-GUI — Technik

Richtet sich an Entwickler, die die Demo-GUI weiterbauen. Bedienung:
[10-A-Demo-GUI.md](10-A-Demo-GUI.md).

## Grundsatz

Die Logik bleibt wie beim Streamdeck in den Ziel-Nodes (`arm_controller`,
`loco_sim`, `loco_client`, Hand-Bridge). Die GUI publiziert auf die
bestehenden Topics und liest Zustände, sie rechnet selbst keine Bewegung.
`DemoNode` erbt die Publisher von `StreamDeck` (`ui_interface.py`), damit
beide Oberflächen garantiert dasselbe senden. Es läuft immer **nur eine** der
beiden GUIs (`G1_GUI=demo|streamdeck`), sonst liefe der Sim-Auto-Start doppelt.

## Beteiligte Dateien

| Datei | Inhalt |
|---|---|
| `g1pilot/teleoperation/demo_gui.py` | PyQt6-Oberfläche: `DemoNode` (ROS), `ModeTile`, `TempPanel`, Seiten ARME/HÄNDE, Startleiste, Status, Vollbild |
| `g1pilot/teleoperation/ui_interface.py` | Basisklasse `StreamDeck` (Publisher), `VirtualJoystick`, Dialoge `PoseSaveDialog` / `PoseLoadDialog` |
| `g1pilot/teleoperation/hand_panel.py` | Seite HÄNDE (Finger, Kraft, Kraftzonen) über `/g1pilot/hand_cmd` und `/g1pilot/hand_status` |
| `g1pilot/teleoperation/sequence_runner.py` | Ablauf abfahren (Am Stück / Schritt für Schritt), ohne Qt/ROS testbar, siehe [23-T-Posen-und-Ablaeufe.md](23-T-Posen-und-Ablaeufe.md) |
| `g1pilot/teleoperation/sequence_editor.py` | Fenster „Posen & Abläufe“ |
| `g1pilot/teleoperation/shared_state.py` | gemeinsamer Bedienzustand mit dem Controller (`/g1pilot/teleop_state`) |
| `g1pilot/teleoperation/status_line.py` | Texte für Startleiste, Info-Zeile, Hüfte, Controller-Belegung (ohne Qt/ROS) |
| `g1pilot/teleoperation/sim_autostart.py` | Sim-Auto-Start GREIFEN mit Quittung |
| `g1pilot/teleoperation/hand_jog.py` | Jog-Rechnung »Hand bewegen« (gemeinsam mit dem Controller) |
| `g1pilot/teleoperation/phone/link.py` | Anbindung des Handy-Gateways an die GUI-Methoden, siehe [12-T-Handy.md](12-T-Handy.md) |
| `g1pilot/utils/g1_temperature.py` | Temperaturen aufbereiten (Gruppen, Schwellen, heißester Motor) |
| `g1pilot/utils/push_charge.py` | aufladbarer Schubs (Haltedauer → Kraft), gemeinsam mit `loco_sim` |
| `g1pilot/utils/window_style.py` | dunkles Fenster-Design, Fenster-Icon und `WM_CLASS` |
| `launch/teleoperation_launcher.launch.py` | startet je nach `G1_GUI` **entweder** `ui_interface` **oder** `demo_gui` |
| `setup.py` | Entry-Point `demo_gui` |

## Topics

| Richtung | Topic | Zweck |
|---|---|---|
| sendet | `/g1pilot/start_walking`, `/g1pilot/start_balancing`, `/g1pilot/arms/enabled` | Modus-Kacheln |
| sendet | `/g1pilot/start` | Startleiste »Start« / NOT-HALT quittieren |
| sendet | `/g1pilot/loco_request` | Startleiste Damp / Preparation / Walk (real, `loco_client` prüft erneut) |
| sendet | `/g1pilot/emergency_stop` | NOT-HALT |
| sendet | `/g1pilot/loco_cmd_vel` | GEHEN: Knopf und Pfeile, ~30 Hz, außerhalb von GEHEN 0 |
| sendet | `/g1pilot/auto_enable`, `/g1pilot/goal` | AUTO NAV, Zur Station |
| sendet | `/g1pilot/hand_goal/<side>`, `/g1pilot/arms/home`, `/g1pilot/pose_store/goto`, `/g1pilot/pose_store/cancel`, `/g1pilot/arm_command` | Arme: Jog, Grundstellung, Posen, Abläufe, relative Bewegungen |
| sendet | `/g1pilot/hand_action/<side>`, `/g1pilot/hand_cmd` | Hände |
| sendet | `/g1pilot/waist/center` | Knopf Hüfte Mitte |
| sendet | `/g1pilot/push`, `/g1pilot/push_charge`, `/g1pilot/scene_reset` | nur Sim |
| liest | `/g1pilot/arms/walk_ready`, `/g1pilot/arms/state`, `/g1pilot/loco_state` | Moduswechsel bestätigt, Sim-Auto-Start |
| liest | `/g1pilot/arm_command/status`, `/g1pilot/pose_store/save/status` | Ablauf-Schritte, Speichern quittiert |
| liest | `/g1pilot/onboard_state` | Onboard-Modus des echten G1 (Startleiste, Bodenkontakt) |
| liest | `/g1pilot/temperatures` | Temperatur-Kacheln (real) |
| liest | `/g1pilot/nav_status` | AUTO-NAV-Zustand |
| liest | `/g1pilot/waist/state` | Anzeige Hüfte |
| liest | `/g1pilot/ps4/status`, `/g1pilot/ps4/walk_status`, `/g1pilot/ps4/device` | Controller-Anzeige |
| beides | `/g1pilot/teleop_state` | gemeinsamer Bedienzustand mit dem Controller |
| beides | `/g1pilot/sequence/next`, `/g1pilot/sequence/state` | Ablauf »Weiter« (Controller L3) und Zustand |

## Beenden (Sim/Real)

Die GUI beendet den Stack nicht selbst, sondern legt eine Anforderungsdatei
im gemounteten Repo an, die ein Watcher auf dem Host abarbeitet:

| Datei | Watcher | Wirkung |
|---|---|---|
| `.sim_shutdown_request` | `docker/sim_shutdown_watcher.sh` | `docker compose kill` + `down` (sofort) |
| `.real_shutdown_request` | `docker/real_shutdown_watcher.sh` | `docker compose stop` (SIGTERM) + `down`, erst nachdem die Arme abgegeben sind |

Die Watcher starten `./start.sh`, `make sim`, `make real` und `make real-full`.

## Tests

| Test | Inhalt |
|---|---|
| `test/test_demo_gui_controller_sync.py` | GUI folgt dem Controller (Kacheln, Auswahl, Tempo), Sim und Real |
| `test/test_demo_gui_fullscreen.py` | Vollbild, Mindesthöhe, Scrollbereich |
| `test/test_demo_gui_ground_precheck.py` | Bodenkontakt-Vorabprüfung an »Walk« |
| `test/test_demo_gui_jog_hold.py` | »Hand bewegen«: Halte-Ziel beim Loslassen |
| `test/test_demo_gui_real_quit.py` | »Real beenden«: Reihenfolge, Abbruch durch NOT-HALT |
| `test/test_demo_gui_sequences.py` | Abläufe in der GUI |
| `test/test_demo_gui_temperature.py`, `test/test_g1_temperature.py` | Temperatur-Kacheln und Aufbereitung |
| `test/test_demo_gui_waist.py` | Anzeige Hüfte, Knopf Hüfte Mitte |
| `test/test_gui_external_estop.py` | NOT-HALT von außen (Controller) |
| `test/test_status_line.py` | Texte der Info-Zeile |
| `test/test_sim_autostart.py` | Auto-Start mit Quittung |
| `test/test_push_charge.py` | aufladbarer Schubs |

Alle GUI-Tests laufen offscreen (`QT_QPA_PLATFORM=offscreen`), siehe
[60-A-Tests.md](60-A-Tests.md).

## Offene Punkte / nächste Schritte

1. **Echter Zustand statt angenommener Zustand:** `loco_sim` meldet seinen
   Zustand inzwischen auf `/g1pilot/loco_state` (für den Auto-Start), real
   gibt es `/g1pilot/onboard_state`. Die Modus-Kacheln selbst werten das noch
   nicht aus, etwa wenn die Sim nach einem Sturz auf DAMP geht.
2. **Pose-Status-Zuordnung:** `pose_store/goto` hat keine Request-ID. Die GUI
   nimmt deshalb den nächsten Endzustand nach dem `accepted` als „ihren“. Mit
   einer ID wie bei der Arm-API wäre das eindeutig.
3. **Sprache:** Labels sind Deutsch. Ein EN-Umschalter wäre für Messen sinnvoll.
4. **Bilder statt Text** auf den Bewegungs-Knöpfen, z. B. ein Vorschaubild je
   Pose im Pose-Store.

Erledigt seit dem Konzept: Vollbild (F11, `G1_GUI_FULLSCREEN`), feste
Navigationsziele (»Zur Station«), Handy-Bedienung, Temperaturanzeige.

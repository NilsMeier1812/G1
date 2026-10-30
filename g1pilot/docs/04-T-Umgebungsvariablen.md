# Umgebungsvariablen — Technik

Richtet sich an alle, die ohne Startmenü starten oder ein Verhalten
umstellen wollen. Alle Variablen, die der Stack auswertet, mit Default und
Verweis aufs Thema. Das Startmenü setzt die meisten davon selbst
([03-A-Startmenue.md](03-A-Startmenue.md)).

Gesetzt werden sie auf dem Host vor `./start.sh --yes`, `make …` oder
`docker compose …`. `docker-compose.yml` reicht sie an die Container weiter;
die Launch-Dateien machen daraus ROS-Parameter. Eine Änderung braucht einen
Neustart des Stacks, aber keinen Image-Neubau.

```bash
G1_ESTOP_DAMP_LEGS=1 G1_MAX_VX=0.3 make real ROBOT_INTERFACE=enp3s0
```

## Start und Oberfläche

| Variable | Default | Wirkung | Doku |
|---|---|---|---|
| `G1_MODE` | — | `sim` oder `real` für `start.sh --yes` | [03-A](03-A-Startmenue.md) |
| `G1_REAL_CONFIRM` | `0` | `1` = Sicherheitshinweise bestätigt; ohne startet REAL mit `--yes` nicht | [03-A](03-A-Startmenue.md) |
| `G1_NO_GUI` | `0` | `1` = Text-Menü statt Startmenü | [03-A](03-A-Startmenue.md) |
| `G1_GUI_FULLSCREEN` | `0` | Demo-GUI im Vollbild starten | [10-A](10-A-Demo-GUI.md) |
| `G1_SIM_MODE` | `false` | `true` = Sim-Pfad (setzen `start.sh`/`make sim`) | [02-T](02-T-Architektur.md) |
| `ROBOT_INTERFACE` | `eth0` | Netzwerkkarte zum echten G1 | [50-A](50-A-Echter-Roboter.md) |
| `USE_RVIZ` | Sim `false`, Real `true` | RViz mitstarten | [03-A](03-A-Startmenue.md) |
| `G1_ICON_DIR` | — | anderer Ordner für Fenster-Icons | `resource/icons/README.md` |

## Simulation

| Variable | Default | Wirkung | Doku |
|---|---|---|---|
| `G1_ENV` | — (Standard-Szene) | Umgebung aus `scene_editor/scenes/` | [40-A](40-A-Simulation.md#umgebungen) |
| `G1_WALK_POLICY` | `g1_wholebody` | Lauf-Policy der Sim | [30-A](30-A-Stehen-und-Gehen.md#lauf-regler-wählen-nur-sim) |
| `G1_LOCO_DEBUG` | `0` | ausführliches Log von `loco_sim` | [30-T](30-T-Stehen-und-Gehen.md) |
| `G1_PUSH_MAX_N`, `G1_PUSH_CHARGE_S`, `G1_PUSH_TAP_S` | `2000`, `3.0`, `0.3` | aufladbarer Schubs | [40-T](40-T-Simulation.md#zusatzbefehle-udp-über-loopback) |
| `G1_GRASP_TEST`, `G1_GRASP_BOX` | aus, an bei Inspire | Testkugel in der Hand | [40-T](40-T-Simulation.md#zusatzbefehle-udp-über-loopback) |
| `G1_SIM_APPARMOR` | `docker-default` | `unconfined` für Bluetooth im Sim-Container (setzt das Startmenü mit Handy-Zugang) | [12-T](12-T-Handy.md) |
| `G1_NV_PRIME_OFFLOAD` | automatisch | `1` = NVIDIA-Rendern auf Hybrid-Laptops; bei Bildschirm direkt an der NVIDIA `0` (sonst schwarzes Fenster) | [01-A](01-A-Installation.md) |
| `G1_SKIP_GPU_CHECK` | `0` | Warnung der GPU-Prüfung (`docker/check_gpu.sh`) abschalten | [61-A](61-A-Werkzeuge.md) |
| `HOLD_BASE_MODE`, `SIM_LOCKSTEP`, `SIM_REALTIME_FACTOR`, `SIM_*` | siehe 40-T | Physik und Ports der Sim | [40-T](40-T-Simulation.md#umgebungsvariablen-der-sim) |

## Hände

| Variable | Default | Wirkung | Doku |
|---|---|---|---|
| `G1_INSPIRE_HANDS` | `0` (Startmenü: an) | Inspire-Hände (Sim-Modell bzw. Modbus real) | [22-A](22-A-Inspire-Haende.md) |
| `G1_HAND_LEFT_HOST`, `G1_HAND_RIGHT_HOST` | `192.168.123.210`, `.211` | IP der Hände | [22-A](22-A-Inspire-Haende.md) |
| `G1_HAND_PORT` | `6000` | Modbus-TCP-Port | [22-T](22-T-Inspire-Haende.md) |

## Arme (echter Roboter)

| Variable | Default | Wirkung | Doku |
|---|---|---|---|
| `G1_ARM_VEL_LIMIT` | `1.5` rad/s | Gelenk-Tempo | [20-T](20-T-Arm-Manipulation.md) |
| `G1_ARM_ACCEL_LIMIT` | `3.0` rad/s² | Gelenk-Beschleunigung | [20-T](20-T-Arm-Manipulation.md) |
| `G1_EE_VEL_LIMIT` | `0.25` m/s | Hand-Tempo | [20-T](20-T-Arm-Manipulation.md) |
| `G1_ARM_MAX_TRACKING_ERROR` | `0.3` rad | Abbruch bei zu großem Schleppfehler | [20-T](20-T-Arm-Manipulation.md) |
| `G1_IK_ASYNC` | `1` | IK in eigenem Prozess (`0` = alt, im Haupttakt) | [20-T](20-T-Arm-Manipulation.md) |
| `G1_JOG_REFERENCE` | `command` | Bezug der Jogs: `command`, `start`, `tf` | [11-T](11-T-Controller.md) |
| `G1_WALK_PARK_ARMS` | `false` | Arme beim Gehen in die Lauf-Pose parken | [30-A](30-A-Stehen-und-Gehen.md) |
| `G1_WAIST_HOLD` | `true` | Taille per `arm_sdk` halten | [24-T](24-T-Hueftdrehung.md) |
| `G1_WAIST_KP`, `G1_WAIST_KD` | `150.0`, `4.0` | Gains der Taille | [24-T](24-T-Hueftdrehung.md) |
| `G1_WAIST_YAW` | `1` | Hüftdrehung (`0` = aus) | [24-A](24-A-Hueftdrehung.md) |
| `G1_WAIST_YAW_LIMIT_DEG` | `105` | Bereich der Hüftdrehung um die Mitte | [24-T](24-T-Hueftdrehung.md) |

## Gehen (echter Roboter)

| Variable | Default | Wirkung | Doku |
|---|---|---|---|
| `G1_MAX_VX`, `G1_MAX_VY`, `G1_MAX_VYAW` | `0.4`, `0.3`, `0.4` | Geh-Limits (m/s, rad/s) | [30-A](30-A-Stehen-und-Gehen.md) |
| `G1_WALK_FSM_ID` | `501` | Walk-Modus der Firmware (501 = mit Taille) | [30-T](30-T-Stehen-und-Gehen.md) |
| `G1_LOCO_SERVICE` | `sport` | RPC-Dienst (`loco` bis ai_sport 8.1) | [50-A](50-A-Echter-Roboter.md) |
| `G1_GROUND_CHECK` | `knee_load` | Bodenkontakt vor Walk: `knee_load`, `fsm_mode`, `none` | [30-T](30-T-Stehen-und-Gehen.md) |
| `G1_KNEE_LOAD_MIN_NM` | `10.0` | Mindest-Kniemoment für »steht am Boden« | [30-T](30-T-Stehen-und-Gehen.md) |
| `G1_ENABLE_NAV` | `0` | Navigation mitstarten (nur Sim) | [31-A](31-A-Navigation.md) |

## Notaus

| Variable | Default | Wirkung | Doku |
|---|---|---|---|
| `G1_ESTOP_DAMP_LEGS` | `0` | NOT-HALT dämpft auch die Beine | [51-A](51-A-Notaus.md) |
| `G1_ESTOP_WAIST_HOLD` | `true` | Taille beim NOT-HALT halten | [51-A](51-A-Notaus.md) |

## Controller

| Variable | Default | Wirkung | Doku |
|---|---|---|---|
| `G1_PS4_ARMS` | `0` (Startmenü: an) | Controller für Arme und Hände (sonst alter Joystick-Modus) | [11-A](11-A-Controller.md) |
| `G1_CONTROLLER` | `auto` | `ps4`, `xbox` oder automatisch | [11-A](11-A-Controller.md) |
| `G1_XBOX_PS_BUTTON` | `xbox` | NOT-HALT-Taste am Xbox-Controller (`view` als Ausweich) | [11-A](11-A-Controller.md) |
| `G1_PS4_PS_ESTOP` | `1` | PS = NOT-HALT (`0`: nur Stopp) | [51-A](51-A-Notaus.md) |
| `G1_PS4_SPEED`, `G1_PS4_MAX_SPEED` | `Langsam`, `Normal` | Arm-Tempo beim Start, höchste Stufe | [11-A](11-A-Controller.md) |
| `G1_PS4_WALK` | `1` | Gehen mit L2 | [11-A](11-A-Controller.md) |
| `G1_PS4_WALK_SPEED`, `G1_PS4_WALK_MAX_SPEED` | `Langsam`, `Schnell` | Geh-Tempo beim Start, höchste Stufe | [11-A](11-A-Controller.md) |
| `JOYSTICK_NAME` | `Wireless Controller` | Gerätename im alten Joystick-Modus | [13-A](13-A-Alter-Joystick.md) |

## Handy

| Variable | Default | Wirkung | Doku |
|---|---|---|---|
| `G1_PHONE` | `0` | Handy-Zugang (nur mit Demo-GUI) | [12-A](12-A-Handy.md) |
| `G1_PHONE_BT_ADAPTER` | `auto` | Bluetooth-Adapter | [12-A](12-A-Handy.md) |
| `G1_PHONE_BT_NAME` | `G1 SIM` / `G1 REAL` | angezeigter Name | [12-A](12-A-Handy.md) |
| `G1_PHONE_BT_ENCRYPT` | `0` | BLE-Kopplung verlangen | [12-A](12-A-Handy.md) |
| `G1_PHONE_VIEW_ESTOP` | `1` | Ansicht-Handys dürfen NOT-HALT | [12-A](12-A-Handy.md) |
| `G1_PHONE_STORE` | `~/.config/g1pilot/phone_devices.json` | Datei der gekoppelten Handys | [12-T](12-T-Handy.md) |

## Posen, Abläufe, Anzeige

| Variable | Default | Wirkung | Doku |
|---|---|---|---|
| `G1_POSE_STORE` | Repo-Mount | Ablageort der Posen-Datei | [23-T](23-T-Posen-und-Ablaeufe.md#speicherort) |
| `G1_SEQUENCE_MODE` | Sim `auto`, Real `step` | Ablauf-Modus beim Start: am Stück oder Schritt für Schritt | [23-A](23-A-Posen-und-Ablaeufe.md) |
| `G1_DEMO_CATEGORY` | `Demo` | vorgeschlagene Kategorie beim Pose speichern (auch `demo_sequence.py`) | [10-A](10-A-Demo-GUI.md) |
| `G1_SAFE_POSE` | `Sichere_Pose` | gespeicherte Pose hinter dem Knopf »Sichere Pose« | [10-A](10-A-Demo-GUI.md) |
| `G1_TEMP_WARN`, `G1_TEMP_CRIT` | `60`, `75` °C | Schwellen der Temperaturanzeige | [10-A](10-A-Demo-GUI.md) |

Nur in Tests bzw. Testskripten benutzt (nicht für den Betrieb): Namen wie
`G1_CHROMIUM`, `G1_PHONE_SHOTS`, `G1_TEST_DIR` ([60-A-Tests.md](60-A-Tests.md)).

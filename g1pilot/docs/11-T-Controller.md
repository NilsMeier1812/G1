# Controller (PS4/Xbox) — Technik

Richtet sich an Entwickler. Bedienung und Belegung: [11-A-Controller.md](11-A-Controller.md).

## Dateien

| Datei | Inhalt |
|---|---|
| `g1pilot/teleoperation/ps4_joystick.py` | Node `ps4_joystick`: evdev → `/g1pilot/ps4/joy` in fester Belegung, PS4/Xbox erkennen, Wiederverbinden, LED (sysfs) + Vibration (Force-Feedback) aus `/g1pilot/ps4/feedback`, Lebendprüfung (PS4: Sensor-Teilgerät, Xbox: HID-Rohstrom) |
| `g1pilot/teleoperation/ps4_layout.py` | feste Belegung (Indizes) und evdev-Code-Zuordnung, PS4 (`Ps4State`) und Xbox (`XboxState`, drei Treiber-Varianten), ohne ROS/evdev testbar |
| `g1pilot/teleoperation/ps4_arm_teleop.py` | `ArmTeleopLogic` (reine Bedienlogik) + Node `ps4_arm_teleop`; `walk_deadman`: pausiert bei gehaltenem L2; `trigger_modes`: L2/R2 = GEHEN/GREIFEN |
| `g1pilot/teleoperation/ps4_walk.py` | `WalkLogic` (reine Bedienlogik) + Node `ps4_walk`: L2-Deadman → `/g1pilot/ps4/walk_cmd` (Twist, normiert), `/g1pilot/ps4/walk_status` |
| `navigation/loco_client.py` | `/g1pilot/ps4/walk_cmd` → `Move()` nur im Walk-Modus, ohne E-Stop; Timeout → StopMove; Vorrang vor `loco_cmd_vel` |
| `navigation/loco_sim.py` | Sim: `/g1pilot/ps4/walk_cmd` wie `loco_cmd_vel`, mit Vorrang (die GUIs senden dort dauernd 0) |
| `g1pilot/teleoperation/hand_jog.py` | Jog-Rechnung (Ziel = Hand + Geschwindigkeit, Vorlauf entlang der Bewegung begrenzt), Halte-Ziel (`hold_pose`, `HoldStreamer`), gemeinsam mit der Demo-GUI |
| `launch/teleoperation_launcher.launch.py` | Argument `ps4_arms` (Default `false`): startet `ps4_joystick` + `ps4_arm_teleop` statt `joystick`; `ps4_speed`, `ps4_max_speed`, `ps4_options_balancing`, `ps4_ps_estop`; `ps4_walk` (Default `true`) startet zusätzlich `ps4_walk`, `ps4_walk_speed`, `ps4_walk_max_speed` |
| `launch/bringup_sim.launch.py` | setzt `ps4_arms` aus `G1_PS4_ARMS` (Sim-Defaults) |
| `launch/bringup_real.launch.py` | setzt `ps4_arms` aus `G1_PS4_ARMS` mit Real-Werten (Langsam/max Normal, R2 ohne `start_balancing`) |
| `teleoperation/demo_gui.py`, `teleoperation/ui_interface.py` | zeigen einen NOT-HALT von außen (PS-Taste) an; Demo-GUI folgt dem gemeinsamen Zustand (Kacheln, Hand-Auswahl, Tempo) |
| `g1pilot/teleoperation/shared_state.py` | gemeinsamer Bedienzustand GUI ↔ Controller (`/g1pilot/teleop_state`, latched JSON, Zeitstempel je Feld) |
| `g1pilot/teleoperation/status_line.py` | Texte der Startleiste (Onboard-Modus) und Controller-Zustand (Typ, Belegung) |
| `docker-compose.ps4.yml` | `/dev/input` + `device_cgroup_rules: c 13:* rmw` für `g1pilot-sim` |
| `test/test_ps4_arm_teleop.py`, `test/test_ps4_joystick.py` | Unit-Tests (Belegung, Logik, Jog, simuliertes evdev-Gerät inkl. hängender Funkverbindung; Xbox per USB/Bluetooth, Auswahl, Consumer-Control, HID-Lebendprüfung) |
| `test/test_xbox_layout.py` | Xbox ohne ROS: Erkennung, drei Treiber-Varianten, Zuordnung nach Position, View-Ersatz, Gehen mit LT, NOT-HALT, Voll-Notaus RB + LB + Xbox |
| `test/test_ps4_walk.py`, `test/test_loco_client_ps4_walk.py`, `test/test_ps4_walk_node.py` | Gehen: Deadman, Tempo, NOT-HALT, Abriss, Arme pausieren; loco_client nur im Walk-Modus, Timeout, Vorrang; Node mit simuliertem Joy |
| `test/test_ps4_arm_teleop_node.py` | Node mit Real-Einstellungen gegen simuliertes Joy + TF (R2 = Arme, Options-Sperre, Tempo, NOT-HALT von außen, kein Laufbefehl) |
| `test/test_gui_external_estop.py` | Demo-GUI + Streamdeck folgen einem NOT-HALT von außen (Sim und Real, offscreen) |
| `test/test_demo_gui_jog_hold.py` | Demo-GUI »Hand bewegen«: Halte-Ziel beim Loslassen (auf der Linie, 1 s wiederholt, Grundstellung löscht es) |
| `test/test_shared_state.py`, `test/test_demo_gui_controller_sync.py` | gemeinsamer Zustand: neuester gewinnt, Echo/alte Stände ignoriert; Demo-GUI folgt Controller (Kacheln, Auswahl, Tempo) ohne etwas zu senden, Sim und Real offscreen |
| `test_ps4_arm_sim.py` | Ende-zu-Ende-Test gegen die laufende Sim (simulierte Joy-Eingabe) |

## Datenfluss

```
DualShock 4 ──evdev──> ps4_joystick ──/g1pilot/ps4/joy (Joy, feste Belegung)──> ps4_arm_teleop
                            ^                                                        │
                            └──── /g1pilot/ps4/feedback (LED, Vibration) ────────────┤
                                                                                     ├─> /g1pilot/hand_goal/<side>   (PoseStamped, pelvis)
                                                                                     ├─> /g1pilot/hand_action/<side> ("open"/"close")
                                                                                     ├─> /g1pilot/arms/enabled (+ /g1pilot/start_balancing nur Sim) (R2)
                                                                                     ├─> /g1pilot/arms/home (Share, Impuls)
                                                                                     ├─> /g1pilot/pose_store/cancel (Kreuz)
                                                                                     ├─<> /g1pilot/arm_command (+ /status) (Steuerkreuz + Kreuz 2 s: Arm spiegeln)
                                                                                     ├─> /g1pilot/emergency_stop (PS)
                                                                                     ├─> /g1pilot/ps4/status (JSON, latched)
                                                                                     ├─<>  /g1pilot/teleop_state (JSON, latched; mit Demo-GUI + ps4_walk)
                                                                                     └─> /g1pilot/ps4/markers (RViz)
                       /g1pilot/ps4/joy ──> ps4_walk (L2 gehalten) ──/g1pilot/ps4/walk_cmd──> loco_client (real, Move) / loco_sim (Sim)
```

Bewusst **nicht** über `joy_mux`/`/g1pilot/joy`: dort hängen `joy_to_cmdvel`
(Sim, mit Navigation) und `loco_client` (real) — die Sticks würden sonst den
Roboter auch beim Bewegen der Hände laufen lassen. Gehen hat ein eigenes
Topic, das nur bei gehaltenem L2 gesendet wird. Der alte `joystick`-Node startet im PS4-Modus nicht,
weil er denselben Controller lesen würde.

Eingänge außer dem Joy: `/g1pilot/emergency_stop` (NOT-HALT aus der GUI →
Controller sperren), `/g1pilot/start` (Quittung → freigeben) und
`/g1pilot/teleop_state` (Auswahl/Tempo/Modus aus der Demo-GUI).
`ps4_joystick` meldet zusätzlich `/g1pilot/ps4/device` (latched,
`{"kind": "ps4"|"xbox"|null, "name": …}`) für die Anzeige.

## Feste Belegung (`/g1pilot/ps4/joy`)

Achsen wie beim ROS-Paket `joy`: links = +1, oben = +1; Trigger 0…1.

| Index | Achse | Index | Taste |
|---|---|---|---|
| 0 | linker Stick X | 0–3 | Kreuz, Kreis, Dreieck, Viereck |
| 1 | linker Stick Y | 4–7 | L1, R1, L2, R2 |
| 2 | rechter Stick X | 8–10 | Share, Options, PS |
| 3 | rechter Stick Y | 11–12 | L3, R3 |
| 4–5 | L2, R2 (0…1) | 13–16 | Steuerkreuz oben, unten, links, rechts |
| 6–7 | Steuerkreuz X, Y | | |

Die evdev-Codes sind die des Linux-Treibers (hid-sony ab Kernel 4.10,
hid-playstation). Der Xbox-Controller landet nach Position in derselben
Belegung (A = Index 0 usw., Xbox-Taste = 10, LT/RT = Achsen 4/5);
`header.frame_id` ist `ps4` bzw. `xbox`. Der alte `joystick`-Node nummeriert dagegen in
Geräte-Reihenfolge — für eine Tastenbelegung zu wackelig.

## Sicherheit

- Totzone 0.12 (mit Neuskalierung) + Expo 0.4 → kein Kriechen, feinfühlig um die Mitte.
- Kein Joy länger als 0.5 s (`joy_timeout_s`) → Hand hält an. `ps4_joystick`
  publiziert ohne Verbindung **nichts** (kein eingefrorener Wert).
- Hängende Funkverbindung (Bluetooth friert ein, der Kernel meldet die
  Trennung erst nach Sekunden): `ps4_joystick` überwacht das Teilgerät
  „… Motion Sensors“ desselben Controllers (gleiche phys/uniq), das mit
  jedem Funkbericht Ereignisse liefert. Kommen länger als 0,3 s
  (`stall_timeout_s`) keine Daten, publiziert es nichts mehr → nach
  `joy_timeout_s` hält die Hand. Ohne zuordenbares Sensor-Teilgerät gibt es
  keine Lebendprüfung (Log-Hinweis beim Verbinden).
- TF-Lücke beim Loslassen: Halte-Ziel aus der Hand-Pose des letzten Takts
  (statt nichts zu senden und den Vorlauf abfahren zu lassen).
- Abgewählte Hand hält sofort an (Ziel = echte Hand).
- Loslassen / Abwählen / Abriss: Halte-Ziel (Hand stoppt entlang der
  Bewegungsrichtung) wird 1 s lang wiederholt (`JOG_HOLD_S`), sonst führe der
  arm_controller wegen seiner Ziel-Glättung noch den Vorlauf ab.
- Grundstellung nur nach 1 s Halten von Share.
- NOT-HALT (PS oder GUI) sperrt den Controller bis zur Quittung (`/g1pilot/start`).
- Real: R2 ohne Walk-Modus ohne `start_balancing`, Tempo höchstens Normal.
- Options sperrt den Controller (Ablegen); PS-NOT-HALT und Voll-Notaus wirken auch gesperrt.
- Alle Ziele laufen durch den arm_controller (IK, Kollisions-Gate, EE-Tempo-Limit).

## Reaktionszeit

Kette: Stick → `ps4_arm_teleop` (30 Hz) → `/g1pilot/hand_goal/<side>` →
arm_controller (Ziel-Filter pro Nachricht, IK, Ziel-/Gelenk-Glättung pro
Regeltakt, Tempo-Limits, Kollisions-Gate) → `rt/arm_sdk`. Das Ziel läuft
der Hand höchstens `max(4 cm, Tempo × 0,5 s)` voraus — begrenzt **entlang**
der Bewegungsrichtung; quer dazu bleibt es auf seiner Linie, damit eine
Querabweichung der Hand (IK-Restfehler) nicht ins Ziel übernommen wird.

Behoben (2026-10-13), nachdem die Hand in der Sim erst 1–2 s nach dem Stick
losfuhr und nach dem Loslassen 2–3 cm / 1,5–1,8 s weiterlief:

| Ursache | Wirkt | Änderung |
|---|---|---|
| `main_loop` las LowState zusätzlich **blockierend** (wartete auf die nächste Nachricht) | Sim + real (real ~2 ms/Takt) | entfernt, der LowState-Thread reicht |
| Ziel-Queue Tiefe 10, bedient einmal pro Takt → bis 0,8 s alte Ziele | Sim, real nur bei Takt < 30 Hz | Tiefe 1 (neuestes Ziel) |
| Halte-Ziel beim Loslassen nur **einmal** gesendet, Ziel-Filter (0,25 pro Nachricht) zog nur 25 % zurück | Sim + real | Halte-Ziel 1 s wiederholen |
| Glättung pro Takt, Sim-Takt nur 10–15 Hz | nur Sim | `smoothing_ref_hz` = 50 (zeitbasiert), real aus |
| Ziel übernahm die Querdrift der Hand (Vorlauf-Kugel um die Hand) | Sim + real | Vorlauf nur entlang der Bewegung |

Gemessen in der Sim (Cloud-PC, 4 Kerne, Last ~10, Regeltakt 12–16 Hz),
Stick 2 s voll: Hand > 5 mm nach **0,4–0,7 s** (vorher 1,2–2,1 s);
Nachlauf bei Langsam 0–1,5 cm, bei Normal ~3 cm bei 11 cm/s (≈ 0,3 s
Fahrt); steht 0–1 s nach dem Loslassen (vorher 1,5–1,8 s). Real
(30–50 Hz Regeltakt) wird es schneller sein; offline (IK allein, 40 Hz)
fährt das Kommando nach ~0,2 s 5 mm.

## Nebenachsen halten (2026-10-14)

Beim Verfahren kippte die Hand nach oben und wanderte quer. Ursache: jedes
neue Jog-Ziel startete an der **gemessenen** Hand (TF). Gemessen und
kommandiert unterscheiden sich um einen bleibenden Regelfehler (Sim ~3°
Kippen + ~1 cm); der Arm übernahm ihn bei jeder Fahrt erneut (Ratsche). Die
IK selbst hält die Orientierung (offline < 1°).

Jetzt startet ein neues Ziel an der **kommandierten** Hand: der arm_controller
sendet sie auf `/g1pilot/hand_cmd_pose/<side>` (FK der Soll-Gelenke, mit
EE-Offsets, ≤ 30 Hz, erst nach der Auto-Kalibrierung der Seite). Vorlauf und
Anhalten bleiben an der gemessenen Hand (Vorlauf relativ zur kommandierten
Hand machte die Fahrt unruhiger und lief nach). Fehlt das Topic, gilt TF wie
bisher. Parameter `jog_reference`: `command` (Sim-Default) | `tf`
(Real-Default, bis zum ersten Test am Roboter), live umschaltbar; die
Demo-GUI folgt dem Sim-/Real-Modus.

Sim (ohne Inspire-Hände, 5 Startposen × 6 Richtungen, je 10 cm, Normal),
vorher → nachher: Kippen nach der Fahrt Median 3,1° → 0,6°, Querversatz
0,9 → 0,4 cm; 8× Tippen vor/zurück: Kippen 15° → 0,4°, Rückkehrfehler
7,4 → 4,5 cm. Grenzen: Richtung Körpermitte und am Rand der Reichweite
kippt die Hand in beiden Varianten während der Fahrt kurz bis ~10°
(Gelenkgrenzen/Kollisions-Gate). Messskript: `jog_axis_probe_sim.py`.

## Präzise Achsen (2026-10-21)

Rückmeldung Nils: am Controller kommt fast nie die gewollte Bewegung raus
(hoch/runter dreht die Hand mit, nach innen drehen sich die Hände,
gespiegelte Fahrten aus derselben Lage enden verschieden), der RViz-Marker
fährt dagegen sauber in einer Achse.

Befund:

- Die IK-Kette selbst (Ziel-Filter, Servo-IK, Tempo-Grenzen) ist sauber:
  offline (nur Kinematik) hält sie bei Fahrten in jeder Achse die Orientierung
  auf < 0,1° und die Nebenachsen auf < 0,3 cm, links und rechts exakt
  gespiegelt.
- Der Unterschied ist der **Bezug**. Der Marker gibt ein Ziel vor, der
  arm_controller bringt die *kommandierte* Hand genau dorthin. Der Jog bezog
  sich real (`jog_reference=tf`) auf die *gemessene* Hand: jede Fahrt
  startete dort, Vorlauf und Anhalten hingen daran. Durchhang und Reibung der
  echten Arme wurden so bei jeder Fahrt als neue Soll-Lage übernommen (die
  Hand kippt Fahrt für Fahrt weiter), und das Fahrtende hing am
  Schleppfehler des jeweiligen Arms (links ≠ rechts). In der Sim war seit
  2026-10-14 nur der Start umgestellt.
- Ein Daumen drückt den Stick nie exakt gerade: 10–15° schräg gaben trotz
  Totzone 3–5 % des Tempos in der Nebenachse.

Änderung:

- `jog_reference=command` (Default Sim **und Real**, Env `G1_JOG_REFERENCE`):
  Start, Vorlauf und Anhalten an der kommandierten Hand
  (`/g1pilot/hand_cmd_pose`), wie beim Marker. Die Quelle wird je Fahrt
  festgelegt; fehlt die kommandierte Pose (vor der ersten Auto-Kalibrierung,
  arm_controller aus), läuft die Fahrt an TF. `start` = Stand 2026-10-14,
  `tf` = alter Real-Stand (Rückfall ohne Neubau: `G1_JOG_REFERENCE=tf` bzw.
  `ros2 param set /ps4_arm_teleop jog_reference tf`). Die Demo-GUI
  (»Hand bewegen«) nutzt dieselbe Logik.
- Linker Stick rastet auf die Achse ein (`axis_snap_deg` = 20): bis 20°
  daneben zählt nur die Hauptrichtung, VERSCHIEBEN wie DREHEN. Echte
  Diagonalen gehen weiter.
- Folge: nach dem Loslassen läuft die echte Hand um ihren Schleppfehler
  nach (sie fährt dahin, wo der Sollwert schon ist), statt dort zu stehen,
  wo sie gerade war. Real ist das bei den kleinen Real-Gains nur wenige mm.

Messungen:

| Prüfung | `tf` (alter Real-Stand) | `start` (alter Sim-Stand) | `command` (neu) |
|---|---|---|---|
| Real-Pfad ohne Hardware¹: 8× tippen hoch/runter, Kippen / Rückkehrfehler | 49° / 28 cm | 2,7° / 0,1 cm | 0,9° / 0,3 cm |
| Real-Pfad¹: 5 Fahrten gespiegelt, Abweichung vom Spiegelbild (gemessen) / Drehung | 8,0 cm / 46° | 3,8 cm / 3,5° | 0,7 cm / 3,1° (kommandiert 0,06 cm / < 1°) |
| MuJoCo-Sim², 18 Fahrten: Kippen nach der Fahrt (Median) / Querversatz | 3,0° / 1,0 cm | 0,5° / 0,1 cm | 1,0° / 0,2 cm |
| MuJoCo-Sim²: 8× tippen vor/zurück, Kippen / Rückkehrfehler (Median) | 11,7° / 5,8 cm | 0,2° / 2,4 cm | 0,4° / 2,2 cm |
| MuJoCo-Sim²: gespiegelt, 14 Fahrten, Abweichung vom Spiegelbild / Drehung (Median) | 1,2 cm / 3,5° | – | 0,2 cm / 0,6° |

¹ Echter arm_controller (Real-Modus) gegen den Fake-G1 aus
`test_arm_controller_real` mit bleibendem Regelfehler (Durchhang bis
0,09 rad je Gelenk) und Haftreibung, links stärker als rechts
(`test/test_jog_reference_real.py`). Modell, nicht der echte Roboter.
² Ohne Inspire-Hände, Cloud-PC mit 4 Kernen unter Volllast (Load 12–16):
einzelne Fahrten kamen in 1 s kaum vom Fleck, Ausreißer bis 9 cm quer in
beiden Varianten; die Mediane sind belastbarer als die Maxima. Messskript
`jog_axis_probe_sim.py [--mirror] [--reference command|start|tf]`.

In der Sim liegen `start` und `command` gleichauf (dort sind gemessene und
kommandierte Hand fast deckungsgleich); entscheidend ist der echte Roboter
mit Reibung, dort trennt erst `command` die beiden Arme sauber.

Grenze: in der **Grundstellung** (Arme hängen neben dem Körper) steht die
Hand bei 97 % der Reichweite, genau an der Reichweiten-Klemme der IK.
»Runter« und »außen« sind dort kaum möglich, die Hand weicht entlang der
Kugel um die Schulter aus (offline bis 1,8 cm quer). Erst ein Stück nach
vorn/oben fahren, dann fein positionieren. Das gilt für den Marker genauso.

## Parameter `ps4_arm_teleop`

| Parameter | Default | Bedeutung |
|---|---|---|
| `joy_topic` | `/g1pilot/ps4/joy` | Eingang |
| `rate_hz` | 30 | Takt der Hand-Ziele |
| `deadzone` / `expo` | 0.12 / 0.4 | Stick-Kennlinie |
| `speed` | `Normal` (Real `Langsam`) | Start-Tempo |
| `max_speed` | `Schnell` (Real `Normal`) | höchstes Tempo per R1 |
| `ps_estop` | `true` | PS = NOT-HALT; `false` = PS wirkt wie Kreuz |
| `home_hold_s` | 1.0 | Share-Haltezeit für Grundstellung |
| `full_estop_hold_s` | 2.0 | Haltezeit R1 + L1 + PS für den Voll-Notaus; `0` = aus |
| `mirror_hold_s` | 2.0 | Haltezeit Steuerkreuz ←/→ + Kreuz für Arm spiegeln; `0` = aus |
| `walk_deadman` | `false` (Launch: `ps4_walk`) | gehaltenes L2 pausiert die Arm-Steuerung |
| `trigger_modes` | `false` (Launch: `ps4_walk`) | L2 drücken = Modus GEHEN (Arme gesperrt), R2 = GREIFEN |
| `modes_need_onboard_walk` | Sim `false`, Real `true` | start_walking/start_balancing nur, wenn der G1 laut `/g1pilot/onboard_state` im Walk steht |
| `joy_timeout_s` | 0.5 | Abriss-Erkennung |
| `options_starts_balancing` | `true` (Real `false`) | R2 „Arme übernehmen“ (ohne Moduswechsel, z. B. nach NOT-HALT) sendet zusätzlich `start_balancing` (Name aus der Zeit, als Options das machte) |
| `jog_reference` | `command` (Env `G1_JOG_REFERENCE`) | Bezug der Jog-Ziele: `command` = Start, Vorlauf und Anhalten an der kommandierten Hand (`/g1pilot/hand_cmd_pose`, wie der RViz-Marker), `start` = nur der Start dort (Stand 2026-10-14), `tf` = alles an der gemessenen Hand (alter Real-Stand) |
| `axis_snap_deg` | 20 | Achsen-Einrasten des linken Sticks; 0 = aus |
| `waist_yaw` | `true` | R2 halten + rechter Stick = Hüfte drehen, R2 + R3 = Mitte (nur mit `trigger_modes`) |
| `jog_torso_frame` | `true` | Jog-Richtungen um den gemessenen Taillen-Yaw drehen (Oberkörper-Bezug) |

## Hüftdrehung

`ps4_arm_teleop` sendet bei gehaltenem R2 `/g1pilot/waist/yaw_rate` und
`/g1pilot/waist/center`; den Sollwert führt der arm_controller. Topics,
Parameter und Sim/Real-Unterschiede: [24-T-Hueftdrehung.md](24-T-Hueftdrehung.md).

## Tests

```bash
# Unit-Tests (ohne Controller/Roboter; die ROS-/GUI-Tests brauchen rclpy/PyQt6 -> im Container)
python3 -m pytest -q test/test_ps4_arm_teleop.py test/test_ps4_joystick.py test/test_xbox_layout.py \
    test/test_ps4_arm_teleop_node.py test/test_gui_external_estop.py \
    test/test_demo_gui_jog_hold.py test/test_arm_mirror.py

# Ende-zu-Ende gegen die laufende Sim (G1_PS4_ARMS=1, Controller nicht nötig)
docker exec -it g1pilot-g1pilot-sim-1 bash -c \
  "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/g1pilot/test_ps4_arm_sim.py"
```

Der Sim-Test stellt das Tempo auf Normal, fährt jede Prüfung aus derselben
Ausgangslage (Grundstellung, dann beide Hände schräg nach vorn-oben, mit
Abstand zur Reichweitengrenze) und misst per TF: Richtung je Stick, nur die
gewählte Hand bewegt sich, Drehen ohne Versatz, Spiegeln, Hände auf/zu
(Finger-Gelenke), Abriss, Share/Kreuz, NOT-HALT — und dass dabei kein
Laufbefehl entsteht und die Basis stehen bleibt. Nach einem Abriss darf nur
noch das Halte-Ziel kommen (gleiche Pose, höchstens 0,5 s + `JOG_HOLD_S`).
Am Ende steht der Roboter
im NOT-HALT (`--no-estop` lässt das aus).

## Bekannte Einschränkungen

- Real ist nur ohne Roboter getestet (Launch-Konfiguration, Node mit
  simuliertem Joy/TF, GUIs offscreen). Der erste Lauf am Roboter folgt dem
  Ablauf unter [Echter Roboter](11-A-Controller.md#echter-roboter).
- Options ist seit 2026-10-27 die Sperre (vorher „Arme übernehmen“, das
  macht jetzt R2). Die Sperre gilt nur im Speicher von ps4_arm_teleop: nach
  einem Neustart des Nodes ist der Controller entsperrt.
- Nach einem NOT-HALT hat die GUI keinen Modus mehr, der Controller behält
  seinen letzten (z. B. GEHEN mit Arm-Sperre, wie bisher). Beim nächsten
  Modus-Wechsel auf einer der beiden Seiten sind sie wieder gleich.
- Der Streamdeck (`ui_interface`) nimmt am gemeinsamen Zustand nicht teil;
  der Controller folgt seinen Modus-Knöpfen wie bisher über die Topics.
- `g1pilot-real-full` (LiDAR-Profil) startet ein anderes Bringup ohne den
  Schalter.
- LED im Sim-Container nicht setzbar (sysfs read-only), siehe oben.
- Reaktionszeit in der Sim hängt an der PC-Last (siehe
  [Reaktionszeit](#reaktionszeit)); real ist sie nur offline abgeschätzt.
- Der IK-Solver lässt posenabhängig 0,5–3 cm Restfehler (Nullraum-Haltung,
  Reichweiten-Klemme); weit vorn/außen knickt eine gerade Fahrt ab, die Hand
  bleibt dann 1–3 cm neben der Linie stehen. Betrifft Marker und Demo-GUI
  genauso, siehe [20-T-Arm-Manipulation.md](20-T-Arm-Manipulation.md).

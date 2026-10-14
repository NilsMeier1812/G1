# PS4-Controller für den Oberkörper (Arme + Hände)

Ein PS4-Controller (DualShock 4) steuert die Hände des G1: Position und
Drehung beider Hände, Hände auf/zu, Grundstellung, Stopp und NOT-HALT.
**Laufen ist bewusst nicht belegt** — der Controller kann den Roboter weder
loslaufen lassen noch drehen.

Funktioniert in der **Simulation und am echten Roboter** (Unterschiede am
echten Roboter: siehe [Echter Roboter](#echter-roboter)). Ohne den Schalter
bleibt alles wie bisher — am echten Roboter steuert der Controller dann wie
gehabt das Gehen (Deadman, `loco_client`).

## Starten

| | Startmenü (GUI) | Text-Menü (`start.sh`) | ohne Menü |
|---|---|---|---|
| Sim | Simulation → Ausstattung → **PS4-Controller** | *3c) PS4-Controller für Arme + Hände?* → Ja | `G1_PS4_ARMS=1 ./start.sh --yes` |
| Real | Echter Roboter → Ausstattung → **PS4-Controller für Arme + Hände** | *R3c) PS4-Controller steuert …* → Arme + Hände | `G1_MODE=real G1_PS4_ARMS=1 ROBOT_INTERFACE=… G1_REAL_CONFIRM=1 ./start.sh --yes` |

Den Controller per Bluetooth koppeln oder per USB anstecken — vor oder nach
dem Start, auch ein kurzer Funkabriss ist kein Problem (er verbindet sich
selbst neu). In der Sim zuerst **Options** drücken: der Roboter steht und
die Arme gehören dem Controller (wie die Kachel GREIFEN in der Demo-GUI).
Am echten Roboter erst in der GUI starten, siehe unten.

## Belegung

![Belegung](images/ps4_belegung.svg)

| Taste | Wirkung |
|---|---|
| Steuerkreuz ← | **linke** Hand wählen (LED blau) |
| Steuerkreuz → | **rechte** Hand wählen (LED grün, Standard) |
| Steuerkreuz ↑ | **beide** Hände, parallel (LED violett) — z. B. eine Kiste tragen |
| Steuerkreuz ↓ | **beide gespiegelt** (LED türkis): links/rechts und Drehungen spiegelbildlich, Bezug ist die rechte Hand — z. B. Hände auseinander/zueinander |
| Dreieck | Umschalten **VERSCHIEBEN ↔ DREHEN** (beim Drehen leuchtet die LED gedimmt) |
| Linker Stick | VERSCHIEBEN: vor/zurück + links/rechts · DREHEN: kippen (Finger hoch/runter) + schwenken |
| Rechter Stick | VERSCHIEBEN: hoch/runter (vertikal) · DREHEN: rollen (horizontal) |
| L1 / R1 | Tempo langsamer / schneller: Langsam 5 cm/s · Normal 15 cm/s · Schnell 30 cm/s (Drehen 20 / 45 / 90 °/s). Start: Sim **Normal**, Real **Langsam** (höchstens Normal) |
| Kreis | gewählte Hand(e) **öffnen** |
| Viereck | gewählte Hand(e) **schließen** |
| Kreuz | **Stopp**: Hand hält sofort an, eine laufende geplante Bewegung (Grundstellung, Ablauf) wird abgebrochen |
| Options | **Arme übernehmen**. Sim zusätzlich Greifen-Modus (stehen), wie die Kachel GREIFEN; Real nur die Arme |
| Share **1 s halten** | Grundstellung (Arme neben den Körper, geplant um Hindernisse herum) |
| PS | **NOT-HALT** (wie in den GUIs: Arme schlaff, Roboter gedämpft — real sackt er zusammen) |
| L2, R2, L3, R3, Touchpad | frei (Reserve) |

Richtungen gelten **aus Sicht des Roboters** (vor = in Blickrichtung des
Roboters, links = Roboter-links). Steht man dem Roboter gegenüber, sind
links/rechts also vertauscht.

Loslassen = Hand steht. Die Hand läuft dem Stick höchstens ein kleines Stück
voraus (wie in der Demo-GUI); kommt der Arm nicht weiter (Gelenkgrenze,
Kollisions-Gate vor Körper/Tisch), wandert das Ziel nicht davon. Wie schnell
die Hand reagiert und wie weit sie nachläuft, hängt vom Regeltakt des
arm_controllers ab, siehe [Reaktionszeit](#reaktionszeit).

**Rückmeldung**, welche Hand aktiv ist:
- Controller-LED in der Farbe der Auswahl (siehe Tabelle), rot bei NOT-HALT,
- kurze Vibration bei jedem Umschalten, lange bei NOT-HALT,
- in RViz eine farbige Kugel an der aktiven Hand, Display „PS4-Controller“
  (kein Text über dem Roboter, der stand im Weg; den vollen Status
  `PS4 · RECHTS · Verschieben · Normal` zeigt `ros2 topic echo /g1pilot/ps4/status`).

Hände öffnen/schließen wirkt mit **Inspire-Händen**; mit den starren Händen
passiert dabei nichts.

## NOT-HALT und Quittieren

PS löst den NOT-HALT aus — derselbe wie der Knopf in Demo-GUI und Streamdeck.
Die GUIs zeigen ihn dann auch an (Demo-GUI: Modus aus, real erscheint wieder
»Roboter starten«; Streamdeck: alle Knöpfe aus, EMERGENCY rot). Umgekehrt
sperrt ein NOT-HALT aus der GUI auch den Controller (LED rot).

Danach ignoriert der Controller alles außer PS, bis quittiert ist: Sim in
der Demo-GUI einen Modus wählen, Real »Roboter starten« bzw. Streamdeck
START. Das Quittieren liegt absichtlich nicht auf dem Controller: in der Sim
setzt START den Roboter auch zurück, real schaltet es den Roboter in Standby.

## Echter Roboter

Vorher [70_echtroboter_anleitung.md](70_echtroboter_anleitung.md) lesen.
Mit dem Schalter gilt am echten Roboter:

- **Kein Gehen per Controller.** Der alte `joystick`-Node startet nicht,
  `loco_client` bekommt keine Sticks mehr (auch kein Deadman-Gehen).
  Gehen geht weiter über die GUI (Demo-GUI GEHEN, Streamdeck WALK).
- **Der Controller ändert nie den Zustand der Beine.** Options übernimmt nur
  die Arme, kein Aufstehen/Balancing. Starten und Stehen bleiben in der GUI.
  Einzige Ausnahme ist PS (NOT-HALT).
- **Langsamer:** Start-Tempo Langsam (5 cm/s), mit R1 höchstens Normal
  (15 cm/s). Der arm_controller begrenzt real zusätzlich auf 0,25 m/s.
- **PS = NOT-HALT = Damp:** der Roboter sackt zusammen. Nur drücken, wenn
  das vertretbar ist (Gantry/hängend). Zum sanften Anhalten **Kreuz**
  (Hand steht, Roboter bleibt stehen). Übergeordnet bleibt die
  Unitree-Fernbedienung. Wer PS nur als Stopp will: `G1_PS4_PS_ESTOP=0`
  (dann wirkt PS wie Kreuz).
- LED-Farbe funktioniert real (Container `privileged`), in der Sim nicht.
- Hände auf/zu nur mit Inspire-Händen (Ausstattung → Inspire-Hände).
- **Seit 2026-10-13 auch real geändert** (siehe [Reaktionszeit](#reaktionszeit)):
  arm_controller liest LowState nicht mehr blockierend im Takt, nimmt nur
  das neueste Hand-Ziel, Halte-Ziel wird 1 s wiederholt, Vorlauf nur
  entlang der Bewegung. Das gilt für Controller, Demo-GUI und Marker
  gleichermaßen: die Hand reagiert eher und läuft nach dem Loslassen
  weniger nach. Glättung und Tempo-Limits sind real unverändert
  (`smoothing_ref_hz` bleibt 0).

Ablauf:

1. Start wie gewohnt; Log zeigt `PS4-Arm-Steuerung bereit … (max. Normal,
   PS=NOT-HALT, Options = nur Arme)` und `PS4-Controller: Daten kommen an`.
2. In der GUI **Roboter starten**, dann **GREIFEN** (Demo-GUI) bzw. START →
   START BALANCING → ENABLE MANIPULATION (Streamdeck). Für die ersten Tests
   den Roboter hängend lassen und nur ENABLE MANIPULATION bzw. **Options**.
3. Steuerkreuz → Hand wählen (LED-Farbe prüfen), Stick **wenig** auslenken.
   Erwartung: Hand fährt langsam, steht beim Loslassen.
4. **Kreuz** testen (Hand steht sofort), dann Kreis/Viereck (Hände).
5. **NOT-HALT-Test nur hängend:** PS. Erwartung wie beim GUI-Knopf (Arme
   schlaff, kein Sprung). Danach Roboter starten → Options (bzw. ENABLE
   MANIPULATION) übernimmt die Arme wieder ruckfrei.
6. **Abriss-Test:** Stick halten und dabei USB-Kabel ziehen bzw. Bluetooth
   am PC trennen (`bluetoothctl disconnect …`). Nicht PS lange drücken —
   das wäre der NOT-HALT. Erwartung: Hand steht innerhalb 0,5 s. Bei
   Bluetooth zusätzlich: Controller mit ausgelenktem Stick abschirmen bzw.
   außer Reichweite bringen (Funk hängt, ohne sauber zu trennen). Erwartung:
   Log „seit 0.3 s keine Daten“, Hand steht nach < 1 s.

| Env-Variable (Real) | Default | Bedeutung |
|---|---|---|
| `G1_PS4_ARMS` | `0` | `1` = Controller für Arme + Hände statt fürs Gehen |
| `G1_PS4_SPEED` | `Langsam` | Start-Tempo |
| `G1_PS4_MAX_SPEED` | `Normal` | höchstes Tempo per R1 |
| `G1_PS4_PS_ESTOP` | `1` | `0` = PS nur Stopp (wie Kreuz) statt NOT-HALT |

## Verbindung, Container, LED

- Der Sim-Container bekommt `/dev/input` über `docker-compose.ps4.yml`
  (start.sh nimmt die Datei automatisch dazu, wenn `G1_PS4_ARMS=1`). Kein
  `privileged`; erlaubt ist nur das Öffnen von Eingabegeräten. Der
  Real-Container (`g1pilot-real`) hat `/dev/input` schon immer.
- Die **LED** wird über sysfs gesetzt. Das ist im Sim-Container schreibgeschützt
  — dann bleibt die LED wie sie ist und die Auswahl zeigen RViz und die
  Vibration. Ein Hinweis steht einmal im Log. Real (privileged) geht es.
- Gerätename: Bluetooth meldet „Wireless Controller“, USB je nach Treiber
  „Sony Interactive Entertainment Wireless Controller“ — beides wird erkannt.
  Anderer Name: `JOYSTICK_NAME=...`.
- Unter WSL gibt es kein `/dev/input` → der Controller ist dort nicht nutzbar
  (start.sh warnt).

## Fehlerbehebung

| Symptom | Ursache / Abhilfe |
|---|---|
| Log: „Kein PS4-Controller gefunden … Warte“ | Controller nicht gekoppelt oder Container ohne `/dev/input` (über start.sh starten bzw. `COMPOSE_FILE=…:docker-compose.ps4.yml`). |
| LED/Kugel zeigt die Auswahl, aber die Hand bewegt sich nicht | Arme nicht übernommen → **Options** drücken (oder GUI: GREIFEN / ENABLE MANIPULATION). Im Gehen-Modus hängen die Arme in der Lauf-Pose. Real nach NOT-HALT: erst »Roboter starten«, dann Options. |
| Hand bleibt an einer Stelle hängen | Kollisions-Gate (Log: „Kollisions-Gate …“) oder Reichweite erreicht → in eine andere Richtung bewegen. |
| Sim: Hand reagiert spürbar verzögert / fährt nach dem Loslassen weiter | PC ausgelastet → arm_controller regelt nur mit 10–15 Hz. RViz und Navigation abschalten (`USE_RVIZ=false G1_ENABLE_NAV=0`), Last mit `top` prüfen. Siehe [Reaktionszeit](#reaktionszeit). |
| Nach PS reagiert nichts mehr | NOT-HALT aktiv → in der GUI quittieren (Sim: Modus wählen, Real: Roboter starten). |
| Real: Controller lässt den Roboter nicht mehr gehen | So gewollt im PS4-Arm-Modus. Fürs Gehen per Controller ohne den Schalter starten (`G1_PS4_ARMS=0`). |

---

## Technik

### Dateien

| Datei | Inhalt |
|---|---|
| `g1pilot/teleoperation/ps4_joystick.py` | Node `ps4_joystick`: evdev → `/g1pilot/ps4/joy` in fester Belegung, Wiederverbinden, LED (sysfs) + Vibration (Force-Feedback) aus `/g1pilot/ps4/feedback` |
| `g1pilot/teleoperation/ps4_layout.py` | feste Belegung (Indizes) und evdev-Code-Zuordnung, ohne ROS/evdev testbar |
| `g1pilot/teleoperation/ps4_arm_teleop.py` | `ArmTeleopLogic` (reine Bedienlogik) + Node `ps4_arm_teleop` |
| `g1pilot/teleoperation/hand_jog.py` | Jog-Rechnung (Ziel = Hand + Geschwindigkeit, Vorlauf entlang der Bewegung begrenzt), Halte-Ziel (`hold_pose`, `HoldStreamer`), gemeinsam mit der Demo-GUI |
| `launch/teleoperation_launcher.launch.py` | Argument `ps4_arms` (Default `false`): startet `ps4_joystick` + `ps4_arm_teleop` statt `joystick`; `ps4_speed`, `ps4_max_speed`, `ps4_options_balancing`, `ps4_ps_estop` |
| `launch/bringup_sim.launch.py` | setzt `ps4_arms` aus `G1_PS4_ARMS` (Sim-Defaults) |
| `launch/bringup_real.launch.py` | setzt `ps4_arms` aus `G1_PS4_ARMS` mit Real-Werten (Langsam/max Normal, Options ohne `start_balancing`) |
| `teleoperation/demo_gui.py`, `teleoperation/ui_interface.py` | zeigen einen NOT-HALT von außen (PS-Taste) an |
| `docker-compose.ps4.yml` | `/dev/input` + `device_cgroup_rules: c 13:* rmw` für `g1pilot-sim` |
| `test/test_ps4_arm_teleop.py`, `test/test_ps4_joystick.py` | Unit-Tests (Belegung, Logik, Jog, simuliertes evdev-Gerät inkl. hängender Funkverbindung) |
| `test/test_ps4_arm_teleop_node.py` | Node mit Real-Einstellungen gegen simuliertes Joy + TF (Options, Tempo, NOT-HALT von außen, kein Laufbefehl) |
| `test/test_gui_external_estop.py` | Demo-GUI + Streamdeck folgen einem NOT-HALT von außen (Sim und Real, offscreen) |
| `test/test_demo_gui_jog_hold.py` | Demo-GUI »Hand bewegen«: Halte-Ziel beim Loslassen (auf der Linie, 1 s wiederholt, Grundstellung löscht es) |
| `test_ps4_arm_sim.py` | Ende-zu-Ende-Test gegen die laufende Sim (simulierte Joy-Eingabe) |

### Datenfluss

```
DualShock 4 ──evdev──> ps4_joystick ──/g1pilot/ps4/joy (Joy, feste Belegung)──> ps4_arm_teleop
                            ^                                                        │
                            └──── /g1pilot/ps4/feedback (LED, Vibration) ────────────┤
                                                                                     ├─> /g1pilot/hand_goal/<side>   (PoseStamped, pelvis)
                                                                                     ├─> /g1pilot/hand_action/<side> ("open"/"close")
                                                                                     ├─> /g1pilot/arms/enabled (+ /g1pilot/start_balancing nur Sim) (Options)
                                                                                     ├─> /g1pilot/arms/home (Share, Impuls)
                                                                                     ├─> /g1pilot/pose_store/cancel (Kreuz)
                                                                                     ├─> /g1pilot/emergency_stop (PS)
                                                                                     ├─> /g1pilot/ps4/status (JSON, latched)
                                                                                     └─> /g1pilot/ps4/markers (RViz)
```

Bewusst **nicht** über `joy_mux`/`/g1pilot/joy`: dort hängen `joy_to_cmdvel`
(Sim, mit Navigation) und `loco_client` (real) — die Sticks würden sonst den
Roboter laufen lassen. Der alte `joystick`-Node startet im PS4-Modus nicht,
weil er denselben Controller lesen würde.

Eingänge außer dem Joy: `/g1pilot/emergency_stop` (NOT-HALT aus der GUI →
Controller sperren) und `/g1pilot/start` (Quittung → freigeben).

### Feste Belegung (`/g1pilot/ps4/joy`)

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
hid-playstation). Der alte `joystick`-Node nummeriert dagegen in
Geräte-Reihenfolge — für eine Tastenbelegung zu wackelig.

### Sicherheit

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
- Real: Options ohne `start_balancing`, Tempo höchstens Normal.
- Alle Ziele laufen durch den arm_controller (IK, Kollisions-Gate, EE-Tempo-Limit).

### Reaktionszeit

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

### Parameter `ps4_arm_teleop`

| Parameter | Default | Bedeutung |
|---|---|---|
| `joy_topic` | `/g1pilot/ps4/joy` | Eingang |
| `rate_hz` | 30 | Takt der Hand-Ziele |
| `deadzone` / `expo` | 0.12 / 0.4 | Stick-Kennlinie |
| `speed` | `Normal` (Real `Langsam`) | Start-Tempo |
| `max_speed` | `Schnell` (Real `Normal`) | höchstes Tempo per R1 |
| `ps_estop` | `true` | PS = NOT-HALT; `false` = PS wirkt wie Kreuz |
| `home_hold_s` | 1.0 | Share-Haltezeit für Grundstellung |
| `joy_timeout_s` | 0.5 | Abriss-Erkennung |
| `options_starts_balancing` | `true` (Real `false`) | Options sendet zusätzlich `start_balancing` |

### Tests

```bash
# Unit-Tests (ohne Controller/Roboter; die ROS-/GUI-Tests brauchen rclpy/PyQt6 -> im Container)
python3 -m pytest -q test/test_ps4_arm_teleop.py test/test_ps4_joystick.py \
    test/test_ps4_arm_teleop_node.py test/test_gui_external_estop.py \
    test/test_demo_gui_jog_hold.py

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

### Bekannte Einschränkungen

- Real ist nur ohne Roboter getestet (Launch-Konfiguration, Node mit
  simuliertem Joy/TF, GUIs offscreen). Der erste Lauf am Roboter folgt dem
  Ablauf unter [Echter Roboter](#echter-roboter).
- Options ↔ Demo-GUI: Options übernimmt die Arme, die Kachel in der Demo-GUI
  weiß davon nichts (zeigt weiter ihren letzten Zustand). Der NOT-HALT ist
  dagegen abgeglichen.
- `g1pilot-real-full` (LiDAR-Profil) startet ein anderes Bringup ohne den
  Schalter.
- LED im Sim-Container nicht setzbar (sysfs read-only), siehe oben.
- Reaktionszeit in der Sim hängt an der PC-Last (siehe
  [Reaktionszeit](#reaktionszeit)); real ist sie nur offline abgeschätzt.
- Der IK-Solver lässt posenabhängig 0,5–3 cm Restfehler (Nullraum-Haltung,
  Reichweiten-Klemme); weit vorn/außen knickt eine gerade Fahrt ab, die Hand
  bleibt dann 1–3 cm neben der Linie stehen. Betrifft Marker und Demo-GUI
  genauso, siehe [11_arm_manipulation_technik.md](11_arm_manipulation_technik.md).

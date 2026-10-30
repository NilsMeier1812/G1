# Streamdeck — Anleitung

Richtet sich an: Entwickler und Betreuer, die den G1 über das
Streamdeck-Fenster (alle Einzelfunktionen) oder den alten
Joystick-Controller bedienen. Für die interne Funktionsweise siehe
[13-T-Streamdeck.md](13-T-Streamdeck.md).

## Überblick

Der Streamdeck ist ein virtuelles Tastenfeld mit allen Einzelfunktionen. Er
ist die Oberfläche für Entwicklung und Fehlersuche. Für Vorführungen und den
normalen Betrieb ist die [Demo-GUI](10-A-Demo-GUI.md) gedacht (Default im
Startmenü). Gewählt wird im Startmenü unter *Bedienoberfläche* bzw. mit
`G1_GUI=streamdeck`. Es läuft immer nur eine der beiden GUIs.

Der heutige Controller (PS4/Xbox für Arme, Hände und Gehen) ist in
[11-A-Controller.md](11-A-Controller.md) beschrieben. Dieses Dokument
beschreibt am Ende nur noch den **alten Joystick-Modus**, der läuft, wenn der
Schalter *PS4-Controller* im Startmenü aus ist.

## Der Streamdeck

Der Streamdeck ist ein 5×5-Tastenraster. Aktive Buttons leuchten grün.

| Bereich | Buttons | Wirkung |
|---|---|---|
| Locomotion | START, START BALANCING, WALK | siehe [30-A-Stehen-und-Gehen.md](30-A-Stehen-und-Gehen.md) |
| Arme | ENABLE MANIPULATION, HOMING ARMS | siehe [20-A-Arm-Manipulation.md](20-A-Arm-Manipulation.md) |
| Hände | OPEN/CLOSE LEFT/RIGHT HAND, INSPIRE FTP GUIs | siehe [22-A-Inspire-Haende.md](22-A-Inspire-Haende.md) |
| Positionsspeicher | POSE SPEICHERN, POSE ANFAHREN, POSE ABBRECHEN | siehe [20-A-Arm-Manipulation.md](20-A-Arm-Manipulation.md) |
| Sonstiges | MARKER FOLLOW, AUTO NAV (nur mit Navigation) | — |
| Nur Sim | PUSH ROBOT (kurz klicken = Schubs, halten = aufladen), GRASP BOX (Testkugel an/aus), RESET SCENE (Objekte zurück) | siehe [40-A-Simulation.md](40-A-Simulation.md) |
| Notaus | EMERGENCY STOP (unten rechts, rot) | sofortiger Stopp, siehe unten |

Unterhalb des Tastenrasters befindet sich ein Bildschirm-Joystick (Ziehen mit
der Maus = vorwärts/seitwärts) und ein Dreh-Regler (Yaw). Beide wirken nur im
Zustand WALK; Loslassen zentriert automatisch auf 0 (Stopp).

**MARKER FOLLOW** (Standard: an) lässt die RViz-Zielmarker der Arme der Hand
folgen, solange sie nicht gerade gezogen werden — sie „springen" also nach
externen Bewegungen (Homing, Positionsspeicher) nicht, sondern gleiten
unauffällig hinterher.

**In der Simulation startet der Stack automatisch**: nach kurzer Anlaufzeit
werden die Arme aktiviert und der Roboter geht in den balancierten Stand
(die Anforderung wird wiederholt, bis arm_controller und loco_sim sie
bestätigen). **Auf dem echten Roboter passiert das absichtlich nicht** — dort
bewegt sich nichts, bevor der Bediener klickt.

Der Streamdeck nimmt nicht am gemeinsamen Bedienzustand von Demo-GUI und
Controller teil. Der Controller folgt seinen Modus-Knöpfen trotzdem, weil
beide dieselben Topics nutzen, und ein NOT-HALT vom Controller färbt
EMERGENCY STOP rot.

## Notaus

**EMERGENCY STOP** ist derselbe NOT-HALT wie in der Demo-GUI: Arme werden
drehmomentfrei und sacken gedämpft (kein aktiver Sprung in eine Home-Pose),
Hände werden deaktiviert (halten ihre Position). Die Beine bekommen am echten
G1 nur `StopMove` und **kein** `Damp()` (seit 2026-10-15, alter Stand mit
`G1_ESTOP_DAMP_LEGS=1`), die Taille bleibt gehalten. In der Sim dämpft
`loco_sim` den ganzen Roboter. Quittieren mit **START**; Arme bleiben danach
schlaff, bis **ENABLE MANIPULATION** die Kontrolle bewusst zurückholt.

Übersicht aller Halt-Wege (GUI, Controller, Handy, Voll-Notaus):
[51-A-Notaus.md](51-A-Notaus.md). Sicherheitsablauf für echte Hardware:
[50-A-Echter-Roboter.md](50-A-Echter-Roboter.md).

## Alter Joystick-Modus (ohne Schalter PS4-Controller)

Ist im Startmenü *PS4-Controller* **nicht** angehakt (`G1_PS4_ARMS=0`),
startet statt `ps4_joystick`/`ps4_arm_teleop` der alte Node `joystick`. Er
liest einen Controller mit exakt passendem Gerätenamen (`JOYSTICK_NAME`,
Default „Wireless Controller“) und gibt die Rohdaten über `joy_mux` an
`loco_client` (real) bzw. `joy_to_cmdvel` (Sim mit Navigation). Die
Indizes folgen der Geräte-Reihenfolge des Treibers, nicht der festen
Belegung des PS4-Modus. Xbox-Controller sind hier nicht vorgesehen.

| Eingabe (Index im `Joy`) | Wirkung (real, `loco_client`) |
|---|---|
| Button 8 (Deadman, halten) | Steuert; loslassen stoppt sofort |
| Linker Stick (bei gehaltenem Deadman) | vx/vy |
| Rechter Stick X (bei gehaltenem Deadman) | Drehen |
| Steuerkreuz oben (letzte Achse) | START (wie in den GUIs) |
| Button 6 | START BALANCING (nur aus Preparation) |
| Button 5 | EMERGENCY STOP |
| Button 0 | Arm-Steuerung ein/aus |
| Button 1 | Arme homen |
| Button 3 + Achse 4 (rechts/links) | Greifer öffnen/schließen |
| Dreieck (`BTN_NORTH`) | Autonome Navigation umschalten |

Der Deadman-Button ist Pflicht: Wird er losgelassen, stoppt der Roboter
innerhalb von rund 0,5 Sekunden von selbst, auch wenn die Software gerade
abstürzt oder das Fenster geschlossen wird.

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| Kein Streamdeck-Fenster | X11-Display verfügbar? (`xhost`, siehe [01-A-Installation.md](01-A-Installation.md)) |
| Alter Joystick reagiert nicht | Gerätename passt zu `JOYSTICK_NAME`? Log prüfen: „Joystick found" bzw. „No joystick found! (gesucht: …)". USB-Controller heißen teils anders als Bluetooth-Controller. Für den PS4-/Xbox-Modus siehe [11-A-Controller.md](11-A-Controller.md#fehlerbehebung). |
| Roboter läuft nach GUI-Absturz weiter | Darf nicht passieren (Deadman) — melden, falls beobachtet. |
| AUTO NAV Button fehlt | Erscheint nur, wenn der Navigations-Stack läuft (`G1_ENABLE_NAV`/`G1_ENABLE_LIDAR`). |

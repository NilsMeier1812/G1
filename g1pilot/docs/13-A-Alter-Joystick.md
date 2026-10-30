# Alter Joystick — Anleitung

Richtet sich an: Entwickler und Betreuer, die den G1 mit dem alten
Joystick-Controller bedienen. Für die interne Funktionsweise siehe
[13-T-Alter-Joystick.md](13-T-Alter-Joystick.md).

## Überblick

Der heutige Controller (PS4/Xbox für Arme, Hände und Gehen) ist in
[11-A-Controller.md](11-A-Controller.md) beschrieben, die Bedienoberfläche
ist die [Demo-GUI](10-A-Demo-GUI.md). Dieses Dokument beschreibt nur noch den
**alten Joystick-Modus**, der läuft, wenn der Schalter *PS4-Controller* im
Startmenü aus ist.

Die frühere Streamdeck-GUI (`ui_interface`, Tastenraster mit allen
Einzelfunktionen) wurde am 2026-10-30 entfernt, weil die Demo-GUI sie
vollständig ersetzt. Die reinen Sim-Testknöpfe von damals (GRASP BOX,
CATCH FALLS) gibt es nur noch als Topic, z. B.
`ros2 topic pub --once /g1pilot/grasp_box std_msgs/msg/Bool "{data: true}"`.

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
| Alter Joystick reagiert nicht | Gerätename passt zu `JOYSTICK_NAME`? Log prüfen: „Joystick found" bzw. „No joystick found! (gesucht: …)". USB-Controller heißen teils anders als Bluetooth-Controller. Für den PS4-/Xbox-Modus siehe [11-A-Controller.md](11-A-Controller.md#fehlerbehebung). |
| Roboter läuft nach GUI-Absturz weiter | Darf nicht passieren (Deadman) — melden, falls beobachtet. |

# 1 · Bedienung

Womit man den G1 steuert. Alle Wege senden dieselben Befehle; die Logik
sitzt in den Reglern, nicht in der Oberfläche. Den NOT-HALT gibt es überall
([51-A-Notaus.md](51-A-Notaus.md)).

| Möglichkeit | Wofür | Anleitung | Technik |
|---|---|---|---|
| **Demo-GUI** | Standard am PC und für Vorführungen: Modi GREIFEN/GEHEN/AUTO NAV, Hand bewegen, Hände, Posen und Abläufe, Startleiste am echten G1, Temperaturen | [10-A](10-A-Demo-GUI.md) | [10-T](10-T-Demo-GUI.md) |
| **Controller** (PS4/Xbox) | direkt am Roboter: Hände bewegen und drehen, Hände auf/zu, Hüftdrehung, Gehen mit L2 (Totmann), GREIFEN mit R2, Sperre zum Ablegen, Voll-Notaus | [11-A](11-A-Controller.md) | [11-T](11-T-Controller.md) |
| **Handy** (Android, Bluetooth) | Zusatz zur Demo-GUI: Zustand sehen, NOT-HALT, Startleiste, Modus, Hände, Pose speichern | [12-A](12-A-Handy.md) | [12-T](12-T-Handy.md) |
| **Alter Joystick** | Joystick-Modus ohne PS4-Schalter (Sticks gehen über `joy_mux` ans Laufen); die frühere Streamdeck-GUI wurde 2026-10-30 entfernt | [13-A](13-A-Alter-Joystick.md) | [13-T](13-T-Alter-Joystick.md) |

Die Bedienoberfläche ist immer die Demo-GUI. Controller und Handy schaltet man dort unter
*Ausstattung* zu; beide setzen die Demo-GUI voraus bzw. teilen ihren Zustand
mit ihr.

Weitere Wege an die Arme: RViz-Marker ([20-A](20-A-Arm-Manipulation.md)) und
die HTTP-Arm-API für eigene Programme ([21-A](21-A-Arm-API.md)).

Alle Gruppen: [README.md](README.md).

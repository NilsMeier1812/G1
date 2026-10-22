# Navigation — Anleitung

Richtet sich an: Anwender, die den G1 autonom von Punkt zu Punkt fahren
lassen wollen. Für die interne Funktionsweise siehe
[51_navigation_technik.md](51_navigation_technik.md).

## Überblick

Kurzfassung des Prinzips: Ein **Ziel** (Punkt in der Karte) wird gesetzt. Ein
globaler Planer (Dijkstra) berechnet einen Pfad, ein Follower fährt ihn ab,
indem er einen **virtuellen Joystick** erzeugt — denselben Signalweg, den
auch eine manuelle Bedienung nutzt. Die Autonomie läuft dadurch über exakt
denselben, erprobten Steuerpfad wie die Teleoperation.

Es ist derselbe Navigations-Stack in Simulation und auf echter Hardware; nur
die Pose-Quelle unterscheidet sich (Simulation: aus der MuJoCo-Ground-Truth,
Real: MOLA-LiDAR-Odometrie).

## Voraussetzungen

- **Simulation:** Navigation beim Start aktivieren (Startmenü → Simulation →
  Ausstattung → „Navigation“, oder `G1_ENABLE_NAV=1 ./start.sh --yes`).
- **Echter Roboter:** Livox-MID360-LiDAR + das LiDAR-Image `g1pilot-real-lidar`,
  aktiviert über `G1_ENABLE_LIDAR=1 ./start.sh --build` (siehe „Echter Roboter“).
- RViz wird bei aktiver Navigation automatisch mitgestartet (Nav-Ansicht,
  Fixed Frame `map`) — es gibt kein eigenes Navigationsfenster.

## Bedienung

**1. Roboter laufbereit machen**

```bash
ros2 topic pub --once /g1pilot/start_balancing std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /g1pilot/start_walking   std_msgs/msg/Bool "{data: true}"   # nur Sim nötig
```

Oder am Streamdeck: START BALANCING, dann (in der Sim) WALK. Erst danach
reagiert der Roboter auf Geschwindigkeitskommandos.

**2. Ziel setzen**

In RViz: Toolbar-Werkzeug „2D Goal Pose" anklicken, dann in die Karte
klicken-ziehen. Die Zieh-Richtung bestimmt die End-Ausrichtung — der
Roboter fährt zum Punkt und dreht sich dort auf der Stelle auf genau diesen
Winkel (kürzester Weg). Nur klicken ohne Ziehen ergibt Ausrichtung 0.

Per Kommandozeile:

```bash
ros2 topic pub --once /g1pilot/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}}}"
```

**3. Autonomie anfragen (vormerken)**

Am Streamdeck oder in der Demo-GUI: **AUTO NAV** (bzw. eine Station). Per
Kommandozeile:

```bash
ros2 topic pub --once /g1pilot/auto_enable std_msgs/msg/Bool "{data: true}"
```

Das ist nur eine **Anfrage**. Der `nav_supervisor` entscheidet, ob der Roboter
wirklich läuft (Zustand in GUI/Log: „vorgemerkt“ → „läuft“).

**4. Starten mit Totmann (real immer, Sim mit `G1_NAV_DEADMAN=1`)**

Am PS4-/Xbox-Controller **L2 halten und den linken Stick kurz bewegen** —
dann läuft er los. Danach reicht L2 halten (Stick loslassen). **L2 los =
sofort Stopp** (pausiert); erneut L2 + Stick setzt fort. Solange AUTO NAV
angefragt ist, steuert L2 die Autonomie, nicht das manuelle Gehen; nach dem
Abschalten muss L2 einmal losgelassen werden, bevor manuelles Gehen wieder
greift (kein Losrennen in Stick-Richtung).

In der Sim ohne Totmann (Default) läuft der Roboter direkt nach der Anfrage.

**5. Abschalten / anhalten**

**AUTO NAV** erneut klicken oder L2 loslassen — der Roboter stoppt sofort.
Controller-Voll-Notaus und EMERGENCY STOP wirken wie immer.

## Sicherheitswächter (`nav_supervisor`, Stand 2026-10-22)

Zwischen Folger und `joy_mux` sitzt der `nav_supervisor`. Nur er gibt den
Auto-Joystick frei; ohne ihn ist AUTO NAV gesperrt (`joy_mux` hört auf
`/g1pilot/nav/auto_active`, `nav2point` schreibt auf `/g1pilot/nav/raw_joy`).
Jeder Ausfall → genau ein Stopp-Kommando, Grund in GUI/Log
(`/g1pilot/nav/state`):

| Wächter | Grenze | Folge |
|---|---|---|
| Totmann L2 / Controller-Funk weg | 0,3 s | Pause, L2 + Stick setzt fort |
| Herzschlag GUI/Streamdeck (Funk/WLAN, Absturz) | 1,0 s | Stopp, neue Anfrage nötig |
| Pose fehlt / springt (> 0,2 m oder 15°) | 0,3 s | Stopp |
| Folger schweigt | 0,3 s | Stopp |
| Abweichung vom Pfad | 0,3 m | Stopp |
| Kein Fortschritt | 5 s | Stopp |
| Ziel zu weit | 10 m | gesperrt |
| Roboter nicht im Gehen (real `balanced`, Sim `walk`) | sofort | gesperrt/Stopp |
| Notaus | sofort | Stopp, neue Anfrage nötig |
| Neigung (wenn IMU kommt) | 15° | Stopp |
| Karte: Roboter/Ziel auf belegter/unbekannter Zelle | — | gesperrt/Stopp |

Tempo-Deckel real: `G1_NAV_MAX_VX` (0,15 m/s) und `G1_NAV_MAX_VYAW`
(0,3 rad/s), unabhängig von den Hand-Gehen-Limits.

## Echter Roboter (Stand 2026-10-22, nicht an Hardware getestet)

- **LiDAR-Image:** `G1_ENABLE_LIDAR=1 ./start.sh --build` (oder
  `make build-g1pilot-real-lidar`) baut `g1pilot-real-lidar` mit Livox-SDK2,
  `livox_ros_driver2` und MOLA. Ohne LiDAR-Image startet der Real-Stack normal,
  nur ohne LiDAR (Hinweis im Log).
- **LiDAR-Netz:** Der MID-360 schickt seine Daten an den PC am Roboter-LAN.
  Host-IP = `G1_LIDAR_HOST_IP`, sonst die IPv4 von `ROBOT_INTERFACE`;
  LiDAR-IP = `G1_LIDAR_IP` (Default 192.168.123.120). Die erzeugte Datei liegt
  in `/tmp/g1pilot_livox_mid360.json`, das Log nennt die IPs.
- **Pflicht: Pose-Quelle.** Planer (`dijkstra_planner`) und Folger (`nav2point`)
  brauchen `/lidar_odometry/pose_fixed`. Auf dem echten G1 liefert die nur die
  MOLA-LiDAR-Odometrie — **die ist noch nicht eingebunden (Stufe 2)**. Bis dahin
  gibt es real keine Pose: der Planer meldet „No odom pose yet“, der
  `nav_supervisor` sperrt AUTO NAV.
- **Drift-Test (Stufe 0):** `python3 tools/odom_drift_log.py --iface <ROBOT_INTERFACE>`
  neben dem Real-Stack starten, G1 per Controller eine vermessene Strecke laufen
  lassen (z. B. 4 m geradeaus, 1 m rechts), Strg+C. Ausgabe: Endpose im
  Start-Bezug je Odometrie-Topic + CSV — mit dem Maßband vergleichen.
  `sim_localization` (Pose aus `rt/sportmodestate`) läuft nur in der Sim.
- **Ziel ohne Karte funktioniert:** `create_map`/`scene_bridge` laufen nur in
  der Sim, real gibt es kein `/map`. Dann publiziert der Planer eine **gerade
  Linie** von der aktuellen Pose zum RViz-Ziel („2D Goal Pose“ → `/g1pilot/goal`).
  Der Roboter läuft sie ohne Hindernisvermeidung ab (LiDAR-Punkte fließen nicht
  in die Planung). Fläche freihalten, Aufsicht mit Fernbedienung.
- **Kette:** Ziel → `dijkstra_planner` → `/g1pilot/path` → `nav2point` →
  `/g1pilot/nav/raw_joy` → `nav_supervisor` → `/g1pilot/auto_joy` →
  `joy_mux` (`/g1pilot/nav/auto_active`) → `/g1pilot/joy` →
  `loco_client` (`Move`, Limits `G1_MAX_VX/VY/VYAW`). Der Roboter muss im
  Walk-Zustand sein (LAUFEN); bleibt der Joy-Strom aus, stoppt `loco_client`
  nach 0,5 s.
- Das Koordinatensystem „map“ ist die MOLA-Startpose; Drift der LiDAR-Odometrie
  ist nicht vermessen.

## Worauf zu achten ist

- **Hindernisvermeidung nur für die geladene Umgebung, nicht spontan.** In
  der Simulation rastert die Karte die Objekte der geladenen Umgebung
  (siehe [61_inspire_haende_technik.md](61_inspire_haende_technik.md) für
  verwandte Szenen-Konzepte bzw. die Architektur-Doku für die Szenen-Brücke)
  — spontane Hindernisse (Menschen, verschobene Objekte) werden **nicht**
  erkannt, es gibt keine Live-Perzeption. Auf echter Hardware ohne
  entsprechendes Setup bleibt die Karte leer. Nur in freier, kontrollierter
  Fläche einsetzen.
- **Auto ODER manuell, nicht beides gleichzeitig.** Bei aktiver Autonomie
  hat der Navigations-Joystick Vorrang; L2 ist dann der Totmann der Autonomie.
- **Deadman gilt weiter.** Fällt der Navigationsstream aus, stoppt der
  `nav_supervisor`; dahinter greift derselbe Timeout wie beim manuellen Gehen.
- **Konservative Geschwindigkeitslimits** gelten auch für die Autonomie.

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| Ziel gesetzt, aber kein Pfad | Liegt eine Pose vor? `ros2 topic echo /lidar_odometry/pose_fixed` — läuft die Lokalisierung (Sim: `sim_localization`; Real: MOLA/Livox)? |
| Pfad vorhanden, Roboter fährt nicht | Grund in GUI/`ros2 topic echo /g1pilot/nav/state`. „bereit: L2 halten + linken Stick bewegen“ = Totmann fehlt. Sonst: Anfrage gesendet? Roboter laufbereit? Läuft `nav_supervisor`? |
| Stoppt mit „Herzschlag“ | GUI/Streamdeck geschlossen oder Netz weg — gewollt. Neu anfragen. |
| Roboter ruckelt / zwei Geschwindigkeitsquellen | Manuellen Joystick loslassen — bei aktiver Autonomie nicht gleichzeitig manuell fahren. |
| Fährt am Ziel vorbei / dreht unruhig | Regelparameter des Followers prüfen (siehe Technik-Dokument). |
| RViz zeigt den Roboter fest im Ursprung | Nur bei aktiver Navigation bewegt sich das TF-Modell; Fixed Frame in RViz muss `map` sein. |
| „Fährt in Hindernis" | Läuft die Szenen-Brücke? Objekt in der geladenen Umgebung enthalten? Ohne eigene Umgebung bzw. auf real ist die Karte weiterhin leer — Fläche freihalten. |

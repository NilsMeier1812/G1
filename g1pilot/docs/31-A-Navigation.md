# Navigation — Anleitung

Richtet sich an: Anwender, die den G1 autonom von Punkt zu Punkt fahren
lassen wollen. Für die interne Funktionsweise siehe
[31-T-Navigation.md](31-T-Navigation.md).

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
- **Echter Roboter:** Livox-MID360-LiDAR + das große Docker-Image, aktiviert
  über `G1_ENABLE_LIDAR=1`.
- RViz wird bei aktiver Navigation automatisch mitgestartet (Nav-Ansicht,
  Fixed Frame `map`) — es gibt kein eigenes Navigationsfenster.

## Bedienung

**1. Roboter laufbereit machen**

```bash
ros2 topic pub --once /g1pilot/start_balancing std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /g1pilot/start_walking   std_msgs/msg/Bool "{data: true}"   # nur Sim nötig
```

Demo-GUI: Kachel **GEHEN** (dort sitzen AUTO NAV und die Stations-Knöpfe).
Streamdeck: START BALANCING, dann (in der Sim) WALK. Erst danach reagiert
der Roboter auf Geschwindigkeitskommandos.

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

**3. Autonomie scharfschalten**

Demo-GUI: im Modus GEHEN der Knopf **AUTO NAV** (Zustände *aus* → *AN,
wartet auf Ziel* → *läuft …* → *AN, Ziel erreicht*; Knopf und Pfeile sind
währenddessen gesperrt). Streamdeck: Button **AUTO NAV** (erscheint
automatisch, sobald der Nav-Stack läuft). Per Kommandozeile:

```bash
ros2 topic pub --once /g1pilot/auto_enable std_msgs/msg/Bool "{data: true}"
```

Der Roboter folgt jetzt dem Pfad und stoppt selbstständig am Ziel.

**Feste Ziele: Stationen (Demo-GUI).** Jedes Objekt `station_<Name>` der
geladenen Umgebung wird in GEHEN ein Knopf **Zur Station <Name>**. Ein Klick
setzt das Ziel und schaltet AUTO NAV ein; beim Ankommen geht AUTO NAV von
selbst wieder aus und der Status meldet „Angekommen an Station …“. Ohne Weg
(oder ohne Pfad nach 5 s) geht AUTO NAV ebenfalls aus. Stationen legt man im
Umgebungs-Editor an ([40-A-Simulation.md](40-A-Simulation.md#umgebungen)).

**4. Abschalten / anhalten**

**AUTO NAV** erneut klicken — der Roboter stoppt sofort. Beim Wechsel zu
GREIFEN und bei NOT-HALT geht AUTO NAV automatisch aus. Sanftes Anhalten wie
immer: GREIFEN bzw. START BALANCING. Harter Halt: NOT-HALT
([51-A-Notaus.md](51-A-Notaus.md)).

## Echter Roboter (Stand 2026-10-15, Code-Prüfung, nicht an Hardware getestet)

> **Achtung:** Eine Analyse vom 2026-10-22 (Projekt-Analysen,
> `lidar_autonav_real_g1.md`) kam zum Ergebnis, dass der Real-Pfad so noch
> nicht läuft: im schlanken Real-Image fehlt der Livox-Treiber, MOLA wurde
> nicht gestartet und es gibt einen TF-Konflikt. Vor einem Versuch am
> Roboter erst diese Punkte klären.

- **Pflicht: Pose-Quelle.** Planer (`dijkstra_planner`) und Folger (`nav2point`)
  brauchen `/lidar_odometry/pose_fixed`. Auf dem echten G1 liefert die nur die
  MOLA-LiDAR-Odometrie (`G1_ENABLE_LIDAR=1`, großes Image `g1pilot-real-full`,
  Livox MID360). Ohne LiDAR gibt es keine Pose: der Planer meldet „No odom pose
  yet“ und publiziert keinen Pfad, AUTO NAV ist in der GUI gesperrt.
  `sim_localization` (Pose aus `rt/sportmodestate`) läuft nur in der Sim.
- **Ziel ohne Karte funktioniert:** `create_map`/`scene_bridge` laufen nur in
  der Sim, real gibt es kein `/map`. Dann publiziert der Planer eine **gerade
  Linie** von der aktuellen Pose zum RViz-Ziel („2D Goal Pose“ → `/g1pilot/goal`).
  Der Roboter läuft sie ohne Hindernisvermeidung ab (LiDAR-Punkte fließen nicht
  in die Planung). Fläche freihalten, Aufsicht mit Fernbedienung.
- **Kette:** Ziel → `dijkstra_planner` → `/g1pilot/path` → `nav2point` →
  `/g1pilot/auto_joy` → `joy_mux` (AUTO NAV an) → `/g1pilot/joy` →
  `loco_client` (`Move`, Limits `G1_MAX_VX/VY/VYAW`). Der Roboter muss im
  Walk-Zustand sein (LAUFEN); bleibt der Joy-Strom aus, stoppt `loco_client`
  nach 0,5 s.
- Das Koordinatensystem „map“ ist die MOLA-Startpose; Drift der LiDAR-Odometrie
  ist nicht vermessen.

## Worauf zu achten ist

- **Hindernisvermeidung nur für die geladene Umgebung, nicht spontan.** In
  der Simulation rastert die Karte die Objekte der geladenen Umgebung
  (Szenen-Brücke, siehe [31-T-Navigation.md](31-T-Navigation.md#scene_bridge-nur-sim)
  und [40-A-Simulation.md](40-A-Simulation.md#umgebungen))
  — spontane Hindernisse (Menschen, verschobene Objekte) werden **nicht**
  erkannt, es gibt keine Live-Perzeption. Auf echter Hardware ohne
  entsprechendes Setup bleibt die Karte leer. Nur in freier, kontrollierter
  Fläche einsetzen.
- **Auto ODER manuell, nicht beides gleichzeitig.** Bei aktivem `auto_enable`
  hat der Navigations-Joystick Vorrang; UI-/PS4-Joystick nicht gleichzeitig
  anfassen.
- **Deadman gilt weiter.** Fällt der Navigationsstream aus, greift derselbe
  Timeout-Mechanismus wie beim manuellen Gehen.
- **Konservative Geschwindigkeitslimits** gelten auch für die Autonomie.

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| Ziel gesetzt, aber kein Pfad | Liegt eine Pose vor? `ros2 topic echo /lidar_odometry/pose_fixed` — läuft die Lokalisierung (Sim: `sim_localization`; Real: MOLA/Livox)? |
| Pfad vorhanden, Roboter fährt nicht | `auto_enable` gesendet? Roboter laufbereit (Sim: WALK; Real: BALANCING)? |
| Roboter ruckelt / zwei Geschwindigkeitsquellen | Manuellen Joystick loslassen — bei aktiver Autonomie nicht gleichzeitig manuell fahren. |
| Fährt am Ziel vorbei / dreht unruhig | Regelparameter des Followers prüfen (siehe Technik-Dokument). |
| RViz zeigt den Roboter fest im Ursprung | Nur bei aktiver Navigation bewegt sich das TF-Modell; Fixed Frame in RViz muss `map` sein. |
| „Fährt in Hindernis" | Läuft die Szenen-Brücke? Objekt in der geladenen Umgebung enthalten? Ohne eigene Umgebung bzw. auf real ist die Karte weiterhin leer — Fläche freihalten. |

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

Die Navigation läuft **nur in der Simulation**; die Pose kommt dort aus der
MuJoCo-Ground-Truth. Die frühere LiDAR-Anbindung für den echten G1 (Livox
MID360 + MOLA) wurde verworfen und am 2026-10-30 entfernt.

## Voraussetzungen

- **Simulation:** Navigation beim Start aktivieren (Startmenü → Simulation →
  Ausstattung → „Navigation“, oder `G1_ENABLE_NAV=1 ./start.sh --yes`).
- RViz wird bei aktiver Navigation automatisch mitgestartet (Nav-Ansicht,
  Fixed Frame `map`) — es gibt kein eigenes Navigationsfenster.

## Bedienung

**1. Roboter laufbereit machen**

```bash
ros2 topic pub --once /g1pilot/start_balancing std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /g1pilot/start_walking   std_msgs/msg/Bool "{data: true}"   # nur Sim nötig
```

Demo-GUI: Kachel **GEHEN** (dort sitzen AUTO NAV und die Stations-Knöpfe).
Erst danach reagiert
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
währenddessen gesperrt). Per Kommandozeile:

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
immer: GREIFEN. Harter Halt: NOT-HALT
([51-A-Notaus.md](51-A-Notaus.md)).

## Echter Roboter

Auf dem echten G1 gibt es keine Navigation: ohne LiDAR fehlt eine Pose-Quelle
(`/lidar_odometry/pose_fixed`). Die Vorarbeiten dazu (Analyse
`lidar_autonav_real_g1.md` in den Projekt-Analysen, PR #28 `lidar_test`)
wurden nicht weiterverfolgt.

## Worauf zu achten ist

- **Hindernisvermeidung nur für die geladene Umgebung, nicht spontan.** In
  der Simulation rastert die Karte die Objekte der geladenen Umgebung
  (Szenen-Brücke, siehe [31-T-Navigation.md](31-T-Navigation.md#scene_bridge-nur-sim)
  und [40-A-Simulation.md](40-A-Simulation.md#umgebungen))
  — spontane Hindernisse (Menschen, verschobene Objekte) werden **nicht**
  erkannt, es gibt keine Live-Perzeption.
- **Auto ODER manuell, nicht beides gleichzeitig.** Bei aktivem `auto_enable`
  hat der Navigations-Joystick Vorrang; UI-/PS4-Joystick nicht gleichzeitig
  anfassen.
- **Deadman gilt weiter.** Fällt der Navigationsstream aus, greift derselbe
  Timeout-Mechanismus wie beim manuellen Gehen.
- **Konservative Geschwindigkeitslimits** gelten auch für die Autonomie.

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| Ziel gesetzt, aber kein Pfad | Liegt eine Pose vor? `ros2 topic echo /lidar_odometry/pose_fixed` — läuft `sim_localization`? (Der Topic-Name stammt noch aus der LiDAR-Zeit.) |
| Pfad vorhanden, Roboter fährt nicht | `auto_enable` gesendet? Roboter laufbereit (GEHEN)? |
| Roboter ruckelt / zwei Geschwindigkeitsquellen | Manuellen Joystick loslassen — bei aktiver Autonomie nicht gleichzeitig manuell fahren. |
| Fährt am Ziel vorbei / dreht unruhig | Regelparameter des Followers prüfen (siehe Technik-Dokument). |
| RViz zeigt den Roboter fest im Ursprung | Nur bei aktiver Navigation bewegt sich das TF-Modell; Fixed Frame in RViz muss `map` sein. |
| „Fährt in Hindernis" | Läuft die Szenen-Brücke? Objekt in der geladenen Umgebung enthalten? Ohne eigene Umgebung ist die Karte leer. |

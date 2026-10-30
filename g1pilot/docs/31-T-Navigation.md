# Navigation — Technik

Richtet sich an: Entwickler, die den Navigations-Stack ändern oder erweitern
wollen. Für die Bedienung siehe
[31-A-Navigation.md](31-A-Navigation.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/navigation/scene_bridge.py` | Umgebungsobjekte (Sim, per UDP) → `/scene_markers` |
| `g1pilot/navigation/scene_markers.py` | gemeinsame Logik des Weltmodells `/scene_markers` (Objektarten, Stationen `station_<Name>`, AABB) für Nav, IK-Kollision und Demo-GUI |
| `g1pilot/navigation/create_map.py` | `/scene_markers` → 2D-Belegungskarte `/map` |
| `g1pilot/navigation/dijkstra_planner.py` | globaler Pfadplaner |
| `g1pilot/navigation/nav2point.py` | Pure-Pursuit-Waypoint-Follower |
| `g1pilot/navigation/sim_localization.py` | Pose aus der MuJoCo-Ground-Truth |
| `g1pilot/teleoperation/joy_mux.py`, `navigation/joy_to_cmdvel.py` | Kopplung an die Loco-Schicht |

## Datenfluss

```
   Ziel  /g1pilot/goal (PoseStamped, map)
     │
     ▼
 ┌───────────────────┐      /map (OccupancyGrid)        ┌──────────────────────┐
 │ dijkstra_planner  │◄──── create_map ──────────────────│  Pose                │
 │ Grid-Dijkstra +   │                                   │  /lidar_odometry/    │
 │ Inflation + LOS-  │◄──────────────────────────────────│  pose_fixed          │
 │ Shortcut + Glättung│                                  │                      │
 └─────────┬─────────┘                                   │ sim_localization      │
           │ /g1pilot/path                               │  (rt/sportmodestate)  │
           ▼                                              │                      │
 ┌───────────────────┐                                    └──────────┬───────────┘
 │ nav2point         │◄────────────── Pose ──────────────────────────┘
 │ Pure-Pursuit-     │
 │ Waypoint-Follower │────► /g1pilot/auto_joy  (virtueller Joystick, Deadman-Btn 8)
 └───────────────────┘                    │
                                          ▼
                         ┌────────────────────────────┐   /g1pilot/auto_enable
                         │ joy_mux (Auto/Manuell-Mux)  │◄── (scharfschalten)
                         └──────────────┬──────────────┘
                                        │ /g1pilot/joy
                       ┌────────────────┴─────────────────┐
                       ▼ Sim                               ▼ Real
             ┌───────────────────┐                ┌──────────────────┐
             │ joy_to_cmdvel     │                │ loco_client      │
             │ Joy → loco_cmd_vel│                │ Move()           │
             └─────────┬─────────┘                └──────────────────┘
                       ▼
             ┌───────────────────┐
             │ loco_sim (Policy) │
             └───────────────────┘
```

## Node für Node

### `scene_bridge` (nur Sim)

Empfängt die Umgebungsobjekte (Hindernisse + greifbare Objekte) per UDP vom
MuJoCo-Container (`scene_state_publisher.py` in `unitree_mujoco`, Sender) und
veröffentlicht sie als `visualization_msgs/MarkerArray` auf
`/scene_markers` — QoS `TRANSIENT_LOCAL`, damit ein später gestarteter
Abnehmer sofort den letzten Stand bekommt. Grund für UDP statt ROS-Topic:
der MuJoCo-Container hat kein ROS; beide Container laufen mit
`network_mode: host`, Loopback verbindet sie ohne DDS/ROS.

`/scene_markers` ist das **eine geteilte Weltmodell**:

- RViz zeigt es direkt an.
- `create_map` rastert daraus die 2D-Karte.
- `arm_controller`/`ik_solver` speisen daraus die Umgebungskollision
  (siehe [20-T-Arm-Manipulation.md](20-T-Arm-Manipulation.md)).

### `create_map`

Rastert die aus `/scene_markers` gemeldeten Objekte (Hindernisse **und**
Greif-Objekte — ein greifbares Objekt am Boden ist bis zum Greifen trotzdem
ein Hindernis für die laufende Basis) in eine `OccupancyGrid` (`/map`,
Default 100×100 Zellen à 0,1 m, zentriert im Ursprung). Auf echter Hardware
ohne entsprechendes Setup bleibt die Karte leer (kein `/scene_markers`-Sender).

### `dijkstra_planner`

Klassischer Grid-Dijkstra mit:

- **Inflation** (`inflation_radius_m`, Default 0.30 m) — Sicherheitsabstand
  um belegte Zellen.
- **Zielanfahrt** (`goal_approach_m` 0.60 m, `goal_min_clearance_m` 0.10 m) —
  die letzten 0.6 m vor dem Ziel dürfen in den Sicherheitsabstand hinein, aber
  nie näher als 0.10 m an ein Objekt. Nur so sind Stationen direkt am Tisch
  (z. B. `station_Arbeitsplatz_2`, 0.25 m vor der Kante) erreichbar.
- **Start/Ziel im Sicherheitsabstand** werden auf die nächste freie Zelle
  geschoben; ohne kollisionsfreien Pfad wird **kein** Pfad publiziert (früher:
  gerade Linie durch die Objekte).
- **Turn-Cost** (`turn_cost_gain`) — bevorzugt gerade Wege gegenüber
  Zickzack.
- **Line-of-Sight-Shortcut** (`shortcut_path`) — entfernt unnötige
  Zwischenpunkte, wenn die direkte Sichtlinie frei ist.
- **Catmull-Rom-Glättung** (`_catmull_rom_centripetal`) — glättet den
  Rasterpfad zu einer weichen Kurve.

Ohne gültige Karte oder bei Start/Ziel außerhalb bzw. in einer belegten
Zelle fällt der Planer auf eine direkte, geglättete Linie zurück
(`line_points`). Ausgabe: `/g1pilot/path`.

### `nav2point`

Pure-Pursuit-Follower: zielt auf einen Punkt in fixem Vorlauf-Abstand
(`lookahead`) auf dem Pfad, regelt Position (`pos_kp`) und Ausrichtung
(`yaw_kp`) proportional, transformiert das Ergebnis ins Roboter-Koordinatensystem
und kodiert es als `sensor_msgs/Joy` mit gedrücktem Deadman-Button (Index 8)
auf `/g1pilot/auto_joy` — als ob ein Mensch den Joystick hielte.

Endphase (`final_align`): erreicht der Roboter die Zieltoleranz
(`goal_tolerance`), schaltet der Node in eine Ausrichtungsphase
(`aligning=True`) um: keine Translation mehr (verhindert Umkreisen), nur
noch Drehen auf den mitgelieferten Ziel-Yaw (`yaw_tol_deg` Toleranz). Ab
`yaw_hold_dist` vor dem Ziel wird das kontinuierliche Nachdrehen zum
aktuellen Wegpunkt abgeschaltet (Orbit-Fix) — sonst würde der Roboter
knapp vor dem Ziel um den (dann instabilen) Punkt kreisen.

Zielgenauigkeit (fuer Stationen/Greifabläufe wichtig):

- `goal_tolerance` 0.05 m; der Planer legt das Pfadende exakt aufs Ziel
  (nicht auf die Zellmitte).
- Ab `yaw_hold_dist` dreht der Roboter auf den **Ziel-Yaw** (nicht mehr zum
  Pfadpunkt) und gleitet holonom hinein.
- `min_axis` 0.35 / `min_yaw_axis` 0.25: Mindest-Ausschlag, weil `loco_sim`
  bei ‖cmd‖ < `stand_eps` (0.1) nur steht — sonst blieb der Roboter 10–20 cm
  vor dem Ziel stehen, die Endausrichtung startete nie (Folge: schräg, z. B. 45°).
- Kein Fortschritt `stall_s` (3 s) innerhalb `stall_dist` (0.35 m) →
  Endausrichtung trotzdem.

Fortschritt meldet der Node als `std_msgs/String` auf `/g1pilot/nav_status`
(TRANSIENT_LOCAL, Ereignisse):

| Wert | Bedeutung |
|---|---|
| `idle` | seit dem Start noch kein Ziel |
| `moving` | neuer Pfad empfangen; läuft, sobald `auto_enable` an ist |
| `arrived` | Ziel inkl. Endausrichtung erreicht, Pfad verworfen |
| `no_path` | Planer fand keinen Weg (leerer Pfad), Roboter wurde angehalten |

`dijkstra_planner` publiziert bei Fehlschlag (keine Pose, Start/Ziel
blockiert, kein kollisionsfreier Weg) einen **leeren Pfad**. `nav2point` hält
dann an, statt dem vorherigen Pfad weiter zu folgen. Die Demo-GUI nutzt
`nav_status` für die AUTO-NAV-Anzeige (siehe `10-A-Demo-GUI.md`).

### `sim_localization`

`sim_localization` liest die Basis-Position/-Geschwindigkeit aus
`rt/sportmodestate` (MuJoCo-Ground-Truth) und die Orientierung aus der
IMU in `rt/lowstate`, und veröffentlicht daraus exakt die Topics, die der
Rest des Stacks erwartet: `/lidar_odometry/pose_fixed`
(`nav_msgs/Odometry`, Frame `map`) und die dynamische TF
`odom_unitree → base_link`. Der Topic-Name `/lidar_odometry/…` stammt aus
der früheren LiDAR-Anbindung (MOLA/Livox, am 2026-10-30 entfernt) und ist
geblieben, damit Planer und Follower unverändert bleiben.

### `joy_mux` / `joy_to_cmdvel`

Siehe [13-T-Alter-Joystick.md](13-T-Alter-Joystick.md). Auf real
liest `loco_client` `/g1pilot/joy` direkt; in der Simulation liest
`loco_sim` keinen `Joy`, daher übersetzt `joy_to_cmdvel` das (bereits durch
`joy_mux`/`auto_enable` gegatete) `/g1pilot/joy` in `/g1pilot/loco_cmd_vel`
(mit Watchdog: bleibt der Strom aus, wird einmalig 0 gesendet).

## Warum der Umweg über einen „Fake-Joystick"?

Damit die Autonomie exakt denselben, erprobten Pfad nimmt wie die manuelle
Teleoperation — inklusive Deadman, Geschwindigkeitslimits und Notaus. Es
gibt keinen zweiten, ungetesteten Steuerweg zum Roboter.

## Nur Simulation

Der Stack läuft nur in der Simulation (`G1_ENABLE_NAV=1`). Auf dem echten G1
fehlt eine Pose-Quelle; die LiDAR-Anbindung (Livox MID360 + MOLA) wurde
verworfen. Die Kopplung über `joy_mux` → `/g1pilot/joy` würde real
grundsätzlich auch `loco_client` erreichen, das ist aber nicht mehr
vorgesehen.

## Bekannte Einschränkungen

- Keine Live-Perzeption: die Karte spiegelt nur die beim Sim-Start geladene
  Umgebung, keine spontanen Hindernisse.
- Kein Betrieb auf echter Hardware (keine Lokalisierung).

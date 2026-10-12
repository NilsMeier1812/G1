# Arm-Manipulation — Technik

Richtet sich an: Entwickler, die an der Armregelung, IK oder dem
Positionsspeicher etwas ändern wollen. Für die Bedienung siehe
[10_arm_manipulation_anleitung.md](10_arm_manipulation_anleitung.md), für die
HTTP-Brücke siehe [21_arm_api_technik.md](21_arm_api_technik.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/manipulation/arm_controller.py` | Hauptregler-Node (250 Hz), IK-Anwendung, Sicherheitsgates, State-Machine |
| `g1pilot/utils/ik_solver.py` | `G1IKSolver`: Pinocchio-basierte IK, Selbstkollisionsprüfung (Meshes) |
| `g1pilot/manipulation/arm_geometry.py` | Kugelmodell der Arme, Abstandsfunktion zu Objekten (Planer + Laufzeit-Gate) |
| `g1pilot/manipulation/arm_planner.py` | Eigener Bahnplaner (Optimierung mit ortsabhängigem Abstand, RRT-Rückfall, Zeitparametrierung) |
| `g1pilot/manipulation/arm_planner_process.py` | Planer als eigener Prozess (Zeitlimit, Abbruch, Neustart) |
| `g1pilot/manipulation/pose_store.py` | Dateibasierte Ablage gespeicherter Posen |
| `g1pilot/manipulation/arm_command.py` | Wire-Format der Live-Pose-Schnittstelle (gemeinsam mit `arm_api.py`) |
| `g1pilot/manipulation/interactive_marker.py` | RViz-Marker, Leader-Follower-Verhalten; im WALK ausgeblendet, bei BALANCING an der Hand-TF neu erzeugt |
| `g1pilot/utils/joints_names.py` | Gelenk-Indizes, -Limits, -Namen (einzige Quelle der Wahrheit) |

## Node: `arm_controller`

Läuft mit fester Taktrate (`rate_hz`, Default 250 Hz) über
`self.timer = self.create_timer(1.0/rate_hz, self.main_loop)`. Schreibt bei
`use_robot=true` direkt auf `rt/arm_sdk` (DDS), sonst auf `/joint_states`
(ROS, für reine RViz-Anzeige ohne Roboter/Sim).

### Zustandsmaschine

Der Node hält mehrere unabhängige, aber interagierende Zustände:

- `arms_enabled` — Manipulation aktiv/inaktiv (`/g1pilot/arms/enabled`).
- `estop_active` — Notaus-Latch (`/g1pilot/emergency_stop`), blockiert
  `ENABLE`, bis `/g1pilot/start` quittiert.
- `_arm_slack` — Arme drehmomentfrei/gedämpft gehalten, ohne die Kontrolle
  an den Onboard-Regler zurückzugeben (siehe unten, „E-Stop").
- `homing_active` / `homing_reached` — Fahrt zur Home-Pose.
- `walk_mode` — während `WALK` (siehe [31_loco_technik.md](31_loco_technik.md))
  hält der Controller die Arme in einer definierten Lauf-Pose, damit die
  Lauf-Policy stabil bleibt.
- `_walk_transition` — der Weg **in** die Lauf-Pose läuft beim WALK-Start
  als geplante Bewegung (wie Homing, um Tisch/Hindernisse herum). Erst danach
  hält der WALK-Zweig die Pose und meldet `walk_ready`. Findet der Planer
  keinen Weg, fährt der Controller wie früher direkt (das Gate hält bei
  Kollision an). Marker-Bewegung, ein ignoriertes HOMING und POSE ABBRECHEN
  brechen den Übergang nicht ab; BALANCING, E-Stop und DISABLE schon.
- `_planned_motion_active` — eine geplante Bewegung (Positionsspeicher /
  Live-Kommando) fährt gerade eine vorab berechnete Wegpunktliste ab.

`_arms_not_ready_reason()` ist die **einzige** Stelle, die prüft, ob jetzt
eine neue Bewegung starten darf (E-Stop, Enable, Homing, Walk, laufende
Planung) — sowohl der Positionsspeicher als auch die Live-Pose-Schnittstelle
(`arm_command`) rufen sie auf, damit die Schranken nicht auseinanderlaufen.

### Reaktives Servoing (Marker-Pfad)

`_right_goal_callback` / `_left_goal_callback` (Subscriber auf
`/g1pilot/hand_goal/{left,right}`, `PoseStamped`):

1. Pose in den IK-World-Frame transformieren (TF, Default `pelvis`).
2. `_apply_offsets_and_filters`: statischer End-Effektor-Offset (Parameter
   `ee_offset_*`), optional automatische Kalibrierung
   (`_gate_auto_calibration`, greift, wenn das eingehende Ziel nahe an der
   aktuellen Handpose liegt — richtet Marker und Ist-Hand einmalig
   aufeinander aus), Tiefpassfilterung (`ik_goal_filter_alpha`),
   Orientierungs-Schrittbegrenzung (`ik_max_ori_step_rad`).
3. `ik_solver.set_goal(side, T_goal_use)` — das eigentliche Lösen passiert im
   `main_loop`.

Ein Marker-Griff bricht **immer** eine laufende geplante Bewegung ab
(`_abort_planned_motion("Marker bewegt")`) — manuelle Eingabe hat Vorrang.

### `main_loop()` — pro Tick

Reihenfolge (vereinfacht):

1. Workspace-Marker publizieren (rein visuell).
2. `_poll_plan_result()` — prüft billig (Lock + Identitätsvergleich), ob ein
   Hintergrund-Planungsthread fertig geworden ist.
3. Ist E-Stop/Slack aktiv und Manipulation nicht aktiv → `_publish_arm_slack()`
   und zurück (siehe unten).
4. Ist Manipulation deaktiviert → ggf. Gewichtsrampe abwärts fahren, sonst
   still sein.
5. Zielkonfiguration `q_target` je nach aktivem Modus bestimmen: `walk_mode`
   ohne `_walk_transition` (Lauf-Pose halten) > `homing_active`/`homing_reached`
   (Home-Pose) > `_planned_motion_active` (Sollwert der zeitparametrierten
   Bahn zur aktuellen Bahnzeit) > `_walk_transition` (Weg in die Lauf-Pose wird noch geplant →
   stillhalten) > sonst IK-Ergebnis aus dem reaktiven Servoing
   (`ik_solver.get_joint_targets`).
6. Geschwindigkeitslimit (`arm_velocity_limit`) + exponentielle Glättung
   (`ik_alpha`).
7. Kartesisches Geschwindigkeitslimit (`_limit_cartesian_speed`, überwacht
   Hand-TCPs + Ellbogen).
8. Kollisions-Gate (`_apply_collision_gate`), siehe unten „Laufzeit-Gate".
9. Schreiben: bei `use_robot=true` `rt/arm_sdk` mit
   Schwerkraft-Feedforward-Drehmoment (`_arm_gravity_tau`,
   Parameter `gravity_comp`), sonst `/joint_states`.

### `rt/arm_sdk`-Gewichtsrampe

`motor_cmd[29].q` (der `kNotUsedJoint0`-Slot) trägt das Blend-Gewicht 0..1,
mit dem die Bridge (Sim) bzw. die G1-Firmware (Real) zwischen
Onboard-Steuerung und `arm_sdk`-Kommando mischt. Beim `ENABLE` rampt das
Gewicht über `arm_weight_ramp_up_s` von 0 auf 1, beim `DISABLE` symmetrisch
zurück (`_advance_arm_weight`, `_publish_weight_ramp_down`). Die Sim setzt
beide Rampenzeiten auf 0.0 (sofortiges Umschalten, im Bridge-Merge auf 0/1
getestet); der reale Roboter nutzt 2.0 s (weiche Übergabe, sonst reißt die
Steuerungsübernahme an den Armen).

### E-Stop-Semantik (`_on_emergency_stop` / `_publish_arm_slack`)

Kritisch: Beim E-Stop bleibt das `arm_sdk`-Gewicht auf **1** — würde es auf 0
gesetzt, übernähme der Onboard-Regler die Arme und führe sie *aktiv* mit
voller Geschwindigkeit in seine Default-Pose (ein Sprung). Stattdessen
schreibt `_publish_arm_slack()` weiterhin auf `rt/arm_sdk`, aber mit
`kp=0, tau=0, kd=estop_arm_kd` — die Arme werden drehmomentfrei und sacken
gedämpft, statt aktiv irgendwohin zu fahren. `/g1pilot/start` quittiert nur
den Latch (`estop_active`); die Arme bleiben schlaff, bis
`ENABLE MANIPULATION` die Kontrolle bewusst zurückholt.

### Positionsspeicher (plan-execute)

`_on_pose_save` / `_on_pose_goto` / `_on_pose_cancel` (Topics
`/g1pilot/pose_store/{save,goto,cancel}`).

- **Speichern:** schreibt die zuletzt *kommandierte* Zielkonfiguration
  (`_last_q_target`, nicht die verrauschte Messung) plus optional den
  zuletzt gemeldeten Fingerzustand (`_hand_state`, kommt von der
  Inspire-Bridge) in `PoseStore`. Rückkanal:
  `/g1pilot/pose_store/save/status` (JSON, siehe `arm_command.save_status_message`).
- **Anfahren:** `_start_planned_motion()` — gemeinsamer Einstiegspunkt für
  Positionsspeicher UND die Live-Pose-Schnittstelle. Bricht eine
  eventuell laufende ältere Bewegung sauber ab, erhöht die
  „Planungs-Generation" (`_plan_gen`) und startet `_plan_pose_worker` in
  einem **Hintergrund-Thread** (Planung kann Sekunden dauern, darf den
  250-Hz-Regelkreis nie blockieren).
- `_plan_pose_worker` (Hintergrund-Thread) schickt die Anfrage an den
  **Planer-Prozess** (`arm_planner_process.PlannerProcess`) und wartet dort
  blockierend (gibt das GIL frei). Kartesische Ziele löst der Prozess zuerst
  per IK (`ik_solver.solve_pose`). Ergebnis (zeitparametrierte Trajektorie
  `t`, `q`) geht über `DataBuffer` zurück.
- `_poll_plan_result()`: aktiviert nur Ergebnisse der **aktuellen**
  Generation — ein Abbruch während der Planung (E-Stop, neuer Marker-Griff,
  DISABLE) entwertet ein später eintreffendes Ergebnis race-frei und bricht
  die Planung im Prozess ab (`PlannerProcess.cancel`).
- Ein/beide Arme werden **gemeinsam** als ein 7- bzw. 14-DOF-Problem
  geplant (Arm-gegen-Arm an jedem Zwischenzustand geprüft) und über **eine
  gemeinsame Bahnuhr** synchron abgefahren (`_planned_time`,
  `_traj_sample`). Die Uhr läuft mit der echten Zeit, aber nur, solange das
  Kommando der Bahn folgt (Abweichung < 0,05 rad) — hält das Laufzeit-Gate
  an, steht auch die Bahn. Steht sie länger als 3 s, wird die Bewegung mit
  `failed` („blockiert") beendet. Beim Abschluss loggt der Controller
  Bahnzeit, tatsächliche Fahrzeit, Haltezeit und Regeltakt.

### `pose_store.py`

Dateiformat Version 2:

```json
{"version": 2,
 "categories": ["Allgemein", "Greifen"],
 "poses": {"<name>": {"category": "...", "left_arm": [...], "saved_at": 0.0}}}
```

Speicherbare Komponenten: `left_arm`, `right_arm`, `left_hand`, `right_hand`
(`COMPONENTS`-Tupel). Speicherpfad-Priorität: `G1_POSE_STORE`-Env-Variable
> `<repo>/data/arm_poses.json` (bind-gemountet, überlebt Container-Neustarts)
> `~/.g1pilot/` (Host-Betrieb ohne Mount). Schreiben ist atomar
(`write-tmp-then-rename`), Version-1-Dateien (flaches Format ohne
Kategorien) werden transparent migriert.

## `G1IKSolver` (`utils/ik_solver.py`)

Pinocchio-basierter, gedämpfter Pseudo-Inverse-Löser mit:

- adaptiver Dämpfung nahe Singularitäten (`adaptive_damping`,
  `sigma_min_thresh`),
- Nullraum-Regularisierung zu einer Ruhepose (`null_space_gain`,
  `set_rest_posture`) — hält den redundanten 7. Freiheitsgrad (Ellbogen-
  Swivel) natürlich, damit der Arm beim Marker-Ziehen nicht in den zum
  Körper geklappten IK-Zweig fällt,
- Selbstkollisions-Gate (Convex-Hulls, konfigurierbare Marge,
  `arm_command_in_collision`),
- `solve_pose(side, T_goal, current_all)` — Einzelaufruf-Variante (kein
  gemeinsamer Solver-Zustand), genutzt vom Planungs-Thread, damit dieser
  nicht mit dem reaktiven 250-Hz-Servoing um denselben mutablen
  Pinocchio-Zustand konkurriert.

## Kugelmodell (`arm_geometry.py`)

Grundlage von Planer und Laufzeit-Gate gegen die Umgebung. Beim Start
werden aus den Kollisions-Meshes der Arme Kugeln angepasst (k-means-
Überdeckung, `fit_spheres`):

- **fein** für den Abstand zu Objekten: Hand (Handballen + Finger in
  Neutralstellung) Radius ≤ 2,2 cm, Arm ≤ 4 cm, 87 Kugeln je Arm,
- **grob** (≤ 5 cm, ab Ellbogen, 16 Kugeln) für die Selbstkollision im Planer,
- Körper (Rumpf, Becken, Beine) ~39 Kugeln ≤ 9 cm.

Die Vorwärtskinematik der 7er-Ketten ist vektorisiert (viele
Konfigurationen auf einmal, gegen pinocchio geprüft). Objekte aus
`/scene_markers` sind orientierte Boxen (`Environment`); `env_dist` liefert
den **vorzeichenbehafteten Abstand** jeder Kugel zu jeder Box samt
Gradient. `reachable()` sortiert Objekte aus, die ein Arm von der Schulter
aus gar nicht erreichen kann (Arbeitsplatz-Szene: 44 Teile → 4-8).

Hand-Kugeln ignorieren **Greif-Objekte** (Klasse `grasp`, gewollt
berührbar, ohne Unterscheidung nach Zielobjekt); Arm-Kugeln nicht.

## `arm_planner.py`

Eigenes System (kein MoveIt/OMPL). Leitidee: **in der Mitte der Bewegung
Abstand, an Start und Ziel darf es eng werden** — die Hand darf am Ende auf
dem Tisch liegen und am Anfang dort starten, ohne dass die Bahn deshalb
erst weit weg, dann zurück und zum Schluss auf einer Geraden fährt.

**Abstandsbedarf je Kugel, Objekt und Bahnpunkt** (`_Problem.requirement`):

    w     = smoothstep( min(|c - c_start|, |c - c_ziel|) / relax_radius )
    boden = min(0, Abstand derselben Kugel am näheren Endpunkt)
    bedarf = boden + w · (clearance - boden)

Um Start und Ziel jeder Armkugel liegt also ein glatter „Trichter"
(`relax_radius`, Default 15 cm): dort reicht, was die Endpose selbst hat
(Berührung, sogar leicht drin), weiter weg gilt `clearance` (Default 5 cm).
Da der Bedarf stetig ist, entsteht keine Knickstelle in der Bahn.

**Optimierung** (CHOMP-artig, `_Problem.optimize`): 24 Stützpunkte im
Gelenkraum, Kosten = Glattheit (Geschwindigkeit + Beschleunigung) +
TCP-Weglänge + quadratische Strafe für Unterschreitung des Bedarfs
(Umgebung, Körper, Arm-gegen-Arm) + Anfahr-/Abfahrrichtung. Vorkonditioniert
mit der inversen Glattheitsmatrix (Schritte bleiben glatt), Trust-Region,
Backtracking, Strafgewicht-Fortsetzung (×5 bis zu 4×), es gewinnt die beste
gültige Iteration.

**Anfahrrichtung:** liegt das Ziel nahe an einem Hindernis (Tisch), soll
die Hand auf den letzten `approach_len` (10 cm) entlang der Handachse
ankommen; ist das nicht möglich, von oben. Abfahrt vom Start analog.

**Rückfallkette:** (1) Gerade im Gelenkraum direkt gültig → fertig.
(2) Optimierung von mehreren Startbahnen (gerade, angehoben,
zurückgezogen). (3) Eigener vektorisierter RRT-Connect als Startbahn, dann
Glättung mit schrittweise erhöhtem Abstand. (4) Reduzierter Abstand
(×0,6 bis `min_clearance`) und Startbahn über die Lauf-Pose. Ziel in harter
Selbstkollision oder > 2 cm in einem Objekt → `goal_in_collision`.

**Abschlussprüfung:** dicht nachabgetastet (Kugelmodell) plus exakte
Mesh-Selbstkollision (coal).

**Zeitparametrierung** (`time_parameterize`): Gelenk- und kartesische
Geschwindigkeits-/Beschleunigungsgrenzen, nahe an Objekten langsamer
(`slow_dist`, `slow_factor`), Abtastung 0,02 s.

**Zeitbudget:** `time_budget_s` (2,5 s) als **Rechenzeit** des
Planer-Prozesses, plus Wandzeit-Obergrenze 3× Budget. Typisch 20-500 ms
(Arbeitsplatz-Szenen), schwierige Fälle mit RRT ~2,5 s.

### Planer-Prozess (`arm_planner_process.py`)

Reines Python/numpy — im Controller-Prozess hielte es das GIL und der
250-Hz-Regelkreis bekäme Aussetzer. Deshalb eigener Prozess (`spawn`), der
beim Start dasselbe URDF lädt (IK + Kugelmodell). Ein noch ladender Prozess
wird nicht abgebrochen (auf einem ausgelasteten Rechner dauert das Laden
deutlich länger); eine Anfrage in dieser Zeit endet mit „Planer startet
noch". Hängt oder stirbt er, wird er nach dem Zeitlimit beendet und beim
nächsten Aufruf neu gestartet. `cancel()` setzt ein geteiltes Event, das
der Planer in seinen Schleifen prüft.

## Laufzeit-Gate (`_apply_collision_gate`)

Pro Tick auf dem kommandierten Schritt, für Marker-Steuerung UND geplante
Bahn:

- **Umgebung (Kugelmodell, Abstände):** ein Paar (Armkugel, Objekt) unter
  seiner Untergrenze darf sich **nicht weiter annähern** — alles andere ist
  erlaubt. Untergrenze: Hand `gate_hand_floor` (0 = aufsetzen erlaubt),
  Arm `gate_arm_floor` (1 cm), Hand gegen Greif-Objekte keine. Während einer
  geplanten Fahrt zusätzlich so tief wie Start bzw. Ziel der Bahn. Ein
  verletzender Schritt wird auf die erlaubte Richtung **projiziert** — die
  Hand gleitet an der Oberfläche entlang, statt einzufrieren. Ein Start
  dicht am oder minimal im Tisch blockiert damit keine Bewegung mehr (auch
  nicht die Marker-Steuerung). Marker-Steuerung wird unter `gate_slow_dist`
  (5 cm) verlangsamt (bis 25 %), damit die Hand sanft aufsetzt. Ein
  billiger Vorab-Test (`_env_near`) überspringt die volle Prüfung, wenn
  nichts in der Nähe ist.
- **Selbstkollision (exakte Meshes, coal):** hart → halten; im Margenband
  (3 cm) nur geplante Bahn oder Herausfahren.

## Konfiguration (Auszug, vollständige Liste in `manipulation_launcher.launch.py`)

| Parameter | Bedeutung |
|---|---|
| `rate_hz` | Regeltakt des `main_loop` (Default 250) |
| `ik_alpha`, `ik_goal_filter_alpha` | Glättung Servoing-Pfad |
| `ik_orientation_mode` | `full` / weitere Modi, siehe `set_orientation_mode` |
| `ik_null_space_gain` | Stärke der Nullraum-Regularisierung |
| `planned_motion_tolerance` | Toleranz [rad], ab der das Bahnende als erreicht gilt |
| `planner_clearance` / `planner_min_clearance` | Abstand in der Bahnmitte (5 cm) / Untergrenze beim Reduzieren (1,5 cm) |
| `planner_relax_radius` | Größe des Trichters um Start und Ziel (15 cm) |
| `planner_time_budget_s` | Rechenzeit-Budget einer Planung (2,5 s) |
| `planner_vel_limit` / `planner_cart_vel_limit` | Tempo der geplanten Fahrt (rad/s, m/s) |
| `gate_hand_floor` / `gate_arm_floor` / `gate_slow_dist` | Laufzeit-Gate, siehe oben |
| `arm_api_*` | Parameter der HTTP-Brücke, siehe [21_arm_api_technik.md](21_arm_api_technik.md) |

## Bekannte Einschränkungen

- Der IK-Solve kostet bei aktiv verfolgtem Marker-Ziel ~20-30 ms/Zyklus —
  der Regel-Loop läuft dann effektiv mit ~30-50 Hz statt 250 Hz. Alle
  Limits rechnen mit dem tatsächlichen `dt`, bleiben also korrekt; im
  Halte-Zustand läuft der Loop mit voller Rate.
- Die Taille (12-14) ist über `rt/arm_sdk` real ansteuerbar, wird von
  `arm_controller` aber bewusst nie beschrieben — ein Kommando dort würde
  gegen den Unitree-Loco-Controller arbeiten.

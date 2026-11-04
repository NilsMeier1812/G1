# Arm-Manipulation — Technik

Richtet sich an: Entwickler, die an der Armregelung, IK oder dem
Positionsspeicher etwas ändern wollen. Für die Bedienung siehe
[20-A-Arm-Manipulation.md](20-A-Arm-Manipulation.md), für die
HTTP-Brücke siehe [21-T-Arm-API.md](21-T-Arm-API.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/manipulation/arm_controller.py` | Hauptregler-Node (250 Hz), IK-Anwendung, Sicherheitsgates, State-Machine |
| `g1pilot/utils/ik_solver.py` | `G1IKSolver`: Pinocchio-basierte IK, Selbstkollisionsprüfung (Meshes) |
| `g1pilot/manipulation/arm_geometry.py` | Kugelmodell der Arme, Abstandsfunktion zu Objekten (Planer + Laufzeit-Gate) |
| `g1pilot/manipulation/arm_planner.py` | Eigener Bahnplaner (Optimierung mit ortsabhängigem Abstand, RRT-Rückfall, Zeitparametrierung) |
| `g1pilot/manipulation/arm_planner_process.py` | Planer als eigener Prozess (Zeitlimit, Abbruch, Neustart) |
| `g1pilot/manipulation/pose_store.py` | Dateibasierte Ablage gespeicherter Posen, Abläufe, relative Bewegungen ([23-T-Posen-und-Ablaeufe.md](23-T-Posen-und-Ablaeufe.md)) |
| `g1pilot/manipulation/ik_servo_process.py` | Marker-/Jog-IK im eigenen Prozess (`ik_async`) |
| `g1pilot/manipulation/waist_yaw.py`, `waist_hold.py` | Hüftdrehung und Taille halten ([24-T-Hueftdrehung.md](24-T-Hueftdrehung.md)) |
| `g1pilot/manipulation/arm_command.py` | Wire-Format der Live-Pose-Schnittstelle (gemeinsam mit `arm_api.py`) |
| `g1pilot/manipulation/interactive_marker.py` | RViz-Marker, Leader-Follower-Verhalten; im WALK ausgeblendet (`hide_in_walk`, Sim: sichtbar), bei BALANCING an der Hand-TF neu erzeugt |
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
- `walk_mode` — während `WALK` (siehe [30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md))
  hält der Controller die Arme in einer definierten Lauf-Pose. Nur mit
  `walk_park_arms=true` (Node-Standard). Real setzt `bringup_real` es aus
  `G1_WALK_PARK_ARMS` (Default `false`, Nils 2026-10-19): die Arme bleiben beim
  LAUFEN in ihrer Pose, `_on_walk_mode` bricht eine laufende geplante Fahrt und
  Marker-Ziele ab und hält die Gelenke per `_align_ik_to_config` exakt an der
  kommandierten Stellung; `arm_sdk` behält Gewicht 1, die Taille bleibt
  gehalten. In der Sim setzt
  `bringup_sim` es passend zur Lauf-Policy: `true` bei der alten Policy
  `g1_wholebody` (Standard), `false` bei den AGILE-Policies. Die brauchen keine
  Armpose, `walk_mode` bleibt aus, die Arme halten ihre Stellung und bleiben
  bedienbar, und `/g1pilot/arms/walk_ready` kommt sofort (die Demo-GUI wartet
  darauf).
- `_walk_transition` (nur mit `walk_park_arms=true`) — der Weg **in** die Lauf-Pose läuft beim WALK-Start
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

Die beiden Ziel-Subscriptions haben Queue-Tiefe **1**: der Executor bedient
sie nur einmal pro `main_loop`-Takt. Kommen Ziele schneller als der Takt
(PS4-Controller/Demo-GUI mit 30 Hz, Sim-Takt auf einem schwachen PC
10–20 Hz), stauten sich mit Tiefe 10 alte Ziele — die Hand reagierte erst
0,3–0,8 s später. Jetzt zählt immer das neueste Ziel.

Ein Marker-Griff bricht **immer** eine laufende geplante Bewegung ab
(`_abort_planned_motion("Marker bewegt")`) — manuelle Eingabe hat Vorrang.

#### IK im eigenen Prozess (`ik_servo_process.py`, Parameter `ik_async`)

Die Servo-IK (60 DLS-Iterationen je Arm) kostet ~45 ms. Lief sie im
`main_loop`, kam der 250-Hz-Takt beim Joggen (PS4, Demo-GUI-Joystick,
RViz-Marker) nur auf ~20–40 Hz: der Motor bekam alle 25–50 ms einen
Sollwert-Sprung von 10–25 mrad, der Arm ratterte hörbar (Real-Test
2026-10-16). Geplante Fahrten und Home rechnen keine IK und liefen sauber.

Jetzt schickt der Takt je Tick den neuesten Auftrag (gefilterte Ziele +
Konfiguration) an einen eigenen Prozess (`IkServoProcess`; ein Thread reicht
nicht, die IK hält das GIL). Das letzte Ergebnis ist das Gelenkziel, zu dem
der Takt das Kommando mit allen Limits in kleinen Schritten fährt.
`_ik_gen` verwirft Ergebnisse nach ENABLE/Home/geplanter Fahrt. Bis der
Prozess geladen ist (oder falls er ausfällt), rechnet der Takt wie früher.
`ik_servo_step_tol` (2e-4 rad) beendet die Iteration am Fixpunkt (~15 statt 60
Iterationen, gleiches Ergebnis innerhalb weniger mrad). Rückfall: real
`G1_IK_ASYNC=0`, sonst `ik_async:=false` und `ik_servo_step_tol:=0.0`.
Messsonde ohne Hardware: `test/teleop_jitter_probe.py`.

#### Ziel-Rückmeldung an den Marker (`/g1pilot/hand_goal_status`)

Jedes Marker-Ziel wird je Seite verfolgt (`_track_marker_goal`), bis es
abgeschlossen ist. `_update_marker_goals` läuft im `main_loop` mit
`marker_goal_check_hz` (Default 25 Hz) und

- füttert den Ziel-Filter erneut mit dem Endziel (`auto_reissue_goals`) —
  sonst macht der Tiefpass nur einen Schritt pro eingehender Nachricht und der
  Sollwert bleibt nach dem Loslassen vor dem Marker stehen,
- meldet `reached`, sobald die FK der **kommandierten** Armstellung innerhalb
  `goal_pos_tol` / `goal_ori_tol_deg` liegt,
- meldet `unreachable`, wenn sich die kommandierte Armstellung
  `marker_goal_stall_time_s` lang um weniger als `marker_goal_stall_tol_rad`
  bewegt hat, ohne anzukommen (Reichweite, Kollisions-Gate, Gelenklimit).

Homing, WALK, BALANCING, E-Stop, Enable/Disable und geplante Bewegungen melden
offene Ziele als `cancelled`; Ziele, die wegen Homing/deaktivierter
Manipulation verworfen werden, als `ignored`. Format (JSON in
`std_msgs/String`):

```json
{"side": "right", "state": "reached", "goal_stamp": {"sec": 12, "nanosec": 345},
 "pos_err_m": 0.006, "ori_err_deg": 1.2}
```

`goal_stamp` ist der `header.stamp` der Ziel-Nachricht. Der RViz-Marker
(`interactive_marker.py`) bleibt nach dem Loslassen auf der Zielpose stehen und
folgt erst wieder der Hand, wenn für **sein zuletzt gesendetes** Ziel eine
Abschlussmeldung kommt (Sicherheitsnetz: `goal_timeout_s`, Default 30 s).
Beide Seiten sind unabhängig — linken Marker setzen, rechten Marker setzen,
beide Arme fahren gleichzeitig. Ein erneuter Griff während der Fahrt ersetzt
das Ziel einfach.

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
   Bahn zur aktuellen Bahnzeit) > `_releasing` (Rückgabe an den Onboard-Regler,
   siehe unten) > `_walk_transition` (Weg in die Lauf-Pose wird noch geplant →
   stillhalten) > sonst IK-Ergebnis aus dem reaktiven Servoing
   (`ik_solver.get_joint_targets`). Eine Seite, deren Ziel noch das per
   `_align_ik_to_config` gesetzte Halteziel ist, wird **ohne IK** exakt gehalten
   (`_ik_side_held`) — sonst drehte der Nullraum-Term den Ellbogen beim ENABLE
   in die Ruhe-Konfiguration.
6. Geschwindigkeitslimit (`arm_velocity_limit`) + exponentielle Glättung
   (`ik_alpha`; mit `smoothing_ref_hz` > 0 zeitbasiert, siehe Konfiguration).
7. Kartesisches Geschwindigkeitslimit (`_limit_cartesian_speed`, überwacht
   Hand-TCPs + Ellbogen).
8. Beschleunigungslimit (`_limit_accel`, nur real: `arm_accel_limit`,
   begrenzt nur das Schneller-Werden; Bremsen bleibt frei).
9. Kollisions-Gate (`_apply_collision_gate`), siehe unten „Laufzeit-Gate".
10. Schleppfehler-Begrenzung (`_limit_tracking_error`, nur real bei Gewicht 1:
    Sollwert höchstens `max_tracking_error` rad von der Messung).
11. Schreiben (`_write_arm_cmd`): bei `use_robot=true` `rt/arm_sdk` mit
   Schwerkraft-Feedforward-Drehmoment (`_arm_gravity_tau`,
   Parameter `gravity_comp`), sonst `/joint_states`.

Den LowState liest **nur** der Thread `_subscribe_motor_state`
(`lowstate_buffer`). Früher las der `main_loop` zusätzlich blockierend
`lowstate_subscriber.Read()`: jeder Tick wartete auf die nächste
LowState-Nachricht und nahm sie dem Thread weg (real ~2 ms je Tick, in der
Sim halbierte es die Regelrate).

### `rt/arm_sdk`-Gewichtsrampe

`motor_cmd[29].q` (der `kNotUsedJoint0`-Slot) trägt das Blend-Gewicht 0..1,
mit dem die Bridge (Sim) bzw. die G1-Firmware (Real) zwischen
Onboard-Steuerung und `arm_sdk`-Kommando mischt. Beim `ENABLE` rampt das
Gewicht über `arm_weight_ramp_up_s` von 0 auf 1, beim `DISABLE` symmetrisch
zurück (`_advance_arm_weight`, `_publish_weight_ramp_down`). Die Sim setzt
beide Rampenzeiten auf 0.0 (sofortiges Umschalten, im Bridge-Merge auf 0/1
getestet); der reale Roboter nutzt 2.0 s (weiche Übergabe, sonst reißt die
Steuerungsübernahme an den Armen).

### Übergabe an den Onboard-Regler (nur Echtbetrieb relevant)

Die Rampe allein blendet bei `DISABLE` von der aktuellen Armpose in die Pose
des Onboard-Reglers über — steht der Arm weit vorn, schwingt er in 2 s nach
unten. Deshalb:

- `_record_onboard_pose` merkt sich beim `ENABLE` (Gewicht 0) und beim E-Stop
  die gemessene Armpose = Onboard-Pose (`_onboard_q`).
- `DISABLE` ruft `_start_release`: Marker-Ziele, Bahnen, WALK und Homing
  werden beendet, dann fährt eine **geplante** Bahn (`source="release"`) in
  `_onboard_q`. Erst bei Ankunft (`release_tolerance`, 0,05 rad) folgt
  `_finish_release` → Rampe 1→0. Planer-Fehler → direkte, geschwindigkeits-
  und gate-begrenzte Fahrt; kein Fortschritt für 1,5 s oder
  `release_timeout_s` (12 s) → Rampe aus der erreichten Pose.
  `release_to_onboard_pose=false` schaltet das ab (alte direkte Rampe).
- Während `_releasing` lehnen Ziel-, Homing- und WALK-Callbacks mit
  „Rueckgabe an den Onboard-Regler" ab; `ENABLE` bricht ab und hält.
- Programmende: `main()` behandelt SIGTERM wie Strg+C und ruft im `finally`
  `shutdown_release()`: bis `shutdown_return_s` (2,5 s) direkt Richtung
  `_onboard_q` (≤ 0,8 rad/s, sanfter Start, kartesisches Limit, Gate), dann
  Rampe 1→0 bei 250 Hz. Im E-Stop nur die Rampe.
- `_apply_real_safety_limits` (nur real) begrenzt Gains auf
  `REAL_MAX_GAINS`, Rampen auf ≥ 1 s und die Geschwindigkeiten auf 1,5 rad/s /
  0,25 m/s.

Tests ohne Hardware: `test/test_arm_controller_real.py` (Fake-G1, der wie die
Firmware `arm_sdk` und Onboard-Pose nach dem Gewicht mischt).

### E-Stop-Semantik (`_on_emergency_stop` / `_publish_arm_slack`)

Kritisch: Beim E-Stop bleibt das `arm_sdk`-Gewicht auf **1** — würde es auf 0
gesetzt, übernähme der Onboard-Regler die Arme und führe sie *aktiv* mit
voller Geschwindigkeit in seine Default-Pose (ein Sprung). Stattdessen
schreibt `_publish_arm_slack()` weiterhin auf `rt/arm_sdk`, aber mit
`kp=0, tau=0, kd=estop_arm_kd` — die Arme werden drehmomentfrei und sacken
gedämpft, statt aktiv irgendwohin zu fahren. `/g1pilot/start` quittiert nur
den Latch (`estop_active`); die Arme bleiben schlaff, bis
`/g1pilot/arms/enabled` (GREIFEN, R2) die Kontrolle bewusst zurückholt.

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

Dateiformat (Version 3 mit Abläufen und relativen Bewegungen),
Speicherpfade und Migration: [23-T-Posen-und-Ablaeufe.md](23-T-Posen-und-Ablaeufe.md).

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
| `ik_alpha`, `ik_goal_filter_alpha` | Glättung Servoing-Pfad (`ik_goal_filter_alpha` pro Ziel-Nachricht, `ik_alpha` und der Ziel-Filter im Solver pro Takt) |
| `smoothing_ref_hz` | 0 = Glättung pro Takt (Real-Default). > 0: die Alphas pro Takt gelten für diese Bezugsrate und werden auf das echte `dt` umgerechnet, `1-(1-a)^(dt·ref)` — gleiches Zeitverhalten bei niedriger Regelrate. `bringup_sim` setzt 50 |
| `ik_async` | Marker-/Jog-IK im eigenen Prozess (Default true; false = IK im Takt, alt) |
| `ik_servo_step_tol` | Fruehes Ende der Servo-IK [rad] (Default 2e-4, 0.0 = immer alle Iterationen) |
| `ik_orientation_mode` | `full` / weitere Modi, siehe `set_orientation_mode` |
| `ik_null_space_gain` | Stärke der Nullraum-Regularisierung |
| `planned_motion_tolerance` | Toleranz [rad], ab der das Bahnende als erreicht gilt |
| `goal_pos_tol`, `goal_ori_tol_deg` | Ab hier gilt ein Marker-Ziel als erreicht (Default 1 cm / 3°) |
| `marker_goal_stall_time_s`, `marker_goal_stall_tol_rad` | Stillstand ohne Ankunft → Marker-Ziel unerreichbar (Default 0.8 s / 0.01 rad) |
| `planner_clearance` / `planner_min_clearance` | Abstand in der Bahnmitte (5 cm) / Untergrenze beim Reduzieren (1,5 cm) |
| `planner_relax_radius` | Größe des Trichters um Start und Ziel (15 cm) |
| `planner_time_budget_s` | Rechenzeit-Budget einer Planung (2,5 s) |
| `planner_vel_limit` / `planner_cart_vel_limit` | Tempo der geplanten Fahrt (rad/s, m/s) |
| `gate_hand_floor` / `gate_arm_floor` / `gate_slow_dist` | Laufzeit-Gate, siehe oben |
| `release_to_onboard_pose`, `release_tolerance`, `release_timeout_s` | Rückfahrt vor der Übergabe (Default an / 0,05 rad / 12 s) |
| `shutdown_return_s` | Rückfahrzeit beim Programmende (2,5 s) |
| `arm_accel_limit` | Gelenkbeschleunigung [rad/s²], 0 = aus (real über Launch 3,0, `G1_ARM_ACCEL_LIMIT`) |
| `max_tracking_error` | max. Abstand Soll–Messung [rad], 0 = aus (real 0,3, `G1_ARM_MAX_TRACKING_ERROR`) |
| `arm_api_*` | Parameter der HTTP-Brücke, siehe [21-T-Arm-API.md](21-T-Arm-API.md) |

## Bekannte Einschränkungen

- Der IK-Solve kostet bei aktiv verfolgtem Marker-Ziel ~20-30 ms/Zyklus —
  der Regel-Loop läuft dann effektiv mit ~30-50 Hz statt 250 Hz. Alle
  Limits rechnen mit dem tatsächlichen `dt`, bleiben also korrekt; im
  Halte-Zustand läuft der Loop mit voller Rate. Die Glättung dagegen wirkt
  pro Takt — je langsamer der Takt, desto träger der Arm (in der Sim auf
  einem ausgelasteten PC 10–15 Hz; dafür `smoothing_ref_hz`).
- Der IK-Solver erreicht posenabhängig das Ziel nicht ganz: Nullraum-Haltung
  und Reichweiten-Klemme lassen in manchen Posen 0,5–3 cm Restfehler
  (gemessen offline, z. B. Hand weit vorn und außen). Am Rand der Reichweite
  knickt eine gerade Fahrt deshalb ab.
- Die Taille (12-14) ist über `rt/arm_sdk` real ansteuerbar, wird von
  `arm_controller` aber bewusst nie beschrieben — ein Kommando dort würde
  gegen den Unitree-Loco-Controller arbeiten.

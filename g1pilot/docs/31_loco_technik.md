# Locomotion — Technik

Richtet sich an: Entwickler, die den Stand-/Lauf-Regler ändern oder tunen
wollen. Für die Bedienung siehe [30_loco_anleitung.md](30_loco_anleitung.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/navigation/loco_sim.py` | Sim-Stellvertreter für Stehen/Laufen (nur Simulation): ROS, Zustandsmaschine, DDS |
| `g1pilot/navigation/walk_policy.py` | Lauf-Policy laden, Obs bauen, Aktion → Motor-Sollwerte (ohne ROS) |
| `g1pilot/navigation/stand_balancer.py` | PD-Balancer und WALK→STAND-Übergabe (ohne ROS) |
| `g1pilot/navigation/loco_client.py` | Ansteuerung des Unitree-Onboard-Reglers (nur echter Roboter) |
| `g1pilot/policies/agile_velocity_g1/` | **Standard:** Lauf-Policy NVIDIA WBC-AGILE „Velocity-G1-History-v0" (Apache-2.0, Herkunft in `ATTRIBUTION.md`) |
| `g1pilot/policies/g1_wholebody/` | Alte Lauf-Policy (unitree_rl_mjlab G1 Velocity, Apache-2.0), per Parameter wählbar |
| `g1pilot/test_agile_walk_sim.py` | Headless-MuJoCo-Test von Laufen, Stehen und Übergaben (ohne ROS) |
| `g1pilot/test/test_walk_policy.py` | Unit-Tests: Obs-Layout, ONNX = Original, Übergabe-Logik |
| `unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py` | Mischt Bein-/Taillen-Kommandos (`rt/lowcmd`) mit Arm-Kommandos (`rt/arm_sdk`) |

## Warum zwei völlig verschiedene Implementierungen?

Auf dem **echten** G1 läuft Stehen/Balancieren/Gehen auf einem
Onboard-Controller von Unitree, den man nur über eine High-Level-API
(`LocoClient.BalanceStand`/`Move`/`StopMove`/`Damp`) anspricht. In **MuJoCo
gibt es diesen Onboard-Controller nicht** — die Simulation liefert nur die
rohe Low-Level-Schnittstelle (Gelenke + IMU rein, Motor-Befehle raus). Daher
gibt es einen eigenen Sim-Ersatzregler, `loco_sim`, der dieselben
Steuertopics wie `loco_client` bedient, intern aber komplett anders
funktioniert.

## `loco_sim` (Simulation)

### Zwei kombinierte Regler

- **STAND** (am Platz): modellbasierter Knöchel-/Hüft-PD-Regler. Hält die
  Füße geplant und richtet die per IMU gemessene Neigung aktiv auf — Arm-
  und Oberkörperstörungen werden ohne Schritte abgefangen.
- **WALK** (laufen): eine vortrainierte, velocity-konditionierte ONNX-Policy.
  Standard ist NVIDIAs WBC-AGILE „Velocity-G1-History-v0"
  (`policies/agile_velocity_g1`). Sie steuert nur Beine und Taille
  (roll/pitch) und wurde mit ständig zufällig bewegten Armen trainiert, darum
  **bleiben die Arme beim Laufen frei** (Marker, Kommandos, Box tragen). Läuft
  omnidirektional; bei `cmd=0` steht sie am Platz.

Grund für die Kombination: Die Policy steht bei `cmd=0` zwar sicher, macht
dabei aber kleine Ausgleichsschritte (Füße wandern einige Zentimeter, siehe
Testergebnisse unten). Der PD-Regler hält die Füße wirklich fest. Also:
Policy fürs Laufen, PD fürs stationäre Stehen.

Policy wechseln: Parameter `policy` von `loco_sim` (Ordnername unter
`policies/`). Die alte Policy braucht die Arme in der Lauf-Pose, also dazu
`walk_park_arms:=true` für die Manipulation (siehe
[11_arm_manipulation_technik.md](11_arm_manipulation_technik.md)); beides
steht in `launch/bringup_sim.launch.py`. Welche Logik eine Policy braucht,
erkennt `walk_policy.load_walk_policy()` am Feld `format` in `deploy.yaml`.

### Zustandsmaschine

```python
HOLD = "hold"    # Standby, Basis gehalten (Bridge: weld)
STAND = "stand"  # PD-Balancer, Füße geplant
WALK = "walk"    # ONNX-Policy
DAMP = "damp"    # Emergency / Sturz: alle Motoren weich
```

Übergänge **ausschließlich per Nutzer-Button/Topic**, kein automatisches
Umschalten zwischen STAND und WALK:

- `/g1pilot/start_balancing(True)` → STAND. Aus HOLD sofort
  (`_enter_stand()`, Beine in die Standpose). Aus WALK erst nach dem
  **Ausbremsen** (siehe „Übergabe WALK → STAND").
- `/g1pilot/start_walking(True)` → `_enter_walk()`. Nur wenn die Policy eine
  feste Armpose braucht (`needs_arm_pose`, alte Policy `g1_wholebody`), wartet
  `loco_sim` vorher im STAND, bis der `arm_controller` die Arme in die
  Lauf-Pose gebracht hat (`/g1pilot/arms/walk_ready`) oder
  `walk_arm_timeout_s` abläuft. Die AGILE-Policy läuft sofort los. Ein
  START WALKING während WALK setzt die Policy nicht zurück; es bricht nur eine
  laufende STAND-Übergabe ab.
- `/g1pilot/emergency_stop(True)` → sofort `DAMP` + Arme deaktivieren.
- `/g1pilot/start(True)` → `HOLD`.
- Sturz-Erkennung (`_fallen`, IMU-Neigung über `fall_gz` für
  `fall_debounce_s`) → automatisch `DAMP`, unabhängig vom Auslöser.

### Zuständigkeit der Gelenke

`loco_sim` regelt **ausschließlich** Beine (0–11) und Taille (12–14),
niemals die Arme (15–28) — die gehören komplett dem `arm_controller`
(`rt/arm_sdk`). Das verhindert, dass ein Zustandswechsel im Loco-Regler
die Arme „teleportiert".

### Regelschleife (`_control_loop`)

Läuft in einem eigenen Thread, getaktet über `control_dt` aus
`deploy.yaml` (Policy-Rate, i. d. R. 20 ms = 50 Hz). Zwei Taktmodi:

- **Lockstep** (`SIM_LOCKSTEP=1`, Standard im Betrieb): wartet pro
  Iteration auf den nächsten `rt/lowstate`-Eingang, statt auf die
  Wall-Clock zu takten — deterministische 50-Hz-Regelrate unabhängig von
  der Rechnerlast.
- **Echtzeit** (`SIM_LOCKSTEP=0`): schläft die Restzeit von `control_dt`
  ab; `SIM_REALTIME_FACTOR` skaliert das Tempo.

`_send_hold` / `_send_damp` / `_send_balance_pd` / `_send_policy` schreiben
jeweils `rt/lowcmd` und (über `_write`) einen Zustandscode in
`motor_cmd[29].q` (`STATE_IDX`), den die Bridge zur Basis-Physik nutzt
(`weld` im HOLD, frei sonst).

### Übergabe WALK → STAND

Die AGILE-Policy steht in leichter Hocke (Becken ca. 0.73 m statt 0.78 m,
Knie ca. 0.8 rad) und lässt die Füße oft leicht versetzt stehen. Der frühere
Wechsel zog die Beine sofort in die gestreckte Standpose. Dabei wandert das
Becken nach vorn, und mit Armen vor dem Körper kippte der Roboter in der
Headless-Sim zuverlässig nach vorn. Deshalb läuft der Wechsel jetzt in zwei
Schritten (`stand_balancer.SettleGate`, `StandBalancer.enter(hold_pose=True)`):

1. START BALANCING im WALK setzt das Kommando auf 0, die Policy bremst aus.
2. Sobald der Roboter ruhig steht (frühestens nach `settle_s`, dann
   `settle_quiet_s` lang |Gyro| < `settle_gyro_max` und alle Bein-|dq| <
   `settle_dq_max`; spätestens nach `settle_timeout_s`), übernimmt der PD und
   **hält die vorgefundene Beinpose**, statt sie in die Standpose zu ziehen.

In den Tests dauert das Ausbremsen 0.6 bis 1.1 s. Aus HOLD (START BALANCING
nach dem Reset) bleibt alles wie bisher: Beine in die Standpose.

### PD-Balancer (`_send_balance_pd`)

Feedforward-Drehmoment auf Knöchel (primär) und Hüfte (sekundär), berechnet
aus der IMU-Neigung (`get_gravity_orientation`) und Gyroskop-Rate:

```
t_ankle_pitch = kp*pitch_err + kd*pitch_rate + integral_trim
t_ankle_roll  = -(kp*roll_err + kd*roll_rate)
t_hip_pitch   = kp*pitch_err + kd*pitch_rate
t_hip_roll    = -(kp*roll_err + kd*roll_rate)
t_hip_yaw     = -kd*yaw_rate
```

Ein Integral-Trim auf den Knöchel-Pitch (`bal_ki_pitch`) gleicht statische
Schwerpunktversätze aus (z. B. schwerere Inspire-FTP-Hände verschieben den
Schwerpunkt nach vorn) — ohne ihn bliebe eine Dauerneigung stehen. Ein
sanfter Eintritts-Rampe (`bal_ramp_s`) blendet beim Wechsel aus WALK die
aktuelle Beinpose zur Standardpose, damit der steife PD die Beine nicht aus
der Lauf-Stellung reißt; aus HOLD ist die Rampe bewusst kurz (0.1 s), weil
eine längere Weich-Phase den (durch Hände kopflastigeren) Roboter
unaufholbar nach vorn kippen ließ.

### Policy (`_send_policy`, `walk_policy.py`)

**AGILE** (`AgileHistoryPolicy`, `format: agile_history`): 255 Obs = 6 Terme
× 5 Zeitschritte (Gyro × 0.2, projizierte Gravitation, Kommando,
Gelenkabweichung, Gelenkgeschwindigkeit × 0.05, letzte Aktion), Term für Term,
älteste zuerst. 14 Aktionen für Beine + Taille roll/pitch,
`q = default + 0.5 · action`, PD mit `stiffness`/`damping` aus `deploy.yaml`;
`waist_yaw` hält `loco_sim` auf 0. Alle Werte und ihre Quelle stehen in
`policies/agile_velocity_g1/deploy.yaml` und `ATTRIBUTION.md`. Kommandos mit
‖cmd‖ < 0.1 werden wie im Training zu 0.

**Alt** (`MjlabVelocityPolicy`, `g1_wholebody`): 98 Obs mit Gait-Phase
sin/cos (0 bei ‖cmd‖ < 0.1), 29 Aktionen, davon nur Beine + Taille aktuiert.

Die Joystick-Werte (−1…1) skaliert `scale_command()` auf die
Trainingsbereiche der Policy (`commands.base_velocity.ranges`).

### Konfigurierbare Parameter (Auszug)

| Parameter | Bedeutung |
|---|---|
| `policy` | Ordner der Lauf-Policy unter `policies/` (Standard `agile_velocity_g1`) |
| `settle_s`, `settle_quiet_s`, `settle_gyro_max`, `settle_dq_max`, `settle_timeout_s` | Übergabe WALK → STAND |
| `fall_gz`, `fall_debounce_s` | Sturz-Erkennungsschwelle/-Entprellung |
| `hold_kd_scale` | Dämpfungsfaktor der Beine im HOLD |
| `bal_kp_scale`, `bal_ramp_s` | Steifigkeit/Eintrittsrampe des PD-Balancers |
| `bal_ki_pitch`, `bal_i_limit` | Integral-Trim gegen statische Neigung |
| `bal_ankle_kp_pitch/roll`, `bal_hip_kp_pitch/roll`, `bal_yaw_kd` | PD-Gains je Achse |
| `walk_arm_wait`, `walk_arm_timeout_s` | Warten auf Arm-Aufräumen vor WALK (nur alte Policy) |

Live änderbar via `ros2 param set /loco_sim <name> <wert>`.

### Zusatzfunktionen (nur Sim)

- **PUSH** (`/g1pilot/push`) — schickt einen UDP-Stoßimpuls an die Sim, um
  die Störunterdrückung zu testen.
- **GRASP BOX** (`/g1pilot/grasp_box`) — schaltet eine greifbare Testkugel
  in der Handfläche an/aus (Inspire-Hände, siehe
  [61_inspire_haende_technik.md](61_inspire_haende_technik.md)).

## `loco_client` (echter Roboter)

Dünner ROS-Node um `unitree_sdk2py.g1.loco.g1_loco_client.LocoClient`. Bildet
dieselben Streamdeck-Topics wie `loco_sim` auf die Unitree-High-Level-RPCs
ab:

| Topic | RPC |
|---|---|
| `/g1pilot/start` | `SetFsmId(4)` (Standby) |
| `/g1pilot/start_balancing` | `entering_balancing()` — Höhenrampe + `BalanceStand(1)` + `Start()` |
| `/g1pilot/start_walking` + `/g1pilot/loco_cmd_vel` | `Move(vx, vy, vyaw, continous_move=True)` in `_cmd_vel_tick` (20 Hz) |
| `/g1pilot/emergency_stop` | `Damp()`, sofort, eigene Callback-Gruppe |

Wichtige Sicherheitsmechanismen:

- **RPC-Serialisierung** (`_rpc_lock`): der E-Stop läuft in einer eigenen
  `MultiThreadedExecutor`-Callback-Gruppe, damit er auch während eines
  laufenden, blockierenden `entering_balancing()` sofort durchkommt.
- **Deadman/Timeout** (`_cmd_vel_tick`): `Move()` läuft nur, solange
  balanciert, nicht gestoppt, `WALK` aktiv und der `loco_cmd_vel`-Stream
  frisch ist (`cmd_vel_timeout`, Default 0.5 s). Fällt eine Bedingung weg,
  wird genau einmal `StopMove()` gesendet.
- **PS4-Controller-Pfad** (`joystick_callback`): physischer Deadman-Button
  (Index 8) hat Vorrang vor dem Streamdeck-`loco_cmd_vel`-Pfad, solange
  gedrückt.
- `damp_on_init=False`: der Node greift beim Start nicht von sich aus in
  den Roboterzustand ein — ein bereits stehender G1 würde bei `Damp()`
  sofort zusammensacken.

## Bridge-Merge (Sim)

`unitree_sdk2py_bridge.py` (in `unitree_mujoco`) empfängt sowohl
`rt/lowcmd` (Beine/Taille, von `loco_sim`) als auch `rt/arm_sdk`
(Arme, von `arm_controller`) und mischt sie pro Motor: Beine/Taille
übernehmen unverändert `rt/lowcmd`; die Arme werden mit dem in
`rt/arm_sdk`-`motor_cmd[29].q` transportierten Gewicht zwischen
`rt/lowcmd`-Fallback (0) und `rt/arm_sdk`-Kommando (1) überblendet. Details
zum DDS-Merge in [02_architektur.md](02_architektur.md).

## Testergebnisse (Headless-MuJoCo)

`python3 test_agile_walk_sim.py [--inspire] [--policy g1_wholebody]` fährt
denselben Regler-Code wie `loco_sim` (`walk_policy`, `stand_balancer`) in
MuJoCo nach: 1 ms Physik, 50 Hz Lockstep, Reset wie die Bridge, Arme per
arm_controller-Ersatz (PD + Schwerkraftkompensation, 1.5 rad/s). Ebener Boden
(die Sim-Szene hat ab x = 1 m Hindernisse). Exit-Code 0 = alle
Pflicht-Szenarien bestanden; „Info"-Zeilen messen Grenzen.

AGILE, Stand 2026-10-08 (Standard-Szene / Inspire-Szene):

| Szenario | Ergebnis |
|---|---|
| Laufen vor/zurück/seitlich/drehen | steht nie um; Ist ≈ 0.36 / −0.31 / ±0.30 m/s, Drehen 0.83 rad/s (Soll 0.5 / −0.4 / ±0.4 / 0.8); Nachlauf nach Stopp ≤ 1 cm |
| Laufen mit freien Armen (hängend, ständig bewegt, Box 0.5 und 1.0 kg je Hand vor dem Körper) | alle stabil, auch beim Drehen; Neigung ≤ 0.12 |
| Policy allein im Stand, 20 s (cmd = 0) | steht; Füße wandern 4–5 cm (Arme ruhig) bzw. 4 cm / 22 cm (Arme kräftig bewegt, Standard / Inspire) |
| PD-Stand ab START BALANCING, Arme ruhig oder mäßig bewegt | Füße bleiben stehen (≤ 1 mm) |
| WALK → STAND (8 feste + 20 zufällige Wechsel, auch direkt aus vollem Lauf, mit Box bis 1 kg je Hand) | 0 Stürze; PD übernimmt nach 0.6–1.1 s, danach Füße ≤ 1 cm |
| *Info:* PD-Stand ab START BALANCING, Box 0.5 kg je Hand wird nach vorn gehoben | **fällt** (beide Szenen); nach einer Übergabe aus dem Laufen hält der PD 1.0 kg sicher (2.0 kg nicht zuverlässig), die Policy allein 2.0 kg |
| *Info:* PD-Stand ab START BALANCING, Arme kräftig bewegt | Standard steht, Inspire **fällt** (bei mäßiger Bewegung steht er) |
| *Info:* START WALKING mit cmd = 0 | aus dem geraden PD-Stand: 0.4–0.8 m Anlauf nach vorn; aus dem PD nach einer Übergabe: ca. 2 cm |

Die alte Policy (`--policy g1_wholebody`) läuft mit Armen in der Lauf-Pose gut,
fällt aber mit hängenden Armen, mit Box und unter Arm-Bewegung im Stand.

## Bekannte Einschränkungen

- `loco_sim` ist ein reiner **Sim-Stellvertreter**. Die konkrete
  Balance-/Lauf-Strategie ist eigens für die Simulation gebaut und teils auf
  simulationsinterne Größen angewiesen (z. B. Basisgeschwindigkeit/Fußkraft
  in `reserve[]` der Bridge) — sie ist **nicht** für den Realeinsatz gedacht.
- Die AGILE-Policy erreicht in MuJoCo etwa 75 % der kommandierten
  Geschwindigkeit (0.36 m/s bei 0.5 m/s vor, 0.3 m/s bei 0.4 m/s seitlich);
  Drehen trifft den Sollwert. Ursache ist der Sim-Unterschied zum Isaac-Lab-Training.
- Ein driftfreier Stand kommt ausschließlich vom modellbasierten PD. Seine
  Grenzen (Box vor dem Körper, kräftig bewegte Arme mit Inspire-Händen) stehen
  in den Testergebnissen; die Policy allein hält dort noch.
- Anlauf beim ersten Loslaufen: Die AGILE-Policy steht in einer etwa 5 cm
  tieferen Hocke als der PD-Stand. Startet WALK aus dem geraden PD-Stand (also
  direkt nach START BALANCING), sucht sie diese Hocke mit einigen Schritten und
  wandert dabei 0.4–0.8 m nach vorn, auch bei cmd = 0. Nach einem
  Lauf-→-Stand-Wechsel hält der PD die Hocke, dann startet WALK ruhig.
- Die Taille (roll/pitch) bewegt sich beim Laufen mit der AGILE-Policy. Die
  Arm-IK rechnet mit fester Taille, darum verschiebt sich eine gehaltene
  Handpose beim Laufen um einige Zentimeter mit dem Oberkörper.
- `USE_JOYSTICK=0` ist zwingend, solange kein Gamepad im Container hängt —
  sonst stirbt der Sim-Thread.

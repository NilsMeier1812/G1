# Locomotion — Technik

Richtet sich an: Entwickler, die den Stand-/Lauf-Regler ändern oder tunen
wollen. Für die Bedienung siehe [30_loco_anleitung.md](30_loco_anleitung.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/navigation/loco_sim.py` | Sim-Stellvertreter für Stehen/Laufen (nur Simulation): ROS, Zustandsmaschine, DDS |
| `g1pilot/navigation/walk_policy.py` | Lauf-Policy laden, Obs bauen, Aktion → Motor-Sollwerte (ohne ROS) |
| `g1pilot/navigation/stand_balancer.py` | PD-Balancer mit Schwerpunkt-Führung, Kipp-Erkennung, WALK→STAND-Übergabe (ohne ROS) |
| `g1pilot/navigation/com_model.py` | Schwerpunkt relativ zu den Füßen aus den Gelenkwinkeln (Pinocchio, MJCF-Modell der Sim) |
| `g1pilot/navigation/loco_client.py` | Ansteuerung des Unitree-Onboard-Reglers (nur echter Roboter) |
| `g1pilot/policies/agile_velocity_g1/` | **Standard:** Lauf-Policy NVIDIA WBC-AGILE „Velocity-G1-History-v0" (Apache-2.0, Herkunft in `ATTRIBUTION.md`) |
| `g1pilot/policies/agile_velocity_height_g1/` | Alternative: NVIDIA WBC-AGILE „Velocity-Height-G1-History-v0" (Apache-2.0), gleichmäßiger Gang; fängt mit der Standard-Policy ab |
| `g1pilot/policies/g1_wholebody/` | Alte Lauf-Policy (unitree_rl_mjlab G1 Velocity, Apache-2.0), per Parameter wählbar |
| `g1pilot/test_agile_walk_sim.py` | Headless-MuJoCo-Test von Laufen, Stehen und Übergaben (ohne ROS) |
| `g1pilot/test/test_walk_policy.py` | Unit-Tests: Obs-Layout, ONNX = Original, Übergabe-Logik, Schwerpunkt-Modell, Kipp-Erkennung |
| `unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py` | Mischt Bein-/Taillen-Kommandos (`rt/lowcmd`) mit Arm-Kommandos (`rt/arm_sdk`); stellt den Roboter bei START BALANCING in die von `loco_sim` kommandierte Stand-Pose |

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
  Füße geplant, richtet die per IMU gemessene Neigung aktiv auf und schiebt
  das Becken so, dass der Schwerpunkt über der Fußmitte bleibt. Arm-Bewegungen
  und Lasten in den Händen fängt er ohne Schritte ab. Er steht in der
  Stand-Pose der Lauf-Policy (AGILE: leichte Hocke mit breiter Spur).
- **WALK** (laufen): eine vortrainierte, velocity-konditionierte ONNX-Policy.
  Standard ist NVIDIAs WBC-AGILE „Velocity-G1-History-v0"
  (`policies/agile_velocity_g1`). Sie steuert nur Beine und Taille
  (roll/pitch) und wurde mit ständig zufällig bewegten Armen trainiert, darum
  **bleiben die Arme beim Laufen frei** (Marker, Kommandos, Box tragen). Läuft
  omnidirektional; bei `cmd=0` steht sie am Platz.

Grund für die Kombination: Die Policy steht bei `cmd=0` zwar sicher, macht
dabei aber kleine Ausgleichsschritte (Füße wandern einige Zentimeter, mit
bewegten Armen bis über 20 cm, siehe Testergebnisse unten). Der PD-Regler hält
die Füße wirklich fest. Also: Policy fürs Laufen, PD fürs stationäre Stehen.
Nur wenn der PD den Roboter nicht mehr halten kann (ein Fuß hebt ab, z. B.
nach einem Stoß), fängt die Policy mit Schritten ab und gibt danach wieder an
den PD (siehe „Abfangen").

Policy wechseln: Umgebungsvariable `G1_WALK_POLICY` beim Start
(`docker-compose.yml` → `launch/bringup_sim.launch.py` → Parameter `policy`
von `loco_sim`, Ordnername unter `policies/`):

| `G1_WALK_POLICY` | Gang | Arme beim Laufen |
|---|---|---|
| `agile_velocity_g1` (Standard) | sehr robust, aber Fangschritte ohne festen Rhythmus (siehe „Gangbild") | frei |
| `agile_velocity_height_g1` | gleichmäßig, trifft das Tempo, bis 1 m/s | frei |
| `g1_wholebody` (alt) | gleichmäßig | fest in der Lauf-Pose |

Die alte Policy braucht die Arme in der Lauf-Pose, also dazu
`walk_park_arms:=true` für die Manipulation (siehe
[11_arm_manipulation_technik.md](11_arm_manipulation_technik.md), steht in
`launch/bringup_sim.launch.py`). Welche Logik eine Policy braucht, erkennt
`walk_policy.load_walk_policy()` am Feld `format` in `deploy.yaml`.

### Gangbild

Gemessen mit `test_agile_walk_sim.py`-Aufbau, geradeaus, Arme in Lauf-Pose,
Standard-Szene, Mittel über 9 s (Schwankung der Schrittabstände als
Variationskoeffizient):

| Policy | Tempo Soll → Ist | Schritte/s | Schwankung | beide Füße am Boden | Becken-Nicken |
|---|---|---|---|---|---|
| `agile_velocity_g1` | 0.4 → 0.30 m/s | 2.0 | ±42 % | 66 % | ±1.2° |
| `agile_velocity_g1` | 0.2 → 0.13 m/s | 1.0 | ±69 % | 85 % | ±1.2° |
| `agile_velocity_height_g1` | 0.4 → 0.38 m/s | 2.9 | < 1 % | 35 % | ±0.1° |
| `agile_velocity_height_g1` | 0.3 → 0.28 m/s | 3.1 | ±1 % | 40 % | ±0.0° |
| `agile_velocity_height_g1` | 1.0 → 1.01 m/s | 2.8 | ±1 % | 27 % | ±0.5° |
| `g1_wholebody` | 0.4 → 0.41 m/s | 3.2 | ±5 % | 13 % | ±0.8° |

Die Standard-Policy kippt das Becken leicht nach vorn und setzt dann einen
schnellen Fangschritt: im Training (AGILE `velocity_history_env_cfg.py`) gibt
es nur Belohnungen für Tempo, flache Haltung und Regularisierung, aber keinen
Gang-Term (Schrittrhythmus, Schwungzeit). Mehr Dämpfung (im Trainingsbereich
bis 2×), die MuJoCo-Einstellungen aus NVIDIAs eigenem Sim2MuJoCo-Test
(Armature 0.02, Gelenkreibung 0.1) oder hängende Arme ändern daran nichts.
`agile_velocity_height_g1` geht unter 0.25 m/s ebenfalls unregelmäßig (Tippel-
schritte); sie läuft mit leicht nach hinten geneigtem Oberkörper (≈ 3°) und
mit etwa 2–3 cm Fußhub.

### Zustandsmaschine

```python
HOLD = "hold"    # Standby, Basis gehalten (Bridge: weld)
STAND = "stand"  # PD-Balancer, Füße geplant
WALK = "walk"    # ONNX-Policy
DAMP = "damp"    # Emergency / Sturz: alle Motoren weich
```

Übergänge per Nutzer-Button/Topic. Automatisch ist nur das Abfangen (STAND
→ Policy → STAND, siehe unten) und der Sturz → DAMP:

- `/g1pilot/start_balancing(True)` → STAND. Aus HOLD/DAMP sofort
  (`_enter_stand()`): `loco_sim` kommandiert die Stand-Pose, die Bridge stellt
  den Roboter beim Wechsel genau dorthin (siehe „Stand-Pose"). Aus WALK erst
  nach dem **Ausbremsen** (siehe „Übergabe WALK → STAND").
- `/g1pilot/start_walking(True)` → `_enter_walk()`. Nur wenn die Policy eine
  feste Armpose braucht (`needs_arm_pose`, alte Policy `g1_wholebody`), wartet
  `loco_sim` vorher im STAND, bis der `arm_controller` die Arme in die
  Lauf-Pose gebracht hat (`/g1pilot/arms/walk_ready`) oder
  `walk_arm_timeout_s` abläuft. Die AGILE-Policy läuft sofort los. Ein
  START WALKING während WALK setzt die Policy nicht zurück; es bricht nur eine
  laufende STAND-Übergabe oder ein Abfangen ab. Direkt aus HOLD sendet
  `loco_sim` im ersten Takt die Stand-Pose (`_fresh_walk`), damit die Bridge
  dort aufstellt, und startet die Policy erst im zweiten Takt.
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

### Stand-Pose

Der PD steht dort, wo die Policy beim Loslaufen anfängt: in ihrer eigenen
Stand-Pose (`stand_pose_legs` in `deploy.yaml`, gemessen: leichte Hocke,
Becken ca. 0.73 m, Füße ca. 0.41 m auseinander). Policies ohne diesen Eintrag
(alte `g1_wholebody`) bekommen die gerade Standpose `LEG_STAND_POSE`.

Früher stand der PD gerade (Becken 0.78 m, Füße 0.24 m auseinander). Beim
START WALKING suchte die Policy dann ihre Hocke und lief dabei 0.4–0.8 m nach
vorn, auch bei cmd = 0. Vom geraden Stand in die Hocke kommt man nicht ohne
Schritt, weil die Füße breiter stehen müssen; ein langsames Absinken im Stand
ließ die Füße 7–12 cm rutschen. Deshalb stellt die Bridge den Roboter beim
Wechsel nach RUN direkt in die Hocke: Sie übernimmt Beine/Taille aus dem
umschaltenden `rt/lowcmd` (`LOCO_RESET_FROM_CMD`, Plausibilitätsprüfung: alle
Bein-kp > 0, |q| < 3 rad, sonst die alte Config-Pose) und berechnet die
Beckenhöhe so, dass die Füße genau auf dem Boden stehen
(`LOCO_RESET_PELVIS_Z = None`). `StandBalancer.enter(goal)` kommandiert die
Pose dafür schon im ersten Takt.

Die Hand-/Greifhöhe im BALANCING liegt dadurch etwa 5 cm tiefer als früher.
Nach einem Lauf-→-Stand-Wechsel stand der Roboter ohnehin schon in dieser
Höhe.

### Übergabe WALK → STAND

Die AGILE-Policy lässt die Füße beim Anhalten oft leicht versetzt stehen und
steht nicht exakt in der Stand-Pose. Der frühere Wechsel zog die Beine sofort
in die Standpose; dabei wanderte das Becken, und mit Armen vor dem Körper
kippte der Roboter in der Headless-Sim zuverlässig nach vorn. Deshalb läuft
der Wechsel in zwei Schritten (`stand_balancer.SettleGate`,
`StandBalancer.enter()` ohne Ziel):

1. START BALANCING im WALK setzt das Kommando auf 0, die Policy bremst aus.
2. Sobald der Roboter ruhig steht (frühestens nach `settle_s`, dann
   `settle_quiet_s` lang |Gyro| < `settle_gyro_max` und alle Bein-|dq| <
   `settle_dq_max`; spätestens nach `settle_timeout_s`), übernimmt der PD und
   **hält die vorgefundene Beinpose**.

In den Tests dauert das Ausbremsen 0.6 bis 1.4 s.

### PD-Balancer (`_send_balance_pd`, `stand_balancer.py`)

Posture-PD auf die Stand-Pose (Steifigkeit × `bal_kp_scale`) plus
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
Neigung aus. Roll ist halb so steif wie Pitch (75/20 am Knöchel, 100/20 an der
Hüfte): Mit den alten Werten (150/40, 200/40) schaukelte sich der Roboter in
der breiten Hocke mit Inspire-Händen seitlich auf, die Füße rutschten 5–20 cm.
Beim Wechsel aus WALK fährt eine Rampe (`bal_ramp_s`) die Steifigkeit weich
hoch; aus HOLD ist sie bewusst kurz (0.1 s), weil eine längere Weich-Phase den
Roboter unaufholbar nach vorn kippen ließ.

**Schwerpunkt-Führung.** Der Neigungsregler allein hält nur das Becken
aufrecht. Arme, Hände und Last verschieben aber den Schwerpunkt, und wandert
der zur Ferse oder zu den Zehen, kippt der Roboter über die Fußkante. In der
Policy-Hocke liegt der Schwerpunkt schon in Ruhe nur 7 cm vor der Ferse; mit
Inspire-Händen kippte der Roboter bei mäßig bewegten Armen nach hinten. Darum:

1. `com_model.py` rechnet den Schwerpunkt aus allen 29 Gelenkwinkeln
   (Pinocchio, `buildModelFromMJCF` auf dasselbe MJCF, das die Sim simuliert;
   Massen identisch mit MuJoCo). Ergebnis: Lage vor dem Knöchel im
   Fuß-Koordinatensystem, ohne IMU.
2. Der Balancer schiebt das Becken (Knöchel-Pitch −d, Hüft-Pitch +d, der
   Oberkörper bleibt aufrecht), bis der Schwerpunkt bei `bal_com_target_m`
   (3.5 cm vor dem Knöchel = Fußmitte) liegt. Zeitkonstante `bal_com_tau_s`,
   Vorhalt auf die Schwerpunkt-Geschwindigkeit `bal_com_lead_s`, Begrenzung
   0.5 rad/s und `bal_com_limit`.
3. Lasten kennt das Modell nicht. Eine langsame Korrektur
   (`bal_bias_tau_s`) zieht die Schätzung nach dem Druckpunkt aus dem eigenen
   Knöchelmoment nach (τ / (m·g) vor dem Knöchel), nur im ruhigen Stand.
   Direkt aus dem Knöchelmoment zu regeln, ohne Modell, funktioniert nicht:
   Schiebt der Balancer das Becken, schlägt das Moment zuerst in die
   Gegenrichtung aus, und das schaukelte sich in jedem Test auf.

Das MJCF kommt im `g1pilot-sim`-Container per Mount aus `unitree_mujoco`
(`docker-compose.yml`, Pfad `/unitree_mujoco/unitree_robots/g1`), das
richtige Modell wählt `loco_sim` über `G1_INSPIRE_HANDS` oder den Parameter
`robot_mjcf`. Braucht Pinocchio ≥ 3 (`Dockerfile.sim`). Fehlt das Modell,
läuft der Balancer ohne Schwerpunkt-Führung weiter und `loco_sim` warnt beim
Start.

### Abfangen (STAND → Policy → STAND)

`TipDetector` prüft in jedem PD-Takt, ob eine Fußsohle mehr als
`rescue_foot_tilt_deg` (6°) gegen die Horizontale kippt, also abhebt, oder
das Becken mehr als `rescue_tilt_max` kippt, jeweils `rescue_debounce_s`
lang. Die Sohlen-Neigung kommt aus IMU-Quaternion und Bein-Encodern
(`stand_balancer.foot_tilt`, Kette mit den ±10°-Drehungen im Hüft-/Knie-Link
des MJCF). Im normalen Stand liegt sie unter 2°, auch mit Box und kräftig
bewegten Armen.

Dann gibt `loco_sim` an die Abfang-Policy (History frisch, cmd = 0, Joystick
wird ignoriert), die mit Schritten abfängt. Danach geht es wie bei START
BALANCING im Laufen über die `SettleGate` zurück in den PD, der die neue Pose
hält. Der Bridge-Zustand bleibt dabei „aktiv" (Code 1), es gibt also keinen
Reset. Abschalten: `rescue_enable:=false`.

Abfang-Policy ist die Lauf-Policy selbst, außer ihre `deploy.yaml` nennt eine
andere (`rescue_policy`) oder der Parameter `rescue_policy` ist gesetzt.
`agile_velocity_height_g1` fängt mit `agile_velocity_g1` ab: allein stürzte
sie bei 150 N von hinten und 250 N von vorn/hinten. Ein START WALKING während
des Abfangens wird zu normalem Laufen; bei einer eigenen Abfang-Policy erst,
wenn das Abfangen fertig ist (kein Policy-Wechsel mitten im Fangschritt).

### Policy (`_send_policy`, `walk_policy.py`)

**AGILE** (`AgileHistoryPolicy`, `format: agile_history`): 255 Obs = 6 Terme
× 5 Zeitschritte (Gyro × 0.2, projizierte Gravitation, Kommando,
Gelenkabweichung, Gelenkgeschwindigkeit × 0.05, letzte Aktion), Term für Term,
älteste zuerst. 14 Aktionen für Beine + Taille roll/pitch,
`q = default + 0.5 · action`, PD mit `stiffness`/`damping` aus `deploy.yaml`;
`waist_yaw` hält `loco_sim` auf 0. Alle Werte und ihre Quelle stehen in
`policies/agile_velocity_g1/deploy.yaml` und `ATTRIBUTION.md`. Kommandos mit
‖cmd‖ < 0.1 werden wie im Training zu 0.

**AGILE Velocity-Height** (`AgileHistoryPolicy`, `format:
agile_height_history`, `policies/agile_velocity_height_g1`): 400 Obs = 6
Terme × 5 Zeitschritte (Kommando vx/vy/vyaw + Becken-Höhe 0.72 m, Gyro,
projizierte Gravitation, Abweichung und Geschwindigkeit × 0.1 **aller 29
Gelenke inkl. Arme**, letzte Aktion). 12 Aktionen nur für die Beine,
Skalierung je Gelenk (Hüfte/Knie 0.25 · Momentgrenze / kp, Knöchel 1.0),
Aktion auf ±6 begrenzt; die Taille hält `loco_sim` auf 0. Geprüft gegen
NVIDIAs LEAPP-Export derselben Policy (ONNX mit eingebauter Obs-Verarbeitung),
siehe `ATTRIBUTION.md` dort. Ihre eigene Ruhepose ist in MuJoCo eine tiefe
Hocke (Becken ≈ 0.66 m); der PD steht trotzdem in der Hocke der
Standard-Policy (Begründung in der `deploy.yaml`).

**Alt** (`MjlabVelocityPolicy`, `g1_wholebody`): 98 Obs mit Gait-Phase
sin/cos (0 bei ‖cmd‖ < 0.1), 29 Aktionen, davon nur Beine + Taille aktuiert.

Die Joystick-Werte (−1…1) skaliert `scale_command()` auf die
Trainingsbereiche der Policy (`commands.base_velocity.ranges`).

### Konfigurierbare Parameter (Auszug)

| Parameter | Bedeutung |
|---|---|
| `policy` | Ordner der Lauf-Policy unter `policies/` (Standard `agile_velocity_g1`, im Bringup aus `G1_WALK_POLICY`) |
| `settle_s`, `settle_quiet_s`, `settle_gyro_max`, `settle_dq_max`, `settle_timeout_s` | Übergabe WALK → STAND |
| `fall_gz`, `fall_debounce_s` | Sturz-Erkennungsschwelle/-Entprellung |
| `hold_kd_scale` | Dämpfungsfaktor der Beine im HOLD |
| `bal_kp_scale`, `bal_ramp_s` | Steifigkeit/Eintrittsrampe des PD-Balancers |
| `bal_ki_pitch`, `bal_i_limit` | Integral-Trim gegen statische Neigung |
| `bal_ankle_kp_pitch/roll`, `bal_hip_kp_pitch/roll`, `bal_yaw_kd` | PD-Gains je Achse |
| `bal_com_target_m`, `bal_com_tau_s`, `bal_com_lead_s`, `bal_com_limit`, `bal_bias_tau_s` | Schwerpunkt-Führung (`bal_com_tau_s` 0 = aus) |
| `robot_mjcf` | MJCF für das Schwerpunkt-Modell (leer = aus `G1_INSPIRE_HANDS`) |
| `rescue_enable`, `rescue_foot_tilt_deg`, `rescue_tilt_max`, `rescue_debounce_s` | Abfangen mit der Policy |
| `rescue_policy` | Abfang-Policy (leer = `rescue_policy` aus der `deploy.yaml`, sonst die Lauf-Policy) |
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

`python3 test_agile_walk_sim.py [--inspire] [--policy NAME] [--rescue-policy NAME]` fährt
denselben Regler-Code wie `loco_sim` (`walk_policy`, `stand_balancer`) in
MuJoCo nach: 1 ms Physik, 50 Hz Lockstep, Reset wie die Bridge, Arme per
arm_controller-Ersatz (PD + Schwerkraftkompensation, 1.5 rad/s). Ebener Boden
(die Sim-Szene hat ab x = 1 m Hindernisse). Exit-Code 0 = alle
Pflicht-Szenarien bestanden; „Info"-Zeilen messen Grenzen.

AGILE, Stand 2026-10-08 (Standard-Szene / Inspire-Szene, beide bestanden):

| Szenario | Ergebnis |
|---|---|
| Laufen vor/zurück/seitlich/drehen | steht nie um; Ist ≈ 0.36 / −0.31 / ±0.30 m/s, Drehen 0.83 rad/s (Soll 0.5 / −0.4 / ±0.4 / 0.8); Nachlauf nach Stopp ≤ 1 cm |
| Laufen mit freien Armen (hängend, ständig bewegt, Box 0.5 und 1.0 kg je Hand vor dem Körper) | alle stabil, auch beim Drehen; Neigung ≤ 0.10 |
| Policy allein im Stand, 20 s (cmd = 0), zum Vergleich | steht; Füße wandern 3 cm (Arme ruhig) bzw. 6 cm / 22 cm (Arme kräftig bewegt, Standard / Inspire) |
| PD-Stand nach START BALANCING, 20 s: Arme ruhig, hängend, mäßig und kräftig bewegt, weit vor/zurück, Box 0.5 und 1.0 kg je Hand (auch mit bewegten Armen) | Füße bleiben stehen (≤ 3 mm), kein Abfangen nötig; Neigung ≤ 0.04 |
| WALK → STAND (8 feste + 20 zufällige Wechsel, auch direkt aus vollem Lauf, mit Box bis 1 kg je Hand) | 0 Stürze, kein Abfangen; PD übernimmt nach 0.6–1.0 s, danach Füße ≤ 5 mm |
| START WALKING mit cmd = 0, aus dem PD nach START BALANCING oder nach einer Übergabe | Becken bewegt sich höchstens 3.5 cm (früher aus dem geraden PD-Stand 0.4–0.8 m) |
| Stöße 80 / 150 / 250 N für 0.12 s auf den Torso, vorn/hinten/seitlich/schräg | 0 Stürze. Seitlich bis 150 N und schräg mit 80 N hält der PD ohne Schritt; sonst fängt die Policy mit Schritten ab (Becken 0.1–0.3 m bei 80 N, bis 2.2 m bei 250 N) und gibt zurück an den PD |
| *Info:* Box 2.0 kg je Hand | Inspire: PD hält. Standard: beim Anheben direkt nach START BALANCING einmal abgefangen (Becken 0.5 m). Nach einem Lauf-→-Stand-Wechsel in beiden Szenen einmal abgefangen. Kein Sturz |

`agile_velocity_height_g1` (Abfangen mit `agile_velocity_g1`), Stand
2026-10-08, Standard-Szene / Inspire-Szene, beide bestanden:

| Szenario | Ergebnis |
|---|---|
| Laufen vor/zurück/seitlich/drehen | Ist 0.48 / −0.31 / ±0.38 m/s, Drehen 0.78 rad/s (Soll 0.5 / −0.4 / ±0.4 / 0.8); Nachlauf ≤ 1 cm |
| Laufen mit freien Armen (hängend, ständig bewegt, Box 0.5 und 1.0 kg je Hand) | alle stabil, Tempo 0.38–0.40 m/s bei Soll 0.4; Neigung ≤ 0.14 |
| Policy allein im Stand, 20 s (cmd = 0) | Becken ≤ 6 cm, Füße 4–8 cm (setzt sich zuerst in ihre Hocke) |
| PD-Stand (wie oben, 8 Fälle) | Füße ≤ 3 mm, kein Abfangen |
| WALK → STAND (8 feste + 20 zufällige Wechsel) | 0 Stürze, kein Abfangen; PD nach 0.6–0.9 s, danach Füße ≤ 6 mm |
| START WALKING mit cmd = 0 aus dem PD | Becken ≤ 2.1 cm nach START BALANCING, ≤ 4.9 cm nach einer Übergabe |
| Stöße 80 / 150 / 250 N | 0 Stürze, wie oben |

Die alte Policy (`--policy g1_wholebody`) läuft mit Armen in der Lauf-Pose gut,
fällt aber mit hängenden Armen, mit Box und unter Arm-Bewegung im Stand.

## Bekannte Einschränkungen

- `loco_sim` ist ein reiner **Sim-Stellvertreter**. Die konkrete
  Balance-/Lauf-Strategie ist eigens für die Simulation gebaut und teils auf
  simulationsinterne Größen angewiesen (z. B. Basisgeschwindigkeit/Fußkraft
  in `reserve[]` der Bridge) — sie ist **nicht** für den Realeinsatz gedacht.
- Die Standard-Policy `agile_velocity_g1` läuft mit Fangschritten ohne
  festen Rhythmus (siehe „Gangbild"); das ist ihr Trainingsergebnis, kein
  Fehler der Anbindung.
- Die AGILE-Policy erreicht in MuJoCo etwa 75 % der kommandierten
  Geschwindigkeit (0.36 m/s bei 0.5 m/s vor, 0.3 m/s bei 0.4 m/s seitlich);
  Drehen trifft den Sollwert. Ursache ist der Sim-Unterschied zum Isaac-Lab-Training.
- Ein driftfreier Stand kommt ausschließlich vom modellbasierten PD. Was er
  nicht halten kann (Stöße, sehr schwere Lasten), fängt die Policy mit
  Schritten ab; dabei verlässt der Roboter seinen Platz.
- Die Schwerpunkt-Führung braucht das Sim-Modell (MJCF) und rechnet nur in
  Längsrichtung. Lasten in den Händen kennt sie nicht; die Korrektur über das
  Knöchelmoment ist bewusst langsam (Sekunden).
- BALANCING steht in der Hocke der Policy, die Hände liegen dadurch etwa 5 cm
  tiefer als beim früheren geraden Stand.
- Die Taille (roll/pitch) bewegt sich beim Laufen mit der AGILE-Policy. Die
  Arm-IK rechnet mit fester Taille, darum verschiebt sich eine gehaltene
  Handpose beim Laufen um einige Zentimeter mit dem Oberkörper.
- `USE_JOYSTICK=0` ist zwingend, solange kein Gamepad im Container hängt —
  sonst stirbt der Sim-Thread.

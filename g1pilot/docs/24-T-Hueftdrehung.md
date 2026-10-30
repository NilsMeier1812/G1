# Hüftdrehung — Technik

Richtet sich an Entwickler. Bedienung: [24-A-Hueftdrehung.md](24-A-Hueftdrehung.md).

## Beteiligte Dateien

| Datei | Inhalt |
|---|---|
| `g1pilot/manipulation/waist_yaw.py` | `WaistYaw`: Sollwert aus Rate (Totmann 0,3 s), Bereich, Mitte-Fahrt; Topic-Namen; `rotate_twist_yaw` für den Jog im Oberkörper-Bezug. Ohne ROS testbar |
| `g1pilot/manipulation/waist_hold.py` | `WaistHold`: Taille (Gelenke 12–14) real über `rt/arm_sdk` halten, Rampe, passive Dämpfung im E-Stop |
| `g1pilot/manipulation/arm_controller.py` | führt `WaistYaw`, publiziert `/g1pilot/waist/state`, schiebt GEHEN/Grundstellung/DISABLE auf, bis die Hüfte in der Mitte ist |
| `g1pilot/teleoperation/ps4_arm_teleop.py` | R2 halten + rechter Stick → `/g1pilot/waist/yaw_rate`, R2 + R3 → `/g1pilot/waist/center` |
| `g1pilot/navigation/loco_sim.py` | Sim: setzt `target_rad` im Stand (HOLD/STAND) |
| `g1pilot/navigation/loco_client.py` | Real: `Move()` nur mit 0, solange `walk_ok` false ist |
| `g1pilot/teleoperation/demo_gui.py`, `status_line.py` | Anzeige „Hüfte …“, Knopf **Hüfte Mitte** |
| `test/test_waist_yaw.py`, `test/test_waist_hold.py`, `test/test_demo_gui_waist.py` | Unit-Tests Sollwert, Taille halten, GUI-Anzeige |
| `test_waist_yaw_sim.py` | Ende-zu-Ende gegen die laufende Sim |

## Datenfluss

`ps4_arm_teleop` → `/g1pilot/waist/yaw_rate` (`Float32`, −1..1, + = links,
~30 Hz solange R2 gehalten; beim Loslassen einmal 0) und
`/g1pilot/waist/center` (`Bool`). Der **arm_controller** führt den Sollwert
(`manipulation/waist_yaw.py`: Rate × 15 °/s, Bereich, 0,3 s Totmann,
Mitte-Fahrt) und publiziert `/g1pilot/waist/state` (JSON, 10 Hz:
`available`, `reason`, `owner`, `sim`, `yaw_rad`, `target_rad`, `home_rad`, `offset_deg`,
`meas_offset_deg`, `limit_deg`, `moving`, `centering`, `pending`, `walk_ok`).

- **Real:** Sollwert → `WaistHold.target[yaw]` (rt/arm_sdk, Rampe max.
  0,3 rad/s wie bisher). Nur mit Gewicht 1 und gehaltener Taille.
- **Sim:** `loco_sim` setzt `target_rad` im Stand (HOLD/STAND, Rampe
  0,5 rad/s); die Lauf-Policy führt die Taille selbst. Ohne frischen
  Zustand fährt es langsam auf 0.
- `loco_client` sperrt `Move()` (sendet 0), solange `walk_ok` false ist
  (Zustand jünger als 1 s; ohne arm_controller wie bisher).
- GEHEN / Grundstellung / DISABLE werden aufgeschoben, bis Sollwert und
  Messung in der Mitte sind (±2° / ±3°).

| Parameter `arm_controller` | Default | Bedeutung |
|---|---|---|
| `waist_yaw_control` | `true` | Hüftdrehung an/aus |
| `waist_yaw_limit_deg` | 105 | Bereich um die Mitte |
| `waist_yaw_rate_deg` / `waist_yaw_center_rate_deg` | 15 / 15 | Tempo Stick / zur Mitte [°/s] |
| `waist_yaw_center_tol_deg` | 2 | „in der Mitte“ |
| `waist_yaw_center_timeout_s` | 10 | danach aufgeschobene Aktion trotzdem |

## Taille halten (real)

Das `arm_sdk`-Gewicht schaltet am G1 neben den Armen auch die Taille um.
Deshalb hält der arm_controller die Taille (Gelenke 12–14) real mit
`WaistHold` (kp 150 / kd 4, `G1_WAIST_KP` / `G1_WAIST_KD`), solange er die
Arme führt. Die Hüftdrehung setzt nur das Ziel von Gelenk 12. Hintergrund
und Sicherheitsverhalten: [50-A-Echter-Roboter.md › E](50-A-Echter-Roboter.md#e--guarantees-die-bereits-im-code-stehen).

| Parameter `arm_controller` | Default | Bedeutung |
|---|---|---|
| `waist_hold` | `true` (Env `G1_WAIST_HOLD`) | Taille real halten |
| `waist_joints` | `[12, 13, 14]` | gehaltene Gelenke; `[12]` für G1 mit gesperrter Taille |
| `kp_waist` / `kd_waist` | 150 / 4 | Gains |
| `estop_waist_hold` | `true` (Env `G1_ESTOP_WAIST_HOLD`) | Taille im NOT-HALT gehalten statt passiv |
| `waist_max_vel` / `waist_engage_s` | 0,3 rad/s / 1,5 s | Rampe des Ziels / weiches Zupacken nach Quittung |

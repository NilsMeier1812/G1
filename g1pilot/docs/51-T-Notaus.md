# Notaus — Technik

Richtet sich an Entwickler. Welche Topics die Halt-Wege benutzen und welcher
Node was tut. Bedienung: [51-A-Notaus.md](51-A-Notaus.md).

## Topics

| Topic | Typ | Sender | Empfänger |
|---|---|---|---|
| `/g1pilot/emergency_stop` | Bool | Demo-GUI, `ps4_arm_teleop` (PS), Handy über die Demo-GUI (`phone/link.py`), alter Joystick | `arm_controller`, `loco_client` / `loco_sim`, Inspire-Bridge, `ps4_arm_teleop`, `ps4_walk`, GUIs |
| `/g1pilot/full_emergency_stop` | Bool | `ps4_arm_teleop` (R1 + L1 + PS 2 s) | `arm_controller`, `loco_client` |
| `/g1pilot/start` | Bool | Demo-GUI-Startleiste, Handy | quittiert in `arm_controller`, `loco_client` / `loco_sim` |

Die Demo-GUI hört selbst auf `/g1pilot/emergency_stop`, damit ein NOT-HALT
vom Controller dort angezeigt wird (`demo_gui._external_estop`).

## Wer was tut

| Node | NOT-HALT | Voll-Notaus | `/g1pilot/start` |
|---|---|---|---|
| `arm_controller` (`_on_emergency_stop`) | Latch `estop_active`, Arme schlaff (`_publish_arm_slack`, jedes Tick), `arm_sdk`-Gewicht bleibt 1 (sonst posiert der Onboard-Regler die Arme), geplante Bewegung und Marker-Ziele abbrechen, Hüftdrehung hält auf Sollwert, Taille gehalten (`estop_waist_hold`) | wie NOT-HALT, zusätzlich Taille passiv | Latch weg, Taille wieder halten (Rampe ab Messung); Arme bleiben schlaff bis ENABLE |
| `loco_client` (real) | `_halt_legs`: `StopMove` immer, `Damp` nur mit `estop_damp_legs`; Gehen gesperrt | `Damp` zuerst, dann `StopMove` | Standby; Beine stehen geblieben → Onboard-Modus unverändert; Walk-Modus wird übernommen. Nach Voll-Notaus nur quittieren, bleibt Damp |
| `loco_sim` | Zustand DAMP, Arme aus | — (wirkt über NOT-HALT) | Zustand HOLD |
| Inspire-Bridge | beide Hände `set_enabled(False)`, halten Position | — | — |
| `ps4_arm_teleop` | Latch, LED rot, ignoriert alles außer PS | sendet nach 2 s Halten (mit Vibration) | — |
| `ps4_walk` | Latch, Gehen gesperrt, bis L2 einmal losgelassen | — | — |

RPCs am echten G1 laufen über eigene Callback-Gruppen
(`_estop_group`, `_full_estop_group`) und `_robot_rpc_urgent`, damit ein
hängender Gehbefehl den Halt nicht verzögert. Antwort-Codes stehen im Log
(„EMERGENCY STOP: StopMove (Code 0) …“).

## Parameter

| Env (`docker-compose.yml`) | Parameter | Default | Node |
|---|---|---|---|
| `G1_ESTOP_DAMP_LEGS` | `estop_damp_legs` | `0` / `false` | `loco_client` |
| `G1_ESTOP_WAIST_HOLD` | `estop_waist_hold` | `true` | `arm_controller` |
| `G1_PS4_PS_ESTOP` | `ps_estop` | `1` | `ps4_arm_teleop` |
| `G1_XBOX_PS_BUTTON` | | `xbox` | `ps4_joystick` (Xbox) |
| `G1_PHONE_VIEW_ESTOP` | | `1` | Handy-Gateway |

## Zeitgrenzen ohne Notaus-Taste

| Wo | Grenze | Wirkung |
|---|---|---|
| `ps4_walk` | 0,3 s kein `/joy` | einmal Stopp |
| `loco_client` | 0,5 s kein Gehbefehl | `StopMove` |
| G1-Firmware | `Move()` gilt 1 s | Roboter hält von selbst |
| Handy | 1,5 s kein Status | alles außer NOT-HALT gesperrt ([12-T-Handy.md](12-T-Handy.md#zeiten)) |

## Tests

| Test | Inhalt |
|---|---|
| `test/test_arm_controller_real.py` | NOT-HALT/Voll-Notaus am Fake-G1: Arme schlaff, kein Onboard-Snap, Taille, Quittieren |
| `test/test_loco_client_real.py` | `StopMove` ohne Damp, `estop_damp_legs`, Voll-Notaus-Damp, Quittieren im Walk |
| `test/test_loco_client_ps4_walk.py` | Gehen nach NOT-HALT gesperrt |
| `test/test_ps4_arm_teleop_node.py` | PS, R1 + L1 + PS, Sperre |
| `test/test_ps4_walk_node.py` | Latch und Abriss beim Gehen |
| `test/test_gui_external_estop.py` | Demo-GUI zeigt einen NOT-HALT von außen |
| `test/test_phone_demo_gui.py` | NOT-HALT vom Handy |

Alle in der Gruppe `test/run_tests.sh sicherheit` ([60-A-Tests.md](60-A-Tests.md)).

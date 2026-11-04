# Simulation — Technik

Richtet sich an Entwickler. Wie die MuJoCo-Sim an den ROS-Stack angebunden
ist. Bedienung: [40-A-Simulation.md](40-A-Simulation.md). Die Lauf-Policy
selbst (`loco_sim`) steht in [30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md#loco_sim-simulation).

## Container

| Dienst (`docker-compose.yml`) | Inhalt |
|---|---|
| `mujoco-sim` | `unitree_mujoco/simulate_python/unitree_mujoco.py`: Physik, Viewer, DDS-Bridge. **Kein ROS** |
| `g1pilot-sim` | ROS-2-Stack (`launch/bringup_sim.launch.py`): `robot_state`, `arm_controller`, `loco_sim`, GUI, ggf. Navigation/Controller/Handy |

Beide laufen mit `network_mode: host`. DDS (Domain 1, Interface `lo`)
verbindet `loco_sim`/`arm_controller` mit der Bridge, ROS-freie Zusatzbefehle
laufen als UDP über Loopback.

## Beteiligte Dateien (`unitree_mujoco/simulate_python/`)

| Datei | Inhalt |
|---|---|
| `unitree_mujoco.py` | Hauptschleife: Physik (1 kHz), Viewer (50 fps), Lockstep, startet die Listener |
| `config.py` | Szene (`G1_ENV`, `G1_INSPIRE_HANDS`), Zeitschritt, Lockstep, Basis-Halten, Schubs, Ports |
| `unitree_sdk2py_bridge.py` | `rt/lowstate` publizieren, `rt/lowcmd` + `rt/arm_sdk` mischen (Bridge-Merge, [30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md#bridge-merge-sim)) |
| `hold_base.py` | Basis halten: `weld` / `teleport` / `off` (`HOLD_BASE_MODE`) |
| `push_listener.py` | Schubs per UDP |
| `grasp_box.py` | Testkugel in der Handfläche per UDP an/aus |
| `scene_reset.py` | bewegliche Objekte zurücksetzen per UDP |
| `scene_objects.py`, `scene_state_publisher.py` | Objekte der Szene an `scene_bridge` melden (Marker, Hindernisse) |
| `window_icon.py`, `icons/` | Fenster-Icon und `WM_CLASS` des Viewers |
| `gl_check.py` | warnt, wenn auf der CPU gerendert wird |
| `sim_hold_test.py` | Physik-Testwerkzeug für das Basis-Halten |

Auf der ROS-Seite: `g1pilot/navigation/loco_sim.py` (Policy, Schubs/Grasp-UDP),
`g1pilot/navigation/scene_bridge.py` (Szene → RViz, Szenen-Reset, siehe
[31-T-Navigation.md](31-T-Navigation.md#scene_bridge-nur-sim)),
`g1pilot/teleoperation/sim_autostart.py`.

## Feste Voreinstellungen von `start.sh`

`start.sh` setzt für die Sim immer `HOLD_BASE_MODE=off` (Basis frei, die
Policy trägt den Körper), `SIM_LOCKSTEP=1` und `G1_SIM_MODE=true`.
`run_sim.sh` ohne diese Werte startet die alte Arm-only-Sim mit
festgeschweißtem Oberkörper (`weld`).

## Lockstep

Mit `SIM_LOCKSTEP=1` macht die Sim pro empfangenem `rt/lowcmd` genau
`SIM_CONTROL_DT / SIMULATE_DT` = 0,02 / 0,001 = 20 Physikschritte und
publiziert danach einen frischen `rt/lowstate`. Zusammen mit »ein Kommando
pro lowstate« in `loco_sim` sieht die 50-Hz-Policy auf jedem PC ihre
trainierten 20 ms Physik je Schritt. Nach oben ist das auf Echtzeit
gedeckelt; `SIM_REALTIME_FACTOR` (GUI: Sim-Tempo) setzt die Obergrenze.
Kommt kein Kommando, schreitet ein Watchdog nach `SIM_LOCKSTEP_WATCHDOG_S`
(0,2 s) trotzdem weiter, damit der Viewer nicht einfriert.

## Auto-Start (GREIFEN)

`SimAutoStart` (`g1pilot/teleoperation/sim_autostart.py`, genutzt von der
Demo-GUI, nur bei `is_sim_mode()`):

- schickt `/g1pilot/arms/enabled=True` und `/g1pilot/start_balancing=True`,
- wiederholt alle 1,5 s nur das, was noch nicht bestätigt ist:
  `arm_controller` über `/g1pilot/arms/state` (Bool), `loco_sim` über
  `/g1pilot/loco_state` (`stand` oder `walk`); beide latched,
- gibt nach 120 s ohne Quittung auf (`on_done(False)`),
- bricht ab bei jedem Eingriff des Nutzers (Modus, NOT-HALT, ENABLE,
  Roboter zurücksetzen).

Hintergrund: Bug 2026-10-13. Vorher ging die einmalige Nachricht verloren,
wenn die Regler nach 3 s noch nicht bereit waren. Test:
`test/test_sim_autostart.py`, Ende-zu-Ende `test_sim_startup_grasp.py` und
`run_startup_grasp_loop.sh` ([60-A-Tests.md](60-A-Tests.md#ende-zu-ende-gegen-die-laufende-sim)).

## Umgebungen erzeugen

`start.sh` ruft für die gewählte Umgebung
`unitree_mujoco/scene_editor/build_env_scene.py --env scenes/<name>.xml --inspire 0|1`
auf. Das Ergebnis `unitree_robots/g1/scene_env_<name>.xml` enthält G1
(passende Hand-Variante), Licht, Boden, Weld und die Objekte. Dabei werden
`grasp_*` zu freien Körpern, `g1_spawn` und `station_*` entfernt und als
Startpunkt bzw. Stationen hinterlegt, große Mesh-Hindernisse in konvexe
Teilhüllen zerlegt (`trimesh`, `vhacdx`). `config.py` lädt bei gesetztem
`G1_ENV` diese Szene, sonst `scene.xml` bzw. `scene_inspire_ftp.xml`.
Details: [`scene_editor/README.md`](../../unitree_mujoco/scene_editor/README.md).

## Zusatzbefehle (UDP über Loopback)

| ROS-Topic (`g1pilot-sim`) | umgesetzt von | UDP-Port | Wirkung in der Sim |
|---|---|---|---|
| `/g1pilot/push` (Bool), `/g1pilot/push_charge` (Float32 0..1) | `loco_sim` | 47900 (`SIM_PUSH_PORT`) | Stoß in zufälliger Richtung, Payload = Kraft in N |
| `/g1pilot/grasp_box` (Bool) | `loco_sim` | 47901 (`SIM_GRASP_PORT`) | Testkugel an/aus |
| `/g1pilot/scene_reset` (Bool) | `scene_bridge` | 47903 | bewegliche Objekte auf Startpose |
| — (Sim → ROS) | `scene_state_publisher` → `scene_bridge` | 47902 (`SIM_SCENE_PORT`) | Objektposen für RViz/Hindernisse |

Schubs: 150 N für 0,12 s (`SIM_PUSH_FORCE_N`, `SIM_PUSH_DURATION_S`),
aufgeladen bis `G1_PUSH_MAX_N` (2000 N), gekappt bei `SIM_PUSH_FORCE_CAP_N`
(3000 N). Formel in `g1pilot/utils/push_charge.py`. Von Hand:
`echo reset | nc -u -w0 127.0.0.1 47903`.

GRASP BOX: Bei Inspire-Händen ist die Kugel immer im Modell, aber aus.
`G1_GRASP_TEST=1` startet sie an, `G1_GRASP_BOX=0` lässt sie ganz weg.

## Roboter zurücksetzen

Die Demo-GUI schickt `start_balancing`, `start_walking`, `arms/enabled` je
`False` und danach `/g1pilot/start`. `loco_sim` geht dadurch auf HOLD, die
Bridge stellt den Roboter an den Startpunkt (bzw. `g1_spawn`) und hält ihn
per Weld; der nächste Modus-Klick stellt ihn in die Stand-Pose und gibt ihn
frei.

## Umgebungsvariablen der Sim

| Variable | Default | Wirkung |
|---|---|---|
| `HOLD_BASE_MODE` | `weld` (`start.sh`: `off`) | Basis festhalten oder frei |
| `SIM_LOCKSTEP` | `0` (`start.sh`: `1`) | Gleichschritt mit der Policy |
| `SIM_REALTIME_FACTOR` | `1.0` | Obergrenze Sim-Tempo |
| `SIM_CONTROL_DT`, `SIM_LOCKSTEP_WATCHDOG_S` | `0.02`, `0.2` | Lockstep-Takt, Watchdog |
| `SIM_PUSH_FORCE_N`, `SIM_PUSH_DURATION_S`, `SIM_PUSH_FORCE_CAP_N` | `150`, `0.12`, `3000` | Schubs |
| `SIM_PUSH_PORT`, `SIM_GRASP_PORT`, `SIM_SCENE_PORT` | `47900`, `47901`, `47902` | UDP-Ports |
| `G1_GRASP_TEST`, `G1_GRASP_BOX` | aus, an bei Inspire | Testkugel |

Alle `G1_*`-Variablen: [04-T-Umgebungsvariablen.md](04-T-Umgebungsvariablen.md).

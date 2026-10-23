# Locomotion (Stehen/Laufen) — Anleitung

Richtet sich an: Anwender, die den G1 in der Simulation oder auf echter
Hardware stehen und laufen lassen wollen. Für die interne Funktionsweise
siehe [31_loco_technik.md](31_loco_technik.md).

## Überblick

Die Locomotion (Beine + Taille) ist eine kleine Zustandsmaschine mit vier
Zuständen, gesteuert über den Streamdeck oder ROS-Topics. In der Simulation
übernimmt der Node `loco_sim` die Rolle des Unitree-Onboard-Reglers (den es
in MuJoCo nicht gibt); auf dem echten Roboter läuft weiterhin Unitrees
eigener Onboard-Regler, angesprochen über `loco_client`.

| Zustand | Was passiert | Auslöser |
|---|---|---|
| **HOLD** | Standby: steifer Stand, Basis gehalten | Start / `/g1pilot/start` |
| **BALANCE** (Sim: STAND) | Modellbasierter Stand am Platz, Oberkörper frei bewegbar | **START BALANCING** / `/g1pilot/start_balancing` |
| **RUN** (Sim: WALK) | Lauf-Regler; Geschwindigkeit per Joystick | **WALK** + `/g1pilot/loco_cmd_vel` |
| **DAMP** | Notaus: alle Motoren weich | **EMERGENCY** / `/g1pilot/emergency_stop` |

## Bedienung

**Typischer Ablauf:**

1. **START BALANCING** → der Roboter steht frei und balanciert; die Arme
   lassen sich parallel über die Marker bewegen. In der Sim bleiben die Füße
   dabei stehen, auch mit bewegten Armen oder einer Box in den Händen. Erst
   wenn er wirklich kippt (z. B. nach einem Stoß, ein Fuß hebt ab), fängt er
   sich mit ein paar Schritten und steht danach wieder still. Er steht in der
   leicht gebeugten Haltung des Lauf-Reglers, damit er ohne Satz loslaufen kann.
2. **WALK** → der Bildschirm-Joystick (bzw. echter Controller auf real) gibt
   Vorwärts/Seitwärts/Drehen vor.
3. Joystick loslassen → der Roboter bremst ab und bleibt stehen. In der Sim
   steht dann die Lauf-Policy am Platz und macht kleine Ausgleichsschritte.
   Für den stationären Stand mit geplanten Füßen **START BALANCING** drücken:
   Die Sim lässt die Policy erst ausbremsen und übergibt nach etwa einer
   Sekunde an den Stand-Regler.

Über die Kommandozeile:

```bash
ros2 topic pub --once /g1pilot/start_balancing std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /g1pilot/start_walking   std_msgs/msg/Bool "{data: true}"
ros2 topic pub /g1pilot/loco_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}"   # vorwärts
ros2 topic pub --once /g1pilot/loco_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"  # Stop
```

`loco_cmd_vel` ist normiert `[-1, 1]`; `linear.x` = vorwärts/rückwärts,
`linear.y` = seitwärts, `angular.z` = drehen. Die tatsächliche
Geschwindigkeit ergibt sich aus den konfigurierten Limits (siehe unten).

### Lauf-Regler wählen (nur Sim)

Welche Lauf-Policy die Sim benutzt, wählt man im Startmenü
(Simulation → „Lauf-Policy"); die Wahl wird gemerkt. Im Text-Menü ist es
Frage 4, ohne Menü die Umgebungsvariable `G1_WALK_POLICY`:

```bash
G1_WALK_POLICY=agile_velocity_height_g1 ./start.sh --yes   # gleichmäßiger AGILE-Gang
```

| Wert | Wirkung |
|---|---|
| `g1_wholebody` (Standard, „Alt") | natürlicher, gleichmäßiger Gang; die Arme fahren beim Laufen automatisch in die Lauf-Pose |
| `agile_velocity_g1` („AGILE Velocity") | sehr robust, Arme frei; der Gang sieht aber nach Stolpern aus (Fangschritte ohne festen Rhythmus) |
| `agile_velocity_height_g1` („AGILE Velocity-Height") | gleichmäßiger Gang, trifft das Tempo, bis 1 m/s vorwärts, Arme frei; Stöße im Stand fängt `agile_velocity_g1` ab |

Ob die Arme beim Laufen geparkt werden, stellt der Sim-Start passend zur
Policy ein. Im Stand hält bei allen dreien derselbe Stand-Regler die Füße fest.
Vergleichswerte stehen in [31_loco_technik.md](31_loco_technik.md#gangbild).

## Sim ↔ Real

| Aspekt | Simulation | Echter Roboter |
|---|---|---|
| Regler | `loco_sim` (RL-Lauf-Policy + modellbasierter Stand-Regler) | Unitrees Onboard-High-Level, über `loco_client` |
| Node-Start | automatisch mit `bringup_sim.launch.py` | automatisch mit `bringup_real.launch.py` |
| Steuertopics | identisch | identisch |

Der Streamdeck verhält sich in beiden Modi identisch — dieselben Buttons,
dieselben Topics.

**Wichtig:** `loco_sim` ist ein **reiner Sim-Stellvertreter**. Auf dem
echten G1 übernimmt ausschließlich Unitrees Onboard-Regler das
Stehen/Laufen; für Details zur echten Hardware siehe
[70_echtroboter_anleitung.md](70_echtroboter_anleitung.md).

## Notaus

**EMERGENCY STOP** versetzt die Beine sofort in `Damp()` — alle Motoren
werden weich, der Roboter sackt kontrolliert zusammen. Für sanftes Anhalten
im laufenden Betrieb stattdessen **START BALANCING** drücken (der Roboter
bleibt stehen). Details und Sicherheitsablauf für echte Hardware:
[70_echtroboter_anleitung.md](70_echtroboter_anleitung.md).

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| WALK-Klick ohne Wirkung | Erst START BALANCING, dann WALK — WALK wird sonst mit Warnung ignoriert. |
| Roboter fällt beim Eintritt in BALANCE | (Sim) Prüfen, ob `USE_JOYSTICK=0` gesetzt ist, sofern kein Gamepad im Container hängt. |
| Roboter reagiert nicht auf den Joystick | Ist WALK aktiv (nicht nur BALANCING)? Kommt der `loco_cmd_vel`-Stream regelmäßig an (Deadman-Timeout)? |
| (Sim) Roboter macht im BALANCING bei Arm-Bewegung Schritte | Im Log von `loco_sim` steht dann „Schwerpunkt-Modell nicht geladen": Der `g1pilot-sim`-Container sieht das Robotermodell nicht (Mount in `docker-compose.yml`) oder Pinocchio ist älter als 3. Container mit `--build` neu bauen. |
| Arme hängen seltsam beim Loslaufen | Echter Roboter: normal für kurze Zeit, die Arme werden erst in eine definierte Lauf-Pose gebracht, bevor der Lauf-Regler startet. In der Sim bleiben die Arme frei und behalten ihre Stellung. |
| Echter G1 driftet beim Gehen zu einer Seite | Ursachen und Testplan: `/mnt/project-files/analysen/linksdrall_gehen_g1.md`. Messen mit der Geh-Diagnose (unten). |

## Geh-Diagnose am echten G1 (`walk_diag`)

Nur lesend: sendet nichts an den Roboter (kein RPC, kein `rt/arm_sdk`, kein
`rt/lowcmd`) und laeuft als eigener Prozess neben dem Stack. Zeichnet mit 50 Hz
auf: IMU, Beine + Taille (Stellung, Moment, Temperatur, Fehlerbits), Armstellung,
Funk-Fernbedienung, die Sollwerte des Onboard-Reglers (`rt/lowcmd`, falls
sichtbar), Onboard-Odometrie (falls ein `*modestate`-Topic sichtbar ist) und die
Geh-Befehle bzw. den FSM-Zustand aus ROS.

```bash
# Je Versuch eine Aufnahme (Strg+C beendet), Interface = ROBOT_INTERFACE:
docker compose exec g1pilot-real python3 -m g1pilot.tools.walk_diag \
    record enp0s31f6 --label geradeaus_arme_onboard --note "Halle, Langsam"
# Auswerten (Stand: Lastverteilung/IMU-Neigung; je Geh-Abschnitt Befehl vs. Odometrie,
# Blickrichtung, Momente und Regelabweichung links/rechts, Temperaturen):
docker compose exec g1pilot-real python3 -m g1pilot.tools.walk_diag \
    analyze diag_logs/walk_diag_*_geradeaus_arme_onboard.csv
```

Die Dateien liegen auf dem Host in `g1pilot/diag_logs/` (CSV + JSON mit Label,
Limits und G1_-Umgebung). Ungeprueft am G1: ob `rt/lowcmd` und ein Odometrie-Topic
(`rt/odommodestate`, `rt/sportmodestate`, ...) sichtbar sind -- die Aufnahme nennt
beim Start alle `rt/`-Topics mit Schreiber; ohne sie fehlen nur die Spalten.

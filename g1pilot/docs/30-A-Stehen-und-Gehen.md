# Stehen und Gehen — Anleitung

Richtet sich an: Anwender, die den G1 in der Simulation oder auf echter
Hardware stehen und laufen lassen wollen. Für die interne Funktionsweise
siehe [30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md).

## Überblick

Die Locomotion (Beine + Taille) ist eine kleine Zustandsmaschine mit vier
Zuständen, gesteuert über die Demo-GUI (Kacheln GEHEN / GREIFEN), den
Controller (L2 / R2) oder ROS-Topics. In der Simulation
übernimmt der Node `loco_sim` die Rolle des Unitree-Onboard-Reglers (den es
in MuJoCo nicht gibt); auf dem echten Roboter läuft weiterhin Unitrees
eigener Onboard-Regler, angesprochen über `loco_client`.

| Zustand | Was passiert | Auslöser |
|---|---|---|
| **HOLD** | Standby: steifer Stand, Basis gehalten | Start / `/g1pilot/start` |
| **BALANCE** (Sim: STAND) | Modellbasierter Stand am Platz, Oberkörper frei bewegbar | Demo-GUI **GREIFEN**, Controller **R2** / `/g1pilot/start_balancing` |
| **RUN** (Sim: WALK) | Lauf-Regler; Geschwindigkeit per Knopf/Stick | Demo-GUI **GEHEN**, Controller **L2** / `/g1pilot/start_walking` + `/g1pilot/loco_cmd_vel` |
| **DAMP** | Sim: Notaus, alle Motoren weich | NOT-HALT / `/g1pilot/emergency_stop` |

Am **echten G1** heißen die Onboard-Zustände wie in der Unitree-App: Damp
(FSM 1), Preparation (FSM 4) und Walk (FSM 500/501). Dorthin schaltet die
**Startleiste** der Demo-GUI (*Start | Damp | Preparation | Walk*), Ablauf am
Kran in [50-A-Echter-Roboter.md](50-A-Echter-Roboter.md).

## Bedienung

**Typischer Ablauf:**

1. **GREIFEN** (Demo-GUI, intern `start_balancing`) → der Roboter steht frei und balanciert; die Arme
   lassen sich parallel über die Marker bewegen. In der Sim bleiben die Füße
   dabei stehen, auch mit bewegten Armen oder einer Box in den Händen. Erst
   wenn er wirklich kippt (z. B. nach einem Stoß, ein Fuß hebt ab), fängt er
   sich mit ein paar Schritten und steht danach wieder still. Er steht in der
   leicht gebeugten Haltung des Lauf-Reglers, damit er ohne Satz loslaufen kann.
2. **GEHEN** → der Bildschirm-Joystick bzw. die Pfeiltasten der Demo-GUI
   (Tempo Langsam/Normal) oder der Controller mit **gehaltenem L2** geben
   Vorwärts/Seitwärts/Drehen vor ([11-A-Controller.md › Gehen mit L2](11-A-Controller.md#gehen-mit-l2)).
3. Joystick loslassen → der Roboter bremst ab und bleibt stehen. In der Sim
   steht dann die Lauf-Policy am Platz und macht kleine Ausgleichsschritte.
   Für den stationären Stand mit geplanten Füßen **GREIFEN** wählen:
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
Vergleichswerte stehen in [30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md#gangbild).

## Sim ↔ Real

| Aspekt | Simulation | Echter Roboter |
|---|---|---|
| Regler | `loco_sim` (RL-Lauf-Policy + modellbasierter Stand-Regler) | Unitrees Onboard-High-Level, über `loco_client` |
| Node-Start | automatisch mit `bringup_sim.launch.py` | automatisch mit `bringup_real.launch.py` |
| Steuertopics | identisch | identisch |

Die Demo-GUI verhält sich in beiden Modi gleich — dieselben
Knöpfe, dieselben Topics. Nur real gibt es die Startleiste und die
Bodenkontakt-Prüfung vor »Walk«; nur in der Sim die Wahl der Lauf-Policy.

**Wichtig:** `loco_sim` ist ein **reiner Sim-Stellvertreter**. Auf dem
echten G1 übernimmt ausschließlich Unitrees Onboard-Regler das
Stehen/Laufen; für Details zur echten Hardware siehe
[50-A-Echter-Roboter.md](50-A-Echter-Roboter.md).

## Notaus

**NOT-HALT** (Demo-GUI, EMERGENCY STOP, Controller-PS, Handy):

- **Sim:** `loco_sim` geht in DAMP, der ganze Roboter wird weich.
- **Echter G1:** die Beine bekommen nur `StopMove`, **kein** `Damp()`; der
  Onboard-Regler balanciert weiter (seit 2026-10-15). Den ganzen G1 in Damp
  setzt nur der **Voll-Notaus** (Controller R1 + L1 + PS 2 s halten) oder die
  Startleiste *Damp*.

Für sanftes Anhalten im laufenden Betrieb Stick bzw. Knopf loslassen oder
GREIFEN wählen (der Roboter bleibt stehen). Übersicht aller Halt-Wege:
[51-A-Notaus.md](51-A-Notaus.md).

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| GEHEN ohne Wirkung | Sim: erst stehen (GREIFEN bzw. Auto-Start abwarten); real: Startleiste auf Walk. Walk wird sonst mit Warnung ignoriert. |
| Roboter fällt beim Eintritt in BALANCE | (Sim) Prüfen, ob `USE_JOYSTICK=0` gesetzt ist, sofern kein Gamepad im Container hängt. |
| Roboter reagiert nicht auf den Joystick | Ist WALK aktiv (nicht nur BALANCING)? Kommt der `loco_cmd_vel`-Stream regelmäßig an (Deadman-Timeout)? |
| (Sim) Roboter macht im BALANCING bei Arm-Bewegung Schritte | Im Log von `loco_sim` steht dann „Schwerpunkt-Modell nicht geladen": Der `g1pilot-sim`-Container sieht das Robotermodell nicht (Mount in `docker-compose.yml`) oder Pinocchio ist älter als 3. Container mit `--build` neu bauen. |
| Arme fahren beim Loslaufen in die Lauf-Pose | Sim mit der Standard-Policy `g1_wholebody` (oder real mit `G1_WALK_PARK_ARMS=true`): gewollt. Mit AGILE-Policy (Sim) bzw. real seit 2026-10-19 bleiben die Arme in ihrer Pose. |
| Real: »Walk« gesperrt „kein Bodenkontakt“ | Füße tragen zu wenig Last (Knie-Last unter 10 Nm) → Kran weiter lockern, siehe [50-A-Echter-Roboter.md](50-A-Echter-Roboter.md#fehlerbehebung). |
| Real: GEHEN per L2 tut nichts, lange Vibration | G1 steht nicht im Walk-Modus → erst Startleiste »Walk«. |

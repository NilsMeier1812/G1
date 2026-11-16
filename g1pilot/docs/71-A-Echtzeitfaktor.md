# V4 Echtzeitfaktor und Takttreue — Anleitung

## Zweck

Misst über eine einstellbare Dauer (Standard 10 min je Lastfall) die
simulierte Zeit je verstrichener Wanduhrzeit (Echtzeitfaktor) und für jeden
Regelkreis die tatsächliche Rate und die Streuung der Zykluszeit gegenüber
der Sollrate:

| Kreis | gemessen an | Sollrate | Zeitbasis |
|---|---|---|---|
| Physik | `rt/lowstate.tick` (Sim-Zeit) | 1000 Hz (`SIMULATE_DT` 1 ms) | Sim |
| `sim_zustand` | Ankunft `rt/lowstate` | Lockstep 50 Hz | Sim |
| `lauf_regler` (loco_sim) | Ankunft `rt/lowcmd` | 50 Hz | Sim |
| `arm_regler` (arm_controller) | Ankunft `rt/arm_sdk` | 250 Hz | Wanduhr |
| `roboterzustand` (robot_state) | Ankunft `/joint_states` | wie `rt/lowstate` | Sim |
| `tf` | Ankunft `/tf` | mehrere Sender, kein Soll | – |

Drei Lastfälle: **stillstand** (GREIFEN, keine Bewegung), **arm** (beide
Arme fahren geplant zwischen zwei Posen hin und her) und **gehen** (GEHEN,
Kreis mit 0,3 m/s und 0,3 rad/s). Die Last erzeugt das Werkzeug mit
denselben Topics wie die Demo-GUI, **nur in der Simulation**. Am echten G1
gibt es nur den Lastfall **passiv** (das Werkzeug sendet nichts).

## Vorbereitung

Messpunkt für die Sim-Zeit einschalten (sonst wird sie im Lockstep aus der
Zahl der `rt/lowstate`-Nachrichten geschätzt):

```bash
G1_EVAL_SIM_TIME=1 ./start.sh      # oder in .env / beim docker compose up setzen
```

## Aufruf

```bash
python3 tools/eval/v4_echtzeit.py                       # 3 × 10 min
python3 tools/eval/v4_echtzeit.py --dauer 120 --faelle arm
python3 tools/eval/v4_echtzeit.py --faelle passiv       # realer G1, nur lesend
```

Wichtige Optionen: `--dauer` (s je Fall), `--vorlauf` (Einschwingen, 10 s),
`--arm-rate`, `--state-rate`, `--sim-dt`, `--walk-vx`, `--walk-wz`.

## Ausgabe

| Datei | Inhalt |
|---|---|
| `ereignisse.csv` | Ankunftszeit jeder Nachricht je Kreis und Fall (Rohdaten), dazu `tick` |
| `kreise.csv` | je Fall und Kreis: Rate (Wand- und Sim-Zeit), Verhältnis Ist/Soll, Zykluszeit Mittel/Std/P50/P95/P99/Max, Abweichung zur Sollperiode, Anteil Zyklen > 1,5× und > 2× Soll |
| `echtzeitfaktor.csv` | Echtzeitfaktor und Physikrate je 1-s-Fenster |
| `summary.json` | je Fall: Echtzeitfaktor gesamt, Methode der Sim-Zeit, Verteilung der Fensterwerte, Kreise, Armfahrten |
| `diagramme.pdf` | Echtzeitfaktor über der Zeit, Histogramme der Zykluszeiten |

## Grenzen

- Die Zykluszeit ist der Abstand der **Ankunftszeiten** im Messprozess. Sie
  enthält die Zustellung über DDS/ROS, nicht nur den Takt des Senders.
- Im Lockstep laufen `rt/lowstate`, `rt/lowcmd` und `/joint_states`
  absichtlich mit der Sim-Zeit; ihr Soll in Wandzeit ist Sollrate ×
  Echtzeitfaktor. Der Armregler läuft auf einem Wanduhr-Timer.
- Das Werkzeug läuft auf demselben Rechner und belastet ihn leicht mit.
- Am echten G1 sendet der Herstellerregler kein `rt/lowcmd` über DDS: dort
  entfallen Lauf-Regler, Physik und Echtzeitfaktor.

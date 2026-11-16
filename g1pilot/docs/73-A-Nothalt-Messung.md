# V8 Not-Halt, Ausfall und Zielsprung — Anleitung

## Zweck

Misst gegen die Attrappe (`FakeG1` aus `test/test_arm_controller_real.py`)
und in der Simulation:

- **Not-Halt je Bedienweg**: Zeit vom Auslösen bis zum geänderten Befehl an
  der Herstellerschnittstelle (erstes `rt/arm_sdk`-Kommando mit Gewicht > 0
  und allen Arm-kp = 0).
- **Ausfall** eines Bedienwegs bzw. der Verbindung: Zeit bis die Bewegung
  hält (Kommandos ändern sich 0,2 s lang um höchstens 1e-4 rad).
- **Zielsprung**: größte Geschwindigkeit der Hand, wenn das Hand-Ziel
  springt (Vorwärtskinematik der kommandierten Gelenkwinkel).

Je Fall mindestens 100 Wiederholungen (Standard `--wdh 100`); ausgegeben
werden Median, 95-%-Wert und Maximum (plus volle Verteilung).

| Fall | Auslösen | Antwort |
|---|---|---|
| `nothalt_topic` | Bool auf `/g1pilot/emergency_stop` (so senden Demo-GUI-Knopf und Handy) | erstes Slack-Kommando |
| `nothalt_ps` | Joy-Nachricht mit PS-Taste an `ps4_arm_teleop` | erstes Slack-Kommando |
| `vollnotaus` | R1 + L1 + PS gehalten (2 s) | erstes Slack-Kommando (der PS-Druck löst schon den normalen NOT-HALT aus), Zeit bis `/g1pilot/full_emergency_stop`, Zeit bis die Taille schlaff ist; netto = minus Haltezeit |
| `ausfall_joy` | Joy-Strom reißt während eines Jogs ab | Halt |
| `ausfall_ziel` | Strom von Hand-Zielen (`/g1pilot/hand_goal`) reißt ab | Halt |
| `ausfall_zustand` | nur Attrappe: `rt/lowstate` liefert während einer geplanten Fahrt nichts Neues | Halt (oder keiner) |
| `zielsprung` | Hand-Ziel springt um `--sprung` (15 cm), zufällige Richtung | größte Handgeschwindigkeit (je Kommando und über 20 ms) |

Vor `ausfall_joy`, `ausfall_ziel` und `zielsprung` fahren die Arme geplant
in eine feste Ausgangslage, damit die Hand über die Wiederholungen nicht
abwandert.

## Aufruf

**Attrappe** (kein Roboter, keine Sim nötig; braucht ROS 2, pinocchio und
pytest wie die Tests):

```bash
python3 tools/eval/v8_nothalt.py --ziel attrappe                 # alle Fälle, je 100
python3 tools/eval/v8_nothalt.py --ziel attrappe --faelle nothalt_topic --wdh 200
```

Der `arm_controller` läuft im Realbetriebs-Modus mit den Parametern aus
`bringup_real` (u. a. `ee_velocity_limit` 0,25 m/s, `arm_accel_limit` 3,0,
`max_tracking_error` 0,3) in einer abgeschotteten ROS-Domain
(`--domain`, Standard 87, nur localhost). `ps4_arm_teleop` startet als
eigener Prozess in derselben Domain.

**Simulation** (nur Sim, das Werkzeug sendet Befehle):

```bash
HOLD_BASE_MODE=weld G1_PS4_ARMS=1 ./start.sh     # Basis fest, Controller-Node an
python3 tools/eval/v8_nothalt.py --ziel sim
```

Mit freier Basis (Loco-Modus) daempft loco_sim beim NOT-HALT die Beine und
der G1 fällt um; deshalb `weld`.

Optionen: `--faelle`, `--wdh`, `--seed`, `--vollnotaus-halten`,
`--jog-dauer`, `--ziel-tempo`, `--halt-timeout`, `--sprung`, `--sprung-dauer`.

## Ausgabe

| Datei | Inhalt |
|---|---|
| `wiederholungen.csv` | je Wiederholung: Antwortzeit (ms), netto, bis Topic, bis Taille schlaff, Handgeschwindigkeit, Weg der Hand, „bewegt vor Ausfall“, Fehler |
| `summary.json` | je Fall: Verteilung (n, Median, P95, Max …), Fehler, Wiederholungen ohne Antwort, Anteil über `ee_velocity_limit` |
| `diagramme.pdf` | Boxplots der Antwortzeiten (Whisker = Min/Max), Histogramm Zielsprung |

## Grenzen

- Attrappe: die Zeit enthält ROS-Zustellung im selben Rechner und den
  250-Hz-Takt des `arm_controller`, nicht die Strecke zum Roboter und nicht
  dessen Reaktion. Die Attrappe ist ein einfaches Mischmodell, keine Physik.
- Handy: gemessen ist der gemeinsame Weg ab `/g1pilot/emergency_stop`; die
  Strecke Handy → Bluetooth → Demo-GUI fehlt.
- Voll-Notaus: die Beine (Damp über `loco_client` bzw. `loco_sim`) sind nicht
  Teil der Messung.
- `ausfall_zustand`: der `arm_controller` hat keinen eigenen Wächter für
  ausbleibende Zustandsdaten. Was die Messung als Halt zeigt, ist die
  Schleppfehler-Begrenzung (`max_tracking_error`) bzw. das Fahrtende.
- Sim: `rt/arm_sdk` wird mitgelesen; die Zeit enthält die DDS-Zustellung.
  Kommen Nachrichten gebündelt an, wird der Zeitabstand zweier Kommandos
  winzig und die Geschwindigkeit „je Kommando“ unsinnig groß. Aussagekräftig
  ist dort der Wert über 20 ms (`v_hand_max_20ms_mps`); beide werden ausgegeben.
- Sim: die Taille hält der `arm_controller` dort nicht (`waist_hold` nur
  real), „bis Taille schlaff“ bleibt leer; netto rechnet ab dem Topic
  `/g1pilot/full_emergency_stop`.
- Grenze für den Zielsprung: `ee_velocity_limit` 0,25 m/s (real/Attrappe),
  0,5 m/s (Sim, `bringup_sim`).

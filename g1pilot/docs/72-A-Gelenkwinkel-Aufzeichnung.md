# V7 Gelenkwinkel aufzeichnen und vergleichen — Anleitung

## Zweck

Spielt eine gespeicherte Posenfolge der Arme ab und zeichnet Soll- und
Ist-Gelenkwinkel mit Zeitstempel auf, mit identischem Ablauf in der
Simulation und am echten G1. Ein Auswerteskript gleicht zwei Aufzeichnungen
(Simulation gegen Roboter) zeitlich ab und berechnet je Gelenk die mittlere
und größte Abweichung sowie per Vorwärtskinematik die Abweichung der
Handposition am Ende jeder Bewegung. Diagramm über der Zeit als PDF.

## Aufzeichnen

Die Posenfolge ist ein Ablauf aus dem Posen-Speicher
([23-A-Posen-und-Ablaeufe.md](23-A-Posen-und-Ablaeufe.md)), z. B.
„Arbeitsplatz 1“. Relative Bewegungen sind nicht erlaubt, weil ihr Ziel von
der aktuellen Handlage abhängt.

**Am echten G1 (nur lesend):**

```bash
python3 tools/eval/v7_gelenkwinkel_aufzeichnen.py --ablauf "Arbeitsplatz 1"
```

Das Werkzeug wartet, bis der Bediener den Ablauf in der Demo-GUI startet
(**Posen & Abläufe** → Ablauf, Modus **Am Stück**), und zeichnet bis zum
Ende des Ablaufs auf. Es sendet nichts.

**In der Simulation** geht dasselbe; zusätzlich kann das Werkzeug den Ablauf
selbst starten (gleiche Logik wie die Demo-GUI, `sequence_runner.py`):

```bash
python3 tools/eval/v7_gelenkwinkel_aufzeichnen.py --ablauf "Arbeitsplatz 1" --trigger auto
```

Optionen: `--posen P1 P2 …` statt eines Ablaufs, `--pose-store`,
`--vorlauf`, `--nachlauf`, `--max-dauer`, `--warte-start`.

Gleiche Ausgangslage: Für einen fairen Vergleich vor beiden Aufzeichnungen
dieselbe Pose anfahren (der erste Schritt startet sonst aus verschiedenen
Lagen).

### Ausgabe der Aufzeichnung

| Datei | Inhalt |
|---|---|
| `ist.csv` | aus `rt/lowstate`: Zeit, `tick`, Taille (3), q/dq/tau der 14 Armgelenke |
| `soll.csv` | aus `rt/arm_sdk`: Zeit, Blend-Gewicht, q und kp der 14 Armgelenke |
| `ereignisse.csv` | `pose_store/goto`, `arm_command/status`, `sequence/state` mit Zeit |
| `summary.json` | Schrittliste mit Zeiten (goto, accepted, executing, Ende, Zustand), Gelenkwerte der Posen aus der Pose-Datei, Raten |

## Vergleichen

```bash
python3 tools/eval/v7_auswerten.py <ordner_sim> <ordner_real> --name-a Simulation --name-b Roboter
```

- `--abgleich schritt` (Standard): jeder Schritt wird auf seinen
  Fahrtbeginn (`executing`) gelegt; verglichen wird bis zum nächsten
  Schritt, so lang wie das kürzere Fenster. `--abgleich start`: nur der
  erste Schritt wird gleichgesetzt (zeigt Laufzeit-Drift).
- `--ab-schritt N`: erst ab Schritt N vergleichen.
- `--raster` (100 Hz): gemeinsames Zeitraster, lineare Interpolation, keine Filter.
- `--no-fk`: ohne Vorwärtskinematik (ohne pinocchio).

Vor dem Vergleich prüft das Skript, ob beide Aufzeichnungen dieselbe
Posenfolge mit denselben Gelenkwerten hatten und alle Schritte erreicht
wurden. Abweichungen stehen unter `hinweise`.

### Ausgabe des Vergleichs

| Datei | Inhalt |
|---|---|
| `gelenke.csv` | je Armgelenk: mittlere, größte und RMS-Abweichung Ist A − Ist B (rad und Grad), Folgefehler Soll − Ist je Aufzeichnung |
| `schritte.csv` | je Schritt: Fensterlänge, mittlere und größte Abweichung, Gelenk mit der größten, Unterschied der Ausgangslage |
| `hand_ende.csv` | je Schritt und Arm: TCP-Lage A und B am Ende der Bewegung, Abstand A–B und je zur Zielpose (mm) |
| `verlauf.csv` | abgeglichene Verläufe auf dem Raster |
| `summary.json` | alles zusammengefasst, Schrittdauern A/B |
| `diagramme.pdf` | Ist A, Ist B, Soll A je Gelenk über der Zeit; Abweichung über der Zeit |

## Grenzen

- Zeitstempel sind die Ankunftszeiten im Messprozess (monotone Uhr).
- Ist kommt in der Sim mit 50 Hz (Lockstep), am echten G1 mit 500 Hz;
  verglichen wird auf dem gemeinsamen Raster.
- Die Vorwärtskinematik nutzt das URDF des Planers und den TCP-Rahmen der
  Kollisionsprüfung (`ArmGeometry.fk`), im Becken-Rahmen mit gemessener Taille.
- Am echten G1 startet der Bediener den Ablauf. Schritt-Zeitpunkte kommen
  aus den Statusmeldungen des `arm_controller` und sind daher genau so gut
  wie deren Zustellung.

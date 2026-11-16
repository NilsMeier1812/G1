# V6 Messreihe Armplaner — Anleitung

## Zweck

Ausführliche Messreihe des eigenen Armplaners
(`g1pilot/manipulation/arm_planner.py`, siehe
[20-T-Arm-Manipulation.md](20-T-Arm-Manipulation.md)) mit einem festen Satz
von 34 Testfällen und zufälligen Start-Ziel-Paaren mit festem Seed, je Fall
10 Wiederholungen. Der Planer läuft unverändert im Messprozess; gemessen
wird über Hüllen um seine Stufen, nur in diesem Prozess.

| Gruppe | Fälle | Inhalt |
|---|---|---|
| `frei` | F1–F6 | freier Raum, ohne Hindernisse |
| `tisch` | T1–T6 | Arbeitsplatz-Tisch, Ziele über dem Tisch |
| `hindernis_mitte` | H1–H5 | Würfel an der Hand-Lage der Gelenkraum-Mitte, Start und Ziel bleiben frei |
| `ziel_nah_tisch` | N1–N4 | Ziel wenige Millimeter über der Tischplatte |
| `hand_auf_tisch` | A1–A4 | Start auf dem Tisch, Ziel darüber und umgekehrt |
| `beidarmig` | B1–B5 | beide Arme gemeinsam (14 DOF) |
| `unerreichbar` | U1–U4 | IK zu weit, Wand, Tisch im Ziel, Selbstkollision im Ziel |
| `zufall` | Z… | `--random` Paare (Standard 30), Seed `--seed` (12345) |

## Aufruf

```bash
python3 tools/eval/v6_armplaner.py                     # alles, 10 Wiederholungen
python3 tools/eval/v6_armplaner.py --only tisch beidarmig --reps 3
python3 tools/eval/v6_armplaner.py --param clearance=0.04 --param time_budget_s=1.5
```

Braucht pinocchio und numpy (Sim-Container oder Host mit `pip install pin`),
kein ROS, keinen laufenden Stack.

## Je Lauf gemessen

Erfolg und Grund (`direct`, `optimized`, `optimized_reduced`, `rrt…`,
`goal_in_collision`, `no_path_found`, `ik_failed`); Planungszeit gesamt und
je Stufe (direkt, Optimierer, RRT-Connect-Rückfall, reduzierter Abstand,
Übergang über die Lauf-Pose, Zeitparametrierung, sonstiges); kleinster
Abstand zu Objekten in der Bahnmitte (und gesamt, am Ende);
Selbstkollisionsfreiheit nach dichter Nachprüfung (≤ 0,02 rad je Schritt);
Bahnlänge im Gelenkraum und für die Hand; Glattheit (größte Beschleunigung
und größter Ruck, Gelenk und Hand); Einhaltung von Gelenk-, Tempo- und
Beschleunigungsgrenzen; Dauer der zeitparametrierten Bahn.

## Ausgabe

| Datei | Inhalt |
|---|---|
| `faelle.csv` | Fallbeschreibung, Gruppe, Start/Ziel, Szene |
| `laeufe.csv` | ein Lauf je Zeile mit allen Messgrößen |
| `summary.json` | je Gruppe: Erfolgsquote, Gründe, Planungszeit (Median, Spannweite, Verteilung), Stufenzeiten, Kennzahlen |
| `diagramme.pdf` | Erfolgsquote je Gruppe, Planungszeit je Gruppe (Boxplot), Stufenanteile, Verteilungen der Bahnkennzahlen |

## Grenzen

- Ableitungen (Tempo, Beschleunigung, Ruck) sind finite Differenzen auf den
  abgetasteten Bahnpunkten des Planers. Übergänge zwischen Segmenten
  ungleicher Länge erzeugen Sprünge; die Werte werden so ausgegeben.
- Planungszeit ist Wanduhrzeit im Messprozess (Budget des Planers ist
  CPU-Zeit). Die Messung belastet nur einen Kern; andere Last auf dem
  Rechner verfälscht die Zeiten.
- Die Szenen sind Quader im Planer-Format, keine MuJoCo-Szenen.

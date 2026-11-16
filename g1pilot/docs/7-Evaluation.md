# 7 · Evaluation

Messwerkzeuge für die Evaluation (Bachelorarbeit). Die Werkzeuge **messen
nur**, sie ändern das Verhalten des Systems nicht. Am echten G1 sind sie
lesend: sie senden dort keine Befehle an den Roboter.

| Versuch | Werkzeug | Läuft in | Anleitung |
|---|---|---|---|
| **V1** Code-Vergleich Simulation/Realbetrieb | `v1_code_vergleich.py` | offline (Quelltext) | [70-A](70-A-Code-Vergleich-Sim-Real.md) |
| **V4** Echtzeitfaktor und Takttreue | `v4_echtzeit.py` | Sim; real nur passiv | [71-A](71-A-Echtzeitfaktor.md) |
| **V7** Gelenkwinkel aufzeichnen und vergleichen | `v7_gelenkwinkel_aufzeichnen.py`, `v7_auswerten.py` | Sim und realer G1 (lesend) | [72-A](72-A-Gelenkwinkel-Aufzeichnung.md) |
| **V8** Not-Halt, Ausfall, Zielsprung | `v8_nothalt.py` | Attrappe (FakeG1) und Sim | [73-A](73-A-Nothalt-Messung.md) |
| **V6** Messreihe Armplaner | `v6_armplaner.py` | offline (Planer im eigenen Prozess) | [74-A](74-A-Armplaner-Messreihe.md) |
| Erweiterbarkeit (Beispielprojekt) | `beispiele/arm_winken/` | Sim und realer G1 | [75-A](75-A-Erweiterungsbeispiel.md) |

## Gemeinsam für alle Werkzeuge

- Skripte unter `g1pilot/tools/eval/`, Aufruf mit `--help`.
- Ausgabe nach `g1pilot/data/eval/<versuch>/<datum-uhrzeit>/` (anderer Ort:
  `--out <ordner>` oder `G1_EVAL_OUT`). Der Ordner ist in Git ignoriert.
- Rohdaten als CSV, Zusammenfassung als `summary.json`. Jede CSV beginnt mit
  Kopfzeilen `# schluessel: wert`: Werkzeug, Versuch, Zeitpunkt, Git-Commit
  (mit Kennzeichen „geändert“, wenn die Arbeitskopie nicht sauber ist),
  Betriebsart (Simulation / Realbetrieb / Attrappe / offline), Rechner (CPU,
  RAM, GPU, OS, Container ja/nein), Parameter des Laufs, Aufruf. Dieselben
  Angaben stehen in `summary.json` unter `meta`. Ohne `.git` im Container
  liest `eval_common.py` den Commit aus `G1_EVAL_GIT_COMMIT`.
- Werte werden nicht geglättet und keine Ausreißer entfernt; Statistik:
  n, Mittel, Standardabweichung, Min, P5, Median, P95, P99, Max.
- Auswertung einer CSV in eigenen Skripten: `eval_common.read_csv(pfad)`
  liefert Kopf, Spalten und Zeilen.
- Im Container starten: `docker exec -it g1pilot-sim bash`, dann
  `cd /ros2_ws/src/g1pilot && python3 tools/eval/<werkzeug>.py --help`.

## Änderungen am bestehenden Code

Nur ein Messpunkt war unvermeidbar (V4, simulierte Zeit), standardmäßig aus:

- `unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py`: mit
  `G1_EVAL_SIM_TIME=1` schreibt die Bridge die Sim-Zeit in Millisekunden in
  `rt/lowstate.tick`. Ohne die Variable bleibt `tick` wie bisher.
- `g1pilot/docker-compose.yml`: reicht `G1_EVAL_SIM_TIME` (Standard `0`) an
  den Container `mujoco-sim` durch.

Tests der Werkzeuge: `test/test_eval_tools.py`.

Alle Gruppen: [README.md](README.md).

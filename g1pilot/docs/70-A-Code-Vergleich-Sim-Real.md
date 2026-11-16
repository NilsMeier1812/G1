# V1 Code-Vergleich Simulation/Realbetrieb — Anleitung

## Zweck

Findet alle Stellen, an denen der Code die Betriebsart abfragt (Schalter
`G1_SIM_MODE`, `is_sim_mode()` in `g1pilot/utils/common.py` und alle
Nutzer, auch in `launch/`), mit Datei, Zeile und Art des Unterschieds. Dazu
die Unterschiede zwischen den Container-Images (`docker/Dockerfile.sim`,
`Dockerfile.real`, `Dockerfile.mujoco`), Compose-Diensten (`g1pilot-sim`,
`g1pilot-real`, `mujoco-sim`), `start.sh` und den Launch-Dateien
`bringup_sim` / `bringup_real`.

## Aufruf

```bash
python3 tools/eval/v1_code_vergleich.py            # Quellbaum ohne Tests
python3 tools/eval/v1_code_vergleich.py --include-tests
```

Läuft offline auf dem Quelltext (Host oder Container), braucht nur Python 3
und `pyyaml`.

## Wie gesucht wird

- **Direkte Abfragen** (Python-AST): Aufrufe `is_sim_mode()`,
  `resolve_dds()`, `init_dds()`, die Konstanten `SIM_DDS_*` und das Literal
  `"G1_SIM_MODE"`.
- **Abfrage-Funktionen**: Funktionen, deren Rückgabe die Betriebsart ist
  (z. B. ein Helfer, der `is_sim_mode()` zurückgibt), werden als Abfrage
  mitgezählt.
- **Abgeleitete Namen**: ein Attribut wie `self.sim_mode` zählt, wenn
  *jede* Zuweisung daran die Betriebsart ist; jede Verwendung ist dann eine
  Fundstelle.
- **Indirekt** (eine Aufrufebene): Aufrufe solcher Abfrage-Funktionen.
- **Text** (Shell, YAML, Makefile, Dockerfiles): Vorkommen von
  `G1_SIM_MODE` und verwandten Variablen.
- Für jede Python-Fundstelle bestimmt das Werkzeug die **Wirkung** aus dem
  Kontext (Zweig Sim/Real, Wertauswahl, Parameter-Standard,
  GUI-Sichtbarkeit …) und ergänzt aus einem Katalog eine Beschreibung des
  **Unterschieds**. Neue, noch nicht beschriebene Stellen erscheinen in
  `summary.json` unter `nicht_katalogisiert` (Katalog `CATALOG` im Skript
  ergänzen).

## Ausgabe

| Datei | Inhalt |
|---|---|
| `abfragen.csv` | je Abfrage: Datei, Zeile, Bereich, Funktion, Art der Abfrage, Wirkung, Unterschied, Code |
| `indirekt.csv` | Aufrufe von Abfrage-Funktionen |
| `text_fundstellen.csv` | Fundstellen in Shell/YAML/Makefile/Dockerfiles |
| `launch.csv` | Nodes und Parameter bringup_sim gegen bringup_real |
| `images.csv` | Pakete/Basis/ENV je Dockerfile im Vergleich |
| `konfiguration.csv` | Compose-Dienste und `start.sh`-Exporte Sim gegen Real |
| `tabelle.md` | alles als Markdown-Tabellen (für die Arbeit) |
| `summary.json` | Zählungen je Datei/Art, nicht katalogisierte Stellen |

## Grenzen

- Statische Analyse: Abfragen über `getattr`, `eval` oder dynamisch
  zusammengesetzte Namen findet sie nicht. Indirekte Aufrufe nur eine Ebene tief.
- Die Beschreibung des Unterschieds stammt aus dem Katalog im Skript; ändert
  sich der Code, kann sie veralten (Prüfung: `nicht_katalogisiert` leer und
  Zeilennummern stimmen).
- Image-Vergleich liest die Dockerfiles, nicht gebaute Images (keine
  Versionen aus `pip install` ohne Pin).

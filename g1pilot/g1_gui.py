#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
#  g1_gui.py — Grafisches Start-Menue fuer den G1-Stack (Sim / Real / Umgebungen).
#
#  Ziel: Im normalen Betrieb muss man KEIN cmd-Fenster anfassen. Das Fenster
#  ersetzt die Text-Menues aus g1pilot/start.sh und scene_editor/launch.sh,
#  implementiert die Orchestrierung aber NICHT neu, sondern ruft die bewaehrten
#  Skripte env-getrieben auf:
#     Sim / Real  ->  start.sh --yes           (alle Optionen als Env-Vars)
#     Umgebungen  ->  scene_editor/launch.sh <cmd> [datei]
#  Die Ausgabe landet live in einer Log-Ansicht mit Stop-Knopf.
#
#  Oberflaeche: PyQt6 (wie die Demo-GUI), Code in start_gui/:
#     core.py         Logik ohne GUI (Pfade, Env-Variablen, Doku-Verzeichnis)
#     views.py        Menue, Simulation (Umgebungs-Kacheln), Echter Roboter, Umgebungen
#     console.py      Prozess-Log mit Stoppen
#     docs_viewer.py  eingebauter Doku-Viewer (Markdown gerendert)
#     widgets.py, theme.py   Schalter, Karten, Farben
#  Vorschaubilder der Umgebungen: unitree_mujoco/scene_editor/render_thumbs.py
#
#  Start:  python3 g1_gui.py        (start.sh startet das automatisch)
#  Host-Paket: sudo apt install python3-pyqt6 libqt6svg6
#  Fehlt PyQt6 oder ein Display, nimmt start.sh das Text-Menue.
# ════════════════════════════════════════════════════════════════════════
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    from PyQt6 import QtWidgets  # noqa: F401
except Exception as exc:  # pragma: no cover - nur wenn PyQt6 fehlt
    sys.stderr.write(
        "[g1_gui] PyQt6 ist nicht verfuegbar (%s).\n"
        "         Installieren: sudo apt install python3-pyqt6 libqt6svg6\n"
        "         Oder das Text-Menue nutzen: ./start.sh --menu\n" % exc)
    sys.exit(2)

from start_gui.app import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())

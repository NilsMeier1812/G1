"""Gemeinsame Qt-Basis der Hand-Werkzeuge (Neue Hand anlegen, Hand einpassen).

Gleiches Aussehen wie das Startmenue: Farben/Stylesheet aus g1pilot/start_gui/
theme.py, Bausteine aus start_gui/widgets.py, Icons aus g1pilot/utils/ui_icons.py
(Lucide-SVG, keine Emojis). Die Werkzeuge liegen im selben Repo, darum wird
g1pilot/ einfach auf sys.path gelegt.

MuJoCo-Fenster (GLFW) und Qt laufen weiterhin in GETRENNTEN Prozessen: beide
greifen sonst aus zwei Threads auf X11 zu (eingefrorenes Bild, "X Error").
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]                       # .../G1
G1PILOT_DIR = REPO / "g1pilot"
APP_ICON = G1PILOT_DIR / "resource" / "icons" / "g1_start.png"

if str(G1PILOT_DIR) not in sys.path:
    sys.path.insert(0, str(G1PILOT_DIR))

QT_MISSING = ("PyQt6 fehlt fuer {py}. Einmalig: {py} -m pip install 'PyQt6>=6.4' "
              "(hand_tool.sh macht das im Szenen-Editor-venv selbst).")


def have_qt() -> bool:
    try:
        import PyQt6.QtWidgets  # noqa: F401
        return True
    except Exception:
        return False


def make_app(app_id: str = "g1-hand-tool"):
    """QApplication mit Startmenue-Theme und eigener WM_CLASS (Dock-Icon)."""
    os.environ.setdefault("RESOURCE_NAME", app_id)
    from PyQt6.QtGui import QIcon
    from PyQt6.QtWidgets import QApplication

    from start_gui import theme

    app = QApplication.instance() or QApplication(sys.argv[:1])
    app.setApplicationName(app_id)
    app.setDesktopFileName(app_id)
    if APP_ICON.is_file():
        app.setWindowIcon(QIcon(str(APP_ICON)))
    theme.apply(app)
    return app

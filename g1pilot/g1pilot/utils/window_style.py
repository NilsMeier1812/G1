import os
from pathlib import Path

from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QColor, QIcon, QPalette
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QStyleFactory

def DarkStyle(app: QApplication):
    """Gemeinsames dunkles Theme (g1pilot/utils/ui_theme.py, wie das Startmenue).
    Schriftgroessen bleiben bei den Widgets der Demo-GUI."""
    from g1pilot.utils import ui_theme
    ui_theme.apply(app, fonts=False)
    app.setStyleSheet(app.styleSheet() + (
        f"QPushButton:checked {{ background-color: {ui_theme.ACCENT}; color: white; }}"))

def LightStyle(app: QApplication):
    app.setStyle(QStyleFactory.create('Fusion'))
    light_palette = QPalette()

    light_palette.setColor(QPalette.ColorRole.Window, QColor(255, 255, 255))
    light_palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.black)
    light_palette.setColor(QPalette.ColorRole.Base, QColor(240, 240, 240))
    light_palette.setColor(QPalette.ColorRole.AlternateBase, QColor(220, 220, 220))
    light_palette.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.black)
    light_palette.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.black)
    light_palette.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.black)
    light_palette.setColor(QPalette.ColorRole.Button, QColor(240, 240, 240))
    light_palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, QColor(200, 200, 200))
    light_palette.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.black)
    light_palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, QColor(150, 150, 150))
    light_palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    light_palette.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
    light_palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
    light_palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Highlight, QColor(200, 200, 200))
    light_palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.white)

    app.setPalette(light_palette)
    app.setStyleSheet(
        "QToolTip { color: #000000; background-color: #ffffff; border: 1px solid #000000; }"
    )   

# ── Fenster-Icons ─────────────────────────────────────────────────────────
# Ohne eigenes Icon zeigt das Dock/die Taskleiste fuer jedes G1-Fenster nur ein
# Zahnrad. Die Icons liegen in g1pilot/resource/icons (Quelle: SVG, PNG daraus
# erzeugt) und werden zusaetzlich nach share/g1pilot/resource/icons installiert.
ICON_SUBDIR = Path("resource") / "icons"


def find_icon(name: str):
    """Pfad zu resource/icons/<name>.png oder None (dann bleibt das Standard-Icon)."""
    candidates = []
    env_dir = os.environ.get("G1_ICON_DIR", "").strip()
    if env_dir:
        candidates.append(Path(env_dir))
    # resolve(): bei colcon --symlink-install zeigt __file__ in build/, der
    # Symlink fuehrt zurueck in den Quellbaum (.../g1pilot/g1pilot/utils/).
    candidates.append(Path(__file__).resolve().parents[2] / ICON_SUBDIR)
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory("g1pilot")) / ICON_SUBDIR)
    except Exception:
        pass
    for d in candidates:
        p = d / f"{name}.png"
        if p.is_file():
            return p
    return None


def set_app_identity(app: QApplication, app_id: str, icon_name: str):
    """Eigenes Icon + WM_CLASS fuer alle Fenster dieser Qt-Anwendung.

    Muss VOR dem ersten show() laufen: Qt (xcb) liest WM_CLASS beim Anlegen des
    ersten Fensters. Instanz = RESOURCE_NAME, Klasse = applicationName -> beide
    app_id, damit Dock/.desktop-Eintraege (StartupWMClass) das Fenster zuordnen.
    """
    os.environ.setdefault("RESOURCE_NAME", app_id)
    app.setApplicationName(app_id)
    app.setDesktopFileName(app_id)
    path = find_icon(icon_name)
    if path is not None:
        app.setWindowIcon(QIcon(str(path)))
    return path

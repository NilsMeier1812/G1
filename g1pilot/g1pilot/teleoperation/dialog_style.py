"""Gemeinsamer Look fuer die Neben-Fenster der Demo-GUI (Handy-Zugang,
Controller-Belegung): dunkle Karten mit Icon-Kopf, Tasten-Chips, Status-Punkt.
Farben passend zur Demo-GUI, Icons aus g1pilot/utils/ui_icons.py (keine Emojis).
"""
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout

from g1pilot.utils import ui_icons

BG = "#141414"
CARD = "#1d1f23"
LINE = "#2c3038"
FG = "#e8eaed"
MUTED = "#9aa0a6"
ACCENT = "#4fc3f7"
OK = "#66bb6a"
WARN = "#ffb300"
BAD = "#ef5350"



def _icon_file(name, color):
    """Lucide-Icon in Farbe als Datei (Qt-Stylesheets brauchen einen Pfad,
    z. B. fuer die Pfeile von Auswahlliste/Zahlenfeld). Leer, wenn es fehlt."""
    import tempfile
    from pathlib import Path
    data = ui_icons.svg_bytes(name, color)
    if not data:
        return ""
    d = Path(tempfile.gettempdir()) / "g1pilot_ui_icons"
    try:
        d.mkdir(exist_ok=True)
        f = d / f"{name}_{color.lstrip('#')}.svg"
        if not f.is_file() or f.read_bytes() != data:
            f.write_bytes(data)
    except OSError:
        return ""
    return f.as_posix()


ARROW_DOWN = _icon_file("chevron-down", MUTED)
ARROW_UP = _icon_file("chevron-up", MUTED)

DIALOG_CSS = f"""
QDialog {{ background:{BG}; }}
QLabel {{ color:{FG}; background:transparent; }}
QFrame#dcard {{ background:{CARD}; border:1px solid {LINE}; border-radius:14px; }}
QPushButton {{ background:#2a2e35; color:{FG}; border:1px solid #3a3f48; border-radius:10px;
               padding:9px 16px; font-size:14px; font-weight:600; }}
QPushButton:hover {{ border-color:#6b7280; }}
QPushButton:pressed {{ background:#3a3f48; }}
QPushButton:disabled {{ color:#5d6470; background:#1a1c20; border-color:#24272d; }}
QPushButton[kind="primary"] {{ background:#1565c0; border:none; color:white; }}
QPushButton[kind="primary"]:hover {{ background:#1e78d8; }}
QPushButton[kind="warn"] {{ background:#5d4037; border:none; }}
QPushButton[kind="warn"]:disabled, QPushButton[kind="primary"]:disabled {{
    background:#1a1c20; color:#5d6470; border:1px solid #24272d; }}
QPushButton[kind="ghost"] {{ background:transparent; }}
QListWidget {{ background:#16181b; border:1px solid {LINE}; border-radius:10px; color:{FG};
               font-size:14px; padding:4px; outline:none; }}
QListWidget::item {{ padding:8px 6px; border-radius:6px; }}
QListWidget::item:selected {{ background:#263445; color:white; }}
QScrollArea {{ border:none; background:{BG}; }}
QWidget#dbody {{ background:{BG}; }}
QLineEdit, QComboBox, QDoubleSpinBox {{ background:#16181b; color:{FG}; border:1px solid #3a3f48;
    border-radius:8px; padding:6px 10px; font-size:14px; min-height:22px; }}
QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus {{ border-color:{ACCENT}; }}
QComboBox QAbstractItemView {{ background:#16181b; color:{FG}; border:1px solid #3a3f48;
    selection-background-color:#263445; }}
QCheckBox {{ color:{FG}; font-size:14px; spacing:8px; padding:4px 2px; }}
QTreeWidget {{ background:#16181b; border:1px solid {LINE}; border-radius:10px; color:{FG};
    font-size:14px; padding:4px; outline:none; }}
QTreeWidget::item {{ padding:6px 4px; border-radius:6px; }}
QTreeWidget::item:selected {{ background:#263445; color:white; }}
QHeaderView::section {{ background:{CARD}; color:{MUTED}; border:none;
    border-bottom:1px solid {LINE}; padding:6px; font-weight:700; }}
QComboBox::drop-down {{ border:none; width:26px; }}
QComboBox::down-arrow {{ image:url({ARROW_DOWN}); width:14px; height:14px; }}
QDoubleSpinBox {{ padding-right:30px; }}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{ subcontrol-origin:border; width:26px;
    background:#262a31; border:none; border-left:1px solid #3a3f48; }}
QDoubleSpinBox::up-button {{ subcontrol-position:top right; border-top-right-radius:8px; }}
QDoubleSpinBox::down-button {{ subcontrol-position:bottom right; border-bottom-right-radius:8px; }}
QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {{ background:#3a3f48; }}
QDoubleSpinBox::up-arrow {{ image:url({ARROW_UP}); width:12px; height:12px; }}
QDoubleSpinBox::down-arrow {{ image:url({ARROW_DOWN}); width:12px; height:12px; }}
QTabWidget::pane {{ border:none; top:-1px; }}
QTabBar::tab {{ background:transparent; color:{MUTED}; padding:9px 18px; font-size:14px;
    font-weight:700; border:none; border-bottom:2px solid transparent; }}
QTabBar::tab:selected {{ color:{FG}; border-bottom:2px solid {ACCENT}; }}
QTabBar::tab:hover {{ color:{FG}; }}
"""


def icon_box(name, color=FG, size=20, box=36, bg="#2a2e35"):
    lbl = QLabel()
    lbl.setPixmap(ui_icons.pixmap(name, color, size))
    lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
    lbl.setFixedSize(box, box)
    lbl.setStyleSheet(f"background:{bg}; border-radius:{box // 4}px;")
    return lbl


def card(title, icon="", accent=FG, sub=""):
    """Karte mit Kopf (Icon + Titel [+ Untertitel]) -> (frame, body-Layout)."""
    f = QFrame()
    f.setObjectName("dcard")
    lay = QVBoxLayout(f)
    lay.setContentsMargins(16, 14, 16, 14)
    lay.setSpacing(10)
    head = QHBoxLayout()
    head.setSpacing(12)
    if icon:
        head.addWidget(icon_box(icon, accent))
    col = QVBoxLayout()
    col.setSpacing(1)
    t = QLabel(title)
    t.setStyleSheet(f"font-size:16px; font-weight:800; color:{FG};")
    col.addWidget(t)
    if sub:
        s = QLabel(sub)
        s.setWordWrap(True)
        s.setStyleSheet(f"color:{MUTED}; font-size:13px;")
        col.addWidget(s)
    head.addLayout(col, 1)
    lay.addLayout(head)
    return f, lay


def key_chip(text, color=ACCENT):
    """Taste als abgerundeter Chip (z. B. »L2 halten«)."""
    lbl = QLabel(text)
    lbl.setStyleSheet(f"background:#262a31; color:{color}; border:1px solid #3a3f48;"
                      "border-radius:8px; padding:4px 10px; font-size:13px; font-weight:700;")
    lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
    return lbl


def dot(color, size=10):
    d = QLabel()
    d.setFixedSize(size, size)
    d.setStyleSheet(f"background:{color}; border-radius:{size // 2}px;")
    return d


def set_dot(d, color):
    d.setStyleSheet(f"background:{color}; border-radius:{d.width() // 2}px;")


def kind(btn, k):
    btn.setProperty("kind", k)
    btn.style().unpolish(btn)
    btn.style().polish(btn)
    return btn


def header(title, icon="", accent=ACCENT, sub=""):
    """Kopfzeile eines Dialogs wie »Controller-Belegung«: grosses Icon,
    Titel, grauer Untertitel. -> QHBoxLayout."""
    head = QHBoxLayout()
    head.setSpacing(14)
    if icon:
        head.addWidget(icon_box(icon, accent, 24, 46), 0, Qt.AlignmentFlag.AlignTop)
    col = QVBoxLayout()
    col.setSpacing(2)
    t = QLabel(title)
    t.setStyleSheet("font-size:20px; font-weight:800;")
    col.addWidget(t)
    if sub:
        s = QLabel(sub)
        s.setWordWrap(True)
        s.setStyleSheet(f"color:{MUTED}; font-size:13px;")
        col.addWidget(s)
    head.addLayout(col, 1)
    return head


def button(text, icon="", k=None, color=FG, size=16):
    """QPushButton im Dialog-Look, optional mit Lucide-Icon und Art
    (primary/warn/ghost)."""
    from PyQt6.QtWidgets import QPushButton
    b = QPushButton(text)
    if icon:
        ui_icons.set_icon(b, icon, color, size)
    if k:
        kind(b, k)
    return b


def hint(text):
    """Grauer Erklaertext (umbrechend)."""
    lbl = QLabel(text)
    lbl.setWordWrap(True)
    lbl.setStyleSheet(f"color:{MUTED}; font-size:13px;")
    return lbl

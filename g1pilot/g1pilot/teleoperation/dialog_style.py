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

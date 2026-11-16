"""Farben + Qt-Stylesheet des Startmenues (dunkel, ruhig, ein Akzent).

Eine Quelle fuer alle Farben, damit Startmenue, Doku-Viewer und spaeter die
Demo-GUI gleich aussehen. Knoepfe/Karten waehlen ihre Variante ueber
objectName bzw. die Property "kind" (primary | success | danger | ghost).
"""
from __future__ import annotations

BG = "#14171d"          # Fensterhintergrund
SURFACE = "#1b1f27"     # Kopf/Fuss-Leisten
CARD = "#1f242d"        # Karten
CARD_HI = "#272d38"     # Karte unter der Maus / Eingabefelder
LINE = "#2e3541"        # Trennlinien, Rahmen
FG = "#e8eaed"
MUTED = "#8f97a3"
ACCENT = "#4c8bf5"
ACCENT_SOFT = "#4c8bf533"
GREEN = "#3ecf7a"
RED = "#e5484d"
AMBER = "#e0a13b"
DANGER_BG = "#2d1a1d"
CONSOLE_BG = "#0f1216"

FONT_FAMILY = '"Inter", "Ubuntu", "Noto Sans", "DejaVu Sans", sans-serif'
MONO_FAMILY = '"JetBrains Mono", "Ubuntu Mono", "DejaVu Sans Mono", monospace'


def stylesheet() -> str:
    return f"""
* {{ font-family: {FONT_FAMILY}; font-size: 10.5pt; }}
QWidget {{ color: {FG}; }}
QMainWindow, QDialog, QMessageBox, QInputDialog {{ background: {BG}; }}
QLabel {{ background: transparent; }}
QLabel[role="h1"] {{ font-size: 18pt; font-weight: 650; }}
QLabel[role="sub"] {{ color: {MUTED}; }}
QLabel[role="section"] {{ color: {MUTED}; font-size: 9pt; font-weight: 600;
                          letter-spacing: 1px; padding-top: 14px; }}
QLabel[role="hint"] {{ color: {MUTED}; font-size: 9.5pt; }}
QLabel[role="muted"] {{ color: {MUTED}; }}
QLabel a {{ color: {ACCENT}; }}

QFrame#bar {{ background: {SURFACE}; border: none; }}
QFrame#barline {{ background: {LINE}; max-height: 1px; min-height: 1px; border: none; }}
QFrame#card {{ background: {CARD}; border-radius: 12px; }}
QFrame#danger {{ background: {DANGER_BG}; border: 1px solid #5a2a30; border-radius: 12px; }}
QFrame#rowline {{ background: {LINE}; max-height: 1px; min-height: 1px; border: none; }}

QPushButton {{ background: {CARD_HI}; color: {FG}; border: 1px solid {LINE};
               border-radius: 8px; padding: 7px 14px; }}
QPushButton:hover {{ background: #303746; }}
QPushButton:pressed {{ background: #262c37; }}
QPushButton:disabled {{ color: #5d6470; background: {CARD}; border-color: {CARD}; }}
QPushButton[kind="primary"] {{ background: {ACCENT}; color: white; border: none;
                               font-weight: 600; padding: 10px 20px; }}
QPushButton[kind="primary"]:hover {{ background: #5d97f7; }}
QPushButton[kind="success"] {{ background: {GREEN}; color: #08240f; border: none;
                               font-weight: 700; padding: 11px 24px; font-size: 11.5pt; }}
QPushButton[kind="success"]:hover {{ background: #55d98b; }}
QPushButton[kind="danger"] {{ background: {RED}; color: white; border: none;
                              font-weight: 700; padding: 11px 24px; font-size: 11.5pt; }}
QPushButton[kind="danger"]:hover {{ background: #ec6064; }}
QPushButton[kind="danger"]:disabled, QPushButton[kind="success"]:disabled,
QPushButton[kind="primary"]:disabled {{ background: #3a3f49; color: #7b818b; }}
QPushButton[kind="ghost"] {{ background: transparent; border: 1px solid {LINE}; }}
QPushButton[kind="ghost"]:hover {{ background: {CARD}; }}
QPushButton[kind="small-danger"] {{ background: transparent; color: {RED};
                                    border: 1px solid #5a2a30; padding: 5px 12px; }}
QPushButton[kind="small-danger"]:hover {{ background: {DANGER_BG}; }}
QPushButton[kind="link"] {{ background: transparent; border: none; color: {ACCENT};
                            padding: 4px 2px; text-align: left; }}
QPushButton[kind="link"]:hover {{ text-decoration: underline; }}

QPushButton#choice {{ background: {CARD}; border: 2px solid transparent;
                      border-radius: 12px; padding: 0; text-align: left; }}
QPushButton#choice:hover {{ background: {CARD_HI}; }}
QPushButton#choice:checked {{ border: 2px solid {ACCENT}; background: {CARD_HI}; }}
QPushButton#choice:disabled {{ background: {CARD}; }}
QPushButton#tile {{ background: {CARD}; border: 1px solid transparent; border-radius: 14px;
                    padding: 0; text-align: left; }}
QPushButton#tile:hover {{ background: {CARD_HI}; border-color: {LINE}; }}

QLineEdit, QComboBox {{ background: {CARD_HI}; border: 1px solid {LINE}; border-radius: 8px;
                        padding: 6px 10px; selection-background-color: {ACCENT}; }}
QLineEdit:focus, QComboBox:focus {{ border-color: {ACCENT}; }}
QLineEdit:disabled {{ color: #5d6470; background: {CARD}; }}
QComboBox::drop-down {{ border: none; width: 26px; }}
QComboBox QAbstractItemView {{ background: {CARD_HI}; border: 1px solid {LINE};
                               selection-background-color: {ACCENT}; outline: none; }}

QCheckBox {{ spacing: 10px; background: transparent; }}
QCheckBox::indicator {{ width: 20px; height: 20px; border-radius: 6px;
                        border: 2px solid #6b7380; background: {BG}; }}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}

QScrollArea {{ border: none; background: {BG}; }}
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: #394150; border-radius: 4px; min-height: 30px; }}
QScrollBar::handle:vertical:hover {{ background: #4a5364; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: #394150; border-radius: 4px; min-width: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: none; }}

QPlainTextEdit#console {{ background: {CONSOLE_BG}; color: #d7dbe0; border: 1px solid {LINE};
                          border-radius: 10px; padding: 6px;
                          font-family: {MONO_FAMILY}; font-size: 9.5pt; }}

QTreeWidget {{ background: {SURFACE}; border: none; outline: none; padding: 4px; }}
QTreeWidget::item {{ padding: 5px 4px; border-radius: 6px; }}
QTreeWidget::item:hover {{ background: {CARD}; }}
QTreeWidget::item:selected {{ background: {CARD_HI}; color: white; }}
QTextBrowser {{ background: {BG}; border: none; }}

QMenu {{ background: {CARD_HI}; border: 1px solid {LINE}; padding: 4px; }}
QMenu::item {{ padding: 6px 18px; border-radius: 6px; }}
QMenu::item:selected {{ background: {ACCENT}; color: white; }}
QToolTip {{ background: {CARD_HI}; color: {FG}; border: 1px solid {LINE}; padding: 4px; }}
QStatusBar {{ background: {SURFACE}; color: {MUTED}; border-top: 1px solid {LINE}; }}
QStatusBar QLabel {{ color: {MUTED}; padding: 0 8px; }}
"""

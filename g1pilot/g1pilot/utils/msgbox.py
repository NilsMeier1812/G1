"""Rueckfrage-/Hinweis-/Eingabe-Dialoge im Karten-Look (ersetzt QMessageBox/QInputDialog).

Gleicher Look in Startmenue, Demo-GUI und Hand-Werkzeugen: dunkle Karte, Icon-Kopf
(Lucide, keine Emojis), Farben aus g1pilot/utils/ui_theme.py. Aufrufe wie bei Qt:

    msgbox.info(parent, "Titel", "Text")
    msgbox.warn(parent, ...)  /  msgbox.error(parent, ...)
    if msgbox.confirm(parent, "Stoppen", "Wirklich?", yes="Stoppen", danger=True): ...
    text, ok = msgbox.ask_text(parent, "Name", "Name der Kategorie:", text="Demo")

Tests koennen `msgbox.run_dialog` ersetzen (dann oeffnet sich kein Fenster).
"""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QDialog, QFrame, QHBoxLayout, QLabel, QLineEdit,
                             QPushButton, QVBoxLayout)

from g1pilot.utils import ui_icons, ui_theme as T

# Art -> (Icon, Farbe)
KINDS = {
    "info": ("circle-help", T.ACCENT),
    "question": ("circle-help", T.ACCENT),
    "warn": ("triangle-alert", T.AMBER),
    "error": ("octagon-alert", T.RED),
}


def run_dialog(dlg: QDialog) -> bool:
    """Dialog modal ausfuehren (Testpunkt)."""
    return dlg.exec() == QDialog.DialogCode.Accepted


def _button(text: str, kind: str | None, default: bool = False) -> QPushButton:
    b = QPushButton(text)
    if kind:
        b.setProperty("kind", kind)
    b.setDefault(default)
    b.setAutoDefault(default)
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    return b


def build(parent, title: str, text: str, kind: str = "info", yes: str = "OK",
          no: str | None = None, danger: bool = False, field: str | None = None,
          value: str = "") -> QDialog:
    """Dialog bauen (ohne ihn zu zeigen). `no=None` -> nur ein OK-Knopf.
    `field` (Beschriftung) -> mit Eingabefeld, Ergebnis in `dlg.line`."""
    dlg = QDialog(parent)
    dlg.setWindowTitle(title)
    dlg.setModal(True)
    dlg.setMinimumWidth(420)
    dlg.setStyleSheet(T.stylesheet())
    icon, color = KINDS.get(kind, KINDS["info"])
    root = QVBoxLayout(dlg)
    root.setContentsMargins(18, 18, 18, 16)
    root.setSpacing(14)

    card = QFrame()
    card.setObjectName("card")
    row = QHBoxLayout(card)
    row.setContentsMargins(16, 16, 16, 16)
    row.setSpacing(14)
    badge = QLabel()
    badge.setPixmap(ui_icons.pixmap(icon, color, 22))
    badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
    badge.setFixedSize(40, 40)
    badge.setStyleSheet(f"background:{T.CARD_HI}; border-radius:10px;")
    row.addWidget(badge, 0, Qt.AlignmentFlag.AlignTop)
    col = QVBoxLayout()
    col.setSpacing(6)
    head = QLabel(title)
    head.setStyleSheet("font-size:13pt; font-weight:700;")
    col.addWidget(head)
    if text:
        body = QLabel(text)
        body.setWordWrap(True)
        body.setTextFormat(Qt.TextFormat.PlainText)
        body.setStyleSheet(f"color:{T.MUTED};")
        col.addWidget(body)
    dlg.line = None
    if field is not None:
        lab = QLabel(field)
        lab.setWordWrap(True)
        col.addWidget(lab)
        dlg.line = QLineEdit(value)
        col.addWidget(dlg.line)
    row.addLayout(col, 1)
    root.addWidget(card)

    btns = QHBoxLayout()
    btns.addStretch(1)
    if no is not None:
        b_no = _button(no, "ghost")
        b_no.clicked.connect(dlg.reject)
        btns.addWidget(b_no)
    b_yes = _button(yes, "danger" if danger else "primary", default=True)
    b_yes.clicked.connect(dlg.accept)
    btns.addWidget(b_yes)
    dlg.yes_button = b_yes
    root.addLayout(btns)
    if dlg.line is not None:
        dlg.line.returnPressed.connect(dlg.accept)
        dlg.line.setFocus()
        dlg.line.selectAll()
    return dlg


def info(parent, title: str, text: str) -> None:
    run_dialog(build(parent, title, text, "info"))


def warn(parent, title: str, text: str) -> None:
    run_dialog(build(parent, title, text, "warn"))


def error(parent, title: str, text: str) -> None:
    run_dialog(build(parent, title, text, "error"))


def confirm(parent, title: str, text: str, yes: str = "Ja", no: str = "Abbrechen",
            danger: bool = False) -> bool:
    kind = "warn" if danger else "question"
    return bool(run_dialog(build(parent, title, text, kind, yes=yes, no=no, danger=danger)))


def ask_text(parent, title: str, label: str, text: str = "", ok: str = "OK",
             cancel: str = "Abbrechen", info_text: str = "") -> tuple[str, bool]:
    """Wie QInputDialog.getText -> (Text, bestaetigt)."""
    dlg = build(parent, title, info_text, "question", yes=ok, no=cancel, field=label, value=text)
    accepted = bool(run_dialog(dlg))
    return dlg.line.text(), accepted

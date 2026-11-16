#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
quickslot_panel.py — Reiter »Controller-Schnellwahl« im Fenster »Posen &
Abläufe« der Demo-GUI.

Am Controller: R2 halten, Steuerkreuz ZWEIMAL druecken (z. B. runter, hoch),
R2 loslassen -> die zugeordnete Pose faehrt an (geplant, ps4_arm_teleop).
Das ergibt 16 Plaetze. Hier sieht man sie als 4 x 4-Raster: jede ZEILE ist
die erste Richtung, jede SPALTE die zweite; jedes Feld zeigt seine beiden
Pfeile und eine Auswahlliste der gespeicherten Posen. Aenderungen werden
sofort in pose_store ("quickslots") gespeichert.

Waehlt jemand am Controller gerade einen Platz, leuchtet das Feld (bzw. die
Zeile nach dem ersten Druck) auf -- highlight(), von der Demo-GUI aus dem
Controller-Status (/g1pilot/ps4/status "quick") gefuettert.
"""
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget,
)

from g1pilot.manipulation.pose_store import QUICK_DIRS, quick_combo
from g1pilot.teleoperation import dialog_style as ds
from g1pilot.utils import ui_icons

DIR_ICON = {"up": "arrow-up", "down": "arrow-down", "left": "arrow-left", "right": "arrow-right"}
DIR_TEXT = {"up": "hoch", "down": "runter", "left": "links", "right": "rechts"}
EMPTY_TEXT = "– leer –"
ROLE = Qt.ItemDataRole.UserRole

CELL_CSS = f"""
QFrame#qcell {{ background:{ds.CARD}; border:1px solid {ds.LINE}; border-radius:10px; }}
QFrame#qcell[state="set"] {{ border:1px solid #3d5a80; }}
QFrame#qcell[state="missing"] {{ border:1px solid {ds.BAD}; }}
QFrame#qcell[state="live"] {{ border:2px solid {ds.OK}; background:#1f2a22; }}
QFrame#qcell[state="row"] {{ border:2px solid #ffffff; }}
QLabel#qhead {{ color:{ds.MUTED}; font-size:13px; font-weight:700; }}
"""


def arrow_label(direction, color=ds.FG, size=20):
    lbl = QLabel()
    lbl.setPixmap(ui_icons.pixmap(DIR_ICON[direction], color, size))
    lbl.setToolTip(DIR_TEXT[direction])
    lbl.setFixedSize(size + 4, size + 4)
    lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
    return lbl


def _repolish(w, state):
    if w.property("state") == state:
        return
    w.setProperty("state", state)
    w.style().unpolish(w)
    w.style().polish(w)


class QuickSlotCell(QFrame):
    """Ein Platz: zwei Pfeile (erste, zweite Richtung) + Pose-Auswahl."""

    def __init__(self, first, second, on_change, parent=None):
        super().__init__(parent)
        self.setObjectName("qcell")
        self.combo_key = quick_combo(first, second)
        self._on_change = on_change
        self._loading = False
        self._base = "empty"
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 8)
        lay.setSpacing(4)
        head = QHBoxLayout()
        head.setSpacing(2)
        head.addWidget(arrow_label(first))
        head.addWidget(arrow_label(second))
        head.addStretch(1)
        lay.addLayout(head)
        self.box = QComboBox()
        self.box.setMinimumWidth(150)
        self.box.setToolTip(f"R2 halten, Steuerkreuz {DIR_TEXT[first]} und dann "
                            f"{DIR_TEXT[second]}, R2 loslassen")
        lay.addWidget(self.box)
        self.box.currentIndexChanged.connect(self._changed)

    def fill(self, grouped, current):
        """grouped = {kategorie: [posen]}, current = zugeordnete Pose (oder None)."""
        self._loading = True
        self.box.clear()
        self.box.addItem(EMPTY_TEXT, None)
        known = set()
        for cat, names in grouped.items():
            if not names:
                continue
            if len(grouped) > 1:
                self.box.addItem(f"{cat}", "__cat__")
                idx = self.box.count() - 1
                item = self.box.model().item(idx)
                item.setEnabled(False)
            for n in names:
                self.box.addItem(f"  {n}" if len(grouped) > 1 else n, n)
                known.add(n)
        if current and current not in known:
            # Verweis auf geloeschte Pose: sichtbar lassen, rot markieren.
            self.box.addItem(f"{current} (fehlt)", current)
        idx = self.box.findData(current) if current else 0
        self.box.setCurrentIndex(max(0, idx))
        self._base = ("missing" if current and current not in known
                      else "set" if current else "empty")
        _repolish(self, self._base)
        self._loading = False

    def current(self):
        data = self.box.currentData()
        return data if isinstance(data, str) and data != "__cat__" else None

    def set_live(self, state):
        """None = normal, "live" = gewaehlt, "row" = erste Richtung gedrueckt."""
        _repolish(self, state or self._base)

    def _changed(self, _idx):
        if self._loading:
            return
        self._on_change(self.combo_key, self.current())


class QuickSlotPanel(QWidget):
    """gui: Objekt mit _pose_store() (Demo-GUI oder Ersatz in Tests)."""

    def __init__(self, gui, parent=None):
        super().__init__(parent)
        self.gui = gui
        self.setStyleSheet(CELL_CSS)
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(10)

        # Erklaerung: so wird gewaehlt.
        howto = QHBoxLayout()
        howto.setSpacing(8)
        howto.addWidget(ds.icon_box("gamepad-2", ds.ACCENT))
        for i, chip in enumerate(("R2 halten", "Steuerkreuz 1. Richtung",
                                  "Steuerkreuz 2. Richtung", "R2 loslassen")):
            if i:
                lbl = QLabel()
                lbl.setPixmap(ui_icons.pixmap("chevron-right", ds.MUTED, 16))
                howto.addWidget(lbl)
            howto.addWidget(ds.key_chip(chip))
        howto.addWidget(QLabel("  Pose fährt an (geplant)."))
        howto.addStretch(1)
        root.addLayout(howto)
        hint = QLabel("Zeile = erste Richtung, Spalte = zweite. Beispiel: R2 halten, ↓ dann ↑, "
                      "R2 loslassen fährt die Pose im Feld »↓ ↑«. Kreuz verwirft die Wahl "
                      "vor dem Loslassen. Leere Felder tun nichts (Controller vibriert lang).")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color:{ds.MUTED};")
        root.addWidget(hint)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        corner = QLabel("zuerst ↓ / dann →")
        corner.setObjectName("qhead")
        grid.addWidget(corner, 0, 0, Qt.AlignmentFlag.AlignCenter)
        for c, d in enumerate(QUICK_DIRS):
            grid.addWidget(self._header("dann", d), 0, c + 1, Qt.AlignmentFlag.AlignCenter)
        self.cells = {}
        self.rows = {}
        for r, first in enumerate(QUICK_DIRS):
            grid.addWidget(self._header("zuerst", first), r + 1, 0, Qt.AlignmentFlag.AlignCenter)
            for c, second in enumerate(QUICK_DIRS):
                cell = QuickSlotCell(first, second, self._assign)
                self.cells[cell.combo_key] = cell
                self.rows.setdefault(first, []).append(cell)
                grid.addWidget(cell, r + 1, c + 1)
        for c in range(1, 5):
            grid.setColumnStretch(c, 1)
        root.addLayout(grid)
        self.status = QLabel("")
        self.status.setStyleSheet(f"color:{ds.MUTED};")
        root.addWidget(self.status)
        root.addStretch(1)
        self.reload()

    def _header(self, word, direction):
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        t = QLabel(word)
        t.setObjectName("qhead")
        lay.addWidget(t)
        lay.addWidget(arrow_label(direction, ds.ACCENT, 18))
        return w

    def reload(self):
        store = self.gui._pose_store()
        grouped = store.list_grouped() if store is not None else {}
        slots = store.list_quickslots() if store is not None else {}
        for key, cell in self.cells.items():
            cell.fill(grouped, slots.get(key))
        n = len(slots)
        self.status.setText(f"{n} von 16 Plätzen belegt." if n else
                            "Noch kein Platz belegt – Pose im Feld auswählen.")

    def _assign(self, key, pose):
        store = self.gui._pose_store()
        if store is None:
            return
        try:
            store.set_quickslot(key, pose)
        except ValueError as e:
            self.status.setText(str(e))
        self.reload()

    def highlight(self, quick):
        """Controller-Status "quick" ({first, combo} oder None) anzeigen."""
        quick = quick or {}
        combo, first = quick.get("combo"), quick.get("first")
        for key, cell in self.cells.items():
            if combo:
                cell.set_live("live" if key == combo else None)
            elif first:
                cell.set_live("row" if key.split("-")[0] == first else None)
            else:
                cell.set_live(None)

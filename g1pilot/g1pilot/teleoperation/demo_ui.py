#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo_ui — Bausteine der Demo-GUI (nur Oberflaeche, kein ROS).

Die Demo-GUI (demo_gui.py) ist seit 2026-11-17 aus wenigen, immer gleichen
Bausteinen zusammengesetzt, damit jede Funktion einen festen Platz hat und
alles mit der Fenstergroesse mitwaechst:

  Kopfleiste        SIM/REAL-Kennung, Modus-Umschalter (ModeTile), Chips rechts
  Arbeitsflaeche    Karten (Card) nebeneinander, bei schmalem Fenster
                    untereinander (ResponsiveRow)
  Listen            TileGrid: Kacheln im Raster, Spaltenzahl nach Breite,
                    Suche ab SEARCH_FROM Eintraegen, scrollt (Ablaeufe, Stationen)
  Seitenleiste      NOT-HALT, VOLL-NOTAUS (HoldButton), Roboter (real) bzw.
                    Simulation, Beenden
  Statusleiste      aktuelle Meldung + Verlauf
  Controller-Ansicht  ControllerCard (Zustand), LegendCard (Belegung), QuickGrid
                    (16 Plaetze der Pose-Schnellwahl) statt Joysticks

Farben aus utils/ui_theme.py (eine Quelle fuer alle G1-Fenster) plus die
Modus-Farben GEHEN blau / GREIFEN orange.
"""

import time

from PyQt6.QtWidgets import (
    QWidget, QFrame, QLabel, QPushButton, QLineEdit, QGridLayout, QVBoxLayout,
    QHBoxLayout, QBoxLayout, QScrollArea, QSizePolicy, QDialog, QTableWidget,
    QTableWidgetItem, QListWidget, QListWidgetItem, QStackedWidget,
)
from PyQt6.QtCore import Qt, QRectF, QPointF, QTimer, QSize, pyqtSignal
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont

from g1pilot.utils import ui_theme as T
from g1pilot.utils import ui_icons
from g1pilot.utils import g1_temperature as g1t
from g1pilot.utils import g1_battery as g1b

WALK = "walk"
MANIP = "manip"
# Farbcode je Modus -- Modus-Umschalter, Linie unter der Kopfleiste, Karten-Icons.
MODE_COLOR = {WALK: "#1e88e5", MANIP: "#fb8c00"}
MODE_TITLE = {WALK: "GEHEN", MANIP: "GREIFEN"}
MODE_ICON = {WALK: "footprints", MANIP: "hand"}

WAIST_COLOR = "#ab47bc"
SEQ_COLOR = "#1565c0"
STATION_COLOR = "#2e7d32"
DIM = "#5d6470"           # ausgegraut
SIDEBAR_W = 300
# Ab so vielen Eintraegen bekommt ein TileGrid ein Suchfeld.
SEARCH_FROM = 9


# ── Stil-Helfer ─────────────────────────────────────────────────────────
def button_css(bg=T.CARD_HI, fg="white", font=16, radius=12, border="#3a4250"):
    return f"""
        QPushButton {{ background:{bg}; color:{fg}; font-size:{font}px; font-weight:700;
                       border:1px solid {border}; border-radius:{radius}px; padding:6px 12px; }}
        QPushButton:hover {{ border:2px solid #8f97a3; }}
        QPushButton:pressed {{ background:#3a4250; }}
        QPushButton:checked {{ background:{T.ACCENT}; border:2px solid #8db4fa; }}
        QPushButton:disabled {{ background:{T.CARD}; color:{DIM}; border:1px solid {T.CARD_HI}; }}
    """


def state_css(bg, fg, border, font, radius=12):
    """Knopf-Stil fuer Zustaende (z.B. gestrichelt = laeuft)."""
    return f"""
        QPushButton {{ background:{bg}; color:{fg}; font-size:{font}px; font-weight:700;
                       border:{border}; border-radius:{radius}px; padding:6px 12px; }}
        QPushButton:disabled {{ background:{T.CARD}; color:{DIM}; border:1px solid {T.CARD_HI}; }}
    """


def big_button(text, color=T.CARD_HI, font=16, height=48, icon=None, icon_color="#ffffff"):
    """Grosser Bedienknopf (waechst in der Breite mit)."""
    b = QPushButton(text)
    b.setMinimumHeight(height)
    b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
    b.setStyleSheet(button_css(color, font=font))
    if icon:
        ui_icons.set_icon(b, icon, icon_color, max(16, font + 2))
        if text:
            b.setText("  " + text)      # Abstand Icon -> Text
    return b


def chip_button(text, icon=None):
    """Flacher Knopf der Kopfleiste (Controller, Handy, Vollbild, Verlauf)."""
    b = QPushButton(text)
    b.setFixedHeight(40)
    b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
    b.setStyleSheet(f"""
        QPushButton {{ background:{T.CARD}; color:{T.FG}; font-size:14px; font-weight:700;
                       border:1px solid {T.LINE}; border-radius:10px; padding:2px 14px; }}
        QPushButton:hover {{ border:1px solid #8f97a3; background:{T.CARD_HI}; }}
        QPushButton:pressed {{ background:#3a4250; }}
    """)
    if icon:
        ui_icons.set_icon(b, icon, T.FG, 18)
    return b


def caption(text, color=T.MUTED, size=13):
    lbl = QLabel(text)
    lbl.setStyleSheet(f"color:{color}; font-size:{size}px; background:transparent;")
    return lbl


def section(text):
    """Kleine Abschnitts-Ueberschrift in Grossbuchstaben (Seitenleiste)."""
    lbl = QLabel(text.upper())
    lbl.setStyleSheet(f"color:{T.MUTED}; font-size:11px; font-weight:700; letter-spacing:2px;"
                      "background:transparent; padding-top:4px;")
    return lbl


def segmented(names, on_pick, color=T.ACCENT, font=15, height=42, min_w=0):
    """Umschalter aus aneinanderliegenden Knoepfen (genau einer aktiv).
    -> (widget, {name: QPushButton}). Knoepfe sind checkable; on_pick(name)."""
    w = QWidget()
    lay = QHBoxLayout(w)
    lay.setContentsMargins(0, 0, 0, 0)
    lay.setSpacing(0)
    btns = {}
    n = len(names)
    for i, name in enumerate(names):
        b = QPushButton(name)
        b.setCheckable(True)
        b.setMinimumHeight(height)
        if min_w:
            b.setMinimumWidth(min_w)
        b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        left = "10px" if i == 0 else "0"
        right = "10px" if i == n - 1 else "0"
        b.setStyleSheet(f"""
            QPushButton {{ background:{T.CARD_HI}; color:{T.MUTED}; font-size:{font}px;
                           font-weight:800; border:1px solid {T.LINE};
                           border-left:{'1px' if i == 0 else '0'} solid {T.LINE};
                           border-top-left-radius:{left}; border-bottom-left-radius:{left};
                           border-top-right-radius:{right}; border-bottom-right-radius:{right};
                           padding:4px 14px; }}
            QPushButton:hover:!checked:enabled {{ color:{T.FG}; background:#303746; }}
            QPushButton:checked {{ background:{color}; color:white; border-color:{color}; }}
            QPushButton:disabled {{ color:{DIM}; background:{T.CARD}; }}
            QPushButton:checked:disabled {{ background:#1b2a3a; color:#c9d1dc;
                                            border:2px dashed #8db4fa; }}
        """)
        if on_pick is not None:
            b.clicked.connect(lambda _=False, nm=name: on_pick(nm))
        lay.addWidget(b)
        btns[name] = b
    return w, btns


# ── Karte ───────────────────────────────────────────────────────────────
class Card(QFrame):
    """Abgerundete Flaeche mit Kopf (Icon + Titel + Hinweis rechts).
    Inhalt in self.body (QVBoxLayout); self.head fuer Extras im Kopf."""

    def __init__(self, title, icon=None, color=T.FG, hint=""):
        super().__init__()
        self.setObjectName("dcard")
        self.setStyleSheet(f"QFrame#dcard {{ background:{T.CARD}; border-radius:14px; }}"
                           "QLabel { background:transparent; }")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 12, 16, 14)
        outer.setSpacing(10)
        self.head = QHBoxLayout()
        self.head.setSpacing(8)
        if icon:
            ic = QLabel()
            ic.setPixmap(ui_icons.pixmap(icon, color, 18))
            self.head.addWidget(ic)
        self.title = QLabel(title)
        self.title.setStyleSheet(f"color:{T.FG}; font-size:16px; font-weight:800;")
        self.head.addWidget(self.title)
        self.head.addStretch(1)
        self.hint = caption(hint, size=12)
        self.head.addWidget(self.hint)
        outer.addLayout(self.head)
        self.body = QVBoxLayout()
        self.body.setSpacing(10)
        outer.addLayout(self.body, 1)

    def set_hint(self, text):
        self.hint.setText(text)


class CurrentStack(QStackedWidget):
    """QStackedWidget, dessen Groesse nur von der SICHTBAREN Seite abhaengt
    (sonst bestimmt die breiteste Seite, z. B. HAENDE, die Mindestbreite)."""

    def sizeHint(self):
        w = self.currentWidget()
        return w.sizeHint() if w is not None else super().sizeHint()

    def minimumSizeHint(self):
        w = self.currentWidget()
        return w.minimumSizeHint() if w is not None else super().minimumSizeHint()


# ── Nebeneinander / untereinander ───────────────────────────────────────
class ResponsiveRow(QWidget):
    """Spalten nebeneinander; ist das Widget schmaler als `breakpoint`,
    stehen sie untereinander (die Arbeitsflaeche scrollt dann)."""

    def __init__(self, breakpoint=1050, spacing=14):
        super().__init__()
        self.breakpoint = breakpoint
        self.lay = QBoxLayout(QBoxLayout.Direction.LeftToRight, self)
        self.lay.setContentsMargins(0, 0, 0, 0)
        self.lay.setSpacing(spacing)
        self._stretch = []

    def add(self, widget, stretch=1):
        self.lay.addWidget(widget, stretch)
        self._stretch.append(stretch)

    @property
    def stacked(self):
        return self.lay.direction() == QBoxLayout.Direction.TopToBottom

    def set_stacked(self, on):
        want = QBoxLayout.Direction.TopToBottom if on else QBoxLayout.Direction.LeftToRight
        if self.lay.direction() != want:
            self.lay.setDirection(want)
            for i, s in enumerate(self._stretch):
                self.lay.setStretch(i, 0 if on else s)

    def resizeEvent(self, e):
        self.set_stacked(e.size().width() < self.breakpoint)
        super().resizeEvent(e)

    def minimumSizeHint(self):
        # Breite nie ueber dem Umbruchpunkt erzwingen: schmaler -> untereinander.
        h = super().minimumSizeHint()
        if not self.stacked:
            ws = [self.lay.itemAt(i).widget() for i in range(self.lay.count())]
            w = max([x.minimumSizeHint().width() for x in ws if not x.isHidden()] or [0])
            return QSize(min(h.width(), w), h.height())
        return h


# ── Kachel-Raster (Ablaeufe, Stationen) ─────────────────────────────────
class _GridBody(QWidget):
    def __init__(self, owner):
        super().__init__()
        self.owner = owner

    def resizeEvent(self, e):
        self.owner._reflow(e.size().width())
        super().resizeEvent(e)


class TileGrid(QWidget):
    """Knoepfe als Raster: Spaltenzahl folgt der Breite (min. Kachelbreite
    `tile_w`), ab SEARCH_FROM Eintraegen erscheint ein Suchfeld, mehr als
    sichtbar passt scrollt. Das Layout der Kacheln ist `self.grid`
    (QGridLayout, count()/itemAt() wie frueher die Knopf-Zeile)."""

    def __init__(self, tile_w=180, min_rows=2, tile_h=56, placeholder="suchen …"):
        super().__init__()
        self.tile_w = tile_w
        self.tile_h = tile_h
        self._cols = 0
        self._widgets = []
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        self.search = QLineEdit()
        self.search.setPlaceholderText(placeholder)
        self.search.setClearButtonEnabled(True)
        self.search.setMinimumHeight(36)
        self.search.addAction(ui_icons.icon("search", T.MUTED, 16),
                              QLineEdit.ActionPosition.LeadingPosition)
        self.search.textChanged.connect(self._filter)
        self.search.setVisible(False)
        lay.addWidget(self.search)
        self.empty = caption("")
        self.empty.setWordWrap(True)
        self.empty.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self.empty.setVisible(False)
        lay.addWidget(self.empty, 1)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.setStyleSheet("QScrollArea { background:transparent; }")
        self.body = _GridBody(self)
        self.body.setStyleSheet("background:transparent;")
        self.grid = QGridLayout(self.body)
        self.grid.setContentsMargins(0, 0, 4, 0)
        self.grid.setSpacing(8)
        self.grid.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.scroll.setWidget(self.body)
        self.scroll.setMinimumHeight(min_rows * (tile_h + 8))
        lay.addWidget(self.scroll, 1)

    def set_widgets(self, widgets, empty_text=""):
        """Kacheln ersetzen (alte werden geloescht)."""
        for w in self._widgets:
            self.grid.removeWidget(w)
            w.hide()            # deleteLater allein laesst sie bis zum Loeschen sichtbar
            w.deleteLater()
        self._widgets = list(widgets)
        for w in self._widgets:
            w.setMinimumHeight(self.tile_h)
            w.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.empty.setText(empty_text)
        self.empty.setVisible(not self._widgets and bool(empty_text))
        self.scroll.setVisible(bool(self._widgets))
        self.search.setVisible(len(self._widgets) >= SEARCH_FROM)
        if not self.search.isVisible():
            self.search.clear()
        self._cols = 0
        self._reflow(self.body.width() or self.width())

    def widgets(self):
        return list(self._widgets)

    def _visible(self):
        q = self.search.text().strip().lower()
        if not q:
            return list(self._widgets)
        return [w for w in self._widgets
                if q in (w.text() + " " + (w.toolTip() or "")).lower()]

    def _filter(self, *_):
        self._cols = 0
        self._reflow(self.body.width())

    def _reflow(self, width):
        cols = max(1, int((max(width, 1) + 8) // (self.tile_w + 8)))
        if cols == self._cols and self.grid.count() == len(self._visible()):
            return
        self._cols = cols
        while self.grid.count():
            self.grid.takeAt(0)
        shown = self._visible()
        for w in self._widgets:
            w.setVisible(w in shown)
        for i, w in enumerate(shown):
            self.grid.addWidget(w, i // cols, i % cols)
        for c in range(cols):
            self.grid.setColumnStretch(c, 1)
        for c in range(cols, cols + 8):
            self.grid.setColumnStretch(c, 0)


# ── Halte-Knopf (VOLL-NOTAUS) ───────────────────────────────────────────
class HoldButton(QPushButton):
    """Knopf, der erst nach ``hold_s`` ununterbrochenem Halten ausloest
    (Schutz gegen versehentliches Antippen, wie R1 + L1 + PS am Controller).

    Signale: ``armed`` sofort beim Druecken, ``fired`` nach Ablauf der
    Haltezeit (genau einmal je Druck), ``aborted`` beim Loslassen vorher.
    Der Fortschritt fuellt den Knopf von links (Farbe ``fill``). Gezeichnet
    wird per Stylesheet-Verlauf, damit Theme/Schrift wie bei den anderen
    Knoepfen bleiben. ``clock`` ist fuer Tests austauschbar."""

    armed = pyqtSignal()
    fired = pyqtSignal()
    aborted = pyqtSignal()

    TICK_MS = 30

    def __init__(self, text, hold_s=1.5, base="#3a1216", fill="#ff1744",
                 border="#ff5252", font=17, height=64, clock=time.monotonic):
        super().__init__(text)
        self.hold_s = max(0.05, float(hold_s))
        self._base, self._fill, self._border, self._font = base, fill, border, font
        self._clock = clock
        self._since = None          # Zeitpunkt des Drueckens (None = nicht gehalten)
        self._fired = False
        self.setMinimumHeight(height)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)   # Leertaste loest nichts aus
        self._timer = QTimer(self)
        self._timer.setInterval(self.TICK_MS)
        self._timer.timeout.connect(self._tick)
        self.pressed.connect(self.begin)
        self.released.connect(self.cancel)
        self._paint(0.0)

    @property
    def holding(self):
        """True, solange gehalten und noch nicht ausgeloest."""
        return self._since is not None and not self._fired

    def progress(self):
        if self._since is None:
            return 0.0
        if self._fired:
            return 1.0
        return min(1.0, max(0.0, (self._clock() - self._since) / self.hold_s))

    def begin(self):
        if not self.isEnabled() or self._since is not None:
            return
        self._since = self._clock()
        self._fired = False
        self._timer.start()
        self.armed.emit()
        self._paint(0.0)

    def cancel(self):
        """Loslassen: vor Ablauf = Abbruch, danach nur zuruecksetzen."""
        if self._since is None:
            return
        was = self.holding
        self._since = None
        self._fired = False
        self._timer.stop()
        self._paint(0.0)
        if was:
            self.aborted.emit()

    def _tick(self):
        if self._since is None or self._fired:
            self._timer.stop()
            return
        p = self.progress()
        self._paint(p)
        if p >= 1.0:
            self._fired = True
            self._timer.stop()
            self.fired.emit()

    def _paint(self, p):
        if p <= 0.0:
            bg = self._base
        elif p >= 1.0:
            bg = self._fill
        else:
            q = min(0.999, p)
            bg = (f"qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {self._fill}, "
                  f"stop:{q:.3f} {self._fill}, stop:{q + 0.001:.3f} {self._base}, "
                  f"stop:1 {self._base})")
        self.setStyleSheet(f"""
            QPushButton {{ background:{bg}; color:white; font-size:{self._font}px;
                           font-weight:900; border:2px solid {self._border};
                           border-radius:14px; padding:4px 10px; }}
            QPushButton:hover {{ border:3px solid {self._border}; }}
            QPushButton:disabled {{ background:{T.CARD}; color:{DIM};
                                    border:1px solid {T.CARD_HI}; }}
        """)

    def hideEvent(self, e):
        self.cancel()
        super().hideEvent(e)


# ── Modus-Umschalter ────────────────────────────────────────────────────
class ModeTile(QPushButton):
    """Ein Teil des Modus-Umschalters in der Kopfleiste. Drei Zustaende:
    aktiv (Modusfarbe) / wechselt (gestrichelt) / inaktiv (grau, Untertitel =
    Hinweis wie »antippen zum Wechseln« oder »gesperrt · kein Bodenkontakt«).
    text() bleibt »TITEL\\nUntertitel« (Tests, Handy), gezeichnet wird selbst."""

    def __init__(self, mode, side="left"):
        super().__init__()
        self.mode = mode
        self.side = side
        self.hint = None
        self.state = "inactive"
        self.setFixedHeight(54)
        self.setMinimumWidth(200)
        self.setMaximumWidth(360)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.set_state("inactive")

    def set_hint(self, hint):
        """Vorab-Hinweis (real, GEHEN): z. B. »gesperrt · kein Bodenkontakt«."""
        if hint != self.hint:
            self.hint = hint
            self.set_state(self.state)

    def subtitle(self):
        if self.state == "active":
            return "AKTIV"
        if self.state == "pending":
            return "wechselt …"
        return self.hint or "antippen zum Wechseln"

    def set_state(self, state):
        self.state = state
        self.setText(f"{MODE_TITLE[self.mode]}\n{self.subtitle()}")
        self.setToolTip(f"{MODE_TITLE[self.mode]}: {self.subtitle()}")
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = QColor(MODE_COLOR[self.mode])
        r = QRectF(self.rect()).adjusted(1.5, 1.5, -1.5, -1.5)
        enabled = self.isEnabled()
        if self.state == "active":
            bg, fg, sub = color, QColor("white"), QColor(255, 255, 255, 220)
        elif self.state == "pending":
            bg, fg, sub = QColor(T.CARD_HI), color, color
        else:
            bg = QColor(T.CARD_HI if self.underMouse() and enabled else T.CARD)
            fg = QColor(T.MUTED if enabled else DIM)
            sub = QColor("#6b7380" if enabled else DIM)
        p.setBrush(QBrush(bg))
        if self.state == "pending":
            pen = QPen(color, 3, Qt.PenStyle.DashLine)
        elif self.state == "active":
            pen = QPen(QColor("white"), 2)
        else:
            pen = QPen(QColor(T.LINE), 1)
        p.setPen(pen)
        p.drawRoundedRect(r, 12, 12)
        # Icon + Titel + Untertitel, zentriert
        icon = ui_icons.pixmap(MODE_ICON[self.mode], fg.name(), 24)
        tf = QFont(self.font())
        tf.setPixelSize(19)
        tf.setBold(True)
        sf = QFont(self.font())
        sf.setPixelSize(12)
        sf.setBold(True)
        title = MODE_TITLE[self.mode]
        p.setFont(tf)
        tw = p.fontMetrics().horizontalAdvance(title)
        p.setFont(sf)
        subtext = p.fontMetrics().elidedText(self.subtitle(), Qt.TextElideMode.ElideRight,
                                             int(r.width() - 70))
        sw = p.fontMetrics().horizontalAdvance(subtext)
        block = 30 + max(tw, sw)
        x = r.center().x() - block / 2
        if not icon.isNull():
            p.drawPixmap(QPointF(x, r.center().y() - 12), icon)
        p.setPen(fg)
        p.setFont(tf)
        p.drawText(QRectF(x + 32, r.top() + 5, tw + 4, 26),
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, title)
        p.setPen(sub)
        p.setFont(sf)
        p.drawText(QRectF(x + 32, r.top() + 29, sw + 4, 18),
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, subtext)

    def enterEvent(self, e):
        self.update()
        super().enterEvent(e)

    def leaveEvent(self, e):
        self.update()
        super().leaveEvent(e)


# ── Senkrechter Schieber ────────────────────────────────────────────────
class VerticalRocker(QWidget):
    """Senkrechter Schieber, der zurueckfedert (Gegenstueck zum VirtualJoystick):
    nach oben ziehen -> value = +1 (hoch), nach unten -> -1, loslassen -> 0."""

    def __init__(self, height=190, width=60, labels=("chevron-up", "chevron-down")):
        super().__init__()
        self.setFixedSize(width, height)
        self._labels = labels
        self._travel = height / 2 - 24     # Knopf-Weg je Richtung (Pixel)
        self._knob = 0.0                   # Versatz vom Zentrum, + = nach unten
        self.value = 0.0

    def set_length(self, height):
        """Hoehe aendern (folgt der Joystick-Groesse)."""
        height = int(height)
        if height == self.height():
            return
        self.setFixedSize(self.width(), height)
        self._travel = height / 2 - 24
        self.mouseReleaseEvent(None)

    def _set_from_y(self, y):
        self._knob = max(-self._travel, min(self._travel, y - self.height() / 2))
        self.value = -self._knob / self._travel
        self.update()

    def mousePressEvent(self, e):
        self._set_from_y(e.position().y())

    def mouseMoveEvent(self, e):
        self._set_from_y(e.position().y())

    def mouseReleaseEvent(self, e):
        self._knob = 0.0
        self.value = 0.0
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        p.setBrush(QBrush(QColor("#1b1f27")))
        p.setPen(QPen(QColor("#3a4250"), 2))
        p.drawRoundedRect(QRectF(cx - 25, 4, 50, h - 8), 25, 25)
        p.setPen(QPen(QColor(T.LINE), 1))
        p.drawLine(int(cx - 16), int(cy), int(cx + 16), int(cy))
        for name, y in ((self._labels[0], 9), (self._labels[1], h - 27)):
            pm = ui_icons.pixmap(name, "#6b7380", 18)
            if not pm.isNull():
                p.drawPixmap(QPointF(cx - 9, y), pm)
        active = abs(self.value) > 1e-3
        p.setBrush(QBrush(QColor(T.ACCENT) if active else QColor(T.CARD_HI)))
        p.setPen(QPen(QColor("#8db4fa") if active else QColor("#6b7380"), 2))
        p.drawEllipse(QPointF(cx, cy + self._knob), 15, 15)


class HorizontalRocker(QWidget):
    """Waagrechter Schieber, der zurueckfedert (Hueftdrehung): nach links
    ziehen -> value = +1 (links drehen, wie waist_yaw: + = links), nach rechts
    -> -1, loslassen -> 0. Mindestbreite `width`, waechst mit."""

    def __init__(self, width=260, height=52, labels=("rotate-ccw", "rotate-cw")):
        super().__init__()
        self.setMinimumWidth(width)
        self.setFixedHeight(height)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._labels = labels
        self._knob = 0.0                   # Versatz vom Zentrum, + = nach rechts
        self.value = 0.0

    def _travel(self):
        return max(10.0, self.width() / 2 - 30)

    def _set_from_x(self, x):
        t = self._travel()
        self._knob = max(-t, min(t, x - self.width() / 2))
        self.value = -self._knob / t
        self.update()

    def mousePressEvent(self, e):
        self._set_from_x(e.position().x())

    def mouseMoveEvent(self, e):
        self._set_from_x(e.position().x())

    def mouseReleaseEvent(self, e):
        self._knob = 0.0
        self.value = 0.0
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        p.setBrush(QBrush(QColor("#1b1f27")))
        p.setPen(QPen(QColor("#3a4250"), 2))
        p.drawRoundedRect(QRectF(4, cy - 23, w - 8, 46), 23, 23)
        p.setPen(QPen(QColor(T.LINE), 1))
        p.drawLine(int(cx), int(cy - 14), int(cx), int(cy + 14))
        for name, x in ((self._labels[0], 14), (self._labels[1], w - 32)):
            pm = ui_icons.pixmap(name, "#6b7380", 18)
            if not pm.isNull():
                p.drawPixmap(QPointF(x, cy - 9), pm)
        active = abs(self.value) > 1e-3
        p.setBrush(QBrush(QColor(WAIST_COLOR) if active else QColor(T.CARD_HI)))
        p.setPen(QPen(QColor("#ce93d8") if active else QColor("#6b7380"), 2))
        p.drawEllipse(QPointF(cx + self._knob, cy), 15, 15)


# ── Akku + Temperaturen (real, Seitenleiste) ────────────────────────────
class TempPanel(QFrame):
    """Real: Akku und Temperaturen des G1 als Kachel-Raster in der Seitenleiste.
    Oben Akku (rt/lf/bmsstate) und heissester Motor, darunter je Gruppe (Bein
    L/R, Taille, Arm L/R) das Maximum, farbig nach Schwelle. Klick = Tabelle
    aller 29 Motoren (beide Sensoren) + IMU."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.warn, self.crit = g1t.thresholds()
        self.batt_warn, self.batt_crit = g1b.thresholds()
        self.setObjectName("temp")
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Klick: alle Motortemperaturen")
        self.setStyleSheet("QFrame#temp { background:transparent; } "
                           "QLabel { background:transparent; }")
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(6)
        self.batt_tile = self._tile()
        grid.addWidget(self.batt_tile, 0, 0, 1, 3)
        self._paint(self.batt_tile, "Akku", "–", "", "none")
        self.tiles = {}
        keys = (("max", "Max"),) + tuple((k, t) for k, t, _ in g1t.GROUPS)
        self.tiles["max"] = self._tile()
        grid.addWidget(self.tiles["max"], 0, 3, 1, 3)
        for i, (key, title) in enumerate(keys[1:]):
            tile = self._tile()
            grid.addWidget(tile, 1 + i // 3, (i % 3) * 2, 1, 2)
            self.tiles[key] = tile
        for key, title in keys:
            self._paint(self.tiles[key], title, "–", "", "none")
        self.dialog = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)
        self.refresh()

    @staticmethod
    def _tile():
        t = QLabel()
        t.setAlignment(Qt.AlignmentFlag.AlignCenter)
        t.setMinimumHeight(52)
        t.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        return t

    @staticmethod
    def _paint(tile, title, value, sub, lvl):
        tile.setText(f"<div style='font-size:10px; color:#e8eaed'>{title}</div>"
                     f"<div style='font-size:16px; font-weight:800'>{value}</div>"
                     f"<div style='font-size:9px; color:#eee'>{sub}</div>")
        tile.setStyleSheet(f"background:{g1t.LEVEL_COLOR[lvl]}; color:white;"
                           " border-radius:8px;")

    def refresh_battery(self):
        data = self.node.battery
        if data is None or g1b.is_stale(self.node.battery_t, time.monotonic()):
            self._paint(self.batt_tile, "Akku", "–", "keine Daten", "none")
            self.batt_tile.setToolTip(f"Keine Akkudaten vom Roboter "
                                      f"(robot_state, {g1b.TOPIC_DDS}).")
            return
        soc = data["soc"]
        self._paint(self.batt_tile, "Akku", f"{soc:.0f} %", g1b.subtitle(data),
                    g1b.level(soc, self.batt_warn, self.batt_crit))
        self.batt_tile.setToolTip(
            g1b.details(data) + f"\n\nGelb ab {self.batt_warn:.0f} %, rot ab "
            f"{self.batt_crit:.0f} % (Annahme, anpassbar: G1_BATT_WARN / G1_BATT_CRIT).")

    def refresh(self):
        self.refresh_battery()
        data = self.node.temps
        if data is None or g1t.is_stale(self.node.temps_t, time.monotonic()):
            for key, tile in self.tiles.items():
                title = "Max" if key == "max" else dict((k, t) for k, t, _ in g1t.GROUPS)[key]
                self._paint(tile, title, "–", "keine Daten", "none")
                tile.setToolTip("Keine Temperaturdaten vom Roboter (robot_state).")
            return
        idx, top = g1t.hottest(data)
        self._paint(self.tiles["max"], "Max", f"{top:.0f} °C", g1t.motor_label(idx),
                    g1t.level(top, self.warn, self.crit))
        self.tiles["max"].setToolTip(f"Heissester Motor: {g1t.motor_label(idx)}, {top:.0f} °C")
        for g in g1t.group_summary(data):
            tile = self.tiles[g["key"]]
            self._paint(tile, g["title"], f"{g['max']:.0f} °C", g["label"],
                        g1t.level(g["max"], self.warn, self.crit))
            tile.setToolTip("\n".join(
                f"{g1t.motor_label(i)}: {t:.0f} °C" for i, t in g["temps"].items()))
        if self.dialog is not None and self.dialog.isVisible():
            self._fill_dialog(data)

    def mousePressEvent(self, e):
        data = self.node.temps
        if data is None:
            return
        if self.dialog is None:
            dlg = self.dialog = QDialog(self)
            dlg.setWindowTitle("Motortemperaturen")
            dlg.setStyleSheet(T.stylesheet())
            v = QVBoxLayout(dlg)
            v.setContentsMargins(18, 18, 18, 16)
            v.setSpacing(12)
            head = QHBoxLayout()
            head.setSpacing(12)
            badge = QLabel()
            badge.setPixmap(ui_icons.pixmap("cpu", T.ACCENT, 22))
            badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            badge.setFixedSize(40, 40)
            badge.setStyleSheet(f"background:{T.CARD_HI}; border-radius:10px;")
            head.addWidget(badge)
            col = QVBoxLayout()
            col.setSpacing(1)
            t = QLabel("Motortemperaturen")
            t.setStyleSheet("font-size:14pt; font-weight:700;")
            col.addWidget(t)
            sub = QLabel(f"Gelb ab {self.warn:.0f} °C, rot ab {self.crit:.0f} °C "
                         "(Annahme, anpassbar: G1_TEMP_WARN / G1_TEMP_CRIT).")
            sub.setWordWrap(True)
            sub.setStyleSheet(f"color:{T.MUTED};")
            col.addWidget(sub)
            head.addLayout(col, 1)
            v.addLayout(head)
            self.table = QTableWidget(g1t.N_MOTORS + 1, 4)
            self.table.setHorizontalHeaderLabels(
                ["Motor", "Sensor 1 [°C]", "Sensor 2 [°C]", "Spannung [V]"])
            self.table.verticalHeader().setVisible(False)
            self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
            self.table.setShowGrid(False)
            self.table.horizontalHeader().setStretchLastSection(True)
            self.table.setColumnWidth(0, 200)
            for c in (1, 2, 3):
                self.table.setColumnWidth(c, 110)
            v.addWidget(self.table, 1)
            self.dialog.resize(600, 760)
        self._fill_dialog(data)
        self.dialog.show()
        self.dialog.raise_()

    def _fill_dialog(self, data):
        def cell(r, c, text, lvl=None):
            it = QTableWidgetItem(text)
            if lvl:
                it.setBackground(QColor(g1t.LEVEL_COLOR[lvl]))
            self.table.setItem(r, c, it)
        for i in range(g1t.N_MOTORS):
            cell(i, 0, g1t.motor_label(i))
            for c in (0, 1):
                t = data["t"][i][c]
                cell(i, c + 1, f"{t:.0f}", g1t.level(t, self.warn, self.crit))
            cell(i, 3, f"{data['v'][i]:.1f}" if i < len(data["v"]) else "–")
        r = g1t.N_MOTORS
        cell(r, 0, "IMU")
        cell(r, 1, f"{data['imu']:.0f}", g1t.level(data["imu"], self.warn, self.crit))
        cell(r, 2, "–")
        cell(r, 3, "–")


# ── Meldungs-Verlauf ────────────────────────────────────────────────────
class HistoryDialog(QDialog):
    """Die letzten Meldungen der Statusleiste (neueste oben)."""

    def __init__(self, parent, entries):
        super().__init__(parent)
        self.setWindowTitle("Meldungen")
        self.setStyleSheet(T.stylesheet())
        self.resize(760, 560)
        v = QVBoxLayout(self)
        v.setContentsMargins(18, 16, 18, 14)
        v.setSpacing(10)
        t = QLabel("Meldungen")
        t.setStyleSheet("font-size:15pt; font-weight:700;")
        v.addWidget(t)
        v.addWidget(caption("Was die Demo-GUI zuletzt gemeldet hat, neueste oben."))
        self.list = QListWidget()
        self.list.setStyleSheet(f"QListWidget {{ background:{T.CARD}; border:none;"
                                f" border-radius:10px; padding:6px; font-size:14px; }}"
                                f"QListWidget::item {{ padding:6px 4px; border-bottom:1px solid {T.LINE}; }}")
        v.addWidget(self.list, 1)
        row = QHBoxLayout()
        row.addStretch(1)
        close = QPushButton("Schließen")
        close.clicked.connect(self.close)
        row.addWidget(close)
        v.addLayout(row)
        self.fill(entries)

    def fill(self, entries):
        self.list.clear()
        for stamp, text, color in reversed(list(entries)):
            it = QListWidgetItem(f"{stamp}   {text}")
            it.setForeground(QColor(color))
            self.list.addItem(it)


# ── Controller-Ansicht ──────────────────────────────────────────────────
# Umschalter in der Kopfleiste: TOUCH (alles per Klick) | CONTROLLER (was man
# am Controller ohnehin macht -- Joysticks, Pfeiltasten -- faellt weg, dafuer
# Zustand des Controllers, Belegung und Pose-Schnellwahl gross).
VIEW_TOUCH = "touch"
VIEW_CONTROLLER = "controller"
VIEW_TITLE = {VIEW_TOUCH: "Touch", VIEW_CONTROLLER: "Controller"}
VIEW_ICON = {VIEW_TOUCH: "layout-grid", VIEW_CONTROLLER: "gamepad-2"}
VIEW_ENV = "G1_DEMO_VIEW"
QUICK_ARROW = {"up": "arrow-up", "right": "arrow-right", "down": "arrow-down",
               "left": "arrow-left"}
QUICK_DIRS = ("up", "right", "down", "left")      # wie pose_store (Uhrzeigersinn)
LIVE_COLOR = "#66bb6a"


def default_view(env=None):
    """Start-Ansicht aus G1_DEMO_VIEW (touch | controller), sonst Touch."""
    import os
    v = (env if env is not None else os.environ.get(VIEW_ENV, "")).strip().lower()
    return VIEW_CONTROLLER if v in (VIEW_CONTROLLER, "pad", "gamepad") else VIEW_TOUCH


def key_chip(text, color=T.ACCENT):
    lbl = QLabel(text)
    lbl.setStyleSheet(f"background:{T.CARD_HI}; color:{color}; border:1px solid {T.LINE};"
                      "border-radius:8px; padding:3px 9px; font-size:13px; font-weight:700;")
    lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
    return lbl


class FactTile(QFrame):
    """Kleine Kachel: Titel (grau) + Wert (gross, farbig)."""

    def __init__(self, title):
        super().__init__()
        self.setObjectName("fact")
        self.setStyleSheet(f"QFrame#fact {{ background:{T.CARD_HI}; border-radius:10px; }}"
                           "QLabel { background:transparent; }")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(2)
        self.title = caption(title, size=12)
        lay.addWidget(self.title)
        self.value = QLabel("–")
        self.value.setWordWrap(True)
        lay.addWidget(self.value)
        self.set_value("–")

    def set_value(self, text, color=T.FG):
        self.value.setText(text)
        self.value.setStyleSheet(f"color:{color}; font-size:18px; font-weight:800;")


class ControllerCard(Card):
    """Zustand des Controllers fuer die Controller-Ansicht: eine Zeile, was
    der Controller gerade tut (status_line.ps4_text), darunter Kacheln mit
    den wichtigsten Werten."""

    def __init__(self, color, facts):
        super().__init__("Controller", "gamepad-2", color)
        self.line = QLabel()
        self.line.setWordWrap(True)
        self.body.addWidget(self.line)
        grid = QGridLayout()
        grid.setSpacing(8)
        self.facts = {}
        for i, (key, title) in enumerate(facts):
            t = FactTile(title)
            grid.addWidget(t, i // 2, i % 2)
            self.facts[key] = t
        self.body.addLayout(grid)
        self.set_line(None)

    def set_line(self, res):
        """res = (Text, Farbe) aus status_line.ps4_text oder None."""
        text, color = res if res else ("Kein Controller aktiv – im Startmenü unter "
                                       "Ausstattung einschalten.", T.MUTED)
        self.line.setText(text)
        self.line.setStyleSheet(f"color:{color}; font-size:16px; font-weight:800; "
                                f"background:{T.BG}; border-left:4px solid {color}; "
                                "border-radius:6px; padding:8px 10px;")

    def set_fact(self, key, text, color=T.FG):
        self.facts[key].set_value(text, color)


class LegendCard(Card):
    """Tastenbelegung (Taste als Chip + Wirkung), nach Bereichen, scrollt."""

    def __init__(self):
        super().__init__("Belegung", "book-open", T.FG)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.setStyleSheet("QScrollArea { background:transparent; }")
        inner = QWidget()
        inner.setStyleSheet("background:transparent;")
        self.lay = QVBoxLayout(inner)
        self.lay.setContentsMargins(0, 0, 8, 0)
        self.lay.setSpacing(6)
        self.scroll.setWidget(inner)
        self.scroll.setMinimumHeight(130)
        self.body.addWidget(self.scroll, 1)
        self._key = None

    def rows(self):
        """[(Taste, Wirkung)] wie angezeigt (Tests)."""
        out = []
        for i in range(self.lay.count()):
            w = self.lay.itemAt(i).widget()
            if w is not None and getattr(w, "entry", None):
                out.append(w.entry)
        return out

    def set_groups(self, groups, colors):
        """groups = [(Titel, Icon, Farbschluessel, [(Taste, Wirkung)])]."""
        key = repr(groups)
        if key == self._key:
            return
        self._key = key
        while self.lay.count():
            item = self.lay.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        for title, _icon, ck, rows in groups:
            col = colors.get(ck, T.ACCENT)
            head = QLabel(title.upper())
            head.setStyleSheet(f"color:{col}; font-size:11px; font-weight:800; "
                               "letter-spacing:1px; padding-top:6px;")
            self.lay.addWidget(head)
            for k, what in rows:
                row = QWidget()
                row.entry = (k, what)
                hl = QHBoxLayout(row)
                hl.setContentsMargins(0, 0, 0, 0)
                hl.setSpacing(10)
                hl.addWidget(key_chip(k, col), 0, Qt.AlignmentFlag.AlignTop)
                w = QLabel(what)
                w.setWordWrap(True)
                w.setStyleSheet(f"color:{T.FG}; font-size:13px;")
                hl.addWidget(w, 1)
                self.lay.addWidget(row)
        self.lay.addStretch(1)


class QuickGrid(QWidget):
    """Pose-Schnellwahl (R2 halten + Steuerkreuz 2x) als 4x4-Kacheln: Zeile =
    erste Richtung, Spalte = zweite. Klick faehrt die Pose an (on_pick(name)),
    die Wahl am Controller leuchtet live (highlight)."""

    def __init__(self, on_pick=None):
        super().__init__()
        self.on_pick = on_pick
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(6)
        for c, d in enumerate(QUICK_DIRS):
            grid.addWidget(self._arrow(d), 0, c + 1, Qt.AlignmentFlag.AlignCenter)
        self.cells = {}
        self.names = {}
        for r, first in enumerate(QUICK_DIRS):
            grid.addWidget(self._arrow(first), r + 1, 0, Qt.AlignmentFlag.AlignCenter)
            for c, second in enumerate(QUICK_DIRS):
                key = f"{first}-{second}"
                b = QPushButton("–")
                b.setMinimumHeight(44)
                b.setMinimumWidth(70)
                b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
                b.clicked.connect(lambda _=False, k=key: self._clicked(k))
                grid.addWidget(b, r + 1, c + 1)
                self.cells[key] = b
        for c in range(1, 5):
            grid.setColumnStretch(c, 1)
        self._live = (None, None)
        self.set_slots({})

    @staticmethod
    def _arrow(direction):
        lbl = QLabel()
        lbl.setPixmap(ui_icons.pixmap(QUICK_ARROW[direction], WAIST_COLOR, 18))
        lbl.setStyleSheet("background:transparent;")
        return lbl

    def _clicked(self, key):
        name = self.names.get(key)
        if name and self.on_pick is not None:
            self.on_pick(name)

    def set_slots(self, slots):
        """slots = {"up-down": Pose, ...} (pose_store.list_quickslots)."""
        self.names = dict(slots or {})
        for key, b in self.cells.items():
            name = self.names.get(key)
            b.setText(name or "–")      # leer: bleibt klickbar, tut aber nichts
            arrows = " ".join({"up": "↑", "right": "→", "down": "↓", "left": "←"}[d]
                              for d in key.split("-"))
            b.setToolTip(f"R2 halten, Steuerkreuz {arrows}, R2 loslassen"
                         + (f" = »{name}« (Klick fährt hier an)" if name else " (leer)"))
        self._paint()

    def highlight(self, quick):
        """Controller-Status "quick" ({first, combo} oder None)."""
        quick = quick or {}
        live = (quick.get("first"), quick.get("combo"))
        if live != self._live:
            self._live = live
            self._paint()

    def state_of(self, key):
        first, combo = self._live
        if combo:
            return "live" if key == combo else ""
        if first and key.split("-")[0] == first:
            return "row"
        return ""

    def _paint(self):
        for key, b in self.cells.items():
            st = self.state_of(key)
            if st == "live":
                c = LIVE_COLOR if self.names.get(key) else "#ffb300"   # leer = gelb
                css = state_css(c, "white", f"3px solid {c}", 13, radius=10)
            elif st == "row":
                css = state_css(T.CARD_HI, T.FG, f"2px solid {WAIST_COLOR}", 13, radius=10)
            elif self.names.get(key):
                css = button_css(T.CARD_HI, T.FG, 13, radius=10)
            else:
                css = state_css(T.CARD, DIM, f"1px dashed {T.LINE}", 13, radius=10)
            b.setStyleSheet(css + " QPushButton { padding:4px 4px; }")

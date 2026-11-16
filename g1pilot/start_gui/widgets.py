"""Eigene Bausteine des Startmenues: Schalter, Auswahl-Karten, Umgebungs-
Kacheln mit Vorschaubild, Abschnitte, aufklappbarer Bereich, Kachel-Raster."""
from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import (QEasingCurve, QPropertyAnimation, QRectF, QSize, Qt,
                          pyqtProperty, pyqtSignal)
from PyQt6.QtGui import (QBrush, QColor, QFont, QLinearGradient, QPainter,
                         QPainterPath, QPen, QPixmap)
from PyQt6.QtWidgets import (QAbstractButton, QButtonGroup, QFrame, QGridLayout,
                             QHBoxLayout, QLabel, QPushButton, QSizePolicy,
                             QToolButton, QVBoxLayout, QWidget)

from . import theme as T


def label(text: str, role: str = "", wrap: bool = False, rich: bool = False) -> QLabel:
    lbl = QLabel(text)
    if role:
        lbl.setProperty("role", role)
    if wrap:
        lbl.setWordWrap(True)
    if rich:
        lbl.setTextFormat(Qt.TextFormat.RichText)
        lbl.setTextInteractionFlags(Qt.TextInteractionFlag.LinksAccessibleByMouse)
    return lbl


def button(text: str, kind: str = "", on_click=None) -> QPushButton:
    b = QPushButton(text)
    if kind:
        b.setProperty("kind", kind)
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    if on_click is not None:
        b.clicked.connect(lambda _=False: on_click())
    return b


def section_title(text: str) -> QLabel:
    return label(text.upper(), "section")


def card(parent=None, name: str = "card") -> tuple[QFrame, QVBoxLayout]:
    f = QFrame(parent)
    f.setObjectName(name)
    lay = QVBoxLayout(f)
    lay.setContentsMargins(16, 12, 16, 12)
    lay.setSpacing(6)
    return f, lay


def hline(name: str = "rowline") -> QFrame:
    f = QFrame()
    f.setObjectName(name)
    f.setFixedHeight(1)
    f.setStyleSheet(f"background:{T.LINE};")
    return f


# ════════════════════════════════════════════════════════════════════════
#  Schalter (statt Haekchen)
# ════════════════════════════════════════════════════════════════════════
class Toggle(QAbstractButton):
    """An/Aus-Schalter mit kurzer Animation."""

    def __init__(self, checked: bool = False, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setChecked(checked)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self._pos = 1.0 if checked else 0.0
        self._anim = QPropertyAnimation(self, b"knob", self)
        self._anim.setDuration(120)
        self._anim.setEasingCurve(QEasingCurve.Type.OutCubic)
        self.toggled.connect(self._animate)

    def sizeHint(self) -> QSize:
        return QSize(44, 26)

    def _get_knob(self) -> float:
        return self._pos

    def _set_knob(self, v: float) -> None:
        self._pos = v
        self.update()

    knob = pyqtProperty(float, _get_knob, _set_knob)

    def _animate(self, on: bool) -> None:
        self._anim.stop()
        self._anim.setStartValue(self._pos)
        self._anim.setEndValue(1.0 if on else 0.0)
        self._anim.start()

    def setChecked(self, on: bool) -> None:  # noqa: N802 (Qt-Name)
        super().setChecked(on)
        if not self.signalsBlocked():
            self._pos = 1.0 if on else 0.0
            self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(1, 1, 42, 24)
        off, on = QColor("#3a4250"), QColor(T.ACCENT)
        c = QColor(
            int(off.red() + (on.red() - off.red()) * self._pos),
            int(off.green() + (on.green() - off.green()) * self._pos),
            int(off.blue() + (on.blue() - off.blue()) * self._pos))
        if not self.isEnabled():
            c.setAlpha(110)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(c)
        p.drawRoundedRect(r, 12, 12)
        knob = QColor("#ffffff" if self._pos > 0.5 else "#cfd4dc")
        if not self.isEnabled():
            knob.setAlpha(150)
        p.setBrush(knob)
        x = 4 + self._pos * 18
        p.drawEllipse(QRectF(x, 4, 18, 18))


class ToggleRow(QWidget):
    """Zeile: Titel + Erklaerung links, Schalter rechts. Klick auf die Zeile schaltet."""

    toggled = pyqtSignal(bool)

    def __init__(self, title: str, hint: str = "", checked: bool = False):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 8, 0, 8)
        lay.setSpacing(12)
        text = QVBoxLayout()
        text.setSpacing(1)
        self.title = label(title)
        f = self.title.font()
        f.setWeight(QFont.Weight.DemiBold)
        self.title.setFont(f)
        text.addWidget(self.title)
        self.hint = label(hint, "hint", wrap=True) if hint else None
        if self.hint:
            text.addWidget(self.hint)
        lay.addLayout(text, 1)
        self.switch = Toggle(checked)
        self.switch.toggled.connect(self.toggled.emit)
        lay.addWidget(self.switch, 0, Qt.AlignmentFlag.AlignVCenter)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def mouseReleaseEvent(self, e) -> None:
        if self.isEnabled() and e.button() == Qt.MouseButton.LeftButton:
            self.switch.toggle()

    def isChecked(self) -> bool:  # noqa: N802
        return self.switch.isChecked()

    def setChecked(self, on: bool) -> None:  # noqa: N802
        self.switch.setChecked(on)

    def setHint(self, text: str) -> None:  # noqa: N802
        if self.hint:
            self.hint.setText(text)


class RowList(QFrame):
    """Karte mit untereinander liegenden Zeilen und Trennlinien."""

    def __init__(self):
        super().__init__()
        self.setObjectName("card")
        self._lay = QVBoxLayout(self)
        self._lay.setContentsMargins(16, 4, 16, 4)
        self._lay.setSpacing(0)
        self._n = 0

    def add(self, w: QWidget) -> QWidget:
        if self._n:
            self._lay.addWidget(hline())
        self._lay.addWidget(w)
        self._n += 1
        return w


# ════════════════════════════════════════════════════════════════════════
#  Auswahl-Karten (statt Radiobuttons / Dropdowns)
# ════════════════════════════════════════════════════════════════════════
class ChoiceCard(QPushButton):
    """Ankreuzbare Karte mit optionalem Symbol, Titel und Kurztext."""

    def __init__(self, value: str, title: str, text: str = "", icon: str = ""):
        super().__init__()
        self.value = value
        self.setObjectName("choice")
        self.setCheckable(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(14, 11, 14, 11)
        lay.setSpacing(12)
        if icon:
            ic = QLabel(icon)
            ic.setAlignment(Qt.AlignmentFlag.AlignCenter)
            ic.setFixedSize(40, 40)
            ic.setStyleSheet(f"background:{T.BG}; border-radius:10px; font-size:18pt;")
            lay.addWidget(ic)
        col = QVBoxLayout()
        col.setSpacing(2)
        self.title_lbl = label(title)
        f = self.title_lbl.font()
        f.setWeight(QFont.Weight.DemiBold)
        self.title_lbl.setFont(f)
        col.addWidget(self.title_lbl)
        self.text_lbl = label(text, "hint", wrap=True)
        if text:
            col.addWidget(self.text_lbl)
        lay.addLayout(col, 1)
        for w in self.findChildren(QLabel):
            w.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)

    def sizeHint(self) -> QSize:
        return self.layout().sizeHint()

    def minimumSizeHint(self) -> QSize:
        return self.layout().minimumSize()

    def hasHeightForWidth(self) -> bool:  # noqa: N802
        return True

    def heightForWidth(self, w: int) -> int:  # noqa: N802
        return self.layout().totalHeightForWidth(w)


class ChoiceGroup(QWidget):
    """Mehrere ChoiceCards nebeneinander; genau eine ist gewaehlt."""

    changed = pyqtSignal(str)

    def __init__(self, choices, value: str, icons: dict | None = None, columns: int = 0):
        super().__init__()
        self._group = QButtonGroup(self)
        self._group.setExclusive(True)
        lay = QGridLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)
        cols = columns or len(choices)
        self.cards: dict[str, ChoiceCard] = {}
        for i, (v, title, text) in enumerate(choices):
            c = ChoiceCard(v, title, text, (icons or {}).get(v, ""))
            self._group.addButton(c)
            lay.addWidget(c, i // cols, i % cols)
            self.cards[v] = c
        for i in range(cols):
            lay.setColumnStretch(i, 1)
        self.set_value(value)
        self._group.buttonClicked.connect(lambda b: self.changed.emit(b.value))

    def value(self) -> str:
        b = self._group.checkedButton()
        return b.value if b else ""

    def set_value(self, v: str) -> None:
        c = self.cards.get(v)
        if c is not None:
            c.setChecked(True)


# ════════════════════════════════════════════════════════════════════════
#  Umgebungs-Kachel mit Vorschaubild
# ════════════════════════════════════════════════════════════════════════
def rounded_top(pix: QPixmap, w: int, h: int, radius: int = 10) -> QPixmap:
    """Bild auf w x h zuschneiden (cover) und oben abrunden."""
    out = QPixmap(w, h)
    out.fill(Qt.GlobalColor.transparent)
    p = QPainter(out)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
    path = QPainterPath()
    path.addRoundedRect(QRectF(0, 0, w, h + radius), radius, radius)
    p.setClipPath(path)
    if pix is not None and not pix.isNull():
        sc = pix.scaled(w, h, Qt.AspectRatioMode.KeepAspectRatioByExpanding,
                        Qt.TransformationMode.SmoothTransformation)
        p.drawPixmap(int((w - sc.width()) / 2), int((h - sc.height()) / 2), sc)
    else:
        g = QLinearGradient(0, 0, w, h)
        g.setColorAt(0, QColor("#2a3342"))
        g.setColorAt(1, QColor("#1a1f28"))
        p.fillRect(0, 0, w, h, QBrush(g))
    p.end()
    return out


class SceneCard(QPushButton):
    """Kachel: Vorschaubild oben, Name + Kurzbeschreibung darunter, Haken wenn gewaehlt."""

    IMG_H = 132

    def __init__(self, value: str, title: str, text: str, thumb: Path | None):
        super().__init__()
        self.value = value
        self.setObjectName("choice")
        self.setCheckable(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._pix: QPixmap | None = None
        self._title = title
        lay = QVBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 10)
        lay.setSpacing(2)
        self._img = QLabel()
        self._img.setFixedHeight(self.IMG_H)
        self._img.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self._img)
        t = label(title)
        f = t.font()
        f.setWeight(QFont.Weight.DemiBold)
        t.setFont(f)
        t.setContentsMargins(10, 6, 10, 0)
        lay.addWidget(t)
        s = label(text, "hint")
        s.setContentsMargins(10, 0, 10, 0)
        lay.addWidget(s)
        for w in self.findChildren(QLabel):
            w.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.set_thumb(thumb)

    def set_thumb(self, thumb: Path | None) -> None:
        pix = QPixmap(str(thumb)) if thumb and Path(thumb).is_file() else None
        self._pix = pix if pix is not None and not pix.isNull() else None
        self._render_img()

    def _render_img(self) -> None:
        w = max(80, self.width() - 4)
        img = rounded_top(self._pix, w, self.IMG_H)
        if self._pix is None:
            # Platzhalter: Anfangsbuchstabe gross in der Mitte
            p = QPainter(img)
            p.setPen(QColor("#56627a"))
            f = QFont()
            f.setPointSize(26)
            f.setWeight(QFont.Weight.Bold)
            p.setFont(f)
            p.drawText(img.rect(), Qt.AlignmentFlag.AlignCenter,
                       (self._title[:1] or "?").upper())
            p.end()
        self._img.setPixmap(img)

    def resizeEvent(self, e) -> None:
        super().resizeEvent(e)
        self._render_img()

    def sizeHint(self) -> QSize:
        return QSize(240, self.IMG_H + 62)

    def paintEvent(self, e) -> None:
        super().paintEvent(e)
        if self.isChecked():
            p = QPainter(self)
            p.setRenderHint(QPainter.RenderHint.Antialiasing)
            r = QRectF(self.width() - 36, 12, 24, 24)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(T.ACCENT))
            p.drawEllipse(r)
            p.setPen(QPen(QColor("white"), 2.4, Qt.PenStyle.SolidLine,
                          Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            x, y = r.x(), r.y()
            path = QPainterPath()
            path.moveTo(x + 7, y + 12.5)
            path.lineTo(x + 10.5, y + 16)
            path.lineTo(x + 17, y + 8.5)
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPath(path)


class AddCard(QPushButton):
    """Gestrichelte Kachel 'Neue Umgebung'."""

    def __init__(self, text: str):
        super().__init__()
        self._text = text
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setStyleSheet("QPushButton{background:transparent;border:none;}")

    def sizeHint(self) -> QSize:
        return QSize(240, SceneCard.IMG_H + 62)

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        hover = self.underMouse()
        pen = QPen(QColor(T.ACCENT if hover else "#3a4250"), 2, Qt.PenStyle.DashLine)
        p.setPen(pen)
        p.drawRoundedRect(QRectF(self.rect()).adjusted(1, 1, -1, -1), 12, 12)
        p.setPen(QColor(T.FG if hover else T.MUTED))
        f = QFont(self.font())
        f.setPointSize(20)
        p.setFont(f)
        r = self.rect().adjusted(0, -20, 0, -20)
        p.drawText(r, Qt.AlignmentFlag.AlignCenter, "+")
        f.setPointSize(10)
        p.setFont(f)
        p.drawText(self.rect().adjusted(0, 26, 0, 26), Qt.AlignmentFlag.AlignCenter, self._text)

    def enterEvent(self, e) -> None:
        self.update()
        super().enterEvent(e)

    def leaveEvent(self, e) -> None:
        self.update()
        super().leaveEvent(e)


class TileGrid(QWidget):
    """Raster, das die Spaltenzahl an die Breite anpasst (min. Kachelbreite)."""

    def __init__(self, min_w: int = 230):
        super().__init__()
        self._min_w = min_w
        self._items: list[QWidget] = []
        self._lay = QGridLayout(self)
        self._lay.setContentsMargins(0, 0, 0, 0)
        self._lay.setSpacing(14)
        self._cols = 0

    def set_items(self, items: list[QWidget]) -> None:
        for w in self._items:
            self._lay.removeWidget(w)
            w.setParent(None)
            w.deleteLater()
        self._items = list(items)
        self._cols = 0
        self._relayout(self.width() or 900)

    def _relayout(self, width: int) -> None:
        cols = max(1, min(4, (width + 14) // (self._min_w + 14)))
        if cols == self._cols:
            return
        self._cols = cols
        for w in self._items:
            self._lay.removeWidget(w)
        for i, w in enumerate(self._items):
            self._lay.addWidget(w, i // cols, i % cols)
        for c in range(4):
            self._lay.setColumnStretch(c, 1 if c < cols else 0)

    def resizeEvent(self, e) -> None:
        super().resizeEvent(e)
        self._relayout(e.size().width())


# ════════════════════════════════════════════════════════════════════════
#  Aufklappbarer Bereich ("Erweitert")
# ════════════════════════════════════════════════════════════════════════
class Collapsible(QWidget):
    def __init__(self, title: str):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 8, 0, 0)
        lay.setSpacing(8)
        self._btn = QToolButton()
        self._btn.setText(f"  {title}")
        self._btn.setCheckable(True)
        self._btn.setArrowType(Qt.ArrowType.RightArrow)
        self._btn.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self._btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._btn.setStyleSheet(f"QToolButton{{border:none;color:{T.MUTED};"
                                f"font-weight:600;background:transparent;}}"
                                f"QToolButton:hover{{color:{T.FG};}}")
        self._btn.toggled.connect(self._toggle)
        lay.addWidget(self._btn)
        self.body = RowList()
        self.body.setVisible(False)
        lay.addWidget(self.body)

    def _toggle(self, on: bool) -> None:
        self._btn.setArrowType(Qt.ArrowType.DownArrow if on else Qt.ArrowType.RightArrow)
        self.body.setVisible(on)

    def is_open(self) -> bool:
        return self._btn.isChecked()

    def set_open(self, on: bool) -> None:
        self._btn.setChecked(on)


class FieldRow(QWidget):
    """Beschriftetes Eingabefeld in einer RowList-Zeile."""

    def __init__(self, title: str, field: QWidget, hint: str = "", unit: str = ""):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 8, 0, 8)
        lay.setSpacing(12)
        col = QVBoxLayout()
        col.setSpacing(1)
        t = label(title)
        f = t.font()
        f.setWeight(QFont.Weight.DemiBold)
        t.setFont(f)
        col.addWidget(t)
        if hint:
            col.addWidget(label(hint, "hint", wrap=True))
        lay.addLayout(col, 1)
        lay.addWidget(field)
        if unit:
            u = label(unit, "muted")
            u.setFixedWidth(42)
            lay.addWidget(u)
        self.field = field

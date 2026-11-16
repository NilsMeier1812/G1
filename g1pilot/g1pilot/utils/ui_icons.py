"""Einheitliche Bedien-Icons fuer alle Qt-Oberflaechen (Startmenue, Demo-GUI,
Hand-Panel, Ablauf-Editor, Hand-Werkzeuge) -- statt Emojis.

Quelle: Lucide (ISC-Lizenz, resource/icons/ui/LICENSE), Strich-Icons mit
stroke="currentColor". Hier wird die Farbe eingesetzt und per Qt-SVG-Plugin
(imageformats/libqsvg) gerastert -- kein QtSvg-Modul noetig. Fehlt ein Icon
oder das Plugin, gibt es ein leeres QIcon (Knopf zeigt dann nur Text).

    from g1pilot.utils.ui_icons import icon
    btn.setIcon(icon("smartphone", "#e8eaed"))
"""
from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path

UI_SUBDIR = Path("resource") / "icons" / "ui"


@lru_cache(maxsize=1)
def icon_dir() -> Path | None:
    """resource/icons/ui im Quellbaum bzw. im installierten share-Verzeichnis."""
    candidates = []
    env_dir = os.environ.get("G1_UI_ICON_DIR", "").strip()
    if env_dir:
        candidates.append(Path(env_dir))
    # resolve(): bei colcon --symlink-install zeigt __file__ in build/, der
    # Symlink fuehrt zurueck in den Quellbaum (.../g1pilot/g1pilot/utils/).
    candidates.append(Path(__file__).resolve().parents[2] / UI_SUBDIR)
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory("g1pilot")) / UI_SUBDIR)
    except Exception:
        pass
    for d in candidates:
        if (d / "play.svg").is_file():
            return d
    return None


def names() -> list[str]:
    d = icon_dir()
    return sorted(p.stem for p in d.glob("*.svg")) if d else []


@lru_cache(maxsize=256)
def svg_bytes(name: str, color: str = "#e8eaed", stroke: float = 2.0) -> bytes:
    d = icon_dir()
    if d is None or not (d / f"{name}.svg").is_file():
        return b""
    text = (d / f"{name}.svg").read_text(encoding="utf-8")
    text = text.replace("currentColor", color)
    if stroke != 2.0:
        text = text.replace('stroke-width="2"', f'stroke-width="{stroke:g}"')
    return text.encode("utf-8")


def pixmap(name: str, color: str = "#e8eaed", size: int = 20, stroke: float = 2.0):
    """QPixmap in Geraetepixeln (HiDPI-scharf). Leer, wenn nicht ladbar."""
    from PyQt6.QtCore import QBuffer, QByteArray, QIODevice, QSize
    from PyQt6.QtGui import QGuiApplication, QImageReader, QPixmap

    data = svg_bytes(name, color, stroke)
    if not data:
        return QPixmap()
    app = QGuiApplication.instance()
    dpr = app.devicePixelRatio() if app is not None else 1.0
    px = max(1, int(round(size * max(1.0, dpr))))
    buf = QBuffer()
    buf.setData(QByteArray(data))
    buf.open(QIODevice.OpenModeFlag.ReadOnly)
    reader = QImageReader(buf, b"svg")
    reader.setScaledSize(QSize(px, px))
    img = reader.read()
    if img.isNull():
        return QPixmap()
    pm = QPixmap.fromImage(img)
    pm.setDevicePixelRatio(max(1.0, dpr))
    return pm


def icon(name: str, color: str = "#e8eaed", size: int = 24, disabled_color: str = "#5d6470"):
    """QIcon mit normalem und ausgegrautem Zustand."""
    from PyQt6.QtGui import QIcon

    ic = QIcon()
    pm = pixmap(name, color, size)
    if pm.isNull():
        return ic
    ic.addPixmap(pm, QIcon.Mode.Normal)
    ic.addPixmap(pm, QIcon.Mode.Active)
    dpm = pixmap(name, disabled_color, size)
    if not dpm.isNull():
        ic.addPixmap(dpm, QIcon.Mode.Disabled)
    return ic


def set_icon(button, name: str, color: str = "#e8eaed", size: int = 18) -> None:
    """Icon an einen QPushButton/QToolButton haengen (Groesse in logischen px)."""
    from PyQt6.QtCore import QSize

    button.setIcon(icon(name, color, size))
    button.setIconSize(QSize(size, size))

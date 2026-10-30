#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fenster-Icons: jedes G1-Fenster hat ein eigenes Icon statt Zahnrad.

Prueft die Icon-Dateien, die Qt-Hilfsfunktion (Icon + WM_CLASS) und den
PNG -> _NET_WM_ICON-Weg fuer den MuJoCo-Viewer. Ohne Roboter, Qt offscreen.
"""
import importlib.util
import os
import struct
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
PKG_ROOT = HERE.parent                               # .../g1pilot
ICON_DIR = PKG_ROOT / "resource" / "icons"
SIM_PY = PKG_ROOT.parent / "unitree_mujoco" / "simulate_python"
ICONS = ["g1_start", "g1_demo_sim", "g1_demo_real", "g1_mujoco"]


def _png_size(path):
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    return struct.unpack(">II", data[16:24])


@pytest.mark.parametrize("name", ICONS)
def test_icon_files_exist(name):
    assert (ICON_DIR / f"{name}.svg").is_file()
    assert _png_size(ICON_DIR / f"{name}.png") == (256, 256)


def test_icons_are_distinct():
    blobs = {(ICON_DIR / f"{n}.png").read_bytes() for n in ICONS}
    assert len(blobs) == len(ICONS)


def test_mujoco_icon_copy_matches_source():
    # Der mujoco-sim-Container sieht nur unitree_mujoco/ -> Kopie muss aktuell sein.
    assert (SIM_PY / "icons" / "g1_mujoco.png").read_bytes() == \
        (ICON_DIR / "g1_mujoco.png").read_bytes()


def _window_icon_module():
    spec = importlib.util.spec_from_file_location("window_icon", SIM_PY / "window_icon.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_mujoco_png_decode_and_net_wm_icon():
    wi = _window_icon_module()
    w, h, rgba = wi.load_png_rgba(wi.ICON_PATH)
    assert (w, h) == (256, 256) and len(rgba) == w * h * 4
    # Ecke transparent (abgerundet), Mitte deckend.
    assert rgba[3] == 0
    mid = ((h // 2) * w + w // 2) * 4
    assert rgba[mid + 3] == 255

    cards = wi.net_wm_icon_data(wi.ICON_PATH)
    pos, sizes = 0, []
    while pos < len(cards):
        cw, ch = cards[pos], cards[pos + 1]
        sizes.append(cw)
        pos += 2 + cw * ch
    assert pos == len(cards)
    assert sizes == [128, 64, 32]
    assert all(0 <= c < 2 ** 32 for c in cards)


def test_mujoco_prepare_env_sets_resource_name(monkeypatch):
    wi = _window_icon_module()
    monkeypatch.delenv("RESOURCE_NAME", raising=False)
    wi.prepare_env()
    assert os.environ["RESOURCE_NAME"] == "g1-mujoco"


def test_mujoco_apply_async_without_display_is_silent(monkeypatch):
    wi = _window_icon_module()
    monkeypatch.delenv("DISPLAY", raising=False)
    t = wi.apply_async(timeout=1.0, interval=0.1)
    t.join(timeout=3.0)
    assert not t.is_alive()


def test_qt_set_app_identity(monkeypatch):
    pytest.importorskip("PyQt6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt6.QtWidgets import QApplication
    from g1pilot.utils import window_style as ws

    monkeypatch.delenv("RESOURCE_NAME", raising=False)
    app = QApplication.instance() or QApplication([])
    for app_id, icon in [("g1-demo-sim", "g1_demo_sim"),
                         ("g1-demo-real", "g1_demo_real")]:
        monkeypatch.delenv("RESOURCE_NAME", raising=False)
        path = ws.set_app_identity(app, app_id, icon)
        assert path == ICON_DIR / f"{icon}.png"
        assert app.applicationName() == app_id
        assert app.desktopFileName() == app_id
        assert os.environ["RESOURCE_NAME"] == app_id
        assert not app.windowIcon().isNull()


def test_qt_missing_icon_keeps_default():
    pytest.importorskip("PyQt6")
    from g1pilot.utils import window_style as ws
    assert ws.find_icon("gibt_es_nicht") is None

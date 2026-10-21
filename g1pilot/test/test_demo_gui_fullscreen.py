#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Demo-GUI: Vollbild per Button, F11 (umschalten) und Esc (verlassen).

Offscreen, ohne Roboter. Am Roboter-Rechner fehlt oft die Titelleiste des
Fenstermanagers, darum muss es immer einen Weg IN der GUI geben.
"""
import os
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy                                       # noqa: E402
from PyQt6.QtCore import Qt                       # noqa: E402
from PyQt6.QtTest import QTest                    # noqa: E402
from PyQt6.QtWidgets import QApplication          # noqa: E402


@pytest.fixture(params=["true", "false"], ids=["sim", "real"])
def gui(request, monkeypatch):
    monkeypatch.setenv("G1_SIM_MODE", request.param)
    rclpy.init()
    app = QApplication.instance() or QApplication([])   # Referenz halten (sonst GC)
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    g.show()
    _pump(app)
    try:
        yield g, app
    finally:
        g.close()
        node.tf_listener.executor.shutdown()            # TF-Spin-Thread vor rclpy beenden
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def _pump(app, seconds=0.2):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        app.processEvents()
        time.sleep(0.01)


def _key(g, app, key):
    g.activateWindow()
    QTest.keyClick(g, key)
    _pump(app)


def test_button_visible_in_window(gui):
    g, app = gui
    assert not g.isFullScreen()
    assert g.btn_fullscreen.isVisible()
    assert "Vollbild" in g.btn_fullscreen.text()
    assert "beenden" not in g.btn_fullscreen.text()


def test_button_toggles_fullscreen(gui):
    g, app = gui
    g.btn_fullscreen.click()
    _pump(app)
    assert g.isFullScreen()
    assert g.btn_fullscreen.isVisible()               # auch im Vollbild sichtbar
    assert "beenden" in g.btn_fullscreen.text()
    g.btn_fullscreen.click()
    _pump(app)
    assert not g.isFullScreen()
    assert "beenden" not in g.btn_fullscreen.text()


def test_f11_toggles_and_esc_leaves(gui):
    g, app = gui
    _key(g, app, Qt.Key.Key_F11)
    assert g.isFullScreen()
    _key(g, app, Qt.Key.Key_Escape)
    assert not g.isFullScreen()
    _key(g, app, Qt.Key.Key_Escape)                   # Esc im Fenster: nichts passiert
    assert not g.isFullScreen() and g.isVisible()
    _key(g, app, Qt.Key.Key_F11)
    _key(g, app, Qt.Key.Key_F11)
    assert not g.isFullScreen()


def test_fullscreen_keeps_estop_and_publishes_nothing(gui):
    g, app = gui
    node = g.node
    sent = []
    node.publish_bool = lambda pub, v: sent.append((pub.topic_name, bool(v)))
    g.set_fullscreen(True)
    _pump(app)
    g.set_fullscreen(True)                            # doppelt: kein Effekt
    assert g.isFullScreen()
    assert g.btn_estop.isVisible()
    assert sent == []                                 # reine Fenster-Sache
    g.set_fullscreen(False)
    _pump(app)
    assert not g.isFullScreen()


def test_external_state_change_updates_label(gui):
    g, app = gui
    g.showFullScreen()                                # z. B. vom Fenstermanager
    _pump(app)
    assert "beenden" in g.btn_fullscreen.text()
    g.showNormal()
    _pump(app)
    assert "beenden" not in g.btn_fullscreen.text()


def test_window_fits_small_screen(gui):
    """Real-Test 2026-10-21: Mindesthoehe lag bei ~1160 px (> 1080er-Schirm) ->
    der Fenstermanager bot kein Maximieren an, Vollbild blieb zu gross.
    Jetzt muss das Fenster auch auf einen 1366x768-Laptop passen."""
    g, app = gui
    from g1pilot.teleoperation import demo_gui as dg
    g.started = True
    for mode, view in ((None, None), (dg.WALK, None), (dg.MANIP, "arms"),
                       (dg.MANIP, "hands")):
        g.mode = mode
        g._refresh()
        if view:
            g.manip_panel.show_view(view)
        _pump(app, 0.05)
        hint = g.minimumSizeHint()
        assert hint.height() <= 700, (mode, view, hint)
        assert hint.width() <= 1366, (mode, view, hint)


def test_fullscreen_button_in_mode_header(gui):
    """Vollbild-Knopf oben rechts in der Zeile »1 · MODUS«, ueber den Kacheln."""
    g, app = gui
    from g1pilot.teleoperation import demo_gui as dg
    btn = g.btn_fullscreen.mapTo(g, g.btn_fullscreen.rect().bottomLeft())
    tile = g.tiles[dg.WALK].mapTo(g, g.tiles[dg.WALK].rect().topLeft())
    assert btn.y() <= tile.y()
    assert btn.x() > g.width() // 2


def test_start_bar_holds_start_and_onboard(gui):
    """Real: Start | Damp | Preparation | Walk | loco_client-Rueckmeldung in EINER
    Zeile; Start ist bis zum Klick frei und danach grau »Gestartet ✓«."""
    g, app = gui
    if g.sim_mode:
        assert g.start_bar.isHidden()
        return
    for w in [g.btn_start, g.info_onboard] + list(g.start_btns.values()):
        assert w.parentWidget() is g.start_bar
    assert g.btn_start.isEnabled() and g.btn_start.text() == "Start"
    sent = []
    g.node.publish_bool = lambda pub, v: sent.append((pub.topic_name, bool(v)))
    g.btn_start.click()
    assert sent == [("/g1pilot/start", True)] and g.started
    assert not g.btn_start.isEnabled() and "Gestartet" in g.btn_start.text()
    g._start_robot()                                  # schon gestartet: nichts
    assert len(sent) == 1

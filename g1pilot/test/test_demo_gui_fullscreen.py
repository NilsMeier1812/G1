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

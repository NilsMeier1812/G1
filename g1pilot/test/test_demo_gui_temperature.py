#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Demo-GUI: Temperaturanzeige des echten G1 (offscreen, ohne Roboter)."""
import os
import time
from types import SimpleNamespace as NS

import pytest

from g1pilot.utils import g1_temperature as g1t

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import rclpy                                       # noqa: E402
from PyQt6.QtWidgets import QApplication          # noqa: E402


def _motors(t1=30, t2=30, n=35):
    return [NS(temperature=[t1, t2], vol=48.0) for _ in range(n)]


def _gui(monkeypatch, sim):
    monkeypatch.setenv("G1_SIM_MODE", "true" if sim else "false")
    rclpy.init()
    app = QApplication.instance() or QApplication([])
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    return app, node, dg.DemoGUI(node)


def _close(node, g):
    g.close()
    node.tf_listener.executor.shutdown()
    node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
    node.tf_node.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def test_gui_real_shows_temperatures(monkeypatch):
    app, node, g = _gui(monkeypatch, sim=False)
    try:
        p = g.temp_panel
        assert not p.isHidden()
        assert "keine Daten" in p.tiles["max"].text()          # noch nichts empfangen
        from std_msgs.msg import String
        node._temperatures(String(data=g1t.build_payload(_motors(40, 78), 30)))
        p.refresh()
        assert "78 °C" in p.tiles["max"].text() and "Hüfte Nick links" in p.tiles["max"].text()
        assert g1t.LEVEL_COLOR["crit"] in p.tiles["max"].styleSheet()
        node._temperatures(String(data="kaputt"))              # Muell aendert nichts
        assert node.temps["t"][0] == [40.0, 78.0]
        node.temps_t = time.monotonic() - 10                   # veraltet
        p.refresh()
        assert "keine Daten" in p.tiles["max"].text()
    finally:
        _close(node, g)


def test_gui_sim_hides_temperatures(monkeypatch):
    app, node, g = _gui(monkeypatch, sim=True)
    try:
        assert g.temp_panel.isHidden() and not g.temp_panel.timer.isActive()
    finally:
        _close(node, g)


def test_gui_info_line_onboard_and_ps4(monkeypatch):
    """Info-Zeile: Onboard-Modus (nur real) + PS4-Auswahl (Review 2026-10-16)."""
    from std_msgs.msg import String
    app, node, g = _gui(monkeypatch, sim=False)
    try:
        g._refresh_info()
        assert not g.info_onboard.isHidden() and "keine Daten" in g.info_onboard.text()
        assert g.info_ps4.isHidden(), "ohne PS4-Arm-Modus keine Anzeige"
        node._latched_json("onboard", String(
            data='{"fsm": 500, "name": "Walk", "walk": false, "estop": false}'))
        node._latched_json("ps4", String(
            data='{"selection": "right", "mode": "move", "speed": "Langsam", '
                 '"connected": true, "estop": false}'))
        g._refresh_info()
        assert g.info_onboard.text() == "G1: Walk (FSM 500) · bereit"
        assert not g.info_ps4.isHidden() and "Hand RECHTS" in g.info_ps4.text()
        g._ps4_help()                                  # darf nicht werfen
    finally:
        _close(node, g)


def test_gui_sim_hides_onboard_line(monkeypatch):
    app, node, g = _gui(monkeypatch, sim=True)
    try:
        g._refresh_info()
        assert g.info_onboard.isHidden()
    finally:
        _close(node, g)

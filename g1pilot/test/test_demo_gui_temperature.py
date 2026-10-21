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
    """Startleiste: Onboard-Modus (nur real); Controller nur als Belegungs-Knopf."""
    from std_msgs.msg import String
    app, node, g = _gui(monkeypatch, sim=False)
    try:
        g._refresh_info()
        assert not g.info_onboard.isHidden() and "keine Daten" in g.info_onboard.text()
        assert g.btn_pad_help.isHidden(), "ohne Controller kein Belegungs-Knopf"
        node._latched_json("onboard", String(
            data='{"fsm": 501, "name": "Walk mit Taille", "walk": false, '
                 '"estop": false, "walk_fsm": 501}'))
        node._latched_json("ps4", String(
            data='{"selection": "right", "mode": "move", "speed": "Langsam", '
                 '"connected": true, "estop": false}'))
        g._refresh_info()
        assert g.info_onboard.text() == "G1: Walk mit Taille (FSM 501) · bereit"
        assert not g.btn_pad_help.isHidden()
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


def _onboard(node, fsm, estop=False, note=""):
    import json
    from std_msgs.msg import String
    node._latched_json("onboard", String(data=json.dumps(
        {"fsm": fsm, "name": "x", "walk": False, "estop": estop, "note": note})))


def test_gui_start_bar_crane_sequence(monkeypatch):
    """Startleiste: Damp -> Preparation -> Walk (Walk erst nach zweitem Klick)."""
    app, node, g = _gui(monkeypatch, sim=False)
    sent = []
    node.pub_loco_request.publish = lambda m: sent.append(m.data)
    try:
        assert not g.start_bar.isHidden()
        g._refresh_info()
        assert not any(b.isEnabled() for b in g.start_btns.values()), "ohne Daten gesperrt"
        _onboard(node, 0)
        g._refresh_info()
        g._start_step("walk")
        g._start_step("damp")
        assert sent == ["damp"], "Walk aus Zero Torque gesperrt, Damp ohne Rueckfrage"
        _onboard(node, 1)
        g._refresh_info()
        g._start_step("preparation")
        assert sent[-1] == "preparation"
        _onboard(node, 4)
        g._refresh_info()
        g._start_step("walk")
        assert sent[-1] == "preparation" and "Kran" in g.start_btns["walk"].text()
        g._start_step("walk")
        assert sent[-1] == "walk" and g.start_btns["walk"].text() == "Walk"
    finally:
        _close(node, g)


def test_gui_start_bar_damp_from_walk_needs_confirm(monkeypatch):
    app, node, g = _gui(monkeypatch, sim=False)
    sent = []
    node.pub_loco_request.publish = lambda m: sent.append(m.data)
    try:
        _onboard(node, 500)
        g._refresh_info()
        g._start_step("damp")
        assert sent == []
        g._start_step("damp")
        assert sent == ["damp"]
    finally:
        _close(node, g)


def test_gui_walk_tile_refused_unless_onboard_walk(monkeypatch):
    """REAL: GEHEN stellt nicht mehr selbst auf Walk um; GREIFEN in Preparation
    schaltet nur die Arme ein (kein start_balancing)."""
    from g1pilot.teleoperation import demo_gui as dg
    app, node, g = _gui(monkeypatch, sim=False)
    pubs = []
    node.publish_bool = lambda pub, v: pubs.append((pub, v))
    try:
        g.started = True
        _onboard(node, 4)
        g.request_mode(dg.WALK)
        assert g.pending is None and g.mode is None
        assert (node.pub_start_walking, True) not in pubs
        g.request_mode(dg.MANIP)
        assert g.mode == dg.MANIP
        assert (node.pub_arms_enabled, True) in pubs
        assert (node.pub_start_balancing, True) not in pubs
        _onboard(node, 500)
        g.request_mode(dg.WALK)
        assert g.pending == dg.WALK and (node.pub_start_walking, True) in pubs
    finally:
        _close(node, g)


def test_gui_start_bar_hidden_in_sim(monkeypatch):
    app, node, g = _gui(monkeypatch, sim=True)
    try:
        assert g.start_bar.isHidden()
    finally:
        _close(node, g)

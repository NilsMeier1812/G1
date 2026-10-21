#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Demo-GUI (real): Bodenkontakt-Vorabpruefung fuer den Walk-Modus 501.

Nils 2026-10-20: Beim Umschalten auf Walk prueft der loco_client, ob die
Fuesse Last haben (FSM-Mode 0). Das soll schon VOR dem Klick sichtbar sein:
Startleiste »Walk«, Info-Zeile und Kachel GEHEN zeigen, ob es geht. Offscreen,
ohne Roboter: onboard_state wird direkt eingespielt.
"""
import os
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy                                       # noqa: E402
from PyQt6.QtWidgets import QApplication          # noqa: E402


@pytest.fixture
def gui(monkeypatch):
    monkeypatch.setenv("G1_SIM_MODE", "false")
    rclpy.init()
    app = QApplication.instance() or QApplication([])   # noqa: F841
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    requests = []
    node.pub_loco_request.publish = lambda m: requests.append(m.data)
    try:
        yield dg, node, g, requests
    finally:
        g.close()
        node.tf_listener.executor.shutdown()
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def _onboard(node, **st):
    node.onboard = dict({"name": "Preparation", "estop": False, "walk_fsm": 501}, **st)
    node.onboard_t = time.monotonic()


def test_no_ground_contact_shown_and_walk_blocked_before_click(gui):
    dg, node, g, requests = gui
    _onboard(node, fsm=4, fsm_mode=2)
    g._refresh_info()
    b = g.start_btns["walk"]
    assert not b.isEnabled() and b.text() == "Walk\nkein Bodenkontakt"
    assert "kein Bodenkontakt" in g.info_onboard.text()
    assert g.tiles[dg.WALK].text().endswith("gesperrt · kein Bodenkontakt")
    g._start_step("walk")
    g._start_step("walk")
    assert requests == []                         # nichts an den loco_client
    # GEHEN-Kachel: klarer Grund in der Statuszeile, nichts gesendet
    g._start_robot()
    g.request_mode(dg.WALK)
    assert g.pending is None and "kein Bodenkontakt" in g.status.text()


def test_ground_contact_ok_enables_walk_with_confirmation(gui):
    dg, node, g, requests = gui
    _onboard(node, fsm=4, fsm_mode=0)
    g._refresh_info()
    b = g.start_btns["walk"]
    assert b.isEnabled() and b.text() == "Walk\nBodenkontakt ✓"
    assert "Bodenkontakt ✓" in g.info_onboard.text()
    assert g.tiles[dg.WALK].text().endswith("erst »Walk« · Bodenkontakt ✓")
    # Rueckfrage bleibt; der 2-Hz-Refresh ueberschreibt sie nicht
    g._start_step("walk")
    assert b.text().startswith("Kran gelockert?") and requests == []
    g._refresh_info()
    assert b.text().startswith("Kran gelockert?")
    g._start_step("walk")
    assert requests == ["walk"]
    assert b.text() == "Walk\nBodenkontakt ✓"


def test_ground_lost_while_armed_disarms(gui):
    """Erster Klick bei Bodenkontakt, dann zieht der Kran an -> Rueckfrage
    verfaellt, »Walk« gesperrt; der zweite Klick sendet nichts."""
    dg, node, g, requests = gui
    _onboard(node, fsm=4, fsm_mode=0)
    g._refresh_info()
    g._start_step("walk")
    _onboard(node, fsm=4, fsm_mode=2)
    g._refresh_info()
    b = g.start_btns["walk"]
    assert g._start_armed is None and not b.isEnabled()
    assert b.text() == "Walk\nkein Bodenkontakt"
    g._start_step("walk")
    assert requests == []


def test_unknown_mode_keeps_old_behaviour(gui):
    """Alter loco_client ohne fsm_mode: »Walk« wie bisher frei, er prueft selbst."""
    dg, node, g, requests = gui
    _onboard(node, fsm=4)
    g._refresh_info()
    b = g.start_btns["walk"]
    assert b.isEnabled() and b.text() == "Walk"
    assert g.tiles[dg.WALK].text().endswith("erst »Walk« (Startleiste)")


def test_walk_tile_normal_in_walk_and_follows_state(gui):
    dg, node, g, requests = gui
    g._refresh_info()
    assert g.tiles[dg.WALK].text().endswith("gesperrt · keine Daten vom G1")
    _onboard(node, fsm=501, name="Walk mit Taille")
    g._refresh_info()
    assert g.tiles[dg.WALK].text().endswith("antippen zum Wechseln")
    # Aktiv/wechselt ueberdeckt den Hinweis nicht dauerhaft
    g._start_robot()
    g.request_mode(dg.WALK)
    assert g.tiles[dg.WALK].text().endswith("wechselt …")
    node.on_walk_ready(True)
    assert g.tiles[dg.WALK].text().endswith("● AKTIV")
    g._refresh_info()
    assert g.tiles[dg.WALK].text().endswith("● AKTIV")
    # GREIFEN-Kachel bekommt nie einen Bodenkontakt-Hinweis
    _onboard(node, fsm=4, fsm_mode=2)
    g._refresh_info()
    assert "Bodenkontakt" not in g.tiles[dg.MANIP].text()


def test_sim_has_no_ground_hint(monkeypatch):
    monkeypatch.setenv("G1_SIM_MODE", "true")
    rclpy.init()
    app = QApplication.instance() or QApplication([])   # noqa: F841
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    try:
        g.autostart.cancel("Test")
        g._refresh_info()
        assert g.tiles[dg.WALK].hint is None
    finally:
        g.close()
        node.tf_listener.executor.shutdown()
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

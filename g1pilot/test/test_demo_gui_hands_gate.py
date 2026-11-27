#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HAENDE-Seite in GREIFEN nur mit Inspire-Haenden: Sim mit Rubber oder eigenem
starren Profil (G1_HANDS) -> Knopf ausgegraut, Seite nicht erreichbar.
Real frei mit Inspire-Haenden, ausgegraut mit eigenem starren Profil."""
import os
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy                                       # noqa: E402
from PyQt6.QtWidgets import QApplication          # noqa: E402

from g1pilot.teleoperation import demo_gui as dg  # noqa: E402


def test_hands_controllable_rule():
    f = dg.hands_controllable
    assert f({"G1_SIM_MODE": "true", "G1_HANDS": "inspire"})
    assert not f({"G1_SIM_MODE": "true", "G1_HANDS": "rubber"})
    assert not f({"G1_SIM_MODE": "true", "G1_HANDS": "haken"})
    assert f({"G1_SIM_MODE": "true", "G1_INSPIRE_HANDS": "1"})       # altes Flag
    assert not f({"G1_SIM_MODE": "true"})                             # Default rubber
    assert f({"G1_SIM_MODE": "false"})                                # real: Inspire
    assert f({"G1_SIM_MODE": "false", "G1_HANDS": "rubber"})
    assert f({"G1_SIM_MODE": "false", "G1_HANDS": "inspire"})
    assert not f({"G1_SIM_MODE": "false", "G1_HANDS": "haken"})    # real mit Haken


@pytest.fixture
def make_gui(monkeypatch):
    made = []

    def make(sim, hands):
        monkeypatch.setenv("G1_PHONE", "0")
        monkeypatch.setenv("G1_SIM_MODE", sim)
        monkeypatch.setenv("G1_HANDS", hands)
        rclpy.init()
        app = QApplication.instance() or QApplication([])
        node = dg.DemoNode()
        g = dg.DemoGUI(node)
        g.show()
        app.processEvents()
        made.append((g, node, app))         # app-Referenz halten (sonst GC)
        return g
    yield make
    for g, node, _app in made:
        g.close()
        node.tf_listener.executor.shutdown()
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        time.sleep(0.05)


@pytest.mark.parametrize("sim,hands,ok", [("true", "rubber", False), ("true", "haken", False),
                                          ("true", "inspire", True), ("false", "", True)])
def test_hands_button_follows_profile(make_gui, sim, hands, ok):
    g = make_gui(sim, hands)
    mp = g.manip_panel
    b = mp.view_btns["hands"]
    assert b.isEnabled() is ok
    mp.show_view("hands")
    page = mp.views.currentWidget()
    assert (page is mp.hand_panel) is ok
    assert mp.view_btns["arms"].isChecked() is (not ok)
    if not ok:
        assert "Inspire" in b.toolTip()

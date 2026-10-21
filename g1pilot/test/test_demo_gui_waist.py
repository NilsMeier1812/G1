#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Demo-GUI: Hueftdrehung -- Anzeige, Knopf »Hüfte Mitte«, Jog im Oberkoerper-
Bezug. Offscreen, ohne Roboter (Zustand /g1pilot/waist/state von Hand)."""
import math
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
    monkeypatch.setenv("G1_SIM_MODE", "true")
    rclpy.init()
    app = QApplication.instance() or QApplication([])
    assert app is not None
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    g.jog_timer.stop()
    try:
        yield dg, g, node
    finally:
        g.close()
        node.tf_listener.executor.shutdown()
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_waist_label_and_center_button(gui):
    dg, g, node = gui
    mp = g.manip_panel
    g._refresh_info()
    assert mp.waist_label.text() == "Hüfte: keine Daten" and not mp.btn_waist_center.isEnabled()
    node.waist, node.waist_t = {"owner": True, "meas_offset_deg": 12.4, "limit_deg": 30.0,
                                "yaw_rad": 0.2}, time.monotonic()
    g._refresh_info()
    assert mp.waist_label.text().startswith("Hüfte 12° links")
    assert mp.btn_waist_center.isEnabled()
    sent = []
    node.publish_bool = lambda pub, v: sent.append((pub, v))
    mp.btn_waist_center.click()
    assert sent == [(node.pub_waist_center, True)]
    node.waist = {"owner": True, "meas_offset_deg": 0.2, "limit_deg": 30.0}
    g._refresh_info()
    assert mp.waist_label.text() == "Hüfte Mitte (max. ±30°)"
    assert not mp.btn_waist_center.isEnabled()


def test_jog_forward_follows_torso_yaw(gui):
    dg, g, node = gui
    mp = g.manip_panel
    sent = []
    node.hand_pose = lambda side: ([0.35, -0.23, 0.11], (0.0, 0.0, 0.0, 1.0))
    node.jog_pose = lambda side, new: (None, ([0.35, -0.23, 0.11], (0.0, 0.0, 0.0, 1.0)))
    node.publish_hand_goal = lambda side, p, q: sent.append((side, list(p)))
    g.mode, g.pending = dg.MANIP, None
    mp.views.setCurrentWidget(mp.arms_page)
    mp.jog.twist = lambda: ((0.15, 0.0, 0.0), (0.0, 0.0, 0.0))   # "vor"
    mp.jog.sides = lambda: ("right",)
    node.waist, node.waist_t = {"owner": True, "yaw_rad": math.radians(90)}, time.monotonic()
    g._jog_t = time.monotonic() - 0.05
    g._jog_tick()
    assert sent, "Jog-Ziel erwartet"
    _, p = sent[-1]
    # Huefte 90 deg nach links: "vor" (Brust) zeigt im Becken-Frame nach +y
    assert p[1] > -0.23 + 1e-4 and abs(p[0] - 0.35) < 1e-6

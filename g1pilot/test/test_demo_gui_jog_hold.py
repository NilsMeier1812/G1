#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Demo-GUI »Hand bewegen«: Loslassen setzt ein Halte-Ziel und wiederholt es.

Offscreen, ohne Roboter: hand_pose/publish_hand_goal des Nodes werden
abgefangen. Prueft das Zusammenspiel aus _jog_stop (hold_pose) und
_jog_tick (HoldStreamer) -- dieselbe Logik wie am PS4-Controller.
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
    monkeypatch.setenv("G1_SIM_MODE", "true")
    rclpy.init()
    app = QApplication.instance() or QApplication([])   # Referenz halten (sonst GC)
    assert app is not None
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    g.jog_timer.stop()                              # Takt per Hand
    sent = []
    hand = {"right": ([0.35, -0.23, 0.11], (0.0, 0.0, 0.0, 1.0))}
    node.hand_pose = lambda side: hand.get(side)
    node.publish_hand_goal = lambda side, p, q: sent.append((side, list(p), tuple(q)))
    try:
        yield g, sent
    finally:
        g.close()
        node.tf_listener.executor.shutdown()            # TF-Spin-Thread vor rclpy beenden
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_jog_stop_holds_on_line_and_repeats(gui):
    g, sent = gui
    from g1pilot.teleoperation.hand_jog import JOG_HOLD_S
    # laufendes Ziel 5 cm vor der Hand (+x), Hand 3 cm quer abgedriftet
    g._jog_targets = {"right": [[0.40, -0.20, 0.10], (0.0, 0.0, 0.0, 1.0)]}
    g._jog_twist = ((0.15, 0.0, 0.0), (0.0, 0.0, 0.0))
    g._jog_stop()
    assert g._jog_targets == {}
    assert len(sent) == 1
    side, p, _ = sent[0]
    assert side == "right" and p == pytest.approx([0.35, -0.20, 0.10])   # x Hand, y/z Linie
    # Folgetakte ohne Auslenkung: dasselbe Halte-Ziel wird wiederholt ...
    for _ in range(3):
        g._jog_tick()
    assert len(sent) == 4 and all(s[1] == p for s in sent[1:])
    # ... bis JOG_HOLD_S abgelaufen ist
    g._jog_holds._held["right"] = (g._jog_holds._held["right"][0], time.monotonic() - 0.01)
    n = len(sent)
    g._jog_tick()
    assert len(sent) == n and JOG_HOLD_S > 0.5


def test_home_clears_hold(gui):
    g, sent = gui
    g._jog_targets = {"right": [[0.40, -0.20, 0.10], (0.0, 0.0, 0.0, 1.0)]}
    g._jog_twist = ((0.15, 0.0, 0.0), (0.0, 0.0, 0.0))
    g._jog_stop()
    g._home()                                       # geplante Fahrt: kein Halte-Ziel mehr
    n = len(sent)
    g._jog_tick()
    assert len(sent) == n

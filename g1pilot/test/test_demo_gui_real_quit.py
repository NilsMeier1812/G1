#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Demo-GUI »Real beenden« (echter Roboter): erst Laufen stoppen und die Arme
abgeben, erst DANN den Stack per Host-Watcher stoppen.

Offscreen, ohne Roboter: publish_bool des Nodes wird mitgeschrieben, der
Arm-Regler-Zustand (/g1pilot/arms/state) von Hand gesetzt, die Trigger-Datei
liegt in tmp_path.
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
def gui(monkeypatch, tmp_path):
    monkeypatch.setenv("G1_SIM_MODE", "false")
    rclpy.init()
    app = QApplication.instance() or QApplication([])   # Referenz halten (sonst GC)
    from g1pilot.teleoperation import demo_gui as dg
    trigger = tmp_path / ".real_shutdown_request"
    monkeypatch.setattr(dg, "REAL_SHUTDOWN_TRIGGER", str(trigger))
    monkeypatch.setattr(dg, "REAL_RAMP_WAIT_MS", 100)
    monkeypatch.setattr(dg, "QUIT_ACK_TIMEOUT_MS", 300)
    monkeypatch.setattr(dg, "REAL_RELEASE_TIMEOUT_MS", 1500)
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    sent = []
    orig = node.publish_bool
    node.publish_bool = lambda pub, v: (sent.append((pub.topic_name, bool(v))), orig(pub, v))
    try:
        yield g, node, sent, trigger, app
    finally:
        g.close()
        node.tf_listener.executor.shutdown()            # TF-Spin-Thread vor rclpy beenden
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def _pump(app, seconds, until=None):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        app.processEvents()
        if until is not None and until():
            return True
        time.sleep(0.01)
    return until() if until is not None else True


def _double_click(g):
    g._quit_clicked()
    g._quit_clicked()


def test_real_button_visible_and_needs_confirm(gui):
    g, node, sent, trigger, app = gui
    assert not g.sim_mode
    assert g.btn_quit.text() == "Real\nbeenden" and not g.btn_quit.isHidden()
    g._quit_clicked()                               # erster Klick: nur scharf
    assert sent == [] and not trigger.exists()
    assert "nochmal" in g.btn_quit.text()


def test_real_quit_releases_arms_before_stopping(gui):
    g, node, sent, trigger, app = gui
    node.arms_state = True
    _double_click(g)
    assert ("/g1pilot/start_walking", False) in sent     # Laufen -> StopMove
    assert ("/g1pilot/arms/enabled", False) in sent      # Arme abgeben
    assert ("/g1pilot/start_balancing", True) not in sent
    assert ("/g1pilot/emergency_stop", True) not in sent
    # Solange der Arm-Regler die Arme noch fuehrt: Stack NICHT stoppen.
    _pump(app, 0.5)
    assert not trigger.exists()
    g.request_mode("manip")                             # waehrenddessen gesperrt
    assert ("/g1pilot/arms/enabled", True) not in sent
    # Rueckfahrt fertig (arms/state False) -> nach der Rampe Trigger anlegen.
    node.arms_state = False
    assert _pump(app, 2.0, until=trigger.exists)


def test_real_quit_without_active_arms_stops_directly(gui):
    g, node, sent, trigger, app = gui
    node.arms_state = None                              # Arm-Regler nie gemeldet
    _double_click(g)
    assert ("/g1pilot/arms/enabled", False) not in sent
    assert trigger.exists()


def test_estop_aborts_real_quit(gui):
    g, node, sent, trigger, app = gui
    node.arms_state = True
    _double_click(g)
    g.emergency_stop()
    node.arms_state = False                             # E-Stop: Arme schlaff
    _pump(app, 2.5)
    assert not trigger.exists()
    assert g.btn_quit.isEnabled() and g.btn_quit.text() == "Real\nbeenden"
    assert g._real_quit == 0


def test_no_release_ack_still_stops_after_timeout(gui):
    g, node, sent, trigger, app = gui
    node.arms_state = True                              # bleibt True (Regler haengt)
    _double_click(g)
    assert _pump(app, 3.0, until=trigger.exists)


def test_missing_watcher_reported(gui):
    g, node, sent, trigger, app = gui
    node.arms_state = None
    _double_click(g)
    assert trigger.exists()
    assert _pump(app, 1.5, until=lambda: not trigger.exists())   # GUI raeumt auf
    assert g.btn_quit.isEnabled() and g._real_quit == 0
    assert "make stop" in g.status.text()

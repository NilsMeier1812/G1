#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NOT-HALT von aussen (PS-Taste am PS4-Controller) in der Demo-GUI.

Die GUI muss einen fremden /g1pilot/emergency_stop genauso anzeigen wie
den eigenen Knopf -- sonst bleibt z.B. die Demo-GUI auf REAL im alten Modus
stehen und »Start« (Quittung) gesperrt. Offscreen, ohne Roboter.
"""
import os

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy                                       # noqa: E402
from PyQt6.QtWidgets import QApplication          # noqa: E402
from std_msgs.msg import Bool                      # noqa: E402


@pytest.fixture(params=["real", "sim"])
def mode(request, monkeypatch):
    monkeypatch.setenv("G1_SIM_MODE", "false" if request.param == "real" else "true")
    rclpy.init()
    app = QApplication.instance() or QApplication([])
    yield request.param, app
    rclpy.shutdown()


def _published(node):
    """Alle Publizier-Aufrufe der GUI mitschneiden (Topic, Wert)."""
    log = []
    orig = node.publish_bool
    node.publish_bool = lambda pub, v: (log.append((pub.topic_name, v)), orig(pub, v))
    return log


def test_demo_gui_follows_external_estop(mode):
    which, app = mode
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    gui = dg.DemoGUI(node)
    try:
        sim = which == "sim"
        assert gui.sim_mode == sim
        if not sim:
            gui._start_robot()
        gui.request_mode(dg.MANIP)
        assert gui.mode == dg.MANIP and gui.started
        log = _published(node)

        node._emergency_stop(Bool(data=True))          # z.B. PS-Taste
        assert gui._estop_shown and gui.mode is None
        assert gui.started == sim                       # real: erst wieder START
        assert gui.btn_start.isVisibleTo(gui) == (not sim)
        assert gui.btn_start.isEnabled() == (not sim)    # real: Quittung frei
        if not sim:
            assert "quittieren" in gui.btn_start.text()
        assert gui._estop_unacked == sim                # sim: naechster Modus quittiert
        assert ("/g1pilot/emergency_stop", True) not in log   # nicht erneut gesendet
        node._emergency_stop(Bool(data=False))          # False aendert nichts
        assert gui._estop_shown

        # Quittieren wie gewohnt
        if sim:
            gui.request_mode(dg.MANIP)
            assert ("/g1pilot/start", True) in log
        else:
            gui._start_robot()
            assert ("/g1pilot/start", True) in log
            gui.request_mode(dg.MANIP)
            assert gui.mode == dg.MANIP
        assert not gui._estop_shown

        # Eigener NOT-HALT: genau einmal gesendet, Echo wird ignoriert
        log.clear()
        gui.emergency_stop()
        node._emergency_stop(Bool(data=True))
        assert log.count(("/g1pilot/emergency_stop", True)) == 1 and gui._estop_shown
    finally:
        gui.close()
        node.tf_listener.executor.shutdown()            # TF-Spin-Thread vor rclpy beenden
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()

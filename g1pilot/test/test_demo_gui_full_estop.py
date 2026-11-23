#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VOLL-NOTAUS-Knopf in der Demo-GUI (Seitenleiste unter dem NOT-HALT).

Derselbe Pfad wie R1 + L1 + PS am Controller: zuerst
/g1pilot/full_emergency_stop, dann /g1pilot/emergency_stop. Schutz gegen
Antippen: erst nach FULL_ESTOP_HOLD_S Halten; schon der Druck loest den
normalen NOT-HALT aus. Offscreen, ohne Roboter, Uhr des Knopfs per Stub.
"""
import os

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy                                       # noqa: E402
from PyQt6.QtWidgets import QApplication, QPushButton  # noqa: E402
from PyQt6.QtCore import QTimer                    # noqa: E402
from std_msgs.msg import Bool                      # noqa: E402

FULL = "/g1pilot/full_emergency_stop"
ESTOP = "/g1pilot/emergency_stop"


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture(params=["real", "sim"])
def gui(request, monkeypatch):
    monkeypatch.setenv("G1_SIM_MODE", "false" if request.param == "real" else "true")
    rclpy.init()
    app = QApplication.instance() or QApplication([])
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    clock = Clock()
    g.btn_full_estop._clock = clock
    log = []
    orig = node.publish_bool
    node.publish_bool = lambda pub, v: (log.append((pub.topic_name, v)), orig(pub, v))
    try:
        yield request.param == "sim", dg, g, node, clock, log
    finally:
        for t in g.findChildren(QTimer):
            t.stop()
        g.close()
        node.destroy_node()
        rclpy.shutdown()


def _start(dg, g):
    if not g.sim_mode:
        g._start_robot()
    g.request_mode(dg.MANIP)
    assert g.mode == dg.MANIP


def test_button_below_estop_in_sidebar(gui):
    sim, dg, g, node, clock, log = gui
    btn = g.btn_full_estop
    assert btn.isVisibleTo(g) and btn.isEnabled()
    assert "VOLL-NOTAUS" in btn.text() and "1,5 s" in btn.text()
    lay = g.btn_estop.parentWidget().layout()
    assert lay.indexOf(btn) == lay.indexOf(g.btn_estop) + 1
    assert dg.FULL_ESTOP_HOLD_S == btn.hold_s == 1.5


def test_hold_fires_full_estop_in_controller_order(gui):
    sim, dg, g, node, clock, log = gui
    _start(dg, g)
    log.clear()
    btn = g.btn_full_estop
    btn.begin()                                  # Druck
    # sofort normaler NOT-HALT, noch kein Voll-Notaus
    assert (ESTOP, True) in log and (FULL, True) not in log
    assert g._estop_shown and not g._full_estop_shown and g.mode is None
    assert btn.holding
    clock.t += 1.0
    btn._tick()
    assert (FULL, True) not in log and 0.6 < btn.progress() < 0.7
    clock.t += 0.6
    btn._tick()                                  # 1,6 s > 1,5 s
    assert not btn.holding
    i_full = log.index((FULL, True))
    assert (ESTOP, True) in log[i_full + 1:]      # danach erneut NOT-HALT
    assert g._full_estop_shown and g._estop_shown
    assert "VOLL-NOTAUS aktiv" in g.status.text()
    if not sim:
        assert g.btn_start.isEnabled() and "VOLL-NOTAUS" in g.btn_start.text()
        assert "VOLL-NOTAUS" in g._idle_text()
    # weiteres Halten / Loslassen sendet nichts nach
    n = log.count((FULL, True))
    clock.t += 5.0
    btn._tick()
    btn.cancel()
    assert log.count((FULL, True)) == n == 1


def test_early_release_aborts_but_estop_stays(gui):
    sim, dg, g, node, clock, log = gui
    _start(dg, g)
    log.clear()
    btn = g.btn_full_estop
    seen = []
    btn.aborted.connect(lambda: seen.append("abort"))
    btn.fired.connect(lambda: seen.append("fire"))
    btn.begin()
    clock.t += 1.4
    btn._tick()
    btn.cancel()                                 # zu frueh losgelassen
    assert seen == ["abort"]
    assert (FULL, True) not in log
    assert g._estop_shown and not g._full_estop_shown
    assert btn.progress() == 0.0 and not btn.holding
    clock.t += 10.0
    btn._tick()                                  # Timer-Rest loest nichts aus
    assert (FULL, True) not in log
    # Neues Halten startet von vorn, NOT-HALT nicht doppelt
    log.clear()
    btn.begin()
    assert (ESTOP, True) not in log
    clock.t += 1.5
    btn._tick()
    assert seen[-1] == "fire" and (FULL, True) in log


def test_real_mouse_press_release_signals(gui):
    """QPushButton pressed/released treiben den Halte-Knopf (Maus/Touch)."""
    sim, dg, g, node, clock, log = gui
    btn = g.btn_full_estop
    btn.pressed.emit()
    assert btn.holding and g._estop_shown
    btn.released.emit()
    assert not btn.holding and not g._full_estop_shown


def test_disabled_or_hidden_does_not_fire(gui):
    sim, dg, g, node, clock, log = gui
    btn = g.btn_full_estop
    btn.setEnabled(False)
    btn.begin()
    assert not btn.holding
    btn.setEnabled(True)
    g.show()
    btn.begin()
    btn.hide()                                   # z. B. Fenster weg -> Abbruch
    assert not btn.holding
    clock.t += 3.0
    btn._tick()
    assert (FULL, True) not in log


def test_external_full_estop_from_controller(gui):
    """R1 + L1 + PS am Controller: GUI zeigt den Voll-Notaus an, ohne selbst
    zu senden; Quittieren wie gewohnt."""
    sim, dg, g, node, clock, log = gui
    _start(dg, g)
    log.clear()
    node._full_emergency_stop(Bool(data=True))   # Controller: zuerst voll ...
    node._emergency_stop(Bool(data=True))        # ... dann NOT-HALT
    assert g._full_estop_shown and g._estop_shown and g.mode is None
    assert (FULL, True) not in log and (ESTOP, True) not in log
    node._full_emergency_stop(Bool(data=False))  # False aendert nichts
    assert g._full_estop_shown
    if sim:
        g.request_mode(dg.MANIP)                 # Sim: Modus waehlen quittiert
        assert ("/g1pilot/start", True) in log
        assert not g._full_estop_shown and not g._estop_shown
    else:
        assert "VOLL-NOTAUS" in g.btn_start.text()
        g._start_robot()
        assert ("/g1pilot/start", True) in log
        assert not g._full_estop_shown and not g._estop_shown
        assert "bleibt in Damp" in g.status.text()
        assert g.btn_start.text() == "Gestartet"
        # naechster NOT-HALT ist wieder ein normaler
        g.emergency_stop()
        assert g._estop_shown and not g._full_estop_shown
        assert "NOT-HALT" in g.btn_start.text() and "VOLL" not in g.btn_start.text()


def test_own_echo_is_idempotent(gui):
    sim, dg, g, node, clock, log = gui
    g.full_emergency_stop()
    assert log.count((FULL, True)) == 1 and log.count((ESTOP, True)) == 1
    node._full_emergency_stop(Bool(data=True))   # Echo der eigenen Meldung
    node._emergency_stop(Bool(data=True))
    assert log.count((FULL, True)) == 1 and log.count((ESTOP, True)) == 1
    assert g._full_estop_shown


def test_reset_robot_sim_clears_full_estop(gui):
    sim, dg, g, node, clock, log = gui
    if not sim:
        pytest.skip("Roboter zuruecksetzen gibt es nur in der Sim")
    g.full_emergency_stop()
    g._reset_robot()
    assert not g._full_estop_shown and not g._estop_shown


def test_hold_button_standalone():
    """HoldButton ohne ROS: Fortschritt, genau ein fired je Druck."""
    from g1pilot.teleoperation.demo_ui import HoldButton
    app = QApplication.instance() or QApplication([])  # noqa: F841
    clock = Clock()
    b = HoldButton("X", hold_s=2.0, clock=clock)
    ev = []
    b.armed.connect(lambda: ev.append("armed"))
    b.fired.connect(lambda: ev.append("fired"))
    b.aborted.connect(lambda: ev.append("aborted"))
    assert isinstance(b, QPushButton) and b.progress() == 0.0
    b.begin()
    b.begin()                                    # doppelt druecken zaehlt einmal
    clock.t += 1.0
    b._tick()
    assert b.progress() == pytest.approx(0.5)
    assert "qlineargradient" in b.styleSheet()
    clock.t += 1.0
    b._tick()
    b._tick()
    b.cancel()
    assert ev == ["armed", "fired"]
    assert "qlineargradient" not in b.styleSheet()

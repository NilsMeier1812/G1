#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""loco_client: Gehen per PS4 (/g1pilot/ps4/walk_cmd von ps4_walk) -- ohne Hardware.

Move() nur, solange der G1 schon im Walk-Modus steht und kein E-Stop aktiv ist;
der Controller schaltet den Modus der Beine nie selbst um. Bleibt der Strom
aus, StopMove. Solange er frisch ist, pausiert der Streamdeck-Pfad.
Fake-LocoClient und Fixture aus test_loco_client_real.py.
"""
import pytest

pytest.importorskip("rclpy")

from test_loco_client_real import _bool, _close, _cmd, _make, lc_module  # noqa: E402,F401


def _walk_mode(node, robot):
    robot.fsm = 500
    node._sync_onboard_state(force=True)
    assert node.balanced


def test_ps4_walk_moves_in_walk_mode_without_laufen(lc_module):
    node, robot = _make(lc_module)
    try:
        _walk_mode(node, robot)
        assert not node.walk_enabled, "LAUFEN in der GUI ist nicht noetig"
        node.ps4_walk_callback(_cmd(vx=0.35, vy=-0.5, wz=1.0))
        node._cmd_vel_tick()
        assert robot.vel == pytest.approx((0.14, -0.15, 0.4))
        assert robot.calls[-1][-1] == pytest.approx(1.0), "Move mit kurzer Dauer"
        assert node._move_active
    finally:
        _close(node)


@pytest.mark.parametrize("fsm", [1, 4, 0])
def test_ps4_walk_never_switches_leg_mode(lc_module, fsm):
    node, robot = _make(lc_module)
    try:
        robot.fsm = fsm
        node._sync_onboard_state(force=True)
        robot.calls.clear()
        for _ in range(3):
            node.ps4_walk_callback(_cmd(vx=1.0))
            node._cmd_vel_tick()
        assert "SetVelocity" not in robot.names()
        assert "SetFsmId" not in robot.names() and "Start" not in robot.names()
    finally:
        _close(node)


def test_ps4_walk_zero_and_timeout_stop(lc_module):
    node, robot = _make(lc_module)
    try:
        _walk_mode(node, robot)
        node.ps4_walk_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()
        assert robot.vel[0] > 0.0
        node.ps4_walk_callback(_cmd())          # L2 los -> ps4_walk sendet 0
        node._cmd_vel_tick()
        assert robot.vel == (0.0, 0.0, 0.0) and not node._move_active
        # Strom bricht MIT Fahrbefehl ab (Node tot) -> Timeout -> StopMove
        node.ps4_walk_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()
        assert robot.vel[0] > 0.0
        node._ps4_stamp -= 5.0
        node._cmd_vel_tick()
        assert robot.vel == (0.0, 0.0, 0.0) and not node._ps4_active
        n = len(robot.calls)
        node._cmd_vel_tick()
        assert len(robot.calls) == n, "StopMove nur einmal"
    finally:
        _close(node)


def test_ps4_walk_blocked_by_estop(lc_module):
    node, robot = _make(lc_module)
    try:
        _walk_mode(node, robot)
        node.ps4_walk_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()
        node.emergency_callback(_bool(True))
        assert robot.vel == (0.0, 0.0, 0.0)
        n = len(robot.calls)
        node.ps4_walk_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()
        assert all(c[0] != "SetVelocity" or c[1:4] == (0.0, 0.0, 0.0)
                   for c in robot.calls[n:])
    finally:
        _close(node)


def test_ps4_walk_has_priority_over_streamdeck(lc_module):
    node, robot = _make(lc_module)
    try:
        _walk_mode(node, robot)
        node.start_walking_callback(_bool(True))
        node.cmd_vel_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()
        assert robot.vel[0] == pytest.approx(0.4)
        node.ps4_walk_callback(_cmd(wz=0.5))
        node.cmd_vel_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()
        assert robot.vel == pytest.approx((0.0, 0.0, 0.2)), "PS4 hat Vorrang"
        node._ps4_stamp -= 5.0
        node.cmd_vel_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()                     # PS4-Timeout: einmal StopMove
        node._cmd_vel_tick()                     # danach wieder Streamdeck
        assert robot.vel[0] == pytest.approx(0.4)
    finally:
        _close(node)

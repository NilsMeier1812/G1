#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joy_mux gibt nur FRISCHE Joy-Eingaenge weiter (Review 2026-10-16).

Befund: joy_mux wiederholte die letzte Joy-Nachricht endlos. Starb der
Joystick-Node (oder nav2point) mit gedruecktem Deadman, sah loco_client
weiter einen frischen Strom -> der echte G1 ging weiter; der Joy-Timeout in
loco_client griff nie.
"""
import pytest

rclpy = pytest.importorskip("rclpy")
from sensor_msgs.msg import Joy        # noqa: E402
from std_msgs.msg import Bool          # noqa: E402

from g1pilot.teleoperation.joy_mux import JoyMux   # noqa: E402


def _joy(deadman=1, vx=-1.0):
    j = Joy()
    j.axes = [0.0] * 8
    j.buttons = [0] * 14
    j.buttons[8] = deadman
    j.axes[1] = vx
    return j


@pytest.fixture
def mux():
    rclpy.init()
    node = JoyMux()
    out = []
    node.pub.publish = out.append
    yield node, out
    node.destroy_node()
    rclpy.shutdown()


def test_manual_forwarded_while_fresh(mux):
    node, out = mux
    node.cb_manual(_joy())
    node.loop()
    assert len(out) == 1 and out[0].buttons[8] == 1


def test_stale_manual_is_not_repeated(mux):
    node, out = mux
    node.cb_manual(_joy())
    node.t_last_manual -= 1.0          # Joystick-Node tot / Controller weg
    for _ in range(5):
        node.loop()
    assert out == []


def test_stale_auto_is_not_repeated(mux):
    node, out = mux
    node.cb_enable(Bool(data=True))
    out.clear()
    node.cb_auto(_joy())
    node.loop()
    assert len(out) == 1
    node.t_last_auto -= 1.0            # nav2point tot / keine Pose mehr
    node.loop()
    assert len(out) == 1


def test_disable_auto_still_sends_stop(mux):
    node, out = mux
    node.cb_enable(Bool(data=True))
    node.cb_enable(Bool(data=False))
    assert out and all(b == 0 for b in out[-1].buttons)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gehen per PS4-Controller (ps4_walk, Wunsch Nils 2026-10-19) -- reine Logik.

L2 halten = Deadman; linker Stick vor/seitlich, rechter Stick drehen,
L2 + Steuerkreuz oben/unten = Geh-Tempo. Die Arm-Steuerung pausiert,
solange L2 gehalten wird (walk_deadman). Ohne ROS lauffaehig.
"""
import pytest

from g1pilot.teleoperation import ps4_layout as L
from g1pilot.teleoperation.ps4_walk import (
    WALK_SPEED_SCALE, WalkLogic, deadman_held,
)
from g1pilot.teleoperation.ps4_arm_teleop import ArmTeleopLogic


def joy(l2=0.0, buttons=(), **axes):
    a = [0.0] * L.NUM_AXES
    a[L.L2_AXIS] = l2
    for k, v in axes.items():
        a[getattr(L, k)] = v
    b = [0] * L.NUM_BUTTONS
    for i in buttons:
        b[i] = 1
    return a, b


def armed(logic, **kw):
    """L2 einmal los (scharf), dann halten -> aktiv."""
    logic.update(*joy())
    return logic.update(*joy(l2=1.0, **kw))


# ── Deadman ────────────────────────────────────────────────────────────

def test_deadman_axis_or_button():
    assert not deadman_held(*joy(l2=0.3))
    assert deadman_held(*joy(l2=0.6))
    assert deadman_held(*joy(buttons=(L.L2,)))
    assert not deadman_held([], [])


def test_walks_only_while_l2_held():
    w = WalkLogic()
    a, b = joy(LY=1.0)
    assert w.update(a, b) == [] and w.command(a) is None, "ohne L2 kein Gehen"
    assert armed(w, LY=1.0) == [("start",)]
    vx, vy, vyaw = w.command(joy(l2=1.0, LY=1.0)[0])
    assert vx == pytest.approx(WALK_SPEED_SCALE["Langsam"]) and vy == 0.0 and vyaw == 0.0
    assert w.update(*joy(LY=1.0)) == [("stop",)]
    assert w.command(joy(LY=1.0)[0]) is None


def test_l2_already_held_at_start_does_not_walk():
    """Node-Start / Wiederverbinden mit gedruecktem L2: erst loslassen."""
    w = WalkLogic()
    assert w.update(*joy(l2=1.0, LY=1.0)) == []
    assert w.command(joy(l2=1.0, LY=1.0)[0]) is None
    assert armed(w) == [("start",)]


def test_stick_directions_and_deadzone():
    w = WalkLogic(speed="Schnell")
    armed(w)
    a = joy(l2=1.0, LY=-1.0, LX=1.0, RX=-1.0)[0]
    assert w.command(a) == pytest.approx((-1.0, 1.0, -1.0))
    assert w.command(joy(l2=1.0, LY=0.1, RX=-0.1)[0]) == (0.0, 0.0, 0.0)
    # Rechter Stick vertikal, Trigger R2 usw. haben keine Wirkung aufs Gehen.
    a = joy(l2=1.0, RY=1.0)[0]
    a[L.R2_AXIS] = 1.0
    assert w.command(a) == (0.0, 0.0, 0.0)


def test_speed_steps_with_dpad_only_while_walking():
    w = WalkLogic(speed="Langsam", max_speed="Normal")
    w.update(*joy(buttons=(L.DPAD_UP,)))
    assert w.speed == "Langsam", "ohne L2 gehoert das Steuerkreuz den Haenden"
    armed(w)
    assert w.update(*joy(l2=1.0, buttons=(L.DPAD_UP,))) == [("speed",)]
    w.update(*joy(l2=1.0))
    assert w.update(*joy(l2=1.0, buttons=(L.DPAD_UP,))) == [], "max_speed deckelt"
    assert w.speed == "Normal"
    assert w.command(joy(l2=1.0, LY=1.0)[0])[0] == pytest.approx(WALK_SPEED_SCALE["Normal"])
    w.update(*joy(l2=1.0))
    w.update(*joy(l2=1.0, buttons=(L.DPAD_DOWN,)))
    w.update(*joy(l2=1.0))
    assert w.update(*joy(l2=1.0, buttons=(L.DPAD_DOWN,))) == []
    assert w.speed == "Langsam"


def test_ps_estop_stops_and_latches_until_start():
    w = WalkLogic()
    armed(w, LY=1.0)
    ev = w.update(*joy(l2=1.0, LY=1.0, buttons=(L.PS,)))
    assert ("stop",) in ev and ("estop",) in ev
    assert w.command(joy(l2=1.0, LY=1.0)[0]) is None
    w.update(*joy())
    w.update(*joy(l2=1.0, LY=1.0))
    assert w.command(joy(l2=1.0, LY=1.0)[0]) is None, "gesperrt bis START"
    assert w.clear_estop()
    w.update(*joy(l2=1.0, LY=1.0))
    assert w.command(joy(l2=1.0)[0]) is None, "nach START erst L2 loslassen"
    assert armed(w) == [("start",)]


def test_external_estop_and_joy_loss():
    w = WalkLogic()
    armed(w)
    assert w.latch_estop() == [("stop",)]
    assert w.latch_estop() == []
    w.clear_estop()
    armed(w)
    assert w.release() == [("stop",)]
    assert w.update(*joy(l2=1.0)) == [], "nach Abriss erst L2 loslassen"


def test_ps_without_estop_does_not_stop_walking():
    w = WalkLogic(ps_estop=False)
    armed(w)
    assert w.update(*joy(l2=1.0, buttons=(L.PS,))) == []
    assert w.active


# ── Zusammenspiel mit der Arm-Steuerung ────────────────────────────────

def test_arms_pause_while_walking():
    arm = ArmTeleopLogic(walk_deadman=True)
    a, b = joy(LY=1.0)
    arm.update(a, b, 0.0)
    assert arm.twist(a), "ohne L2: Stick bewegt die Hand"
    a, b = joy(l2=1.0, LY=1.0)
    arm.update(a, b, 0.1)
    assert arm.twist(a) == {}, "mit L2: Stick gehoert dem Gehen"
    # Steuerkreuz waehlt mit L2 keine Hand (ist dann Geh-Tempo)
    arm.update(*joy(l2=1.0, buttons=(L.DPAD_LEFT,)), 0.2)
    assert arm.selection == "right"
    arm.update(*joy(), 0.3)
    arm.update(*joy(buttons=(L.DPAD_LEFT,)), 0.4)
    assert arm.selection == "left"


def test_arm_hand_buttons_still_work_while_walking():
    arm = ArmTeleopLogic(walk_deadman=True)
    ev = arm.update(*joy(l2=1.0, buttons=(L.CIRCLE,)), 0.0)
    assert ("hand", "right", "open") in ev
    ev = arm.update(*joy(l2=1.0, buttons=(L.PS,)), 0.1)
    assert ("estop",) in ev


def test_without_walk_deadman_l2_is_ignored():
    """G1_PS4_WALK=0: Arm-Steuerung wie bisher, L2 ohne Wirkung."""
    arm = ArmTeleopLogic(walk_deadman=False)
    a, b = joy(l2=1.0, LY=1.0)
    arm.update(a, b, 0.0)
    assert arm.twist(a)
    arm.update(*joy(l2=1.0, buttons=(L.DPAD_LEFT,)), 0.1)
    assert arm.selection == "left"


def test_walk_set_speed_from_gui_capped():
    w = WalkLogic(speed="Langsam", max_speed="Normal")
    assert w.set_speed("Schnell") and w.speed == "Normal"
    assert not w.set_speed("Normal") and not w.set_speed("quatsch")
    assert w.set_speed("Langsam") and w.status()["speed"] == "Langsam"

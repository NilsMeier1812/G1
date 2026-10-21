#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gemeinsamer Bedienzustand GUI <-> Controller (shared_state.py), reine Logik."""
import json

from g1pilot.teleoperation import shared_state as ss


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def test_newer_wins_echo_and_old_ignored():
    ck = Clock()
    gui, ctl = ss.SharedState("gui", ck), ss.SharedState("ctl", ck)
    assert gui.set("arm_side", "left")
    assert ctl.merge(gui.to_json()) == {"arm_side": "left"}
    assert ctl.merge(gui.to_json()) == {}                 # gleiche Nachricht nochmal
    assert gui.merge(gui.payload()) == {}                 # eigenes Echo
    ck.t += 1
    assert ctl.set("arm_side", "right")
    assert gui.merge(ctl.to_json()) == {"arm_side": "right"}
    old = {"fields": {"arm_side": {"v": "both", "t": 999.0}}}
    assert gui.merge(old) == {} and gui.get("arm_side") == "right"


def test_same_value_is_a_new_choice():
    """GEHEN nach NOT-HALT erneut gewaehlt: gleicher Wert, aber neue Wahl."""
    ck = Clock()
    gui, ctl = ss.SharedState("gui", ck), ss.SharedState("ctl", ck)
    ctl.set("mode", "walk")
    gui.merge(ctl.payload())
    ck.t += 5
    ctl.set("mode", "walk")
    assert gui.merge(ctl.payload()) == {"mode": "walk"}


def test_default_stamp_loses_to_controller_start_values():
    ck = Clock()
    gui, ctl = ss.SharedState("gui", ck), ss.SharedState("ctl", ck)
    assert gui.set("arm_speed", "Normal", stamp=ss.DEFAULT_STAMP)
    ctl.set("arm_speed", "Langsam")                       # Start-Parameter (real)
    assert gui.merge(ctl.payload()) == {"arm_speed": "Langsam"}
    assert ctl.merge(gui.payload()) == {}
    # GUI startet spaeter neu: ihr Default kommt nicht mehr durch
    gui2 = ss.SharedState("gui2", ck)
    gui2.merge(ctl.payload())
    assert not gui2.set("arm_speed", "Normal", stamp=ss.DEFAULT_STAMP)
    assert gui2.get("arm_speed") == "Langsam"


def test_clock_skew_local_change_still_newest():
    """Andere Uhr geht vor: eigene Aenderung bekommt trotzdem den neuesten Stempel."""
    a = ss.SharedState("a", Clock(1000.0))
    b = ss.SharedState("b", Clock(2000.0))
    b.set("walk_speed", "Schnell")
    a.merge(b.payload())
    a.set("walk_speed", "Langsam")
    assert b.merge(a.payload()) == {"walk_speed": "Langsam"}


def test_invalid_input_ignored():
    s = ss.SharedState("x", Clock())
    assert not s.set("arm_side", "oben") and not s.set("nope", "left")
    assert s.merge("kein json") == {} and s.merge({"fields": 3}) == {}
    assert s.merge({"fields": {"mode": {"v": "fly", "t": 5000}}}) == {}
    assert s.merge({"fields": {"mode": {"v": "walk", "t": "spaet"}}}) == {}
    assert s.merge(json.dumps({"fields": {"mode": {"v": "grasp", "t": 5000}}})) == \
        {"mode": "grasp"}


def test_capped():
    assert ss.capped("Schnell", "Normal") == "Normal"
    assert ss.capped("Langsam", "Normal") == "Langsam"
    assert ss.capped("quatsch", "Normal") == "Normal"
    assert ss.capped("Normal", "Schnell") == "Normal"

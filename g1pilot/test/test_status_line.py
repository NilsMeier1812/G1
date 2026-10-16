#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Info-Zeile der Demo-GUI: Onboard-Modus + PS4-Auswahl (reine Logik)."""
from g1pilot.teleoperation import status_line as sl


def test_onboard_texts():
    assert "keine Daten" in sl.onboard_text(None, None)[0]
    assert "keine Daten" in sl.onboard_text({"fsm": 500}, 10.0)[0]
    t, c = sl.onboard_text({"fsm": 500, "name": "Walk", "walk": False}, 0.1)
    assert t == "G1: Walk (FSM 500) · bereit" and c == sl.GREEN
    t, c = sl.onboard_text({"fsm": 500, "name": "Walk", "walk": True}, 0.1)
    assert "Gehen aktiv" in t
    t, c = sl.onboard_text({"fsm": 4, "name": "Preparation"}, 0.1)
    assert "Preparation (FSM 4)" in t and c == sl.AMBER
    t, c = sl.onboard_text({"fsm": 1, "name": "Damp"}, 0.1, walk_requested=True)
    assert "Gehen gesperrt" in t and c == sl.RED
    t, c = sl.onboard_text({"fsm": 500, "name": "Walk", "estop": True}, 0.1)
    assert "NOT-HALT" in t and c == sl.RED
    assert "unbekannt" in sl.onboard_text({"fsm": None}, 0.1)[0]


def test_ps4_texts():
    assert sl.ps4_text(None, None) is None
    st = {"selection": "left", "mode": "move", "speed": "Langsam", "connected": True}
    t, c = sl.ps4_text(st, 0.2)
    assert t == "PS4: Hand LINKS · Verschieben · Langsam" and c == "#005aff"
    t, _ = sl.ps4_text(dict(st, selection="mirror", mode="turn"), 0.2)
    assert "BEIDE gespiegelt · Drehen" in t
    assert "nicht verbunden" in sl.ps4_text(dict(st, connected=False), 0.2)[0]
    assert "Roboter starten" in sl.ps4_text(dict(st, estop=True), 0.2)[0]
    assert "Modus wählen" in sl.ps4_text(dict(st, estop=True), 0.2, sim=True)[0]
    assert "antwortet nicht" in sl.ps4_text(st, 9.0)[0]


def test_ps4_help_follows_ps_estop():
    assert sl.ps4_help({"ps_estop": True})[-1][1] == "NOT-HALT"
    assert "STOPP" in sl.ps4_help({"ps_estop": False})[-1][1]
    assert len(sl.ps4_help()) == len(sl.PS4_HELP)


def test_parse_ignores_garbage():
    assert sl.parse("kaputt") is None and sl.parse("[1]") is None
    assert sl.parse('{"fsm": 4}') == {"fsm": 4}

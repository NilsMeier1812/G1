#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Info-Zeile der Demo-GUI: Onboard-Modus + PS4-Auswahl (reine Logik)."""
from g1pilot.teleoperation import status_line as sl


def test_onboard_texts():
    assert "keine Daten" in sl.onboard_text(None, None)[0]
    assert "keine Daten" in sl.onboard_text({"fsm": 500}, 10.0)[0]
    t, c = sl.onboard_text({"fsm": 500, "name": "Walk", "walk": False, "walk_fsm": 500}, 0.1)
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


def test_start_buttons_follow_onboard_mode():
    off = {k: (False, False) for k in sl.START_STEPS}
    assert sl.start_buttons(None, None) == off
    assert sl.start_buttons({"fsm": 4}, 10.0) == off          # veraltet
    b = sl.start_buttons({"fsm": 0}, 0.1)                    # nach dem Einschalten
    assert b["damp"] == (True, False) and not b["preparation"][0] and not b["walk"][0]
    b = sl.start_buttons({"fsm": 1}, 0.1)
    assert not b["damp"][0] and b["preparation"][0] and not b["walk"][0]
    b = sl.start_buttons({"fsm": 4}, 0.1)
    assert b["damp"] == (True, True) and not b["preparation"][0] and b["walk"] == (True, True)
    b = sl.start_buttons({"fsm": 501}, 0.1)
    assert b["damp"] == (True, True) and b["preparation"][0] and not b["walk"][0]
    b = sl.start_buttons({"fsm": 4, "estop": True}, 0.1)
    assert b["damp"][0] and not b["preparation"][0] and not b["walk"][0]
    b = sl.start_buttons({"fsm": None}, 0.1)                 # Modus unbekannt
    assert b["damp"] == (True, True) and not b["walk"][0]


def test_onboard_text_rpc_error():
    """Real 2026-10-16: Roboter beantwortet die Loco-RPCs nicht -> klare Meldung
    statt »keine Daten vom loco_client«."""
    t, _ = sl.onboard_text({"fsm": 4, "name": "Preparation", "rpc_err": 3104}, 0.1)
    assert "keine Antwort vom Onboard-Regler" in t and "Zeitüberschreitung" in t
    t, _ = sl.onboard_text({"fsm": None, "rpc_err": 3205}, 0.1)
    assert "3205" in t


def test_walk_501_shown_and_500_offers_switch():
    """Nils 2026-10-19: Walk = 501 (Walk with waist). Steht der G1 per App in
    500, zeigt die Info-Zeile das und die Startleiste bietet »Walk« zum
    Umschalten an (mit Rueckfrage)."""
    st = {"fsm": 501, "name": "Walk mit Taille", "walk_fsm": 501}
    t, c = sl.onboard_text(st, 0.1)
    assert t == "G1: Walk mit Taille (FSM 501) · bereit" and c == sl.GREEN
    assert not sl.start_buttons(st, 0.1)["walk"][0]
    st = {"fsm": 500, "name": "Walk", "walk_fsm": 501}
    t, c = sl.onboard_text(st, 0.1)
    assert "»Walk« schaltet auf FSM 501" in t and c == sl.GREEN
    assert sl.start_buttons(st, 0.1)["walk"] == (True, True)
    assert not sl.start_buttons(dict(st, estop=True), 0.1)["walk"][0]
    # Ziel 500 konfiguriert (G1_WALK_FSM_ID=500): 500 ist dann richtig.
    st = {"fsm": 500, "name": "Walk", "walk_fsm": 500}
    assert "schaltet" not in sl.onboard_text(st, 0.1)[0]
    assert not sl.start_buttons(st, 0.1)["walk"][0]

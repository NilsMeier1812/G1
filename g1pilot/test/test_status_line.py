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
    ps = lambda rows: dict(rows)["PS"]   # noqa: E731
    assert ps(sl.ps4_help({"ps_estop": True})) == "NOT-HALT"
    assert "STOPP" in ps(sl.ps4_help({"ps_estop": False}))
    assert len(sl.ps4_help()) == len(sl.PS4_HELP)
    assert "VOLL-NOTAUS" in dict(sl.ps4_help())["R1 + L1 + PS (2 s halten)"]


def test_full_estop_texts():
    t, c = sl.onboard_text({"fsm": 1, "name": "Damp", "estop": True, "full_estop": True}, 0.1)
    assert "VOLL-NOTAUS" in t and "Startleiste" in t and c == sl.RED
    st = {"selection": "left", "mode": "move", "speed": "Langsam", "connected": True}
    t, c = sl.ps4_text(dict(st, full_arming=True), 0.2)
    assert "loslassen bricht ab" in t and c == sl.RED
    t, c = sl.ps4_text(dict(st, estop=True, full_estop=True), 0.2)
    assert "VOLL-NOTAUS" in t and "Roboter starten" in t
    # Startleiste nach Voll-Notaus (Damp): erst quittieren, dann Preparation.
    b = sl.start_buttons({"fsm": 1, "estop": True, "full_estop": True}, 0.1)
    assert not b["preparation"][0] and not b["walk"][0]
    assert sl.start_buttons({"fsm": 1}, 0.1)["preparation"][0]


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


def test_ps4_text_walk_mode_locked():
    text, _ = sl.ps4_text({"connected": True, "arms_locked": True, "selection": "right"}, 0.1)
    assert "GEHEN" in text and "R2" in text


def test_ps4_text_controller_walk_and_device():
    st = {"connected": True, "selection": "right", "mode": "move", "speed": "Normal",
          "speed_req": "Normal"}
    assert sl.ps4_text(st, 0.1, device={"kind": "xbox"})[0].startswith("Xbox: Hand")
    assert sl.ps4_text(st, 0.1, device={"kind": None})[0].startswith("PS4:")
    t, _ = sl.ps4_text(dict(st, speed_req="Schnell"), 0.1)
    assert t.endswith("Normal (max.)")
    walk = {"active": True, "speed": "Normal", "walk_ready": True, "estop": False}
    t, c = sl.ps4_text(st, 0.1, walk=walk)
    assert "GEHT" in t and "Normal" in t and c == "#ff6e00"
    t, c = sl.ps4_text(st, 0.1, walk=dict(walk, walk_ready=False))
    assert "nicht im Walk" in t and c == sl.AMBER
    t, _ = sl.ps4_text(dict(st, arms_locked=True), 0.1, walk=dict(walk, active=False))
    assert "GEHEN" in t and "(Normal)" in t
    # NOT-HALT hat Vorrang vor Gehen
    assert "NOT-HALT" in sl.ps4_text(dict(st, estop=True), 0.1, walk=walk)[0]

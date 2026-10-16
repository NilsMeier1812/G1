#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
status_line — Texte fuer die Info-Zeile der Demo-GUI (reine Logik, ohne Qt/ROS).

Zwei Anzeigen, beide nur lesend:
  * Onboard-Modus des ECHTEN G1 (loco_client -> /g1pilot/onboard_state):
    "G1: Walk (FSM 500)" usw. Damit sieht man nach einem Start per Handy-App
    oder Fernbedienung sofort, in welchem Modus der Roboter steht und ob
    GEHEN ueberhaupt freigegeben ist.
  * PS4-Controller (ps4_arm_teleop -> /g1pilot/ps4/status): welche Hand
    (links/rechts/beide/gespiegelt), Verschieben/Drehen, Tempo, Verbindung.
    Klick auf die Anzeige zeigt die Belegung (PS4_HELP).
"""
import json

from g1pilot.teleoperation.ps4_arm_teleop import COLORS, MODE_LABEL, SELECTION_LABEL

STALE_S = 3.0           # aelter -> "keine Daten"
GREY, GREEN, AMBER, RED = "#888888", "#66bb6a", "#ffb300", "#ef5350"
WALK_FSM = (500, 501)


def parse(text):
    """JSON-String -> dict oder None (Muell wird ignoriert)."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _hex(rgb):
    return "#%02x%02x%02x" % tuple(int(c) for c in rgb)


def onboard_text(state, age_s, walk_requested=False):
    """-> (Text, Farbe). state = dict von /g1pilot/onboard_state oder None."""
    if state is None or age_s is None or age_s > STALE_S:
        return "G1: keine Daten vom loco_client", GREY
    fsm = state.get("fsm")
    if fsm is None:
        return "G1: Modus unbekannt (keine Antwort vom Roboter)", AMBER
    name = state.get("name") or "Modus"
    text = f"G1: {name} (FSM {fsm})"
    if state.get("estop"):
        return text + " · NOT-HALT – mit »Roboter starten« quittieren", RED
    if fsm in WALK_FSM:
        text += " · Gehen aktiv" if state.get("walk") else " · bereit"
        return text, GREEN
    if fsm == 4:
        return text + " · Kran lockern, dann »Walk«", AMBER
    if walk_requested:
        return text + " · Gehen gesperrt – erst Damp → Preparation → Walk", RED
    return text, RED if fsm in (0, 1) else AMBER


def ps4_text(status, age_s, sim=False):
    """-> (Text, Farbe) oder None (kein PS4-Arm-Modus aktiv)."""
    if status is None:
        return None
    if age_s is None or age_s > STALE_S:
        return "PS4: Steuerung antwortet nicht", GREY
    if status.get("estop"):
        how = "Modus wählen" if sim else "»Roboter starten«"
        return f"PS4: NOT-HALT – gesperrt bis {how}", RED
    if not status.get("connected"):
        return "PS4: nicht verbunden", GREY
    sel = status.get("selection", "")
    label = SELECTION_LABEL.get(sel, sel.upper())
    mode = MODE_LABEL.get(status.get("mode"), status.get("mode", ""))
    speed = status.get("speed", "")
    color = _hex(COLORS[sel]) if sel in COLORS else GREY
    return f"PS4: Hand {label} · {mode} · {speed}", color


# Startleiste (nur real): Damp -> Preparation -> Walk wie in der Handy-App.
START_STEPS = ("damp", "preparation", "walk")
START_LABEL = {"damp": "Damp", "preparation": "Preparation", "walk": "Walk"}


def start_buttons(state, age_s):
    """-> {schritt: (freigegeben, rueckfrage)} fuer die Startleiste.

    Ohne frische Daten vom loco_client ist alles gesperrt. Damp geht immer
    (Rueckfrage, ausser der G1 ist schon schlaff/Damp: er sackt sonst in den
    Kran). Preparation nur aus Damp oder Walk, Walk nur aus Preparation --
    beides nicht im NOT-HALT; Walk immer mit Rueckfrage (Kran gelockert?)."""
    off = {k: (False, False) for k in START_STEPS}
    if state is None or age_s is None or age_s > STALE_S:
        return off
    fsm = state.get("fsm")
    estop = bool(state.get("estop"))
    return {
        "damp": (fsm != 1, fsm not in (0, 1)),
        "preparation": (not estop and (fsm == 1 or fsm in WALK_FSM), False),
        "walk": (not estop and fsm == 4, True),
    }


# Belegung (wie ps4_arm_teleop / docs/43_ps4_controller.md).
PS4_HELP = (
    ("Steuerkreuz ◀ / ▶", "Hand LINKS / RECHTS auswählen"),
    ("Steuerkreuz ▲ / ▼", "BEIDE Hände / BEIDE gespiegelt"),
    ("Dreieck", "Umschalten Verschieben ↔ Drehen"),
    ("Linker Stick", "Verschieben: vor/zurück + links/rechts · Drehen: kippen + schwenken"),
    ("Rechter Stick", "Verschieben: hoch/runter · Drehen: rollen"),
    ("L1 / R1", "Tempo langsamer / schneller"),
    ("Kreis / Viereck", "Hand öffnen / schließen"),
    ("Kreuz", "STOPP: Hand hält sofort an, Bewegung abbrechen"),
    ("Options", "Arme übernehmen"),
    ("Share (1 s halten)", "Grundstellung"),
    ("PS", "NOT-HALT"),
)


def ps4_help(status=None):
    """Belegung als (Taste, Wirkung)-Paare; PS je nach ps_estop des Nodes."""
    rows = list(PS4_HELP)
    if status is not None and status.get("ps_estop") is False:
        rows[-1] = ("PS", "STOPP wie Kreuz (kein NOT-HALT)")
    return rows

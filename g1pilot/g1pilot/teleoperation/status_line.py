#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
status_line — Texte fuer die Info-Zeile der Demo-GUI (reine Logik, ohne Qt/ROS).

Zwei Anzeigen, beide nur lesend:
  * Onboard-Modus des ECHTEN G1 (loco_client -> /g1pilot/onboard_state):
    "G1: Walk (FSM 500)" usw. Damit sieht man nach einem Start per Handy-App
    oder Fernbedienung sofort, in welchem Modus der Roboter steht und ob
    GEHEN ueberhaupt freigegeben ist.
  * PS4-/Xbox-Controller (ps4_arm_teleop -> /g1pilot/ps4/status, ps4_walk ->
    /g1pilot/ps4/walk_status, ps4_joystick -> /g1pilot/ps4/device): welche
    Hand (links/rechts/beide/gespiegelt), Verschieben/Drehen, Tempo, Gehen mit
    L2 (Totmann) samt Geh-Tempo, Arm-Sperre im GEHEN, Sperre per Options,
    NOT-HALT, Verbindung und
    welcher Controller. Klick auf die Anzeige zeigt die Belegung (PS4_HELP).
"""
import json

from g1pilot.teleoperation.ps4_arm_teleop import COLORS, MODE_LABEL, SELECTION_LABEL

STALE_S = 3.0           # aelter -> "keine Daten"
GREY, GREEN, AMBER, RED = "#888888", "#66bb6a", "#ffb300", "#ef5350"
WALK_FSM = (500, 501)
WALK_FSM_DEFAULT = 501   # Walk with waist (loco_client start_fsm_id)


def parse(text):
    """JSON-String -> dict oder None (Muell wird ignoriert)."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _hex(rgb):
    return "#%02x%02x%02x" % tuple(int(c) for c in rgb)


def walk_switch(state):
    """True: G1 steht in einem Walk-Modus, aber nicht im Ziel-Walk des
    loco_client (z. B. 500 per Handy-App statt 501) -> »Walk« schaltet um."""
    if not state:
        return False
    fsm = state.get("fsm")
    return fsm in WALK_FSM and fsm != state.get("walk_fsm", WALK_FSM_DEFAULT)


def ground_contact(state):
    """Bodenkontakt-Vorabpruefung fuer »Walk« aus Preparation (FSM 4).

    True  = Fuesse belastet -> der Walk-Start (501) geht.
    False = Fuesse ohne Last (z. B. am Kran) oder nicht messbar -> der
            loco_client wuerde »Walk« ablehnen.
    None  = unbekannt: nicht in Preparation oder ein aelterer loco_client.

    Quelle: Feld "ground" des loco_client (Knie-Last aus rt/lowstate, Grund
    in "ground_why"). Fehlt es (loco_client vor 2026-10-21), wie damals der
    FSM-Mode -- der meldet am Kran haengend aber trotzdem 0."""
    if not state or state.get("fsm") != 4:
        return None
    if "ground" in state:
        g = state.get("ground")
        return None if g is None else bool(g)
    mode = state.get("fsm_mode")
    if mode is None:
        return None
    return mode == 0


def onboard_text(state, age_s, walk_requested=False):
    """-> (Text, Farbe). state = dict von /g1pilot/onboard_state oder None."""
    if state is None or age_s is None or age_s > STALE_S:
        return "G1: keine Daten vom loco_client", GREY
    fsm = state.get("fsm")
    err = state.get("rpc_err")
    if err is not None:
        # Lesefehler hat Vorrang: der zuletzt bekannte Modus koennte veraltet sein.
        why = "Zeitüberschreitung" if err == 3104 else f"RPC-Fehler {err}"
        return f"G1: keine Antwort vom Onboard-Regler ({why}) – Modus unbekannt", AMBER
    if fsm is None:
        return "G1: Modus unbekannt (keine Antwort vom Roboter)", AMBER
    name = state.get("name") or "Modus"
    text = f"G1: {name} (FSM {fsm})"
    if state.get("full_estop"):
        return text + " · VOLL-NOTAUS – »Start« quittiert, dann Startleiste", RED
    if state.get("estop"):
        return text + " · NOT-HALT – mit »Start« quittieren", RED
    if fsm in WALK_FSM:
        text += " · Gehen aktiv" if state.get("walk") else " · bereit"
        if walk_switch(state):
            text += f" · »Walk« schaltet auf FSM {state.get('walk_fsm', WALK_FSM_DEFAULT)}"
        return text, GREEN
    if fsm == 4:
        ground = ground_contact(state)
        why = state.get("ground_why") or (
            f"FSM-Mode {state.get('fsm_mode')}" if "ground" not in state else "")
        why = f" ({why})" if why else ""
        if ground:
            return text + f" · Bodenkontakt ✓{why} – »Walk« möglich", GREEN
        if ground is False:
            return (text + f" · kein Bodenkontakt{why}"
                    " – »Walk« gesperrt, Kran lockern"), RED
        return text + " · Kran lockern, dann »Walk«", AMBER
    if walk_requested:
        return text + " · Gehen gesperrt – erst Damp → Preparation → Walk", RED
    return text, RED if fsm in (0, 1) else AMBER


def controller_label(device):
    """'PS4' oder 'Xbox' (device = dict von /g1pilot/ps4/device oder None)."""
    return "Xbox" if (device or {}).get("kind") == "xbox" else "PS4"


def ps4_text(status, age_s, sim=False, walk=None, device=None):
    """-> (Text, Farbe) oder None (kein PS4-Arm-Modus aktiv).
    walk = /g1pilot/ps4/walk_status (ps4_walk), device = /g1pilot/ps4/device."""
    if status is None:
        return None
    c = controller_label(device)
    if age_s is None or age_s > STALE_S:
        return f"{c}: Steuerung antwortet nicht", GREY
    if status.get("full_arming"):
        return f"{c}: VOLL-NOTAUS wird ausgelöst – loslassen bricht ab", RED
    if status.get("full_estop"):
        how = "Modus wählen" if sim else "»Start«"
        return f"{c}: VOLL-NOTAUS (ganzer G1 Damp) – gesperrt bis {how}", RED
    if status.get("estop"):
        how = "Modus wählen" if sim else "»Start«"
        return f"{c}: NOT-HALT – gesperrt bis {how}", RED
    if not status.get("connected"):
        return f"{c}: nicht verbunden", GREY
    if status.get("locked"):
        btn = "Menu" if c == "Xbox" else "Options"
        return f"{c}: GESPERRT – {btn} entsperrt (PS-Notaus wirkt weiter)", GREY
    walk = walk or {}
    if walk.get("active") and not walk.get("estop"):
        if walk.get("walk_ready") is False:
            return f"{c}: L2 gehalten – G1 steht nicht im Walk-Modus", AMBER
        return f"{c}: GEHT (L2 gehalten) · Geh-Tempo {walk.get('speed', '')}", \
            _hex(COLORS["walk"])
    if status.get("arms_locked"):
        speed = f" ({walk['speed']})" if walk.get("speed") else ""
        return (f"{c}: GEHEN – L2 halten = laufen{speed}, R2 = Greifen",
                _hex(COLORS["walk"]))
    if status.get("waist_mode"):
        return f"{c}: HÜFTE – rechter Stick ◀/▶ dreht, R3 = Mitte", _hex(COLORS["waist"])
    sel = status.get("selection", "")
    label = SELECTION_LABEL.get(sel, sel.upper())
    mode = MODE_LABEL.get(status.get("mode"), status.get("mode", ""))
    speed = status.get("speed", "")
    if status.get("speed_req") not in (None, speed):
        speed += " (max.)"     # GUI wollte schneller, Controller-Grenze max_speed
    color = _hex(COLORS[sel]) if sel in COLORS else GREY
    return f"{c}: Hand {label} · {mode} · {speed}", color


# Startleiste (nur real): Damp -> Preparation -> Walk wie in der Handy-App.
START_STEPS = ("damp", "preparation", "walk")
START_LABEL = {"damp": "Damp", "preparation": "Preparation", "walk": "Walk"}


def start_buttons(state, age_s):
    """-> {schritt: (freigegeben, rueckfrage)} fuer die Startleiste.

    Ohne frische Daten vom loco_client ist alles gesperrt. Damp geht immer
    (Rueckfrage, ausser der G1 ist schon schlaff/Damp: er sackt sonst in den
    Kran). Preparation nur aus Damp oder Walk, Walk nur aus Preparation --
    beides nicht im NOT-HALT; Walk immer mit Rueckfrage (Kran gelockert?).
    Meldet der loco_client vorab »kein Bodenkontakt« (ground_contact False),
    ist Walk schon vor dem Klick gesperrt -- er wuerde ihn sonst ablehnen."""
    off = {k: (False, False) for k in START_STEPS}
    if state is None or age_s is None or age_s > STALE_S:
        return off
    fsm = state.get("fsm")
    estop = bool(state.get("estop"))
    return {
        "damp": (fsm != 1, fsm not in (0, 1)),
        "preparation": (not estop and (fsm == 1 or fsm in WALK_FSM), False),
        "walk": (not estop and ((fsm == 4 and ground_contact(state) is not False)
                                or walk_switch(state)), True),
    }


def walk_button_label(state, age_s):
    """Beschriftung von »Walk« in der Startleiste: zeigt das Ergebnis der
    Bodenkontakt-Vorabpruefung schon vor dem Klick."""
    if state is None or age_s is None or age_s > STALE_S or walk_switch(state):
        return START_LABEL["walk"]
    ground = ground_contact(state)
    if ground:
        return "Walk\nBodenkontakt ✓"
    if ground is False:
        return "Walk\nkein Bodenkontakt"
    return START_LABEL["walk"]


def walk_tile_hint(state, age_s):
    """Untertitel der Kachel GEHEN (nur real, inaktiv): geht GEHEN jetzt?
    None = normaler Text (»antippen zum Wechseln«, G1 steht im Walk)."""
    if state is None or age_s is None or age_s > STALE_S:
        return "gesperrt · keine Daten vom G1"
    fsm = state.get("fsm")
    if state.get("estop") or state.get("full_estop"):
        return "gesperrt · NOT-HALT"
    if fsm in WALK_FSM:
        return None
    if fsm == 4:
        ground = ground_contact(state)
        if ground:
            return "erst »Walk« · Bodenkontakt ✓"
        if ground is False:
            return "gesperrt · kein Bodenkontakt"
        return "erst »Walk« (Startleiste)"
    return "gesperrt · G1 nicht im Walk"


# Belegung (wie ps4_arm_teleop / docs/11-A-Controller.md).
PS4_HELP = (
    ("Steuerkreuz ◀ / ▶", "Hand LINKS / RECHTS auswählen"),
    ("Steuerkreuz ▲ / ▼", "BEIDE Hände / BEIDE gespiegelt"),
    ("Dreieck", "Umschalten Verschieben ↔ Drehen"),
    ("Linker Stick", "Verschieben: vor/zurück + links/rechts · Drehen: kippen + schwenken"),
    ("Rechter Stick", "Verschieben: hoch/runter · Drehen: rollen"),
    ("L1 / R1", "Tempo langsamer / schneller"),
    ("Kreis / Viereck", "Hand öffnen / schließen"),
    ("Kreuz", "STOPP: Hand hält sofort an, Bewegung abbrechen"),
    ("Options", "Controller sperren / entsperren (zum Ablegen; PS und Voll-Notaus wirken weiter)"),
    ("Share (1 s halten)", "Grundstellung"),
    ("L3 (linken Stick drücken)", "Ablauf »Schritt für Schritt«: nächster Schritt (Weiter ▶)"),
    ("L2 drücken", "Modus GEHEN (Arm-Tasten gesperrt, LED blau)"),
    ("L2 halten", "GEHEN: linker Stick vor/seitlich, rechter Stick drehen, "
                  "▲/▼ Geh-Tempo (nur im Walk-Modus)"),
    ("R2 drücken", "Modus GREIFEN (Arme übernehmen – auch nach NOT-HALT)"),
    ("R2 halten + rechter Stick ◀/▶", "HÜFTE drehen (Oberkörper links/rechts, Arme drehen mit)"),
    ("R2 halten + R3", "Hüfte zurück zur Mitte"),
    ("PS", "NOT-HALT"),
    ("R1 + L1 + PS (2 s halten)", "VOLL-NOTAUS: ganzer G1 in Damp (vibriert, loslassen = Abbruch)"),
)


def ps4_help(status=None):
    """Belegung als (Taste, Wirkung)-Paare; PS je nach ps_estop des Nodes."""
    rows = list(PS4_HELP)
    if status is not None and status.get("ps_estop") is False:
        i = [k for k, _ in rows].index("PS")
        rows[i] = ("PS", "STOPP wie Kreuz (kein NOT-HALT)")
    return rows


def waist_text(state, age_s):
    """Hueftdrehung (arm_controller -> /g1pilot/waist/state) -> (Text, Farbe)
    oder None (keine Daten, Hueftdrehung aus)."""
    if state is None or age_s is None or age_s > STALE_S or not state.get("owner"):
        return None
    try:
        off = float(state.get("meas_offset_deg")
                    if state.get("meas_offset_deg") is not None else state.get("offset_deg", 0.0))
    except (TypeError, ValueError):
        off = 0.0
    if abs(off) < 1.0:
        where = "Mitte"
    else:
        where = f"{abs(off):.0f}° {'links' if off > 0 else 'rechts'}"
    pend = {"walk": "vor GEHEN", "home": "vor Grundstellung",
            "release": "vor Rückgabe"}.get(state.get("pending") or "")
    if state.get("centering"):
        return f"Hüfte {where} – fährt zur Mitte{' ' + pend if pend else ''}", AMBER
    if state.get("moving"):
        return f"Hüfte {where} – dreht", GREEN
    lim = state.get("limit_deg")
    lim_txt = f" (max. ±{float(lim):.0f}°)" if isinstance(lim, (int, float)) else ""
    return f"Hüfte {where}{lim_txt}", GREEN if where == "Mitte" else AMBER

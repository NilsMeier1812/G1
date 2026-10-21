#!/usr/bin/env python3
"""
shared_state — EIN gemeinsamer Bedienzustand fuer Demo-GUI und Controller.

Demo-GUI, ps4_arm_teleop und ps4_walk publizieren und lesen dasselbe latched
Topic /g1pilot/teleop_state (String, JSON). Wer etwas aendert (Kachel in der
GUI, L2/R2 oder L1/R1 am Controller), schickt den Wert mit Zeitstempel; alle
anderen uebernehmen ihn, wenn er neuer ist als ihr Stand. So zeigen GUI und
Controller immer dasselbe -- egal, auf welcher Seite gewechselt wurde.

Felder (jedes mit eigenem Zeitstempel, letzter Schreiber gewinnt):
  mode        "walk" | "grasp"   Modus GEHEN / GREIFEN (Kacheln, L2 / R2)
  arm_side    "left" | "right" | "both" | "mirror"   Hand-Auswahl
  arm_speed   "Langsam" | "Normal" | "Schnell"       Tempo "Hand bewegen"
  walk_speed  "Langsam" | "Normal" | "Schnell"       Geh-Tempo

Nur Anzeige/Einstellung -- KEINE Fahrbefehle. Der Modus-Wechsel selbst laeuft
weiter ueber start_walking/start_balancing/arms/enabled (wer wechselt, sendet
die wie bisher); dieses Topic sagt den anderen nur, DASS gewechselt wurde.
Tempo-Stufen sind Wuensche: jeder Teilnehmer begrenzt auf sein eigenes
Maximum (z. B. Controller real max. Normal, GUI-Gehen max. Normal).

Zeitstempel: Wanduhr, aber immer groesser als jeder schon gesehene Stempel des
Feldes (gegen Uhr-Versatz). Startwerte: die Controller-Nodes publizieren ihre
Start-Parameter beim Start (Real: Langsam) mit aktuellem Stempel; die GUI
meldet ihre Startwerte nur mit DEFAULT_STAMP und uebernimmt damit, was schon
da ist.
"""
import json
import time

from g1pilot.teleoperation.hand_jog import JOG_SPEED_ORDER

TOPIC = "/g1pilot/teleop_state"
SPEED_ORDER = ("Langsam", "Normal", "Schnell")
FIELDS = {
    "mode": ("walk", "grasp"),
    "arm_side": ("left", "right", "both", "mirror"),
    "arm_speed": tuple(JOG_SPEED_ORDER),
    "walk_speed": SPEED_ORDER,
}
# Stempel fuer "nur Startwert, alles andere gewinnt" (GUI-Defaults).
DEFAULT_STAMP = 1.0
_EPS = 1e-3


def capped(level, max_level, order=SPEED_ORDER):
    """Tempo-Stufe auf das eigene Maximum begrenzen (unbekannt -> max_level)."""
    if max_level not in order:
        return level if level in order else order[0]
    if level not in order:
        return max_level
    return order[min(order.index(level), order.index(max_level))]


class SharedState:
    """Reine Logik (ohne ROS): set() fuer eigene Aenderungen, merge() fuer
    empfangene Nachrichten, payload() zum Publizieren."""

    def __init__(self, source, clock=time.time):
        self.source = str(source)
        self.clock = clock
        self.values = {}     # Feld -> Wert
        self.stamps = {}     # Feld -> Zeitstempel

    def get(self, field, default=None):
        return self.values.get(field, default)

    def set(self, field, value, stamp=None):
        """Eigene Einstellung/Wahl. -> True, wenn uebernommen (dann publizieren).
        Auch derselbe Wert zaehlt als neue Wahl (z. B. GEHEN nach einem
        NOT-HALT erneut gewaehlt). Unbekannte Felder/Werte: False."""
        if value not in FIELDS.get(field, ()):
            return False
        if stamp is None:
            stamp = max(self.clock(), self.stamps.get(field, 0.0) + _EPS)
        elif stamp <= self.stamps.get(field, 0.0):
            return False      # Startwert, aber es gibt schon einen Stand
        self.values[field] = value
        self.stamps[field] = float(stamp)
        return True

    def merge(self, data):
        """Empfangene Nachricht (dict oder JSON-Text) einarbeiten.
        -> {Feld: Wert} fuer jede NEUERE Wahl (auch mit gleichem Wert -- die
        Empfaenger setzen idempotent). Das eigene Echo und alte latched
        Staende liefern nichts."""
        if isinstance(data, (str, bytes)):
            try:
                data = json.loads(data)
            except (TypeError, ValueError):
                return {}
        if not isinstance(data, dict) or not isinstance(data.get("fields"), dict):
            return {}
        out = {}
        for field, entry in data["fields"].items():
            if field not in FIELDS or not isinstance(entry, dict):
                continue
            value, stamp = entry.get("v"), entry.get("t")
            if value not in FIELDS[field] or not isinstance(stamp, (int, float)):
                continue
            if stamp <= self.stamps.get(field, 0.0):
                continue
            self.stamps[field] = float(stamp)
            self.values[field] = value
            out[field] = value
        return out

    def payload(self):
        return {"source": self.source,
                "fields": {f: {"v": v, "t": self.stamps[f]} for f, v in self.values.items()}}

    def to_json(self):
        return json.dumps(self.payload())

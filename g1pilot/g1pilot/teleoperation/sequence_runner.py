#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sequence_runner.py — einen Ablauf (Liste gespeicherter Posen) abfahren, ohne
Qt/ROS (testbar). Die Demo-GUI haengt ihn an ihre Knoepfe, das Handy und den
Controller (L3 = Weiter, /g1pilot/sequence/next).

Jeder Schritt ist ein POSE ANFAHREN (/g1pilot/pose_store/goto): der
arm_controller plant die Bahn jedes Mal neu (Kollisionsschutz). Der Runner
selbst bewegt nichts -- er entscheidet nur, WANN der naechste Schritt
geschickt wird:

  am Stueck            nach 'reached' (+ Pause wait_s des Schritts) sofort weiter
  Schritt fuer Schritt nach 'reached' warten, bis jemand bestaetigt
                       (GUI-Knopf »Weiter ▶«, Handy, Controller L3)

Den Modus darf man waehrend des Ablaufs umschalten; er gilt ab dem naechsten
Schritt (so haelt man einen Ablauf »am Stueck« an: auf Schritt umschalten).

Abbruch (kein automatisches Weitermachen, auch nicht nach Quittieren):
NOT-HALT, Stopp (GUI/Handy/Controller-Kreuz), Moduswechsel, Hand-Jog waehrend
einer Fahrt, Endzustand 'failed'/'rejected'/'cancelled' der laufenden Fahrt,
keine Rueckmeldung des Arm-Reglers.

Statusfolge des arm_controller fuer EIN goto: (cancelled der alten Fahrt) ->
accepted -> executing -> reached|failed|cancelled, bzw. sofort rejected
(Pose fehlt). Endzustaende VOR dem accepted gehoeren zur alten Fahrt und
werden ignoriert -- frueher brach ein spaetes 'cancelled' den neuen Ablauf ab.
"""
import re
import time

# Zustaende
IDLE = "idle"
SENT = "sent"          # goto geschickt, noch kein accepted/executing
MOVING = "moving"      # Fahrt laeuft
PAUSE = "pause"        # erreicht, Pause wait_s laeuft (am Stueck)
WAITING = "waiting"    # erreicht, wartet auf Bestaetigung (Schritt fuer Schritt)

MODE_AUTO = "auto"
MODE_STEP = "step"
MODE_TITLE = {MODE_AUTO: "Am Stück", MODE_STEP: "Schritt für Schritt"}

# Ohne accepted/executing/rejected innerhalb dieser Zeit: Arm-Regler hat das goto
# ignoriert (z. B. Arme nicht uebernommen) -> abbrechen statt ewig warten.
ACK_TIMEOUT_S = 4.0
# Kleiner Abstand zwischen 'reached' und dem naechsten goto (wie frueher 300 ms).
MIN_GAP_S = 0.3

# Topics: »Weiter« (Bool True, Controller L3 / andere Oberflaechen) und der
# Stand des Ablaufs (JSON snapshot(), latched, von der Demo-GUI).
NEXT_TOPIC = "/g1pilot/sequence/next"
STATE_TOPIC = "/g1pilot/sequence/state"

_START = ("accepted", "executing")
_ABORT = ("failed", "rejected", "cancelled")


def pretty_pose_name(name: str) -> str:
    """'AP1_02_Links_ueber_Tisch' -> 'Links ueber Tisch' (Reihenfolge-Praefix weg)."""
    short = re.sub(r"^[^_]+_\d+_", "", name or "")
    return short.replace("_", " ")


class SequenceRunner:
    """goto(pose): Schritt anfahren (publiziert /g1pilot/pose_store/goto).
    on_change(runner, event, text): Zustand hat sich geaendert -- event ist
    'step' | 'waiting' | 'done' | 'aborted' | 'mode'; text fuer die Statuszeile."""

    def __init__(self, goto, on_change=None, clock=time.monotonic, mode=MODE_AUTO):
        self._goto = goto
        self._on_change = on_change or (lambda *a: None)
        self._clock = clock
        self.mode = mode
        self.state = IDLE
        self.label = None       # Anzeigename (Ablauf) oder None (Einzelpose)
        self.steps = []         # [{"pose", "wait_s"}]
        self.index = -1         # aktueller Schritt (0-basiert)
        self._t = 0.0           # Zeitpunkt fuer SENT-Timeout bzw. Pausenende

    # ── Abfrage ─────────────────────────────────────────────────────────
    @property
    def active(self):
        return self.state != IDLE

    @property
    def moving(self):
        """Arm faehrt (oder soll gleich fahren) -- Jog bricht dann ab."""
        return self.state in (SENT, MOVING)

    @property
    def total(self):
        return len(self.steps)

    @property
    def current_pose(self):
        if self.state == IDLE or not (0 <= self.index < len(self.steps)):
            return None
        return self.steps[self.index]["pose"]

    def prefix(self):
        """'Ablauf X · Schritt 2/5: ' bzw. '' (Einzelpose)."""
        if not self.label:
            return ""
        return f"{self.label} · Schritt {self.index + 1}/{self.total}: "

    def snapshot(self):
        """Kurzstand fuer Topic/Handy: {state, name, step, n, mode, pose}."""
        return {"state": self.state, "name": self.label or "", "step": self.index + 1,
                "n": self.total, "mode": self.mode, "pose": self.current_pose or ""}

    # ── Bedienung ───────────────────────────────────────────────────────
    def start(self, steps, label=None):
        """Ablauf (oder Einzelpose mit label=None) von vorn starten. Ein laufender
        wird ersetzt. -> False bei leerer Liste."""
        clean = []
        for st in steps or []:
            if isinstance(st, str):
                st = {"pose": st}
            if st and st.get("pose"):
                clean.append({"pose": st["pose"], "wait_s": float(st.get("wait_s") or 0.0)})
        if not clean:
            return False
        self.steps, self.label, self.index = clean, label, -1
        self._send_next()
        return True

    def set_mode(self, mode):
        """Modus umschalten; gilt ab dem naechsten Schritt. Wartet der Ablauf
        gerade auf Bestaetigung und wird auf »am Stueck« gestellt, geht es
        NICHT von selbst weiter -- erst nach »Weiter«."""
        if mode not in MODE_TITLE or mode == self.mode:
            return
        self.mode = mode
        if self.state == PAUSE and mode == MODE_STEP:
            self._wait_confirm()
        self._on_change(self, "mode", f"Ablauf-Modus: {MODE_TITLE[mode]}.")

    def confirm(self):
        """»Weiter«: naechster Schritt. Nur wenn der Ablauf darauf wartet.
        -> True, wenn ein Schritt losgeschickt wurde."""
        if self.state != WAITING:
            return False
        self._send_next()
        return True

    def cancel(self, reason=""):
        """Ablauf beenden (NOT-HALT, Stopp, Moduswechsel ...). -> True, wenn einer lief."""
        if self.state == IDLE:
            return False
        was = self.label
        self._reset()
        name = f"Ablauf »{was}«" if was else "Bewegung"
        self._on_change(self, "aborted", f"{name} abgebrochen: {reason}" if reason else "")
        return True

    # ── Rueckmeldungen ──────────────────────────────────────────────────
    def on_status(self, data):
        """arm_command/status (dict). Nur waehrend SENT/MOVING relevant."""
        if self.state not in (SENT, MOVING):
            return
        state = (data or {}).get("state")
        if state in _START:
            self.state = MOVING
            return
        if self.state == SENT and state != "rejected":
            return      # Endzustand der vorigen Fahrt (kommt vor unserem accepted)
        if state == "reached":
            self._reached()
        elif state in _ABORT:
            pose = pretty_pose_name(self.current_pose)
            reason = (data or {}).get("reason", "")
            name = f"Ablauf »{self.label}«" if self.label else f"Pose »{pose}«"
            self._reset()
            self._on_change(self, "aborted",
                            f"{name} abgebrochen ({state}) {reason}".rstrip())

    def tick(self):
        """Regelmaessig aufrufen (GUI-Timer): Pausen und Timeout."""
        now = self._clock()
        if self.state == SENT and now - self._t > ACK_TIMEOUT_S:
            pose = pretty_pose_name(self.current_pose)
            self._reset()
            self._on_change(self, "aborted",
                            f"Keine Rückmeldung vom Arm-Regler zu »{pose}« — "
                            "Arme übernommen (GREIFEN)?")
        elif self.state == PAUSE and now >= self._t:
            self._send_next()

    # ── intern ──────────────────────────────────────────────────────────
    def _reset(self):
        self.state, self.steps, self.index, self.label = IDLE, [], -1, None

    def _reached(self):
        if self.index + 1 >= len(self.steps):
            label = self.label
            self._reset()
            self._on_change(self, "done",
                            f"Ablauf »{label}« fertig." if label else "Pose erreicht.")
            return
        if self.mode == MODE_STEP:
            self._wait_confirm()
            return
        self.state = PAUSE
        self._t = self._clock() + max(MIN_GAP_S, self.steps[self.index]["wait_s"])
        wait = self.steps[self.index]["wait_s"]
        if wait >= 1.0:
            self._on_change(self, "pause", f"{self.prefix()}erreicht — Pause {wait:.0f} s …")

    def _wait_confirm(self):
        self.state = WAITING
        nxt = pretty_pose_name(self.steps[self.index + 1]["pose"])
        self._on_change(self, "waiting",
                        f"{self.prefix()}erreicht. Weiter mit »Weiter ▶« oder L3 "
                        f"(nächster: {nxt}).")

    def _send_next(self):
        self.index += 1
        self.state = SENT
        self._t = self._clock()
        pose = self.steps[self.index]["pose"]
        self._goto(pose)
        self._on_change(self, "step", f"{self.prefix()}{pretty_pose_name(pose)} …")

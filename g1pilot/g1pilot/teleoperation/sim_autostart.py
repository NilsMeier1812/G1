#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sim-Auto-Start (GREIFEN) mit Quittung -- gemeinsam fuer Streamdeck und Demo-GUI.

Problem (Bug 2026-10-13): Die GUIs schickten in der Sim genau EINMAL, 3 s nach
ihrem eigenen Start, /g1pilot/arms/enabled=True + /g1pilot/start_balancing=True.
Die GUI startet aber im selben Launch wie arm_controller und loco_sim; die laden
erst IK-Modell/Planer bzw. die Lauf-Policy und warten auf rt/lowstate der
MuJoCo-Sim. Waren ihre Subscriber nach 3 s noch nicht da (langsamer Rechner,
colcon-Build, Sim startet spaet), gingen beide Nachrichten verloren (volatile
QoS). Die GUI zeigte trotzdem GREIFEN an, die Arme blieben aus -> weder GUI noch
RViz-Marker noch PS4 bewegten etwas. Erst GEHEN (aktiviert die Arme in der Sim
selbst) und zurueck half.

Loesung: Der Auto-Start wiederholt die Anforderung, bis BEIDE Regler sie
bestaetigen -- arm_controller ueber /g1pilot/arms/state (Bool, latched) und
loco_sim ueber /g1pilot/loco_state (String, latched: hold|stand|walk|damp).
Wiederholt wird nur, was noch fehlt; danach ist der Auto-Start fertig und
sendet nichts mehr. Ein Eingriff des Nutzers (Modus, NOT-HALT, ENABLE-Knopf)
bricht ihn ab. NUR SIM -- die Aufrufer starten ihn nur bei is_sim_mode().

Die Logik hier ist ROS-frei (Uhr/Sender injiziert) und dadurch testbar.
"""
import time

# Topics der Quittung (latched, TRANSIENT_LOCAL).
ARMS_STATE_TOPIC = "/g1pilot/arms/state"
LOCO_STATE_TOPIC = "/g1pilot/loco_state"

# loco_sim-Zustaende, in denen der Roboter aufgestellt ist (GREIFEN moeglich).
LOCO_UP = ("stand", "walk")


class SimAutoStart:
    """Schickt ENABLE + BALANCING, bis arm_controller und loco_sim bestaetigen.

    send_enable(), send_balance(): publizieren je einmal True.
    tick() regelmaessig aufrufen (z.B. 10 Hz aus einem Qt-Timer) mit dem
    zuletzt empfangenen Zustand (None = noch nichts empfangen).
    on_done(ok: bool): einmalig, ok=False nach timeout_s ohne Quittung.
    """

    def __init__(self, send_enable, send_balance, period_s=1.5, timeout_s=120.0,
                 clock=time.monotonic, log=None, on_done=None):
        self._send_enable = send_enable
        self._send_balance = send_balance
        self.period_s = float(period_s)
        self.timeout_s = float(timeout_s)
        self._clock = clock
        self._log = log or (lambda _msg: None)
        self._on_done = on_done
        self.active = False
        self.sent_enable = 0          # Anzahl gesendeter Anforderungen (Diagnose/Test)
        self.sent_balance = 0
        self._t0 = 0.0
        self._last_send = None

    def start(self):
        """Ersten Versuch sofort senden, danach bis zur Quittung wiederholen."""
        self.active = True
        self._t0 = self._clock()
        self._last_send = None
        self.sent_enable = self.sent_balance = 0

    def cancel(self, reason=""):
        if self.active:
            self.active = False
            self._log(f"Auto-Start abgebrochen ({reason})." if reason
                      else "Auto-Start abgebrochen.")

    def tick(self, arms_enabled, loco_state):
        if not self.active:
            return
        arms_ok = arms_enabled is True
        loco_ok = loco_state in LOCO_UP
        now = self._clock()
        if arms_ok and loco_ok:
            self.active = False
            self._log(f"Auto-Start bestaetigt nach {now - self._t0:.1f} s "
                      f"(ENABLE x{self.sent_enable}, BALANCING x{self.sent_balance}).")
            if self._on_done:
                self._on_done(True)
            return
        if now - self._t0 > self.timeout_s:
            self.active = False
            missing = ", ".join(n for n, ok in (("arm_controller", arms_ok),
                                                ("loco_sim", loco_ok)) if not ok)
            self._log(f"Auto-Start: keine Quittung von {missing} nach "
                      f"{self.timeout_s:.0f} s -- bitte GREIFEN erneut waehlen.")
            if self._on_done:
                self._on_done(False)
            return
        if self._last_send is not None and now - self._last_send < self.period_s:
            return
        if self._last_send is not None:
            self._log("Auto-Start: noch keine Quittung von "
                      + ("arm_controller" if not arms_ok else "")
                      + (" + " if not arms_ok and not loco_ok else "")
                      + ("loco_sim" if not loco_ok else "")
                      + " -- sende erneut.")
        self._last_send = now
        # Reihenfolge wie bisher: erst Arme, dann Stand.
        if not arms_ok:
            self._send_enable()
            self.sent_enable += 1
        if not loco_ok:
            self._send_balance()
            self.sent_balance += 1

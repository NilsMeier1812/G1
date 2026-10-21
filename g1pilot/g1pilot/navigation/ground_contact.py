#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ground_contact — Bodenkontakt des echten G1 aus der Knie-Last (reine Logik).

Hintergrund (Real-Test Nils 2026-10-21): GetFsmMode (API 7002) meldet in
Preparation (FSM 4) immer 0, auch wenn der G1 ~10 cm ueber dem Boden am Kran
haengt. Die alte Pruefung "FSM-Mode 0 = Fuesse belastet" war damit wirkungslos.

Messgroesse: geschaetztes Motormoment (rt/lowstate, motor_state[].tau_est) der
beiden Kniegelenke. Steht der G1, traegt jedes Knie das halbe Koerpergewicht
ueber den gebeugten Unterschenkel; haengt er, nur Unterschenkel + Fuss.
MuJoCo-Abschaetzung (G1 29 DoF, 35 kg, Knie 0.3 rad, PD-gehalten, Probe
2026-10-21): Boden 6.6 Nm je Knie, Kran traegt 30 % -> 3.7, 50 % -> 2.0,
70 % -> 0.4, frei haengend 0.0. Real-Test Nils 2026-10-21: frei stehend
~14 Nm je Knie -> Schwelle knee_load_min_nm Default 10.0 (Parameter, die GUI
zeigt die Werte).

Beide Knie muessen die Schwelle erreichen (eine Fussspitze am Boden reicht
nicht). Werte werden geglaettet (Zeitkonstante tau_s); ohne frische Daten
(> stale_s) gilt "nicht messbar" = kein Bodenkontakt.
"""
import math
import threading
import time

# Gelenk-Indizes im G1-LowState (29 und 23 DoF gleich): linkes / rechtes Knie.
LEFT_KNEE, RIGHT_KNEE = 3, 9

CHECK_MODES = ("knee_load", "fsm_mode", "none")


class GroundContact:
    def __init__(self, min_nm=10.0, tau_s=0.2, stale_s=0.5, clock=time.monotonic):
        self.min_nm = float(min_nm)
        self.tau_s = max(1e-3, float(tau_s))
        self.stale_s = float(stale_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._load = None          # geglaettet (links, rechts) in Nm, Betrag
        self._t = None

    def feed(self, tau_left, tau_right, now=None):
        """Neue Knie-Momente (Nm, Vorzeichen egal)."""
        now = self._clock() if now is None else now
        raw = (abs(float(tau_left)), abs(float(tau_right)))
        if not all(math.isfinite(v) for v in raw):
            return
        with self._lock:
            if self._load is None or self._t is None or now - self._t > self.stale_s:
                self._load = raw
            else:
                a = 1.0 - math.exp(-max(0.0, now - self._t) / self.tau_s)
                self._load = tuple(o + a * (r - o) for o, r in zip(self._load, raw))
            self._t = now

    def feed_lowstate(self, msg):
        """unitree_hg LowState_ (rt/lowstate)."""
        ms = msg.motor_state
        self.feed(ms[LEFT_KNEE].tau_est, ms[RIGHT_KNEE].tau_est)

    def load(self, now=None):
        """(links, rechts) in Nm oder None (keine frischen Daten)."""
        now = self._clock() if now is None else now
        with self._lock:
            if self._load is None or self._t is None or now - self._t > self.stale_s:
                return None
            return self._load

    def check(self, now=None):
        """-> (ok, Grund). ok False auch ohne Messwerte."""
        load = self.load(now)
        if load is None:
            return False, "Knie-Last nicht messbar (keine rt/lowstate)"
        left, right = load
        if min(left, right) >= self.min_nm:
            return True, f"Knie-Last {left:.1f}/{right:.1f} Nm"
        return False, (f"Knie-Last {left:.1f}/{right:.1f} Nm "
                       f"< {self.min_nm:.1f} Nm (Füße ohne Last)")

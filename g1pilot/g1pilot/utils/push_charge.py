# -*- coding: utf-8 -*-
"""Aufladbarer Schubs (nur Sim): Haltedauer des PUSH-Buttons -> Ladegrad -> Kraft.

Kurzer Klick (Haltedauer <= TAP_S) = Ladegrad 0 = bisheriger Schubs (SIM_PUSH_FORCE_N,
Default 150 N). Danach waechst der Ladegrad linear bis 1.0 nach CHARGE_S; Ladegrad 1.0
entspricht MAX_N. Die Kraft wirkt weiter PUSH_DURATION_S (0.12 s) -> Impuls = F * 0.12 s.

Reines Python (kein Qt/ROS), damit GUI (Anzeige) und loco_sim (Umrechnung auf Newton)
dieselbe Formel benutzen und sie ohne Roboter testbar ist. Einstellbar per Umgebung:
    SIM_PUSH_FORCE_N  Kraft bei Ladegrad 0 [N]   (Default 150, wie push_listener/config)
    G1_PUSH_MAX_N     Kraft bei Ladegrad 1 [N]   (Default 500)
    G1_PUSH_CHARGE_S  Haltedauer bis voll [s]    (Default 3.0)
    G1_PUSH_TAP_S     Klick bis zu dieser Dauer = kein Aufladen [s] (Default 0.3)
"""
import os
import time


def _env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


BASE_N = _env_float("SIM_PUSH_FORCE_N", 150.0)
MAX_N = _env_float("G1_PUSH_MAX_N", 500.0)
CHARGE_S = _env_float("G1_PUSH_CHARGE_S", 3.0)
TAP_S = _env_float("G1_PUSH_TAP_S", 0.3)
PUSH_DURATION_S = _env_float("SIM_PUSH_DURATION_S", 0.12)


def charge_level(hold_s, tap_s=None, charge_s=None):
    """Haltedauer [s] -> Ladegrad 0..1 (0 bis tap_s, dann linear bis charge_s)."""
    tap_s = TAP_S if tap_s is None else tap_s
    charge_s = CHARGE_S if charge_s is None else charge_s
    if hold_s <= tap_s:
        return 0.0
    span = max(charge_s - tap_s, 1e-6)
    return min(1.0, (hold_s - tap_s) / span)


def force_for_level(level, base_n=None, max_n=None):
    """Ladegrad 0..1 -> Kraft [N] (linear von base_n bis max_n; max_n >= base_n)."""
    base_n = BASE_N if base_n is None else base_n
    max_n = MAX_N if max_n is None else max_n
    level = min(1.0, max(0.0, float(level)))
    return base_n + level * (max(max_n, base_n) - base_n)


def impulse_for_force(force_n, duration_s=None):
    """Kraftstoss [N*s] bei Kraft force_n und der Push-Dauer."""
    return force_n * (PUSH_DURATION_S if duration_s is None else duration_s)


class PushCharger:
    """Haelt den Zustand 'Button gedrueckt seit ...'. Zeit wird injiziert (testbar)."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._t0 = None

    @property
    def holding(self):
        return self._t0 is not None

    def press(self):
        self._t0 = self._clock()

    def level(self):
        """Aktueller Ladegrad (0 wenn nicht gehalten)."""
        if self._t0 is None:
            return 0.0
        return charge_level(self._clock() - self._t0)

    def release(self):
        """Loslassen -> Ladegrad zum Zeitpunkt des Loslassens (0 ohne vorheriges press)."""
        lvl = self.level()
        self._t0 = None
        return lvl

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hueftdrehung: Sollwert fuer den Taillen-Yaw (Gelenk 12) per Controller/GUI.

Nils 2026-10-21: am Tisch den Oberkoerper ein Stueck nach links/rechts drehen
(R2 halten + rechter Stick, R2 + R3 = zurueck zur Mitte). Beim Wechsel nach
GEHEN, bei der Grundstellung und vor der Rueckgabe an den Onboard-Regler faehrt
die Huefte erst sanft zur Mitte.

Dieses Modul rechnet nur den Sollwert (ohne ROS testbar). Der arm_controller
haelt ihn: real ueber WaistHold (rt/arm_sdk), in der Sim setzt ihn loco_sim im
Stand (/g1pilot/waist/state, target_rad).

  * ``set_rate(r, now)``: Stick-Ausschlag -1..1 (+ = nach links drehen), wird
    mit ``max_rate`` [rad/s] integriert. Bleibt der Strom ``rate_timeout_s``
    aus (Taste los ohne 0, Node tot), gilt 0 -- Totmann wie beim Gehen.
  * ``center()``: mit ``center_rate`` zur Mitte (``home``) fahren.
  * ``step(dt, now)``: einen Takt weiter -> Sollwert [rad].
  * Bereich: ``home +- limit``, zusaetzlich die Gelenkgrenze.
"""
from __future__ import annotations

import math
from typing import Optional

# Gelenk und Gelenkgrenze waist_yaw (rad), wie WAIST_LIMITS[12] in waist_hold.py.
YAW_JOINT = 12
YAW_JOINT_LIMIT = 2.618

# Topics. Rate: std_msgs/Float32 -1..1 (+ = links), solange gedreht wird mit
# ~30 Hz; Mitte: std_msgs/Bool True (Impuls); Zustand: std_msgs/String (JSON,
# 10 Hz, arm_controller), Felder siehe arm_controller._publish_waist_state.
WAIST_RATE_TOPIC = "/g1pilot/waist/yaw_rate"
WAIST_CENTER_TOPIC = "/g1pilot/waist/center"
WAIST_STATE_TOPIC = "/g1pilot/waist/state"
# Aelter als das: Zustand unbekannt (arm_controller laeuft nicht).
STATE_STALE_S = 1.0


def parse_state(text):
    """JSON des Zustands -> dict (oder None bei Murks)."""
    import json
    try:
        st = json.loads(text)
    except (TypeError, ValueError):
        return None
    return st if isinstance(st, dict) else None


def rotate_twist_yaw(lin, ang, yaw):
    """Jog-Bewegung aus dem Oberkoerper-Bezug (um den Taillen-Yaw gedreht) in
    den Becken-Frame (JOG_FRAME): beide Vektoren um z drehen. yaw 0 -> gleich."""
    if not yaw:
        return tuple(lin), tuple(ang)
    c, s = math.cos(yaw), math.sin(yaw)
    rot = lambda v: (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])   # noqa: E731
    return rot(lin), rot(ang)


class WaistYaw:
    def __init__(self, limit: float = math.radians(30.0),
                 max_rate: float = math.radians(15.0),
                 center_rate: float = math.radians(15.0),
                 rate_timeout_s: float = 0.3,
                 center_tol: float = math.radians(2.0)):
        self.limit = abs(float(limit))
        self.max_rate = abs(float(max_rate))
        self.center_rate = abs(float(center_rate))
        self.rate_timeout_s = float(rate_timeout_s)
        self.center_tol = abs(float(center_tol))
        self.home = 0.0
        self.target: Optional[float] = None
        self.centering = False
        self._rate = 0.0
        self._rate_t: Optional[float] = None

    # ── Eingaben ────────────────────────────────────────────────────────
    def clip(self, yaw: float) -> float:
        lo = max(-YAW_JOINT_LIMIT, self.home - self.limit)
        hi = min(YAW_JOINT_LIMIT, self.home + self.limit)
        return min(max(float(yaw), lo), hi)

    def reset(self, yaw: float, home: Optional[float] = None) -> None:
        """Sollwert auf eine (gemessene) Stellung setzen, ohne Bewegung.
        Kein Clip: steht die Taille ausserhalb, faehrt sie nicht von allein."""
        if home is not None:
            self.home = float(home)
        self.target = float(yaw)
        self.centering = False
        self.stop()

    def stop(self) -> None:
        """Drehen sofort beenden (NOT-HALT, Kreuz): Rate 0, keine Mitte-Fahrt."""
        self._rate = 0.0
        self._rate_t = None
        self.centering = False

    def set_rate(self, rate: float, now: float) -> None:
        """Stick -1..1 (+ = links). Ein Ausschlag bricht eine Mitte-Fahrt ab."""
        r = float(rate)
        # Erst auf NaN pruefen: min/max liessen NaN als +-1 durch.
        r = max(-1.0, min(1.0, r)) if math.isfinite(r) else 0.0
        self._rate = r * self.max_rate
        self._rate_t = float(now)
        if abs(r) > 1e-6:
            self.centering = False

    def center(self) -> None:
        if self.target is not None and not self.at_home:
            self.centering = True

    # ── Zustand ─────────────────────────────────────────────────────────
    def rate(self, now: float) -> float:
        if self._rate_t is None or now - self._rate_t > self.rate_timeout_s:
            return 0.0
        return self._rate

    @property
    def offset(self) -> float:
        return 0.0 if self.target is None else self.target - self.home

    @property
    def at_home(self) -> bool:
        """Sollwert in der Mitte (Toleranz center_tol)."""
        return self.target is None or abs(self.offset) <= self.center_tol

    def moving(self, now: float) -> bool:
        """Dreht gerade (Stick ausgelenkt oder Mitte-Fahrt)."""
        return self.centering or abs(self.rate(now)) > 1e-9

    # ── Takt ────────────────────────────────────────────────────────────
    def step(self, dt: float, now: float) -> Optional[float]:
        if self.target is None:
            return None
        dt = max(0.0, min(0.1, float(dt)))
        if self.centering:
            err = self.home - self.target
            step = self.center_rate * dt
            if abs(err) <= step or self.center_rate <= 0.0:
                self.target = self.home
                self.centering = False
            else:
                self.target += math.copysign(step, err)
            return self.target
        r = self.rate(now)
        if r != 0.0:
            new = self.target + r * dt
            if self.clip(new) == new:
                self.target = new
            elif abs(new - self.home) < abs(self.target - self.home):
                # Ausserhalb des Bereichs (z. B. Messung beim Reset): zurueck
                # Richtung Bereich ist erlaubt ...
                self.target = new
            elif self.clip(self.target) == self.target:
                # ... hinaus nur bis an den Rand.
                self.target = self.clip(new)
        return self.target

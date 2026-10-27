#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Taille (Waist yaw/roll/pitch, Gelenke 12-14) halten, solange arm_sdk aktiv ist.

Warum das noetig ist: Das arm_sdk-Gewicht (motor_cmd[29].q) blendet beim G1
NICHT nur die 14 Armgelenke vom Onboard-Regler auf rt/arm_sdk um, sondern auch
die Taille. In den offiziellen Unitree-Beispielen stehen WaistYaw/Roll/Pitch
deshalb in der arm_joints-Liste (g1_arm7_sdk_dds_example.py: kp=60/kd=1.5,
unitree_ros2 g1_arm_sdk_dds_example.cpp: Taille kp=240/kd=6). Schreibt man dort
kp=0/kd=0, ist die Taille bei Gewicht 1 kraftlos: der Oberkoerper (umgekehrtes
Pendel ueber der Taille) kippt unkontrolliert nach vorn/hinten.

Dieses Modul rechnet nur das Kommando (q, kp, kd) fuer die Taillengelenke; der
arm_controller schreibt es in jede rt/arm_sdk-Nachricht. Ohne ROS testbar.

Zustand:
  * Ziel (``target``): die Taillenstellung, die der Onboard-Regler beim ENABLE
    (Gewicht noch 0) hielt -> beim Hochrampen des Gewichts kein Sprung.
  * Kommando (``q_cmd``): laeuft mit hoechstens ``max_vel`` aufs Ziel zu.
  * Gain-Skala (``scale`` 0..1): 0 = passiv (kp=0, nur Daempfung, E-Stop),
    1 = volle Haltung. Hochfahren dauert ``engage_s`` (weiches Wiederzupacken
    nach dem Quittieren eines E-Stops).
"""
from __future__ import annotations

from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

# Gelenkgrenzen (rad) aus joints_names.JOINT_LIMITS_RAD, hier dupliziert, damit
# das Modul ohne ROS-Paket importierbar bleibt.
WAIST_LIMITS = {12: (-2.618, 2.618), 13: (-0.52, 0.52), 14: (-0.52, 0.52)}


class WaistHold:
    def __init__(self, joints: Iterable[int] = (12, 13, 14), kp: float = 150.0,
                 kd: float = 4.0, passive_kd: float = 3.0, max_vel: float = 0.3,
                 engage_s: float = 1.5):
        self.joints = [int(j) for j in joints]
        for j in self.joints:
            if j not in WAIST_LIMITS:
                raise ValueError(f"Kein Taillengelenk: {j} (erlaubt: 12, 13, 14)")
        self.kp = float(kp)
        self.kd = float(kd)
        self.passive_kd = float(passive_kd)
        self.max_vel = float(max_vel)
        self.engage_s = float(engage_s)
        self.target: Optional[np.ndarray] = None
        self.q_cmd: Optional[np.ndarray] = None
        self.scale = 1.0
        self._scale_goal = 1.0

    # ── Eingaben ────────────────────────────────────────────────────────
    def clip(self, q: Sequence[float]) -> np.ndarray:
        q = np.asarray(q, dtype=float).copy()
        for k, j in enumerate(self.joints):
            lo, hi = WAIST_LIMITS[j]
            q[k] = min(max(q[k], lo), hi)
        return q

    def set_target(self, q_measured: Sequence[float]) -> None:
        """Ziel = gemessene Taillenstellung (z. B. beim ENABLE, Gewicht 0)."""
        self.target = self.clip(q_measured)
        if self.q_cmd is None:
            self.q_cmd = self.target.copy()

    def ensure_target(self, q_measured: Sequence[float]) -> None:
        """Noch kein Ziel (z. B. E-Stop vor dem ersten ENABLE): aktuelle Pose."""
        if self.target is None:
            self.set_target(q_measured)

    def go_passive(self) -> None:
        """E-Stop: sofort kp=0 (nur Daempfung), wie die Arme."""
        self.scale = 0.0
        self._scale_goal = 0.0

    def engage(self, q_measured: Sequence[float]) -> None:
        """Weich wieder zupacken: Kommando startet an der GEMESSENEN Stellung
        (kein Ruck), kp rampt 0->kp, dann faehrt das Kommando langsam aufs Ziel."""
        if self._scale_goal >= 1.0 and self.scale >= 1.0:
            return
        self.q_cmd = self.clip(q_measured)
        self.ensure_target(q_measured)
        self._scale_goal = 1.0

    @property
    def passive(self) -> bool:
        return self._scale_goal <= 0.0

    # ── Ausgabe ─────────────────────────────────────────────────────────
    def step(self, dt: float, q_measured: Sequence[float]) -> Tuple[np.ndarray, float, float]:
        """Einen Takt weiter: -> (q_cmd, kp, kd) fuer die Taillengelenke."""
        self.ensure_target(q_measured)
        dt = max(0.0, float(dt))
        if self.q_cmd is None:
            self.q_cmd = self.target.copy()
        if self.passive:
            # Passiv: Kommando folgt der Messung (bei kp=0 ohne Wirkung, aber
            # sauber fuer ein spaeteres Wiederzupacken).
            self.q_cmd = self.clip(q_measured)
            return self.q_cmd.copy(), 0.0, self.passive_kd
        if self.scale < 1.0:
            self.scale = 1.0 if self.engage_s <= 0.0 else min(1.0, self.scale + dt / self.engage_s)
        err = self.target - self.q_cmd
        m = float(np.max(np.abs(err))) if err.size else 0.0
        if m > 1e-12:
            step = self.max_vel * dt
            self.q_cmd = self.q_cmd + err * min(1.0, step / m) if self.max_vel > 0.0 else self.target.copy()
        kd = self.passive_kd + (self.kd - self.passive_kd) * self.scale
        return self.q_cmd.copy(), self.kp * self.scale, kd


# ── Taillen-Modus beim Halten (Nils 2026-10-27, Linksdrall-Analyse) ─────────
#   hold     altes Verhalten: Ziel = Taillenstellung beim ENABLE (eingefroren)
#   zero     Ziel = 0 (Yaw/Roll/Pitch gerade), langsam angefahren -- ein beim
#            ENABLE schief stehender Oberkoerper bleibt nicht schief
#   onboard  wie zero; WAEHREND DES GEHENS reicht der arm_controller die
#            Taillen-Sollwerte des Onboard-Reglers (rt/lowcmd) durch, damit die
#            Lauf-Policy (FSM 501 "3Dof-waist") die Taille selbst nutzt.
#            Ohne brauchbare rt/lowcmd-Daten: Rueckfall auf das Halteziel (0).
WAIST_MODES = ("hold", "zero", "onboard")
# Weiter als das steht die Taille beim ENABLE von 0 weg: NICHT auf 0 fahren
# (z. B. absichtlich gedreht/gebeugt) -- dann wie "hold".
ZERO_MAX_OFFSET = 0.3


def parse_waist_mode(text) -> str:
    m = str(text or "").strip().lower()
    return m if m in WAIST_MODES else "hold"


def enable_target(mode: str, q_measured: Sequence[float],
                  max_offset: float = ZERO_MAX_OFFSET) -> Tuple[np.ndarray, str]:
    """Halteziel beim ENABLE -> (Ziel, Hinweis). "zero"/"onboard": 0, solange
    keine Taillenachse weiter als max_offset davon weg steht; sonst Messung."""
    q = np.asarray(q_measured, dtype=float)
    if mode not in ("zero", "onboard"):
        return q.copy(), ""
    off = float(np.max(np.abs(q))) if q.size else 0.0
    if off > max_offset:
        return q.copy(), (f"Taille {np.degrees(off):.0f} deg aus der Mitte (> "
                          f"{np.degrees(max_offset):.0f}) -- bleibt, wo sie ist")
    return np.zeros_like(q), ""


class WaistPassthrough:
    """Reicht beim Gehen die Taillen-Sollwerte des Onboard-Reglers durch
    (Modus "onboard"). Ohne ROS/DDS testbar; feed() kommt aus einem Lese-
    Thread (rt/lowcmd), alles andere aus dem Regeltakt.

    Gueltig ist ein Sollwert nur, wenn er frisch ist (stale_s), kp > 0 hat und
    NICHT unser eigenes Kommando ist: zeigt rt/lowcmd das schon gemischte
    Kommando (Gewicht 1 -> unsere Werte), waere Durchreichen ein Kreis. Das
    wird geprueft, solange wir NICHT durchreichen (Ausgang = Halteziel): stimmen
    q und kp mehrmals mit dem zuletzt Gesendeten ueberein -> "echo", dauerhaft
    aus; erst nach ECHO_HITS abweichenden Samples gilt die Quelle als geprueft
    ("verified") und wird ueberhaupt durchgereicht. Uebergaenge blendet
    `alpha` ueber blend_s (kein Sprung)."""

    ECHO_HITS = 5

    def __init__(self, n: int = 3, stale_s: float = 0.1, blend_s: float = 0.5,
                 kp_max: float = 250.0, kd_max: float = 6.0, tau_max: float = 25.0):
        import threading
        self.n = int(n)
        self.stale_s = float(stale_s)
        self.blend_s = float(blend_s)
        self.kp_max, self.kd_max, self.tau_max = float(kp_max), float(kd_max), float(tau_max)
        self.alpha = 0.0
        self.echo = False
        self.verified = False
        self._echo_hits = 0
        self._clean_hits = 0
        self._lock = threading.Lock()
        self._sample = None            # (t, q, kp, kd, tau)
        self._sent = None              # (q, kp) zuletzt gesendet
        self.samples = 0

    def feed(self, q, kp, kd, tau, now: float) -> None:
        s = (float(now), np.asarray(q, float), np.asarray(kp, float),
             np.asarray(kd, float), np.asarray(tau, float))
        with self._lock:
            self._sample = s
            self.samples += 1
            sent = self._sent
            if sent is not None and self.alpha <= 0.0 and not self.echo:
                same = (s[1].shape == sent[0].shape and s[2].shape == sent[1].shape
                        and np.allclose(s[1], sent[0], atol=1e-6)
                        and np.allclose(s[2], sent[1], atol=1e-6))
                if same:
                    self._echo_hits += 1
                    self._clean_hits = 0
                    if self._echo_hits >= self.ECHO_HITS:
                        self.echo = True
                else:
                    self._echo_hits = 0
                    self._clean_hits += 1
                    if self._clean_hits >= self.ECHO_HITS:
                        self.verified = True

    def note_sent(self, q, kp) -> None:
        with self._lock:
            self._sent = (np.asarray(q, float).copy(), np.asarray(kp, float).copy())

    def sample(self, now: float):
        """Gueltiger Onboard-Sollwert (q, kp, kd, tau) oder None."""
        with self._lock:
            s = self._sample
        if s is None or self.echo or not self.verified or now - s[0] > self.stale_s:
            return None
        if s[1].size != self.n or not np.all(np.isfinite(s[1])) or float(np.min(s[2])) <= 0.0:
            return None
        return (s[1], np.minimum(s[2], self.kp_max), np.clip(s[3], 0.0, self.kd_max),
                np.clip(s[4], -self.tau_max, self.tau_max))

    def reason(self, now: float) -> str:
        """Warum gerade nicht durchgereicht wird ('' = es wird)."""
        with self._lock:
            s = self._sample
        if self.echo:
            return "rt/lowcmd zeigt unser eigenes Kommando (bereits gemischt)"
        if s is None:
            return "keine rt/lowcmd-Daten vom Onboard-Regler"
        if now - s[0] > self.stale_s:
            return "rt/lowcmd veraltet"
        if not self.verified:
            return "rt/lowcmd noch nicht geprueft (Echo-Test laeuft)"
        if float(np.min(s[2])) <= 0.0:
            return "Onboard-Regler kommandiert die Taille nicht (kp 0)"
        return ""

    def step(self, dt: float, want: bool, now: float):
        """Einen Takt weiter: want = Gehen + Modus onboard. -> Onboard-Sollwert
        oder None; alpha laeuft dabei Richtung 1 (gueltig) bzw. 0."""
        smp = self.sample(now) if want else None
        rate = 1.0 if self.blend_s <= 0.0 else max(0.0, float(dt)) / self.blend_s
        goal = 1.0 if smp is not None else 0.0
        if self.alpha < goal:
            self.alpha = min(goal, self.alpha + rate)
        elif self.alpha > goal:
            self.alpha = max(goal, self.alpha - rate)
        if smp is not None:
            self._last = smp
        elif self.alpha <= 0.0:
            self._last = None
        return getattr(self, "_last", None)

    def blend(self, hold, onboard):
        """hold/onboard: (q, kp, kd, tau) -> gemischt nach alpha."""
        if onboard is None or self.alpha <= 0.0:
            return hold
        a = self.alpha
        return tuple((1.0 - a) * np.asarray(h, float) + a * np.asarray(o, float)
                     for h, o in zip(hold, onboard))

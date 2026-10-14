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

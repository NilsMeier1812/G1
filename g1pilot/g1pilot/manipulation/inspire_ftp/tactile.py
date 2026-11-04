#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tactile.py — Taktil-/Kraft-Zonenlayout der Inspire RH56DFTP-2
=============================================================
Uebernommen aus dem urspruenglichen inspire_hand_bridge.py (Viewer). Beschreibt
die 17 Sensor-Zonen je Hand (Name, Finger, Zone, Modbus-Reg-Adresse, rows, cols).

Diese 17 Zonen sind die EINZIGE Wahrheit fuer alle Backends:
  * Stufe 3 (echte Hand): liest sie per Modbus (reg-Adressen 3000..5123).
  * Stufe 2 (MuJoCo-Sim): das Modell g1_29dof_inspire_ftp traegt genau diese 17
    <touch>-Sensoren JE HAND (an den *_force_sensor_*-Frames des URDF). Der
    MujocoContactBackend fuellt daraus echte, physikbasierte Kontaktkraefte in
    dieselben Zonen -> die Demo-GUI zeigt in Sim UND real identische Zonen.
  * Stufe 1 (RViz-only): keine Kontaktphysik -> ``zero_zones()`` (Platzhalter).
"""

from __future__ import annotations
from typing import Dict, List

# (zone_id, finger, zone, reg_addr, rows, cols) — reg_addr nur fuer echte Hardware.
TACTILE_ZONES = [
    ("kleiner_tip",  "Kleiner", "tip",  3000,  3,  3),
    ("kleiner_nail", "Kleiner", "nail", 3018, 12,  8),
    ("kleiner_pad",  "Kleiner", "pad",  3210, 10,  8),
    ("ring_tip",     "Ring",    "tip",  3370,  3,  3),
    ("ring_nail",    "Ring",    "nail", 3388, 12,  8),
    ("ring_pad",     "Ring",    "pad",  3580, 10,  8),
    ("mittel_tip",   "Mittel",  "tip",  3740,  3,  3),
    ("mittel_nail",  "Mittel",  "nail", 3758, 12,  8),
    ("mittel_pad",   "Mittel",  "pad",  3950, 10,  8),
    ("zeige_tip",    "Zeige",   "tip",  4110,  3,  3),
    ("zeige_nail",   "Zeige",   "nail", 4128, 12,  8),
    ("zeige_pad",    "Zeige",   "pad",  4320, 10,  8),
    ("daumen_tip",   "Daumen",  "tip",  4480,  3,  3),
    ("daumen_nail",  "Daumen",  "nail", 4498, 12,  8),
    ("daumen_mid",   "Daumen",  "mid",  4690,  3,  3),
    ("daumen_pad",   "Daumen",  "pad",  4708, 12,  8),
    ("palme",        "Palme",   "palm", 4900,  8, 14),
]


def zero_zones() -> Dict[str, List[int]]:
    """Leere (Null-)Zonen in der richtigen Form — Startwert (Stufe 1 / kein Kontakt)."""
    return {z[0]: [0] * (z[4] * z[5]) for z in TACTILE_ZONES}


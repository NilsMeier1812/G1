#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drift-Werkzeug (Stufe 0): Endpose im Start-Bezug wie mit dem Massband."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "tools"))
import odom_drift_log as odl   # noqa: E402


def test_start_frame_forward_and_left():
    # Start bei (1, 2) mit Blick nach +y (90 deg): 4 m in +y = 4 m vor.
    x, y, a = odl.in_start_frame((1.0, 2.0, math.pi / 2), (1.0, 6.0, math.pi / 2))
    assert (x, y, a) == pytest.approx((4.0, 0.0, 0.0), abs=1e-9)
    # 1 m nach rechts (vom Roboter aus) = -1 m links.
    x, y, _ = odl.in_start_frame((1.0, 2.0, math.pi / 2), (2.0, 6.0, 0.0))
    assert (x, y) == pytest.approx((4.0, -1.0), abs=1e-9)


def test_track_length_and_turn():
    tr = odl.Track("t")
    assert tr.rel() is None and "keine Daten" in tr.line()
    for i in range(5):
        tr.add(i * 1.0, 0.0, 0.0)
    tr.add(4.0, -1.0, -math.pi / 2)
    x, y, a = tr.rel()
    assert tr.length == pytest.approx(5.0)
    assert (x, y) == pytest.approx((4.0, -1.0)) and math.degrees(a) == pytest.approx(-90.0)
    assert "Strecke  5.00 m" in tr.line()


def test_yaw_wraps():
    assert odl.wrap(3 * math.pi) == pytest.approx(math.pi)
    _, _, a = odl.in_start_frame((0, 0, math.radians(170)), (0, 0, math.radians(-170)))
    assert math.degrees(a) == pytest.approx(20.0)

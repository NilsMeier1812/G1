#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hueftdrehung: Sollwert (waist_yaw.WaistYaw) und GUI-Text -- stdlib only."""
import math

import pytest

from g1pilot.manipulation.waist_yaw import WaistYaw, parse_state, YAW_JOINT_LIMIT

D = math.radians


def _run(wy, rate, t0, t1, dt=0.01):
    t = t0
    while t < t1 - 1e-9:
        if rate is not None:
            wy.set_rate(rate, t)
        wy.step(dt, t)
        t += dt
    return t


def test_rate_integrates_with_max_rate_and_limit():
    wy = WaistYaw(limit=D(30), max_rate=D(15))
    wy.reset(0.0, home=0.0)
    _run(wy, 1.0, 0.0, 1.0)
    assert wy.target == pytest.approx(D(15), abs=D(0.3))
    _run(wy, 1.0, 1.0, 4.0)
    assert wy.target == pytest.approx(D(30))                  # Grenze
    _run(wy, -0.5, 4.0, 5.0)
    assert wy.target == pytest.approx(D(22.5), abs=D(0.3))


def test_limit_around_home_and_joint_limit():
    wy = WaistYaw(limit=D(30))
    wy.reset(0.1, home=0.1)
    _run(wy, -1.0, 0.0, 6.0)
    assert wy.target == pytest.approx(0.1 - D(30))
    wy = WaistYaw(limit=10.0, max_rate=10.0)
    wy.reset(0.0)
    _run(wy, 1.0, 0.0, 1.0)
    assert wy.target == pytest.approx(YAW_JOINT_LIMIT)


def test_rate_timeout_is_deadman():
    wy = WaistYaw(max_rate=D(15), rate_timeout_s=0.3)
    wy.reset(0.0)
    wy.set_rate(1.0, 0.0)
    _run(wy, None, 0.0, 2.0)                                  # kein Strom mehr
    assert wy.target == pytest.approx(D(15) * 0.3, abs=D(0.3))
    assert not wy.moving(2.0)


def test_center_ramps_back_and_rate_aborts_centering():
    wy = WaistYaw(center_rate=D(15))
    wy.reset(D(20), home=0.0)
    assert not wy.at_home
    wy.center()
    assert wy.moving(0.0)
    _run(wy, None, 0.0, 0.5)
    assert wy.target == pytest.approx(D(12.5), abs=D(0.3))
    wy.set_rate(0.5, 0.5)                                     # Stick bricht ab
    assert not wy.centering
    wy.center()
    _run(wy, None, 0.6, 3.0)
    assert wy.target == 0.0 and wy.at_home and not wy.centering


def test_out_of_range_reset_only_moves_back():
    wy = WaistYaw(limit=D(15))
    wy.reset(D(25), home=0.0)                                 # z. B. Messung nach Notaus
    _run(wy, 1.0, 0.0, 1.0)
    assert wy.target == pytest.approx(D(25)), "nicht weiter hinaus"
    _run(wy, -1.0, 1.0, 1.5)
    assert wy.target < D(25)


def test_stop_and_parse_state():
    wy = WaistYaw()
    wy.reset(0.0)
    wy.set_rate(1.0, 0.0)
    wy.center()
    wy.stop()
    assert not wy.moving(0.0)
    assert wy.set_rate(float("nan"), 0.1) is None and wy.rate(0.1) == 0.0
    assert parse_state('{"walk_ok": true}') == {"walk_ok": True}
    assert parse_state("[1]") is None and parse_state("x") is None


def test_waist_text_for_gui():
    from g1pilot.teleoperation.status_line import waist_text, GREEN, AMBER
    assert waist_text(None, 0.1) is None
    assert waist_text({"owner": False}, 0.1) is None
    assert waist_text({"owner": True, "meas_offset_deg": 0.0}, 99.0) is None
    t, c = waist_text({"owner": True, "meas_offset_deg": 0.4, "limit_deg": 15.0}, 0.1)
    assert t == "Hüfte Mitte (max. ±15°)" and c == GREEN
    t, c = waist_text({"owner": True, "meas_offset_deg": 12.3}, 0.1)
    assert t.startswith("Hüfte 12° links") and c == AMBER
    t, _ = waist_text({"owner": True, "meas_offset_deg": -8.0, "moving": True}, 0.1)
    assert t == "Hüfte 8° rechts – dreht"
    t, _ = waist_text({"owner": True, "meas_offset_deg": 5.0, "centering": True,
                       "pending": "walk"}, 0.1)
    assert t == "Hüfte 5° links – fährt zur Mitte vor GEHEN"

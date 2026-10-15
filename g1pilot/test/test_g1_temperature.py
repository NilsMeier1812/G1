#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Temperaturanzeige des echten G1: Datenaufbereitung (ohne ROS/SDK) und
Demo-GUI-Panel (offscreen)."""
import os
import time
from types import SimpleNamespace as NS

import pytest

from g1pilot.utils import g1_temperature as g1t


def _motors(t1=30, t2=30, n=35):
    return [NS(temperature=[t1, t2], vol=48.0) for _ in range(n)]


def _data(**hot):
    ms = _motors()
    for idx, (a, b) in hot.items():
        ms[int(idx[1:])].temperature = [a, b]
    return g1t.parse_payload(g1t.build_payload(ms, 35))


def test_sensor_count():
    assert g1t.N_MOTORS == 29 and len(g1t.MOTOR_NAMES) == 29
    assert sorted(i for _, _, idxs in g1t.GROUPS for i in idxs) == list(range(29))


def test_payload_roundtrip_ignores_extra_motors():
    d = g1t.parse_payload(g1t.build_payload(_motors(41, 44, n=35), 33))
    assert len(d["t"]) == 29 and d["t"][0] == [41.0, 44.0] and d["imu"] == 33.0
    assert d["v"][0] == 48.0


def test_payload_scalar_temperature():
    ms = [NS(temperature=50, vol=1.0) for _ in range(29)]
    assert g1t.parse_payload(g1t.build_payload(ms, 0))["t"][3] == [50.0, 50.0]


@pytest.mark.parametrize("bad", ["", "{", "[]", '{"t": []}', '{"t": [[1]]}',
                                 '{"t": [[1,2]]}', '{"x": 1}'])
def test_parse_rejects_garbage(bad):
    assert g1t.parse_payload(bad) is None


def test_hottest_uses_max_of_both_sensors():
    d = _data(m3=(40, 71), m20=(65, 30))
    assert g1t.motor_temp(d, 3) == 71
    assert g1t.hottest(d) == (3, 71)


def test_groups_cover_and_pick_hot_motor():
    d = _data(m9=(80, 50), m13=(61, 20), m25=(55, 58))
    g = {x["key"]: x for x in g1t.group_summary(d)}
    assert (g["leg_r"]["idx"], g["leg_r"]["max"]) == (9, 80)
    assert (g["waist"]["idx"], g["waist"]["max"]) == (13, 61)
    assert (g["arm_r"]["idx"], g["arm_r"]["max"]) == (25, 58)
    assert g["leg_l"]["max"] == 30


def test_level_boundaries():
    assert g1t.level(59.9) == "ok" and g1t.level(60) == "warn"
    assert g1t.level(74.9) == "warn" and g1t.level(75) == "crit"


def test_thresholds_env(monkeypatch):
    monkeypatch.setenv("G1_TEMP_WARN", "50")
    monkeypatch.setenv("G1_TEMP_CRIT", "70")
    assert g1t.thresholds() == (50.0, 70.0)
    monkeypatch.setenv("G1_TEMP_CRIT", "40")          # crit <= warn -> Defaults
    assert g1t.thresholds() == (g1t.DEFAULT_WARN_C, g1t.DEFAULT_CRIT_C)
    monkeypatch.setenv("G1_TEMP_WARN", "abc")
    assert g1t.thresholds() == (g1t.DEFAULT_WARN_C, g1t.DEFAULT_CRIT_C)


def test_stale():
    assert g1t.is_stale(None, 10.0)
    assert not g1t.is_stale(9.0, 10.0) and g1t.is_stale(5.0, 10.0)


def test_labels():
    assert g1t.motor_label(3) == "Knie links"
    assert g1t.motor_label(0) == "Hüfte Nick links"
    assert g1t.motor_label(10) == "Knöchel Nick rechts"
    assert g1t.motor_label(12) == "Taille Gier"
    assert g1t.motor_label(18) == "Ellbogen links"
    assert g1t.motor_label(28) == "Handgelenk Gier rechts"
    assert len({g1t.motor_label(i) for i in range(29)}) == 29

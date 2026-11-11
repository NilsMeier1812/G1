#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Akku-Aufbereitung (rt/lf/bmsstate -> /g1pilot/battery), ohne ROS/Qt."""
import json
from types import SimpleNamespace as NS

from g1pilot.utils import g1_battery as g1b


def _bms(soc=78, mv=49800, cur=-3200, cells=None, temps=None):
    cells = cells if cells is not None else [3830] * 13 + [0] * 27
    temps = temps if temps is not None else [31, 33, 0] + [0] * 9
    return NS(soc=soc, soh=97, bmsvoltage=[mv, 0, 0], current=cur, cell_vol=cells,
              temperature=temps, cycle=42)


def test_roundtrip():
    d = g1b.parse_payload(g1b.build_payload(_bms()))
    assert d["soc"] == 78 and d["soh"] == 97 and d["cycle"] == 42
    assert d["v"] == 49.8                       # mV -> V
    assert d["i"] == -3.2                       # mA -> A
    assert d["cell"] == [3830.0, 3830.0]        # Nullen (unbelegt) ignoriert
    assert d["tmax"] == 33.0
    assert d["raw"]["bmsvoltage"][0] == 49800.0


def test_voltage_fallbacks():
    assert g1b.pack_voltage([50], []) == 50.0                       # schon in V
    assert g1b.pack_voltage([0], [3800] * 13) == 49.4               # Zellsumme
    assert g1b.pack_voltage([0], []) is None                        # nichts plausibel
    assert g1b.pack_voltage([123456789], [0] * 40) is None
    d = g1b.parse_payload(g1b.build_payload(_bms(mv=0, cells=[0] * 40)))
    assert d["v"] is None and d["cell"] is None
    assert g1b.subtitle(d) == "42 Zyklen"


def test_parse_rejects_garbage():
    assert g1b.parse_payload("kaputt") is None
    assert g1b.parse_payload("{}") is None
    assert g1b.parse_payload(json.dumps({"soc": 140})) is None
    assert g1b.parse_payload(json.dumps({"soc": "x"})) is None
    assert g1b.parse_payload(json.dumps({"soc": 55}))["soc"] == 55.0


def test_level_and_thresholds(monkeypatch):
    assert g1b.level(80) == "ok"
    assert g1b.level(30) == "warn" and g1b.level(16) == "warn"
    assert g1b.level(15) == "crit" and g1b.level(0) == "crit"
    monkeypatch.setenv("G1_BATT_WARN", "40")
    monkeypatch.setenv("G1_BATT_CRIT", "20")
    assert g1b.thresholds() == (40.0, 20.0)
    monkeypatch.setenv("G1_BATT_CRIT", "50")                  # crit >= warn -> Default
    assert g1b.thresholds() == (g1b.DEFAULT_WARN_PCT, g1b.DEFAULT_CRIT_PCT)
    monkeypatch.setenv("G1_BATT_WARN", "abc")
    monkeypatch.setenv("G1_BATT_CRIT", "10")
    assert g1b.thresholds() == (g1b.DEFAULT_WARN_PCT, 10.0)


def test_details_and_stale():
    d = g1b.parse_payload(g1b.build_payload(_bms()))
    txt = g1b.details(d)
    assert "Ladestand: 78 %" in txt and "49.8 V" in txt and "Ladezyklen: 42" in txt
    assert g1b.is_stale(None, 10.0)
    assert not g1b.is_stale(8.0, 10.0) and g1b.is_stale(1.0, 10.0)

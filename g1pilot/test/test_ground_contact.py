#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bodenkontakt aus der Knie-Last (ground_contact, reine Logik)."""
import os
import sys
from types import SimpleNamespace as NS

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from g1pilot.navigation.ground_contact import GroundContact, LEFT_KNEE, RIGHT_KNEE  # noqa: E402


def test_no_data_is_not_ground():
    g = GroundContact(clock=lambda: 0.0)
    ok, why = g.check()
    assert not ok and "nicht messbar" in why and g.load() is None


def test_both_knees_needed_and_sign_ignored():
    g = GroundContact(min_nm=3.0, clock=lambda: 0.0)
    g.feed(-6.6, -6.6, now=0.0)
    assert g.check(now=0.0)[0]
    g = GroundContact(min_nm=3.0)
    g.feed(6.6, 1.0, now=0.0)
    ok, why = g.check(now=0.0)
    assert not ok and "6.6/1.0" in why


def test_stale_data_is_not_ground():
    g = GroundContact(stale_s=0.5)
    g.feed(14.0, 14.0, now=0.0)
    assert g.check(now=0.4)[0]
    assert not g.check(now=0.6)[0]


def test_filter_smooths_spike_but_follows_lift():
    """Einzelner Ausreisser kippt die Anzeige nicht; Anheben ist nach
    ~3 Zeitkonstanten (0.6 s bei 500 Hz) erkannt."""
    g = GroundContact(min_nm=3.0, tau_s=0.2)
    t = 0.0
    for _ in range(500):                        # 1 s stehen
        t += 0.002
        g.feed(6.6, 6.6, now=t)
    t += 0.002
    g.feed(0.0, 0.0, now=t)                     # Ausreisser
    assert g.check(now=t)[0]
    for _ in range(300):                        # 0.6 s angehoben
        t += 0.002
        g.feed(0.1, 0.1, now=t)
    assert not g.check(now=t)[0]


def test_nonfinite_ignored_and_lowstate_indices():
    g = GroundContact(clock=lambda: 0.0)
    g.feed(float("nan"), 6.0, now=0.0)
    assert g.load(now=0.0) is None
    ms = [NS(tau_est=0.0) for _ in range(35)]
    ms[LEFT_KNEE].tau_est, ms[RIGHT_KNEE].tau_est = 5.0, -4.0
    g.feed_lowstate(NS(motor_state=ms))
    assert g.load(now=0.0) == (5.0, 4.0)


def test_default_threshold_from_real_test():
    """Nils 2026-10-21: frei stehend ~14 Nm je Knie -> Schwelle 10 Nm."""
    g = GroundContact()
    assert g.min_nm == 10.0
    g.feed(14.0, 13.5, now=0.0)
    assert g.check(now=0.0)[0]
    g.feed(6.0, 6.0, now=1.0)          # Kran traegt gut die Haelfte
    assert not g.check(now=1.0)[0]

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WaistHold (Taillen-Kommando unter arm_sdk) -- reine Logik, ohne ROS."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..",
                                                "g1pilot", "manipulation")))
from waist_hold import (  # noqa: E402
    WaistHold, WaistPassthrough, enable_target, parse_waist_mode,
)


def test_target_is_measured_pose_clipped():
    w = WaistHold()
    w.set_target([0.1, 0.9, -0.9])            # Roll/Pitch ausserhalb +-0.52
    assert np.allclose(w.target, [0.1, 0.52, -0.52])
    q, kp, kd = w.step(0.004, [0.1, 0.0, 0.0])
    assert np.allclose(q, w.target) and kp == 150.0 and kd == 4.0


def test_locked_waist_variant_only_yaw():
    w = WaistHold(joints=[12])
    w.set_target([0.05])
    q, kp, _ = w.step(0.004, [0.05])
    assert q.shape == (1,) and kp == 150.0


def test_invalid_joint_rejected():
    with pytest.raises(ValueError):
        WaistHold(joints=[11, 12])


def test_passive_follows_measurement():
    w = WaistHold(passive_kd=3.0)
    w.set_target([0.0, 0.0, 0.0])
    w.go_passive()
    q, kp, kd = w.step(0.004, [0.0, 0.1, 0.3])
    assert kp == 0.0 and kd == 3.0 and np.allclose(q, [0.0, 0.1, 0.3])


def test_engage_ramps_gain_and_moves_slowly():
    w = WaistHold(kp=150.0, kd=4.0, passive_kd=3.0, max_vel=0.3, engage_s=1.0)
    w.set_target([0.0, 0.0, 0.0])
    w.go_passive()
    w.engage([0.0, 0.0, 0.3])
    dt = 0.01
    q, kp, _ = w.step(dt, [0.0, 0.0, 0.3])
    assert kp == pytest.approx(1.5) and q[2] == pytest.approx(0.3 - 0.003)
    for _ in range(99):
        q, kp, kd = w.step(dt, [0.0, 0.0, 0.3])
    assert kp == pytest.approx(150.0) and kd == pytest.approx(4.0)
    assert q[2] == pytest.approx(0.0, abs=1e-9)        # 0.3 rad bei 0.3 rad/s = 1 s


def test_engage_when_active_is_noop():
    w = WaistHold()
    w.set_target([0.0, 0.0, 0.0])
    w.engage([0.0, 0.0, 0.2])                  # schon aktiv: kein Neustart ab Messung
    q, kp, _ = w.step(0.004, [0.0, 0.0, 0.2])
    assert np.allclose(q, 0.0) and kp == 150.0


# ── Taillen-Modus (hold | zero | onboard), Linksdrall-Test 2026-10-27 ────────
def test_parse_waist_mode_defaults_to_hold():
    assert parse_waist_mode("ZERO ") == "zero"
    assert parse_waist_mode("onboard") == "onboard"
    assert parse_waist_mode("") == "hold" and parse_waist_mode("quatsch") == "hold"


def test_enable_target_zero_and_far_off():
    meas = [0.03, 0.05, -0.04]                      # ~3 deg schief
    tgt, note = enable_target("hold", meas)
    assert np.allclose(tgt, meas) and note == ""
    for mode in ("zero", "onboard"):
        tgt, note = enable_target(mode, meas)
        assert np.allclose(tgt, 0.0) and note == ""
    tgt, note = enable_target("zero", [0.6, 0.0, 0.0])  # absichtlich gedreht
    assert np.allclose(tgt, [0.6, 0.0, 0.0]) and "bleibt" in note


def test_zero_target_is_approached_slowly():
    w = WaistHold(max_vel=0.3)
    w.set_target([0.0, 0.0, 0.0])
    w.q_cmd = np.array([0.0, 0.06, 0.0])            # Messung beim ENABLE
    q, _, _ = w.step(0.01, [0.0, 0.06, 0.0])
    assert abs(q[1] - 0.057) < 1e-9                 # 0.3 rad/s * 10 ms
    for _ in range(30):
        q, _, _ = w.step(0.01, q)
    assert np.allclose(q, 0.0)


def _pt(verified=True):
    wp = WaistPassthrough(n=3, stale_s=0.1, blend_s=0.5)
    wp.verified = verified          # Echo-Pruefung hier uebersprungen (eigener Test)
    return wp


def test_passthrough_blends_in_and_out():
    wp = _pt()
    hold = (np.zeros(3), np.full(3, 150.0), np.full(3, 4.0), np.zeros(3))
    pol = ([0.01, 0.05, -0.02], [200.0] * 3, [5.0] * 3, [0.0] * 3)
    t = 0.0
    wp.feed(*pol, now=t)
    out = wp.blend(hold, wp.step(0.25, True, t))
    assert abs(wp.alpha - 0.5) < 1e-9 and np.allclose(out[0], [0.005, 0.025, -0.01])
    wp.step(0.25, True, t)
    out = wp.blend(hold, wp.step(0.0, True, t))
    assert wp.alpha == 1.0 and np.allclose(out[0], pol[0]) and np.allclose(out[1], 200.0)
    # Gehen vorbei -> zurueck aufs Halteziel, wieder ueber blend_s
    out = wp.blend(hold, wp.step(0.25, False, t))
    assert abs(wp.alpha - 0.5) < 1e-9
    wp.step(0.25, False, t)
    out = wp.blend(hold, wp.step(0.0, False, t))
    assert wp.alpha == 0.0 and np.allclose(out[0], 0.0)


def test_passthrough_invalid_data_falls_back():
    wp = _pt()
    assert wp.step(0.1, True, 0.0) is None and "keine" in wp.reason(0.0)
    wp.feed([0.0] * 3, [0.0] * 3, [0.0] * 3, [0.0] * 3, now=0.0)     # kp 0
    assert wp.step(0.1, True, 0.0) is None and "kp 0" in wp.reason(0.0)
    wp.feed([0.0] * 3, [200.0] * 3, [5.0] * 3, [0.0] * 3, now=0.0)
    assert wp.sample(0.05) is not None
    assert wp.sample(0.5) is None and "veraltet" in wp.reason(0.5)   # stale


def test_passthrough_caps_gains():
    wp = _pt()
    wp.feed([0.0] * 3, [400.0] * 3, [20.0] * 3, [99.0] * 3, now=0.0)
    q, kp, kd, tau = wp.sample(0.0)
    assert np.all(kp <= 250.0) and np.all(kd <= 6.0) and np.all(tau <= 25.0)


def test_passthrough_detects_own_command_echo():
    """Zeigt rt/lowcmd nur unser eigenes (gemischtes) Kommando, darf es nicht
    durchgereicht werden -- erkannt, solange wir das Halteziel senden."""
    wp = _pt(verified=False)
    ours = ([0.0, 0.0, 0.0], [150.0] * 3)
    # Echo kommt sofort beim Gehen: vor der Pruefung wird nichts durchgereicht
    wp.note_sent(*ours)
    wp.feed(ours[0], ours[1], [4.0] * 3, [0.0] * 3, now=0.0)
    assert wp.step(0.1, True, 0.0) is None and wp.alpha == 0.0
    assert "geprueft" in wp.reason(0.0)
    for i in range(WaistPassthrough.ECHO_HITS):
        wp.note_sent(*ours)
        wp.feed(ours[0], ours[1], [4.0] * 3, [0.0] * 3, now=0.0)
    assert wp.echo and wp.sample(0.0) is None and "eigenes" in wp.reason(0.0)
    # Gegenprobe: echte Policy-Werte (anders als unsere) -> kein Echo
    wp2 = _pt(verified=False)
    for i in range(10):
        wp2.note_sent(*ours)
        wp2.feed([0.001 * i, 0.02, 0.0], [200.0] * 3, [5.0] * 3, [0.0] * 3, now=0.0)
    assert not wp2.echo and wp2.verified and wp2.sample(0.0) is not None

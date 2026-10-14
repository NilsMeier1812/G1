#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WaistHold (Taillen-Kommando unter arm_sdk) -- reine Logik, ohne ROS."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..",
                                                "g1pilot", "manipulation")))
from waist_hold import WaistHold  # noqa: E402


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

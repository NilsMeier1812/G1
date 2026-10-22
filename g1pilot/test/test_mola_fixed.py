#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mola_fixed.fix_pose: Spiegelung (MID-360 kopfueber) + Hueftdrehung heraus."""
import math

import pytest

from g1pilot.navigation.fix_mola_odometry import euler_to_quat, fix_pose, quat_to_euler


def yaw(q):
    return math.degrees(quat_to_euler(q)[2])


def test_mirror_y_and_yaw():
    (x, y, z), q = fix_pose((1.0, 0.5, 0.1), euler_to_quat(0, 0, math.radians(30)))
    assert (x, y, z) == pytest.approx((1.0, -0.5, 0.1))
    assert yaw(q) == pytest.approx(-30.0)


def test_no_mirror_is_identity():
    q_in = euler_to_quat(0.1, 0.2, 0.3)
    (x, y, z), q = fix_pose((1.0, 0.5, 0.0), q_in, mirror_y=False)
    assert (x, y) == pytest.approx((1.0, 0.5))
    assert quat_to_euler(q) == pytest.approx((0.1, 0.2, 0.3))


def test_waist_yaw_removed():
    # Sensor (Oberkoerper) bei +70 Grad nach der Spiegelung, Huefte +20 -> Becken +50
    _, q = fix_pose((0, 0, 0), euler_to_quat(0, 0, math.radians(-70)), math.radians(20))
    assert yaw(q) == pytest.approx(50.0)
    _, q = fix_pose((0, 0, 0), euler_to_quat(0, 0, math.radians(170)), math.radians(-30),
                    mirror_y=False)
    assert yaw(q) == pytest.approx(-160.0)       # ueber +-180 gewickelt


def test_unnormalized_quaternion():
    _, q = fix_pose((0, 0, 0), (0.0, 0.0, 0.0, 2.0))
    assert sum(c * c for c in q) == pytest.approx(1.0)

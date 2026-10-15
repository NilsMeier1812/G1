#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit-Tests des aufladbaren Schubses: Ladekurve, Kraft-Skala, PushCharger und die
UDP-Kette bis zum PushListener der Sim (ohne ROS/Qt/MuJoCo-Modell)."""
import os
import socket
import sys
import time
import types

import pytest

from g1pilot.utils import push_charge as pc

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_tap_is_unchanged_base_push():
    assert pc.charge_level(0.0) == 0.0
    assert pc.charge_level(pc.TAP_S) == 0.0
    assert pc.force_for_level(0.0) == pc.BASE_N == 150.0


def test_level_is_linear_and_saturates():
    t, c = pc.TAP_S, pc.CHARGE_S
    assert pc.charge_level((t + c) / 2) == pytest.approx(0.5)
    assert pc.charge_level(c) == pytest.approx(1.0)
    assert pc.charge_level(c + 10) == 1.0
    lv = [pc.charge_level(x / 10) for x in range(0, 50)]
    assert lv == sorted(lv)


def test_force_scale():
    assert pc.force_for_level(1.0) == pc.MAX_N
    assert pc.force_for_level(0.5) == pytest.approx((pc.BASE_N + pc.MAX_N) / 2)
    assert pc.force_for_level(-1) == pc.BASE_N and pc.force_for_level(7) == pc.MAX_N
    assert pc.force_for_level(1.0, base_n=200, max_n=100) == 200   # max < base: nie unter base
    assert pc.impulse_for_force(150.0, 0.12) == pytest.approx(18.0)


def test_charger_with_fake_clock():
    now = [100.0]
    ch = pc.PushCharger(clock=lambda: now[0])
    assert not ch.holding and ch.level() == 0.0 and ch.release() == 0.0
    ch.press()
    now[0] += 0.1
    assert ch.holding and ch.level() == 0.0
    assert ch.release() == 0.0 and not ch.holding        # kurzer Klick
    ch.press()
    now[0] += pc.CHARGE_S + 1
    assert ch.level() == 1.0
    assert ch.release() == 1.0 and not ch.holding


def _listener(port, cap=3000.0):
    sys.path.insert(0, os.path.join(REPO, "unitree_mujoco", "simulate_python"))
    from push_listener import PushListener
    model = types.SimpleNamespace(body=lambda n: types.SimpleNamespace(id=1))
    cfg = types.SimpleNamespace(PUSH_ENABLE=True, PUSH_UDP_PORT=port, PUSH_FORCE_N=150.0,
                                PUSH_DURATION_S=0.12, PUSH_BODY="torso_link",
                                PUSH_FORCE_CAP_N=cap)
    return PushListener(model, cfg)


def test_listener_clamp_and_udp_payload():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    pl = _listener(port)
    assert pl.clamp_force(400.0) == 400.0
    assert pl.clamp_force(1e9) == 3000.0
    assert pl.clamp_force(-5.0) == 0.0
    assert pl.clamp_force(float("nan")) == 150.0
    time.sleep(0.3)
    import numpy as np
    class D: xfrc_applied = np.zeros((3, 6))
    d = D()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for payload, expect in ((b"push", 150.0), (b"325.5", 325.5), (b"99999", 3000.0)):
        tx.sendto(payload, ("127.0.0.1", port))
        time.sleep(0.1)
        pl.apply(d)
        assert np.hypot(*d.xfrc_applied[1, 0:2]) == pytest.approx(expect, rel=1e-3)
        time.sleep(0.2)
        pl.apply(d)
        assert not d.xfrc_applied[1, 0:3].any()

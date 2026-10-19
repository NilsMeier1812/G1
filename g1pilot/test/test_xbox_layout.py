#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Xbox-Controller (Xbox One / Series) in der festen PS4-Belegung, ohne ROS.

  * Erkennung (Name/Hersteller), PS4-Erkennung unveraendert
  * drei Treiber-Varianten: xpad (USB), xpadneo (Bluetooth), hid-generic
    (Bluetooth ohne xpadneo: rechter Stick auf ABS_Z/RZ, Trigger GAS/BRAKE)
  * Zuordnung nach Position, Steuerkreuz als Hat oder Tasten
  * Ersatz-NOT-HALT-Taste View
  * Funktionen Ende-zu-Ende ueber die Bedienlogik: Gehen mit LT (Totmann),
    NOT-HALT mit der Xbox-Taste, Voll-Notaus RB + RT + Xbox 2 s
Laeuft mit pytest (stdlib only).
"""
import pytest

from g1pilot.teleoperation import ps4_layout as L
from g1pilot.teleoperation.ps4_arm_teleop import ArmTeleopLogic
from g1pilot.teleoperation.ps4_walk import WalkLogic

K = L.EV_KEY
BTN_A, BTN_B, BTN_X, BTN_Y = 0x130, 0x131, 0x133, 0x134
BTN_TL, BTN_TR, BTN_SELECT, BTN_START, BTN_MODE = 0x136, 0x137, 0x13a, 0x13b, 0x13c

XPAD_ABS = {L.ABS_X: (-32768, 32767), L.ABS_Y: (-32768, 32767),
            L.ABS_RX: (-32768, 32767), L.ABS_RY: (-32768, 32767),
            L.ABS_Z: (0, 1023), L.ABS_RZ: (0, 1023),
            L.ABS_HAT0X: (-1, 1), L.ABS_HAT0Y: (-1, 1)}
GENERIC_BT_ABS = {L.ABS_X: (0, 65535), L.ABS_Y: (0, 65535),
                  L.ABS_Z: (0, 65535), L.ABS_RZ: (0, 65535),
                  L.ABS_BRAKE: (0, 1023), L.ABS_GAS: (0, 1023),
                  L.ABS_HAT0X: (-1, 1), L.ABS_HAT0Y: (-1, 1)}


# ── Erkennung ────────────────────────────────────────────────────────────
@pytest.mark.parametrize("name", [
    "Microsoft X-Box One S pad", "Microsoft Xbox Series S|X Controller",
    "Microsoft X-Box One pad", "Microsoft X-Box 360 pad", "Generic X-Box pad",
    "Xbox Wireless Controller", "Microsoft Xbox Controller"])
def test_xbox_names(name):
    assert L.controller_kind(name) == "xbox"
    assert not L.is_ps4_name(name)


def test_xbox_by_vendor_and_subdevices():
    assert L.controller_kind("Wireless Controller", vendor=L.XBOX_VENDOR) == "xbox"
    assert L.controller_kind("Xbox Wireless Controller Consumer Control") is None
    assert L.controller_kind("AT Keyboard", vendor=L.XBOX_VENDOR) is None
    assert L.controller_kind("Microsoft Natural Keyboard", vendor=L.XBOX_VENDOR) is None


@pytest.mark.parametrize("name", [
    "Wireless Controller", "Sony Interactive Entertainment Wireless Controller",
    "Sony Computer Entertainment Wireless Controller"])
def test_ps4_names_unchanged(name):
    assert L.controller_kind(name, vendor=0x054c) == "ps4"
    assert L.is_ps4_name(name)
    assert L.controller_kind(name + " Motion Sensors") is None
    assert L.controller_kind(name + " Touchpad") is None


def test_ps4_state_unchanged():
    """PS4-Belegung bitgenau wie vorher (Kreuz/Kreis/Dreieck/Viereck = SOUTH/EAST/NORTH/WEST)."""
    s = L.make_state("ps4")
    assert type(s) is L.Ps4State
    expect = {0x130: L.CROSS, 0x131: L.CIRCLE, 0x133: L.TRIANGLE, 0x134: L.SQUARE,
              0x136: L.L1, 0x137: L.R1, 0x138: L.L2, 0x139: L.R2, 0x13a: L.SHARE,
              0x13b: L.OPTIONS, 0x13c: L.PS, 0x13d: L.L3, 0x13e: L.R3}
    for code, idx in expect.items():
        s.reset()
        assert s.apply(K, code, 1)
        assert s.buttons[idx] == 1 and sum(s.buttons) == 1
    assert not s.apply(K, 0xac, 1) and not s.apply(K, 0x2c0, 1)   # Xbox-Codes nicht im PS4
    s.reset()
    s.apply(L.EV_ABS, L.ABS_Z, 255)                                # PS4: Z = L2
    assert s.axes[L.L2_AXIS] == pytest.approx(1.0)


# ── Belegung nach Position ───────────────────────────────────────────────
@pytest.mark.parametrize("code,idx", [
    (BTN_A, L.CROSS), (BTN_B, L.CIRCLE), (BTN_X, L.SQUARE), (BTN_Y, L.TRIANGLE),
    (BTN_TL, L.L1), (BTN_TR, L.R1), (BTN_SELECT, L.SHARE), (BTN_START, L.OPTIONS),
    (BTN_MODE, L.PS), (L.KEY_HOMEPAGE, L.PS), (0x13d, L.L3), (0x13e, L.R3),
    (0x138, L.L2), (0x139, L.R2)])
def test_xbox_buttons_by_position(code, idx):
    s = L.XboxState(XPAD_ABS)
    assert s.apply(K, code, 1)
    assert s.buttons[idx] == 1 and sum(s.buttons) == 1
    s.apply(K, code, 0)
    assert sum(s.buttons) == 0


def test_xbox_xpad_axes():
    s = L.XboxState(XPAD_ABS)
    assert not s.generic_bt
    s.apply(L.EV_ABS, L.ABS_X, -32768)      # links
    s.apply(L.EV_ABS, L.ABS_Y, -32768)      # oben
    s.apply(L.EV_ABS, L.ABS_RX, 32767)      # rechts
    s.apply(L.EV_ABS, L.ABS_RY, 32767)      # unten
    s.apply(L.EV_ABS, L.ABS_Z, 1023)        # LT voll
    s.apply(L.EV_ABS, L.ABS_RZ, 512)        # RT halb
    a = s.axes
    assert a[L.LX] == pytest.approx(1.0) and a[L.LY] == pytest.approx(1.0)
    assert a[L.RX] == pytest.approx(-1.0) and a[L.RY] == pytest.approx(-1.0)
    assert a[L.L2_AXIS] == pytest.approx(1.0) and a[L.R2_AXIS] == pytest.approx(0.5, abs=0.01)
    s.apply(L.EV_ABS, L.ABS_X, 0)
    assert abs(s.axes[L.LX]) < 0.001


def test_xbox_defaults_without_absinfo():
    s = L.XboxState()
    s.apply(L.EV_ABS, L.ABS_Y, -32768)
    s.apply(L.EV_ABS, L.ABS_Z, 1023)
    assert s.axes[L.LY] == pytest.approx(1.0) and s.axes[L.L2_AXIS] == pytest.approx(1.0)


def test_xbox_generic_bluetooth_axes():
    """Bluetooth ohne xpadneo: ABS_Z/RZ ist der rechte Stick, NICHT die Trigger."""
    s = L.XboxState(GENERIC_BT_ABS)
    assert s.generic_bt
    s.apply(L.EV_ABS, L.ABS_Z, 0)           # rechter Stick links
    s.apply(L.EV_ABS, L.ABS_RZ, 0)          # rechter Stick oben
    assert s.axes[L.RX] == pytest.approx(1.0) and s.axes[L.RY] == pytest.approx(1.0)
    assert s.axes[L.L2_AXIS] == 0.0 and s.axes[L.R2_AXIS] == 0.0
    s.apply(L.EV_ABS, L.ABS_BRAKE, 1023)    # LT
    s.apply(L.EV_ABS, L.ABS_GAS, 1023)      # RT
    assert s.axes[L.L2_AXIS] == pytest.approx(1.0) and s.axes[L.R2_AXIS] == pytest.approx(1.0)
    s.apply(L.EV_ABS, L.ABS_Y, 65535)       # links unten
    assert s.axes[L.LY] == pytest.approx(-1.0)


def test_xbox_dpad_hat_and_buttons():
    s = L.XboxState(XPAD_ABS)
    s.apply(L.EV_ABS, L.ABS_HAT0X, -1)
    s.apply(L.EV_ABS, L.ABS_HAT0Y, -1)
    assert s.buttons[L.DPAD_LEFT] and s.buttons[L.DPAD_UP]
    assert s.axes[L.DPAD_X] == 1.0 and s.axes[L.DPAD_Y] == 1.0
    s = L.XboxState(XPAD_ABS)                # xpad mit Steuerkreuz als Tasten
    s.apply(K, 0x2c1, 1)                     # rechts
    s.apply(K, 0x2c3, 1)                     # unten
    assert s.buttons[L.DPAD_RIGHT] and s.buttons[L.DPAD_DOWN]
    assert s.axes[L.DPAD_X] == -1.0 and s.axes[L.DPAD_Y] == -1.0
    s.apply(K, 0x2c1, 0)
    s.apply(K, 0x2c3, 0)
    assert s.axes[L.DPAD_X] == 0.0 and s.axes[L.DPAD_Y] == 0.0 and not any(s.buttons)


def test_xbox_view_as_ps_and_swap():
    s = L.XboxState(XPAD_ABS, ps_button="view")
    s.apply(K, BTN_SELECT, 1)
    assert s.buttons[L.PS] == 1 and s.buttons[L.SHARE] == 0
    s.reset()
    assert s.apply(K, BTN_MODE, 1) and not any(s.buttons)      # Xbox-Taste ohne Wirkung
    s.apply(K, L.KEY_RECORD, 1)                                 # Series-Share = Grundstellung
    assert s.buttons[L.SHARE] == 1
    s = L.XboxState(XPAD_ABS, swap_xy=True)
    s.apply(K, BTN_X, 1)
    assert s.buttons[L.TRIANGLE] == 1 and s.buttons[L.SQUARE] == 0


def test_xbox_reset_neutral():
    s = L.XboxState(XPAD_ABS)
    s.apply(L.EV_ABS, L.ABS_X, 32767)
    s.apply(K, BTN_A, 1)
    s.reset()
    assert s.axes == [0.0] * L.NUM_AXES and s.buttons == [0] * L.NUM_BUTTONS


# ── Funktionen ueber die Bedienlogik ─────────────────────────────────────
@pytest.mark.parametrize("absinfo,lt", [(XPAD_ABS, L.ABS_Z), (GENERIC_BT_ABS, L.ABS_BRAKE)])
def test_xbox_walk_with_lt_deadman(absinfo, lt):
    s = L.XboxState(absinfo)
    w = WalkLogic()
    lo, hi = absinfo[L.ABS_Y]
    assert w.update(s.axes, s.buttons) == []                    # LT los -> scharf
    s.apply(L.EV_ABS, lt, 1023)
    s.apply(L.EV_ABS, L.ABS_Y, lo)                              # Stick voll vor
    assert w.update(s.axes, s.buttons) == [("start",)]
    vx, vy, vyaw = w.command(s.axes)
    assert vx > 0.3 and abs(vy) < 1e-6 and abs(vyaw) < 1e-6
    s.apply(L.EV_ABS, lt, 0)                                    # LT los = Stopp
    assert w.update(s.axes, s.buttons) == [("stop",)]
    assert w.command(s.axes) is None


def test_xbox_button_is_upper_body_estop():
    s = L.XboxState(XPAD_ABS)
    lg = ArmTeleopLogic()
    lg.update(s.axes, s.buttons, 0.0)
    s.apply(K, BTN_MODE, 1)
    assert ("estop",) in lg.update(s.axes, s.buttons, 0.02)


@pytest.mark.parametrize("absinfo,rt", [(XPAD_ABS, L.ABS_RZ), (GENERIC_BT_ABS, L.ABS_GAS)])
def test_xbox_full_estop_rb_rt_xbox_two_seconds(absinfo, rt):
    s = L.XboxState(absinfo)
    lg = ArmTeleopLogic()
    lg.update(s.axes, s.buttons, 0.0)
    s.apply(K, BTN_TR, 1)
    s.apply(L.EV_ABS, rt, 1023)
    s.apply(K, BTN_MODE, 1)
    out, t = [], 0.02
    while t < 1.9:
        out += lg.update(s.axes, s.buttons, t)
        t += 0.02
    assert ("estop",) in out and ("full_arm",) in out and ("full_estop",) not in out
    while t < 2.2:
        out += lg.update(s.axes, s.buttons, t)
        t += 0.02
    assert out.count(("full_estop",)) == 1


def test_xbox_full_estop_with_view_alternative():
    s = L.XboxState(XPAD_ABS, ps_button="view")
    lg = ArmTeleopLogic()
    lg.update(s.axes, s.buttons, 0.0)
    for code in (BTN_TR, BTN_SELECT):
        s.apply(K, code, 1)
    s.apply(L.EV_ABS, L.ABS_RZ, 1023)
    out, t = [], 0.02
    while t < 2.2:
        out += lg.update(s.axes, s.buttons, t)
        t += 0.02
    assert ("full_estop",) in out

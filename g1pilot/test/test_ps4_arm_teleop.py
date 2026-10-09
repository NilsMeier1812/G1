#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tests der PS4-Oberkoerper-Steuerung ohne Hardware und ohne ROS.

  * ps4_layout: evdev-Ereignisse -> feste Joy-Belegung (Vorzeichen, Trigger,
    Steuerkreuz, Neutralstellung nach Abriss, Geraetenamen)
  * ArmTeleopLogic: Handauswahl, Umschalten, Tempo, Haende auf/zu, Stopp,
    Grundstellung (nur gehalten), NOT-HALT-Latch, Totzone/Expo, Spiegeln
  * hand_jog: identisch zur bisherigen Jog-Rechnung der Demo-GUI
Laeuft mit pytest (stdlib only).
"""
import math
import random

import pytest

from g1pilot.teleoperation import ps4_layout as L
from g1pilot.teleoperation.ps4_arm_teleop import (
    ArmTeleopLogic, shape_axis, mirror_twist, COLORS,
)
from g1pilot.teleoperation import hand_jog as hj


# ── Hilfen ────────────────────────────────────────────────────────────────
def joy(buttons=(), **axes):
    """Joy-Zustand: gedrueckte Tasten (Indizes) + Achsen per Name (LX=..)."""
    a = [0.0] * L.NUM_AXES
    for k, v in axes.items():
        a[getattr(L, k)] = v
    b = [0] * L.NUM_BUTTONS
    for i in buttons:
        b[i] = 1
    return a, b


def press(logic, btn, t=0.0):
    """Taste druecken + loslassen -> Befehle des Drueckens."""
    a, b = joy([btn])
    out = logic.update(a, b, t)
    a, b = joy()
    logic.update(a, b, t + 0.02)
    return out


# ── ps4_layout ───────────────────────────────────────────────────────────
def test_layout_sticks_ros_convention():
    s = L.Ps4State({L.ABS_X: (0, 255), L.ABS_Y: (0, 255), L.ABS_RX: (0, 255), L.ABS_RY: (0, 255)})
    s.apply(L.EV_ABS, L.ABS_X, 0)       # ganz links
    s.apply(L.EV_ABS, L.ABS_Y, 0)       # ganz oben
    s.apply(L.EV_ABS, L.ABS_RX, 255)    # ganz rechts
    s.apply(L.EV_ABS, L.ABS_RY, 255)    # ganz unten
    assert s.axes[L.LX] == pytest.approx(1.0)
    assert s.axes[L.LY] == pytest.approx(1.0)
    assert s.axes[L.RX] == pytest.approx(-1.0)
    assert s.axes[L.RY] == pytest.approx(-1.0)
    s.apply(L.EV_ABS, L.ABS_X, 128)     # Mitte (Rohwert) -> praktisch 0
    assert abs(s.axes[L.LX]) < 0.01


def test_layout_triggers_and_buttons():
    s = L.Ps4State({L.ABS_Z: (0, 255), L.ABS_RZ: (0, 255)})
    s.apply(L.EV_ABS, L.ABS_Z, 255)
    s.apply(L.EV_ABS, L.ABS_RZ, 0)
    assert s.axes[L.L2_AXIS] == pytest.approx(1.0)
    assert s.axes[L.R2_AXIS] == pytest.approx(0.0)
    expect = {0x130: L.CROSS, 0x131: L.CIRCLE, 0x133: L.TRIANGLE, 0x134: L.SQUARE,
              0x136: L.L1, 0x137: L.R1, 0x13a: L.SHARE, 0x13b: L.OPTIONS, 0x13c: L.PS}
    for code, idx in expect.items():
        assert s.apply(L.EV_KEY, code, 1)
        assert s.buttons[idx] == 1
        s.apply(L.EV_KEY, code, 0)
        assert s.buttons[idx] == 0
    assert not s.apply(L.EV_KEY, 0x110, 1)   # BTN_LEFT (Touchpad) gehoert nicht dazu


def test_layout_dpad_buttons():
    s = L.Ps4State()
    s.apply(L.EV_ABS, L.ABS_HAT0X, -1)
    assert s.buttons[L.DPAD_LEFT] == 1 and s.buttons[L.DPAD_RIGHT] == 0
    assert s.axes[L.DPAD_X] == 1.0
    s.apply(L.EV_ABS, L.ABS_HAT0X, 1)
    assert s.buttons[L.DPAD_LEFT] == 0 and s.buttons[L.DPAD_RIGHT] == 1
    s.apply(L.EV_ABS, L.ABS_HAT0X, 0)
    assert s.buttons[L.DPAD_LEFT] == 0 and s.buttons[L.DPAD_RIGHT] == 0
    s.apply(L.EV_ABS, L.ABS_HAT0Y, -1)
    assert s.buttons[L.DPAD_UP] == 1
    s.apply(L.EV_ABS, L.ABS_HAT0Y, 1)
    assert s.buttons[L.DPAD_UP] == 0 and s.buttons[L.DPAD_DOWN] == 1


def test_layout_reset_is_neutral():
    s = L.Ps4State()
    s.apply(L.EV_ABS, L.ABS_X, 0)
    s.apply(L.EV_KEY, 0x130, 1)
    s.reset()
    assert s.axes == [0.0] * L.NUM_AXES and s.buttons == [0] * L.NUM_BUTTONS


def test_device_names():
    assert L.is_ps4_name("Wireless Controller")
    assert L.is_ps4_name("Sony Interactive Entertainment Wireless Controller")
    assert not L.is_ps4_name("Wireless Controller Motion Sensors")
    assert not L.is_ps4_name("Wireless Controller Touchpad")
    assert not L.is_ps4_name("Logitech USB Keyboard")
    assert L.is_ps4_name("Mein Pad", wanted="Mein Pad")


# ── Logik: Auswahl, Modus, Tempo ──────────────────────────────────────────
def test_default_state():
    lg = ArmTeleopLogic()
    assert lg.selection == "right" and lg.mode == "move" and lg.speed == "Normal"
    assert lg.color() == COLORS["right"]


def test_dpad_selects_hand():
    lg = ArmTeleopLogic()
    out = press(lg, L.DPAD_LEFT)
    assert lg.sides == ("left",)
    assert ("hold", ("right",)) in out          # abgewaehlte Hand haelt an
    press(lg, L.DPAD_UP)
    assert lg.sides == ("left", "right") and lg.selection == "both"
    press(lg, L.DPAD_DOWN)
    assert lg.selection == "mirror"
    out = press(lg, L.DPAD_RIGHT)
    assert lg.sides == ("right",) and ("hold", ("left",)) in out
    assert press(lg, L.DPAD_RIGHT) == []       # schon gewaehlt -> nichts


def test_held_button_fires_once():
    lg = ArmTeleopLogic()
    a, b = joy([L.CIRCLE])
    first = lg.update(a, b, 0.0)
    again = lg.update(a, b, 0.02)
    assert first == [("hand", "right", "open")] and again == []


def test_triangle_toggles_mode_and_dims_led():
    lg = ArmTeleopLogic()
    press(lg, L.TRIANGLE)
    assert lg.mode == "turn"
    assert all(c <= v for c, v in zip(lg.color(), COLORS["right"]))
    press(lg, L.TRIANGLE)
    assert lg.mode == "move"


def test_speed_steps_clamped():
    lg = ArmTeleopLogic()
    press(lg, L.R1)
    assert lg.speed == "Schnell"
    assert press(lg, L.R1) == []                # oben angeschlagen
    press(lg, L.L1); press(lg, L.L1)
    assert lg.speed == "Langsam"
    assert press(lg, L.L1) == []


def test_hand_open_close_follow_selection():
    lg = ArmTeleopLogic()
    assert press(lg, L.CIRCLE) == [("hand", "right", "open")]
    press(lg, L.DPAD_UP)
    out = press(lg, L.SQUARE)
    assert out == [("hand", "left", "close"), ("hand", "right", "close")]


def test_cross_stops_and_cancels():
    lg = ArmTeleopLogic()
    out = press(lg, L.CROSS)
    assert ("hold", ("left", "right")) in out and ("cancel",) in out


def test_options_enables():
    lg = ArmTeleopLogic()
    assert press(lg, L.OPTIONS) == [("enable",)]


def test_share_must_be_held_for_home():
    lg = ArmTeleopLogic(home_hold_s=1.0)
    a, b = joy([L.SHARE])
    assert lg.update(a, b, 0.0) == []
    assert lg.update(a, b, 0.5) == []
    a0, b0 = joy()
    lg.update(a0, b0, 0.6)                      # zu frueh losgelassen
    assert lg.update(a, b, 1.0) == []
    assert lg.update(a, b, 1.9) == []
    assert lg.update(a, b, 2.05) == [("home",)]
    assert lg.update(a, b, 3.5) == []           # nur einmal je Druecken


def test_estop_latches_until_cleared():
    lg = ArmTeleopLogic()
    out = press(lg, L.PS)
    assert ("estop",) in out and ("hold", ("left", "right")) in out
    assert lg.estop and lg.color() == COLORS["estop"]
    # gesperrt: keine Bewegung, keine Hand, keine Auswahl
    assert press(lg, L.CIRCLE) == [] and press(lg, L.DPAD_LEFT) == []
    assert lg.selection == "right"
    a, b = joy(LY=1.0)
    assert lg.twist(a) == {}
    # erneutes PS sendet den NOT-HALT trotzdem nochmal
    assert ("estop",) in press(lg, L.PS)
    assert lg.clear_estop() and not lg.estop
    assert lg.twist(a) != {}


def test_estop_wins_over_other_buttons_same_packet():
    lg = ArmTeleopLogic()
    a, b = joy([L.PS, L.CIRCLE, L.DPAD_LEFT])
    out = lg.update(a, b, 0.0)
    assert ("estop",) in out
    assert not any(c[0] == "hand" for c in out) and lg.selection == "right"


# ── Logik: Bewegung ───────────────────────────────────────────────────────
def test_shape_axis_deadzone_and_expo():
    assert shape_axis(0.1, 0.12, 0.4) == 0.0
    assert shape_axis(-0.12, 0.12, 0.4) == 0.0
    assert shape_axis(1.0, 0.12, 0.4) == pytest.approx(1.0)
    assert shape_axis(-1.0, 0.12, 0.4) == pytest.approx(-1.0)
    # stetig am Totzonen-Rand, monoton, feiner als linear in der Mitte
    assert shape_axis(0.13, 0.12, 0.4) < 0.02
    xs = [i / 100 for i in range(13, 101)]
    ys = [shape_axis(x, 0.12, 0.4) for x in xs]
    assert all(b >= a for a, b in zip(ys, ys[1:]))
    assert shape_axis(0.5, 0.12, 0.4) < (0.5 - 0.12) / 0.88


def test_twist_move_mode_axes():
    lg = ArmTeleopLogic(expo=0.0, deadzone=0.0)
    v, _w = hj.JOG_SPEEDS["Normal"]
    a, _ = joy(LY=1.0)                          # linker Stick oben -> vor
    assert lg.twist(a) == {"right": ((v, 0.0, 0.0), (0.0, 0.0, 0.0))}
    a, _ = joy(LX=1.0)                          # links -> +y
    assert lg.twist(a)["right"][0] == (0.0, v, 0.0)
    a, _ = joy(RY=-1.0)                         # rechter Stick unten -> runter
    assert lg.twist(a)["right"][0] == (0.0, 0.0, -v)
    a, _ = joy(RX=1.0)                          # rechter Stick quer: frei
    assert lg.twist(a) == {}


def test_twist_turn_mode_axes():
    lg = ArmTeleopLogic(expo=0.0, deadzone=0.0)
    press(lg, L.TRIANGLE)
    _v, w = hj.JOG_SPEEDS["Normal"]
    a, _ = joy(LY=1.0)                          # oben = Finger hoch = -wy
    assert lg.twist(a)["right"] == ((0.0, 0.0, 0.0), (0.0, -w, 0.0))
    a, _ = joy(LX=1.0)                          # links = schwenken links = +wz
    assert lg.twist(a)["right"][1] == (0.0, 0.0, w)
    a, _ = joy(RX=-1.0)                         # rechter Stick rechts = rollen +wx
    assert lg.twist(a)["right"][1] == (w, 0.0, 0.0)


def test_twist_speed_scales():
    lg = ArmTeleopLogic(expo=0.0, deadzone=0.0)
    press(lg, L.L1)
    a, _ = joy(LY=1.0)
    assert lg.twist(a)["right"][0][0] == pytest.approx(hj.JOG_SPEEDS["Langsam"][0])


def test_twist_both_parallel_and_mirror():
    lg = ArmTeleopLogic(expo=0.0, deadzone=0.0)
    press(lg, L.DPAD_UP)
    a, _ = joy(LX=1.0, LY=0.5)
    tw = lg.twist(a)
    assert tw["left"] == tw["right"]
    press(lg, L.DPAD_DOWN)
    tw = lg.twist(a)
    assert tw["right"][0][1] > 0 and tw["left"][0][1] == -tw["right"][0][1]
    assert tw["left"][0][0] == tw["right"][0][0]          # vor/zurueck gleich
    press(lg, L.TRIANGLE)
    a, _ = joy(LX=1.0, RX=1.0, LY=1.0)
    tw = lg.twist(a)
    (_, wl), (_, wr) = tw["left"], tw["right"]
    assert wl == (-wr[0], wr[1], -wr[2])


def test_mirror_twist_is_reflection():
    """Spiegelung an der x-z-Ebene: S = diag(1,-1,1). Fuer Drehvektoren gilt
    w' = det(S) * S w = (-wx, wy, -wz)."""
    lin, ang = mirror_twist((1.0, 2.0, 3.0), (0.1, 0.2, 0.3))
    assert lin == (1.0, -2.0, 3.0) and ang == (-0.1, 0.2, -0.3)


def test_no_twist_inside_deadzone():
    lg = ArmTeleopLogic()
    a, _ = joy(LX=0.08, LY=-0.1, RY=0.05)
    assert lg.twist(a) == {}


# ── hand_jog: identisch zur bisherigen Demo-GUI-Rechnung ──────────────────
def _old_gui_step(tgt, hand, lin, ang, dt, lin_speed, ang_speed):
    """Wortgleiche Kopie aus demo_gui._jog_tick (Stand arm_control, 662f340)."""
    JOG_LEAD_S, JOG_MIN_LEAD_M, JOG_MIN_LEAD_RAD = 0.5, 0.04, math.radians(15)
    lead_m = max(JOG_MIN_LEAD_M, lin_speed * JOG_LEAD_S)
    lead_rad = max(JOG_MIN_LEAD_RAD, ang_speed * JOG_LEAD_S)
    dq = hj.qfrom_rotvec(*(w * dt for w in ang))
    pos, rot = hand
    if tgt is None:
        tgt = [list(pos), rot]
    p = [tgt[0][i] + lin[i] * dt for i in range(3)]
    d = [p[i] - pos[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in d))
    if n > lead_m:
        p = [pos[i] + d[i] * lead_m / n for i in range(3)]
    q = hj.qmul(dq, tgt[1])
    err = hj.qangle(q, rot)
    if err > lead_rad:
        q = hj.qslerp(rot, q, lead_rad / err)
    return [p, q]


def _rand_quat(rng):
    q = [rng.gauss(0, 1) for _ in range(4)]
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def test_jog_step_matches_old_gui():
    rng = random.Random(7)
    for _ in range(300):
        speed = rng.choice(list(hj.JOG_SPEEDS))
        v, w = hj.JOG_SPEEDS[speed]
        hand = ([rng.uniform(-0.5, 0.5) for _ in range(3)], _rand_quat(rng))
        tgt = None if rng.random() < 0.3 else \
            [[c + rng.uniform(-0.1, 0.1) for c in hand[0]], _rand_quat(rng)]
        lin = tuple(rng.uniform(-v, v) for _ in range(3))
        ang = tuple(rng.uniform(-w, w) for _ in range(3))
        dt = rng.uniform(0.0, 0.1)
        lead_m, lead_rad = hj.lead_limits(v, w)
        new = hj.jog_step(tgt, hand, lin, ang, dt, lead_m, lead_rad)
        old = _old_gui_step(tgt, hand, lin, ang, dt, v, w)
        assert new[0] == pytest.approx(old[0], abs=1e-12)
        assert new[1] == pytest.approx(old[1], abs=1e-12)


def test_jog_step_lead_limit_holds_when_arm_stuck():
    """Arm folgt nicht (Hand bleibt stehen): Ziel bleibt im Vorlauf."""
    v, w = hj.JOG_SPEEDS["Schnell"]
    lead_m, lead_rad = hj.lead_limits(v, w)
    hand = ([0.3, -0.2, 0.1], (0.0, 0.0, 0.0, 1.0))
    tgt = None
    for _ in range(200):     # 200 x 33 ms = 6.6 s Vollausschlag
        tgt = hj.jog_step(tgt, hand, (v, 0, 0), (0, 0, w), 1 / 30, lead_m, lead_rad)
    d = math.dist(tgt[0], hand[0])
    assert d == pytest.approx(lead_m, abs=1e-9)
    assert hj.qangle(tgt[1], hand[1]) == pytest.approx(lead_rad, abs=1e-6)


def test_jog_step_moves_with_speed_when_arm_follows():
    v, w = hj.JOG_SPEEDS["Normal"]
    lead_m, lead_rad = hj.lead_limits(v, w)
    hand = [[0.3, -0.2, 0.1], (0.0, 0.0, 0.0, 1.0)]
    tgt = None
    for _ in range(30):      # 1 s, Hand folgt exakt
        tgt = hj.jog_step(tgt, hand, (0, 0, v), (0, 0, 0), 1 / 30, lead_m, lead_rad)
        hand = [list(tgt[0]), tgt[1]]
    assert hand[0][2] - 0.1 == pytest.approx(v, rel=1e-6)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tests der PS4-Oberkoerper-Steuerung ohne Hardware und ohne ROS.

  * ps4_layout: evdev-Ereignisse -> feste Joy-Belegung (Vorzeichen, Trigger,
    Steuerkreuz, Neutralstellung nach Abriss, Geraetenamen)
  * ArmTeleopLogic: Handauswahl, Umschalten, Tempo, Haende auf/zu, Stopp,
    Grundstellung (nur gehalten), NOT-HALT-Latch, Totzone/Expo, Spiegeln
  * hand_jog: identisch zur bisherigen Jog-Rechnung der Demo-GUI
  * Achsen-Einrasten und Jog-Bezug (kommandierte Hand): Regelkreis-Modell mit
    Durchhang, Reibung und links/rechts verschiedenem Schleppfehler
Laeuft mit pytest (stdlib only).
"""
import math
import random

import pytest

from g1pilot.teleoperation import ps4_layout as L
from g1pilot.teleoperation.ps4_arm_teleop import (
    ArmTeleopLogic, shape_axis, snap_stick, mirror_twist, COLORS,
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
    assert press(lg, L.TRIANGLE) == [("jog_mode", "turn")]
    assert lg.mode == "turn"
    assert all(c <= v for c, v in zip(lg.color(), COLORS["right"]))
    assert press(lg, L.TRIANGLE) == [("jog_mode", "move")]
    assert lg.mode == "move"


def test_led_walk_blue_like_gui_tile_and_distinct_from_hands():
    """Nils 2026-10-21: GEHEN/GREIFEN-Farben waren vertauscht (GEHEN orange,
    in der GUI ist orange GREIFEN). GEHEN jetzt blau wie die Kachel."""
    gui_walk = (0x1e, 0x88, 0xe5)          # demo_gui.MODE_COLOR[WALK] (#1e88e5)
    assert COLORS["walk"] == gui_walk
    for side in ("left", "right", "both", "mirror"):
        dist = sum(abs(a - b) for a, b in zip(COLORS[side], COLORS["walk"]))
        assert dist > 100, side             # am Lichtbalken unterscheidbar (tuerkis ~120)


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


# ── Real-Einstellungen (bringup_real) ──────────────────────────────────────
def test_max_speed_caps_r1_and_start_speed():
    lg = ArmTeleopLogic(speed="Langsam", max_speed="Normal")
    assert lg.speed == "Langsam"
    assert press(lg, L.R1) == [("changed",)] and lg.speed == "Normal"
    assert press(lg, L.R1) == [] and lg.speed == "Normal"     # nicht Schnell
    a, b = joy(LY=1.0)
    v = max(abs(c) for c in lg.twist(a)["right"][0])
    assert v == pytest.approx(0.15) and v < 0.25               # < EE-Limit real
    # Start-Tempo oberhalb des Maximums wird gekappt
    assert ArmTeleopLogic(speed="Schnell", max_speed="Normal").speed == "Normal"
    # unbekannter Wert -> kein Deckel (Sim-Default)
    assert ArmTeleopLogic(max_speed="??").max_idx == 2


def test_ps_without_estop_acts_like_cross():
    lg = ArmTeleopLogic(ps_estop=False)
    out = press(lg, L.PS)
    assert out == [("hold", ("left", "right")), ("cancel",)]
    assert not lg.estop
    assert press(lg, L.CIRCLE) == [("hand", "right", "open")]   # nicht gesperrt


def test_external_estop_latches_once():
    lg = ArmTeleopLogic()
    assert lg.latch_estop() and lg.estop
    assert not lg.latch_estop()                                # schon gesperrt
    assert press(lg, L.CIRCLE) == [] and lg.twist(joy(LY=1.0)[0]) == {}
    assert lg.label().startswith("PS4 · NOT-HALT")
    assert lg.clear_estop() and not lg.estop


# ── Voll-Notaus: R1 + L1 + PS halten ──────────────────────────────────────
COMBO = (L.R1, L.L1, L.PS)


def hold_combo(lg, t0, t1, dt=0.02, buttons=COMBO, **axes):
    """Kombination von t0 bis t1 im 50-Hz-Takt halten -> alle Befehle."""
    out, t = [], t0
    while t <= t1 + 1e-9:
        a, b = joy(buttons, **axes)
        out += lg.update(a, b, t)
        t += dt
    return out


def test_full_estop_after_two_seconds_hold():
    lg = ArmTeleopLogic()
    out = hold_combo(lg, 0.0, 1.9)
    # PS allein wirkt sofort wie heute (Oberkoerper-NOT-HALT), dazu startet das Halten.
    assert out[:3] == [("hold", ("left", "right")), ("estop",), ("full_arm",)]
    assert ("full_estop",) not in out and lg.full_arming and not lg.full_estop
    assert lg.status()["full_arming"] is True
    out = hold_combo(lg, 1.92, 2.1)
    assert out.count(("full_estop",)) == 1                  # genau einmal
    assert lg.full_estop and lg.estop and not lg.full_arming
    assert lg.color() == COLORS["estop"] and "VOLL-NOTAUS" in lg.label()
    assert lg.status()["full_estop"] is True
    assert hold_combo(lg, 2.12, 4.0) == []                  # weiter gehalten: nichts mehr
    a, b = joy()
    assert lg.update(a, b, 4.1) == []                       # Loslassen: kein Abbruch-Signal
    assert lg.full_estop                                    # bleibt bis START
    assert press(lg, L.CIRCLE, 4.2) == [] and lg.twist(joy(LY=1.0)[0]) == {}
    assert lg.clear_estop() and not lg.full_estop and not lg.estop


def test_full_estop_aborts_on_early_release():
    lg = ArmTeleopLogic()
    hold_combo(lg, 0.0, 1.5)
    a, b = joy([L.R1, L.L1])                                # PS los
    assert lg.update(a, b, 1.52) == [("full_abort",)]
    assert not lg.full_arming and not lg.full_estop
    assert lg.estop                                         # PS-NOT-HALT bleibt
    # Neues Halten zaehlt von vorn, nicht ab dem ersten Druecken.
    out = hold_combo(lg, 1.6, 3.4)
    assert ("full_estop",) not in out and ("full_arm",) in out
    assert ("full_estop",) in hold_combo(lg, 3.42, 3.7)


@pytest.mark.parametrize("missing", COMBO)
def test_full_estop_needs_all_three(missing):
    lg = ArmTeleopLogic()
    out = hold_combo(lg, 0.0, 5.0, buttons=[x for x in COMBO if x != missing])
    assert not any(c[0].startswith("full") for c in out) and not lg.full_estop


def test_full_estop_r2_no_longer_counts():
    """Bis 2026-10-19 war es R1 + R2 + PS -- R2 (Taste oder Trigger) loest nichts mehr aus."""
    lg = ArmTeleopLogic()
    out = hold_combo(lg, 0.0, 3.0, buttons=(L.R1, L.R2, L.PS), R2_AXIS=1.0)
    assert not any(c[0].startswith("full") for c in out) and not lg.full_estop


@pytest.mark.parametrize("first", [(L.L1,), (L.R1,), (L.L1, L.R1)])
def test_full_estop_speed_step_before_ps_is_undone(first):
    """L1/R1 vor PS gedrueckt: deren Tempo-Schritt wird beim Scharfwerden zurueckgenommen."""
    lg = ArmTeleopLogic(speed="Normal")
    t = 0.0
    held = []
    for btn in first:
        held.append(btn)
        a, b = joy(held)
        lg.update(a, b, t)
        t += 0.02
    assert lg.speed != "Normal" or first == (L.L1, L.R1)
    out = hold_combo(lg, t, t + 0.1)
    assert ("full_arm",) in out and lg.speed == "Normal"


def test_speed_step_alone_still_works():
    lg = ArmTeleopLogic(speed="Normal")
    assert press(lg, L.L1) == [("changed",)] and lg.speed == "Langsam"
    hold_combo(lg, 1.0, 1.2, buttons=(L.PS,))               # PS allein: kein Voll-Notaus
    assert lg.speed == "Langsam" and not lg.full_arming


def test_full_estop_gap_in_stream_restarts_hold():
    """Funkabriss mitten im Halten: kommt der Strom nach 1.5 s mit weiter
    gedrueckten Tasten zurueck, loest das NICHT sofort aus -- neu 2 s halten."""
    lg = ArmTeleopLogic()
    hold_combo(lg, 0.0, 1.0)
    out = hold_combo(lg, 2.5, 2.6)                          # Luecke 1.5 s
    assert ("full_estop",) not in out and ("full_arm",) in out
    assert ("full_estop",) not in hold_combo(lg, 2.62, 4.4)
    assert ("full_estop",) in hold_combo(lg, 4.42, 4.6)


def test_full_estop_works_while_estop_latched_and_without_ps_estop():
    lg = ArmTeleopLogic()
    lg.latch_estop()                                        # NOT-HALT aus der GUI
    assert ("full_estop",) in hold_combo(lg, 0.0, 2.1)
    lg = ArmTeleopLogic(ps_estop=False)
    out = hold_combo(lg, 0.0, 2.1)
    assert ("cancel",) in out and ("estop",) not in out[:3]
    assert ("full_estop",) in out and lg.full_estop and lg.estop


def test_full_estop_combo_blocks_r1_speed_step():
    lg = ArmTeleopLogic(ps_estop=False)                     # nicht durch NOT-HALT gesperrt
    a, b = joy([L.PS, L.L1])
    lg.update(a, b, 0.0)
    a, b = joy(COMBO)                                       # R1 zuletzt
    out = lg.update(a, b, 0.02)
    assert ("full_arm",) in out and lg.speed == "Normal"        # L1-Schritt zurueckgenommen, R1 kein Schritt


def test_full_estop_disabled_with_zero_hold():
    lg = ArmTeleopLogic(full_estop_hold_s=0.0)
    out = hold_combo(lg, 0.0, 5.0)
    assert not any(c[0].startswith("full") for c in out) and lg.estop


def test_full_estop_abort_via_timeout_helper():
    lg = ArmTeleopLogic()
    hold_combo(lg, 0.0, 0.5)
    assert lg.abort_full() and not lg.full_arming
    assert not lg.abort_full()


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
    """Innerhalb des Vorlaufs identisch zur alten GUI-Rechnung; nur der
    Positions-Deckel ist neu (entlang der Bewegung statt Kugel um die Hand)."""
    rng = random.Random(7)
    n_same = n_capped = 0
    for _ in range(600):
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
        assert new[1] == pytest.approx(old[1], abs=1e-12)      # Drehung unveraendert
        p0 = (tgt or [hand[0]])[0]
        cand = [p0[i] + lin[i] * dt for i in range(3)]
        if math.dist(cand, hand[0]) <= lead_m:
            n_same += 1
            assert new[0] == pytest.approx(old[0], abs=1e-12)
        else:
            n_capped += 1
            d = [new[0][i] - hand[0][i] for i in range(3)]
            sp = math.sqrt(sum(c * c for c in lin))
            along = sum(d[i] * lin[i] / sp for i in range(3))
            assert along <= lead_m + 1e-12
            assert math.dist(new[0], hand[0]) <= hj.JOG_MAX_LEAD_FACTOR * lead_m + 1e-12
    assert n_same > 50 and n_capped > 50


def test_jog_step_keeps_line_when_hand_drifts_sideways():
    """Hand bleibt hinter dem Ziel zurueck UND driftet quer (IK-Restfehler):
    das Ziel bleibt auf seiner Linie (y, z fest), statt die Drift zu uebernehmen."""
    v, w = hj.JOG_SPEEDS["Normal"]
    lead_m, lead_rad = hj.lead_limits(v, w)
    start = [0.3, -0.2, 0.1]
    rot = (0.0, 0.0, 0.0, 1.0)
    tgt = None
    for k in range(60):              # 2 s Vollausschlag +x, Hand halb so schnell
        hand = [[start[0] + 0.5 * v * k / 30,
                 start[1] - 0.0005 * k,            # 3 cm Querdrift ueber 2 s
                 start[2]], rot]
        tgt = hj.jog_step(tgt, hand, (v, 0, 0), (0, 0, 0), 1 / 30, lead_m, lead_rad)
        assert tgt[0][0] - hand[0][0] <= lead_m + 1e-9
    assert tgt[0][1] == pytest.approx(start[1], abs=1e-9)
    assert tgt[0][2] == pytest.approx(start[2], abs=1e-9)
    assert tgt[0][0] - hand[0][0] == pytest.approx(lead_m, abs=1e-9)


def test_jog_step_safety_cap_on_total_distance():
    """Haengt die Hand quer weit weg, wird das Ziel trotzdem herangezogen."""
    lead_m, lead_rad = 0.04, math.radians(15)
    hand = ([0.3, -0.5, 0.1], (0.0, 0.0, 0.0, 1.0))
    tgt = [[0.3, -0.2, 0.1], (0.0, 0.0, 0.0, 1.0)]
    new = hj.jog_step(tgt, hand, (0.15, 0, 0), (0, 0, 0), 1 / 30, lead_m, lead_rad)
    assert math.dist(new[0], hand[0]) == pytest.approx(hj.JOG_MAX_LEAD_FACTOR * lead_m)


def test_hold_pose_stops_along_motion_keeps_line():
    rot_t = (0.0, 0.0, 0.0, 1.0)
    rot_h = hj.qfrom_rotvec(0.0, 0.0, 0.2)
    tgt = [[0.40, -0.20, 0.10], rot_t]
    hand = ([0.35, -0.23, 0.11], rot_h)
    # nach Verschieben +x: x von der Hand, y/z vom Ziel, Orientierung vom Ziel
    p, q = hj.hold_pose(tgt, hand, (0.15, 0, 0), (0, 0, 0))
    assert p == pytest.approx([0.35, -0.20, 0.10])
    assert q == pytest.approx(rot_t)
    # schraeg: nur die Komponente entlang der Bewegung faellt weg
    u = (1 / math.sqrt(2), 1 / math.sqrt(2), 0.0)
    p, _ = hj.hold_pose(tgt, hand, tuple(0.1 * c for c in u), (0, 0, 0))
    d_t = [tgt[0][i] - p[i] for i in range(3)]
    d_h = [hand[0][i] - p[i] for i in range(3)]
    assert sum(d_h[i] * u[i] for i in range(3)) == pytest.approx(0.0, abs=1e-12)
    assert sum(d_t[i] * (u[1], -u[0], 0.0)[i] for i in range(3)) == pytest.approx(0.0, abs=1e-12)
    # nach Drehen: Position fest (Ziel), Orientierung der echten Hand
    p, q = hj.hold_pose(tgt, hand, (0, 0, 0), (0, 0, 0.5))
    assert p == pytest.approx(tgt[0]) and q == pytest.approx(rot_h)
    # ohne Ziel: echte Hand
    p, q = hj.hold_pose(None, hand, (0.1, 0, 0), (0, 0, 0))
    assert p == pytest.approx(hand[0]) and q == pytest.approx(rot_h)


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


def test_hold_streamer_repeats_hold_until_expiry():
    """Halte-Ziel nach dem Loslassen JOG_HOLD_S lang wiederholen, dann Ruhe."""
    hs = hj.HoldStreamer(hold_s=0.5)
    pose = ([0.2, -0.2, 0.05], (1.0, 0.0, 0.0, 0.0))
    hs.hold("right", pose, now=10.0)
    assert hs.due(10.0) == {"right": pose}
    assert hs.due(10.5) == {"right": pose}          # Grenze noch drin
    assert hs.due(10.51) == {}                       # abgelaufen
    assert hs.due(10.6) == {}                        # und vergessen


def test_hold_streamer_cancel_and_clear():
    hs = hj.HoldStreamer()
    pl = ([0.2, 0.2, 0.05], (1.0, 0.0, 0.0, 0.0))
    pr = ([0.2, -0.2, 0.05], (1.0, 0.0, 0.0, 0.0))
    hs.hold("left", pl, 0.0)
    hs.hold("right", pr, 0.0)
    hs.cancel("left")                                # links bewegt sich wieder
    assert hs.due(0.1) == {"right": pr}
    hs.hold("right", pl, 0.2)                        # neues Halte-Ziel ersetzt das alte
    assert hs.due(0.3) == {"right": pl}
    hs.clear()                                       # Grundstellung / NOT-HALT
    assert hs.due(0.3) == {}


def test_hold_streamer_converges_controller_goal_filter():
    """Mit der Glaettung PRO NACHRICHT im arm_controller (alpha 0.25): ein
    einzelnes Halte-Ziel laesst ~75 % des Vorlaufs stehen, das Wiederholen
    zieht den gefilterten Sollwert auch bei nur ~14 Nachrichten/s (langsame
    Sim, 1 Ziel pro Regeltakt) auf < 2 % an die Hand."""
    alpha, lead = 0.25, 0.04
    one = lead * (1 - alpha)
    assert one > 0.029
    n = int(hj.JOG_HOLD_S * 14)
    assert lead * (1 - alpha) ** n < 0.02 * lead


# ── Jog-Bezug: kommandierte statt gemessene Hand (hand_jog.JOG_CMD_TOPIC) ──
def test_commanded_poses_fresh_and_stale():
    cp = hj.CommandedPoses(max_age_s=0.3)
    assert cp.get("right", 0.0) is None
    cp.update("right", (0.3, -0.1, 0.2), (0.0, 0.0, 0.0, 1.0), 10.0)
    assert cp.get("right", 10.2) == ([0.3, -0.1, 0.2], (0.0, 0.0, 0.0, 1.0))
    assert cp.get("right", 10.31) is None      # zu alt -> Fallback TF
    assert cp.get("left", 10.0) is None


def _jog_sessions(reference, n=8, offset_deg=3.0):
    """Stark vereinfachter Regelkreis: der Arm erreicht das Ziel exakt
    (kommandiert = Ziel), gemessen ist um einen festen Regelfehler gekippt
    (Durchhang). n Tipp-Fahrten nach vorn; Vorlauf und Halten wie im Node an
    der gemessenen Hand, nur der Start je nach `reference`.
    -> Kippwinkel der gemessenen Hand gegenueber der Ausgangslage [deg]."""
    off = hj.qfrom_rotvec(0.0, -math.radians(offset_deg), 0.0)
    cmd = ([0.3, -0.15, 0.2], hj.qfrom_rotvec(0.0, 0.3, 0.0))
    meas = lambda c: (list(c[0]), hj.qmul(c[1], off))   # noqa: E731
    start_meas = meas(cmd)
    lin, ang = (0.15, 0.0, 0.0), (0.0, 0.0, 0.0)
    for _ in range(n):
        target = None
        for _ in range(5):
            start = cmd if (reference == "command" and target is None) else None
            target = hj.jog_step(target, meas(cmd), lin, ang, 0.033, 0.075,
                                 math.radians(22), start=start)
            cmd = (list(target[0]), tuple(target[1]))
        p, q = hj.hold_pose(target, meas(cmd), lin, ang)
        cmd = (list(p), tuple(q))
    return math.degrees(hj.qangle(start_meas[1], meas(cmd)[1]))


def test_jog_from_measured_hand_ratchets_orientation():
    # Bisheriges Verhalten: jede Fahrt uebernimmt den Regelfehler erneut.
    assert _jog_sessions("tf") == pytest.approx(8 * 3.0, abs=0.5)


def test_jog_from_commanded_hand_keeps_orientation():
    assert _jog_sessions("command") < 0.1


def test_jog_step_start_pose_only_for_new_target():
    hand = ([0.3, -0.15, 0.2], (0.0, 0.0, 0.0, 1.0))
    start = ([0.31, -0.15, 0.21], hj.qfrom_rotvec(0.0, -0.05, 0.0))
    p, q = hj.jog_step(None, hand, (0, 0, 0), (0, 0, 0), 0.03, 0.04, 0.3, start=start)
    assert p == pytest.approx(start[0]) and q == pytest.approx(start[1])
    # laufendes Ziel: start wird ignoriert
    p2, q2 = hj.jog_step([p, q], hand, (0, 0, 0), (0, 0, 0), 0.03, 0.04, 0.3,
                         start=([9, 9, 9], (0, 0, 0, 1)))
    assert p2 == pytest.approx(p) and q2 == pytest.approx(q)



# ── Achsen-Einrasten am linken Stick ──────────────────────────────────────
def test_snap_stick_sectors():
    c, s20 = math.cos(math.radians(12)), math.sin(math.radians(12))
    assert snap_stick(s20, c, 20.0) == (0.0, c)            # 12 Grad neben "vor"
    assert snap_stick(-c, s20, 20.0) == (-c, 0.0)          # 12 Grad neben "rechts"
    d = math.sqrt(0.5)
    assert snap_stick(d, d, 20.0) == (d, d)                # Diagonale bleibt
    assert snap_stick(s20, c, 0.0) == (s20, c)             # aus
    assert snap_stick(0.0, 0.0, 20.0) == (0.0, 0.0)


def test_twist_slightly_diagonal_stick_moves_one_axis_only():
    """Daumen 10-15 Grad schraeg: frueher fuhr die Hand ein Stueck seitlich mit."""
    a, _ = joy(LY=0.95, LX=0.25)                           # ~14.7 Grad
    old = ArmTeleopLogic(axis_snap_deg=0.0).twist(a)["right"][0]
    assert abs(old[1]) > 0.003                             # > 3 mm/s seitlich
    lin, ang = ArmTeleopLogic().twist(a)["right"]
    assert lin[0] > 0.1 and lin[1] == 0.0 and lin[2] == 0.0 and ang == (0.0, 0.0, 0.0)
    lg = ArmTeleopLogic()
    press(lg, L.TRIANGLE)                                  # DREHEN: kippen ohne schwenken
    _, ang = lg.twist(a)["right"]
    assert ang[1] < 0 and ang[0] == 0.0 and ang[2] == 0.0
    a, _ = joy(LY=0.7, LX=0.7)                             # gewollte Diagonale
    lin, _ = ArmTeleopLogic().twist(a)["right"]
    assert lin[0] > 0 and lin[1] > 0


# ── Jog-Bezug (hand_jog.JogReference) ───────────────────────────────────
CMD = ([0.30, -0.15, 0.20], (0.0, 0.0, 0.0, 1.0))
TF = ([0.30, -0.15, 0.19], hj.qfrom_rotvec(0.0, 0.05, 0.0))


def test_jog_reference_modes():
    r = hj.JogReference("tf")
    assert r.pose("right", CMD, TF, True) == (None, TF)
    r = hj.JogReference("start")
    assert r.pose("right", CMD, TF, True) == (CMD, TF)
    assert r.pose("right", CMD, TF, False) == (None, TF)
    r = hj.JogReference("command")
    assert r.pose("right", CMD, TF, True) == (None, CMD)
    assert r.pose("right", CMD, TF, False) == (None, CMD)
    assert hj.JogReference("quatsch").mode == hj.JOG_REFERENCE_DEFAULT == "command"


def test_jog_reference_source_fixed_per_motion():
    r = hj.JogReference("command")
    # Fahrt beginnt ohne kommandierte Pose (vor der Auto-Kalibrierung): TF ...
    assert r.pose("right", None, TF, True) == (None, TF)
    # ... und bleibt TF, auch wenn das Topic mitten in der Fahrt kommt.
    assert r.pose("right", CMD, TF, False) == (None, TF)
    r.end("right")
    assert r.pose("right", CMD, TF, True) == (None, CMD)   # naechste Fahrt
    # Faellt die kommandierte Pose aus: weiter an TF (nicht zurueck).
    assert r.pose("right", None, TF, False) == (None, TF)
    assert r.pose("right", CMD, TF, False) == (None, TF)
    assert r.pose("left", CMD, None, True) == (None, CMD)  # ohne TF: Bezug cmd


def test_jog_reference_mode_from_env(monkeypatch):
    monkeypatch.setenv("G1_JOG_REFERENCE", "TF")
    assert hj.jog_reference_mode() == "tf"
    monkeypatch.delenv("G1_JOG_REFERENCE")
    assert hj.jog_reference_mode() == "command"
    assert hj.jog_reference_mode("start") == "start"


# Regelkreis-Modell: der arm_controller bringt die KOMMANDIERTE Hand auf das
# Ziel (wie der RViz-Marker -- seine IK-Kette ist in Position und Orientierung
# offline sauber). Die GEMESSENE Hand weicht davon ab: Durchhang (Hand nickt
# um ``sag_deg`` nach unten und sitzt tiefer), Haftreibung und Schleppfehler
# (bleibt entgegen der letzten Bewegungsrichtung um ``drag`` zurueck), je Arm
# verschieden -- so wie am echten G1.
def _plant(ref_mode, moves, side="right", sag_deg=3.0, drag=0.012, sag_z=0.006,
           start=None, mirror=False, steps=6, dt=1 / 30):
    """moves: Liste von lin (m/s) je Fahrt (Stick 'steps' Takte, dann los).
    -> (Startpose gemessen, Endpose gemessen, Endpose kommandiert)."""
    sag = hj.qfrom_rotvec(0.0, math.radians(sag_deg), 0.0)
    cmd = start or ([0.30, -0.15, 0.20], hj.qfrom_rotvec(0.0, 0.3, 0.0))
    last_u = [0.0, 0.0, 0.0]

    def meas(c):
        p = [c[0][i] - drag * last_u[i] for i in range(3)]
        p[2] -= sag_z
        return p, hj.qmul(sag, c[1])

    m0 = meas(cmd)
    ref = hj.JogReference(ref_mode)
    v, w = hj.JOG_SPEEDS["Normal"]
    lead_m, lead_rad = hj.lead_limits(v, w)
    for lin in moves:
        if mirror:
            lin = (lin[0], -lin[1], lin[2])
        target = None
        for _ in range(steps):
            st, hand = ref.pose(side, cmd, meas(cmd), target is None)
            target = hj.jog_step(target, hand, lin, (0, 0, 0), dt, lead_m, lead_rad, start=st)
            cmd = (list(target[0]), tuple(target[1]))      # kommandiert folgt dem Ziel
            sp = math.sqrt(sum(c * c for c in lin))
            last_u = [c / sp for c in lin]
        _, hand = ref.pose(side, cmd, meas(cmd), False)
        ref.end(side)
        p, q = hj.hold_pose(target, hand, lin, (0, 0, 0))
        cmd = (list(p), tuple(q))
    return m0, meas(cmd), cmd


UP, DOWN, IN, OUT = (0, 0, 0.15), (0, 0, -0.15), (0, 0.15, 0), (0, -0.15, 0)


@pytest.mark.parametrize("moves", [[UP, DOWN] * 4, [IN, OUT] * 4,
                                   [(0.15, 0, 0), (-0.15, 0, 0)] * 4])
def test_taps_back_and_forth_return_without_tilt(moves):
    """8x Tippen hin und zurueck: kommandiert bezogen kommt die Hand an den
    Start zurueck, ohne zu kippen; an der gemessenen Hand bezogen ratscht
    der Durchhang je Fahrt weiter (alter Real-Stand) und die Reibung
    verschiebt den Rueckweg."""
    m0, m1, _ = _plant("command", moves)
    assert math.degrees(hj.qangle(m0[1], m1[1])) < 0.01
    assert math.dist(m0[0], m1[0]) < 0.013                  # nur Reibung der letzten Richtung
    t0, t1, _ = _plant("tf", moves)
    assert math.degrees(hj.qangle(t0[1], t1[1])) > 20.0     # 8 x 3 Grad
    assert math.dist(t0[0], t1[0]) > 0.04


def test_move_up_keeps_orientation_and_side_axes():
    m0, m1, c1 = _plant("command", [UP])
    d = [m1[0][i] - m0[0][i] for i in range(3)]
    assert d[2] > 0.015 and abs(d[0]) < 1e-9 and abs(d[1]) < 1e-9
    assert c1[0][2] - 0.20 == pytest.approx(6 * 0.15 / 30)   # Sollweg exakt
    assert math.degrees(hj.qangle(m0[1], m1[1])) < 0.01


def test_mirror_moves_end_symmetric_despite_different_arms():
    """Gespiegelt aus symmetrischer Startlage, linker Arm mit mehr Reibung/
    Durchhang als der rechte: die kommandierten Haende enden exakt
    spiegelbildlich (wie zwei gespiegelte Marker-Ziele)."""
    q_r = hj.qfrom_rotvec(0.0, 0.3, 0.0)
    start_r = ([0.30, -0.15, 0.20], q_r)
    start_l = ([0.30, 0.15, 0.20], q_r)
    moves = [IN, UP, (0.15, 0, 0), OUT, DOWN]
    for mode, ok in (("command", True), ("tf", False)):
        _, _, cr = _plant(mode, moves, "right", start=start_r)
        _, _, cl = _plant(mode, moves, "left", start=start_l, mirror=True,
                          sag_deg=5.0, drag=0.02, sag_z=0.01)
        err = math.dist(cr[0], (cl[0][0], -cl[0][1], cl[0][2]))
        assert (err < 1e-9) == ok, (mode, err)

# ── L2 = Modus GEHEN (Totmann), R2 = Modus GREIFEN ─────────────────────────
def trig(lg, t, buttons=(), **axes):
    a, b = joy(buttons, **axes)
    return lg.update(a, b, t)


def test_trigger_modes_off_by_default():
    lg = ArmTeleopLogic(walk_deadman=True)
    assert ("mode", "walk") not in trig(lg, 0.0, L2_AXIS=1.0)
    assert ("mode", "grasp") not in trig(lg, 0.1, R2_AXIS=1.0)
    assert not lg.arms_locked


def test_l2_switches_to_walk_and_locks_arms_after_release():
    lg = ArmTeleopLogic(walk_deadman=True, trigger_modes=True)
    out = trig(lg, 0.0, L2_AXIS=1.0)
    assert ("mode", "walk") in out and ("hold", ("left", "right")) in out
    assert lg.robot_mode == "walk" and lg.arms_locked
    assert trig(lg, 0.1, L2_AXIS=1.0) == []                  # gehalten: kein neuer Wechsel
    trig(lg, 0.2)                                            # L2 los: GEHEN bleibt
    assert lg.arms_locked and lg.color() == COLORS["walk"] and "GEHEN" in lg.label()
    assert lg.twist(joy(LY=1.0)[0]) == {}                    # Sticks gesperrt
    for btn in (L.CIRCLE, L.SQUARE, L.TRIANGLE, L.L1, L.R1, L.DPAD_LEFT):
        assert press(lg, btn, 1.0) == [], btn
    assert lg.speed == "Normal" and lg.selection == "right" and lg.mode == "move"
    t, out = 2.0, []
    while t < 4.0:                                           # Share halten: keine Grundstellung
        out += trig(lg, t, [L.SHARE])
        t += 0.05
    assert ("home",) not in out
    assert ("cancel",) in press(lg, L.CROSS, 5.0)            # Stopp geht
    a, b = joy([L.PS])
    assert ("estop",) in lg.update(a, b, 6.0)                # NOT-HALT geht


def test_l2_again_in_walk_sends_nothing_and_r2_switches_to_grasp():
    lg = ArmTeleopLogic(walk_deadman=True, trigger_modes=True)
    trig(lg, 0.0, L2_AXIS=1.0)
    trig(lg, 0.1)
    assert ("mode", "walk") not in trig(lg, 0.2, L2_AXIS=1.0)
    trig(lg, 0.3)
    out = trig(lg, 0.4, R2_AXIS=1.0)
    assert ("mode", "grasp") in out and lg.robot_mode == "grasp" and not lg.arms_locked
    assert trig(lg, 0.5, R2_AXIS=1.0) == []
    trig(lg, 0.6)
    assert lg.twist(joy(LY=1.0)[0])                          # Arme wieder frei
    assert press(lg, L.CIRCLE, 1.0) == [("hand", "right", "open")]


def test_trigger_hysteresis_and_buttons():
    lg = ArmTeleopLogic(trigger_modes=True)
    assert ("mode", "grasp") not in trig(lg, 0.0, R2_AXIS=0.45)
    assert ("mode", "grasp") in trig(lg, 0.1, R2_AXIS=0.55)
    lg.set_robot_mode(None)
    trig(lg, 0.2, R2_AXIS=0.35)                               # noch gedrueckt (> 0.3)
    assert ("mode", "grasp") not in trig(lg, 0.3, R2_AXIS=0.6)
    trig(lg, 0.4, R2_AXIS=0.1)
    assert ("mode", "grasp") in trig(lg, 0.5, [L.R2])         # R2 als Taste
    assert ("mode", "walk") in trig(lg, 0.6, [L.L2])          # L2 als Taste


def test_options_unlocks_walk_mode():
    lg = ArmTeleopLogic(walk_deadman=True, trigger_modes=True)
    trig(lg, 0.0, L2_AXIS=1.0)
    trig(lg, 0.1)
    out = press(lg, L.OPTIONS, 1.0)
    assert ("enable",) in out and not lg.arms_locked


def test_external_mode_follows_gui_and_estop_blocks_switch():
    lg = ArmTeleopLogic(walk_deadman=True, trigger_modes=True)
    assert lg.set_robot_mode("walk") and lg.arms_locked      # GUI-Kachel GEHEN
    assert not lg.set_robot_mode("walk")
    assert lg.set_robot_mode("grasp") and not lg.arms_locked
    lg.latch_estop()
    assert trig(lg, 0.0, L2_AXIS=1.0) == [] and lg.robot_mode == "grasp"


def test_full_estop_still_works_in_walk_mode():
    lg = ArmTeleopLogic(walk_deadman=True, trigger_modes=True)
    trig(lg, 0.0, L2_AXIS=1.0)
    trig(lg, 0.1)
    assert ("full_estop",) in hold_combo(lg, 0.2, 2.4)


# ── Gemeinsamer Zustand mit der Demo-GUI (set_selection / set_speed) ─────
def test_set_selection_from_gui_holds_dropped_side():
    lg = ArmTeleopLogic()
    press(lg, L.DPAD_UP)                                # beide
    out = lg.set_selection("left")
    assert out == [("hold", ("right",)), ("changed",)] and lg.sides == ("left",)
    assert lg.set_selection("left") == [] and lg.set_selection("oben") == []
    assert lg.set_selection("mirror") == [("changed",)]


def test_set_speed_from_gui_capped_and_requested_kept():
    lg = ArmTeleopLogic(speed="Langsam", max_speed="Normal")
    assert lg.set_speed("Schnell")
    assert lg.speed == "Normal" and lg.speed_req == "Schnell"
    assert lg.status()["speed_req"] == "Schnell" and lg.status()["max_speed"] == "Normal"
    assert not lg.set_speed("Schnell") and not lg.set_speed("quatsch")
    press(lg, L.L1)                                     # am Controller: Langsam
    assert lg.speed == lg.speed_req == "Langsam"
    press(lg, L.R1)
    assert lg.speed == lg.speed_req == "Normal"


# ── Hueftdrehung: R2 halten + rechter Stick (Nils 2026-10-21) ────────────
def _grasp(**kw):
    lg = ArmTeleopLogic(trigger_modes=True, waist_yaw=True, walk_deadman=True, **kw)
    lg.set_robot_mode("grasp")
    return lg


def test_waist_r2_hold_turns_waist_and_locks_arm_sticks():
    lg = _grasp()
    out = trig(lg, 0.0, R2_AXIS=1.0)
    assert ("waist", "start") in out and ("hold", ("left", "right")) in out
    assert ("mode", "grasp") not in out                      # schon GREIFEN
    assert lg.waist_mode and lg.color() == COLORS["waist"] and "HÜFTE" in lg.label()
    a, _ = joy(R2_AXIS=1.0, RX=1.0, LY=1.0, RY=1.0)
    assert lg.twist(a) == {}, "Arme bewegen sich nicht"
    assert lg.waist_rate(a) == pytest.approx(1.0)            # Stick links = +links
    a, _ = joy(R2_AXIS=1.0, RX=-0.05)
    assert lg.waist_rate(a) == 0.0                            # Totzone
    a, _ = joy(R2_AXIS=1.0, RX=-1.0)
    assert lg.waist_rate(a) == pytest.approx(-1.0)
    # Steuerkreuz/Dreieck/Tempo wirken nicht, Haende schon
    for btn in (L.DPAD_LEFT, L.TRIANGLE, L.L1, L.R1):
        trig(lg, 1.0, [btn], R2_AXIS=1.0)
        trig(lg, 1.05, R2_AXIS=1.0)
    assert lg.selection == "right" and lg.mode == "move" and lg.speed == "Normal"
    assert ("hand", "right", "open") in trig(lg, 1.1, [L.CIRCLE], R2_AXIS=1.0)
    out = trig(lg, 2.0)                                       # R2 los
    assert ("waist", "end") in out and not lg.waist_mode
    assert lg.waist_rate(joy(RX=1.0)[0]) is None
    assert lg.twist(joy(LY=1.0)[0]), "Arme wieder frei"


def test_waist_r3_centers_and_cross_stops_until_stick_released():
    lg = _grasp()
    trig(lg, 0.0, R2_AXIS=1.0)
    assert ("waist_center",) in trig(lg, 0.1, [L.R3], R2_AXIS=1.0)
    assert ("waist_center",) not in trig(lg, 0.2, [L.R3], R2_AXIS=1.0)   # gehalten
    out = trig(lg, 0.3, [L.CROSS], R2_AXIS=1.0, RX=1.0)
    assert ("waist_stop",) in out and ("cancel",) in out
    assert lg.waist_rate(joy(R2_AXIS=1.0, RX=1.0)[0]) == 0.0              # Stopp haelt
    assert lg.waist_rate(joy(R2_AXIS=1.0)[0]) == 0.0                      # Stick zurueck
    assert lg.waist_rate(joy(R2_AXIS=1.0, RX=1.0)[0]) == pytest.approx(1.0)
    # R3 ohne R2: nichts
    trig(lg, 1.0)
    assert ("waist_center",) not in trig(lg, 1.1, [L.R3])


def test_waist_not_in_walk_mode_estop_or_when_off():
    lg = _grasp()
    trig(lg, 0.0, L2_AXIS=1.0)                                # GEHEN
    trig(lg, 0.1)
    out = trig(lg, 0.2, R2_AXIS=1.0)                          # R2: zurueck nach GREIFEN ...
    assert ("mode", "grasp") in out and lg.waist_mode          # ... gehalten = Huefte
    trig(lg, 0.3)
    a, b = joy([L.PS], R2_AXIS=1.0)
    lg.update(a, b, 0.4)                                      # NOT-HALT
    assert not lg.waist_mode and lg.waist_rate(joy(R2_AXIS=1.0, RX=1.0)[0]) is None
    # Abgeschaltet bzw. ohne trigger_modes: R2 halten ohne Wirkung
    for lg in (ArmTeleopLogic(trigger_modes=True, waist_yaw=False),
               ArmTeleopLogic(trigger_modes=False, waist_yaw=True)):
        lg.set_robot_mode("grasp")
        trig(lg, 0.0, R2_AXIS=1.0)
        assert not lg.waist_mode
        assert lg.waist_rate(joy(R2_AXIS=1.0, RX=1.0)[0]) is None


def test_waist_l2_walking_wins_over_r2():
    lg = _grasp()
    trig(lg, 0.0, R2_AXIS=1.0)
    assert lg.waist_mode
    trig(lg, 0.1, R2_AXIS=1.0, L2_AXIS=1.0)                   # L2 = GEHEN, Arme gesperrt
    assert not lg.waist_mode


def test_rotate_twist_yaw_torso_frame():
    from g1pilot.manipulation.waist_yaw import rotate_twist_yaw
    lin, ang = rotate_twist_yaw((1.0, 0.0, 0.2), (0.0, 0.5, 0.1), math.radians(90))
    assert lin == pytest.approx((0.0, 1.0, 0.2), abs=1e-9)    # "vor" zeigt nach links
    assert ang == pytest.approx((-0.5, 0.0, 0.1), abs=1e-9)
    assert rotate_twist_yaw((1, 2, 3), (4, 5, 6), 0.0) == ((1, 2, 3), (4, 5, 6))
    # Spiegeln im Oberkoerper-Bezug: Hand links/rechts symmetrisch zur Brust
    yaw = math.radians(20)
    lin_r, ang_r = (0.0, 0.1, 0.0), (0.0, 0.0, 0.3)
    lin_l, ang_l = mirror_twist(lin_r, ang_r)
    r = rotate_twist_yaw(lin_r, ang_r, yaw)
    l_ = rotate_twist_yaw(lin_l, ang_l, yaw)
    # zurueck in den Oberkoerper gedreht = exakt gespiegelt
    back_r = rotate_twist_yaw(*r, -yaw)
    back_l = rotate_twist_yaw(*l_, -yaw)
    assert mirror_twist(*back_r)[0] == pytest.approx(back_l[0])


# ── Arm spiegeln: Steuerkreuz links/rechts + Kreuz halten ─────────────────
def test_mirror_after_two_seconds_hold_left():
    """Links + Kreuz: Auswahl LINKS und Stopp sofort (wie bisher), dazu startet
    das Halten; nach 2 s genau einmal ("mirror", "left") -- vorher beide Arme
    angehalten (Jog-Ziele weg)."""
    lg = ArmTeleopLogic(selection="right")
    out = hold_combo(lg, 0.0, 1.9, buttons=(L.DPAD_LEFT, L.CROSS))
    assert lg.selection == "left"
    assert out[:4] == [("hold", ("right",)), ("changed",), ("hold", ("left", "right")),
                       ("cancel",)]
    assert out[4] == ("mirror_arm", "left") and out.count(("mirror_arm", "left")) == 1
    assert not any(c[0] == "mirror" for c in out) and lg.mirror_arming
    assert lg.status()["mirror_arming"] == "left"
    out = hold_combo(lg, 1.92, 2.1, buttons=(L.DPAD_LEFT, L.CROSS))
    assert out == [("hold", ("left", "right")), ("mirror", "left")]
    assert not lg.mirror_arming and lg.status()["mirror_arming"] is None
    assert hold_combo(lg, 2.12, 4.0, buttons=(L.DPAD_LEFT, L.CROSS)) == []   # nur einmal
    a, b = joy()
    assert lg.update(a, b, 4.1) == []                       # Loslassen danach: kein Abbruch-Signal
    # Erneutes Kreuz stoppt die laufende Fahrt wie immer.
    assert press(lg, L.CROSS, 4.2) == [("hold", ("left", "right")), ("cancel",)]


def test_mirror_right_any_press_order():
    lg = ArmTeleopLogic(selection="left")
    a, b = joy([L.CROSS])
    lg.update(a, b, 0.0)                                    # erst Kreuz ...
    out = hold_combo(lg, 0.02, 2.1, buttons=(L.CROSS, L.DPAD_RIGHT))   # ... dann rechts
    assert ("mirror_arm", "right") in out and ("mirror", "right") in out
    assert lg.selection == "right"


def test_mirror_aborts_on_early_release_and_restarts():
    lg = ArmTeleopLogic()
    hold_combo(lg, 0.0, 1.5, buttons=(L.DPAD_LEFT, L.CROSS))
    a, b = joy([L.DPAD_LEFT])                               # Kreuz los
    assert lg.update(a, b, 1.52) == [("mirror_abort",)]
    assert not lg.mirror_arming
    out = hold_combo(lg, 1.6, 3.4, buttons=(L.DPAD_LEFT, L.CROSS))
    assert ("mirror_arm", "left") in out and not any(c[0] == "mirror" for c in out)
    assert ("mirror", "left") in hold_combo(lg, 3.42, 3.7, buttons=(L.DPAD_LEFT, L.CROSS))


@pytest.mark.parametrize("buttons", [
    (L.DPAD_LEFT,), (L.CROSS,), (L.DPAD_LEFT, L.DPAD_RIGHT, L.CROSS),
    (L.DPAD_UP, L.CROSS), (L.DPAD_DOWN, L.CROSS), (L.DPAD_LEFT, L.CIRCLE),
])
def test_mirror_needs_cross_and_exactly_one_side(buttons):
    lg = ArmTeleopLogic()
    out = hold_combo(lg, 0.0, 3.0, buttons=buttons)
    assert not any(c[0].startswith("mirror") for c in out)


def test_mirror_switching_side_restarts_hold():
    lg = ArmTeleopLogic()
    hold_combo(lg, 0.0, 1.5, buttons=(L.DPAD_LEFT, L.CROSS))
    out = hold_combo(lg, 1.52, 1.6, buttons=(L.DPAD_RIGHT, L.CROSS))
    assert out[-1] == ("mirror_arm", "right") and ("mirror_abort",) in out
    out = hold_combo(lg, 1.62, 3.4, buttons=(L.DPAD_RIGHT, L.CROSS))
    assert not any(c[0] == "mirror" for c in out)           # zaehlt ab 1.52 s neu
    assert ("mirror", "right") in hold_combo(lg, 3.42, 3.7, buttons=(L.DPAD_RIGHT, L.CROSS))


def test_mirror_gap_in_stream_restarts_hold():
    lg = ArmTeleopLogic()
    hold_combo(lg, 0.0, 1.0, buttons=(L.DPAD_LEFT, L.CROSS))
    out = hold_combo(lg, 2.5, 2.6, buttons=(L.DPAD_LEFT, L.CROSS))   # Luecke 1.5 s
    assert ("mirror_arm", "left") in out and not any(c[0] == "mirror" for c in out)
    assert not any(c[0] == "mirror" for c in hold_combo(lg, 2.62, 4.4, buttons=(L.DPAD_LEFT, L.CROSS)))
    assert ("mirror", "left") in hold_combo(lg, 4.42, 4.6, buttons=(L.DPAD_LEFT, L.CROSS))


def test_mirror_blocked_by_estop_and_ps_aborts_hold():
    lg = ArmTeleopLogic()
    lg.latch_estop()
    out = hold_combo(lg, 0.0, 3.0, buttons=(L.DPAD_LEFT, L.CROSS))
    assert not any(c[0].startswith("mirror") for c in out)
    lg.clear_estop()
    hold_combo(lg, 3.1, 4.0, buttons=(L.DPAD_LEFT, L.CROSS))
    a, b = joy([L.DPAD_LEFT, L.CROSS, L.PS])                # PS = NOT-HALT mitten im Halten
    out = lg.update(a, b, 4.02)
    assert ("mirror_abort",) in out and ("estop",) in out and not lg.mirror_arming
    out = hold_combo(lg, 4.04, 7.0, buttons=(L.DPAD_LEFT, L.CROSS))
    assert not any(c[0] == "mirror" for c in out)


def test_mirror_blocked_in_walk_mode_and_with_l2_held():
    lg = ArmTeleopLogic(trigger_modes=True, walk_deadman=True)
    lg.set_robot_mode("walk")                               # GEHEN: Arme gesperrt
    out = hold_combo(lg, 0.0, 3.0, buttons=(L.DPAD_LEFT, L.CROSS))
    assert not any(c[0].startswith("mirror") for c in out)
    lg = ArmTeleopLogic(walk_deadman=True)
    out = hold_combo(lg, 0.0, 3.0, buttons=(L.DPAD_LEFT, L.CROSS, L.L2), L2_AXIS=1.0)
    assert not any(c[0].startswith("mirror") for c in out)
    # L2 erst mitten im Halten: Abbruch.
    lg = ArmTeleopLogic(walk_deadman=True)
    hold_combo(lg, 0.0, 1.0, buttons=(L.DPAD_LEFT, L.CROSS))
    out = hold_combo(lg, 1.02, 3.0, buttons=(L.DPAD_LEFT, L.CROSS, L.L2), L2_AXIS=1.0)
    assert ("mirror_abort",) in out and not any(c[0] == "mirror" for c in out)


def test_mirror_full_estop_combo_has_priority():
    lg = ArmTeleopLogic(ps_estop=False)
    hold_combo(lg, 0.0, 1.0, buttons=(L.DPAD_LEFT, L.CROSS))
    out = hold_combo(lg, 1.02, 3.2, buttons=(L.DPAD_LEFT, L.CROSS) + COMBO)
    assert ("mirror_abort",) in out and ("full_estop",) in out
    assert not any(c[0] == "mirror" for c in out)


def test_mirror_disabled_with_zero_hold_and_timeout_helper():
    lg = ArmTeleopLogic(mirror_hold_s=0.0)
    out = hold_combo(lg, 0.0, 3.0, buttons=(L.DPAD_LEFT, L.CROSS))
    assert not any(c[0].startswith("mirror") for c in out)
    assert ("cancel",) in out and lg.selection == "left"    # Einzelfunktionen bleiben
    lg = ArmTeleopLogic()
    hold_combo(lg, 0.0, 0.5, buttons=(L.DPAD_RIGHT, L.CROSS))
    assert lg.abort_mirror() and not lg.mirror_arming and not lg.abort_mirror()


def test_mirror_combo_xbox_a_is_cross():
    """Xbox: A liegt auf Kreuz (ps4_layout) -> Steuerkreuz + A spiegelt."""
    assert ArmTeleopLogic.mirror_combo(joy([L.DPAD_LEFT, L.CROSS])[1]) == "left"
    assert ArmTeleopLogic.mirror_combo(joy([L.DPAD_RIGHT, L.CROSS])[1]) == "right"
    assert ArmTeleopLogic.mirror_combo(joy([L.DPAD_LEFT, L.SQUARE])[1]) is None   # Xbox-X


def test_waist_r2_hold_aborts_mirror_and_never_mirrors():
    """Spiegeln + Hueftdrehung: R2 halten bricht ein laufendes Spiegel-Halten
    ab; Steuerkreuz + Kreuz unter R2 spiegelt nicht (Kreuz = Stopp)."""
    lg = _grasp()
    out = hold_combo(lg, 0.0, 1.0, buttons=(L.DPAD_LEFT, L.CROSS))
    assert ("mirror_arm", "left") in out and lg.mirror_arming
    out = trig(lg, 1.02, (L.DPAD_LEFT, L.CROSS), R2_AXIS=1.0)
    assert ("mirror_abort",) in out and ("waist", "start") in out
    assert not lg.mirror_arming
    trig(lg, 1.04, R2_AXIS=1.0)
    out = hold_combo(lg, 1.06, 4.0, buttons=(L.DPAD_LEFT, L.CROSS), R2_AXIS=1.0)
    assert not any(c[0] in ("mirror", "mirror_arm") for c in out)
    assert ("waist_stop",) in out and lg.waist_mode


# ── L3 = Ablauf weiter ────────────────────────────────────────────────────
def test_l3_sends_seq_next_in_grasp():
    lg = ArmTeleopLogic(walk_deadman=True, trigger_modes=True)
    assert ("seq_next",) in press(lg, L.L3)
    lg.robot_mode = "grasp"
    assert ("seq_next",) in press(lg, L.L3, 1.0)


def test_l3_without_effect_in_walk_estop_and_waist():
    lg = ArmTeleopLogic(walk_deadman=True, trigger_modes=True, waist_yaw=True)
    trig(lg, 0.0, L2_AXIS=1.0)                               # GEHEN
    trig(lg, 0.1)
    assert ("seq_next",) not in press(lg, L.L3, 0.2)
    trig(lg, 0.3, R2_AXIS=1.0)                               # GREIFEN, R2 gehalten = Huefte
    assert lg.waist_mode
    assert ("seq_next",) not in trig(lg, 0.4, [L.L3], R2_AXIS=1.0)
    trig(lg, 0.5)
    press(lg, L.PS, 0.6)                                     # NOT-HALT
    assert ("seq_next",) not in press(lg, L.L3, 0.7)


def test_l3_blocks_jog_until_stick_released_and_centered():
    """Beim Druecken kippt der Stick leicht: kein Hand-Ziel (es braeche die
    gerade gestartete geplante Fahrt ab), bis L3 los und der Stick mittig ist."""
    lg = ArmTeleopLogic()
    a, b = joy([L.L3], LY=0.6)
    assert ("seq_next",) in lg.update(a, b, 0.0)
    assert lg.twist(a) == {}
    a, b = joy(LY=0.6)                                       # los, Stick noch ausgelenkt
    lg.update(a, b, 0.05)
    assert lg.twist(a) == {}
    a, b = joy()                                             # Mitte -> Sperre weg
    lg.update(a, b, 0.1)
    assert lg.twist(a) == {}
    a, b = joy(LY=0.6)
    lg.update(a, b, 0.15)
    assert "right" in lg.twist(a)

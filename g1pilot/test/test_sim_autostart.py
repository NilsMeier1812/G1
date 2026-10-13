#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sim-Auto-Start (GREIFEN) mit Quittung -- teleoperation/sim_autostart.py.

Bug 2026-10-13: Beim Sim-Start schickten die GUIs ENABLE + BALANCING nur EINMAL
nach 3 s. Waren arm_controller/loco_sim da noch nicht bereit, gingen beide
Nachrichten verloren -> GREIFEN angezeigt, Arme aus. Hier geprueft:
  1. Logik (ohne ROS): wiederholen bis Quittung, nur Fehlendes, Abbruch, Timeout.
  2. Ueber echtes DDS (rclpy): mehrere Starts, bei denen die Regler-Attrappen
     ERST NACH dem ersten Versuch auftauchen (alter Code: Arme bleiben aus).
"""
import threading
import time

import pytest

from g1pilot.teleoperation.sim_autostart import SimAutoStart


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make(**kw):
    clock = Clock()
    sent = []
    done = []
    a = SimAutoStart(lambda: sent.append("enable"), lambda: sent.append("balance"),
                     clock=clock, on_done=done.append, **kw)
    return a, clock, sent, done


# ── 1. Logik ─────────────────────────────────────────────────────────────
def test_first_attempt_is_immediate_and_repeats_until_confirmed():
    a, clock, sent, done = make(period_s=1.0)
    a.start()
    a.tick(None, None)
    assert sent == ["enable", "balance"]
    clock.t = 0.5
    a.tick(None, None)
    assert sent == ["enable", "balance"]          # Periode noch nicht um
    clock.t = 1.0
    a.tick(None, None)
    assert sent == ["enable", "balance"] * 2      # Regler lud noch -> erneut
    clock.t = 1.2
    a.tick(True, "stand")
    assert done == [True] and not a.active
    clock.t = 5.0
    a.tick(False, "hold")
    assert len(sent) == 4                         # fertig -> sendet nie mehr


def test_only_missing_part_is_resent():
    a, clock, sent, done = make(period_s=1.0)
    a.start()
    a.tick(None, None)
    clock.t = 1.0
    a.tick(True, "hold")                          # Arme da, Stand fehlt
    assert sent[2:] == ["balance"]
    clock.t = 2.0
    a.tick(False, "stand")                        # Stand da, Arme fehlen
    assert sent[3:] == ["enable"]
    clock.t = 2.1
    a.tick(True, "stand")
    assert done == [True]
    assert (a.sent_enable, a.sent_balance) == (2, 2)


def test_already_up_sends_nothing():
    """Laeuft schon alles (z.B. GUI neu gestartet), kein zweites BALANCING."""
    a, clock, sent, done = make()
    a.start()
    a.tick(True, "walk")
    assert sent == [] and done == [True]


def test_cancel_stops_sending():
    a, clock, sent, done = make(period_s=1.0)
    a.start()
    a.tick(None, None)
    a.cancel("NOT-HALT")
    clock.t = 3.0
    a.tick(None, None)
    assert sent == ["enable", "balance"] and done == []


def test_timeout_reports_failure():
    a, clock, sent, done = make(period_s=1.0, timeout_s=10.0)
    a.start()
    for i in range(12):
        clock.t = float(i)
        a.tick(False, "hold")
    assert done == [False] and not a.active
    assert a.sent_enable == 11                   # t = 0..10 s, je 1 s


# ── 2. Ueber DDS, mehrere Starts mit spaet startenden Reglern ────────────
rclpy = pytest.importorskip("rclpy")


def _run_start(delay_s, use_autostart):
    """Ein 'Sim-Start': GUI-Node feuert bei t=0 (entspricht den 3 s), die
    Regler-Attrappe (arm_controller + loco_sim) erscheint erst nach delay_s.
    -> (arme_aktiv, stand, Anzahl ENABLE, Anzahl BALANCING beim Regler)."""
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import Bool, String
    from g1pilot.teleoperation.sim_autostart import ARMS_STATE_TOPIC, LOCO_STATE_TOPIC

    latched = QoSProfile(depth=1)
    latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    ex = MultiThreadedExecutor(context=ctx)
    gui = rclpy.create_node("autostart_gui", context=ctx)
    pub_en = gui.create_publisher(Bool, "/g1pilot/arms/enabled", 10)
    pub_bal = gui.create_publisher(Bool, "/g1pilot/start_balancing", 10)
    st = {"arms": None, "loco": None}
    gui.create_subscription(Bool, ARMS_STATE_TOPIC,
                            lambda m: st.__setitem__("arms", m.data), latched)
    gui.create_subscription(String, LOCO_STATE_TOPIC,
                            lambda m: st.__setitem__("loco", m.data), latched)
    ex.add_node(gui)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    ctrl = {"enabled": False, "state": "hold", "n_en": 0, "n_bal": 0}

    def make_controller():
        n = rclpy.create_node("fake_controllers", context=ctx)
        p_arms = n.create_publisher(Bool, ARMS_STATE_TOPIC, latched)
        p_loco = n.create_publisher(String, LOCO_STATE_TOPIC, latched)

        def on_en(m):
            ctrl["n_en"] += 1
            ctrl["enabled"] = bool(m.data)

        def on_bal(m):
            if m.data:
                ctrl["n_bal"] += 1
                ctrl["state"] = "stand"

        n.create_subscription(Bool, "/g1pilot/arms/enabled", on_en, 10)
        n.create_subscription(Bool, "/g1pilot/start_balancing", on_bal, 10)
        n.create_timer(0.2, lambda: (p_arms.publish(Bool(data=ctrl["enabled"])),
                                     p_loco.publish(String(data=ctrl["state"]))))
        ex.add_node(n)
        return n

    send_en = lambda: pub_en.publish(Bool(data=True))   # noqa: E731
    send_bal = lambda: pub_bal.publish(Bool(data=True))  # noqa: E731
    a = SimAutoStart(send_en, send_bal, period_s=0.5, timeout_s=15.0)
    time.sleep(0.5)                       # GUI-Publisher stehen (wie nach 3 s)
    if use_autostart:
        a.start()
    else:                                 # alter Code: genau ein Versuch
        send_en()
        send_bal()
    ctrl_node = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < delay_s + 5.0:
        if ctrl_node is None and time.monotonic() - t0 >= delay_s:
            ctrl_node = make_controller()
        a.tick(st["arms"], st["loco"])
        if use_autostart and not a.active:
            break
        time.sleep(0.05)
    time.sleep(0.5)
    result = (ctrl["enabled"], ctrl["state"], ctrl["n_en"], ctrl["n_bal"])
    ex.shutdown()
    rclpy.shutdown(context=ctx)
    spin.join(timeout=2.0)
    return result


@pytest.mark.parametrize("delay_s", [0.0, 1.0, 2.5, 4.0, 6.0])
def test_dds_repeated_starts_with_late_controllers(delay_s):
    enabled, state, n_en, n_bal = _run_start(delay_s, use_autostart=True)
    assert enabled and state == "stand", (delay_s, n_en, n_bal)
    # Nach der Quittung wird nichts mehr nachgeschoben (hoechstens die eine
    # Anforderung, die beim Erscheinen der Regler schon unterwegs war).
    assert n_en <= 2 and n_bal <= 2


def test_dds_old_single_shot_loses_request_when_controllers_late():
    """Gegenprobe = der gemeldete Fehler: ein einziger Versuch, Regler kommen
    spaeter -> Arme bleiben aus, Roboter bleibt in HOLD."""
    enabled, state, n_en, n_bal = _run_start(2.0, use_autostart=False)
    assert (enabled, state, n_en, n_bal) == (False, "hold", 0, 0)

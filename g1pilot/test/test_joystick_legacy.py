#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Alter joystick-Node (G1_PS4_ARMS=0, Gehen auf dem echten G1) bei Abriss.

Befund (Review 2026-10-16): Bei einer Trennung starb der Lese-Thread still,
der Timer publizierte den letzten Zustand weiter -- mit gedruecktem Deadman
lief der G1 weiter. Jetzt: getrennt bzw. Funk haengt -> nichts publizieren,
dann wiederverbinden. Ohne echtes evdev (Attrappe).
"""
import sys
import threading
import time
import types

import pytest

rclpy = pytest.importorskip("rclpy")

EV_ABS, EV_KEY = 3, 1
ABS_X, ABS_Y = 0, 1
BTN_SOUTH, BTN_NORTH, BTN_SELECT = 304, 307, 314


class Ev:
    def __init__(self, t, c, v):
        self.type, self.code, self.value = t, c, v


class FakeDev:
    def __init__(self, name, path, events=(), fail_after=True, hold=None, uniq="aa"):
        self.name, self.path, self.uniq, self.phys = name, path, uniq, "p"
        self._events = list(events)
        self._fail = fail_after
        self.hold = hold or threading.Event()

    def capabilities(self):
        return {EV_ABS: [(ABS_X, None), (ABS_Y, None)],
                EV_KEY: [BTN_SOUTH, BTN_NORTH, BTN_SELECT]}

    def absinfo(self, code):
        return types.SimpleNamespace(min=-1, max=1)

    def read_loop(self):
        for e in self._events:
            yield e
        self.hold.wait()
        if self._fail:
            raise OSError(19, "No such device")

    def close(self):
        pass


@pytest.fixture
def jm(monkeypatch):
    fake = types.ModuleType("evdev")
    fake.ecodes = types.SimpleNamespace(EV_ABS=EV_ABS, EV_KEY=EV_KEY, BTN_NORTH=BTN_NORTH)
    fake.devices = {}
    fake.list_devices = lambda: list(fake.devices)
    fake.InputDevice = lambda p: fake.devices[p]
    monkeypatch.setitem(sys.modules, "evdev", fake)
    import importlib
    import g1pilot.teleoperation.joystick as mod
    importlib.reload(mod)
    rclpy.init()
    yield mod, fake
    rclpy.shutdown()


def _node(mod, **params):
    node = mod.ManualJoystick()
    node.reconnect_s = 0.05
    out = []
    node.publisher.publish = out.append
    return node, out


def _wait(pred, t=2.0):
    end = time.monotonic() + t
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_disconnect_stops_publishing_then_reconnects(jm):
    mod, fake = jm
    hold = threading.Event()
    dev = FakeDev("Wireless Controller", "/dev/input/event5",
                  [Ev(EV_KEY, BTN_SELECT, 1), Ev(EV_ABS, ABS_Y, -1)], hold=hold)
    fake.devices = {dev.path: dev}
    node, out = _node(mod)
    try:
        assert _wait(lambda: node.device is not None and node.buttons[2] == 1)
        node.publish_joy()
        assert out[-1].buttons[2] == 1 and out[-1].axes[1] == -1.0   # Deadman + vor
        fake.devices = {}
        hold.set()                                                   # Trennung
        assert _wait(lambda: node.device is None)
        n = len(out)
        for _ in range(5):
            node.publish_joy()
        assert len(out) == n, "getrennt: kein eingefrorener Zustand mehr"
        dev2 = FakeDev("Wireless Controller", "/dev/input/event6")
        fake.devices = {dev2.path: dev2}
        assert _wait(lambda: node.device is dev2)
        node.publish_joy()
        assert list(out[-1].buttons) == [0, 0, 0], "nach Wiederverbinden neutral"
    finally:
        node.destroy_node()


def test_stalled_radio_stops_publishing(jm):
    mod, fake = jm
    dev = FakeDev("Wireless Controller", "/dev/input/event5")
    motion = FakeDev("Wireless Controller Motion Sensors", "/dev/input/event7",
                     [Ev(EV_ABS, 0, 1)], fail_after=False)
    fake.devices = {dev.path: dev, motion.path: motion}
    node, out = _node(mod)
    try:
        assert _wait(lambda: node._alive_t is not None)
        node.publish_joy()
        assert len(out) == 1
        node._alive_t -= 1.0             # keine Funkberichte mehr
        node.publish_joy()
        assert len(out) == 1
    finally:
        node.destroy_node()     # Lese-Threads (daemon) bleiben blockiert


def test_no_controller_publishes_nothing(jm):
    mod, fake = jm
    fake.devices = {}
    node, out = _node(mod)
    try:
        time.sleep(0.1)
        node.publish_joy()
        assert out == []
    finally:
        node.destroy_node()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ps4_joystick gegen ein simuliertes evdev-Geraet (braucht rclpy, kein Controller).

Prueft: Warten ohne Controller (nichts publizieren), Verbinden, feste Belegung
im Joy, Abriss (nichts mehr publizieren, Zustand neutral), Wiederverbinden,
LED ueber sysfs und Vibration ueber Force-Feedback.
"""
import json
import os
import queue
import threading
import time
import types

import pytest

rclpy = pytest.importorskip("rclpy")
import rclpy.executors                    # noqa: E402
from sensor_msgs.msg import Joy            # noqa: E402
from std_msgs.msg import String            # noqa: E402

from g1pilot.teleoperation import ps4_joystick as pj   # noqa: E402
from g1pilot.teleoperation import ps4_layout as L      # noqa: E402

EV_FF, FF_RUMBLE = 0x15, 0x50


class FakeAbs:
    def __init__(self, value, lo=0, hi=255):
        self.value, self.min, self.max = value, lo, hi


class FakeDevice:
    """Minimales evdev.InputDevice: Ereignisse kommen aus einer Queue,
    None in der Queue = Abriss (OSError wie bei Bluetooth-Verlust)."""

    def __init__(self, name, path="/dev/input/event7"):
        self.name, self.path = name, path
        self.q = queue.Queue()
        self.closed = False
        self.ff_written = []

    def capabilities(self):
        abs_ = [(c, FakeAbs(128)) for c in (L.ABS_X, L.ABS_Y, L.ABS_RX, L.ABS_RY)]
        abs_ += [(L.ABS_Z, FakeAbs(0)), (L.ABS_RZ, FakeAbs(0)),
                 (L.ABS_HAT0X, FakeAbs(0, -1, 1)), (L.ABS_HAT0Y, FakeAbs(0, -1, 1))]
        return {L.EV_ABS: abs_, L.EV_KEY: list(range(0x130, 0x13f)), EV_FF: [FF_RUMBLE]}

    def read_loop(self):
        while True:
            ev = self.q.get()
            if ev is None:
                raise OSError(19, "No such device")
            yield types.SimpleNamespace(type=ev[0], code=ev[1], value=ev[2])

    def upload_effect(self, effect):
        self.last_effect = effect
        return 3

    def erase_effect(self, eid):
        pass

    def write(self, etype, code, value):
        self.ff_written.append((etype, code, value))

    def close(self):
        self.closed = True


class FakeEvdev:
    def __init__(self):
        self.devices = {}       # path -> FakeDevice
        self.ff = types.SimpleNamespace(
            Rumble=lambda **kw: kw, Trigger=lambda *a: a, Replay=lambda *a: a,
            EffectType=lambda **kw: kw, Effect=lambda *a: a)

    def list_devices(self):
        return list(self.devices)

    def InputDevice(self, path):   # noqa: N802 -- evdev-API
        return self.devices[path]


@pytest.fixture
def env(monkeypatch, tmp_path):
    fake = FakeEvdev()
    monkeypatch.setattr(pj, "evdev", fake)
    monkeypatch.setattr(pj, "ecodes", types.SimpleNamespace(
        EV_ABS=L.EV_ABS, EV_KEY=L.EV_KEY, EV_FF=EV_FF, FF_RUMBLE=FF_RUMBLE))
    leds = {}
    for c in ("red", "green", "blue"):
        d = tmp_path / f"led:{c}"
        d.mkdir()
        (d / "brightness").write_text("0")
        leds[c] = str(d)
    monkeypatch.setattr(pj.Ps4Joystick, "_find_led", staticmethod(lambda dev: ("split", leds)))
    rclpy.init()
    node = pj.Ps4Joystick()
    node.reconnect_s = 0.05
    probe = rclpy.create_node("ps4_probe")
    got = []
    probe.create_subscription(Joy, "/g1pilot/ps4/joy", got.append, 50)
    fb = probe.create_publisher(String, "/g1pilot/ps4/feedback", 10)
    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(node)
    ex.add_node(probe)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    yield types.SimpleNamespace(fake=fake, node=node, got=got, fb=fb, leds=leds)
    node._stop = True
    ex.shutdown()
    node.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()


def wait_for(cond, timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_lifecycle(env):
    # 1) Kein Controller: nichts publizieren
    time.sleep(0.4)
    assert env.got == []
    # Fremdgeraete werden ignoriert, Sensor-Teilgeraet auch
    env.fake.devices["/dev/input/event3"] = FakeDevice("AT Keyboard", "/dev/input/event3")
    env.fake.devices["/dev/input/event8"] = FakeDevice("Wireless Controller Motion Sensors",
                                                       "/dev/input/event8")
    time.sleep(0.3)
    assert env.got == []
    # 2) Controller erscheint (USB-Name) -> Joy in fester Belegung, neutral
    dev = FakeDevice("Sony Interactive Entertainment Wireless Controller")
    env.fake.devices[dev.path] = dev
    assert wait_for(lambda: len(env.got) > 3)
    m = env.got[-1]
    assert len(m.axes) == L.NUM_AXES and len(m.buttons) == L.NUM_BUTTONS
    assert all(abs(a) < 0.01 for a in m.axes) and not any(m.buttons)
    # 3) Ereignisse -> Belegung
    dev.q.put((L.EV_ABS, L.ABS_Y, 0))          # linker Stick ganz oben
    dev.q.put((L.EV_KEY, 0x131, 1))            # Kreis
    dev.q.put((L.EV_ABS, L.ABS_HAT0X, -1))     # Steuerkreuz links
    assert wait_for(lambda: env.got[-1].buttons[L.CIRCLE] == 1)
    m = env.got[-1]
    assert m.axes[L.LY] == pytest.approx(1.0) and m.buttons[L.DPAD_LEFT] == 1
    # 4) Abriss -> keine Nachrichten mehr (Empfaenger merkt den Timeout).
    #    Wie im Kernel: Geraeteknoten verschwindet, read_loop wirft OSError.
    del env.fake.devices[dev.path]
    dev.q.put(None)
    time.sleep(0.15)
    n = len(env.got)
    time.sleep(0.4)
    assert len(env.got) == n
    assert env.node.state.axes == [0.0] * L.NUM_AXES
    # 5) Wiederverbinden (Controller kommt zurueck) -> wieder neutral
    dev2 = FakeDevice("Wireless Controller")
    env.fake.devices[dev2.path] = dev2
    assert wait_for(lambda: len(env.got) > n + 3)
    assert env.got[-1].axes[L.LY] == pytest.approx(0.0, abs=0.01)
    assert env.got[-1].buttons[L.CIRCLE] == 0


def test_feedback_led_and_rumble(env):
    dev = FakeDevice("Wireless Controller")
    env.fake.devices[dev.path] = dev
    assert wait_for(lambda: len(env.got) > 1)
    env.fb.publish(String(data=json.dumps(
        {"rgb": [0, 200, 60], "rumble": {"ms": 100, "strong": 0.0, "weak": 0.5}})))
    rd = lambda c: open(os.path.join(env.leds[c], "brightness")).read()   # noqa: E731
    assert wait_for(lambda: rd("green") == "200")
    assert rd("red") == "0" and rd("blue") == "60"
    assert wait_for(lambda: dev.ff_written == [(EV_FF, 3, 1)])

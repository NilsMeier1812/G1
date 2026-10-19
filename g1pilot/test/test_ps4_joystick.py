#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ps4_joystick gegen ein simuliertes evdev-Geraet (braucht rclpy, kein Controller).

Prueft: Warten ohne Controller (nichts publizieren), Verbinden, feste Belegung
im Joy, Abriss (nichts mehr publizieren, Zustand neutral), Wiederverbinden,
LED ueber sysfs und Vibration ueber Force-Feedback. Dazu der Xbox-Controller:
Erkennung, Belegung, Auswahl PS4/Xbox, Xbox-Taste auf dem Bluetooth-
Teilgeraet, Lebendpruefung ueber den HID-Rohstrom.
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

    def __init__(self, name, path="/dev/input/event7", phys="", uniq="", abs_caps=None,
                 vendor=None, keys=None):
        self.name, self.path = name, path
        self.phys, self.uniq = phys, uniq
        self.abs_caps = abs_caps
        self.keys = keys
        self.info = types.SimpleNamespace(vendor=vendor)
        self.q = queue.Queue()
        self.closed = False
        self.ff_written = []

    def capabilities(self):
        if self.abs_caps is not None or self.keys is not None:
            caps = {L.EV_ABS: list((self.abs_caps or {}).items()),
                    L.EV_KEY: list(self.keys or [])}
            if self.abs_caps:
                caps[EV_FF] = [FF_RUMBLE]
            return caps
        abs_ = [(c, FakeAbs(128)) for c in (L.ABS_X, L.ABS_Y, L.ABS_RX, L.ABS_RY)]
        abs_ += [(L.ABS_Z, FakeAbs(0)), (L.ABS_RZ, FakeAbs(0)),
                 (L.ABS_HAT0X, FakeAbs(0, -1, 1)), (L.ABS_HAT0Y, FakeAbs(0, -1, 1))]
        return {L.EV_ABS: abs_, L.EV_KEY: list(range(0x130, 0x13f)), EV_FF: [FF_RUMBLE]}

    def read_loop(self):
        self.reading = True
        while True:
            ev = self.q.get()
            if ev is None:
                raise OSError(19, "No such device")
            yield types.SimpleNamespace(type=ev[0], code=ev[1], value=ev[2])

    def upload_effect(self, effect):
        self.last_effect = effect
        return 3

    def erase_effect(self, eid):
        self.erased = getattr(self, "erased", 0) + 1

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
def env(monkeypatch, tmp_path, request):
    """Node mit simuliertem evdev. Indirekt parametrisierbar mit ROS-Args,
    z.B. ["-p", "controller:=xbox"]."""
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
    hidraw = {"path": None}
    monkeypatch.setattr(pj.Ps4Joystick, "_find_hidraw", staticmethod(lambda dev: hidraw["path"]))
    ros_args = getattr(request, "param", None)
    rclpy.init(args=["--ros-args"] + list(ros_args) if ros_args else None)
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
    yield types.SimpleNamespace(fake=fake, node=node, got=got, fb=fb, leds=leds, hidraw=hidraw)
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


def test_rumble_stop_never_uploads_endless_effect(env):
    """{"rumble": {"ms": 0}} (Voll-Notaus abgebrochen) stoppt die laufende
    Vibration und laedt KEIN neues Effekt hoch -- Replay(0) waere endlos."""
    dev = FakeDevice("Wireless Controller")
    env.fake.devices[dev.path] = dev
    assert wait_for(lambda: len(env.got) > 1)
    env.fb.publish(String(data=json.dumps({"rumble": {"ms": 2300, "strong": 1.0, "weak": 1.0}})))
    assert wait_for(lambda: len(dev.ff_written) == 1)
    assert dev.last_effect[4] == (2300, 0)            # Replay(Laenge, Verzoegerung)
    env.fb.publish(String(data=json.dumps({"rumble": {"ms": 0}})))
    assert wait_for(lambda: getattr(dev, "erased", 0) == 1)
    time.sleep(0.1)
    assert len(dev.ff_written) == 1 and dev.last_effect[4] == (2300, 0)
    env.fb.publish(String(data=json.dumps({"rumble": {"ms": "kaputt"}})))   # ignoriert
    time.sleep(0.1)
    assert len(dev.ff_written) == 1


def test_stall_detection_motion_sensors(env):
    """Bluetooth haengt, ohne dass evdev einen Fehler meldet: das Sensor-
    Teilgeraet liefert nichts mehr -> nichts publizieren; Daten wieder da ->
    weiter. Ein Sensor-Geraet eines ANDEREN Controllers zaehlt nicht."""
    other = FakeDevice("Wireless Controller Motion Sensors", "/dev/input/event9",
                       phys="aa:bb:cc:00:00:02")
    motion = FakeDevice("Wireless Controller Motion Sensors", "/dev/input/event8",
                        phys="aa:bb:cc:00:00:01")
    dev = FakeDevice("Wireless Controller", phys="aa:bb:cc:00:00:01")
    env.fake.devices[other.path] = other
    env.fake.devices[motion.path] = motion
    env.fake.devices[dev.path] = dev
    feed = {"on": True, "run": True}

    def feeder():                      # Sensor-Berichte ~200 Hz
        while feed["run"]:
            if feed["on"]:
                motion.q.put((0x04, 0x05, int(time.time() * 1e6) & 0x7fffffff))   # MSC_TIMESTAMP
            time.sleep(0.005)
    th = threading.Thread(target=feeder, daemon=True)
    th.start()
    try:
        assert wait_for(lambda: len(env.got) > 10)
        dev.q.put((L.EV_ABS, L.ABS_Y, 0))          # Stick voll ausgelenkt
        assert wait_for(lambda: env.got[-1].axes[L.LY] == pytest.approx(1.0))
        # Funk haengt: keine Sensor-Ereignisse mehr, evdev meldet nichts
        feed["on"] = False
        time.sleep(0.3 + 0.15)
        n = len(env.got)
        time.sleep(0.5)
        assert len(env.got) == n, "eingefrorener Stick wird weiter publiziert"
        # Daten kommen wieder -> weiter publizieren
        feed["on"] = True
        assert wait_for(lambda: len(env.got) > n + 5)
        assert not other.q.qsize() and other.closed       # fremdes Geraet nicht benutzt
    finally:
        feed["run"] = False


def test_no_motion_device_keeps_old_behaviour(env):
    """Sensor-Teilgeraet nicht zuzuordnen (keine phys/uniq) oder liefert nie
    etwas: ohne Lebendpruefung weiter publizieren wie bisher."""
    motion = FakeDevice("Wireless Controller Motion Sensors", "/dev/input/event8",
                        phys="usb-1")
    dev = FakeDevice("Wireless Controller", phys="usb-1")
    env.fake.devices[motion.path] = motion            # zugeordnet, aber stumm
    env.fake.devices[dev.path] = dev
    assert wait_for(lambda: len(env.got) > 3)
    n = len(env.got)
    time.sleep(0.6)
    assert len(env.got) > n + 10


# ══ Xbox-Controller ══════════════════════════════════════════════════════
XPAD = {L.ABS_X: FakeAbs(0, -32768, 32767), L.ABS_Y: FakeAbs(0, -32768, 32767),
        L.ABS_RX: FakeAbs(0, -32768, 32767), L.ABS_RY: FakeAbs(0, -32768, 32767),
        L.ABS_Z: FakeAbs(0, 0, 1023), L.ABS_RZ: FakeAbs(0, 0, 1023),
        L.ABS_HAT0X: FakeAbs(0, -1, 1), L.ABS_HAT0Y: FakeAbs(0, -1, 1)}
GENERIC_BT = {L.ABS_X: FakeAbs(32768, 0, 65535), L.ABS_Y: FakeAbs(32768, 0, 65535),
              L.ABS_Z: FakeAbs(32768, 0, 65535), L.ABS_RZ: FakeAbs(32768, 0, 65535),
              L.ABS_BRAKE: FakeAbs(0, 0, 1023), L.ABS_GAS: FakeAbs(0, 0, 1023),
              L.ABS_HAT0X: FakeAbs(0, -1, 1), L.ABS_HAT0Y: FakeAbs(0, -1, 1)}


def xbox(name="Microsoft Xbox Series S|X Controller", path="/dev/input/event7", **kw):
    kw.setdefault("abs_caps", XPAD)
    return FakeDevice(name, path, vendor=L.XBOX_VENDOR, **kw)


def test_xbox_usb_lifecycle(env):
    """xpad (USB): erkannt, neutral, Belegung nach Position, Trigger als Achse,
    Vibration, keine LED, Abriss -> nichts mehr publizieren."""
    dev = xbox()
    env.fake.devices[dev.path] = dev
    assert wait_for(lambda: len(env.got) > 3)
    m = env.got[-1]
    assert m.header.frame_id == "xbox" and env.node.kind == "xbox"
    assert all(abs(a) < 0.01 for a in m.axes) and not any(m.buttons)
    dev.q.put((L.EV_KEY, 0x130, 1))             # A
    dev.q.put((L.EV_KEY, 0x133, 1))             # X (links)
    dev.q.put((L.EV_ABS, L.ABS_Z, 1023))        # LT voll
    dev.q.put((L.EV_ABS, L.ABS_Y, -32768))      # linker Stick oben
    assert wait_for(lambda: env.got[-1].buttons[L.SQUARE] == 1)
    m = env.got[-1]
    assert m.buttons[L.CROSS] == 1 and m.buttons[L.TRIANGLE] == 0
    assert m.axes[L.L2_AXIS] == pytest.approx(1.0) and m.axes[L.LY] == pytest.approx(1.0)
    env.fb.publish(String(data=json.dumps(
        {"rgb": [255, 0, 0], "rumble": {"ms": 2300, "strong": 1.0, "weak": 1.0}})))
    assert wait_for(lambda: dev.ff_written == [(EV_FF, 3, 1)])
    time.sleep(0.1)
    assert all(open(os.path.join(env.leds[c], "brightness")).read() == "0" for c in env.leds)
    del env.fake.devices[dev.path]
    dev.q.put(None)
    time.sleep(0.15)
    n = len(env.got)
    time.sleep(0.4)
    assert len(env.got) == n and env.node.state.buttons == [0] * L.NUM_BUTTONS


def test_ps4_preferred_when_both_connected(env):
    x = xbox(path="/dev/input/event5")
    p = FakeDevice("Wireless Controller", "/dev/input/event9")
    env.fake.devices[x.path] = x
    env.fake.devices[p.path] = p
    assert wait_for(lambda: len(env.got) > 3)
    assert env.got[-1].header.frame_id == "ps4" and x.closed and not p.closed


@pytest.mark.parametrize("env", [["-p", "controller:=xbox"]], indirect=True)
def test_controller_param_xbox_only(env):
    p = FakeDevice("Wireless Controller", "/dev/input/event9")
    env.fake.devices[p.path] = p
    time.sleep(0.3)
    assert env.got == []                        # PS4 wird ignoriert
    x = xbox(path="/dev/input/event5")
    env.fake.devices[x.path] = x
    assert wait_for(lambda: len(env.got) > 3)
    assert env.got[-1].header.frame_id == "xbox"


def test_xbox_bluetooth_generic_and_guide_on_consumer_control(env):
    """Bluetooth ohne xpadneo: rechter Stick auf ABS_Z/RZ, Trigger GAS/BRAKE;
    die Xbox-Taste kommt als KEY_HOMEPAGE vom Teilgeraet "... Consumer Control"
    (gleiche uniq). Das eines ANDEREN Controllers zaehlt nicht."""
    cc = FakeDevice("Xbox Wireless Controller Consumer Control", "/dev/input/event8",
                    uniq="98:7a:14:00:00:01", vendor=L.XBOX_VENDOR, keys=[L.KEY_HOMEPAGE])
    other = FakeDevice("Xbox Wireless Controller Consumer Control", "/dev/input/event9",
                       uniq="98:7a:14:00:00:02", vendor=L.XBOX_VENDOR, keys=[L.KEY_HOMEPAGE])
    dev = xbox("Xbox Wireless Controller", abs_caps=GENERIC_BT, uniq="98:7a:14:00:00:01")
    for d in (cc, other, dev):
        env.fake.devices[d.path] = d
    assert wait_for(lambda: len(env.got) > 3)
    assert env.node.state.generic_bt
    assert all(abs(a) < 0.01 for a in env.got[-1].axes)
    dev.q.put((L.EV_ABS, L.ABS_Z, 0))           # rechter Stick links
    dev.q.put((L.EV_ABS, L.ABS_GAS, 1023))      # RT
    cc.q.put((L.EV_KEY, L.KEY_HOMEPAGE, 1))     # Xbox-Taste
    assert wait_for(lambda: env.got[-1].buttons[L.PS] == 1)
    m = env.got[-1]
    assert m.axes[L.RX] == pytest.approx(1.0) and m.axes[L.R2_AXIS] == pytest.approx(1.0)
    assert m.axes[L.L2_AXIS] == 0.0
    # (FakeEvdev gibt bei jedem Oeffnen dasselbe Objekt -> "closed" sagt hier nichts)
    assert cc.reading and not getattr(other, "reading", False)
    cc.q.put((L.EV_KEY, L.KEY_HOMEPAGE, 0))
    assert wait_for(lambda: env.got[-1].buttons[L.PS] == 0)


def _fifo(tmp_path):
    path = str(tmp_path / "hidraw0")
    os.mkfifo(path)
    return path


def test_xbox_hidraw_stall_detection(env, tmp_path):
    """Bluetooth-Xbox, der durchgehend HID-Berichte sendet: nach 20 Berichten
    ohne Eingabe-Aenderung ist die Lebendpruefung scharf. Haengt der Funk
    (keine Berichte mehr, evdev meldet nichts) -> nichts publizieren."""
    path = _fifo(tmp_path)
    env.hidraw["path"] = path
    feed = {"on": True, "run": True}
    opened = threading.Event()

    def feeder():                               # ~125 Hz wie ein Funk-Controller
        fd = os.open(path, os.O_WRONLY)         # blockiert, bis der Node liest
        opened.set()
        while feed["run"]:
            if feed["on"]:
                os.write(fd, b"\x01" + bytes(16))
            time.sleep(0.008)
        os.close(fd)
    threading.Thread(target=feeder, daemon=True).start()
    dev = xbox("Xbox Wireless Controller", uniq="98:7a:14:00:00:01")
    env.fake.devices[dev.path] = dev
    try:
        assert opened.wait(2.0)
        assert wait_for(lambda: env.node._alive_t is not None)
        dev.q.put((L.EV_ABS, L.ABS_Z, 1023))    # LT gehalten (Gehen)
        dev.q.put((L.EV_ABS, L.ABS_Y, -32768))  # Stick voll vor
        assert wait_for(lambda: env.got[-1].axes[L.LY] == pytest.approx(1.0))
        time.sleep(0.5)                         # ruhig halten: Strom laeuft -> weiter
        n = len(env.got)
        time.sleep(0.3)
        assert len(env.got) > n + 5
        feed["on"] = False                      # Funk haengt
        time.sleep(0.3 + 0.15)
        n = len(env.got)
        time.sleep(0.5)
        assert len(env.got) == n, "eingefrorener Stick wird weiter publiziert"
        feed["on"] = True
        assert wait_for(lambda: len(env.got) > n + 5)
    finally:
        feed["run"] = False


def test_xbox_hidraw_only_on_change_never_arms(env, tmp_path):
    """Controller, der nur bei Aenderungen sendet: jeder HID-Bericht bringt
    eine evdev-Aenderung -> Lebendpruefung wird nie scharf, ruhig gehaltener
    Stick wird weiter publiziert (kein falscher Stopp)."""
    path = _fifo(tmp_path)
    env.hidraw["path"] = path
    dev = xbox("Xbox Wireless Controller", uniq="98:7a:14:00:00:01")
    env.fake.devices[dev.path] = dev
    fd = os.open(path, os.O_WRONLY)
    try:
        assert wait_for(lambda: len(env.got) > 3)
        for i in range(60):                     # 60 Aenderungen, je Bericht + evdev
            dev.q.put((L.EV_ABS, L.ABS_X, -1000 * (i + 1)))
            time.sleep(0.004)
            os.write(fd, b"\x01" + bytes(16))
            time.sleep(0.004)
        assert env.node._alive_t is None
        n = len(env.got)
        time.sleep(0.6)                          # Stick ruhig, kein Bericht
        assert len(env.got) > n + 10
    finally:
        os.close(fd)

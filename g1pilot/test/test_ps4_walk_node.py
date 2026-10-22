#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ps4_walk als echter Node (Real-Defaults) gegen simuliertes Joy.

Prueft /g1pilot/ps4/walk_cmd: nur bei gehaltenem L2 (nach einmal Loslassen),
Stopp beim Loslassen und bei Joy-Abriss, NOT-HALT sperrt bis START, Hinweis-
Vibration, wenn der G1 nicht im Walk-Modus steht.
"""
import json
import os
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
import rclpy.executors                                  # noqa: E402
from geometry_msgs.msg import Twist                      # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile       # noqa: E402
from sensor_msgs.msg import Joy                          # noqa: E402
from std_msgs.msg import Bool, String                    # noqa: E402

from g1pilot.teleoperation import ps4_layout as L        # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def wait_for(cond, timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def rig():
    env = dict(os.environ)
    env["PYTHONPATH"] = REPO + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.Popen([sys.executable, "-m", "g1pilot.teleoperation.ps4_walk"],
                            cwd=REPO, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    state = {"run": False, "send": True}
    ex = node = th = None
    try:
        node = rclpy.create_node("ps4_walk_probe")
        got = {"cmd": [], "feedback": [], "status": []}
        node.create_subscription(Twist, "/g1pilot/ps4/walk_cmd",
                                 lambda m: got["cmd"].append((time.monotonic(), m)), 50)
        node.create_subscription(String, "/g1pilot/ps4/feedback",
                                 lambda m: got["feedback"].append(json.loads(m.data)), 50)
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        node.create_subscription(String, "/g1pilot/ps4/walk_status",
                                 lambda m: got["status"].append(json.loads(m.data)), latched)
        pub_joy = node.create_publisher(Joy, "/g1pilot/ps4/joy", 10)
        pub_estop = node.create_publisher(Bool, "/g1pilot/emergency_stop", 10)
        pub_start = node.create_publisher(Bool, "/g1pilot/start", 10)
        pub_onboard = node.create_publisher(String, "/g1pilot/onboard_state", latched)
        got["shared"] = []
        node.create_subscription(String, "/g1pilot/teleop_state",
                                 lambda m: got["shared"].append(json.loads(m.data)), latched)
        pub_shared = node.create_publisher(String, "/g1pilot/teleop_state", latched)
        got["deadman"] = []
        node.create_subscription(String, "/g1pilot/ps4/deadman",
                                 lambda m: got["deadman"].append((time.monotonic(),
                                                                  json.loads(m.data))), 50)
        pub_nav = node.create_publisher(String, "/g1pilot/nav/state", latched)
        state["nav"] = None
        state.update({"axes": [0.0] * L.NUM_AXES, "buttons": [0] * L.NUM_BUTTONS,
                      "run": True})

        def joy_loop():
            while state["run"]:
                if state["send"]:
                    m = Joy()
                    m.axes = [float(a) for a in state["axes"]]
                    m.buttons = [int(b) for b in state["buttons"]]
                    pub_joy.publish(m)
                if state["nav"] is not None:     # nav_supervisor: 10 Hz-artig frisch
                    pub_nav.publish(String(data=json.dumps(
                        {"state": state["nav"], "require_deadman": True})))
                time.sleep(1 / 50)

        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        threading.Thread(target=joy_loop, daemon=True).start()

        r = type("Rig", (), {})()
        r.got, r.state = got, state
        r.estop = lambda: pub_estop.publish(Bool(data=True))
        r.start = lambda: pub_start.publish(Bool(data=True))
        r.onboard = lambda bal: pub_onboard.publish(
            String(data=json.dumps({"fsm": 500 if bal else 4, "balanced": bal})))
        r.nav = lambda st: state.__setitem__("nav", st)
        r.l2 = lambda v: state["axes"].__setitem__(L.L2_AXIS, v)
        r.gui_walk_speed = lambda name: pub_shared.publish(String(data=json.dumps(
            {"source": "demo_gui", "fields": {"walk_speed": {"v": name, "t": time.time()}}})))
        r.gui_mode = lambda mode: pub_shared.publish(String(data=json.dumps(
            {"source": "demo_gui", "fields": {"mode": {"v": mode, "t": time.time()}}})))
        assert wait_for(lambda: got["status"], 15.0), "Node startet nicht"
        yield r
    finally:
        state["run"] = False
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
        if ex is not None:
            ex.shutdown()
        if th is not None:
            th.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        print(out[-3000:])


def cmds_since(rig, t0):
    return [m for t, m in rig.got["cmd"] if t >= t0]


def test_walk_node_end_to_end(rig):
    g = rig.got
    rig.onboard(True)
    # Stick vor ohne L2 -> kein Gehen
    rig.state["axes"][L.LY] = 1.0
    time.sleep(0.5)
    assert g["cmd"] == []
    # L2 halten -> Strom mit Langsam (0.35), 20 Hz
    t0 = time.monotonic()
    rig.l2(1.0)
    time.sleep(1.0)
    cs = cmds_since(rig, t0)
    assert len(cs) >= 12
    assert cs[-1].linear.x == pytest.approx(0.35) and cs[-1].angular.z == 0.0
    # Tempo: L2 + Steuerkreuz oben -> Normal
    rig.state["buttons"][L.DPAD_UP] = 1
    time.sleep(0.15)
    rig.state["buttons"][L.DPAD_UP] = 0
    assert wait_for(lambda: g["cmd"][-1][1].linear.x == pytest.approx(0.65))
    assert wait_for(lambda: g["status"][-1]["speed"] == "Normal")
    # Loslassen -> eine 0, dann Ruhe
    rig.l2(0.0)
    time.sleep(0.4)
    assert g["cmd"][-1][1].linear.x == 0.0
    t1 = time.monotonic()
    time.sleep(0.5)
    assert cmds_since(rig, t1) == []

    # Joy-Abriss mit gehaltenem L2 + Stick -> 0 innerhalb joy_timeout (0.3 s)
    rig.l2(1.0)
    assert wait_for(lambda: g["cmd"][-1][1].linear.x > 0.5)
    rig.state["send"] = False
    t2 = time.monotonic()
    assert wait_for(lambda: g["cmd"][-1][1].linear.x == 0.0, 1.5)
    assert g["cmd"][-1][0] - t2 < 0.6
    time.sleep(0.4)
    assert len(cmds_since(rig, t2 + 0.6)) == 0
    # Verbindung zurueck, L2 noch gedrueckt -> nicht losgehen
    rig.state["send"] = True
    t3 = time.monotonic()
    time.sleep(0.5)
    assert cmds_since(rig, t3) == []
    rig.l2(0.0)
    time.sleep(0.15)
    rig.l2(1.0)
    assert wait_for(lambda: len(cmds_since(rig, t3)) > 3)

    # NOT-HALT von aussen -> 0, gesperrt; START + Loslassen -> frei
    rig.estop()
    assert wait_for(lambda: g["status"][-1]["estop"] is True)
    time.sleep(0.2)
    assert g["cmd"][-1][1].linear.x == 0.0
    t4 = time.monotonic()
    time.sleep(0.4)
    assert cmds_since(rig, t4) == []
    rig.start()
    assert wait_for(lambda: g["status"][-1]["estop"] is False)
    rig.l2(0.0)
    time.sleep(0.15)
    rig.l2(1.0)
    assert wait_for(lambda: len(cmds_since(rig, t4)) > 3)

    # PS -> NOT-HALT, Stopp
    rig.state["buttons"][L.PS] = 1
    time.sleep(0.15)
    rig.state["buttons"][L.PS] = 0
    assert wait_for(lambda: g["status"][-1]["estop"] is True)
    assert g["cmd"][-1][1].linear.x == 0.0
    rig.l2(0.0)
    rig.start()
    assert wait_for(lambda: g["status"][-1]["estop"] is False)

    # Nicht im Walk-Modus: L2 -> lange, kraeftige Vibration als Hinweis
    rig.onboard(False)
    time.sleep(0.3)
    n = len(g["feedback"])
    rig.l2(1.0)
    assert wait_for(lambda: any(f.get("rumble", {}).get("ms") == 400
                                for f in g["feedback"][n:]))
    # Rueckmeldung setzt nie die LED-Farbe (gehoert ps4_arm_teleop)
    assert all("rgb" not in f for f in g["feedback"])


def _own_walk_speed(g):
    own = [d for d in g["shared"] if d.get("source") == "ps4_walk"]
    return own[-1]["fields"]["walk_speed"]["v"] if own else None


def test_walk_speed_shared_with_gui(rig):
    """Geh-Tempo im gemeinsamen Zustand: GUI-Stufe gilt fuer L2, Stufe am
    Controller geht an die GUI. Kein Gehen dadurch (nur Einstellung)."""
    g = rig.got
    rig.onboard(True)
    assert wait_for(lambda: _own_walk_speed(g) == "Langsam", 3.0)   # Start-Parameter
    rig.gui_walk_speed("Normal")
    assert wait_for(lambda: g["status"][-1]["speed"] == "Normal", 2.0)
    time.sleep(0.3)
    assert g["cmd"] == []                                 # nur Einstellung, kein Befehl
    t0 = time.monotonic()
    rig.l2(1.0)
    rig.state["axes"][L.LY] = 1.0
    assert wait_for(lambda: any(m.linear.x == pytest.approx(0.65)
                                for m in cmds_since(rig, t0)), 2.0)
    rig.state["buttons"][L.DPAD_UP] = 1                   # L2 + ▲ -> Schnell
    time.sleep(0.15)
    rig.state["buttons"][L.DPAD_UP] = 0
    assert wait_for(lambda: _own_walk_speed(g) == "Schnell", 2.0)
    rig.l2(0.0)


def _tap(rig, btn):
    rig.state["buttons"][btn] = 1
    time.sleep(0.15)
    rig.state["buttons"][btn] = 0
    time.sleep(0.15)


def test_r1_l1_walk_speed_in_walk_mode_reaches_gui(rig):
    """Nils 2026-10-21: Geh-Tempo auch per R1/L1 (wie das Arm-Tempo), im
    Modus GEHEN auch ohne gehaltenes L2 -- und die GUI sieht die Stufe."""
    g = rig.got
    assert wait_for(lambda: _own_walk_speed(g) == "Langsam", 3.0)
    _tap(rig, L.R1)                                       # GREIFEN: R1 gehoert den Armen
    time.sleep(0.2)
    assert _own_walk_speed(g) == "Langsam"
    rig.gui_mode("walk")                                  # Kachel/L2: Modus GEHEN
    time.sleep(0.3)
    n_fb = len(g["feedback"])
    _tap(rig, L.R1)
    assert wait_for(lambda: _own_walk_speed(g) == "Normal", 2.0)
    assert wait_for(lambda: g["status"][-1]["speed"] == "Normal", 2.0)
    assert any("rumble" in f for f in g["feedback"][n_fb:])   # Quittung je Stufe
    _tap(rig, L.L1)
    assert wait_for(lambda: _own_walk_speed(g) == "Langsam", 2.0)
    assert g["cmd"] == []                                 # nur Einstellung, kein Befehl
    rig.gui_mode("grasp")                                 # zurueck: R1 wieder Arm-Tempo
    time.sleep(0.3)
    _tap(rig, L.R1)
    time.sleep(0.2)
    assert _own_walk_speed(g) == "Langsam"


def test_l2_is_nav_deadman_while_auto_nav_requested(rig):
    """AUTO NAV angefragt: L2 geht nicht selbst, sondern meldet den Totmann
    (held/go) an den nav_supervisor; Funkabriss = keine Meldung; nach AUTO NAV
    aus erst nach L2 los wieder Hand-Gehen."""
    g = rig.got
    rig.onboard(True)
    rig.nav("queued")
    time.sleep(0.3)
    rig.l2(1.0)
    rig.state["axes"][L.LY] = 1.0           # L2 + Stick = Losgeh-Geste
    t0 = time.monotonic()
    assert wait_for(lambda: any(d["held"] and d["go"] for t, d in g["deadman"] if t >= t0))
    time.sleep(0.5)
    assert all(m.linear.x == 0.0 for m in cmds_since(rig, t0)), "L2 darf nicht selbst gehen"
    rig.state["axes"][L.LY] = 0.0
    assert wait_for(lambda: g["deadman"][-1][1] == {"held": True, "go": False})
    rig.l2(0.0)
    assert wait_for(lambda: g["deadman"][-1][1]["held"] is False)
    # Funkabriss: keine Totmann-Meldungen mehr
    rig.l2(1.0)
    rig.state["send"] = False
    time.sleep(0.5)
    t1 = time.monotonic()
    time.sleep(0.5)
    assert [d for t, d in g["deadman"] if t >= t1] == []
    rig.state["send"] = True
    # Waechter-Stopp: L2 + Stick geht trotzdem nicht von Hand los
    rig.l2(0.0)
    rig.nav("stopped")
    time.sleep(0.2)
    rig.l2(1.0)
    rig.state["axes"][L.LY] = 1.0
    t2 = time.monotonic()
    time.sleep(0.6)
    assert all(m.linear.x == 0.0 for m in cmds_since(rig, t2))
    # AUTO NAV aus (Supervisor verstummt): erst L2 los, dann wieder Hand-Gehen
    rig.nav(None)
    time.sleep(0.8)
    assert all(m.linear.x == 0.0 for m in cmds_since(rig, t2))
    rig.l2(0.0)
    time.sleep(0.15)
    t3 = time.monotonic()
    rig.l2(1.0)
    assert wait_for(lambda: any(m.linear.x > 0.3 for m in cmds_since(rig, t3)))

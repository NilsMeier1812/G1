#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav_supervisor als echter Node (Real-Einstellung: Totmann Pflicht).

Fehler-Injektion wie im Plan (Stufe 1): erst L2 + Stick gibt den Auto-Joy
frei, L2 los pausiert, Herzschlag weg (GUI/Funk) stoppt mit Grund, ohne
Totmann-Meldungen (Controller-Funk weg) kommt nichts durch.
"""
import json
import os
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
import rclpy.executors                                   # noqa: E402
from geometry_msgs.msg import PoseStamped                 # noqa: E402
from nav_msgs.msg import Odometry, Path                   # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile        # noqa: E402
from sensor_msgs.msg import Joy                           # noqa: E402
from std_msgs.msg import Bool, String                     # noqa: E402

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
    proc = subprocess.Popen([sys.executable, "-m", "g1pilot.navigation.nav_supervisor",
                             "--ros-args", "-p", "require_deadman:=true"],
                            cwd=REPO, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    st = {"run": True, "heartbeat": True, "deadman": (False, False), "dm_on": True}
    ex = node = th = None
    try:
        node = rclpy.create_node("nav_supervisor_probe")
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        got = {"joy": [], "active": [], "state": []}
        node.create_subscription(Joy, "/g1pilot/auto_joy",
                                 lambda m: got["joy"].append((time.monotonic(), m)), 100)
        node.create_subscription(Bool, "/g1pilot/nav/auto_active",
                                 lambda m: got["active"].append(m.data), latched)
        node.create_subscription(String, "/g1pilot/nav/state",
                                 lambda m: got["state"].append(json.loads(m.data)), latched)
        pub = {
            "req": node.create_publisher(Bool, "/g1pilot/auto_enable", 10),
            "hb": node.create_publisher(Bool, "/g1pilot/nav/heartbeat", 10),
            "dm": node.create_publisher(String, "/g1pilot/ps4/deadman", 10),
            "pose": node.create_publisher(Odometry, "/lidar_odometry/pose_fixed", 10),
            "raw": node.create_publisher(Joy, "/g1pilot/nav/raw_joy", 10),
            "loco": node.create_publisher(String, "/g1pilot/loco_state", latched),
            "path": node.create_publisher(Path, "/g1pilot/path", 10),
            "goal": node.create_publisher(PoseStamped, "/g1pilot/goal", 10),
        }

        def loop():
            while st["run"]:
                if st["heartbeat"]:
                    pub["hb"].publish(Bool(data=True))
                if st["dm_on"]:
                    held, go = st["deadman"]
                    pub["dm"].publish(String(data=json.dumps({"held": held, "go": go})))
                o = Odometry()
                o.header.frame_id = "map"
                o.pose.pose.orientation.w = 1.0
                pub["pose"].publish(o)
                j = Joy()
                j.axes = [0.0] * 8
                j.axes[4] = -0.5               # vor
                j.buttons = [0] * 14
                j.buttons[8] = 1               # nav2point-Deadman
                pub["raw"].publish(j)
                time.sleep(1 / 50)

        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        assert wait_for(lambda: got["state"], 15.0), "Node startet nicht"
        pub["loco"].publish(String(data="walk"))
        threading.Thread(target=loop, daemon=True).start()
        time.sleep(0.3)
        g = PoseStamped()
        g.header.frame_id = "map"
        g.pose.position.x = 2.0
        g.pose.orientation.w = 1.0
        pub["goal"].publish(g)
        p = Path()
        p.header.frame_id = "map"
        for x in (0.0, 1.0, 2.0):
            q = PoseStamped()
            q.pose.position.x = x
            p.poses.append(q)
        pub["path"].publish(p)
        time.sleep(0.2)

        r = type("Rig", (), {})()
        r.got, r.st = got, st
        r.request = lambda on: pub["req"].publish(Bool(data=on))
        r.last = lambda: got["state"][-1]
        r.moving_since = lambda t0: [m for t, m in got["joy"] if t >= t0 and m.buttons[8] == 1]
        yield r
    finally:
        st["run"] = False
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


def test_queue_then_l2_and_stick_starts(rig):
    rig.request(True)
    assert wait_for(lambda: rig.last()["state"] == "queued" and rig.last()["reason"] == "wait_go")
    t0 = time.monotonic()
    time.sleep(0.4)
    assert rig.moving_since(t0) == [], "vorgemerkt darf nicht laufen"
    rig.st["deadman"] = (True, False)          # nur L2
    time.sleep(0.4)
    assert rig.moving_since(t0) == []
    rig.st["deadman"] = (True, True)           # L2 + Stick
    assert wait_for(lambda: rig.last()["state"] == "running")
    assert wait_for(lambda: rig.got["active"] and rig.got["active"][-1] is True)
    rig.st["deadman"] = (True, False)          # L2 halten reicht
    t1 = time.monotonic()
    time.sleep(0.4)
    assert len(rig.moving_since(t1)) >= 10
    # L2 los -> Pause, Freigabe weg, kein Fahr-Joy mehr
    rig.st["deadman"] = (False, False)
    assert wait_for(lambda: rig.last()["state"] == "paused", 1.0)
    assert wait_for(lambda: rig.got["active"][-1] is False, 1.0)
    t2 = time.monotonic() + 0.1
    time.sleep(0.4)
    assert rig.moving_since(t2) == []


def test_heartbeat_loss_stops(rig):
    rig.st["deadman"] = (True, True)
    rig.request(True)
    assert wait_for(lambda: rig.last()["state"] == "running")
    rig.st["heartbeat"] = False                # GUI zu / WLAN weg
    t0 = time.monotonic()
    assert wait_for(lambda: rig.last()["state"] == "stopped", 2.0)
    assert time.monotonic() - t0 < 1.4
    assert rig.last()["reason"] == "heartbeat"
    assert rig.got["active"][-1] is False
    rig.st["heartbeat"] = True                 # kommt wieder -> bleibt gestoppt
    t1 = time.monotonic() + 0.1
    time.sleep(0.5)
    assert rig.last()["state"] == "stopped" and rig.moving_since(t1) == []


def test_controller_radio_loss_pauses(rig):
    rig.st["deadman"] = (True, True)
    rig.request(True)
    assert wait_for(lambda: rig.last()["state"] == "running")
    rig.st["dm_on"] = False                    # ps4_walk meldet nichts mehr
    t0 = time.monotonic()
    assert wait_for(lambda: rig.last()["state"] == "paused", 1.5)
    assert time.monotonic() - t0 < 0.7
    assert rig.got["active"][-1] is False

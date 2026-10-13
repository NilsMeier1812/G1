#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ps4_arm_teleop als echter Node mit den REAL-Einstellungen (wie bringup_real).

Ohne Roboter und ohne Controller: der Test spielt sensor_msgs/Joy ein, stellt
die Hand-Frames per statischer TF bereit und prueft die Topics, die der Node
an arm_controller / loco_client / Inspire-Bridge schickt:

  - Options uebernimmt nur die Arme (KEIN start_balancing -> Beine unberuehrt),
  - Tempo startet Langsam, R1 endet bei Normal, Ziel-Vorlauf begrenzt,
  - NOT-HALT von aussen (GUI) sperrt den Controller (LED rot), START gibt frei,
  - PS sendet den NOT-HALT, Kreis/Viereck die Hand-Befehle,
  - nichts an Laufen (loco_cmd_vel, /g1pilot/joy, start_walking).
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
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist   # noqa: E402
from sensor_msgs.msg import Joy                          # noqa: E402
from std_msgs.msg import Bool, String                    # noqa: E402
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster   # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray   # noqa: E402

from g1pilot.teleoperation import ps4_layout as L        # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HAND = {"left": (0.20, 0.20, 0.05), "right": (0.20, -0.20, 0.05)}
REAL_PARAMS = {"speed": "Langsam", "max_speed": "Normal",
               "options_starts_balancing": False, "ps_estop": True}


@pytest.fixture
def rig():
    env = dict(os.environ)
    env["PYTHONPATH"] = REPO + os.pathsep + env.get("PYTHONPATH", "")
    args = [sys.executable, "-m", "g1pilot.teleoperation.ps4_arm_teleop", "--ros-args"]
    for k, v in REAL_PARAMS.items():
        args += ["-p", f"{k}:={str(v).lower() if isinstance(v, bool) else v}"]
    proc = subprocess.Popen(args, cwd=REPO, env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    rclpy.init()
    state = {"run": False}
    ex = node = th = None
    try:
        node = rclpy.create_node("ps4_real_probe")
        got = {k: [] for k in ("goal_left", "goal_right", "enabled", "balance", "estop",
                               "hand_right", "hand_left", "feedback", "status", "walk",
                               "markers")}
        sub = lambda t, typ, key: node.create_subscription(   # noqa: E731
            typ, t, lambda m: got[key].append((time.monotonic(), m)), 50)
        sub("/g1pilot/hand_goal/left", PoseStamped, "goal_left")
        sub("/g1pilot/hand_goal/right", PoseStamped, "goal_right")
        sub("/g1pilot/arms/enabled", Bool, "enabled")
        sub("/g1pilot/start_balancing", Bool, "balance")
        sub("/g1pilot/emergency_stop", Bool, "estop")
        sub("/g1pilot/hand_action/right", String, "hand_right")
        sub("/g1pilot/hand_action/left", String, "hand_left")
        sub("/g1pilot/ps4/feedback", String, "feedback")
        sub("/g1pilot/ps4/status", String, "status")
        sub("/g1pilot/loco_cmd_vel", Twist, "walk")
        sub("/g1pilot/joy", Joy, "walk")
        sub("/g1pilot/start_walking", Bool, "walk")
        sub("/g1pilot/ps4/markers", MarkerArray, "markers")
        pub_joy = node.create_publisher(Joy, "/g1pilot/ps4/joy", 10)
        pub_estop = node.create_publisher(Bool, "/g1pilot/emergency_stop", 10)
        pub_start = node.create_publisher(Bool, "/g1pilot/start", 10)

        tfb = StaticTransformBroadcaster(node)
        tfs = []
        for side, (x, y, z) in HAND.items():
            t = TransformStamped()
            t.header.frame_id = "pelvis"
            t.child_frame_id = f"{side}_hand_point_contact"
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            tfs.append(t)
        tfb.sendTransform(tfs)

        state.update({"axes": [0.0] * L.NUM_AXES, "buttons": [0] * L.NUM_BUTTONS, "run": True})

        def joy_loop():
            while state["run"]:
                m = Joy()
                m.axes = [float(a) for a in state["axes"]]
                m.buttons = [int(b) for b in state["buttons"]]
                pub_joy.publish(m)
                time.sleep(1 / 50)

        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        jt = threading.Thread(target=joy_loop, daemon=True)
        jt.start()

        def press(btn, hold=0.15):
            state["buttons"][btn] = 1
            time.sleep(hold)
            state["buttons"][btn] = 0
            time.sleep(0.15)

        def status():
            return json.loads(got["status"][-1][1].data) if got["status"] else {}

        rig = type("Rig", (), {})()
        rig.got, rig.state, rig.press, rig.status = got, state, press, status
        rig.estop = lambda: pub_estop.publish(Bool(data=True))
        rig.start = lambda: pub_start.publish(Bool(data=True))
        # Node bereit (latched Status da) und Joy kommt an
        assert wait_for(lambda: status().get("connected") is True, 15.0), "Node startet nicht"
        yield rig
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
            th.join(timeout=2.0)     # Spin-Thread beenden, bevor rclpy herunterfaehrt
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        print(out[-3000:])


def wait_for(cond, timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def goals_since(rig, side, t0):
    return [m for t, m in rig.got[f"goal_{side}"] if t >= t0]


def test_real_settings_end_to_end(rig):
    g = rig.got
    st = rig.status()
    assert st["speed"] == "Langsam" and st["selection"] == "right" and not st["estop"]

    # Options: nur Arme uebernehmen, Beine unberuehrt
    rig.press(L.OPTIONS)
    assert wait_for(lambda: any(m.data for _, m in g["enabled"]))
    time.sleep(0.3)
    assert g["balance"] == [], "Options darf auf real kein start_balancing senden"

    # Linker Stick vor -> rechte Hand-Ziele laufen +x, Vorlauf begrenzt (4 cm)
    t0 = time.monotonic()
    rig.state["axes"][L.LY] = 1.0
    time.sleep(1.5)
    rig.state["axes"][L.LY] = 0.0
    gs = goals_since(rig, "right", t0)
    assert len(gs) > 20 and goals_since(rig, "left", t0) == []
    xs = [m.pose.position.x for m in gs]
    assert xs[-1] - HAND["right"][0] == pytest.approx(0.04, abs=0.005)   # Vorlauf-Deckel
    assert max(xs) - HAND["right"][0] <= 0.0405
    assert all(abs(m.pose.position.y - HAND["right"][1]) < 1e-6 for m in gs)
    # Loslassen -> Ziel zurueck auf die echte Hand (Halten)
    assert wait_for(lambda: abs(g["goal_right"][-1][1].pose.position.x - HAND["right"][0]) < 1e-6)

    # Tempo: R1 zweimal -> nur bis Normal
    rig.press(L.R1)
    rig.press(L.R1)
    assert wait_for(lambda: rig.status().get("speed") == "Normal", 2.5)
    time.sleep(1.2)
    assert rig.status()["speed"] == "Normal"

    # Hand-Befehle
    rig.press(L.SQUARE)
    rig.press(L.DPAD_UP)
    rig.press(L.CIRCLE)
    assert wait_for(lambda: [m.data for _, m in g["hand_right"]] == ["close", "open"])
    assert [m.data for _, m in g["hand_left"]] == ["open"]

    # NOT-HALT von aussen (GUI) -> gesperrt, LED rot, keine Ziele
    rig.estop()
    assert wait_for(lambda: rig.status().get("estop") is True, 2.5)
    assert wait_for(lambda: json.loads(g["feedback"][-1][1].data)["rgb"] == [255, 0, 0])
    time.sleep(0.2)
    t1 = time.monotonic()
    rig.state["axes"][L.LY] = 1.0
    rig.press(L.CIRCLE)
    time.sleep(0.8)
    rig.state["axes"][L.LY] = 0.0
    assert goals_since(rig, "right", t1) == [] and goals_since(rig, "left", t1) == []
    assert [m.data for _, m in g["hand_right"]] == ["close", "open"]
    # START quittiert -> wieder frei
    rig.start()
    assert wait_for(lambda: rig.status().get("estop") is False, 2.5)
    t2 = time.monotonic()
    rig.state["axes"][L.LY] = 1.0
    time.sleep(0.6)
    rig.state["axes"][L.LY] = 0.0
    assert len(goals_since(rig, "right", t2)) > 5 and len(goals_since(rig, "left", t2)) > 5

    # PS -> NOT-HALT wird gesendet
    n = len(g["estop"])
    rig.press(L.PS)
    assert wait_for(lambda: any(m.data for _, m in g["estop"][n:]))
    assert wait_for(lambda: rig.status().get("estop") is True)

    # Nie ein Laufbefehl
    assert g["walk"] == []

    # RViz: nur die Kugel an der Hand, kein Text ueber dem Roboter
    assert g["markers"]
    for _, arr in g["markers"]:
        assert not any(m.type == Marker.TEXT_VIEW_FACING and m.action == Marker.ADD
                       for m in arr.markers)
    assert any(m.ns == "ps4_label" and m.action == Marker.DELETE
               for m in g["markers"][-1][1].markers)

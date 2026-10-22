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
import math
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
def rig(request):
    """Indirekt parametrisierbar mit zusaetzlichen Node-Parametern (dict)."""
    env = dict(os.environ)
    env["PYTHONPATH"] = REPO + os.pathsep + env.get("PYTHONPATH", "")
    env.pop("G1_SIM_MODE", None)                     # Real-Einstellungen
    args = [sys.executable, "-m", "g1pilot.teleoperation.ps4_arm_teleop", "--ros-args"]
    params = dict(REAL_PARAMS, **(getattr(request, "param", None) or {}))
    for k, v in params.items():
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
                               "markers", "full", "shared", "arm_cmd", "seq_next")}
        sub = lambda t, typ, key: node.create_subscription(   # noqa: E731
            typ, t, lambda m: got[key].append((time.monotonic(), m)), 50)
        sub("/g1pilot/hand_goal/left", PoseStamped, "goal_left")
        sub("/g1pilot/hand_goal/right", PoseStamped, "goal_right")
        sub("/g1pilot/arms/enabled", Bool, "enabled")
        sub("/g1pilot/start_balancing", Bool, "balance")
        sub("/g1pilot/emergency_stop", Bool, "estop")
        sub("/g1pilot/full_emergency_stop", Bool, "full")
        sub("/g1pilot/hand_action/right", String, "hand_right")
        sub("/g1pilot/hand_action/left", String, "hand_left")
        sub("/g1pilot/ps4/feedback", String, "feedback")
        sub("/g1pilot/ps4/status", String, "status")
        sub("/g1pilot/loco_cmd_vel", Twist, "walk")
        sub("/g1pilot/joy", Joy, "walk")
        sub("/g1pilot/start_walking", Bool, "walk")
        sub("/g1pilot/ps4/markers", MarkerArray, "markers")
        sub("/g1pilot/arm_command", String, "arm_cmd")
        sub("/g1pilot/sequence/next", Bool, "seq_next")
        pub_cmd_status = node.create_publisher(String, "/g1pilot/arm_command/status", 10)
        pub_joy = node.create_publisher(Joy, "/g1pilot/ps4/joy", 10)
        pub_estop = node.create_publisher(Bool, "/g1pilot/emergency_stop", 10)
        pub_start = node.create_publisher(Bool, "/g1pilot/start", 10)
        pub_gui_walk = node.create_publisher(Bool, "/g1pilot/start_walking", 10)
        qos_latched = rclpy.qos.QoSProfile(depth=1)
        qos_latched.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        pub_onboard = node.create_publisher(String, "/g1pilot/onboard_state", qos_latched)
        # Gemeinsamer Zustand mit der Demo-GUI (shared_state.py)
        node.create_subscription(String, "/g1pilot/teleop_state",
                                 lambda m: got["shared"].append((time.monotonic(), m)),
                                 qos_latched)
        pub_shared = node.create_publisher(String, "/g1pilot/teleop_state", qos_latched)
        pub_seq = node.create_publisher(String, "/g1pilot/sequence/state", qos_latched)

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
        rig.cmd_status = lambda **st: pub_cmd_status.publish(String(data=json.dumps(st)))
        rig.start = lambda: pub_start.publish(Bool(data=True))
        rig.gui_walk = lambda: pub_gui_walk.publish(Bool(data=True))
        rig.onboard = lambda fsm: pub_onboard.publish(String(data=json.dumps({"fsm": fsm})))
        rig.seq_state = lambda st: pub_seq.publish(String(data=json.dumps(
            {"state": st, "name": "Kiste", "step": 1, "n": 3, "mode": "step"})))

        def gui_share(stamp=None, **fields):
            """Wie die Demo-GUI: Felder mit Zeitstempel (Default: jetzt)."""
            t = time.time() if stamp is None else stamp
            msg = {"source": "demo_gui",
                   "fields": {k: {"v": v, "t": t} for k, v in fields.items()}}
            pub_shared.publish(String(data=json.dumps(msg)))

        def shared():
            """Letzter Stand, den der Node publiziert hat: {feld: (wert, stempel)}."""
            for _, m in reversed(got["shared"]):
                d = json.loads(m.data)
                if d.get("source") == "ps4_arm_teleop":
                    return {k: (e["v"], e["t"]) for k, e in d["fields"].items()}
            return {}
        rig.gui_share, rig.shared = gui_share, shared
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


def test_full_estop_hold_rumble_abort_and_trigger(rig):
    """R1 + L1 + PS: Vibration ueber die Haltezeit, vorzeitiges Loslassen
    stoppt sie und loest nichts aus; 2 s gehalten -> full_emergency_stop
    (vor emergency_stop), lange Vibration, Status VOLL-NOTAUS; START gibt frei."""
    g, st = rig.got, rig.state
    rumbles = lambda: [json.loads(m.data).get("rumble") for _, m in g["feedback"]  # noqa: E731
                       if "rumble" in json.loads(m.data)]
    # 1) 1 s halten, dann loslassen -> Abbruch
    n_fb = len(rumbles())
    for b in (L.R1, L.L1, L.PS):
        st["buttons"][b] = 1
    assert wait_for(lambda: rig.status().get("full_arming") is True, 2.0)
    time.sleep(0.9)
    for b in (L.R1, L.L1, L.PS):
        st["buttons"][b] = 0
    assert wait_for(lambda: {"ms": 0} in rumbles()[n_fb:], 2.0)
    hold_rumble = [r for r in rumbles()[n_fb:] if r.get("ms", 0) > 0]
    assert hold_rumble and hold_rumble[0]["ms"] >= 2000 and hold_rumble[0]["strong"] == 1.0
    assert len(hold_rumble) == 1, "Quittungs-Vibration (PS-NOT-HALT) darf nicht dazwischen"
    time.sleep(0.5)
    assert g["full"] == []
    assert g["estop"], "PS allein: Oberkoerper-NOT-HALT wie bisher"
    # 2) START quittiert den PS-NOT-HALT, dann 2 s halten -> Voll-Notaus
    rig.start()
    assert wait_for(lambda: rig.status().get("estop") is False, 3.0)
    t_press = time.monotonic()
    for b in (L.R1, L.L1, L.PS):
        st["buttons"][b] = 1
    assert wait_for(lambda: len(g["full"]) == 1, 4.0)
    t_full = g["full"][0][0]
    assert 1.9 <= t_full - t_press <= 2.6
    assert g["full"][0][1].data is True
    late = [t for t, m in g["estop"] if t >= t_full - 0.01]
    assert late, "nach dem Voll-Notaus auch emergency_stop"
    assert wait_for(lambda: rig.status().get("full_estop") is True, 2.0)
    assert any(r.get("ms") == 1500 for r in rumbles())
    time.sleep(1.0)
    assert len(g["full"]) == 1, "weiter gehalten: nur einmal"
    for b in (L.R1, L.L1, L.PS):
        st["buttons"][b] = 0
    rig.start()
    assert wait_for(lambda: rig.status().get("full_estop") is False, 3.0)
    assert g["walk"] == []


# ── L2 = Modus GEHEN (Totmann), R2 = GREIFEN, Real ───────────────────────
MODES = {"trigger_modes": True, "walk_deadman": True}


def _walk_bools(g):
    return [m for _, m in g["walk"] if isinstance(m, Bool)]


@pytest.mark.parametrize("rig", [MODES], indirect=True)
def test_real_l2_walk_mode_refused_without_onboard_walk(rig):
    """Real, G1 nicht im Walk (Preparation, FSM 4): L2 schaltet NICHT um,
    kein start_walking (wuerde den G1 aufstellen), lange Vibration."""
    g, st = rig.got, rig.state
    rig.onboard(4)
    time.sleep(0.3)
    n_fb = len(g["feedback"])
    st["axes"][L.L2_AXIS] = 1.0
    time.sleep(0.4)
    st["axes"][L.L2_AXIS] = 0.0
    time.sleep(0.3)
    assert _walk_bools(g) == []
    assert not rig.status().get("arms_locked") and rig.status().get("robot_mode") is None
    rumbles = [json.loads(m.data).get("rumble") for _, m in g["feedback"][n_fb:]]
    assert any(r and r["ms"] >= 500 for r in rumbles)
    rig.press(L.R2)                                     # GREIFEN: nur Arme, Beine nicht
    assert wait_for(lambda: len(g["enabled"]) >= 1, 2.0)
    time.sleep(0.2)
    assert g["balance"] == []


@pytest.mark.parametrize("rig", [MODES], indirect=True)
def test_real_l2_walk_mode_locks_arms_r2_back_to_grasp(rig):
    g, st = rig.got, rig.state
    rig.onboard(501)                                    # G1 steht im Walk
    time.sleep(0.3)
    st["axes"][L.L2_AXIS] = 1.0                         # L2 druecken + halten
    assert wait_for(lambda: len(_walk_bools(g)) == 1, 2.0)
    time.sleep(0.3)
    st["axes"][L.L2_AXIS] = 0.0                         # loslassen: GEHEN bleibt
    assert wait_for(lambda: rig.status().get("arms_locked") is True, 2.0)
    assert len(_walk_bools(g)) == 1                     # nur einmal gesendet
    n_goal = len(g["goal_right"])
    st["axes"][L.LY] = 1.0                              # Stick: Arme gesperrt
    time.sleep(0.5)
    st["axes"][L.LY] = 0.0
    rig.press(L.CIRCLE)
    assert len(g["goal_right"]) == n_goal and g["hand_right"] == []
    rig.press(L.R2)                                     # GREIFEN
    assert wait_for(lambda: len(g["balance"]) == 1 and len(g["enabled"]) >= 1, 2.0)
    assert wait_for(lambda: rig.status().get("robot_mode") == "grasp", 2.0)
    assert not rig.status().get("arms_locked")
    rig.press(L.CIRCLE)
    assert wait_for(lambda: len(g["hand_right"]) == 1, 2.0)


@pytest.mark.parametrize("rig", [MODES], indirect=True)
def test_gui_walk_tile_locks_controller_arms(rig):
    rig.gui_walk()                                      # GEHEN aus der Demo-GUI
    assert wait_for(lambda: rig.status().get("arms_locked") is True, 2.0)
    assert "GEHEN" in rig.status().get("label", "")


# ── Gemeinsamer Zustand mit der Demo-GUI (/g1pilot/teleop_state) ─────────
@pytest.mark.parametrize("rig", [MODES], indirect=True)
def test_shared_state_follows_gui_and_reports_controller(rig):
    g = rig.got
    # Start: Node meldet seine Start-Parameter (real: rechts, Langsam)
    assert wait_for(lambda: rig.shared().get("arm_side", (None,))[0] == "right", 3.0)
    assert rig.shared()["arm_speed"][0] == "Langsam"
    assert "mode" not in rig.shared()

    # GUI waehlt linke Hand + Schnell: Controller folgt, Schnell -> max. Normal
    n_fb = len(g["feedback"])
    rig.gui_share(arm_side="left", arm_speed="Schnell")
    assert wait_for(lambda: rig.status().get("selection") == "left", 2.0)
    st = rig.status()
    assert st["speed"] == "Normal" and st["speed_req"] == "Schnell"
    time.sleep(0.2)
    fbs = [json.loads(m.data) for _, m in g["feedback"][n_fb:]]
    assert fbs and all("rumble" not in f for f in fbs)     # nur LED, keine Vibration
    assert fbs[-1]["rgb"] == [255, 200, 0]                   # gelb = links

    # Alter Stand (Stempel vor dem Node-Start) aendert nichts
    rig.gui_share(stamp=5.0, arm_side="both")
    time.sleep(0.3)
    assert rig.status()["selection"] == "left"

    # Steuerkreuz rechts am Controller -> GUI bekommt "right" mit neuerem Stempel
    t_gui = time.time()
    rig.press(L.DPAD_RIGHT)
    assert wait_for(lambda: rig.shared()["arm_side"][0] == "right", 2.0)
    assert rig.shared()["arm_side"][1] > t_gui
    rig.press(L.L1)                                          # Tempo Normal -> Langsam
    assert wait_for(lambda: rig.shared()["arm_speed"][0] == "Langsam", 2.0)

    # GUI-Kachel GEHEN ueber den gemeinsamen Zustand: Arme gesperrt, aber der
    # Controller sendet KEIN start_walking (das macht die GUI selbst)
    rig.gui_share(mode="walk")
    assert wait_for(lambda: rig.status().get("arms_locked") is True, 2.0)
    time.sleep(0.2)
    assert _walk_bools(g) == []
    # R2 am Controller -> GREIFEN an die GUI
    rig.press(L.R2)
    assert wait_for(lambda: rig.shared().get("mode", (None,))[0] == "grasp", 2.0)
    assert not rig.status().get("arms_locked")


@pytest.mark.parametrize("rig", [MODES], indirect=True)
def test_shared_mode_only_after_real_walk_switch(rig):
    """Real, G1 nicht im Walk: L2 wird abgelehnt -> GUI erfaehrt KEIN GEHEN.
    Im Walk: L2 -> mode walk im gemeinsamen Zustand (GUI-Kachel folgt)."""
    rig.onboard(4)
    time.sleep(0.3)
    st = rig.state
    st["axes"][L.L2_AXIS] = 1.0
    time.sleep(0.4)
    st["axes"][L.L2_AXIS] = 0.0
    time.sleep(0.3)
    assert "mode" not in rig.shared()
    rig.onboard(501)
    time.sleep(0.3)
    st["axes"][L.L2_AXIS] = 1.0
    assert wait_for(lambda: rig.shared().get("mode", (None,))[0] == "walk", 2.0)
    st["axes"][L.L2_AXIS] = 0.0


def test_triangle_rumbles_only_when_switching_to_move(rig):
    """Nils 2026-10-21: Dreieck -> VERSCHIEBEN kurz vibrieren, -> DREHEN nicht
    (Modus ohne GUI unterscheidbar)."""
    g = rig.got
    assert wait_for(lambda: rig.status().get("mode") == "move", 5.0)
    rumbles = lambda n: [json.loads(m.data).get("rumble") for _, m in g["feedback"][n:]]  # noqa: E731
    n = len(g["feedback"])
    rig.press(L.TRIANGLE)
    assert wait_for(lambda: rig.status().get("mode") == "turn", 2.0)
    time.sleep(0.2)
    assert len(g["feedback"]) > n and all(r is None for r in rumbles(n))   # nur LED
    n = len(g["feedback"])
    rig.press(L.TRIANGLE)
    assert wait_for(lambda: rig.status().get("mode") == "move", 2.0)
    assert wait_for(lambda: any(r and r["ms"] > 0 for r in rumbles(n)), 1.0)


@pytest.mark.parametrize("rig", [MODES], indirect=True)
def test_r2_hold_streams_waist_rate_and_r3_centers(rig):
    """R2 halten + rechter Stick: Rate-Strom auf /g1pilot/waist/yaw_rate, Hand-
    Ziele bleiben aus; R2 + R3: Mitte; R2 los: einmal 0. Danach joggt die Hand
    im Oberkoerper-Bezug (gemeldeter Taillen-Yaw 90 deg -> "vor" = +y)."""
    from std_msgs.msg import Float32
    from g1pilot.manipulation.waist_yaw import (
        WAIST_RATE_TOPIC, WAIST_CENTER_TOPIC, WAIST_STATE_TOPIC)
    g, st = rig.got, rig.state
    node = rclpy.create_node("ps4_waist_probe")
    rates, centers = [], []
    node.create_subscription(Float32, WAIST_RATE_TOPIC, lambda m: rates.append(m.data), 50)
    node.create_subscription(Bool, WAIST_CENTER_TOPIC, lambda m: centers.append(m.data), 10)
    pub_state = node.create_publisher(String, WAIST_STATE_TOPIC, 10)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        rig.onboard(501)
        time.sleep(0.3)
        rig.press(L.R2)                                     # GREIFEN
        assert wait_for(lambda: rig.status().get("robot_mode") == "grasp", 2.0)
        st["axes"][L.R2_AXIS] = 1.0                         # R2 halten
        assert wait_for(lambda: rig.status().get("waist_mode") is True, 2.0)
        n_goal = len(g["goal_right"])
        st["axes"][L.RX] = 1.0
        st["axes"][L.LY] = 1.0                              # Arm-Stick: gesperrt
        assert wait_for(lambda: len([r for r in rates if r > 0.9]) >= 5, 2.0)
        assert len(g["goal_right"]) == n_goal
        st["axes"][L.RX] = 0.0
        st["axes"][L.LY] = 0.0
        rig.press(L.R3)
        assert wait_for(lambda: centers == [True], 2.0)
        n_rates = len(rates)
        st["axes"][L.R2_AXIS] = 0.0                         # loslassen
        assert wait_for(lambda: rig.status().get("waist_mode") is False, 2.0)
        time.sleep(0.3)
        assert rates[-1] == 0.0 and len(rates) - n_rates <= 12   # Strom endet
        # Oberkoerper-Bezug: Zustand wie der arm_controller mit 10 Hz
        alive = {"on": True}

        def state_loop():
            while alive["on"]:
                pub_state.publish(String(data=json.dumps({"owner": True, "yaw_rad": math.pi / 2})))
                time.sleep(0.1)
        threading.Thread(target=state_loop, daemon=True).start()
        time.sleep(0.5)
        t0 = time.monotonic()
        st["axes"][L.LY] = 1.0                              # "vor"
        assert wait_for(lambda: len(goals_since(rig, "right", t0)) >= 15, 3.0)
        st["axes"][L.LY] = 0.0
        alive["on"] = False
        p = goals_since(rig, "right", t0)[-1].pose.position
        dx, dy = p.x - HAND["right"][0], p.y - HAND["right"][1]
        assert dy > 0.005 and abs(dx) < dy / 3, (dx, dy)   # "vor" = +y im Becken
    finally:
        ex.shutdown()
        th.join(timeout=2.0)
        node.destroy_node()


def test_mirror_hold_rumble_abort_and_command(rig):
    """Steuerkreuz links + Kreuz: Vibration ueber die Haltezeit, vorzeitiges
    Loslassen stoppt sie und sendet nichts; 2 s gehalten -> genau ein
    arm_command {"type": "mirror", "side": "left"}; Ablehnung des
    arm_controllers (Arme nicht aktiv) -> lange Vibration. Nichts ans Laufen."""
    g, st = rig.got, rig.state
    rumbles = lambda: [json.loads(m.data).get("rumble") for _, m in g["feedback"]  # noqa: E731
                       if "rumble" in json.loads(m.data)]
    # 1) 1 s halten, dann loslassen -> Abbruch
    n_fb = len(rumbles())
    st["buttons"][L.DPAD_LEFT] = st["buttons"][L.CROSS] = 1
    assert wait_for(lambda: rig.status().get("mirror_arming") == "left", 2.0)
    time.sleep(0.9)
    st["buttons"][L.DPAD_LEFT] = st["buttons"][L.CROSS] = 0
    assert wait_for(lambda: {"ms": 0} in rumbles()[n_fb:], 2.0)
    hold = [r for r in rumbles()[n_fb:] if r.get("ms", 0) >= 2000]
    assert len(hold) == 1 and hold[0]["strong"] == 1.0
    time.sleep(0.4)
    assert g["arm_cmd"] == [] and rig.status().get("selection") == "left"
    # 2) 2 s halten -> Kommando
    t_press = time.monotonic()
    st["buttons"][L.DPAD_LEFT] = st["buttons"][L.CROSS] = 1
    assert wait_for(lambda: len(g["arm_cmd"]) == 1, 4.0)
    t_cmd, m = g["arm_cmd"][0]
    assert 1.9 <= t_cmd - t_press <= 2.6
    cmd = json.loads(m.data)
    assert cmd["type"] == "mirror" and cmd["side"] == "left" and cmd["id"]
    time.sleep(0.8)
    assert len(g["arm_cmd"]) == 1, "weiter gehalten: nur einmal"
    st["buttons"][L.DPAD_LEFT] = st["buttons"][L.CROSS] = 0
    # 3) arm_controller lehnt ab -> lange starke Vibration
    n_fb = len(rumbles())
    rig.cmd_status(id="fremd", state="rejected", reason="x")       # fremde ID: nichts
    rig.cmd_status(id=cmd["id"], state="rejected",
                   reason="Arme nicht aktiviert (ENABLE MANIPULATION).")
    assert wait_for(lambda: any(r.get("ms") == 600 for r in rumbles()[n_fb:]), 2.0)
    assert len([r for r in rumbles()[n_fb:] if r.get("ms") == 600]) == 1
    assert g["walk"] == []


@pytest.mark.parametrize("rig", [{"mirror_hold_s": 0.0}], indirect=True)
def test_mirror_disabled_by_param(rig):
    st = rig.state
    st["buttons"][L.DPAD_LEFT] = st["buttons"][L.CROSS] = 1
    time.sleep(2.6)
    st["buttons"][L.DPAD_LEFT] = st["buttons"][L.CROSS] = 0
    assert rig.got["arm_cmd"] == [] and rig.status().get("selection") == "left"


def test_l3_next_only_while_sequence_waits_and_rumbles(rig):
    """L3 = Ablauf weiter (Demo-GUI, Schritt fuer Schritt): ohne wartenden
    Ablauf nichts; beim Uebergang zu »wartet« vibriert der Controller."""
    g = rig.got
    rumbles = lambda n: [json.loads(m.data).get("rumble") for _, m in g["feedback"][n:]]  # noqa: E731
    rig.press(L.L3)
    time.sleep(0.3)
    assert g["seq_next"] == []
    n = len(g["feedback"])
    rig.seq_state("waiting")
    assert wait_for(lambda: any(r and r["ms"] > 0 for r in rumbles(n)), 2.0)
    t0 = time.monotonic()
    rig.state["axes"][L.LY] = 0.6                        # Druecken kippt den Stick
    rig.press(L.L3)
    assert wait_for(lambda: len(g["seq_next"]) == 1 and g["seq_next"][0][1].data, 2.0)
    time.sleep(0.3)
    assert goals_since(rig, "right", t0) == []           # kein Hand-Ziel waehrend L3
    rig.state["axes"][L.LY] = 0.0
    rig.seq_state("moving")
    time.sleep(0.3)
    rig.press(L.L3)
    time.sleep(0.3)
    assert len(g["seq_next"]) == 1


def test_l3_while_jogging_sends_one_hold_then_nothing(rig):
    """Nils 2026-10-22: Stick beim L3 versehentlich bewegt -> Ablauf brach mit
    »manueller Vorrang« ab. Jetzt: EIN Halte-Ziel vor dem »Weiter«, danach kein
    Halte-Strom; solange der Ablauf faehrt, sind die Sticks gesperrt."""
    g = rig.got
    rig.seq_state("waiting")
    time.sleep(0.3)
    rig.state["axes"][L.LY] = 0.6                        # nachkorrigieren beim Warten
    assert wait_for(lambda: len(g["goal_right"]) >= 3, 2.0)
    rig.state["buttons"][L.L3] = 1                       # L3 mit ausgelenktem Stick
    assert wait_for(lambda: len(g["seq_next"]) == 1, 2.0)
    t_next = g["seq_next"][0][0]
    rig.seq_state("sent")                                # Demo-GUI schickt den Schritt
    time.sleep(0.15)
    rig.state["buttons"][L.L3] = 0
    rig.seq_state("moving")
    time.sleep(1.2)                                      # Stick bleibt ausgelenkt
    late = goals_since(rig, "right", t_next + 0.05)
    assert late == [], "nach Weiter keine Hand-Ziele (braechen die Fahrt ab)"
    assert rig.status().get("seq_busy") and "ABLAUF" in rig.status()["label"]
    rig.state["axes"][L.LY] = 0.0
    rig.seq_state("waiting")                             # Schritt erreicht -> Jog wieder frei
    time.sleep(0.3)
    t0 = time.monotonic()
    rig.state["axes"][L.LY] = 0.6
    assert wait_for(lambda: len(goals_since(rig, "right", t0)) >= 2, 2.0)
    rig.state["axes"][L.LY] = 0.0


def test_stick_during_sequence_motion_sends_no_goal(rig):
    rig.seq_state("moving")
    time.sleep(0.3)
    t0 = time.monotonic()
    rig.state["axes"][L.LY] = 0.8
    rig.state["axes"][L.RX] = 0.8
    time.sleep(0.8)
    rig.state["axes"][L.LY] = rig.state["axes"][L.RX] = 0.0
    time.sleep(0.3)
    assert goals_since(rig, "right", t0) == [] and goals_since(rig, "left", t0) == []
    rig.seq_state("idle")                                # Ablauf fertig/abgebrochen
    time.sleep(0.3)
    t1 = time.monotonic()
    rig.state["axes"][L.LY] = 0.6
    assert wait_for(lambda: goals_since(rig, "right", t1), 2.0)
    rig.state["axes"][L.LY] = 0.0

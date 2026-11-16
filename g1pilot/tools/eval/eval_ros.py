#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
eval_ros.py -- gemeinsame ROS-/DDS-Hilfen der Messwerkzeuge (V4, V7, V8).

Alles hier ist LESEND, ausser den ausdruecklich als "nur Sim" markierten
Hilfen zum Erzeugen von Last/Ablaeufen (sim_*). Die Werkzeuge rufen diese
nur auf, wenn G1_SIM_MODE=true ist (assert_sim()).

DDS: dieselbe Auswahl wie die Nodes (g1pilot.utils.common.resolve_dds):
Sim Domain 1 auf lo, Real Domain 0 auf dem Roboter-Interface. Die
Unitree-Topics werden nur ABONNIERT (Reader), nie beschrieben.
"""
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_common as ec  # noqa: E402

if ec.PKG_ROOT not in sys.path:
    sys.path.append(ec.PKG_ROOT)          # Quellbaum, falls g1pilot nicht installiert ist

# Gelenkindizes der Arme im 29-DOF-Layout (G1_29_JointArmIndex)
ARM_IDX = list(range(15, 29))
ARM_NAMES = ["l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
             "l_wrist_roll", "l_wrist_pitch", "l_wrist_yaw",
             "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow",
             "r_wrist_roll", "r_wrist_pitch", "r_wrist_yaw"]


def is_sim():
    return ec.env_mode() == ec.MODE_SIM


def assert_sim(what):
    if not is_sim():
        raise SystemExit(f"{what} sendet Befehle und laeuft deshalb NUR in der Simulation "
                         f"(G1_SIM_MODE=true). Am realen G1 sind die Werkzeuge nur lesend.")


class DdsTap:
    """Liest rt/lowstate, rt/lowcmd und rt/arm_sdk mit Ankunftszeit
    (time.monotonic). Callback-Funktionen bekommen (t, msg)."""

    def __init__(self, interface="lo"):
        from g1pilot.utils.common import resolve_dds
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        self.domain, self.iface = resolve_dds(interface)
        ChannelFactoryInitialize(self.domain, self.iface)
        self._Sub = ChannelSubscriber
        self._types = {"rt/lowstate": LowState_, "rt/lowcmd": LowCmd_, "rt/arm_sdk": LowCmd_}
        self._subs = []

    def subscribe(self, topic, fn, queue=50):
        sub = self._Sub(topic, self._types[topic])

        def handler(msg):
            fn(time.monotonic(), msg)
        sub.Init(handler, queue)
        self._subs.append(sub)
        return sub

    def close(self):
        for s in self._subs:
            try:
                s.Close()
            except Exception:
                pass


def arm_q_from_state(msg):
    return [float(msg.motor_state[j].q) for j in ARM_IDX]


def arm_q_from_cmd(msg):
    return [float(msg.motor_cmd[j].q) for j in ARM_IDX]


def arm_kp_from_cmd(msg):
    return [float(msg.motor_cmd[j].kp) for j in ARM_IDX]


def arm_weight(msg):
    return float(msg.motor_cmd[29].q)


def make_node(name):
    import rclpy
    if not rclpy.ok():
        rclpy.init(args=None)
    return rclpy.create_node(name)


def spin_until(node, deadline, step=0.01, stop=None):
    import rclpy
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=step)
        if stop is not None and stop():
            return True
    return False


class ArmStatus:
    """Abonniert /g1pilot/arm_command/status und sammelt Zustaende je id."""

    def __init__(self, node):
        from std_msgs.msg import String
        self.events = []          # (t, dict)
        self._lock = threading.Lock()
        node.create_subscription(String, "/g1pilot/arm_command/status", self._cb, 50)

    def _cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self.events.append((time.monotonic(), d))

    def terminal(self, req_id=None, since=0.0):
        with self._lock:
            for t, d in self.events:
                if t < since:
                    continue
                if req_id is not None and d.get("id") != req_id:
                    continue
                if d.get("state") in ("reached", "failed", "rejected", "cancelled"):
                    return t, d
        return None

    def all_since(self, since):
        with self._lock:
            return [(t, d) for t, d in self.events if t >= since]


# ── nur Sim: Bedienhandlungen wie die Demo-GUI ──────────────────────────
class SimOperator:
    """Sendet dieselben Topics wie die Demo-GUI-Kacheln (gui_base.py). Nur Sim."""

    def __init__(self, node):
        assert_sim("SimOperator")
        from std_msgs.msg import Bool, String
        from geometry_msgs.msg import Twist
        self._Bool, self._String, self._Twist = Bool, String, Twist
        self.node = node
        self.pub = {n: node.create_publisher(Bool, t, 10) for n, t in (
            ("arms_enabled", "/g1pilot/arms/enabled"), ("start_balancing", "/g1pilot/start_balancing"),
            ("start_walking", "/g1pilot/start_walking"), ("start", "/g1pilot/start"),
            ("estop", "/g1pilot/emergency_stop"), ("full_estop", "/g1pilot/full_emergency_stop"))}
        self.pub_cmd = node.create_publisher(String, "/g1pilot/arm_command", 10)
        self.pub_goto = node.create_publisher(String, "/g1pilot/pose_store/goto", 10)
        self.pub_vel = node.create_publisher(Twist, "/g1pilot/loco_cmd_vel", 10)

    def bool(self, name, value=True):
        self.pub[name].publish(self._Bool(data=bool(value)))

    def greifen(self):
        """Kachel GREIFEN: Arme uebernehmen + Stand halten."""
        self.bool("arms_enabled", True)
        self.bool("start_balancing", True)

    def gehen(self):
        self.bool("start_walking", True)

    def vel(self, vx=0.0, vy=0.0, wz=0.0):
        m = self._Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
        self.pub_vel.publish(m)

    def joints(self, req_id, left=None, right=None):
        d = {"id": req_id, "type": "joints"}
        if left is not None:
            d["left"] = [float(x) for x in left]
        if right is not None:
            d["right"] = [float(x) for x in right]
        self.pub_cmd.publish(self._String(data=json.dumps(d)))

    def goto(self, pose_name):
        self.pub_goto.publish(self._String(data=pose_name))

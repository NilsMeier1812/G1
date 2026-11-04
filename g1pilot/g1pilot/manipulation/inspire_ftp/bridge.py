#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bridge.py — Inspire-FTP-Hand-Bridge (Sim UND echte Haende)
==========================================================
Hervorgegangen aus dem frueheren ftp_hand_controller (zwei eigenstaendige
Python-Skripte mit Browser-GUIs, die per Modbus TCP mit den echten Haenden
sprachen). Die Browser-GUIs samt WebSocket-/HTTP-Server sind entfernt
(2026-11-04), bedient wird ueber die Demo-GUI (hand_panel.py):

  * ROS2 /joint_states                                            (RViz-Finger)
  * ROS2 /g1pilot/hand_cmd + /g1pilot/hand_status  <-> Qt-Demo-GUI (Seite HAENDE)

Die eigentliche "Hardware"-Anbindung steckt im Backend (siehe backends.py):
Stufe 1 = SimJointStateBackend (RViz), Stufe 2 = MujocoContactBackend
(Sim-Physik, Default), Stufe 3 = InspireModbusBackend (ECHTE Haende via
Modbus TCP, backend:=modbus + left_host/right_host).

WICHTIG: Wenn dieser Node laeuft, muss robot_state mit publish_hand_joints:=false
gestartet werden, sonst ueberschreiben sich die beiden /joint_states-Quellen fuer
die Finger gegenseitig (so auch im robot_state.py-Kommentar vermerkt).
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, Float32MultiArray

from . import joint_map
from .backends import SimJointStateBackend
from .model import HandModel


class InspireFtpBridge(Node):
    def __init__(self):
        super().__init__("inspire_ftp_bridge")

        # ── Parameter ────────────────────────────────────────────────────────
        self.declare_parameter("update_rate_hz", 50.0)
        # backend: "mujoco" = Stufe 2 (echte Finger-Physik + Touch-Kraefte via DDS),
        #          "sim"    = Stufe 1 (open-loop /joint_states, Kraft/Taktil = 0),
        #          "modbus" = Stufe 3 (ECHTE Haende via Modbus TCP, Roboter-LAN).
        self.declare_parameter("backend", "mujoco")
        self.declare_parameter("interface", "lo")        # DDS-Interface fuer 'mujoco'
        # Sim: nur kosmetische GUI-Anzeige. Modbus-Backend: ECHTE Ziel-IPs der
        # Haende im Roboter-LAN (Standard laut reference/: .210 links, .211
        # rechts, Port 6000). Leer lassen = diese Seite nicht ansteuern.
        self.declare_parameter("left_host", "sim://left")
        self.declare_parameter("right_host", "sim://right")
        self.declare_parameter("modbus_port", 6000)
        # Poll-Rate des Modbus-I/O-Threads je Hand + Taktil-Leserate
        # (tactile_every: jede n-te Runde alle 17 Zonen lesen; 0 = aus).
        self.declare_parameter("hand_poll_rate_hz", 20.0)
        self.declare_parameter("tactile_every", 2)

        rate = float(self.get_parameter("update_rate_hz").value)
        backend_name = self.get_parameter("backend").get_parameter_value().string_value
        interface = self.get_parameter("interface").get_parameter_value().string_value
        left_host = self.get_parameter("left_host").get_parameter_value().string_value
        right_host = self.get_parameter("right_host").get_parameter_value().string_value

        # ── /joint_states-Publisher (nur die 24 Finger-Gelenke) ──────────────
        self.joint_pub = self.create_publisher(JointState, "/joint_states", QoSProfile(depth=10))

        # ── Hand-Aktionen (OPEN/CLOSE LEFT/RIGHT HAND) ───────────────────────
        #  Demo-GUI (Hand-Panel) und loco_client publizieren "open"/"close" als
        #  String auf diese Topics. Diese Bridge bildet sie auf die 6 Inspire-DOF ab.
        self.create_subscription(
            String, "/g1pilot/hand_action/left",
            lambda m: self._on_hand_action("left", m), 10)
        self.create_subscription(
            String, "/g1pilot/hand_action/right",
            lambda m: self._on_hand_action("right", m), 10)
        # E-Stop: beide Haende disablen -> es werden keine neuen Sollwerte mehr
        # geschrieben, die Haende halten ihre Position (kein aktives Weiter-
        # schliessen). Re-Aktivierung bewusst manuell (GUI-Hauptschalter bzw.
        # naechster OPEN/CLOSE-Befehl setzt enabled wieder).
        self.create_subscription(
            Bool, "/g1pilot/emergency_stop", self._on_emergency_stop, 10)

        # ── Positionsspeicher-Anbindung (siehe manipulation/arm_controller.py) ──
        #  hand_state: aktuelle 6 Fingerwinkel je Hand (angle_act, 0..1000) RAUS,
        #  damit arm_controller sie beim Speichern einer Pose mitnehmen kann.
        #  finger_goal: gespeicherte 6 Winkel je Hand REIN, zum Wiederherstellen.
        #  (Nicht hand_goal: das ist das PoseStamped-Armziel des Markers.)
        self.hand_state_pub = {
            "left":  self.create_publisher(Float32MultiArray, "/g1pilot/hand_state/left", 10),
            "right": self.create_publisher(Float32MultiArray, "/g1pilot/hand_state/right", 10),
        }
        self.create_subscription(
            Float32MultiArray, "/g1pilot/finger_goal/left",
            lambda m: self._on_finger_goal("left", m), 10)
        self.create_subscription(
            Float32MultiArray, "/g1pilot/finger_goal/right",
            lambda m: self._on_finger_goal("right", m), 10)

        # ── Steuerung durch die Qt-Demo-GUI ──────────────────────────────────
        #  hand_cmd: JSON-Befehle (set_angle, set_force,
        #            set_speed, set_enabled, set_all_angles, open_hand, close_hand).
        #  hand_status: ~10 Hz JSON {left|right: controller_state + "zones"}, wobei
        #            "zones" je Taktil-Zone nur den Spitzenwert (0..4095) enthaelt
        #            -- genug fuer die Kraftzonen-Anzeige, ohne volle Taxel-Matrix.
        self.create_subscription(String, "/g1pilot/hand_cmd", self._on_hand_cmd, 10)
        self.hand_status_pub = self.create_publisher(String, "/g1pilot/hand_status", 10)
        self.create_timer(0.1, self._publish_hand_status)

        # ── Mapping-Limits an die echte URDF klemmen, falls auffindbar ───────
        urdf = joint_map.default_urdf_path()
        limits = joint_map.load_limits_from_urdf(urdf) if urdf else {}
        closed_rad = joint_map._closed_rad(limits)
        if urdf:
            self.get_logger().info(f"URDF-Limits geladen: {urdf}")
        else:
            self.get_logger().warn("URDF nicht gefunden -> CLOSED_RAD-Defaults (ungeklemmt).")

        # ── Modelle + Backend ────────────────────────────────────────────────
        self.models = {
            "left":  HandModel("links",  left_host),
            "right": HandModel("rechts", right_host),
        }
        if backend_name == "modbus":
            from .backends import InspireModbusBackend
            self.backend = InspireModbusBackend(
                self._publish_joint_states,
                hosts={"left": left_host, "right": right_host},
                models=self.models,
                port=int(self.get_parameter("modbus_port").value),
                closed_rad=closed_rad,
                poll_rate_hz=float(self.get_parameter("hand_poll_rate_hz").value),
                tactile_every=int(self.get_parameter("tactile_every").value),
            )
            active = ", ".join(f"{s}={w.client.host}:{w.client.port}"
                               for s, w in self.backend.workers.items()) or "KEINE"
            self.get_logger().info(
                f"Backend=modbus (Stufe 3, ECHTE Haende): {active}")
            if not self.backend.workers:
                self.get_logger().error(
                    "modbus-Backend ohne gueltige Hand-IPs! left_host/right_host "
                    "setzen (z.B. 192.168.123.210/.211).")
        elif backend_name == "mujoco":
            from .backends import MujocoContactBackend
            self.backend = MujocoContactBackend(
                self._publish_joint_states, interface=interface, closed_rad=closed_rad)
            self.get_logger().info(
                f"Backend=mujoco (Stufe 2): DDS rt/inspire/cmd|state ueber '{interface}' "
                f"-> echte Finger-Winkel + Touch-Kraefte.")
        else:
            if backend_name != "sim":
                self.get_logger().warn(
                    f"backend='{backend_name}' unbekannt -> nutze 'sim' (Stufe 1).")
            self.backend = SimJointStateBackend(self._publish_joint_states, closed_rad=closed_rad)

        # ── Update-Schleife (rclpy-Timer): Backend treiben + /joint_states ───
        self._last = time.monotonic()
        self.create_timer(1.0 / max(rate, 1.0), self._step)

        self.get_logger().info(
            f"Inspire-FTP-Bridge aktiv | Backend={backend_name}")

    # ── ROS-seitig ────────────────────────────────────────────────────────────
    def _step(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        self.backend.update(self.models, dt)
        # Aktuellen Fingerzustand (Ist-Winkel) fuer den Positionsspeicher raus.
        for side in ("left", "right"):
            msg = Float32MultiArray()
            with self.models[side].lock:
                msg.data = [float(a) for a in self.models[side].angle_act]
            self.hand_state_pub[side].publish(msg)

    def _on_hand_cmd(self, msg: String):
        try:
            self._handle_cmd(json.loads(msg.data))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"hand_cmd-Fehler: {e}")

    def _publish_hand_status(self):
        out = {}
        for side in ("left", "right"):
            model = self.models[side]
            st = model.controller_state()
            with model.lock:
                st["zones"] = {k: (max(v) if v else 0) for k, v in model.zones.items()}
            out[side] = st
        self.hand_status_pub.publish(String(data=json.dumps(out)))

    def _on_finger_goal(self, side: str, msg: Float32MultiArray):
        """Gespeicherte Handstellung wiederherstellen: 6 Sollwinkel (0..1000)
        setzen (siehe arm_controller._publish_hand_goals). Hand wird dazu
        aktiviert."""
        vals = list(msg.data)
        if len(vals) != 6:
            self.get_logger().warn(
                f"finger_goal/{side} ignoriert: erwarte 6 Werte, bekam {len(vals)}.")
            return
        hand = self.models["right"] if side == "right" else self.models["left"]
        hand.set_enabled(True)
        for i, v in enumerate(vals):
            hand.set_angle(i, int(v))
        self.get_logger().info(f"Handposition ({side}) aus Positionsspeicher gesetzt.")

    def _publish_joint_states(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(p) for p in positions]
        self.joint_pub.publish(msg)

    def _handle_cmd(self, cmd: dict):
        side = cmd.get("side", "left")
        hand = self.models["right"] if side == "right" else self.models["left"]
        t = cmd.get("type")
        if t == "set_angle":
            hand.set_angle(cmd["dof"], cmd["value"])
        elif t == "set_force":
            hand.set_force(cmd["dof"], cmd["value"])
        elif t == "set_speed":
            # Beide Haende gleich schnell (wie im Original).
            self.models["left"].set_speed_all(cmd["value"])
            self.models["right"].set_speed_all(cmd["value"])
        elif t == "set_enabled":
            hand.set_enabled(cmd["value"])
        elif t == "set_all_angles":
            for i, v in enumerate(cmd["values"]):
                hand.set_angle(i, v)
        elif t == "open_hand":
            self._apply_open_close(hand, "open")
        elif t == "close_hand":
            self._apply_open_close(hand, "close")

    @staticmethod
    def _apply_open_close(hand: HandModel, action: str):
        """Alle 6 DOF auf offen (1000) bzw. zu setzen. Gemeinsam genutzt von
        hand_cmd (open_hand/close_hand) und den Hand-Aktionen (hand_action)."""
        hand.set_enabled(True)
        if action == "open":
            for i in range(6):
                hand.set_angle(i, 1000)
        else:  # close
            for i in range(6):
                hand.set_angle(i, 200 if i == 4 else 0)  # Daumen-Beugung nicht ganz zu

    def _on_emergency_stop(self, msg):
        """EMERGENCY STOP (GUI): beide Haende sofort disablen."""
        if not msg.data:
            return
        for side in ("left", "right"):
            self.models[side].set_enabled(False)
        self.get_logger().warn("EMERGENCY STOP: beide Haende disabled "
                               "(halten Position, keine neuen Sollwerte).")

    def _on_hand_action(self, side: str, msg: String):
        """Hand-Aktion OPEN/CLOSE {LEFT,RIGHT} HAND -> Inspire-DOF-Sollwerte."""
        action = (msg.data or "").strip().lower()
        if action not in ("open", "close"):
            return
        hand = self.models["right"] if side == "right" else self.models["left"]
        self._apply_open_close(hand, action)
        self.get_logger().info(f"Hand-Aktion: Hand {side} -> {action}")

def main(args=None):
    rclpy.init(args=args)
    node = InspireFtpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.backend.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

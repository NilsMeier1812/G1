#!/usr/bin/env python3
"""
ps4_arm_teleop — Oberkoerper (Arme + Haende) mit dem PS4-Controller steuern.

Liest den Controller in fester Belegung (ps4_layout.py) von /g1pilot/ps4/joy
(Quelle: ps4_joystick) und spricht DIESELBEN Schnittstellen wie RViz-Marker,
Demo-GUI und Streamdeck -- die Logik (IK, Kollisions-Gate, Haende) bleibt im
arm_controller bzw. in der Inspire-Bridge:

  /g1pilot/hand_goal/<side>     PoseStamped  kartesisches Hand-Ziel (pelvis)
  /g1pilot/hand_action/<side>   String       "open" / "close"
  /g1pilot/arms/enabled         Bool         Arme uebernehmen
  /g1pilot/start_balancing      Bool         Greifen-Modus (stehen, Arme frei)
  /g1pilot/arms/home            Bool         Grundstellung (Impuls)
  /g1pilot/pose_store/cancel    Bool         laufende geplante Bewegung abbrechen
  /g1pilot/emergency_stop       Bool         NOT-HALT (wie in den GUIs)

KEIN Laufen: der Node publiziert nichts an loco_cmd_vel / joy_mux. Rueckmeldung:
  /g1pilot/ps4/feedback  String(JSON)  LED-Farbe + Vibration -> ps4_joystick
  /g1pilot/ps4/status    String(JSON)  Auswahl/Modus/Tempo (fuer GUIs/Logs)
  /g1pilot/ps4/markers   MarkerArray   farbige Kugel an der aktiven Hand + Text

Belegung (Details: g1pilot/docs/43_ps4_controller.md):
  Steuerkreuz  links/rechts/oben/unten = linke / rechte / beide / beide gespiegelt
  Dreieck      Umschalten VERSCHIEBEN <-> DREHEN
  Linker Stick VERSCHIEBEN: vor/zurueck + links/rechts | DREHEN: kippen + schwenken
  Rechter Stick VERSCHIEBEN: hoch/runter (vertikal)    | DREHEN: rollen (horizontal)
  L1 / R1      Tempo langsamer / schneller (Langsam, Normal, Schnell)
  Kreis / Viereck  Hand oeffnen / schliessen (ausgewaehlte Hand/Haende)
  Kreuz        Stopp: Hand haelt sofort an, geplante Bewegung abbrechen
  Options      Greifen-Modus: stehen + Arme uebernehmen
  Share (1 s halten)  Grundstellung
  PS           NOT-HALT
"""
import json
import math
import time

from g1pilot.teleoperation import ps4_layout as L
from g1pilot.teleoperation.hand_jog import (
    JOG_FRAME, JOG_TF, JOG_SPEEDS, JOG_SPEED_ORDER, lead_limits, jog_step,
)

# Auswahl -> betroffene Seiten
SELECTIONS = {
    "left":   ("left",),
    "right":  ("right",),
    "both":   ("left", "right"),
    "mirror": ("left", "right"),
}
SELECTION_LABEL = {"left": "LINKS", "right": "RECHTS", "both": "BEIDE",
                   "mirror": "BEIDE gespiegelt"}
MODE_LABEL = {"move": "Verschieben", "turn": "Drehen"}

# Farben (RGB 0..255) -- Controller-LED und RViz-Marker gleich.
COLORS = {
    "left":   (0, 90, 255),     # blau
    "right":  (0, 200, 60),     # gruen
    "both":   (170, 0, 255),    # violett
    "mirror": (0, 200, 200),    # tuerkis
    "estop":  (255, 0, 0),      # rot
}
TURN_DIM = 0.35   # DREHEN: LED gedimmt (gleiche Farbe), Verschieben = voll


def shape_axis(x, deadzone, expo):
    """Stick-Wert -> Sollwert: Totzone (mit Neuskalierung, kein Sprung am Rand)
    und Expo-Kurve (feinfuehlig um die Mitte, voller Ausschlag bleibt 1)."""
    a = abs(x)
    if a <= deadzone:
        return 0.0
    a = min(1.0, (a - deadzone) / (1.0 - deadzone))
    a = (1.0 - expo) * a + expo * a ** 3
    return math.copysign(a, x)


def mirror_twist(lin, ang):
    """Spiegelung an der Sagittalebene (x-z) des Roboters: Verschiebung
    (vx, -vy, vz), Drehvektor (-wx, wy, -wz) -- Drehungen sind axial."""
    return (lin[0], -lin[1], lin[2]), (-ang[0], ang[1], -ang[2])


class ArmTeleopLogic:
    """Reine Bedienlogik (ohne ROS): Joy-Zustand rein, Befehle + Sollbewegung raus.

    update(axes, buttons, now) -> Liste von Befehlen (Tupel), z.B.
      ("hand", side, "open"|"close"), ("enable",), ("home",), ("cancel",),
      ("estop",), ("hold", sides), ("changed",)
    twist(axes) -> {side: (lin, ang)} fuer die gerade aktiven Seiten (oder {}).
    """

    def __init__(self, deadzone=0.12, expo=0.4, speed="Normal", home_hold_s=1.0,
                 selection="right"):
        self.deadzone = float(deadzone)
        self.expo = float(expo)
        self.home_hold_s = float(home_hold_s)
        self.selection = selection
        self.mode = "move"
        self.speed_idx = JOG_SPEED_ORDER.index(speed) if speed in JOG_SPEED_ORDER else 1
        self.estop = False
        self._prev = [0] * L.NUM_BUTTONS
        self._share_since = None
        self._home_sent = False

    # ── Zustand ─────────────────────────────────────────────────────────
    @property
    def speed(self):
        return JOG_SPEED_ORDER[self.speed_idx]

    @property
    def sides(self):
        return SELECTIONS[self.selection]

    def color(self):
        if self.estop:
            return COLORS["estop"]
        c = COLORS[self.selection]
        if self.mode == "turn":
            c = tuple(int(round(v * TURN_DIM)) for v in c)
        return c

    def label(self):
        if self.estop:
            return "PS4 · NOT-HALT (in GUI quittieren)"
        return f"PS4 · {SELECTION_LABEL[self.selection]} · {MODE_LABEL[self.mode]} · {self.speed}"

    def status(self):
        return {"selection": self.selection, "sides": list(self.sides),
                "mode": self.mode, "speed": self.speed, "estop": self.estop,
                "label": self.label()}

    def clear_estop(self):
        """NOT-HALT quittiert (START aus GUI/Streamdeck gesehen)."""
        if self.estop:
            self.estop = False
            return True
        return False

    # ── Eingabe ─────────────────────────────────────────────────────────
    def update(self, axes, buttons, now):
        b = list(buttons) + [0] * max(0, L.NUM_BUTTONS - len(buttons))
        rise = [1 if (b[i] and not self._prev[i]) else 0 for i in range(L.NUM_BUTTONS)]
        self._prev = b[:L.NUM_BUTTONS]
        out = []

        # NOT-HALT zuerst und unabhaengig von allem anderen.
        if rise[L.PS]:
            out.append(("hold", ("left", "right")))
            out.append(("estop",))
            if not self.estop:
                self.estop = True
                out.append(("changed",))
            return out
        if self.estop:
            return out   # bis zur Quittung in der GUI: nichts bewegen

        # Auswahl der Hand / Haende (Steuerkreuz)
        for btn, sel in ((L.DPAD_LEFT, "left"), (L.DPAD_RIGHT, "right"),
                         (L.DPAD_UP, "both"), (L.DPAD_DOWN, "mirror")):
            if rise[btn] and sel != self.selection:
                dropped = tuple(s for s in self.sides if s not in SELECTIONS[sel])
                if dropped:
                    out.append(("hold", dropped))
                self.selection = sel
                out.append(("changed",))

        if rise[L.TRIANGLE]:
            self.mode = "turn" if self.mode == "move" else "move"
            out.append(("changed",))
        if rise[L.L1] and self.speed_idx > 0:
            self.speed_idx -= 1
            out.append(("changed",))
        if rise[L.R1] and self.speed_idx < len(JOG_SPEED_ORDER) - 1:
            self.speed_idx += 1
            out.append(("changed",))

        if rise[L.CIRCLE]:
            out.extend(("hand", s, "open") for s in self.sides)
        if rise[L.SQUARE]:
            out.extend(("hand", s, "close") for s in self.sides)

        if rise[L.CROSS]:
            out.append(("hold", ("left", "right")))
            out.append(("cancel",))
        if rise[L.OPTIONS]:
            out.append(("enable",))

        # Grundstellung nur nach bewusstem Halten (versehentliches Antippen)
        if b[L.SHARE]:
            if self._share_since is None:
                self._share_since = now
            elif not self._home_sent and now - self._share_since >= self.home_hold_s:
                self._home_sent = True
                out.append(("home",))
        else:
            self._share_since = None
            self._home_sent = False
        return out

    def twist(self, axes):
        """-> {side: ((vx, vy, vz) m/s, (wx, wy, wz) rad/s)} im JOG_FRAME."""
        if self.estop:
            return {}
        a = list(axes) + [0.0] * max(0, L.NUM_AXES - len(axes))
        s = lambda i: shape_axis(a[i], self.deadzone, self.expo)   # noqa: E731
        v, w = JOG_SPEEDS[self.speed]
        if self.mode == "move":
            # Stick oben = vor (+x), links = links (+y); rechter Stick oben = hoch (+z)
            lin, ang = (s(L.LY) * v, s(L.LX) * v, s(L.RY) * v), (0.0, 0.0, 0.0)
        else:
            # Stick oben = Finger hoch: NEGATIVE Drehung um y (wie Demo-GUI);
            # links = schwenken nach links (+z); rechter Stick rechts = rollen
            # im Uhrzeigersinn von hinten gesehen (+x).
            lin = (0.0, 0.0, 0.0)
            ang = (-s(L.RX) * w, -s(L.LY) * w, s(L.LX) * w)
        if max(abs(c) for c in lin + ang) < 1e-6:
            return {}
        out = {}
        for side in self.sides:
            if self.selection == "mirror" and side == "left":
                out[side] = mirror_twist(lin, ang)   # Referenz = rechte Hand
            else:
                out[side] = (lin, ang)
        return out


# ════════════════════════════════════════════════════════════════════════
#  ROS-Node
# ════════════════════════════════════════════════════════════════════════
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, String
    from geometry_msgs.msg import PoseStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener

    class Ps4ArmTeleop(Node):
        def __init__(self):
            super().__init__("ps4_arm_teleop")
            self.declare_parameter("joy_topic", "/g1pilot/ps4/joy")
            self.declare_parameter("rate_hz", 30.0)
            self.declare_parameter("deadzone", 0.12)
            self.declare_parameter("expo", 0.4)
            self.declare_parameter("speed", "Normal")
            self.declare_parameter("home_hold_s", 1.0)
            # Bleibt der Joy-Stream laenger aus (Funkabriss, Akku leer), haelt
            # die Hand an -- der ps4_joystick publiziert nur bei Verbindung.
            self.declare_parameter("joy_timeout_s", 0.5)
            # Options = Greifen-Modus: zusaetzlich start_balancing senden (wie die
            # Kachel GREIFEN der Demo-GUI). false -> nur Arme uebernehmen.
            self.declare_parameter("options_starts_balancing", True)
            gp = lambda n: self.get_parameter(n).value   # noqa: E731

            self.logic = ArmTeleopLogic(gp("deadzone"), gp("expo"), gp("speed"),
                                        gp("home_hold_s"))
            self.joy_timeout = float(gp("joy_timeout_s"))
            self.options_balancing = bool(gp("options_starts_balancing"))
            self.axes, self.t_joy = [], None
            self.connected = False
            self.targets = {}            # side -> [pos, quat] laufendes Jog-Ziel
            self.t_tick = time.monotonic()
            self._tf_warned = False
            self._home_off_at = None

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.pub_goal = {s: self.create_publisher(PoseStamped, f"/g1pilot/hand_goal/{s}", 10)
                             for s in ("left", "right")}
            self.pub_hand = {s: self.create_publisher(String, f"/g1pilot/hand_action/{s}", 10)
                             for s in ("left", "right")}
            self.pub_enable = self.create_publisher(Bool, "/g1pilot/arms/enabled", 10)
            self.pub_balance = self.create_publisher(Bool, "/g1pilot/start_balancing", 10)
            self.pub_home = self.create_publisher(Bool, "/g1pilot/arms/home", 10)
            self.pub_cancel = self.create_publisher(Bool, "/g1pilot/pose_store/cancel", 10)
            self.pub_estop = self.create_publisher(Bool, "/g1pilot/emergency_stop", 10)
            self.pub_feedback = self.create_publisher(String, "/g1pilot/ps4/feedback", 10)
            latched = QoSProfile(depth=1)
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.pub_status = self.create_publisher(String, "/g1pilot/ps4/status", latched)
            self.pub_markers = self.create_publisher(MarkerArray, "/g1pilot/ps4/markers", 10)

            self.create_subscription(Joy, gp("joy_topic"), self._on_joy, 10)
            # START (GUI/Streamdeck) quittiert den NOT-HALT im arm_controller.
            self.create_subscription(Bool, "/g1pilot/start", self._on_start, 10)

            self.create_timer(1.0 / float(gp("rate_hz")), self._tick)
            self.create_timer(0.2, self._publish_markers)
            self.create_timer(1.0, self._publish_status)
            self._send_feedback(rumble_ms=0)
            self.get_logger().info(
                f"PS4-Arm-Steuerung bereit ({gp('joy_topic')}): {self.logic.label()}")

        # ── Eingaenge ──────────────────────────────────────────────────
        def _on_joy(self, msg):
            now = time.monotonic()
            if not self.connected:
                self.connected = True
                self.get_logger().info("PS4-Controller: Daten kommen an.")
                self._send_feedback()
            self.axes, self.t_joy = list(msg.axes), now
            for cmd in self.logic.update(msg.axes, msg.buttons, now):
                self._run(cmd)

        def _on_start(self, msg):
            if msg.data and self.logic.clear_estop():
                self.get_logger().info("NOT-HALT quittiert (START) -- PS4 wieder aktiv.")
                self._changed()

        def _run(self, cmd):
            kind = cmd[0]
            if kind == "hold":
                self._hold(cmd[1])
            elif kind == "hand":
                self.pub_hand[cmd[1]].publish(String(data=cmd[2]))
                self.get_logger().info(f"Hand {cmd[1]}: {cmd[2]}")
                self._send_feedback(rumble_ms=60, strong=0.0, weak=0.4)
            elif kind == "enable":
                if self.options_balancing:
                    self.pub_balance.publish(Bool(data=True))
                self.pub_enable.publish(Bool(data=True))
                self.get_logger().info("Greifen-Modus: Arme uebernommen.")
                self._send_feedback(rumble_ms=150)
            elif kind == "home":
                self._hold(("left", "right"))
                self.pub_home.publish(Bool(data=True))
                # Impuls wie _pulse der GUIs: nach 1 s False (in _tick)
                self._home_off_at = time.monotonic() + 1.0
                self.get_logger().info("Grundstellung angefordert (Share gehalten).")
                self._send_feedback(rumble_ms=300)
            elif kind == "cancel":
                self.pub_cancel.publish(Bool(data=True))
                self.get_logger().info("STOPP: Hand haelt, geplante Bewegung abgebrochen.")
                self._send_feedback(rumble_ms=80, strong=0.6, weak=0.0)
            elif kind == "estop":
                self.pub_estop.publish(Bool(data=True))
                self.get_logger().warn("NOT-HALT (PS-Taste). Quittieren in GUI/Streamdeck.")
                self._send_feedback(rumble_ms=600, strong=1.0, weak=1.0)
            elif kind == "changed":
                self._changed()

        def _changed(self):
            self.get_logger().info(self.logic.label())
            self._send_feedback(rumble_ms=100)
            self._publish_status()
            self._publish_markers()

        # ── Rueckmeldung ───────────────────────────────────────────────
        def _send_feedback(self, rumble_ms=0, strong=0.0, weak=0.5):
            fb = {"rgb": list(self.logic.color())}
            if rumble_ms > 0:
                fb["rumble"] = {"ms": int(rumble_ms), "strong": strong, "weak": weak}
            self.pub_feedback.publish(String(data=json.dumps(fb)))

        def _publish_status(self):
            st = self.logic.status()
            st["connected"] = self.connected
            self.pub_status.publish(String(data=json.dumps(st)))

        def _publish_markers(self):
            arr = MarkerArray()
            stamp = self.get_clock().now().to_msg()
            r, g, b = (c / 255.0 for c in self.logic.color())
            if self.logic.estop:
                r, g, b = 1.0, 0.0, 0.0
            active = self.logic.sides if not self.logic.estop else ()
            for i, side in enumerate(("left", "right")):
                m = Marker()
                m.header.frame_id = JOG_TF[side]
                m.header.stamp = stamp
                m.ns, m.id = "ps4_hand", i
                m.frame_locked = True
                if side in active:
                    m.type, m.action = Marker.SPHERE, Marker.ADD
                    m.pose.orientation.w = 1.0
                    m.scale.x = m.scale.y = m.scale.z = 0.09
                    m.color.r, m.color.g, m.color.b = r, g, b
                    m.color.a = 0.55 if self.connected else 0.2
                else:
                    m.action = Marker.DELETE
                arr.markers.append(m)
            t = Marker()
            t.header.frame_id = JOG_FRAME
            t.header.stamp = stamp
            t.ns, t.id = "ps4_label", 0
            t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.z = 0.75
            t.pose.orientation.w = 1.0
            t.scale.z = 0.06
            t.color.r, t.color.g, t.color.b, t.color.a = max(r, 0.3), max(g, 0.3), max(b, 0.3), 1.0
            t.text = self.logic.label() + ("" if self.connected else " · nicht verbunden")
            arr.markers.append(t)
            self.pub_markers.publish(arr)

        # ── Hand bewegen ───────────────────────────────────────────────
        def _hand_pose(self, side):
            try:
                tr = self.tf_buffer.lookup_transform(JOG_FRAME, JOG_TF[side], rclpy.time.Time())
            except Exception:   # noqa: BLE001 -- Lookup/Connectivity/Extrapolation
                return None
            p, q = tr.transform.translation, tr.transform.rotation
            return [p.x, p.y, p.z], (q.x, q.y, q.z, q.w)

        def _publish_goal(self, side, pos, rot):
            msg = PoseStamped()
            msg.header.frame_id = JOG_FRAME
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pos)
            o = msg.pose.orientation
            o.x, o.y, o.z, o.w = map(float, rot)
            self.pub_goal[side].publish(msg)

        def _hold(self, sides):
            """Ziel auf die echte Hand setzen -> Arm bleibt sofort stehen statt
            den Vorlauf noch abzufahren (wie Loslassen in der Demo-GUI)."""
            for side in sides:
                if self.targets.pop(side, None) is None:
                    continue
                hand = self._hand_pose(side)
                if hand is not None:
                    self._publish_goal(side, *hand)

        def _tick(self):
            now = time.monotonic()
            dt = min(0.1, now - self.t_tick)
            self.t_tick = now
            if self._home_off_at is not None and now >= self._home_off_at:
                self._home_off_at = None
                self.pub_home.publish(Bool(data=False))
            if self.t_joy is not None and now - self.t_joy > self.joy_timeout:
                if self.connected:
                    self.connected = False
                    self.get_logger().warn("PS4-Controller: keine Daten mehr -> Hand haelt an.")
                    self._publish_status()
                self._hold(tuple(self.targets))
                return
            tw = self.logic.twist(self.axes) if self.connected else {}
            # Seiten, die nicht mehr bewegt werden (Stick losgelassen): anhalten
            self._hold(tuple(s for s in self.targets if s not in tw))
            if not tw:
                return
            v, w = JOG_SPEEDS[self.logic.speed]
            lead_m, lead_rad = lead_limits(v, w)
            for side, (lin, ang) in tw.items():
                hand = self._hand_pose(side)
                if hand is None:
                    if not self._tf_warned:
                        self._tf_warned = True
                        self.get_logger().warn(
                            f"Handposition {side} unbekannt (TF {JOG_FRAME}->{JOG_TF[side]} "
                            "fehlt) -- laeuft robot_state?")
                    continue
                self._tf_warned = False
                p, q = jog_step(self.targets.get(side), hand, lin, ang, dt, lead_m, lead_rad)
                self.targets[side] = [p, q]
                self._publish_goal(side, p, q)

    rclpy.init(args=args)
    node = Ps4ArmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

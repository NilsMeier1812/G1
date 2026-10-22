#!/usr/bin/env python3
"""
ps4_walk — Gehen mit dem PS4-Controller (neben der Arm-/Hand-Steuerung).

Liest denselben Controller wie ps4_arm_teleop (/g1pilot/ps4/joy, feste
Belegung ps4_layout.py). Gehen nur, solange L2 gehalten wird (Deadman):

  L2 halten          Gehen freigeben; Sticks gehoeren dann dem Gehen
                     (die Arm-Steuerung pausiert, Arme halten ihre Lage)
  Linker Stick       vor/zurueck + seitlich
  Rechter Stick      drehen (horizontal)
  R1 / L1           Geh-Tempo schneller / langsamer (wie das Arm-Tempo),
                     solange L2 gehalten wird oder Modus GEHEN gewaehlt ist
  L2 + Steuerkreuz ▲/▼  ebenso Geh-Tempo schneller / langsamer
  L2 loslassen       sofort Stopp

Ausgang: /g1pilot/ps4/walk_cmd (Twist, normiert [-1, 1] wie loco_cmd_vel).
loco_client (real) skaliert mit max_vx/max_vy/max_vyaw und ruft Move() nur,
solange der G1 schon im Walk-Modus steht -- der Controller schaltet den
Modus der Beine nie selbst um (Damp/Preparation/Walk bleibt GUI/Handy-App).
In der Sim nimmt loco_sim das Topic im WALK-Zustand wie loco_cmd_vel.

Sicherheit: kommt kein Joy mehr (Funkabriss, Stall-Erkennung im
ps4_joystick), sendet der Node einmal 0 und dann nichts mehr; loco_client
stoppt zusaetzlich selbst, wenn der Strom ausbleibt (cmd_vel_timeout).
NOT-HALT (PS bzw. von GUI/Streamdeck) sperrt das Gehen bis START.

AUTO NAV (nav_supervisor, docs/50_navigation_anleitung.md): solange eine
Navigation angefragt ist (nav/state queued/running/paused mit Totmann), geht
L2 NICHT selbst, sondern ist der Totmann der Autonomie: L2 + linker Stick
startet, L2 halten laeuft weiter, L2 los = Stopp. Dafuer meldet der Node
/g1pilot/ps4/deadman mit 20 Hz, solange Joy-Daten kommen.

Rueckmeldung: /g1pilot/ps4/walk_status (JSON) + Vibration ueber
/g1pilot/ps4/feedback (ohne LED-Farbe -- die LED gehoert ps4_arm_teleop).
Geh-Tempo im gemeinsamen Zustand mit der Demo-GUI (/g1pilot/teleop_state,
shared_state.py): Stufe am Controller -> GUI zeigt sie, Stufe in der GUI ->
gilt auch fuer L2 (auf max_speed begrenzt).
"""
import json
import time

from g1pilot.teleoperation import ps4_layout as L
from g1pilot.teleoperation import shared_state

WALK_SPEED_ORDER = ("Langsam", "Normal", "Schnell")
# Anteil der loco_client-Limits (max_vx/max_vy/max_vyaw) je Stufe.
WALK_SPEED_SCALE = {"Langsam": 0.35, "Normal": 0.65, "Schnell": 1.0}
WALK_CMD_TOPIC = "/g1pilot/ps4/walk_cmd"
DEADMAN_THRESHOLD = 0.5   # L2 mindestens halb gedrueckt
# Totmann fuer AUTO NAV (nav_supervisor): {"held": L2 scharf gehalten,
# "go": linker Stick ausgelenkt}. Kommt nur, solange der Controller Daten
# liefert -- Funkabriss = keine Meldung = Stopp im nav_supervisor.
DEADMAN_TOPIC = "/g1pilot/ps4/deadman"
NAV_STATE_TOPIC = "/g1pilot/nav/state"
GO_THRESHOLD = 0.5        # linker Stick so weit ausgelenkt = "los"
NAV_STATE_TIMEOUT_S = 0.5  # aelterer nav/state gilt nicht (Supervisor tot)
# In diesen AUTO-NAV-Zustaenden gehoert L2 dem nav_supervisor: kein eigenes Gehen.
# Auch "stopped": wer nach einem Waechter-Stopp L2 + Stick drueckt, will weiter
# navigieren -- der G1 darf dann NICHT von Hand in Stick-Richtung loslaufen.
NAV_OWNS_L2 = ("queued", "running", "paused", "stopped")


def go_gesture(axes):
    """Linker Stick deutlich ausgelenkt (Losgeh-Geste fuer AUTO NAV)."""
    a = list(axes) + [0.0] * max(0, L.NUM_AXES - len(axes))
    return abs(a[L.LX]) >= GO_THRESHOLD or abs(a[L.LY]) >= GO_THRESHOLD


def deadman_held(axes, buttons):
    """True, solange L2 (Gehen) gehalten wird -- Achse oder Taste."""
    if len(axes) > L.L2_AXIS and axes[L.L2_AXIS] >= DEADMAN_THRESHOLD:
        return True
    return len(buttons) > L.L2 and bool(buttons[L.L2])


def shape_axis(x, deadzone, expo):
    """Totzone + Expo wie ps4_arm_teleop.shape_axis (hier eigene Kopie: der
    Arm-Node importiert dieses Modul, umgekehrt gaebe es einen Import-Kreis)."""
    a = abs(x)
    if a <= deadzone:
        return 0.0
    a = min(1.0, (a - deadzone) / (1.0 - deadzone))
    a = (1.0 - expo) * a + expo * a ** 3
    return a if x >= 0 else -a


class WalkLogic:
    """Reine Bedienlogik (ohne ROS).

    update(axes, buttons) -> Liste von Ereignissen: ("start",), ("stop",),
    ("speed",), ("estop",). command(axes) -> (vx, vy, vyaw) normiert oder None
    (kein Gehen: Deadman los oder NOT-HALT)."""

    def __init__(self, deadzone=0.12, expo=0.4, speed="Langsam", max_speed="Schnell",
                 ps_estop=True):
        self.deadzone = float(deadzone)
        self.expo = float(expo)
        self.max_idx = (WALK_SPEED_ORDER.index(max_speed) if max_speed in WALK_SPEED_ORDER
                        else len(WALK_SPEED_ORDER) - 1)
        idx = WALK_SPEED_ORDER.index(speed) if speed in WALK_SPEED_ORDER else 0
        self.speed_idx = min(idx, self.max_idx)
        self.ps_estop = bool(ps_estop)
        self.estop = False
        self.active = False
        # Modus GEHEN gewaehlt (gemeinsamer Zustand, Kachel bzw. L2-Druck):
        # dann gehoeren R1/L1 dem Geh-Tempo, auch ohne gehaltenes L2 (die
        # Arm-Tasten sind im GEHEN gesperrt).
        self.walk_mode = False
        # Tempo vor dem ersten R1/L1-Schritt, solange R1 oder L1 gehalten:
        # wird daraus der Voll-Notaus (R1 + L1 + PS), zuruecknehmen.
        self._speed_undo = None
        # Erst nach einem gesehenen "L2 los" scharf: nach Node-Start, Funkabriss
        # oder NOT-HALT laeuft der G1 nicht los, nur weil L2 noch gedrueckt ist.
        self._armed = False
        self._prev = [0] * L.NUM_BUTTONS

    @property
    def speed(self):
        return WALK_SPEED_ORDER[self.speed_idx]

    def set_speed(self, name):
        """Geh-Tempo von aussen (GUI), auf max_speed begrenzt. -> geaendert?"""
        if name not in WALK_SPEED_ORDER:
            return False
        idx = min(WALK_SPEED_ORDER.index(name), self.max_idx)
        changed = idx != self.speed_idx
        self.speed_idx = idx
        return changed

    def status(self):
        return {"active": self.active, "speed": self.speed, "estop": self.estop,
                "max_speed": WALK_SPEED_ORDER[self.max_idx]}

    def latch_estop(self):
        was = self.estop
        self.estop = True
        return self._deactivate() if not was else []

    def clear_estop(self):
        if self.estop:
            self.estop = False
            self._armed = False
            return True
        return False

    def release(self):
        """Joy-Strom weg: wie Deadman los."""
        self._prev = [0] * L.NUM_BUTTONS
        return self._deactivate()

    def disarm(self):
        """Wie Deadman los, aber ohne die Tastenflanken zurueckzusetzen
        (Wechsel Gehen <-> AUTO NAV): erst nach "L2 los" wieder scharf."""
        return self._deactivate()

    def _deactivate(self):
        self._armed = False
        if self.active:
            self.active = False
            return [("stop",)]
        return []

    def update(self, axes, buttons):
        b = list(buttons) + [0] * max(0, L.NUM_BUTTONS - len(buttons))
        rise = [1 if (b[i] and not self._prev[i]) else 0 for i in range(L.NUM_BUTTONS)]
        self._prev = b[:L.NUM_BUTTONS]
        out = []
        if not b[L.L1] and not b[L.R1]:
            self._speed_undo = None
        if rise[L.PS] and b[L.L1] and b[L.R1] and self._speed_undo is not None:
            # R1 + L1 vor PS gedrueckt: deren Tempo-Schritt gehoerte zum
            # Voll-Notaus -- ohne Vibration (die Halte-Vibration laeuft).
            if self._speed_undo != self.speed_idx:
                self.speed_idx = self._speed_undo
                out.append(("speed", "quiet"))
            self._speed_undo = None
        if rise[L.PS] and self.ps_estop:
            out += self.latch_estop()
            out.append(("estop",))
            return out
        held = deadman_held(axes, b)
        if not held:
            out += self._deactivate()
            self._armed = not self.estop
        elif self._armed and not self.active and not self.estop:
            self.active = True
            out.append(("start",))
        if self.estop:
            return out
        faster = slower = False
        if self.active or self.walk_mode:
            faster, slower = rise[L.R1], rise[L.L1]
        if self.active:
            faster, slower = faster or rise[L.DPAD_UP], slower or rise[L.DPAD_DOWN]
        step = 0
        if faster and not slower and self.speed_idx < self.max_idx:
            step = +1
        elif slower and not faster and self.speed_idx > 0:
            step = -1
        if step:
            if self._speed_undo is None and (b[L.L1] or b[L.R1]):
                self._speed_undo = self.speed_idx
            self.speed_idx += step
            out.append(("speed",))
        return out

    def command(self, axes):
        """Stick oben = vor (+x), links = links (+y), rechter Stick links =
        nach links drehen (+yaw). Werte normiert, mit Tempo-Stufe skaliert."""
        if not self.active or self.estop:
            return None
        a = list(axes) + [0.0] * max(0, L.NUM_AXES - len(axes))
        k = WALK_SPEED_SCALE[self.speed]
        s = lambda i: shape_axis(a[i], self.deadzone, self.expo) * k   # noqa: E731
        return s(L.LY), s(L.LX), s(L.RX)


# ════════════════════════════════════════════════════════════════════════
#  ROS-Node
# ════════════════════════════════════════════════════════════════════════
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, String

    class Ps4Walk(Node):
        def __init__(self):
            super().__init__("ps4_walk")
            self.declare_parameter("joy_topic", "/g1pilot/ps4/joy")
            self.declare_parameter("rate_hz", 20.0)
            self.declare_parameter("deadzone", 0.12)
            self.declare_parameter("expo", 0.4)
            # Real: Start Langsam (35 % der loco_client-Limits).
            self.declare_parameter("speed", "Langsam")
            self.declare_parameter("max_speed", "Schnell")
            self.declare_parameter("ps_estop", True)
            # Kommt so lange kein Joy, gilt der Deadman als losgelassen.
            self.declare_parameter("joy_timeout_s", 0.3)
            gp = lambda n: self.get_parameter(n).value   # noqa: E731
            self.logic = WalkLogic(gp("deadzone"), gp("expo"), gp("speed"),
                                   gp("max_speed"), gp("ps_estop"))
            self.joy_timeout = float(gp("joy_timeout_s"))
            self.axes, self.t_joy = [], None
            self.walk_ready = None       # G1 im Walk-Modus? None = unbekannt
            self.nav_state, self.t_nav = None, None
            self._owned = False

            self.pub_cmd = self.create_publisher(Twist, WALK_CMD_TOPIC, 10)
            self.pub_deadman = self.create_publisher(String, DEADMAN_TOPIC, 10)
            self.pub_feedback = self.create_publisher(String, "/g1pilot/ps4/feedback", 10)
            latched = QoSProfile(depth=1)
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.pub_status = self.create_publisher(String, "/g1pilot/ps4/walk_status", latched)

            self.create_subscription(Joy, gp("joy_topic"), self._on_joy, 10)
            self.create_subscription(Bool, "/g1pilot/emergency_stop", self._on_estop, 10)
            self.create_subscription(Bool, "/g1pilot/start", self._on_start, 10)
            # Nur fuer die Rueckmeldung "G1 nicht im Walk-Modus" (Real/Sim).
            self.create_subscription(String, "/g1pilot/onboard_state", self._on_onboard,
                                     latched)
            self.create_subscription(String, "/g1pilot/loco_state", self._on_loco_state,
                                     latched)
            self.create_subscription(String, NAV_STATE_TOPIC, self._on_nav_state, latched)
            self.create_timer(1.0 / float(gp("rate_hz")), self._tick)
            # Gemeinsamer Zustand mit der Demo-GUI: Start-Tempo gilt ab Node-Start.
            self.shared = shared_state.SharedState("ps4_walk")
            self.t_start_wall = time.time()
            self.shared.set("walk_speed", self.logic.speed)
            self.pub_shared = self.create_publisher(String, shared_state.TOPIC, latched)
            self.create_subscription(String, shared_state.TOPIC, self._on_shared, latched)
            self.pub_shared.publish(String(data=self.shared.to_json()))
            self._publish_status()
            self.get_logger().info(
                f"PS4-Gehen bereit: L2 halten = Gehen, Tempo {self.logic.speed} "
                f"(max. {WALK_SPEED_ORDER[self.logic.max_idx]}) -> {WALK_CMD_TOPIC}")

        # ── Eingaenge ──────────────────────────────────────────────────
        def _on_joy(self, msg):
            self.axes, self.t_joy = list(msg.axes), time.monotonic()
            for ev in self.logic.update(msg.axes, msg.buttons):
                self._event(ev)

        def _on_estop(self, msg):
            if msg.data and not self.logic.estop:
                for ev in self.logic.latch_estop():
                    self._event(ev)
                self.get_logger().warn("NOT-HALT -- Gehen gesperrt bis START.")
                self._publish_status()

        def _on_start(self, msg):
            if msg.data and self.logic.clear_estop():
                self.get_logger().info("NOT-HALT quittiert -- Gehen wieder frei.")
                self._publish_status()

        def _on_shared(self, msg):
            ch = self.shared.merge(msg.data)
            if "walk_speed" in ch and self.logic.set_speed(ch["walk_speed"]):
                self.get_logger().info(f"Geh-Tempo von der GUI: {self.logic.speed}")
                self._publish_status()
            # Modus GEHEN (Kachel oder L2-Druck am Controller) -> R1/L1 = Geh-Tempo.
            # Nur Wechsel nach dem Node-Start: ein alter latched Stand beschreibt
            # den Roboter nicht mehr (wie ps4_arm_teleop).
            if "mode" in ch and self.shared.stamps.get("mode", 0.0) > self.t_start_wall:
                self.logic.walk_mode = ch["mode"] == "walk"

        def _on_onboard(self, msg):
            try:
                self.walk_ready = bool(json.loads(msg.data).get("balanced"))
            except (TypeError, ValueError, AttributeError):
                pass

        def _on_loco_state(self, msg):
            self.walk_ready = msg.data == "walk"

        def _on_nav_state(self, msg):
            try:
                d = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            owns = d.get("state") in NAV_OWNS_L2 and bool(d.get("require_deadman"))
            self.nav_state, self.t_nav = (d.get("state") if owns else None), time.monotonic()
            self._nav_handover()

        def _nav_handover(self):
            """Wechsel der L2-Zustaendigkeit Gehen <-> AUTO NAV. In beide
            Richtungen erst nach einem gesehenen "L2 los" wieder scharf, und
            ein laufendes Hand-Gehen bekommt sofort eine 0."""
            owns = self._nav_owns()
            if owns == self._owned:
                return
            self._owned = owns
            self.get_logger().info(
                "AUTO NAV angefragt: L2 ist jetzt der Totmann der Navigation "
                "(L2 + linker Stick = los)." if owns else
                "AUTO NAV aus: L2 geht wieder selbst (erst L2 loslassen).")
            for ev in self.logic.disarm():
                self._event(ev)
            self._publish_status()

        def _nav_owns(self):
            return (self.nav_state is not None and self.t_nav is not None
                    and time.monotonic() - self.t_nav <= NAV_STATE_TIMEOUT_S)

        def _event(self, ev):
            kind = ev[0]
            if kind == "start":
                if self._nav_owns():
                    self.get_logger().info("L2: Totmann AUTO NAV gehalten.")
                    self._rumble(80, strong=0.0, weak=0.5)
                elif self.walk_ready is False:
                    self.get_logger().warn(
                        "L2: G1 steht nicht im Walk-Modus -- erst in der GUI "
                        "(Startleiste »Walk« bzw. LAUFEN) oder per Handy-App umschalten.")
                    self._rumble(400, strong=0.8, weak=0.0)
                else:
                    self.get_logger().info(f"Gehen frei (L2), Tempo {self.logic.speed}.")
                    self._rumble(80, strong=0.0, weak=0.5)
            elif kind == "stop":
                self._send(0.0, 0.0, 0.0)
                self.get_logger().info("Gehen: Stopp (L2 los).")
            elif kind == "speed":
                self.get_logger().info(f"Geh-Tempo: {self.logic.speed}")
                if self.shared.set("walk_speed", self.logic.speed):
                    self.pub_shared.publish(String(data=self.shared.to_json()))
                if ev[1:] != ("quiet",):
                    self._rumble(60 + 80 * self.logic.speed_idx, strong=0.0, weak=0.6)
            elif kind == "estop":
                self._send(0.0, 0.0, 0.0)
            self._publish_status()

        # ── Ausgaenge ──────────────────────────────────────────────────
        def _send(self, vx, vy, vyaw):
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(vyaw)
            self.pub_cmd.publish(t)

        def _rumble(self, ms, strong, weak):
            fb = {"rumble": {"ms": int(ms), "strong": strong, "weak": weak}}
            self.pub_feedback.publish(String(data=json.dumps(fb)))

        def _publish_status(self):
            st = self.logic.status()
            st["walk_ready"] = self.walk_ready
            self.pub_status.publish(String(data=json.dumps(st)))

        def _tick(self):
            if self.t_joy is not None and time.monotonic() - self.t_joy > self.joy_timeout:
                if self.logic.active:
                    self.get_logger().warn("PS4: keine Daten mehr -> Gehen gestoppt.")
                for ev in self.logic.release():
                    self._event(ev)
                return
            self._nav_handover()     # Supervisor verstummt -> Zustaendigkeit endet
            if self.t_joy is not None:
                held = self.logic.active and not self.logic.estop
                self.pub_deadman.publish(String(data=json.dumps(
                    {"held": held, "go": held and go_gesture(self.axes)})))
            if self._nav_owns():
                return          # Sticks gehoeren der Navigation, nicht dem Gehen
            cmd = self.logic.command(self.axes)
            if cmd is not None:
                self._send(*cmd)

    rclpy.init(args=args)
    node = Ps4Walk()
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

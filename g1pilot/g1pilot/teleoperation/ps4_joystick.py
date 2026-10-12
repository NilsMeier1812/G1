#!/usr/bin/env python3
"""
ps4_joystick — PS4-Controller (evdev) -> /g1pilot/ps4/joy in FESTER Belegung.

Unterschiede zum alten `joystick`-Node (der fuer loco_client/joy_mux bleibt
und hier NICHT angefasst wird):
  * feste Belegung aus ps4_layout.py statt evdev-Reihenfolge,
  * eigenes Topic (/g1pilot/ps4/joy) -> nichts davon erreicht joy_mux und damit
    das Laufen (joy_to_cmdvel / loco_client),
  * Wiederverbinden: Controller darf spaeter eingeschaltet werden oder kurz
    wegbrechen. Ohne Verbindung wird NICHTS publiziert (der Empfaenger merkt
    den Abriss am Timeout und haelt die Hand an), nie ein eingefrorener Wert,
  * Rueckmeldung am Controller (best effort): LED-Farbe ueber sysfs und
    Vibration ueber evdev-Force-Feedback, gesteuert ueber /g1pilot/ps4/feedback.
"""
import glob
import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from g1pilot.teleoperation import ps4_layout as L

try:
    import evdev
    from evdev import ecodes
except ImportError:   # Tests/Container ohne evdev: Node meldet es sauber
    evdev = None
    ecodes = None


class Ps4Joystick(Node):
    def __init__(self):
        super().__init__("ps4_joystick")
        self.declare_parameter("joystick_name", "Wireless Controller")
        self.declare_parameter("topic", "/g1pilot/ps4/joy")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("reconnect_period_s", 1.0)
        self.wanted = self.get_parameter("joystick_name").value
        self.reconnect_s = float(self.get_parameter("reconnect_period_s").value)

        self.pub = self.create_publisher(Joy, self.get_parameter("topic").value, 10)
        self.create_subscription(String, "/g1pilot/ps4/feedback", self._on_feedback, 10)

        self.lock = threading.Lock()
        self.state = L.Ps4State()
        self.device = None
        self._led = None          # (kind, paths) oder None
        self._led_warned = False
        self._ff_effect = None
        self._rgb = None
        self._stop = False

        if evdev is None:
            self.get_logger().error("Python-Paket 'evdev' fehlt -- kein PS4-Controller.")
            return
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish)

    # ── Geraet finden / lesen ──────────────────────────────────────────
    def _find(self):
        candidates = []
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
            except OSError:
                continue
            if dev.name == self.wanted:
                for extra in candidates:
                    extra.close()
                return dev
            if L.is_ps4_name(dev.name, self.wanted):
                candidates.append(dev)
            else:
                dev.close()
        for extra in candidates[1:]:
            extra.close()
        return candidates[0] if candidates else None

    def _reader(self):
        missing_logged = False
        while not self._stop:
            dev = self._find()
            if dev is None:
                if not missing_logged:
                    missing_logged = True
                    self.get_logger().warn(
                        f"Kein PS4-Controller gefunden (gesucht: '{self.wanted}'). "
                        "Warte ... (Bluetooth koppeln oder per USB anstecken; im "
                        "Container braucht es /dev/input, siehe docs/43_ps4_controller.md)")
                time.sleep(self.reconnect_s)
                continue
            missing_logged = False
            absinfo = {}
            for code, info in dev.capabilities().get(ecodes.EV_ABS, []):
                absinfo[code] = (info.min, info.max)
            state = L.Ps4State(absinfo)
            # Startwerte (z.B. Trigger halb gedrueckt beim Verbinden)
            for code, info in dev.capabilities().get(ecodes.EV_ABS, []):
                state.apply(L.EV_ABS, code, info.value)
            with self.lock:
                self.state = state
                self.device = dev
            self._led = self._find_led(dev)
            self._ff_effect = None
            self.get_logger().info(f"PS4-Controller verbunden: {dev.name} ({dev.path})"
                                   + ("" if self._led else " -- LED nicht steuerbar"))
            if self._rgb is not None:
                self._set_led(self._rgb)
            try:
                for ev in dev.read_loop():
                    with self.lock:
                        self.state.apply(ev.type, ev.code, ev.value)
            except OSError as e:
                self.get_logger().warn(f"PS4-Controller getrennt ({e}). Warte auf Wiederverbindung ...")
            with self.lock:
                self.device = None
                self.state.reset()
            try:
                dev.close()
            except Exception:   # noqa: BLE001
                pass

    def _publish(self):
        with self.lock:
            if self.device is None:
                return
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "ps4"
            msg.axes = [float(a) for a in self.state.axes]
            msg.buttons = list(self.state.buttons)
        self.pub.publish(msg)

    # ── Rueckmeldung: LED + Vibration (best effort) ────────────────────
    @staticmethod
    def _find_led(dev):
        """LED-Klassen des Controllers in sysfs suchen.
        hid-playstation: ein Mehrfarb-LED '<...>:rgb:indicator' (multi_intensity),
        hid-sony: getrennte '<...>:red|green|blue'."""
        ev = os.path.basename(dev.path)
        base = os.path.realpath(f"/sys/class/input/{ev}/device/device")
        leds = glob.glob(os.path.join(base, "leds", "*"))
        rgb = [p for p in leds if p.endswith(":rgb:indicator")]
        if rgb:
            return ("multi", rgb[0])
        parts = {c: [p for p in leds if p.endswith(":" + c)] for c in ("red", "green", "blue")}
        if all(parts.values()):
            return ("split", {c: v[0] for c, v in parts.items()})
        return None

    def _set_led(self, rgb):
        if not self._led:
            return
        kind, where = self._led
        try:
            if kind == "multi":
                with open(os.path.join(where, "multi_intensity"), "w") as f:
                    f.write(" ".join(str(int(c)) for c in rgb))
                with open(os.path.join(where, "max_brightness")) as f:
                    mx = f.read().strip()
                with open(os.path.join(where, "brightness"), "w") as f:
                    f.write(mx)
            else:
                for c, v in zip(("red", "green", "blue"), rgb):
                    with open(os.path.join(where[c], "brightness"), "w") as f:
                        f.write(str(int(v)))
        except OSError as e:
            if not self._led_warned:
                self._led_warned = True
                self.get_logger().warn(f"LED nicht setzbar ({e}) -- Farbe nur in RViz.")

    def _rumble(self, ms, strong, weak):
        dev = self.device
        if dev is None or ecodes.EV_FF not in dev.capabilities():
            return
        try:
            if self._ff_effect is not None:
                dev.erase_effect(self._ff_effect)
                self._ff_effect = None
            rumble = evdev.ff.Rumble(strong_magnitude=int(0xFFFF * max(0.0, min(1.0, strong))),
                                     weak_magnitude=int(0xFFFF * max(0.0, min(1.0, weak))))
            effect = evdev.ff.Effect(
                ecodes.FF_RUMBLE, -1, 0, evdev.ff.Trigger(0, 0),
                evdev.ff.Replay(int(ms), 0),
                evdev.ff.EffectType(ff_rumble_effect=rumble))
            self._ff_effect = dev.upload_effect(effect)
            dev.write(ecodes.EV_FF, self._ff_effect, 1)
        except OSError:
            pass   # z.B. nur Lesezugriff -> Vibration einfach weglassen

    def _on_feedback(self, msg):
        try:
            fb = json.loads(msg.data)
        except ValueError:
            return
        if "rgb" in fb and len(fb["rgb"]) == 3:
            self._rgb = tuple(max(0, min(255, int(c))) for c in fb["rgb"])
            self._set_led(self._rgb)
        r = fb.get("rumble")
        if r and evdev is not None:
            self._rumble(r.get("ms", 100), r.get("strong", 0.0), r.get("weak", 0.5))

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ps4Joystick()
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

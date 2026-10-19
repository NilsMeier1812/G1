#!/usr/bin/env python3
"""
ps4_joystick — PS4- oder Xbox-Controller (evdev) -> /g1pilot/ps4/joy in
FESTER Belegung.

Welcher Controller, erkennt der Node selbst (Name/Hersteller, Parameter
`controller` = auto | ps4 | xbox). Der Xbox-Controller wird nach Position auf
die PS4-Belegung abgebildet (ps4_layout.XboxState) -- alle Nodes dahinter
bleiben unveraendert.

Unterschiede zum alten `joystick`-Node (der fuer loco_client/joy_mux bleibt
und hier NICHT angefasst wird):
  * feste Belegung aus ps4_layout.py statt evdev-Reihenfolge,
  * eigenes Topic (/g1pilot/ps4/joy) -> nichts davon erreicht joy_mux und damit
    das Laufen (joy_to_cmdvel / loco_client),
  * Wiederverbinden: Controller darf spaeter eingeschaltet werden oder kurz
    wegbrechen. Ohne Verbindung wird NICHTS publiziert (der Empfaenger merkt
    den Abriss am Timeout und haelt die Hand an), nie ein eingefrorener Wert,
  * Rueckmeldung am Controller (best effort): LED-Farbe ueber sysfs und
    Vibration ueber evdev-Force-Feedback, gesteuert ueber /g1pilot/ps4/feedback,
  * Lebendpruefung: haengt die Bluetooth-Verbindung, ohne dass der Kernel die
    Trennung meldet (das kommt erst nach dem Supervision-Timeout, mehrere
    Sekunden), wuerde sonst der letzte Zustand -- z.B. ein ausgelenkter
    Stick -- weiter publiziert. Das Teilgeraet "<Name> Motion Sensors"
    liefert mit jedem Funkbericht Ereignisse (u.a. MSC_TIMESTAMP); bleiben
    sie (und Tasten/Sticks) laenger als `stall_timeout_s` aus, wird nichts
    mehr publiziert, bis wieder Daten kommen.
    Der Xbox-Controller hat kein Sensor-Teilgeraet. Ueber Bluetooth wird
    stattdessen der rohe HID-Strom (/dev/hidrawN desselben Geraets) gelesen
    -- aber erst dann als Lebenszeichen genommen, wenn er nachweislich auch
    ohne Eingabe-Aenderung weiterlaeuft (`hidraw_arm_reports` Berichte am
    Stueck, ohne dass sich an Tasten/Sticks etwas aendert). Ein Controller,
    der nur bei Aenderungen sendet, bleibt so ohne Lebendpruefung, statt
    beim ruhig gehaltenen Stick faelschlich abzuschalten.
"""
import glob
import json
import os
import select
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
        self.declare_parameter("stall_timeout_s", 0.3)
        # auto: PS4 oder Xbox, was da ist (beide da: PS4, wie bisher)
        self.declare_parameter("controller", "auto")
        # Xbox: welche Taste ist "PS" (NOT-HALT, Voll-Notaus)? xbox | view
        self.declare_parameter("xbox_ps_button", "xbox")
        self.declare_parameter("xbox_swap_xy", False)
        self.declare_parameter("hidraw_arm_reports", 20)
        self.wanted = self.get_parameter("joystick_name").value
        self.kind_wanted = str(self.get_parameter("controller").value).strip().lower() or "auto"
        if self.kind_wanted not in ("auto", "ps4", "xbox"):
            self.get_logger().warn(f"controller='{self.kind_wanted}' unbekannt -> auto")
            self.kind_wanted = "auto"
        self.xbox_ps = str(self.get_parameter("xbox_ps_button").value).strip().lower()
        if self.xbox_ps not in L.XBOX_PS_BUTTONS:
            self.get_logger().warn(f"xbox_ps_button='{self.xbox_ps}' unbekannt -> xbox")
            self.xbox_ps = "xbox"
        self.xbox_swap = bool(self.get_parameter("xbox_swap_xy").value)
        self.hidraw_arm_n = max(1, int(self.get_parameter("hidraw_arm_reports").value))
        self.reconnect_s = float(self.get_parameter("reconnect_period_s").value)
        self.stall_s = float(self.get_parameter("stall_timeout_s").value)

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
        # Lebendpruefung: Zeit des letzten Ereignisses; None = nicht scharf
        # (kein Sensor-Teilgeraet gefunden oder es hat noch nichts geliefert).
        self._alive_t = None
        self._gen = 0             # Verbindungs-Nummer (alte Sensor-Threads ignorieren)
        self._stalled = False
        self.kind = None          # 'ps4' | 'xbox' des verbundenen Controllers
        self._quiet_reports = 0   # hidraw-Berichte seit der letzten evdev-Aenderung

        if evdev is None:
            self.get_logger().error("Python-Paket 'evdev' fehlt -- kein Controller.")
            return
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish)

    # ── Geraet finden / lesen ──────────────────────────────────────────
    @staticmethod
    def _vendor(dev):
        info = getattr(dev, "info", None)
        return getattr(info, "vendor", None)

    def _kind_of(self, dev):
        kind = L.controller_kind(dev.name, self._vendor(dev), self.wanted)
        if kind is None and dev.name == self.wanted:
            kind = "ps4"          # eigener Name per JOYSTICK_NAME: wie bisher PS4
        if self.kind_wanted != "auto" and kind != self.kind_wanted:
            return None
        return kind

    def _find(self):
        """-> (Geraet, 'ps4' | 'xbox') oder (None, None). Exakter Name
        (`joystick_name`) zuerst, sonst PS4 vor Xbox."""
        candidates = []
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
            except OSError:
                continue
            kind = self._kind_of(dev)
            if kind is None:
                dev.close()
                continue
            if dev.name == self.wanted:
                for extra, _ in candidates:
                    extra.close()
                return dev, kind
            candidates.append((dev, kind))
        candidates.sort(key=lambda c: 0 if c[1] == "ps4" else 1)
        for extra, _ in candidates[1:]:
            extra.close()
        return candidates[0] if candidates else (None, None)

    def _find_companions(self, dev):
        """Xbox ueber Bluetooth: weitere evdev-Teilgeraete desselben Controllers
        (z.B. "<Name> Consumer Control" mit der Xbox-Taste als KEY_HOMEPAGE).
        Gleicher Namensanfang und gleiche uniq/phys."""
        phys = getattr(dev, "phys", "") or ""
        uniq = getattr(dev, "uniq", "") or ""
        if not (phys or uniq):
            return []
        found = []
        for path in evdev.list_devices():
            if path == dev.path:
                continue
            try:
                c = evdev.InputDevice(path)
            except OSError:
                continue
            same = ((uniq and getattr(c, "uniq", "") == uniq)
                    or (phys and getattr(c, "phys", "") == phys))
            if c.name != dev.name and c.name.startswith(dev.name) and same:
                found.append(c)
            else:
                c.close()
        return found

    def _companion_reader(self, cdev, gen):
        try:
            for ev in cdev.read_loop():
                with self.lock:
                    if self._gen != gen or self._stop:
                        break
                    if ev.type == L.EV_KEY and self.state.apply(ev.type, ev.code, ev.value):
                        self._quiet_reports = 0
                        if self._alive_t is not None:
                            self._alive_t = time.monotonic()
        except OSError:
            pass
        try:
            cdev.close()
        except Exception:   # noqa: BLE001
            pass

    @staticmethod
    def _find_hidraw(dev):
        """/dev/hidrawN desselben HID-Geraets (nur HID-Treiber, also Bluetooth;
        xpad/USB hat keins). None, wenn keins da oder nicht lesbar."""
        ev = os.path.basename(dev.path)
        base = os.path.realpath(f"/sys/class/input/{ev}/device/device")
        for p in sorted(glob.glob(os.path.join(base, "hidraw", "hidraw*"))):
            node = "/dev/" + os.path.basename(p)
            if os.access(node, os.R_OK):
                return node
        return None

    def _hidraw_reader(self, path, gen):
        """Roh-Berichte zaehlen. Scharf (Lebenszeichen) erst nach
        `hidraw_arm_n` Berichten ohne evdev-Aenderung dazwischen."""
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            return
        try:
            while True:
                with self.lock:
                    if self._gen != gen or self._stop:
                        break
                r, _, _ = select.select([fd], [], [], 0.1)
                if not r:
                    continue
                try:
                    data = os.read(fd, 128)
                except BlockingIOError:
                    continue
                if not data:        # (nur Test-FIFO ohne Schreiber) -> abwarten
                    time.sleep(0.05)
                    continue
                with self.lock:
                    if self._gen != gen:
                        break
                    now = time.monotonic()
                    if self._alive_t is not None:
                        self._alive_t = now
                    else:
                        self._quiet_reports += 1
                        if self._quiet_reports >= self.hidraw_arm_n:
                            self._alive_t = now
                            self.get_logger().info(
                                "Xbox-Controller: HID-Strom laeuft durchgehend -- "
                                f"Lebendpruefung aktiv ({self.stall_s:.1f} s).")
        except OSError:
            pass
        finally:
            try:
                os.close(fd)
            except OSError:
                pass

    def _find_motion(self, dev):
        """Sensor-Teilgeraet desselben Controllers: Name + " Motion Sensors"
        und gleiche phys/uniq (hid-sony/hid-playstation setzen beide gleich).
        None, wenn nicht eindeutig zuzuordnen -> keine Lebendpruefung."""
        want = f"{dev.name} Motion Sensors"
        phys = getattr(dev, "phys", "") or ""
        uniq = getattr(dev, "uniq", "") or ""
        for path in evdev.list_devices():
            if path == dev.path:
                continue
            try:
                m = evdev.InputDevice(path)
            except OSError:
                continue
            if m.name == want and ((uniq and getattr(m, "uniq", "") == uniq)
                                   or (phys and getattr(m, "phys", "") == phys)):
                return m
            m.close()
        return None

    def _motion_reader(self, mdev, gen):
        try:
            for _ in mdev.read_loop():
                with self.lock:
                    if self._gen != gen or self._stop:
                        break
                    self._alive_t = time.monotonic()
        except OSError:
            pass
        try:
            mdev.close()
        except Exception:   # noqa: BLE001
            pass

    def _reader(self):
        missing_logged = False
        while not self._stop:
            dev, kind = self._find()
            if dev is None:
                if not missing_logged:
                    missing_logged = True
                    what = {"auto": "PS4- oder Xbox-Controller", "ps4": "PS4-Controller",
                            "xbox": "Xbox-Controller"}[self.kind_wanted]
                    self.get_logger().warn(
                        f"Kein {what} gefunden (PS4-Name: '{self.wanted}'). "
                        "Warte ... (Bluetooth koppeln oder per USB anstecken; im "
                        "Container braucht es /dev/input, siehe docs/43_ps4_controller.md)")
                time.sleep(self.reconnect_s)
                continue
            missing_logged = False
            absinfo = {}
            for code, info in dev.capabilities().get(ecodes.EV_ABS, []):
                absinfo[code] = (info.min, info.max)
            state = L.make_state(kind, absinfo, ps_button=self.xbox_ps,
                                 swap_xy=self.xbox_swap)
            # Startwerte (z.B. Trigger halb gedrueckt beim Verbinden)
            for code, info in dev.capabilities().get(ecodes.EV_ABS, []):
                state.apply(L.EV_ABS, code, info.value)
            motion = self._find_motion(dev) if kind == "ps4" else None
            companions = self._find_companions(dev) if kind == "xbox" else []
            hidraw = self._find_hidraw(dev) if kind == "xbox" else None
            with self.lock:
                self.state = state
                self.device = dev
                self.kind = kind
                self._gen += 1
                self._alive_t = None
                self._stalled = False
                self._quiet_reports = 0
                gen = self._gen
            for c in companions:
                threading.Thread(target=self._companion_reader, args=(c, gen),
                                 daemon=True).start()
            if motion is not None:
                threading.Thread(target=self._motion_reader, args=(motion, gen),
                                 daemon=True).start()
            elif hidraw is not None:
                threading.Thread(target=self._hidraw_reader, args=(hidraw, gen),
                                 daemon=True).start()
            elif kind == "ps4":
                self.get_logger().info(
                    "Kein Sensor-Teilgeraet zum Controller gefunden -- ohne "
                    "Lebendpruefung (Trennung erst, wenn der Kernel sie meldet).")
            if kind == "xbox" and hidraw is None:
                self.get_logger().info(
                    "Xbox-Controller ohne HID-Rohstrom (USB/xpad oder kein Zugriff) -- "
                    "Lebendpruefung nur ueber die Trennung durch den Kernel.")
            self._led = self._find_led(dev) if kind == "ps4" else None
            self._ff_effect = None
            label = "PS4" if kind == "ps4" else "Xbox"
            extra = ""
            if kind == "ps4" and not self._led:
                extra = " -- LED nicht steuerbar"
            elif kind == "xbox":
                extra = (f" -- NOT-HALT-Taste: {'View' if self.xbox_ps == 'view' else 'Xbox'}"
                         + (", Bluetooth ohne xpadneo" if state.generic_bt else ""))
            self.get_logger().info(f"{label}-Controller verbunden: {dev.name} ({dev.path}){extra}")
            if self._rgb is not None:
                self._set_led(self._rgb)
            try:
                for ev in dev.read_loop():
                    with self.lock:
                        if self.state.apply(ev.type, ev.code, ev.value):
                            self._quiet_reports = 0
                        if self._alive_t is not None:
                            self._alive_t = time.monotonic()
            except OSError as e:
                self.get_logger().warn(f"{label}-Controller getrennt ({e}). Warte auf Wiederverbindung ...")
            with self.lock:
                self.device = None
                self.kind = None
                self.state.reset()
                self._gen += 1           # Sensor-Thread dieser Verbindung beenden
                self._alive_t = None
            try:
                dev.close()
            except Exception:   # noqa: BLE001
                pass

    def _publish(self):
        with self.lock:
            if self.device is None:
                return
            stalled = (self._alive_t is not None
                       and time.monotonic() - self._alive_t > self.stall_s)
            if stalled != self._stalled:
                self._stalled = stalled
                if stalled:
                    self.get_logger().warn(
                        f"Controller: seit {self.stall_s:.1f} s keine Daten "
                        "(Funkverbindung haengt?) -- publiziere nichts, Hand haelt an.")
                else:
                    self.get_logger().info("Controller: Daten kommen wieder.")
            if stalled:
                return
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.kind or "ps4"
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
            if ms <= 0:
                # Stopp (z.B. Voll-Notaus-Halten abgebrochen). NIE Replay(0)
                # hochladen: Laenge 0 heisst beim Kernel "endlos".
                return
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
        if isinstance(r, dict) and evdev is not None:
            try:
                ms = int(r.get("ms", 100))
            except (TypeError, ValueError):
                return
            self._rumble(ms, r.get("strong", 0.0), r.get("weak", 0.5))

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

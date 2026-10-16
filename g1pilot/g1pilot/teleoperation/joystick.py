#!/usr/bin/env python3
"""
joystick — alter Controller-Node (G1_PS4_ARMS=0): evdev -> /g1pilot/joy_manual
-> joy_mux -> /g1pilot/joy -> loco_client (Gehen mit Deadman auf dem echten G1).

Belegung wie bisher: Achsen/Tasten in der Reihenfolge der evdev-Capabilities.

Sicherheit (2026-10-16): Frueher starb der Lese-Thread bei einer Trennung
still, und der Timer publizierte den LETZTEN Zustand weiter -- mit gedruecktem
Deadman und ausgelenktem Stick lief der echte G1 also weiter. Jetzt:
  * getrennt -> NICHTS publizieren (loco_client stoppt nach cmd_vel_timeout),
    dann automatisch wiederverbinden,
  * Funkverbindung haengt, ohne dass der Kernel trennt: das Teilgeraet
    "<Name> Motion Sensors" liefert mit jedem Funkbericht Ereignisse; bleiben
    sie laenger als stall_timeout_s aus, wird nichts publiziert (wie
    ps4_joystick).
"""
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import evdev


class ManualJoystick(Node):
    def __init__(self):
        super().__init__('manual_joystick')

        self.publisher = self.create_publisher(Joy, '/g1pilot/joy_manual', 10)
        self.auto_pub  = self.create_publisher(Bool, '/g1pilot/auto_enable', 10)

        self.declare_parameter("publish_rate", 50.0)
        self.rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self.declare_parameter("joystick_name", "Wireless Controller")
        self.joystick_name = self.get_parameter("joystick_name").get_parameter_value().string_value
        self.declare_parameter("reconnect_period_s", 1.0)
        self.declare_parameter("stall_timeout_s", 0.3)
        self.reconnect_s = float(self.get_parameter("reconnect_period_s").value)
        self.stall_s = float(self.get_parameter("stall_timeout_s").value)

        self.axes = []
        self.buttons = []
        self.lock = threading.Lock()
        self.device = None
        self._stop = False
        self._alive_t = None      # letztes Lebenszeichen; None = ohne Lebendpruefung
        self._gen = 0             # Verbindungs-Nummer (alte Sensor-Threads beenden)
        self._stalled = False

        self.auto_enabled = False
        self.triangle_prev = 0

        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

        self.create_timer(1.0 / self.rate, self.publish_joy)

    def find_joystick(self):
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
            except OSError:
                continue
            if dev.name == self.joystick_name:
                return dev
            dev.close()
        return None

    def _find_motion(self, dev):
        """Sensor-Teilgeraet desselben Controllers (gleiche uniq/phys) oder None."""
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

    def init_controls(self, device):
        abs_info = device.capabilities().get(evdev.ecodes.EV_ABS, [])
        key_info = device.capabilities().get(evdev.ecodes.EV_KEY, [])

        self.axes = [0.0] * len(abs_info)
        self.buttons = [0] * len(key_info)

        self.axis_map = {code: idx for idx, (code, _) in enumerate(abs_info)}
        self.button_map = {code: idx for idx, code in enumerate(key_info)}

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
            dev = self.find_joystick()
            if dev is None:
                if not missing_logged:
                    missing_logged = True
                    self.get_logger().error(
                        f"No joystick found! (gesucht: '{self.joystick_name}') -- warte ...")
                time.sleep(self.reconnect_s)
                continue
            missing_logged = False
            motion = self._find_motion(dev)
            with self.lock:
                self.init_controls(dev)
                self.device = dev
                self._gen += 1
                self._alive_t = None
                self._stalled = False
            if motion is not None:
                threading.Thread(target=self._motion_reader, args=(motion, self._gen),
                                 daemon=True).start()
            else:
                self.get_logger().warn(
                    "Kein Sensor-Teilgeraet gefunden -- ohne Lebendpruefung (eine "
                    "haengende Funkverbindung faellt erst beim Trennen auf).")
            self.get_logger().info(f'Using joystick: {dev.name} ({dev.path})')
            try:
                self.read_joystick(dev)
            except OSError as e:
                self.get_logger().warn(f"Joystick getrennt ({e}) -- publiziere nichts, "
                                       "warte auf Wiederverbindung ...")
            with self.lock:
                self.device = None
                self.axes = [0.0] * len(self.axes)
                self.buttons = [0] * len(self.buttons)
                self._gen += 1
                self._alive_t = None
            try:
                dev.close()
            except Exception:   # noqa: BLE001
                pass

    def read_joystick(self, device):
        BTN_TRIANGLE = evdev.ecodes.BTN_NORTH
        for event in device.read_loop():
            with self.lock:
                if self._alive_t is not None:
                    self._alive_t = time.monotonic()
                if event.type == evdev.ecodes.EV_ABS and event.code in self.axis_map:
                    idx = self.axis_map[event.code]
                    absinfo = device.absinfo(event.code)
                    val = event.value
                    if absinfo.max != absinfo.min:
                        val = (2 * (val - absinfo.min) / (absinfo.max - absinfo.min)) - 1
                    self.axes[idx] = float(val)

                elif event.type == evdev.ecodes.EV_KEY and event.code in self.button_map:
                    idx = self.button_map[event.code]
                    self.buttons[idx] = int(event.value)

                    if event.code == BTN_TRIANGLE:
                        if self.triangle_prev == 0 and event.value == 1:
                            self.auto_enabled = not self.auto_enabled
                            self.auto_pub.publish(Bool(data=self.auto_enabled))
                        self.triangle_prev = event.value

    def publish_joy(self):
        with self.lock:
            if self.device is None:
                return                  # getrennt: Strom bleibt aus -> Empfaenger stoppt
            stalled = (self._alive_t is not None
                       and time.monotonic() - self._alive_t > self.stall_s)
            if stalled != self._stalled:
                self._stalled = stalled
                if stalled:
                    self.get_logger().warn(
                        f"Joystick: seit {self.stall_s:.1f} s keine Daten (Funk haengt?) "
                        "-- publiziere nichts.")
                else:
                    self.get_logger().info("Joystick: Daten kommen wieder.")
            if stalled:
                return
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = self.axes.copy()
            msg.buttons = self.buttons.copy()
        self.publisher.publish(msg)

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManualJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

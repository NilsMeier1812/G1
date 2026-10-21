#!/usr/bin/env python3
"""Ende-zu-Ende-Test der Hueftdrehung (R2 halten + rechter Stick) in der Sim.

Simuliert den Controller auf /g1pilot/ps4/joy (wie test_ps4_arm_sim.py) und
misst ueber /g1pilot/waist/state (gemessener Taillen-Yaw), TF (Hand im Becken)
und /g1pilot/loco_state:

  1. R2 (GREIFEN), R2 halten + rechter Stick links: Huefte dreht, Hand dreht
     mit (Winkel im Becken aendert sich um den Hueftwinkel), Roboter steht.
  2. Lange nach rechts: Bereich (Grenze) haelt.
  3. R2 + R3: zurueck zur Mitte.
  4. Gedreht -> L2 (GEHEN): walk_ready erst, wenn die Huefte in der Mitte
     steht; danach WALK, kein Sturz. R2 -> wieder Stand.

Voraussetzung: Sim-Stack im Loco-Modus (run_sim_loco.sh) mit G1_PS4_ARMS=1.
Im Container:
    docker exec -it g1pilot-g1pilot-sim-1 bash -c \\
      "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/g1pilot/test_waist_yaw_sim.py"
Exit-Code 0 = alle Pruefungen bestanden.
"""
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from g1pilot.teleoperation import ps4_layout as L
from g1pilot.manipulation.waist_yaw import WAIST_STATE_TOPIC

RATE = 50.0


class Probe(Node):
    def __init__(self):
        super().__init__("waist_yaw_sim_probe")
        self.pub = self.create_publisher(Joy, "/g1pilot/ps4/joy", 10)
        self.pub_start = self.create_publisher(Bool, "/g1pilot/start", 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.waist = {}
        self.loco = None
        self.loco_hist = []
        self.walk_ready = []        # (t, gemessener Offset in Grad)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, WAIST_STATE_TOPIC, self._waist, 10)
        self.create_subscription(String, "/g1pilot/loco_state", self._loco, qos)
        self.create_subscription(Bool, "/g1pilot/arms/walk_ready", self._ready, 10)
        self.axes = [0.0] * L.NUM_AXES
        self.buttons = [0] * L.NUM_BUTTONS
        self.create_timer(1.0 / RATE, self._send)

    def _waist(self, m):
        try:
            self.waist = json.loads(m.data)
        except ValueError:
            pass

    def _loco(self, m):
        self.loco = m.data
        self.loco_hist.append(m.data)

    def _ready(self, m):
        if m.data:
            self.walk_ready.append((time.monotonic(), self.waist.get("meas_offset_deg")))

    def _send(self):
        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.axes = [float(a) for a in self.axes]
        m.buttons = [int(b) for b in self.buttons]
        self.pub.publish(m)

    def hand_angle(self, side="right"):
        """Winkel der Hand um die Hochachse im Becken [deg] (atan2 y/x)."""
        try:
            t = self.tf.lookup_transform("pelvis", f"{side}_hand_point_contact",
                                         rclpy.time.Time())
        except Exception:   # noqa: BLE001
            return None
        p = t.transform.translation
        return math.degrees(math.atan2(p.y, p.x))


def wait_for(cond, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.05)
    return False


def main():
    rclpy.init()
    pr = Probe()
    ex = SingleThreadedExecutor()
    ex.add_node(pr)
    threading.Thread(target=ex.spin, daemon=True).start()
    fails = []

    def check(ok, what):
        print(("OK   " if ok else "FAIL ") + what, flush=True)
        if not ok:
            fails.append(what)

    def press(btn, hold=0.2):
        pr.buttons[btn] = 1
        time.sleep(hold)
        pr.buttons[btn] = 0
        time.sleep(0.2)

    def off():
        return pr.waist.get("meas_offset_deg")

    try:
        check(wait_for(lambda: pr.waist.get("owner"), 60.0), "Zustand /g1pilot/waist/state kommt")
        pr.pub_start.publish(Bool(data=True))
        time.sleep(1.0)
        press(L.R2)                                       # GREIFEN: Stand + Arme
        check(wait_for(lambda: pr.loco == "stand", 30.0), f"Stand (loco_state={pr.loco})")
        check(wait_for(lambda: pr.waist.get("available"), 20.0),
              f"Hueftdrehung verfuegbar ({pr.waist.get('reason')})")
        time.sleep(2.0)
        a0, y0 = pr.hand_angle(), off()
        print(f"     Start: Huefte {y0:+.1f} deg, Hand {a0}")

        # 1. nach links drehen
        pr.axes[L.R2_AXIS] = 1.0
        time.sleep(0.3)
        pr.axes[L.RX] = 1.0
        time.sleep(1.3)
        pr.axes[L.RX] = 0.0
        time.sleep(0.2)
        pr.axes[L.R2_AXIS] = 0.0
        time.sleep(1.5)
        y1, a1 = off(), pr.hand_angle()
        print(f"     Links: Huefte {y1:+.1f} deg (Soll {pr.waist.get('offset_deg'):+.1f}), Hand {a1}")
        check(12.0 <= y1 <= 24.0, f"Huefte ~18 deg links ({y1:+.1f})")
        check(abs(y1 - pr.waist.get("offset_deg", 0.0)) < 2.0, "Messung folgt dem Sollwert")
        if a0 is not None and a1 is not None:
            check(abs((a1 - a0) - (y1 - y0)) < 4.0,
                  f"Hand dreht mit ({a1 - a0:+.1f} deg vs. Huefte {y1 - y0:+.1f})")
        check(pr.loco == "stand", f"steht weiter ({pr.loco})")
        y_hold = off()
        time.sleep(1.0)
        check(abs(off() - y_hold) < 1.0, "R2 los: Huefte bleibt stehen")

        # 2. Grenze
        pr.axes[L.R2_AXIS] = 1.0
        time.sleep(0.3)
        pr.axes[L.RX] = -1.0
        time.sleep(5.0)
        pr.axes[L.RX] = 0.0
        time.sleep(1.0)
        lim = pr.waist.get("limit_deg", 30.0)
        print(f"     Rechts: Huefte {off():+.1f} deg, Grenze {lim}")
        check(abs(off() + lim) < 2.5, f"Grenze -{lim} deg ({off():+.1f})")
        check(pr.loco == "stand", f"steht an der Grenze ({pr.loco})")

        # 3. R2 + R3 -> Mitte
        pr.buttons[L.R3] = 1
        time.sleep(0.2)
        pr.buttons[L.R3] = 0
        check(wait_for(lambda: abs(off()) < 2.0 and not pr.waist.get("centering"), 6.0),
              f"R2 + R3: Mitte ({off():+.1f})")
        pr.axes[L.R2_AXIS] = 0.0
        time.sleep(0.5)

        # 4. gedreht -> GEHEN: erst Mitte, dann walk_ready/WALK
        pr.axes[L.R2_AXIS] = 1.0
        time.sleep(0.3)
        pr.axes[L.RX] = 1.0
        time.sleep(1.2)
        pr.axes[L.RX] = 0.0
        pr.axes[L.R2_AXIS] = 0.0
        time.sleep(1.0)
        print(f"     vor GEHEN: Huefte {off():+.1f} deg")
        pr.walk_ready.clear()
        pr.loco_hist.clear()
        t_walk = time.monotonic()
        pr.axes[L.L2_AXIS] = 1.0                          # L2 druecken = GEHEN
        time.sleep(0.4)
        pr.axes[L.L2_AXIS] = 0.0
        check(wait_for(lambda: pr.walk_ready, 10.0), "walk_ready kommt")
        if pr.walk_ready:
            t_r, y_r = pr.walk_ready[0]
            print(f"     walk_ready nach {t_r - t_walk:.1f} s bei {y_r:+.1f} deg")
            check(y_r is not None and abs(y_r) < 3.5, "walk_ready erst in der Mitte")
            check(t_r - t_walk > 0.5, "walk_ready nicht sofort (Mitte-Fahrt)")
        check(wait_for(lambda: pr.loco == "walk", 10.0), f"WALK ({pr.loco})")
        time.sleep(2.0)
        check("damp" not in pr.loco_hist, "kein Sturz")
        press(L.R2)                                       # zurueck GREIFEN
        check(wait_for(lambda: pr.loco == "stand", 20.0), f"wieder Stand ({pr.loco})")
        check(abs(off()) < 3.0, f"Huefte in der Mitte ({off():+.1f})")
    finally:
        pr.axes = [0.0] * L.NUM_AXES
        pr.buttons = [0] * L.NUM_BUTTONS
        time.sleep(0.3)
        ex.shutdown()
        pr.destroy_node()
        rclpy.shutdown()
    print(f"\n{'BESTANDEN' if not fails else 'FEHLER: ' + str(len(fails))}")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()

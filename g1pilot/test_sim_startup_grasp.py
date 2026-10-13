#!/usr/bin/env python3
"""Ende-zu-Ende: Ist GREIFEN nach dem Sim-Start sofort bedienbar?

Bug 2026-10-13 (Nils): Beim Sim-Start schaltet die GUI automatisch auf GREIFEN,
oft bewegte sich dort aber nichts (GUI, RViz-Marker, PS4) -- erst GEHEN und
zurueck half. Ursache: der Auto-Start schickte ENABLE + BALANCING nur einmal,
3 s nach GUI-Start; luden arm_controller/loco_sim da noch, ging es verloren.

Dieser Test laeuft IM g1pilot-sim-Container direkt nach dem Stack-Start und
fasst nichts an ausser einem Hand-Ziel (wie der RViz-Marker):
  1. wartet (max. --wait s) auf den Auto-Start: arm_controller meldet Arme aktiv
     (/g1pilot/arms/state), loco_sim STAND (/g1pilot/loco_state). Aeltere Staende
     ohne diese Topics: nur Schritt 2 zaehlt.
  2. schickt ein Hand-Ziel 6 cm ueber die aktuelle rechte Hand und prueft per TF,
     dass die Hand sich mindestens 1.5 cm darauf zu bewegt (= Arme reagieren).
     Voll erreicht wird das Ziel aus der haengenden Startpose meist nicht: der
     IK bleibt dort 2-3 cm darunter stehen -- auch lange nach dem Start, also
     unabhaengig von diesem Fehler (bekannte IK-Restabweichung).
Exit 0 = GREIFEN funktioniert ohne Moduswechsel. Mehrere Starts: run_startup_grasp_loop.sh.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

FRAME = "pelvis"
HAND_TF = "right_hand_point_contact"


class Probe(Node):
    def __init__(self):
        super().__init__("startup_grasp_probe")
        q = QoSProfile(depth=1)
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.arms = None
        self.loco = None
        self.create_subscription(Bool, "/g1pilot/arms/state",
                                 lambda m: setattr(self, "arms", m.data), q)
        self.create_subscription(String, "/g1pilot/loco_state",
                                 lambda m: setattr(self, "loco", m.data), q)
        self.pub = self.create_publisher(PoseStamped, "/g1pilot/hand_goal/right", 1)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

    def hand(self, frame=FRAME, child=HAND_TF):
        try:
            t = self.tf.lookup_transform(frame, child, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        return t.transform

    def spin_for(self, s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=float, default=90.0, help="max. Wartezeit auf den Auto-Start")
    ap.add_argument("--settle", type=float, default=6.0,
                    help="nach dem Auto-Start-Zeitpunkt (GUI 3 s) zusaetzlich warten")
    ap.add_argument("--move-timeout", type=float, default=10.0)
    ap.add_argument("--min-move", type=float, default=0.015, help="Mindest-Hub [m]")
    ap.add_argument("--hand-tf", default=HAND_TF)
    args = ap.parse_args()

    rclpy.init()
    n = Probe()
    t0 = time.monotonic()
    # 1. Auto-Start abwarten
    while time.monotonic() - t0 < args.wait:
        n.spin_for(0.2)
        if n.arms is True and n.loco in ("stand", "walk"):
            break
    print(f"[probe] nach {time.monotonic() - t0:.1f} s: arms/state={n.arms} "
          f"loco_state={n.loco}", flush=True)
    n.spin_for(args.settle)

    tf = None
    while tf is None and time.monotonic() - t0 < args.wait + args.settle + 10:
        n.spin_for(0.2)
        tf = n.hand(child=args.hand_tf)
    if tf is None:
        print("[probe] FAIL: keine TF fuer die Hand", flush=True)
        return 2
    start = (tf.translation.x, tf.translation.y, tf.translation.z)
    goal = PoseStamped()
    goal.header.frame_id = FRAME
    goal.pose.position.x, goal.pose.position.y = start[0], start[1]
    goal.pose.position.z = start[2] + 0.06
    goal.pose.orientation = tf.rotation
    t1 = time.monotonic()
    best = 0.0
    while time.monotonic() - t1 < args.move_timeout:
        goal.header.stamp = n.get_clock().now().to_msg()
        n.pub.publish(goal)
        n.spin_for(0.2)
        cur = n.hand(child=args.hand_tf)
        if cur is not None:
            best = max(best, cur.translation.z - start[2])
            if best >= 0.04:
                break
    ok = best >= args.min_move
    print(f"[probe] Hand-Ziel +6 cm: Hand hob sich um {best * 100:.1f} cm in "
          f"{time.monotonic() - t1:.1f} s -> {'PASS' if ok else 'FAIL'} "
          f"(arms/state={n.arms}, loco_state={n.loco})", flush=True)
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

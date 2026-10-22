#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mola_fixed — MOLA-LiDAR-Odometrie -> Pose des G1-Beckens fuer den Nav-Stack.

MOLA schaetzt die Bewegung des LiDAR (Livox MID-360 im Kopf) mit fester
Sensorpose = Ursprung (mola_launcher: ignore_lidar_pose_from_tf). Daraus wird:

  1. Spiegelung an der x-Achse (mirror_y, Default an): y -> -y, Gier/Roll
     -> -Gier/-Roll. So hatte es das Original-g1pilot am echten G1 (README-GIF);
     der MID-360 sitzt im G1 kopfueber. Bei der Inbetriebnahme pruefen:
     1 m vor -> x +1, nach links drehen -> Gier steigt (docs/50, Testplan).
  2. Hueftdrehung heraus (waist_compensation, Default an): der LiDAR sitzt am
     OBERKOERPER, der dreht mit waist_yaw_joint gegen das Becken (Hueftdrehung
     per Controller R2). Becken-Gier = Oberkoerper-Gier - waist_yaw.
     Der kleine Hebelarm (LiDAR ~0,4 m ueber der Hueftachse) bleibt drin.

Ausgabe wie in der Sim (sim_localization), damit planner/nav2point/
nav_supervisor/RViz identisch arbeiten:
  /lidar_odometry/pose_fixed  nav_msgs/Odometry, frame 'map', child 'base_link'
  TF odom_unitree -> base_link (map -> world -> odom_unitree sind statisch
  Identitaet, bringup_real). Frueher: TF map -> pelvis, pelvis hatte damit
  zwei Eltern (base_link -> pelvis statisch) -> RViz/TF kaputt.
"""
import math
import time
from typing import Tuple

WAIST_JOINT = "waist_yaw_joint"


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def quat_to_euler(q) -> Tuple[float, float, float]:
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (0.0, 0.0, 0.0, 1.0) if n == 0.0 else (x / n, y / n, z / n, w / n)


def fix_pose(pos, quat, waist_yaw=0.0, mirror_y=True):
    """MOLA-Sensorpose -> Beckenpose. -> ((x, y, z), (qx, qy, qz, qw))."""
    x, y, z = pos
    roll, pitch, yaw = quat_to_euler(quat_normalize(quat))
    if mirror_y:
        y, roll, yaw = -y, -roll, -yaw
    yaw = math.atan2(math.sin(yaw - waist_yaw), math.cos(yaw - waist_yaw))
    return (x, y, z), euler_to_quat(roll, pitch, yaw)


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Quaternion, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from tf2_ros import TransformBroadcaster

    class FixMolaOdometry(Node):
        def __init__(self):
            super().__init__('fix_mola_odometry')
            self.declare_parameter('in_topic', '/lidar_odometry/pose')
            self.declare_parameter('out_topic', '/lidar_odometry/pose_fixed')
            self.declare_parameter('map_frame', 'map')
            self.declare_parameter('odom_frame', 'odom_unitree')
            self.declare_parameter('base_frame', 'base_link')
            self.declare_parameter('mirror_y', True)
            self.declare_parameter('waist_compensation', True)
            self.declare_parameter('joint_timeout_s', 0.5)
            gp = lambda n: self.get_parameter(n).value   # noqa: E731
            self.map_frame = gp('map_frame')
            self.odom_frame = gp('odom_frame')
            self.base_frame = gp('base_frame')
            self.mirror_y = bool(gp('mirror_y'))
            self.waist_comp = bool(gp('waist_compensation'))
            self.joint_timeout = float(gp('joint_timeout_s'))
            self.waist_yaw, self.t_waist = 0.0, None
            self._warned_waist = False

            self.tf_broadcaster = TransformBroadcaster(self)
            self.create_subscription(Odometry, gp('in_topic'), self.cb_odom, 10)
            if self.waist_comp:
                self.create_subscription(JointState, '/joint_states', self.cb_joints, 10)
            self.pub = self.create_publisher(Odometry, gp('out_topic'), 10)
            self.get_logger().info(
                f"MOLA {gp('in_topic')} -> {gp('out_topic')} ({self.map_frame}), TF "
                f"{self.odom_frame} -> {self.base_frame}; Spiegelung y: {self.mirror_y}, "
                f"Hueftdrehung herausrechnen: {self.waist_comp}")

        def cb_joints(self, msg):
            try:
                i = list(msg.name).index(WAIST_JOINT)
            except ValueError:
                return
            if i < len(msg.position):
                self.waist_yaw, self.t_waist = float(msg.position[i]), time.monotonic()

        def _waist(self):
            if not self.waist_comp:
                return 0.0
            if self.t_waist is None or time.monotonic() - self.t_waist > self.joint_timeout:
                if not self._warned_waist:
                    self.get_logger().warn(
                        "Keine aktuelle Hueftdrehung (/joint_states waist_yaw_joint) -- "
                        "rechne mit 0. Huefte nicht drehen, solange AUTO NAV laeuft.")
                    self._warned_waist = True
                return 0.0
            self._warned_waist = False
            return self.waist_yaw

        def cb_odom(self, msg):
            p, o = msg.pose.pose.position, msg.pose.pose.orientation
            (x, y, z), q = fix_pose((p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
                                    self._waist(), self.mirror_y)
            out = Odometry()
            out.header = msg.header
            out.header.frame_id = self.map_frame
            out.child_frame_id = self.base_frame
            out.pose.pose.position.x, out.pose.pose.position.y = x, y
            out.pose.pose.position.z = z
            out.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            out.pose.covariance = msg.pose.covariance
            out.twist = msg.twist
            self.pub.publish(out)

            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.translation.z = z
            t.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    rclpy.init()
    node = FixMolaOdometry()
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

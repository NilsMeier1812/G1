#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MOLA-Kette mit kuenstlichem LiDAR (Stufe 2), ohne Roboter.

Ein MID-360 KOPFUEBER (wie im G1, Grund fuer die Spiegelung in mola_fixed)
faehrt durch einen erfundenen Raum: 2 m vor, 90 Grad links drehen, 1 m
weiter (nach links), dann dreht nur die Huefte (Becken steht). Durch den echten MOLA (mola_launcher)
und mola_fixed muss /lidar_odometry/pose_fixed das Becken richtig zeigen.

Braucht das LiDAR-Image (mola_lidar_odometry), sonst uebersprungen. ~60 s.
"""
import math
import os
import struct
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
np = pytest.importorskip("numpy")
try:
    from ament_index_python.packages import get_package_share_directory
    get_package_share_directory("mola_lidar_odometry")
    HAVE_MOLA = True
except Exception:                                    # noqa: BLE001
    HAVE_MOLA = False
# Marker statt Modul-Skip: pytest 6 (apt) bricht sonst die GANZE Sammlung ab.
pytestmark = pytest.mark.skipif(not HAVE_MOLA,
                                reason="mola_lidar_odometry fehlt (nur im Image g1pilot-real-lidar)")

import rclpy.executors                                    # noqa: E402
from nav_msgs.msg import Odometry                         # noqa: E402
from sensor_msgs.msg import JointState, PointCloud2, PointField   # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SENSOR_Z = 1.2
RATE = 10.0


def room_points(rng):
    """Raum 9 x 7 x 3 m mit Pfeilern, Kisten und Schraege (Struktur fuer ICP)."""
    pts = []
    step = 0.08

    def plane(origin, u, v, nu, nv):
        a = np.arange(0, nu, step)
        b = np.arange(0, nv, step)
        A, B = np.meshgrid(a, b)
        o, u, v = np.array(origin), np.array(u), np.array(v)
        pts.append(o + A.reshape(-1, 1) * u + B.reshape(-1, 1) * v)

    X0, X1, Y0, Y1, H = -3.0, 6.0, -3.5, 3.5, 3.0
    plane((X0, Y0, 0), (1, 0, 0), (0, 1, 0), X1 - X0, Y1 - Y0)        # Boden
    plane((X0, Y0, H), (1, 0, 0), (0, 1, 0), X1 - X0, Y1 - Y0)        # Decke
    plane((X0, Y0, 0), (1, 0, 0), (0, 0, 1), X1 - X0, H)              # Waende
    plane((X0, Y1, 0), (1, 0, 0), (0, 0, 1), X1 - X0, H)
    plane((X0, Y0, 0), (0, 1, 0), (0, 0, 1), Y1 - Y0, H)
    plane((X1, Y0, 0), (0, 1, 0), (0, 0, 1), Y1 - Y0, H)
    for cx, cy, s in ((1.5, 1.8, 0.5), (4.0, -2.0, 0.7), (-1.5, -1.0, 0.4),
                      (3.0, 2.5, 0.3), (0.5, -2.6, 0.6)):
        for (ox, oy, ux, uy) in ((cx, cy, 1, 0), (cx, cy, 0, 1),
                                 (cx + s, cy, 0, 1), (cx, cy + s, 1, 0)):
            plane((ox, oy, 0), (ux, uy, 0), (0, 0, 1), s, H * 0.7)
    plane((5.0, -1.0, 0), (0.7, 0, 0.7), (0, 1, 0), 1.2, 2.0)        # Schraege
    P = np.concatenate(pts)
    return P + rng.normal(0, 0.005, P.shape)


def sensor_cloud(P, x, y, yaw, rng):
    """Weltpunkte -> Sensorrahmen eines kopfueber montierten LiDAR."""
    d = P - np.array([x, y, SENSOR_Z])
    c, s = math.cos(yaw), math.sin(yaw)
    lx = c * d[:, 0] + s * d[:, 1]
    ly = -s * d[:, 0] + c * d[:, 1]
    lz = d[:, 2]
    q = np.stack([lx, -ly, -lz], axis=1)                 # Roll 180 Grad (kopfueber)
    r = np.linalg.norm(q, axis=1)
    elev = np.degrees(np.arcsin(np.clip(-q[:, 2] / np.maximum(r, 1e-6), -1, 1)))
    keep = (r > 0.3) & (r < 25.0) & (elev > -7) & (elev < 52)   # MID-360-Sichtfeld (oben = -z)
    q = q[keep]
    sel = rng.choice(len(q), size=min(len(q), 20000), replace=False)
    return q[sel].astype(np.float32)


def to_msg(points, stamp):
    m = PointCloud2()
    m.header.stamp = stamp
    m.header.frame_id = "livox_frame"
    m.height, m.width = 1, len(points)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(("x", "y", "z", "intensity"))]
    m.is_bigendian = False
    m.point_step = 16
    m.row_step = 16 * len(points)
    data = np.zeros((len(points), 4), np.float32)
    data[:, :3] = points
    data[:, 3] = 50.0
    m.data = data.tobytes()
    m.is_dense = True
    return m


def trajectory(t):
    """-> (x, y, Becken-Gier, Huefte) zur Zeit t."""
    if t < 3.0:
        return 0.0, 0.0, 0.0, 0.0
    if t < 13.0:
        return 0.2 * (t - 3.0), 0.0, 0.0, 0.0
    if t < 18.0:
        return 2.0, 0.0, (math.pi / 2) * (t - 13.0) / 5.0, 0.0
    if t < 23.0:                                   # nach links (Welt +y) laufen
        return 2.0, 0.2 * (t - 18.0), math.pi / 2, 0.0
    if t < 26.0:                                   # nur die Huefte dreht
        return 2.0, 1.0, math.pi / 2, 0.35 * (t - 23.0) / 3.0
    return 2.0, 1.0, math.pi / 2, 0.35


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


@pytest.fixture
def stack():
    env = dict(os.environ)
    env["PYTHONPATH"] = REPO + os.pathsep + env.get("PYTHONPATH", "")
    procs = [
        subprocess.Popen(["ros2", "launch", os.path.join(REPO, "launch", "mola_launcher.launch.py")],
                         env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                         start_new_session=True),
        subprocess.Popen([sys.executable, "-m", "g1pilot.navigation.fix_mola_odometry"],
                         cwd=REPO, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, start_new_session=True),
    ]
    rclpy.init()
    node = rclpy.create_node("mola_synthetic_probe")
    yield node
    node.destroy_node()
    rclpy.shutdown()
    for p in procs:
        try:
            os.killpg(p.pid, 2)
        except ProcessLookupError:
            pass
    for p in procs:
        try:
            out, _ = p.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, 9)
            out, _ = p.communicate()
        print(out[-4000:])


def test_mola_chain_tracks_pelvis(stack):
    node = stack
    from rclpy.qos import qos_profile_sensor_data
    pub = node.create_publisher(PointCloud2, "/livox/lidar", qos_profile_sensor_data)
    pub_js = node.create_publisher(JointState, "/joint_states", 10)
    got = {"raw": [], "fixed": []}
    node.create_subscription(Odometry, "/lidar_odometry/pose",
                             lambda m: got["raw"].append(m), 50)
    node.create_subscription(Odometry, "/lidar_odometry/pose_fixed",
                             lambda m: got["fixed"].append((time.monotonic(), m)), 50)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        _drive_and_check(node, pub, pub_js, got)
    finally:
        ex.shutdown()
        th.join(timeout=2.0)


def _drive_and_check(node, pub, pub_js, got):

    rng = np.random.default_rng(1)
    P = room_points(rng)
    time.sleep(8.0)                      # MOLA laedt
    truth = []
    t0 = time.monotonic()
    t = 0.0
    while t < 29.0:
        x, y, yaw, waist = trajectory(t)
        stamp = node.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = ["waist_yaw_joint"]
        js.position = [waist]
        pub_js.publish(js)
        pub.publish(to_msg(sensor_cloud(P, x, y, yaw + waist, rng), stamp))
        truth.append((time.monotonic(), x, y, yaw))
        t += 1.0 / RATE
        time.sleep(max(0.0, t0 + t - time.monotonic()))
    time.sleep(2.0)

    assert got["raw"], "MOLA publiziert nichts auf /lidar_odometry/pose"
    span = got["fixed"][-1][0] - got["fixed"][0][0] if len(got["fixed"]) > 1 else 0.0
    rate = (len(got["fixed"]) - 1) / span if span > 0 else 0.0
    print(f"Posen: {len(got['fixed'])} von {len(truth)} Scans, {rate:.1f} Hz "
          f"(Testrechner; MOLA laesst Scans aus, wenn die CPU nicht reicht)")
    assert len(got["fixed"]) >= 30, (len(got["fixed"]), len(truth))

    def at(tq):
        best = min(got["fixed"], key=lambda e: abs(e[0] - tq))
        p = best[1].pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def check(i, tol_xy, tol_deg):
        tq, x, y, yaw = truth[i]
        ex_, ey, eyaw = at(tq + 0.15)
        dyaw = math.degrees(math.atan2(math.sin(eyaw - yaw), math.cos(eyaw - yaw)))
        msg = f"t={i / RATE:.1f}s soll ({x:.2f},{y:.2f},{math.degrees(yaw):.0f}) " \
              f"ist ({ex_:.2f},{ey:.2f},{math.degrees(eyaw):.0f})"
        print(msg)
        assert abs(ex_ - x) < tol_xy and abs(ey - y) < tol_xy and abs(dyaw) < tol_deg, msg

    check(int(8.0 * RATE), 0.10, 3)      # halbe Strecke vor
    check(int(13.0 * RATE), 0.10, 3)     # 2 m vor
    check(int(16.0 * RATE), 0.15, 5)     # beim Drehen: links = positive Gier
    check(int(23.0 * RATE), 0.15, 4)     # 90 Grad links, 1 m gelaufen -> y = +1
    check(len(truth) - 2, 0.15, 4)       # Huefte 20 Grad gedreht, Becken unveraendert

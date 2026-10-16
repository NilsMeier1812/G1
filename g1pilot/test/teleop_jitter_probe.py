#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Messsonde: Befehlsstrom des arm_controller (REAL-Pfad) bei Jog-Teleop vs.
geplanter Fahrt -- ohne Hardware, gegen einen Fake-G1 mit Gelenkdynamik.

Laeuft wie test_arm_controller_real.py (Unitree-SDK-Attrappen), aber mit dem
echten ROS-Timer des Nodes (250 Hz, eigener Executor-Thread) und einem
1-kHz-Gelenkmodell je Armgelenk:  I*q'' = kp*(q_cmd - q) - kd*q'
(Motor haelt das letzte Kommando, wie der echte Treiber). Die Taille kann mit
einer kleinen Stoerung belegt werden (der echte G1 balanciert/schwingt, die
Encoder rauschen) -- im Sim-Modus gibt es das nicht.

Aufruf (im Test-Container, aus g1pilot/, PYTHONPATH=.):
  python3 test/teleop_jitter_probe.py [--waist-noise 0.002] [--enc-noise 0.0003]
  Vergleich alt:  --param ik_async=false --param ik_servo_step_tol=0.0

Ausgabe je Phase (JOG, HALTEN, MARKER, GEPLANT): Senderate, Abstaende der
Writes, groesster Sollwertsprung, hochfrequente Gelenkbeschleunigung im Modell
(acc_hf_rms, Mass fuer Vibration). Gelenkmodell und Traegheit sind grobe
Annahmen -- die Zahlen taugen zum Vergleich, nicht als Absolutwerte.
"""
import argparse
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_arm_controller_real as T          # noqa: E402  (Stubs + FakeG1)

I_ARM = 0.03            # kg m^2, grobe effektive Traegheit je Armgelenk (Annahme)


class Plant:
    """1-kHz-Gelenkmodell hinter dem Fake-DDS. Ersetzt FakeG1.write."""

    def __init__(self, waist_noise=0.0, waist_hz=6.0, enc_noise=0.0):
        self.q = T.ONBOARD.copy()
        self.dq = np.zeros(14)
        self.cmd = T.ONBOARD.copy()
        self.kp = np.full(14, 60.0)
        self.kd = np.full(14, 1.5)
        self.w = 0.0
        self.lock = threading.Lock()
        self.log = []           # (t, q_cmd14) je Write
        self.alog = []          # (t, q14, ddq14) Plant 1 kHz
        self.waist_noise = waist_noise
        self.waist_hz = waist_hz
        self.enc_noise = enc_noise
        self.rng = np.random.default_rng(0)
        self.run = True
        self.t0 = time.monotonic()
        threading.Thread(target=self._loop, daemon=True).start()

    def write(self, msg):
        with self.lock:
            self.w = float(msg.motor_cmd[29].q)
            self.cmd = np.array([msg.motor_cmd[j].q for j in T.ARM_IDX])
            self.kp = np.array([msg.motor_cmd[j].kp for j in T.ARM_IDX])
            self.kd = np.array([msg.motor_cmd[j].kd for j in T.ARM_IDX])
            self.log.append((time.monotonic(), self.cmd.copy()))

    def _loop(self):
        dt = 0.001
        nxt = time.monotonic()
        while self.run:
            with self.lock:
                soll = self.w * self.cmd + (1.0 - self.w) * T.ONBOARD
                kp = self.w * self.kp + (1.0 - self.w) * 60.0
                kd = self.w * self.kd + (1.0 - self.w) * 1.5
                ddq = (kp * (soll - self.q) - kd * self.dq) / I_ARM
                self.dq += ddq * dt
                self.q += self.dq * dt
                t = time.monotonic() - self.t0
                self.alog.append((t, self.q.copy(), ddq.copy()))
                meas = self.q + (self.rng.normal(0, self.enc_noise, 14)
                                 if self.enc_noise > 0 else 0.0)
                for k, j in enumerate(T.ARM_IDX):
                    T.ROBOT.state.motor_state[j].q = float(meas[k])
                wn = 0.0
                if self.waist_noise > 0:
                    wn = (self.waist_noise * math.sin(2 * math.pi * self.waist_hz * t)
                          + self.rng.normal(0, self.waist_noise * 0.3))
                for k, j in enumerate(T.WAIST_IDX):
                    T.ROBOT.state.motor_state[j].q = float(T.ONBOARD_WAIST[k] + wn)
            nxt += dt
            time.sleep(max(0.0, nxt - time.monotonic()))


def metrics(plant, t_a, t_b, label):
    """Kennzahlen im Fenster [t_a, t_b] (monotonic)."""
    wr = [(t, q) for t, q in plant.log if t_a <= t <= t_b]
    if len(wr) < 3:
        print(f"{label}: zu wenige Writes ({len(wr)})")
        return {}
    ts = np.array([t for t, _ in wr])
    qs = np.array([q for _, q in wr])
    dts = np.diff(ts)
    steps = np.abs(np.diff(qs, axis=0)).max(axis=1)
    moving = steps > 1e-6
    # Befehls-"Geschwindigkeit" je Write und ihr Wechsel (Ruckeln im Sollwert)
    v = np.diff(qs, axis=0) / dts[:, None]
    dv = np.abs(np.diff(v, axis=0)).max(axis=1)
    # Plant: Gelenkbeschleunigung (Vibration) -- hochpass: Abweichung vom
    # gleitenden Mittel ueber 50 ms
    pa = [(t, a) for t, _, a in plant.alog if t_a - plant.t0 <= t <= t_b - plant.t0]
    acc = np.array([a for _, a in pa])
    k = 50
    if len(acc) > 2 * k:
        kern = np.ones(k) / k
        smooth = np.array([np.convolve(acc[:, j], kern, mode="same") for j in range(14)]).T
        hf = acc[k:-k] - smooth[k:-k]
        acc_hf = float(np.sqrt(np.mean(hf ** 2, axis=0)).max())
    else:
        acc_hf = float("nan")
    out = {
        "writes_hz": len(wr) / (ts[-1] - ts[0]),
        "dt_p50_ms": 1e3 * float(np.median(dts)),
        "dt_p95_ms": 1e3 * float(np.percentile(dts, 95)),
        "dt_max_ms": 1e3 * float(dts.max()),
        "step_max_mrad": 1e3 * float(steps.max()),
        "stand_ticks_pct": 100.0 * float(np.mean(~moving)),
        "cmd_dv_p95": float(np.percentile(dv, 95)),
        "acc_hf_rms": acc_hf,
    }
    print(f"{label}: " + ", ".join(f"{k}={v:.2f}" for k, v in out.items()), flush=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--waist-noise", type=float, default=0.0)
    ap.add_argument("--enc-noise", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--speed", type=float, default=0.05)     # m/s (Langsam)
    ap.add_argument("--jog-rate", type=float, default=30.0)
    ap.add_argument("--param", action="append", default=[])
    a = ap.parse_args()

    import tempfile
    os.environ["G1_SIM_MODE"] = "false"
    os.environ["G1_POSE_STORE"] = os.path.join(tempfile.mkdtemp(), "poses.json")
    os.environ["ROS_PACKAGE_PATH"] = os.path.dirname(T.PKG)
    T._install_unitree_stubs()
    import ament_index_python.packages as aip
    aip.get_package_share_directory = lambda name: T.PKG
    from g1pilot.manipulation import arm_controller as ac
    ac.init_dds = lambda *x, **k: None
    import rclpy
    import pinocchio as pin
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String
    from g1pilot.teleoperation.hand_jog import jog_step, lead_limits

    plant = Plant(a.waist_noise, enc_noise=a.enc_noise)
    T.ROBOT.write = plant.write
    extra = {}
    for p in a.param:
        k, v = p.split("=", 1)
        extra[k] = v
    base = {"kp_low": 60.0, "kd_low": 1.5, "kp_wrist": 40.0, "kd_wrist": 1.5,
            "arm_accel_limit": 3.0, "max_tracking_error": 0.3}
    base.update(extra)
    args = ["--ros-args"] + sum((["-p", f"{k}:={v}"] for k, v in base.items()), [])
    rclpy.init(args=args)
    node = ac.ArmController()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    T._wait_planner(node)
    t_w = time.monotonic()
    while node._ik_proc is not None and not node._ik_proc.ready and time.monotonic() - t_w < 120:
        time.sleep(0.1)
    prof = {}

    def wrap(obj, name):
        f = getattr(obj, name)

        def w(*x, **k):
            t = time.perf_counter()
            try:
                return f(*x, **k)
            finally:
                e = prof.setdefault(name, [0, 0.0])
                e[0] += 1
                e[1] += time.perf_counter() - t
        setattr(obj, name, w)
    for n in ("main_loop", "_apply_collision_gate", "_limit_cartesian_speed",
              "_publish_hand_cmd_poses", "_update_marker_goals", "_write_arm_cmd",
              "_publish_workspace", "_right_goal_callback"):
        wrap(node, n)
    wrap(node.ik_solver, "get_joint_targets")
    iters = []
    f_sol = node.ik_solver._solve_to_goal

    def sol(*x, **k):
        r = f_sol(*x, **k)
        iters.append(node.ik_solver.last_iters)
        return r
    node.ik_solver._solve_to_goal = sol
    main._iters = iters
    node.timer.callback = node.main_loop

    node._arms_controlled_callback(Bool(data=True))
    time.sleep(3.0)

    # Hand-Pose aus der Messung (wie TF): eigenes pin.Data
    ik = node.ik_solver
    data = pin.Data(ik.model)

    def hand_meas():
        cur = node.get_current_motor_q()
        q = pin.neutral(ik.model)
        for jid_idx, name in enumerate(ik._ros_joint_names):
            if name in ik._name_to_q_index:
                q[ik._name_to_q_index[name]] = float(cur[jid_idx])
        pin.forwardKinematics(ik.model, data, q)
        pin.updateFramePlacements(data=data, model=ik.model)
        M = data.oMf[ik._fid_right]
        qq = pin.Quaternion(M.rotation)
        return list(M.translation), (qq.x, qq.y, qq.z, qq.w)

    # ── Jog: rechte Hand nach vorn/oben, wie ps4_arm_teleop (jog_reference tf)
    lin = (a.speed * 0.7, 0.0, a.speed * 0.7)
    lead_m, lead_rad = lead_limits(a.speed, math.radians(20))
    target = None
    t_a = time.monotonic()
    tick_ms = []
    while time.monotonic() - t_a < a.seconds:
        t = time.monotonic()
        target = jog_step(target, hand_meas(), lin, (0, 0, 0), 1.0 / a.jog_rate,
                          lead_m, lead_rad)
        msg = PoseStamped()
        msg.header.frame_id = "pelvis"
        msg.header.stamp = node.get_clock().now().to_msg()
        p, q = target
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = map(float, q)
        pub_t = time.monotonic()
        # Zustellung ueber das Topic (Callback laeuft im Executor-Thread)
        if not hasattr(main, "_pub"):
            main._pub = node.create_publisher(PoseStamped, "/g1pilot/hand_goal/right", 1)
        main._pub.publish(msg)
        tick_ms.append(1e3 * (time.monotonic() - pub_t))
        time.sleep(max(0.0, 1.0 / a.jog_rate - (time.monotonic() - t)))
    t_b = time.monotonic()
    hp_end = hand_meas()[0]
    print(f"JOG    : Hand {1e3 * np.linalg.norm(np.array(hp_end) - np.array(target[0])):.1f} mm "
          f"hinter dem Jog-Ziel am Ende", flush=True)
    m_jog = metrics(plant, t_a + 0.5, t_b, "JOG    ")
    for k, (n, tt) in sorted(prof.items(), key=lambda kv: -kv[1][1]):
        print(f"  {k:28s} n={n:5d}  {1e3 * tt / max(n, 1):7.2f} ms/Aufruf", flush=True)
    prof.clear()
    if main._iters:
        it = np.array(main._iters)
        print(f"  IK-Iterationen: median {np.median(it):.0f}, p95 {np.percentile(it, 95):.0f}, max {it.max()}", flush=True)
        main._iters.clear()
    time.sleep(1.5)
    t_h = time.monotonic()
    time.sleep(2.0)
    m_hold = metrics(plant, t_h, time.monotonic(), "HALTEN ")

    # ── Marker-Sprung: ein Ziel 6 cm vor/2 cm hoch, wie ein RViz-Marker-Zug
    hp, hq = hand_meas()
    goal_p = [hp[0] + 0.06, hp[1], hp[2] + 0.02]
    msg = PoseStamped()
    msg.header.frame_id = "pelvis"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, goal_p)
    o = msg.pose.orientation
    o.x, o.y, o.z, o.w = map(float, hq)
    main._pub.publish(msg)
    t_m = time.monotonic()
    reach_t = None
    while time.monotonic() - t_m < 4.0:
        e = float(np.linalg.norm(np.array(hand_meas()[0]) - np.array(goal_p)))
        if reach_t is None and e < 0.01:
            reach_t = time.monotonic() - t_m
        time.sleep(0.02)
    e_end = float(np.linalg.norm(np.array(hand_meas()[0]) - np.array(goal_p)))
    metrics(plant, t_m, t_m + 1.5, "MARKER ")
    print(f"MARKER : Rest {1e3 * e_end:.1f} mm nach 4 s, <1 cm nach "
          f"{reach_t if reach_t is not None else float('nan'):.2f} s", flush=True)
    prof.clear()

    # ── Geplante Fahrt (wie Positionsspeicher/Home): Gelenkziel
    cur = node._last_q_target.copy()
    goal = cur.copy()
    goal[7:14] += np.array([-0.35, 0.0, 0.0, -0.25, 0.0, 0.0, 0.0])
    goal_l = {"left": list(map(float, cur[0:7])), "right": list(map(float, goal[7:14]))}
    import json
    if not hasattr(main, "_pubc"):
        main._pubc = node.create_publisher(String, "/g1pilot/arm_command", 10)
    main._pubc.publish(String(data=json.dumps({"id": "probe", "type": "joints", **goal_l})))
    t0 = time.monotonic()
    while not node._planned_motion_active and time.monotonic() - t0 < 20:
        time.sleep(0.01)
    t_p = time.monotonic()
    while node._planned_motion_active and time.monotonic() - t_p < 20:
        time.sleep(0.01)
    m_plan = metrics(plant, t_p + 0.3, time.monotonic() - 0.3, "GEPLANT")
    for k, (n, tt) in sorted(prof.items(), key=lambda kv: -kv[1][1]):
        print(f"  {k:28s} n={n:5d}  {1e3 * tt / max(n, 1):7.2f} ms/Aufruf", flush=True)
    plant.run = False
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return m_jog, m_hold, m_plan


if __name__ == "__main__":
    main()

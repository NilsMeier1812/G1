#!/usr/bin/env python3
"""Headless-Test der Lauf-Policy + Stand-Balancer in der MuJoCo-Szene (ohne ROS/DDS).

Bildet die echte Sim nach, benutzt aber DENSELBEN Regler-Code wie loco_sim:
  * g1pilot.navigation.walk_policy    (WALK: Obs, History, Gelenk-Zuordnung, Gains)
  * g1pilot.navigation.stand_balancer (STAND: PD-Balancer, Schwerpunkt-Fuehrung,
                                       Kipp-Erkennung, WALK->STAND-Uebergabe)
  * g1pilot.navigation.com_model      (Schwerpunkt aus den Gelenkwinkeln)
Sim wie die Bridge: dt = 1 ms, Aufstellen in die Stand-Pose der Policy (Becken-
Hoehe so, dass die Fuesse auf dem Boden stehen, Weld aus), PD je Sim-Schritt mit
Aktuator-ctrlrange-Clamping; Regeltakt 50 Hz im Lockstep (Befehl aus dem Zustand
am Anfang jedes 20-ms-Blocks, wie SIM_LOCKSTEP=1). Stoesse wie push_listener.py
(Kraft auf torso_link). Lasten in den Haenden kennt das Schwerpunkt-Modell nicht
(wie in der echten Sim).
Die Arme emuliert ein arm_controller-Ersatz: PD (kp/kd wie arm_controller) +
Schwerkraft-Feedforward auf eine Soll-Armpose je Szenario, mit dessen Gelenk-
Speedlimit (arm_velocity_limit 1.5 rad/s). Der Reset setzt die Arme wie die Bridge
in die Lauf-Pose; von dort fahren sie mit dem Speedlimit in die Szenario-Pose.

Lauf:  python3 test_agile_walk_sim.py [--policy agile_velocity_g1] [--rescue-policy NAME]
                                      [--inspire] [--quick]
Exit-Code 0 = alle Pflicht-Szenarien bestanden.
"""
import argparse
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from g1pilot.navigation.walk_policy import get_gravity_orientation, load_walk_policy  # noqa: E402
from g1pilot.navigation.stand_balancer import (  # noqa: E402
    BalancerParams, SettleGate, SettleParams, StandBalancer, TipDetector, TipParams,
    LEG_IDX, LEG_KP, LEG_KD, LEG_STAND_POSE, WAIST_IDX, WAIST_KP, WAIST_KD, WAIST_TARGET)
from g1pilot.navigation.com_model import ComModel  # noqa: E402

G1_DIR = os.path.join(ROOT, "unitree_mujoco/unitree_robots/g1")
SIM_DT = 0.001
NJ = 29
ARM_IDX = np.arange(15, 29)
WRIST = {19, 20, 21, 26, 27, 28}
ARM_VEL_LIMIT = 1.5    # rad/s, arm_controller arm_velocity_limit (Default, auch Sim)

# Arm-Posen (je 7 Gelenke: shoulder p/r/y, elbow, wrist r/p/y)
ARMS_WALK = np.array([0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0,
                      0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0])      # Lauf-Pose (Bridge-Reset)
ARMS_CARRY = np.array([-0.45, 0.10, 0.0, 0.25, 0.0, 0.0, 0.0,
                       -0.45, -0.10, 0.0, 0.25, 0.0, 0.0, 0.0])    # Unterarme nach vorn (Box tragen)
ARMS_DOWN = np.zeros(14)                                            # haengend (AGILE-Default)


def arms_wiggle(t, amp=1.0):
    """Staendige Arm-Bewegung (asymmetrisch, ~0.4-0.7 Hz). amp=1: kraeftig (Schulter
    +-0.8 rad, laeuft ins Speedlimit), amp=0.5: maessig (wie gemaechliches Greifen)."""
    a = ARMS_WALK.copy()
    a[0] += amp * 0.8 * math.sin(2 * math.pi * 0.5 * t)
    a[7] += amp * 0.8 * math.sin(2 * math.pi * 0.4 * t + 1.0)
    a[1] += amp * 0.3 * (1 + math.sin(2 * math.pi * 0.7 * t))
    a[8] -= amp * 0.3 * (1 + math.sin(2 * math.pi * 0.6 * t + 2.0))
    a[3] += amp * 0.5 * math.sin(2 * math.pi * 0.45 * t)
    a[10] += amp * 0.5 * math.sin(2 * math.pi * 0.55 * t + 0.5)
    return a


def arms_moderate(t):
    return arms_wiggle(t, 0.5)


def arms_carry(t):
    return ARMS_CARRY


def arms_carry_moving(t):
    """Box vor dem Koerper, dabei Schultern und Ellbogen bewegt (+-0.3 rad)."""
    a = ARMS_CARRY.copy()
    a[0] += 0.3 * math.sin(2 * math.pi * 0.5 * t)
    a[7] += 0.3 * math.sin(2 * math.pi * 0.4 * t + 1.0)
    a[3] += 0.3 * math.sin(2 * math.pi * 0.45 * t)
    a[10] += 0.3 * math.sin(2 * math.pi * 0.55 * t + 0.5)
    return a


def arms_reach(t):
    """Beide Arme weit nach vorn/oben und zurueck (Schulter -1.5..+0.3 rad)."""
    a = ARMS_WALK.copy()
    a[0] = a[7] = -0.6 + 0.9 * math.sin(2 * math.pi * 0.3 * t)
    a[3] = a[10] = 0.3
    return a


class Sim:
    def __init__(self, scene, payload_kg=0.0):
        # Ebener Boden: Die Sim-Szene hat ab x = 1 m Hindernisse (Kiste, Rampe,
        # Treppen, Hoehenfelder). Hier zaehlt nur das Regelverhalten, also alles
        # ausser dem Boden entfernen.
        self.scene = scene
        spec = mujoco.MjSpec.from_file(scene)
        for g in list(spec.worldbody.geoms):
            if g.name != "floor":
                spec.delete(g)
        self.m = spec.compile()
        self.m.opt.timestep = SIM_DT
        if payload_kg > 0.0:
            for side in ("left", "right"):
                bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wrist_yaw_link")
                self.m.body_mass[bid] += payload_kg
        self.d = mujoco.MjData(self.m)
        wid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_EQUALITY, "hold_base_weld")
        if wid >= 0:
            self.m.eq_active0[wid] = 0
            self.d.eq_active[wid] = 0
        mujoco.mj_setConst(self.m, self.d)
        self.n = 29
        self.dim_motor = 3 * self.n
        self.feet = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, f"{s}_ankle_roll_link")
                     for s in ("left", "right")]
        self.arm_dof = [self.m.jnt_dofadr[self.m.actuator_trnid[i, 0]] for i in ARM_IDX]
        self.arm_cmd = ARMS_WALK.copy()
        self.torso = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
        self.push = (0.0, 0.0, 0.0, 0.0)     # (t_start, fx, fy, dauer)

    def stance_height(self, legw):
        """Wie Bridge._stance_height: Becken-Hoehe, bei der die Fuesse in Pose legw
        genau auf dem Boden stehen."""
        m = self.m
        d = mujoco.MjData(m)
        d.qpos[:] = m.qpos0
        d.qpos[2] = 1.0
        for i in range(self.n):
            d.qpos[7 + i] = legw[i] if i < len(legw) else 0.0
        mujoco.mj_kinematics(m, d)
        zmin = min(d.geom_xpos[g][2] - m.geom_rbound[g] for g in range(m.ngeom)
                   if m.geom_bodyid[g] in self.feet
                   and (m.geom_contype[g] or m.geom_conaffinity[g]))
        return 1.0 - zmin

    def reset_stance(self, legs, arms=ARMS_WALK):
        """Wie Bridge._handle_managed_weld beim Wechsel nach RUN: Beine/Taille in die
        kommandierte Pose (legs: 12 Bein- oder 15 Bein+Taille-Werte), Arme in die
        Lauf-Pose, Becken so hoch, dass die Fuesse auf dem Boden stehen."""
        legw = list(legs) + [0.0] * (15 - len(legs))
        d = self.d
        mujoco.mj_resetData(self.m, d)
        d.qpos[0:2] = self.m.qpos0[0:2]
        d.qpos[2] = self.stance_height(legw)
        d.qpos[3:7] = self.m.qpos0[3:7]
        for i in range(self.n):
            d.qpos[7 + i] = legw[i] if i < 15 else arms[i - 15]
        self.arm_cmd = np.array(arms, dtype=float)
        d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, d)

    def state(self):
        sd = self.d.sensordata
        q = np.array(sd[0:self.n], np.float32)
        dq = np.array(sd[self.n:2 * self.n], np.float32)
        quat = np.array(sd[self.dim_motor:self.dim_motor + 4])
        gyro = np.array(sd[self.dim_motor + 4:self.dim_motor + 7], np.float32)
        return q, dq, quat, gyro

    def step(self, cmd_q, cmd_kp, cmd_kd, cmd_tau, arm_target, t=0.0):
        d = self.d
        t0, fx, fy, dur = self.push
        d.xfrc_applied[self.torso, 0:3] = (fx, fy, 0.0) if t0 <= t < t0 + dur else 0.0
        sd = d.sensordata
        q = sd[0:self.n]
        dq = sd[self.n:2 * self.n]
        ctrl = cmd_tau + cmd_kp * (cmd_q - q) + cmd_kd * (0.0 - dq)
        # arm_controller-Ersatz: Soll mit Speedlimit nachfuehren, PD + Schwerkraft-FF
        step = ARM_VEL_LIMIT * SIM_DT
        self.arm_cmd += np.clip(np.asarray(arm_target) - self.arm_cmd, -step, step)
        for k, i in enumerate(ARM_IDX):
            kp, kd = (40.0, 4.0) if i in WRIST else (150.0, 12.0)
            ctrl[i] = (kp * (self.arm_cmd[k] - q[i]) - kd * dq[i]
                       + d.qfrc_bias[self.arm_dof[k]])
        d.ctrl[:self.n] = ctrl
        mujoco.mj_step(self.m, d)

    def feet_xy(self):
        return np.array([self.d.xpos[b][:2] for b in self.feet])


def yaw_of(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


_COM_MODELS = {}


def com_model_for(scene):
    """Schwerpunkt-Modell zum Roboter der Szene (wie loco_sim robot_mjcf)."""
    name = "g1_29dof_inspire_ftp.xml" if "inspire" in os.path.basename(scene) else "g1_29dof.xml"
    if name not in _COM_MODELS:
        _COM_MODELS[name] = ComModel(os.path.join(G1_DIR, name))
    return _COM_MODELS[name]


def run(policy, scene, schedule, t_end, arms_fn, payload_kg=0.0, push=None, rescue=True,
        rescue_policy=None):
    """schedule(t) -> (mode, cmd), mode in {"walk", "stand"} = vom Nutzer angeforderter
    Zustand. Ablauf wie loco_sim:
      * Start (aus HOLD): Bridge stellt den Roboter in die Stand-Pose der Policy;
        "stand" -> PD mit goal = Stand-Pose, "walk" -> ein Takt Stand-Pose, dann Policy.
      * "stand" aus WALK startet die SettleGate (Policy bremst mit cmd = 0), danach
        PD mit gehaltener Pose.
      * Im PD: kippt der Roboter (TipDetector), faengt rescue_policy (None: die
        Lauf-Policy) mit cmd = 0 ab und gibt ueber die SettleGate zurueck (rescue=False:
        aus). Wird waehrend des Abfangens "walk" angefordert, geht es danach mit der
        Lauf-Policy weiter (wie loco_sim).
    push = (t, fx, fy, dauer): Stoss auf torso_link. Gibt (Sturzzeit, Log, Abfang-
    Zeitpunkte) zurueck; im Log ist mode der tatsaechlich aktive Regler."""
    sim = Sim(scene, payload_kg)
    rescue_policy = policy if rescue_policy is None else rescue_policy
    if push is not None:
        sim.push = push
    sp = policy.stand_leg_pose
    stand_pose = LEG_STAND_POSE.copy() if sp is None else sp.copy()
    sim.reset_stance(stand_pose)
    cm = com_model_for(scene)
    bal = StandBalancer(policy.step_dt)
    params = BalancerParams(mass_kg=cm.mass)
    gate = SettleGate()
    settle = SettleParams()
    tip = TipDetector()
    tip_p = TipParams()
    decim = int(round(policy.step_dt / SIM_DT))
    cmd_q = np.zeros(NJ)
    cmd_kp = np.zeros(NJ)
    cmd_kd = np.zeros(NJ)
    cmd_tau = np.zeros(NJ)
    mode = None
    rescuing = False
    fresh_walk = False
    rescues = []
    log = []
    fall_t = None
    fall_since = None
    for s in range(int(t_end / SIM_DT)):
        t = s * SIM_DT
        if s % decim == 0:
            want, cmd = schedule(t)
            q, dq, quat, gyro = sim.state()
            g = get_gravity_orientation(quat)
            # Sturz wie loco_sim._fallen (grav z > -0.5 fuer 0.3 s)
            if g[2] > -0.5:
                fall_since = t if fall_since is None else fall_since
                if fall_t is None and t - fall_since > 0.3:
                    fall_t = t
            else:
                fall_since = None
            if mode is None:                       # Start aus HOLD (Bridge hat aufgestellt)
                mode = want
                if mode == "walk":
                    fresh_walk = True
                else:
                    bal.enter(stand_pose)
            elif want == "walk" and mode == "stand":
                policy.reset()
                mode = "walk"
            elif want == "walk" and (not rescuing or rescue_policy is policy):
                gate.cancel()                      # STAND-Uebergabe/Abfangen abbrechen,
                rescuing = False                   # weiterlaufen (wie loco_sim._enter_walk)
            elif mode == "walk" and not gate.active:   # STAND angefordert
                gate.start(t)
            if mode == "stand" and rescue and tip.update(t, quat, q[LEG_IDX], g, tip_p):
                rescue_policy.reset()              # Abfangen: Policy mit cmd = 0
                mode = "walk"
                rescuing = True
                gate.start(t)
                rescues.append(t)
            if mode == "walk" and gate.active:
                cmd = [0.0, 0.0, 0.0]              # ausbremsen / abfangen
                if gate.update(t, gyro, dq[LEG_IDX], settle) is not None:
                    if rescuing and want == "walk":    # loco_sim._walk_after_rescue
                        policy.reset()
                        cmd = [0.0, 0.0, 0.0]
                    else:
                        bal.enter()                # PD haelt die vorgefundene Pose
                        tip.reset()
                        mode = "stand"
                    rescuing = False
            cmd_tau[:] = 0.0
            if mode == "walk" and fresh_walk:      # loco_sim._fresh_walk
                fresh_walk = False
                policy.reset()
                cmd_q[LEG_IDX], cmd_kp[LEG_IDX], cmd_kd[LEG_IDX] = stand_pose, LEG_KP, LEG_KD
                cmd_q[WAIST_IDX], cmd_kp[WAIST_IDX], cmd_kd[WAIST_IDX] = WAIST_TARGET, WAIST_KP, WAIST_KD
            elif mode == "walk":
                active = rescue_policy if rescuing else policy
                tg = active.step(q, dq, gyro, g, np.asarray(cmd, np.float32))
                cmd_q[tg.idx] = tg.q
                cmd_kp[tg.idx] = tg.kp
                cmd_kd[tg.idx] = tg.kd
            else:
                com_x = float(cm.com_in_foot(q)[0])
                c = bal.step(q[LEG_IDX], dq[LEG_IDX], g, gyro, t, params, com_x)
                cmd_q[LEG_IDX] = c.leg_q
                cmd_kp[LEG_IDX] = c.leg_kp
                cmd_kd[LEG_IDX] = c.leg_kd
                cmd_tau[LEG_IDX] = c.leg_tau
                cmd_q[WAIST_IDX] = c.waist_q
                cmd_kp[WAIST_IDX] = c.waist_kp
                cmd_kd[WAIST_IDX] = c.waist_kd
            d = sim.d
            yaw = yaw_of(d.qpos[3:7])
            v_w = d.qvel[0:2]
            v_b = np.array([math.cos(yaw) * v_w[0] + math.sin(yaw) * v_w[1],
                            -math.sin(yaw) * v_w[0] + math.cos(yaw) * v_w[1]])
            log.append(dict(t=t, mode=mode, cmd=np.array(cmd, float), xy=d.qpos[0:2].copy(),
                            yaw=yaw, v_b=v_b, wz=float(d.qvel[5]), grav=g.copy(),
                            z=float(d.qpos[2]), feet=sim.feet_xy(),
                            waist=q[12:15].copy(), shift=bal.shift))
        sim.step(cmd_q, cmd_kp, cmd_kd, cmd_tau, arms_fn(t), t)
    return fall_t, log, rescues


def window(log, t0, t1):
    return [r for r in log if t0 <= r["t"] < t1]


def mean_v(log, t0, t1):
    w = window(log, t0, t1)
    return (np.mean([r["v_b"][0] for r in w]), np.mean([r["v_b"][1] for r in w]),
            np.mean([r["wz"] for r in w]))


def disp(log, t0, t1):
    """Pelvis-Verschiebung [m] und max. Fuss-Verschiebung [m] in [t0, t1)."""
    w = window(log, t0, t1)
    pel = float(np.linalg.norm(w[-1]["xy"] - w[0]["xy"]))
    feet = float(np.max(np.linalg.norm(w[-1]["feet"] - w[0]["feet"], axis=1)))
    return pel, feet


def max_tilt(log, t0, t1):
    return float(max(math.hypot(r["grav"][0], r["grav"][1]) for r in window(log, t0, t1)))


class Report:
    def __init__(self):
        self.rows = []
        self.fails = 0

    def add(self, name, ok, detail, required=True):
        self.rows.append((name, ok, detail, required))
        if required and not ok:
            self.fails += 1
        tag = ("OK  " if ok else "FAIL") if required else "info"
        print(f"  [{tag}] {name:<44} {detail}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="agile_velocity_g1")
    ap.add_argument("--rescue-policy", default="",
                    help="Policy fuers Abfangen im PD-Stand (wie loco_sim rescue_policy; "
                         "leer: wie in der deploy.yaml der Lauf-Policy)")
    ap.add_argument("--inspire", action="store_true", help="Szene mit Inspire-FTP-Haenden")
    ap.add_argument("--quick", action="store_true", help="nur Kernszenarien")
    a = ap.parse_args()

    scene = os.path.join(G1_DIR, "scene_inspire_ftp.xml" if a.inspire else "scene.xml")
    policy = load_walk_policy(os.path.join(HERE, "policies", a.policy))
    rescue_name = a.rescue_policy or policy.rescue_policy_name or a.policy
    rescue_policy = (policy if rescue_name == a.policy else
                     load_walk_policy(os.path.join(HERE, "policies", rescue_name)))
    print(f"Policy: {a.policy} ({type(policy).__name__}), Abfangen: {rescue_name}   "
          f"Szene: {os.path.basename(scene)}")

    def run(*args, **kw):
        return globals()["run"](*args, rescue_policy=rescue_policy, **kw)
    R = Report()
    walk_arms = (lambda t: ARMS_WALK)

    # 1) Laufen in alle Richtungen, Arme in Lauf-Pose: Tracking + Bremsen
    print("\n=== 1) Laufen + Anhalten (Arme Lauf-Pose) ===")
    cases = [("vor 0.5", [0.5, 0, 0]), ("zurueck 0.4", [-0.4, 0, 0]),
             ("links 0.4", [0, 0.4, 0]), ("rechts 0.4", [0, -0.4, 0]),
             ("drehen 0.8", [0, 0, 0.8]), ("vor+drehen", [0.4, 0, 0.5])]
    if a.quick:
        cases = cases[:1]
    for nm, c in cases:
        sched = (lambda c: lambda t: ("walk", c if 1.0 <= t < 6.0 else [0, 0, 0]))(c)
        fall, log, _ = run(policy, scene, sched, 10.0, walk_arms)
        vx, vy, wz = mean_v(log, 3.0, 6.0)
        trk = abs(vx - c[0]) < 0.15 and abs(vy - c[1]) < 0.15 and abs(wz - c[2]) < 0.3
        pel, _ = disp(log, 8.0, 10.0)
        ok = fall is None and trk and pel < 0.10
        R.add(f"Laufen {nm}", ok,
              f"ist v=({vx:+.2f},{vy:+.2f},{wz:+.2f}) soll {tuple(c)}  "
              f"Nachlauf 8-10s {pel:.3f} m" + (f"  GESTUERZT t={fall:.1f}" if fall else ""))

    # 2) Arme frei beim Laufen
    print("\n=== 2) Arme frei beim Laufen ===")
    arm_cases = [("Arme haengend (Null-Pose)", lambda t: ARMS_DOWN, 0.0),
                 ("Arme staendig bewegt", arms_wiggle, 0.0),
                 ("Box tragen, 0.5 kg je Hand", lambda t: ARMS_CARRY, 0.5),
                 ("Box tragen, 1.0 kg je Hand", lambda t: ARMS_CARRY, 1.0)]
    if a.quick:
        arm_cases = arm_cases[1:3]
    for nm, fn, pl in arm_cases:
        sched = lambda t: ("walk", [0.4, 0, 0] if 1.0 <= t < 9.0 else [0, 0, 0])
        fall, log, _ = run(policy, scene, sched, 12.0, fn, payload_kg=pl)
        vx, vy, wz = mean_v(log, 3.0, 9.0)
        ok = fall is None and abs(vx - 0.4) < 0.15
        R.add(nm, ok, f"vx={vx:+.2f} (soll 0.40) vy={vy:+.2f} wz={wz:+.2f}  "
              f"max Neigung {max_tilt(log, 1.0, 12.0):.2f}"
              + (f"  GESTUERZT t={fall:.1f}" if fall else ""))
        fall, log, _ = run(policy, scene, lambda t: ("walk", [0, 0, 0.6] if 1.0 <= t < 7.0
                                                  else [0, 0, 0]), 9.0, fn, payload_kg=pl)
        R.add(nm + " (drehen)", fall is None,
              "steht" if fall is None else f"GESTUERZT t={fall:.1f}")

    # 3) Steht die Policy allein (cmd = 0)? (Vergleich: sie korrigiert mit Schritten)
    print("\n=== 3) Policy allein im Stand (cmd = 0, 20 s) ===")
    for nm, fn in [("Arme ruhig", walk_arms), ("Arme staendig bewegt", arms_wiggle)]:
        fall, log, _ = run(policy, scene, lambda t: ("walk", [0, 0, 0]), 20.0, fn)
        pel, feet = disp(log, 2.0, 20.0)
        R.add(f"Policy-Stand, {nm}", fall is None,
              f"Drift Becken {pel:.3f} m, Fuesse {feet:.3f} m in 18 s, "
              f"max Neigung {max_tilt(log, 2.0, 20.0):.2f}"
              + (f"  GESTUERZT t={fall:.1f}" if fall else ""))

    # 4) PD-Stand nach START BALANCING (Stand-Pose der Policy, 20 s): Fuesse bleiben
    #    stehen, kein Abfangen noetig. Box 2 kg je Hand ist Info: das Schwerpunkt-
    #    Modell kennt die Last nicht, beim schnellen Anheben kann ein Schritt kommen.
    print("\n=== 4) PD-Stand mit Arm-Bewegung und Last (20 s) ===")
    stand_cases = [("Arme ruhig", walk_arms, 0.0, True),
                   ("Arme haengend", lambda t: ARMS_DOWN, 0.0, True),
                   ("Arme maessig bewegt", arms_moderate, 0.0, True),
                   ("Arme kraeftig bewegt", arms_wiggle, 0.0, True),
                   ("Arme weit vor/zurueck", arms_reach, 0.0, True),
                   ("Box 0.5 kg je Hand, Arme vorn", arms_carry, 0.5, True),
                   ("Box 1.0 kg je Hand, Arme vorn", arms_carry, 1.0, True),
                   ("Box 1.0 kg je Hand, Arme bewegt", arms_carry_moving, 1.0, True),
                   ("Box 2.0 kg je Hand, Arme vorn", arms_carry, 2.0, False)]
    if a.quick:
        stand_cases = stand_cases[2:4]
    for nm, fn, pl, req in stand_cases:
        fall, log, res = run(policy, scene, lambda t: ("stand", [0, 0, 0]), 20.0, fn,
                             payload_kg=pl)
        pel, feet = disp(log, 1.0, 20.0)
        R.add(f"PD-Stand, {nm}", fall is None and not res and feet < 0.02,
              f"Fuesse {feet:.3f} m, max Neigung {max_tilt(log, 1.0, 20.0):.2f}, "
              f"max Verschiebung {max(abs(r['shift']) for r in log):.2f} rad"
              + (f", abgefangen {len(res)}x (Becken {pel:.2f} m)" if res else "")
              + (f"  GESTUERZT t={fall:.1f}" if fall else ""), required=req)

    # 5) Uebergaenge STAND <-> WALK (wie loco_sim: START BALANCING im Laufen ->
    #    Policy bremst aus -> PD haelt die Pose)
    print("\n=== 5) Uebergaenge PD-Stand <-> Policy ===")

    # Start im PD-Stand (wie nach START BALANCING), ab 2 s laufen, dann zurueck in
    # den PD.
    def stand_walk_stand(cmd, t_walk_end, t_stand, t_pd_start=2.0):
        return lambda t: (("stand", [0, 0, 0]) if t < t_pd_start or t >= t_stand else
                          ("walk", cmd) if t < t_walk_end else ("walk", [0, 0, 0]))

    trans = [  # (Name, Arme, kg je Hand, cmd, Lauf-Ende, STAND-Anforderung, Pflicht)
        ("Stand -> Laufen -> anhalten -> Stand", walk_arms, 0.0, [0.4, 0, 0], 6.0, 7.5, True),
        ("Stand direkt aus vollem Lauf", walk_arms, 0.0, [0.4, 0, 0], 6.0, 6.0, True),
        ("Stand direkt aus Drehung", walk_arms, 0.0, [0, 0, 0.8], 6.0, 6.0, True),
        ("Arme haengend, Stand direkt aus vollem Lauf", lambda t: ARMS_DOWN, 0.0,
         [0.4, 0, 0], 6.0, 6.0, True),
        ("Arme maessig bewegt", arms_moderate, 0.0, [0.4, 0, 0], 6.0, 7.5, True),
        ("Box 0.5 kg, anhalten -> Stand", arms_carry, 0.5, [0.4, 0, 0], 6.0, 7.5, True),
        ("Box 0.5 kg, Stand direkt aus vollem Lauf", arms_carry, 0.5, [0.4, 0, 0], 6.0, 6.0,
         True),
        ("Box 1.0 kg, Stand direkt aus Seitwaertslauf", arms_carry, 1.0, [0, 0.3, 0], 6.0, 6.0,
         True),
        ("Box 2.0 kg, Stand direkt aus vollem Lauf", arms_carry, 2.0, [0.4, 0, 0], 6.0, 6.0,
         False),
    ]
    if a.quick:
        trans = trans[:2]
    for nm, fn, pl, cmd, t_end_walk, t_stand, req in trans:
        fall, log, res = run(policy, scene, stand_walk_stand(cmd, t_end_walk, t_stand),
                             15.0, fn, payload_kg=pl)
        # Tatsaechlicher PD-Einsatz (nach dem Ausbremsen) und danach Fuss-Drift.
        t_pd = next((r["t"] for r in log if r["t"] >= t_stand and r["mode"] == "stand"), None)
        if fall is not None or t_pd is None:
            R.add(nm, False, f"GESTUERZT t={fall:.1f}" if fall else "PD hat nie uebernommen",
                  required=req)
            continue
        pel, feet = disp(log, t_pd + 0.5, 15.0)
        R.add(nm, feet < 0.02 and not res,
              f"PD nach {t_pd - t_stand:.2f} s Ausbremsen, danach Fuesse {feet:.3f} m, "
              f"max Neigung {max_tilt(log, t_stand, 15.0):.2f}"
              + (f", abgefangen {len(res)}x" if res else ""), required=req)

    # 6) Zufalls-Uebergaenge: beliebige Kommandos, beliebige Wechselzeitpunkte
    if not a.quick:
        print("\n=== 6) Zufalls-Uebergaenge (Laufen -> START BALANCING) ===")
        rng = np.random.default_rng(7)
        arm_sets = [(arms_carry, 0.5), (walk_arms, 0.0), (arms_moderate, 0.0),
                    (lambda t: ARMS_DOWN, 0.0), (arms_carry, 1.0)]
        n, falls, n_res, worst = 20, 0, 0, 0.0
        for k in range(n):
            cmd = [rng.uniform(-0.4, 0.5), rng.uniform(-0.4, 0.4), rng.uniform(-0.8, 0.8)]
            t_stand = rng.uniform(4.0, 6.0)
            fn, pl = arm_sets[k % len(arm_sets)]
            fall, log, res = run(policy, scene, stand_walk_stand(cmd, t_stand, t_stand),
                                 11.0, fn, payload_kg=pl)
            n_res += len(res)
            if fall is not None:
                falls += 1
                print(f"    Sturz: Satz {k % len(arm_sets)}, cmd={np.round(cmd, 2)}, "
                      f"Wechsel {t_stand:.1f} s, Sturz {fall:.1f} s")
            else:
                t_pd = next(r["t"] for r in log if r["t"] >= t_stand and r["mode"] == "stand")
                worst = max(worst, disp(log, t_pd + 0.5, 11.0)[1])
        R.add(f"{n} Zufalls-Wechsel Laufen -> PD", falls == 0 and n_res == 0 and worst < 0.02,
              f"Stuerze {falls}/{n}, abgefangen {n_res}x, "
              f"max Fuss-Drift nach Uebergabe {worst:.3f} m")

    # 7) Loslaufen: START WALKING mit cmd = 0 aus dem PD-Stand nach START BALANCING
    #    und aus dem PD nach einer Uebergabe. Gemessen: max. Becken-Weg in 4 s.
    print("\n=== 7) Anlauf bei START WALKING mit cmd = 0 ===")
    for nm, sched, t_w in [
            ("aus PD-Stand nach START BALANCING", lambda t: ("stand", [0, 0, 0]) if t < 2.5
             else ("walk", [0, 0, 0]), 2.5),
            ("aus PD nach Uebergabe", lambda t: ("walk", [0, 0, 0]) if t < 3.0
             else ("stand", [0, 0, 0]) if t < 7.0 else ("walk", [0, 0, 0]), 7.0)]:
        fall, log, _ = run(policy, scene, sched, t_w + 4.0, walk_arms)
        w = window(log, t_w, t_w + 4.0)
        pel = max(float(np.linalg.norm(r["xy"] - w[0]["xy"])) for r in w)
        R.add(f"Loslaufen {nm}", fall is None and pel < 0.05,
              f"Becken bis {pel:.3f} m verschoben" + (f"  GESTUERZT t={fall:.1f}" if fall else ""))

    # 8) Stoesse im PD-Stand (wie der PUSH-Button: Kraft 0.12 s auf den Torso).
    #    Pflicht: kein Sturz. Info: ob abgefangen werden musste und wie weit er ging.
    if not a.quick:
        print("\n=== 8) Stoesse im PD-Stand (0.12 s auf den Torso) ===")
        for f_n in (80.0, 150.0, 250.0):
            for dn, (ux, uy) in (("vorn", (1, 0)), ("hinten", (-1, 0)), ("seitlich", (0, 1)),
                                 ("schraeg", (0.7, -0.7))):
                fall, log, res = run(policy, scene, lambda t: ("stand", [0, 0, 0]), 9.0,
                                     walk_arms, push=(3.0, f_n * ux, f_n * uy, 0.12))
                pel, _ = disp(log, 2.5, 9.0)
                back = log[-1]["mode"] == "stand"
                R.add(f"Stoss {f_n:.0f} N {dn}", fall is None and back,
                      ("abgefangen mit Schritten" if res else "PD haelt ohne Schritt")
                      + f", Becken {pel:.2f} m" + ("" if back else ", PD nicht zurueck")
                      + (f"  GESTUERZT t={fall:.1f}" if fall else ""))

    print("\nERGEBNIS:", "BESTANDEN" if R.fails == 0 else f"DURCHGEFALLEN ({R.fails} Pflicht-Szenarien)")
    return R.fails


if __name__ == "__main__":
    raise SystemExit(1 if main() else 0)

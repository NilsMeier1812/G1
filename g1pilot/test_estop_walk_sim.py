#!/usr/bin/env python3
"""Headless-Test: NOT-HALT waehrend des Laufens (MuJoCo, ohne ROS/DDS).

Frage (Nils, 2026-10-19): Bleibt der G1 stehen oder faellt er um, wenn im
Walk-Modus beim Laufen der E-Stop gedrueckt wird?

Szenarien (Notaus bei t_estop, vorher Laufen mit cmd):
  sim_damp   : was loco_sim heute in der Sim macht (_on_emergency -> DAMP):
               alle 29 Motoren kp=0, kd=damp_kd (8).
  real       : Nachbildung des Real-Pfads (demo_real): loco_client schickt
               StopMove (Policy laeuft mit cmd=0 weiter, Beine bleiben geregelt,
               estop_damp_legs=false), arm_controller daempft die Arme
               (kp=0, kd=estop_arm_kd=3) und haelt die Taille an der Stelle des
               Notaus (estop_waist_hold, WaistHold kp=150/kd=4, arm_sdk-Gewicht 1).
  real_arms_held : Vergleich: wie real, aber Arme an der Stelle des Notaus
               gehalten statt schlaff (trennt den Einfluss der schlaffen Arme).
  real_waist_limp : wie real, aber Taille passiv (kp=0/kd=3) = Verhalten vor
               dem Fix (go_passive), das Nils am 2026-10-15 am echten G1 sah.
Die Lauf-Policy steht hier fuer den Onboard-Regler des echten G1 (dessen Code
ist nicht verfuegbar) -> "real" ist eine Naeherung, kein Beweis fuer die Hardware.

Lauf:  python3 test_estop_walk_sim.py [--policy NAME] [--speed 0.5] [--inspire]
"""
import argparse
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from test_agile_walk_sim import (  # noqa: E402
    Sim, ARMS_WALK, ARM_IDX, WRIST, SIM_DT, NJ, G1_DIR, yaw_of)
from g1pilot.navigation.walk_policy import get_gravity_orientation, load_walk_policy  # noqa: E402
from g1pilot.navigation.stand_balancer import (  # noqa: E402
    LEG_IDX, LEG_KP, LEG_KD, LEG_STAND_POSE, WAIST_IDX, WAIST_KP, WAIST_KD, WAIST_TARGET)

DAMP_KD = 8.0          # loco_sim damp_kd
ESTOP_ARM_KD = 3.0     # arm_controller estop_arm_kd
WAIST_HOLD_KP, WAIST_HOLD_KD, WAIST_PASSIVE_KD = 150.0, 4.0, 3.0   # waist_hold.WaistHold


def run(policy, scene, scenario, cmd, t_estop, t_end):
    sim = Sim(scene)
    sp = policy.stand_leg_pose
    stand_pose = LEG_STAND_POSE.copy() if sp is None else sp.copy()
    sim.reset_stance(stand_pose)
    policy.reset()
    decim = int(round(policy.step_dt / SIM_DT))
    cmd_q, cmd_kp, cmd_kd, cmd_tau = (np.zeros(NJ) for _ in range(4))
    waist_lock = None
    first = True
    log = []
    m, d = sim.m, sim.d
    for s in range(int(t_end / SIM_DT)):
        t = s * SIM_DT
        estop = t >= t_estop
        q, dq, quat, gyro = sim.state()
        if s % decim == 0:
            g = get_gravity_orientation(quat)
            if estop and scenario == "sim_damp":
                cmd_q[:] = 0.0
                cmd_kp[:] = 0.0
                cmd_kd[:] = DAMP_KD
            elif first:                         # loco_sim._fresh_walk
                first = False
                cmd_q[LEG_IDX], cmd_kp[LEG_IDX], cmd_kd[LEG_IDX] = stand_pose, LEG_KP, LEG_KD
                cmd_q[WAIST_IDX], cmd_kp[WAIST_IDX], cmd_kd[WAIST_IDX] = WAIST_TARGET, WAIST_KP, WAIST_KD
            else:
                c = np.zeros(3, np.float32) if estop else np.asarray(cmd, np.float32)
                tg = policy.step(q, dq, gyro, g, c)
                cmd_q[tg.idx], cmd_kp[tg.idx], cmd_kd[tg.idx] = tg.q, tg.kp, tg.kd
            yaw = yaw_of(d.qpos[3:7])
            v = d.qvel[0:2]
            log.append(dict(t=t, x=d.qpos[0:2].copy(), z=float(d.qpos[2]),
                            tilt=math.degrees(math.acos(max(-1.0, min(1.0, -g[2])))),
                            vx=math.cos(yaw) * v[0] + math.sin(yaw) * v[1]))
        if estop and waist_lock is None:
            waist_lock = q[WAIST_IDX].copy()
            arm_lock = q[ARM_IDX].copy()
        # Sollwerte -> Drehmomente (wie Sim.step), Arme/Taille je Szenario
        ctrl = cmd_tau + cmd_kp * (cmd_q - q) + cmd_kd * (0.0 - dq)
        if not estop:
            step = 1.5 * SIM_DT
            sim.arm_cmd += np.clip(ARMS_WALK - sim.arm_cmd, -step, step)
            for k, i in enumerate(ARM_IDX):
                kp, kd = (40.0, 4.0) if i in WRIST else (150.0, 12.0)
                ctrl[i] = kp * (sim.arm_cmd[k] - q[i]) - kd * dq[i] + d.qfrc_bias[sim.arm_dof[k]]
        elif scenario != "sim_damp":
            if scenario == "real_arms_held":
                for k, i in enumerate(ARM_IDX):
                    kp, kd = (40.0, 4.0) if i in WRIST else (150.0, 12.0)
                    ctrl[i] = (kp * (arm_lock[k] - q[i]) - kd * dq[i]
                               + d.qfrc_bias[sim.arm_dof[k]])
            else:
                ctrl[ARM_IDX] = -ESTOP_ARM_KD * dq[ARM_IDX]      # Arme schlaff, gedaempft
            if scenario in ("real", "real_arms_held"):
                ctrl[WAIST_IDX] = (WAIST_HOLD_KP * (waist_lock - q[WAIST_IDX])
                                   - WAIST_HOLD_KD * dq[WAIST_IDX])
            else:
                ctrl[WAIST_IDX] = -WAIST_PASSIVE_KD * dq[WAIST_IDX]
        d.ctrl[:NJ] = ctrl
        mujoco.mj_step(m, d)
    return log


def summarize(log, t_estop):
    pre = [r for r in log if t_estop - 0.5 <= r["t"] < t_estop]
    post = [r for r in log if r["t"] >= t_estop]
    x0 = post[0]["x"]
    fallen = next((r["t"] - t_estop for r in post if r["z"] < 0.45 or r["tilt"] > 60), None)
    still = next((r["t"] - t_estop for r in post
                  if all(abs(p["vx"]) < 0.05 for p in post if p["t"] >= r["t"])), None)
    return dict(
        v_pre=np.mean([r["vx"] for r in pre]), z_pre=np.mean([r["z"] for r in pre]),
        z_min=min(r["z"] for r in post), z_end=post[-1]["z"],
        tilt_max=max(r["tilt"] for r in post), tilt_end=post[-1]["tilt"],
        run_out=float(np.linalg.norm(post[-1]["x"] - x0)), fallen=fallen, still=still)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", action="append",
                    help="Lauf-Policy (mehrfach moeglich), Default: alle drei")
    ap.add_argument("--speed", type=float, action="append", help="vx in m/s (mehrfach)")
    ap.add_argument("--inspire", action="store_true")
    ap.add_argument("--timeline", action="store_true", help="Verlauf je 0.25 s ausgeben")
    a = ap.parse_args()
    policies = a.policy or ["g1_wholebody", "agile_velocity_g1", "agile_velocity_height_g1"]
    speeds = a.speed or [0.5]
    scene = os.path.join(G1_DIR, "scene_inspire_ftp.xml" if a.inspire else "scene.xml")
    t_estop, t_end = 4.0, 9.0
    ok_all = True
    for pname in policies:
        policy = load_walk_policy(os.path.join(HERE, "policies", pname))
        for vx in speeds:
            for sc in ("sim_damp", "real", "real_arms_held", "real_waist_limp"):
                log = run(policy, scene, sc, [vx, 0.0, 0.0], t_estop, t_end)
                r = summarize(log, t_estop)
                verdict = (f"UMGEFALLEN nach {r['fallen']:.2f} s" if r["fallen"] is not None
                           else ("steht" + (f", ruhig nach {r['still']:.2f} s"
                                            if r["still"] is not None else ", aber in Bewegung")))
                print(f"{pname:<26} v={vx:.2f} {sc:<16} vorher v={r['v_pre']:+.2f} m/s "
                      f"z={r['z_pre']:.3f} | danach z_min={r['z_min']:.3f} z_end={r['z_end']:.3f} "
                      f"Neigung max={r['tilt_max']:5.1f} end={r['tilt_end']:5.1f} Grad "
                      f"Auslauf={r['run_out']:.2f} m -> {verdict}", flush=True)
                if a.timeline:
                    for row in log:
                        if row["t"] >= t_estop - 0.25 and abs(row["t"] / 0.25 - round(row["t"] / 0.25)) < 1e-6:
                            print(f"     t={row['t'] - t_estop:+.2f}s z={row['z']:.3f} "
                                  f"Neigung={row['tilt']:5.1f} vx={row['vx']:+.2f}")
                if sc == "real" and r["fallen"] is not None:
                    ok_all = False
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()

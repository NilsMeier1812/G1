#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
walk_policy — Lauf-Policies fuer loco_sim (WALK-Zustand), ohne ROS-Abhaengigkeit.

Kapselt alles Policy-Spezifische (Obs-Aufbau, History, Gelenk-Zuordnung, Gains),
damit loco_sim und die Headless-Tests EXAKT denselben Code benutzen. Welche Klasse
geladen wird, entscheidet  format:  in der deploy.yaml des Policy-Ordners:

  * agile_history  (Default, policies/agile_velocity_g1)
      NVIDIA WBC-AGILE "Velocity-G1-History-v0". Steuert NUR Beine + Taille
      roll/pitch (14 Gelenke) und SIEHT die Arme nicht. Im Training wurden die
      Arme auch beim Laufen staendig zufaellig bewegt -> die Arme sind beim
      Laufen frei (arm_controller/Marker), keine Lauf-Pose noetig. Gang: Fang-
      schritte ohne festen Rhythmus (kein Gang-Term im Training), sehr robust.
  * agile_height_history (policies/agile_velocity_height_g1)
      NVIDIA WBC-AGILE "Velocity-Height-G1-History-v0". Steuert NUR die Beine
      (12), SIEHT alle 29 Gelenke inkl. Arme. Gleichmaessiger Gang, Arme frei.
  * mjlab_velocity (Legacy, policies/g1_wholebody)
      unitree_rl_mjlab G1 Velocity. Ganzkoerper-Obs (29 Gelenke inkl. Arme) ->
      laeuft nur stabil mit Armen nahe ihrer Default-Pose.

Gemeinsame Schnittstelle:
  reset()                                  vor dem ersten step() nach WALK-Eintritt
  step(q, dq, gyro, gravity, cmd)          -> MotorTargets fuer die geregelten Motoren
  scale_command(nx, ny, nz)                normierte [-1, 1] -> phys. Sollwert
  stand_leg_pose, rescue_policy_name       aus der deploy.yaml (None, wenn nicht gesetzt)
Eingaben in Unitree-Motorreihenfolge (29), gyro im Body-Frame, gravity = projizierte
Gravitation (aufrecht [0, 0, -1]), cmd = [vx, vy, vyaw].
"""
import math
import os
from dataclasses import dataclass

import numpy as np
import yaml

from g1pilot.utils.joints_names import JOINT_NAMES_ROS

NJ = 29
_MOTOR_INDEX = {name: idx for idx, name in JOINT_NAMES_ROS.items()}


@dataclass
class MotorTargets:
    """PD-Sollwerte fuer eine Teilmenge der Motoren (Unitree-Indizes)."""
    idx: np.ndarray   # int, Motorindizes
    q: np.ndarray     # Sollwinkel [rad]
    kp: np.ndarray
    kd: np.ndarray


def get_gravity_orientation(quat):
    """Projizierte Gravitation aus dem Pelvis-Quaternion [w,x,y,z] (aufrecht=[0,0,-1])."""
    qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
    g = np.zeros(3, dtype=np.float32)
    g[0] = 2.0 * (-qz * qx + qw * qy)
    g[1] = -2.0 * (qz * qy + qw * qx)
    g[2] = 1.0 - 2.0 * (qw * qw + qz * qz)
    return g


def _motor_indices(names):
    missing = [n for n in names if n not in _MOTOR_INDEX]
    if missing:
        raise ValueError(f"Unbekannte Gelenknamen in deploy.yaml: {missing}")
    return np.array([_MOTOR_INDEX[n] for n in names], dtype=np.int64)


def _make_session(path):
    import onnxruntime as ort
    so = ort.SessionOptions()
    so.intra_op_num_threads = 1
    so.inter_op_num_threads = 1
    return ort.InferenceSession(path, sess_options=so, providers=["CPUExecutionProvider"])


class _VelocityPolicyBase:
    # True: die Policy laeuft nur stabil, wenn die Arme in ihrer Trainings-Pose
    # stehen -> loco_sim wartet vor WALK auf /g1pilot/arms/walk_ready.
    needs_arm_pose = True

    def __init__(self, pdir, dep):
        self.pdir = pdir
        self.policy_path = os.path.join(pdir, dep.get("policy_file", "policy.onnx"))
        self.step_dt = float(dep.get("step_dt", 0.02))
        rng = dep["commands"]["base_velocity"]["ranges"]
        self.cmd_x = (float(rng["lin_vel_x"][0]), float(rng["lin_vel_x"][1]))
        self.cmd_y = (float(rng["lin_vel_y"][0]), float(rng["lin_vel_y"][1]))
        self.cmd_yaw = (float(rng["ang_vel_z"][0]), float(rng["ang_vel_z"][1]))
        # Stand-Pose der Beine (Motoren 0..11), in der loco_sim den Roboter bei
        # START BALANCING aufstellt. None -> Standard-Standpose des Balancers.
        sp = dep.get("stand_pose_legs")
        self.stand_leg_pose = None if sp is None else np.array(sp, dtype=np.float32)
        if self.stand_leg_pose is not None and self.stand_leg_pose.shape != (12,):
            raise ValueError("deploy.yaml: stand_pose_legs braucht 12 Werte (Motoren 0..11)")
        # Policy fuers Abfangen im PD-Stand (Ordnername unter policies/). None -> diese.
        self.rescue_policy_name = dep.get("rescue_policy") or None
        self.sess = _make_session(self.policy_path)
        self.in_name = self.sess.get_inputs()[0].name
        self.num_obs = int(self.sess.get_inputs()[0].shape[-1])

    def scale_command(self, nx, ny, nz):
        """Normierte Joystick-Werte [-1, 1] auf den Trainingsbereich abbilden."""
        nx = max(-1.0, min(1.0, nx))
        ny = max(-1.0, min(1.0, ny))
        nz = max(-1.0, min(1.0, nz))
        vx = nx * (self.cmd_x[1] if nx >= 0 else -self.cmd_x[0])
        vy = ny * (self.cmd_y[1] if ny >= 0 else -self.cmd_y[0])
        vyaw = nz * (self.cmd_yaw[1] if nz >= 0 else -self.cmd_yaw[0])
        return np.array([vx, vy, vyaw], dtype=np.float32)

    def _run(self, obs):
        return self.sess.run(None, {self.in_name: obs})[0][0].astype(np.float32)

    def warmup(self):
        obs = np.zeros((1, self.num_obs), dtype=np.float32)
        for _ in range(3):
            self._run(obs)


class AgileHistoryPolicy(_VelocityPolicyBase):
    """WBC-AGILE-Policies mit Obs-History (Isaac-Lab-Export):
      * Velocity-G1-History-v0        (policies/agile_velocity_g1, format agile_history)
      * Velocity-Height-G1-History-v0 (policies/agile_velocity_height_g1,
                                       format agile_height_history)
    Beide bauen die Obs gleich: je Term ein History-Puffer, Term fuer Term
    hintereinander. Welche Terme in welcher Reihenfolge, welche Gelenke die Policy
    steuert und welche sie sieht, steht in der deploy.yaml."""

    needs_arm_pose = False   # Arme im Training randomisiert, keine Lauf-Pose noetig

    # Bekannte Obs-Terme -> Groesse (None: Anzahl Gelenke des Terms).
    _KNOWN_TERMS = {
        "base_ang_vel": 3, "projected_gravity": 3,
        "velocity_commands": 3,           # vx, vy, vyaw
        "velocity_height_commands": 4,    # vx, vy, vyaw, Becken-Hoehe
        "joint_pos_rel": None, "joint_vel_rel": None, "last_action": None,
    }

    def __init__(self, pdir, dep):
        super().__init__(pdir, dep)
        # Gesteuerte Gelenke (Aktionen, last_action)
        self.joint_names = list(dep["joint_names"])
        self.idx = _motor_indices(self.joint_names)
        n = len(self.idx)
        self.default = np.array(dep["default_joint_pos"], dtype=np.float32)
        self.kp = np.array(dep["stiffness"], dtype=np.float32)
        self.kd = np.array(dep["damping"], dtype=np.float32)
        if not (len(self.default) == len(self.kp) == len(self.kd) == n):
            raise ValueError("deploy.yaml: joint_names/default/stiffness/damping ungleich lang")
        # action_scale: eine Zahl oder je Gelenk; action_clip: Roh-Aktion begrenzen
        sc = np.atleast_1d(np.array(dep["action_scale"], dtype=np.float32))
        if sc.size not in (1, n):
            raise ValueError("deploy.yaml: action_scale braucht 1 oder len(joint_names) Werte")
        self.action_scale = sc if sc.size == n else np.full(n, sc[0], dtype=np.float32)
        clip = dep.get("action_clip")
        self.action_clip = None if clip is None else float(clip)
        # Beobachtete Gelenke (joint_pos_rel/joint_vel_rel). Fehlt obs_joint_names,
        # sieht die Policy genau die gesteuerten Gelenke.
        if "obs_joint_names" in dep:
            self.obs_idx = _motor_indices(list(dep["obs_joint_names"]))
            self.obs_default = np.array(dep["obs_default_joint_pos"], dtype=np.float32)
            if len(self.obs_default) != len(self.obs_idx):
                raise ValueError("deploy.yaml: obs_joint_names/obs_default_joint_pos ungleich lang")
        else:
            self.obs_idx, self.obs_default = self.idx, self.default
        self.history = int(dep["history_length"])
        self.min_norm = float(dep["commands"]["base_velocity"].get("min_norm", 0.0))
        # Feste Becken-Hoehe fuer velocity_height_commands [m]
        self.base_height = float(dep["commands"].get("base_height", 0.0))

        held = dep.get("held_joints", {}) or {}
        self.held_idx = _motor_indices(list(held.keys()))
        self.held_q = np.array([float(v["q"]) for v in held.values()], dtype=np.float32)
        self.held_kp = np.array([float(v["kp"]) for v in held.values()], dtype=np.float32)
        self.held_kd = np.array([float(v["kd"]) for v in held.values()], dtype=np.float32)

        terms = dep["observations"]
        self.term_names = [t["name"] for t in terms]
        joint_dim = {"joint_pos_rel": len(self.obs_idx), "joint_vel_rel": len(self.obs_idx),
                     "last_action": n}
        self.term_dims, self.term_scales, self.term_clips = [], [], []
        for t in terms:
            name = t["name"]
            if name not in self._KNOWN_TERMS:
                raise ValueError(f"deploy.yaml: unbekannter Obs-Term {name}")
            want = self._KNOWN_TERMS[name] or joint_dim[name]
            if int(t["dim"]) != want:
                raise ValueError(f"deploy.yaml: Obs-Term {name} hat dim {t['dim']}, erwartet {want}")
            self.term_dims.append(want)
            self.term_scales.append(float(t.get("scale", 1.0)))
            self.term_clips.append(None if t.get("clip") is None else float(t["clip"]))
        if "velocity_height_commands" in self.term_names and self.base_height <= 0.0:
            raise ValueError("deploy.yaml: velocity_height_commands braucht commands.base_height")
        frame = sum(self.term_dims)
        if frame * self.history != self.num_obs:
            raise ValueError(f"ONNX erwartet {self.num_obs} Obs, deploy.yaml ergibt "
                             f"{self.history} x {frame}")

        self.out_idx = np.concatenate([self.idx, self.held_idx])
        self.obs = np.zeros((1, self.num_obs), dtype=np.float32)
        self.reset()
        self.warmup()
        self.reset()

    def reset(self):
        self.last_action = np.zeros(len(self.idx), dtype=np.float32)
        # Je Term ein (history, dim)-Puffer, Zeile 0 = aeltester Wert.
        self._hist = [np.zeros((self.history, d), dtype=np.float32) for d in self.term_dims]
        self._fresh = True

    def command_for_policy(self, cmd):
        cmd = np.asarray(cmd, dtype=np.float32)
        return cmd if float(np.linalg.norm(cmd)) >= self.min_norm else np.zeros(3, np.float32)

    def _term_value(self, name, q, dq, gyro, gravity, cmd):
        if name == "base_ang_vel":
            return np.asarray(gyro, dtype=np.float32)
        if name == "projected_gravity":
            return np.asarray(gravity, dtype=np.float32)
        if name == "velocity_commands":
            return self.command_for_policy(cmd)
        if name == "velocity_height_commands":
            return np.append(self.command_for_policy(cmd), np.float32(self.base_height))
        if name == "joint_pos_rel":
            return q[self.obs_idx] - self.obs_default
        if name == "joint_vel_rel":
            return dq[self.obs_idx]
        return self.last_action                      # last_action

    def build_obs(self, q, dq, gyro, gravity, cmd):
        """Obs-Vektor (1, num_obs) aus dem aktuellen Zustand bauen und die History fortschreiben."""
        q = np.asarray(q, dtype=np.float32)
        dq = np.asarray(dq, dtype=np.float32)
        off = 0
        for k, buf in enumerate(self._hist):
            val = self._term_value(self.term_names[k], q, dq, gyro, gravity, cmd)
            if self.term_clips[k] is not None:       # Isaac Lab: clip vor scale
                val = np.clip(val, -self.term_clips[k], self.term_clips[k])
            val = val * self.term_scales[k]
            if self._fresh:
                buf[:] = val                 # wie Isaac Lab: erster Wert fuellt die History
            else:
                buf[:-1] = buf[1:]
                buf[-1] = val
            size = buf.size
            self.obs[0, off:off + size] = buf.reshape(-1)
            off += size
        self._fresh = False
        return self.obs

    def step(self, q, dq, gyro, gravity, cmd):
        obs = self.build_obs(q, dq, gyro, gravity, cmd)
        action = self._run(obs)
        self.last_action = action
        a = action if self.action_clip is None else np.clip(action, -self.action_clip,
                                                            self.action_clip)
        q_tgt = self.default + self.action_scale * a
        return MotorTargets(
            idx=self.out_idx,
            q=np.concatenate([q_tgt, self.held_q]),
            kp=np.concatenate([self.kp, self.held_kp]),
            kd=np.concatenate([self.kd, self.held_kd]),
        )


class MjlabVelocityPolicy(_VelocityPolicyBase):
    """Legacy: unitree_rl_mjlab G1 Velocity (policies/g1_wholebody/deploy.yaml).

    98 Obs, 29 Aktionen; aktuiert werden nur Beine + Taille (0..14), die Arme
    gehoeren dem arm_controller."""

    def __init__(self, pdir, dep):
        super().__init__(pdir, dep)
        self.kps = np.array(dep["stiffness"], dtype=np.float32)
        self.kds = np.array(dep["damping"], dtype=np.float32)
        self.default = np.array(dep["default_joint_pos"], dtype=np.float32)
        self.action_scale = np.array(dep["actions"]["JointPositionAction"]["scale"],
                                     dtype=np.float32)
        self.gait_period = float(dep["observations"]["gait_phase"]["params"].get("period", 0.6))
        self.stand_eps = 0.1   # wie im Training (mjlab phase() < 0.1)
        if len(self.kps) != NJ or len(self.default) != NJ:
            raise ValueError("deploy.yaml: erwarte 29 Gelenke")
        if self.num_obs != 98:
            raise ValueError(f"erwarte 98 Obs, ONNX meldet {self.num_obs}")
        self.out_idx = np.arange(15)
        self.obs = np.zeros((1, self.num_obs), dtype=np.float32)
        self.warmup()
        self.reset()

    def reset(self):
        self.counter = 0
        self.last_action = np.zeros(NJ, dtype=np.float32)

    def step(self, q, dq, gyro, gravity, cmd):
        self.counter += 1
        cmd = np.asarray(cmd, dtype=np.float32)
        if float(np.linalg.norm(cmd)) < self.stand_eps:
            sin_p = cos_p = 0.0
        else:
            phase = (self.counter * self.step_dt % self.gait_period) / self.gait_period
            sin_p = math.sin(2.0 * math.pi * phase)
            cos_p = math.cos(2.0 * math.pi * phase)
        o = self.obs[0]
        o[0:3] = gyro
        o[3:6] = gravity
        o[6:9] = cmd
        o[9] = sin_p
        o[10] = cos_p
        o[11:11 + NJ] = np.asarray(q, dtype=np.float32) - self.default
        o[11 + NJ:11 + 2 * NJ] = dq
        o[11 + 2 * NJ:11 + 3 * NJ] = self.last_action
        self.last_action = self._run(self.obs)
        target = self.default + self.last_action * self.action_scale
        i = self.out_idx
        return MotorTargets(idx=i, q=target[i], kp=self.kps[i], kd=self.kds[i])


_FORMATS = {
    "agile_history": AgileHistoryPolicy,
    "agile_height_history": AgileHistoryPolicy,
    "mjlab_velocity": MjlabVelocityPolicy,
}


def load_walk_policy(pdir):
    """Policy aus einem Ordner mit deploy.yaml laden (format: siehe _FORMATS)."""
    with open(os.path.join(pdir, "deploy.yaml"), "r") as f:
        dep = yaml.safe_load(f)
    fmt = dep.get("format", "mjlab_velocity")   # alte deploy.yaml ohne format-Feld
    if fmt not in _FORMATS:
        raise ValueError(f"Unbekanntes Policy-Format '{fmt}' in {pdir}/deploy.yaml")
    return _FORMATS[fmt](pdir, dep)

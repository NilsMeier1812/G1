#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stand_balancer — modellbasierter Knoechel-/Hueft-PD-Balancer fuer loco_sim (STAND),
ohne ROS-Abhaengigkeit.

Aus loco_sim herausgeloest (Regelgesetz unveraendert), damit loco_sim und die
Headless-Tests denselben Code fahren statt einer Kopie.

Posture-PD auf die Standpose (Steifigkeit x kp_scale) + Feedforward-DREHMOMENT [Nm]
auf Knoechel (primaer) und Huefte (sekundaer), das die per IMU gemessene Neigung
aktiv aufrichtet. Braucht nur IMU + Encoder -> Fuesse geplant, faengt Oberkoerper-/
Arm-Stoerungen ohne Schritte. Die Bridge addiert tau zum Posture-PD.

Uebergabe WALK -> STAND (SettleGate + hold_pose): Die AGILE-Lauf-Policy steht in
leichter Hocke (Becken ~0.73 m, Knie ~0.8 rad) und laesst die Fuesse oft leicht
versetzt stehen. Reisst der PD die Beine von dort in die gestreckte Standpose,
wandert das Becken nach vorn; mit Armen vor dem Koerper (Box tragen) kippt der
Roboter dann nach vorn (headless reproduziert). Deshalb:
  1. Die Policy bremst zuerst mit cmd = 0 aus, bis der Roboter ruhig steht
     (SettleGate: Gyro + Bein-Gelenkgeschwindigkeiten klein).
  2. Erst dann uebernimmt der PD und haelt die Beinpose, in der er sie vorfindet
     (StandBalancer.enter(hold_pose=True)), statt sie in die Standpose zu ziehen.
"""
import math
from dataclasses import dataclass

import numpy as np

# Bewaehrte Bein-Config (1:1 aus dem fruehstabilen g1.yaml). Reihenfolge = Unitree-
# Motorindizes 0..11 (je Bein: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll).
LEG_IDX = np.arange(12)
LEG_KP = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], np.float32)
LEG_KD = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], np.float32)
LEG_STAND_POSE = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
                           -0.1, 0.0, 0.0, 0.3, -0.2, 0.0], np.float32)
WAIST_IDX = np.array([12, 13, 14])                 # Taille: yaw, roll, pitch
WAIST_KP = np.array([300, 300, 300], np.float32)
WAIST_KD = np.array([3, 3, 3], np.float32)
WAIST_TARGET = np.zeros(3, np.float32)             # aufrecht

# Indizes im 12er-Bein-Array:
L_HIP_PITCH, L_HIP_ROLL, L_HIP_YAW, L_ANKLE_PITCH, L_ANKLE_ROLL = 0, 1, 2, 4, 5
R_HIP_PITCH, R_HIP_ROLL, R_HIP_YAW, R_ANKLE_PITCH, R_ANKLE_ROLL = 6, 7, 8, 10, 11


@dataclass
class BalancerParams:
    """Gains des Balancers (in loco_sim als ROS-Parameter live tunebar)."""
    kp_scale: float = 10.0          # Posture-Steifigkeit (Haupthebel)
    ramp_s: float = 0.4             # weicher Eintritt aus WALK
    ki_pitch: float = 80.0          # Integral-Trim Knoechel-Pitch [Nm/(rad*s)], 0 = aus
    i_limit: float = 25.0           # Anti-Windup [Nm]
    ankle_kp_pitch: float = 150.0
    ankle_kd_pitch: float = 40.0
    ankle_kp_roll: float = 150.0
    ankle_kd_roll: float = 40.0
    ankle_tau_limit: float = 50.0
    hip_kp_pitch: float = 200.0
    hip_kd_pitch: float = 40.0
    hip_kp_roll: float = 200.0
    hip_kd_roll: float = 40.0
    hip_tau_limit: float = 80.0
    yaw_kd: float = 30.0


@dataclass
class SettleParams:
    """Wann die Policy nach einer STAND-Anforderung an den PD uebergeben darf."""
    min_s: float = 0.6        # mindestens so lange mit cmd = 0 unter der Policy
    quiet_s: float = 0.3      # so lange ununterbrochen ruhig
    gyro_max: float = 0.3     # |Gyro| [rad/s]
    dq_max: float = 0.5       # max. |dq| der Beingelenke [rad/s]
    timeout_s: float = 3.0    # spaetestens dann trotzdem uebergeben


class SettleGate:
    """Wartet nach einer STAND-Anforderung im WALK, bis die Policy den Roboter
    ruhig hingestellt hat. Ohne ROS; loco_sim und die Headless-Tests nutzen sie."""

    def __init__(self):
        self._t0 = None
        self._quiet_since = None

    @property
    def active(self):
        return self._t0 is not None

    def start(self, now):
        if self._t0 is None:
            self._t0 = now
            self._quiet_since = None

    def cancel(self):
        self._t0 = None
        self._quiet_since = None

    def update(self, now, gyro, dq_legs, p: SettleParams):
        """Gibt None (weiter warten), "ruhig" oder "timeout" zurueck. Bei einer
        Rueckgabe != None ist das Gate danach wieder inaktiv."""
        if self._t0 is None:
            return None
        quiet = (float(np.linalg.norm(gyro)) < p.gyro_max
                 and float(np.max(np.abs(dq_legs))) < p.dq_max)
        if not quiet:
            self._quiet_since = None
        elif self._quiet_since is None:
            self._quiet_since = now
        waited = now - self._t0
        result = None
        if (waited >= p.min_s and self._quiet_since is not None
                and now - self._quiet_since >= p.quiet_s):
            result = "ruhig"
        elif waited >= p.timeout_s:
            result = "timeout"
        if result is not None:
            self.cancel()
        return result


@dataclass
class BalanceCommand:
    """Ein Regeltakt: Bein-PD (+ tau) und Taillen-Halten."""
    leg_q: np.ndarray
    leg_tau: np.ndarray
    leg_kp: np.ndarray
    leg_kd: np.ndarray
    waist_q: np.ndarray
    waist_kp: np.ndarray
    waist_kd: np.ndarray


def _clamp(x, lim):
    return max(-lim, min(lim, x))


class StandBalancer:
    def __init__(self, control_dt):
        self.control_dt = float(control_dt)
        self._entering = False
        self._q_start = None
        self._t0 = 0.0
        self._ramp_eff = 0.4
        self._integ = 0.0
        self._hold_pose = False

    def enter(self, hold_pose=False):
        """Beim Wechsel nach STAND: Rampe beim naechsten step() aus der aktuellen
        Beinpose starten, Integral-Trim neu lernen.
        hold_pose=False: Beine in die Standpose LEG_STAND_POSE fuehren (Start aus HOLD).
        hold_pose=True:  die vorgefundene Beinpose halten (Uebergabe aus WALK)."""
        self._entering = True
        self._q_start = None
        self._integ = 0.0
        self._hold_pose = bool(hold_pose)

    def step(self, q_legs, gravity, gyro, now, p: BalancerParams) -> BalanceCommand:
        """q_legs: Ist-Winkel Motoren 0..11; gravity: projizierte Gravitation;
        gyro: Body-Rate; now: monotone Zeit [s] (fuer die Eintritts-Rampe)."""
        # Sanfter Eintritt: aktuelle Beinpose -> Standpose ueber ramp_s blenden,
        # damit der steife PD die Beine nicht aus der (Lauf-)Stellung reisst.
        # WICHTIG: Die Rampe nur fahren, wenn die Beine wirklich WEIT von der
        # Standpose weg sind (Eintritt aus WALK). Beim Start aus HOLD hat die
        # Bridge den Roboter gerade frisch in die Standpose gestellt -- waehrend
        # einer 0.4-s-Weich-Phase kippte der (durch die Inspire-Haende kopf-
        # lastigere) Roboter dann unaufholbar nach vorn. Headless validiert:
        # ramp 0.4 s -> Sturz bei ~2 s; ramp 0.1 s -> steht (|gx|max=0.04).
        if self._entering:
            self._q_start = np.asarray(q_legs, dtype=np.float32).copy()
            self._t0 = now
            self._entering = False
            leg_err = float(np.max(np.abs(self._q_start - LEG_STAND_POSE)))
            self._ramp_eff = p.ramp_s if leg_err > 0.15 else 0.1
        ramp_s = self._ramp_eff
        if self._q_start is not None and ramp_s > 1e-3:
            ramp = max(0.0, min(1.0, (now - self._t0) / ramp_s))
        else:
            ramp = 1.0
        kp_scale_eff = 1.0 + (p.kp_scale - 1.0) * ramp
        q_start = self._q_start if self._q_start is not None else LEG_STAND_POSE
        q_goal = q_start if self._hold_pose else LEG_STAND_POSE

        pitch_err, roll_err = float(gravity[0]), float(gravity[1])
        pitch_rate, roll_rate, yaw_rate = float(gyro[1]), float(gyro[0]), float(gyro[2])

        # Integral-Trim: langsam die STATISCHE Neigung wegintegrieren (CoM-Versatz
        # durch Haende/Payload). Nur solange der Roboter nicht am Kippen ist
        # (|gx| < 0.4), mit Anti-Windup-Klemme.
        if p.ki_pitch > 0.0 and abs(pitch_err) < 0.4:
            self._integ = _clamp(self._integ + p.ki_pitch * pitch_err * self.control_dt, p.i_limit)

        # Feedforward [Nm]. Vorzeichen aus der Gelenk-Kinematik (validiert):
        t_ankle_pitch = _clamp(p.ankle_kp_pitch * pitch_err + p.ankle_kd_pitch * pitch_rate
                               + self._integ, p.ankle_tau_limit)
        t_ankle_roll = _clamp(-(p.ankle_kp_roll * roll_err + p.ankle_kd_roll * roll_rate),
                              p.ankle_tau_limit)
        t_hip_pitch = _clamp(p.hip_kp_pitch * pitch_err + p.hip_kd_pitch * pitch_rate,
                             p.hip_tau_limit)
        t_hip_roll = _clamp(-(p.hip_kp_roll * roll_err + p.hip_kd_roll * roll_rate),
                            p.hip_tau_limit)
        t_hip_yaw = _clamp(-p.yaw_kd * yaw_rate, 40.0)

        tau = np.zeros(12, dtype=np.float32)
        tau[L_ANKLE_PITCH] = tau[R_ANKLE_PITCH] = t_ankle_pitch
        tau[L_ANKLE_ROLL] = tau[R_ANKLE_ROLL] = t_ankle_roll
        tau[L_HIP_PITCH] = tau[R_HIP_PITCH] = t_hip_pitch
        tau[L_HIP_ROLL] = tau[R_HIP_ROLL] = t_hip_roll
        tau[L_HIP_YAW] = tau[R_HIP_YAW] = t_hip_yaw
        tau *= ramp

        kd_scale = math.sqrt(max(kp_scale_eff, 1e-6))
        # Taille aufrecht, mit der Posture-Steifigkeit angesteift (Oberkoerper flopt nicht).
        return BalanceCommand(
            leg_q=(q_start * (1.0 - ramp) + q_goal * ramp).astype(np.float32),
            leg_tau=tau,
            leg_kp=LEG_KP * kp_scale_eff,
            leg_kd=LEG_KD * kd_scale,
            waist_q=WAIST_TARGET.copy(),
            waist_kp=WAIST_KP * kp_scale_eff,
            waist_kd=WAIST_KD * math.sqrt(kp_scale_eff),
        )

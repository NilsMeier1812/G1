#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stand_balancer — modellbasierter Knoechel-/Hueft-PD-Balancer fuer loco_sim (STAND),
ohne ROS-Abhaengigkeit.

Liegt ausserhalb von loco_sim, damit loco_sim und die Headless-Tests
(test_agile_walk_sim.py) denselben Code fahren statt einer Kopie.

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
     (StandBalancer.enter() ohne Ziel), statt sie in die Standpose zu ziehen.

Stand-Pose aus HOLD (START BALANCING): die Stand-Pose der Lauf-Policy (bei AGILE
die Hocke mit breiter Spur aus deploy.yaml stand_pose_legs). Die Bridge stellt
den Roboter beim Wechsel direkt in die kommandierte Beinpose
(StandBalancer.enter(goal)). So steht der PD genau dort, wo die Policy beim
Loslaufen anfaengt -> kein Anlauf-Satz bei START WALKING.

Schwerpunkt-Fuehrung: Arme, Haende und Last verschieben den Schwerpunkt. Der
reine Neigungs-Regler haelt nur das Becken aufrecht; wandert der Schwerpunkt zur
Ferse oder zu den Zehen, kippt der Roboter ueber die Fusskante (in der Policy-
Hocke liegt er schon ohne Arm-Bewegung nur 7 cm vor der Ferse). Deshalb rechnet
loco_sim den Schwerpunkt aus den Gelenkwinkeln (com_model.py, Sim-Modell) und der
Balancer schiebt das Becken langsam (Knoechel -d, Huefte +d: Oberkoerper bleibt
aufrecht), bis er ueber der Fussmitte liegt. Was das Modell nicht kennt (Box in
den Haenden), zieht eine langsame Korrektur aus dem eigenen Knoechelmoment nach
(Druckpunkt ~ tau / (m*g) vor dem Knoechel), nur im ruhigen Stand. Direkt aus
dem Moment nachregeln geht nicht: Verschiebt der Balancer das Becken, schlaegt
das Moment erst in die Gegenrichtung aus; das schaukelte sich headless auf.

Kipp-Erkennung (TipDetector): Hebt ein Fuss ab (Sohle kippt > rescue-Schwelle)
oder kippt das Becken stark, kann der PD das nicht mehr ohne Schritt halten.
loco_sim gibt dann an die Lauf-Policy ab (cmd = 0), die mit Schritten abfaengt,
und nimmt danach wieder den PD (wie bei START BALANCING im Laufen).
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
ANKLE_PITCH = np.array([L_ANKLE_PITCH, R_ANKLE_PITCH])
HIP_PITCH = np.array([L_HIP_PITCH, R_HIP_PITCH])

# Feste Pitch-Drehungen in der Beinkette des Sim-Modells (unitree_robots/g1,
# g1_29dof*.xml): hip_roll_link um -10 Grad, knee_link um +10 Grad um y gedreht.
HIP_ROLL_BODY_PITCH = -0.17453
KNEE_BODY_PITCH = 0.17453
GRAVITY = 9.81


@dataclass
class BalancerParams:
    """Gains des Balancers (in loco_sim als ROS-Parameter live tunebar)."""
    kp_scale: float = 10.0          # Posture-Steifigkeit (Haupthebel)
    ramp_s: float = 0.4             # weicher Eintritt aus WALK
    ki_pitch: float = 80.0          # Integral-Trim Knoechel-Pitch [Nm/(rad*s)], 0 = aus
    i_limit: float = 25.0           # Anti-Windup [Nm]
    ankle_kp_pitch: float = 150.0
    ankle_kd_pitch: float = 40.0
    # Roll: halb so steif wie Pitch. Mit 150/40 bzw. 200/40 schaukelte sich der
    # Roboter in der breiten Policy-Hocke seitlich auf (Inspire-Haende), die Fuesse
    # rutschten 5-20 cm. Die breite Spur stabilisiert Roll ohnehin.
    ankle_kp_roll: float = 75.0
    ankle_kd_roll: float = 20.0
    ankle_tau_limit: float = 50.0
    hip_kp_pitch: float = 200.0
    hip_kd_pitch: float = 40.0
    hip_kp_roll: float = 100.0
    hip_kd_roll: float = 20.0
    hip_tau_limit: float = 80.0
    yaw_kd: float = 30.0
    # Schwerpunkt-Fuehrung (siehe Modul-Docstring). com_tau_s = 0 -> aus.
    com_target_m: float = 0.035     # Soll-Schwerpunkt vor dem Knoechel (Fuss: -0.05..+0.12)
    com_lead_s: float = 0.25        # Vorhalt: Schwerpunkt + lead * Geschwindigkeit
    com_tau_s: float = 0.2          # Zeitkonstante der Verschiebung
    com_m_per_rad: float = 0.47     # Schwerpunkt-Weg je rad Knoechel/Huefte (Hocke)
    com_rate: float = 0.5           # max. Verschiebe-Geschwindigkeit [rad/s]
    com_limit: float = 0.2          # max. Verschiebung [rad]
    # Last-Korrektur: Was das Modell nicht kennt (Box in den Haenden), zeigt sich im
    # Knoechelmoment. Der Druckpunkt daraus (tau / (m*g)) zieht die Modell-Schaetzung
    # langsam nach. Langsam, weil das Moment beim Verschieben selbst kurz ausschlaegt.
    bias_tau_s: float = 2.0         # 0 = aus
    bias_limit: float = 0.05        # max. Korrektur [m]
    bias_filter_s: float = 0.5      # Tiefpass auf den Druckpunkt
    bias_quiet_vel: float = 0.02    # nur nachziehen, solange |Schwerpunkt-Geschw.| < [m/s]
    mass_kg: float = 35.0           # nur fuer tau -> Druckpunkt


@dataclass
class TipParams:
    """Wann der PD aufgibt und die Policy mit Schritten abfangen soll."""
    foot_tilt_deg: float = 6.0      # eine Sohle kippt mehr als das (normal < 2 Grad)
    tilt_max: float = 0.2           # |proj. Gravitation xy| des Beckens (~11.5 Grad)
    debounce_s: float = 0.04        # so lange ununterbrochen


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


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat_to_mat(q):
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def foot_tilt(quat, q_legs):
    """Neigung der beiden Fusssohlen gegen die Horizontale [rad] (links, rechts),
    aus Becken-IMU (quat w,x,y,z) und Bein-Encodern. Steht ein Fuss flach, ist sie
    ~0, egal wie das Becken steht."""
    r_pelvis = _quat_to_mat(quat)
    out = np.zeros(2)
    for k, o in enumerate((0, 6)):
        hp, hr, hy, kn, ap, ar = (float(v) for v in q_legs[o:o + 6])
        r = (r_pelvis @ _rot_y(hp) @ _rot_y(HIP_ROLL_BODY_PITCH) @ _rot_x(hr) @ _rot_z(hy)
             @ _rot_y(KNEE_BODY_PITCH) @ _rot_y(kn) @ _rot_y(ap) @ _rot_x(ar))
        out[k] = math.acos(max(-1.0, min(1.0, r[2, 2])))
    return out


class TipDetector:
    """Erkennt im PD-Stand, dass der Roboter ohne Schritt umfaellt: eine Sohle
    hebt ab (kippt ueber die Fusskante) oder das Becken kippt stark. Normaler
    Betrieb (Arme, Box bis 2 kg je Hand) bleibt unter 2 Grad Sohlen-Neigung."""

    def __init__(self):
        self._since = None

    def reset(self):
        self._since = None

    def update(self, now, quat, q_legs, gravity, p: TipParams):
        tip = (float(np.max(foot_tilt(quat, q_legs))) > math.radians(p.foot_tilt_deg)
               or math.hypot(float(gravity[0]), float(gravity[1])) > p.tilt_max)
        if not tip:
            self._since = None
            return False
        if self._since is None:
            self._since = now
        return now - self._since >= p.debounce_s - 1e-9


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
        self._q_goal = None
        self._goal = None
        self._t0 = 0.0
        self._ramp_eff = 0.4
        self._integ = 0.0
        self._last = None          # letzter Befehl (leg_q, kp, kd, tau) fuers Knoechelmoment
        self._com_prev = None
        self._com_vel = 0.0
        self.shift = 0.0           # aktuelle Becken-Verschiebung [rad]
        self.bias = 0.0            # Last-Korrektur der Schwerpunkt-Schaetzung [m]
        self._shift_prev = 0.0
        self.cop = 0.0             # Druckpunkt aus dem Knoechelmoment [m vor dem Knoechel]

    def enter(self, goal=None):
        """Beim Wechsel nach STAND: beim naechsten step() neu aufsetzen, Integral-Trim
        und Schwerpunkt-Verschiebung neu lernen.
        goal=None:  die vorgefundene Beinpose halten (Uebergabe aus WALK), weiche
                    Steifigkeits-Rampe ramp_s.
        goal=Pose:  Start aus HOLD/DAMP. Die Bridge stellt den Roboter beim Wechsel
                    in genau die kommandierte Beinpose -> sofort goal kommandieren,
                    kurze Rampe (0.1 s)."""
        self._entering = True
        self._goal = None if goal is None else np.asarray(goal, np.float32).copy()
        self._q_start = None
        self._integ = 0.0
        self._last = None
        self._com_prev = None
        self._com_vel = 0.0
        self.shift = 0.0
        self._shift_prev = 0.0
        self.bias = 0.0

    def _update_shift(self, q_legs, dq_legs, com_x, p: BalancerParams):
        """Becken-Verschiebung nachfuehren, bis der Schwerpunkt ueber der Fussmitte
        liegt (siehe Modul-Docstring). com_x: Modell-Schwerpunkt vor dem Knoechel."""
        if com_x is None or p.com_tau_s <= 0.0:
            return
        dt = self.control_dt
        # Last-Korrektur aus dem eigenen Knoechelmoment (letzter Befehl + Ist-Zustand)
        if self._last is not None and p.bias_tau_s > 0.0:
            lq, lkp, lkd, ltau = self._last
            a = ANKLE_PITCH
            tau = float(np.sum(lkp[a] * (lq[a] - q_legs[a]) - lkd[a] * dq_legs[a] + ltau[a]))
            self.cop += dt / (p.bias_filter_s + dt) * (tau / (p.mass_kg * GRAVITY) - self.cop)
            # nur im ruhigen Stand nachziehen (Schwerpunkt und Becken in Ruhe)
            if abs(self._com_vel) < p.bias_quiet_vel and abs(self.shift - self._shift_prev) < 1e-3:
                self.bias = _clamp(self.bias + dt / p.bias_tau_s * (self.cop - com_x - self.bias),
                                   p.bias_limit)
        self._shift_prev = self.shift
        com = float(com_x) + self.bias
        if self._com_prev is not None:
            v = (com - self._com_prev) / dt
            self._com_vel += 0.3 * (v - self._com_vel)
        self._com_prev = com
        err = p.com_target_m - (com + p.com_lead_s * self._com_vel)
        step = dt / p.com_tau_s * err / p.com_m_per_rad
        self.shift = _clamp(self.shift + _clamp(step, p.com_rate * dt), p.com_limit)

    def step(self, q_legs, dq_legs, gravity, gyro, now, p: BalancerParams,
             com_x=None) -> BalanceCommand:
        """q_legs/dq_legs: Ist-Winkel/-Geschwindigkeit Motoren 0..11; gravity:
        projizierte Gravitation; gyro: Body-Rate; now: monotone Zeit [s];
        com_x: Schwerpunkt vor dem Knoechel aus com_model (None = ohne
        Schwerpunkt-Fuehrung)."""
        q_legs = np.asarray(q_legs, dtype=np.float32)
        dq_legs = np.asarray(dq_legs, dtype=np.float32)
        # Eintritt: aus WALK (goal=None) die aktuelle Pose halten und die Steifigkeit
        # ueber ramp_s hochfahren, damit der steife PD nicht ruckt. Aus HOLD/DAMP
        # (goal) hat die Bridge den Roboter gerade frisch in die Pose gestellt ->
        # nur 0.1 s Rampe: waehrend einer 0.4-s-Weich-Phase kippte der (Inspire-)
        # Roboter sonst unaufholbar nach vorn (headless: ramp 0.4 s -> Sturz bei
        # ~2 s; ramp 0.1 s -> steht).
        if self._entering:
            # Mit goal steht der Roboter nach dem Bridge-Reset schon in goal: sofort
            # goal kommandieren (die Bridge liest die Reset-Pose aus diesem Befehl).
            self._q_start = (self._goal if self._goal is not None else q_legs).copy()
            self._q_goal = self._q_start
            self._t0 = now
            self._entering = False
            self._ramp_eff = 0.1 if self._goal is not None else p.ramp_s
        if self._q_start is None:            # step() ohne enter(): Pose halten
            self.enter()
            return self.step(q_legs, dq_legs, gravity, gyro, now, p, com_x)
        ramp_s = self._ramp_eff
        ramp = max(0.0, min(1.0, (now - self._t0) / ramp_s)) if ramp_s > 1e-3 else 1.0
        kp_scale_eff = 1.0 + (p.kp_scale - 1.0) * ramp

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

        # Schwerpunkt-Fuehrung: Becken so verschieben, dass der Druckpunkt in der
        # Fussmitte liegt (Knoechel -shift, Huefte +shift -> Oberkoerper aufrecht).
        self._update_shift(q_legs, dq_legs, com_x, p)
        leg_q = (self._q_start * (1.0 - ramp) + self._q_goal * ramp).astype(np.float32)
        leg_q[ANKLE_PITCH] -= self.shift
        leg_q[HIP_PITCH] += self.shift

        kd_scale = math.sqrt(max(kp_scale_eff, 1e-6))
        cmd = BalanceCommand(
            leg_q=leg_q,
            leg_tau=tau,
            leg_kp=LEG_KP * kp_scale_eff,
            leg_kd=LEG_KD * kd_scale,
            # Taille aufrecht, mit der Posture-Steifigkeit angesteift (Oberkoerper flopt nicht).
            waist_q=WAIST_TARGET.copy(),
            waist_kp=WAIST_KP * kp_scale_eff,
            waist_kd=WAIST_KD * math.sqrt(kp_scale_eff),
        )
        self._last = (cmd.leg_q, cmd.leg_kp, cmd.leg_kd, cmd.leg_tau)
        return cmd

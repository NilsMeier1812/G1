#!/usr/bin/env python3
"""
hand_jog.py — »Hand bewegen« ohne Qt/ROS: Quaternion-Helfer + ein Jog-Schritt.

Gemeinsam genutzt von der Demo-GUI (Bildschirm-Joystick, demo_gui.py) und dem
PS4-Controller (ps4_arm_teleop.py). Beide schicken kartesische Hand-Ziele im
pelvis-Frame auf /g1pilot/hand_goal/<side>; der arm_controller loest per IK.

Prinzip: Das Ziel startet an der echten Hand (TF) und wird mit der Soll-
Geschwindigkeit verschoben/gedreht, laeuft der echten Hand aber hoechstens
`lead_m` / `lead_rad` voraus. Sonst wuerde es weiterwandern, wenn der Arm
nicht folgen kann (Gelenkgrenze, Tisch), und beim Loslassen noch lange
nachfahren.
"""
import math

# Hand-Ziele im pelvis-Frame, Hand-Pose per TF (wie RViz-Marker und Demo-GUI).
JOG_FRAME = "pelvis"
JOG_TF = {"left": "left_hand_point_contact", "right": "right_hand_point_contact"}

# Tempo-Stufen bei Vollausschlag: (Verschieben m/s, Drehen rad/s).
JOG_SPEEDS = {"Langsam": (0.05, math.radians(20)),
              "Normal":  (0.15, math.radians(45)),
              "Schnell": (0.30, math.radians(90))}
JOG_SPEED_ORDER = ("Langsam", "Normal", "Schnell")

# Vorlauf = JOG_LEAD_S x Tempo, mindestens JOG_MIN_LEAD_*. Zu knapp darf er
# nicht sein: der arm_controller glaettet Ziele (ik_goal_filter_alpha), die
# Hand haengt bei hohem Tempo einige cm hinterher -- ein fester kleiner
# Vorlauf bremste sie aus.
JOG_LEAD_S = 0.5
JOG_MIN_LEAD_M = 0.04
JOG_MIN_LEAD_RAD = math.radians(15)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def qfrom_rotvec(rx, ry, rz):
    """Drehvektor (Achse * Winkel, rad) -> Quaternion (x, y, z, w)."""
    ang = math.sqrt(rx * rx + ry * ry + rz * rz)
    if ang < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(ang / 2) / ang
    return (rx * s, ry * s, rz * s, math.cos(ang / 2))


def qangle(a, b):
    """Drehwinkel zwischen zwei Orientierungen (rad)."""
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, d))


def qslerp(a, b, t):
    d = sum(x * y for x, y in zip(a, b))
    if d < 0.0:
        b, d = tuple(-x for x in b), -d
    if d > 0.9995:
        q = tuple(x + t * (y - x) for x, y in zip(a, b))
    else:
        th = math.acos(d)
        sa, sb = math.sin((1 - t) * th), math.sin(t * th)
        q = tuple((sa * x + sb * y) / math.sin(th) for x, y in zip(a, b))
    n = math.sqrt(sum(x * x for x in q))
    return tuple(x / n for x in q)


def lead_limits(lin_speed, ang_speed):
    """-> (lead_m, lead_rad): wie weit das Ziel der Hand vorauslaufen darf."""
    return (max(JOG_MIN_LEAD_M, lin_speed * JOG_LEAD_S),
            max(JOG_MIN_LEAD_RAD, ang_speed * JOG_LEAD_S))


def jog_step(target, hand, lin, ang, dt, lead_m, lead_rad):
    """Einen Jog-Schritt rechnen.

    target : [pos(list3), quat(tuple4)] bisheriges Ziel (oder None -> Start an der Hand)
    hand   : (pos, quat) echte Hand im JOG_FRAME
    lin    : (vx, vy, vz) m/s im JOG_FRAME
    ang    : (wx, wy, wz) rad/s um feste JOG_FRAME-Achsen, Drehpunkt = Hand
    -> neues Ziel [pos, quat]
    """
    pos, rot = hand
    if target is None:
        target = [list(pos), tuple(rot)]
    # Verschieben, Vorlauf zur echten Hand begrenzen
    p = [target[0][i] + lin[i] * dt for i in range(3)]
    d = [p[i] - pos[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in d))
    if n > lead_m:
        p = [pos[i] + d[i] * lead_m / n for i in range(3)]
    # Drehen (feste Achsen -> von links multiplizieren), Vorlauf begrenzen
    dq = qfrom_rotvec(*(w * dt for w in ang))
    q = qmul(dq, target[1])
    err = qangle(q, rot)
    if err > lead_rad:
        q = qslerp(rot, q, lead_rad / err)
    return [p, q]

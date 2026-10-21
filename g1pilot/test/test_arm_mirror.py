#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Arm spiegeln (Controller: Steuerkreuz links/rechts + Kreuz halten).

Belegt, dass die Spiegelung in GELENKWINKELN (arm_command.mirror_arm_joints:
Pitch/Ellbogen gleich, Roll/Yaw mit umgekehrtem Vorzeichen) GENAU der an der
Sagittalebene (x-z) gespiegelten Hand-Pose entspricht -- Position (x, -y, z),
Orientierung S*R*S mit S = diag(1, -1, 1) -- und zwar am URDF selbst
(Vorwaertskinematik, pinocchio), fuer zufaellige Posen im ganzen Gelenkraum
und fuer jedes Glied der Kette (Schulter, Ellbogen, Handgelenk, Greifpunkt).
Dazu das Wire-Format {"type": "mirror"} (ROS-frei).
"""
import os

import numpy as np
import pytest

from g1pilot.manipulation import arm_command as ac
from g1pilot.utils.joints_names import (
    JOINT_LIMITS_RAD, LEFT_JOINT_INDICES_LIST, RIGHT_JOINT_INDICES_LIST,
)

URDF_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "description_files", "urdf")
JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
          "wrist_roll", "wrist_pitch", "wrist_yaw")
FRAMES = ("shoulder_roll_link", "elbow_link", "wrist_yaw_link", "hand_point_contact")
S = np.diag([1.0, -1.0, 1.0])


# ── ROS-frei: Vorzeichen, Limits, Format ──────────────────────────────────
def test_mirror_joints_signs_and_involution():
    q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    m = ac.mirror_arm_joints(q)
    assert m == [0.1, -0.2, -0.3, 0.4, -0.5, 0.6, -0.7]
    assert ac.mirror_arm_joints(m) == q
    with pytest.raises(ac.CommandError):
        ac.mirror_arm_joints([0.0] * 6)
    with pytest.raises(ac.CommandError):
        ac.mirror_arm_joints([float("nan")] + [0.0] * 6)


def test_joint_limits_are_mirrored():
    """Jede gueltige Pose des einen Arms bleibt gespiegelt gueltig."""
    for i, (jl, jr) in enumerate(zip(LEFT_JOINT_INDICES_LIST, RIGHT_JOINT_INDICES_LIST)):
        lo_l, hi_l = JOINT_LIMITS_RAD[jl]
        lo_r, hi_r = JOINT_LIMITS_RAD[jr]
        if ac.MIRROR_SIGNS[i] > 0:
            assert (lo_l, hi_l) == (lo_r, hi_r)
        else:
            assert (lo_l, hi_l) == (-hi_r, -lo_r)


def test_parse_mirror_command():
    cmd = ac.parse_command('{"id": "m1", "type": "mirror", "side": "left"}')
    assert cmd["type"] == ac.TYPE_MIRROR and cmd["sides"] == ["left"]
    assert cmd["targets"] == {"left": {"mirror_of": "right"}} and cmd["id"] == "m1"
    cmd = ac.parse_command({"type": "mirror", "side": "right"})
    assert cmd["targets"] == {"right": {"mirror_of": "left"}} and cmd["id"]
    for bad in ({"type": "mirror"}, {"type": "mirror", "side": "both"},
                {"type": "mirror", "side": "left", "left": [0.0] * 7}):
        with pytest.raises(ac.CommandError):
            ac.parse_command(bad)


# ── Am URDF: Gelenk-Spiegelung == Spiegelung der Hand-Pose ───────────────
@pytest.mark.parametrize("urdf", ["g1_29dof_inspire_ftp.urdf", "29dof.urdf"])
def test_joint_mirror_equals_cartesian_mirror_fk(urdf):
    pin = pytest.importorskip("pinocchio")
    model = pin.buildModelFromUrdf(os.path.join(URDF_DIR, urdf))
    data = model.createData()
    idx = {s: [model.idx_qs[model.getJointId(f"{s}_{j}_joint")] for j in JOINTS]
           for s in ("left", "right")}
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    rng = np.random.default_rng(1812)
    worst_p = worst_r = 0.0
    for src, dst in (("right", "left"), ("left", "right")):
        for _ in range(400):
            q = pin.neutral(model)
            q_src = [rng.uniform(lo[i], hi[i]) for i in idx[src]]
            for i, v in zip(idx[src], q_src):
                q[i] = v
            for i, v in zip(idx[dst], ac.mirror_arm_joints(q_src)):
                assert lo[i] - 1e-9 <= v <= hi[i] + 1e-9
                q[i] = v
            pin.framesForwardKinematics(model, data, q)
            for f in FRAMES:
                Ts = data.oMf[model.getFrameId(f"{src}_{f}")]
                Td = data.oMf[model.getFrameId(f"{dst}_{f}")]
                worst_p = max(worst_p, float(np.linalg.norm(Td.translation - S @ Ts.translation)))
                worst_r = max(worst_r, float(np.linalg.norm(
                    pin.log3(Td.rotation.T @ (S @ Ts.rotation @ S)))))
    # Rest = Rundung der URDF-Zahlen (0.01 mm), Drehung exakt.
    assert worst_p < 5e-5, worst_p
    assert worst_r < 1e-9, worst_r


def test_mirror_is_about_torso_when_waist_turned():
    """Bei verdrehter Taille gilt die Spiegelung im TORSO-Frame (die Arme
    haengen am Oberkoerper) -- im pelvis-Frame nur bei gerader Taille, wie sie
    die Taillen-Haltung im Greifen-Modus haelt."""
    pin = pytest.importorskip("pinocchio")
    model = pin.buildModelFromUrdf(os.path.join(URDF_DIR, "g1_29dof_inspire_ftp.urdf"))
    data = model.createData()
    q = pin.neutral(model)
    q_r = [-0.6, -0.4, 0.3, 0.9, 0.2, -0.3, 0.4]
    for j, v in zip(JOINTS, q_r):
        q[model.idx_qs[model.getJointId(f"right_{j}_joint")]] = v
    for j, v in zip(JOINTS, ac.mirror_arm_joints(q_r)):
        q[model.idx_qs[model.getJointId(f"left_{j}_joint")]] = v
    q[model.idx_qs[model.getJointId("waist_yaw_joint")]] = 0.3
    pin.framesForwardKinematics(model, data, q)
    torso = data.oMf[model.getFrameId("torso_link")]
    Tl = torso.actInv(data.oMf[model.getFrameId("left_hand_point_contact")])
    Tr = torso.actInv(data.oMf[model.getFrameId("right_hand_point_contact")])
    assert np.linalg.norm(Tl.translation - S @ Tr.translation) < 5e-5
    # im pelvis-Frame weicht es dann ab (cm-Bereich)
    Pl = data.oMf[model.getFrameId("left_hand_point_contact")].translation
    Pr = data.oMf[model.getFrameId("right_hand_point_contact")].translation
    assert np.linalg.norm(Pl - S @ Pr) > 0.05

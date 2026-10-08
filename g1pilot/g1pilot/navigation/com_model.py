#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
com_model — Schwerpunkt des G1 relativ zu den Fuessen, aus den Gelenkwinkeln.

Fuer die Schwerpunkt-Fuehrung des Stand-Balancers (stand_balancer.py) in der
MuJoCo-Sim. Laedt dasselbe MJCF-Modell, das die Sim simuliert
(unitree_robots/g1/g1_29dof.xml bzw. g1_29dof_inspire_ftp.xml), mit Pinocchio
(buildModelFromMJCF, ab Pinocchio 3). Massen und Schwerpunkte stimmen damit genau
mit der Sim ueberein; die URDFs unter description_files weichen um bis zu 2 cm ab
(andere Massen der Haende/des Rumpfs) und taugen dafuer nicht.

Gerechnet wird ohne IMU: Basis = Becken im Ursprung, Gelenke aus den Encodern,
Ergebnis im Koordinatensystem des linken Fusses (ankle_roll_link) relativ zur
Mitte der beiden Knoechel. Steht der Fuss flach, ist x die horizontale Lage des
Schwerpunkts vor dem Knoechel.
"""
import numpy as np

from g1pilot.utils.joints_names import JOINT_NAMES_ROS


class ComModel:
    def __init__(self, mjcf_path):
        import pinocchio as pin
        self._pin = pin
        self.model = pin.buildModelFromMJCF(mjcf_path)
        self.data = self.model.createData()
        self.mass = float(pin.computeTotalMass(self.model))
        self.q = pin.neutral(self.model)
        # Unitree-Motorindex i -> Index in q (ueber den Gelenknamen)
        self._qidx = []
        for i in range(29):
            name = JOINT_NAMES_ROS[i]
            if not self.model.existJointName(name):
                raise ValueError(f"Gelenk {name} fehlt im Modell {mjcf_path}")
            self._qidx.append(self.model.joints[self.model.getJointId(name)].idx_q)
        self._qidx = np.array(self._qidx)
        self._lf = self.model.getFrameId("left_ankle_roll_link")
        self._rf = self.model.getFrameId("right_ankle_roll_link")

    def com_in_foot(self, q29):
        """Schwerpunkt [m] im linken Fuss-Frame, relativ zur Knoechel-Mitte."""
        pin = self._pin
        self.q[self._qidx] = np.asarray(q29, dtype=float)[:29]
        pin.centerOfMass(self.model, self.data, self.q)
        pin.framesForwardKinematics(self.model, self.data, self.q)
        lf = self.data.oMf[self._lf]
        rf = self.data.oMf[self._rf]
        mid = 0.5 * (lf.translation + rf.translation)
        return lf.rotation.T @ (self.data.com[0] - mid)

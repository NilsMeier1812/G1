#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_planner.py — Bahnplaner fuer einen oder beide Arme (7 bzw. 14 DOF).

Konzept (Vorgabe Nils, 2026-10-12):
  * In der MITTE einer Bewegung haelt der Arm Abstand zu Objekten.
  * Nahe am ZIEL darf er dicht heran, ganz am Ende sogar beruehren (Hand auf
    den Tisch legen). Ebenso am START (z.B. Vor-Pose dicht am Tisch).
  * Die Bahn bleibt dabei ein glatter Bogen -- NICHT "erst weit weg, dann in
    die Naehe des Ziels, dann gerade Linie zum Endpunkt".

Umsetzung:
  1. Abstandsbedarf je Koerperpunkt und Ort statt fester Marge. Jede Kugel des
     Armmodells (arm_geometry.py) braucht zu jedem Objekt den Abstand
         req = floor + w * (clearance - floor),
         w   = smoothstep( min(|c - c_start|, |c - c_ziel|) / relax_radius ),
     c = Kugelmitte, c_start/c_ziel = dieselbe Kugel in Start-/Zielpose. Eine
     Kugel, die gerade an ihrer Start- oder Zielposition ist, darf also bis
     auf `floor` heran (0 = Beruehrung; steckt die Pose selbst schon minimal im
     Objekt -- Modellungenauigkeit, Roboter naeher herangekommen --, so tief wie
     dort); mit wachsendem Abstand von beiden Enden steigt der Bedarf glatt auf
     `clearance` (Default 5 cm). Kugeln, die sich kaum bewegen (Oberarm), bleiben
     entspannt; die Hand muss in der Mitte Abstand halten. Der Bedarf ist ein
     glatter "Trichter" um Start und Ziel -- daraus entsteht von selbst ein
     weicher Abhebe- und Anfahrbogen statt Knick + Gerade.
  2. Bahnoptimierung (CHOMP-artig): ~24 Stuetzpunkte im Gelenkraum, Kosten =
     Glattheit (Geschwindigkeit + Beschleunigung) + quadratische Strafe auf
     (req - Abstand) + Anfahrrichtung. Gradient analytisch ueber das
     Kugelmodell, Schritt mit der inversen Glattheitsmatrix vorkonditioniert
     (verteilt jede Korrektur glatt ueber die ganze Bahn).
  3. Startbahnen der Reihe nach: Gerade im Gelenkraum, "Hand anheben"-Bogen,
     "Hand zurueckziehen"-Bogen; haengt der Optimierer damit fest (Hindernis
     mitten im Weg), liefert ein eigener RRT-Connect (vektorisierte Kanten-
     pruefung) die Vorlage, die danach geglaettet wird. Geht der volle Abstand
     nicht, wird er stufenweise bis `min_clearance` reduziert; als letzte
     Startbahn ueber die Lauf-Pose.
  4. Anfahrrichtung: liegt das Ziel der Hand nahe an Objekten, soll sie in den
     letzten Zentimetern entlang der Handrichtung (Finger voraus) ankommen,
     wenn diese Vor-Pose frei ist, sonst von oben. Dasselbe beim Abheben vom
     Start (zurueck entgegen der Handrichtung bzw. nach oben).
  5. Nachpruefung dicht abgetastet: Kugel-Abstaende gegen den Bedarf und die
     exakte Mesh-Selbstkollision (coal, ik_solver.arm_command_in_collision).
  6. Zeitparametrierung: Gelenk- und kartesisches Tempolimit, Beschleunigungs-
     limit, langsamer nahe an Objekten (sanftes Aufsetzen).

Greif-Objekte (Klasse grasp) bleiben fuer die HAND bewusst beruehrbar (ACM,
keine Unterscheidung nach Zielobjekt); Arm/Ellbogen halten auch zu ihnen
Abstand.

Laeuft im eigenen Prozess (arm_planner_process.py), nicht im 250-Hz-Regel-
prozess -- reines numpy, das GIL des Controllers bleibt frei.
"""
import math
import time

import numpy as np

from g1pilot.manipulation.arm_geometry import HAND_LINK_MIN, _norm
from g1pilot.utils.joints_names import LEFT_JOINT_INDICES_LIST, RIGHT_JOINT_INDICES_LIST




def _now():
    """Planer-Uhr fuer das Zeitbudget: CPU-Zeit des Prozesses. Auf einem
    ausgelasteten Rechner bekommt der Planer so dieselbe Rechenmenge wie auf
    einem freien (statt bei Wandzeit-Budget unter Last zu scheitern); eine
    Wandzeit-Obergrenze (3x Budget) setzt plan_arms zusaetzlich."""
    return time.process_time()

DEFAULTS = {
    # Abstand [m] Kugeloberflaeche <-> Objekt in der Mitte der Bewegung.
    "clearance": 0.05,
    # Kleinster Abstand, auf den der Planer zurueckgeht, wenn der volle nicht
    # geht (enge Stellen). Darunter: Fehlschlag statt Streifen.
    "min_clearance": 0.015,
    # Abstand Arm <-> eigener Koerper / anderer Arm in der Mitte [m].
    "self_clearance": 0.015,
    # In diesem Abstand [m] von der Start-/Zielposition eines Koerperpunkts
    # ist der Abstandsbedarf auf den Endwert abgeklungen.
    "relax_radius": 0.15,
    # Anfahr-/Abhebeweg [m], auf dem die Hand der Anfahrrichtung folgt.
    "approach_len": 0.10,
    "n_waypoints": 24,          # innere Stuetzpunkte der Bahn
    "time_budget_s": 2.5,       # Gesamtbudget inkl. Rueckfallebenen
    "max_iters": 120,           # Optimierer-Iterationen je Versuch
    "tol": 0.004,               # erlaubte Restverletzung des Abstandsbedarfs [m]
    # Zeitparametrierung
    "vel_limit": 1.2,           # rad/s je Gelenk (Controller-Limit 1.5)
    "acc_limit": 2.0,           # rad/s^2
    "cart_vel_limit": 0.22,     # m/s jeder Kugel/TCP (Controller-Limit 0.25)
    "cart_acc_limit": 0.6,      # m/s^2
    "slow_dist": 0.05,          # unter diesem Abstand langsamer ...
    "slow_factor": 0.35,        # ... bis auf diesen Anteil bei Beruehrung
    "sample_dt": 0.02,          # Abtastung der ausgegebenen Trajektorie [s]
}

# Gewichte der Kostenterme (rad bzw. m, siehe _Problem.cost).
W_VEL = 1.0
W_ACC = 4.0
# Weglaenge der Hand (TCP) -- haelt die Handbahn gestreckt, statt dass ein
# im Gelenkraum glatter Weg die Hand in weitem Bogen seitlich ausschwenkt.
W_CART = 10.0
W_COLL = 4000.0
W_APP = 400.0


def _smoothstep(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def _dsmoothstep(u):
    inside = (u > 0.0) & (u < 1.0)
    return np.where(inside, 6.0 * u * (1.0 - u), 0.0)


def joint_indices_for(sides):
    idx = []
    for s in sides:
        idx += list(LEFT_JOINT_INDICES_LIST if s == "left" else RIGHT_JOINT_INDICES_LIST)
    return idx


# ══════════════════════════════════════════════════════════════════════
# Problem: Kosten, Gradient, Pruefung
# ══════════════════════════════════════════════════════════════════════
class _Problem:
    def __init__(self, geom, sides, q_start, q_goal, limits, prm):
        self.g = geom
        self.sides = list(sides)
        self.qs = np.asarray(q_start, dtype=float)
        self.qg = np.asarray(q_goal, dtype=float)
        self.D = len(self.qs)
        lim = np.asarray(limits, dtype=float)
        # Start/Ziel koennen minimal jenseits der Limits liegen (Messrauschen).
        self.lo = np.minimum(lim[:, 0], np.minimum(self.qs, self.qg))
        self.hi = np.maximum(lim[:, 1], np.maximum(self.qs, self.qg))
        self.prm = prm
        self.stop = lambda: False      # Abbruch von aussen (neues Ziel, E-Stop)
        self.w_coll = W_COLL
        self.R = float(prm["relax_radius"])
        self.self_clear = float(prm["self_clearance"])
        self.clearance = float(prm["clearance"])
        env = geom.env
        # Masken je Seite: (S, N) welche Kugel-Objekt-Paare zaehlen.
        self.radii = {s: geom.radii(s) for s in self.sides}
        self.links = {s: geom.links(s) for s in self.sides}
        self.env_idx = {s: geom.env_idx(s) for s in self.sides}
        self.self_idx = {s: geom.self_idx(s) for s in self.sides}
        self.env_mask = {}
        for s in self.sides:
            hand = self.links[s][self.env_idx[s]] >= HAND_LINK_MIN
            self.env_mask[s] = ~(hand[:, None] & env.is_grasp[None, :])

        # Endpunkte: Kugelmitten und Abstaende (-> floor).
        E = np.stack([self.qs, self.qg])
        fk = geom.fk(E, self.sides)
        self.c_end = {s: fk[s]["c"] for s in self.sides}          # (2,S,3)
        self.tcp_end = {s: fk[s]["tcp"] for s in self.sides}      # (2,3)
        self.R6_end = {s: fk[s]["R6"] for s in self.sides}
        self.floor_env, self.floor_body = {}, {}
        for s in self.sides:
            ei = self.env_idx[s]
            d, _ = geom.env_dist(fk[s]["c"][:, ei], self.radii[s][ei])
            self.floor_env[s] = np.minimum(0.0, d)                  # (2,S,N)
            si = self.self_idx[s]
            d, _ = geom.self_body_dist(fk[s]["c"][:, si], self.radii[s][si])
            self.floor_body[s] = np.minimum(0.0, d)
        self.floor_aa = None
        if len(self.sides) == 2:
            l, r = self.sides
            il, ir = self.self_idx[l], self.self_idx[r]
            d, _ = geom.pair_dist(fk[l]["c"][:, il], self.radii[l][il],
                                  fk[r]["c"][:, ir], self.radii[r][ir])
            self.floor_aa = np.minimum(0.0, d)
        self.end_env_d = {}
        for s in self.sides:
            ei = self.env_idx[s]
            d, _ = geom.env_dist(fk[s]["c"][:, ei], self.radii[s][ei])
            d = np.where(self.env_mask[s][None], d, np.inf)
            self.end_env_d[s] = d.min(axis=(1, 2)) if d.size else np.array([np.inf, np.inf])
        self.approach = {s: self._approach_dirs(s) for s in self.sides}

        # Glattheitsmatrix fuer K innere Stuetzpunkte.
        self.K = int(prm["n_waypoints"])
        self._build_smoothness(self.K)

    # ------------------------------------------------------------------
    def _approach_dirs(self, side):
        """(e_goal, e_start): Richtung, in der die Hand am Ziel ankommt bzw.
        den Start verlaesst -- oder None, wenn das Ende frei liegt."""
        g, L = self.g, float(self.prm["approach_len"])
        ei = self.env_idx[side]
        hand_rel = self.links[side][ei] >= HAND_LINK_MIN
        hand = ei[hand_rel]
        out = []
        for end in (1, 0):
            d_end = self.end_env_d[side][end]
            if not np.isfinite(d_end) or d_end >= self.clearance:
                out.append(None)
                continue
            axis = self.R6_end[side][end] @ g.hand_axis_local
            # Ziel: Hand kommt entlang der Fingerrichtung an (Vor-Pose dahinter),
            # sonst von oben. Start: zieht entgegen der Fingerrichtung zurueck,
            # sonst nach oben.
            cands = ([axis, np.array([0.0, 0.0, -1.0])] if end == 1
                     else [-axis, np.array([0.0, 0.0, 1.0])])
            chosen = None
            for e in cands:
                if end == 1:
                    c = self.c_end[side][end][hand] - L * e
                else:
                    c = self.c_end[side][end][hand] + L * e
                d, _ = g.env_dist(c, self.radii[side][hand])
                if d.size:
                    d = np.where(self.env_mask[side][hand_rel], d, np.inf).min()
                else:
                    d = np.inf
                # Vor-Pose deutlich freier als die Endpose -> Richtung taugt.
                if d >= min(self.clearance, d_end + 0.5 * L):
                    chosen = e / np.linalg.norm(e)
                    break
            out.append(chosen)
        return {"goal": out[0], "start": out[1]}

    def _build_smoothness(self, K):
        T = K + 2
        # Geschwindigkeit: V (T-1, T), Beschleunigung: A (T-2, T)
        V = np.zeros((T - 1, T))
        for i in range(T - 1):
            V[i, i], V[i, i + 1] = -1.0, 1.0
        A = np.zeros((T - 2, T))
        for i in range(T - 2):
            A[i, i], A[i, i + 1], A[i, i + 2] = 1.0, -2.0, 1.0
        # Skaliert auf die Stuetzpunktzahl, damit die Gewichte unabhaengig von K sind.
        H = W_VEL * (T - 1) * V.T @ V + W_ACC * (T - 1) ** 3 * A.T @ A
        self.H_full = H
        Hi = H[1:-1, 1:-1]
        self.M = np.linalg.inv(Hi)
        self.M /= np.abs(self.M).max()

    def full(self, Xi):
        return np.vstack([self.qs, Xi, self.qg])

    # ------------------------------------------------------------------
    def requirement(self, side, C, floor, idx=None):
        """Abstandsbedarf (T,S',X) und d req/d c (T,S',X,3) fuer die Kugeln
        C (T,S',3); floor (2,S',X) = Endwerte an Start/Ziel."""
        ce = self.c_end[side] if idx is None else self.c_end[side][:, idx]
        vs = C - ce[0]
        vg = C - ce[1]
        a_s = _norm(vs)
        a_g = _norm(vg)
        near_s = a_s <= a_g
        a = np.where(near_s, a_s, a_g)
        u = a / self.R
        w = _smoothstep(u)
        fl = np.where(near_s[..., None], floor[0][None], floor[1][None])
        return w, fl, u, np.where(near_s[..., None], vs, vg), a

    def _collision(self, fk, clear, grad):
        """Kollisions-Terme fuer die Konfigurationen in fk (T Zeilen).
        -> (Strafkosten, Verletzung je Zeile (T,), Kraefte je Seite (T,S,3))."""
        T = next(iter(fk.values()))["c"].shape[0]
        f = 0.0
        viol = np.full(T, -np.inf)
        forces = {}

        def hinge(v, F_target, idx, nrm, dreq_scale, u, vend, a):
            # Strafe und Kraefte eines Paar-Blocks (T,S',X): d(req-d)/dc =
            # dreq/dc - n; dreq/dc = (clear-floor) * w'(u)/R * Richtung vom Ende.
            nonlocal f, viol
            vp = np.maximum(v, 0.0)
            viol = np.maximum(viol, v.reshape(T, -1).max(axis=1))
            f += self.w_coll * float(np.sum(vp ** 2))
            if grad:
                coef = 2.0 * self.w_coll * vp
                dw = _dsmoothstep(u) / self.R
                unit = vend / np.maximum(a, 1e-9)[..., None]
                Fb = np.einsum("tsn,tsn->ts", coef, dreq_scale)[..., None] * dw[..., None] * unit
                Fb -= np.einsum("tsn,tsni->tsi", coef, nrm)
                F_target[:, idx] += Fb

        for s in self.sides:
            C = fk[s]["c"]                      # (T,S,3)
            r = self.radii[s]
            F = np.zeros_like(C) if grad else None
            # --- Umgebung ---
            if self.g.env.n:
                ei = self.env_idx[s]
                Ce = C[:, ei]
                d, nrm = self.g.env_dist(Ce, r[ei])
                w, fl, u, vend, a = self.requirement(s, Ce, self.floor_env[s], ei)
                v = np.where(self.env_mask[s][None], fl + w[..., None] * (clear - fl) - d, -1.0)
                hinge(v, F, ei, nrm, clear - fl, u, vend, a)
            # --- Koerper ---
            si = self.self_idx[s]
            if len(si) and len(self.g.body_r):
                Cs = C[:, si]
                d, nrm = self.g.self_body_dist(Cs, r[si])
                w, fl, u, vend, a = self.requirement(s, Cs, self.floor_body[s], si)
                v = fl + w[..., None] * (self.self_clear - fl) - d
                hinge(v, F, si, nrm, self.self_clear - fl, u, vend, a)
            forces[s] = F
        # --- Arm gegen Arm ---
        if self.floor_aa is not None:
            l, rr = self.sides
            il, ir = self.self_idx[l], self.self_idx[rr]
            Cl, Cr = fk[l]["c"][:, il], fk[rr]["c"][:, ir]
            d, nrm = self.g.pair_dist(Cl, self.radii[l][il], Cr, self.radii[rr][ir])
            zl, zr = np.zeros((2, len(il), 1)), np.zeros((2, len(ir), 1))
            wl, _, _, vl, al = self.requirement(l, Cl, zl, il)
            wr, _, _, vr, ar = self.requirement(rr, Cr, zr, ir)
            # Paar entspannt, sobald EINE der beiden Kugeln nahe ihrem Ende ist;
            # floor je Paar = Wert an dem Ende, an dem diese Kugel gerade ist.
            use_l = wl[:, :, None] <= wr[:, None, :]
            w = np.where(use_l, wl[:, :, None], wr[:, None, :])
            nl = _norm(Cl - self.c_end[l][0][il]) <= _norm(Cl - self.c_end[l][1][il])
            nr = _norm(Cr - self.c_end[rr][0][ir]) <= _norm(Cr - self.c_end[rr][1][ir])
            near_s = np.where(use_l, nl[:, :, None], nr[:, None, :])
            fl = np.where(near_s, self.floor_aa[0][None], self.floor_aa[1][None])
            v = fl + w * (self.self_clear - fl) - d
            vp = np.maximum(v, 0.0)
            viol = np.maximum(viol, v.reshape(T, -1).max(axis=1))
            f += self.w_coll * float(np.sum(vp ** 2))
            if grad:
                # Ableitung des Bedarfs vernachlaessigt (nur Abstossung) -- die
                # Arm-Arm-Paare sind selten aktiv.
                coef = 2.0 * self.w_coll * vp
                forces[l][:, il] -= np.einsum("tab,tabi->tai", coef, nrm)
                forces[rr][:, ir] += np.einsum("tab,tabi->tbi", coef, nrm)
        return f, viol, forces

    def row_violation(self, Q, clearance):
        """Verletzung des Abstandsbedarfs je Konfiguration (Zeile von Q)."""
        fk = self.g.fk(Q, self.sides)
        return self._collision(fk, clearance, False)[1]

    def violation(self, Q, clearance):
        """Groesste Verletzung des Abstandsbedarfs entlang Q (inkl. Enden)."""
        return float(self.row_violation(Q, clearance).max())

    def cost(self, Xi, grad=True, clearance=None):
        clear = self.clearance if clearance is None else clearance
        Q = self.full(Xi)
        T = Q.shape[0]
        # --- Glattheit ---
        HQ = self.H_full @ Q
        f = 0.5 * float(np.sum(Q * HQ))
        G = HQ.copy() if grad else None
        fk = self.g.fk(Q, self.sides)
        fc, viol, forces = self._collision(fk, clear, grad)
        f += fc
        viol_max = float(viol.max())
        # --- Handweg + Anfahr-/Abhebe-Richtung (TCP) ---
        L = float(self.prm["approach_len"])
        Ftcp = {}
        for s in self.sides:
            P = fk[s]["tcp"]                        # (T,3)
            dP = np.diff(P, axis=0)
            f += W_CART * (T - 1) * float(np.sum(dP ** 2))
            Ft = np.zeros_like(P)
            if grad:
                Ft[:-1] -= 2.0 * W_CART * (T - 1) * dP
                Ft[1:] += 2.0 * W_CART * (T - 1) * dP
            for key, end in (("goal", 1), ("start", 0)):
                e = self.approach[s][key]
                if e is None:
                    continue
                v = P - self.tcp_end[s][end]
                dist = np.linalg.norm(v, axis=-1)
                om = 1.0 - _smoothstep(dist / L)
                dom = -_dsmoothstep(dist / L) / L
                # Ziel: Hand kommt aus Richtung -e (v = -s*e, s>=0);
                # Start: verlaesst in Richtung +e (v = s*e).
                ee = -e if end == 1 else e
                along = v @ ee
                perp = v - along[:, None] * ee
                back = np.minimum(along, 0.0)
                h = np.sum(perp ** 2, axis=-1) + back ** 2
                f += W_APP * float(np.sum(om * h))
                if grad:
                    dh = 2.0 * perp - 2.0 * (perp @ ee)[:, None] * ee + 2.0 * back[:, None] * ee
                    unit = v / np.maximum(dist, 1e-9)[:, None]
                    Ft += W_APP * (om[:, None] * dh + (dom * h)[:, None] * unit)
            Ftcp[s] = Ft
        if not grad:
            return f, viol_max
        for n, s in enumerate(self.sides):
            G[:, 7 * n:7 * n + 7] += self.g.forces_to_q(fk[s], self.links[s], forces[s], Ftcp[s])
        return f, viol_max, G[1:-1]

    # ------------------------------------------------------------------
    def optimize(self, Xi, clearance, deadline, max_iters, w0=W_COLL):
        """Vorkonditionierter Gradientenabstieg mit Vertrauensbereich: die
        Suchrichtung -M*G wird auf groessten Gelenkschritt 1 normiert, die
        Schrittweite `trust` [rad] waechst bei Erfolg und schrumpft bei
        Misserfolg (Armijo-artig: Kosten muessen sinken). Zurueck kommt die
        letzte Bahn, die den Bedarf an den Stuetzpunkten erfuellt hat, falls
        der Optimierer sie spaeter wieder verlassen hat."""
        Xi = np.clip(Xi, self.lo, self.hi)
        self.w_coll = w0
        f, viol, G = self.cost(Xi, clearance=clearance)
        feasible = Xi if viol <= self.prm["tol"] * 0.5 else None
        trust = 0.1
        escalations = 0
        hist = []
        it = 0
        for it in range(max_iters):
            if _now() > deadline or self.stop():
                break
            D = -(self.M @ G)
            dmax = float(np.abs(D).max())
            accepted = False
            if dmax > 1e-12:
                D /= dmax
                while trust > 1e-4:
                    Xn = np.clip(Xi + trust * D, self.lo, self.hi)
                    fn, vn, Gn = self.cost(Xn, clearance=clearance)
                    if fn < f:
                        accepted = True
                        break
                    trust *= 0.4
            rel = (f - fn) / max(f, 1e-9) if accepted else 0.0
            if accepted:
                Xi, f, viol, G = Xn, fn, vn, Gn
                trust = min(trust * 1.5, 0.4)
                if viol <= self.prm["tol"] * 0.5:
                    feasible = Xi
            # Haengt der Optimierer in einem lokalen Minimum fest (Arm steckt
            # weiter tief in einem Hindernis), lohnt kein weiteres Rechnen --
            # andere Startbahn bzw. RRT.
            hist.append(viol)
            if len(hist) > 25 and viol > 0.02 and viol > 0.9 * hist[-25]:
                break
            converged = (not accepted) or rel < 1e-3
            if converged:
                if viol <= self.prm["tol"] * 0.5:
                    break
                # Quadratische Strafe gleicht Rest-Verletzung gegen Glattheit
                # ab -> Gewicht erhoehen, bis der Bedarf wirklich erfuellt ist.
                if escalations >= 4:
                    break
                escalations += 1
                self.w_coll *= 5.0
                trust = 0.1
                f, viol, G = self.cost(Xi, clearance=clearance)
        if viol > self.prm["tol"] * 0.5 and feasible is not None:
            Xi = feasible
            viol = self.violation(self.full(Xi), clearance)
        return Xi, f, viol, it + 1

    # ------------------------------------------------------------------
    def densify(self, Q, max_joint_step=0.03, max_cart_step=0.012):
        """Bahn so unterteilen, dass zwischen zwei Stuetzpunkten kein Gelenk
        mehr als max_joint_step und keine Kugel mehr als max_cart_step faehrt."""
        fk = self.g.fk(Q, self.sides)
        disp = np.zeros(len(Q) - 1)
        for s in self.sides:
            c = fk[s]["c"]
            disp = np.maximum(disp, np.linalg.norm(c[1:] - c[:-1], axis=-1).max(axis=-1))
        jstep = np.abs(np.diff(Q, axis=0)).max(axis=1)
        out = [Q[0]]
        for i in range(len(Q) - 1):
            n = int(max(1, math.ceil(max(disp[i] / max_cart_step, jstep[i] / max_joint_step))))
            for k in range(1, n + 1):
                out.append(Q[i] + (Q[i + 1] - Q[i]) * (k / n))
        return np.array(out)

    def validate(self, Q, clearance, self_check=None):
        """-> (ok, Restverletzung [m], Grund). Dicht abgetastet."""
        Qd = self.densify(Q)
        viol = self.violation(Qd, clearance)
        if viol > self.prm["tol"]:
            return False, viol, "clearance"
        if np.any(Qd < self.lo - 1e-6) or np.any(Qd > self.hi + 1e-6):
            return False, viol, "joint_limits"
        if self_check is not None:
            # Exakte Mesh-Selbstkollision (hart). Steckt schon der Start drin,
            # zaehlt sie erst ab dem ersten freien Zustand.
            started_free = not self_check(Qd[0])
            for q in Qd[1:]:
                hit = self_check(q)
                if hit and started_free:
                    return False, viol, "self_collision"
                if not hit:
                    started_free = True
        return True, viol, ""

# ══════════════════════════════════════════════════════════════════════
# Startbahnen
# ══════════════════════════════════════════════════════════════════════
def _resample(Q, K):
    """Polylinie Q auf K+2 Punkte gleichmaessig nach Bogenlaenge."""
    seg = np.linalg.norm(np.diff(Q, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < 1e-9:
        return np.repeat(Q[:1], K + 2, axis=0)
    t = np.linspace(0.0, s[-1], K + 2)
    return np.array([np.interp(t, s, Q[:, j]) for j in range(Q.shape[1])]).T


def _bump_seed(prob, direction, height):
    """Gerade + glatter Bogen, der die Hand(e) in der Mitte um `height` m in
    Richtung `direction` (Welt, Becken-Frame) verschiebt (Jacobi-transponiert,
    in der Bahnmitte bestimmt)."""
    K = prob.K
    s = np.linspace(0.0, 1.0, K + 2)[1:-1]
    base = prob.qs[None] + s[:, None] * (prob.qg - prob.qs)[None]
    mid = 0.5 * (prob.qs + prob.qg)
    fk = prob.g.fk(mid[None], prob.sides)
    dq = np.zeros(prob.D)
    for n, side in enumerate(prob.sides):
        # Pseudo-Inverse der TCP-Positions-Jacobi (numerisch, 7 Spalten).
        J = np.zeros((3, 7))
        for j in range(7):
            e = np.zeros(prob.D)
            e[7 * n + j] = 1e-4
            J[:, j] = (prob.g.fk((mid + e)[None], [*prob.sides])[side]["tcp"][0]
                       - fk[side]["tcp"][0]) / 1e-4
        dq[7 * n:7 * n + 7] = np.linalg.pinv(J, rcond=0.05) @ (direction * height)
    bump = np.sin(np.pi * s)[:, None] * dq[None]
    return np.clip(base + bump, prob.lo, prob.hi)


def _via_seed(prob, q_via):
    K = prob.K
    Q = np.vstack([prob.qs, q_via, prob.qg])
    return _resample(Q, K)[1:-1]


def _rrt_seed(prob, clearance, deadline, rng=None, step=0.3, res=0.04):
    """RRT-Connect im Gelenkraum als Vorlage fuer den Optimierer, fuer enge
    Umwege (Arm muss um ein Hindernis herum, an dem die glatte Optimierung in
    einem lokalen Minimum haengt). Gueltig = Abstandsbedarf (Kugelmodell)
    erfuellt; jede Kante wird in EINEM vektorisierten Aufruf dicht geprueft.
    Die exakte Mesh-Selbstkollision prueft danach validate()."""
    rng = rng or np.random.default_rng()
    tol = prob.prm["tol"]
    lo, hi = prob.lo, prob.hi

    def edge_free(a, b):
        n = max(1, int(math.ceil(np.abs(b - a).max() / res)))
        Q = a[None] + np.linspace(0.0, 1.0, n + 1)[1:, None] * (b - a)[None]
        return bool(prob.row_violation(Q, clearance).max() <= tol)

    trees = [([prob.qs.copy()], [-1]), ([prob.qg.copy()], [-1])]

    def extend(tree, target):
        nodes, parents = tree
        arr = np.asarray(nodes)
        i = int(np.argmin(np.abs(arr - target).max(axis=1)))
        q_near = arr[i]
        delta = target - q_near
        dist = float(np.abs(delta).max())
        if dist < 1e-9:
            return "reached", i
        q_new = target if dist <= step else q_near + delta * (step / dist)
        if not edge_free(q_near, q_new):
            return "trapped", None
        nodes.append(q_new)
        parents.append(i)
        return ("reached" if dist <= step else "advanced"), len(nodes) - 1

    def branch(tree, i):
        nodes, parents = tree
        out = []
        while i != -1:
            out.append(nodes[i])
            i = parents[i]
        return out

    a_is_start = True
    A, B = trees
    while _now() < deadline and not prob.stop():
        target = lo + rng.random(prob.D) * (hi - lo)
        st, ia = extend(A, target)
        if st != "trapped":
            q_new = A[0][ia]
            while True:
                st_b, ib = extend(B, q_new)
                if st_b != "advanced":
                    break
            if st_b == "reached":
                pa = list(reversed(branch(A, ia)))
                pb = branch(B, ib)
                path = pa + pb
                if not a_is_start:
                    path = list(reversed(path))
                Q = np.array(path)
                # Abkuerzen: unnoetige Knoten entfernen (gierig von vorn).
                out, i = [Q[0]], 0
                while i < len(Q) - 1:
                    j = len(Q) - 1
                    while j > i + 1 and not edge_free(Q[i], Q[j]):
                        j -= 1
                    out.append(Q[j])
                    i = j
                return _resample(np.array(out), prob.K)[1:-1]
        A, B = B, A
        a_is_start = not a_is_start
    return None


# ══════════════════════════════════════════════════════════════════════
# Zeitparametrierung
# ══════════════════════════════════════════════════════════════════════
def time_parameterize(prob, Q):
    """Dichte Bahn Q -> (t (M,), q (M,D)) mit Gelenk-/kartesischen Tempo- und
    Beschleunigungslimits; langsamer nahe an Objekten."""
    prm = prob.prm
    Qd = prob.densify(Q, max_joint_step=0.01, max_cart_step=0.005)
    n = len(Qd)
    if n < 2:
        return np.array([0.0]), Qd
    fk = prob.g.fk(Qd, prob.sides)
    dq = np.abs(np.diff(Qd, axis=0)).max(axis=1)                 # rad je Segment
    dc = np.zeros(n - 1)
    dmin = np.full(n, np.inf)
    for s in prob.sides:
        pts = np.concatenate([fk[s]["c"], fk[s]["tcp"][:, None]], axis=1)
        dc = np.maximum(dc, np.linalg.norm(np.diff(pts, axis=0), axis=-1).max(axis=1))
        if prob.g.env.n:
            ei = prob.env_idx[s]
            d, _ = prob.g.env_dist(fk[s]["c"][:, ei], prob.radii[s][ei])
            dmin = np.minimum(dmin, d.min(axis=(1, 2)))
    eps = 1e-9
    slow = prm["slow_factor"] + (1 - prm["slow_factor"]) * _smoothstep(
        np.maximum(dmin, 0.0) / prm["slow_dist"])
    slow_seg = np.minimum(slow[:-1], slow[1:])
    # Tempo in "Segmenten pro Sekunde" je Segment
    v_lim = np.minimum(prm["vel_limit"] / np.maximum(dq, eps),
                       prm["cart_vel_limit"] / np.maximum(dc, eps)) * slow_seg
    a_lim = np.minimum(prm["acc_limit"] / np.maximum(dq, eps),
                       prm["cart_acc_limit"] / np.maximum(dc, eps))
    # Knoten-Tempo (Segmente/s) mit Vor-/Rueckwaertspass (v^2 <= v0^2 + 2 a)
    v = np.zeros(n)
    vmax_node = np.concatenate([[0.0], np.minimum(v_lim[:-1], v_lim[1:]), [0.0]])
    for i in range(1, n):
        v[i] = min(vmax_node[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_lim[i - 1]))
    v[-1] = 0.0
    for i in range(n - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_lim[i]))
    dt = 2.0 / np.maximum(v[:-1] + v[1:], eps)
    dt = np.minimum(dt, 1.0)
    t = np.concatenate([[0.0], np.cumsum(dt)])
    ts = np.arange(0.0, t[-1], prm["sample_dt"])
    ts = np.append(ts, t[-1])
    qs = np.array([np.interp(ts, t, Qd[:, j]) for j in range(Qd.shape[1])]).T
    return ts, qs


# ══════════════════════════════════════════════════════════════════════
# Oeffentliche API
# ══════════════════════════════════════════════════════════════════════
def plan_arms(geom, sides, q_start, q_goal, limits, params=None, self_check=None,
              walk_pose=None, should_stop=None):
    """Siehe _plan_arms. Rechnet nur mit den Objekten, die die bewegten Arme
    ueberhaupt erreichen koennen (ArmGeometry.reachable)."""
    env_full = geom.env
    geom.set_environment(geom.reachable(env_full, list(sides)))
    try:
        return _plan_arms(geom, sides, q_start, q_goal, limits, params, self_check,
                          walk_pose, should_stop)
    finally:
        geom.set_environment(env_full)


def _plan_arms(geom, sides, q_start, q_goal, limits, params=None, self_check=None,
               walk_pose=None, should_stop=None):
    """Plant eine glatte, abstandshaltende Bahn fuer einen oder beide Arme.

    geom      : ArmGeometry mit gesetzter Basis (set_base) und Umgebung.
    sides     : ['left'] / ['right'] / ['left','right'] (Spaltenreihenfolge).
    q_start/q_goal/limits : ueber die Seiten konkateniert (7 je Seite).
    self_check: optional f(qN)->bool, True = harte Mesh-Selbstkollision.
    walk_pose : optional dict side->q7 (Lauf-Pose) fuer die Rueckfall-Startbahn.
    should_stop: optional f()->bool; True bricht die Planung ab ('cancelled').

    Rueckgabe dict: ok, reason, waypoints (K+2,D), t, q (Trajektorie),
    clearance (tatsaechlich eingehaltener Mittelabstand), stats.
    Reasons: 'direct' | 'optimized' | 'optimized_reduced' | 'rrt_optimized' |
             'rrt_optimized_reduced' | 'rrt' | 'goal_in_collision' |
             'no_path_found' | 'cancelled'."""
    prm = dict(DEFAULTS)
    prm.update(params or {})
    t0 = time.monotonic()
    deadline = _now() + float(prm["time_budget_s"])
    wall_cap = t0 + 3.0 * float(prm["time_budget_s"])
    prob = _Problem(geom, sides, q_start, q_goal, limits, prm)
    user_stop = should_stop if should_stop is not None else (lambda: False)
    prob.stop = lambda: user_stop() or time.monotonic() > wall_cap
    stats = {"tries": []}

    def expired():
        return _now() > deadline or prob.stop()

    def result(ok, reason, Xi=None, clearance=0.0, viol=None):
        out = {"ok": ok, "reason": reason, "stats": stats,
               "plan_time_s": round(time.monotonic() - t0, 3)}
        if ok:
            Q = prob.full(Xi)
            out["waypoints"] = Q
            out["t"], out["q"] = time_parameterize(prob, Q)
            out["clearance"] = clearance
            out["violation"] = viol
            out["approach"] = {s: {k: (None if v is None else [round(float(x), 3) for x in v])
                                   for k, v in prob.approach[s].items()} for s in prob.sides}
        return out

    # Ziel/Start in harter Selbstkollision (Mesh)?
    if self_check is not None and self_check(prob.qg):
        return result(False, "goal_in_collision")
    # Ziel tiefer als 2 cm in einem Objekt -> kein sinnvolles Ziel.
    for s in prob.sides:
        fe = np.where(prob.env_mask[s][None], prob.floor_env[s], 0.0)
        if fe.size and fe[1].min() < -0.02:
            return result(False, "goal_in_collision")

    K = prob.K
    s_lin = np.linspace(0.0, 1.0, K + 2)[1:-1]
    straight = prob.qs[None] + s_lin[:, None] * (prob.qg - prob.qs)[None]

    clear_levels = [prm["clearance"]]
    c = prm["clearance"]
    while c * 0.6 > prm["min_clearance"]:
        c *= 0.6
        clear_levels.append(c)
    if clear_levels[-1] > prm["min_clearance"]:
        clear_levels.append(prm["min_clearance"])

    # 1) Gerade direkt gueltig?
    ok, viol, why = prob.validate(prob.full(straight), clear_levels[0], self_check)
    stats["tries"].append(("straight", round(viol, 4), why))
    if ok:
        return result(True, "direct", straight, clear_levels[0], viol)

    def attempt(name, Xi0, clear, budget, w0=W_COLL):
        # Optimiert mit etwas Reserve auf den Bedarf (die Stuetzpunkte sind
        # duenner als die Nachpruefung), geprueft wird mit dem echten Wert.
        Xi, f, viol, n_it = prob.optimize(Xi0, clear + 0.5 * prm["tol"],
                                          min(deadline, _now() + budget),
                                          prm["max_iters"], w0)
        ok, vv, why = prob.validate(prob.full(Xi), clear, self_check)
        stats["tries"].append((name, round(clear, 3), n_it, round(vv, 4), why))
        return ok, Xi, vv

    total = float(prm["time_budget_s"])
    best = []          # (Restverletzung, Name, Bahn)
    # 2) Optimierer mit vollem Abstand aus einfachen Startbahnen.
    seeds = [("straight", straight),
             ("lift", _bump_seed(prob, np.array([0.0, 0.0, 1.0]), 0.15)),
             ("retract", _bump_seed(prob, np.array([-1.0, 0.0, 0.5]) / math.sqrt(1.25), 0.15))]
    for k, (name, Xi0) in enumerate(seeds):
        if expired():
            break
        # Die Gerade ist fast immer die schoenste Vorlage -> groesster Anteil.
        ok, Xi, vv = attempt(name, Xi0, clear_levels[0], (0.4 if k == 0 else 0.12) * total)
        if ok:
            return result(True, "optimized", Xi, clear_levels[0], vv)
        best.append((vv, name, Xi))

    # 3) RRT-Connect als Vorlage -- findet Wege um Hindernisse, an denen der
    #    Optimierer in einem lokalen Minimum haengt. Danach geglaettet, mit
    #    dem groessten Abstand, der sich halten laesst.
    remaining = deadline - _now()
    if remaining > 0.15:
        Xi0 = _rrt_seed(prob, clear_levels[-1], _now() + 0.5 * remaining)
        if Xi0 is not None:
            # Fortsetzung: erst mit dem kleinsten Abstand glaetten (schnell
            # gueltig), dann den Abstand stufenweise erhoehen, solange es
            # geht -- jeweils warm aus der letzten gueltigen Bahn.
            found, Xi = None, Xi0
            for clear in reversed(clear_levels):
                if expired():
                    break
                ok, Xn, vv = attempt("rrt", Xi, clear, 0.15 * total, 25.0 * W_COLL)
                if not ok:
                    break
                found, Xi = (Xn, clear, vv), Xn
            if found is not None:
                Xn, clear, vv = found
                reason = "rrt_optimized" if clear == clear_levels[0] else "rrt_optimized_reduced"
                return result(True, reason, Xn, clear, vv)
            ok, vv, why = prob.validate(prob.full(Xi0), clear_levels[-1], self_check)
            if ok:
                return result(True, "rrt", Xi0, clear_levels[-1], vv)

    # 4) Weniger Abstand mit den besten bisherigen Bahnen; als letztes ueber
    #    die Lauf-Pose.
    best.sort(key=lambda b: b[0])
    cands = [(n, X) for _, n, X in best[:2]]
    if walk_pose is not None:
        via = np.concatenate([walk_pose[s] for s in prob.sides])
        cands.append(("via_walk", _via_seed(prob, via)))
    for clear in clear_levels[1:]:
        for name, Xi0 in cands:
            if expired():
                break
            ok, Xi, vv = attempt(name, Xi0, clear, 0.1 * total)
            if ok:
                return result(True, "optimized_reduced", Xi, clear, vv)
    return result(False, "cancelled" if user_stop() else "no_path_found")

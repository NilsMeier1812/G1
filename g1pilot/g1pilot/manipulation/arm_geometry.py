#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_geometry.py — Kugelmodell der G1-Arme mit ABSTANDSFUNKTION und Gradient.

Grundlage des Armplaners (arm_planner.py) und des Laufzeit-Gates im
arm_controller. Ersetzt die fruehere Ja/Nein-Pruefung mit fester 3-cm-Marge
(Punktwolke gegen Boxen) durch echte, vorzeichenbehaftete Abstaende:

  * Jeder Arm (Oberarm ab shoulder_yaw, Ellbogen, Unterarm, Handgelenk, Hand
    inkl. Finger in Neutralstellung) ist durch ~40 Kugeln angenaehert. Die
    Kugeln werden beim Start aus den Kollisions-Meshes der URDF gefittet
    (k-means auf den Mesh-Vertices, Radius = groesster Abstand eines Vertex
    zur Kugelmitte -> die Kugel umschliesst ihren Mesh-Anteil).
  * Koerper (Torso, Kopf, Becken, Huefte, Knie) ebenfalls als Kugeln; die
    Lage haengt von Beinen/Taille ab und wird je Planung EINMAL gesetzt
    (set_base) -- die Arme bewegen den Koerper nicht.
  * Umgebungs-Objekte (/scene_markers) als orientierte Boxen.

Abstaende sind Oberflaeche-zu-Oberflaeche in Metern (negativ = Eindringen),
der Gradient nach den Gelenkwinkeln kommt aus einer eigenen, VEKTORISIERTEN
Vorwaertskinematik der zwei 7-Gelenk-Ketten (numpy, viele Konfigurationen
gleichzeitig). Das ist der Grund fuer das eigene Modell statt Pinocchio je
Konfiguration: der Optimierer bewertet pro Iteration eine ganze Bahn
(~30 Stuetzpunkte x 2 Arme) in einem einzigen Satz Matrixoperationen.

Pinocchio wird NUR beim Aufbau gebraucht (Gelenk-Placements, Achsen, Mesh-
Vertices). Die exakte Mesh-Selbstkollision (coal, ik_solver.arm_command_in_
collision) bleibt die letzte Instanz: der Planer prueft das Ergebnis damit
nach, das Gate prueft sie pro Tick.
"""
import numpy as np

SIDES = ("left", "right")
ARM_JOINT_SUFFIXES = ("shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint",
                      "elbow_joint", "wrist_roll_joint", "wrist_pitch_joint",
                      "wrist_yaw_joint")
# Kettenglied ab dem eine Kugel als HAND gilt (wrist_pitch, wrist_yaw + Finger).
# Hand-Kugeln duerfen Greif-Objekte beruehren (ACM, bewusst: Nils, keine
# Unterscheidung nach Zielobjekt), alle anderen nicht.
HAND_LINK_MIN = 5
# Kettenglied ab dem eine Kugel gegen den eigenen Koerper / den anderen Arm
# geprueft wird (Ellbogen abwaerts). Schulter/Oberarm sitzen konstruktiv
# dauerhaft wenige cm am Torso (wie beim coal-Gate, _GATE_ARM_KEYS).
SELF_LINK_MIN = 3
BODY_GEOMS = (
    "torso_link_0", "head_link_0", "pelvis_contour_link_0", "logo_link_0",
    "left_hip_pitch_link_0", "right_hip_pitch_link_0",
    "left_hip_roll_link_0", "right_hip_roll_link_0",
    "left_hip_yaw_link_0", "right_hip_yaw_link_0",
    "left_knee_link_0", "right_knee_link_0",
)
# Objekte, deren naechster Punkt weiter als das vom Becken liegt, kann kein Arm
# erreichen (Schulter ~0.3 m + Arm/Hand < 1 m); Rest = Puffer fuer Roboter-
# bewegung zwischen zwei /scene_markers-Updates.
REACH_RADIUS = 1.5


# ══════════════════════════════════════════════════════════════════════
# Kugel-Fit
# ══════════════════════════════════════════════════════════════════════
def _fps(points, k, rng):
    """Farthest-Point-Sampling: k gleichmaessig verteilte Startzentren."""
    first = int(rng.integers(len(points)))
    chosen = [first]
    dmin = np.linalg.norm(points - points[first], axis=1)
    for _ in range(k - 1):
        i = int(np.argmax(dmin))
        chosen.append(i)
        dmin = np.minimum(dmin, np.linalg.norm(points - points[i], axis=1))
    return points[chosen].copy()


def fit_spheres(points, r_max, k_max=24, seed=0):
    """Kugeln, die die Punktwolke `points` (n,3) UMSCHLIESSEN (jeder Punkt
    liegt in mindestens einer Kugel), mit Radius <= r_max wo moeglich.
    k-means mit FPS-Start; k wird erhoeht, bis der groesste Radius passt.
    Rueckgabe (centers (k,3), radii (k,))."""
    pts = np.asarray(points, dtype=float)
    rng = np.random.default_rng(seed)
    if len(pts) > 2000:
        pts = pts[rng.choice(len(pts), 2000, replace=False)]
    extent = float(np.linalg.norm(pts.max(0) - pts.min(0)))
    k = max(1, int(np.ceil(extent / (2.0 * r_max))))
    best = None
    while True:
        k_eff = min(k, len(pts))
        centers = _fps(pts, k_eff, rng)
        for _ in range(25):
            d = np.linalg.norm(pts[:, None, :] - centers[None], axis=2)
            lab = np.argmin(d, axis=1)
            new = np.array([pts[lab == i].mean(0) if np.any(lab == i) else centers[i]
                            for i in range(k_eff)])
            if np.allclose(new, centers, atol=1e-5):
                break
            centers = new
        d = np.linalg.norm(pts[:, None, :] - centers[None], axis=2)
        lab = np.argmin(d, axis=1)
        radii = np.array([d[lab == i, i].max() if np.any(lab == i) else 0.0
                          for i in range(k_eff)])
        keep = radii > 0.0
        best = (centers[keep], radii[keep])
        if radii.max() <= r_max or k_eff >= k_max or k_eff >= len(pts):
            return best
        k += max(1, k // 4)


def _cylinder_points(radius, length, n=64):
    """Mantel- und Deckelpunkte eines Zylinders (Achse z, zentriert)."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    zs = np.linspace(-length / 2, length / 2, 5)
    ring = np.stack([radius * np.cos(a), radius * np.sin(a)], axis=1)
    return np.concatenate([np.column_stack([ring, np.full(n, z)]) for z in zs])


def _geometry_points(geom):
    """Vertices einer coal-Geometrie (Mesh/Convex), Zylinder/Kugel gesampelt."""
    name = type(geom).__name__
    if name == "Cylinder":
        return _cylinder_points(float(geom.radius), 2.0 * float(geom.halfLength))
    if name == "Sphere":
        r = float(geom.radius)
        return np.array([[r, 0, 0], [-r, 0, 0], [0, r, 0], [0, -r, 0], [0, 0, r], [0, 0, -r]])
    for g in (geom, getattr(geom, "convex", None)):
        if g is None:
            continue
        for getter in ("points", "vertices"):
            fn = getattr(g, getter, None)
            if fn is None:
                continue
            try:
                pts = np.asarray(fn(), dtype=float)
                if pts.ndim == 2 and pts.shape[1] == 3 and len(pts):
                    return pts
            except Exception:
                pass
    try:
        geom.buildConvexRepresentation(False)
        return np.asarray(geom.convex.points(), dtype=float)
    except Exception:
        return None


def _norm(x):
    """Euklidische Norm ueber die letzte Achse (schneller als np.linalg.norm)."""
    return np.sqrt(np.einsum("...i,...i->...", x, x))


def _axis_rot(axis_idx, q):
    """Batch von Drehmatrizen um die lokale Achse axis_idx (0=x,1=y,2=z)."""
    c, s = np.cos(q), np.sin(q)
    R = np.zeros(q.shape + (3, 3))
    i, j = [(1, 2), (2, 0), (0, 1)][axis_idx]
    R[..., axis_idx, axis_idx] = 1.0
    R[..., i, i] = c
    R[..., j, j] = c
    R[..., i, j] = -s
    R[..., j, i] = s
    return R


def _quat_wxyz_to_matrix(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Environment:
    """Umgebungs-Objekte als Arrays (orientierte Boxen) -- picklebar, damit
    der Controller sie an den Planer-Prozess schicken kann."""

    def __init__(self, objects=()):
        objs = []
        for o in objects:
            try:
                pos = np.asarray(o["pos"], dtype=float).reshape(3)
                quat = np.asarray(o["quat"], dtype=float).reshape(4)
                half = np.abs(np.asarray(o["half_extents"], dtype=float).reshape(3))
                R = _quat_wxyz_to_matrix(quat)
            except (KeyError, ValueError, TypeError):
                continue
            # Naechster Punkt der Box vom Becken (Ursprung) -- Rest unerreichbar.
            local = R.T @ (-pos)
            if np.linalg.norm(np.maximum(np.abs(local) - half, 0.0)) > REACH_RADIUS:
                continue
            objs.append((str(o.get("name", "")), o.get("cls", "obstacle") == "grasp",
                         pos, R, half))
        self.names = [o[0] for o in objs]
        self.n = len(objs)
        self.is_grasp = np.array([o[1] for o in objs], dtype=bool)
        self.P = np.array([o[2] for o in objs]).reshape(-1, 3)
        self.R = np.array([o[3] for o in objs]).reshape(-1, 3, 3)
        self.H = np.array([o[4] for o in objs]).reshape(-1, 3)
        # Box-Achsen als (3,3N): local = c @ R_cat - offset in EINEM Produkt.
        self.R_cat = self.R.transpose(1, 0, 2).reshape(3, 3 * self.n)
        self.offset = np.einsum("nj,nji->ni", self.P, self.R)

    def __len__(self):
        return self.n

    def subset(self, mask):
        """Neue Environment mit den Objekten, fuer die mask True ist."""
        out = Environment.__new__(Environment)
        idx = np.flatnonzero(mask)
        out.names = [self.names[i] for i in idx]
        out.n = len(idx)
        out.is_grasp = self.is_grasp[idx]
        out.P, out.R, out.H = self.P[idx], self.R[idx], self.H[idx]
        out.R_cat = out.R.transpose(1, 0, 2).reshape(3, 3 * out.n)
        out.offset = np.einsum("nj,nji->ni", out.P, out.R)
        return out


# ══════════════════════════════════════════════════════════════════════
# Modell
# ══════════════════════════════════════════════════════════════════════
class ArmGeometry:
    """Kugelmodell + Abstands-/Gradientenrechnung fuer einen oder beide Arme.

    Konvention: Konfigurationen der Arme sind (..., 7*len(sides)), Seiten in
    der Reihenfolge von `sides` konkateniert (wie im Planer und in
    _planned_waypoints des Controllers)."""

    def __init__(self, model, collision_model, arm_r_max=0.04, hand_r_max=0.022,
                 self_r_max=0.05, body_r_max=0.09, tcp_frames=None):
        import pinocchio as pin   # nur zum Aufbau
        self._pin = pin
        self.model = model
        tcp_frames = tcp_frames or {"left": "left_hand_point_contact",
                                    "right": "right_hand_point_contact"}
        data = pin.Data(model)
        q0 = pin.neutral(model)
        pin.forwardKinematics(model, data, q0)
        pin.updateFramePlacements(model, data)

        # --- Ketten -----------------------------------------------------
        self.chain = {}
        for side in SIDES:
            jids = [model.getJointId(f"{side}_{s}") for s in ARM_JOINT_SUFFIXES]
            plc_R, plc_t, axes = [], [], []
            for jid in jids:
                M = model.jointPlacements[jid]
                plc_R.append(np.array(M.rotation))
                plc_t.append(np.array(M.translation))
                sn = model.joints[jid].shortname()
                axes.append({"JointModelRX": 0, "JointModelRY": 1, "JointModelRZ": 2}[sn])
            parent = model.parents[jids[0]]
            # Indizes im 29er-ROS-Vektor kennt der Aufrufer; hier zaehlt nur
            # die pinocchio-q-Position der Elternkette (Beine/Taille).
            self.chain[side] = {
                "jids": jids, "plc_R": np.array(plc_R), "plc_t": np.array(plc_t),
                "axes": axes, "parent": parent,
            }

        # --- Kugeln je Arm ------------------------------------------------
        cm = collision_model
        self.spheres = {}
        for side in SIDES:
            jids = self.chain[side]["jids"]
            subtree_of = {}
            for k, jid in enumerate(jids):
                for j in model.subtrees[jid]:
                    # tiefstes Kettenglied gewinnt (Finger -> wrist_yaw)
                    subtree_of[j] = k
            groups = {k: [] for k in range(7)}
            for g in cm.geometryObjects:
                k = subtree_of.get(g.parentJoint)
                if k is None:
                    continue
                pts = _geometry_points(g.geometry)
                if pts is None:
                    continue
                # Vertices -> Frame des Kettengelenks k (Finger: neutral).
                T = data.oMi[jids[k]].inverse() * data.oMi[g.parentJoint] * g.placement
                groups[k].append(pts @ np.array(T.rotation).T + np.array(T.translation))
            # Zwei Kugelsaetze: FEIN fuer die Umgebung (Hand ~2 cm, Arm ~4 cm --
            # Hand auf dem Tisch soll realistisch aufliegen) und GROB fuer die
            # Selbstkollision (nur Ellbogen abwaerts; die exakte Mesh-Pruefung
            # macht ohnehin coal, die Kugeln lenken nur den Optimierer).
            centers, radii, link, kind = [], [], [], []
            for k in range(7):
                if not groups[k]:
                    continue
                pts = np.concatenate(groups[k])
                r_max = hand_r_max if k >= HAND_LINK_MIN else arm_r_max
                c, r = fit_spheres(pts, r_max, k_max=40 if k >= HAND_LINK_MIN else 12,
                                   seed=k)
                centers.append(c)
                radii.append(r)
                link += [k] * len(r)
                kind += [0] * len(r)
                if k >= SELF_LINK_MIN:
                    c, r = fit_spheres(pts, self_r_max, k_max=10, seed=10 + k)
                    centers.append(c)
                    radii.append(r)
                    link += [k] * len(r)
                    kind += [1] * len(r)
            # TCP als Punkt (Radius 0) an Glied 6 -- fuer Anfahrrichtung/Tempo.
            fid = model.getFrameId(tcp_frames[side])
            T_tcp = data.oMi[jids[6]].inverse() * data.oMf[fid]
            kind = np.array(kind, dtype=int)
            self.spheres[side] = {
                "c": np.concatenate(centers), "r": np.concatenate(radii),
                "link": np.array(link, dtype=int),
                "env_idx": np.nonzero(kind == 0)[0],
                "self_idx": np.nonzero(kind == 1)[0],
                "tcp": np.array(T_tcp.translation),
            }
        # Hand-Vorwaertsachse (Fingerrichtung) im wrist_yaw-Frame: +x (die
        # Handbasis reicht von x=0.04 bis 0.20 m).
        self.hand_axis_local = np.array([1.0, 0.0, 0.0])

        # --- Koerperkugeln (lokal je Elterngelenk) -------------------------
        self._body = []
        names = {g.name: g for g in cm.geometryObjects}
        for nm in BODY_GEOMS:
            g = names.get(nm)
            if g is None:
                continue
            pts = _geometry_points(g.geometry)
            if pts is None:
                continue
            pts = pts @ np.array(g.placement.rotation).T + np.array(g.placement.translation)
            c, r = fit_spheres(pts, body_r_max, k_max=24, seed=7)
            self._body.append((g.parentJoint, c, r))
        # Reichweite je Arm: obere Schranke fuer den Abstand jeder Armkugel-
        # Oberflaeche vom Ursprung des ersten Schultergelenks (Summe der
        # Gliedlaengen + Kugel-Offset + Radius) -- konfigurationsunabhaengig.
        self.reach_radius = {}
        for side in SIDES:
            ch, sp = self.chain[side], self.spheres[side]
            seg = np.concatenate([[0.0], np.cumsum(np.linalg.norm(ch["plc_t"][1:], axis=1))])
            self.reach_radius[side] = float(np.max(
                seg[sp["link"]] + np.linalg.norm(sp["c"], axis=1) + sp["r"]))
        self._ros_to_q = {}
        self.set_base(np.zeros(29))
        self.set_environment(Environment())

    # ------------------------------------------------------------------
    # Basis (Beine/Taille) und Umgebung
    # ------------------------------------------------------------------
    def set_base(self, full29):
        """Beine/Taille aus der 29-DOF-Konfiguration (ROS-Reihenfolge)
        uebernehmen: Basis-Transformation der Arm-Ketten + Koerperkugeln."""
        pin = self._pin
        from g1pilot.utils.joints_names import JOINT_NAMES_ROS
        model = self.model
        if not self._ros_to_q:
            for i in range(29):
                jid = model.getJointId(JOINT_NAMES_ROS[i])
                self._ros_to_q[i] = model.joints[jid].idx_q
        q = pin.neutral(model)
        for i, iq in self._ros_to_q.items():
            q[iq] = float(full29[i])
        data = pin.Data(model)
        pin.forwardKinematics(model, data, q)
        self.base_full = np.array(full29, dtype=float).copy()
        self._base = {}
        for side in SIDES:
            M = data.oMi[self.chain[side]["parent"]]
            self._base[side] = (np.array(M.rotation), np.array(M.translation))
        bc, br = [], []
        for jid, c, r in self._body:
            M = data.oMi[jid]
            bc.append(c @ np.array(M.rotation).T + np.array(M.translation))
            br.append(r)
        self.body_c = np.concatenate(bc) if bc else np.zeros((0, 3))
        self.body_r = np.concatenate(br) if br else np.zeros(0)

    def shoulder(self, side):
        """Ursprung des ersten Schultergelenks (Weltkoordinaten, aktuelle Basis)."""
        R0, t0 = self._base[side]
        return t0 + R0 @ self.chain[side]["plc_t"][0]

    def reachable(self, env, sides, margin=0.02):
        """Nur die Objekte von env, die einer der Arme in `sides` ueberhaupt
        beruehren kann (Box-Abstand zur Schulter < Reichweite). Spart in einer
        grossen Szene (Arbeitsplatz aus vielen Teilen) den Grossteil der
        Abstandsrechnung, ohne etwas Erreichbares wegzulassen."""
        if env.n == 0:
            return env
        keep = np.zeros(env.n, dtype=bool)
        for side in sides:
            local = np.einsum("nji,nj->ni", env.R, self.shoulder(side) - env.P)
            d = _norm(np.maximum(np.abs(local) - env.H, 0.0))
            keep |= d < self.reach_radius[side] + margin
        return env if keep.all() else env.subset(keep)

    def set_environment(self, env):
        self.env = env if isinstance(env, Environment) else Environment(env)

    # ------------------------------------------------------------------
    # Vorwaertskinematik (vektorisiert)
    # ------------------------------------------------------------------
    def fk(self, Q, sides):
        """Q (..., 7*len(sides)) -> dict je Seite mit
             'c'   Kugelmitten (..., S, 3)
             'o'   Gelenk-Ursprung (..., 7, 3),  'a' Gelenkachse (..., 7, 3)
             'tcp' (..., 3),  'R6' Orientierung des wrist_yaw-Frames (..., 3, 3)
        """
        Q = np.asarray(Q, dtype=float)
        out = {}
        for n, side in enumerate(sides):
            ch, sp = self.chain[side], self.spheres[side]
            q = Q[..., 7 * n:7 * n + 7]
            R0, t0 = self._base[side]
            shp = q.shape[:-1]
            R = np.broadcast_to(R0, shp + (3, 3))
            p = np.broadcast_to(t0, shp + (3,))
            o = np.empty(shp + (7, 3))
            a = np.empty(shp + (7, 3))
            Rs = []
            for k in range(7):
                p = p + R @ ch["plc_t"][k]
                R = R @ ch["plc_R"][k]
                ax = ch["axes"][k]
                a[..., k, :] = R[..., :, ax]
                o[..., k, :] = p
                R = R @ _axis_rot(ax, q[..., k])
                Rs.append(R)
            Rst = np.stack(Rs, axis=-3)                      # (..., 7, 3, 3)
            Ot = o
            link = sp["link"]
            c = np.einsum("...sij,sj->...si", Rst[..., link, :, :], sp["c"]) + Ot[..., link, :]
            tcp = Rs[6] @ sp["tcp"] + o[..., 6, :]
            out[side] = {"c": c, "o": o, "a": a, "tcp": tcp, "R6": Rs[6]}
        return out

    def radii(self, side):
        return self.spheres[side]["r"]

    def env_idx(self, side):
        return self.spheres[side]["env_idx"]

    def self_idx(self, side):
        return self.spheres[side]["self_idx"]

    def links(self, side):
        return self.spheres[side]["link"]

    # ------------------------------------------------------------------
    # Abstaende
    # ------------------------------------------------------------------
    def env_dist(self, C, r):
        """Kugeln (..., S, 3) mit Radien (S,) gegen alle Boxen.
        -> (d (..., S, N) vorzeichenbehaftet, n (..., S, N, 3) Gradient
        dd/dc in Weltkoordinaten)."""
        env = self.env
        shp = C.shape[:-1]
        if env.n == 0:
            return np.zeros(shp + (0,)), np.zeros(shp + (0, 3))
        local = (C @ env.R_cat).reshape(shp + (env.n, 3)) - env.offset
        sgn = np.where(local >= 0.0, 1.0, -1.0)
        inner = np.abs(local) - env.H
        outer = np.maximum(inner, 0.0)
        out_norm = _norm(outer)
        inside = out_norm <= 0.0
        maxin = inner.max(axis=-1)
        d = np.where(inside, maxin, out_norm) - r[:, None]
        # Gradient lokal: aussen = Richtung zum naechsten Punkt, innen = Achse
        # der naechsten Seitenflaeche.
        n_loc = np.where(inside[..., None],
                         (inner == maxin[..., None]) * sgn,
                         outer * sgn / np.maximum(out_norm, 1e-12)[..., None])
        n = np.einsum("nij,...nj->...ni", env.R, n_loc)
        return d, n

    def self_body_dist(self, C, r):
        """Kugeln (..., S, 3) gegen Koerperkugeln -> (d (...,S,B), n (...,S,B,3))."""
        diff = C[..., :, None, :] - self.body_c
        nrm = _norm(diff)
        d = nrm - r[:, None] - self.body_r
        n = diff / np.maximum(nrm, 1e-12)[..., None]
        return d, n

    @staticmethod
    def pair_dist(C1, r1, C2, r2):
        """Kugeln Arm 1 (...,S1,3) gegen Arm 2 (...,S2,3) -> (d, n) mit n =
        dd/dc1 (dd/dc2 = -n)."""
        diff = C1[..., :, None, :] - C2[..., None, :, :]
        nrm = _norm(diff)
        d = nrm - r1[:, None] - r2[None, :]
        n = diff / np.maximum(nrm, 1e-12)[..., None]
        return d, n

    # ------------------------------------------------------------------
    # Gradient: Kraefte auf Kugelmitten -> Gelenkraum
    # ------------------------------------------------------------------
    @staticmethod
    def forces_to_q(fk_side, link, F, F_tcp=None):
        """F (..., S, 3) = dCost/dc je Kugel -> dCost/dq (..., 7) ueber die
        Kettenregel: dc/dq_j = a_j x (c - o_j) fuer alle Kugeln an Glied >= j.
        Optional F_tcp (..., 3) auf den TCP."""
        c, o, a = fk_side["c"], fk_side["o"], fk_side["a"]
        # Sum_s (c_s - o_j) x F_s = Sum_s c_s x F_s - o_j x Sum_s F_s, ueber
        # alle Kugeln an Glied >= j: Gliedsummen einmal bilden, dann Suffix-
        # Summen -- statt 7 maskierter Kreuzprodukte.
        onehot = (link[:, None] == np.arange(7)[None]).astype(float)   # (S,7)
        mom = np.einsum("...si,sk->...ki", np.cross(c, F), onehot)       # (...,7,3)
        frc = np.einsum("...si,sk->...ki", F, onehot)
        if F_tcp is not None:
            mom[..., 6, :] += np.cross(fk_side["tcp"], F_tcp)
            frc[..., 6, :] += F_tcp
        mom = np.flip(np.cumsum(np.flip(mom, -2), -2), -2)
        frc = np.flip(np.cumsum(np.flip(frc, -2), -2), -2)
        torque = mom - np.cross(o, frc)
        return np.einsum("...ji,...ji->...j", a, torque)

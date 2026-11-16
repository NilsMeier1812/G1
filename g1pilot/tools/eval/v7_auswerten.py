#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v7_auswerten.py -- V7: zwei Gelenkwinkel-Aufzeichnungen vergleichen
(typisch Simulation A gegen realen G1 B).

Zeitlicher Abgleich (--abgleich):
  schritt (Standard)  je Schritt wird auf den Beginn der Fahrt gelegt
                      (Status 'executing', sonst 'accepted', sonst goto);
                      verglichen wird das Fenster bis zum Beginn des naechsten
                      Schritts (bzw. Aufzeichnungsende), so lang wie das
                      kuerzere der beiden Fenster.
  start               nur der Beginn des ersten Schritts wird gleichgesetzt,
                      danach laeuft die Zeit frei (zeigt Laufzeit-Drift).
Beide Ist-Verlaeufe werden auf ein gemeinsames Raster (--raster, 100 Hz)
linear interpoliert -- keine Glaettung, keine Filter.

Ergebnis je Armgelenk: mittlere und groesste Abweichung |q_A - q_B| (Ist),
dazu je Aufzeichnung der Folgefehler |Soll - Ist|. Je Schritt: Abweichung
der Handposition (TCP, Vorwaertskinematik mit demselben URDF wie der
Planer) am Ende der Bewegung ('reached'), links und rechts, sowie die
Abweichung jeder Aufzeichnung zur Ziel-TCP-Lage der gespeicherten Pose.
Diagramm ueber der Zeit: diagramme.pdf.

Aufruf:
    python3 tools/eval/v7_auswerten.py <ordner_A> <ordner_B> [--name-a Simulation --name-b Roboter]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_common as ec  # noqa: E402

VERSUCH = "v7_vergleich"
ARM_NAMES = ["l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
             "l_wrist_roll", "l_wrist_pitch", "l_wrist_yaw",
             "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow",
             "r_wrist_roll", "r_wrist_pitch", "r_wrist_yaw"]


class Aufzeichnung:
    def __init__(self, folder):
        import json
        import numpy as np
        self.folder = folder
        self.meta, cols, rows = ec.read_csv(os.path.join(folder, "ist.csv"))
        self.ist_t = np.array([float(r["t_s"]) for r in rows])
        self.ist_q = np.array([[float(r[f"q_{n}"]) for n in ARM_NAMES] for r in rows]).reshape(-1, 14)
        self.waist = np.array([[float(r[c]) for c in ("waist_yaw", "waist_roll", "waist_pitch")]
                               for r in rows]).reshape(-1, 3)
        _m, _c, srows = ec.read_csv(os.path.join(folder, "soll.csv"))
        self.soll_t = np.array([float(r["t_s"]) for r in srows])
        self.soll_q = np.array([[float(r[f"q_{n}"]) for n in ARM_NAMES] for r in srows]).reshape(-1, 14)
        self.soll_w = np.array([float(r["gewicht"]) for r in srows])
        with open(os.path.join(folder, "summary.json"), encoding="utf-8") as f:
            self.summary = json.load(f)
        self.steps = self.summary.get("schritte", [])
        self.mode = self.meta.get("betriebsart", "?")

    @staticmethod
    def step_start(s):
        for k in ("t_executing", "t_accepted", "t_goto"):
            if s.get(k) is not None:
                return float(s[k])
        return None

    def q_at(self, t):
        import numpy as np
        return np.array([np.interp(t, self.ist_t, self.ist_q[:, j]) for j in range(14)])

    def waist_at(self, t):
        import numpy as np
        return np.array([np.interp(t, self.ist_t, self.waist[:, j]) for j in range(3)])

    def ist_on(self, grid):
        import numpy as np
        return np.stack([np.interp(grid, self.ist_t, self.ist_q[:, j]) for j in range(14)], axis=1)

    def soll_on(self, grid):
        import numpy as np
        if len(self.soll_t) < 2:
            return np.full((len(grid), 14), np.nan)
        out = np.stack([np.interp(grid, self.soll_t, self.soll_q[:, j]) for j in range(14)], axis=1)
        out[(grid < self.soll_t[0]) | (grid > self.soll_t[-1])] = np.nan
        return out


def check_identical(A, B):
    """-> Liste von Hinweisen, wenn die Ablaeufe nicht identisch sind."""
    import numpy as np
    msgs = []
    pa = [s.get("pose") for s in A.summary.get("schritte_soll", [])]
    pb = [s.get("pose") for s in B.summary.get("schritte_soll", [])]
    if pa != pb:
        msgs.append(f"Unterschiedliche Posenfolge: A={pa} B={pb}")
    for name in set(pa) & set(pb):
        da, db = A.summary["posen"].get(name, {}), B.summary["posen"].get(name, {})
        for k in ("left_arm", "right_arm"):
            if (k in da) != (k in db):
                msgs.append(f"Pose {name}: {k} nur in einer Aufzeichnung")
            elif k in da and np.max(np.abs(np.array(da[k]) - np.array(db[k]))) > 1e-6:
                msgs.append(f"Pose {name}: {k} unterscheidet sich (Pose-Datei geaendert?)")
    for X, lab in ((A, "A"), (B, "B")):
        bad = [s["pose"] for s in X.steps if s.get("zustand") != "reached"]
        if bad:
            msgs.append(f"{lab}: Schritte nicht erreicht: {bad}")
    return msgs


def align_windows(A, B, mode, raster):
    """-> Liste (schritt_index, grid_A, grid_B, t_rel) gemeinsamer Fenster."""
    import numpy as np
    n = min(len(A.steps), len(B.steps))
    out = []
    dt = 1.0 / raster
    if n == 0:
        return out
    if mode == "start":
        a0, b0 = A.step_start(A.steps[0]), B.step_start(B.steps[0])
        length = min(A.ist_t[-1] - a0, B.ist_t[-1] - b0)
        rel = np.arange(0.0, max(length, 0.0), dt)
        return [(None, a0 + rel, b0 + rel, rel)]
    for k in range(n):
        a0, b0 = A.step_start(A.steps[k]), B.step_start(B.steps[k])
        if a0 is None or b0 is None:
            continue
        a1 = A.step_start(A.steps[k + 1]) if k + 1 < len(A.steps) else A.ist_t[-1]
        b1 = B.step_start(B.steps[k + 1]) if k + 1 < len(B.steps) else B.ist_t[-1]
        length = min(a1 - a0, b1 - b0)
        if length <= dt:
            continue
        rel = np.arange(0.0, length, dt)
        out.append((k, a0 + rel, b0 + rel, rel))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="V7: zwei Gelenkwinkel-Aufzeichnungen vergleichen.")
    ap.add_argument("a", help="Ordner der Aufzeichnung A (z. B. Simulation)")
    ap.add_argument("b", help="Ordner der Aufzeichnung B (z. B. realer G1)")
    ap.add_argument("--name-a", default=None, help="Beschriftung A (Default: Betriebsart)")
    ap.add_argument("--name-b", default=None, help="Beschriftung B (Default: Betriebsart)")
    ap.add_argument("--abgleich", choices=("schritt", "start"), default="schritt")
    ap.add_argument("--raster", type=float, default=100.0, help="Vergleichsraster [Hz]")
    ap.add_argument("--ab-schritt", type=int, default=1,
                    help="erst ab diesem Schritt vergleichen (1 = alle). Der erste Schritt startet "
                         "oft aus unterschiedlichen Ausgangslagen.")
    ap.add_argument("--urdf", default=os.path.join(ec.PKG_ROOT, "description_files", "urdf",
                                                   "g1_29dof_inspire_ftp.urdf"),
                    help="URDF fuer die Vorwaertskinematik")
    ap.add_argument("--no-fk", action="store_true", help="ohne Vorwaertskinematik (kein pinocchio)")
    ap.add_argument("--no-plot", action="store_true")
    ec.add_common_args(ap, VERSUCH)
    a = ap.parse_args(argv)
    import numpy as np

    A, B = Aufzeichnung(a.a), Aufzeichnung(a.b)
    na = a.name_a or f"A ({A.mode})"
    nb = a.name_b or f"B ({B.mode})"
    run = ec.Run("v7_auswerten.py", VERSUCH, ec.MODE_OFFLINE,
                 {"a": os.path.abspath(a.a), "b": os.path.abspath(a.b), "name_a": na, "name_b": nb,
                  "a_git": A.meta.get("git"), "b_git": B.meta.get("git"),
                  "a_betriebsart": A.mode, "b_betriebsart": B.mode,
                  "abgleich": a.abgleich, "raster_hz": a.raster, "ab_schritt": a.ab_schritt,
                  "urdf": os.path.relpath(a.urdf, ec.PKG_ROOT)}, out_root=a.out)
    hinweise = check_identical(A, B)
    for h in hinweise:
        print(f"[v7] Hinweis: {h}")

    wins = align_windows(A, B, a.abgleich, a.raster)
    if a.abgleich == "schritt":
        wins = [w for w in wins if w[0] + 1 >= a.ab_schritt]
    all_d, all_ea, all_eb = [], [], []
    rows_t, rows_s = [], []
    for k, ga, gb, rel in wins:
        qa, qb = A.ist_on(ga), B.ist_on(gb)
        sa, sb = A.soll_on(ga), B.soll_on(gb)
        d = qa - qb
        all_d.append(d)
        all_ea.append(sa - qa)
        all_eb.append(sb - qb)
        if k is not None:
            ad = np.abs(d)
            rows_s.append([k + 1, A.steps[k].get("pose"), round(float(rel[-1]), 4),
                           float(ad.mean()), float(ad.max()), ARM_NAMES[int(ad.max(axis=0).argmax())],
                           float(np.max(np.abs(qa[0] - qb[0])))])
        for i in range(len(rel)):
            rows_t.append([("" if k is None else k + 1), round(float(rel[i]), 4),
                           round(float(ga[i]), 4), round(float(gb[i]), 4)]
                          + [float(x) for x in qa[i]] + [float(x) for x in qb[i]]
                          + [float(x) for x in d[i]])
    D = np.concatenate(all_d) if all_d else np.zeros((0, 14))
    EA = np.concatenate(all_ea) if all_ea else np.zeros((0, 14))
    EB = np.concatenate(all_eb) if all_eb else np.zeros((0, 14))

    def stat(X, j):
        col = X[:, j]
        col = col[~np.isnan(col)]
        if col.size == 0:
            return None, None, None
        ab = np.abs(col)
        return float(ab.mean()), float(ab.max()), float(np.sqrt((col ** 2).mean()))

    rows_j, gelenke = [], {}
    for j, n in enumerate(ARM_NAMES):
        m, mx, rms = stat(D, j)
        ma, mxa, _ = stat(EA, j)
        mb, mxb, _ = stat(EB, j)
        gelenke[n] = {"abw_mittel_rad": m, "abw_max_rad": mx, "abw_rms_rad": rms,
                      "folgefehler_a_mittel_rad": ma, "folgefehler_a_max_rad": mxa,
                      "folgefehler_b_mittel_rad": mb, "folgefehler_b_max_rad": mxb}
        rows_j.append([n, m, mx, rms, ma, mxa, mb, mxb,
                       None if m is None else float(np.degrees(m)),
                       None if mx is None else float(np.degrees(mx))])

    # Handposition am Ende jeder Bewegung
    rows_fk, fk_sum = [], {}
    if not a.no_fk:
        try:
            from v6_armplaner import Model
            model = Model(a.urdf)
            g = model.geom
            idx = {s: model.ap.joint_indices_for([s]) for s in ("left", "right")}

            def tcp(q14, waist):
                full = np.zeros(29)
                full[12:15] = waist
                full[15:29] = q14
                g.set_base(full)
                return {s: g.fk(full[idx[s]][None], [s])[s]["tcp"][0] for s in ("left", "right")}

            for k in range(min(len(A.steps), len(B.steps))):
                sa, sb = A.steps[k], B.steps[k]
                if sa.get("t_ende") is None or sb.get("t_ende") is None:
                    continue
                ta, tb = float(sa["t_ende"]), float(sb["t_ende"])
                pa, pb = tcp(A.q_at(ta), A.waist_at(ta)), tcp(B.q_at(tb), B.waist_at(tb))
                pose = A.summary.get("posen", {}).get(sa["pose"], {})
                q_goal = np.zeros(14)
                have_goal = True
                for s, sl in (("left", slice(0, 7)), ("right", slice(7, 14))):
                    if f"{s}_arm" in pose:
                        q_goal[sl] = pose[f"{s}_arm"]
                    else:
                        q_goal[sl] = A.q_at(ta)[sl]
                pg = tcp(q_goal, A.waist_at(ta)) if have_goal else None
                for s in ("left", "right"):
                    if f"{s}_arm" not in pose:
                        continue        # Arm in dieser Pose nicht gespeichert -> nicht bewegt
                    dab = float(np.linalg.norm(pa[s] - pb[s]))
                    rows_fk.append([k + 1, sa["pose"], s, round(ta, 4), round(tb, 4),
                                    dab * 1000.0,
                                    float(np.linalg.norm(pa[s] - pg[s])) * 1000.0,
                                    float(np.linalg.norm(pb[s] - pg[s])) * 1000.0]
                                   + [float(x) for x in pa[s]] + [float(x) for x in pb[s]])
            vals = [r[5] for r in rows_fk]
            fk_sum = {"abw_hand_mm": ec.describe(vals),
                      "a_zu_ziel_mm": ec.describe([r[6] for r in rows_fk]),
                      "b_zu_ziel_mm": ec.describe([r[7] for r in rows_fk]),
                      "bezug": "TCP-Rahmen des Planers (ArmGeometry.fk), im Becken-Rahmen; "
                               "Taille aus der jeweiligen Messung."}
        except Exception as e:      # noqa: BLE001
            fk_sum = {"fehler": f"Vorwaertskinematik nicht moeglich: {e}"}
            print(f"[v7] {fk_sum['fehler']}")

    run.write_csv("gelenke.csv",
                  ["gelenk", "abw_mittel_rad", "abw_max_rad", "abw_rms_rad",
                   "folgefehler_a_mittel_rad", "folgefehler_a_max_rad",
                   "folgefehler_b_mittel_rad", "folgefehler_b_max_rad", "abw_mittel_grad", "abw_max_grad"],
                  rows_j, f"Abweichung Ist {na} gegen Ist {nb} je Armgelenk; Folgefehler = |Soll-Ist|")
    run.write_csv("schritte.csv",
                  ["schritt", "pose", "fensterlaenge_s", "abw_mittel_rad", "abw_max_rad",
                   "gelenk_max", "abw_beim_start_rad"],
                  rows_s, "Abweichung Ist A - Ist B je Schritt (alle 14 Armgelenke); "
                          "abw_beim_start = Unterschied der Ausgangslage")
    run.write_csv("hand_ende.csv",
                  ["schritt", "pose", "seite", "t_ende_a_s", "t_ende_b_s", "abw_ab_mm",
                   "a_zu_ziel_mm", "b_zu_ziel_mm", "a_x", "a_y", "a_z", "b_x", "b_y", "b_z"],
                  rows_fk, "TCP-Lage am Ende jeder Bewegung (Status reached), Becken-Rahmen [m]")
    run.write_csv("verlauf.csv",
                  ["schritt", "t_rel_s", "t_a_s", "t_b_s"] + [f"a_{n}" for n in ARM_NAMES]
                  + [f"b_{n}" for n in ARM_NAMES] + [f"d_{n}" for n in ARM_NAMES],
                  rows_t, "Abgeglichene Ist-Verlaeufe auf dem Raster (linear interpoliert)")
    dauer = []
    for k in range(min(len(A.steps), len(B.steps))):
        sa, sb = A.steps[k], B.steps[k]
        da = (sa["t_ende"] - A.step_start(sa)) if sa.get("t_ende") is not None else None
        db = (sb["t_ende"] - B.step_start(sb)) if sb.get("t_ende") is not None else None
        dauer.append({"schritt": k + 1, "pose": sa.get("pose"), "dauer_a_s": da, "dauer_b_s": db,
                      "zustand_a": sa.get("zustand"), "zustand_b": sb.get("zustand")})
    allab = np.abs(D[~np.isnan(D).any(axis=1)]) if D.size else D
    summary = {"name_a": na, "name_b": nb, "hinweise": hinweise, "abgleich": a.abgleich,
               "ab_schritt": a.ab_schritt, "schritte": [dict(zip(
                   ["schritt", "pose", "fensterlaenge_s", "abw_mittel_rad", "abw_max_rad",
                    "gelenk_max", "abw_beim_start_rad"], r)) for r in rows_s],
               "vergleichszeit_s": len(D) / a.raster,
               "gesamt": {"abw_mittel_rad": float(allab.mean()) if allab.size else None,
                          "abw_max_rad": float(allab.max()) if allab.size else None},
               "gelenke": gelenke, "hand_ende": fk_sum, "schrittdauer": dauer}
    if not a.no_plot and wins:
        try:
            plot(run, A, B, na, nb, wins)
        except Exception as e:      # noqa: BLE001
            summary["diagramm_fehler"] = str(e)
            print(f"[v7] Diagramm nicht erzeugt: {e}")
    run.finish(summary)
    return 0


def plot(run, A, B, na, nb, wins):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.backends.backend_pdf import PdfPages
    # zusammenhaengende Zeitachse: Fenster hintereinander
    xs, qa, qb, sa, bounds = [], [], [], [], []
    off = 0.0
    for k, ga, gb, rel in wins:
        xs.append(rel + off)
        qa.append(A.ist_on(ga))
        qb.append(B.ist_on(gb))
        sa.append(A.soll_on(ga))
        bounds.append((off, k))
        off += rel[-1] + (rel[1] - rel[0] if len(rel) > 1 else 0.01)
    X, QA, QB, SA = np.concatenate(xs), np.concatenate(qa), np.concatenate(qb), np.concatenate(sa)
    with PdfPages(run.path("diagramme.pdf")) as pdf:
        for side, js in (("links", range(0, 7)), ("rechts", range(7, 14))):
            fig, axs = plt.subplots(7, 1, figsize=(11, 15), sharex=True)
            for ax, j in zip(axs, js):
                ax.plot(X, np.degrees(QA[:, j]), lw=1.0, label=f"Ist {na}")
                ax.plot(X, np.degrees(QB[:, j]), lw=1.0, label=f"Ist {nb}")
                ax.plot(X, np.degrees(SA[:, j]), lw=0.8, ls="--", color="gray", label=f"Soll {na}")
                for b, k in bounds:
                    ax.axvline(b, color="k", lw=0.4, alpha=0.4)
                ax.set_ylabel(f"{ARM_NAMES[j]}\n[°]", fontsize=8)
            axs[0].legend(fontsize=8, loc="upper right")
            axs[-1].set_xlabel("abgeglichene Zeit [s] (senkrechte Linien = Schrittbeginn)")
            fig.suptitle(f"V7 Gelenkwinkel Arm {side}: {na} gegen {nb}")
            fig.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)
        fig, axs = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        for ax, js, side in ((axs[0], range(0, 7), "links"), (axs[1], range(7, 14), "rechts")):
            for j in js:
                ax.plot(X, np.degrees(QA[:, j] - QB[:, j]), lw=0.8, label=ARM_NAMES[j])
            ax.set_ylabel(f"Ist A - Ist B [°] ({side})")
            ax.legend(fontsize=7, ncol=4)
        axs[-1].set_xlabel("abgeglichene Zeit [s]")
        fig.suptitle("V7 Abweichung je Gelenk über der Zeit")
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)
    run.files.append("diagramme.pdf")


if __name__ == "__main__":
    sys.exit(main())

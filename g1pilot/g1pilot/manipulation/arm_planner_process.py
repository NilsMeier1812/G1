#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_planner_process.py — der Armplaner als EIGENER Prozess.

Warum ein Prozess und kein Thread: die Planung ist reines Python/numpy und
hielte im Prozess des arm_controller das GIL -- der 250-Hz-Regelkreis
bekaeme dann waehrend jeder Planung Aussetzer. Im eigenen Prozess rechnet
der Planer auf einem anderen Kern; der Controller wartet in einem Thread
blockierend auf die Pipe (gibt das GIL frei).

Der Arbeiter laedt beim Start dasselbe URDF wie der Controller (G1IKSolver
fuer die kartesische IK und die exakte Mesh-Selbstkollision, ArmGeometry fuer
das Kugelmodell) und beantwortet danach Anfragen:

  Anfrage  {id, sides, base_full (29), q_start (7*n), q_goal (7*n) | None,
            ik_targets {side: 4x4} | None, limits [(lo,hi)...],
            env [Objekt-Dicts], params {...}, walk_pose {side: q7}}
  Antwort  {id, ok, reason, t, q, waypoints, clearance, plan_time_s,
            ik_detail, approach, stats}

Abbruch: cancel() setzt ein geteiltes Event, der Planer prueft es in seinen
Schleifen und antwortet mit reason='cancelled'. Haengt der Arbeiter dennoch
(Absturz, Endlosschleife), wird er nach dem Zeitlimit beendet und neu
gestartet.
"""
import multiprocessing as mp
import os
import threading
import time
import traceback

import numpy as np


def _worker_main(conn, cancel_event, cfg):
    """Einstieg des Arbeiterprozesses (spawn: frischer Interpreter, kein
    geerbter ROS-/DDS-Zustand)."""
    try:
        os.nice(2)          # Regelkreis hat (leicht) Vorrang
    except Exception:
        pass
    try:
        import pinocchio as pin
        from g1pilot.utils.ik_solver import G1IKSolver
        from g1pilot.manipulation.arm_geometry import ArmGeometry, Environment
        from g1pilot.manipulation.arm_planner import plan_arms, joint_indices_for

        ik = G1IKSolver(urdf_path=cfg["urdf_path"], mesh_dir=cfg["mesh_dir"], debug=False)
        for side, q7 in (cfg.get("rest") or {}).items():
            ik.set_rest_posture(side, np.asarray(q7, dtype=float))
        ik.null_space_gain = float(cfg.get("null_space_gain", ik.null_space_gain))
        geom = ArmGeometry(ik.model, ik.collision_model)
        scratch = ik.make_scratch_buffers()
        conn.send({"ready": True})
    except Exception as e:   # noqa: BLE001
        conn.send({"ready": False, "error": f"{e}\n{traceback.format_exc()}"})
        return

    while True:
        try:
            req = conn.recv()
        except (EOFError, OSError):
            return
        if req is None:
            return
        t0 = time.monotonic()
        out = {"id": req.get("id"), "ok": False}
        try:
            sides = list(req["sides"])
            base = np.asarray(req["base_full"], dtype=float)
            ik_detail = None
            q_goal = req.get("q_goal")
            if req.get("ik_targets"):
                goals, ik_detail, failed = {}, {}, []
                for side in sides:
                    T = np.asarray(req["ik_targets"][side], dtype=float)
                    res = ik.solve_pose(side, pin.SE3(T[:3, :3], T[:3, 3]), base)
                    ik_detail[side] = {
                        "pos_err_m": round(float(res["pos_err_m"]), 5),
                        "ori_err_deg": round(float(res["ori_err_deg"]), 2),
                        "converged": bool(res["converged"]),
                    }
                    if res["q"] is None or not res["converged"]:
                        failed.append(side)
                    else:
                        goals[side] = np.asarray(res["q"], dtype=float)
                out["ik_detail"] = ik_detail
                if failed:
                    out["reason"] = "ik_failed"
                    out["failed_sides"] = failed
                    conn.send(out)
                    continue
                q_goal = np.concatenate([goals[s] for s in sides])
            q_goal = np.asarray(q_goal, dtype=float)
            q_start = np.asarray(req["q_start"], dtype=float)

            geom.set_base(base)
            geom.set_environment(Environment(req.get("env") or []))
            idx = joint_indices_for(sides)
            check_self = bool(req.get("self_collision", True))

            def self_check(qN):
                if not check_self:
                    return False
                full = base.copy()
                full[idx] = qN
                return ik.arm_command_in_collision(full, hard=True, data=scratch["data"],
                                                   cdata=scratch["cdata_hard"])

            walk = req.get("walk_pose")
            walk = {s: np.asarray(v, dtype=float) for s, v in walk.items()} if walk else None
            res = plan_arms(geom, sides, q_start, q_goal, req["limits"],
                            params=req.get("params"), self_check=self_check,
                            walk_pose=walk, should_stop=cancel_event.is_set)
            out.update({
                "ok": bool(res["ok"]), "reason": res["reason"],
                "plan_time_s": round(time.monotonic() - t0, 3),
                "stats": res.get("stats"), "q_goal": q_goal,
            })
            if res["ok"]:
                out.update({"t": res["t"], "q": res["q"], "waypoints": res["waypoints"],
                            "clearance": res["clearance"], "approach": res["approach"]})
        except Exception as e:   # noqa: BLE001
            out["reason"] = f"Planer-Fehler: {e}"
            out["traceback"] = traceback.format_exc()
        conn.send(out)


class PlannerProcess:
    """Steuert den Arbeiterprozess (Start, Anfrage, Abbruch, Neustart)."""

    def __init__(self, urdf_path, mesh_dir, rest=None, null_space_gain=0.15, logger=None):
        self._cfg = {"urdf_path": urdf_path, "mesh_dir": mesh_dir,
                     "rest": {k: list(map(float, v)) for k, v in (rest or {}).items()},
                     "null_space_gain": float(null_space_gain)}
        self._log = logger
        self._ctx = mp.get_context("spawn")
        self._lock = threading.Lock()
        self._proc = None
        self._conn = None
        self._cancel = self._ctx.Event()
        self._ready = False
        self._start_error = None
        self._req_counter = 0

    # ------------------------------------------------------------------
    def _spawn(self):
        parent, child = self._ctx.Pipe()
        self._proc = self._ctx.Process(target=_worker_main, args=(child, self._cancel, self._cfg),
                                       name="g1_arm_planner", daemon=True)
        self._proc.start()
        child.close()
        self._conn = parent
        self._ready = False
        self._start_error = None

    def _await_ready(self, wait_s):
        """Auf die 'ready'-Meldung warten. Ein Arbeiter, der noch LAEDT, wird
        nach Ablauf NICHT beendet (auf einem ausgelasteten Rechner dauert das
        Laden des Modells deutlich laenger) -- nur ein abgestuerzter."""
        if self._ready:
            return True
        if self._proc is None:
            return False
        try:
            if self._conn.poll(wait_s):
                msg = self._conn.recv()
                self._ready = bool(msg.get("ready"))
                self._start_error = msg.get("error")
                if not self._ready:
                    self._kill()
                return self._ready
        except (EOFError, OSError) as e:
            self._start_error = f"Arbeiter beendet ({e})"
            self._kill()
            return False
        if not self._proc.is_alive():
            self._start_error = "Arbeiter beim Laden abgestuerzt"
            self._kill()
        return False

    def start(self, wait_s=120.0):
        """Arbeiter starten (falls keiner laeuft) und auf 'ready' warten."""
        with self._lock:
            if self._proc is None or not self._proc.is_alive():
                self._spawn()
            return self._await_ready(wait_s)

    def start_async(self):
        """Start im Hintergrund (der Controller soll nicht auf das Modell-
        Laden warten). Wartet in kurzen Scheiben, damit eine Anfrage
        dazwischen den Lock bekommt."""
        th = threading.Thread(target=self._background_start, daemon=True)
        th.start()
        return th

    def _background_start(self):
        with self._lock:
            if self._proc is None or not self._proc.is_alive():
                self._spawn()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 600.0:
            with self._lock:
                if self._ready or self._proc is None:
                    break
                self._await_ready(0.5)
            time.sleep(0.05)
        if self._log is not None:
            if self._ready:
                self._log.info(f"Armplaner-Prozess bereit ({time.monotonic() - t0:.1f} s).")
            else:
                self._log.error(f"Armplaner-Prozess nicht gestartet: {self._start_error}")

    @property
    def ready(self):
        return self._ready and self._proc is not None and self._proc.is_alive()

    @property
    def start_error(self):
        return self._start_error

    def _kill(self):
        try:
            if self._proc is not None and self._proc.is_alive():
                self._proc.kill()
                self._proc.join(1.0)
        except Exception:
            pass
        self._proc = None
        self._ready = False

    def cancel(self):
        """Laufende Planung abbrechen (Antwort kommt mit reason='cancelled')."""
        self._cancel.set()

    # ------------------------------------------------------------------
    def plan(self, request, timeout_s):
        """Anfrage senden und blockierend auf die Antwort warten (Aufruf aus
        einem Hintergrund-Thread). Serialisiert ueber einen Lock: eine noch
        laufende alte Anfrage wird vorher abgebrochen (cancel) und beendet."""
        with self._lock:
            if not self.ready:
                if self._proc is None or not self._proc.is_alive():
                    self._spawn()
                if not self._await_ready(max(timeout_s, 30.0)):
                    if self._proc is not None:
                        return {"ok": False, "reason": "Planer startet noch (Modell wird geladen) "
                                                       "-- gleich erneut versuchen"}
                    return {"ok": False, "reason": f"Planer nicht verfuegbar ({self._start_error})"}
            self._req_counter += 1
            req = dict(request)
            req["id"] = self._req_counter
            self._cancel.clear()
            try:
                self._conn.send(req)
            except (OSError, BrokenPipeError) as e:
                self._kill()
                return {"ok": False, "reason": f"Planer nicht erreichbar ({e})"}
            t_end = time.monotonic() + timeout_s
            while True:
                left = t_end - time.monotonic()
                if left <= 0:
                    # Haengt -> beenden; der naechste Aufruf startet neu.
                    self._kill()
                    return {"ok": False, "reason": "Planer-Zeitlimit ueberschritten (neu gestartet)"}
                try:
                    if self._conn.poll(min(left, 0.2)):
                        out = self._conn.recv()
                        if out.get("id") == req["id"]:
                            return out
                        # Antwort einer abgebrochenen Altanfrage -> verwerfen.
                except (EOFError, OSError) as e:
                    self._kill()
                    return {"ok": False, "reason": f"Planer-Prozess beendet ({e})"}
                if self._proc is None or not self._proc.is_alive():
                    self._kill()
                    return {"ok": False, "reason": "Planer-Prozess abgestuerzt (neu gestartet)"}

    def shutdown(self):
        try:
            if self._conn is not None:
                self._conn.send(None)
        except Exception:
            pass
        self._kill()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_servo_process.py — die Marker-/Jog-IK des arm_controller als EIGENER Prozess.

Warum: Im Marker-/Jog-Betrieb (PS4-Controller, Demo-GUI-Joystick, RViz-Marker)
rechnete main_loop JEDEN Takt die IK beider Arme (60 DLS-Iterationen je Arm,
~45 ms). Der 250-Hz-Takt kam dadurch real nur auf ~20-40 Hz; der Motor bekam
alle 25-50 ms einen Sollwert-Sprung von 10-25 mrad -> der Arm ratterte/hackte
hoerbar. Geplante Fahrten und Home kommen ohne IK aus und liefen mit 250 Hz
sauber. Ein Thread hilft nicht (die IK haelt das GIL, der Takt bekam dann
Luecken bis 100 ms) -- also wie der Armplaner ein eigener Prozess.

Ablauf: main_loop schickt je Takt den NEUESTEN Auftrag (gefilterte Ziele +
aktuelle Konfiguration, siehe G1IKSolver.solve_servo); ist der Arbeiter noch
beschaeftigt, ersetzt der neue Auftrag den wartenden. Ein Lesethread wartet
blockierend auf die Pipe (gibt das GIL frei) und legt das Ergebnis ab; der
Takt faehrt das Kommando mit seinen Limits (Gelenk-/TCP-Speed, Glaettung,
Anfahrrampe, Kollisions-Gate) in kleinen Schritten zum letzten Ergebnis.

  Auftrag   {gen, goals {side: 4x4}, current_all (29), cfg {null_space_gain,
             orientation_mode, servo_step_tol}}
  Ergebnis  {gen, out {side: q7 | None}}     None = divergiert -> halten
"""
import multiprocessing as mp
import os
import threading
import traceback

import numpy as np


def _worker_main(conn, cfg):
    try:
        os.nice(1)          # Regelkreis hat (leicht) Vorrang
    except Exception:
        pass
    try:
        import pinocchio as pin
        from g1pilot.utils.ik_solver import G1IKSolver

        ik = G1IKSolver(urdf_path=cfg["urdf_path"], mesh_dir=cfg["mesh_dir"], debug=False)
        for side, q7 in (cfg.get("rest") or {}).items():
            ik.set_rest_posture(side, np.asarray(q7, dtype=float))
        data = pin.Data(ik.model)
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
        out = {}
        try:
            c = req.get("cfg") or {}
            if "null_space_gain" in c:
                ik.null_space_gain = float(c["null_space_gain"])
            if "orientation_mode" in c and hasattr(ik, "set_orientation_mode"):
                ik.set_orientation_mode(c["orientation_mode"])
            if "servo_step_tol" in c:
                ik.servo_step_tol = float(c["servo_step_tol"]) or None
            cur = np.asarray(req["current_all"], dtype=float)
            for side, T in req["goals"].items():
                T = np.asarray(T, dtype=float)
                q7, diverged = ik.solve_servo(side, pin.SE3(T[:3, :3], T[:3, 3]), cur, data)
                out[side] = None if diverged else np.asarray(q7, dtype=float)
            conn.send({"gen": req["gen"], "out": out})
        except Exception as e:   # noqa: BLE001
            conn.send({"gen": req.get("gen"), "out": {}, "error": f"{e}"})


class IkServoProcess:
    """Arbeiterprozess + Lesethread. Alle Methoden sind nicht-blockierend."""

    def __init__(self, urdf_path, mesh_dir, rest=None, logger=None):
        self._cfg = {"urdf_path": urdf_path, "mesh_dir": mesh_dir,
                     "rest": {k: list(map(float, v)) for k, v in (rest or {}).items()}}
        self._log = logger
        self._ctx = mp.get_context("spawn")
        self._lock = threading.Lock()
        self._proc = None
        self._conn = None
        self._ready = False
        self._busy = False
        self._pending = None
        self._result = None
        self._stop = False
        self.error = None

    def start_async(self):
        parent, child = self._ctx.Pipe()
        self._proc = self._ctx.Process(target=_worker_main, args=(child, self._cfg),
                                       name="g1_ik_servo", daemon=True)
        self._proc.start()
        child.close()
        self._conn = parent
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        conn = self._conn
        try:
            msg = conn.recv()
        except (EOFError, OSError) as e:
            self.error = f"Arbeiter beendet ({e})"
            return
        if not msg.get("ready"):
            self.error = msg.get("error")
            if self._log is not None:
                self._log.error(f"IK-Prozess nicht gestartet: {self.error}")
            return
        self._ready = True
        if self._log is not None:
            self._log.info("IK-Prozess (Marker/Jog) bereit.")
        while not self._stop:
            try:
                msg = conn.recv()
            except (EOFError, OSError) as e:
                self.error = f"Arbeiter beendet ({e})"
                break
            if msg.get("error") and self._log is not None:
                self._log.warn(f"IK-Prozess: {msg['error']}", throttle_duration_sec=2.0)
            with self._lock:
                self._result = (msg.get("gen"), msg.get("out") or {})
                nxt, self._pending = self._pending, None
                self._busy = nxt is not None
            if nxt is not None:
                self._send(nxt)
        with self._lock:
            self._ready = False
        if self._log is not None and not self._stop:
            self._log.error(f"IK-Prozess ausgefallen ({self.error}) -- IK wieder im Regeltakt.")

    def _send(self, req):
        try:
            self._conn.send(req)
        except (OSError, BrokenPipeError, ValueError) as e:
            self.error = f"Pipe: {e}"
            with self._lock:
                self._ready = False
                self._busy = False

    @property
    def ready(self):
        return self._ready and self._proc is not None and self._proc.is_alive()

    def submit(self, gen, goals, current_all, cfg):
        """Neuesten Auftrag abgeben (ersetzt einen noch wartenden)."""
        req = {"gen": gen,
               "goals": {s: np.asarray(T.homogeneous, dtype=float) for s, T in goals.items()},
               "current_all": np.asarray(current_all, dtype=float), "cfg": cfg}
        with self._lock:
            if self._busy:
                self._pending = req
                return
            self._busy = True
        self._send(req)

    def result(self):
        """-> (gen, {side: q7 | None}) des letzten Ergebnisses oder None."""
        with self._lock:
            return self._result

    def shutdown(self):
        self._stop = True
        try:
            if self._conn is not None:
                self._conn.send(None)
        except Exception:
            pass
        try:
            if self._proc is not None and self._proc.is_alive():
                self._proc.join(0.5)
                if self._proc.is_alive():
                    self._proc.kill()
        except Exception:
            pass
        self._ready = False

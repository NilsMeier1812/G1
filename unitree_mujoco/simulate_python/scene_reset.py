# -*- coding: utf-8 -*-
"""
scene_reset.py — Umgebungs-Objekte zuruecksetzen, OHNE den Roboter anzufassen.

Setzt alle frei beweglichen Objekte der Umgebung (Koerper mit freiem Gelenk,
die NICHT zum G1 gehoeren -- z.B. eine heruntergefallene Box) auf ihre
Startpose aus der Szene (qpos0) zurueck und stoppt sie (qvel = 0). Der Roboter,
sein Loco-Zustand und die Arm-Steuerung laufen unveraendert weiter.

Warum UDP statt ROS-Topic? Wie bei push_listener.py/grasp_box.py: der
MuJoCo-Container hat KEIN ROS. Der Streamdeck-Button "RESET SCENE" (ROS, im
g1pilot-Container) wird von scene_bridge auf ein UDP-Datagramm an
127.0.0.1:<port> abgebildet (beide Container: network_mode host -> Loopback).
Von Hand:  echo reset | nc -u -w0 127.0.0.1 47903

Protokoll: jedes Datagramm = ein Reset (Payload egal).
"""
import socket
import threading

import mujoco


class SceneReset:
    def __init__(self, mj_model, config):
        self.model = mj_model
        self.port = int(getattr(config, "SCENE_RESET_PORT", 47903))
        self._lock = threading.Lock()
        self._pending = False

        # Freie Gelenke der Umgebung: alle, deren Koerper NICHT im Baum des
        # Roboters (Wurzel = pelvis) haengt.
        try:
            robot_root = mj_model.body_rootid[mj_model.body("pelvis").id]
        except Exception:
            robot_root = -1
        self.joints = []   # (Name, qpos-Adresse, dof-Adresse)
        for j in range(mj_model.njnt):
            if mj_model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
                continue
            body = mj_model.jnt_bodyid[j]
            if mj_model.body_rootid[body] == robot_root:
                continue
            self.joints.append((mj_model.body(body).name, int(mj_model.jnt_qposadr[j]),
                                int(mj_model.jnt_dofadr[j])))

        if not self.joints:
            return
        threading.Thread(target=self._listen, daemon=True).start()
        print(f"[scene-reset] UDP auf 127.0.0.1:{self.port} -- setzt "
              f"{len(self.joints)} Objekt(e) zurueck (Streamdeck 'RESET SCENE').", flush=True)

    def _listen(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", self.port))
        except Exception as e:
            print(f"[scene-reset] WARN: konnte UDP-Port {self.port} nicht binden: {e}")
            return
        while True:
            try:
                sock.recvfrom(64)
            except Exception:
                break
            with self._lock:
                self._pending = True

    # ── Pro Sim-Schritt aufrufen (unter locker) ──────────────────────────────
    def apply(self, mj_data):
        """Fuehrt einen ausstehenden Reset aus. Guenstig: tut nur etwas, wenn
        ein Befehl anliegt."""
        if not self.joints:
            return
        with self._lock:
            if not self._pending:
                return
            self._pending = False
        for _name, qadr, dadr in self.joints:
            mj_data.qpos[qadr:qadr + 7] = self.model.qpos0[qadr:qadr + 7]
            mj_data.qvel[dadr:dadr + 6] = 0.0
        mujoco.mj_forward(self.model, mj_data)
        print(f"[scene-reset] {len(self.joints)} Objekt(e) auf Startpose zurueckgesetzt: "
              + ", ".join(n for n, _, _ in self.joints), flush=True)

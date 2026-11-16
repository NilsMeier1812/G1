#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v7_gelenkwinkel_aufzeichnen.py -- V7: Soll- und Ist-Gelenkwinkel der Arme
waehrend einer gespeicherten Posenfolge aufzeichnen (Sim und realer G1).

Aufgezeichnet wird (alles nur ABONNIERT):
  rt/lowstate  Ist: Arm- und Taillengelenke q, dq, tau_est (Roboter / MuJoCo)
  rt/arm_sdk   Soll: q, kp der Armgelenke und das Blend-Gewicht (arm_controller)
  /g1pilot/pose_store/goto, /g1pilot/arm_command/status, /g1pilot/sequence/state
               Zeitpunkte der Schritte (goto -> accepted -> executing -> reached)

Ablauf auslösen:
  --trigger manuell  (Standard, einzige Moeglichkeit am realen G1)
      Der Bediener startet den Ablauf wie gewohnt in der Demo-GUI
      (»Posen & Abläufe« -> Ablauf, Modus »Am Stück«). Das Werkzeug wartet auf
      das erste goto und zeichnet bis zum Ende des Ablaufs auf
      (/g1pilot/sequence/state wieder idle) bzw. bis --max-dauer / Strg+C.
      Es sendet NICHTS.
  --trigger auto     (nur Simulation)
      Das Werkzeug faehrt den Ablauf selbst mit derselben Logik wie die
      Demo-GUI (teleoperation/sequence_runner.py, »Am Stück«): je Schritt
      /g1pilot/pose_store/goto, weiter nach 'reached' + wait_s + 0,3 s.

Identischer Ablauf: Gelenkwerte der Posen stehen aus der Pose-Datei mit im
summary.json; v7_auswerten.py prueft beide Aufzeichnungen darauf.

Aufruf:
    python3 tools/eval/v7_gelenkwinkel_aufzeichnen.py --ablauf "Arbeitsplatz 1" [--trigger auto]
Auswertung: tools/eval/v7_auswerten.py <sim-ordner> <real-ordner>
"""
import argparse
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_common as ec  # noqa: E402
import eval_ros as er  # noqa: E402

VERSUCH = "v7_gelenkwinkel"
WAIST_IDX = [12, 13, 14]


def load_sequence(name, poses, store_path=None):
    """-> (schritte, posen-definitionen, pfad). Liest nur."""
    from g1pilot.manipulation.pose_store import PoseStore
    store = PoseStore(store_path) if store_path else PoseStore()
    if poses:
        steps = [{"pose": p, "wait_s": 0.0} for p in poses]
    else:
        steps = store.get_sequence(name)
        if steps is None:
            names = [n for n, _ in store.list_sequences()]
            raise SystemExit(f"Ablauf '{name}' nicht in {store.path}. Vorhanden: {names}")
    defs = {}
    for st in steps:
        if st.get("move"):
            raise SystemExit("Relative Bewegungen ({'move'}) sind fuer V7 nicht vorgesehen: "
                             "ihr Ziel haengt von der aktuellen Handlage ab, der Ablauf waere "
                             "in Sim und Realbetrieb nicht identisch.")
        e = store.get(st["pose"])
        if e is None:
            raise SystemExit(f"Pose '{st['pose']}' fehlt in {store.path}.")
        defs[st["pose"]] = {k: e[k] for k in ("left_arm", "right_arm", "left_hand", "right_hand")
                            if k in e}
    return steps, defs, str(store.path)


def derive_steps(events):
    """Schritte aus den Ereignissen: je goto ein Schritt mit den Zeiten der
    folgenden pose_store-Statusmeldungen. events: [(t, quelle, dict|str)]."""
    steps, cur = [], None
    for t, src, data in sorted(events, key=lambda e: e[0]):
        if src == "goto":
            cur = {"pose": data, "t_goto": t, "t_accepted": None, "t_executing": None,
                   "t_ende": None, "zustand": None, "grund": ""}
            steps.append(cur)
        elif src == "status" and cur is not None and isinstance(data, dict):
            if data.get("source") not in ("pose_store", None):
                continue
            st = data.get("state")
            if st == "accepted" and cur["t_accepted"] is None:
                cur["t_accepted"] = t
            elif st == "executing" and cur["t_executing"] is None:
                cur["t_executing"] = t
            elif st in ("reached", "failed", "rejected", "cancelled") and cur["t_ende"] is None:
                if st != "rejected" and cur["t_accepted"] is None and cur["t_executing"] is None:
                    continue    # Endzustand der vorigen Fahrt (wie sequence_runner)
                cur["t_ende"] = t
                cur["zustand"] = st
                cur["grund"] = data.get("reason", "")
    return steps


class Recorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.ist, self.soll, self.events = [], [], []
        self.t0 = None
        self.active = False

    def rel(self, t):
        return t - self.t0

    def on_state(self, t, m):
        if not self.active:
            return
        row = [t, int(getattr(m, "tick", 0))]
        row += [float(m.motor_state[j].q) for j in WAIST_IDX]
        row += [float(m.motor_state[j].q) for j in er.ARM_IDX]
        row += [float(m.motor_state[j].dq) for j in er.ARM_IDX]
        row += [float(m.motor_state[j].tau_est) for j in er.ARM_IDX]
        with self.lock:
            self.ist.append(row)

    def on_cmd(self, t, m):
        if not self.active:
            return
        row = [t, er.arm_weight(m)] + er.arm_q_from_cmd(m) + er.arm_kp_from_cmd(m)
        with self.lock:
            self.soll.append(row)

    def event(self, src, data):
        t = time.monotonic()
        if not self.active:
            return
        with self.lock:
            self.events.append((t, src, data))


def main(argv=None):
    ap = argparse.ArgumentParser(description="V7: Soll-/Ist-Gelenkwinkel einer Posenfolge aufzeichnen.")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--ablauf", help="Name des gespeicherten Ablaufs (pose_store 'sequences')")
    g.add_argument("--posen", nargs="+", help="statt eines Ablaufs: Posennamen in dieser Reihenfolge")
    ap.add_argument("--trigger", choices=("manuell", "auto"), default="manuell",
                    help="manuell = Bediener startet in der Demo-GUI (Default, real einzig erlaubt); "
                         "auto = Werkzeug faehrt den Ablauf (nur Sim)")
    ap.add_argument("--pose-store", default=None, help="Pose-Datei (Default wie arm_controller)")
    ap.add_argument("--interface", default=os.getenv("ROBOT_INTERFACE", "lo"),
                    help="DDS-Interface im Realbetrieb (Sim: immer lo)")
    ap.add_argument("--vorlauf", type=float, default=2.0, help="Aufzeichnung vor dem ersten Schritt [s]")
    ap.add_argument("--nachlauf", type=float, default=3.0, help="Aufzeichnung nach dem letzten Schritt [s]")
    ap.add_argument("--max-dauer", type=float, default=600.0, help="Abbruch spaetestens nach [s]")
    ap.add_argument("--warte-start", type=float, default=300.0,
                    help="manuell: so lange auf das erste goto warten [s]")
    ap.add_argument("--schritt-timeout", type=float, default=60.0,
                    help="auto: Schritt gilt als haengend nach [s]")
    ec.add_common_args(ap, VERSUCH)
    a = ap.parse_args(argv)

    sim = er.is_sim()
    if a.trigger == "auto":
        er.assert_sim("--trigger auto")
    steps, defs, store_path = load_sequence(a.ablauf, a.posen, a.pose_store)
    run = ec.Run("v7_gelenkwinkel_aufzeichnen.py", VERSUCH, ec.env_mode(),
                 {"ablauf": a.ablauf, "posen": a.posen, "trigger": a.trigger, "pose_datei": store_path,
                  "schritte": steps, "vorlauf_s": a.vorlauf, "nachlauf_s": a.nachlauf,
                  "max_dauer_s": a.max_dauer, "umgebung": {k: os.getenv(k, "") for k in (
                      "HOLD_BASE_MODE", "SIM_LOCKSTEP", "G1_HANDS", "G1_ENV", "G1_EVAL_SIM_TIME")}},
                 out_root=a.out)

    rec = Recorder()
    tap = er.DdsTap(a.interface)
    tap.subscribe("rt/lowstate", rec.on_state, 1000)
    tap.subscribe("rt/arm_sdk", rec.on_cmd, 1000)
    node = er.make_node("eval_v7_gelenkwinkel")
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy
    seq_state = {"last": None}

    def on_status(m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        rec.event("status", d)
        if runner is not None:
            runner.on_status(d)

    def on_seq(m):
        try:
            d = json.loads(m.data)
        except Exception:
            d = {"raw": m.data}
        seq_state["last"] = (time.monotonic(), d)
        rec.event("sequence", d)

    node.create_subscription(String, "/g1pilot/arm_command/status", on_status, 50)
    node.create_subscription(String, "/g1pilot/pose_store/goto", lambda m: rec.event("goto", m.data), 50)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/g1pilot/sequence/state", on_seq, latched)

    runner, done = None, {"event": None, "text": ""}
    if a.trigger == "auto":
        from g1pilot.teleoperation.sequence_runner import SequenceRunner, MODE_AUTO
        op = er.SimOperator(node)

        def goto(step):
            op.goto(step["pose"])
            return None

        def on_change(_r, event, text):
            if event in ("done", "aborted"):
                done["event"], done["text"] = event, text

        runner = SequenceRunner(goto, on_change=on_change, mode=MODE_AUTO)

    import rclpy
    rec.t0 = time.monotonic()
    rec.active = True
    t_first = None
    if a.trigger == "auto":
        op.greifen()          # Arme uebernehmen (wie Kachel GREIFEN); im GREIFEN ohne Wirkung
        er.spin_until(node, time.monotonic() + a.vorlauf)
        print(f"[v7] Starte Ablauf ({len(steps)} Schritte) ...", flush=True)
        runner.start(steps, label=a.ablauf or "Posenfolge")
        t_first = time.monotonic()
        deadline = t_first + a.max_dauer
        while time.monotonic() < deadline and runner.active:
            rclpy.spin_once(node, timeout_sec=0.01)
            runner.tick()
        if runner.active:
            runner.cancel("max-dauer")
            done["event"], done["text"] = "aborted", "max-dauer erreicht"
    else:
        print(f"[v7] Warte auf Start des Ablaufs in der Demo-GUI (max. {a.warte_start:.0f} s) ...",
              flush=True)

        def started():
            return any(src == "goto" for _t, src, _d in rec.events)
        if not er.spin_until(node, time.monotonic() + a.warte_start, stop=started):
            rec.active = False
            tap.close()
            raise SystemExit("Kein goto gesehen -- Ablauf nicht gestartet?")
        t_first = next(t for t, src, _d in rec.events if src == "goto")
        print("[v7] Ablauf laeuft, zeichne auf ...", flush=True)
        deadline = t_first + a.max_dauer

        def finished():
            last = seq_state["last"]
            if last is None or last[0] < t_first:
                # ohne Demo-GUI-Status: Ende = so viele Endzustaende wie Schritte
                st = derive_steps(rec.events)
                return len(st) >= len(steps) and all(s["t_ende"] for s in st)
            return last[1].get("state") == "idle"
        try:
            er.spin_until(node, deadline, stop=finished)
        except KeyboardInterrupt:
            print("[v7] Abbruch durch Bediener.")
        done["event"] = "beendet"
    er.spin_until(node, time.monotonic() + a.nachlauf)
    rec.active = False
    tap.close()

    with rec.lock:
        ist, soll, events = list(rec.ist), list(rec.soll), list(rec.events)
    t0 = rec.t0
    names = er.ARM_NAMES
    run.write_csv("ist.csv",
                  ["t_s", "tick_ms", "waist_yaw", "waist_roll", "waist_pitch"]
                  + [f"q_{n}" for n in names] + [f"dq_{n}" for n in names] + [f"tau_{n}" for n in names],
                  [[round(r[0] - t0, 6)] + r[1:] for r in ist],
                  "Ist-Werte aus rt/lowstate (rad, rad/s, Nm); t = monotone Uhr ab Aufzeichnungsbeginn")
    run.write_csv("soll.csv", ["t_s", "gewicht"] + [f"q_{n}" for n in names] + [f"kp_{n}" for n in names],
                  [[round(r[0] - t0, 6)] + r[1:] for r in soll],
                  "Soll-Werte aus rt/arm_sdk (rad); gewicht = Blend motor_cmd[29].q")
    run.write_csv("ereignisse.csv", ["t_s", "quelle", "inhalt"],
                  [[round(t - t0, 6), src, json.dumps(d, ensure_ascii=False) if not isinstance(d, str) else d]
                   for t, src, d in sorted(events, key=lambda e: e[0])],
                  "goto / arm_command-Status / sequence-Status")
    dsteps = derive_steps(events)
    for s in dsteps:
        for k in ("t_goto", "t_accepted", "t_executing", "t_ende"):
            if s[k] is not None:
                s[k] = round(s[k] - t0, 6)
    ok = sum(1 for s in dsteps if s["zustand"] == "reached")
    summary = {"schritte_soll": steps, "posen": defs, "schritte": dsteps,
               "erreicht": ok, "anzahl_schritte": len(steps),
               "ende": done, "n_ist": len(ist), "n_soll": len(soll),
               "ist_rate_hz": (len(ist) - 1) / (ist[-1][0] - ist[0][0]) if len(ist) > 1 else 0.0,
               "soll_rate_hz": (len(soll) - 1) / (soll[-1][0] - soll[0][0]) if len(soll) > 1 else 0.0}
    if not soll:
        summary["hinweis"] = ("Keine rt/arm_sdk-Nachricht gesehen: Arme nicht uebernommen (GREIFEN) "
                              "oder falsches Interface.")
    print(f"[v7] {ok}/{len(steps)} Schritte erreicht, {len(ist)} Ist- / {len(soll)} Soll-Zeilen.")
    run.finish(summary)
    return 0 if ok == len(steps) else 2


if __name__ == "__main__":
    sys.exit(main())

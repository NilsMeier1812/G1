#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v4_echtzeit.py -- V4: Echtzeitfaktor und Takttreue der Simulation.

Misst ueber eine einstellbare Dauer (Standard 10 min je Lastfall):
  * Echtzeitfaktor = simulierte Zeit / verstrichene Wanduhrzeit,
  * Physikrate (Schritte je Wandsekunde, Soll 1000 Hz bei SIMULATE_DT 1 ms),
  * fuer jeden Regelkreis die tatsaechliche Rate und die Streuung der
    Zykluszeit gegenueber der Sollrate -- gemessen an den Ankunftszeiten
    seiner Ausgabe in diesem Prozess:
        sim_zustand   rt/lowstate   (MuJoCo-Bruecke; Lockstep: ein Regelzyklus)
        lauf_regler   rt/lowcmd     (loco_sim, Policy)
        arm_regler    rt/arm_sdk    (arm_controller.main_loop)
        roboterzustand /joint_states (robot_state)
        tf            /tf           (robot_state, TF-Baum)
Lastfaelle: stillstand (GREIFEN, keine Bewegung), arm (beide Arme fahren
geplant zwischen zwei Posen hin und her), gehen (GEHEN, Kreis mit
vx 0,3 m/s / wz 0,3 rad/s). Die Last erzeugt das Werkzeug selbst mit
denselben Topics wie die Demo-GUI -- NUR in der Simulation. Am realen G1
gibt es nur den Lastfall 'passiv' (misst, sendet nichts).

Simulierte Zeit: mit G1_EVAL_SIM_TIME=1 im mujoco-sim-Container steht die
Sim-Zeit in rt/lowstate.tick (Messpunkt in unitree_sdk2py_bridge.py, Standard
aus). Ohne den Messpunkt (tick bleibt 0) wird sie im Lockstep aus der Zahl
der rt/lowstate-Nachrichten x SIM_CONTROL_DT geschaetzt (Spalte methode).

Aufruf (im g1pilot-sim-Container, Stack laeuft):
    python3 tools/eval/v4_echtzeit.py [--dauer 600] [--faelle stillstand arm gehen]
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_common as ec  # noqa: E402
import eval_ros as er  # noqa: E402

VERSUCH = "v4_echtzeit"
LOOPS = ("sim_zustand", "lauf_regler", "arm_regler", "roboterzustand", "tf")
FAELLE = ("stillstand", "arm", "gehen", "passiv")
# Posen fuer den Lastfall 'arm' (beide Arme, linker Arm; rechts gespiegelt)
POSE_A = [0.3, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0]                       # home
POSE_B = [-0.6, 0.3, 0.0, 0.7, 0.0, 0.0, 0.0]                      # Arme vorn oben
MIRROR = [1, -1, -1, 1, -1, 1, -1]


def soll_rates(args, sim=True):
    """Sollraten [Hz] je Kreis und ihre Zeitbasis.

    Zeitbasis 'sim': der Kreis laeuft im Takt der simulierten Zeit (Lockstep:
    ein rt/lowstate je SIM_CONTROL_DT; loco_sim und robot_state haengen am
    rt/lowstate-Eingang) -> Soll in Wandzeit = Soll x Echtzeitfaktor.
    Zeitbasis 'wand': eigener ROS-Timer auf der Wanduhr (arm_controller).
    Zeitbasis None: kein einzelner Sender (tf = robot_state_publisher +
    robot_state + weitere) -> nur Rate und Streuung, keine Soll-Abweichung.
    Realbetrieb: rt/lowstate 500 Hz vom Roboter, alles in Wandzeit."""
    lock = os.getenv("SIM_LOCKSTEP", "1").strip().lower() in ("1", "true", "yes", "on")
    hold = os.getenv("HOLD_BASE_MODE", "off").strip().lower()
    lockstep = sim and lock and hold == "off"
    ctrl = 1.0 / float(os.getenv("SIM_CONTROL_DT", "0.02"))
    state_hz = ctrl if lockstep else (500.0 if not sim else 1.0 / max(args.sim_dt, 0.002))
    basis_state = "sim" if sim else "wand"
    soll = {
        "physik": 1.0 / args.sim_dt,
        "sim_zustand": state_hz,
        "lauf_regler": ctrl if sim else None,      # real: kein rt/lowcmd vom Stack (Herstellerregler)
        "arm_regler": args.arm_rate,
        "roboterzustand": state_hz,                # robot_state publiziert je rt/lowstate
        "tf": None,
    }
    basis = {"sim_zustand": basis_state, "lauf_regler": basis_state, "arm_regler": "wand",
             "roboterzustand": basis_state, "tf": None}
    return soll, basis, lockstep


class Collector:
    def __init__(self):
        self.lock = threading.Lock()
        self.events = {k: [] for k in LOOPS}
        self.ticks = []                       # (t_wall, tick_ms) aus rt/lowstate
        self.active = False

    def add(self, loop, t, tick=None):
        if not self.active:
            return
        with self.lock:
            self.events[loop].append(t)
            if tick is not None:
                self.ticks.append((t, tick))

    def take(self):
        with self.lock:
            ev = {k: list(v) for k, v in self.events.items()}
            ticks = list(self.ticks)
            for v in self.events.values():
                v.clear()
            self.ticks.clear()
        return ev, ticks


def loop_stats(times, soll_hz, basis, rtf):
    """Rate und Zykluszeit-Streuung eines Kreises aus Ankunftszeiten [s].

    soll_hz: Sollrate in der Zeitbasis 'basis' ('sim' | 'wand' | None).
    rtf: Echtzeitfaktor (fuer basis 'sim' -> Soll in Wandzeit)."""
    n = len(times)
    if basis == "sim":
        soll_wand = soll_hz * rtf if (soll_hz and rtf) else None
    elif basis == "wand":
        soll_wand = soll_hz
    else:
        soll_wand = None
    ist = (n - 1) / (times[-1] - times[0]) if n > 1 and times[-1] > times[0] else 0.0
    out = {"n": n, "zeitbasis": basis or "keine", "soll_hz": soll_hz, "soll_hz_wand": soll_wand,
           "ist_hz_wand": ist,
           "ist_hz_simzeit": (ist / rtf) if rtf else None,
           "ist_zu_soll": (ist / soll_wand) if soll_wand else None}
    if n < 3:
        out["hinweis"] = "zu wenige Nachrichten (Kreis laeuft nicht?)"
        return out, []
    dts = [b - a for a, b in zip(times[:-1], times[1:])]
    st = ec.describe(dts)
    out.update({"dt_mittel_s": st["mittel"], "dt_std_s": st["std"], "dt_min_s": st["min"],
                "dt_p50_s": st["median"], "dt_p95_s": st["p95"], "dt_p99_s": st["p99"],
                "dt_max_s": st["max"]})
    if soll_wand:
        T = 1.0 / soll_wand
        dev = [abs(d - T) for d in dts]
        out.update({"periode_soll_wand_s": T,
                    "abweichung_mittel_s": sum(dev) / len(dev),
                    "abweichung_p95_s": ec.percentile(dev, 95), "abweichung_max_s": max(dev),
                    "anteil_ueber_1_5_soll": sum(1 for d in dts if d > 1.5 * T) / len(dts),
                    "anteil_ueber_2_soll": sum(1 for d in dts if d > 2.0 * T) / len(dts),
                    "jitter_rel_std": st["std"] / T})
    return out, dts


def rtf_windows(ticks, window=1.0):
    """[(t_rel, rtf, physik_hz)] je Fenster aus (t_wall, tick_ms)."""
    if len(ticks) < 2:
        return []
    out = []
    t0 = ticks[0][0]
    i0 = 0
    for i in range(1, len(ticks)):
        if ticks[i][0] - ticks[i0][0] >= window:
            dw = ticks[i][0] - ticks[i0][0]
            ds = (ticks[i][1] - ticks[i0][1]) / 1000.0
            out.append((ticks[i0][0] - t0, ds / dw, ds / 0.001 / dw))
            i0 = i
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="V4: Echtzeitfaktor und Takttreue (Sim; real nur 'passiv').")
    ap.add_argument("--dauer", type=float, default=600.0, help="Messdauer je Lastfall [s] (Default 600)")
    ap.add_argument("--vorlauf", type=float, default=10.0, help="Einschwingzeit vor jeder Messung [s]")
    ap.add_argument("--faelle", nargs="+", default=None, choices=FAELLE,
                    help="Lastfaelle (Sim-Default: stillstand arm gehen; real: passiv)")
    ap.add_argument("--interface", default=os.getenv("ROBOT_INTERFACE", "lo"),
                    help="DDS-Interface im Realbetrieb (Sim: immer lo)")
    ap.add_argument("--arm-rate", type=float, default=250.0, help="Sollrate arm_controller (rate_hz)")
    ap.add_argument("--state-rate", type=float, default=50.0,
                    help="Sollrate robot_state (sim_rate_hz; real 500 Hz lowstate-getrieben)")
    ap.add_argument("--sim-dt", type=float, default=0.001, help="SIMULATE_DT der Sim [s]")
    ap.add_argument("--walk-vx", type=float, default=0.3)
    ap.add_argument("--walk-wz", type=float, default=0.3)
    ap.add_argument("--no-plot", action="store_true")
    ec.add_common_args(ap, VERSUCH)
    a = ap.parse_args(argv)

    sim = er.is_sim()
    faelle = a.faelle or (["stillstand", "arm", "gehen"] if sim else ["passiv"])
    if not sim and any(f != "passiv" for f in faelle):
        ap.error("Am realen G1 nur --faelle passiv (das Werkzeug sendet dort nichts).")
    soll, basis, lockstep = soll_rates(a, sim)
    env = {k: os.getenv(k, "") for k in ("SIM_LOCKSTEP", "HOLD_BASE_MODE", "SIM_REALTIME_FACTOR",
                                        "SIM_CONTROL_DT", "G1_WALK_POLICY", "G1_EVAL_SIM_TIME",
                                        "G1_HANDS", "G1_ENV")}
    run = ec.Run("v4_echtzeit.py", VERSUCH, ec.env_mode(),
                 {"dauer_s": a.dauer, "vorlauf_s": a.vorlauf, "faelle": faelle,
                  "soll_hz": soll, "zeitbasis": basis, "lockstep": lockstep, "umgebung": env,
                  "arm_posen": {"a": POSE_A, "b": POSE_B}, "gehen": {"vx": a.walk_vx, "wz": a.walk_wz}},
                 out_root=a.out)

    col = Collector()
    tap = er.DdsTap(a.interface)

    def on_state(t, m):
        col.add("sim_zustand", t, int(getattr(m, "tick", 0)))
    tap.subscribe("rt/lowstate", on_state, 200)
    tap.subscribe("rt/lowcmd", lambda t, m: col.add("lauf_regler", t), 200)
    tap.subscribe("rt/arm_sdk", lambda t, m: col.add("arm_regler", t), 500)
    node = er.make_node("eval_v4_echtzeit")
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    node.create_subscription(JointState, "/joint_states",
                             lambda m: col.add("roboterzustand", time.monotonic()), 200)
    node.create_subscription(TFMessage, "/tf", lambda m: col.add("tf", time.monotonic()), 200)
    status = er.ArmStatus(node)
    op = er.SimOperator(node) if sim else None

    raw_rows, loop_rows, rtf_rows, summary = [], [], [], {"faelle": {}}
    plot_data = {}
    for fall in faelle:
        print(f"[v4] Lastfall {fall}: Vorlauf {a.vorlauf:.0f} s ...", flush=True)
        # Last vorbereiten
        if op is not None:
            if fall in ("stillstand", "arm"):
                op.greifen()
            elif fall == "gehen":
                op.gehen()
        state = {"k": 0, "req": None, "t_req": 0.0, "moves": [], "last_vel": 0.0}

        def load_tick():
            now = time.monotonic()
            if op is None:
                return
            if fall == "gehen" and now - state["last_vel"] >= 0.1:
                op.vel(a.walk_vx, 0.0, a.walk_wz)
                state["last_vel"] = now
            if fall == "arm":
                if state["req"] is not None:
                    done = status.terminal(state["req"], state["t_req"])
                    if done is None and now - state["t_req"] < 60.0:
                        return
                    state["moves"].append({"id": state["req"], "start": state["t_req"],
                                           "ende": done[0] if done else now,
                                           "zustand": done[1].get("state") if done else "timeout"})
                state["k"] += 1
                q = POSE_B if state["k"] % 2 else POSE_A
                state["req"] = f"v4_{fall}_{state['k']}"
                state["t_req"] = now
                op.joints(state["req"], left=q, right=[s * v for s, v in zip(MIRROR, q)])

        t_end = time.monotonic() + a.vorlauf
        while time.monotonic() < t_end:
            import rclpy
            rclpy.spin_once(node, timeout_sec=0.02)
            load_tick()
        col.take()
        col.active = True
        t_start = time.monotonic()
        t_end = t_start + a.dauer
        last_print = t_start
        while time.monotonic() < t_end:
            import rclpy
            rclpy.spin_once(node, timeout_sec=0.005)
            load_tick()
            if time.monotonic() - last_print > 60:
                last_print = time.monotonic()
                print(f"[v4] {fall}: {last_print - t_start:.0f}/{a.dauer:.0f} s", flush=True)
        col.active = False
        t_stop = time.monotonic()
        ev, ticks = col.take()
        if op is not None and fall == "gehen":
            for _ in range(20):
                op.vel(0.0, 0.0, 0.0)
                time.sleep(0.05)
            op.greifen()

        # Sim-Zeit
        nz = [(t, k) for t, k in ticks if k]
        if len(nz) >= 2:
            methode = "tick (G1_EVAL_SIM_TIME=1)"
            sim_s = (nz[-1][1] - nz[0][1]) / 1000.0
            wall_s = nz[-1][0] - nz[0][0]
            windows = rtf_windows(nz)
        elif lockstep and len(ticks) >= 2:
            methode = "geschaetzt: Anzahl rt/lowstate x SIM_CONTROL_DT (Lockstep)"
            dt_c = float(os.getenv("SIM_CONTROL_DT", "0.02"))
            sim_s = (len(ticks) - 1) * dt_c
            wall_s = ticks[-1][0] - ticks[0][0]
            est = [(t, int(round(i * dt_c * 1000))) for i, (t, _k) in enumerate(ticks)]
            windows = rtf_windows(est)
        else:
            methode = "nicht bestimmbar (kein tick, kein Lockstep)" if sim else "Realbetrieb: entfaellt"
            sim_s, wall_s, windows = None, t_stop - t_start, []
        rtf = (sim_s / wall_s) if (sim_s and wall_s) else None
        f_sum = {"methode_simzeit": methode, "wandzeit_s": t_stop - t_start, "simzeit_s": sim_s,
                 "echtzeitfaktor": rtf,
                 "physik_hz_wand": (sim_s / a.sim_dt / wall_s) if (sim_s and wall_s) else None,
                 "physik_soll_hz": soll["physik"],
                 "echtzeitfaktor_1s_fenster": ec.describe([w[1] for w in windows]),
                 "physik_hz_1s_fenster": ec.describe([w[2] for w in windows]),
                 "kreise": {}}
        if fall == "arm":
            moves = state["moves"]
            f_sum["armfahrten"] = {"anzahl": len(moves),
                                   "zustaende": {z: sum(1 for m in moves if m["zustand"] == z)
                                                 for z in set(m["zustand"] for m in moves)}}
        for w in windows:
            rtf_rows.append([fall, w[0], w[1], w[2]])
        rtf_sim = rtf if sim else 1.0
        plot_data[fall] = {"windows": windows, "dts": {}}
        for loop in LOOPS:
            times = ev[loop]
            st, dts = loop_stats(times, soll[loop], basis[loop], rtf_sim)
            f_sum["kreise"][loop] = st
            plot_data[fall]["dts"][loop] = dts
            row = {"fall": fall, "kreis": loop}
            row.update({k: v for k, v in st.items() if not isinstance(v, dict)})
            loop_rows.append(row)
            for t in times:
                raw_rows.append([fall, loop, round(t - t_start, 6)])
        for t, k in ticks:
            raw_rows.append([fall, "sim_zustand_tick_ms", round(t - t_start, 6), k])
        summary["faelle"][fall] = f_sum
        print(f"[v4] {fall}: Echtzeitfaktor {rtf if rtf is None else round(rtf, 3)} ({methode}); "
              + ", ".join(f"{k} {f_sum['kreise'][k]['ist_hz_wand']:.1f} Hz" for k in LOOPS), flush=True)

    tap.close()
    run.write_csv("ereignisse.csv", ["fall", "kreis", "t_s", "tick_ms"], raw_rows,
                  "Ankunftszeit jeder Nachricht (monotone Uhr, relativ zum Messbeginn je Fall)")
    cols = sorted({k for r in loop_rows for k in r}, key=lambda k: (k not in ("fall", "kreis"), k))
    run.write_csv("kreise.csv", cols, loop_rows, "Rate und Zykluszeit-Streuung je Regelkreis und Lastfall")
    run.write_csv("echtzeitfaktor.csv", ["fall", "t_s", "echtzeitfaktor", "physik_hz"], rtf_rows,
                  "Echtzeitfaktor und Physikrate je 1-s-Fenster")
    summary["hinweise"] = [
        "Zykluszeit = Abstand der Ankunftszeiten in diesem Prozess; enthaelt die DDS-/ROS-"
        "Zustellung, nicht nur den Takt des Senders.",
        "Sollrate in Wandzeit = Sollrate (Sim-Zeit) x Echtzeitfaktor; im Lockstep laufen "
        "lowstate/lowcmd absichtlich mit der Sim-Zeit.",
        "arm_regler laeuft auf einem ROS-Timer (Wandzeit, 250 Hz), unabhaengig vom Echtzeitfaktor.",
        "tf hat mehrere Sender (robot_state_publisher, robot_state, ...): nur Rate/Streuung, kein Soll.",
        "Realbetrieb (Lastfall passiv): lauf_regler entfaellt (rt/lowcmd sendet dort der Herstellerregler "
        "nicht ueber DDS); Physik/Echtzeitfaktor entfallen.",
    ]
    if not a.no_plot:
        try:
            plot(run, plot_data, soll)
        except Exception as e:      # noqa: BLE001
            summary["diagramm_fehler"] = str(e)
            print(f"[v4] Diagramme nicht erzeugt: {e}")
    run.finish(summary)
    return 0


def plot(run, data, soll):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    with PdfPages(run.path("diagramme.pdf")) as pdf:
        fig, ax = plt.subplots(figsize=(10, 4.5))
        for fall, d in data.items():
            if d["windows"]:
                ax.plot([w[0] for w in d["windows"]], [w[1] for w in d["windows"]], label=fall)
        ax.axhline(1.0, color="k", lw=0.8, ls="--")
        ax.set_xlabel("Zeit seit Messbeginn [s]")
        ax.set_ylabel("Echtzeitfaktor (1-s-Fenster)")
        ax.set_title("V4 Echtzeitfaktor")
        ax.legend()
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)
        for fall, d in data.items():
            fig, axs = plt.subplots(1, len(LOOPS), figsize=(16, 3.8))
            for ax, loop in zip(axs, LOOPS):
                dts = [x * 1000 for x in d["dts"].get(loop, [])]
                if dts:
                    ax.hist(dts, bins=60, color="#4c78a8")
                ax.set_title(f"{loop} ({len(dts)})", fontsize=9)
                ax.set_xlabel("Zykluszeit [ms]")
            fig.suptitle(f"V4 Zykluszeiten, Lastfall {fall}")
            fig.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)
    run.files.append("diagramme.pdf")


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""walk_diag -- Geh-Diagnose am echten G1 aufzeichnen und auswerten (NUR LESEND).

Anlass (Nils 2026-10-23): der G1 driftet beim Gehen mit dem Onboard-Regler
stark nach links (geradeaus, Drehen auf der Stelle, seitlich rechts kaum).
Bericht: /mnt/project-files/analysen/linksdrall_gehen_g1.md.

Das Werkzeug sendet NICHTS an den Roboter: kein RPC, kein rt/arm_sdk, kein
rt/lowcmd. Es liest per Abfrage (Read() im Takt, KEIN Listener -- siehe
loco_client._init_lowstate) und laeuft als eigener Prozess neben dem Stack.

  record   Aufzeichnen als CSV (Standard 50 Hz):
             rt/lowstate   IMU (rpy, Gyro, Beschl.), Beine + Taille q/dq/tau/
                           Temperatur/Fehlerbits, Armstellung, Funk-Fernbedienung
             rt/lowcmd     Sollwerte des Onboard-Reglers (falls sichtbar)
             Odometrie     rt/odommodestate o. ae. (falls sichtbar)
             ROS (optional) /g1pilot/ps4/walk_cmd, /g1pilot/loco_cmd_vel,
                           /g1pilot/onboard_state, /g1pilot/waist/state
  analyze  CSV auswerten: Stand (Lastverteilung, IMU-Neigung) und je
           Geh-Abschnitt (Befehl vs. Odometrie/Gyro, Seitenvergleich).

Aufruf im laufenden Real-Container (Interface = ROBOT_INTERFACE):
    docker compose exec g1pilot-real python3 -m g1pilot.tools.walk_diag \\
        record enp0s31f6 --label geradeaus_arme_onboard
    docker compose exec g1pilot-real python3 -m g1pilot.tools.walk_diag \\
        analyze diag_logs/<datei>.csv
Die CSV landet in g1pilot/diag_logs/ (im Container /ros2_ws/src/g1pilot).
Ende der Aufnahme: Strg+C oder --duration.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import struct
import sys
import time

# ── G1-29: Gelenk-Indizes ──────────────────────────────────────────────────
LEG_NAMES = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
LEFT_LEG = {n: i for i, n in enumerate(LEG_NAMES)}          # 0..5
RIGHT_LEG = {n: 6 + i for i, n in enumerate(LEG_NAMES)}     # 6..11
WAIST = {"waist_yaw": 12, "waist_roll": 13, "waist_pitch": 14}
ARM_JOINTS = tuple(range(15, 29))
# Gespiegelte Gelenke (roll/yaw): links ~ -rechts; Summe = Asymmetrie.
# pitch/knee: links ~ rechts; Differenz = Asymmetrie.
MIRRORED = ("hip_roll", "hip_yaw", "ankle_roll")

ODOM_TOPICS = ("rt/odommodestate", "rt/lf/odommodestate",
               "rt/sportmodestate", "rt/lf/sportmodestate")
ROS_CMD_TOPICS = ("/g1pilot/ps4/walk_cmd", "/g1pilot/loco_cmd_vel")
STAND_CMD_EPS = 0.03     # wie loco_client: darunter = Stopp


def _joint_label(j):
    for side, tab in (("L", LEFT_LEG), ("R", RIGHT_LEG)):
        for n, i in tab.items():
            if i == j:
                return f"{side}_{n}"
    for n, i in WAIST.items():
        if i == j:
            return n
    return f"j{j}"


LEG_WAIST_JOINTS = tuple(range(0, 15))
JOINT_LABELS = {j: _joint_label(j) for j in LEG_WAIST_JOINTS}


def columns(with_cmd_ref=True):
    """Spaltenreihenfolge der CSV."""
    cols = ["t", "tick", "fsm", "balanced", "cmd_src", "cmd_vx", "cmd_vy", "cmd_wz",
            "waist_target_deg",
            "imu_roll", "imu_pitch", "imu_yaw", "gyro_x", "gyro_y", "gyro_z",
            "acc_x", "acc_y", "acc_z", "imu_temp"]
    for j in LEG_WAIST_JOINTS:
        lb = JOINT_LABELS[j]
        cols += [f"{lb}_q", f"{lb}_dq", f"{lb}_tau", f"{lb}_temp", f"{lb}_err"]
        if with_cmd_ref:
            cols += [f"{lb}_qcmd", f"{lb}_kp"]
    cols += [f"arm{j}_q" for j in ARM_JOINTS]
    cols += ["odom_px", "odom_py", "odom_vx", "odom_vy", "odom_wz",
             "rc_lx", "rc_ly", "rc_rx", "rc_ry"]
    return cols


# ── Umrechnungen (ohne DDS testbar) ─────────────────────────────────────────
def parse_remote(raw):
    """lowstate.wireless_remote (40 Byte) -> (lx, ly, rx, ry) oder None.
    Unitree-Layout (xRockerBtnDataStruct): head[2], btn[2], lx, rx, ry, L2, ly
    als float32 little endian ab Byte 4."""
    try:
        b = bytes(bytearray(int(x) & 0xFF for x in raw))
    except (TypeError, ValueError):
        return None
    if len(b) < 24:
        return None
    lx, rx, ry, _l2, ly = struct.unpack_from("<5f", b, 4)
    vals = (lx, ly, rx, ry)
    if not all(math.isfinite(v) and abs(v) <= 1.5 for v in vals):
        return None
    return vals


def body_velocity(vx_world, vy_world, yaw):
    """Welt-Geschwindigkeit -> Koerper-Frame (vorn, links)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx_world + s * vy_world, -s * vx_world + c * vy_world


def _temp(t):
    try:
        return float(max(t))
    except TypeError:
        return float(t)


def lowstate_fields(msg):
    """LowState_ (oder Attrappe mit gleichen Feldern) -> dict der CSV-Spalten."""
    imu = msg.imu_state
    row = {"tick": int(getattr(msg, "tick", 0)),
           "imu_roll": float(imu.rpy[0]), "imu_pitch": float(imu.rpy[1]),
           "imu_yaw": float(imu.rpy[2]),
           "gyro_x": float(imu.gyroscope[0]), "gyro_y": float(imu.gyroscope[1]),
           "gyro_z": float(imu.gyroscope[2]),
           "acc_x": float(imu.accelerometer[0]), "acc_y": float(imu.accelerometer[1]),
           "acc_z": float(imu.accelerometer[2]),
           "imu_temp": float(getattr(imu, "temperature", 0))}
    for j in LEG_WAIST_JOINTS:
        m = msg.motor_state[j]
        lb = JOINT_LABELS[j]
        row[f"{lb}_q"] = float(m.q)
        row[f"{lb}_dq"] = float(m.dq)
        row[f"{lb}_tau"] = float(m.tau_est)
        row[f"{lb}_temp"] = _temp(m.temperature)
        row[f"{lb}_err"] = int(getattr(m, "motorstate", 0))
    for j in ARM_JOINTS:
        row[f"arm{j}_q"] = float(msg.motor_state[j].q)
    rc = parse_remote(getattr(msg, "wireless_remote", ()))
    if rc is not None:
        row["rc_lx"], row["rc_ly"], row["rc_rx"], row["rc_ry"] = rc
    return row


def lowcmd_fields(msg):
    row = {}
    for j in LEG_WAIST_JOINTS:
        c = msg.motor_cmd[j]
        lb = JOINT_LABELS[j]
        row[f"{lb}_qcmd"] = float(c.q)
        row[f"{lb}_kp"] = float(c.kp)
    return row


def odom_fields(msg):
    vx_b, vy_b = body_velocity(float(msg.velocity[0]), float(msg.velocity[1]),
                               float(msg.imu_state.rpy[2]))
    return {"odom_px": float(msg.position[0]), "odom_py": float(msg.position[1]),
            "odom_vx": vx_b, "odom_vy": vy_b, "odom_wz": float(msg.yaw_speed)}


# ════════════════════════════════════════════════════════════════════════
#  Auswertung (reine Python-Funktionen, ohne DDS/ROS)
# ════════════════════════════════════════════════════════════════════════
def _f(row, key):
    v = row.get(key, "")
    if v in ("", None):
        return None
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _mean(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else None


def load_csv(path):
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    meta = {}
    meta_path = os.path.splitext(path)[0] + ".json"
    if os.path.exists(meta_path):
        with open(meta_path) as fh:
            meta = json.load(fh)
    return rows, meta


def classify(row):
    """Befehl -> Abschnittsart: 'stand' | '+vx' | '-vx' | '+vy' | ... | None."""
    vals = {k: _f(row, k) for k in ("cmd_vx", "cmd_vy", "cmd_wz")}
    if any(v is None for v in vals.values()):
        return None
    big = max(vals, key=lambda k: abs(vals[k]))
    if abs(vals[big]) < STAND_CMD_EPS:
        return "stand"
    return ("+" if vals[big] > 0 else "-") + big[4:]


def segments(rows, min_s=1.5):
    """Zusammenhaengende Abschnitte gleicher Art (nur Walk-FSM, wenn bekannt)."""
    out, cur = [], None
    for r in rows:
        t = _f(r, "t")
        fsm = _f(r, "fsm")
        kind = classify(r)
        if fsm is not None and int(fsm) not in (500, 501):
            kind = None
        if cur is not None and kind == cur["kind"]:
            cur["rows"].append(r)
            cur["t1"] = t
            continue
        if cur is not None and cur["kind"] is not None and cur["t1"] - cur["t0"] >= min_s:
            out.append(cur)
        cur = {"kind": kind, "t0": t, "t1": t, "rows": [r]}
    if cur is not None and cur["kind"] is not None and cur["t1"] - cur["t0"] >= min_s:
        out.append(cur)
    return out


def side_compare(rows, field):
    """Mittelwerte links/rechts je Beingelenk -> {name: (L, R, Asymmetrie)}.
    Asymmetrie: roll/yaw-Gelenke L+R (gespiegelt), sonst L-R."""
    res = {}
    for n in LEG_NAMES:
        lv = _mean(_f(r, f"L_{n}_{field}") for r in rows)
        rv = _mean(_f(r, f"R_{n}_{field}") for r in rows)
        if lv is None or rv is None:
            continue
        res[n] = (lv, rv, lv + rv if n in MIRRORED else lv - rv)
    return res


def tracking_rms(rows):
    """RMS(q_cmd - q) je Beingelenk L/R (nur wenn rt/lowcmd aufgezeichnet)."""
    res = {}
    for n in LEG_NAMES:
        acc = {}
        for side in ("L", "R"):
            e = []
            for r in rows:
                qc, q = _f(r, f"{side}_{n}_qcmd"), _f(r, f"{side}_{n}_q")
                if qc is not None and q is not None:
                    e.append((qc - q) ** 2)
            acc[side] = math.sqrt(sum(e) / len(e)) if e else None
        if acc["L"] is not None and acc["R"] is not None:
            res[n] = (acc["L"], acc["R"])
    return res


def segment_summary(seg, meta):
    rows = seg["rows"]
    scale = {"vx": meta.get("max_vx", 0.4), "vy": meta.get("max_vy", 0.3),
             "wz": meta.get("max_vyaw", 0.4)}
    s = {"kind": seg["kind"], "t0": seg["t0"], "dauer_s": seg["t1"] - seg["t0"]}
    for ax in ("vx", "vy", "wz"):
        m = _mean(_f(r, f"cmd_{ax}") for r in rows)
        s[f"cmd_{ax}"] = None if m is None else m * scale[ax]
        s[f"odom_{ax}"] = _mean(_f(r, f"odom_{ax}") for r in rows)
    s["gyro_z"] = _mean(_f(r, "gyro_z") for r in rows)
    s["imu_roll_deg"] = _deg(_mean(_f(r, "imu_roll") for r in rows))
    s["imu_pitch_deg"] = _deg(_mean(_f(r, "imu_pitch") for r in rows))
    yaws = [_f(r, "imu_yaw") for r in rows if _f(r, "imu_yaw") is not None]
    s["yaw_change_deg"] = _deg(_unwrap_delta(yaws)) if len(yaws) > 1 else None
    px = [(_f(r, "odom_px"), _f(r, "odom_py")) for r in rows]
    px = [p for p in px if None not in p]
    if len(px) > 1 and yaws:
        dx, dy = px[-1][0] - px[0][0], px[-1][1] - px[0][1]
        fwd, left = body_velocity(dx, dy, yaws[0])
        s["weg_vor_m"], s["weg_links_m"] = fwd, left
    s["tau"] = side_compare(rows, "tau")
    s["tracking"] = tracking_rms(rows)
    return s


def _deg(v):
    return None if v is None else math.degrees(v)


def _unwrap_delta(vals):
    total = 0.0
    for a, b in zip(vals, vals[1:]):
        d = b - a
        d = (d + math.pi) % (2 * math.pi) - math.pi
        total += d
    return total


def temperature_peaks(rows):
    res = {}
    for n in LEG_NAMES:
        lv = [_f(r, f"L_{n}_temp") for r in rows]
        rv = [_f(r, f"R_{n}_temp") for r in rows]
        lv = [v for v in lv if v is not None]
        rv = [v for v in rv if v is not None]
        if lv and rv:
            res[n] = (max(lv), max(rv))
    return res


def motor_errors(rows):
    res = {}
    for j in LEG_WAIST_JOINTS:
        lb = JOINT_LABELS[j]
        bits = {int(v) for v in (_f(r, f"{lb}_err") for r in rows) if v}
        if bits:
            res[lb] = sorted(bits)
    return res


def analyze(rows, meta):
    """-> Textbericht (Liste von Zeilen)."""
    out = [f"Aufnahme: {meta.get('label', '?')}  ({len(rows)} Zeilen, "
           f"Quellen: {', '.join(meta.get('sources', [])) or '?'})"]
    segs = segments(rows)
    stands = [s for s in segs if s["kind"] == "stand"]
    walks = [s for s in segs if s["kind"] != "stand"]
    if stands:
        rows_st = [r for s in stands for r in s["rows"]]
        out.append("")
        out.append(f"== Stehen, Befehl 0 ({sum(s['t1'] - s['t0'] for s in stands):.0f} s) ==")
        roll = _deg(_mean(_f(r, "imu_roll") for r in rows_st))
        pitch = _deg(_mean(_f(r, "imu_pitch") for r in rows_st))
        if roll is not None:
            out.append(f"IMU-Neigung: Roll {roll:+.2f} deg, Pitch {pitch:+.2f} deg "
                       "(Roll +: rechte Seite tiefer)")
        out += _side_lines("Moment tau [Nm]", side_compare(rows_st, "tau"))
        out += _side_lines("Stellung q [deg]", side_compare(rows_st, "q"), deg=True)
        wr = _deg(_mean(_f(r, "waist_roll_q") for r in rows_st))
        wy = _deg(_mean(_f(r, "waist_yaw_q") for r in rows_st))
        if wr is not None:
            out.append(f"Taille: Roll {wr:+.2f} deg, Yaw {wy:+.2f} deg")
    for s in walks:
        sm = segment_summary(s, meta)
        out.append("")
        out.append(f"== Gehen {sm['kind']} ab t={sm['t0']:.1f} s, {sm['dauer_s']:.1f} s ==")
        out.append("Befehl  vx {:+.3f} m/s  vy {:+.3f} m/s  wz {:+.3f} rad/s".format(
            *(sm[f"cmd_{a}"] or 0.0 for a in ("vx", "vy", "wz"))))
        if sm["odom_vx"] is not None:
            out.append("Odometr vx {:+.3f} m/s  vy {:+.3f} m/s  wz {:+.3f} rad/s".format(
                sm["odom_vx"], sm["odom_vy"] or 0.0, sm["odom_wz"] or 0.0))
            out.append("Abweichung seitlich (Odometrie - Befehl): {:+.3f} m/s (+ = links)".format(
                (sm["odom_vy"] or 0.0) - (sm["cmd_vy"] or 0.0)))
        if "weg_links_m" in sm:
            out.append(f"Weg: {sm['weg_vor_m']:+.2f} m vor, {sm['weg_links_m']:+.2f} m links")
        if sm["gyro_z"] is not None:
            out.append(f"Gyro z (Becken) {sm['gyro_z']:+.3f} rad/s, Blickrichtung "
                       f"{(sm['yaw_change_deg'] or 0.0):+.1f} deg (+ = nach links gedreht)")
        if sm["imu_roll_deg"] is not None:
            out.append(f"IMU-Neigung im Mittel: Roll {sm['imu_roll_deg']:+.2f} deg, "
                       f"Pitch {sm['imu_pitch_deg']:+.2f} deg")
        out += _side_lines("Moment tau [Nm]", sm["tau"])
        if sm["tracking"]:
            out.append("Regelabweichung RMS(q_soll - q) [deg] L / R:")
            for n, (lv, rv) in sm["tracking"].items():
                flag = "  <-- Seiten ungleich" if max(lv, rv) > 1.5 * min(lv, rv) + 0.002 else ""
                out.append(f"   {n:12s} {math.degrees(lv):6.2f} / {math.degrees(rv):6.2f}{flag}")
    temps = temperature_peaks(rows)
    if temps:
        out.append("")
        out.append("== Hoechste Motortemperatur [degC] L / R ==")
        for n, (lv, rv) in temps.items():
            flag = "  <-- >10 degC Unterschied" if abs(lv - rv) > 10 else ""
            out.append(f"   {n:12s} {lv:5.0f} / {rv:5.0f}{flag}")
    errs = motor_errors(rows)
    if errs:
        out.append("")
        out.append("== Motor-Fehlerbits (motorstate != 0) ==")
        for lb, bits in errs.items():
            out.append(f"   {lb}: {bits}")
    if not walks and not stands:
        out.append("Keine Abschnitte erkannt (kein Befehl/FSM aufgezeichnet? ROS-Teil aus?).")
    return out


def _side_lines(title, comp, deg=False):
    if not comp:
        return []
    k = math.degrees if deg else (lambda v: v)
    lines = [f"{title} L / R / Asymmetrie (roll+yaw: L+R, sonst L-R):"]
    for n, (lv, rv, asym) in comp.items():
        lines.append(f"   {n:12s} {k(lv):+8.2f} / {k(rv):+8.2f} / {k(asym):+7.2f}")
    return lines


# ════════════════════════════════════════════════════════════════════════
#  Aufnahme (DDS + optional ROS)
# ════════════════════════════════════════════════════════════════════════
def _discover_writers(listen_s):
    """Topics mit Schreiber auf der Unitree-Domain (wie dds_api_check)."""
    from unitree_sdk2py.core.channel import ChannelFactory
    from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
    dp = ChannelFactory()._ChannelFactory__participant
    pubs = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)
    seen = set()
    t_end = time.monotonic() + listen_s
    while time.monotonic() < t_end:
        for p in pubs.take(N=200):
            if getattr(p, "topic_name", None):
                seen.add(p.topic_name)
        time.sleep(0.1)
    return seen


class Recorder:
    def __init__(self, args):
        self.args = args
        self.row = {}
        self.cmd = {}           # topic -> (t, vx, vy, wz)
        self.sources = []

    def open_dds(self):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        ChannelFactoryInitialize(0, self.args.iface)
        topics = _discover_writers(self.args.discover_s)
        print(f"[walk_diag] rt/-Topics mit Schreiber: {sorted(t for t in topics if t.startswith('rt/'))}")
        self.subs = []

        def sub(name, typ, fn):
            s = ChannelSubscriber(name, typ)
            s.Init()                    # ohne Listener -> Read() im Takt
            self.subs.append((name, s, fn))
            self.sources.append(name)

        sub("rt/lowstate", LowState_, lowstate_fields)
        if "rt/lowcmd" in topics and not self.args.no_lowcmd:
            sub("rt/lowcmd", LowCmd_, lowcmd_fields)
        for t in ODOM_TOPICS:
            if t in topics:
                sub(t, SportModeState_, odom_fields)
                break
        return topics

    def poll_dds(self):
        for name, s, fn in self.subs:
            try:
                msg = s.Read(0.002)
            except Exception:
                msg = None
            if msg is not None:
                try:
                    self.row.update(fn(msg))
                except Exception as e:      # ein kaputtes Paket beendet nichts
                    print(f"[walk_diag] {name} unlesbar: {e}", file=sys.stderr)

    def on_cmd(self, topic, msg):
        self.cmd[topic] = (time.monotonic(), float(msg.linear.x), float(msg.linear.y),
                           float(msg.angular.z))

    def on_onboard(self, text):
        try:
            st = json.loads(text)
        except (TypeError, ValueError):
            return
        self.row["fsm"] = st.get("fsm")
        self.row["balanced"] = int(bool(st.get("balanced")))

    def on_waist(self, text):
        try:
            st = json.loads(text)
            self.row["waist_target_deg"] = float(st.get("offset_deg") or 0.0)
        except (TypeError, ValueError, AttributeError):
            pass

    def current_cmd(self):
        """Frischester Befehl; walk_cmd hat Vorrang wie im loco_client."""
        now = time.monotonic()
        for topic in ROS_CMD_TOPICS:
            c = self.cmd.get(topic)
            if c is not None and now - c[0] <= 0.5:
                return topic.rsplit("/", 1)[-1], c[1:]
        return "", (0.0, 0.0, 0.0)


def record(args):
    os.makedirs(args.out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = os.path.join(args.out_dir, f"walk_diag_{stamp}_{args.label}")
    rec = Recorder(args)
    node = None
    if not args.no_ros:
        try:
            import rclpy
            rclpy.init()
        except Exception as e:
            print(f"[walk_diag] ROS nicht verfuegbar ({e}) -- nur DDS.")
            args.no_ros = True
    topics = rec.open_dds()
    if not args.no_ros:
        import rclpy
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
        node = rclpy.create_node("walk_diag")
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        for t in ROS_CMD_TOPICS:
            node.create_subscription(Twist, t, lambda m, t=t: rec.on_cmd(t, m), 10)
        node.create_subscription(String, "/g1pilot/onboard_state",
                                 lambda m: rec.on_onboard(m.data), latched)
        node.create_subscription(String, "/g1pilot/waist/state",
                                 lambda m: rec.on_waist(m.data), 10)
        rec.sources += list(ROS_CMD_TOPICS) + ["/g1pilot/onboard_state", "/g1pilot/waist/state"]
    meta = {"label": args.label, "iface": args.iface, "rate_hz": args.rate,
            "max_vx": args.max_vx, "max_vy": args.max_vy, "max_vyaw": args.max_vyaw,
            "sources": rec.sources, "dds_topics": sorted(topics),
            "start": stamp, "note": args.note,
            "env": {k: os.environ.get(k) for k in (
                "G1_WALK_FSM_ID", "G1_WALK_PARK_ARMS", "G1_WAIST_HOLD", "G1_WAIST_KP",
                "G1_WAIST_KD", "G1_MAX_VX", "G1_MAX_VY", "G1_MAX_VYAW", "G1_CONTROLLER")}}
    with open(base + ".json", "w") as fh:
        json.dump(meta, fh, indent=2)
    cols = columns()
    period = 1.0 / max(1.0, args.rate)
    t0 = time.monotonic()
    n = 0
    print(f"[walk_diag] Aufnahme -> {base}.csv  (Strg+C beendet)")
    with open(base + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        try:
            while args.duration <= 0 or time.monotonic() - t0 < args.duration:
                t_tick = time.monotonic()
                if node is not None:
                    rclpy.spin_once(node, timeout_sec=0.0)
                rec.poll_dds()
                src, (vx, vy, wz) = rec.current_cmd()
                rec.row.update({"t": round(t_tick - t0, 4), "cmd_src": src,
                                "cmd_vx": vx, "cmd_vy": vy, "cmd_wz": wz})
                w.writerow(rec.row)
                n += 1
                if n % int(max(1, args.rate * 5)) == 0:
                    fh.flush()
                    r = rec.row
                    print(f"[walk_diag] {r['t']:7.1f} s  FSM {r.get('fsm')}  cmd "
                          f"({vx:+.2f},{vy:+.2f},{wz:+.2f})  roll "
                          f"{math.degrees(r.get('imu_roll', 0.0)):+.1f} deg")
                time.sleep(max(0.0, period - (time.monotonic() - t_tick)))
        except KeyboardInterrupt:
            pass
    if node is not None:
        node.destroy_node()
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    print(f"[walk_diag] {n} Zeilen gespeichert: {base}.csv")
    return base + ".csv"


def main(argv=None):
    ap = argparse.ArgumentParser(description="Geh-Diagnose G1 (nur lesend).")
    sp = ap.add_subparsers(dest="cmd", required=True)
    r = sp.add_parser("record", help="aufzeichnen")
    r.add_argument("iface", help="Netzwerk-Interface zum G1 (ROBOT_INTERFACE)")
    r.add_argument("--label", default="test", help="Kurzname des Versuchs (Dateiname)")
    r.add_argument("--note", default="", help="Freitext (Boden, Armpose, Tempo ...)")
    r.add_argument("--rate", type=float, default=50.0)
    r.add_argument("--duration", type=float, default=0.0, help="Sekunden, 0 = bis Strg+C")
    r.add_argument("--out-dir", default="diag_logs")
    r.add_argument("--discover-s", type=float, default=2.0)
    r.add_argument("--no-ros", action="store_true", help="ohne ROS-Topics (nur DDS)")
    r.add_argument("--no-lowcmd", action="store_true", help="rt/lowcmd nicht mitlesen")
    r.add_argument("--max-vx", type=float, default=float(os.environ.get("G1_MAX_VX", 0.4)))
    r.add_argument("--max-vy", type=float, default=float(os.environ.get("G1_MAX_VY", 0.3)))
    r.add_argument("--max-vyaw", type=float, default=float(os.environ.get("G1_MAX_VYAW", 0.4)))
    a = sp.add_parser("analyze", help="CSV auswerten")
    a.add_argument("csv", nargs="+")
    args = ap.parse_args(argv)
    if args.cmd == "record":
        args.label = "".join(c if c.isalnum() or c in "-_" else "_" for c in args.label)
        record(args)
    else:
        for path in args.csv:
            rows, meta = load_csv(path)
            print("\n".join(analyze(rows, meta)))
            print()


if __name__ == "__main__":
    main()

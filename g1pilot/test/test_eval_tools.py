#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tests der Messwerkzeuge unter tools/eval/ (Bachelorarbeit, V1-V8).

Geprueft wird die Auswertelogik mit synthetischen Daten (ohne Sim, ohne
Roboter): Kopf/CSV/JSON, Statistik, Taktmessung (V4), Schritt-Erkennung und
Vergleich (V7), Not-Halt-/Stillstand-Erkennung (V8), Stufenzuordnung (V6)
und ein kompletter V1-Lauf auf dem echten Quellbaum. Ein kleiner V6-Lauf
braucht pinocchio (sonst uebersprungen).
"""
import json
import math
import os
import sys

import pytest

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
EVAL = os.path.join(PKG, "tools", "eval")
sys.path.insert(0, EVAL)

import eval_common as ec  # noqa: E402


# ── eval_common ─────────────────────────────────────────────────────────
def test_run_writes_header_csv_json(tmp_path):
    run = ec.Run("test.py", "vtest", ec.MODE_SIM, {"a": 1, "b": [1, 2]}, out_root=str(tmp_path))
    run.write_csv("x.csv", ["t", "v"], [[0.0, 1.5], [0.1, None]], "Testdaten")
    run.finish({"wert": 3})
    assert os.path.dirname(run.dir) == str(tmp_path / "vtest")
    meta, cols, rows = ec.read_csv(os.path.join(run.dir, "x.csv"))
    for k in ("git", "betriebsart", "rechner", "parameter"):
        assert k in meta, k
    assert meta["betriebsart"] == ec.MODE_SIM
    assert meta["parameter"] == {"a": 1, "b": [1, 2]}
    assert set(meta["rechner"]) >= {"cpu", "ram", "gpu", "os"}
    assert cols == ["t", "v"]
    assert rows[0] == {"t": "0.0", "v": "1.5"} and rows[1]["v"] == ""
    with open(os.path.join(run.dir, "summary.json"), encoding="utf-8") as f:
        s = json.load(f)
    assert list(s)[0] == "meta" and s["wert"] == 3
    assert "x.csv" in s["dateien"]


def test_run_dirs_do_not_collide(tmp_path):
    a = ec.Run("t", "v", ec.MODE_SIM, {}, out_root=str(tmp_path), stamp=None)
    b = ec.Run("t", "v", ec.MODE_SIM, {}, out_root=str(tmp_path), stamp=None)
    assert a.dir != b.dir


def test_statistics_unsmoothed():
    v = [1, 2, 3, 4, 100]
    d = ec.describe(v)
    assert d["n"] == 5 and d["max"] == 100 and d["median"] == 3      # Ausreisser bleibt
    assert ec.percentile([0, 10], 50) == 5
    assert ec.describe([]) == {"n": 0}
    assert math.isclose(d["mittel"], 22.0)


def test_env_mode(monkeypatch):
    monkeypatch.setenv("G1_SIM_MODE", "true")
    assert ec.env_mode() == ec.MODE_SIM
    monkeypatch.setenv("G1_SIM_MODE", "false")
    assert ec.env_mode() == ec.MODE_REAL


# ── V4 ──────────────────────────────────────────────────────────────────
def test_v4_loop_stats_and_rtf():
    import v4_echtzeit as v4
    times = [i * 0.02 for i in range(101)]
    times[50] += 0.015                       # ein spaetes Paket
    st, dts = v4.loop_stats(times, 50.0, "sim", 1.0)
    assert st["n"] == 101 and abs(st["ist_hz_wand"] - 50.0) < 1e-6
    assert math.isclose(st["dt_max_s"], 0.035, abs_tol=1e-9)
    assert math.isclose(st["abweichung_max_s"], 0.015, abs_tol=1e-9)
    # Zeitbasis sim: Soll in Wandzeit skaliert mit dem Echtzeitfaktor
    st2, _ = v4.loop_stats(times, 50.0, "sim", 0.5)
    assert math.isclose(st2["soll_hz_wand"], 25.0)
    st3, _ = v4.loop_stats(times, 250.0, "wand", 0.5)
    assert st3["soll_hz_wand"] == 250.0
    st4, _ = v4.loop_stats(times, None, None, 1.0)
    assert "abweichung_max_s" not in st4 and st4["zeitbasis"] == "keine"
    # tick in ms: 2 s Wandzeit, 1 s Sim-Zeit -> Echtzeitfaktor 0,5
    ticks = [(i * 0.1, i * 50) for i in range(21)]
    w = v4.rtf_windows(ticks, window=1.0)
    assert w and all(abs(x[1] - 0.5) < 1e-9 for x in w)
    assert all(abs(x[2] - 500.0) < 1e-6 for x in w)


# ── V7 ──────────────────────────────────────────────────────────────────
def test_v7_derive_steps_ignores_old_terminal():
    import v7_gelenkwinkel_aufzeichnen as v7
    ev = [(1.0, "goto", "P1"),
          (1.05, "status", {"state": "cancelled", "source": "pose_store"}),   # alte Fahrt
          (1.1, "status", {"state": "accepted", "source": "pose_store"}),
          (1.5, "status", {"state": "executing", "source": "pose_store"}),
          (3.0, "status", {"state": "reached", "source": "pose_store"}),
          (3.3, "goto", "P2"),
          (3.4, "status", {"state": "rejected", "source": "pose_store", "reason": "x"}),
          (3.5, "status", {"state": "reached", "source": "arm_api"})]
    st = v7.derive_steps(ev)
    assert [s["pose"] for s in st] == ["P1", "P2"]
    assert st[0]["t_executing"] == 1.5 and st[0]["t_ende"] == 3.0 and st[0]["zustand"] == "reached"
    assert st[1]["zustand"] == "rejected" and st[1]["grund"] == "x"


def _fake_recording(root, name, offset_q=0.0, t_shift=0.0, mode=ec.MODE_SIM):
    """Zwei Schritte: Gelenk 0 faehrt 0 -> 1 rad (1 s), dann 1 -> 0."""
    run = ec.Run("synthetisch", name, mode, {}, out_root=str(root))
    ist, soll = [], []
    for i in range(0, 501):
        t = i * 0.01 + t_shift
        tr = i * 0.01
        q0 = min(1.0, tr / 1.0) if tr < 2.5 else max(0.0, 1.0 - (tr - 2.5))
        row = [t, 0, 0.0, 0.0, 0.0] + [q0 + offset_q] + [0.0] * 13 + [0.0] * 28
        ist.append(row)
        soll.append([t, 1.0, q0] + [0.0] * 13 + [60.0] * 14)
    names = ["l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
             "l_wrist_roll", "l_wrist_pitch", "l_wrist_yaw",
             "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow",
             "r_wrist_roll", "r_wrist_pitch", "r_wrist_yaw"]
    run.write_csv("ist.csv", ["t_s", "tick_ms", "waist_yaw", "waist_roll", "waist_pitch"]
                  + [f"q_{n}" for n in names] + [f"dq_{n}" for n in names] + [f"tau_{n}" for n in names],
                  ist)
    run.write_csv("soll.csv", ["t_s", "gewicht"] + [f"q_{n}" for n in names] + [f"kp_{n}" for n in names],
                  soll)
    pose = {"left_arm": [1.0] + [0.0] * 6}
    steps = [{"pose": "A", "t_goto": 0.0 + t_shift, "t_accepted": 0.0 + t_shift,
              "t_executing": 0.0 + t_shift, "t_ende": 1.0 + t_shift, "zustand": "reached", "grund": ""},
             {"pose": "B", "t_goto": 2.5 + t_shift, "t_accepted": 2.5 + t_shift,
              "t_executing": 2.5 + t_shift, "t_ende": 3.5 + t_shift, "zustand": "reached", "grund": ""}]
    run.finish({"schritte_soll": [{"pose": "A"}, {"pose": "B"}],
                "posen": {"A": pose, "B": {"left_arm": [0.0] * 7}}, "schritte": steps})
    return run.dir


def test_v7_compare_identical_and_offset(tmp_path):
    import v7_auswerten as v7a
    a = _fake_recording(tmp_path / "a", "rec")
    b = _fake_recording(tmp_path / "b", "rec", offset_q=0.02, t_shift=0.7)   # anderer Startzeitpunkt
    out = tmp_path / "out"
    assert v7a.main([a, b, "--no-fk", "--no-plot", "--out", str(out)]) == 0
    res = os.path.join(out, "v7_vergleich", os.listdir(out / "v7_vergleich")[0])
    with open(os.path.join(res, "summary.json"), encoding="utf-8") as f:
        s = json.load(f)
    g = s["gelenke"]["l_shoulder_pitch"]
    # Zeitversatz wird je Schritt abgeglichen -> nur der konstante Versatz bleibt
    assert abs(g["abw_mittel_rad"] - 0.02) < 1e-6 and abs(g["abw_max_rad"] - 0.02) < 1e-6
    assert s["gelenke"]["l_elbow"]["abw_max_rad"] == 0.0
    assert s["hinweise"] == []
    # Folgefehler Soll-Ist von B = 0,02 rad
    assert abs(g["folgefehler_b_max_rad"] - 0.02) < 1e-6
    # Abgleich "start": ebenfalls Versatz 0,02 (Start gleichgesetzt)
    out2 = tmp_path / "out2"
    assert v7a.main([a, b, "--no-fk", "--no-plot", "--abgleich", "start", "--out", str(out2)]) == 0


def test_v7_detects_different_sequences(tmp_path):
    import v7_auswerten as v7a
    a = _fake_recording(tmp_path / "a", "rec")
    b = _fake_recording(tmp_path / "b", "rec")
    with open(os.path.join(b, "summary.json"), encoding="utf-8") as f:
        s = json.load(f)
    s["posen"]["A"]["left_arm"][0] = 0.9
    with open(os.path.join(b, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(s, f)
    A, B = v7a.Aufzeichnung(a), v7a.Aufzeichnung(b)
    msgs = v7a.check_identical(A, B)
    assert any("unterscheidet sich" in m for m in msgs)


# ── V8 ──────────────────────────────────────────────────────────────────
def _row(t, q0, w=1.0, kp=60.0, kpw=150.0):
    return (t, w, [q0] + [0.0] * 13, [kp] * 14, [kpw] * 3)


def test_v8_slack_and_standstill_detection():
    import v8_nothalt as v8
    rows = [_row(i * 0.004, i * 0.001) for i in range(50)]
    rows.append(_row(0.2, 0.05, kp=0.0))
    rows.append(_row(0.204, 0.05, kp=0.0, kpw=0.0))
    assert v8.first_slack(rows) == 0.2
    assert v8.first_waist_passive(rows) == 0.204
    assert v8.first_slack([_row(0.0, 0.0, w=0.0, kp=0.0)]) is None   # Gewicht 0 = kein Slack
    # Bewegung bis 0.4 s, danach Stillstand
    mv = [_row(i * 0.004, min(i, 100) * 0.001) for i in range(300)]
    t = v8.standstill(mv)
    assert t is not None and abs(t - 0.4) < 0.005
    assert v8.standstill(mv[:100]) is None
    assert v8.moved_since(mv[:10]) and not v8.moved_since(mv[150:])


# ── V6 ──────────────────────────────────────────────────────────────────
def test_v6_stage_mapping():
    import v6_armplaner as v6
    ev = [("validate", 0, 0.01, 0.01),                 # Direktpruefung
          ("optimize", 0, 0.30, 0.3), ("validate", 0, 0.02, 0.02),    # Optimierer
          ("optimize", 0, 0.20, 0.2), ("validate", 0, 0.02, 0.02),    # reduzierter Abstand
          ("rrt_seed", 0, 0.10, 0.1), ("validate", 0, 0.01, 0.01),    # RRT roh
          ("optimize", 0, 0.05, 0.05), ("validate", 0, 0.01, 0.01),   # RRT optimiert
          ("time_param", 0, 0.03, 0.03)]
    tries = [("direct", 0.05), ("optimized", 0.05), ("optimized", 0.015), ("rrt", 0.05)]
    st = v6.stage_times(ev, tries, 0.05, 1.0)
    assert math.isclose(st["direkt"], 0.01)
    assert math.isclose(st["optimierer"], 0.32)
    assert math.isclose(st["reduzierter_abstand"], 0.22)
    assert math.isclose(st["rrt"], 0.17)
    assert math.isclose(st["zeitparametrierung"], 0.03)
    assert math.isclose(sum(st.values()), 1.0)


def test_v6_small_run(tmp_path):
    pytest.importorskip("pinocchio")
    import v6_armplaner as v6
    assert v6.main(["--only", "unerreichbar", "--reps", "1", "--random", "0", "--no-plot",
                    "--out", str(tmp_path)]) == 0
    d = os.path.join(tmp_path, "v6_armplaner", os.listdir(tmp_path / "v6_armplaner")[0])
    _m, _c, rows = ec.read_csv(os.path.join(d, "laeufe.csv"))
    assert len(rows) == 4                    # U1-U4, je 1 Wiederholung
    assert all(r["erfolg"] in ("0", "False", "false") for r in rows)   # unerreichbar bleibt unerreichbar
    assert all(r["grund"] for r in rows)


# ── V1 ──────────────────────────────────────────────────────────────────
def test_v1_full_run_everything_catalogued(tmp_path):
    pytest.importorskip("yaml")
    import v1_code_vergleich as v1
    assert v1.main(["--out", str(tmp_path)]) == 0
    d = os.path.join(tmp_path, "v1_code_vergleich", os.listdir(tmp_path / "v1_code_vergleich")[0])
    with open(os.path.join(d, "summary.json"), encoding="utf-8") as f:
        s = json.load(f)
    assert s["nicht_katalogisiert"] == [], s["nicht_katalogisiert"]
    assert s["abfragen_ohne_tests"] > 0
    for f in ("abfragen.csv", "launch.csv", "images.csv", "tabelle.md"):
        assert os.path.exists(os.path.join(d, f)), f
    _m, _c, rows = ec.read_csv(os.path.join(d, "abfragen.csv"))
    files = {r["datei"] for r in rows}
    assert "g1pilot/utils/common.py" in files
    assert all(r["wirkung"] for r in rows), "jede Fundstelle braucht eine Art des Unterschieds"

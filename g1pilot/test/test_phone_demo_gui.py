#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Handy-Bedienung in der Demo-GUI (offscreen, ohne Bluetooth): Befehle laufen
ueber dieselben Methoden wie die Knoepfe; Verbindungsverlust bricht vom
Handy gestartete Bewegungen ab; Real-Startleiste nur mit Freigabe."""
import json
import os
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy                                       # noqa: E402
from PyQt6.QtWidgets import QApplication          # noqa: E402

from g1pilot.teleoperation.phone import gateway as G     # noqa: E402
from g1pilot.teleoperation.phone import protocol as P    # noqa: E402
from phone_client import FakePhone                       # noqa: E402


class Clock:
    def __init__(self):
        self.t = 5000.0

    def __call__(self):
        return self.t


def _make(monkeypatch, tmp_path, sim, phone=True):
    monkeypatch.setenv("G1_SIM_MODE", "true" if sim else "false")
    monkeypatch.setenv("G1_PHONE", "1" if phone else "0")
    monkeypatch.setenv("G1_PHONE_STORE", str(tmp_path / "devices.json"))
    monkeypatch.setenv("G1_POSE_STORE", str(tmp_path / "poses.json"))
    from g1pilot.teleoperation.phone import ble_server
    monkeypatch.setattr(ble_server.BleServer, "start", lambda self: True)
    rclpy.init()
    app = QApplication.instance() or QApplication([])
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    sent, strs = [], []
    ob, os_ = node.publish_bool, node.publish_str
    node.publish_bool = lambda pub, v: (sent.append((pub.topic_name, bool(v))), ob(pub, v))
    node.publish_str = lambda pub, v: (strs.append((pub.topic_name, v)), os_(pub, v))
    loco = []
    node.pub_loco_request.publish = lambda msg: loco.append(msg.data)
    return dg, g, node, app, sent, strs, loco


def _close(g, node):
    if g.phone is not None:
        g.phone.stop()
    g.close()
    node.tf_listener.executor.shutdown()
    node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
    node.tf_node.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def real(monkeypatch, tmp_path):
    dg, g, node, app, sent, strs, loco = _make(monkeypatch, tmp_path, sim=False)
    try:
        yield dg, g, node, app, sent, strs, loco
    finally:
        _close(g, node)


@pytest.fixture
def sim(monkeypatch, tmp_path):
    dg, g, node, app, sent, strs, loco = _make(monkeypatch, tmp_path, sim=True)
    try:
        yield dg, g, node, app, sent, strs, loco
    finally:
        _close(g, node)


def _phone(g, role=G.ROLE_OPERATE, take=True):
    link = g.phone
    clk = Clock()
    link.gw._clock = clk
    ph = FakePhone(link.gw)
    assert ph.pair(link.gw.start_pairing(role))
    assert ph.login() == "ok"
    link.publish_status()
    if take:
        ph.cmd("take")
        assert link.gw.has_control()
    return ph, clk


def _run(g, ph, action, arg=None):
    g.phone.publish_status()
    cid = ph.cmd(action, arg)
    g.phone.tick()
    g.phone.publish_status()
    return ph.result(cid)


def test_phone_off_by_default(monkeypatch, tmp_path):
    dg, g, node, app, *_ = _make(monkeypatch, tmp_path, sim=True, phone=False)
    try:
        assert g.phone is None and g.btn_phone.isHidden()
    finally:
        _close(g, node)


def test_button_shows_connection(real):
    dg, g, node, app, *_ = real
    assert not g.btn_phone.isHidden() and "Handy" in g.btn_phone.text()
    ph, clk = _phone(g, take=False)
    g.phone.tick()
    assert "Testhandy" in g.btn_phone.text()
    ph.cmd("take")
    g.phone.tick()
    assert "bedient" in g.btn_phone.text()


def test_real_estop_and_ack_from_phone(real):
    dg, g, node, app, sent, strs, loco = real
    ph, clk = _phone(g)
    assert ph.last["st"]["ack"] is True and ph.last["st"]["started"] is False
    r = _run(g, ph, "ack")
    assert r["ok"] and g.started
    assert ("/g1pilot/start", True) in sent
    sent.clear()
    r = _run(g, ph, "estop")
    assert r["ok"] and g._estop_shown and not g.started
    assert ("/g1pilot/emergency_stop", True) in sent
    assert ph.last["st"]["estop"] is True
    assert "Handy" in g.status.text()
    r = _run(g, ph, "ack")
    assert r["ok"] and "quittiert" in r["m"] and g.started


def test_view_role_estop_but_no_ack(real):
    dg, g, node, app, sent, strs, loco = real
    ph, clk = _phone(g, role=G.ROLE_VIEW, take=False)
    r = _run(g, ph, "ack")
    assert r["ok"] is False and not g.started
    r = _run(g, ph, "estop")
    assert r is None or r["ok"]
    assert ("/g1pilot/emergency_stop", True) in sent


def test_real_loco_step_needs_fresh_onboard_and_skips_double_tap(real):
    dg, g, node, app, sent, strs, loco = real
    ph, clk = _phone(g)
    r = _run(g, ph, "loco", "damp")                      # keine Daten vom G1 -> gesperrt
    assert r["ok"] is False and loco == []
    node.onboard = {"name": "Preparation", "fsm": 4, "estop": False, "walk_fsm": 501}
    node.onboard_t = time.monotonic()
    r = _run(g, ph, "loco", "damp")                      # Rueckfrage war am Handy (Schieber)
    assert r["ok"] and loco == ["damp"]
    r = _run(g, ph, "loco", "walk")                      # aus FSM 4 ohne Bodenkontakt-Info frei
    assert loco[-1] == "walk"
    r = _run(g, ph, "loco", "jump")
    assert r["ok"] is False


def test_sim_sequence_cancelled_when_heartbeat_stops(sim):
    dg, g, node, app, sent, strs, loco = sim
    ph, clk = _phone(g)
    g.mode, g.pending = dg.MANIP, None
    from g1pilot.manipulation.pose_store import PoseStore
    store = PoseStore()
    for n in ("W_01_a", "W_02_b"):
        store.save(n, category="Ablauf Winken", left_arm=[0.0] * 7)
    r = _run(g, ph, "seq", "Winken")
    assert r["ok"] and ("/g1pilot/pose_store/goto", "W_01_a") in strs
    assert ph.last["st"]["busy"] is True
    sent.clear()
    clk.t += G.CONTROL_TIMEOUT_S + 0.1                   # Handy verstummt
    g.phone.tick()
    assert ("/g1pilot/pose_store/cancel", True) in sent
    assert g.current_pose is None and not g.queue
    assert "abgebrochen" in g.status.text()


def test_sim_sequence_runs_on_when_pc_started_it(sim):
    dg, g, node, app, sent, strs, loco = sim
    ph, clk = _phone(g)
    g.mode, g.pending = dg.MANIP, None
    g.run_poses(["A", "B"], "PC")                        # am PC gestartet
    sent.clear()
    clk.t += G.CONTROL_TIMEOUT_S + 0.1
    g.phone.tick()
    assert ("/g1pilot/pose_store/cancel", True) not in sent
    assert g.current_pose == "A"


def test_sim_hands_only_in_grasp_and_ack_after_estop(sim):
    dg, g, node, app, sent, strs, loco = sim
    ph, clk = _phone(g)
    g.mode = None
    r = _run(g, ph, "hand", "open")
    assert r["ok"] is False
    g.mode, g.pending = dg.MANIP, None
    r = _run(g, ph, "hand", "left_close")
    assert r["ok"] and ("/g1pilot/hand_action/left", "close") in strs
    r = _run(g, ph, "estop")
    assert g._estop_unacked
    sent.clear()
    r = _run(g, ph, "ack")
    assert r["ok"] and ("/g1pilot/start", True) in sent and not g._estop_unacked
    r = _run(g, ph, "ack")
    assert r["ok"] is False


def test_mode_needs_control(sim):
    dg, g, node, app, sent, strs, loco = sim
    ph, clk = _phone(g, take=False)
    r = _run(g, ph, "mode", "walk")
    assert r["ok"] is False and g.pending is None
    ph.cmd("take")
    r = _run(g, ph, "mode", "walk")
    assert r["ok"] and g.pending == dg.WALK
    assert ("/g1pilot/start_walking", True) in sent


def test_revoke_from_pc_dialog(sim):
    dg, g, node, app, sent, strs, loco = sim
    ph, clk = _phone(g)
    g.phone.show_dialog()
    d = g.phone.dialog
    assert d.btn_revoke.isEnabled()
    d.btn_revoke.click()
    assert not g.phone.gw.has_control()
    d.btn_pair_op.click()
    code = d.lbl_code.text()
    assert len(P.normalize_code(code)) == P.CODE_LEN
    assert d.list.count() == 1
    d.close()


def test_status_fits_tx_pages(real):
    dg, g, node, app, *_ = real
    ph, clk = _phone(g)
    g.phone.publish_status()
    data = json.dumps(ph.tx).encode()
    assert len(data) < P.TX_MAX


def _poses(tmp_path):
    from g1pilot.manipulation.pose_store import PoseStore
    st = PoseStore(str(tmp_path / "poses.json"))
    st.save("Regal_oben", category="Demo", left_arm=[0.0] * 7, right_arm=[0.1] * 7)
    st.save("A_1", category="Ablauf Test", right_arm=[0.2] * 7)
    return st


def test_pose_goto_and_catalog(sim, tmp_path):
    dg, g, node, app, sent, strs, loco = sim
    _poses(tmp_path)
    ph, clk = _phone(g)
    g.phone.refresh_catalog()
    g.phone.publish_status()
    ph.signed({"t": "cat"})
    assert ph.last["cat"]["g"]["Demo"] == ["Regal_oben"]
    r = _run(g, ph, "pose", "Regal_oben")              # nicht im Modus GREIFEN
    assert r["ok"] is False and "GREIFEN" in r["m"]
    g.mode, g.pending = dg.MANIP, None
    r = _run(g, ph, "pose", "Gibt_es_nicht")
    assert r["ok"] is False
    r = _run(g, ph, "pose", "Regal_oben")
    assert r["ok"] and ("/g1pilot/pose_store/goto", "Regal_oben") in strs
    sent.clear()
    clk.t += G.CONTROL_TIMEOUT_S + 0.1                   # Handy verstummt -> Abbruch
    g.phone.tick()
    assert ("/g1pilot/pose_store/cancel", True) in sent


def test_pose_save_from_phone(sim, tmp_path):
    dg, g, node, app, sent, strs, loco = sim
    ph, clk = _phone(g)
    g.mode, g.pending = dg.MANIP, None
    r = _run(g, ph, "pose_save", "1010|Demo|Tisch links")      # alte Seite (3 Teile)
    assert r["ok"], r
    topic, payload = [x for x in strs if x[0].endswith("save")][-1]
    data = json.loads(payload)
    assert data.pop("id").startswith("demo_gui_save_")       # Quittung wie am PC
    assert data == {"name": "Tisch links", "category": "Demo",
                    "components": ["left_arm", "left_hand"]}
    r = _run(g, ph, "pose_save", "0000|Demo|Nix")
    assert r["ok"] is False
    r = _run(g, ph, "pose_save", "1111|Demo|  ")
    assert r["ok"] is False
    g.phone.gw.revoke_control()
    r = _run(g, ph, "pose_save", "1111|Demo|Ohne Hoheit")
    assert r["ok"] is False


def test_status_pad_and_temps(real):
    dg, g, node, app, sent, strs, loco = real
    ph, clk = _phone(g, take=False)
    now = time.monotonic()
    node.ps4 = {"connected": True, "selection": "right", "mode": "turn", "speed": "Normal"}
    node.ps4_t = now
    node.ps4dev = {"kind": "ps4", "name": "Wireless Controller"}
    from g1pilot.utils import g1_temperature as g1t
    node.temps = g1t.parse_payload(g1t.build_payload(
        [type("M", (), {"temperature": [40 + i % 3, 41], "vol": 30.0})() for i in range(g1t.N_MOTORS)], 33))
    node.temps_t = now
    g.phone.publish_status()
    pad, tmp = ph.last["st"]["pad"], ph.last["st"]["tmp"]
    assert pad["k"] == "arm" and pad["jm"] == "Drehen" and pad["sp"] == "Normal" and pad["sel"] == "RECHTS"
    assert tmp["max"][1] == 42 and len(tmp["g"]) == len(g1t.GROUPS)
    node.ps4walk = {"active": True, "speed": "Schnell"}
    node.ps4["arms_locked"] = True
    g.phone.publish_status()
    assert ph.last["st"]["pad"]["k"] == "walking" and ph.last["st"]["pad"]["sp"] == "Schnell"


def test_sim_sequence_step_mode_next_and_mode_from_phone(sim):
    dg, g, node, app, sent, strs, loco = sim
    from g1pilot.manipulation.pose_store import PoseStore
    from g1pilot.teleoperation import sequence_runner as sr
    store = PoseStore()
    for n in ("A", "B"):
        store.save(n, category="Demo", left_arm=[0.0] * 7)
    store.save_sequence("Kiste", ["A", "B"])
    g.phone.refresh_catalog()
    assert g.phone.gw._catalog["s"] == ["Kiste"]
    ph, clk = _phone(g)
    g.mode, g.pending = dg.MANIP, None
    r = _run(g, ph, "seq_mode", "step")
    assert r["ok"] and g.seq.mode == sr.MODE_STEP and ph.last["st"]["sm"] == "step"
    assert _run(g, ph, "seq_next")["ok"] is False             # nichts wartet
    assert _run(g, ph, "seq", "Kiste")["ok"]
    for st in ("accepted", "reached"):
        g._on_arm_status({"state": st})
    g.phone.publish_status()
    assert ph.last["st"]["seq"] == {"s": "waiting", "n": "Kiste", "i": 1, "t": 2}
    assert _run(g, ph, "seq_next")["ok"]
    assert [v for t, v in strs if t == "/g1pilot/pose_store/goto"] == ["A", "B"]
    assert _run(g, ph, "seq_mode", "quatsch")["ok"] is False


def test_pose_save_appends_to_sequence(sim, tmp_path):
    dg, g, node, app, sent, strs, loco = sim
    st = _poses(tmp_path)
    st.create_sequence("Leer")
    ph, clk = _phone(g)
    g.mode, g.pending = dg.MANIP, None
    r = _run(g, ph, "pose_save", "1111|Demo|Gibt es nicht|X")
    assert r["ok"] is False and "nicht gefunden" in r["m"]
    r = _run(g, ph, "pose_save", "1100|Demo|Leer|Greifen_1")
    assert r["ok"] and "Leer" in r["m"]
    data = json.loads([x for x in strs if x[0].endswith("save")][-1][1])
    assert "sequence" not in data                          # Anhaengen erst nach Quittung
    st.save("Greifen_1", category="Demo", left_arm=[0.0] * 7, right_arm=[0.0] * 7)   # = arm_controller
    g._on_save_status({"id": data["id"], "state": "saved"})
    assert [s_["pose"] for s_ in st.get_sequence("Leer")] == ["Greifen_1"]
    assert "angehängt" in g.status.text()


def test_catalog_lists_moves_and_empty_sequences(sim, tmp_path):
    dg, g, node, app, sent, strs, loco = sim
    st = _poses(tmp_path)
    st.create_sequence("Leer")
    st.save_move("Box anheben", dz=0.05)
    ph, clk = _phone(g)
    g.phone.refresh_catalog()
    g.phone.publish_status()
    ph.signed({"t": "cat"})
    cat = ph.last["cat"]
    assert cat["m"] == ["Box anheben"] and "Leer" in cat["e"] and "Leer" not in cat["s"]
    r = _run(g, ph, "move", "Box anheben")                  # nicht GREIFEN
    assert r["ok"] is False
    g.mode, g.pending = dg.MANIP, None
    r = _run(g, ph, "move", "Gibt es nicht")
    assert r["ok"] is False

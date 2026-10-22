#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Demo-GUI: Ablaeufe am Stueck / Schritt fuer Schritt (offscreen, ohne Roboter).

Die Rueckmeldungen des arm_controller (/g1pilot/arm_command/status) werden
direkt eingespielt. Geprueft: Schritt-Modus wartet auf »Weiter« (Knopf und
Controller-Topic), NOT-HALT / Stopp / Moduswechsel beenden auch einen
wartenden Ablauf ohne spaeteres Weiterfahren, fehlende Posen starten nicht,
Fenster »Posen & Abläufe« (zusammenstellen, umsortieren, Teach-in),
relative Bewegungen (»Box anheben«: aktuelle Hand + Verschiebung als
arm_command, Ziel unbekannt -> Abbruch, im Editor anlegen/einfuegen)."""
import json
import os

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy                                       # noqa: E402
from std_msgs.msg import Bool, String             # noqa: E402
from PyQt6.QtWidgets import QApplication, QDialog  # noqa: E402

from g1pilot.manipulation.pose_store import PoseStore     # noqa: E402
from g1pilot.teleoperation import sequence_runner as sr   # noqa: E402

ARM = [0.1] * 7


def _make(monkeypatch, tmp_path, sim):
    monkeypatch.setenv("G1_SIM_MODE", "true" if sim else "false")
    monkeypatch.setenv("G1_PHONE", "0")
    monkeypatch.setenv("G1_POSE_STORE", str(tmp_path / "poses.json"))
    monkeypatch.delenv("G1_SEQUENCE_MODE", raising=False)
    rclpy.init()
    app = QApplication.instance() or QApplication([])
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    sent, strs, states = [], [], []
    ob, os_ = node.publish_bool, node.publish_str
    node.publish_bool = lambda pub, v: (sent.append((pub.topic_name, bool(v))), ob(pub, v))
    node.publish_str = lambda pub, v: (strs.append((pub.topic_name, v)), os_(pub, v))
    node.pub_seq_state.publish = lambda msg: states.append(json.loads(msg.data))
    return dg, g, node, app, sent, strs, states


@pytest.fixture(params=["sim", "real"])
def gui(request, monkeypatch, tmp_path):
    dg, g, node, app, sent, strs, states = _make(monkeypatch, tmp_path, request.param == "sim")
    store = PoseStore()
    for n in ("Sichere_Pose", "Hand_vor", "Greifen"):
        store.save(n, category="Demo", left_arm=ARM)
    store.save_sequence("Kiste", ["Sichere_Pose", "Hand_vor", "Greifen", "Sichere_Pose"])
    g.mode, g.pending = dg.MANIP, None
    try:
        yield dg, g, node, sent, strs, states, store
    finally:
        if g.seq_editor is not None:
            g.seq_editor.close()
        g.close()
        node.tf_listener.executor.shutdown()
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def gotos(strs):
    return [v for t, v in strs if t == "/g1pilot/pose_store/goto"]


def reach(g):
    for st in ("accepted", "executing", "reached"):
        g._on_arm_status({"state": st, "source": "pose_store"})
    g.seq._t = 0.0                          # Abstand MIN_GAP_S sofort abgelaufen
    g.seq.tick()


def test_default_mode_real_step_sim_auto(gui):
    dg, g, *_ = gui
    expected = sr.MODE_AUTO if g.sim_mode else sr.MODE_STEP
    assert g.seq.mode == expected
    assert g.manip_panel.seq_mode_btns[expected].isChecked()
    assert not g.manip_panel.btn_next.isEnabled()


def test_auto_runs_whole_sequence(gui):
    dg, g, node, sent, strs, states, store = gui
    g.set_sequence_mode(sr.MODE_AUTO)
    assert g.run_sequence("Kiste")
    for _ in range(4):
        reach(g)
    assert gotos(strs) == ["Sichere_Pose", "Hand_vor", "Greifen", "Sichere_Pose"]
    assert not g.seq.active and "fertig" in g.status.text()
    assert states[-1]["state"] == "idle"


def test_step_mode_gui_button_and_controller_topic(gui):
    dg, g, node, sent, strs, states, store = gui
    g.set_sequence_mode(sr.MODE_STEP)
    g.run_sequence("Kiste")
    reach(g)
    mp = g.manip_panel
    assert gotos(strs) == ["Sichere_Pose"] and g.seq.state == sr.WAITING
    assert mp.btn_next.isEnabled() and "2/4" in mp.btn_next.text()
    assert states[-1]["state"] == "waiting" and states[-1]["name"] == "Kiste"
    mp.btn_next.click()                                      # GUI
    assert gotos(strs)[-1] == "Hand_vor" and not mp.btn_next.isEnabled()
    reach(g)
    node.on_seq_next()                                       # Controller L3 (/g1pilot/sequence/next)
    assert gotos(strs)[-1] == "Greifen"
    node.on_seq_next()                                       # faehrt noch -> ignoriert
    assert len(gotos(strs)) == 3


def test_seq_next_topic_wired(gui):
    dg, g, node, sent, strs, states, store = gui
    g.set_sequence_mode(sr.MODE_STEP)
    g.run_sequence("Kiste")
    reach(g)
    pub = node.create_publisher(Bool, sr.NEXT_TOPIC, 10)
    import time
    t0 = time.monotonic()
    while len(gotos(strs)) < 2 and time.monotonic() - t0 < 3.0:
        pub.publish(Bool(data=True))
        rclpy.spin_once(node, timeout_sec=0.05)
    assert gotos(strs)[-1] == "Hand_vor"


@pytest.mark.parametrize("how", ["estop_gui", "estop_extern", "stop", "controller_cross",
                                 "mode_walk"])
def test_waiting_sequence_ends_and_never_resumes(gui, how):
    dg, g, node, sent, strs, states, store = gui
    g.set_sequence_mode(sr.MODE_STEP)
    g.run_sequence("Kiste")
    reach(g)
    assert g.seq.state == sr.WAITING
    if how == "estop_gui":
        g.emergency_stop()
    elif how == "estop_extern":
        g._external_estop()                                  # PS am Controller, Handy, Streamdeck
    elif how == "stop":
        g._cancel()
    elif how == "controller_cross":
        node.on_pose_cancel()                                # /g1pilot/pose_store/cancel
    else:
        g.started = True
        g.request_mode(dg.WALK)
    assert not g.seq.active and states[-1]["state"] == "idle"
    g.mode, g.pending = dg.MANIP, None                       # selbst nach Quittieren/Rueckkehr
    node.on_seq_next()
    g.manip_panel.btn_next.click()
    reach(g)
    assert gotos(strs) == ["Sichere_Pose"]


def test_estop_during_motion_aborts(gui):
    dg, g, node, sent, strs, states, store = gui
    g.set_sequence_mode(sr.MODE_AUTO)
    g.run_sequence("Kiste")
    g._on_arm_status({"state": "accepted"})
    g.emergency_stop()
    g._on_arm_status({"state": "cancelled", "reason": "E-Stop"})
    reach(g)
    assert gotos(strs) == ["Sichere_Pose"] and not g.seq.active


def test_jog_cancels_only_during_motion(gui):
    dg, g, node, sent, strs, states, store = gui
    g.set_sequence_mode(sr.MODE_STEP)
    g.run_sequence("Kiste")
    reach(g)
    g.manip_panel.show_view("arms")
    hand = ([0.3, 0.0, 0.1], (0.0, 0.0, 0.0, 1.0))
    node.jog_pose = lambda side, new: (None, hand)
    node.jog_end = lambda side: None
    node.publish_hand_goal = lambda *a: None
    g.manip_panel.jog.twist = lambda: ((0.05, 0.0, 0.0), (0.0, 0.0, 0.0))
    g._jog_tick()                                            # Warten: nachkorrigieren erlaubt
    assert g.seq.state == sr.WAITING
    g._jog_stop()
    g.manip_panel.jog.twist = lambda: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    g._jog_tick()
    g.sequence_next()
    g.manip_panel.jog.twist = lambda: ((0.05, 0.0, 0.0), (0.0, 0.0, 0.0))
    g._jog_tick()                                            # Fahrt laeuft: Jog beendet Ablauf
    assert not g.seq.active and states[-1]["event"] == "aborted"


def commands(strs):
    return [json.loads(v) for t, v in strs if t == "/g1pilot/arm_command"]


HANDS = {"left": ([0.30, 0.20, 0.10], (0.0, 0.0, 0.0, 1.0)),
         "right": ([0.30, -0.20, 0.12], (0.0, 0.7071068, 0.0, 0.7071068))}


def test_relative_move_step_from_current_hands(gui):
    dg, g, node, sent, strs, states, store = gui
    store.save_move("Box anheben", dz=0.05)
    store.save_sequence("Heben", ["Greifen", {"move": "Box anheben"}, "Sichere_Pose"])
    node.hand_reference = lambda side: HANDS[side]
    node.torso_yaw = lambda: 0.0
    g.set_sequence_mode(sr.MODE_AUTO)
    assert g.run_sequence("Heben")
    reach(g)
    cmd = commands(strs)[-1]
    assert cmd["type"] == "pose" and cmd["id"].startswith("demo_gui_move_")
    assert cmd["left"]["position"] == pytest.approx([0.30, 0.20, 0.15])
    assert cmd["right"]["position"] == pytest.approx([0.30, -0.20, 0.17])
    assert cmd["right"]["orientation"] == pytest.approx([0.0, 0.7071068, 0.0, 0.7071068])
    assert cmd["left"]["frame"] == dg.JOG_FRAME
    assert states[-1]["pose"] == "Box anheben"
    reach(g)                                                 # gleiche Statusfolge wie goto
    assert gotos(strs) == ["Greifen", "Sichere_Pose"]
    # Format passt zum arm_controller-Parser.
    from g1pilot.manipulation.arm_command import parse_command
    assert parse_command(cmd)["sides"] == ["left", "right"]


def test_relative_move_rotated_with_waist_and_one_side():
    from g1pilot.teleoperation import demo_gui as dg
    import math
    cmd, err = dg.relative_move_command(
        {"dx": 0.10, "dy": 0.0, "dz": 0.0, "sides": ["right"]},
        lambda side: HANDS[side], yaw=math.pi / 2)
    assert err is None and "left" not in cmd
    assert cmd["right"]["position"] == pytest.approx([0.30, -0.10, 0.12])
    cmd, err = dg.relative_move_command(
        {"dx": 0.0, "dy": 0.0, "dz": 0.05, "sides": ["left"]}, lambda side: None)
    assert cmd is None and "links" in err


def test_relative_move_unknown_hand_aborts_at_once(gui):
    dg, g, node, sent, strs, states, store = gui
    store.save_move("Box anheben", dz=0.05)
    store.save_sequence("Heben", [{"move": "Box anheben"}, "Greifen"])
    node.hand_reference = lambda side: None
    assert not g.run_sequence("Heben")
    assert commands(strs) == [] and gotos(strs) == []
    assert "unbekannt" in g.status.text() and states[-1]["event"] == "aborted"


def test_missing_move_refuses_start(gui):
    dg, g, node, sent, strs, states, store = gui
    store.save_sequence("Heben", ["Greifen", {"move": "Gibts nicht"}])
    assert not g.run_sequence("Heben") and gotos(strs) == []
    assert "Gibts nicht" in g.status.text()


def test_editor_relative_moves(gui, monkeypatch):
    dg, g, node, sent, strs, states, store = gui
    from g1pilot.teleoperation import sequence_editor as se
    g.open_sequence_editor()
    ed = g.seq_editor
    # »Neue relative Bewegung …« -> Dialog ausfuellen
    def fake_exec(dlg):
        dlg.ed_name.setText("Box anheben")
        dlg.spins["dz"].setValue(5.0)
        dlg.cb_left.setChecked(True)
        dlg.cb_right.setChecked(True)
        return QDialog.DialogCode.Accepted
    monkeypatch.setattr(se.MoveDialog, "exec", fake_exec)
    ed.btn_new_move.click()
    assert store.get_move("Box anheben")["dz"] == pytest.approx(0.05)
    assert ed._selected_item() == {"move": "Box anheben"}
    assert ed.btn_edit_move.isEnabled()
    top = ed.pose_tree.topLevelItem(0)
    assert top.text(0) == se.MOVES_GROUP and "hoch 5 cm · beide" in top.child(0).text(0)
    # als Schritt in einen Ablauf
    ed.reload(select="Kiste")
    ed.step_list.setCurrentRow(0)
    ed._select_item({"move": "Box anheben"})
    ed.btn_add.click()
    assert store.get_sequence("Kiste")[1] == {"move": "Box anheben", "wait_s": 0.0}
    assert "↕ Box anheben" in ed.step_list.item(1).text()
    # Umbenennen ueber »Bearbeiten …« zieht den Schritt mit
    def rename(dlg):
        assert dlg.spins["dz"].value() == pytest.approx(5.0)
        dlg.ed_name.setText("Anheben")
        return QDialog.DialogCode.Accepted
    monkeypatch.setattr(se.MoveDialog, "exec", rename)
    ed.btn_edit_move.click()
    assert store.get_sequence("Kiste")[1]["move"] == "Anheben"
    assert "↕ Anheben" in ed.step_list.item(1).text()
    # Ungueltig (0 cm) -> Dialog meldet und bleibt offen
    warned = []
    monkeypatch.setattr(se.QMessageBox, "warning", lambda *a: warned.append(a[2]))
    dlg = se.MoveDialog(ed, "Nichts", {"dx": 0, "dy": 0, "dz": 0, "sides": ["left"]})
    dlg._accept()
    assert warned and dlg.result()[0] == "Nichts"


def test_missing_pose_refuses_start(gui):
    dg, g, node, sent, strs, states, store = gui
    store.delete("Greifen")
    assert not g.run_sequence("Kiste")
    assert gotos(strs) == [] and "Greifen" in g.status.text()


def test_only_in_grasp(gui):
    dg, g, node, sent, strs, states, store = gui
    g.mode = dg.WALK
    assert not g.run_sequence("Kiste") and gotos(strs) == []


def test_buttons_from_store(gui):
    dg, g, node, sent, strs, states, store = gui
    g._reload_sequence_buttons()
    labels = [g.manip_panel.seq_row.itemAt(i).widget().text()
              for i in range(g.manip_panel.seq_row.count())]
    assert "Kiste" in labels


def test_editor_compose_reorder_wait(gui):
    dg, g, node, sent, strs, states, store = gui
    g.open_sequence_editor()
    ed = g.seq_editor
    store.create_sequence("Neu")
    ed.reload(select="Neu")
    assert ed.current_sequence() == "Neu"
    for name in ("Greifen", "Sichere_Pose"):
        ed.add_step(name)
    ed.step_list.setCurrentRow(1)
    ed.btn_up.click()
    assert [st["pose"] for st in store.get_sequence("Neu")] == ["Sichere_Pose", "Greifen"]
    ed.step_list.setCurrentRow(1)
    ed.spin_wait.setValue(2.5)
    assert store.get_sequence("Neu")[1] == {"pose": "Greifen", "wait_s": 2.5}
    ed.step_list.setCurrentRow(0)
    ed.btn_remove.click()
    assert store.get_sequence("Neu") == [{"pose": "Greifen", "wait_s": 2.5}]
    # Knoepfe im Hauptfenster folgen sofort.
    labels = [g.manip_panel.seq_row.itemAt(i).widget().text()
              for i in range(g.manip_panel.seq_row.count())]
    assert "Neu" in labels
    ed.btn_run.click()
    assert gotos(strs) == ["Greifen"]


def test_teach_in_appends_after_save_ack(gui, monkeypatch):
    dg, g, node, sent, strs, states, store = gui
    from g1pilot.teleoperation import ui_interface as ui
    seen = {}

    class FakeDialog:
        def __init__(self, parent, store=None, categories=None, sequences=None, sequence=None):
            seen.update(sequences=sequences, sequence=sequence)

        def exec(self):
            return QDialog.DialogCode.Accepted

        def result_data(self):
            return {"name": "Neu_greifen", "category": "Demo", "components": ["left_arm"],
                    "sequence": seen["sequence"]}

    monkeypatch.setattr(dg, "PoseSaveDialog", FakeDialog)
    assert ui.PoseSaveDialog is not FakeDialog
    g.save_current_pose(sequence="Kiste")
    assert seen == {"sequences": ["Kiste"], "sequence": "Kiste"}
    sent_json = json.loads([v for t, v in strs if t == "/g1pilot/pose_store/save"][-1])
    assert "sequence" not in sent_json and sent_json["id"]
    # arm_controller speichert und quittiert:
    store.save("Neu_greifen", category="Demo", left_arm=ARM)
    node.on_save_status({"id": "fremd", "state": "saved"})
    assert store.get_sequence("Kiste")[-1]["pose"] == "Sichere_Pose"
    node.on_save_status({"id": sent_json["id"], "state": "saved", "name": "Neu_greifen"})
    assert store.get_sequence("Kiste")[-1]["pose"] == "Neu_greifen"
    assert "angehängt" in g.status.text()


def test_save_failure_reported(gui, monkeypatch):
    dg, g, node, sent, strs, states, store = gui

    class FakeDialog:
        def __init__(self, *a, **k):
            pass

        def exec(self):
            return QDialog.DialogCode.Accepted

        def result_data(self):
            return {"name": "X", "category": "Demo", "components": ["left_arm"], "sequence": "Kiste"}

    monkeypatch.setattr(dg, "PoseSaveDialog", FakeDialog)
    g.save_current_pose()
    sid = json.loads([v for t, v in strs if t == "/g1pilot/pose_store/save"][-1])["id"]
    node.on_save_status({"id": sid, "state": "failed", "reason": "Arme nicht aktiv"})
    assert "NICHT gespeichert" in g.status.text()
    assert len(store.get_sequence("Kiste")) == 4


def test_save_dialog_sequence_field():
    app = QApplication.instance() or QApplication([])   # noqa: F841
    from g1pilot.teleoperation.ui_interface import PoseSaveDialog
    d = PoseSaveDialog(None, categories=["Demo"])
    d.ed_name.setText("A")
    assert "sequence" not in d.result_data()                 # Streamdeck unveraendert
    d = PoseSaveDialog(None, categories=["Demo"], sequences=["Kiste"], sequence="Kiste")
    d.ed_name.setText("A")
    assert d.result_data()["sequence"] == "Kiste"
    d.cb_sequence.setCurrentText(PoseSaveDialog.NO_SEQUENCE)
    assert d.result_data()["sequence"] is None
    d.cb_sequence.setCurrentText("Ganz neu")
    assert d.result_data()["sequence"] == "Ganz neu"

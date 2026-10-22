#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ablaeufe ohne ROS/Qt:

  * pose_store: Ablaeufe als Liste von Pose-Verweisen, Altform (Kategorien
    "Ablauf <Name>") wird gelesen und beim Schreiben uebernommen, Speichern in
    "Ablauf <X>" haengt an, arm_controller-Speichern verliert keine Ablaeufe
  * sequence_runner: am Stueck / Schritt fuer Schritt, Moduswechsel im Lauf,
    Abbruch (Status, Stopp, Timeout), alte Endzustaende vor accepted
"""
import json

import pytest

from g1pilot.manipulation.pose_store import PoseStore
from g1pilot.teleoperation import sequence_runner as sr

ARM = [0.1] * 7


# ── pose_store ─────────────────────────────────────────────────────────────
@pytest.fixture
def store(tmp_path):
    return PoseStore(tmp_path / "poses.json")


def _write_v2(path):
    path.write_text(json.dumps({"version": 2, "categories": ["Ablauf A"], "poses": {
        "A_02_zwei": {"category": "Ablauf A", "left_arm": ARM},
        "a_01_eins": {"category": "Ablauf A", "left_arm": ARM},
        "Sichere_Pose": {"category": "Allgemein", "left_arm": ARM},
    }}), encoding="utf-8")


def test_legacy_categories_become_sequences_and_persist(tmp_path):
    path = tmp_path / "poses.json"
    _write_v2(path)
    s = PoseStore(path)
    assert s.list_sequences() == [("A", [{"pose": "a_01_eins", "wait_s": 0.0},
                                         {"pose": "A_02_zwei", "wait_s": 0.0}])]
    # Erstes Schreiben (z. B. Pose speichern im arm_controller) uebernimmt sie fest.
    s.save("Neu", category="Allgemein", right_arm=ARM)
    raw = json.loads(path.read_text(encoding="utf-8"))
    assert raw["version"] == 3 and list(raw["sequences"]) == ["A"]
    # Danach gilt nur noch die Liste: umsortieren bleibt, Kategorie egal.
    s.save_sequence("A", ["A_02_zwei", "Sichere_Pose", "a_01_eins"])
    assert [st["pose"] for st in s.get_sequence("A")] == ["A_02_zwei", "Sichere_Pose", "a_01_eins"]


def test_same_pose_in_several_sequences_and_twice(store):
    store.save("Sichere_Pose", left_arm=ARM)
    store.save("Greifen", left_arm=ARM)
    store.save_sequence("X", ["Sichere_Pose", "Greifen", "Sichere_Pose"])
    store.save_sequence("Y", ["Sichere_Pose"])
    assert [st["pose"] for st in store.get_sequence("X")] == ["Sichere_Pose", "Greifen", "Sichere_Pose"]
    assert store.get_sequence("Y")[0]["pose"] == "Sichere_Pose"
    assert store.list_names() == ["Greifen", "Sichere_Pose"]      # keine Kopien


def test_save_into_ablauf_category_appends_once(store):
    store.save("P1", category="Ablauf Kiste", left_arm=ARM)
    store.save("P2", category="Ablauf Kiste", left_arm=ARM)
    store.save("P1", category="Ablauf Kiste", left_arm=[0.2] * 7)   # ueberschreiben
    assert [st["pose"] for st in store.get_sequence("Kiste")] == ["P1", "P2"]


def test_pose_save_keeps_sequences(store):
    store.save("P", left_arm=ARM)
    store.save_sequence("S", [{"pose": "P", "wait_s": 2}])
    store.save("Q", right_hand=[500] * 6)                     # wie arm_controller
    store.add_category("Leer")
    store.delete("Q")
    assert store.get_sequence("S") == [{"pose": "P", "wait_s": 2.0}]


def test_rename_copy_conflict_delete(store):
    store.create_sequence("A")
    store.append_step("A", "P", wait_s=1.5)
    store.create_sequence("B")
    with pytest.raises(ValueError):
        store.create_sequence("A")
    with pytest.raises(ValueError):
        store.save_sequence("B", [], old_name="A")            # nie still ueberschreiben
    store.save_sequence("C", store.get_sequence("A"), old_name="A")
    assert [n for n, _ in store.list_sequences()] == ["B", "C"]
    assert store.get_sequence("C") == [{"pose": "P", "wait_s": 1.5}]
    assert store.delete_sequence("C") and not store.delete_sequence("C")
    with pytest.raises(ValueError):
        store.save_sequence("  ", [])


def test_steps_cleaned_and_missing_poses(store):
    store.save("P", left_arm=ARM)
    store.save_sequence("S", ["P", {"pose": ""}, {"pose": "Weg", "wait_s": -3},
                              {"pose": "P", "wait_s": 1e9}, 42])
    steps = store.get_sequence("S")
    assert steps == [{"pose": "P", "wait_s": 0.0}, {"pose": "Weg", "wait_s": 0.0},
                     {"pose": "P", "wait_s": 600.0}]
    assert store.missing_poses(steps) == ["Weg"]


def test_install_example_sequences_order(tmp_path, monkeypatch):
    monkeypatch.setenv("G1_POSE_STORE", str(tmp_path / "poses.json"))
    from g1pilot.tools import install_example_sequences as ex
    for name, cat, comps in ex.POSES:
        PoseStore().save(name, category=cat, **comps)
    seq = PoseStore().get_sequence("Arbeitsplatz 1")
    assert [st["pose"] for st in seq] == sorted(st["pose"] for st in seq)
    assert len(seq) == 5


# ── sequence_runner ────────────────────────────────────────────────────────
class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make(mode=sr.MODE_AUTO):
    sent, events, clk = [], [], Clock()
    r = sr.SequenceRunner(sent.append, lambda run, ev, text: events.append((ev, text)),
                          clock=clk, mode=mode)
    return r, sent, events, clk


def reach(r, clk, gap=sr.MIN_GAP_S + 0.01):
    r.on_status({"state": "accepted"})
    r.on_status({"state": "executing"})
    r.on_status({"state": "reached"})
    clk.t += gap
    r.tick()


def test_auto_runs_through():
    r, sent, events, clk = make()
    assert r.start(["A", "B", "C"], "Demo")
    assert sent == ["A"] and r.moving
    reach(r, clk)
    assert sent == ["A", "B"]
    reach(r, clk)
    reach(r, clk)
    assert sent == ["A", "B", "C"] and not r.active
    assert events[-1] == ("done", "Ablauf »Demo« fertig.")


def test_step_mode_waits_for_confirm():
    r, sent, events, clk = make(sr.MODE_STEP)
    r.start(["A", "B"], "Demo")
    assert not r.confirm()                                   # faehrt noch
    reach(r, clk, gap=5.0)
    assert sent == ["A"] and r.state == sr.WAITING and not r.moving
    assert events[-1][0] == "waiting" and "L3" in events[-1][1]
    assert r.snapshot()["state"] == "waiting"
    assert r.confirm() and sent == ["A", "B"]
    assert not r.confirm()                                   # doppelt gedrueckt
    reach(r, clk)
    assert not r.active and events[-1][0] == "done"


def test_mode_switch_during_run():
    r, sent, events, clk = make()
    r.start(["A", "B", "C"], "Demo")
    r.set_mode(sr.MODE_STEP)                                 # anhalten nach diesem Schritt
    reach(r, clk)
    assert r.state == sr.WAITING and sent == ["A"]
    r.set_mode(sr.MODE_AUTO)                                 # geht NICHT von selbst weiter
    clk.t += 10
    r.tick()
    assert r.state == sr.WAITING
    assert r.confirm()
    reach(r, clk)
    assert sent == ["A", "B", "C"]


def test_switch_to_step_during_pause_waits():
    r, sent, _, clk = make()
    r.start([{"pose": "A", "wait_s": 5.0}, "B"], "Demo")
    r.on_status({"state": "accepted"})
    r.on_status({"state": "reached"})
    assert r.state == sr.PAUSE
    r.set_mode(sr.MODE_STEP)
    clk.t += 10
    r.tick()
    assert r.state == sr.WAITING and sent == ["A"]


def test_wait_s_pause_only_in_auto():
    r, sent, _, clk = make()
    r.start([{"pose": "A", "wait_s": 2.0}, "B"])
    r.on_status({"state": "accepted"})
    r.on_status({"state": "reached"})
    clk.t += 1.9
    r.tick()
    assert sent == ["A"]
    clk.t += 0.2
    r.tick()
    assert sent == ["A", "B"]


def test_old_terminal_before_accepted_ignored():
    """'cancelled' der alten Fahrt kommt nach unserem goto, vor dem accepted."""
    r, sent, _, clk = make()
    r.start(["A", "B"], "Demo")
    r.on_status({"state": "cancelled", "reason": "neues Ziel angefordert"})
    r.on_status({"state": "reached"})
    assert r.state == sr.SENT and sent == ["A"]
    reach(r, clk)
    assert sent == ["A", "B"]


@pytest.mark.parametrize("state", ["failed", "cancelled", "rejected"])
def test_abort_on_terminal_error(state):
    r, sent, events, clk = make()
    r.start(["A", "B"], "Demo")
    r.on_status({"state": "accepted"})
    r.on_status({"state": state, "reason": "Planung"})
    assert not r.active and events[-1][0] == "aborted" and state in events[-1][1]
    clk.t += 10
    r.tick()
    assert sent == ["A"]


def test_rejected_before_accepted_aborts():
    r, sent, events, _ = make()
    r.start(["Fehlt"], None)
    r.on_status({"state": "rejected", "reason": "Pose 'Fehlt' nicht gefunden."})
    assert not r.active and "Fehlt" in events[-1][1]


def test_no_ack_times_out():
    r, sent, events, clk = make()
    r.start(["A", "B"], "Demo")
    clk.t += sr.ACK_TIMEOUT_S + 0.1
    r.tick()
    assert not r.active and "Keine Rückmeldung" in events[-1][1]


def test_cancel_while_waiting_never_resumes():
    r, sent, events, clk = make(sr.MODE_STEP)
    r.start(["A", "B"], "Demo")
    reach(r, clk)
    assert r.cancel("NOT-HALT")
    assert events[-1] == ("aborted", "Ablauf »Demo« abgebrochen: NOT-HALT")
    assert not r.confirm()
    r.on_status({"state": "reached"})
    clk.t += 10
    r.tick()
    assert sent == ["A"] and not r.cancel("x")


def test_status_ignored_while_waiting():
    r, sent, _, clk = make(sr.MODE_STEP)
    r.start(["A", "B"], "Demo")
    reach(r, clk)
    r.on_status({"state": "failed"})                         # z. B. Spiegeln am Controller
    assert r.state == sr.WAITING


def test_single_pose_and_empty():
    r, sent, events, clk = make(sr.MODE_STEP)
    assert not r.start([])
    assert r.start(["Sichere_Pose"])
    assert r.prefix() == ""
    reach(r, clk)
    assert events[-1] == ("done", "Pose erreicht.")

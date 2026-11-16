#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fenster der Hand-Werkzeuge (PyQt6, offscreen): »Neue Hand anlegen«
(hand_new.build_wizard) und das Schieber-Fenster von »Hand einpassen«
(hand_adjust.build_panel). Ohne MuJoCo-Render: Vorschlaege kommen aus der
Erkennung (render=False), die Bild-Karten zeigen dann den Platzhalter.
"""
import os
import sys

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PyQt6.QtWidgets")
pytest.importorskip("trimesh")
pytest.importorskip("yaml")

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REPO = os.path.dirname(PKG)
TOOLS = os.path.join(REPO, "unitree_mujoco", "unitree_robots", "g1", "tools")
HAKEN_STL = os.path.join(TOOLS, "..", "hands", "haken", "G1_Box_Hand_1.stl")
sys.path.insert(0, TOOLS)

import hand_qt  # noqa: E402
import hand_new  # noqa: E402
import hand_adjust  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return hand_qt.make_app("g1-hand-tool-test")


def test_wizard_shows_suggestions_and_creates(app, tmp_path, monkeypatch):
    monkeypatch.setattr(hand_new.gen, "HANDS_DIR", tmp_path)
    w = hand_new.build_wizard()
    try:
        assert not w.btn_ok.isEnabled() and w.empty.isVisibleTo(w)
        res = hand_new.analyse(HAKEN_STL, None, render=False)
        w.file_path = HAKEN_STL
        w._show_file()
        w.ed_title.setText("Box Haken")
        assert w.slug_lbl.text() == "Ordner: hands/box_haken/"
        w.show_result(res)
        assert len(w.cards) == len(res["suggestions"]) >= 2
        assert w.cards[0].isChecked() and w.pick() == 0 and w.btn_ok.isEnabled()
        assert w.unit_btns[res["unit"]].isChecked()
        assert float(w.ed_mass.text()) == pytest.approx(res["mass"])
        w.cards[1].click()
        assert w.pick() == 1 and not w.cards[0].isChecked()
        # Neuer Durchgang (z. B. Einheit) behaelt die Wahl
        w.show_result(res)
        assert w.pick() == 1
        # Anlegen schreibt das Profil; den Build-Lauf nicht starten
        monkeypatch.setattr(hand_new.threading, "Thread",
                            lambda target, daemon: type("T", (), {"start": lambda s: None})())
        w.create()
        yaml = (tmp_path / "box_haken" / "hand.yaml").read_text(encoding="utf-8")
        assert "Vorschlag 2" in yaml and not w.btn_ok.isEnabled()
        w._built("box_haken", 0, "")
        assert w.next_slug == "box_haken"
    finally:
        w.close()


def test_wizard_rejects_bad_mass(app, monkeypatch):
    from PyQt6.QtWidgets import QMessageBox
    warned = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *a: warned.append(a[2]))
    w = hand_new.build_wizard()
    try:
        w.file_path = HAKEN_STL
        w.ed_title.setText("x")
        w.show_result(hand_new.analyse(HAKEN_STL, None, render=False))
        w.ed_mass.setText("-1")
        w.create()
        assert warned and "Masse" in warned[0]
    finally:
        w.close()


def test_adjust_panel_sends_state(app):
    sent = []
    p = hand_adjust.build_panel("haken", sent.append)
    try:
        assert len(p.rows) == 6
        p.push()
        assert sent[-1] == {"mm": [0.0] * 3, "deg": [0.0] * 3, "glass": True, "col": False}
        p.rows[0].slider.setValue(int(3.5 * p.rows[0].scale))    # Schieber -> Zahl
        assert sent[-1]["mm"] == [3.5, 0.0, 0.0]
        p.rows[4].set_value(-12)                                 # Zahl -> Schieber
        assert p.rows[4].slider.value() == -12 * p.rows[4].scale
        assert sent[-1]["deg"] == [0.0, -12.0, 0.0]
        p.col.setChecked(True)
        assert sent[-1]["col"] is True
        n = len(sent)
        p.on_message({"status": "Ergebnis fuer hand.yaml: ..."})
        assert "Ergebnis" in p.status.text() and len(sent) == n
        p.on_message({"reset": True})
        assert sent[-1]["mm"] == [0.0] * 3 and sent[-1]["deg"] == [0.0] * 3
        p.btn_save.click()
        assert sent[-1] == {"cmd": "save"}
    finally:
        p.close()
    assert sent[-1] == {"cmd": "quit"}

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Demo-GUI, Ansicht Touch | Controller (2026-11-18): in der Controller-
Ansicht fallen Joysticks, Pfeiltasten, Hueft-Schieber und HAENDE weg, dafuer
Controller-Zustand, Belegung und die Pose-Schnellwahl. Nur Oberflaeche:
Klicks gehen weiter, Topics bleiben gleich. Offscreen, ohne Roboter."""
import os
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy                                       # noqa: E402
from PyQt6.QtCore import QTimer                  # noqa: E402
from PyQt6.QtWidgets import QApplication           # noqa: E402

SLOTS = {"up-up": "Grundstellung", "down-down": "Becher greifen"}


@pytest.fixture(params=["true", "false"], ids=["sim", "real"])
def gui(request, monkeypatch, tmp_path):
    monkeypatch.setenv("G1_PHONE", "0")
    monkeypatch.setenv("G1_SIM_MODE", request.param)
    monkeypatch.setenv("G1_POSE_STORE", str(tmp_path / "poses.json"))
    monkeypatch.delenv("G1_DEMO_VIEW", raising=False)
    from g1pilot.manipulation.pose_store import PoseStore
    store = PoseStore()
    for name in SLOTS.values():
        store.save(name, left_arm=[0.0] * 7, right_arm=[0.0] * 7)
    for combo, name in SLOTS.items():
        store.set_quickslot(combo, name)
    rclpy.init()
    app = QApplication.instance() or QApplication([])
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    g.resize(1366, 768)
    g.show()
    _pump(app)
    try:
        yield dg, g, app
    finally:
        g.close()
        for t in g.findChildren(QTimer):     # keine Timer nach destroy_node
            t.stop()
        node.tf_listener.executor.shutdown()
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def _pump(app, seconds=0.2):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        app.processEvents()
        time.sleep(0.01)


def _mode(dg, g, app, mode):
    g.started, g.mode = True, mode
    g._refresh()
    g._reload_sequence_buttons()
    _pump(app)


def _fake_pad(g, **extra):
    now = time.monotonic()
    st = {"connected": True, "selection": "right", "mode": "move", "speed": "Normal",
          "speed_req": "Normal", "waist_mode": False, "quick": None}
    st.update(extra)
    g.node.ps4, g.node.ps4_t = st, now
    g.node.ps4dev, g.node.ps4dev_t = {"kind": "ps4", "name": "Wireless Controller"}, now


def test_default_touch_and_env(gui, monkeypatch):
    dg, g, app = gui
    from g1pilot.teleoperation import demo_ui as UI
    assert g.view == UI.VIEW_TOUCH
    assert g.view_btns[UI.VIEW_TOUCH].isChecked()
    assert UI.default_view("controller") == UI.VIEW_CONTROLLER
    assert UI.default_view("") == UI.VIEW_TOUCH
    assert UI.default_view("quatsch") == UI.VIEW_TOUCH


def test_grasp_controller_view_swaps_cards(gui):
    """GREIFEN: Hand bewegen + ARME/HAENDE weg, Controller/Belegung/Schnellwahl
    da; Abläufe, Posen und »Bewegung stoppen« bleiben. Zurueck stellt alles her."""
    dg, g, app = gui
    _mode(dg, g, app, dg.MANIP)
    mp = g.manip_panel
    g.view_btns["controller"].click()
    _pump(app)
    assert g.view == "controller"
    assert not mp.jog_card.isVisibleTo(g) and not mp.view_switch.isVisibleTo(g)
    for w in (mp.ctrl_card, mp.legend_card, mp.quick_card, mp.seq_card, mp.pose_card,
              mp.btn_cancel):
        assert w.isVisibleTo(g)
    assert mp.pose_card.parent() is mp.ctrl_col           # Posen links unter Controller
    g.view_btns["touch"].click()
    _pump(app)
    assert mp.jog_card.isVisibleTo(g) and mp.view_switch.isVisibleTo(g)
    assert not mp.ctrl_col.isVisibleTo(g) and not mp.quick_card.isVisibleTo(g)
    assert mp.pose_card.parent() is not mp.ctrl_col


def test_walk_controller_view(gui):
    dg, g, app = gui
    _mode(dg, g, app, dg.WALK)
    wp = g.walk_panel
    g.set_view("controller")
    _pump(app)
    assert not wp.manual_card.isVisibleTo(g)
    assert wp.ctrl_card.isVisibleTo(g) and wp.nav_card.isVisibleTo(g)
    _fake_pad(g)
    g.node.ps4walk, g.node.ps4walk_t = {"active": True, "speed": "Normal"}, time.monotonic()
    g._refresh_controller_view()
    assert "läuft" in wp.ctrl_card.facts["state"].value.text()
    keys = [k for k, _ in wp.legend_card.rows()]
    assert "L2 halten" in keys and "PS" in keys
    assert "Dreieck" not in keys                       # Arm-Tasten nicht beim Gehen
    g.set_view("touch")
    assert wp.manual_card.isVisibleTo(g)


def test_quick_grid_shows_slots_and_clicks(gui):
    """Schnellwahl: belegte Plaetze mit Namen, Klick faehrt an (run_poses),
    leerer Platz tut nichts; Wahl am Controller leuchtet."""
    dg, g, app = gui
    _mode(dg, g, app, dg.MANIP)
    g.set_view("controller")
    grid = g.manip_panel.quick_grid
    assert grid.cells["up-up"].text() == "Grundstellung"
    assert grid.cells["left-left"].text() == "–"
    started = []
    g.run_poses = lambda names, label=None: started.append(list(names))
    grid.cells["down-down"].click()
    grid.cells["left-left"].click()
    assert started == [["Becher greifen"]]
    _fake_pad(g, waist_mode=True, quick={"first": "down", "combo": None})
    g._refresh_controller_view()
    assert grid.state_of("down-up") == "row" and grid.state_of("up-up") == ""
    _fake_pad(g, waist_mode=True, quick={"first": "down", "combo": "down-down"})
    g._refresh_controller_view()
    assert grid.state_of("down-down") == "live" and grid.state_of("down-up") == ""


def test_controller_facts_and_legend(gui):
    dg, g, app = gui
    _mode(dg, g, app, dg.MANIP)
    g.set_view("controller")
    card = g.manip_panel.ctrl_card
    assert "Kein Controller" in card.line.text()
    _fake_pad(g, selection="left", mode="turn", speed="Langsam", speed_req="Langsam")
    g._refresh_controller_view()
    assert card.facts["hand"].value.text() == "LINKS"
    assert card.facts["motion"].value.text() == "Drehen"
    assert card.facts["speed"].value.text() == "Langsam"
    keys = [k for k, _ in g.manip_panel.legend_card.rows()]
    for k in ("Dreieck", "R2 halten + R3", "L3 (linken Stick drücken)", "PS"):
        assert k in keys
    assert "L2 halten" not in keys                     # Gehen nicht im GREIFEN
    g._refresh_info()
    assert not g.btn_pad_help.isVisibleTo(g)           # Belegung steht schon da
    g.set_view("touch")
    g._refresh_info()
    assert g.btn_pad_help.isVisibleTo(g)


def test_controller_view_sends_no_gui_motion(gui):
    """Hueft-Schieber/Joysticks sind in der Controller-Ansicht weg: auch ein
    haengender Schieber-Wert sendet keine Hueft-Rate."""
    dg, g, app = gui
    _mode(dg, g, app, dg.MANIP)
    g.set_view("controller")
    sent = []
    g.node.pub_waist_rate.publish = lambda m: sent.append(m.data)
    g.manip_panel.waist_rocker.value = 0.8
    g._waist_tick()
    assert sent == [] and not g._waist_sending
    assert g.manip_panel.waist_rocker.value == 0.0


def test_controller_view_min_size(gui):
    dg, g, app = gui
    _mode(dg, g, app, dg.MANIP)
    g.set_view("controller")
    _fake_pad(g)
    g._refresh_info()
    _pump(app)
    hint = g.minimumSizeHint()
    assert hint.width() <= 1366 and hint.height() <= 700

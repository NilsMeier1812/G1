#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Demo-GUI-Aufbau (Neuaufbau 2026-11-17): Listen skalieren auf 20+ Eintraege,
Spalten folgen der Breite, Meldungs-Verlauf, keine Seite erzwingt eine zu
grosse Mindestgroesse. Offscreen, ohne Roboter."""
import os
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy                                       # noqa: E402
from PyQt6.QtWidgets import QApplication, QPushButton   # noqa: E402

SEQS = [f"Ablauf {i:02d}" for i in range(24)]


@pytest.fixture(params=["true", "false"], ids=["sim", "real"])
def gui(request, monkeypatch):
    monkeypatch.setenv("G1_PHONE", "0")
    monkeypatch.setenv("G1_SIM_MODE", request.param)
    rclpy.init()
    app = QApplication.instance() or QApplication([])
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    g = dg.DemoGUI(node)
    g._sequences = lambda: [(s, ["P1", "P2", "P3"]) for s in SEQS]
    g.resize(1366, 768)
    g.show()
    _pump(app)
    try:
        yield dg, g, app
    finally:
        g.close()
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


def _grasp(dg, g, app):
    g.started, g.mode = True, dg.MANIP
    g._refresh()
    g._reload_sequence_buttons()
    _pump(app)


def test_many_sequences_grid_with_search(gui):
    """24 Ablaeufe: alle als Kachel im Raster (mehrere Spalten), Suchfeld da,
    Suche filtert, Klick startet den richtigen Ablauf."""
    dg, g, app = gui
    _grasp(dg, g, app)
    mp = g.manip_panel
    grid = mp.seq_grid
    labels = [mp.seq_row.itemAt(i).widget().text() for i in range(mp.seq_row.count())]
    assert labels == SEQS
    assert grid.search.isVisibleTo(g)
    assert grid._cols >= 2
    assert "24 gespeichert" in mp.seq_card.hint.text()
    grid.search.setText("Ablauf 1")
    _pump(app, 0.05)
    shown = [w.text() for w in grid.widgets() if w.isVisibleTo(g)]
    assert shown == [f"Ablauf {i:02d}" for i in range(10, 20)]
    started = []
    g.run_sequence = lambda name: started.append(name)
    grid.widgets()[13].click()
    assert started == ["Ablauf 13"]


def test_few_sequences_no_search(gui):
    dg, g, app = gui
    g._sequences = lambda: [("Winken", ["P1"]), ("Leer", [])]
    _grasp(dg, g, app)
    grid = g.manip_panel.seq_grid
    assert [w.text() for w in grid.widgets()] == ["Winken"]       # leere ohne Knopf
    assert not grid.search.isVisibleTo(g)
    g._sequences = lambda: []
    g._reload_sequence_buttons()
    assert grid.widgets() == [] and grid.empty.isVisibleTo(g)
    assert "Noch keine Abläufe" in grid.empty.text()


def test_grid_columns_follow_width(gui):
    """Breiteres Fenster -> mehr Spalten im Ablauf-Raster."""
    dg, g, app = gui
    _grasp(dg, g, app)
    narrow = g.manip_panel.seq_grid._cols
    g.resize(1920, 1080)
    _pump(app, 0.3)
    assert g.manip_panel.seq_grid._cols > narrow


def test_running_sequence_dashed(gui):
    dg, g, app = gui
    _grasp(dg, g, app)
    mp = g.manip_panel
    mp._style_sequences("Ablauf 03")
    assert "dashed" in mp.seq_buttons["Ablauf 03"].styleSheet()
    assert "dashed" not in mp.seq_buttons["Ablauf 04"].styleSheet()
    mp._style_sequences(None)
    assert "dashed" not in mp.seq_buttons["Ablauf 03"].styleSheet()


def test_many_stations_grid(gui):
    dg, g, app = gui
    names = [f"station_Platz_{i}" for i in range(22)]
    g.walk_panel.set_stations(names)
    g.started, g.mode = True, dg.WALK
    g._refresh()
    _pump(app)
    sg = g.walk_panel.station_grid
    assert len(sg.widgets()) == 22 and sg.search.isVisibleTo(g)
    assert sg._cols >= 2
    g.walk_panel.set_stations([])
    assert sg.widgets() == [] and "station_<Name>" in sg.empty.text()


def test_status_history(gui):
    """Statusleiste merkt sich die Meldungen (Wiederholung nur einmal), der
    Knopf »Verlauf« zeigt sie, neueste oben."""
    dg, g, app = gui
    g._status("eins", "#aaa")
    g._status("eins", "#aaa")
    g._status("zwei", "#ef5350")
    texts = [t for _, t, _ in g._history]
    assert texts[-2:] == ["eins", "zwei"] and texts.count("eins") == 1
    g.btn_history.click()
    _pump(app, 0.05)
    lst = g._history_dialog.list
    assert lst.item(0).text().endswith("zwei")
    g._status("drei", "#aaa")                    # offenes Fenster zieht nach
    assert lst.item(0).text().endswith("drei")
    g._history_dialog.close()


def test_idle_text_guides(gui):
    """Ohne Modus sagt die Arbeitsflaeche, was als Naechstes zu tun ist."""
    dg, g, app = gui
    g.mode = g.pending = None
    g.started = False
    g._refresh()
    assert "Start" in g.idle_panel.text()
    g.started = True
    g._refresh()
    assert "Modus" in g.idle_panel.text()


def test_sidebar_matches_mode(gui):
    """Seitenleiste: real Startleiste + Akku/Temperaturen, Sim die Sim-Werkzeuge."""
    dg, g, app = gui
    sim = g.sim_mode
    assert g.start_bar.isVisibleTo(g) is (not sim)
    assert g.temp_box.isVisibleTo(g) is (not sim)
    assert g.sim_box.isVisibleTo(g) is sim
    for b in (g.btn_reset_robot, g.btn_push, g.btn_reset_scene):
        assert b.isVisibleTo(g) is sim
    assert g.btn_estop.isVisibleTo(g) and g.btn_quit.isVisibleTo(g)


def test_hands_page_does_not_widen_arms(gui):
    """Die (breite) HAENDE-Seite bestimmt nicht die Mindestbreite, solange
    ARME angezeigt wird (CurrentStack)."""
    dg, g, app = gui
    if not g.manip_panel.hands_available:
        pytest.skip("HAENDE in dieser Sim ausgegraut")
    _grasp(dg, g, app)
    mp = g.manip_panel
    arms_w = mp.minimumSizeHint().width()
    mp.show_view("hands")
    _pump(app, 0.05)
    assert mp.minimumSizeHint().width() >= arms_w
    mp.show_view("arms")
    _pump(app, 0.05)
    assert mp.minimumSizeHint().width() == arms_w


def test_no_plain_emoji_labels(gui):
    """Knoepfe tragen Lucide-Icons, keine Emoji-Glyphen im Text."""
    dg, g, app = gui
    _grasp(dg, g, app)
    for b in g.findChildren(QPushButton):
        assert not any(ord(c) > 0x2FFF and not (0xFE00 <= ord(c) <= 0xFE0F)
                       for c in b.text()), b.text()

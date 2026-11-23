#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Grafisches Startmenue (start_gui, PyQt6): gleiche Start-Variablen wie das
fruehere Tk-Menue, Ansichten bauen sich auf, Doku-Viewer rendert alle Dokumente,
Vorschaubild-Cache. Ohne Docker/Roboter, Qt offscreen."""
import os
import sys
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["G1_START_GUI_NO_EXIT"] = "1"

PKG_ROOT = Path(__file__).resolve().parent.parent          # .../g1pilot
sys.path.insert(0, str(PKG_ROOT))

from start_gui import core  # noqa: E402

pytest.importorskip("PyQt6.QtWidgets")


# ════════════════════════════════════════════════════════════════════════
#  core: Env-Variablen fuer start.sh (sicherheitsrelevant: Real-Gate)
# ════════════════════════════════════════════════════════════════════════
def test_sim_launch_env_matches_old_menu():
    L = core.sim_launch(env_name="scene_A5", policy="agile_velocity_g1", hand_profile="inspire",
                        nav=True, ps4=True, rviz=True, rt="0.5", rebuild=True, base_env={})
    assert L.env == {
        "G1_MODE": "sim", "G1_ENV": "scene_A5", "G1_WALK_POLICY": "agile_velocity_g1",
        "G1_INSPIRE_HANDS": "1", "G1_HANDS": "inspire", "G1_ENABLE_NAV": "1",
        "G1_PS4_ARMS": "1", "G1_PHONE": "1", "G1_SIM_APPARMOR": "unconfined",
        "USE_RVIZ": "true", "SIM_REALTIME_FACTOR": "0.5"}
    assert L.argv[-2:] == ["--yes", "--build"]
    assert L.stop_cmd[:4] == ["docker", "compose", "--profile", "sim"]


def test_sim_launch_custom_hand_is_not_inspire():
    L = core.sim_launch(env_name="", policy="g1_wholebody", hand_profile="haken", nav=False,
                        ps4=False, rviz=False, rt="1.0", rebuild=False, base_env={})
    assert L.env["G1_HANDS"] == "haken" and L.env["G1_INSPIRE_HANDS"] == "0"
    assert L.env["G1_ENV"] == "" and L.env["USE_RVIZ"] == "false"
    assert "--build" not in L.argv


def _real_vals(**kw):
    v = {"iface": "enp3s0", "hands": True, "left": "192.168.123.210",
         "right": "192.168.123.211", "port": "6000", "rviz": True, "vx": "0.4",
         "vy": "0.3", "vyaw": "0.4", "joy": "Wireless Controller", "ps4": False}
    v.update(kw)
    return v


def test_real_launch_env():
    L = core.real_launch(_real_vals(vx="0.2"), rebuild=False, base_env={})
    assert L.env["G1_MODE"] == "real"
    assert L.env["G1_REAL_CONFIRM"] == "1"
    assert L.env["ROBOT_INTERFACE"] == "enp3s0"
    assert L.env["G1_MAX_VX"] == "0.2"
    assert L.env["G1_INSPIRE_HANDS"] == "1"
    assert L.stop_cmd[:4] == ["docker", "compose", "--profile", "real"]


def test_scene_summary(tmp_path):
    x = tmp_path / "s.xml"
    x.write_text('<mujoco><worldbody><geom name="asset_Tisch"/><geom name="grasp_asset_KLT"/>'
                 '<geom name="grasp_asset_KLT__rippe"/><geom name="station_A"/>'
                 '<geom name="station_B"/></worldbody></mujoco>')
    assert core.scene_summary(x) == "1 Möbel · 1 Objekt · 2 Stationen"
    y = tmp_path / "t.xml"
    y.write_text('<mujoco><worldbody><geom name="a"/><geom type="box"/></worldbody></mujoco>')
    assert core.scene_summary(y) == "2 Körper"


def test_stale_thumbs(tmp_path):
    scenes, thumbs = tmp_path / "scenes", tmp_path / "thumbs"
    scenes.mkdir()
    thumbs.mkdir()
    std = tmp_path / "scene.xml"
    std.write_text("<mujoco/>")
    (scenes / "a.xml").write_text("<mujoco/>")
    assert core.stale_thumbs(scenes, thumbs, std) == [core.STANDARD_THUMB, "a"]
    import json
    (thumbs / "a.png").write_bytes(b"x")
    (thumbs / f"{core.STANDARD_THUMB}.png").write_bytes(b"x")
    h = core._xml_hash(std)
    (thumbs / "index.json").write_text(json.dumps({"a": h, core.STANDARD_THUMB: h}))
    assert core.stale_thumbs(scenes, thumbs, std) == []
    (scenes / "a.xml").write_text("<mujoco><worldbody/></mujoco>")   # geaendert
    assert core.stale_thumbs(scenes, thumbs, std) == ["a"]


def test_committed_thumbs_are_pngs():
    # Eingecheckte Vorschaubilder: gueltige PNGs, Standard-Kachel vorhanden.
    pngs = list(core.THUMBS_DIR.glob("*.png"))
    assert core.thumb_path(core.STANDARD_THUMB) is not None
    for p in pngs:
        assert p.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_docs_index_lists_every_doc():
    head, groups = core.docs_index()
    listed = {e.path for e in head}
    for g in groups:
        assert g.entry is not None, f"Gruppe {g.number} ohne Uebersicht"
        listed.add(g.entry.path)
        listed |= {t.path for t in g.topics}
    for p in core.DOCS_DIR.glob("*.md"):
        assert p in listed, f"{p.name} fehlt im Doku-Viewer"


def test_github_slug():
    assert core.github_slug("Vorschaubilder der Umgebungen") == "vorschaubilder-der-umgebungen"
    assert core.github_slug("`make` und `docker compose`") == "make-und-docker-compose"
    assert core.github_slug("B — Netzwerk: Ersteinrichtung") == "b--netzwerk-ersteinrichtung"


# ════════════════════════════════════════════════════════════════════════
#  GUI (offscreen)
# ════════════════════════════════════════════════════════════════════════
@pytest.fixture
def win(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    from start_gui.app import MainWindow, make_app
    app = make_app([])
    w = MainWindow(check_docker=False, auto_thumbs=False)
    started = []

    def fake_start(title, argv, **kw):
        started.append((title, argv, kw))
    w._started = started
    w._fake_start = fake_start
    yield w
    for c in list(w.consoles):
        if c.is_running():
            c._proc.kill()
    if w._docs is not None:
        w._docs.close()
    w.close()
    app.processEvents()


def _pump(seconds=0.0):
    from PyQt6.QtWidgets import QApplication
    end = time.monotonic() + seconds
    while True:
        QApplication.processEvents()
        if time.monotonic() >= end:
            break
        time.sleep(0.01)


def test_menu_tiles_open_views(win):
    from start_gui.views import MenuView, RealView, ScenesView, SimView
    assert isinstance(win.current, MenuView)
    win.current.tiles["sim"].click()
    assert isinstance(win.current, SimView)
    win.show_menu()
    win.current.tiles["real"].click()
    assert isinstance(win.current, RealView)
    win.show_menu()
    win.current.tiles["scenes"].click()
    assert isinstance(win.current, ScenesView)


def test_sim_view_start_and_settings(win, monkeypatch):
    win.open_sim()
    v = win.current
    monkeypatch.setattr(win, "start_process", win._fake_start)
    # Umgebung waehlen: jede Szene hat eine Kachel (+ Standard)
    assert set(v.env_cards) == {""} | {p.stem for p in core.list_scenes()}
    other = next((k for k in v.env_cards if k), "")
    v.env_cards[other].click()
    v.policy.cards["agile_velocity_height_g1"].click()
    v.hand_kind.cards["rubber"].click()
    v.ps4.setChecked(True)
    v.start_btn.click()
    title, argv, kw = win._started[-1]
    assert title == "Simulation"
    env = kw["env"]
    assert env["G1_ENV"] == other
    assert env["G1_WALK_POLICY"] == "agile_velocity_height_g1"
    assert env["G1_HANDS"] == "rubber" and env["G1_INSPIRE_HANDS"] == "0"
    assert env["G1_PS4_ARMS"] == "1"
    # gemerkt -> neue Ansicht startet mit derselben Auswahl
    win.open_sim()
    v2 = win.current
    assert v2.env_name() == other
    assert v2.policy.value() == "agile_velocity_height_g1"
    assert v2.hand_kind.value() == "rubber"
    assert v2.ps4.isChecked()


def test_sim_navigation_forces_rviz_and_restores(win):
    win.open_sim()
    v = win.current
    v.rviz.setChecked(False)
    v.nav.setChecked(True)
    assert v.rviz.isChecked() and not v.rviz.isEnabled()
    v.nav.setChecked(False)
    assert not v.rviz.isChecked() and v.rviz.isEnabled()


def test_sim_rejects_bad_tempo(win, monkeypatch):
    from g1pilot.utils import msgbox
    win.open_sim()
    v = win.current
    monkeypatch.setattr(win, "start_process", win._fake_start)
    shown = []
    monkeypatch.setattr(msgbox, "error", lambda *a, **k: shown.append(a))
    v.rt.setText("schnell")
    v.start_btn.click()
    assert shown and not win._started
    assert v.advanced.is_open()


def test_sim_custom_hand_list(win):
    win.open_sim()
    v = win.current
    v.hand_kind.cards["custom"].click()
    assert v.custom_box.isVisibleTo(v)
    items = [v.hand_combo.itemText(i) for i in range(v.hand_combo.count())]
    assert items[-1].endswith("Neue Hand anlegen …")
    assert items[:-1] == [t for _, t in core.list_hand_profiles()]
    if len(items) > 1:
        v.hand_combo.setCurrentIndex(0)
        assert v.hand_profile() == core.list_hand_profiles()[0][0]
        assert v.edit_btn.text().startswith("Bearbeiten")
    v.hand_combo.setCurrentIndex(len(items) - 1)
    assert v.edit_btn.text().startswith("Anlegen")
    v.hand_kind.cards["inspire"].click()
    assert not v.custom_box.isVisibleTo(v)


def test_sim_many_environments_scale(win, monkeypatch, tmp_path):
    """20 Umgebungen: Suchfeld, eingeklappt (7 + »Neue Umgebung«), gewaehlte
    Umgebung bleibt sichtbar, »Alle anzeigen« klappt auf, Suche filtert."""
    from start_gui import views
    scenes = []
    for i in range(20):
        x = tmp_path / f"halle_{i:02d}.xml"
        x.write_text("<mujoco><worldbody/></mujoco>")
        scenes.append(x)
    monkeypatch.setattr(core, "list_scenes", lambda *a, **k: scenes)
    win.open_sim()
    v = win.current
    assert not v.env_search.isHidden()
    shown = v.visible_envs()
    assert len(shown) == views.ENV_COLLAPSED and not v.env_more.isHidden()
    assert "21" in v.env_more.text()
    # gewaehlte Umgebung ausserhalb der ersten Reihen bleibt sichtbar
    v.env_cards["halle_15"].click()
    v._apply_env_view()
    assert "halle_15" in v.visible_envs() and len(v.visible_envs()) == views.ENV_COLLAPSED
    v.env_more.click()
    assert len(v.visible_envs()) == 21
    v.env_more.click()
    v.env_search.setText("halle_1")
    assert v.visible_envs() == [f"halle_{i}" for i in range(10, 20)]
    assert v.env_more.isHidden()
    v.env_search.setText("standard")
    assert v.visible_envs() == [""]


def test_gui_sources_have_no_emoji():
    """Keine Emoji-/Symbol-Glyphen in den Oberflaechen (Icons kommen aus
    resource/icons/ui). Pfeile im Fliesstext (→) sind erlaubt."""
    import re
    bad = re.compile("[\U0001F000-\U0001FFFF\u2600-\u27BF\u2B00-\u2BFF\u25A0-\u25FF"
                     "\u2300-\u23FF\uFE0F\uFF0B]")
    here = Path(__file__).resolve().parents[1]
    tools = here.parent / "unitree_mujoco" / "unitree_robots" / "g1" / "tools"
    files = (list((here / "start_gui").glob("*.py"))
             + [here / "g1pilot" / "teleoperation" / f for f in (
                 "demo_gui.py", "hand_panel.py", "sequence_editor.py", "status_line.py", "dialog_style.py",
                 "gui_base.py", "sequence_runner.py")]
             + [here / "g1pilot" / "teleoperation" / "phone" / f for f in (
                 "link.py", "web/app.js", "web/index.html")]
             + [tools / "hand_new.py", tools / "hand_adjust.py", tools / "hand_qt.py"])
    hits = []
    for f in files:
        for n, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
            code = line.split("#", 1)[0] if f.suffix == ".py" else line
            if bad.search(code):
                hits.append(f"{f.name}:{n}: {line.strip()[:80]}")
    assert hits == []


def test_ui_icons_render(win):
    from g1pilot.utils import ui_icons
    assert {"play", "smartphone", "gamepad-2", "check", "chevron-right"} <= set(ui_icons.names())
    pm = ui_icons.pixmap("play", "#ff0000", 24)
    assert not pm.isNull() and pm.width() >= 24
    assert ui_icons.pixmap("gibt-es-nicht").isNull()


def test_real_view_gate(win, monkeypatch):
    from start_gui.views import RealView
    v = RealView(win, nics=[("wlp2s0", "10.0.0.5/24"), ("enp0s31f6", "192.168.123.222/24")])
    win.show_view(v)
    monkeypatch.setattr(win, "start_process", win._fake_start)
    assert v._iface() == "enp0s31f6"            # Roboter-LAN vorgewaehlt
    assert "Im Roboter-LAN" in v.lan.text()
    assert not v.start_btn.isEnabled()
    v.start_btn.click()
    assert not win._started                     # ohne Haekchen kein Start
    v.hands.setChecked(False)
    assert not v.left_row.isEnabled()
    v.confirm.setChecked(True)
    assert v.start_btn.isEnabled()
    v.start_btn.click()
    title, argv, kw = win._started[-1]
    assert title == "Echter Roboter"
    assert kw["env"]["G1_REAL_CONFIRM"] == "1"
    assert kw["env"]["ROBOT_INTERFACE"] == "enp0s31f6"
    assert kw["env"]["G1_INSPIRE_HANDS"] == "0"
    # Bestaetigung wird NIE gemerkt
    v2 = RealView(win, nics=[])
    assert not v2.confirm.isChecked() and not v2.start_btn.isEnabled()
    assert "confirm" not in core.load_settings("real")


def test_real_view_unknown_lan(win):
    from start_gui.views import RealView
    v = RealView(win, nics=[("wlp2s0", "10.0.0.5/24")])
    assert "Keine 192.168.123.x" in v.lan.text()


def test_scenes_view_setup_needed(win, monkeypatch):
    from start_gui import views
    monkeypatch.setattr(views.core, "scene_editor_ready", lambda: False)
    win.open_scenes()
    assert win.current.inspire is None           # nur die Setup-Karte


def test_scenes_view_actions(win, monkeypatch):
    from start_gui import views
    monkeypatch.setattr(views.core, "scene_editor_ready", lambda: True)
    win.open_scenes()
    v = win.current
    monkeypatch.setattr(win, "start_process", win._fake_start)
    scenes = core.list_scenes()
    if not scenes:
        pytest.skip("keine Umgebungen im Repo")
    v.cards[scenes[-1].stem].click()
    v._run_scene("with-g1")
    title, argv, kw = win._started[-1]
    assert argv[-2:] == ["with-g1", str(scenes[-1])]
    assert kw["scene_editor"] is True and kw["browser_url"] is None
    v._run_scene("edit")
    assert win._started[-1][2]["browser_url"] == "http://127.0.0.1:8080"


def test_console_runs_and_stops(win):
    from start_gui.views import MenuView
    c = win.start_process("Test", ["bash", "-c", "echo hallo-welt; sleep 30"], cwd=PKG_ROOT,
                          stop_cmd=["bash", "-c", "echo aufgeraeumt"])
    assert win.current is c
    t0 = time.monotonic()
    while "hallo-welt" not in c.log_text() and time.monotonic() - t0 < 5:
        _pump(0.05)
    assert "hallo-welt" in c.log_text() and c.is_running()
    c.stop()
    t0 = time.monotonic()
    while c.state() != "gestoppt" or not c._finished:
        _pump(0.05)
        assert time.monotonic() - t0 < 20
    assert "aufgeraeumt" in c.log_text()
    _pump(0.1)
    assert isinstance(win.current, MenuView)     # nach Nutzer-Stopp zurueck ins Menue
    win.remove_console(c)
    assert c not in win.consoles


def test_console_failed_process_stays_visible(win):
    c = win.start_process("Kaputt", ["bash", "-c", "echo boom; exit 3"], cwd=PKG_ROOT)
    t0 = time.monotonic()
    while not c._finished:
        _pump(0.05)
        assert time.monotonic() - t0 < 10
    assert win.current is c                      # Log bleibt lesbar
    assert "Code 3" in c._status.text()


def test_docs_viewer_renders_all_docs(win):
    win.open_doc("")
    d = win._docs
    assert d._current.name == "README.md"
    head, groups = core.docs_index()
    paths = [e.path for e in head] + [g.entry.path for g in groups] + \
        [t.path for g in groups for t in g.topics]
    for p in paths:
        assert d.open_doc(p), p
        assert d.view.document().characterCount() > 50, p.name


def test_docs_viewer_links_and_history(win):
    from PyQt6.QtCore import QUrl
    win.open_doc("03-A-Startmenue.md#doku-viewer")
    d = win._docs
    assert d._current.name == "03-A-Startmenue.md"
    assert d.view.verticalScrollBar().value() > 0        # zum Abschnitt gesprungen
    d._on_link(QUrl("11-A-Controller.md"))
    assert d._current.name == "11-A-Controller.md"
    d.back()
    assert d._current.name == "03-A-Startmenue.md"
    d.forward()
    assert d._current.name == "11-A-Controller.md"
    opened = []
    d._open_external = opened.append
    d._on_link(QUrl("../start.sh"))
    assert opened and opened[0].name == "start.sh"


def test_docs_viewer_search_filters_by_content(win):
    win.open_doc("")
    d = win._docs
    d.search.setText("render_thumbs")
    visible = [d.tree.topLevelItem(i) for i in range(d.tree.topLevelItemCount())
               if not d.tree.topLevelItem(i).isHidden()]
    names = []
    for top in visible:
        for j in range(top.childCount()):
            if not top.child(j).isHidden():
                names.append(Path(top.child(j).data(0, 256)).name)
    assert "03-A-Startmenue.md" in names
    d.search.setText("")


def test_hint_links_open_viewer(win):
    win.open_sim()
    win.handle_link("doc:25-A-Eigene-Haende.md")
    assert win._docs._current.name == "25-A-Eigene-Haende.md"

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Demo-GUI und Controller teilen EINEN Bedienzustand (/g1pilot/teleop_state).

Offscreen, ohne Roboter: Nachrichten des Controllers (ps4_arm_teleop /
ps4_walk) werden direkt in die GUI eingespielt; geprueft wird, dass Kacheln,
Hand-Auswahl und Tempo folgen, OHNE dass die GUI dabei etwas an den Roboter
sendet, und dass Wechsel in der GUI im gemeinsamen Zustand landen.
"""
import json
import os
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("PyQt6")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy                                       # noqa: E402
from PyQt6.QtWidgets import QApplication          # noqa: E402
from std_msgs.msg import Bool, String              # noqa: E402

from g1pilot.teleoperation import shared_state as ss   # noqa: E402


@pytest.fixture(params=["real", "sim"])
def gui(request, monkeypatch):
    monkeypatch.setenv("G1_SIM_MODE", "false" if request.param == "real" else "true")
    rclpy.init()
    app = QApplication.instance() or QApplication([])
    from g1pilot.teleoperation import demo_gui as dg
    node = dg.DemoNode()
    shared_log = []
    orig = node.pub_shared.publish
    node.pub_shared.publish = lambda m: (shared_log.append(json.loads(m.data)), orig(m))
    g = dg.DemoGUI(node)
    bools = []
    orig_b = node.publish_bool
    node.publish_bool = lambda pub, v: (bools.append((pub.topic_name, v)), orig_b(pub, v))
    g.autostart.cancel("Test")                      # Sim-Auto-Start stoert hier nur
    try:
        yield request.param, dg, node, g, shared_log, bools
    finally:
        g.close()
        node.tf_listener.executor.shutdown()
        node.tf_listener.dedicated_listener_thread.join(timeout=2.0)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def controller(stamp=None, source="ps4_arm_teleop", **fields):
    t = time.time() if stamp is None else stamp
    return String(data=json.dumps(
        {"source": source, "fields": {k: {"v": v, "t": t} for k, v in fields.items()}}))


def test_gui_defaults_are_only_a_proposal(gui):
    _, dg, node, g, shared_log, _ = gui
    st = node.shared
    assert st.get("arm_side") == "right" and st.get("arm_speed") == "Normal"
    assert st.get("walk_speed") == "Langsam"
    assert all(st.stamps[f] == ss.DEFAULT_STAMP for f in ("arm_side", "arm_speed", "walk_speed"))
    # Controller-Start (real: Langsam) gewinnt -> GUI zeigt Langsam
    node._on_shared(controller(arm_side="right", arm_speed="Langsam"))
    jog = g.manip_panel.jog
    assert jog.speed_name == "Langsam" and jog.speed_btns["Langsam"].isChecked()
    assert jog.lin_speed == dg.JOG_SPEEDS["Langsam"][0]


def test_controller_side_speed_follow_without_sending(gui):
    _, dg, node, g, shared_log, bools = gui
    jog = g.manip_panel.jog
    n_shared = len(shared_log)
    node._on_shared(controller(arm_side="mirror", arm_speed="Schnell"))
    node._on_shared(controller(source="ps4_walk", walk_speed="Schnell"))
    assert jog.side == "Gespiegelt" and jog.side_btns["Gespiegelt"].isChecked()
    assert jog.speed_name == "Schnell"
    wp = g.walk_panel
    assert wp.speed_btns["Schnell"].isChecked() and not wp.speed_btns["Schnell"].isEnabled()
    assert wp.speed == dg.SPEEDS["Normal"]          # GUI selbst geht hoechstens Normal
    assert bools == [] and len(shared_log) == n_shared   # nichts gesendet, kein Echo
    # Gespiegelt: linke Hand spiegelbildlich (wie Steuerkreuz ▼ am Controller)
    lin, ang = (0.1, 0.05, 0.02), (0.1, 0.2, 0.3)
    assert jog.side_twist("right", lin, ang) == (lin, ang)
    assert jog.side_twist("left", lin, ang) == ((0.1, -0.05, 0.02), (-0.1, 0.2, -0.3))
    assert jog.sides() == ("left", "right")


def test_gui_choices_go_to_shared_state(gui):
    _, dg, node, g, shared_log, _ = gui
    jog = g.manip_panel.jog
    t0 = time.time()
    jog.side_btns["Links"].click()
    jog.speed_btns["Langsam"].click()
    g.walk_panel.speed_btns["Normal"].click()
    last = shared_log[-1]["fields"]
    assert last["arm_side"]["v"] == "left" and last["arm_side"]["t"] >= t0
    assert last["arm_speed"]["v"] == "Langsam" and last["walk_speed"]["v"] == "Normal"
    assert last["walk_speed"]["t"] >= t0


def test_controller_mode_switch_moves_tiles(gui):
    which, dg, node, g, shared_log, bools = gui
    if which == "real":
        node._on_shared(controller(mode="walk"))     # vor »Roboter starten«: nur Hinweis
        assert g.mode is None and g.pending is None
        assert "Roboter starten" in g.status.text()
        g._start_robot()
        bools.clear()
    # L2 am Controller: Kachel GEHEN »wechselt«, bis die Arme eingeklappt sind
    node._on_shared(controller(mode="walk"))
    assert g.pending == dg.WALK and g.tiles[dg.WALK].text().endswith("wechselt …")
    node.on_walk_ready(True)
    assert g.mode == dg.WALK and g.stack.currentWidget() is g.walk_panel
    # R2 am Controller: GREIFEN
    node._on_shared(controller(mode="grasp"))
    assert g.mode == dg.MANIP and g.stack.currentWidget() is g.manip_panel
    # GUI hat dabei NICHTS an den Roboter gesendet (das tat der Controller)
    assert not any(t in ("/g1pilot/start_walking", "/g1pilot/start_balancing",
                         "/g1pilot/arms/enabled") for t, _ in bools)
    # Alter latched Modus (vor dem GUI-Start) wird ignoriert
    node._on_shared(controller(stamp=node.t_start_wall - 10.0, mode="walk"))
    assert g.mode == dg.MANIP and g.pending is None


def test_gui_tile_reports_mode_and_estop_then_controller_again(gui):
    which, dg, node, g, shared_log, bools = gui
    if which == "real":
        g._start_robot()
        node.onboard = {"fsm": 501}
        node.onboard_t = time.monotonic()
    g.request_mode(dg.WALK)
    assert shared_log[-1]["fields"]["mode"]["v"] == "walk"
    node.on_walk_ready(True)
    g.request_mode(dg.MANIP)
    assert shared_log[-1]["fields"]["mode"]["v"] == "grasp"
    # NOT-HALT: GUI ohne Modus; danach waehlt der Controller wieder GREIFEN
    node._emergency_stop(Bool(data=True))
    assert g.mode is None
    if which == "real":
        g._start_robot()
    else:
        node.publish_bool(node.pub_start, True)
        g._estop_unacked = False
    node._on_shared(controller(mode="grasp"))       # gleicher Wert, neue Wahl
    assert g.mode == dg.MANIP


def test_real_walk_tile_blocked_does_not_share(gui):
    which, dg, node, g, shared_log, _ = gui
    if which != "real":
        pytest.skip("nur real: GEHEN ohne Walk-Modus gesperrt")
    g._start_robot()
    n = len(shared_log)
    g.request_mode(dg.WALK)                          # kein onboard_state -> gesperrt
    assert g.pending is None and len(shared_log) == n


def test_info_line_shows_controller_walk_and_kind(gui):
    _, dg, node, g, _, _ = gui
    now = time.monotonic()
    node.ps4 = {"connected": True, "selection": "left", "mode": "move", "speed": "Normal",
                "speed_req": "Normal"}
    node.ps4_t = now
    node.ps4dev = {"kind": "xbox", "name": "Xbox Wireless Controller"}
    node.ps4walk = {"active": True, "speed": "Langsam", "walk_ready": True, "estop": False}
    g._refresh_info()
    assert g.info_ps4.text().startswith("Xbox: GEHT (L2 gehalten) · Geh-Tempo Langsam")
    assert "Xbox Wireless Controller" in g.info_ps4.toolTip()
    node.ps4walk = dict(node.ps4walk, active=False)
    g._refresh_info()
    assert g.info_ps4.text().startswith("Xbox: Hand LINKS")
    g._ps4_help()                                   # darf nicht werfen

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Start-GUI (g1_gui.py): Navigation mit Totmann bzw. LiDAR direkt aus dem
Startmenue, ohne Umgebungsvariablen von Hand.

Prueft die an start.sh uebergebene Umgebung: Sim-Navigation setzt
G1_NAV_DEADMAN und erzwingt den Controller (ps4_walk liefert den Totmann),
Real-LiDAR erzwingt den Controller und reicht G1_LIDAR_IP durch.
"""
import importlib.util
import os

import pytest

tk = pytest.importorskip("tkinter")

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture
def gui(tmp_path):
    spec = importlib.util.spec_from_file_location("g1_gui", os.path.join(REPO, "g1_gui.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.SETTINGS_FILE = tmp_path / "settings.json"
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("kein Display")
    started = []

    class App:
        def show_menu(self):
            pass

        def start_process(self, title, argv, *, cwd, env=None, **kw):
            started.append(env)

    yield mod, root, App(), started
    root.destroy()


def test_sim_nav_deadman_forces_controller(gui):
    mod, root, app, started = gui
    f = mod.SimFrame(root, app)
    f.v_ps4.set(False)
    f.v_nav.set(True)
    root.update()
    assert f.v_nav_deadman.get() is True        # Default: wie real
    assert f.v_ps4.get() is True                 # Controller erzwungen
    f._start()
    env = started[-1]
    assert env["G1_ENABLE_NAV"] == "1" and env["G1_NAV_DEADMAN"] == "1"
    assert env["G1_PS4_ARMS"] == "1"
    # gemerkt wird die eigene Controller-Wahl, nicht die erzwungene
    assert mod.load_settings("sim")["ps4"] is False


def test_sim_without_deadman_or_nav(gui):
    mod, root, app, started = gui
    f = mod.SimFrame(root, app)
    f.v_ps4.set(False)
    f.v_nav.set(True)
    f.v_nav_deadman.set(False)
    root.update()
    assert f.v_ps4.get() is False                # eigene Wahl zurueck
    f._start()
    assert started[-1]["G1_NAV_DEADMAN"] == "0" and started[-1]["G1_PS4_ARMS"] == "0"
    f.v_nav_deadman.set(True)
    f.v_nav.set(False)
    root.update()
    f._start()
    assert started[-1]["G1_ENABLE_NAV"] == "0" and started[-1]["G1_NAV_DEADMAN"] == "0"


def test_real_lidar_forces_controller_and_ip(gui):
    mod, root, app, started = gui
    f = mod.RealFrame(root, app)
    f.v_iface.set("eth0")
    f.v_ps4.set(False)
    f.v_lidar.set(True)
    f.v_lidar_ip.set("192.168.123.121")
    f.v_confirm.set(True)
    root.update()
    assert f.v_ps4.get() is True
    f._start()
    env = started[-1]
    assert env["G1_ENABLE_LIDAR"] == "1" and env["G1_LIDAR_IP"] == "192.168.123.121"
    assert env["G1_PS4_ARMS"] == "1"
    assert mod.load_settings("real")["ps4"] is False
    f.v_lidar.set(False)
    root.update()
    assert f.v_ps4.get() is False

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav_supervisor: jeder Waechter einzeln, mit Test-Uhr (ohne ROS).

Stufe 1 der LiDAR-Navigation: AUTO NAV laeuft nur, solange alle Waechter
gruen sind; jeder Ausfall -> genau ein Stop-Joy, Freigabe weg, Grund.
"""
import math

import pytest

from g1pilot.navigation import nav_supervisor as ns
from g1pilot.navigation.nav_supervisor import (
    SupervisorLogic, GridCheck, cap_axes, path_distance,
    OFF, QUEUED, RUNNING, PAUSED, STOPPED)

DT = 0.02   # 50 Hz


def raw(vx=-0.5, vy=0.0, yaw=0.0, deadman=1):
    axes = [0.0] * 8
    axes[ns.AX_VX], axes[ns.AX_VY], axes[ns.AX_YAW] = vx, vy, yaw
    buttons = [0] * 14
    buttons[ns.BTN_DEADMAN] = deadman
    return axes, buttons


class Rig:
    """Supervisor + frische Eingaenge; feed() haelt alles am Leben."""

    def __init__(self, **params):
        params.setdefault("require_deadman", False)
        self.s = SupervisorLogic(**params)
        self.t = 0.0
        self.x = 0.0
        self.deadman = (True, True)
        self.alive = {"pose", "heartbeat", "raw", "deadman"}
        self.raw = raw()
        self.s.on_walk_ready(True, self.t)
        self.s.on_goal(3.0, 0.0, None, self.t)
        self.s.on_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)], self.t)
        self.stops = 0
        self.outs = []

    def feed(self):
        s, t = self.s, self.t
        if "pose" in self.alive:
            s.on_pose(self.x, 0.0, 0.0, t)
        if "heartbeat" in self.alive:
            s.on_heartbeat(t)
        if "raw" in self.alive:
            s.on_raw_joy(*self.raw, t)
        if "deadman" in self.alive:
            s.on_deadman(*self.deadman, t)

    def step(self, n=1, move=0.0):
        out = None
        for _ in range(n):
            self.t += DT
            self.x += move
            self.feed()
            out, stop = self.s.tick(self.t)
            self.stops += int(stop)
            self.outs.append(out)
        return out

    def start(self):
        self.feed()
        self.s.on_request(True, self.t)
        return self.step()


# ── Grundablauf ────────────────────────────────────────────────────────
def test_off_passes_nothing():
    r = Rig()
    r.feed()
    assert r.step(5) is None
    assert r.s.state == OFF


def test_without_deadman_runs_and_passes_joy():
    r = Rig()
    out = r.start()
    assert r.s.state == RUNNING
    axes, buttons = out
    assert axes[ns.AX_VX] == pytest.approx(-0.5) and buttons[ns.BTN_DEADMAN] == 1


def test_waits_for_path():
    r = Rig()
    r.s.on_path([], r.t)
    r.start()
    assert r.s.state == QUEUED and r.step() is None


def test_request_off_sends_one_stop():
    r = Rig()
    r.start()
    r.s.on_request(False, r.t)
    assert r.step() is None
    assert r.s.state == OFF and r.stops == 1


# ── Totmann (L2 + Stick) ───────────────────────────────────────────────
def test_deadman_needs_l2_and_stick_to_start():
    r = Rig(require_deadman=True)
    r.deadman = (False, False)
    r.start()
    assert r.s.state == QUEUED and r.s.reason == "wait_go"
    r.deadman = (True, False)          # L2 allein reicht nicht
    assert r.step(5) is None and r.s.state == QUEUED
    r.deadman = (True, True)           # L2 + Stick -> los
    assert r.step() is not None and r.s.state == RUNNING
    r.deadman = (True, False)          # danach reicht L2 halten
    assert r.step(10) is not None and r.s.state == RUNNING


def test_deadman_release_pauses_and_resumes():
    r = Rig(require_deadman=True)
    r.start()
    assert r.s.state == RUNNING
    r.deadman = (False, False)
    assert r.step() is None
    assert r.s.state == PAUSED and r.stops == 1
    r.deadman = (True, False)
    assert r.step(3) is None and r.s.state == PAUSED
    r.deadman = (True, True)
    assert r.step() is not None and r.s.state == RUNNING


def test_controller_radio_loss_stops_within_timeout():
    r = Rig(require_deadman=True)
    r.start()
    r.alive.discard("deadman")         # ps4_walk meldet nichts mehr (Funk weg)
    t0 = r.t
    while r.s.state == RUNNING:
        r.step()
    assert r.s.state == PAUSED and r.s.reason in ("deadman_silent", "wait_go")
    assert r.t - t0 <= 0.3 + 2 * DT and r.stops == 1


# ── Waechter, die sofort stoppen ───────────────────────────────────────
@pytest.mark.parametrize("dead,reason,limit", [
    ("heartbeat", "heartbeat", 1.0),
    ("pose", "pose_timeout", 0.3),
])
def test_silent_input_stops(dead, reason, limit):
    r = Rig()
    r.start()
    r.alive.discard(dead)
    t0 = r.t
    while r.s.state == RUNNING:
        r.step()
    assert r.s.state == STOPPED and r.s.reason == reason
    assert r.t - t0 <= limit + 2 * DT
    assert r.stops == 1


def test_follower_silent_stops():
    r = Rig()
    r.start()
    r.alive.discard("raw")
    while r.s.state == RUNNING:
        r.step()
    assert r.s.reason == "follower_silent" and r.stops == 1


def test_follower_stop_then_silence_is_no_fault():
    r = Rig()
    r.start()
    r.raw = raw(0.0, deadman=0)        # nav2point am Ziel: Stop-Joy, dann Ruhe
    r.step()
    r.alive.discard("raw")
    r.s.on_nav_status("arrived", r.t)
    r.step(30)
    assert r.s.state == QUEUED and r.s.reason == "arrived"


def test_pose_jump_stops():
    r = Rig()
    r.start()
    r.x += 0.5
    r.step()
    assert r.s.state == STOPPED and r.s.reason == "pose_jump"


def test_estop_stops_and_needs_new_request():
    r = Rig()
    r.start()
    r.s.on_estop(True, r.t)
    assert r.step() is None and r.s.reason == "estop" and r.stops == 1
    r.step(10)
    assert r.s.state == STOPPED            # bleibt stehen
    r.s.on_request(False, r.t)
    r.s.on_request(True, r.t)
    assert r.step() is not None and r.s.state == RUNNING


def test_not_walking_blocks_start_and_stops():
    r = Rig()
    r.s.on_walk_ready(False, r.t)
    r.start()
    assert r.s.state == QUEUED and r.s.reason == "not_walking"
    r.s.on_walk_ready(True, r.t)
    r.step()
    assert r.s.state == RUNNING
    r.s.on_walk_ready(False, r.t)
    r.step()
    assert r.s.state == STOPPED and r.s.reason == "not_walking"


def test_unknown_walk_state_blocks():
    r = Rig()
    r.s.on_walk_ready(None, r.t)
    r.start()
    assert r.s.state == QUEUED and r.s.reason == "not_walking"


def test_path_deviation_stops():
    r = Rig()
    r.start()
    for y in (0.1, 0.2, 0.29):
        r.s.on_pose(r.x, y, 0.0, r.t)
        r.s.tick(r.t)
        assert r.s.state == RUNNING
    r.alive.discard("pose")
    r.t += DT
    r.s.on_pose(r.x, 0.35, 0.0, r.t)
    r.s.tick(r.t)
    assert r.s.reason == "deviation"


def test_no_progress_stops():
    r = Rig(progress_timeout_s=2.0)
    r.start()
    r.step(int(1.5 / DT), move=0.01)       # laeuft
    assert r.s.state == RUNNING
    r.step(int(2.5 / DT))                  # steht
    assert r.s.state == STOPPED and r.s.reason == "no_progress"


def test_goal_too_far_blocks():
    r = Rig(max_goal_distance_m=2.0)
    r.start()
    assert r.s.state == QUEUED and r.s.reason == "goal_far"


def test_tilt_stops():
    r = Rig()
    r.start()
    r.s.on_imu(math.radians(20), 0.0, r.t)
    r.step()
    assert r.s.reason == "tilt"


def test_map_checks():
    grid = GridCheck(10, 10, 0.5, -1.0, -1.0, [0] * 100)
    r = Rig()
    r.s.on_map(grid)
    r.start()
    assert r.s.state == RUNNING
    # Ziel ausserhalb der Karte (unbekannt) -> sperrt
    r2 = Rig()
    r2.s.on_map(GridCheck(4, 4, 0.5, -1.0, -1.0, [0] * 16))
    r2.start()
    assert r2.s.reason == "goal_blocked"
    # Roboter auf belegter Zelle
    data = [0] * 100
    data[2 * 10 + 2] = 100               # Zelle um (0, 0)
    r3 = Rig()
    r3.s.on_map(GridCheck(10, 10, 0.5, -1.0, -1.0, data))
    r3.start()
    assert r3.s.reason == "off_map"


def test_loc_quality_and_obstacle_inputs():
    r = Rig(require_loc_quality=True, require_obstacle_field=True)
    r.start()
    assert r.s.state == QUEUED and r.s.reason == "loc_quality"
    r.s.on_loc_quality(0.9, r.t)
    r.step()
    assert r.s.reason == "obstacle_silent"
    r.s.on_loc_quality(0.9, r.t)
    r.s.on_obstacle(False, r.t)
    r.step()
    assert r.s.state == RUNNING
    r.s.on_loc_quality(0.9, r.t)
    r.s.on_obstacle(True, r.t)
    r.step()
    assert r.s.state == STOPPED and r.s.reason == "obstacle"


def test_new_path_while_running_needs_new_go():
    r = Rig(require_deadman=True)
    r.start()
    r.deadman = (True, False)
    r.s.on_path([(0.0, 0.0), (0.0, 2.0)], r.t)
    assert r.step() is None and r.s.state == QUEUED and r.stops == 1


def test_speed_cap_applied():
    r = Rig(axis_cap_lin=0.3, axis_cap_yaw=0.5)
    r.raw = raw(vx=-0.8, vy=-0.6, yaw=0.9)
    axes, _ = r.start()
    assert math.hypot(axes[ns.AX_VX], axes[ns.AX_VY]) == pytest.approx(0.3)
    assert axes[ns.AX_VX] / axes[ns.AX_VY] == pytest.approx(0.8 / 0.6)
    assert axes[ns.AX_YAW] == pytest.approx(0.5)


# ── Hilfsfunktionen ────────────────────────────────────────────────────
def test_cap_axes_no_cap_is_identity():
    a = raw(-0.4, 0.2, 0.1)[0]
    assert cap_axes(a, 1.0, 1.0) == a


def test_path_distance():
    assert path_distance([(0, 0), (2, 0)], 1.0, 0.5) == pytest.approx(0.5)
    assert path_distance([(0, 0), (2, 0)], 3.0, 0.0) == pytest.approx(1.0)
    assert path_distance([], 0, 0) is None


def test_unknown_parameter_rejected():
    with pytest.raises(ValueError):
        SupervisorLogic(require_dedman=True)


def test_status_text():
    r = Rig(require_deadman=True)
    r.deadman = (False, False)
    r.start()
    st = r.s.status()
    assert st["state"] == QUEUED and "L2" in st["text"] and st["require_deadman"]

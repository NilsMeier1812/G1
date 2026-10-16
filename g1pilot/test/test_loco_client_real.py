#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""loco_client (echter G1) gegen einen Fake-LocoClient -- ohne Hardware.

Real-Test 2026-10-15 (Nils):
  * NOT-HALT im Walk-Modus: der G1 verlor das Gleichgewicht und rannte los.
    -> E-Stop sendet IMMER StopMove, kein Damp (estop_damp_legs=false).
  * Umschalten in der GUI aenderte den echten Modus nicht, und nach dem
    Umstellen per Handy-App liess sich der G1 nicht ueber die GUI bewegen
    (balanced blieb False). -> Abgleich mit der Onboard-FSM, LAUFEN stellt
    von Preparation auf Walk um.
Braucht rclpy; unitree_sdk2py ist durch Attrappen ersetzt.
"""
import os
import sys
import time
import types

import pytest

rclpy = pytest.importorskip("rclpy")

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PKG not in sys.path:
    sys.path.insert(0, PKG)


class FakeLoco:
    """Zeichnet die RPCs auf; FSM-Zustand wie beim G1 (4 Ready, 500 Walk, 1 Damp)."""
    last = None

    def __init__(self):
        FakeLoco.last = self
        self.fsm = 4
        self.mode = 0
        self.calls = []
        self.vel = (0.0, 0.0, 0.0)
        self.vel_until = 0.0
        self.fail = set()

    def SetTimeout(self, t):
        pass

    def Init(self):
        pass

    def _rec(self, name, *a):
        self.calls.append((name,) + a)
        if name in self.fail:
            raise RuntimeError("rpc down")

    def _Call(self, api_id, param):
        if api_id == "GET_FSM_ID":
            return 0, '{"data": %d}' % self.fsm
        return 0, '{"data": %d}' % self.mode

    def SetFsmId(self, i):
        self._rec("SetFsmId", i)
        self.fsm = i
        return 0

    def SetVelocity(self, vx, vy, w, duration=1.0):
        self._rec("SetVelocity", vx, vy, w, duration)
        self.vel = (vx, vy, w)
        self.vel_until = time.monotonic() + duration
        return 0

    def StopMove(self):
        self.SetVelocity(0.0, 0.0, 0.0)

    def SetStandHeight(self, h):
        self._rec("SetStandHeight", h)

    def BalanceStand(self, m):
        self._rec("BalanceStand", m)

    def Start(self):
        self._rec("Start")
        self.fsm = 500

    def Damp(self):
        self.SetFsmId(1)

    def names(self):
        return [c[0] for c in self.calls]


@pytest.fixture(scope="module")
def lc_module():
    stubs = {}
    for name in ("unitree_sdk2py", "unitree_sdk2py.g1", "unitree_sdk2py.g1.loco",
                 "unitree_sdk2py.g1.loco.g1_loco_client",
                 "unitree_sdk2py.g1.loco.g1_loco_api"):
        stubs[name] = types.ModuleType(name)
    stubs["unitree_sdk2py.g1.loco.g1_loco_client"].LocoClient = FakeLoco
    api = stubs["unitree_sdk2py.g1.loco.g1_loco_api"]
    api.ROBOT_API_ID_LOCO_GET_FSM_ID = "GET_FSM_ID"
    api.ROBOT_API_ID_LOCO_GET_FSM_MODE = "GET_FSM_MODE"
    saved = {k: sys.modules.get(k) for k in stubs}
    sys.modules.update(stubs)
    import g1pilot.utils.common as common
    old_init = common.init_dds
    common.init_dds = lambda *a, **k: None
    import importlib
    import g1pilot.navigation.loco_client as lc
    importlib.reload(lc)
    yield lc
    common.init_dds = old_init
    for k, v in saved.items():
        if v is None:
            sys.modules.pop(k, None)
        else:
            sys.modules[k] = v


def _make(lc, **params):
    args = ["--ros-args"] + sum((["-p", f"{k}:={v}"] for k, v in params.items()), [])
    rclpy.init(args=args)
    node = lc.G1LocoClient()
    return node, FakeLoco.last


def _close(node):
    try:
        node.destroy_node()
    finally:
        rclpy.shutdown()


def _bool(v):
    from std_msgs.msg import Bool
    return Bool(data=v)


def _joy(buttons=None, axes=None):
    from sensor_msgs.msg import Joy
    j = Joy()
    j.buttons = [0] * 14
    j.axes = [0.0] * 8
    for i, v in (buttons or {}).items():
        j.buttons[i] = v
    for i, v in (axes or {}).items():
        j.axes[i] = v
    return j


def _cmd(vx=0.0, vy=0.0, wz=0.0):
    from geometry_msgs.msg import Twist
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
    return t


# ── NOT-HALT ───────────────────────────────────────────────────────────

def test_estop_stops_move_without_damp_by_default(lc_module):
    """Walk laeuft (Move aktiv), E-Stop: StopMove, KEIN Damp -> die Beine
    bleiben unter der Onboard-Regelung, das Laufen endet."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node._sync_onboard_state(force=True)
        assert node.balanced
        node.start_walking_callback(_bool(True))
        node.cmd_vel_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()
        assert robot.vel[0] > 0.3 and node._move_active
        robot.calls.clear()
        node.emergency_callback(_bool(True))
        assert robot.vel == (0.0, 0.0, 0.0)
        assert ("SetFsmId", 1) not in robot.calls, "kein Damp im Default"
        assert robot.fsm == 500
        assert node.robot_stopped and not node.balanced and not node.walk_enabled
        # Danach keine Bewegung mehr, auch wenn die GUI weiter Kommandos sendet.
        node.cmd_vel_callback(_cmd(vx=1.0))
        robot.calls.clear()
        node._cmd_vel_tick()
        assert "SetVelocity" not in robot.names()
    finally:
        _close(node)


def test_estop_stopmove_even_if_move_flag_was_cleared(lc_module):
    """Frueher: _move_active=False vor Damp -> kein StopMove, das zuletzt
    kommandierte Move() lief weiter. Jetzt sendet der E-Stop StopMove immer."""
    node, robot = _make(lc_module)
    try:
        node._move_active = False
        robot.vel = (0.4, 0.0, 0.0)
        node.emergency_callback(_bool(True))
        assert robot.vel == (0.0, 0.0, 0.0)
    finally:
        _close(node)


def test_estop_damp_legs_optional(lc_module):
    node, robot = _make(lc_module, estop_damp_legs=True)
    try:
        robot.fsm = 500
        node.emergency_callback(_bool(True))
        assert robot.names()[-2:] == ["SetVelocity", "SetFsmId"]
        assert robot.fsm == 1
    finally:
        _close(node)


def test_estop_rpc_error_does_not_raise(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fail = {"SetVelocity"}
        node.emergency_callback(_bool(True))     # darf nicht werfen
        assert node.robot_stopped
    finally:
        _close(node)


def test_estop_not_blocked_by_busy_rpc_lock(lc_module):
    """Ein haengender RPC (Lock gehalten) blockiert den E-Stop nur 0.5 s."""
    node, robot = _make(lc_module)
    try:
        node._rpc_lock.acquire()
        t0 = time.monotonic()
        node.emergency_callback(_bool(True))
        dt = time.monotonic() - t0
        node._rpc_lock.release()
        assert dt < 1.5
        assert robot.vel == (0.0, 0.0, 0.0)
    finally:
        _close(node)


def test_ps4_estop_button_same_as_gui(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.joystick_callback(_joy())
        node.joystick_callback(_joy(buttons={5: 1}))
        assert robot.vel == (0.0, 0.0, 0.0) and ("SetFsmId", 1) not in robot.calls
        assert node.robot_stopped
    finally:
        _close(node)


def test_start_after_estop_keeps_onboard_mode(lc_module):
    """Quittieren nach E-Stop ohne Damp: kein Wechsel nach FSM 4, G1 bleibt
    im Walk-Zustand und wird wieder als balanciert uebernommen."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.emergency_callback(_bool(True))
        robot.calls.clear()
        node.start_callback(_bool(True))
        assert ("SetFsmId", 4) not in robot.calls and robot.fsm == 500
        assert not node.robot_stopped and node.balanced
    finally:
        _close(node)


def test_start_after_damp_estop_goes_to_standby(lc_module):
    node, robot = _make(lc_module, estop_damp_legs=True)
    try:
        robot.fsm = 500
        node.emergency_callback(_bool(True))
        node.start_callback(_bool(True))
        assert robot.fsm == 4 and not node.balanced
    finally:
        _close(node)


# ── Abgleich mit dem Onboard-Zustand / GUI-Umschalten ──────────────────

def test_walk_set_by_phone_app_is_adopted_and_gui_can_move(lc_module):
    """Befund: Handy-App stellt Walk ein, die GUI sagt LAUFEN -- bisher
    'balanced'=False -> WALK ignoriert, G1 nicht ueber die GUI bewegbar."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500                      # per App eingestellt
        node.start_walking_callback(_bool(True))   # GUI LAUFEN, kein Poll dazwischen
        assert node.balanced and node.walk_enabled
        assert "Start" not in robot.names(), "kein erneutes Umschalten noetig"
        node.cmd_vel_callback(_cmd(vx=0.5, wz=-0.5))
        node._cmd_vel_tick()
        assert robot.vel == pytest.approx((0.2, 0.0, -0.2))
        # Move hat eine BEGRENZTE Dauer (Onboard-Deadman), keine 864000 s.
        assert robot.calls[-1][-1] == pytest.approx(1.0)
    finally:
        _close(node)


def test_laufen_in_gui_switches_real_mode_from_preparation(lc_module):
    """LAUFEN aus Preparation (FSM 4): der Node stellt den echten G1 auf Walk
    (Hoehenrampe, BalanceStand, Start) und gibt dann das Gehen frei."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 4
        node.start_walking_callback(_bool(True))
        assert robot.fsm == 500
        assert robot.names().count("SetStandHeight") >= 2
        assert "BalanceStand" in robot.names() and "Start" in robot.names()
        assert node.balanced and node.walk_enabled
    finally:
        _close(node)


@pytest.mark.parametrize("fsm", [0, 1, 2, 3, 702])
def test_laufen_never_stands_up_from_damp_or_sitting(lc_module, fsm):
    node, robot = _make(lc_module)
    try:
        robot.fsm = fsm
        node._sync_onboard_state(force=True)
        node.start_walking_callback(_bool(True))
        assert robot.fsm == fsm and not node.walk_enabled and not node.balanced
        assert "Start" not in robot.names()
    finally:
        _close(node)


def test_laufen_ignored_during_estop_latch(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.emergency_callback(_bool(True))
        robot.calls.clear()
        node.start_walking_callback(_bool(True))
        assert not node.walk_enabled and "Start" not in robot.names()
    finally:
        _close(node)


def test_leaving_walk_state_blocks_move_debounced(lc_module):
    """Fernbedienung dampft den G1: Gehen wird gesperrt (erst nach Schonfrist
    und 2 Polls, damit ein Start() nicht sofort zurueckschaltet)."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node._sync_onboard_state(force=True)
        node.start_walking_callback(_bool(True))
        assert node.walk_enabled
        robot.fsm = 1
        node._sync_onboard_state(force=True)
        node._sync_onboard_state(force=True)
        assert node.balanced, "Schonfrist nach dem Umschalten"
        node._balanced_at -= 10.0
        node._sync_onboard_state(force=True)
        assert not node.balanced and not node.walk_enabled
        node.cmd_vel_callback(_cmd(vx=1.0))
        robot.calls.clear()
        node._cmd_vel_tick()
        assert "SetVelocity" not in robot.names()
    finally:
        _close(node)


def test_sync_reads_none_does_not_change_state(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node._sync_onboard_state(force=True)
        robot._Call = lambda *a: (1, None)
        node._balanced_at -= 10.0
        for _ in range(3):
            node._sync_onboard_state(force=True)
        assert node.balanced
    finally:
        _close(node)


# ── Joy-/Nav-Pfad: Deadman bei ausbleibendem Strom ───────────────────────

def test_joy_stream_timeout_stops_move(lc_module):
    """Nav/PS4 faehrt ueber /g1pilot/joy. Stirbt der Joy-Strom mit gedruecktem
    Deadman (nav2point tot, Bluetooth weg), lief frueher das letzte Move()
    weiter. Jetzt: Timeout -> StopMove."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node._sync_onboard_state(force=True)
        node.joystick_callback(_joy(buttons={8: 1}, axes={1: -1.0}))
        assert robot.vel[0] > 0.0 and node._move_active
        node._cmd_vel_tick()
        assert robot.vel[0] > 0.0, "Strom frisch -> weiter fahren"
        node._joy_stamp -= 5.0
        node._cmd_vel_tick()
        assert robot.vel == (0.0, 0.0, 0.0) and not node._move_active
    finally:
        _close(node)


def test_streamdeck_deadman_still_stops(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.start_walking_callback(_bool(True))
        node.cmd_vel_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()
        assert robot.vel[0] > 0.0
        node._cmd_stamp -= 5.0
        node._cmd_vel_tick()
        assert robot.vel == (0.0, 0.0, 0.0)
    finally:
        _close(node)

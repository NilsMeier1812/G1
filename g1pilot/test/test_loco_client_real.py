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
        # Wie das echte LocoClient: Dienstname zur Erzeugungszeit aus dem Modul.
        self.service = sys.modules["unitree_sdk2py.g1.loco.g1_loco_client"].LOCO_SERVICE_NAME
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
        if i in (500, 501):
            self.calls.append(("Start",))    # Walk-Start fuer die Pruefungen
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
    # Wie das im Real-Image installierte SDK (lnotspotl 7c661d2): alter Name.
    stubs["unitree_sdk2py.g1.loco.g1_loco_client"].LOCO_SERVICE_NAME = "loco"
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


# ── VOLL-NOTAUS (PS4 R1 + R2 + PS) ─────────────────────────────────────

def test_full_estop_damps_whole_robot_even_without_estop_damp_legs(lc_module):
    """Voll-Notaus: Damp (FSM 1) ZUERST, unabhaengig von estop_damp_legs=false,
    danach StopMove; Gehen gesperrt, Zustand fuer die GUI."""
    node, robot = _make(lc_module)
    try:
        assert node.get_parameter("estop_damp_legs").value is False
        robot.fsm = 500
        node._sync_onboard_state(force=True)
        node.start_walking_callback(_bool(True))
        node.cmd_vel_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()
        robot.calls.clear()
        node.full_emergency_callback(_bool(True))
        assert robot.calls[0] == ("SetFsmId", 1), "Damp zuerst"
        assert robot.fsm == 1 and robot.vel == (0.0, 0.0, 0.0)
        assert node.robot_stopped and not node.balanced and not node.walk_enabled
        assert node._full_estop and "VOLL-NOTAUS" in node._note
        node.cmd_vel_callback(_cmd(vx=1.0))
        robot.calls.clear()
        node._cmd_vel_tick()
        assert "SetVelocity" not in robot.names()
        # False auf dem Topic tut nichts
        node.full_emergency_callback(_bool(False))
        assert node._full_estop
    finally:
        _close(node)


def test_full_estop_after_ps_estop_still_damps(lc_module):
    """Ablauf am Controller: PS (NOT-HALT, Beine bleiben stehen), 2 s spaeter
    Voll-Notaus -> jetzt doch Damp."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.emergency_callback(_bool(True))
        assert robot.fsm == 500
        node.full_emergency_callback(_bool(True))
        assert robot.fsm == 1 and not node._estop_legs_kept
        # Ein spaeter eintreffender NOT-HALT macht den Voll-Notaus nicht weicher.
        node.emergency_callback(_bool(True))
        assert robot.fsm == 1 and node._full_estop
    finally:
        _close(node)


def test_full_estop_not_blocked_by_busy_rpc_lock_or_rpc_error(lc_module):
    node, robot = _make(lc_module)
    try:
        node._rpc_lock.acquire()
        t0 = time.monotonic()
        node.full_emergency_callback(_bool(True))
        dt = time.monotonic() - t0
        node._rpc_lock.release()
        assert dt < 2.0 and robot.fsm == 1
        robot.fail = {"SetFsmId", "SetVelocity"}
        node.full_emergency_callback(_bool(True))      # darf nicht werfen
        assert node.robot_stopped
    finally:
        _close(node)


def test_start_after_full_estop_only_acknowledges(lc_module):
    """START nach Voll-Notaus: quittiert, aber KEIN Aufstehen (kein FSM 4) --
    anders als nach einem NOT-HALT mit estop_damp_legs. Zurueck geht es
    ueber die Startleiste: Preparation (aus Damp erlaubt), dann Walk."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.full_emergency_callback(_bool(True))
        # Vor dem Quittieren: Preparation abgelehnt
        from std_msgs.msg import String
        node.mode_request_callback(String(data="preparation"))
        assert robot.fsm == 1
        robot.calls.clear()
        node.start_callback(_bool(True))
        assert ("SetFsmId", 4) not in robot.calls and robot.fsm == 1
        assert not node.robot_stopped and not node._full_estop and not node.balanced
        assert "quittiert" in node._note
        # Startleiste: Preparation -> FSM 4
        node.mode_request_callback(String(data="preparation"))
        assert robot.fsm == 4
        # Ein normaler START danach verhaelt sich wieder wie immer.
        node.start_callback(_bool(True))
        assert robot.fsm == 4
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
        assert robot.fsm == 501
        # Default seit 2026-10-16: wie die Handy-App, ohne Hoehenrampe.
        assert "SetStandHeight" not in robot.names()
        assert "BalanceStand" in robot.names() and "Start" in robot.names()
        assert node.balanced and node.walk_enabled
    finally:
        _close(node)


def test_walk_height_ramp_parameter_keeps_old_ramp(lc_module):
    node, robot = _make(lc_module, walk_height_ramp=True)
    try:
        robot.fsm = 4
        node.start_walking_callback(_bool(True))
        assert robot.names().count("SetStandHeight") >= 2 and node.balanced
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


# ── START / Aufstellen nach Start per Handy-App (Review 2026-10-16) ─────

def test_start_keeps_walk_set_by_phone_app(lc_module):
    """Befund: G1 per Handy auf Walk, dann »Roboter starten« in der GUI ->
    frueher SetFsmId(4): Sprung aus dem balancierenden Walk nach Preparation."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.start_callback(_bool(True))
        assert ("SetFsmId", 4) not in robot.calls and robot.fsm == 500
        assert node.balanced and not node.robot_stopped and not node.walk_enabled
    finally:
        _close(node)


def test_start_in_preparation_sends_nothing(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 4
        node.start_callback(_bool(True))
        assert "SetFsmId" not in robot.names() and not node.balanced
    finally:
        _close(node)


@pytest.mark.parametrize("fsm", [0, 1])
def test_start_from_damp_still_goes_to_standby(lc_module, fsm):
    """Unveraendert (Gantry-Ablauf aus docs/70): START aus Damp -> FSM 4."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = fsm
        node.start_callback(_bool(True))
        assert ("SetFsmId", 4) in robot.calls and robot.fsm == 4
    finally:
        _close(node)


def test_start_with_unreadable_fsm_does_not_switch_blindly(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        robot._Call = lambda *a: (3104, None)        # RPC-Timeout
        node.start_callback(_bool(True))
        assert "SetFsmId" not in robot.names() and robot.fsm == 500
    finally:
        _close(node)


def test_legacy_dpad_up_does_not_leave_walk(lc_module):
    """Alter Joystick: Steuerkreuz oben = START. Frueher immer SetFsmId(4)."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node._sync_onboard_state(force=True)
        node.joystick_callback(_joy())
        node.joystick_callback(_joy(axes={7: -1.0}))
        assert ("SetFsmId", 4) not in robot.calls and robot.fsm == 500
        assert node.balanced
    finally:
        _close(node)


@pytest.mark.parametrize("fsm", [0, 1, 3])
def test_start_balancing_never_starts_walker_from_damp(lc_module, fsm):
    """GREIFEN (start_balancing) aus Damp/Zero-Torque/Sitzen: frueher Rampe +
    BalanceStand + Start() -> Walk-Regler aus dem schlaffen Zustand."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = fsm
        node.start_balancing_callback(_bool(True))
        assert "Start" not in robot.names() and "SetStandHeight" not in robot.names()
        assert robot.fsm == fsm and not node.balanced
    finally:
        _close(node)


def test_start_balancing_from_preparation_still_works(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 4
        node.start_balancing_callback(_bool(True))
        assert robot.fsm == 501 and node.balanced and not node.walk_enabled
        assert ("BalanceStand", 1) in robot.calls, "Default unveraendert"
    finally:
        _close(node)


def test_start_balancing_adopts_phone_walk_without_ramp(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.start_balancing_callback(_bool(True))
        assert node.balanced and "SetStandHeight" not in robot.names()
    finally:
        _close(node)


def test_start_balancing_ignored_during_estop(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 4
        node.emergency_callback(_bool(True))
        robot.calls.clear()
        node.start_balancing_callback(_bool(True))
        assert "SetStandHeight" not in robot.names() and "Start" not in robot.names()
    finally:
        _close(node)


def test_balancing_problem_aborts_without_start(lc_module):
    """FSM-Mode != 0 (Fuesse nicht belastet, z.B. haengend): frueher brach die
    Rampe ab und rief TROTZDEM BalanceStand + Start()."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 4
        robot.mode = 2
        node.start_walking_callback(_bool(True))
        assert "Start" not in robot.names() and "BalanceStand" not in robot.names()
        assert robot.fsm == 4 and not node.balanced and not node.walk_enabled
    finally:
        _close(node)


def test_balancing_mode_rpc_failure_aborts_without_start(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 4
        orig = robot._Call
        robot._Call = lambda api, p: (orig(api, p) if api == "GET_FSM_ID" else (3104, None))
        node.start_balancing_callback(_bool(True))
        assert "Start" not in robot.names() and not node.balanced
    finally:
        _close(node)


def test_balance_mode_parameter(lc_module):
    node, robot = _make(lc_module, balance_mode=0)
    try:
        robot.fsm = 4
        node.start_balancing_callback(_bool(True))
        assert ("BalanceStand", 0) in robot.calls
    finally:
        _close(node)


def test_onboard_state_published_for_gui(lc_module):
    import json
    node, robot = _make(lc_module)
    got = []
    from std_msgs.msg import String
    node.state_pub.publish = lambda m: got.append(json.loads(m.data))
    try:
        robot.fsm = 500
        node._sync_onboard_state(force=True)
        node.start_walking_callback(_bool(True))
        node._publish_state()
        assert got[-1] == {"fsm": 500, "name": "Walk", "balanced": True, "walk": True,
                           "estop": False, "full_estop": False, "robot": True, "note": "",
                           "rpc_err": None, "walk_fsm": 501}
        node.emergency_callback(_bool(True))
        node._publish_state()
        assert got[-1]["estop"] and not got[-1]["walk"]
    finally:
        _close(node)


# ── Startleiste der Demo-GUI (/g1pilot/loco_request) ──────────────────────

def _req(node, what):
    from std_msgs.msg import String
    node.mode_request_callback(String(data=what))


def test_startbar_full_crane_sequence(lc_module):
    """Einschalten (Zero Torque) -> Damp -> Preparation -> Kran lockern -> Walk."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 0
        _req(node, "damp")
        assert robot.fsm == 1
        _req(node, "preparation")
        assert robot.fsm == 4
        _req(node, "walk")
        assert robot.fsm == 501 and node.balanced and not node.walk_enabled
        assert ("BalanceStand", 1) in robot.calls and "SetStandHeight" not in robot.names()
        node.start_walking_callback(_bool(True))
        assert node.walk_enabled
    finally:
        _close(node)


def test_startbar_walk_refused_while_feet_unloaded(lc_module):
    """Haengt straff am Kran (FSM-Mode != 0): kein Start(), Hinweis fuer die GUI."""
    node, robot = _make(lc_module)
    try:
        robot.fsm, robot.mode = 4, 2
        _req(node, "walk")
        assert "Start" not in robot.names() and "BalanceStand" not in robot.names()
        assert robot.fsm == 4 and not node.balanced
        assert "Kran" in node._note
    finally:
        _close(node)


def test_startbar_walk_refused_on_unreadable_mode(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 4
        orig = robot._Call
        robot._Call = lambda api, p: (orig(api, p) if api == "GET_FSM_ID" else (3104, None))
        _req(node, "walk")
        assert "Start" not in robot.names() and not node.balanced
    finally:
        _close(node)


@pytest.mark.parametrize("fsm", [0, 1, 702])
def test_startbar_walk_only_from_preparation(lc_module, fsm):
    node, robot = _make(lc_module)
    try:
        robot.fsm = fsm
        _req(node, "walk")
        assert robot.fsm == fsm and "Start" not in robot.names()
    finally:
        _close(node)


@pytest.mark.parametrize("fsm", [0, 702])
def test_startbar_preparation_not_from_zero_torque(lc_module, fsm):
    node, robot = _make(lc_module)
    try:
        robot.fsm = fsm
        _req(node, "preparation")
        assert robot.fsm == fsm and "SetFsmId" not in robot.names()
    finally:
        _close(node)


def test_startbar_preparation_from_walk_stops_first(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.start_walking_callback(_bool(True))
        node.cmd_vel_callback(_cmd(vx=0.5))
        node._move_active = True
        _req(node, "preparation")
        assert robot.fsm == 4 and not node.walk_enabled and not node.balanced
        names = robot.names()
        assert names.index("SetVelocity") < names.index("SetFsmId")
    finally:
        _close(node)


def test_startbar_refused_during_estop_except_damp(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 4
        node.emergency_callback(_bool(True))
        robot.calls.clear()
        _req(node, "walk")
        _req(node, "preparation")
        assert "Start" not in robot.names() and "SetFsmId" not in robot.names()
        _req(node, "damp")
        assert robot.fsm == 1
    finally:
        _close(node)


def test_startbar_damp_from_walk(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.start_walking_callback(_bool(True))
        _req(node, "damp")
        assert robot.fsm == 1 and not node.walk_enabled and not node.balanced
    finally:
        _close(node)


def test_startbar_ignored_in_sim(lc_module):
    node, robot = _make(lc_module, use_robot=False)
    try:
        before = list(robot.calls) if robot is not None else []
        _req(node, "damp")
        assert (list(robot.calls) if robot is not None else []) == before
    finally:
        _close(node)


def test_rpc_read_error_reported_in_state(lc_module):
    """FSM nicht lesbar (z. B. 3104): Code landet im Anzeige-JSON, nach Erfolg weg."""
    import json as _json
    node, robot = _make(lc_module)
    try:
        got = []
        node.state_pub.publish = lambda m: got.append(_json.loads(m.data))
        orig = robot._Call
        robot._Call = lambda *a: (3104, None)
        node._sync_onboard_state(force=True)
        node._publish_state()
        assert got[-1]["rpc_err"] == 3104
        robot._Call = orig
        robot.fsm = 4
        node._sync_onboard_state(force=True)
        node._publish_state()
        assert got[-1]["rpc_err"] is None and got[-1]["fsm"] == 4
    finally:
        _close(node)


def test_state_publish_not_in_default_group(lc_module):
    """Die Anzeige laeuft in eigener Callback-Gruppe (haengender RPC friert sie
    nicht ein)."""
    node, robot = _make(lc_module)
    try:
        timers = [t for t in node.timers if t.callback == node._publish_state]
        assert timers and timers[0].callback_group is node._state_group
    finally:
        _close(node)


def test_loco_service_is_sport_even_with_old_sdk(lc_module):
    """Real 2026-10-16: alle Loco-RPCs 3102. Das Real-Image installiert ein SDK
    mit Dienst "loco"; der G1 (ai_sport >= 8.2) hoert nur auf "sport"."""
    node, robot = _make(lc_module)
    try:
        assert robot.service == "sport"
    finally:
        _close(node)


def test_walk_start_uses_fsm_501_not_sdk_start(lc_module):
    """Start() ist im alten SDK FSM 200, im aktuellen 500. Nils 2026-10-19:
    Walk soll 501 (Walk with waist) sein -> explizit SetFsmId(501)."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 4
        node.start_walking_callback(_bool(True))
        assert ("SetFsmId", 501) in robot.calls
        assert ("SetFsmId", 500) not in robot.calls
        assert ("SetFsmId", 200) not in robot.calls
    finally:
        _close(node)


@pytest.mark.parametrize("entry", ["startbar", "laufen", "greifen"])
def test_all_walk_entries_use_start_fsm_501(lc_module, entry):
    """Startleiste »Walk«, LAUFEN und GREIFEN (START BALANCING) starten alle
    denselben Walk-Modus 501 -- und das Polling haelt ihn als Walk."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 4
        if entry == "startbar":
            _req(node, "walk")
        elif entry == "laufen":
            node.start_walking_callback(_bool(True))
        else:
            node.start_balancing_callback(_bool(True))
        assert robot.fsm == 501 and node.balanced
        node._balanced_at -= 10.0           # Schonfrist vorbei
        for _ in range(3):
            node._sync_onboard_state(force=True)
        assert node.balanced, "501 muss als Walk erkannt bleiben"
    finally:
        _close(node)


def test_start_fsm_id_configurable_back_to_500(lc_module):
    """G1_WALK_FSM_ID=500 -> alter Walk (Taille fest)."""
    node, robot = _make(lc_module, start_fsm_id=500)
    try:
        robot.fsm = 4
        _req(node, "walk")
        assert robot.fsm == 500 and ("SetFsmId", 501) not in robot.calls
    finally:
        _close(node)


def test_start_fsm_id_always_counts_as_walk(lc_module):
    """Eigene Start-ID fehlt in walk_fsm_ids -> zaehlt trotzdem als Walk."""
    node, robot = _make(lc_module, start_fsm_id=801, walk_fsm_ids="[500]")
    try:
        robot.fsm = 4
        _req(node, "walk")
        assert robot.fsm == 801 and node.balanced
        node._balanced_at -= 10.0
        for _ in range(3):
            node._sync_onboard_state(force=True)
        assert node.balanced
    finally:
        _close(node)


def test_startbar_walk_switches_phone_500_to_501(lc_module):
    """Per Handy-App auf 500 gestellt: Startleiste »Walk« wechselt nur den
    Walk-Modus auf 501 (kein BalanceStand, keine FSM 4 dazwischen)."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node._sync_onboard_state(force=True)
        assert node.balanced
        robot.calls.clear()
        _req(node, "walk")
        assert robot.fsm == 501 and node.balanced
        assert ("SetFsmId", 501) in robot.calls
        assert ("SetFsmId", 4) not in robot.calls and "BalanceStand" not in robot.names()
        assert "501" in node._note
        # Schon in 501: nichts mehr senden.
        robot.calls.clear()
        _req(node, "walk")
        assert "SetFsmId" not in robot.names()
    finally:
        _close(node)


def test_startbar_walk_switch_keeps_gui_walking_but_stops_move(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.start_walking_callback(_bool(True))
        node.cmd_vel_callback(_cmd(vx=1.0))
        node._cmd_vel_tick()
        assert node._move_active
        robot.calls.clear()
        _req(node, "walk")
        names = robot.names()
        assert names.index("SetVelocity") < names.index("SetFsmId"), "erst StopMove"
        assert robot.fsm == 501 and node.walk_enabled
    finally:
        _close(node)


def test_startbar_walk_switch_refused_during_estop(lc_module):
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.emergency_callback(_bool(True))
        robot.calls.clear()
        _req(node, "walk")
        assert "SetFsmId" not in robot.names() and robot.fsm == 500
    finally:
        _close(node)


def test_laufen_does_not_switch_phone_500(lc_module):
    """LAUFEN/GREIFEN schalten den Walk-Modus nicht implizit um (nur die
    Startleiste) -- ein per App gestellter 500 wird wie bisher uebernommen."""
    node, robot = _make(lc_module)
    try:
        robot.fsm = 500
        node.start_walking_callback(_bool(True))
        node.start_balancing_callback(_bool(True))
        assert "SetFsmId" not in robot.names() and robot.fsm == 500
    finally:
        _close(node)

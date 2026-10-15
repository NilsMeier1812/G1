"""loco_sim: Regelschleifen-Statuszeile loggt nur bei Auffaelligkeit."""
import types

import numpy as np
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("unitree_sdk2py")


@pytest.fixture()
def ls():
    # Erst zur Testlaufzeit importieren (nicht beim Einsammeln der Tests).
    from g1pilot.navigation import loco_sim
    return loco_sim


class _Log:
    def __init__(self):
        self.warn_msgs, self.info_msgs = [], []

    def warn(self, m):
        self.warn_msgs.append(m)

    def info(self, m):
        self.info_msgs.append(m)


def _node(ls, grav=(0.0, 0.0, -1.0), lockstep=False):
    log = _Log()
    n = types.SimpleNamespace(
        control_dt=0.02, lockstep=lockstep, state=ls.STAND, cmd=np.zeros(3),
        _dbg_grav=np.array(grav, dtype=np.float32), _health_log_t=-1e9,
        get_logger=lambda: log)
    return n, log


def _run(ls, n, n_ticks=100, span=2.0):
    ls.LocoSim._check_loop_health(n, n_ticks, span)


def test_normal_ist_still(ls):
    n, log = _node(ls)
    _run(ls, n)                      # 50 Hz, aufrecht
    assert log.warn_msgs == [] and log.info_msgs == []


def test_lockstep_ist_still(ls):
    n, log = _node(ls, lockstep=True)
    _run(ls, n, span=10.0)           # Wandzeit egal, Takt ist deterministisch
    assert log.warn_msgs == [] and log.info_msgs == []


def test_langsamer_takt_warnt_und_ist_ratenbegrenzt(ls):
    n, log = _node(ls)
    _run(ls, n, span=3.0)            # 33 Hz
    assert len(log.warn_msgs) == 1 and "zu langsam" in log.warn_msgs[0]
    _run(ls, n, span=3.0)            # direkt danach: unterdrueckt
    assert len(log.warn_msgs) == 1


def test_neigung_warnt(ls):
    n, log = _node(ls, grav=(0.6, 0.0, -0.8))
    _run(ls, n)
    assert len(log.warn_msgs) == 1 and "Neigung" in log.warn_msgs[0]


def test_debug_flag_loggt_immer(ls, monkeypatch):
    monkeypatch.setattr(ls, "LOCO_DEBUG", True)
    n, log = _node(ls)
    _run(ls, n)
    assert len(log.info_msgs) == 1 and "eff=50.0Hz" in log.info_msgs[0]

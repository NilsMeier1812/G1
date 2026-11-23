"""Startmenue: Roboter-LAN (Live-Verbindungsanzeige + Netzwerk-Assistent).

Logik ohne Qt (netsetup) und die GUI offscreen. Nichts hier fasst das echte
Netzwerk an: nmcli/ping/ufw werden durch Attrappen ersetzt.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["G1_START_GUI_NO_EXIT"] = "1"

PKG_ROOT = Path(__file__).resolve().parent.parent          # .../g1pilot
sys.path.insert(0, str(PKG_ROOT))

from start_gui import netsetup  # noqa: E402
from start_gui.netsetup import LinkStatus, Step  # noqa: E402

LAN = [("wlp2s0", "10.0.0.5/24"), ("enp0s31f6", "192.168.123.222/24")]


# ════════════════════════════════════════════════════════════════════════
#  Logik
# ════════════════════════════════════════════════════════════════════════
def test_validate_pc_ip():
    assert netsetup.validate_pc_ip("192.168.123.222") == ("192.168.123.222", "")
    assert netsetup.validate_pc_ip(" 192.168.123.50/24 ") == ("192.168.123.50", "")
    for bad in ("", "abc", "10.0.0.5", "192.168.124.5", "192.168.123.0",
                "192.168.123.255", "192.168.123.161", "192.168.123.210", "192.168.123.211"):
        ip, err = netsetup.validate_pc_ip(bad)
        assert ip == "" and err, bad
    assert "G1-Onboard-PC" in netsetup.validate_pc_ip("192.168.123.161")[1]


def test_in_robot_lan():
    assert netsetup.in_robot_lan("192.168.123.222/24")
    assert netsetup.in_robot_lan("192.168.123.9")
    assert not netsetup.in_robot_lan("192.168.12.3/24")
    assert not netsetup.in_robot_lan("")
    assert netsetup.robot_ifaces(LAN) == ["enp0s31f6"]


def test_plan_new_profile_no_gateway_no_dns():
    steps = netsetup.plan("enp3s0", "192.168.123.222", None)
    assert [s.argv[:3] for s in steps] == [["nmcli", "connection", "add"],
                                           ["nmcli", "connection", "up"]]
    add = steps[0].argv
    assert add[add.index("ifname") + 1] == "enp3s0"
    assert add[add.index("con-name") + 1] == "g1-robot"
    assert add[add.index("ipv4.addresses") + 1] == "192.168.123.222/24"
    assert add[add.index("ipv4.gateway") + 1] == ""
    assert add[add.index("ipv4.dns") + 1] == ""
    assert add[add.index("ipv4.never-default") + 1] == "yes"
    assert add[add.index("ipv6.method") + 1] == "disabled"
    assert not steps[0].root and not steps[0].optional
    assert steps[1].optional                       # ohne Kabel kein Abbruch
    # Anzeige: gequotet, leere Werte sichtbar
    assert "ipv4.gateway ''" in steps[0].shell()
    assert not steps[0].shell().startswith("sudo")


def test_plan_existing_profile_is_modified_not_deleted():
    prof = {"iface": "eth0", "ip": "192.168.123.99/24", "method": "manual", "active": False}
    steps = netsetup.plan("enp3s0", "192.168.123.222", prof, link_up=False)
    mod = steps[0].argv
    assert mod[:4] == ["nmcli", "connection", "modify", "g1-robot"]
    assert mod[mod.index("connection.interface-name") + 1] == "enp3s0"
    assert all("delete" not in s.argv for s in steps)
    assert "aktiviert sich von selbst" in steps[1].note


def test_plan_firewall_step_is_root_and_optional():
    steps = netsetup.plan("enp3s0", "192.168.123.222", None, open_firewall=True)
    fw = steps[-1]
    assert fw.argv == ["ufw", "allow", "in", "on", "enp3s0"]
    assert fw.root and fw.optional
    assert fw.shell().startswith("sudo ufw")
    assert netsetup.remove_plan()[0].argv == ["nmcli", "connection", "delete", "g1-robot"]
    assert "ip addr add 192.168.123.222/24 dev enp3s0" in netsetup.manual_commands(
        "enp3s0", "192.168.123.222")


def test_with_root_and_retry_detection(monkeypatch):
    s = Step("x", ["nmcli", "connection", "up", "g1-robot"])
    monkeypatch.setattr(netsetup.os, "geteuid", lambda: 1000)
    monkeypatch.setattr(netsetup, "have", lambda c: True)
    monkeypatch.setattr(netsetup.shutil, "which", lambda c: f"/usr/bin/{c}")
    assert netsetup.with_root(s) == ["pkexec", "/usr/bin/nmcli", "connection", "up", "g1-robot"]
    monkeypatch.setattr(netsetup.os, "geteuid", lambda: 0)
    assert netsetup.with_root(s) == s.argv
    assert netsetup.needs_root_retry("Error: Failed to add 'g1-robot' connection: "
                                     "Insufficient privileges.")
    assert netsetup.needs_root_retry("Error: Not authorized to control networking.")
    assert not netsetup.needs_root_retry("Error: Connection activation failed: No carrier")


def test_parse_profile_ufw_carrier(tmp_path):
    d = netsetup.parse_profile("connection.interface-name:enp3s0\n"
                               "ipv4.method:manual\nipv4.addresses:192.168.123.222/24\n"
                               "GENERAL.STATE:activated\n")
    assert d["connection.interface-name"] == "enp3s0"
    assert d["ipv4.addresses"] == "192.168.123.222/24"
    conf = tmp_path / "ufw.conf"
    conf.write_text("# x\nENABLED=no\n")
    assert not netsetup.ufw_enabled(conf)
    conf.write_text("ENABLED=yes\nLOGLEVEL=low\n")
    assert netsetup.ufw_enabled(conf)
    assert not netsetup.ufw_enabled(tmp_path / "fehlt")
    (tmp_path / "enp3s0").mkdir()
    (tmp_path / "enp3s0" / "carrier").write_text("1\n")
    assert netsetup.carrier("enp3s0", tmp_path) is True
    (tmp_path / "enp3s0" / "carrier").write_text("0\n")
    assert netsetup.carrier("enp3s0", tmp_path) is False
    assert netsetup.carrier("gibtsnicht", tmp_path) is None
    assert netsetup.carrier("../etc", tmp_path) is None


def test_link_status_levels_and_texts():
    assert LinkStatus(iface="").level == "off"
    st = LinkStatus(iface="enp3s0", ip="", link=False)
    assert st.level == "off" and "kein Kabel" in st.text()
    st = LinkStatus(iface="enp3s0", ip="", link=True)
    assert st.level == "off" and "Keine 192.168.123.x" in st.text()
    assert "Netzwerk einrichten" in st.text()
    st = LinkStatus(iface="enp3s0", ip="192.168.123.222/24", link=True, g1=False)
    assert st.level == "warn" and "antwortet nicht" in st.text()
    st = LinkStatus(iface="enp3s0", ip="192.168.123.222/24", link=True, g1=True)
    assert st.level == "ok" and "G1 antwortet" in st.text()
    st.hands = [("Hand links", "192.168.123.210", True), ("Hand rechts", "x", False)]
    assert st.level == "warn"
    st = LinkStatus(iface="enp3s0", ip="192.168.123.222/24", link=True, g1=True,
                    others=["enx1"])
    assert st.level == "warn" and "enx1" in st.text()


def test_probe_pings_only_inside_robot_lan():
    pinged, tcp = [], []

    def ping_fn(h):
        pinged.append(h)
        return True

    def tcp_fn(h, p):
        tcp.append((h, p))
        return h.endswith(".210")

    kw = dict(nics_fn=lambda: LAN, ping_fn=ping_fn, tcp_fn=tcp_fn, carrier_fn=lambda i: True)
    hands = [("Hand links", "192.168.123.210"), ("Hand rechts", "192.168.123.211")]
    st = netsetup.probe("wlp2s0", hands, 6000, **kw)
    assert st.level == "off" and pinged == [] and tcp == []
    st = netsetup.probe("enp0s31f6", hands, 6000, **kw)
    assert pinged == [netsetup.G1_IP]
    assert tcp == [("192.168.123.210", 6000), ("192.168.123.211", 6000)]
    assert st.hands == [("Hand links", "192.168.123.210", True),
                        ("Hand rechts", "192.168.123.211", False)]
    assert st.level == "warn"
    # Kabel gezogen: kein Ping, obwohl die IP (noch) dran haengt
    pinged.clear()
    st = netsetup.probe("enp0s31f6", [], 6000, **{**kw, "carrier_fn": lambda i: False})
    assert pinged == [] and st.level == "off"


# ════════════════════════════════════════════════════════════════════════
#  GUI (offscreen)
# ════════════════════════════════════════════════════════════════════════
@pytest.fixture
def win(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    from start_gui.app import MainWindow, make_app
    app = make_app([])
    w = MainWindow(check_docker=False, auto_thumbs=False)
    yield w
    w.close()
    app.processEvents()


def _pump_until(cond, timeout=5.0):
    from PyQt6.QtWidgets import QApplication
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    return False


class FakeNet:
    """Veraenderbarer Netzzustand fuer die Live-Anzeige."""

    def __init__(self):
        self.nics = [("enp0s31f6", "")]
        self.g1 = False
        self.calls = 0
        self.delay = 0.0

    def probe(self, iface, hands=None, port=6000):
        self.calls += 1
        if self.delay:
            time.sleep(self.delay)
        return netsetup.probe(iface, hands, port, nics_fn=lambda: list(self.nics),
                              ping_fn=lambda h: self.g1, tcp_fn=lambda h, p: True,
                              carrier_fn=lambda i: True)


def test_real_view_status_updates_without_reopen(win):
    """Nils' Wunsch: die Anzeige wechselt von selbst von orange auf gruen."""
    from start_gui.views import RealView
    net = FakeNet()
    v = RealView(win, nics=list(net.nics), probe=net.probe)
    v.monitor.timer.setInterval(50)
    win.show_view(v)
    win.show()
    assert v.monitor.running
    assert _pump_until(lambda: "Keine 192.168.123.x" in v.lan.text())
    assert v.link.level == "off"
    # IP kommt dazu (Assistent/Kabel) -> Liste + Anzeige ohne neues Oeffnen
    net.nics = [("enp0s31f6", "192.168.123.222/24")]
    assert _pump_until(lambda: "antwortet nicht" in v.lan.text())
    assert "192.168.123.222" in v.iface.currentText()
    assert v._iface() == "enp0s31f6"
    net.g1 = True
    assert _pump_until(lambda: v.link.level == "ok")
    assert "G1 antwortet" in v.lan.text()
    assert "Hand links" in v.link.hands.text() and not v.link.hands.isHidden()
    assert v.link.when.text().startswith("geprüft ")
    # Haende aus -> Haende-Zeile verschwindet
    v.hands.setChecked(False)
    assert _pump_until(lambda: v.link.hands.isHidden())
    # Seite verlassen -> keine Pruefungen mehr
    win.show_menu()
    assert not v.monitor.running


def test_monitor_never_overlaps(win):
    from start_gui.views import RealView
    net = FakeNet()
    net.delay = 0.3
    v = RealView(win, nics=list(net.nics), probe=net.probe)
    v.monitor.stop()
    got = []
    v.monitor.updated.connect(got.append)
    v.monitor.check_now()
    v.monitor.check_now()            # laeuft noch -> nur vormerken
    v.monitor.check_now()
    assert net.calls <= 1
    assert _pump_until(lambda: len(got) >= 2, timeout=4)
    assert net.calls == 2            # genau eine Nachpruefung


def test_real_view_gate_still_uses_fixed_nics(win):
    from start_gui.views import RealView
    v = RealView(win, nics=LAN)
    assert v._iface() == "enp0s31f6"
    assert "Im Roboter-LAN" in v.lan.text() and v.link.level == "ok"
    v.iface.setCurrentText("wlp2s0  (10.0.0.5/24)")
    assert "Keine 192.168.123.x" in v.lan.text()


HOST_NEW = {"nm": True, "profile": None, "ufw": True}
HOST_OLD = {"nm": True, "ufw": False,
            "profile": {"iface": "eth9", "ip": "192.168.123.77/24", "method": "manual",
                        "active": False}}


def _dialog(win, nics=None, iface="", host=None, carrier=lambda i: True):
    from start_gui.network import NetworkSetupDialog
    return NetworkSetupDialog(win, nics if nics is not None else
                              [("wlp2s0", "10.0.0.5/24"), ("enp3s0", "")],
                              iface, host=host or HOST_NEW, carrier_fn=carrier)


def test_dialog_preview_and_validation(win):
    d = _dialog(win, iface="enp3s0")
    assert d.selected_iface() == "enp3s0"
    text = d.preview.toPlainText()
    assert "nmcli connection add type ethernet ifname enp3s0 con-name g1-robot" in text
    assert "192.168.123.222/24" in text
    assert "sudo ufw allow in on enp3s0" in text          # ufw aktiv -> Schalter an
    assert d.apply_btn.isEnabled()
    assert not d.remove_btn.isVisibleTo(d)                # kein Profil da
    d.fw.setChecked(False)
    assert "ufw" not in d.preview.toPlainText()
    d.ip.setText("192.168.123.161")
    assert not d.apply_btn.isEnabled() and "G1-Onboard-PC" in d.findings.text()
    assert d.preview.toPlainText() == ""
    d.ip.setText("192.168.123.230")
    assert d.apply_btn.isEnabled() and "192.168.123.230/24" in d.preview.toPlainText()
    d.iface.setCurrentIndex(0)                            # WLAN gewaehlt
    assert "WLAN" in d.findings.text()


def test_dialog_existing_profile_and_no_cable(win):
    d = _dialog(win, nics=[("eth9", ""), ("enp3s0", "")], host=HOST_OLD,
                carrier=lambda i: False)
    assert d.selected_iface() == "eth9"                   # Interface des Profils
    assert d.ip.text() == "192.168.123.77"
    assert "gibt es schon" in d.findings.text()
    assert "kein Kabel" in d.findings.text()
    assert "nmcli connection modify g1-robot" in d.preview.toPlainText()
    assert not d.fw.isVisibleTo(d)                         # ufw aus -> kein Schalter
    assert d.remove_btn.isVisibleTo(d)


def test_dialog_without_networkmanager_shows_manual_commands(win):
    d = _dialog(win, iface="enp3s0", host={"nm": False, "profile": None, "ufw": False})
    assert not d.apply_btn.isEnabled()
    assert "sudo ip addr add 192.168.123.222/24 dev enp3s0" in d.preview.toPlainText()
    assert "NetworkManager" in d.findings.text()


def test_dialog_warns_about_second_robot_iface(win):
    d = _dialog(win, nics=[("enp3s0", ""), ("enx1", "192.168.123.5/24")], iface="enp3s0")
    assert "enx1" in d.findings.text()


def _run(d, steps):
    d._steps = steps
    applied = []
    d.applied.connect(lambda: applied.append(1))
    d.apply_btn.setEnabled(True)
    d._apply()
    assert _pump_until(lambda: d.ok is not None)
    return applied


def test_dialog_runs_steps_and_tolerates_optional_failure(win):
    d = _dialog(win, iface="enp3s0")
    applied = _run(d, [Step("eins", ["sh", "-c", "echo hallo"]),
                       Step("zwei", ["sh", "-c", "echo kein Kabel; exit 4"], optional=True,
                            note="Aktiviert sich beim Einstecken.")])
    assert d.ok is True and applied == [1]
    log = d.log.toPlainText()
    assert "hallo" in log and "kein Kabel" in log and "Hinweis:" in log
    assert "Aktiviert sich beim Einstecken." in d.result.text()
    assert d.close_btn.text() == "Fertig"


def test_dialog_stops_on_required_failure(win):
    d = _dialog(win, iface="enp3s0")
    applied = _run(d, [Step("eins", ["sh", "-c", "echo kaputt; exit 2"]),
                       Step("zwei", ["sh", "-c", "echo darf-nicht-laufen"])])
    assert d.ok is False and applied == []
    assert "darf-nicht-laufen" not in d.log.toPlainText()
    assert "Abgebrochen" in d.result.text()


def test_dialog_retries_with_root_on_permission_error(win, monkeypatch):
    monkeypatch.setattr(netsetup, "with_root",
                        lambda step: ["sh", "-c", "echo als-root-ok"])
    d = _dialog(win, iface="enp3s0")
    _run(d, [Step("eins", ["sh", "-c", "echo 'Error: Insufficient privileges.'; exit 1"])])
    assert d.ok is True
    log = d.log.toPlainText()
    assert "pkexec" in log and "als-root-ok" in log


def test_real_view_opens_dialog_with_current_iface(win, monkeypatch):
    from g1pilot.utils import msgbox
    from start_gui.views import RealView
    seen = {}

    def fake_run(dlg):
        seen["iface"] = dlg.selected_iface()
        seen["plan"] = dlg.plan()
        return False
    monkeypatch.setattr(msgbox, "run_dialog", fake_run)
    v = RealView(win, nics=[("enp3s0", ""), ("wlp2s0", "10.0.0.5/24")], setup_host=HOST_NEW)
    v.iface.setCurrentIndex(0)
    v.link.setup_btn.click()
    assert seen["iface"] == "enp3s0"
    assert seen["plan"][0].argv[:3] == ["nmcli", "connection", "add"]

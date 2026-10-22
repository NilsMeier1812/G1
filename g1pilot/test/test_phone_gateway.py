#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Handy-Gateway ohne Bluetooth/Qt: Kopplung, Anmeldung, Signaturen,
Wiederholschutz, Rechte, Steuerhoheit, Timeouts."""
import json
import os
import stat

import pytest

from g1pilot.teleoperation.phone import gateway as G
from g1pilot.teleoperation.phone import protocol as P
from phone_client import FakePhone


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def env(tmp_path):
    clk = Clock()
    gw = G.PhoneGateway(store_path=str(tmp_path / "dev.json"), clock=clk)
    gw.publish_status({"x": 1})
    return gw, clk, tmp_path


def _paired(gw, role=G.ROLE_OPERATE, **kw):
    ph = FakePhone(gw, **kw)
    assert ph.pair(gw.start_pairing(role))
    assert ph.login() == "ok"
    gw.publish_status({"x": 2})
    return ph


# ── Kopplung ────────────────────────────────────────────────────────────
def test_pairing_needs_open_code_on_pc(env):
    gw, clk, _ = env
    ph = FakePhone(gw)
    assert not ph.pair("ABCD-EFGH-JKMN-PQRS")
    assert ph.pre()["t"] == "pair_fail"
    assert gw.devices == {}


def test_pairing_wrong_code_and_lockout(env):
    gw, clk, _ = env
    code = gw.start_pairing()
    ph = FakePhone(gw)
    for _ in range(G.PAIRING_MAX_FAILS):
        assert not ph.pair("0000-0000-0000-0000")
    assert not gw.pairing_active                  # Code nach 3 Fehlversuchen ungueltig
    assert not ph.pair(code)
    _, events = gw.tick()
    assert ("pair_failed" in [e[0] for e in events])


def test_pairing_expires(env):
    gw, clk, _ = env
    code = gw.start_pairing()
    clk.t += G.PAIRING_TIMEOUT_S + 1
    assert not FakePhone(gw).pair(code)


def test_pairing_code_tolerant_input(env):
    gw, clk, _ = env
    code = gw.start_pairing()
    sloppy = code.lower().replace("-", " ").replace("0", "o")
    assert FakePhone(gw).pair(sloppy)


def test_device_key_never_sent_and_store_private(env):
    gw, clk, tmp = env
    ph = FakePhone(gw)
    code = gw.start_pairing()
    assert ph.pair(code)
    raw = json.dumps(ph.tx)
    assert ph.dkey.hex() not in raw
    path = tmp / "dev.json"
    assert stat.S_IMODE(os.stat(path).st_mode) == 0o600
    gw2 = G.PhoneGateway(store_path=str(path), clock=clk)   # nach Neustart noch gekoppelt
    assert ph.dev in gw2.devices


def test_code_used_once(env):
    gw, clk, _ = env
    code = gw.start_pairing()
    assert FakePhone(gw).pair(code)
    assert not FakePhone(gw).pair(code)


# ── Anmeldung ───────────────────────────────────────────────────────────
def test_unknown_device_rejected(env):
    gw, clk, _ = env
    ph = FakePhone(gw)
    ph.dkey = b"x" * 32
    assert ph.login() == "unknown"
    assert gw.session is None


def test_wrong_key_fails_and_locks(env):
    gw, clk, _ = env
    ph = FakePhone(gw)
    assert ph.pair(gw.start_pairing())
    good = ph.dkey
    ph.dkey = b"y" * 32
    with pytest.raises(AssertionError):            # Handy erkennt: PC-Beweis passt nicht
        ph.login()
    # Angreifer mit falschem Schluessel, der den PC-Beweis ignoriert:
    for _ in range(G.AUTH_MAX_FAILS):
        nc = P.nonce()
        ph.send({"t": "hello", "dev": ph.dev, "nc": nc})
        ph.send({"t": "auth", "dev": ph.dev, "cli": "0" * 32})
    assert gw.session is None
    ph.dkey = good
    assert ph.login() is None or gw.session is None   # gesperrt
    clk.t += G.AUTH_LOCK_S + 1
    assert ph.login() == "ok"


def test_auth_must_come_from_same_link(env):
    gw, clk, _ = env
    ph = FakePhone(gw)
    assert ph.pair(gw.start_pairing())
    nc = P.nonce()
    ph.send({"t": "hello", "dev": ph.dev, "nc": nc})
    ns = ph.pre()["ns"]
    gw.handle_line("other_link", json.dumps(
        {"t": "auth", "dev": ph.dev, "cli": P.client_proof(ph.dkey, ph.dev, nc, ns)}))
    assert gw.session is None


def test_second_phone_busy_until_first_times_out(env):
    gw, clk, _ = env
    a = _paired(gw, link="L1", name="A")
    b = FakePhone(gw, link="L2", name="B")
    assert b.pair(gw.start_pairing())
    assert b.login() == "busy"
    assert gw.session.dev == a.dev
    clk.t += G.SESSION_TIMEOUT_S + 0.5
    gw.tick()
    assert b.login() == "ok"


# ── Signaturen / Wiederholung ───────────────────────────────────────────
def test_replay_and_forgery_ignored(env):
    gw, clk, _ = env
    ph = _paired(gw)
    ph.signed({"t": "cmd", "a": "take", "s": ph.last["q"], "i": "c1"})
    assert gw.has_control()
    line = ph.signed({"t": "cmd", "a": "estop", "s": 0, "i": "c2"})
    actions, _ = gw.tick()
    assert [a["action"] for a in actions] == ["estop"]
    gw.handle_line(ph.link, line)                       # exakt wiederholt
    forged = line.replace('"estop"', '"seq"')            # veraendert
    gw.handle_line(ph.link, forged)
    c, m, payload = P.split_signed(line)
    gw.handle_line(ph.link, f"A{c + 50}.{m}.{payload}")  # Zaehler hochgesetzt, alte Signatur
    actions, _ = gw.tick()
    assert actions == []


def test_signed_from_other_link_ignored(env):
    gw, clk, _ = env
    ph = _paired(gw)
    ph.c += 1
    line = P.encode_signed(ph.skey, "c2s", ph.c, {"t": "cmd", "a": "estop", "s": 0, "i": "z"})
    gw.handle_line("fremd", line)
    assert gw.tick()[0] == []


# ── Rechte / Steuerhoheit ──────────────────────────────────────────────
def test_view_role_only_estop(env):
    gw, clk, _ = env
    ph = _paired(gw, role=G.ROLE_VIEW)
    cid = ph.cmd("take")
    assert ph.result(cid)["ok"] is False
    assert not gw.has_control()
    ph.cmd("estop")
    actions, _ = gw.tick()
    assert [a["action"] for a in actions] == ["estop"]


def test_view_estop_can_be_disabled(env, tmp_path):
    gw, clk, _ = env
    gw.estop_for_view = False
    ph = _paired(gw, role=G.ROLE_VIEW)
    cid = ph.cmd("estop")
    assert gw.tick()[0] == [] and ph.result(cid)["ok"] is False


def test_commands_need_control(env):
    gw, clk, _ = env
    ph = _paired(gw)
    cid = ph.cmd("ack")
    assert gw.tick()[0] == [] and "übernehmen" in ph.result(cid)["m"]
    ph.cmd("take")
    gw.publish_status({"x": 3})
    ph.cmd("ack")
    assert [a["action"] for a in gw.tick()[0]] == ["ack"]


def test_estop_and_cancel_without_control(env):
    gw, clk, _ = env
    ph = _paired(gw)
    ph.cmd("estop", seq=-1)
    ph.cmd("cancel", seq=-1)
    assert [a["action"] for a in gw.tick()[0]] == ["estop", "cancel"]


def test_stale_status_reference_rejected(env):
    gw, clk, _ = env
    ph = _paired(gw)
    ph.cmd("take")
    old = ph.last["q"]
    for _ in range(G.STATUS_MAX_LAG + 1):
        gw.publish_status({"x": 9})
    cid = ph.cmd("ack", seq=old)
    assert gw.tick()[0] == [] and "nicht aktuell" in ph.result(cid)["m"]
    cid = ph.cmd("ack", seq=gw.status_seq + 3)           # aus der Zukunft
    assert gw.tick()[0] == []


def test_duplicate_command_id_once(env):
    gw, clk, _ = env
    ph = _paired(gw)
    for _ in range(2):
        ph.signed({"t": "cmd", "a": "estop", "s": 0, "i": "same"})
    assert len(gw.tick()[0]) == 1


def test_control_lost_without_heartbeat(env):
    gw, clk, _ = env
    ph = _paired(gw)
    ph.cmd("take")
    gw.tick()
    clk.t += G.CONTROL_TIMEOUT_S - 0.5
    ph.hb()
    clk.t += G.CONTROL_TIMEOUT_S - 0.5
    gw.tick()
    assert gw.has_control()
    clk.t += 1.0
    _, events = gw.tick()
    assert not gw.has_control()
    assert "control_lost" in [e[0] for e in events]


def test_session_ends_without_messages(env):
    gw, clk, _ = env
    ph = _paired(gw)
    ph.cmd("take")
    clk.t += G.SESSION_TIMEOUT_S + 0.1
    _, events = gw.tick()
    kinds = [e[0] for e in events]
    assert gw.session is None and "control_lost" in kinds and "disconnected" in kinds


def test_revoke_control_from_pc(env):
    gw, clk, _ = env
    ph = _paired(gw)
    ph.cmd("take")
    gw.revoke_control()
    assert not gw.has_control() and ph.last["me"]["ctl"] is False
    cid = ph.cmd("ack")
    assert gw.tick()[0] == []


def test_relogin_drops_control(env):
    gw, clk, _ = env
    ph = _paired(gw)
    ph.cmd("take")
    assert ph.login() == "ok"
    assert not gw.has_control()


def test_remove_device_ends_session(env):
    gw, clk, _ = env
    ph = _paired(gw)
    assert gw.remove_device(ph.dev)
    assert gw.session is None
    assert ph.login() == "unknown"


def test_bye_ends_session(env):
    gw, clk, _ = env
    ph = _paired(gw)
    ph.signed({"t": "bye"})
    assert gw.session is None


def test_status_signed_and_sized(env):
    gw, clk, _ = env
    ph = _paired(gw)
    gw.publish_status({"txt": "x" * 100, "tmp": {"g": [["Bein", 40, "ok"]] * 400}})
    assert ph.last["st"].get("slim") is True and "tmp" not in ph.last["st"]
    pages = P.tx_pages(1, json.dumps(ph.tx).encode())
    assert len(pages) <= len(P.TX_UUIDS) and all(len(p) <= 512 for p in pages)
    assert P.join_pages(pages)[1] == json.dumps(ph.tx).encode()


def test_line_buffer_offsets():
    lb = P.LineBuffer()
    assert lb.feed("a", b'{"t":"x"}\n') == ['{"t":"x"}']
    assert lb.feed("a", b'{"t":') == []
    assert lb.feed("a", b'"y"}\n', offset=5) == ['{"t":"y"}']
    assert lb.feed("a", b"zz", offset=99) == []          # Luecke -> verwerfen
    assert lb.feed("a", b"\xff\xfe\n") == []              # kein UTF-8


def test_tx_pages_consistent_with_slow_reader():
    # Echtes BLE: Lesen von TX0..TX2 dauert laenger als ein Statustakt. Jeder
    # Lesedurchgang muss trotzdem einen vollstaendigen Stand liefern.
    import threading
    import time
    txp = P.TxPages()
    stop = threading.Event()

    def publisher():
        k = 0
        while not stop.is_set():
            k += 1
            txp.set(json.dumps({"k": k, "pad": "x" * 1100}).encode())
            time.sleep(0.02)

    txp.set(json.dumps({"k": 0, "pad": "x" * 1100}).encode())
    t = threading.Thread(target=publisher, daemon=True)
    t.start()
    ok = 0
    try:
        for _ in range(10):
            pages = [txp.read("phone", 0)]
            k = int(pages[0].split(b":")[2])
            assert k == 3
            for i in range(1, k):
                time.sleep(0.03)                       # > Takt des Publishers
                pages.append(txp.read("phone", i, 0))
                pages[-1] = pages[-1][:5] + txp.read("phone", i, 5)   # langer Read in zwei Teilen
            joined = P.join_pages(pages)
            assert joined is not None
            json.loads(joined[1])
            ok += 1
    finally:
        stop.set()
        t.join()
    assert ok == 10
    # Anderes Geraet hat seinen eigenen Stand, drop() vergisst ihn.
    txp.drop("phone")
    assert txp.read("other", 1) == txp.pages[1]


def test_catalog_only_on_request(env):
    gw, clk, _ = env
    ph = _paired(gw)
    gw.set_catalog({"Demo": ["Winken", "Zeigen"], "Ablauf A": ["A_1"]})
    gw.publish_status({"txt": "x"})
    assert "cat" not in ph.last and ph.last["ph"]
    h = ph.last["ph"]
    ph.signed({"t": "cat"})                              # Handy fragt nach
    assert ph.last["cat"]["g"]["Demo"] == ["Winken", "Zeigen"] and ph.last["cat"]["h"] == h
    for _ in range(G.CATALOG_SENDS - 1):
        gw.publish_status({"txt": "x"})
        assert "cat" in ph.last
    gw.publish_status({"txt": "x"})
    assert "cat" not in ph.last                         # danach wieder schlank
    gw.set_catalog({"Demo": ["Winken"]})
    gw.publish_status({"txt": "x"})
    assert ph.last["ph"] != h                           # neuer Stand -> neuer Hash
    assert gw.catalog_names() == {"Winken"}


def test_catalog_cut_and_fits_pages(env):
    gw, clk, _ = env
    ph = _paired(gw)
    gw.set_catalog({f"Kategorie {i}": [f"Pose_{i}_{k}_langer_name" for k in range(10)] for i in range(30)})
    gw.publish_status({"txt": "Status " * 20, "pad": {"t": "x" * 80}})
    ph.signed({"t": "cat"})
    assert ph.last["cat"]["cut"] is True
    assert len(json.dumps(ph.tx, ensure_ascii=False).encode()) <= P.TX_MAX

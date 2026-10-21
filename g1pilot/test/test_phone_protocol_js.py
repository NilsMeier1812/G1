#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""web/proto.js (Handy) und protocol.py (PC) bilden dieselben Schluessel und
Signaturen -- und ein kompletter Ablauf Kopplung -> Anmeldung -> Befehl mit
dem echten Gateway gelingt mit dem JS-Code. Braucht node (>= 20)."""
import json
import os
import shutil
import subprocess

import pytest

from g1pilot.teleoperation.phone import gateway as G
from g1pilot.teleoperation.phone import protocol as P

NODE = shutil.which("node")
WEB = os.path.join(os.path.dirname(__file__), "..", "g1pilot", "teleoperation", "phone", "web")
pytestmark = pytest.mark.skipif(NODE is None, reason="node fehlt")


def _js(script):
    """ES-Modul-Schnipsel mit Zugriff auf proto.js ausfuehren, JSON zurueck."""
    proto = os.path.abspath(os.path.join(WEB, "proto.js"))
    code = f"import * as P from 'file://{proto}';\n{script}"
    out = subprocess.run([NODE, "--input-type=module", "-e", code],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout)


def test_vectors_match_python():
    code, dev, name, np_, nc, ns = "AB12-CD34-EF56-GH78", "d0d0", "Pixel 8", "aa11", "bb22", "cc33"
    dk = P.device_key(code, dev, np_)
    sk = P.session_key(dk, nc, ns)
    js = _js(f"""
const code = {json.dumps(code)}, dev = {json.dumps(dev)}, name = {json.dumps(name)};
const np = {json.dumps(np_)}, nc = {json.dumps(nc)}, ns = {json.dumps(ns)};
const dk = await P.deviceKey(code, dev, np);
const sk = await P.sessionKey(dk, nc, ns);
console.log(JSON.stringify({{
  norm: P.normalizeCode("ab12 cd34-ef56 gh78"),
  pair: await P.pairMac(code, dev, name, np),
  paired: await P.pairedMac(code, dev, np),
  dk: P.hex(dk), srv: await P.serverProof(dk, dev, nc, ns),
  cli: await P.clientProof(dk, dev, nc, ns), sk: P.hex(sk),
  signed: await P.encodeSigned(sk, "c2s", 7, {{t: "hb"}}),
}}));
""")
    assert js["norm"] == P.normalize_code(code)
    assert js["pair"] == P.pair_mac(code, dev, name, np_)
    assert js["paired"] == P.paired_mac(code, dev, np_)
    assert js["dk"] == dk.hex()
    assert js["srv"] == P.server_proof(dk, dev, nc, ns)
    assert js["cli"] == P.client_proof(dk, dev, nc, ns)
    assert js["sk"] == sk.hex()
    assert P.check_signed(sk, "c2s", js["signed"]) == (7, {"t": "hb"})


def test_js_verifies_python_signature_and_pages():
    sk = bytes(range(32))
    line = P.encode_signed(sk, "s2c", 3, {"q": 1, "txt": "NOT-HALT äöü"})
    pages = [p.decode("utf-8", "surrogateescape") for p in P.tx_pages(9, ("x" * 700).encode())]
    js = _js(f"""
const sk = P.unhex({json.dumps(sk.hex())});
const ok = await P.checkSigned(sk, "s2c", {json.dumps(line)});
const bad = await P.checkSigned(sk, "s2c", {json.dumps(line.replace("äöü", "abc"))});
const enc = new TextEncoder();
const pages = {json.dumps(pages)}.map((p) => P.parsePage(enc.encode(p)));
const joined = P.joinPages(pages);
const mixed = P.joinPages([pages[0], {{...pages[1], n: 10}}]);
console.log(JSON.stringify({{ok, bad, n: joined.n, len: joined.text.length, mixed}}));
""")
    assert js["ok"] == {"c": 3, "obj": {"q": 1, "txt": "NOT-HALT äöü"}}
    assert js["bad"] is None and js["mixed"] is None
    assert js["n"] == 9 and js["len"] == 700


def test_full_flow_js_phone_with_python_gateway(tmp_path):
    """JS erzeugt pair/auth/cmd-Zeilen, Python-Gateway prueft sie."""
    gw = G.PhoneGateway(store_path=str(tmp_path / "d.json"))
    txs = []
    gw.on_tx = lambda d: txs.append(json.loads(d))
    gw.publish_status({"x": 1})
    code = gw.start_pairing(G.ROLE_OPERATE)
    pair = _js(f"""
const np = "1234abcd";
console.log(JSON.stringify({{t: "pair", dev: "js1", name: "JS", np,
  mac: await P.pairMac({json.dumps(code)}, "js1", "JS", np)}}));
""")
    gw.handle_line("L", json.dumps(pair))
    assert txs[-1]["pre"]["t"] == "paired" and "js1" in gw.devices
    gw.handle_line("L", json.dumps({"t": "hello", "dev": "js1", "nc": "n0n0"}))
    chal = txs[-1]["pre"]
    js = _js(f"""
const dk = await P.deviceKey({json.dumps(code)}, "js1", "1234abcd");
const srvOk = (await P.serverProof(dk, "js1", "n0n0", {json.dumps(chal['ns'])})) === {json.dumps(chal['srv'])};
const cli = await P.clientProof(dk, "js1", "n0n0", {json.dumps(chal['ns'])});
const sk = await P.sessionKey(dk, "n0n0", {json.dumps(chal['ns'])});
const take = await P.encodeSigned(sk, "c2s", 1, {{t: "cmd", a: "take", s: {gw.status_seq}, i: "t1"}});
const estop = await P.encodeSigned(sk, "c2s", 2, {{t: "cmd", a: "estop", s: 0, i: "e1"}});
console.log(JSON.stringify({{srvOk, auth: {{t: "auth", dev: "js1", cli}}, take, estop, sk: P.hex(sk)}}));
""")
    assert js["srvOk"] is True
    gw.handle_line("L", json.dumps(js["auth"]))
    assert gw.session is not None
    gw.handle_line("L", js["take"])
    gw.handle_line("L", js["estop"])
    actions, _ = gw.tick()
    assert gw.has_control() and [a["action"] for a in actions] == ["estop"]
    got = P.check_signed(bytes.fromhex(js["sk"]), "s2c", txs[-1]["a"])
    assert got is not None and got[1]["me"]["ctl"] is True

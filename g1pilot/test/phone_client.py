#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Python-Nachbau der Handy-Seite (web/app.js) fuer Tests: koppeln, anmelden,
Herzschlag, Befehle. Liest die TX-Antworten direkt vom Gateway."""
import json
import secrets

from g1pilot.teleoperation.phone import protocol as P


class FakePhone:
    def __init__(self, gw, link="dev_A", name="Testhandy", dev=None):
        self.gw, self.link, self.name = gw, link, name
        self.dev = dev or secrets.token_hex(8)
        self.dkey = None
        self.skey = None
        self.c = 0
        self.c_in = 0
        self.last = None        # letztes entschluesseltes s2c-Dokument
        self.tx = None
        gw.on_tx = self._on_tx

    def _on_tx(self, data):
        self.tx = json.loads(data.decode("utf-8"))
        if self.tx.get("a") and self.skey is not None:
            got = P.check_signed(self.skey, "s2c", self.tx["a"])
            assert got is not None, "PC-Signatur ungueltig"
            c, body = got
            assert c > self.c_in
            self.c_in = c
            self.last = body

    def pre(self):
        p = (self.tx or {}).get("pre")
        return p if p and p.get("to") == self.dev else None

    def send(self, obj):
        self.gw.handle_line(self.link, json.dumps(obj))

    def pair(self, code):
        np_ = P.nonce()
        self.send({"t": "pair", "dev": self.dev, "name": self.name, "np": np_,
                   "mac": P.pair_mac(code, self.dev, self.name, np_)})
        p = self.pre()
        if p and p["t"] == "paired" and p["mac"] == P.paired_mac(code, self.dev, np_):
            self.dkey = P.device_key(code, self.dev, np_)
            return True
        return False

    def login(self):
        nc = P.nonce()
        self.send({"t": "hello", "dev": self.dev, "nc": nc})
        p = self.pre()
        if not p or p["t"] != "chal":
            return p["t"] if p else None
        assert p["srv"] == P.server_proof(self.dkey, self.dev, nc, p["ns"]), "PC nicht echt"
        self.skey = P.session_key(self.dkey, nc, p["ns"])
        self.c = 0
        self.c_in = 0
        self.send({"t": "auth", "dev": self.dev, "cli": P.client_proof(self.dkey, self.dev, nc, p["ns"])})
        return "ok" if self.gw.session and self.gw.session.dev == self.dev else (self.pre() or {}).get("t")

    def signed(self, obj):
        self.c += 1
        line = P.encode_signed(self.skey, "c2s", self.c, obj)
        self.gw.handle_line(self.link, line)
        return line

    def hb(self):
        return self.signed({"t": "hb"})

    def cmd(self, action, arg=None, seq=None):
        cid = secrets.token_hex(4)
        if seq is None:
            seq = self.last["q"] if self.last else 0
        self.signed({"t": "cmd", "a": action, "x": arg, "s": seq, "i": cid})
        return cid

    def result(self, cid):
        for r in (self.last or {}).get("r", []):
            if r["i"] == cid:
                return r
        return None

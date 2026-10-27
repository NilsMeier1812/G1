#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Handy-Gateway: Kopplung, Sitzung, Rechte, Herzschlag und Steuerhoheit.

Ohne Qt und ohne Bluetooth -- der Transport (ble_server.py) ruft
handle_line() aus seinem Thread, die Demo-GUI (link.py) ruft tick() und
publish_status() aus dem Qt-Thread. Alles hinter einem Lock.

Regeln (docs/12-A-Handy.md):
  * Nur gekoppelte Geraete. Kopplung nur, solange die PC-GUI einen Code zeigt.
  * EINE Sitzung gleichzeitig. Ein anderes Geraet kommt erst dran, wenn die
    alte Sitzung verfallen ist (SESSION_TIMEOUT_S ohne gueltige Nachricht).
  * Rollen: "view" (Ansicht, NOT-HALT optional) | "operate" (Bediener).
  * Bedienen braucht die Steuerhoheit ("take"). Sie faellt weg nach
    CONTROL_TIMEOUT_S ohne Herzschlag, bei neuer Sitzung, oder wenn die PC-GUI
    sie zurueckholt. Dann meldet tick() "control_lost" -> GUI bricht vom
    Handy gestartete Bewegungen ab.
  * Befehle muessen sich auf einen frischen Status beziehen (STATUS_MAX_LAG
    Statusnummern) -> ein nach Neuladen/Aussetzer verspaeteter Befehl verpufft.
  * NOT-HALT und "Bewegung stoppen" gehen immer (keine Statuspruefung).
  * Posenliste (Katalog) geht nur auf Anfrage mit ({"t": "cat"}): sie kaeme sonst
    in jedem Status mit und machte das Lesen ueber BLE langsam. Jeder Status traegt
    nur ihren Hash "ph"; das Handy fragt nach, wenn sein Stand nicht passt.
"""

import json
import os
import zlib
import secrets
import threading
import time
from collections import deque

from g1pilot.teleoperation.phone import protocol as P

ROLE_VIEW = "view"
ROLE_OPERATE = "operate"
ROLES = (ROLE_VIEW, ROLE_OPERATE)
ROLE_TITLE = {ROLE_VIEW: "Ansicht", ROLE_OPERATE: "Bediener"}

PAIRING_TIMEOUT_S = 120.0     # so lange gilt ein angezeigter Kopplungscode
PAIRING_MAX_FAILS = 3         # falsche Versuche -> Code ungueltig
HELLO_TIMEOUT_S = 5.0         # chal -> auth
AUTH_MAX_FAILS = 5            # je Geraet, dann AUTH_LOCK_S gesperrt
AUTH_LOCK_S = 60.0
CONTROL_TIMEOUT_S = 2.0       # ohne Herzschlag -> Steuerhoheit weg
SESSION_TIMEOUT_S = 4.0       # ohne gueltige Nachricht -> Sitzung weg
# Statusnummern (3 s bei 5 Hz). Das Handy sperrt selbst schon nach 1,5 s ohne frischen
# Status ("Daten alt"); das hier faengt nur unterwegs haengengebliebene Befehle ab.
# Mit 1 s (frueher 5) wurden gueltige Befehle ueber langsames BLE oft verworfen.
STATUS_MAX_LAG = 15
MAX_NAME = 32
MAX_ARG = 96                  # Zeichen; Pose speichern: "maske|kategorie|name"
CATALOG_MAX = 1100            # Byte JSON; groesser -> hintere Kategorien weg ("cut")
CATALOG_SENDS = 3             # so viele TX-Werte tragen den Katalog nach einer Anfrage

# Aktion -> (Mindestrolle, braucht Steuerhoheit, prueft Statusbezug)
ACTIONS = {
    "estop":   (ROLE_VIEW, False, False),
    "cancel":  (ROLE_OPERATE, False, False),
    "release": (ROLE_VIEW, False, False),
    "take":    (ROLE_OPERATE, False, True),
    "ack":     (ROLE_OPERATE, True, True),
    "loco":    (ROLE_OPERATE, True, True),
    "mode":    (ROLE_OPERATE, True, True),
    "hand":    (ROLE_OPERATE, True, True),
    "seq":     (ROLE_OPERATE, True, True),
    "seq_next": (ROLE_OPERATE, True, True),
    "seq_mode": (ROLE_OPERATE, True, False),
    "safe":    (ROLE_OPERATE, True, True),
    "home":    (ROLE_OPERATE, True, True),
    "pose":    (ROLE_OPERATE, True, True),
    "pose_save": (ROLE_OPERATE, True, False),     # bewegt nichts -> kein Statusbezug
    "move":    (ROLE_OPERATE, True, True),
}
# Aktionen, die die GUI selbst ausfuehrt (take/release erledigt das Gateway).
GUI_ACTIONS = tuple(a for a in ACTIONS if a not in ("take", "release"))


def default_store_path():
    base = os.environ.get("G1_PHONE_STORE")
    if base:
        return base
    return os.path.join(os.path.expanduser("~"), ".config", "g1pilot", "phone_devices.json")


class _Session:
    def __init__(self, dev, name, role, link, skey, now):
        self.dev, self.name, self.role, self.link = dev, name, role, link
        self.skey = skey
        self.c_in = 0           # letzter Zaehler vom Handy
        self.c_out = 0          # eigener Zaehler
        self.last_rx = now      # letzte gueltige Nachricht
        self.last_hb = now
        self.control = False


class PhoneGateway:
    def __init__(self, store_path=None, clock=time.monotonic,
                 estop_for_view=True, on_tx=None, log=None):
        self._lock = threading.RLock()
        self._clock = clock
        self.store_path = store_path or default_store_path()
        self.estop_for_view = bool(estop_for_view)
        self.on_tx = on_tx          # callable(bytes): neuer TX-Wert (Transport-Thread-sicher!)
        self._log = log or (lambda text: None)
        self.devices = {}           # dev -> {"name", "role", "key"}
        self._pairing = None        # {"code", "role", "expires", "fails"}
        self._hello = {}            # dev -> (nc, ns, link, expires)
        self._fails = {}            # dev -> (anzahl, gesperrt_bis)
        self.session = None
        self._actions = deque()
        self._events = deque()
        self._results = deque(maxlen=8)
        self._seen_ids = deque(maxlen=64)
        self._pre = None            # letzte Antwort vor der Anmeldung (an "to": dev)
        self._status = {}
        self.status_seq = 0
        self._catalog = {"g": {}, "cut": False}
        self._catalog_hash = self._hash(self._catalog)
        self._catalog_left = 0
        self._tx_n = 0
        self._load()

    # ── Geraeteliste ────────────────────────────────────────────────────
    def _load(self):
        try:
            with open(self.store_path, encoding="utf-8") as f:
                data = json.load(f)
            for dev, d in data.get("devices", {}).items():
                if d.get("role") in ROLES and d.get("key"):
                    self.devices[dev] = {"name": str(d.get("name", "?"))[:MAX_NAME],
                                         "role": d["role"], "key": d["key"]}
        except (OSError, ValueError):
            pass

    def _save(self):
        try:
            os.makedirs(os.path.dirname(self.store_path), exist_ok=True)
            tmp = self.store_path + ".tmp"
            fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump({"version": P.PROTO_VERSION, "devices": self.devices}, f, indent=1)
            os.replace(tmp, self.store_path)
        except OSError as e:
            self._log(f"Handy: Geraeteliste nicht speicherbar ({e}) -- Kopplung gilt nur bis zum Neustart.")

    def list_devices(self):
        with self._lock:
            return [{"dev": d, "name": v["name"], "role": v["role"]}
                    for d, v in sorted(self.devices.items(), key=lambda kv: kv[1]["name"])]

    def remove_device(self, dev):
        with self._lock:
            if self.devices.pop(dev, None) is None:
                return False
            self._save()
            if self.session and self.session.dev == dev:
                self._end_session("Geraet entfernt")
            return True

    # ── Kopplung (PC-GUI) ───────────────────────────────────────────────
    def start_pairing(self, role=ROLE_OPERATE):
        if role not in ROLES:
            raise ValueError(role)
        with self._lock:
            code = P.new_code()
            self._pairing = {"code": code, "role": role,
                             "expires": self._clock() + PAIRING_TIMEOUT_S, "fails": 0}
            return code

    def cancel_pairing(self):
        with self._lock:
            self._pairing = None

    @property
    def pairing_active(self):
        with self._lock:
            return self._pairing is not None

    # ── Eingang (Transport-Thread) ──────────────────────────────────────
    def handle_line(self, link, line):
        with self._lock:
            now = self._clock()
            if line.startswith("A"):
                self._handle_signed(link, line, now)
                return
            try:
                msg = json.loads(line)
            except ValueError:
                return
            if not isinstance(msg, dict):
                return
            t = msg.get("t")
            if t == "pair":
                self._handle_pair(msg, now)
            elif t == "hello":
                self._handle_hello(link, msg, now)
            elif t == "auth":
                self._handle_auth(link, msg, now)

    def link_lost(self, link):
        """Transport meldet: Verbindung weg (optional, Timeouts greifen ohnehin)."""
        with self._lock:
            if self.session and self.session.link == link:
                self._end_session("Verbindung getrennt")

    def _pre_reply(self, dev, **kw):
        self._pre = dict(kw, to=dev)
        self._push_tx()

    def _handle_pair(self, msg, now):
        dev, name, np_, m = (str(msg.get(k, "")) for k in ("dev", "name", "np", "mac"))
        name = name.strip()[:MAX_NAME] or "Handy"
        pr = self._pairing
        if not dev or not np_ or len(dev) > 64:
            return
        if pr is None or now > pr["expires"]:
            self._pairing = None
            self._pre_reply(dev, re=np_, t="pair_fail", m="Am PC ist gerade keine Kopplung offen.")
            return
        if not secrets.compare_digest(m, P.pair_mac(pr["code"], dev, name, np_)):
            pr["fails"] += 1
            if pr["fails"] >= PAIRING_MAX_FAILS:
                self._pairing = None
                self._events.append(("pair_failed", "Kopplung abgebrochen: zu viele falsche Codes."))
            self._pre_reply(dev, re=np_, t="pair_fail", m="Code falsch.")
            return
        self.devices[dev] = {"name": name, "role": pr["role"],
                             "key": P.device_key(pr["code"], dev, np_).hex()}
        self._save()
        self._pre_reply(dev, re=np_, t="paired", role=pr["role"], mac=P.paired_mac(pr["code"], dev, np_))
        self._pairing = None
        self._events.append(("paired", f"Handy »{name}« gekoppelt ({ROLE_TITLE[pr['role']]})."))
        self._log(f"Handy gekoppelt: {name} ({dev}), Rolle {pr['role']}")

    def _locked(self, dev, now):
        n, until = self._fails.get(dev, (0, 0.0))
        return now < until

    def _fail(self, dev, now):
        n, until = self._fails.get(dev, (0, 0.0))
        n += 1
        self._fails[dev] = (0, now + AUTH_LOCK_S) if n >= AUTH_MAX_FAILS else (n, until)

    def _handle_hello(self, link, msg, now):
        dev, nc = str(msg.get("dev", "")), str(msg.get("nc", ""))
        d = self.devices.get(dev)
        if d is None:
            self._pre_reply(dev, re=nc, t="unknown", m="Handy ist nicht (mehr) gekoppelt.")
            return
        if self._locked(dev, now) or not nc or len(nc) > 64:
            return
        s = self.session
        if s and s.dev != dev and now - s.last_rx < SESSION_TIMEOUT_S:
            self._pre_reply(dev, re=nc, t="busy", m=f"Gerade ist »{s.name}« verbunden.")
            return
        ns = P.nonce()
        self._hello[dev] = (nc, ns, link, now + HELLO_TIMEOUT_S)
        key = bytes.fromhex(d["key"])
        self._pre_reply(dev, re=nc, t="chal", ns=ns, srv=P.server_proof(key, dev, nc, ns))

    def _handle_auth(self, link, msg, now):
        dev, cli = str(msg.get("dev", "")), str(msg.get("cli", ""))
        h = self._hello.pop(dev, None)
        d = self.devices.get(dev)
        if h is None or d is None or self._locked(dev, now):
            return
        nc, ns, hlink, expires = h
        key = bytes.fromhex(d["key"])
        if now > expires or hlink != link or not secrets.compare_digest(
                cli, P.client_proof(key, dev, nc, ns)):
            self._fail(dev, now)
            self._pre_reply(dev, re=nc, t="auth_fail", m="Anmeldung abgelehnt.")
            return
        self._fails.pop(dev, None)
        s = self.session
        if s and s.dev != dev and now - s.last_rx < SESSION_TIMEOUT_S:
            self._pre_reply(dev, re=nc, t="busy", m=f"Gerade ist »{s.name}« verbunden.")
            return
        if s is not None:
            self._end_session("neue Anmeldung")
        self.session = _Session(dev, d["name"], d["role"], link,
                                P.session_key(key, nc, ns), now)
        self._results.clear()
        self._pre = None
        self._events.append(("connected", f"Handy »{d['name']}« verbunden ({ROLE_TITLE[d['role']]})."))
        self._log(f"Handy-Sitzung: {d['name']} ({dev})")
        self._push_tx()

    def _handle_signed(self, link, line, now):
        s = self.session
        if s is None or s.link != link:
            return
        got = P.check_signed(s.skey, "c2s", line)
        if got is None:
            return
        c, msg = got
        if c <= s.c_in or not isinstance(msg, dict):
            return                          # Wiederholung / alter Rahmen
        s.c_in = c
        s.last_rx = now
        t = msg.get("t")
        if t == "hb":
            s.last_hb = now
        elif t == "bye":
            self._end_session("Handy hat getrennt")
        elif t == "cat":
            self._catalog_left = CATALOG_SENDS
            self._push_tx()
        elif t == "cmd":
            s.last_hb = now
            self._handle_cmd(s, msg)

    def _handle_cmd(self, s, msg):
        action = str(msg.get("a", ""))
        cid = str(msg.get("i", ""))[:32]
        if cid and cid in self._seen_ids:
            return
        if cid:
            self._seen_ids.append(cid)
        spec = ACTIONS.get(action)
        if spec is None:
            self._result(cid, False, "Unbekannter Befehl.")
            return
        min_role, need_control, check_seq = spec
        if action == "estop" and not self.estop_for_view:
            min_role = ROLE_OPERATE
        if min_role == ROLE_OPERATE and s.role != ROLE_OPERATE:
            self._result(cid, False, "Dieses Handy darf nur ansehen.")
            return
        if need_control and not s.control:
            self._result(cid, False, "Erst »Bedienung übernehmen«.")
            return
        if check_seq:
            try:
                seq = int(msg.get("s", -1))
            except (TypeError, ValueError):
                seq = -1
            if seq < 0 or seq > self.status_seq or self.status_seq - seq > STATUS_MAX_LAG:
                self._result(cid, False, "Anzeige war nicht aktuell -- nichts ausgeführt.")
                return
        if action == "take":
            s.control = True
            s.last_hb = self._clock()
            self._events.append(("control", f"Handy »{s.name}« bedient jetzt."))
            self._result(cid, True, "Bedienung übernommen.")
            return
        if action == "release":
            if s.control:
                s.control = False
                self._events.append(("control_lost", f"Handy »{s.name}« hat die Bedienung abgegeben."))
            self._result(cid, True, "Bedienung abgegeben.")
            return
        arg = msg.get("x")
        if arg is not None and not isinstance(arg, (str, int, float, bool)):
            arg = None
        if isinstance(arg, str):
            arg = arg[:MAX_ARG]
        self._actions.append({"id": cid, "action": action, "arg": arg, "name": s.name})

    # ── GUI-Seite (Qt-Thread) ───────────────────────────────────────────
    def tick(self):
        """Timeouts pruefen. -> (actions, events): auszufuehrende Befehle und
        Hinweise [(art, text)], art in paired/connected/control/control_lost/
        disconnected/pair_failed/pair_expired."""
        with self._lock:
            now = self._clock()
            pr = self._pairing
            if pr is not None and now > pr["expires"]:
                self._pairing = None
                self._events.append(("pair_expired", "Kopplungscode abgelaufen."))
            for dev in [d for d, h in self._hello.items() if now > h[3]]:
                self._hello.pop(dev, None)
            s = self.session
            if s is not None:
                if s.control and now - s.last_hb > CONTROL_TIMEOUT_S:
                    s.control = False
                    self._events.append(("control_lost",
                                         f"Handy »{s.name}«: kein Herzschlag -- Bedienung entzogen."))
                if now - s.last_rx > SESSION_TIMEOUT_S:
                    self._end_session("keine Verbindung")
            actions, events = list(self._actions), list(self._events)
            self._actions.clear()
            self._events.clear()
            return actions, events

    def _end_session(self, reason):
        s = self.session
        if s is None:
            return
        if s.control:
            self._events.append(("control_lost", f"Handy »{s.name}«: {reason} -- Bedienung entzogen."))
        self._events.append(("disconnected", f"Handy »{s.name}« getrennt ({reason})."))
        self.session = None
        self._push_tx()

    def revoke_control(self):
        """PC-GUI holt die Bedienung zurueck."""
        with self._lock:
            s = self.session
            if s and s.control:
                s.control = False
                self._events.append(("control_lost", f"PC hat die Bedienung von »{s.name}« zurückgeholt."))
                self._push_tx()

    def has_control(self):
        with self._lock:
            return bool(self.session and self.session.control)

    def state(self):
        """Fuer die PC-Anzeige: (verbunden, name, rolle, bedient)."""
        with self._lock:
            s = self.session
            if s is None:
                return False, None, None, False
            return True, s.name, s.role, s.control

    def result(self, cid, ok, text):
        """Ergebnis eines GUI-Befehls (wird mit dem naechsten Status gesendet)."""
        with self._lock:
            self._result(cid, ok, text)

    def _result(self, cid, ok, text):
        self._results.append({"i": cid, "ok": bool(ok), "m": str(text)[:120]})
        self._push_tx()

    def publish_status(self, status):
        with self._lock:
            self._status = status
            self.status_seq += 1
            self._push_tx()

    @staticmethod
    def _hash(obj):
        raw = json.dumps(obj, sort_keys=True, ensure_ascii=False).encode("utf-8")
        return format(zlib.crc32(raw), "08x")

    def set_catalog(self, grouped, sequences=None, moves=None, empty=None):
        """Posenliste {kategorie: [namen]} (Qt-Thread), dazu die Ablauf-Namen
        ("s" startbar, "e" noch leer -- nur zum Anhaengen) und die relativen
        Bewegungen ("m"); diese Listen sind zuerst eingeplant. Zu gross -> hintere Kategorien weg,
        "cut" gesetzt. Neuer Stand -> neuer Hash im Status."""
        seqs = [str(n)[:40] for n in (sequences or [])][:20]
        mv = [str(n)[:40] for n in (moves or [])][:20]
        se = [str(n)[:40] for n in (empty or [])][:10]
        cat, cut = {}, False
        for k, names in grouped.items():
            trial = dict(cat)
            trial[str(k)[:40]] = [str(n)[:60] for n in names][:40]
            if len(json.dumps({"g": trial, "s": seqs, "m": mv, "e": se}, ensure_ascii=False,
                              separators=(",", ":")).encode()) > CATALOG_MAX:
                cut = True
                break
            cat = trial
        new = {"g": cat, "cut": cut}
        if sequences is not None:
            new["s"] = seqs
        if moves is not None:
            new["m"] = mv
        if empty is not None:
            new["e"] = se
        with self._lock:
            if new != self._catalog:
                self._catalog = new
                self._catalog_hash = self._hash(new)

    def catalog_names(self):
        with self._lock:
            return {n for names in self._catalog["g"].values() for n in names}

    def refresh_tx(self):
        """TX-Wert neu senden (Transport gerade gestartet)."""
        with self._lock:
            self._push_tx()

    # ── TX-Wert bauen ───────────────────────────────────────────────────
    def _push_tx(self):
        self._tx_n += 1
        doc = {"v": P.PROTO_VERSION, "n": self._tx_n, "pre": self._pre, "a": None}
        s = self.session
        if s is not None:
            s.c_out += 1
            body = {"q": self.status_seq, "st": self._status,
                    "me": {"name": s.name, "role": s.role, "ctl": s.control},
                    "r": list(self._results),
                    "lim": {"ctl_s": CONTROL_TIMEOUT_S, "lag": STATUS_MAX_LAG},
                    "ph": self._catalog_hash}
            if self._catalog_left > 0:
                self._catalog_left -= 1
                body["cat"] = dict(self._catalog, h=self._catalog_hash)
            doc["a"] = P.encode_signed(s.skey, "s2c", s.c_out, body)
        data = json.dumps(doc, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        if len(data) > P.TX_MAX and s is not None:
            # Zu gross fuer die TX-Seiten: erst den Status verschlanken (Temperaturen,
            # Info, langer Text), notfalls auch den Katalog weglassen.
            body["st"] = {k: v for k, v in self._status.items() if k not in ("tmp", "info")}
            body["st"]["txt"] = str(self._status.get("txt", ""))[:60]
            body["st"]["slim"] = True
            for drop in (None, "cat"):
                if drop:
                    body.pop(drop, None)
                doc["a"] = P.encode_signed(s.skey, "s2c", s.c_out, body)
                data = json.dumps(doc, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
                if len(data) <= P.TX_MAX:
                    break
        if self.on_tx is not None:
            try:
                self.on_tx(data)
            except Exception as e:  # noqa: BLE001
                self._log(f"Handy: TX fehlgeschlagen ({e})")
        return data

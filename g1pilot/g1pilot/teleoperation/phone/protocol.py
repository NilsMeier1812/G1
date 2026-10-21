#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Protokoll Handy <-> PC (Bluetooth LE). Gegenstueck: web/proto.js -- beide
Seiten MUESSEN dieselben Strings/HMACs bilden (test_phone_protocol_js.py).

Sicherheit liegt hier, nicht in der BLE-Kopplung:

  Kopplung : Die PC-GUI zeigt einen Kopplungscode (16 Zeichen, 80 Bit). Der
             Code geht nur optisch aufs Handy, nie ueber Funk. Das Handy
             schickt {pair, dev, name, np, mac=HMAC(code, ...)}; beide Seiten
             leiten daraus den Geraete-Schluessel D ab. D wird NIE gesendet.
  Sitzung  : hello{dev, nc} -> chal{ns, srv=HMAC(D, ...)} (PC beweist D)
             -> auth{cli=HMAC(D, ...)} (Handy beweist D). Sitzungsschluessel
             S = HMAC(D, "ses|nc|ns").
  Nachricht: jede Zeile Handy->PC "A<zaehler>.<mac>.<json>" mit streng
             steigendem Zaehler (kein Wiederholen), jede PC-Antwort ebenso
             signiert. Inhalt ist lesbar (nicht geheim), aber nicht faelschbar.

Rahmen: Handy->PC schreibt Zeilen mit "\n" (lange Writes kommen bei BlueZ ggf.
in Stuecken mit offset an -> erst bei "\n" auswerten). PC->Handy: lesbares
Merkmal TX (JSON, lange Reads), Notify nur ein kurzer Zaehler "aendert sich".
"""

import hashlib
import hmac
import json
import secrets

# Eigene 128-Bit-UUIDs (zufaellig erzeugt, nur fuer G1-Handy).
SERVICE_UUID = "6b1e0001-6d1f-4c3e-9b1a-5f0e7c3a2b10"
RX_UUID = "6b1e0002-6d1f-4c3e-9b1a-5f0e7c3a2b10"     # Handy -> PC (write)
NOTIFY_UUID = "6b1e0004-6d1f-4c3e-9b1a-5f0e7c3a2b10"  # PC -> Handy: kurzer Stand-Zaehler (notify)
# PC -> Handy: TX-Seiten (read), Seite 0 zusaetzlich notify. Ein GATT-Wert
# darf hoechstens 512 Byte haben -> der TX-Text wird auf bis zu 4 Seiten
# verteilt, jede mit Kopf "<n>:<i>:<k>:" (Stand n, Seite i von k).
TX_UUIDS = tuple(f"6b1e00{i:02x}-6d1f-4c3e-9b1a-5f0e7c3a2b10" for i in range(0x10, 0x14))
TX_PAGE_BYTES = 480
TX_MAX = TX_PAGE_BYTES * len(TX_UUIDS) - 64

PROTO_VERSION = 1
MAC_HEX = 32                 # 128 Bit HMAC-SHA256, gekuerzt
CODE_ALPHABET = "0123456789ABCDEFGHJKMNPQRSTVWXYZ"   # Crockford Base32 (ohne I L O U)
CODE_LEN = 16                # 16 * 5 Bit = 80 Bit
MAX_LINE = 2048              # laengere Zeilen werden verworfen


def new_code():
    """Kopplungscode, 16 Zeichen in Vierergruppen (Anzeige)."""
    raw = "".join(secrets.choice(CODE_ALPHABET) for _ in range(CODE_LEN))
    return "-".join(raw[i:i + 4] for i in range(0, CODE_LEN, 4))


def normalize_code(code):
    """Eingabe tolerant machen: Bindestriche/Leerzeichen weg, gross, O->0, I/L->1."""
    c = "".join(ch for ch in str(code).upper() if ch.isalnum())
    return c.replace("O", "0").replace("I", "1").replace("L", "1")


def nonce():
    return secrets.token_hex(8)


def _hm(key, text):
    return hmac.new(key, text.encode("utf-8"), hashlib.sha256)


def mac(key, text):
    """128-Bit-HMAC als Hex (Signaturen)."""
    return _hm(key, text).hexdigest()[:MAC_HEX]


def kdf(key, text):
    """Volle 256 Bit (Schluessel ableiten)."""
    return _hm(key, text).digest()


def code_key(code):
    return normalize_code(code).encode("ascii")


def pair_mac(code, dev, name, np):
    return mac(code_key(code), f"pair|{dev}|{name}|{np}")


def paired_mac(code, dev, np):
    return mac(code_key(code), f"paired|{dev}|{np}")


def device_key(code, dev, np):
    return kdf(code_key(code), f"dev|{dev}|{np}")


def server_proof(dkey, dev, nc, ns):
    return mac(dkey, f"srv|{dev}|{nc}|{ns}")


def client_proof(dkey, dev, nc, ns):
    return mac(dkey, f"cli|{dev}|{nc}|{ns}")


def session_key(dkey, nc, ns):
    return kdf(dkey, f"ses|{nc}|{ns}")


def sign(skey, direction, counter, payload):
    """direction 'c2s' (Handy->PC) | 's2c' (PC->Handy)."""
    return mac(skey, f"{direction}|{counter}|{payload}")


def encode_signed(skey, direction, counter, obj):
    payload = json.dumps(obj, separators=(",", ":"), ensure_ascii=False)
    return f"A{counter}.{sign(skey, direction, counter, payload)}.{payload}"


def split_signed(line):
    """'A<c>.<mac>.<payload>' -> (c, mac, payload) oder None."""
    if not line.startswith("A"):
        return None
    try:
        c_txt, m, payload = line[1:].split(".", 2)
        return int(c_txt), m, payload
    except ValueError:
        return None


def check_signed(skey, direction, line):
    """-> (counter, obj) bei gueltiger Signatur, sonst None."""
    parts = split_signed(line)
    if parts is None:
        return None
    c, m, payload = parts
    if not hmac.compare_digest(m, sign(skey, direction, c, payload)):
        return None
    try:
        return c, json.loads(payload)
    except ValueError:
        return None


def tx_pages(n, data):
    """TX-Text (bytes) -> Liste der Seitenwerte (bytes) mit Kopf."""
    chunks = [data[i:i + TX_PAGE_BYTES] for i in range(0, len(data), TX_PAGE_BYTES)] or [b""]
    chunks = chunks[:len(TX_UUIDS)]
    k = len(chunks)
    return [f"{n}:{i}:{k}:".encode("ascii") + c for i, c in enumerate(chunks)]


def join_pages(pages):
    """Gegenstueck (Tests/Handy): -> (n, data) oder None bei gemischten Staenden."""
    out, n0 = b"", None
    for i, p in enumerate(pages):
        n, idx, k, rest = p.split(b":", 3)
        if n0 is None:
            n0, k0 = int(n), int(k)
        if int(n) != n0 or int(idx) != i or int(k) != k0:
            return None
        out += rest
    return n0, out


class TxPages:
    """Aktuelle TX-Seiten + Lese-Momentaufnahme je Geraet. Das Handy liest TX0..TXk
    nacheinander; ueber echtes BLE dauert das laenger als ein Statustakt (200 ms).
    Ohne Momentaufnahme passen die Seiten dann nie zusammen (»Daten alt«-Flackern).
    Ein Read von TX0 mit offset 0 friert den Stand fuer dieses Geraet ein, alle
    weiteren Reads (auch mit offset) kommen aus diesem Stand."""

    def __init__(self):
        self.n = 0
        self.pages = [b"0:0:1:"]
        self._frozen = {}

    def set(self, data):
        self.n += 1
        self.pages = tx_pages(self.n, data)
        return self.n

    def read(self, device, idx, offset=0):
        if (idx == 0 and offset == 0) or device not in self._frozen:
            self._frozen[device] = self.pages
        pages = self._frozen[device]
        value = pages[idx] if idx < len(pages) else b""
        return value[offset:]

    def drop(self, device):
        self._frozen.pop(device, None)


class LineBuffer:
    """Setzt BLE-Writes zu Zeilen zusammen. Ein langer Write kann bei BlueZ in
    Stuecken mit offset ankommen (Prepare/Execute Write): Stueck an offset
    einsetzen, fertige Zeilen ("\n") sofort liefern. Jeder Write mit offset 0
    beginnt neu -- das Handy schickt immer ganze Zeilen."""

    def __init__(self):
        self._cur = {}      # link -> (bytes des laufenden Writes, schon ausgewertet bis)

    def feed(self, link, data, offset=0):
        cur, done = self._cur.get(link, (b"", 0))
        data = bytes(data)
        if offset == 0:
            cur, done = data, 0
        elif offset <= len(cur):
            cur = cur[:offset] + data
            done = min(done, offset)
        else:
            self._cur.pop(link, None)
            return []
        lines = []
        while True:
            idx = cur.find(b"\n", done)
            if idx < 0:
                break
            line = cur[done:idx]
            done = idx + 1
            if line and len(line) <= MAX_LINE:
                try:
                    lines.append(line.decode("utf-8"))
                except UnicodeDecodeError:
                    pass
        if len(cur) - done > MAX_LINE:
            cur, done = b"", 0
        self._cur[link] = (cur, done)
        return lines

    def drop(self, link):
        self._cur.pop(link, None)

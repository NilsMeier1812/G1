#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Eigener QR-Erzeuger (g1pilot.utils.qr_code) fuer den Handy-Dialog.

Referenzwerte (sha256 der Matrix) stammen von python-qrcode 8.2 mit gleicher
Version/Maske. Ist ``qrcode`` installiert, vergleicht ein Test zusaetzlich
viele Faelle direkt; sonst wird nur der Teil uebersprungen."""
import hashlib

import pytest

from g1pilot.utils import qr_code as Q

URL = "https://nilsmeier1812.github.io/G1/"


def _digest(m):
    s = "\n".join("".join("1" if c else "0" for c in row) for row in m)
    return hashlib.sha256(s.encode()).hexdigest()[:16]


@pytest.mark.parametrize("text,ecl,ver,mask,digest", [
    (URL, "M", None, None, "c8543bdf9fe49a15"),      # genau das, was der Dialog zeigt
    ("G1", "H", 1, 3, "dc9d279454e6e98d"),
    ("x" * 100, "L", 5, 6, "647fb9803de0f9a1"),
    ("y" * 150, "M", 8, 2, "e7d99ad988dd6759"),      # Version >= 7: Versionsinformation
])
def test_matches_reference(text, ecl, ver, mask, digest):
    assert _digest(Q.qr_matrix(text, ecl, ver, mask)) == digest


def test_default_url_is_small_and_square():
    m = Q.qr_matrix(URL)
    assert len(m) == 29 and all(len(r) == 29 for r in m)      # Version 3


def _read_format(m):
    """Formatbits aus der Kopie oben links lesen -> (Stufe, Maske)."""
    pos = [(8, i) for i in range(6)] + [(8, 7), (8, 8), (7, 8)] + [(14 - i, 8) for i in range(9, 15)]
    bits = sum(1 << i for i, (x, y) in enumerate(pos) if m[y][x])
    for ecl in "LMQH":
        for k in range(8):
            if Q.format_bits(ecl, k) == bits:
                return ecl, k
    return None


def test_finder_patterns_and_format_copies_agree():
    m = Q.qr_matrix(URL, "M")
    n = len(m)
    finder = [[max(abs(x - 3), abs(y - 3)) not in (2, 4) for x in range(7)] for y in range(7)]
    for ox, oy in ((0, 0), (n - 7, 0), (0, n - 7)):
        assert [row[ox:ox + 7] for row in m[oy:oy + 7]] == finder
    assert _read_format(m)[0] == "M"
    # zweite Kopie (unten links + oben rechts) muss gleich sein
    pos2 = [(n - 1 - i, 8) for i in range(8)] + [(8, n - 15 + i) for i in range(8, 15)]
    bits2 = sum(1 << i for i, (x, y) in enumerate(pos2) if m[y][x])
    assert bits2 == Q.format_bits(*_read_format(m))
    assert m[n - 8][8], "dunkles Modul fehlt"


def test_too_long_raises():
    with pytest.raises(ValueError):
        Q.qr_matrix("a" * (Q.capacity(Q.MAX_VERSION, "M") + 1))
    with pytest.raises(ValueError):
        Q.qr_matrix(URL, "M", version=1)


def test_against_python_qrcode():
    qrcode = pytest.importorskip("qrcode")
    from qrcode.util import QRData
    mode = getattr(qrcode.util, "MODE_8BIT_BYTE", 4)
    levels = {"L": 1, "M": 0, "Q": 3, "H": 2}
    texts = [URL, "A", "http://192.168.123.10:8790/index.html", "Grüße äöü" * 3, "z" * 120]
    n = 0
    for t in texts:
        for ecl in "LMQH":
            for v in range(1, Q.MAX_VERSION + 1):
                if len(t.encode()) > Q.capacity(v, ecl):
                    continue
                for mk in (0, 3, 7):
                    q = qrcode.QRCode(version=v, error_correction=levels[ecl], border=0, mask_pattern=mk)
                    q.add_data(QRData(t.encode(), mode=mode))
                    q.make(fit=False)
                    assert [[bool(c) for c in r] for r in q.modules] == Q.qr_matrix(t, ecl, v, mk), (t, ecl, v, mk)
                    n += 1
    assert n > 100

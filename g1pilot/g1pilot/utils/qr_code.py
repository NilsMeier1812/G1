#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kleiner QR-Code-Erzeuger ohne Fremdpaket (nur Standardbibliothek).

Gebraucht fuer den Handy-Dialog der Demo-GUI: dort steht ein QR-Code mit der
Adresse der Handy-Seite. Ein eigenes Modul statt ``qrcode``/``segno``, damit
die Docker-Images (Sim und Real) dafuer nicht neu gebaut werden muessen.

Umfang bewusst klein: Byte-Modus (UTF-8), Fehlerkorrektur L/M/Q/H,
Versionen 1-10 (bei M bis 213 Byte, fuer eine URL reichlich). Die Maske wird
wie in ISO/IEC 18004 ueber die Strafpunkte gewaehlt.

    m = qr_matrix("https://nilsmeier1812.github.io/G1/")   # Liste von Zeilen, True = dunkel
"""

# ── Tabellen (ISO/IEC 18004, Versionen 1-10) ────────────────────────────
MAX_VERSION = 10
# Fehlerkorrektur-Stufe -> (Formatbits, EC-Codewoerter je Block, Bloecke) je Version 1..10
_ECL = {
    "L": (1, (7, 10, 15, 20, 26, 18, 20, 24, 30, 18), (1, 1, 1, 1, 1, 2, 2, 2, 2, 4)),
    "M": (0, (10, 16, 26, 18, 24, 16, 18, 22, 22, 26), (1, 1, 1, 2, 2, 4, 4, 4, 5, 5)),
    "Q": (3, (13, 22, 18, 26, 18, 24, 18, 22, 20, 24), (1, 1, 2, 2, 4, 4, 6, 6, 8, 8)),
    "H": (2, (17, 28, 22, 16, 22, 28, 26, 26, 24, 28), (1, 1, 2, 4, 4, 4, 5, 6, 8, 8)),
}

# ── Galois-Feld GF(256), Polynom 0x11D ──────────────────────────────────
_EXP = [0] * 512
_LOG = [0] * 256
_v = 1
for _i in range(255):
    _EXP[_i] = _v
    _LOG[_v] = _i
    _v <<= 1
    if _v & 0x100:
        _v ^= 0x11D
for _i in range(255, 512):
    _EXP[_i] = _EXP[_i - 255]


def _gf_mul(a, b):
    if a == 0 or b == 0:
        return 0
    return _EXP[_LOG[a] + _LOG[b]]


def _rs_generator(degree):
    """Koeffizienten (hoechster zuerst, ohne fuehrende 1) von prod (x - a^i)."""
    poly = [1]
    for i in range(degree):
        nxt = [0] * (len(poly) + 1)
        for j, c in enumerate(poly):
            nxt[j] ^= c
            nxt[j + 1] ^= _gf_mul(c, _EXP[i])
        poly = nxt
    return poly[1:]


def _rs_remainder(data, degree):
    gen = _rs_generator(degree)
    rem = [0] * degree
    for b in data:
        factor = b ^ rem[0]
        rem = rem[1:] + [0]
        for k in range(degree):
            rem[k] ^= _gf_mul(gen[k], factor)
    return rem


# ── Groessen ─────────────────────────────────────────────────────────────
def _raw_modules(ver):
    """Anzahl Module fuer Daten+EC (ohne Funktionsmuster)."""
    n = (16 * ver + 128) * ver + 64
    if ver >= 2:
        na = ver // 7 + 2
        n -= (25 * na - 10) * na - 55
        if ver >= 7:
            n -= 36
    return n


def _data_codewords(ver, ecl):
    _, ecc, blocks = _ECL[ecl]
    return _raw_modules(ver) // 8 - ecc[ver - 1] * blocks[ver - 1]


def _alignment_positions(ver):
    if ver == 1:
        return []
    na = ver // 7 + 2
    size = ver * 4 + 17
    step = (ver * 8 + na * 3 + 5) // (na * 4 - 4) * 2
    return [6] + sorted(size - 7 - i * step for i in range(na - 1))


def capacity(ver, ecl="M"):
    """Maximale Byte-Laenge fuer Version/Stufe (Byte-Modus)."""
    count_bits = 8 if ver <= 9 else 16
    return (_data_codewords(ver, ecl) * 8 - 4 - count_bits) // 8


# ── Matrix ───────────────────────────────────────────────────────────────
class _Grid:
    def __init__(self, ver):
        self.size = ver * 4 + 17
        self.mod = [[False] * self.size for _ in range(self.size)]
        self.fn = [[False] * self.size for _ in range(self.size)]

    def set_fn(self, x, y, dark):
        self.mod[y][x] = dark
        self.fn[y][x] = True


def _draw_function_patterns(g, ver):
    n = g.size
    for i in range(n):                       # Taktlinien
        g.set_fn(6, i, i % 2 == 0)
        g.set_fn(i, 6, i % 2 == 0)
    for cx, cy in ((3, 3), (n - 4, 3), (3, n - 4)):   # Suchmuster + Trennrand
        for dy in range(-4, 5):
            for dx in range(-4, 5):
                x, y = cx + dx, cy + dy
                if 0 <= x < n and 0 <= y < n:
                    d = max(abs(dx), abs(dy))
                    g.set_fn(x, y, d not in (2, 4))
    pos = _alignment_positions(ver)
    last = len(pos) - 1
    for i, ay in enumerate(pos):              # Ausrichtungsmuster
        for j, ax in enumerate(pos):
            if (i == 0 and j == 0) or (i == 0 and j == last) or (i == last and j == 0):
                continue
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    g.set_fn(ax + dx, ay + dy, max(abs(dx), abs(dy)) != 1)
    _draw_format_bits(g, "M", 0)              # Platz reservieren, spaeter richtig
    if ver >= 7:                              # Versionsinformation
        rem = ver
        for _ in range(12):
            rem = (rem << 1) ^ ((rem >> 11) * 0x1F25)
        bits = ver << 12 | rem
        for i in range(18):
            dark = (bits >> i) & 1 == 1
            a, b = n - 11 + i % 3, i // 3
            g.set_fn(a, b, dark)
            g.set_fn(b, a, dark)


def format_bits(ecl, mask):
    """15 Formatbits (mit BCH-Rest und XOR-Maske 0x5412)."""
    data = _ECL[ecl][0] << 3 | mask
    rem = data
    for _ in range(10):
        rem = (rem << 1) ^ ((rem >> 9) * 0x537)
    return (data << 10 | rem) ^ 0x5412


def _draw_format_bits(g, ecl, mask):
    bits = format_bits(ecl, mask)
    n = g.size

    def bit(i):
        return (bits >> i) & 1 == 1
    for i in range(6):
        g.set_fn(8, i, bit(i))
    g.set_fn(8, 7, bit(6))
    g.set_fn(8, 8, bit(7))
    g.set_fn(7, 8, bit(8))
    for i in range(9, 15):
        g.set_fn(14 - i, 8, bit(i))
    for i in range(8):
        g.set_fn(n - 1 - i, 8, bit(i))
    for i in range(8, 15):
        g.set_fn(8, n - 15 + i, bit(i))
    g.set_fn(8, n - 8, True)                  # immer dunkles Modul


def _codewords(data, ver, ecl):
    count_bits = 8 if ver <= 9 else 16
    cap = _data_codewords(ver, ecl) * 8
    bits = []

    def put(val, n):
        bits.extend((val >> i) & 1 for i in range(n - 1, -1, -1))
    put(0b0100, 4)                            # Byte-Modus
    put(len(data), count_bits)
    for b in data:
        put(b, 8)
    put(0, min(4, cap - len(bits)))           # Endkennung
    put(0, -len(bits) % 8)
    pad = 0xEC
    while len(bits) < cap:
        put(pad, 8)
        pad ^= 0xEC ^ 0x11
    words = [int("".join(map(str, bits[i:i + 8])), 2) for i in range(0, len(bits), 8)]

    _, ecc_t, blocks_t = _ECL[ecl]
    nblocks, ecc = blocks_t[ver - 1], ecc_t[ver - 1]
    raw = _raw_modules(ver) // 8
    nshort = nblocks - raw % nblocks
    short_len = raw // nblocks
    blocks, k = [], 0
    for i in range(nblocks):
        dlen = short_len - ecc + (0 if i < nshort else 1)
        d = words[k:k + dlen]
        k += dlen
        blocks.append((d, _rs_remainder(d, ecc)))
    out = []
    for i in range(short_len - ecc + 1):      # Daten verschraenkt
        for d, _e in blocks:
            if i < len(d):
                out.append(d[i])
    for i in range(ecc):                      # dann Fehlerkorrektur
        for _d, e in blocks:
            out.append(e[i])
    return out


def _draw_codewords(g, words):
    n = g.size
    total = len(words) * 8
    i = 0
    right = n - 1
    while right >= 1:
        if right == 6:
            right = 5
        upward = ((right + 1) & 2) == 0
        for vert in range(n):
            y = n - 1 - vert if upward else vert
            for j in range(2):
                x = right - j
                if not g.fn[y][x] and i < total:
                    g.mod[y][x] = (words[i >> 3] >> (7 - (i & 7))) & 1 == 1
                    i += 1
        right -= 2


_MASKS = (
    lambda x, y: (x + y) % 2 == 0,
    lambda x, y: y % 2 == 0,
    lambda x, y: x % 3 == 0,
    lambda x, y: (x + y) % 3 == 0,
    lambda x, y: (x // 3 + y // 2) % 2 == 0,
    lambda x, y: x * y % 2 + x * y % 3 == 0,
    lambda x, y: (x * y % 2 + x * y % 3) % 2 == 0,
    lambda x, y: ((x + y) % 2 + x * y % 3) % 2 == 0,
)


def _apply_mask(g, mask):
    f = _MASKS[mask]
    for y in range(g.size):
        for x in range(g.size):
            if not g.fn[y][x] and f(x, y):
                g.mod[y][x] = not g.mod[y][x]


def _penalty(m):
    n = len(m)
    score = 0
    lines = [row for row in m] + [[m[y][x] for y in range(n)] for x in range(n)]
    finder = [True, False, True, True, True, False, True]
    light4 = [False] * 4
    for line in lines:
        run, prev = 0, None
        for v in line:                        # Regel 1: Laeufe >= 5
            if v == prev:
                run += 1
            else:
                if run >= 5:
                    score += 3 + run - 5
                run, prev = 1, v
        if run >= 5:
            score += 3 + run - 5
        for s in range(n - 10):               # Regel 3: Suchmuster-aehnlich
            seg = line[s:s + 11]
            if seg == finder + light4 or seg == light4 + finder:
                score += 40
    for y in range(n - 1):                    # Regel 2: 2x2-Bloecke
        for x in range(n - 1):
            c = m[y][x]
            if c == m[y][x + 1] == m[y + 1][x] == m[y + 1][x + 1]:
                score += 3
    dark = sum(sum(row) for row in m)         # Regel 4: Hell/Dunkel-Verhaeltnis
    total = n * n
    score += abs(dark * 20 - total * 10) // total * 10
    return score


def qr_matrix(text, ecl="M", version=None, mask=None):
    """QR-Code als quadratische Liste von Zeilen (True = dunkles Modul), ohne Ruhezone.

    version/mask None = kleinste passende Version bzw. beste Maske.
    ValueError, wenn der Text nicht in Version 10 passt."""
    if ecl not in _ECL:
        raise ValueError(f"Fehlerkorrektur {ecl!r} unbekannt (L/M/Q/H)")
    data = text.encode("utf-8") if isinstance(text, str) else bytes(text)
    if version is None:
        version = next((v for v in range(1, MAX_VERSION + 1) if len(data) <= capacity(v, ecl)), None)
        if version is None:
            raise ValueError(f"Text zu lang fuer QR-Version {MAX_VERSION} ({len(data)} Byte)")
    elif not 1 <= version <= MAX_VERSION or len(data) > capacity(version, ecl):
        raise ValueError(f"Text passt nicht in QR-Version {version}")
    g = _Grid(version)
    _draw_function_patterns(g, version)
    _draw_codewords(g, _codewords(data, version, ecl))
    masks = range(8) if mask is None else (mask,)
    best = None
    for k in masks:
        _apply_mask(g, k)
        _draw_format_bits(g, ecl, k)
        p = _penalty(g.mod) if mask is None else 0
        if best is None or p < best[0]:
            best = (p, k, [row[:] for row in g.mod])
        _apply_mask(g, k)                     # XOR rueckgaengig
    return best[2]

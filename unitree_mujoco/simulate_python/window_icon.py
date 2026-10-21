"""Eigenes Fenster-Icon fuer den MuJoCo-Viewer (X11).

Der Viewer (mujoco.viewer, intern GLFW) setzt kein Icon, Dock/Taskleiste
zeigen dann nur ein Zahnrad. Hier wird nach dem Oeffnen das Fenster dieses
Prozesses gesucht (_NET_WM_PID) und _NET_WM_ICON gesetzt; vor dem Oeffnen
setzt prepare_env() RESOURCE_NAME, das GLFW als WM_CLASS-Instanz nimmt.

Rein kosmetisch: jeder Fehler (kein X11, keine libX11, Fenster nicht
gefunden) wird still ignoriert, die Simulation laeuft unveraendert weiter.
Icon-Quelle: g1pilot/resource/icons/g1_mujoco.svg (PNG hier als Kopie, weil
der mujoco-sim-Container nur unitree_mujoco/ sieht).
"""
import ctypes
import ctypes.util
import os
import struct
import threading
import time
import zlib
from pathlib import Path

ICON_PATH = Path(__file__).resolve().parent / "icons" / "g1_mujoco.png"
WM_CLASS_NAME = "g1-mujoco"


def prepare_env():
    """Vor mujoco.viewer.launch_passive() aufrufen (WM_CLASS-Instanz)."""
    os.environ.setdefault("RESOURCE_NAME", WM_CLASS_NAME)


# ── PNG laden (nur 8 Bit RGB/RGBA, nicht interlaced -- reicht fuer unsere Icons) ──
def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def load_png_rgba(path):
    """-> (breite, hoehe, bytes RGBA). Ohne PIL, damit das Sim-Image nichts braucht."""
    data = Path(path).read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("keine PNG-Datei")
    pos, idat, hdr = 8, b"", None
    while pos < len(data):
        length, ctype = struct.unpack(">I4s", data[pos:pos + 8])
        chunk = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if ctype == b"IHDR":
            hdr = struct.unpack(">IIBBBBB", chunk)
        elif ctype == b"IDAT":
            idat += chunk
        elif ctype == b"IEND":
            break
    w, h, depth, color, _, _, interlace = hdr
    if depth != 8 or color not in (2, 6) or interlace:
        raise ValueError("PNG-Format nicht unterstuetzt")
    bpp = 4 if color == 6 else 3
    raw = zlib.decompress(idat)
    stride = w * bpp
    out = bytearray()
    prev = bytearray(stride)
    i = 0
    for _ in range(h):
        ftype = raw[i]
        line = bytearray(raw[i + 1:i + 1 + stride])
        i += 1 + stride
        for x in range(stride):
            a = line[x - bpp] if x >= bpp else 0
            b = prev[x]
            c = prev[x - bpp] if x >= bpp else 0
            if ftype == 1:
                line[x] = (line[x] + a) & 0xFF
            elif ftype == 2:
                line[x] = (line[x] + b) & 0xFF
            elif ftype == 3:
                line[x] = (line[x] + ((a + b) >> 1)) & 0xFF
            elif ftype == 4:
                line[x] = (line[x] + _paeth(a, b, c)) & 0xFF
        prev = line
        if bpp == 4:
            out += line
        else:
            for x in range(0, stride, 3):
                out += line[x:x + 3] + b"\xff"
    return w, h, bytes(out)


def _halve(w, h, rgba):
    """2x2-Mittelwert -> halbe Groesse (kleinere Icon-Stufen fuer das Dock)."""
    nw, nh = w // 2, h // 2
    out = bytearray(nw * nh * 4)
    for y in range(nh):
        for x in range(nw):
            for ch in range(4):
                s = 0
                for dy in (0, 1):
                    for dx in (0, 1):
                        s += rgba[((2 * y + dy) * w + 2 * x + dx) * 4 + ch]
                out[(y * nw + x) * 4 + ch] = s // 4
    return nw, nh, bytes(out)


def net_wm_icon_data(path, sizes=(128, 64, 32)):
    """_NET_WM_ICON-Inhalt: je Groesse [breite, hoehe, ARGB-Pixel ...]."""
    w, h, rgba = load_png_rgba(path)
    cards = []
    while w >= min(sizes):
        if w in sizes:
            cards += [w, h]
            cards += [(rgba[i + 3] << 24) | (rgba[i] << 16) | (rgba[i + 1] << 8) | rgba[i + 2]
                      for i in range(0, len(rgba), 4)]
        if w % 2 or h % 2:
            break
        w, h, rgba = _halve(w, h, rgba)
    return cards


# ── X11 per ctypes (libX11 ist im Sim-Image durch GLFW ohnehin vorhanden) ──
def _load_xlib():
    name = ctypes.util.find_library("X11") or "libX11.so.6"
    x = ctypes.cdll.LoadLibrary(name)
    ul, vp = ctypes.c_ulong, ctypes.c_void_p
    x.XOpenDisplay.restype = vp
    x.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x.XDefaultRootWindow.restype = ul
    x.XDefaultRootWindow.argtypes = [vp]
    x.XInternAtom.restype = ul
    x.XInternAtom.argtypes = [vp, ctypes.c_char_p, ctypes.c_int]
    x.XQueryTree.argtypes = [vp, ul, ctypes.POINTER(ul), ctypes.POINTER(ul),
                             ctypes.POINTER(ctypes.POINTER(ul)), ctypes.POINTER(ctypes.c_uint)]
    x.XGetWindowProperty.argtypes = [vp, ul, ul, ctypes.c_long, ctypes.c_long, ctypes.c_int, ul,
                                     ctypes.POINTER(ul), ctypes.POINTER(ctypes.c_int),
                                     ctypes.POINTER(ul), ctypes.POINTER(ul),
                                     ctypes.POINTER(ctypes.c_void_p)]
    x.XChangeProperty.argtypes = [vp, ul, ul, ul, ctypes.c_int, ctypes.c_int, ctypes.c_void_p,
                                  ctypes.c_int]
    x.XFree.argtypes = [vp]
    x.XFlush.argtypes = [vp]
    x.XCloseDisplay.argtypes = [vp]
    return x


def _window_pid(x, dpy, win, atom_pid):
    ul = ctypes.c_ulong
    atype, afmt, n, after = ul(), ctypes.c_int(), ul(), ul()
    prop = ctypes.c_void_p()
    if x.XGetWindowProperty(dpy, win, atom_pid, 0, 1, 0, 6,  # 6 = XA_CARDINAL
                            ctypes.byref(atype), ctypes.byref(afmt), ctypes.byref(n),
                            ctypes.byref(after), ctypes.byref(prop)) != 0:
        return None
    try:
        if not prop.value or n.value < 1 or afmt.value != 32:
            return None
        return ctypes.cast(prop, ctypes.POINTER(ctypes.c_long))[0]  # Format 32 = C long
    finally:
        if prop.value:
            x.XFree(prop)


def _windows_of_pid(x, dpy, pid):
    ul = ctypes.c_ulong
    atom_pid = x.XInternAtom(dpy, b"_NET_WM_PID", 1)
    if not atom_pid:
        return []
    found, todo = [], [x.XDefaultRootWindow(dpy)]
    while todo:
        win = todo.pop()
        if _window_pid(x, dpy, win, atom_pid) == pid:
            found.append(win)
            continue
        root, parent = ul(), ul()
        children = ctypes.POINTER(ul)()
        n = ctypes.c_uint()
        if x.XQueryTree(dpy, win, ctypes.byref(root), ctypes.byref(parent),
                        ctypes.byref(children), ctypes.byref(n)):
            todo.extend(children[i] for i in range(n.value))
            if children:
                x.XFree(children)
    return found


def set_icon(pid=None, icon_path=ICON_PATH):
    """Setzt _NET_WM_ICON auf allen Fenstern von pid. -> Anzahl Fenster."""
    pid = os.getpid() if pid is None else pid
    x = _load_xlib()
    dpy = x.XOpenDisplay(None)
    if not dpy:
        return 0
    try:
        wins = _windows_of_pid(x, dpy, pid)
        if not wins:
            return 0
        cards = net_wm_icon_data(icon_path)
        buf = (ctypes.c_long * len(cards))(*[c if c < 2 ** 31 else c - 2 ** 32 for c in cards])
        atom_icon = x.XInternAtom(dpy, b"_NET_WM_ICON", 0)
        for w in wins:
            x.XChangeProperty(dpy, w, atom_icon, 6, 32, 0, buf, len(cards))  # 0 = Replace
        x.XFlush(dpy)
        return len(wins)
    finally:
        x.XCloseDisplay(dpy)


def apply_async(timeout=20.0, interval=0.5):
    """Hintergrund-Thread: wartet, bis das Viewer-Fenster da ist, und setzt das Icon."""
    def worker():
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                if set_icon():
                    return
            except Exception:
                return  # kein X11/libX11 -> Standard-Icon bleibt
            time.sleep(interval)

    t = threading.Thread(target=worker, name="window_icon", daemon=True)
    t.start()
    return t

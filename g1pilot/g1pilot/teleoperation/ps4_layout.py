#!/usr/bin/env python3
"""
ps4_layout.py — feste Joy-Belegung fuer den PS4-Controller (DualShock 4).

Der alte `joystick`-Node nummeriert Achsen/Tasten in der Reihenfolge, in der
evdev die Faehigkeiten des Geraets meldet -- das haengt vom Kernel-Treiber ab
(hid-sony vs. hid-playstation) und ist fuer eine Tastenbelegung zu wackelig.
Der `ps4_joystick`-Node uebersetzt deshalb die evdev-Codes selbst in diese
FESTE Belegung. Die Codes sind die des Linux-Treibers (hid-sony ab Kernel 4.10,
hid-playstation), als Zahlen hinterlegt, damit Tests ohne evdev laufen.

Konvention wie das ROS-Paket `joy`: Stick links = +1, Stick oben = +1.
Trigger L2/R2: 0 = losgelassen ... 1 = voll gedrueckt.
"""

# ── Feste Belegung: Indizes in sensor_msgs/Joy ─────────────────────────────
# Tasten
CROSS, CIRCLE, TRIANGLE, SQUARE = 0, 1, 2, 3
L1, R1, L2, R2 = 4, 5, 6, 7
SHARE, OPTIONS, PS = 8, 9, 10
L3, R3 = 11, 12
DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT = 13, 14, 15, 16
NUM_BUTTONS = 17

# Achsen
LX, LY, RX, RY = 0, 1, 2, 3          # links = +1, oben = +1
L2_AXIS, R2_AXIS = 4, 5              # 0 .. 1
DPAD_X, DPAD_Y = 6, 7                # links = +1, oben = +1
NUM_AXES = 8

BUTTON_NAMES = {
    CROSS: "Kreuz", CIRCLE: "Kreis", TRIANGLE: "Dreieck", SQUARE: "Viereck",
    L1: "L1", R1: "R1", L2: "L2", R2: "R2", SHARE: "Share", OPTIONS: "Options",
    PS: "PS", L3: "L3", R3: "R3", DPAD_UP: "Steuerkreuz oben",
    DPAD_DOWN: "Steuerkreuz unten", DPAD_LEFT: "Steuerkreuz links",
    DPAD_RIGHT: "Steuerkreuz rechts",
}

# ── evdev-Codes (linux/input-event-codes.h) ────────────────────────────────
EV_KEY, EV_ABS = 0x01, 0x03

_KEY_TO_BUTTON = {
    0x130: CROSS,      # BTN_SOUTH
    0x131: CIRCLE,     # BTN_EAST
    0x133: TRIANGLE,   # BTN_NORTH
    0x134: SQUARE,     # BTN_WEST
    0x136: L1,         # BTN_TL
    0x137: R1,         # BTN_TR
    0x138: L2,         # BTN_TL2
    0x139: R2,         # BTN_TR2
    0x13a: SHARE,      # BTN_SELECT
    0x13b: OPTIONS,    # BTN_START
    0x13c: PS,         # BTN_MODE
    0x13d: L3,         # BTN_THUMBL
    0x13e: R3,         # BTN_THUMBR
}

ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
ABS_HAT0X, ABS_HAT0Y = 0x10, 0x11

# Sticks: evdev liefert rechts/unten = max -> Vorzeichen drehen.
_STICKS = {ABS_X: LX, ABS_Y: LY, ABS_RX: RX, ABS_RY: RY}
_TRIGGERS = {ABS_Z: L2_AXIS, ABS_RZ: R2_AXIS}
_HATS = {ABS_HAT0X: DPAD_X, ABS_HAT0Y: DPAD_Y}

# Geraetenamen: Bluetooth "Wireless Controller", USB (hid-playstation)
# "Sony Interactive Entertainment Wireless Controller", aeltere Treiber
# "Sony Computer Entertainment Wireless Controller". Bewegungssensoren und
# Touchpad sind eigene evdev-Geraete mit Namenszusatz -> ausschliessen.
DEVICE_NAME_HINTS = ("Wireless Controller", "DualShock 4", "DUALSHOCK 4")
DEVICE_NAME_EXCLUDE = ("Motion Sensors", "Touchpad")


def is_ps4_name(name, wanted=""):
    """True, wenn `name` der PS4-Controller ist (Hauptgeraet, nicht Sensoren)."""
    if not name or any(x in name for x in DEVICE_NAME_EXCLUDE):
        return False
    if wanted and name == wanted:
        return True
    return any(h in name for h in DEVICE_NAME_HINTS)


def _norm(value, lo, hi):
    """Rohwert -> 0..1 (lo..hi)."""
    if hi == lo:
        return 0.0
    return min(1.0, max(0.0, (value - lo) / float(hi - lo)))


class Ps4State:
    """Haelt den Controller-Zustand in der festen Belegung.

    absinfo: {abs_code: (min, max)} vom Geraet (fuer die Normierung)."""

    def __init__(self, absinfo=None):
        self.absinfo = dict(absinfo or {})
        self.reset()

    def reset(self):
        """Alles neutral (Sticks 0, Trigger 0, keine Taste) -- z.B. nach Abriss."""
        self.axes = [0.0] * NUM_AXES
        self.buttons = [0] * NUM_BUTTONS

    def apply(self, ev_type, code, value):
        """Ein evdev-Ereignis einarbeiten. -> True, wenn es zur Belegung gehoert."""
        if ev_type == EV_KEY and code in _KEY_TO_BUTTON:
            self.buttons[_KEY_TO_BUTTON[code]] = 1 if value else 0
            return True
        if ev_type != EV_ABS:
            return False
        if code in _STICKS:
            lo, hi = self.absinfo.get(code, (0, 255))
            self.axes[_STICKS[code]] = -(2.0 * _norm(value, lo, hi) - 1.0)
            return True
        if code in _TRIGGERS:
            lo, hi = self.absinfo.get(code, (0, 255))
            self.axes[_TRIGGERS[code]] = _norm(value, lo, hi)
            return True
        if code in _HATS:
            v = -max(-1, min(1, int(value)))       # links/oben = +1
            idx = _HATS[code]
            self.axes[idx] = float(v)
            if idx == DPAD_X:
                self.buttons[DPAD_LEFT] = 1 if v > 0 else 0
                self.buttons[DPAD_RIGHT] = 1 if v < 0 else 0
            else:
                self.buttons[DPAD_UP] = 1 if v > 0 else 0
                self.buttons[DPAD_DOWN] = 1 if v < 0 else 0
            return True
        return False

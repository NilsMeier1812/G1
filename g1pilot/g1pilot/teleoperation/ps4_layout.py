#!/usr/bin/env python3
"""
ps4_layout.py — feste Joy-Belegung fuer den PS4-Controller (DualShock 4)
und den Xbox-Controller (Xbox One / Series, z.B. "Shock Blue").

Der alte `joystick`-Node nummeriert Achsen/Tasten in der Reihenfolge, in der
evdev die Faehigkeiten des Geraets meldet -- das haengt vom Kernel-Treiber ab
(hid-sony vs. hid-playstation) und ist fuer eine Tastenbelegung zu wackelig.
Der `ps4_joystick`-Node uebersetzt deshalb die evdev-Codes selbst in diese
FESTE Belegung. Die Codes sind die des Linux-Treibers (hid-sony ab Kernel 4.10,
hid-playstation), als Zahlen hinterlegt, damit Tests ohne evdev laufen.

Der Xbox-Controller landet in DERSELBEN Belegung, zugeordnet nach Position
(A = Kreuz, B = Kreis, X = Viereck, Y = Dreieck, LB/RB = L1/R1, LT/RT =
L2/R2, View = Share, Menu = Options, Xbox-Taste = PS), siehe XboxState.
Alles hinter /g1pilot/ps4/joy (Arme, Gehen, NOT-HALT) merkt keinen Unterschied.

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
    if is_xbox_name(name):        # "Xbox Wireless Controller" enthaelt den PS4-Namen
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


# ══ Xbox-Controller (Xbox One / Series) ════════════════════════════════════
#
# Drei Linux-Treiber, drei Belegungen -- XboxState erkennt sie an den Achsen:
#   * xpad (USB-Kabel; auch xone fuer den Xbox-Funkadapter): Sticks
#     ABS_X/Y/RX/RY (-32768..32767), Trigger ABS_Z/ABS_RZ (0..1023).
#   * xpadneo (Bluetooth, Zusatztreiber): wie xpad.
#   * hid-generic / hid-microsoft (Bluetooth ohne xpadneo, aktuelle Firmware):
#     rechter Stick auf ABS_Z/ABS_RZ (!), Trigger ABS_BRAKE (LT) / ABS_GAS (RT).
# Tasten: alle Treiber melden X als BTN_X (0x133) und Y als BTN_Y (0x134) --
# gleiche Zahlen wie BTN_NORTH/BTN_WEST, aber vertauschte POSITION gegenueber
# dem PS4 (Dreieck oben = 0x133). Deshalb eigene Tabelle; `swap_xy` dreht sie,
# falls ein Treiber doch nach Position meldet.
# Xbox-Taste: BTN_MODE; ueber Bluetooth je nach Firmware als KEY_HOMEPAGE auf
# einem Teilgeraet "<Name> Consumer Control" (ps4_joystick liest es mit).
# Steuerkreuz: Hat-Achsen; xpad meldet es bei manchen Modellen als Tasten.

XBOX_VENDOR = 0x045e   # Microsoft
XBOX_NAME_HINTS = ("xbox", "x-box")

BTN_X, BTN_Y = 0x133, 0x134
KEY_HOMEPAGE, KEY_BACK, KEY_RECORD = 0xac, 0x9e, 0xa7
ABS_GAS, ABS_BRAKE = 0x09, 0x0a

# Tasten-Codes, die bei allen Xbox-Treibern gleich sind
_XBOX_KEYS = {
    0x130: CROSS,      # A
    0x131: CIRCLE,     # B
    BTN_X: SQUARE,     # X (links)
    BTN_Y: TRIANGLE,   # Y (oben)
    0x136: L1,         # LB
    0x137: R1,         # RB
    0x138: L2,         # LT als Taste (manche xpad-Modelle)
    0x139: R2,         # RT als Taste
    0x13a: SHARE,      # View
    0x13b: OPTIONS,    # Menu
    0x13c: PS,         # Xbox-Taste
    KEY_HOMEPAGE: PS,  # Xbox-Taste ueber Bluetooth (Consumer Control)
    0x13d: L3,
    0x13e: R3,
    0x2c0: DPAD_LEFT, 0x2c1: DPAD_RIGHT,     # xpad: Steuerkreuz als Tasten
    0x2c2: DPAD_UP, 0x2c3: DPAD_DOWN,        # (BTN_TRIGGER_HAPPY1..4)
    0x220: DPAD_UP, 0x221: DPAD_DOWN,        # BTN_DPAD_*
    0x222: DPAD_LEFT, 0x223: DPAD_RIGHT,
}

# Ersatz-Belegung, falls die Xbox-Taste nicht ankommt (z.B. vom Desktop/Steam
# abgefangen): View = PS (NOT-HALT), die Share-Taste der Series-Controller
# (KEY_RECORD) uebernimmt dann die Grundstellung.
XBOX_PS_BUTTONS = ("xbox", "view")


def is_xbox_name(name):
    n = (name or "").lower()
    return any(h in n for h in XBOX_NAME_HINTS)


def controller_kind(name, vendor=None, wanted=""):
    """'ps4', 'xbox' oder None fuer ein evdev-Geraet (Name + USB-Vendor).
    Teilgeraete (Bewegungssensoren, Touchpad, Consumer Control) -> None."""
    if not name or any(x in name for x in DEVICE_NAME_EXCLUDE + ("Consumer Control",)):
        return None
    if is_xbox_name(name) or (vendor == XBOX_VENDOR and "controller" in name.lower()):
        return "xbox"
    if is_ps4_name(name, wanted):
        return "ps4"
    return None


class XboxState(Ps4State):
    """Xbox-Controller in der festen (PS4-)Belegung, nach Position.

    absinfo entscheidet die Achsen-Variante (siehe oben): gibt es ABS_BRAKE/
    ABS_GAS, aber kein ABS_RX, liegt der rechte Stick auf ABS_Z/ABS_RZ."""

    def __init__(self, absinfo=None, ps_button="xbox", swap_xy=False):
        super().__init__(absinfo)
        keys = dict(_XBOX_KEYS)
        if swap_xy:
            keys[BTN_X], keys[BTN_Y] = TRIANGLE, SQUARE
        if ps_button == "view":
            keys[0x13a] = PS
            keys[KEY_BACK] = PS
            keys[0x13c] = keys[KEY_HOMEPAGE] = -1     # Xbox-Taste: ohne Wirkung
            keys[KEY_RECORD] = SHARE
        self._keys = keys
        ai = self.absinfo
        self.generic_bt = (ABS_RX not in ai and ABS_Z in ai
                           and (ABS_GAS in ai or ABS_BRAKE in ai))
        if self.generic_bt:
            self._sticks = {ABS_X: LX, ABS_Y: LY, ABS_Z: RX, ABS_RZ: RY}
            self._triggers = {ABS_BRAKE: L2_AXIS, ABS_GAS: R2_AXIS}
        else:
            self._sticks = {ABS_X: LX, ABS_Y: LY, ABS_RX: RX, ABS_RY: RY}
            self._triggers = {ABS_Z: L2_AXIS, ABS_RZ: R2_AXIS}

    def _default_range(self, code):
        if code in self._triggers:
            return (0, 1023)
        return (-32768, 32767)

    def apply(self, ev_type, code, value):
        if ev_type == EV_KEY and code in self._keys:
            idx = self._keys[code]
            if idx < 0:
                return True
            self.buttons[idx] = 1 if value else 0
            if idx in (DPAD_LEFT, DPAD_RIGHT):
                b = self.buttons
                self.axes[DPAD_X] = float(b[DPAD_LEFT] - b[DPAD_RIGHT])
            elif idx in (DPAD_UP, DPAD_DOWN):
                b = self.buttons
                self.axes[DPAD_Y] = float(b[DPAD_UP] - b[DPAD_DOWN])
            return True
        if ev_type != EV_ABS:
            return False
        if code in self._sticks:
            lo, hi = self.absinfo.get(code, self._default_range(code))
            self.axes[self._sticks[code]] = -(2.0 * _norm(value, lo, hi) - 1.0)
            return True
        if code in self._triggers:
            lo, hi = self.absinfo.get(code, self._default_range(code))
            self.axes[self._triggers[code]] = _norm(value, lo, hi)
            return True
        return super().apply(ev_type, code, value) if code in _HATS else False


def make_state(kind, absinfo=None, ps_button="xbox", swap_xy=False):
    """Zustand passend zum Controller-Typ ('ps4' | 'xbox')."""
    if kind == "xbox":
        return XboxState(absinfo, ps_button=ps_button, swap_xy=swap_xy)
    return Ps4State(absinfo)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hand_panel — Handsteuerung der Demo-GUI (Inspire RH56DFTP-2, Sim UND real).

Qt-Ersatz fuer die beiden Browser-GUIs der Hand-Bridge (hand_controller_viewer
und inspire_hand_viewer). Zwei Ebenen, damit es im Demo-Betrieb einfach bleibt:

  * GANZE HAND: Beide/Links/Rechts oeffnen + schliessen.
  * FINGER: je Finger Soll-Schieber, Ist-Oeffnung und gemessene Kraft, dazu
    die Kraftzonen-Anzeige (Taktil-Heatmap auf einer Handskizze).
  * ERWEITERT (eingeklappt): Griffkraft und Tempo als Auswahl, Kraftzonen
    nullen. Die Griffkraft setzt das Kraft-Limit aller Finger.

Kommunikation ausschliesslich ueber ROS (siehe inspire_ftp/bridge.py):
  /g1pilot/hand_action/{left,right}  String "open"/"close" (wie Streamdeck)
  /g1pilot/hand_cmd                  String JSON, gleiche Befehle wie ws://…:8766
  /g1pilot/hand_status               String JSON, ~10 Hz Ist-/Soll-Zustand + Zonen
"""

import json
import time

from PyQt6.QtWidgets import (
    QWidget, QFrame, QLabel, QPushButton, QSlider, QProgressBar, QComboBox,
    QVBoxLayout, QHBoxLayout, QSizePolicy,
)
from PyQt6.QtCore import Qt, QRectF, QTimer
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush

SIDES = ("left", "right")
SIDE_TITLE = {"left": "LINKE HAND", "right": "RECHTE HAND"}
SIDE_SHORT = {"left": "Links", "right": "Rechts"}
FINGER_NAMES = ["Kleiner", "Ring", "Mittel", "Zeige", "Daumen", "Daumen\ndrehen"]
MAX_ANGLE = 1000
MAX_FORCE = 3000            # g, wie die Browser-GUI
CLOSED = [0, 0, 0, 0, 200, 0]   # Schliess-Stellung wie bridge._apply_open_close

# Stufen (gelten fuer alle Finger beider Haende).
GRIP_FORCES = {"Sanft": 300, "Mittel": 800, "Fest": 1500}    # Kraft-Limit in g
HAND_SPEEDS = {"Langsam": 300, "Normal": 600, "Schnell": 1000}

# Nach einer Nutzer-Eingabe den Schieber so lange NICHT vom Status nachziehen
# (sonst springt er zurueck, bevor die Bridge den neuen Sollwert meldet).
USER_HOLD_S = 1.0
# Kein hand_status seit ... -> Bridge gilt als nicht erreichbar.
STALE_S = 1.5

# Taktil-Normierung wie inspire_hand_viewer.html: Delta zur Nulllage, Rauschen
# weg, 1200 ADC-Einheiten = voller Ausschlag.
TACTILE_NOISE = 80
TACTILE_FULL = 1200

# Zonen-Rechtecke der LINKEN Hand in der Skizze (ViewBox 220x370, aus
# inspire_hand_viewer.html): zone_id -> (x, y, w, h). Rechts = gespiegelt.
ZONES_LEFT = {
    "kleiner_tip": (28, 50, 26, 12), "kleiner_nail": (28, 65, 26, 40),
    "kleiner_pad": (28, 120, 26, 40),
    "ring_tip": (60, 40, 26, 12), "ring_nail": (60, 55, 26, 40),
    "ring_pad": (60, 120, 26, 40),
    "mittel_tip": (92, 35, 26, 12), "mittel_nail": (92, 50, 26, 40),
    "mittel_pad": (92, 120, 26, 40),
    "zeige_tip": (124, 40, 26, 12), "zeige_nail": (124, 55, 26, 40),
    "zeige_pad": (124, 120, 26, 40),
    "daumen_tip": (158, 150, 26, 15), "daumen_nail": (158, 168, 26, 40),
    "daumen_mid": (158, 213, 26, 20), "daumen_pad": (158, 240, 26, 40),
    "palme": (28, 188, 114, 70),
}
# Umrisse (Handgelenk, Handflaeche, 4 Finger, Daumen) -- nur Deko.
OUTLINE_LEFT = [
    (28, 300, 118, 60), (26, 172, 120, 134),
    (28, 45, 26, 137), (60, 35, 26, 147), (92, 28, 26, 154), (124, 33, 26, 150),
    (155, 145, 30, 140),
]
VIEW_W, VIEW_H = 220.0, 370.0

_HEAT = [(0, (40, 48, 64)), (35, (4, 45, 110)), (80, (0, 120, 190)),
         (130, (0, 180, 110)), (175, (140, 200, 0)), (210, (255, 160, 0)),
         (255, (240, 20, 20))]


def heat_color(v):
    """0..1 -> Heatmap-Farbe (Skala aus inspire_hand_viewer.html)."""
    v = max(0.0, min(1.0, v)) * 255
    for (va, ca), (vb, cb) in zip(_HEAT, _HEAT[1:]):
        if v <= vb:
            t = (v - va) / (vb - va)
            return QColor(*(round(a + t * (b - a)) for a, b in zip(ca, cb)))
    return QColor(*_HEAT[-1][1])


def _btn(text, color="#2d2d2d", font=16, height=56, checkable=False):
    b = QPushButton(text)
    b.setMinimumHeight(height)
    b.setCheckable(checkable)
    b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
    b.setStyleSheet(f"""
        QPushButton {{ background:{color}; color:white; font-size:{font}px; font-weight:700;
                       border:1px solid #555; border-radius:12px; padding:4px 10px; }}
        QPushButton:hover {{ border:2px solid #aaa; }}
        QPushButton:pressed {{ background:#555; }}
        QPushButton:checked {{ background:#fb8c00; border:2px solid #ffcc80; }}
        QPushButton:disabled {{ background:#1c1c1c; color:#555; border:1px solid #2a2a2a; }}
    """)
    return b


def _caption(text, size=14, color="#aaa"):
    lbl = QLabel(text)
    lbl.setStyleSheet(f"color:{color}; font-size:{size}px; background:transparent;")
    return lbl


# ── Kraftzonen: Handskizze mit Taktil-Heatmap ──────────────────────────────
class TactileHand(QWidget):
    """Zeichnet die 17 Taktil-Zonen einer Hand, eingefaerbt nach Druck."""

    def __init__(self, side):
        super().__init__()
        self.mirror = side == "right"
        self.levels = {}          # zone_id -> 0..1
        self.setMinimumSize(120, 200)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

    def set_levels(self, levels):
        self.levels = levels
        self.update()

    def _rect(self, x, y, w, h, scale, ox, oy):
        if self.mirror:
            x = VIEW_W - x - w
        return QRectF(ox + x * scale, oy + y * scale, w * scale, h * scale)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        scale = min(self.width() / VIEW_W, self.height() / (VIEW_H - 25))
        ox = (self.width() - VIEW_W * scale) / 2
        oy = (self.height() - (VIEW_H - 25) * scale) / 2 - 25 * scale
        p.setPen(QPen(QColor("#2b3d55"), 1.5))
        p.setBrush(QBrush(QColor("#0d1a30")))
        for r in OUTLINE_LEFT:
            p.drawRoundedRect(self._rect(*r, scale, ox, oy), 6 * scale, 6 * scale)
        p.setPen(Qt.PenStyle.NoPen)
        for zid, r in ZONES_LEFT.items():
            p.setBrush(QBrush(heat_color(self.levels.get(zid, 0.0))))
            p.drawRoundedRect(self._rect(*r, scale, ox, oy), 3 * scale, 3 * scale)
        p.end()


# ── Ein Finger (DOF) ───────────────────────────────────────────────────────
class FingerControl(QWidget):
    """Soll-Schieber (oben = offen), Ist-Oeffnung und gemessene Kraft. Das
    Kraft-Limit kommt global ueber die Griffkraft (HandPanel)."""

    BAR_CSS = ("QProgressBar {{ background:#0a1220; border:1px solid #2b3d55; border-radius:4px;"
               " color:white; font-size:11px; }}"
               " QProgressBar::chunk {{ background:{c}; border-radius:3px; }}")

    def __init__(self, name, on_angle):
        super().__init__()
        self.on_angle = on_angle
        self._sync = False
        self._angle_touched = 0.0

        lay = QVBoxLayout(self)
        lay.setContentsMargins(2, 0, 2, 0)
        lay.setSpacing(4)
        title = _caption(name, 13, "#ddd")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setMinimumHeight(30)
        lay.addWidget(title)

        mid = QHBoxLayout()
        mid.setSpacing(4)
        mid.addStretch(1)
        self.angle = QSlider(Qt.Orientation.Vertical)
        self.angle.setRange(0, MAX_ANGLE)
        self.angle.setValue(MAX_ANGLE)
        self.angle.setMinimumHeight(100)
        self.angle.setToolTip("Soll-Stellung: oben = offen, unten = geschlossen")
        self.angle.setStyleSheet("""
            QSlider::groove:vertical { background:#1e2e44; width:8px; border-radius:4px; }
            QSlider::handle:vertical { background:#fb8c00; border:2px solid #ffcc80;
                                       height:22px; margin:0 -9px; border-radius:11px; }
            QSlider::add-page:vertical { background:#fb8c00; width:8px; border-radius:4px; }
        """)
        self.angle.valueChanged.connect(self._angle_changed)
        mid.addWidget(self.angle)
        self.open_bar = QProgressBar()
        self.open_bar.setOrientation(Qt.Orientation.Vertical)
        self.open_bar.setRange(0, MAX_ANGLE)
        self.open_bar.setTextVisible(False)
        self.open_bar.setFixedWidth(14)
        self.open_bar.setMinimumHeight(70)
        self.open_bar.setToolTip("Ist-Öffnung des Fingers (voll = offen)")
        self.open_bar.setStyleSheet(self.BAR_CSS.format(c="#ffd740"))
        mid.addWidget(self.open_bar)
        mid.addStretch(1)
        lay.addLayout(mid, 1)

        self.angle_lbl = _caption("", 11, "#ffd740")
        self.angle_lbl.setToolTip("Ist-Öffnung (100 % = offen)")
        self.angle_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self.angle_lbl)

        self.force_bar = QProgressBar()
        self.force_bar.setRange(0, MAX_FORCE)
        self.force_bar.setFixedHeight(20)
        self.force_bar.setMinimumWidth(36)
        self.force_bar.setFormat("0 g")
        self.force_bar.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.force_bar.setToolTip("Gemessene Kontaktkraft (rot = Griffkraft-Limit erreicht)")
        self.force_bar.setStyleSheet(self.BAR_CSS.format(c="#2e7d32"))
        lay.addWidget(self.force_bar)

    def _angle_changed(self, v):
        if self._sync:
            return
        self._angle_touched = time.monotonic()
        self.on_angle(v)

    def set_target(self, value, user=False):
        """Soll-Schieber setzen. user=True: kommt von einem Ganz-Hand-Knopf ->
        Status-Nachziehen kurz pausieren, aber nichts senden."""
        if user:
            self._angle_touched = time.monotonic()
        self._sync = True
        self.angle.setValue(int(value))
        self._sync = False

    def update_state(self, angle_act, angle_set, force_act, force_set):
        now = time.monotonic()
        self.open_bar.setValue(int(max(0, min(MAX_ANGLE, angle_act))))
        pct = round(100 * max(0, min(MAX_ANGLE, angle_act)) / MAX_ANGLE)
        self.angle_lbl.setText(f"{pct} %")
        f = abs(float(force_act))
        self.force_bar.setValue(int(min(MAX_FORCE, f)))
        self.force_bar.setFormat(f"{f:.0f} g")
        over = force_set > 0 and f >= force_set
        self.force_bar.setStyleSheet(self.BAR_CSS.format(
            c="#e53935" if over else "#fb8c00" if f > 0.5 * force_set else "#2e7d32"))
        self._sync = True
        if not self.angle.isSliderDown() and now - self._angle_touched > USER_HOLD_S:
            self.angle.setValue(int(angle_set if angle_set >= 0 else angle_act))
        self._sync = False


# ── Eine Hand ──────────────────────────────────────────────────────────────
class HandCard(QFrame):
    def __init__(self, side, panel):
        super().__init__()
        self.side = side
        self.setObjectName("handcard")
        self.setStyleSheet("QFrame#handcard { background:#161b24; border:1px solid #2b3d55;"
                           " border-radius:14px; }")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(8)

        head = QHBoxLayout()
        title = _caption(SIDE_TITLE[side], 17, "#fb8c00")
        title.setStyleSheet(title.styleSheet() + "font-weight:800;")
        head.setSpacing(8)
        head.addWidget(title)
        self.conn = _caption("● –", 12, "#666")
        head.addWidget(self.conn)
        head.addStretch(1)
        self.btn_open = _btn("Öffnen", "#2e7d32", font=15, height=44)
        self.btn_close = _btn("Schließen", "#5d4037", font=15, height=44)
        self.btn_open.setMaximumWidth(140)
        self.btn_close.setMaximumWidth(140)
        self.btn_open.clicked.connect(lambda: panel.hand_action(side, "open"))
        self.btn_close.clicked.connect(lambda: panel.hand_action(side, "close"))
        head.addWidget(self.btn_open)
        head.addWidget(self.btn_close)
        lay.addLayout(head)

        body = QHBoxLayout()
        body.setSpacing(6)
        self.tactile = TactileHand(side)
        zone_box = QVBoxLayout()
        zone_box.setSpacing(2)
        zl = _caption("Kraftzonen", 12, "#777")
        zl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        zone_box.addWidget(zl)
        zone_box.addWidget(self.tactile, 1)
        fingers = QHBoxLayout()
        fingers.setSpacing(2)
        self.fingers = {}
        # Wie die Browser-GUI: rechts gespiegelt -> beide Daumen zur Mitte.
        order = range(6) if side == "left" else reversed(range(6))
        prev_thumb = None
        for i in order:
            is_thumb = i >= 4
            if prev_thumb is not None and is_thumb != prev_thumb:
                # Duenne Linie zwischen Daumen-Paar und den vier Fingern.
                sep = QFrame()
                sep.setFixedWidth(1)
                sep.setStyleSheet("background:#4a6880; border:none;")
                fingers.addWidget(sep)
            prev_thumb = is_thumb
            fc = FingerControl(FINGER_NAMES[i], lambda v, d=i: panel.set_angle(side, d, v))
            self.fingers[i] = fc
            fingers.addWidget(fc, 1)
        if side == "left":
            body.addLayout(zone_box, 3)
            body.addLayout(fingers, 7)
        else:
            body.addLayout(fingers, 7)
            body.addLayout(zone_box, 3)
        lay.addLayout(body, 1)

    def show_action(self, action):
        for i, fc in self.fingers.items():
            fc.set_target(MAX_ANGLE if action == "open" else CLOSED[i], user=True)

    def set_connected(self, online, real=None):
        if not online:
            self.conn.setText("● keine Daten")
            self.conn.setStyleSheet("color:#ef5350; font-size:12px; background:transparent;")
        elif real is False:
            self.conn.setText("● nicht verbunden")
            self.conn.setStyleSheet("color:#ffb300; font-size:12px; background:transparent;")
        else:
            self.conn.setText("● verbunden")
            self.conn.setStyleSheet("color:#66bb6a; font-size:12px; background:transparent;")

    def update_state(self, st, baseline):
        aa, as_ = st.get("angle_act", []), st.get("angle_set", [])
        fa, fs = st.get("force_act", []), st.get("force_set", [])
        for i, fc in self.fingers.items():
            if i < len(aa):
                fc.update_state(aa[i], as_[i] if i < len(as_) else -1,
                                fa[i] if i < len(fa) else 0,
                                fs[i] if i < len(fs) else 0)
        zones = st.get("zones", {})
        levels = {}
        for zid, v in zones.items():
            delta = float(v) - baseline.get(zid, 0.0)
            levels[zid] = 0.0 if delta < TACTILE_NOISE else (delta - TACTILE_NOISE) / TACTILE_FULL
        self.tactile.set_levels(levels)
        self.set_connected(True, st.get("connected"))


# ── Gesamtes Hand-Panel ────────────────────────────────────────────────────
class HandPanel(QWidget):
    """node: DemoNode mit pub_left_hand/pub_right_hand/pub_hand_cmd und
    publish_str(). Status kommt ueber on_status() (vom Node-Callback)."""

    def __init__(self, node, status_cb=None):
        super().__init__()
        self.node = node
        self.status_cb = status_cb or (lambda *_: None)
        self.state = {}
        self.baseline = {s: None for s in SIDES}
        self.last_status = 0.0

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)

        # Zeile 1: beide Haende
        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(_caption("Beide Hände:", 15))
        self.btn_open_all = _btn("Öffnen", "#2e7d32", font=18)
        self.btn_close_all = _btn("Schließen", "#5d4037", font=18)
        self.btn_open_all.clicked.connect(lambda: self.hand_action(None, "open"))
        self.btn_close_all.clicked.connect(lambda: self.hand_action(None, "close"))
        row.addWidget(self.btn_open_all)
        row.addWidget(self.btn_close_all)
        lay.addLayout(row)

        self.hint = _caption("", 14, "#ffb300")
        self.hint.setWordWrap(True)
        self.hint.setVisible(False)
        lay.addWidget(self.hint)

        hands = QHBoxLayout()
        hands.setSpacing(12)
        self.cards = {s: HandCard(s, self) for s in SIDES}
        hands.addWidget(self.cards["left"], 1)
        hands.addWidget(self.cards["right"], 1)
        lay.addLayout(hands, 1)

        # Erweitert (eingeklappt): Griffkraft, Tempo, Kraftzonen nullen
        self.btn_expert = QPushButton("Erweitert ▸")
        self.btn_expert.setCheckable(True)
        self.btn_expert.setStyleSheet("QPushButton{background:transparent;color:#888;"
                                      "border:none;font-size:14px;text-align:left;}")
        self.btn_expert.toggled.connect(self._toggle_expert)
        lay.addWidget(self.btn_expert)
        self.expert = QFrame()
        ex = QHBoxLayout(self.expert)
        ex.setContentsMargins(0, 0, 0, 0)
        ex.setSpacing(10)
        ex.addWidget(_caption("Griffkraft", 14))
        self.grip_box = self._combo(GRIP_FORCES, "g", self.set_grip)
        ex.addWidget(self.grip_box)
        ex.addSpacing(16)
        ex.addWidget(_caption("Tempo", 14))
        self.speed_box = self._combo(HAND_SPEEDS, None, self.set_speed)
        ex.addWidget(self.speed_box)
        ex.addStretch(1)
        self.btn_zero = _btn("Kraftzonen nullen", font=14, height=40)
        self.btn_zero.setMaximumWidth(200)
        self.btn_zero.setToolTip("Aktuelle Taktil-Werte als Nulllage übernehmen "
                                 "(Hände dabei nichts berühren lassen).")
        self.btn_zero.clicked.connect(self.zero_tactile)
        ex.addWidget(self.btn_zero)
        self.expert.setVisible(False)
        lay.addWidget(self.expert)

        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._check_stale)
        self.watchdog.start(500)
        self._check_stale()

    @staticmethod
    def _combo(presets, unit, cb):
        """Auswahl der Stufen. 'activated' feuert nur bei Nutzer-Auswahl, nicht
        beim Nachziehen aus dem Status."""
        box = QComboBox()
        for name, val in presets.items():
            box.addItem(f"{name} ({val} {unit})" if unit else name, name)
        box.setCurrentIndex(-1)
        box.setMinimumHeight(40)
        box.setMinimumWidth(170)
        box.setStyleSheet("""
            QComboBox { background:#2d2d2d; color:white; font-size:14px; font-weight:700;
                        border:1px solid #555; border-radius:10px; padding:4px 12px; }
            QComboBox:hover { border:2px solid #aaa; }
            QComboBox QAbstractItemView { background:#2d2d2d; color:white;
                                          selection-background-color:#fb8c00; }
        """)
        box.activated.connect(lambda i: cb(box.itemData(i)))
        return box

    def _toggle_expert(self, on):
        self.expert.setVisible(on)
        self.btn_expert.setText("Erweitert ▾" if on else "Erweitert ▸")

    # ── Befehle ─────────────────────────────────────────────────────────
    def _cmd(self, **cmd):
        self.node.publish_str(self.node.pub_hand_cmd, json.dumps(cmd))

    def _ensure_enabled(self, side):
        """Einzel-Befehle wirken nur auf aktivierte Haende (Bridge-Hauptschalter,
        nach NOT-HALT aus). Oeffnen/Schliessen aktiviert selbst."""
        if not self.state.get(side, {}).get("enabled", False):
            self._cmd(type="set_enabled", side=side, value=True)
            self.state.setdefault(side, {})["enabled"] = True

    def hand_action(self, side, action):
        sides = SIDES if side is None else (side,)
        for s in sides:
            pub = self.node.pub_left_hand if s == "left" else self.node.pub_right_hand
            self.node.publish_str(pub, action)
            self.cards[s].show_action(action)
        who = "Hände" if side is None else f"Hand {SIDE_SHORT[side].lower()}"
        self.status_cb(f"{who} {'öffnen' if action == 'open' else 'schließen'}")

    def set_angle(self, side, dof, value):
        self._ensure_enabled(side)
        self._cmd(type="set_angle", side=side, dof=dof, value=int(value))

    def set_grip(self, name):
        val = GRIP_FORCES[name]
        for s in SIDES:
            for d in range(6):
                self._cmd(type="set_force", side=s, dof=d, value=val)
        self.status_cb(f"Griffkraft: {name} ({val} g je Finger)")

    def set_speed(self, name):
        self._cmd(type="set_speed", value=HAND_SPEEDS[name])
        self.status_cb(f"Hand-Tempo: {name}")

    def zero_tactile(self):
        for s in SIDES:
            zones = self.state.get(s, {}).get("zones")
            if zones:
                self.baseline[s] = {k: float(v) for k, v in zones.items()}
        self.status_cb("Kraftzonen genullt.")

    # ── Status von der Bridge ───────────────────────────────────────────
    def on_status(self, data):
        self.last_status = time.monotonic()
        self.state = data
        for s in SIDES:
            st = data.get(s)
            if not st:
                continue
            if self.baseline[s] is None and st.get("zones"):
                # Erste Daten = Nulllage (Start ohne Kontakt); "nullen" erneuert.
                self.baseline[s] = {k: float(v) for k, v in st["zones"].items()}
            self.cards[s].update_state(st, self.baseline[s] or {})
        self._sync_presets(data)
        self._check_stale()

    def _sync_presets(self, data):
        sets = [data.get(s, {}) for s in SIDES]
        forces = {f for st in sets for f in st.get("force_set", [])}
        grip = next((n for n, v in GRIP_FORCES.items() if forces == {v}), None)
        speeds = {st.get("speed_set") for st in sets if "speed_set" in st}
        speed = next((n for n, v in HAND_SPEEDS.items() if speeds == {v}), None)
        # Passt der Bridge-Zustand zu keiner Stufe -> Auswahl leer.
        for box, name in ((self.grip_box, grip), (self.speed_box, speed)):
            if not box.view().isVisible():
                box.setCurrentIndex(box.findData(name) if name else -1)

    def _check_stale(self):
        online = time.monotonic() - self.last_status < STALE_S
        self.hint.setVisible(not online)
        if not online:
            self.hint.setText("Keine Daten von der Hand-Bridge (/g1pilot/hand_status). "
                              "Läuft sie? (start.sh: USE_HANDS=true) — Öffnen/Schließen "
                              "wird trotzdem gesendet.")
            for c in self.cards.values():
                c.set_connected(False)

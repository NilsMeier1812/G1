#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Handy-Bedienung in der Demo-GUI: Gateway + Bluetooth starten, Befehle des
Handys mit DENSELBEN Methoden ausfuehren wie die Knoepfe der PC-GUI, Status
fuers Handy zusammenstellen, Kopplung/Steuerhoheit am PC.

Nur aktiv mit G1_PHONE=1 (Start-Fenster: »Handy-Zugang«). Sonst gibt es
keinen Knopf, keinen Thread und keine Bluetooth-Werbung.

  G1_PHONE_BT_ADAPTER   hci0 (Default, ein Dongle) | hci1 (zweiter Dongle)
  G1_PHONE_BT_NAME      Name beim Werben (Default "G1 SIM" / "G1 REAL")
  G1_PHONE_BT_ENCRYPT   1 = zusaetzlich BLE-Kopplung verlangen (Default 0)
  G1_PHONE_VIEW_ESTOP   1 = auch Ansicht-Handys duerfen NOT-HALT (Default 1)
  G1_PHONE_STORE        Geraeteliste (Default ~/.config/g1pilot/phone_devices.json)

Verbindungsverlust oder entzogene Steuerhoheit bricht eine vom Handy
gestartete Bewegung (Ablauf, Sichere Pose, Grundstellung) ab.
"""

import os
import time

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QListWidget, QListWidgetItem,
)

from g1pilot.teleoperation import status_line
from g1pilot.teleoperation.phone import gateway as G
from g1pilot.teleoperation.phone.ble_server import BleServer

TICK_MS = 50
STATUS_MS = 200          # 5 Hz -> STATUS_MAX_LAG 5 ~ 1 s
HAND_ARGS = {"open": (None, "open"), "close": (None, "close"),
             "left_open": ("left", "open"), "left_close": ("left", "close"),
             "right_open": ("right", "open"), "right_close": ("right", "close")}


def enabled():
    return os.environ.get("G1_PHONE", "0").strip().lower() in ("1", "true", "yes", "on")


def _env_flag(name, default):
    return os.environ.get(name, default).strip().lower() in ("1", "true", "yes", "on")


def attach(gui, start_ble=True):
    """In DemoGUI.__init__ aufgerufen. -> PhoneLink oder None (aus)."""
    if not enabled():
        return None
    return PhoneLink(gui, start_ble=start_ble)


class PhoneLink:
    def __init__(self, gui, start_ble=True, gateway=None):
        self.gui = gui
        node = gui.node
        log = lambda t: node.get_logger().info(t)   # noqa: E731
        self.gw = gateway or G.PhoneGateway(
            estop_for_view=_env_flag("G1_PHONE_VIEW_ESTOP", "1"), log=log)
        name = os.environ.get("G1_PHONE_BT_NAME") or ("G1 SIM" if gui.sim_mode else "G1 REAL")
        self.ble = BleServer(self.gw, adapter=os.environ.get("G1_PHONE_BT_ADAPTER", "hci0"),
                             name=name, encrypt=_env_flag("G1_PHONE_BT_ENCRYPT", "0"), log=log)
        self._phone_motion = False      # Bewegung vom Handy gestartet, laeuft evtl. noch
        self._ble_state = None
        self.dialog = None
        if start_ble:
            self.ble.start()
        self._tick_timer = QTimer(gui)
        self._tick_timer.timeout.connect(self.tick)
        self._tick_timer.start(TICK_MS)
        self._status_timer = QTimer(gui)
        self._status_timer.timeout.connect(self.publish_status)
        self._status_timer.start(STATUS_MS)
        btn = gui.btn_phone
        self._btn_css = btn.styleSheet()
        btn.setVisible(True)
        btn.clicked.connect(self.show_dialog)
        self._update_button()

    def stop(self):
        self._tick_timer.stop()
        self._status_timer.stop()
        self.ble.stop()

    # ── Takt ────────────────────────────────────────────────────────────
    def tick(self):
        actions, events = self.gw.tick()
        for kind, text in events:
            self._on_event(kind, text)
        for a in actions:
            self._execute(a)
        g = self.gui
        if self._phone_motion and g.current_pose is None and not g.queue:
            self._phone_motion = False
        if self.ble.state != self._ble_state:
            self._ble_state = self.ble.state
            if self._ble_state.startswith("Fehler"):
                g._status(f"📱 Handy-Bluetooth: {self._ble_state}", "#ffb300")
        self._update_button()

    def _on_event(self, kind, text):
        g = self.gui
        if kind in ("control_lost", "disconnected") and self._phone_motion:
            self._phone_motion = False
            g._cancel()
            text += " Vom Handy gestartete Bewegung abgebrochen."
        color = "#ef5350" if kind in ("control_lost", "disconnected", "pair_failed") else "#4fc3f7"
        g._status(f"📱 {text}", color)
        if self.dialog is not None:
            self.dialog.refresh()

    def _update_button(self):
        connected, name, role, ctl = self.gw.state()
        b = self.gui.btn_phone
        if ctl:
            text, color = f"📱 {name} bedient", "#ffb300"
        elif connected:
            text, color = f"📱 {name}", "#4fc3f7"
        else:
            text, color = "📱 Handy", None
        if text != b.text():
            b.setText(text)
            b.setStyleSheet(self._btn_css + (f" QPushButton {{ color:{color}; }}" if color else ""))
        b.setToolTip(f"Handy-Zugang (Bluetooth): {self.ble.state}")

    # ── Befehle ─────────────────────────────────────────────────────────
    def _execute(self, a):
        action, arg, who, cid = a["action"], a["arg"], a["name"], a["id"]
        try:
            ok, text = getattr(self, f"_do_{action}")(arg)
        except Exception as e:  # noqa: BLE001
            ok, text = False, f"Fehler: {e}"
        self.gw.result(cid, ok, text)
        self.gui.node.get_logger().info(f"Handy »{who}«: {action} {arg or ''} -> {text}")
        if action != "estop":       # NOT-HALT setzt seine eigene Statuszeile
            self.gui._status(f"📱 {who}: {text}", "#4fc3f7" if ok else "#ffb300")
        else:
            self.gui._status(f"📱 NOT-HALT vom Handy »{who}« — Roboter ist weich geschaltet.",
                             "#ef5350")

    def _do_estop(self, _arg):
        self._phone_motion = False
        self.gui.emergency_stop()
        return True, "NOT-HALT ausgelöst."

    def _do_cancel(self, _arg):
        self._phone_motion = False
        self.gui._cancel()
        return True, "Bewegung gestoppt."

    def _do_ack(self, _arg):
        g = self.gui
        if g.sim_mode:
            if not g._estop_unacked:
                return False, "Kein NOT-HALT zu quittieren."
            n = g.node
            n.publish_bool(n.pub_start, True)
            g._estop_unacked = False
            g._estop_shown = False
            g._status("NOT-HALT quittiert — oben einen Modus wählen.", "#aaa")
            return True, "NOT-HALT quittiert."
        if g.started:
            return False, "Schon gestartet — nichts zu quittieren."
        was_estop = g._estop_shown
        g._start_robot()
        return True, "NOT-HALT quittiert." if was_estop else "Gestartet."

    def _do_loco(self, step):
        g = self.gui
        if g.sim_mode:
            return False, "Startleiste gibt es nur am echten G1."
        if step not in status_line.START_STEPS:
            return False, "Unbekannter Schritt."
        n = g.node
        age = None if n.onboard_t is None else time.monotonic() - n.onboard_t
        if not status_line.start_buttons(n.onboard, age)[step][0]:
            return False, f"{status_line.START_LABEL[step]} ist gerade gesperrt."
        g._start_step(step, confirmed=True)
        return True, f"{status_line.START_LABEL[step]} angefordert."

    def _do_mode(self, mode):
        g = self.gui
        from g1pilot.teleoperation.demo_gui import WALK, MANIP, MODE_TITLE
        target = {"walk": WALK, "grasp": MANIP}.get(mode)
        if target is None:
            return False, "Unbekannter Modus."
        if not g.started:
            return False, "Erst starten bzw. NOT-HALT quittieren."
        if target in (g.mode, g.pending):
            return True, f"{MODE_TITLE[target]} ist schon gewählt."
        g.request_mode(target)
        if target not in (g.mode, g.pending):
            return False, g.status.text()
        return True, f"{MODE_TITLE[target]} gewählt."

    def _need_grasp(self):
        from g1pilot.teleoperation.demo_gui import MANIP
        g = self.gui
        return g.mode == MANIP and g.pending is None

    def _do_hand(self, arg):
        if arg not in HAND_ARGS:
            return False, "Unbekannter Hand-Befehl."
        if not self._need_grasp():
            return False, "Hände nur im Modus GREIFEN."
        side, action = HAND_ARGS[arg]
        self.gui.manip_panel.hand_panel.hand_action(side, action)
        who = "Hände" if side is None else ("Hand links" if side == "left" else "Hand rechts")
        return True, f"{who} {'öffnen' if action == 'open' else 'schließen'}."

    def _do_seq(self, name):
        if not self._need_grasp():
            return False, "Abläufe nur im Modus GREIFEN."
        for label, names in self.gui._sequences():
            if label == name:
                self.gui.run_poses(names, label)
                self._phone_motion = True
                return True, f"Ablauf »{label}« gestartet."
        return False, f"Ablauf »{name}« nicht gefunden."

    def _do_safe(self, _arg):
        if not self._need_grasp():
            return False, "Nur im Modus GREIFEN."
        g = self.gui
        g._safe_pose()
        if g.current_pose is None:
            return False, g.status.text()
        self._phone_motion = True
        return True, "Sichere Pose wird angefahren."

    def _do_home(self, _arg):
        if not self._need_grasp():
            return False, "Nur im Modus GREIFEN."
        self.gui._home()
        return True, "Arme fahren in Grundstellung."

    # ── Status fuers Handy ──────────────────────────────────────────────
    def snapshot(self):
        g, n = self.gui, self.gui.node
        from g1pilot.teleoperation.demo_gui import WALK, MANIP
        key = {WALK: "walk", MANIP: "grasp", None: None}
        st = {
            "sim": g.sim_mode,
            "started": bool(g.started),
            "estop": bool(g._estop_shown),
            "ack": bool(g._estop_unacked if g.sim_mode else (not g.started)),
            "mode": key.get(g.mode),
            "pend": key.get(g.pending),
            "txt": g.status.text()[:160],
            "col": getattr(g, "_status_color", "#aaa"),
            "busy": g.current_pose is not None or bool(g.queue),
        }
        if not g.sim_mode:
            now = time.monotonic()
            age = None if n.onboard_t is None else now - n.onboard_t
            st["info"] = g.info_onboard.text()[:120]
            st["loco"] = {s: bool(v[0]) for s, v in status_line.start_buttons(n.onboard, age).items()}
        if n.ps4dev:
            st["pad"] = status_line.controller_label(n.ps4dev)
        if g.mode == MANIP:
            st["seqs"] = [label for label, _names in g._sequences()][:24]
        return st

    def publish_status(self):
        try:
            self.gw.publish_status(self.snapshot())
        except Exception as e:  # noqa: BLE001
            self.gui.node.get_logger().warn(f"Handy-Status: {e}")

    # ── Dialog am PC ────────────────────────────────────────────────────
    def show_dialog(self):
        if self.dialog is None:
            self.dialog = PhoneDialog(self)
        self.dialog.refresh()
        self.dialog.show()
        self.dialog.raise_()


class PhoneDialog(QDialog):
    """Kopplung, Geraeteliste, Bedienung zurueckholen."""

    def __init__(self, link):
        super().__init__(link.gui)
        self.link = link
        self.setWindowTitle("Handy-Zugang")
        self.setMinimumWidth(520)
        lay = QVBoxLayout(self)
        self.lbl_state = QLabel()
        self.lbl_state.setWordWrap(True)
        self.lbl_state.setStyleSheet("color:#ddd; font-size:15px;")
        lay.addWidget(self.lbl_state)
        self.btn_revoke = QPushButton("Bedienung zurückholen")
        self.btn_revoke.clicked.connect(lambda: (link.gw.revoke_control(), self.refresh()))
        lay.addWidget(self.btn_revoke)

        lay.addWidget(QLabel("Neues Handy koppeln:"))
        row = QHBoxLayout()
        self.btn_pair_op = QPushButton("Als Bediener koppeln")
        self.btn_pair_view = QPushButton("Nur Ansicht koppeln")
        self.btn_pair_op.clicked.connect(lambda: self._pair(G.ROLE_OPERATE))
        self.btn_pair_view.clicked.connect(lambda: self._pair(G.ROLE_VIEW))
        row.addWidget(self.btn_pair_op)
        row.addWidget(self.btn_pair_view)
        lay.addLayout(row)
        self.lbl_code = QLabel("")
        self.lbl_code.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_code.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.lbl_code.setStyleSheet("color:#ffb300; font-size:34px; font-weight:800;"
                                    "font-family:monospace; padding:10px;")
        lay.addWidget(self.lbl_code)
        self.lbl_hint = QLabel("")
        self.lbl_hint.setWordWrap(True)
        self.lbl_hint.setStyleSheet("color:#aaa;")
        lay.addWidget(self.lbl_hint)

        lay.addWidget(QLabel("Gekoppelte Handys:"))
        self.list = QListWidget()
        lay.addWidget(self.list)
        self.btn_remove = QPushButton("Ausgewähltes Handy entfernen")
        self.btn_remove.clicked.connect(self._remove)
        lay.addWidget(self.btn_remove)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.refresh)
        self._timer.start(500)

    def _pair(self, role):
        code = self.link.gw.start_pairing(role)
        self.lbl_code.setText(code)
        self.lbl_hint.setText(
            f"Code am Handy eingeben (Rolle: {G.ROLE_TITLE[role]}). Gilt "
            f"{int(G.PAIRING_TIMEOUT_S)} s und nur einmal. Der Code geht nie über Funk.")

    def _remove(self):
        item = self.list.currentItem()
        if item is not None:
            self.link.gw.remove_device(item.data(Qt.ItemDataRole.UserRole))
            self.refresh()

    def refresh(self):
        gw = self.link.gw
        connected, name, role, ctl = gw.state()
        lines = [f"Bluetooth: {self.link.ble.state}"]
        if connected:
            lines.append(f"Verbunden: {name} ({G.ROLE_TITLE[role]})"
                         + (" — bedient gerade" if ctl else ""))
        else:
            lines.append("Kein Handy verbunden.")
        self.lbl_state.setText("\n".join(lines))
        self.btn_revoke.setEnabled(ctl)
        if not gw.pairing_active and self.lbl_code.text():
            self.lbl_code.setText("")
            self.lbl_hint.setText("Kopplung beendet.")
        devs = gw.list_devices()
        keys = [d["dev"] for d in devs]
        shown = [self.list.item(i).data(Qt.ItemDataRole.UserRole) for i in range(self.list.count())]
        if keys != shown:
            self.list.clear()
            for d in devs:
                it = QListWidgetItem(f"{d['name']}  ({G.ROLE_TITLE[d['role']]})")
                it.setData(Qt.ItemDataRole.UserRole, d["dev"])
                self.list.addItem(it)

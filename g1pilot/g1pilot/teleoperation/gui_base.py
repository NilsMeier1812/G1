#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gemeinsame Bausteine der Demo-GUI (demo_gui.py).

Hervorgegangen aus der alten Streamdeck-GUI (ui_interface.py, 2026-10-30
entfernt): uebrig sind der ROS-Node mit den Bedien-Publishern (GuiNode,
Basisklasse von demo_gui.DemoNode), der Bildschirm-Joystick und die
Speichern-/Laden-Dialoge des Positionsspeichers.
"""

import math
from PyQt6.QtWidgets import (
    QWidget, QPushButton, QVBoxLayout,
    QHBoxLayout, QLabel, QInputDialog, QMessageBox,
    QDialog, QCheckBox, QDialogButtonBox, QLineEdit, QComboBox,
    QFormLayout, QGroupBox, QTreeWidget, QTreeWidgetItem
)
from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist
from g1pilot.teleoperation.sim_autostart import ARMS_STATE_TOPIC, LOCO_STATE_TOPIC


class VirtualJoystick(QWidget):
    """Bildschirm-Joystick: mit der Maus ziehen -> normierte (vx, vy) in [-1,1].

    Oben = vorwaerts (+vx), links = +vy (Roboter-Koordinaten). Loslassen zentriert
    auf (0,0). loco_sim multipliziert mit max_cmd -> vx=1.0 entspricht ~0.8 m/s.
    Wird im RUN-Zustand als Geschwindigkeitsbefehl ausgewertet; zentrieren loest den
    automatischen Handoff zurueck zum PD-Stand aus."""

    def __init__(self, size=180):
        super().__init__()
        self._size = size
        self.setFixedSize(size, size)
        self._radius = size / 2 - 16
        self._knob = QPointF(0.0, 0.0)   # Offset vom Zentrum (Pixel)
        self.vx = 0.0
        self.vy = 0.0

    def _update_vel(self):
        r = self._radius
        # Bildschirm-y zeigt nach unten -> oben (negatives dy) = vorwaerts.
        self.vx = max(-1.0, min(1.0, -self._knob.y() / r))
        self.vy = max(-1.0, min(1.0, -self._knob.x() / r))

    def _set_from_pos(self, pos):
        c = self._size / 2
        dx = pos.x() - c
        dy = pos.y() - c
        d = math.hypot(dx, dy)
        if d > self._radius and d > 0:
            dx *= self._radius / d
            dy *= self._radius / d
        self._knob = QPointF(dx, dy)
        self._update_vel()
        self.update()

    def mousePressEvent(self, e):
        self._set_from_pos(e.position())

    def mouseMoveEvent(self, e):
        self._set_from_pos(e.position())

    def mouseReleaseEvent(self, e):
        self._knob = QPointF(0.0, 0.0)
        self.vx = self.vy = 0.0
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c = self._size / 2
        p.setBrush(QBrush(QColor("#1e1e1e")))
        p.setPen(QPen(QColor("#444"), 2))
        p.drawEllipse(QPointF(c, c), self._radius + 10, self._radius + 10)
        p.setPen(QPen(QColor("#333"), 1))
        p.drawLine(int(c - self._radius), int(c), int(c + self._radius), int(c))
        p.drawLine(int(c), int(c - self._radius), int(c), int(c + self._radius))
        active = abs(self.vx) > 1e-3 or abs(self.vy) > 1e-3
        p.setBrush(QBrush(QColor("#4CAF50") if active else QColor("#3c3c3c")))
        p.setPen(QPen(QColor("#80ff80") if active else QColor("#666"), 2))
        p.drawEllipse(QPointF(c + self._knob.x(), c + self._knob.y()), 15, 15)


# Anzeigenamen der speicherbaren Pose-Komponenten (Schluessel wie in
# manipulation/pose_store.py COMPONENTS).
COMPONENT_LABELS = {
    "left_arm": "Linker Arm",
    "right_arm": "Rechter Arm",
    "left_hand": "Linke Hand",
    "right_hand": "Rechte Hand",
}


class PoseSaveDialog(QDialog):
    """EIN Fenster fuer den kompletten Speichern-Vorgang: Name, Kategorie
    ("Ordner", mit Anlegen-Button) und die Komponenten-Haekchen -- vorher waren
    das zwei aufeinanderfolgende Dialoge.

    Die vier speicherbaren Komponenten stehen FLACH nebeneinander (linker/
    rechter Arm, linke/rechte Hand) -- so, wie sie der Store fuehrt. Kein
    Sammel-Haekchen "Handposition" mit Zusatz-Tickbox: das war ein Ueberrest
    aus der Version, in der beide Haende eine einzige Komponente waren, und
    kostete nur Klicks. Ergebnis via result_data() als
    {"name", "category", "components"} -- genau das JSON, das
    arm_controller._on_pose_save erwartet.

    Demo-GUI: mit `sequences` (Liste der Ablaeufe) gibt es zusaetzlich »An
    Ablauf anhängen« (`sequence` vorausgewaehlt); result_data() traegt dann
    "sequence" (Name oder None) -- die GUI nimmt es vor dem Senden heraus."""

    NO_SEQUENCE = "— nicht anhängen —"

    def __init__(self, parent, store=None, categories=None, sequences=None, sequence=None):
        super().__init__(parent)
        self._store = store
        self.setWindowTitle("Pose speichern")
        self.setMinimumWidth(420)

        root = QVBoxLayout(self)

        form = QFormLayout()
        self.ed_name = QLineEdit()
        self.ed_name.setPlaceholderText("z. B. Regal oben rechts")
        form.addRow("Name:", self.ed_name)

        self.cb_category = QComboBox()
        self.cb_category.addItems(categories or [])
        btn_new_cat = QPushButton("Kategorie anlegen")
        btn_new_cat.clicked.connect(self._new_category)
        cat_row = QHBoxLayout()
        cat_row.addWidget(self.cb_category, 1)
        cat_row.addWidget(btn_new_cat)
        form.addRow("Kategorie:", cat_row)
        self.cb_sequence = None
        if sequences is not None:
            self.cb_sequence = QComboBox()
            self.cb_sequence.setEditable(True)     # neuer Name = neuer Ablauf
            self.cb_sequence.addItems([self.NO_SEQUENCE] + list(sequences))
            self.cb_sequence.setCurrentText(sequence or self.NO_SEQUENCE)
            self.cb_sequence.setToolTip("Gespeicherte Pose hinten an diesen Ablauf hängen "
                                        "(neuer Name legt den Ablauf an).")
            form.addRow("An Ablauf anhängen:", self.cb_sequence)
        root.addLayout(form)

        box = QGroupBox("Was speichern?")
        box_lay = QVBoxLayout(box)
        self.cb_left_arm = QCheckBox("Linker Arm")
        self.cb_right_arm = QCheckBox("Rechter Arm")
        self.cb_left_hand = QCheckBox("Linke Hand (Finger)")
        self.cb_right_hand = QCheckBox("Rechte Hand (Finger)")
        # Alle vier standardmaessig an: der haeufige Fall ist "ganze Haltung".
        for w in (self.cb_left_arm, self.cb_right_arm,
                  self.cb_left_hand, self.cb_right_hand):
            w.setChecked(True)
            box_lay.addWidget(w)
        root.addWidget(box)

        btns = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        btns.button(QDialogButtonBox.StandardButton.Ok).setText("Speichern")
        btns.button(QDialogButtonBox.StandardButton.Cancel).setText("Abbrechen")
        btns.accepted.connect(self._on_accept)
        btns.rejected.connect(self.reject)
        root.addWidget(btns)

        self.ed_name.setFocus()

    def _new_category(self):
        """Neue Kategorie anlegen. Sie wird SOFORT im Store vermerkt (leerer
        "Ordner"), damit sie auch nach Abbruch dieses Dialogs erhalten bleibt."""
        name, ok = QInputDialog.getText(self, "Kategorie anlegen", "Name der Kategorie:")
        name = (name or "").strip()
        if not ok or not name:
            return
        if self._store is not None:
            try:
                self._store.add_category(name)
            except Exception as e:   # noqa: BLE001 -- Anlegen darf den Dialog nie killen
                QMessageBox.warning(self, "Kategorie", f"Nicht gespeichert: {e}")
        idx = self.cb_category.findText(name)
        if idx < 0:
            self.cb_category.addItem(name)
            idx = self.cb_category.count() - 1
        self.cb_category.setCurrentIndex(idx)

    def _components(self):
        """-> Komponenten-Schluessel in Store-Reihenfolge (pose_store.COMPONENTS)."""
        boxes = (("left_arm", self.cb_left_arm), ("right_arm", self.cb_right_arm),
                 ("left_hand", self.cb_left_hand), ("right_hand", self.cb_right_hand))
        return [key for key, cb in boxes if cb.isChecked()]

    def _on_accept(self):
        if not self.ed_name.text().strip():
            QMessageBox.warning(self, "Pose speichern", "Bitte einen Namen eingeben.")
            self.ed_name.setFocus()
            return
        if not self._components():
            QMessageBox.warning(self, "Pose speichern",
                                "Bitte mindestens eine Komponente auswaehlen.")
            return
        self.accept()

    def result_data(self):
        out = {
            "name": self.ed_name.text().strip(),
            "category": self.cb_category.currentText().strip(),
            "components": self._components(),
        }
        if self.cb_sequence is not None:
            seq = self.cb_sequence.currentText().strip()
            out["sequence"] = None if seq in ("", self.NO_SEQUENCE) else seq
        return out


class PoseLoadDialog(QDialog):
    """Auswahl einer gespeicherten Pose, nach Kategorie ("Ordner") gruppiert.
    Zeigt je Pose auch, welche Komponenten drin sind -- man sieht vor dem
    Anfahren, ob z. B. die Haende mitfahren. selected_name() -> Pose-Name."""

    def __init__(self, parent, grouped, components_of=None):
        super().__init__(parent)
        self.setWindowTitle("Pose anfahren")
        self.resize(560, 420)
        self._name = None

        lay = QVBoxLayout(self)
        lay.addWidget(QLabel("Gespeicherte Posen (nach Kategorie):"))

        self.tree = QTreeWidget()
        self.tree.setColumnCount(2)
        self.tree.setHeaderLabels(["Pose", "Enthaelt"])
        for category, names in grouped.items():
            top = QTreeWidgetItem([category, ""])
            # Kategorie-Zeilen sind Ueberschriften, keine anfahrbaren Ziele.
            top.setFlags(top.flags() & ~Qt.ItemFlag.ItemIsSelectable)
            for name in names:
                comps = (components_of(name) if components_of else None) or []
                labels = ", ".join(COMPONENT_LABELS.get(c, c) for c in comps)
                child = QTreeWidgetItem([name, labels])
                child.setData(0, Qt.ItemDataRole.UserRole, name)
                top.addChild(child)
            if not names:
                empty = QTreeWidgetItem(["(leer)", ""])
                empty.setFlags(empty.flags() & ~Qt.ItemFlag.ItemIsSelectable)
                top.addChild(empty)
            self.tree.addTopLevelItem(top)
        self.tree.expandAll()
        for col in (0, 1):
            self.tree.resizeColumnToContents(col)
        lay.addWidget(self.tree, 1)

        self.btns = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        self.btns.button(QDialogButtonBox.StandardButton.Ok).setText("Anfahren")
        self.btns.button(QDialogButtonBox.StandardButton.Cancel).setText("Abbrechen")
        self.btns.accepted.connect(self._on_accept)
        self.btns.rejected.connect(self.reject)
        lay.addWidget(self.btns)

        self.tree.currentItemChanged.connect(self._sync_ok)
        self.tree.itemDoubleClicked.connect(lambda *_: self._on_accept())
        self._sync_ok()

    def _current_name(self):
        item = self.tree.currentItem()
        if item is None:
            return None
        return item.data(0, Qt.ItemDataRole.UserRole)

    def _sync_ok(self, *_):
        self.btns.button(QDialogButtonBox.StandardButton.Ok).setEnabled(
            self._current_name() is not None)

    def _on_accept(self):
        name = self._current_name()
        if name is None:
            return   # Kategorie-Zeile markiert -> nichts anzufahren
        self._name = name
        self.accept()

    def selected_name(self):
        return self._name


class GuiNode(Node):
    """ROS-Seite der Bedienoberflaeche: Publisher fuer alle Bedien-Topics,
    NOT-HALT von aussen und die Quittung fuer den Sim-Auto-Start."""

    def __init__(self, node_name='demo_gui'):
        super().__init__(node_name)

        self.pub_start = self.create_publisher(Bool, '/g1pilot/start', 10)
        self.pub_start_balancing = self.create_publisher(Bool, '/g1pilot/start_balancing', 10)
        self.pub_start_walking = self.create_publisher(Bool, '/g1pilot/start_walking', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/g1pilot/loco_cmd_vel', 10)
        # Navigation scharfschalten (AUTO NAV-Button): joy_mux gibt den Nav-Joy
        # nur bei True weiter -> der Roboter faehrt das Ziel autonom an.
        self.pub_auto_enable = self.create_publisher(Bool, '/g1pilot/auto_enable', 10)
        self.pub_arms_enabled = self.create_publisher(Bool, '/g1pilot/arms/enabled', 10)
        self.pub_arms_home = self.create_publisher(Bool, '/g1pilot/arms/home', 10)
        self.pub_marker_follow = self.create_publisher(Bool, '/g1pilot/marker_follow_ee', 10)
        self.pub_left_hand = self.create_publisher(String, '/g1pilot/hand_action/left', 10)
        self.pub_right_hand = self.create_publisher(String, '/g1pilot/hand_action/right', 10)
        self.pub_emergency_stop = self.create_publisher(Bool, '/g1pilot/emergency_stop', 10)
        self.pub_push = self.create_publisher(Bool, '/g1pilot/push', 10)
        # Aufladbarer Schubs: Ladegrad 0..1 beim Loslassen (siehe utils/push_charge.py).
        self.pub_push_charge = self.create_publisher(Float32, '/g1pilot/push_charge', 10)
        self.pub_grasp_box = self.create_publisher(Bool, '/g1pilot/grasp_box', 10)
        self.pub_scene_reset = self.create_publisher(Bool, '/g1pilot/scene_reset', 10)
        self.pub_catch_falls = self.create_publisher(Bool, '/g1pilot/catch_falls', 10)
        # Positionsspeicher (plan-execute), siehe g1pilot/docs/20-T-Arm-Manipulation.md (Positionsspeicher).
        self.pub_pose_save = self.create_publisher(String, '/g1pilot/pose_store/save', 10)
        self.pub_pose_goto = self.create_publisher(String, '/g1pilot/pose_store/goto', 10)
        self.pub_pose_cancel = self.create_publisher(Bool, '/g1pilot/pose_store/cancel', 10)
        # NOT-HALT kann auch von aussen kommen (PS-Taste am PS4-Controller,
        # ps4_arm_teleop): die GUI zeigt ihn dann genauso an wie den eigenen.
        # Der Callback laeuft im Qt-Thread (spin_once im QTimer).
        self.on_emergency_stop = None
        self.create_subscription(Bool, '/g1pilot/emergency_stop', self._emergency_stop, 10)
        # Quittung fuer den Sim-Auto-Start (sim_autostart.py): Zustand von
        # arm_controller (Arme aktiv?) und loco_sim (hold|stand|walk|damp), latched.
        self.arms_state = None
        self.loco_state = None
        qos_latched = QoSProfile(depth=1)
        qos_latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, ARMS_STATE_TOPIC,
                                 lambda m: setattr(self, 'arms_state', bool(m.data)), qos_latched)
        self.create_subscription(String, LOCO_STATE_TOPIC,
                                 lambda m: setattr(self, 'loco_state', m.data), qos_latched)

    def _emergency_stop(self, msg: Bool):
        if msg.data and self.on_emergency_stop:
            self.on_emergency_stop()

    def publish_bool(self, pub, value: bool):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def publish_push_charge(self, level: float):
        """Ladegrad 0..1 des Schubses (0 = kurzer Klick) an loco_sim."""
        msg = Float32()
        msg.data = float(level)
        self.pub_push_charge.publish(msg)

    def publish_str(self, pub, text: str):
        msg = String()
        msg.data = text
        pub.publish(msg)

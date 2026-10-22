#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sequence_editor.py — Fenster »Posen & Abläufe« der Demo-GUI.

Ein Ablauf ist eine Liste von Verweisen auf gespeicherte Posen
(pose_store.py): dieselbe Pose kann in mehreren Ablaeufen stehen, die
Reihenfolge stellt man hier ein -- kein Umbenennen, keine Kopien.

  links   Ablaeufe: neu, umbenennen, kopieren, loeschen, starten
  Mitte   Schritte des gewaehlten Ablaufs: verschieben (auch ziehen),
          entfernen, Pause danach (nur »am Stueck«), »Aktuelle Stellung als
          Schritt …« (speichern + anhaengen in einem Zug)
  rechts  gespeicherte Posen nach Kategorie: »← Hinzufügen« (Doppelklick),
          anfahren, loeschen, aktuelle Stellung speichern

Jede Aenderung wird sofort gespeichert. Das Fenster ist nicht modal: man
kann nebenher die Hand bewegen (Jog, Controller) und dann die Stellung als
Schritt uebernehmen. Bewegt wird nur ueber die Methoden der Demo-GUI
(run_sequence/run_poses -> geplantes POSE ANFAHREN im arm_controller).
"""
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QDialog, QHBoxLayout, QVBoxLayout, QGridLayout, QLabel, QListWidget,
    QListWidgetItem, QPushButton, QTreeWidget, QTreeWidgetItem, QInputDialog,
    QMessageBox, QDoubleSpinBox, QAbstractItemView, QGroupBox,
)

from g1pilot.teleoperation.sequence_runner import pretty_pose_name

ROLE = Qt.ItemDataRole.UserRole


class SequenceEditor(QDialog):
    """gui: DemoGUI (oder Ersatz in Tests) mit _pose_store(), run_poses(names),
    run_sequence(name), save_current_pose(sequence=None), _reload_sequence_buttons()."""

    def __init__(self, gui, parent=None):
        super().__init__(parent)
        self.gui = gui
        self.setWindowTitle("Posen & Abläufe")
        self.resize(1080, 620)
        # Gross genug fuer den Touchscreen am Roboter-Rechner.
        self.setStyleSheet("QWidget { font-size:15px; } QListWidget::item, QTreeWidget::item "
                           "{ padding:5px 2px; } QPushButton { min-height:34px; }")
        self._loading = False

        root = QVBoxLayout(self)
        cols = QHBoxLayout()
        root.addLayout(cols, 1)

        # ── Ablaeufe ────────────────────────────────────────────────────
        box_seq = QGroupBox("Abläufe")
        ls = QVBoxLayout(box_seq)
        self.seq_list = QListWidget()
        ls.addWidget(self.seq_list, 1)
        g = QGridLayout()
        self.btn_new = QPushButton("Neu …")
        self.btn_rename = QPushButton("Umbenennen …")
        self.btn_copy = QPushButton("Kopieren …")
        self.btn_del_seq = QPushButton("Löschen")
        for i, b in enumerate((self.btn_new, self.btn_rename, self.btn_copy, self.btn_del_seq)):
            g.addWidget(b, i // 2, i % 2)
        ls.addLayout(g)
        self.btn_run = QPushButton("▶ Ablauf starten")
        self.btn_run.setMinimumHeight(40)
        ls.addWidget(self.btn_run)
        cols.addWidget(box_seq, 3)

        # ── Schritte ────────────────────────────────────────────────────
        self.box_steps = QGroupBox("Schritte")
        lm = QVBoxLayout(self.box_steps)
        self.step_list = QListWidget()
        self.step_list.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        self.step_list.setDefaultDropAction(Qt.DropAction.MoveAction)
        lm.addWidget(self.step_list, 1)
        row = QHBoxLayout()
        self.btn_up = QPushButton("▲")
        self.btn_down = QPushButton("▼")
        self.btn_remove = QPushButton("Entfernen")
        for b in (self.btn_up, self.btn_down, self.btn_remove):
            row.addWidget(b)
        lm.addLayout(row)
        prow = QHBoxLayout()
        prow.addWidget(QLabel("Pause nach dem Schritt (nur »Am Stück«):"))
        self.spin_wait = QDoubleSpinBox()
        self.spin_wait.setRange(0.0, 600.0)
        self.spin_wait.setDecimals(1)
        self.spin_wait.setSingleStep(0.5)
        self.spin_wait.setSuffix(" s")
        prow.addWidget(self.spin_wait)
        lm.addLayout(prow)
        self.btn_teach = QPushButton("＋ Aktuelle Stellung als Schritt …")
        self.btn_teach.setMinimumHeight(40)
        lm.addWidget(self.btn_teach)
        cols.addWidget(self.box_steps, 4)

        # ── Posen ───────────────────────────────────────────────────────
        box_pose = QGroupBox("Gespeicherte Posen")
        lp = QVBoxLayout(box_pose)
        self.pose_tree = QTreeWidget()
        self.pose_tree.setHeaderHidden(True)
        lp.addWidget(self.pose_tree, 1)
        self.btn_add = QPushButton("← Hinzufügen")
        self.btn_add.setMinimumHeight(40)
        lp.addWidget(self.btn_add)
        prow2 = QHBoxLayout()
        self.btn_goto = QPushButton("Anfahren")
        self.btn_del_pose = QPushButton("Löschen")
        prow2.addWidget(self.btn_goto)
        prow2.addWidget(self.btn_del_pose)
        lp.addLayout(prow2)
        self.btn_save_pose = QPushButton("Aktuelle Stellung speichern …")
        lp.addWidget(self.btn_save_pose)
        cols.addWidget(box_pose, 4)

        self.hint = QLabel("Doppelklick auf eine Pose hängt sie an. Reihenfolge per Ziehen "
                           "oder ▲/▼. Änderungen werden sofort gespeichert.")
        self.hint.setWordWrap(True)
        self.hint.setStyleSheet("color:#999;")
        root.addWidget(self.hint)

        self.seq_list.currentItemChanged.connect(lambda *_: self._load_steps())
        self.btn_new.clicked.connect(self._new)
        self.btn_rename.clicked.connect(self._rename)
        self.btn_copy.clicked.connect(self._copy)
        self.btn_del_seq.clicked.connect(self._delete_seq)
        self.btn_run.clicked.connect(self._run)
        self.btn_up.clicked.connect(lambda: self._move(-1))
        self.btn_down.clicked.connect(lambda: self._move(1))
        self.btn_remove.clicked.connect(self._remove_step)
        self.spin_wait.valueChanged.connect(self._wait_changed)
        self.step_list.currentRowChanged.connect(lambda *_: self._sync())
        self.step_list.model().rowsMoved.connect(lambda *_: self._store_steps())
        self.btn_teach.clicked.connect(self._teach)
        self.btn_add.clicked.connect(self._add_pose)
        self.pose_tree.itemDoubleClicked.connect(lambda *_: self._add_pose())
        self.pose_tree.currentItemChanged.connect(lambda *_: self._sync())
        self.btn_goto.clicked.connect(self._goto)
        self.btn_del_pose.clicked.connect(self._delete_pose)
        self.btn_save_pose.clicked.connect(lambda: self.gui.save_current_pose())
        self.reload()

    # ── Daten ───────────────────────────────────────────────────────────
    def _store(self):
        return self.gui._pose_store()

    def current_sequence(self):
        it = self.seq_list.currentItem()
        return None if it is None else it.data(ROLE)

    def _selected_pose(self):
        it = self.pose_tree.currentItem()
        return None if it is None else it.data(0, ROLE)

    def reload(self, select=None):
        """Alles neu aus der Datei lesen (auch nach Speichern von aussen).
        select: Ablauf, der danach gewaehlt sein soll (Default: der bisherige)."""
        store = self._store()
        keep = select if select is not None else self.current_sequence()
        seqs = store.list_sequences() if store is not None else []
        grouped = store.list_grouped() if store is not None else {}
        self._known = {n for names in grouped.values() for n in names}
        self._loading = True
        self.seq_list.clear()
        for name, steps in seqs:
            it = QListWidgetItem(f"{name}   ({len(steps)})")
            it.setData(ROLE, name)
            self.seq_list.addItem(it)
            if name == keep:
                self.seq_list.setCurrentItem(it)
        if self.seq_list.currentItem() is None and self.seq_list.count():
            self.seq_list.setCurrentRow(0)
        pose_keep = self._selected_pose()
        self.pose_tree.clear()
        for cat, names in grouped.items():
            top = QTreeWidgetItem([cat])
            top.setFlags(top.flags() & ~Qt.ItemFlag.ItemIsSelectable)
            for n in names:
                comps = store.components(n) or []
                child = QTreeWidgetItem([f"{n}   [{_comp_short(comps)}]"])
                child.setData(0, ROLE, n)
                child.setToolTip(0, n)
                top.addChild(child)
                if n == pose_keep:
                    self.pose_tree.setCurrentItem(child)
            self.pose_tree.addTopLevelItem(top)
        self.pose_tree.expandAll()
        self._loading = False
        self._load_steps()

    def _load_steps(self):
        if self._loading:
            return
        name = self.current_sequence()
        store = self._store()
        steps = (store.get_sequence(name) if (store is not None and name) else None) or []
        row = self.step_list.currentRow()
        self._loading = True
        self.step_list.clear()
        for st in steps:
            self.step_list.addItem(self._step_item(st))
        self._renumber()
        if self.step_list.count():
            self.step_list.setCurrentRow(min(max(row, 0), self.step_list.count() - 1))
        self._loading = False
        self.box_steps.setTitle(f"Schritte von »{name}«" if name else "Schritte")
        self._sync()

    def _step_item(self, step):
        it = QListWidgetItem()
        it.setData(ROLE, dict(step))
        return it

    def _renumber(self):
        for i in range(self.step_list.count()):
            it = self.step_list.item(i)
            st = it.data(ROLE)
            missing = st["pose"] not in self._known
            text = f"{i + 1}. {pretty_pose_name(st['pose'])}"
            if pretty_pose_name(st["pose"]) != st["pose"].replace("_", " "):
                text += f"   ({st['pose']})"
            if st.get("wait_s"):
                text += f"   · Pause {st['wait_s']:g} s"
            if missing:
                text += "   — FEHLT"
            it.setText(text)
            it.setForeground(QColor("#ef5350") if missing else QColor("#dddddd"))

    def _steps(self):
        return [self.step_list.item(i).data(ROLE) for i in range(self.step_list.count())]

    def _store_steps(self):
        """Schrittliste so, wie sie angezeigt wird, speichern."""
        if self._loading:
            return
        name = self.current_sequence()
        store = self._store()
        if not name or store is None:
            return
        try:
            store.save_sequence(name, self._steps())
        except (OSError, ValueError) as e:
            QMessageBox.warning(self, "Ablauf", f"Nicht gespeichert: {e}")
            return
        self._renumber()
        it = self.seq_list.currentItem()
        if it is not None:
            it.setText(f"{name}   ({self.step_list.count()})")
        self._changed()

    def _changed(self):
        self.gui._reload_sequence_buttons()
        self._sync()

    def _sync(self):
        seq = self.current_sequence() is not None
        row = self.step_list.currentRow()
        n = self.step_list.count()
        for b in (self.btn_rename, self.btn_copy, self.btn_del_seq, self.btn_teach):
            b.setEnabled(seq)
        self.btn_run.setEnabled(seq and n > 0)
        self.btn_up.setEnabled(row > 0)
        self.btn_down.setEnabled(0 <= row < n - 1)
        self.btn_remove.setEnabled(row >= 0)
        self.spin_wait.setEnabled(row >= 0)
        if row >= 0:
            self.spin_wait.blockSignals(True)
            self.spin_wait.setValue(float(self.step_list.item(row).data(ROLE).get("wait_s") or 0.0))
            self.spin_wait.blockSignals(False)
        pose = self._selected_pose() is not None
        self.btn_add.setEnabled(seq and pose)
        self.btn_goto.setEnabled(pose)
        self.btn_del_pose.setEnabled(pose)

    # ── Ablaeufe ────────────────────────────────────────────────────────
    def _ask_name(self, title, text, default=""):
        name, ok = QInputDialog.getText(self, title, text, text=default)
        name = (name or "").strip()
        return name if ok and name else None

    def _new(self):
        name = self._ask_name("Neuer Ablauf", "Name des Ablaufs:")
        if name:
            self._try(lambda s: s.create_sequence(name), select=name)

    def _rename(self):
        old = self.current_sequence()
        name = old and self._ask_name("Ablauf umbenennen", "Neuer Name:", old)
        if name and name != old:
            self._try(lambda s: s.save_sequence(name, s.get_sequence(old) or [], old_name=old),
                      select=name)

    def _copy(self):
        old = self.current_sequence()
        name = old and self._ask_name("Ablauf kopieren", "Name der Kopie:", f"{old} Kopie")
        if name:
            def do(s):
                s.create_sequence(name)
                s.save_sequence(name, s.get_sequence(old) or [])
            self._try(do, select=name)

    def _delete_seq(self):
        name = self.current_sequence()
        if not name:
            return
        if QMessageBox.question(self, "Ablauf löschen",
                                f"Ablauf »{name}« löschen? Die Posen selbst bleiben erhalten."
                                ) != QMessageBox.StandardButton.Yes:
            return
        self._try(lambda s: s.delete_sequence(name), select="")

    def _try(self, fn, select=None):
        store = self._store()
        if store is None:
            QMessageBox.warning(self, "Abläufe", "Posen-Datei nicht lesbar.")
            return
        try:
            fn(store)
        except (OSError, ValueError) as e:
            QMessageBox.warning(self, "Abläufe", str(e))
            return
        self.reload(select=select)
        self._changed()

    def _run(self):
        name = self.current_sequence()
        if name:
            self.gui.run_sequence(name)

    # ── Schritte ────────────────────────────────────────────────────────
    def _move(self, delta):
        row = self.step_list.currentRow()
        new = row + delta
        if row < 0 or not 0 <= new < self.step_list.count():
            return
        self._loading = True
        it = self.step_list.takeItem(row)
        self.step_list.insertItem(new, it)
        self.step_list.setCurrentRow(new)
        self._loading = False
        self._store_steps()

    def _remove_step(self):
        row = self.step_list.currentRow()
        if row < 0:
            return
        self._loading = True
        self.step_list.takeItem(row)
        self._loading = False
        self._store_steps()

    def _wait_changed(self, value):
        row = self.step_list.currentRow()
        if row < 0 or self._loading:
            return
        it = self.step_list.item(row)
        st = dict(it.data(ROLE))
        st["wait_s"] = round(float(value), 2)
        it.setData(ROLE, st)
        self._store_steps()

    def add_step(self, pose):
        """Pose hinter dem markierten Schritt einfuegen (sonst ans Ende)."""
        if self.current_sequence() is None or not pose:
            return
        row = self.step_list.currentRow()
        at = row + 1 if row >= 0 else self.step_list.count()
        self._loading = True
        self.step_list.insertItem(at, self._step_item({"pose": pose, "wait_s": 0.0}))
        self.step_list.setCurrentRow(at)
        self._loading = False
        self._store_steps()

    def _add_pose(self):
        self.add_step(self._selected_pose())

    def _teach(self):
        name = self.current_sequence()
        if name:
            self.gui.save_current_pose(sequence=name)

    # ── Posen ───────────────────────────────────────────────────────────
    def _goto(self):
        pose = self._selected_pose()
        if pose:
            self.gui.run_poses([pose])

    def _delete_pose(self):
        pose = self._selected_pose()
        store = self._store()
        if not pose or store is None:
            return
        used = [n for n, steps in store.list_sequences() if any(s["pose"] == pose for s in steps)]
        extra = (f"\n\nSie wird in {len(used)} Ablauf/Abläufen benutzt ({', '.join(used)}); "
                 "dort erscheint sie dann als FEHLT." if used else "")
        if QMessageBox.question(self, "Pose löschen", f"Pose »{pose}« löschen?{extra}"
                                ) != QMessageBox.StandardButton.Yes:
            return
        self._try(lambda s: s.delete(pose))


def _comp_short(comps):
    """['left_arm', 'right_hand'] -> 'Arm L, Hand R'."""
    names = {"left_arm": "Arm L", "right_arm": "Arm R", "left_hand": "Hand L", "right_hand": "Hand R"}
    return ", ".join(names.get(c, c) for c in comps) or "–"

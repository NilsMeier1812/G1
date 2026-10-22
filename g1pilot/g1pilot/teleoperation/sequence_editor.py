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
          anfahren, loeschen, aktuelle Stellung speichern; darueber die
          RELATIVEN BEWEGUNGEN (z. B. »Box anheben« = Haende 5 cm hoch, egal
          wo sie gerade sind): neu, bearbeiten, loeschen, hinzufuegen

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
    QMessageBox, QDoubleSpinBox, QAbstractItemView, QGroupBox, QFormLayout,
    QLineEdit, QCheckBox, QDialogButtonBox,
)

from g1pilot.manipulation.pose_store import MAX_MOVE_M, clean_move
from g1pilot.teleoperation.sequence_runner import pretty_pose_name, step_name

MOVES_GROUP = "↕ Relative Bewegungen"

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
        box_pose = QGroupBox("Posen & relative Bewegungen")
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
        prow3 = QHBoxLayout()
        self.btn_new_move = QPushButton("Neue relative Bewegung …")
        self.btn_edit_move = QPushButton("Bearbeiten …")
        prow3.addWidget(self.btn_new_move, 2)
        prow3.addWidget(self.btn_edit_move, 1)
        lp.addLayout(prow3)
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
        self.btn_new_move.clicked.connect(lambda: self._edit_move(None))
        self.btn_edit_move.clicked.connect(
            lambda: self._edit_move((self._selected_item() or {}).get("move")))
        self.reload()

    # ── Daten ───────────────────────────────────────────────────────────
    def _store(self):
        return self.gui._pose_store()

    def current_sequence(self):
        it = self.seq_list.currentItem()
        return None if it is None else it.data(ROLE)

    def _selected_item(self):
        """-> {"pose": n} / {"move": n} der Auswahl rechts oder None."""
        it = self.pose_tree.currentItem()
        return None if it is None else it.data(0, ROLE)

    def _selected_pose(self):
        return (self._selected_item() or {}).get("pose")

    def reload(self, select=None):
        """Alles neu aus der Datei lesen (auch nach Speichern von aussen).
        select: Ablauf, der danach gewaehlt sein soll (Default: der bisherige)."""
        store = self._store()
        keep = select if select is not None else self.current_sequence()
        seqs = store.list_sequences() if store is not None else []
        grouped = store.list_grouped() if store is not None else {}
        moves = store.list_moves() if store is not None else {}
        self._known = {n for names in grouped.values() for n in names}
        self._moves = moves
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
        keep_item = self._selected_item()
        self.pose_tree.clear()
        top = QTreeWidgetItem([MOVES_GROUP])
        top.setFlags(top.flags() & ~Qt.ItemFlag.ItemIsSelectable)
        top.setToolTip(0, "Verschieben die Hände relativ zur aktuellen Stellung.")
        for n, mv in moves.items():
            child = QTreeWidgetItem([f"{n}   [{move_text(mv)}]"])
            child.setData(0, ROLE, {"move": n})
            top.addChild(child)
            if keep_item == {"move": n}:
                self.pose_tree.setCurrentItem(child)
        if not moves:
            empty = QTreeWidgetItem(["(noch keine — »Neue relative Bewegung …«)"])
            empty.setFlags(empty.flags() & ~Qt.ItemFlag.ItemIsSelectable)
            top.addChild(empty)
        self.pose_tree.addTopLevelItem(top)
        for cat, names in grouped.items():
            top = QTreeWidgetItem([cat])
            top.setFlags(top.flags() & ~Qt.ItemFlag.ItemIsSelectable)
            for n in names:
                comps = store.components(n) or []
                child = QTreeWidgetItem([f"{n}   [{_comp_short(comps)}]"])
                child.setData(0, ROLE, {"pose": n})
                child.setToolTip(0, n)
                top.addChild(child)
                if keep_item == {"pose": n}:
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
            if "move" in st:
                mv = self._moves.get(st["move"])
                missing = mv is None
                text = f"{i + 1}. ↕ {st['move']}"
                if mv is not None:
                    text += f"   ({move_text(mv)})"
            else:
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
        item = self._selected_item()
        self.btn_add.setEnabled(seq and item is not None)
        self.btn_goto.setEnabled(item is not None)
        self.btn_del_pose.setEnabled(item is not None)
        self.btn_edit_move.setEnabled(bool(item and "move" in item))

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

    def add_step(self, item):
        """Pose (Name oder {"pose"}) bzw. Bewegung ({"move"}) hinter dem
        markierten Schritt einfuegen (sonst ans Ende)."""
        if isinstance(item, str):
            item = {"pose": item}
        if self.current_sequence() is None or not item or not step_name(item):
            return
        row = self.step_list.currentRow()
        at = row + 1 if row >= 0 else self.step_list.count()
        self._loading = True
        self.step_list.insertItem(at, self._step_item(dict(item, wait_s=0.0)))
        self.step_list.setCurrentRow(at)
        self._loading = False
        self._store_steps()

    def _add_pose(self):
        self.add_step(self._selected_item())

    def _teach(self):
        name = self.current_sequence()
        if name:
            self.gui.save_current_pose(sequence=name)

    # ── Posen ───────────────────────────────────────────────────────────
    def _goto(self):
        item = self._selected_item()
        if item:
            self.gui.run_poses([item])

    def _delete_pose(self):
        item = self._selected_item()
        store = self._store()
        if not item or store is None:
            return
        kind = "move" if "move" in item else "pose"
        name = item[kind]
        what = "Bewegung" if kind == "move" else "Pose"
        used = [n for n, steps in store.list_sequences()
                if any(s.get(kind) == name for s in steps)]
        extra = (f"\n\nSie wird in {len(used)} Ablauf/Abläufen benutzt ({', '.join(used)}); "
                 "dort erscheint sie dann als FEHLT." if used else "")
        if QMessageBox.question(self, f"{what} löschen", f"{what} »{name}« löschen?{extra}"
                                ) != QMessageBox.StandardButton.Yes:
            return
        self._try(lambda s: s.delete_move(name) if kind == "move" else s.delete(name))

    def _edit_move(self, name):
        """Relative Bewegung anlegen (name=None) oder aendern."""
        store = self._store()
        if store is None:
            return
        old = store.get_move(name) if name else None
        dlg = MoveDialog(self, name or "", old)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        new_name, mv = dlg.result()
        self._try(lambda s: s.save_move(new_name, old_name=name, **mv))
        self._select_item({"move": new_name})

    def _select_item(self, item):
        for i in range(self.pose_tree.topLevelItemCount()):
            top = self.pose_tree.topLevelItem(i)
            for k in range(top.childCount()):
                if top.child(k).data(0, ROLE) == item:
                    self.pose_tree.setCurrentItem(top.child(k))
                    return


def move_text(mv):
    """{"dx": 0, "dz": 0.05, "sides": [...]} -> 'hoch 5 cm · beide'."""
    words = (("dx", "vor", "zurück"), ("dy", "links", "rechts"), ("dz", "hoch", "runter"))
    parts = [f"{pos if mv[k] > 0 else neg} {abs(mv[k]) * 100:g} cm"
             for k, pos, neg in words if mv.get(k)]
    sides = mv.get("sides", [])
    who = "beide" if len(sides) == 2 else ("links" if sides == ["left"] else "rechts")
    return ", ".join(parts) + f" · {who}"


class MoveDialog(QDialog):
    """Relative Bewegung: Name, Verschiebung in cm (Oberkoerper-Bezug: vor,
    links, hoch positiv), welche Hand/Haende."""

    def __init__(self, parent, name="", move=None):
        super().__init__(parent)
        self.setWindowTitle("Relative Bewegung")
        move = move or {"dx": 0.0, "dy": 0.0, "dz": 0.05, "sides": ["left", "right"]}
        lay = QVBoxLayout(self)
        form = QFormLayout()
        self.ed_name = QLineEdit(name)
        self.ed_name.setPlaceholderText("z. B. Box anheben")
        form.addRow("Name:", self.ed_name)
        self.spins = {}
        for key, label in (("dx", "vor (+) / zurück (−)"), ("dy", "links (+) / rechts (−)"),
                           ("dz", "hoch (+) / runter (−)")):
            sp = QDoubleSpinBox()
            lim = MAX_MOVE_M * 100
            sp.setRange(-lim, lim)
            sp.setDecimals(1)
            sp.setSingleStep(1.0)
            sp.setSuffix(" cm")
            sp.setValue(float(move.get(key, 0.0)) * 100)
            form.addRow(label + ":", sp)
            self.spins[key] = sp
        row = QHBoxLayout()
        self.cb_left = QCheckBox("Linke Hand")
        self.cb_right = QCheckBox("Rechte Hand")
        self.cb_left.setChecked("left" in move["sides"])
        self.cb_right.setChecked("right" in move["sides"])
        row.addWidget(self.cb_left)
        row.addWidget(self.cb_right)
        form.addRow("Hände:", row)
        lay.addLayout(form)
        hint = QLabel("Bezug: Oberkörper des Roboters (bei gedrehter Hüfte mitgedreht). "
                      "Ziel = aktuelle Hand + Verschiebung, Orientierung bleibt; "
                      "die Bahn wird wie bei jeder Pose geplant.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#999;")
        lay.addWidget(hint)
        btns = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                                | QDialogButtonBox.StandardButton.Cancel)
        btns.button(QDialogButtonBox.StandardButton.Ok).setText("Speichern")
        btns.button(QDialogButtonBox.StandardButton.Cancel).setText("Abbrechen")
        btns.accepted.connect(self._accept)
        btns.rejected.connect(self.reject)
        lay.addWidget(btns)

    def result(self):
        sides = [s for s, cb in (("left", self.cb_left), ("right", self.cb_right))
                 if cb.isChecked()]
        return self.ed_name.text().strip(), {
            "dx": self.spins["dx"].value() / 100.0, "dy": self.spins["dy"].value() / 100.0,
            "dz": self.spins["dz"].value() / 100.0, "sides": sides}

    def _accept(self):
        name, mv = self.result()
        try:
            if not name:
                raise ValueError("Bitte einen Namen eingeben.")
            clean_move(**mv)
        except ValueError as e:
            QMessageBox.warning(self, "Relative Bewegung", str(e))
            return
        self.accept()


def _comp_short(comps):
    """['left_arm', 'right_hand'] -> 'Arm L, Hand R'."""
    names = {"left_arm": "Arm L", "right_arm": "Arm R", "left_hand": "Hand L", "right_hand": "Hand R"}
    return ", ".join(names.get(c, c) for c in comps) or "–"

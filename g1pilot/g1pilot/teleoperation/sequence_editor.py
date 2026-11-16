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
  rechts  gespeicherte Posen nach Kategorie: »Hinzufügen« (Doppelklick),
          anfahren, loeschen, aktuelle Stellung speichern; darueber die
          RELATIVEN BEWEGUNGEN (z. B. »Box anheben« = Haende 5 cm hoch, egal
          wo sie gerade sind): neu, bearbeiten, loeschen, hinzufuegen

Zweiter Reiter »Controller-Schnellwahl« (quickslot_panel.py): Posen auf die
16 Plaetze R2 halten + Steuerkreuz zweimal legen.

Jede Aenderung wird sofort gespeichert. Das Fenster ist nicht modal: man
kann nebenher die Hand bewegen (Jog, Controller) und dann die Stellung als
Schritt uebernehmen. Bewegt wird nur ueber die Methoden der Demo-GUI
(run_sequence/run_poses -> geplantes POSE ANFAHREN im arm_controller).
"""
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QIcon
from PyQt6.QtWidgets import (
    QDialog, QHBoxLayout, QVBoxLayout, QGridLayout, QLabel, QListWidget,
    QListWidgetItem, QPushButton, QTreeWidget, QTreeWidgetItem, QInputDialog,
    QMessageBox, QDoubleSpinBox, QAbstractItemView,
    QLineEdit, QCheckBox, QTabWidget, QWidget,
)

from g1pilot.manipulation.pose_store import MAX_MOVE_M, clean_move
from g1pilot.utils import ui_icons
from g1pilot.teleoperation.quickslot_panel import QuickSlotPanel
from g1pilot.teleoperation import dialog_style as ds
from g1pilot.teleoperation.sequence_runner import pretty_pose_name, step_name

MOVES_GROUP = "Relative Bewegungen"
MOVE_ICON = "move"               # Kennzeichen relativer Bewegungen (Baum + Schritte)
SEQ_COLOR = "#66bb6a"            # wie »Abläufe« in der Controller-Belegung
POSE_COLOR = "#ff9800"           # wie GREIFEN

ROLE = Qt.ItemDataRole.UserRole


class SequenceEditor(QDialog):
    """gui: DemoGUI (oder Ersatz in Tests) mit _pose_store(), run_poses(names),
    run_sequence(name), save_current_pose(sequence=None), _reload_sequence_buttons()."""

    def __init__(self, gui, parent=None):
        super().__init__(parent)
        self.gui = gui
        self.setWindowTitle("Posen & Abläufe")
        self.resize(1180, 720)
        # Look wie die anderen Neben-Fenster (Controller-Belegung, Handy):
        # dunkle Karten mit Icon-Kopf (dialog_style.py). Gross genug fuer den
        # Touchscreen am Roboter-Rechner.
        self.setStyleSheet(ds.DIALOG_CSS + "QPushButton { min-height:22px; }")
        self._loading = False

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 18, 20, 16)
        root.setSpacing(12)
        root.addLayout(ds.header(
            "Posen & Abläufe", "layout-grid", ds.ACCENT,
            "Posen speichern und anfahren, zu Abläufen zusammenstellen und auf die "
            "Controller-Schnellwahl legen. Jede Änderung wird sofort gespeichert."))
        self.tabs = QTabWidget()
        root.addWidget(self.tabs, 1)
        page = QWidget()
        page.setObjectName("dbody")
        page_lay = QVBoxLayout(page)
        page_lay.setContentsMargins(0, 12, 0, 0)
        page_lay.setSpacing(10)
        cols = QHBoxLayout()
        cols.setSpacing(12)
        page_lay.addLayout(cols, 1)
        self.tabs.addTab(page, ui_icons.icon("play", ds.FG, 16), "Abläufe und Posen")
        # Reiter 2: Posen auf die 16 Controller-Plaetze legen (R2 + Steuerkreuz 2x).
        self.quick = QuickSlotPanel(gui)
        self.tabs.addTab(self.quick, ui_icons.icon("gamepad-2", ds.FG, 16),
                         "Controller-Schnellwahl")

        # ── Ablaeufe ────────────────────────────────────────────────────
        box_seq, ls = ds.card("Abläufe", "play", SEQ_COLOR, "Folge von Posen und Bewegungen")
        self.seq_list = QListWidget()
        ls.addWidget(self.seq_list, 1)
        g = QGridLayout()
        g.setSpacing(8)
        self.btn_new = ds.button("Neu …", "plus")
        self.btn_rename = ds.button("Umbenennen …", "square-pen")
        self.btn_copy = ds.button("Kopieren …", "folder-open")
        self.btn_del_seq = ds.button("Löschen", "trash-2", color=ds.BAD)
        for i, b in enumerate((self.btn_new, self.btn_rename, self.btn_copy, self.btn_del_seq)):
            g.addWidget(b, i // 2, i % 2)
        ls.addLayout(g)
        self.btn_run = ds.button("Ablauf starten", "play", "primary", "#ffffff")
        self.btn_run.setMinimumHeight(40)
        ls.addWidget(self.btn_run)
        cols.addWidget(box_seq, 3)

        # ── Schritte ────────────────────────────────────────────────────
        self.box_steps, lm = ds.card("Schritte", "arrow-up-down", ds.ACCENT,
                                     "Reihenfolge per Ziehen oder Pfeilen")
        self.steps_title = self.box_steps.findChildren(QLabel)[1]
        self.step_list = QListWidget()
        self.step_list.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
        self.step_list.setDefaultDropAction(Qt.DropAction.MoveAction)
        lm.addWidget(self.step_list, 1)
        row = QHBoxLayout()
        row.setSpacing(8)
        self.btn_up = ds.button("", "chevron-up", size=18)
        self.btn_down = ds.button("", "chevron-down", size=18)
        self.btn_up.setToolTip("Schritt nach oben")
        self.btn_down.setToolTip("Schritt nach unten")
        self.btn_remove = ds.button("Entfernen", "x")
        for b in (self.btn_up, self.btn_down, self.btn_remove):
            row.addWidget(b)
        lm.addLayout(row)
        prow = QHBoxLayout()
        prow.addWidget(QLabel("Pause danach (nur »Am Stück«):"))
        self.spin_wait = QDoubleSpinBox()
        self.spin_wait.setRange(0.0, 600.0)
        self.spin_wait.setDecimals(1)
        self.spin_wait.setSingleStep(0.5)
        self.spin_wait.setSuffix(" s")
        prow.addWidget(self.spin_wait)
        lm.addLayout(prow)
        self.btn_teach = ds.button("Aktuelle Stellung als Schritt …", "plus")
        self.btn_teach.setMinimumHeight(40)
        lm.addWidget(self.btn_teach)
        cols.addWidget(self.box_steps, 4)

        # ── Posen ───────────────────────────────────────────────────────
        box_pose, lp = ds.card("Posen & relative Bewegungen", "hand", POSE_COLOR,
                               "Doppelklick hängt an den Ablauf an")
        self.pose_search = QLineEdit()
        self.pose_search.setPlaceholderText("Pose suchen …")
        self.pose_search.setClearButtonEnabled(True)
        self.pose_search.addAction(ui_icons.icon("search", ds.MUTED, 16),
                                   QLineEdit.ActionPosition.LeadingPosition)
        lp.addWidget(self.pose_search)
        self.pose_tree = QTreeWidget()
        self.pose_tree.setHeaderHidden(True)
        lp.addWidget(self.pose_tree, 1)
        self.btn_add = ds.button("Hinzufügen", "arrow-left", "primary", "#ffffff")
        self.btn_add.setMinimumHeight(40)
        lp.addWidget(self.btn_add)
        prow2 = QHBoxLayout()
        prow2.setSpacing(8)
        self.btn_goto = ds.button("Anfahren", "play")
        self.btn_del_pose = ds.button("Löschen", "trash-2", color=ds.BAD)
        prow2.addWidget(self.btn_goto)
        prow2.addWidget(self.btn_del_pose)
        lp.addLayout(prow2)
        self.btn_save_pose = ds.button("Aktuelle Stellung speichern …", "save")
        lp.addWidget(self.btn_save_pose)
        prow3 = QHBoxLayout()
        prow3.setSpacing(8)
        self.btn_new_move = ds.button("Neue relative Bewegung …", MOVE_ICON)
        self.btn_edit_move = ds.button("Bearbeiten …", "square-pen")
        prow3.addWidget(self.btn_new_move, 2)
        prow3.addWidget(self.btn_edit_move, 1)
        lp.addLayout(prow3)
        cols.addWidget(box_pose, 4)

        self.hint = ds.hint("Doppelklick auf eine Pose hängt sie an. Reihenfolge per Ziehen "
                            "oder mit den Pfeil-Knöpfen. Änderungen werden sofort gespeichert.")
        page_lay.addWidget(self.hint)

        self.pose_search.textChanged.connect(self._filter_poses)
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
        top.setIcon(0, ui_icons.icon(MOVE_ICON, ds.ACCENT, 16))
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
            top.setIcon(0, ui_icons.icon("folder-open", POSE_COLOR, 16))
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
        self._filter_poses(self.pose_search.text())
        self._loading = False
        self._load_steps()
        self.quick.reload()

    def _filter_poses(self, text):
        """Suchfeld: nur passende Posen/Bewegungen zeigen (viele Posen)."""
        needle = (text or "").strip().lower()
        for i in range(self.pose_tree.topLevelItemCount()):
            top = self.pose_tree.topLevelItem(i)
            shown = 0
            for j in range(top.childCount()):
                child = top.child(j)
                hit = not needle or needle in child.text(0).lower() \
                    or needle in top.text(0).lower()
                selectable = child.data(0, ROLE) is not None
                child.setHidden(bool(needle) and not (hit and selectable))
                shown += not child.isHidden()
            top.setHidden(bool(needle) and shown == 0)

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
        self.steps_title.setText(f"Schritte von »{name}«" if name else "Schritte")
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
                text = f"{i + 1}. {st['move']}"
                it.setIcon(ui_icons.icon(MOVE_ICON, ds.ACCENT, 16))
                if mv is not None:
                    text += f"   ({move_text(mv)})"
            else:
                missing = st["pose"] not in self._known
                it.setIcon(QIcon())
                text = f"{i + 1}. {pretty_pose_name(st['pose'])}"
                if pretty_pose_name(st["pose"]) != st["pose"].replace("_", " "):
                    text += f"   ({st['pose']})"
            if st.get("wait_s"):
                text += f"   · Pause {st['wait_s']:g} s"
            if missing:
                text += "   — FEHLT"
            it.setText(text)
            it.setForeground(QColor(ds.BAD) if missing else QColor(ds.FG))

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
        self.setStyleSheet(ds.DIALOG_CSS)
        self.setMinimumWidth(520)
        move = move or {"dx": 0.0, "dy": 0.0, "dz": 0.05, "sides": ["left", "right"]}
        lay = QVBoxLayout(self)
        lay.setContentsMargins(20, 18, 20, 16)
        lay.setSpacing(12)
        lay.addLayout(ds.header("Relative Bewegung", MOVE_ICON, ds.ACCENT,
                                "Hände um ein paar Zentimeter verschieben, egal wo sie gerade "
                                "sind (z. B. »Box anheben« = 5 cm hoch)."))
        card, cl = ds.card("Name", "square-pen")
        self.ed_name = QLineEdit(name)
        self.ed_name.setPlaceholderText("z. B. Box anheben")
        cl.addWidget(self.ed_name)
        lay.addWidget(card)
        card, cl = ds.card("Verschiebung", "move", ds.ACCENT,
                           "Oberkörper-Bezug, bei gedrehter Hüfte mitgedreht")
        form = QGridLayout()
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(10)
        self.spins = {}
        for r, (key, icon, label) in enumerate((
                ("dx", "arrow-up", "vor (+) / zurück (−)"),
                ("dy", "arrow-left", "links (+) / rechts (−)"),
                ("dz", "chevron-up", "hoch (+) / runter (−)"))):
            sp = QDoubleSpinBox()
            lim = MAX_MOVE_M * 100
            sp.setRange(-lim, lim)
            sp.setDecimals(1)
            sp.setSingleStep(1.0)
            sp.setSuffix(" cm")
            sp.setValue(float(move.get(key, 0.0)) * 100)
            sp.setMinimumWidth(130)
            form.setRowMinimumHeight(r, 40)
            vc = Qt.AlignmentFlag.AlignVCenter
            form.addWidget(ds.icon_box(icon, ds.ACCENT, 16, 30), r, 0, vc)
            form.addWidget(QLabel(label), r, 1, vc)
            form.addWidget(sp, r, 2, vc)
            self.spins[key] = sp
        form.setColumnStretch(1, 1)
        cl.addLayout(form)
        lay.addWidget(card)
        card, cl = ds.card("Welche Hände?", "hand", POSE_COLOR)
        row = QHBoxLayout()
        self.cb_left = QCheckBox("Linke Hand")
        self.cb_right = QCheckBox("Rechte Hand")
        self.cb_left.setChecked("left" in move["sides"])
        self.cb_right.setChecked("right" in move["sides"])
        row.addWidget(self.cb_left)
        row.addWidget(self.cb_right)
        row.addStretch(1)
        cl.addLayout(row)
        lay.addWidget(card)
        lay.addWidget(ds.hint("Ziel = aktuelle Hand + Verschiebung, Orientierung bleibt; "
                              f"je Achse höchstens ±{MAX_MOVE_M * 100:.0f} cm. Die Bahn wird "
                              "wie bei jeder Pose geplant (Kollisionsschutz)."))
        lay.addLayout(_dialog_buttons(self, "Speichern", "save", self._accept))

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


def _dialog_buttons(dlg, ok_text, ok_icon, on_ok):
    """Abbrechen (links daneben) + Hauptknopf rechts, wie die Karten-Dialoge."""
    row = QHBoxLayout()
    row.addStretch(1)
    cancel = ds.button("Abbrechen", k="ghost")
    cancel.clicked.connect(dlg.reject)
    ok = ds.button(ok_text, ok_icon, "primary", "#ffffff")
    ok.setDefault(True)
    ok.clicked.connect(on_ok)
    row.addWidget(cancel)
    row.addWidget(ok)
    dlg.btn_ok, dlg.btn_cancel = ok, cancel
    return row


def _comp_short(comps):
    """['left_arm', 'right_hand'] -> 'Arm L, Hand R'."""
    names = {"left_arm": "Arm L", "right_arm": "Arm R", "left_hand": "Hand L", "right_hand": "Hand R"}
    return ", ".join(names.get(c, c) for c in comps) or "–"

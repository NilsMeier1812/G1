"""Ansichten des Startmenues: Menue, Simulation, Echter Roboter, Umgebungen."""
from __future__ import annotations

import os
from functools import partial

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QPainter, QColor
from PyQt6.QtWidgets import (QButtonGroup, QCheckBox, QComboBox, QHBoxLayout,
                             QLineEdit, QPushButton,
                             QScrollArea, QSizePolicy, QVBoxLayout, QWidget)

from g1pilot.utils import msgbox

from . import core, netsetup
from . import theme as T
from .network import LinkMonitor, LinkStatusCard, NetworkSetupDialog
from .widgets import (AddCard, ChoiceGroup, Collapsible, FieldRow, RowList, SceneCard,
                      TileGrid, ToggleRow, button, card, dot, icon_label, label,
                      section_title, set_icon)

NEW_HAND = "Neue Hand anlegen …"            # letzter Eintrag der Custom-Liste
# Ab so vielen Umgebungen gibt es Suche + eingeklappte Ansicht (2 Reihen).
ENV_FILTER_FROM = 7
ENV_COLLAPSED = 7
DEFAULT_ENV_LABEL = "Standard"


def doc_link(text: str, doc: str) -> str:
    return f'<a href="doc:{doc}" style="color:{T.ACCENT};text-decoration:none">{text}</a>'


# ════════════════════════════════════════════════════════════════════════
#  Grundgeruest einer Seite: Kopf, scrollbarer Inhalt, Fusszeile
# ════════════════════════════════════════════════════════════════════════
class Page(QWidget):
    ephemeral = True
    view_title = ""
    help_doc = ""          # z. B. "03-A-Startmenue.md#optionen"

    def __init__(self, app, title: str, subtitle: str, title_color: str = T.FG):
        super().__init__()
        self.app = app
        self.view_title = title
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        head = QWidget()
        head.setObjectName("bar")
        head.setStyleSheet(f"QWidget#bar{{background:{T.SURFACE};border-bottom:1px solid {T.LINE};}}")
        hl = QHBoxLayout(head)
        hl.setContentsMargins(20, 14, 20, 14)
        hl.setSpacing(16)
        hl.addWidget(button("Menü", "ghost", app.show_menu, icon="chevron-left"), 0,
                     Qt.AlignmentFlag.AlignTop)
        col = QVBoxLayout()
        col.setSpacing(0)
        h = label(title, "h1")
        if title_color != T.FG:
            h.setStyleSheet(f"color:{title_color};")
        col.addWidget(h)
        col.addWidget(label(subtitle, "sub"))
        hl.addLayout(col, 1)
        if self.help_doc:
            hb = button("Hilfe", "ghost", lambda: app.open_doc(self.help_doc),
                        icon="circle-help")
            hb.setToolTip("Passende Anleitung im Doku-Viewer öffnen")
            hl.addWidget(hb, 0, Qt.AlignmentFlag.AlignTop)
        outer.addWidget(head)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        inner = QWidget()
        il = QHBoxLayout(inner)
        il.setContentsMargins(24, 4, 24, 24)
        il.addStretch(0)
        content = QWidget()
        content.setMaximumWidth(1100)
        self.body = QVBoxLayout(content)
        self.body.setContentsMargins(0, 0, 0, 0)
        self.body.setSpacing(10)
        il.addWidget(content, 1)
        il.addStretch(0)
        self.scroll.setWidget(inner)
        outer.addWidget(self.scroll, 1)
        self._outer = outer

    def section(self, title: str) -> None:
        self.body.addWidget(section_title(title))

    def hint(self, html: str) -> None:
        lbl = label(html, "hint", wrap=True, rich=True)
        lbl.linkActivated.connect(self.app.handle_link)
        self.body.addWidget(lbl)

    def footer(self, text: str, kind: str, on_click, icon: str = "") -> QPushButton:
        self.body.addStretch(1)
        foot = QWidget()
        foot.setObjectName("bar")
        foot.setStyleSheet(f"QWidget#bar{{background:{T.SURFACE};border-top:1px solid {T.LINE};}}")
        fl = QHBoxLayout(foot)
        fl.setContentsMargins(24, 12, 24, 12)
        fl.setSpacing(12)
        self.summary = label("", "muted")
        self.summary.setWordWrap(True)
        fl.addWidget(self.summary, 1)
        fl.addWidget(button("Abbrechen", "ghost", self.app.show_menu))
        btn = button(text, kind, on_click, icon=icon)
        fl.addWidget(btn)
        self._outer.addWidget(foot)
        return btn


class HandCombo(QComboBox):
    """Liste der Custom-Hände; beim Aufklappen neu einlesen (eben angelegte Hand)."""

    def __init__(self, refresh):
        super().__init__()
        self._refresh = refresh

    def showPopup(self) -> None:  # noqa: N802
        self._refresh()
        super().showPopup()


# ════════════════════════════════════════════════════════════════════════
#  Simulation
# ════════════════════════════════════════════════════════════════════════
class SimView(Page):
    help_doc = "03-A-Startmenue.md#optionen"
    HAND_ICONS = {"rubber": "hand", "inspire": "bot", "custom": "anchor"}

    def __init__(self, app):
        super().__init__(app, "Simulation", "MuJoCo + Whole-Body-Policy — gefahrlos testen.")
        st = core.load_settings("sim")
        self._rviz_own = None   # eigene RViz-Wahl, solange Navigation RViz erzwingt

        # 1) Umgebung -------------------------------------------------------
        self.section("Umgebung")
        scenes = core.list_scenes()
        names = [p.stem for p in scenes]
        env = st.get("env", "")
        self._env = env if env in names else ""
        self.env_group = QButtonGroup(self)
        self.env_group.setExclusive(True)
        self.env_cards: dict[str, SceneCard] = {}
        items = []
        c = SceneCard("", DEFAULT_ENV_LABEL, "scene.xml · Gelände des G1",
                      core.thumb_path(core.STANDARD_THUMB))
        self.env_cards[""] = c
        items.append(c)
        for p in scenes:
            c = SceneCard(p.stem, p.stem, core.scene_summary(p), core.thumb_path(p.stem))
            self.env_cards[p.stem] = c
            items.append(c)
        for c in self.env_cards.values():
            self.env_group.addButton(c)
        self.env_cards[self._env].setChecked(True)
        self._env_text = {k: f"{k} {c.summary}".lower() for k, c in self.env_cards.items()}
        self.env_group.buttonClicked.connect(lambda b: self._sync_summary())
        self.env_add = AddCard("Neue Umgebung")
        self.env_add.setToolTip("Umgebungen anlegen und bearbeiten (Scene-Editor)")
        self.env_add.clicked.connect(app.open_scenes)
        # Viele Umgebungen: Suchfeld + eingeklappt auf 2 Reihen, "Alle anzeigen".
        self._env_expanded = False
        self.env_search = QLineEdit()
        self.env_search.setPlaceholderText(f"{len(scenes) + 1} Umgebungen durchsuchen …")
        self.env_search.setClearButtonEnabled(True)
        self.env_search.textChanged.connect(lambda _t: self._apply_env_view())
        self.env_search.setVisible(len(scenes) + 1 >= ENV_FILTER_FROM)
        self.body.addWidget(self.env_search)
        self.env_grid = TileGrid()
        self.body.addWidget(self.env_grid)
        self.env_more = button("", "ghost", self._toggle_env_expanded)
        row = QHBoxLayout()
        row.addWidget(self.env_more)
        row.addStretch(1)
        self.body.addLayout(row)
        self._apply_env_view()
        self.hint("Der G1 bleibt gleich, nur die Welt wechselt. Vorschaubilder erzeugt das "
                  "Startmenü selbst, sobald eine Umgebung neu ist oder sich ändert.")

        # 2) Lauf-Policy ----------------------------------------------------
        self.section("Lauf-Policy")
        policy = st.get("walk_policy", core.DEFAULT_WALK_POLICY)
        if policy not in {c[0] for c in core.WALK_POLICY_CHOICES}:
            policy = core.DEFAULT_WALK_POLICY
        self.policy = ChoiceGroup(core.WALK_POLICY_CHOICES, policy)
        self.policy.changed.connect(lambda _v: self._sync_summary())
        self.body.addWidget(self.policy)
        self.hint("Stehen ist bei allen gleich (Füße bleiben stehen, Schritte nur beim Kippen). "
                  "Vergleich: " + doc_link("Gangbild", "30-T-Stehen-und-Gehen.md#gangbild"))

        # 3) Haende ---------------------------------------------------------
        self.section("Hände")
        self._customs = core.list_hand_profiles()
        hp = st.get("hand_profile") or ("inspire" if st.get("hands", True) else "rubber")
        cnames = [n for n, _ in self._customs]
        if hp in ("rubber", "inspire"):
            kind = hp
        elif hp in cnames:
            kind = "custom"
        else:
            kind = "inspire"
        self.hand_kind = ChoiceGroup(core.HAND_KIND_CHOICES, kind, self.HAND_ICONS)
        self.hand_kind.changed.connect(lambda _v: self._sync_hands())
        self.body.addWidget(self.hand_kind)

        self.custom_box = RowList()
        row = QWidget()
        rl = QHBoxLayout(row)
        rl.setContentsMargins(0, 8, 0, 8)
        rl.setSpacing(10)
        rl.addWidget(label("Custom Hand"))
        self.hand_combo = HandCombo(self._refresh_customs)
        self.hand_combo.setMinimumWidth(280)
        rl.addWidget(self.hand_combo, 1)
        self.edit_btn = button("Bearbeiten …", "", self._edit_hand)
        rl.addWidget(self.edit_btn)
        self.custom_box.add(row)
        hint = label("Bearbeiten öffnet das Bearbeitungstool (MuJoCo-Ansicht + Schieber für "
                     "Versatz und Drehung). »Neue Hand anlegen«: 3D-Datei wählen, das Tool "
                     "schlägt die Montage am Handgelenk vor. Anleitung: "
                     + doc_link("Eigene Hände", "25-A-Eigene-Haende.md"),
                     "hint", wrap=True, rich=True)
        hint.linkActivated.connect(app.handle_link)
        hint.setContentsMargins(0, 0, 0, 8)
        self.custom_box.add(hint)
        self.body.addWidget(self.custom_box)
        last_custom = hp if hp in cnames else st.get("hand_custom", "")
        self._fill_customs(dict(self._customs).get(last_custom, ""))
        self.hand_combo.currentTextChanged.connect(lambda _t: self._sync_hands())

        # 4) Ausstattung ----------------------------------------------------
        self.section("Ausstattung")
        rows = RowList()
        self.nav = rows.add(ToggleRow("Navigation", "Planer + Stationen / AUTO NAV in der "
                                      "Demo-GUI", st.get("nav", False)))
        self.ps4 = rows.add(ToggleRow("Controller (PS4 / Xbox)",
                                      "Arme + Hände, Gehen mit L2/LT", st.get("ps4", False)))
        self.rviz = rows.add(ToggleRow("RViz", "Zusatzfenster mit TF/Markern; das "
                                       "MuJoCo-Fenster kommt immer", st.get("rviz", False)))
        self.body.addWidget(rows)
        self.nav.toggled.connect(lambda _v: self._sync())
        for r in (self.ps4, self.rviz):
            r.toggled.connect(lambda _v: self._sync_summary())
        self.hint("Controller-Belegung: " + doc_link("Controller-Anleitung",
                                                     "11-A-Controller.md"))

        # 5) Erweitert ------------------------------------------------------
        adv = Collapsible("Erweitert")
        self.rt = QLineEdit(st.get("rt", "1.0"))
        self.rt.setFixedWidth(90)
        adv.body.add(FieldRow("Sim-Tempo", self.rt, "1.0 = Echtzeit, kleiner = Zeitlupe "
                              "(Obergrenze; langsame PCs laufen ohnehin langsamer)"))
        self.rebuild = adv.body.add(ToggleRow("Docker-Images neu bauen", "nur nach Änderungen "
                                              "an Dockerfiles/Abhängigkeiten", False))
        self.body.addWidget(adv)
        self.advanced = adv

        self.start_btn = self.footer("Simulation starten", "success", self._start, icon="play")
        self._sync_hands()
        self._sync()
        app.thumbs_changed.connect(self._reload_thumbs)

    # ── Umgebungen: Suche + Einklappen ───────────────────────────────────
    def visible_envs(self) -> list[str]:
        """Welche Kacheln gerade gezeigt werden (Reihenfolge wie im Raster)."""
        q = self.env_search.text().strip().lower() if not self.env_search.isHidden() else ""
        keys = [k for k in self.env_cards if not q or q in self._env_text[k]
                or (k == "" and q in DEFAULT_ENV_LABEL.lower())]
        if q or self._env_expanded or len(keys) <= ENV_COLLAPSED + 1:
            return keys
        shown = keys[:ENV_COLLAPSED]
        sel = self.env_name()
        if sel not in shown:          # gewaehlte Umgebung immer sichtbar
            shown[-1] = sel
        return shown

    def _apply_env_view(self) -> None:
        keys = self.visible_envs()
        items = [self.env_cards[k] for k in keys]
        if not self.env_search.text().strip():
            items.append(self.env_add)
        self.env_grid.set_items(items, delete=False)
        total = len(self.env_cards)
        q = self.env_search.text().strip()
        if q:
            self.env_more.setVisible(False)
        elif total > ENV_COLLAPSED + 1:
            self.env_more.setVisible(True)
            self.env_more.setText("Weniger anzeigen" if self._env_expanded
                                  else f"Alle {total} Umgebungen anzeigen")
            set_icon(self.env_more, "chevron-down" if not self._env_expanded
                     else "chevron-right", T.MUTED, 16)
        else:
            self.env_more.setVisible(False)

    def _toggle_env_expanded(self) -> None:
        self._env_expanded = not self._env_expanded
        self._apply_env_view()

    # ── Haende ───────────────────────────────────────────────────────────
    def _fill_customs(self, select_title: str = "") -> None:
        self.hand_combo.blockSignals(True)
        cur = select_title or self.hand_combo.currentText()
        self.hand_combo.clear()
        self.hand_combo.addItems([t for _, t in self._customs] + [NEW_HAND])
        idx = self.hand_combo.findText(cur)
        self.hand_combo.setCurrentIndex(idx if idx >= 0 else 0)
        self.hand_combo.blockSignals(False)

    def _refresh_customs(self) -> None:
        self._customs = core.list_hand_profiles()
        self._fill_customs()

    def _custom_name(self) -> str:
        title = self.hand_combo.currentText()
        return next((n for n, t in self._customs if t == title), "")

    def hand_profile(self) -> str:
        """Wert fuer G1_HANDS: rubber | inspire | <custom-profil>."""
        kind = self.hand_kind.value()
        if kind == "custom":
            return self._custom_name() or "rubber"
        return kind

    def _sync_hands(self) -> None:
        self.custom_box.setVisible(self.hand_kind.value() == "custom")
        new = self.hand_combo.currentText() == NEW_HAND
        self.edit_btn.setText("Anlegen …" if new else "Bearbeiten …")
        self._sync_summary()

    def _edit_hand(self) -> None:
        if self.hand_combo.currentText() == NEW_HAND:
            self.app.start_process("Neue Hand anlegen", ["bash", str(core.HAND_TOOL_SH), "new"],
                                   cwd=core.HAND_TOOL_SH.parent, env=os.environ.copy(),
                                   stop_note="Schließt »Neue Hand anlegen« bzw. das "
                                             "Bearbeitungstool danach.")
            return
        name = self._custom_name()
        if not name:
            return
        self.app.start_process(f"Hand bearbeiten — {self.hand_combo.currentText()}",
                               ["bash", str(core.HAND_TOOL_SH), "adjust", name],
                               cwd=core.HAND_TOOL_SH.parent, env=os.environ.copy(),
                               stop_note="Schließt das Bearbeitungstool (nicht "
                                         "Gespeichertes geht verloren).")

    # ── Abhaengigkeiten ──────────────────────────────────────────────────
    def _sync(self) -> None:
        # Navigation braucht RViz (Karte/Ziel-Werkzeug leben dort) -> erzwungen;
        # ohne Navigation wieder die eigene RViz-Wahl.
        nav = self.nav.isChecked()
        self.rviz.setEnabled(not nav)
        if nav and self._rviz_own is None:
            self._rviz_own = self.rviz.isChecked()
            self.rviz.setChecked(True)
            self.rviz.setHint("bei Navigation immer an (Karte und Ziel-Werkzeug leben dort)")
        elif not nav and self._rviz_own is not None:
            self.rviz.setChecked(self._rviz_own)
            self._rviz_own = None
            self.rviz.setHint("Zusatzfenster mit TF/Markern; das MuJoCo-Fenster kommt immer")
        self._sync_summary()

    def env_name(self) -> str:
        b = self.env_group.checkedButton()
        return b.value if b else ""

    def _sync_summary(self) -> None:
        if not hasattr(self, "summary"):
            return
        parts = [self.env_name() or DEFAULT_ENV_LABEL,
                 dict((v, t) for v, t, _ in core.WALK_POLICY_CHOICES).get(self.policy.value(), ""),
                 {"rubber": "Rubber", "inspire": "Inspire"}.get(
                     self.hand_kind.value(), self.hand_combo.currentText()
                     if self._custom_name() else "Custom ?")]
        for row, name in ((self.nav, "Navigation"), (self.ps4, "Controller"),
                          (self.rviz, "RViz")):
            if row.isChecked():
                parts.append(name)
        self.summary.setText("  ·  ".join(p for p in parts if p))

    def _reload_thumbs(self) -> None:
        for name, c in self.env_cards.items():
            c.set_thumb(core.thumb_path(name or core.STANDARD_THUMB))

    # ── Start ────────────────────────────────────────────────────────────
    def _start(self) -> None:
        rt = self.rt.text().strip() or "1.0"
        try:
            float(rt)
        except ValueError:
            self.advanced.set_open(True)
            msgbox.error(self, "Ungültig", "Sim-Tempo muss eine Zahl sein (z. B. 1.0).")
            return
        if self.hand_kind.value() == "custom" and not self._custom_name():
            msgbox.error(self, "Keine Custom Hand",
                                 "Bitte eine eigene Hand auswählen oder erst mit "
                                 "»Neue Hand anlegen« → »Anlegen …« eine anlegen.")
            return
        hp = self.hand_profile()
        rviz = self.rviz.isChecked()
        core.save_settings("sim", {
            "env": self.env_name(), "walk_policy": self.policy.value(),
            "hands": hp == "inspire", "hand_profile": hp,
            "hand_custom": self._custom_name(), "nav": self.nav.isChecked(),
            "rviz": rviz if self._rviz_own is None else self._rviz_own,
            "rt": rt, "ps4": self.ps4.isChecked(),
        })
        L = core.sim_launch(env_name=self.env_name(), policy=self.policy.value(),
                            hand_profile=hp, nav=self.nav.isChecked(), ps4=self.ps4.isChecked(),
                            rviz=rviz, rt=rt, rebuild=self.rebuild.isChecked())
        self.app.start_process(L.title, L.argv, cwd=core.HERE, env=L.env,
                               stop_cmd=L.stop_cmd, stop_note=L.stop_note)


# ════════════════════════════════════════════════════════════════════════
#  Echter Roboter
# ════════════════════════════════════════════════════════════════════════
class RealView(Page):
    """Verbindung -> Ausstattung -> Geh-Limits -> Erweitert -> Sicherheits-
    Bestaetigung. Die Bestaetigung wird NIE gemerkt."""

    help_doc = "50-A-Echter-Roboter.md"

    def __init__(self, app, nics=None, probe=None, setup_host=None):
        super().__init__(app, "Echter Roboter",
                         "G1 per LAN: Unitree-Loco + Arme + Hände. Der Roboter bewegt sich!",
                         T.RED)
        st = core.load_settings("real")
        self._nics = core.detect_nics() if nics is None else nics
        if probe is None:
            if nics is None:
                probe = netsetup.probe
            else:   # feste Testliste: nichts anpingen, kein Kabel-Status
                fixed = list(nics)
                probe = partial(netsetup.probe, nics_fn=lambda: fixed,
                                ping_fn=lambda _h: None, tcp_fn=lambda _h, _p: False,
                                carrier_fn=lambda _i: None)
        self._setup_host = setup_host          # Tests: Ergebnis von inspect_host()

        # 1) Verbindung -----------------------------------------------------
        self.section("Verbindung zum G1")
        rows = RowList()
        self.iface = QComboBox()
        self.iface.setEditable(True)
        self.iface.setMinimumWidth(300)
        default = ""
        self._iface_map: dict[str, str] = {}
        for name, ip in self._nics:
            lbl = f"{name}  ({ip or 'keine IPv4'})"
            self.iface.addItem(lbl)
            self._iface_map[lbl] = name
            if ip.startswith("192.168.123.") or (not default and name == st.get("iface")):
                default = lbl
        if default:
            self.iface.setCurrentText(default)
        elif not self._nics:
            self.iface.setCurrentText(st.get("iface", ""))
        rows.add(FieldRow("Netzwerk-Interface", self.iface,
                          "Kabel zum Roboter, Adresse im Netz 192.168.123.x"))
        self.link = LinkStatusCard(self._open_netsetup, lambda: self.monitor.check_now())
        self.lan = self.link.text
        rows.add(self.link)
        self.body.addWidget(rows)
        self.iface.currentTextChanged.connect(lambda _t: self._sync_lan())
        # Verbindung laufend im Hintergrund pruefen (nur solange sichtbar).
        self.monitor = LinkMonitor(probe, self._probe_params, parent=self)
        self.monitor.updated.connect(self._on_link)

        # 2) Ausstattung ----------------------------------------------------
        self.section("Ausstattung")
        rows = RowList()
        self.hands = rows.add(ToggleRow("Inspire-Hände", "Modbus TCP im Roboter-LAN",
                                        st.get("hands", True)))
        self.left = QLineEdit(st.get("left", "192.168.123.210"))
        self.right = QLineEdit(st.get("right", "192.168.123.211"))
        for e in (self.left, self.right):
            e.setFixedWidth(180)
        self.left_row = rows.add(FieldRow("IP linke Hand", self.left))
        self.right_row = rows.add(FieldRow("IP rechte Hand", self.right))
        self.rviz = rows.add(ToggleRow("RViz", "empfohlen: Arm-Marker + Zustand sichtbar",
                                       st.get("rviz", True)))
        self.ps4 = rows.add(ToggleRow("Controller (PS4 / Xbox) für Arme + Hände",
                                      "aus = Gehen mit Deadman (alter Joystick, nur PS4)",
                                      st.get("ps4", False)))
        self.body.addWidget(rows)
        self.hands.toggled.connect(lambda _v: self._sync())
        for e in (self.left, self.right):
            e.editingFinished.connect(self._sync)

        # 3) Geh-Limits -----------------------------------------------------
        self.section("Geh-Limits (konservativ für die ersten Tests)")
        rows = RowList()
        self.vx = QLineEdit(st.get("vx", "0.4"))
        self.vy = QLineEdit(st.get("vy", "0.3"))
        self.vyaw = QLineEdit(st.get("vyaw", "0.4"))
        for title, e, unit in (("vorwärts", self.vx, "m/s"), ("seitlich", self.vy, "m/s"),
                               ("drehen", self.vyaw, "rad/s")):
            e.setFixedWidth(90)
            rows.add(FieldRow(title, e, unit=unit))
        self.body.addWidget(rows)

        # 4) Erweitert ------------------------------------------------------
        adv = Collapsible("Erweitert")
        self.port = QLineEdit(st.get("port", "6000"))
        self.port.setFixedWidth(90)
        self.joy = QLineEdit(st.get("joy", "Wireless Controller"))
        self.joy.setFixedWidth(220)
        adv.body.add(FieldRow("Modbus-Port der Hände", self.port))
        adv.body.add(FieldRow("Gamepad-Name (evdev)", self.joy, "nur alter Joystick-Modus"))
        self.rebuild = adv.body.add(ToggleRow("Docker-Images neu bauen", "nur nach Änderungen "
                                              "an Dockerfiles/Abhängigkeiten", False))
        self.body.addWidget(adv)

        # 5) Sicherheit -----------------------------------------------------
        gate, gl = card(name="danger")
        gl.setContentsMargins(18, 14, 18, 14)
        t = label("ACHTUNG: ECHTER ROBOTER")
        t.setStyleSheet(f"color:{T.RED}; font-weight:700; font-size:11pt;")
        gl.addWidget(t)
        info = label("Kein Auto-Start: der Roboter bewegt sich erst, wenn er in der Demo-GUI "
                     "gestartet wird (»Start« in der Startleiste).<br>"
                     "NOT-HALT: Arme schlaff, Beine bleiben stehen.<br>"
                     "Voll-Notaus (Controller R1+L1+PS 2 s) = Damp = sackt in den Kran.<br>"
                     "Checkliste vorher: " + doc_link("Echter Roboter — Sicherheit & Ablauf",
                                                      "50-A-Echter-Roboter.md"),
                     wrap=True, rich=True)
        info.linkActivated.connect(app.handle_link)
        gl.addWidget(info)
        self.confirm = QCheckBox("Ich habe die Sicherheitshinweise gelesen und der Bereich "
                                 "ist frei.")
        self.confirm.setCursor(Qt.CursorShape.PointingHandCursor)
        gl.addSpacing(4)
        gl.addWidget(self.confirm)
        self.body.addSpacing(10)
        self.body.addWidget(gate)

        self.start_btn = self.footer("Echten Roboter starten", "danger", self._start, icon="power")
        self.start_btn.setEnabled(False)
        self.confirm.toggled.connect(self._sync_confirm)
        self._sync_lan()
        self._sync()

    def _iface(self) -> str:
        text = self.iface.currentText().strip()
        return self._iface_map.get(text, text.split()[0] if text else "")

    def _sync_lan(self) -> None:
        """Sofort-Anzeige aus der bekannten NIC-Liste (ohne Ping); die
        Hintergrund-Pruefung ergaenzt Kabel, G1 und Haende."""
        iface = self._iface()
        st = netsetup.LinkStatus(iface=iface, ip=dict(self._nics).get(iface, ""),
                                 nics=self._nics,
                                 others=[n for n in netsetup.robot_ifaces(self._nics)
                                         if n != iface])
        self.link.set_plain(st.text(), st.level)
        if hasattr(self, "summary"):
            self.summary.setText(f"Interface: {iface or '—'}")
        if hasattr(self, "monitor") and self.monitor.running:
            self.monitor.check_now()

    def _probe_params(self) -> dict:
        hands = []
        if self.hands.isChecked():
            hands = [("Hand links", self.left.text().strip() or "192.168.123.210"),
                     ("Hand rechts", self.right.text().strip() or "192.168.123.211")]
        try:
            port = int(self.port.text().strip() or "6000")
        except ValueError:
            port = 6000
        return {"iface": self._iface(), "hands": hands, "port": port}

    def _on_link(self, st) -> None:
        self.monitor.finish()
        if st.nics and st.nics != self._nics:
            self._set_nics(st.nics)
        if st.iface == self._iface():
            self.link.show_status(st)
            self.last_link = st

    def _set_nics(self, nics) -> None:
        """NIC-Liste erneuern (USB-Adapter eingesteckt, IP vergeben), Auswahl
        behalten. Nicht, solange jemand im Feld tippt oder die Liste offen ist."""
        if self.iface.lineEdit().hasFocus() or self.iface.view().isVisible():
            return
        cur = self._iface()
        self._nics = list(nics)
        self.iface.blockSignals(True)
        self.iface.clear()
        self._iface_map = {}
        sel = ""
        for name, ip in self._nics:
            lbl = f"{name}  ({ip or 'keine IPv4'})"
            self.iface.addItem(lbl)
            self._iface_map[lbl] = name
            if name == cur:
                sel = lbl
        self.iface.setCurrentText(sel or cur)
        self.iface.blockSignals(False)

    def _open_netsetup(self) -> None:
        dlg = NetworkSetupDialog(self, self._nics, self._iface(), host=self._setup_host)
        dlg.applied.connect(self._after_netsetup)
        self.netsetup_dialog = dlg
        msgbox.run_dialog(dlg)

    def _after_netsetup(self) -> None:
        self._set_nics(core.detect_nics()) if self._setup_host is None else None
        self.monitor.check_now()

    def showEvent(self, e) -> None:  # noqa: N802
        super().showEvent(e)
        if not self.monitor.running:
            self.monitor.start()

    def hideEvent(self, e) -> None:  # noqa: N802
        self.monitor.stop()
        super().hideEvent(e)

    def _sync(self) -> None:
        on = self.hands.isChecked()
        self.left_row.setEnabled(on)
        self.right_row.setEnabled(on)
        if hasattr(self, "monitor") and self.monitor.running:
            self.monitor.check_now()

    def _sync_confirm(self) -> None:
        self.start_btn.setEnabled(self.confirm.isChecked())

    def _start(self) -> None:
        if not self.confirm.isChecked():
            return
        iface = self._iface()
        if not iface:
            msgbox.error(self, "Interface fehlt",
                                 "Bitte ein Netzwerk-Interface wählen oder eintippen.")
            return
        vals = {
            "iface": iface, "hands": self.hands.isChecked(),
            "left": self.left.text().strip() or "192.168.123.210",
            "right": self.right.text().strip() or "192.168.123.211",
            "port": self.port.text().strip() or "6000",
            "rviz": self.rviz.isChecked(),
            "vx": self.vx.text().strip() or "0.4",
            "vy": self.vy.text().strip() or "0.3",
            "vyaw": self.vyaw.text().strip() or "0.4",
            "joy": self.joy.text().strip() or "Wireless Controller",
            "ps4": self.ps4.isChecked(),
        }
        core.save_settings("real", vals)
        L = core.real_launch(vals, rebuild=self.rebuild.isChecked())
        self.app.start_process(L.title, L.argv, cwd=core.HERE, env=L.env,
                               stop_cmd=L.stop_cmd, stop_note=L.stop_note)


# ════════════════════════════════════════════════════════════════════════
#  Umgebungen (scene_editor)
# ════════════════════════════════════════════════════════════════════════
class ScenesView(Page):
    help_doc = "40-A-Simulation.md#umgebungen"

    def __init__(self, app):
        super().__init__(app, "Umgebungen", "Szenen aus scene_editor/scenes/ — dieselben, "
                                            "die beim Sim-Start wählbar sind.")
        self.inspire = None
        if not core.scene_editor_ready():
            self.section("Einmaliges Setup nötig")
            c, cl = card()
            cl.addWidget(label("Der Scene-Editor braucht ein virtualenv (scene_editor/.venv). "
                               "Das wird einmalig eingerichtet (Internet nötig, danach "
                               "gecacht).", wrap=True))
            row = QHBoxLayout()
            row.addWidget(button("Setup jetzt ausführen", "primary", self._run_setup))
            row.addStretch(1)
            cl.addLayout(row)
            self.body.addWidget(c)
            self.body.addStretch(1)
            return

        self.section("Vorhandene Umgebungen")
        self.group = QButtonGroup(self)
        self.group.setExclusive(True)
        self.search = QLineEdit()
        self.search.setPlaceholderText("Umgebungen durchsuchen …")
        self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(lambda _t: self._apply_filter())
        self.body.addWidget(self.search)
        self.grid = TileGrid()
        self.body.addWidget(self.grid)
        self.cards: dict[str, SceneCard] = {}
        self._reload_scenes()
        tools = QHBoxLayout()
        tools.addWidget(button("Liste aktualisieren", "ghost", self._reload_scenes,
                               icon="refresh-cw"))
        tools.addWidget(button("Vorschaubilder neu erzeugen", "ghost",
                               lambda: app.refresh_thumbs(force=True), icon="box"))
        tools.addStretch(1)
        self.body.addLayout(tools)

        self.section("Aktion für die gewählte Umgebung")
        act = QHBoxLayout()
        act.setSpacing(10)
        for text, cmd, kind, ic in (("Im Editor bearbeiten", "edit", "primary", "square-pen"),
                                    ("Mit G1 ansehen", "with-g1", "primary", "bot"),
                                    ("Nur Umgebung ansehen", "view", "", "eye")):
            b = button(text, kind, lambda c=cmd: self._run_scene(c), icon=ic)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            act.addWidget(b)
        self.body.addLayout(act)
        rows = RowList()
        self.inspire = rows.add(ToggleRow("Bei »Mit G1 ansehen« die Inspire-Hände laden", "",
                                          core.load_settings("sim").get("hands", True)))
        self.body.addWidget(rows)

        self.section("Neue Umgebung")
        c, cl = card()
        row = QHBoxLayout()
        row.addWidget(button("Leere Umgebung im Editor", "primary",
                             lambda: self._run_cmd(["new"], "Neue Umgebung", browser=True),
                             icon="plus"))
        row.addWidget(button("Aus Text-Prompt generieren", "", self._run_prompt))
        row.addStretch(1)
        cl.addLayout(row)
        cl.addWidget(label("Der Editor öffnet einen lokalen Webserver (http://127.0.0.1:8080) "
                           "im Browser. Export landet automatisch in scenes/.", "hint",
                           wrap=True))
        self.body.addWidget(c)
        app.thumbs_changed.connect(self._update_thumbs)

    def _reload_scenes(self) -> None:
        sel = self.selected_scene()
        for b in list(self.group.buttons()):
            self.group.removeButton(b)
        for c in self.cards.values():      # auch per Filter ausgeblendete loeschen
            c.setParent(None)
            c.deleteLater()
        self.cards = {}
        self._scenes = core.list_scenes()
        items = []
        for p in self._scenes:
            c = SceneCard(p.stem, p.stem, core.scene_summary(p), core.thumb_path(p.stem))
            self.group.addButton(c)
            self.cards[p.stem] = c
            items.append(c)
        if not items:
            items.append(label("(noch keine Umgebungen — »Leere Umgebung im Editor« anlegen)",
                               "muted"))
        self.grid.set_items(items)
        if self.cards:
            keep = sel.stem if sel is not None and sel.stem in self.cards else \
                next(iter(self.cards))
            self.cards[keep].setChecked(True)
        self.search.setVisible(len(self.cards) >= ENV_FILTER_FROM)
        self._apply_filter()

    def _apply_filter(self) -> None:
        q = self.search.text().strip().lower() if self.search.isVisible() else ""
        if not self.cards:
            return
        shown = [c for k, c in self.cards.items()
                 if not q or q in f"{k} {c.summary}".lower()]
        self.grid.set_items(shown, delete=False)

    def _update_thumbs(self) -> None:
        for name, c in self.cards.items():
            c.set_thumb(core.thumb_path(name))

    def selected_scene(self):
        if not getattr(self, "_scenes", None):
            return None
        b = self.group.checkedButton()
        return next((p for p in self._scenes if b is not None and p.stem == b.value), None)

    def _run_setup(self) -> None:
        self.app.start_process("Scene-Editor Setup", ["bash", str(core.SETUP_SH)],
                               cwd=core.SCENE_DIR, env=os.environ.copy(),
                               stop_note="Bricht das Setup ab.")

    def _run_scene(self, cmd: str) -> None:
        scene = self.selected_scene()
        if scene is None:
            msgbox.info(self, "Keine Auswahl", "Bitte zuerst eine Umgebung wählen.")
            return
        env = os.environ.copy()
        env["G1_INSPIRE_HANDS"] = "1" if self.inspire.isChecked() else "0"
        title = {"edit": "Editor", "view": "Viewer", "with-g1": "Viewer + G1"}[cmd]
        # 'edit' startet den Web-Editor -> Browser oeffnen; view/with-g1 sind MuJoCo-Fenster.
        self._run_cmd([cmd, str(scene)], f"{title}: {scene.stem}", env=env,
                      browser=(cmd == "edit"))

    def _run_prompt(self) -> None:
        text, ok = msgbox.ask_text(self, "Umgebung aus Text",
                                   "Beschreibe die Umgebung (Englisch funktioniert "
                                   "am besten):", ok="Erzeugen")
        if not ok or not text.strip():
            return
        self._run_cmd(["prompt", text.strip()], "Umgebung generieren", browser=True)

    def _run_cmd(self, args: list, title: str, env: dict | None = None,
                 browser: bool = False) -> None:
        self.app.start_process(f"Scene-Editor — {title}",
                               ["bash", str(core.LAUNCH_SH), *args],
                               cwd=core.SCENE_DIR, env=env or os.environ.copy(),
                               stop_note="Beendet den Editor/Viewer-Prozess.",
                               browser_url="http://127.0.0.1:8080" if browser else None,
                               scene_editor=True)


# ════════════════════════════════════════════════════════════════════════
#  Hauptmenue
# ════════════════════════════════════════════════════════════════════════
class Tile(QPushButton):
    """Grosse Karte im Hauptmenue: Farbbalken links, Symbol, Titel, Text."""

    def __init__(self, title: str, text: str, color: str, symbol: str):
        super().__init__()
        self.setObjectName("tile")
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self._color = color
        lay = QHBoxLayout(self)
        lay.setContentsMargins(26, 18, 18, 18)
        lay.setSpacing(16)
        c = QColor(color)
        lay.addWidget(icon_label(symbol, color, 24, box=48,
                                 bg=f"rgba({c.red()},{c.green()},{c.blue()},40)"))
        col = QVBoxLayout()
        col.setSpacing(2)
        t = label(title)
        f = t.font()
        f.setPointSizeF(14)
        f.setWeight(QFont.Weight.DemiBold)
        t.setFont(f)
        col.addWidget(t)
        col.addWidget(label(text, "sub", wrap=True))
        lay.addLayout(col, 1)
        lay.addWidget(icon_label("chevron-right", T.MUTED, 22))
        for w in self.findChildren(QWidget):
            w.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)

    def sizeHint(self):
        return self.layout().sizeHint()

    def paintEvent(self, e) -> None:
        super().paintEvent(e)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(self._color))
        p.drawRoundedRect(0, 14, 5, self.height() - 28, 2, 2)


class MenuView(QWidget):
    ephemeral = True
    view_title = "Startmenü"

    def __init__(self, app):
        super().__init__()
        self.app = app
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        outer.addWidget(scroll)
        inner = QWidget()
        il = QHBoxLayout(inner)
        il.setContentsMargins(32, 28, 32, 24)
        content = QWidget()
        content.setMaximumWidth(860)
        b = QVBoxLayout(content)
        b.setContentsMargins(0, 0, 0, 0)
        b.setSpacing(12)
        il.addStretch(0)
        il.addWidget(content, 1)
        il.addStretch(0)
        scroll.setWidget(inner)

        head = QHBoxLayout()
        col = QVBoxLayout()
        col.setSpacing(2)
        h = label("G1 Robot Control", "h1")
        f = h.font()
        f.setPointSizeF(22)
        h.setFont(f)
        col.addWidget(h)
        col.addWidget(label("Was soll gestartet werden?", "sub"))
        head.addLayout(col, 1)
        head.addWidget(button("Dokumentation", "ghost", lambda: app.open_doc(""),
                              icon="book-open"),
                       0, Qt.AlignmentFlag.AlignTop)
        b.addLayout(head)
        b.addSpacing(10)

        self.tiles = {}
        for key, title, text, color, sym, cb in (
                ("sim", "Simulation", "MuJoCo + Whole-Body-Policy — gefahrlos testen",
                 T.GREEN, "play", app.open_sim),
                ("real", "Echter Roboter", "G1 per LAN — Loco + Arme + Hände "
                 "(der Roboter bewegt sich!)", T.RED, "triangle-alert", app.open_real),
                ("scenes", "Umgebungen", "Szenen anlegen, bearbeiten und mit dem G1 ansehen",
                 T.ACCENT, "layout-grid", app.open_scenes)):
            t = Tile(title, text, color, sym)
            t.clicked.connect(lambda _=False, c=cb: c())
            b.addWidget(t)
            self.tiles[key] = t

        # Laufende / letzte Prozesse (damit nichts 'verloren' geht).
        if app.consoles:
            b.addWidget(section_title("Prozesse"))
            rows = RowList()
            for c in app.consoles:
                row = QWidget()
                rl = QHBoxLayout(row)
                rl.setContentsMargins(0, 8, 0, 8)
                rl.addWidget(dot(T.GREEN if c.is_running() else T.MUTED))
                rl.addWidget(label(f"{c.view_title}"), 1)
                rl.addWidget(label(c.state(), "muted"))
                rl.addWidget(button("Ansehen", "ghost", lambda x=c: app.show_console(x)))
                if c.is_running():
                    rl.addWidget(button("Stoppen", "small-danger", lambda x=c: self._stop(x)))
                else:
                    rl.addWidget(button("Entfernen", "ghost", lambda x=c: app.remove_console(x)))
                rows.add(row)
            b.addWidget(rows)

        b.addStretch(1)
        sec = QHBoxLayout()
        sec.addWidget(button("Alle Stacks stoppen", "ghost", app.stop_all))
        sec.addStretch(1)
        sec.addWidget(button("Beenden", "ghost", app.quit_app))
        b.addSpacing(8)
        b.addLayout(sec)

    def _stop(self, console) -> None:
        console.stop()
        self.app.show_menu()

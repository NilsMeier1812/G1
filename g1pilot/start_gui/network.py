"""Roboter-LAN im Startmenue: Live-Verbindungsanzeige und Einrichtungs-Assistent.

* ``LinkMonitor`` fragt ``netsetup.probe`` alle paar Sekunden in einem
  Hintergrund-Thread ab und meldet das Ergebnis per Signal in den GUI-Thread.
* ``LinkStatusCard`` zeigt das Ergebnis (gruen/orange) samt Haenden an.
* ``NetworkSetupDialog`` richtet das NM-Profil ``g1-robot`` ein. Er zeigt die
  Befehle vor dem Ausfuehren, fuehrt sie nacheinander per QProcess aus (GUI
  bleibt bedienbar) und protokolliert jede Ausgabe.
"""
from __future__ import annotations

import threading
import time

from PyQt6.QtCore import QObject, QProcess, QProcessEnvironment, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (QComboBox, QDialog, QHBoxLayout, QLineEdit,
                             QPlainTextEdit, QVBoxLayout, QWidget)

from g1pilot.utils import msgbox

from . import netsetup
from . import theme as T
from .widgets import (FieldRow, RowList, ToggleRow, button, card, dot, icon_label, label)

POLL_MS = 3000

LEVEL_COLOR = {"ok": T.GREEN, "warn": T.AMBER, "off": T.AMBER, "": T.MUTED}


# ════════════════════════════════════════════════════════════════════════
#  Periodische Pruefung im Hintergrund
# ════════════════════════════════════════════════════════════════════════
class LinkMonitor(QObject):
    """Ruft ``probe_fn(**params_fn())`` periodisch in einem Thread auf.

    Nie zwei Pruefungen gleichzeitig; laeuft eine noch (Ping-Timeout), wird
    der Takt einfach uebersprungen. ``updated`` kommt im GUI-Thread an.
    """
    updated = pyqtSignal(object)

    def __init__(self, probe_fn, params_fn, interval_ms: int = POLL_MS, parent=None):
        super().__init__(parent)
        self._probe = probe_fn
        self._params = params_fn
        self._busy = False
        self._again = False
        self._alive = True
        self.timer = QTimer(self)
        self.timer.setInterval(interval_ms)
        self.timer.timeout.connect(self.check_now)
        self.destroyed.connect(self._dead)

    def _dead(self, *_a) -> None:
        self._alive = False

    def start(self) -> None:
        self.check_now()
        self.timer.start()

    def stop(self) -> None:
        self.timer.stop()

    @property
    def running(self) -> bool:
        return self.timer.isActive()

    def check_now(self) -> None:
        if self._busy:
            self._again = True      # Parameter geaendert -> gleich danach nochmal
            return
        self._busy = True
        params = self._params()

        def worker():
            try:
                result = self._probe(**params)
            except Exception as e:                       # Anzeige statt Absturz
                result = netsetup.LinkStatus(iface=params.get("iface", ""))
                result.error = str(e)
            if self._alive:
                try:
                    self.updated.emit(result)
                except RuntimeError:                     # Fenster schon zu
                    pass

        threading.Thread(target=worker, daemon=True).start()

    def finish(self) -> None:
        """Vom Empfaenger nach jedem ``updated`` aufzurufen (GUI-Thread)."""
        self._busy = False
        if self._again:
            self._again = False
            self.check_now()


# ════════════════════════════════════════════════════════════════════════
#  Anzeige
# ════════════════════════════════════════════════════════════════════════
class LinkStatusCard(QWidget):
    """Status-Zeile mit Punkt, Text, Haenden und Zeitpunkt der Pruefung."""

    def __init__(self, on_setup, on_refresh):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 2, 0, 8)
        lay.setSpacing(6)
        top = QHBoxLayout()
        top.setSpacing(10)
        self.dot = dot(T.MUTED, 10)
        top.addWidget(self.dot, 0, Qt.AlignmentFlag.AlignTop)
        self.text = label("Prüfe Verbindung …", wrap=True)
        top.addWidget(self.text, 1)
        lay.addLayout(top)
        self.hands = label("", "hint", wrap=True, rich=True)
        self.hands.setContentsMargins(20, 0, 0, 0)
        self.hands.hide()
        lay.addWidget(self.hands)
        row = QHBoxLayout()
        row.setContentsMargins(20, 0, 0, 0)
        row.setSpacing(8)
        self.setup_btn = button("Netzwerk einrichten …", "ghost", on_setup, icon="network")
        self.setup_btn.setToolTip("Feste IP im Roboter-LAN per NetworkManager vergeben "
                                  "(zeigt die Befehle vorher an)")
        row.addWidget(self.setup_btn)
        self.refresh_btn = button("Jetzt prüfen", "ghost", on_refresh, icon="refresh-cw")
        row.addWidget(self.refresh_btn)
        row.addStretch(1)
        self.when = label("", "hint")
        row.addWidget(self.when)
        lay.addLayout(row)
        self.level = ""

    def set_plain(self, text: str, level: str) -> None:
        self.level = level
        self.text.setText(text)
        self.text.setStyleSheet(f"color:{LEVEL_COLOR.get(level, T.FG)};")
        self.dot.setStyleSheet(f"background:{LEVEL_COLOR.get(level, T.MUTED)};"
                               "border-radius:5px;")

    def show_status(self, st: netsetup.LinkStatus) -> None:
        err = getattr(st, "error", "")
        if err:
            self.set_plain(f"Prüfung fehlgeschlagen: {err}", "warn")
        else:
            self.set_plain(st.text(), st.level)
            if st.level == "warn" and st.g1 is True and not st.others:
                # nur eine Hand fehlt: Haupttext gruen, Punkt orange
                self.text.setStyleSheet(f"color:{T.GREEN};")
        if st.hands:
            parts = []
            for name, ip, ok in st.hands:
                col = T.GREEN if ok else T.AMBER
                state = "erreichbar" if ok else "nicht erreichbar"
                parts.append(f'<span style="color:{col}">{name} ({ip}) {state}</span>')
            self.hands.setText(" &nbsp;·&nbsp; ".join(parts))
            self.hands.show()
        else:
            self.hands.hide()
        self.when.setText("geprüft " + time.strftime("%H:%M:%S") +
                          f" · alle {POLL_MS // 1000} s")


# ════════════════════════════════════════════════════════════════════════
#  Einrichtungs-Assistent
# ════════════════════════════════════════════════════════════════════════
def inspect_host() -> dict:
    """Was am PC schon da ist (kurz blockierend, nmcli-Timeout 5 s)."""
    nm = netsetup.nm_running()
    return {
        "nm": nm,
        "profile": netsetup.nm_profile() if nm else None,
        "ufw": netsetup.ufw_enabled(),
    }


def _mono(widget) -> None:
    f = QFont()
    f.setFamilies(["JetBrains Mono", "Ubuntu Mono", "DejaVu Sans Mono", "monospace"])
    f.setStyleHint(QFont.StyleHint.Monospace)
    f.setPointSizeF(9.5)
    widget.setFont(f)


class NetworkSetupDialog(QDialog):
    """Feste IP fuer das Roboter-LAN einrichten. Ablauf: Auswahl -> Vorschau
    der Befehle -> »Jetzt einrichten« -> Protokoll. Nichts passiert ohne Klick."""
    applied = pyqtSignal()

    def __init__(self, parent, nics: list[tuple[str, str]], iface: str = "", *,
                 host: dict | None = None, carrier_fn=netsetup.carrier):
        super().__init__(parent)
        self.setWindowTitle("Netzwerk für den echten G1 einrichten")
        self.setModal(True)
        self.setMinimumWidth(640)
        self.setStyleSheet(T.stylesheet())
        self._nics = list(nics)
        self._carrier = carrier_fn
        # Mit festem `host` (Tests) wird nach dem Lauf nicht neu nachgesehen.
        self._inspect = inspect_host if host is None else (lambda: host)
        self.host = self._inspect()
        self.profile = self.host.get("profile")
        self._steps: list[netsetup.Step] = []
        self._queue: list[netsetup.Step] = []
        self._proc: QProcess | None = None
        self._step: netsetup.Step | None = None
        self._retried = False
        self._out = ""
        self._soft_note = ""
        self._done_text = ""
        self.ok = None          # None = nicht gelaufen, True/False = Ergebnis

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 18, 18, 16)
        root.setSpacing(12)

        # Kopf
        head, hl = card()
        hl.setContentsMargins(16, 14, 16, 14)
        top = QHBoxLayout()
        top.setSpacing(14)
        top.addWidget(icon_label("network", T.ACCENT, 22, box=40, bg=T.CARD_HI), 0,
                      Qt.AlignmentFlag.AlignTop)
        col = QVBoxLayout()
        col.setSpacing(4)
        t = label("Feste IP im Roboter-LAN")
        t.setStyleSheet("font-size:13pt; font-weight:700;")
        col.addWidget(t)
        col.addWidget(label(
            "Legt ein dauerhaftes NetworkManager-Profil »g1-robot« an: feste Adresse "
            "im Netz 192.168.123.x, ohne Gateway und ohne DNS. Das WLAN bleibt fürs "
            "Internet zuständig. Andere Verbindungen am PC werden nicht verändert. "
            "Für die Änderung fragt das System nach deinem Passwort.", "muted", wrap=True))
        top.addLayout(col, 1)
        hl.addLayout(top)
        root.addWidget(head)

        # Auswahl
        rows = RowList()
        self.iface = QComboBox()
        self.iface.setMinimumWidth(300)
        self._iface_names: list[str] = []
        for name, ip in self._nics:
            self.iface.addItem(f"{name}  ({ip or 'keine IPv4'})")
            self._iface_names.append(name)
        if iface in self._iface_names:
            self.iface.setCurrentIndex(self._iface_names.index(iface))
        elif self.profile and self.profile.get("iface") in self._iface_names:
            self.iface.setCurrentIndex(self._iface_names.index(self.profile["iface"]))
        rows.add(FieldRow("Interface zum G1", self.iface,
                          "kabelgebunden: enp…/eno…/eth…/enx… (nicht WLAN wl…)"))
        start_ip = netsetup.DEFAULT_PC_IP
        if self.profile and netsetup.in_robot_lan(self.profile.get("ip", "")):
            start_ip = self.profile["ip"].split("/")[0]
        self.ip = QLineEdit(start_ip)
        self.ip.setFixedWidth(180)
        self.ip.setPlaceholderText(netsetup.DEFAULT_PC_IP)
        rows.add(FieldRow("Adresse dieses PCs", self.ip, "Konvention .222, jede freie geht",
                          unit="/24"))
        self.fw = ToggleRow("Firewall (ufw) für dieses Interface öffnen",
                            "ufw ist aktiv; ohne Regel kommen die DDS-Daten vom G1 nicht an",
                            True)
        rows.add(self.fw)
        self.fw.setVisible(bool(self.host.get("ufw")))
        root.addWidget(rows)

        # Befund + Vorschau
        self.findings = label("", "hint", wrap=True, rich=True)
        root.addWidget(self.findings)
        root.addWidget(label("Diese Befehle werden ausgeführt:", "muted"))
        self.preview = QPlainTextEdit()
        self.preview.setReadOnly(True)
        self.preview.setMaximumHeight(130)
        self.preview.setLineWrapMode(QPlainTextEdit.LineWrapMode.WidgetWidth)
        _mono(self.preview)
        self.preview.setStyleSheet(f"background:{T.CONSOLE_BG}; border:1px solid {T.LINE};"
                                   "border-radius:8px; padding:6px;")
        root.addWidget(self.preview)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(110)
        _mono(self.log)
        self.log.setStyleSheet(self.preview.styleSheet())
        self.log.hide()
        root.addWidget(self.log)
        self.result = label("", wrap=True)
        self.result.hide()
        root.addWidget(self.result)

        root.addStretch(1)

        # Knoepfe
        btns = QHBoxLayout()
        self.remove_btn = button("Profil entfernen", "small-danger", self._remove,
                                 icon="trash-2", icon_color=T.RED)
        self.remove_btn.setVisible(self.profile is not None and bool(self.host.get("nm")))
        btns.addWidget(self.remove_btn)
        btns.addStretch(1)
        self.close_btn = button("Schließen", "ghost", self.reject)
        btns.addWidget(self.close_btn)
        self.apply_btn = button("Jetzt einrichten", "primary", self._apply, icon="check")
        self.apply_btn.setDefault(True)
        btns.addWidget(self.apply_btn)
        root.addLayout(btns)

        self.iface.currentIndexChanged.connect(lambda _i: self._update())
        self.ip.textChanged.connect(lambda _t: self._update())
        self.fw.toggled.connect(lambda _v: self._update())
        self._update()

    # ── Auswahl -> Plan ────────────────────────────────────────────────────
    def selected_iface(self) -> str:
        i = self.iface.currentIndex()
        return self._iface_names[i] if 0 <= i < len(self._iface_names) else ""

    def _update(self) -> None:
        iface = self.selected_iface()
        ip, err = netsetup.validate_pc_ip(self.ip.text())
        notes: list[str] = []
        warn = f'<span style="color:{T.AMBER}">'
        if not self.host.get("nm"):
            notes.append(warn + "NetworkManager (nmcli) nicht gefunden. Automatisch "
                         "geht es nur damit. Von Hand (gilt nur bis zum Neustart):</span>")
        if self.profile:
            p = self.profile
            notes.append(f"Profil »{netsetup.PROFILE}« gibt es schon "
                         f"({p.get('iface') or 'ohne Interface'}, {p.get('ip') or 'ohne IP'}"
                         f"{', aktiv' if p.get('active') else ''}). Es wird angepasst.")
        link = self._carrier(iface) if iface else None
        if iface and link is False:
            notes.append(warn + f"An {iface} steckt gerade kein Kabel. Das Profil wird "
                         "trotzdem angelegt und aktiviert sich beim Einstecken.</span>")
        if iface.startswith(("wl", "ww")):
            notes.append(warn + f"{iface} sieht nach WLAN/Mobilfunk aus. Der G1 hängt am "
                         "LAN-Kabel.</span>")
        others = [n for n in netsetup.robot_ifaces(self._nics)
                  if n != iface and not (self.profile and n == self.profile.get("iface"))]
        if others:
            notes.append(warn + "Auch " + ", ".join(others) + " hat eine 192.168.123.x-"
                         "Adresse. Zwei Interfaces im Roboter-LAN stören DDS.</span>")
        if err:
            notes.append(f'<span style="color:{T.RED}">{err}</span>')
        if not iface:
            notes.append(f'<span style="color:{T.RED}">Kein Netzwerk-Interface gefunden.</span>')
        self.findings.setText("<br>".join(notes))
        self.findings.setVisible(bool(notes))

        ok = bool(iface) and not err
        if not self.host.get("nm"):
            self._steps = []
            self.preview.setPlainText(netsetup.manual_commands(iface or "<interface>",
                                                               ip or netsetup.DEFAULT_PC_IP))
            ok = False
        elif ok:
            self._steps = netsetup.plan(iface, ip, self.profile, link_up=link,
                                        open_firewall=bool(self.host.get("ufw")) and
                                        self.fw.isChecked())
            self.preview.setPlainText("\n".join(s.shell() for s in self._steps))
        else:
            self._steps = []
            self.preview.setPlainText("")
        running = self._proc is not None
        self.apply_btn.setEnabled(ok and not running)

    def plan(self) -> list[netsetup.Step]:
        return list(self._steps)

    # ── Ausfuehren ────────────────────────────────────────────────────────
    def _apply(self) -> None:
        if not self._steps or self._proc is not None:
            return
        self._run(list(self._steps), done_text=(
            f"Fertig. {self.selected_iface()} hat jetzt das Profil »{netsetup.PROFILE}«. "
            "Die Verbindungsanzeige im Startmenü prüft gleich neu."))

    def _remove(self) -> None:
        if not msgbox.confirm(self, "Profil entfernen",
                              f"NetworkManager-Profil »{netsetup.PROFILE}« löschen? Das "
                              "Interface verliert dann seine 192.168.123.x-Adresse.",
                              yes="Entfernen", danger=True):
            return
        self._run(netsetup.remove_plan(), done_text=f"Profil »{netsetup.PROFILE}« entfernt.")

    def _run(self, steps: list[netsetup.Step], done_text: str) -> None:
        self._queue = steps
        self._done_text = done_text
        self.ok = None
        self.log.clear()
        self.log.show()
        self.result.hide()
        for w in (self.apply_btn, self.remove_btn, self.iface, self.ip, self.fw):
            w.setEnabled(False)
        self._next()

    def _append(self, text: str) -> None:
        self.log.appendPlainText(text.rstrip("\n"))

    def _next(self) -> None:
        if not self._queue:
            self._finish(True, self._done_text)
            return
        self._step = self._queue.pop(0)
        self._retried = False
        argv = netsetup.with_root(self._step) if self._step.root else list(self._step.argv)
        self._start(argv)

    def _start(self, argv: list[str]) -> None:
        self._append(f"$ {' '.join(argv)}")
        self._out = ""
        p = QProcess(self)
        env = QProcessEnvironment.systemEnvironment()
        env.insert("LC_ALL", "C")
        p.setProcessEnvironment(env)
        p.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        p.readyReadStandardOutput.connect(self._read)
        p.finished.connect(self._step_done)
        p.errorOccurred.connect(self._proc_error)
        self._proc = p
        p.start(argv[0], argv[1:])

    def _read(self) -> None:
        if self._proc is None:
            return
        chunk = bytes(self._proc.readAllStandardOutput()).decode(errors="replace")
        self._out += chunk
        if chunk.strip():
            self._append(chunk)

    def _proc_error(self, err) -> None:
        if err == QProcess.ProcessError.FailedToStart and self._proc is not None:
            self._append("Programm nicht gefunden.")
            self._proc = None
            self._failed()

    def _step_done(self, code: int, _status) -> None:
        if self._proc is None:
            return
        self._read()
        self._proc = None
        step = self._step
        if code == 0:
            self._append("ok")
            self._next()
            return
        if (not step.root and not self._retried and netsetup.needs_root_retry(self._out)
                and netsetup.with_root(step) != list(step.argv)):
            self._retried = True
            self._append("Keine Berechtigung, nochmal mit Passwort-Abfrage (pkexec) …")
            self._start(netsetup.with_root(step))
            return
        self._failed()

    def _failed(self) -> None:
        step = self._step
        if step is not None and step.optional:
            self._append("Hinweis: " + step.note)
            self._soft_note = step.note
            self._next()
            return
        self._queue = []
        self._finish(False, f"Abgebrochen bei »{step.title if step else '?'}«. Nichts weiter "
                            "geändert. Details stehen im Protokoll oben.")

    def _finish(self, ok: bool, text: str) -> None:
        self.ok = ok
        if ok and self._soft_note:
            text += " " + self._soft_note
        self._soft_note = ""
        self.result.setText(text)
        self.result.setStyleSheet(f"color:{T.GREEN if ok else T.RED};")
        self.result.show()
        for w in (self.iface, self.ip, self.fw):
            w.setEnabled(True)
        if ok:
            self.host = self._inspect()
            self.profile = self.host.get("profile")
            self.remove_btn.setVisible(self.profile is not None)
            self.applied.emit()
        self.remove_btn.setEnabled(True)
        self.close_btn.setText("Fertig" if ok else "Schließen")
        self._update()

    def reject(self) -> None:  # noqa: D401 - Qt-Name
        if self._proc is not None:      # laufenden Schritt nicht verwaisen lassen
            return
        super().reject()

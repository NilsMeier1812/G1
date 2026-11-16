"""Prozess-Ansicht: startet einen Subprozess und zeigt dessen Ausgabe live.

Gleiches Verhalten wie die fruehere Tk-Konsole:
- bleibt in der App-Registry, auch wenn man ins Menue zurueckgeht (Prozess
  laeuft weiter, Log wird weiter geschrieben, 'Prozesse' im Menue fuehrt zurueck)
- 'Stoppen' = SIGTERM + optionales Aufraeum-Kommando (docker compose down);
  "beendet" gilt erst, wenn auch das fertig ist
- Editor-Prozesse: Browser oeffnen, sobald der Webserver antwortet (echte URL
  aus der Ausgabe, nicht fest 8080)
Thread-sicher: Reader-Threads schreiben in eine Queue, ein QTimer leert sie.
"""
from __future__ import annotations

import queue
import re
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QFont, QTextCursor
from PyQt6.QtWidgets import QHBoxLayout, QPlainTextEdit, QVBoxLayout, QWidget

from . import core
from . import theme as T
from .widgets import button, label

_EOF = object()   # Reader: Prozess-Ausgabe zu Ende (Prozess beendet)
_DONE = object()  # Stop-Worker: Aufraeumen (docker down) abgeschlossen -> finalisieren
# viser-Banner des Editors ("HTTP │ http://127.0.0.1:8081"). Ist 8080 belegt,
# weicht viser auf 8081, 8082 ... aus -- die echte URL steht nur in der Ausgabe.
_EDITOR_URL_RE = re.compile(r"http://(?:127\.0\.0\.1|localhost):\d+")
MAX_LINES = 4000


class ConsoleView(QWidget):
    ephemeral = False  # nicht zerstoeren, wenn eine andere Ansicht gezeigt wird

    def __init__(self, app, title: str, argv: list, *, cwd: Path, env: dict | None = None,
                 stop_cmd: list | None = None, stop_note: str = "",
                 browser_url: str | None = None, autostart: bool = True):
        super().__init__()
        self.app = app
        self.view_title = title
        self._argv = argv
        self._cwd = cwd
        self._env = env
        self._stop_cmd = stop_cmd
        self._stop_note = stop_note
        self._browser_url = browser_url
        self._server_url: str | None = None
        self._proc: subprocess.Popen | None = None
        self._queue: queue.Queue = queue.Queue()
        self._stopping = False
        self._finished = False
        self._alive = True

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        head = QWidget()
        head.setObjectName("bar")
        head.setStyleSheet(f"QWidget#bar{{background:{T.SURFACE};border-bottom:1px solid {T.LINE};}}")
        hl = QHBoxLayout(head)
        hl.setContentsMargins(16, 10, 16, 10)
        hl.addWidget(button("‹  Menü", "ghost", app.show_menu))
        t = label(title)
        f = t.font()
        f.setPointSizeF(13)
        f.setWeight(QFont.Weight.DemiBold)
        t.setFont(f)
        hl.addWidget(t)
        hl.addStretch(1)
        self._status = label("läuft …")
        hl.addWidget(self._status)
        lay.addWidget(head)

        body = QVBoxLayout()
        body.setContentsMargins(16, 12, 16, 12)
        body.setSpacing(10)
        self._text = QPlainTextEdit()
        self._text.setObjectName("console")
        self._text.setReadOnly(True)
        self._text.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self._text.setMaximumBlockCount(MAX_LINES)
        body.addWidget(self._text, 1)
        foot = QHBoxLayout()
        self._stop_btn = button("Stoppen", "danger", self.stop)
        foot.addWidget(self._stop_btn)
        note = "»Menü« lässt den Prozess im Hintergrund weiterlaufen."
        if stop_note:
            note = f"{stop_note}  {note}"
        foot.addWidget(label(note, "hint"), 1)
        body.addLayout(foot)
        lay.addLayout(body, 1)
        self._set_status("läuft …", T.AMBER)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._drain)
        self._append(f"$ {' '.join(str(a) for a in argv)}\n\n")
        if autostart:
            self.start()

    # ── Status fuer die Menue-Liste ───────────────────────────────────────
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def state(self) -> str:
        if self.is_running():
            return "stoppe …" if self._stopping else "läuft"
        return "gestoppt" if self._stopping else "beendet"

    def log_text(self) -> str:
        return self._text.toPlainText()

    # ── intern ────────────────────────────────────────────────────────────
    def start(self) -> None:
        try:
            self._proc = subprocess.Popen(
                self._argv, cwd=str(self._cwd), env=self._env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1, text=True)
        except Exception as exc:
            self._append(f"[Fehler beim Start] {exc}\n")
            self._finalize(-1)
            return
        threading.Thread(target=self._reader, daemon=True).start()
        self._timer.start(80)
        if self._browser_url:
            threading.Thread(target=self._browser_waiter, daemon=True).start()

    def _reader(self) -> None:
        assert self._proc and self._proc.stdout
        for line in self._proc.stdout:
            if self._browser_url and self._server_url is None:
                m = _EDITOR_URL_RE.search(line)
                if m:
                    self._server_url = m.group(0)
            self._queue.put(line)
        self._proc.wait()
        self._queue.put(_EOF)

    def _browser_waiter(self) -> None:
        """Wartet, bis der Editor-Webserver antwortet, und oeffnet den Browser.

        Bewusst eine echte HTTP-Anfrage statt rohem TCP-Connect: viser bedient
        HTTP und WebSocket auf demselben Port, ein abgebrochener Connect saehe
        wie ein kaputter WebSocket-Handshake aus (harmloser Traceback im Log).
        """
        deadline = time.monotonic() + 180
        while self._alive and not self._stopping and time.monotonic() < deadline:
            url = self._server_url
            if url and self._server_responds(url):
                if self._alive:
                    self._queue.put(f"\n[GUI] Öffne Editor im Browser: {url}\n")
                    core.open_url(url)
                return
            if self._proc is not None and self._proc.poll() is not None:
                return
            time.sleep(0.5)

    @staticmethod
    def _server_responds(url: str) -> bool:
        try:
            urllib.request.urlopen(url, timeout=1).close()
            return True
        except urllib.error.HTTPError:
            return True
        except (urllib.error.URLError, OSError):
            return False

    def _drain(self) -> None:
        if not self._alive:
            return
        chunks = []
        try:
            while True:
                item = self._queue.get_nowait()
                if item is _EOF:
                    # Bei Nutzer-Stopp erst nach dem Aufraeumen (_DONE) fertig melden.
                    if not self._stopping:
                        self._flush(chunks)
                        self._finalize()
                        return
                elif item is _DONE:
                    self._flush(chunks)
                    self._finalize()
                    return
                else:
                    chunks.append(item)
        except queue.Empty:
            pass
        self._flush(chunks)

    def _flush(self, chunks) -> None:
        if chunks:
            self._append("".join(chunks))

    def _append(self, text: str) -> None:
        sb = self._text.verticalScrollBar()
        at_end = sb.value() >= sb.maximum() - 4
        c = self._text.textCursor()
        c.movePosition(QTextCursor.MoveOperation.End)
        c.insertText(text)
        if at_end:
            sb.setValue(sb.maximum())

    def _set_status(self, text: str, color: str) -> None:
        self._status.setText(f"● {text}")
        self._status.setStyleSheet(f"color:{color}; font-weight:600;")

    def _finalize(self, rc: int | None = None) -> None:
        self._timer.stop()
        self._finished = True
        if rc is None:
            rc = self._proc.returncode if self._proc else -1
        user_stopped = self._stopping
        if user_stopped:
            self._set_status("gestoppt", T.MUTED)
        elif rc == 0:
            self._set_status("beendet (ok)", T.GREEN)
        else:
            self._set_status(f"beendet (Code {rc})", T.RED)
        self._stop_btn.setEnabled(False)
        self.app.notify_finished(self, user_stopped=user_stopped)

    def terminate_editor(self) -> None:
        """Beim Schliessen des Startmenues: Web-Editor mitbeenden (sonst haengt
        er verwaist auf Port 8080). Docker-Stacks laufen bewusst weiter."""
        if self._browser_url and self.is_running():
            try:
                self._proc.terminate()
                self._proc.wait(timeout=3)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass

    def stop(self) -> None:
        if self._stopping or not self.is_running():
            return
        self._stopping = True
        self._set_status("stoppe …", T.AMBER)
        self._stop_btn.setEnabled(False)
        self._append("\n[Stoppen angefordert]\n")

        def worker():
            if self._proc and self._proc.poll() is None:
                try:
                    self._proc.terminate()
                    self._proc.wait(timeout=12)
                except Exception:
                    try:
                        self._proc.kill()
                    except Exception:
                        pass
            if self._stop_cmd:
                self._queue.put(f"\n$ {' '.join(self._stop_cmd)}\n")
                try:
                    r = subprocess.run(self._stop_cmd, cwd=str(self._cwd),
                                       capture_output=True, text=True, timeout=90)
                    self._queue.put(r.stdout + r.stderr)
                except Exception as exc:
                    self._queue.put(f"[down-Fehler] {exc}\n")
            self._queue.put(_DONE)

        threading.Thread(target=worker, daemon=True).start()

    def dispose(self) -> None:
        self._alive = False
        self._timer.stop()
        self.deleteLater()

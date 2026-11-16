"""Hauptfenster des Startmenues: ein Fenster, die Ansichten werden getauscht
(Router). Laufende Prozesse bleiben in der Registry, Doku in eigenem Fenster."""
from __future__ import annotations

import os
import sys
import threading
from pathlib import Path

from PyQt6.QtCore import QObject, QProcess, QTimer, pyqtSignal
from PyQt6.QtGui import QIcon, QKeySequence, QShortcut
from PyQt6.QtWidgets import (QApplication, QLabel, QMainWindow, QMessageBox,
                             QStackedWidget)

from . import core
from . import theme as T
from .console import ConsoleView
from .views import MenuView, RealView, ScenesView, SimView

APP_ID = "g1-start"     # WM_CLASS -> eigener Dock-Eintrag (resource/icons/README.md)


class _Bridge(QObject):
    """Signale aus Hintergrund-Threads sicher in den GUI-Thread bringen."""
    status = pyqtSignal(str)


class MainWindow(QMainWindow):
    thumbs_changed = pyqtSignal()

    def __init__(self, *, check_docker: bool = True, auto_thumbs: bool = True):
        super().__init__()
        self.setWindowTitle("G1 — Startmenü")
        self.resize(1040, 860)
        self.setMinimumSize(720, 600)
        self._check_docker = check_docker
        self._current = None
        self.consoles: list[ConsoleView] = []
        self._docs = None
        self._thumb_proc: QProcess | None = None
        self._thumb_again = False

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)
        sb = self.statusBar()
        sb.setSizeGripEnabled(False)
        self._status_lbl = QLabel("")
        sb.addWidget(self._status_lbl, 1)
        self._thumb_lbl = QLabel("")
        sb.addPermanentWidget(self._thumb_lbl)
        self._bridge = _Bridge()
        self._bridge.status.connect(self._status_lbl.setText)

        QShortcut(QKeySequence("F1"), self, activated=lambda: self.open_doc(""))
        QShortcut(QKeySequence("Ctrl+Q"), self, activated=self.quit_app)

        self.show_menu()
        self.refresh_status()
        if auto_thumbs:
            QTimer.singleShot(400, self.refresh_thumbs)

    # ── Router ───────────────────────────────────────────────────────────
    def show_view(self, view) -> None:
        if self._current is view:
            return
        old = self._current
        if self.stack.indexOf(view) < 0:
            self.stack.addWidget(view)
        self.stack.setCurrentWidget(view)
        self._current = view
        self.setWindowTitle(f"G1 — {getattr(view, 'view_title', 'Startmenü')}")
        if old is not None and getattr(old, "ephemeral", False):
            self.stack.removeWidget(old)
            old.deleteLater()

    @property
    def current(self):
        return self._current

    def show_menu(self) -> None:
        self.show_view(MenuView(self))

    def open_sim(self) -> None:
        if self._docker_guard():
            self.show_view(SimView(self))

    def open_real(self) -> None:
        if self._docker_guard():
            self.show_view(RealView(self))

    def open_scenes(self) -> None:
        self.show_view(ScenesView(self))

    def show_console(self, console: ConsoleView) -> None:
        if console not in self.consoles:
            self.consoles.append(console)
        self.show_view(console)

    def notify_finished(self, console: ConsoleView, *, user_stopped: bool) -> None:
        """Ein Prozess ist fertig (sauber gestoppt oder von selbst beendet).

        - Wird er gerade angesehen UND vom Nutzer gestoppt: zurueck ins Menue.
        - Ist das Menue offen: Prozessliste auffrischen.
        - Sonst nichts wegreissen; abgestuerzte Prozesse bleiben mit Log sichtbar.
        """
        if getattr(console, "_scene_editor", False):
            QTimer.singleShot(0, self.refresh_thumbs)   # evtl. neue/geaenderte Szene
        if self._current is console:
            if user_stopped:
                self.show_menu()
        elif isinstance(self._current, MenuView):
            self.show_menu()

    def start_process(self, title, argv, *, cwd, env=None, stop_cmd=None,
                      stop_note="", browser_url=None, scene_editor=False) -> ConsoleView:
        console = ConsoleView(self, title, argv, cwd=cwd, env=env, stop_cmd=stop_cmd,
                              stop_note=stop_note, browser_url=browser_url, autostart=False)
        console._scene_editor = scene_editor
        self.consoles.append(console)
        self.show_view(console)
        console.start()
        return console

    def remove_console(self, console: ConsoleView) -> None:
        if console in self.consoles:
            self.consoles.remove(console)
        if self._current is console:
            self._current = None
        self.stack.removeWidget(console)
        console.dispose()
        self.show_menu()

    # ── Doku ─────────────────────────────────────────────────────────────
    def open_doc(self, ref: str = "") -> None:
        """ref: 'datei.md#anker' relativ zu docs/, '' = Uebersicht."""
        from .docs_viewer import DocsWindow
        if self._docs is None:
            self._docs = DocsWindow(open_external=self._open_external)
            self._docs.setWindowIcon(self.windowIcon())
        name, _, anchor = (ref or "README.md").partition("#")
        self._docs.open_doc(core.DOCS_DIR / name, anchor)
        self._docs.show()
        self._docs.raise_()
        self._docs.activateWindow()

    def handle_link(self, href: str) -> None:
        if href.startswith("doc:"):
            self.open_doc(href[4:])
        else:
            core.open_url(href)

    def _open_external(self, path: Path) -> None:
        err = core.open_path(path)
        if err:
            QMessageBox.critical(self, "Öffnen fehlgeschlagen", f"{path}\n\n{err}")

    # ── Vorschaubilder ───────────────────────────────────────────────────
    def refresh_thumbs(self, force: bool = False) -> None:
        """Fehlende/veraltete Vorschaubilder im Hintergrund erzeugen (scene_editor-venv)."""
        if not core.scene_editor_ready() or not core.THUMB_SCRIPT.is_file():
            return
        if self._thumb_proc is not None:
            self._thumb_again = True
            return
        stale = core.stale_thumbs()
        if not stale and not force:
            return
        proc = QProcess(self)
        proc.setWorkingDirectory(str(core.SCENE_DIR))
        proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        args = [str(core.THUMB_SCRIPT)] + (["--all"] if force else [])
        proc.finished.connect(self._thumbs_done)
        proc.readyReadStandardOutput.connect(
            lambda: sys.stdout.write(bytes(proc.readAllStandardOutput()).decode(errors="replace")))
        self._thumb_proc = proc
        self._thumb_lbl.setText("Vorschaubilder werden erzeugt …")
        proc.start(str(core.SCENE_VENV), args)

    def _thumbs_done(self, *_a) -> None:
        self._thumb_proc = None
        self._thumb_lbl.setText("")
        self.thumbs_changed.emit()
        if self._thumb_again:
            self._thumb_again = False
            QTimer.singleShot(0, self.refresh_thumbs)

    # ── Aktionen ─────────────────────────────────────────────────────────
    def _docker_guard(self) -> bool:
        if not self._check_docker or core.docker_ready():
            return True
        r = QMessageBox.question(self, "Docker nicht erreichbar",
                                 "Docker scheint nicht zu laufen oder ist nicht installiert.\n"
                                 "Trotzdem fortfahren?")
        return r == QMessageBox.StandardButton.Yes

    def stop_all(self) -> None:
        r = QMessageBox.question(self, "Stoppen",
                                 "Sim- UND Real-Stack stoppen (docker compose down)?")
        if r != QMessageBox.StandardButton.Yes:
            return
        cmd = ["bash", "-c",
               "docker compose --profile sim down --remove-orphans; "
               "docker compose --profile real down --remove-orphans"]
        self.start_process("Stacks stoppen", cmd, cwd=core.HERE, env=os.environ.copy())

    def closeEvent(self, e) -> None:  # noqa: N802
        self.quit_app()

    def quit_app(self) -> None:
        # Ein Fenster, ein Schliessen: beendet das Startmenue. Gestartete Stacks
        # laufen im Docker weiter (bewusst); nur der Web-Editor wird mitbeendet.
        for c in list(self.consoles):
            c.terminate_editor()
        if self._thumb_proc is not None:
            self._thumb_proc.kill()
        if os.environ.get("G1_START_GUI_NO_EXIT") == "1":   # Tests
            return
        # Garantiert beenden, auch wenn Log-Reader/Subprozesse noch haengen.
        os._exit(0)

    def refresh_status(self) -> None:
        editor = "Scene-Editor: bereit" if core.scene_editor_ready() else \
            "Scene-Editor: Setup nötig"
        scenes = f"Umgebungen: {len(core.list_scenes())}"
        self._status_lbl.setText(f"Docker: prüfe …   |   {editor}   |   {scenes}")
        if not self._check_docker:
            return

        def worker():
            docker = "Docker: ok" if core.docker_ready() else "Docker: nicht erreichbar"
            self._bridge.status.emit(f"{docker}   |   {editor}   |   {scenes}")

        threading.Thread(target=worker, daemon=True).start()


def make_app(argv=None) -> QApplication:
    # Instanzname fuer WM_CLASS muss vor dem ersten Fenster feststehen.
    os.environ.setdefault("RESOURCE_NAME", APP_ID)
    app = QApplication.instance() or QApplication(argv if argv is not None else sys.argv)
    app.setApplicationName(APP_ID)
    app.setDesktopFileName(APP_ID)
    if core.START_ICON.is_file():
        app.setWindowIcon(QIcon(str(core.START_ICON)))
    T.apply(app)
    return app


def main() -> int:
    app = make_app()
    w = MainWindow()
    w.show()
    return app.exec()

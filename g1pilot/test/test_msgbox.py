"""Karten-Dialoge (g1pilot/utils/msgbox.py) und gemeinsames Theme (ui_theme.py)."""
import os

import pytest

pytest.importorskip("PyQt6.QtWidgets")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication, QLabel, QPushButton  # noqa: E402

from g1pilot.utils import msgbox, ui_theme  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def _texts(dlg):
    return [b.text() for b in dlg.findChildren(QPushButton)]


def test_confirm_buttons_and_result(app, monkeypatch):
    seen = {}

    def fake(dlg):
        seen["buttons"] = _texts(dlg)
        dlg.yes_button.click()
        return dlg.result() == dlg.DialogCode.Accepted

    monkeypatch.setattr(msgbox, "run_dialog", fake)
    assert msgbox.confirm(None, "Stoppen", "Wirklich?", yes="Stoppen", danger=True) is True
    assert seen["buttons"] == ["Abbrechen", "Stoppen"]


def test_confirm_cancel_is_false(app, monkeypatch):
    monkeypatch.setattr(msgbox, "run_dialog", lambda dlg: False)
    assert msgbox.confirm(None, "t", "x") is False


def test_info_has_single_button_and_no_emoji(app):
    dlg = msgbox.build(None, "Hinweis", "Text", "info")
    assert _texts(dlg) == ["OK"]
    for lab in dlg.findChildren(QLabel):
        assert all(ord(c) < 0x2190 or c in "–·€" or ord(c) < 0x2000 for c in lab.text())


def test_ask_text_returns_line(app, monkeypatch):
    def fake(dlg):
        dlg.line.setText("Küche")
        return True

    monkeypatch.setattr(msgbox, "run_dialog", fake)
    assert msgbox.ask_text(None, "Name", "Name:", text="alt") == ("Küche", True)


def test_ask_text_cancel(app, monkeypatch):
    monkeypatch.setattr(msgbox, "run_dialog", lambda dlg: False)
    assert msgbox.ask_text(None, "Name", "Name:")[1] is False


def test_theme_shared_with_start_gui():
    from start_gui import theme
    assert theme.ACCENT == ui_theme.ACCENT and theme.stylesheet() == ui_theme.stylesheet()


def test_no_standard_qt_dialogs_in_gui_sources():
    """Keine grauen QMessageBox/QInputDialog mehr (Karten-Dialoge aus msgbox).
    Posen-/Ablauf-Dialoge (gui_base.py, sequence_editor.py) folgen nach PR #54."""
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    files = list((root / "start_gui").glob("*.py")) + [
        root / "g1pilot/teleoperation/demo_gui.py", root / "g1pilot/teleoperation/hand_panel.py",
        root / "g1pilot/teleoperation/quickslot_panel.py",
        root.parent / "unitree_mujoco/unitree_robots/g1/tools/hand_new.py",
        root.parent / "unitree_mujoco/unitree_robots/g1/tools/hand_adjust.py"]
    for f in files:
        txt = f.read_text(encoding="utf8")
        # theme.py nennt die Klassen nur im Stylesheet-Selektor
        txt = txt.replace("QMessageBox, QInputDialog", "")
        assert "QMessageBox." not in txt and "QInputDialog." not in txt, f.name


def test_phone_web_has_no_browser_dialogs():
    from pathlib import Path
    js = (Path(__file__).resolve().parents[1] / "g1pilot/teleoperation/phone/web/app.js").read_text()
    import re
    assert not re.search(r"(?<![\w.])(confirm|alert|prompt)\(", js)

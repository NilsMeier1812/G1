#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Handy-Seite offline und eingefroren (web/sw.js): einmal geladen laeuft sie ohne
Netz, eine neue Version auf dem Server wird NICHT automatisch aktiv, sondern erst nach
»Aktualisieren«. Braucht playwright + Chromium, sonst uebersprungen."""
import functools
import http.server
import os
import re
import shutil
import threading

import pytest

WEB = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "g1pilot",
                                   "teleoperation", "phone", "web"))


def _read(name):
    with open(os.path.join(WEB, name), encoding="utf-8") as f:
        return f.read()


def test_version_matches_cache_name():
    cache = re.search(r'const CACHE = "g1phone-(v\d+)"', _read("sw.js")).group(1)
    version = re.search(r'const VERSION = "(v\d+)"', _read("app.js")).group(1)
    assert cache == version


def test_sw_has_no_auto_update():
    sw = _read("sw.js")
    install = sw.split('addEventListener("install"')[1].split("});")[0]
    assert "skipWaiting" not in install          # neue Version wartet
    assert "c.put" not in sw                     # Netz-Antworten ueberschreiben den Cache nicht


@pytest.fixture
def site(tmp_path):
    root = tmp_path / "web"
    shutil.copytree(WEB, root)
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(root))
    handler.log_message = lambda *a: None
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield root, f"http://127.0.0.1:{httpd.server_address[1]}/index.html"
    httpd.shutdown()


@pytest.fixture
def browser():
    sync_api = pytest.importorskip("playwright.sync_api")
    pw = sync_api.sync_playwright().start()
    b = err = None
    for exe in (None, os.environ.get("G1_CHROMIUM", "/opt/pw-browsers/chromium")):
        try:
            b = pw.chromium.launch(executable_path=exe) if exe else pw.chromium.launch()
            break
        except Exception as e:  # noqa: BLE001
            err = e
    if b is None:
        pw.stop()
        pytest.skip(f"Chromium nicht startbar: {err}")
    yield b
    b.close()
    pw.stop()


def _bump(root, new):
    for name, pat, rep in (("sw.js", r'g1phone-v\d+', f"g1phone-{new}"),
                           ("app.js", r'const VERSION = "v\d+"', f'const VERSION = "{new}"')):
        p = root / name
        p.write_text(re.sub(pat, rep, p.read_text(encoding="utf-8")), encoding="utf-8")


def test_offline_and_frozen_until_update(site, browser):
    root, url = site
    old = re.search(r'const VERSION = "(v\d+)"', _read("app.js")).group(1)
    ctx = browser.new_context()
    page = ctx.new_page()
    page.goto(url)
    page.wait_for_function("navigator.serviceWorker.ready.then(() => true)")
    page.reload()
    page.wait_for_function("!!navigator.serviceWorker.controller")

    # Ohne Netz: Seite kommt aus dem Cache.
    ctx.set_offline(True)
    page.reload()
    assert page.locator("#version").text_content() == f"Handy-Seite {old}"
    ctx.set_offline(False)

    # Neue Version auf dem Server: wird geladen, bleibt aber wartend.
    _bump(root, "v99")
    page.reload()
    page.evaluate("navigator.serviceWorker.getRegistration().then(r => r.update())")
    page.wait_for_function("navigator.serviceWorker.getRegistration().then(r => !!r.waiting)")
    page.reload()
    assert page.locator("#version").text_content() == f"Handy-Seite {old}"
    page.wait_for_selector("#updateBar:not(.hidden)")

    # Erst »Aktualisieren« macht sie aktiv.
    with page.expect_navigation():
        page.click("#btnUpdate")
    page.wait_for_function("document.getElementById('version').textContent.includes('v99')")
    assert page.locator("#updateBar").is_hidden()
    ctx.close()

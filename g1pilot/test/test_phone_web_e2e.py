#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Handy-Seite (web/app.js) im echten Chromium gegen das echte Gateway:
Web Bluetooth ist durch eine Attrappe ersetzt, die Writes/Reads an Python
weiterreicht (Zeilenpuffer + TX-Seiten wie im BLE-Server).

Prueft: Koppeln mit Code, Anmelden, Bedienung uebernehmen, Quittieren nur
per vollem Schieber, NOT-HALT, Sperre bei alten Daten, Seite verlassen ->
Bedienung abgegeben. Braucht playwright + Chromium, sonst uebersprungen."""
import functools
import http.server
import os
import threading

import pytest

sync_api = pytest.importorskip("playwright.sync_api")

from g1pilot.teleoperation.phone import gateway as G     # noqa: E402
from g1pilot.teleoperation.phone import protocol as P    # noqa: E402

WEB = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "g1pilot",
                                   "teleoperation", "phone", "web"))
SHOT_DIR = os.environ.get("G1_PHONE_SHOTS")    # optional: Bildschirmfotos ablegen

FAKE_BT = """
(() => {
  const mk = (uuid) => ({
    uuid,
    async writeValueWithResponse(d) {
      await window.pyWrite(Array.from(d instanceof Uint8Array ? d : new Uint8Array(d)));
    },
    async readValue() { return new DataView(new Uint8Array(await window.pyRead(uuid)).buffer); },
    async startNotifications() { return this; },
    addEventListener(t, fn) {     // Notify nachbilden: PC meldet laufend neue Staende
      if (t === "characteristicvaluechanged") setInterval(fn, 150);
    },
  });
  const svc = { async getCharacteristic(u) { return mk(u); } };
  const dev = {
    gatt: { connected: true, async connect() { return { async getPrimaryService() { return svc; } }; },
            disconnect() {} },
    addEventListener() {},
  };
  Object.defineProperty(navigator, "bluetooth", { value: { async requestDevice() { return dev; } } });
})();
"""


class Bridge:
    def __init__(self, gw):
        self.gw = gw
        self.lines = P.LineBuffer()
        self.txp = P.TxPages()        # dieselbe Seitenlogik wie der BLE-Server
        gw.on_tx = self._tx
        self.writes = []

    def _tx(self, data):
        self.txp.set(data)

    def write(self, arr):
        for line in self.lines.feed("web", bytes(arr)):
            self.writes.append(line)
            self.gw.handle_line("web", line)

    def read(self, uuid):
        if uuid == P.NOTIFY_UUID:
            return list(str(self.txp.n).encode())
        return list(self.txp.read("web", P.TX_UUIDS.index(uuid)))


@pytest.fixture
def server():
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=WEB)
    handler.log_message = lambda *a, **k: None
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{httpd.server_address[1]}"
    httpd.shutdown()


@pytest.fixture
def browser():
    pw = sync_api.sync_playwright().start()
    b = None
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


def test_phone_page_end_to_end(server, browser, tmp_path):
    gw = G.PhoneGateway(store_path=str(tmp_path / "d.json"))
    br = Bridge(gw)
    status = {"sim": False, "started": False, "estop": True, "ack": True, "mode": None,
              "txt": "NOT-HALT aktiv — Roboter ist weich geschaltet.", "col": "#ef5350",
              "info": "G1: Preparation (FSM 4)", "loco": {"damp": True, "preparation": False, "walk": False}}
    actions = []
    ctx = browser.new_context(viewport={"width": 412, "height": 915}, device_scale_factor=2,
                              is_mobile=True, has_touch=True)
    page = ctx.new_page()
    page.add_init_script(FAKE_BT)
    page.expose_function("pyWrite", br.write)
    page.expose_function("pyRead", br.read)
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))

    def pump(ms, until=None, publish=True):
        for _ in range(max(1, ms // 100)):
            page.wait_for_timeout(100)
            if publish:
                gw.publish_status(status)
            acts, _events = gw.tick()
            actions.extend(acts)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def shot(name):
        if SHOT_DIR:
            os.makedirs(SHOT_DIR, exist_ok=True)
            page.screenshot(path=os.path.join(SHOT_DIR, name), full_page=True)

    page.goto(server + "/index.html")
    assert page.is_visible("#pairView")
    shot("1_koppeln.png")
    code = gw.start_pairing(G.ROLE_OPERATE)
    page.fill("#pairName", "Nils Pixel")
    page.fill("#pairCode", code.lower())
    page.click("#btnPair")
    assert pump(5000, lambda: gw.session is not None), errors
    assert gw.devices and list(gw.devices.values())[0]["name"] == "Nils Pixel"
    assert pump(2000, lambda: page.is_visible("#mainView"))
    assert page.inner_text("#pill") == "Live"
    assert page.is_visible("#estopBanner")
    shot("2_ansicht.png")

    # Ohne Steuerhoheit keine Bedienelemente
    assert not page.is_visible("#btnAck")
    page.click("#btnTake")
    assert pump(2000, lambda: gw.has_control())
    assert pump(1000, lambda: page.is_visible("#btnAck"))

    # Quittieren: Schieber nur halb -> nichts
    page.click("#btnAck")
    assert page.is_visible("#sheet")
    shot("3_quittieren.png")
    box = page.locator("#slider").bounding_box()
    y = box["y"] + box["height"] / 2
    page.mouse.move(box["x"] + 30, y)
    page.mouse.down()
    page.mouse.move(box["x"] + box["width"] * 0.5, y, steps=5)
    page.mouse.up()
    pump(500)
    assert [a for a in actions if a["action"] == "ack"] == []
    # ganz durch -> ack
    page.mouse.move(box["x"] + 30, y)
    page.mouse.down()
    page.mouse.move(box["x"] + box["width"] - 4, y, steps=8)
    page.mouse.up()
    assert pump(2000, lambda: any(a["action"] == "ack" for a in actions))
    assert not page.is_visible("#sheet")

    # NOT-HALT
    status.update(estop=False, ack=False, started=True, txt="Gestartet.", col="#aaa")
    page.click("#estop")
    assert pump(2000, lambda: any(a["action"] == "estop" for a in actions))

    # Alte Daten -> gesperrt (NOT-HALT bleibt)
    pump(300)
    pump(1500, publish=False)
    assert page.is_visible("#staleBar")
    assert page.inner_text("#pill") == "Daten alt"
    assert page.is_disabled("#btnModeWalk") and page.is_enabled("#estop")
    shot("4_daten_alt.png")
    pump(1000)
    assert not page.is_visible("#staleBar")

    # Seite verlassen (Bildschirm aus) -> Bedienung abgegeben
    page.evaluate("""() => {
        Object.defineProperty(document, 'visibilityState', {value: 'hidden', configurable: true});
        document.dispatchEvent(new Event('visibilitychange'));
    }""")
    assert pump(2000, lambda: not gw.has_control())
    assert errors == []
    ctx.close()

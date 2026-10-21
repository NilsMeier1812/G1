#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BLE-Server gegen ein nachgebautes BlueZ auf einem privaten D-Bus (ohne
Bluetooth-Hardware): Registrierung, Objektbaum/Signaturen, Werben, Writes
(auch in Stuecken mit offset), lange Reads, Notify, Verbindungsende.

Braucht dbus-daemon + dbus-next, sonst uebersprungen."""
import asyncio
import json
import shutil
import subprocess
import time

import pytest

pytest.importorskip("dbus_next")
from dbus_next import Variant, Message, MessageType            # noqa: E402
from dbus_next.aio import MessageBus                        # noqa: E402
from dbus_next.service import ServiceInterface, method, dbus_property, PropertyAccess, signal  # noqa: E402,F401

from g1pilot.teleoperation.phone import ble_server as B     # noqa: E402
from g1pilot.teleoperation.phone import gateway as G        # noqa: E402
from g1pilot.teleoperation.phone import protocol as P       # noqa: E402

DAEMON = shutil.which("dbus-daemon")
pytestmark = pytest.mark.skipif(DAEMON is None, reason="dbus-daemon fehlt")


@pytest.fixture
def bus_address(tmp_path):
    proc = subprocess.Popen([DAEMON, "--session", "--nofork", "--print-address=1"],
                            stdout=subprocess.PIPE, text=True)
    addr = proc.stdout.readline().strip()
    try:
        yield addr
    finally:
        proc.terminate()
        proc.wait(timeout=5)


class FakeBluez:
    """org.bluez auf dem Test-Bus: Adapter hci0 mit GattManager1 und
    LEAdvertisingManager1. Bei RegisterApplication wird -- wie bei BlueZ --
    sofort GetManagedObjects der App gelesen."""

    def __init__(self):
        self.apps, self.advs, self.objects, self.adv_props = [], [], None, None

    async def start(self, addr):
        self.bus = await MessageBus(bus_address=addr).connect()
        outer = self

        class Adapter(ServiceInterface):
            def __init__(self):
                super().__init__("org.bluez.Adapter1")

            @dbus_property(access=PropertyAccess.READ)
            def Powered(self) -> "b":
                return True

        class Gatt(ServiceInterface):
            def __init__(self):
                super().__init__("org.bluez.GattManager1")

            @method()
            def RegisterApplication(self, app: "o", options: "a{sv}"):
                outer.apps.append(app)

            @method()
            def UnregisterApplication(self, app: "o"):
                pass

        class Adv(ServiceInterface):
            def __init__(self):
                super().__init__("org.bluez.LEAdvertisingManager1")

            @method()
            def RegisterAdvertisement(self, adv: "o", options: "a{sv}"):
                outer.advs.append(adv)

            @method()
            def UnregisterAdvertisement(self, adv: "o"):
                pass

        for iface in (Adapter(), Gatt(), Adv()):
            self.bus.export("/org/bluez/hci0", iface)
        await self.bus.request_name("org.bluez")

    async def read_tree(self, server_name):
        intro = await self.bus.introspect(server_name, B.APP_PATH)
        om = self.bus.get_proxy_object(server_name, B.APP_PATH, intro).get_interface(
            "org.freedesktop.DBus.ObjectManager")
        self.objects = await om.call_get_managed_objects()
        path = B.APP_PATH + "/adv0"
        intro = await self.bus.introspect(server_name, path)
        props = self.bus.get_proxy_object(server_name, path, intro).get_interface(
            "org.freedesktop.DBus.Properties")
        self.adv_props = await props.call_get_all("org.bluez.LEAdvertisement1")

    async def char(self, server_name, path):
        intro = await self.bus.introspect(server_name, path)
        return self.bus.get_proxy_object(server_name, path, intro).get_interface(
            "org.bluez.GattCharacteristic1")

    async def device_gone(self, dev_path):
        self.bus.export(dev_path, _Dev())
        msg = Message.new_signal(dev_path, "org.freedesktop.DBus.Properties", "PropertiesChanged",
                                 "sa{sv}as", ["org.bluez.Device1", {"Connected": Variant("b", False)}, []])
        await self.bus.send(msg)


class _Dev(ServiceInterface):
    def __init__(self):
        super().__init__("org.bluez.Device1")


def _run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


def test_ble_server_against_fake_bluez(bus_address, tmp_path):
    async def scenario():
        bz = FakeBluez()
        await bz.start(bus_address)
        gw = G.PhoneGateway(store_path=str(tmp_path / "d.json"))
        srv = B.BleServer(gw, adapter="hci0", name="G1-Test", bus_address=bus_address)
        assert srv.start()
        t_end = time.monotonic() + 5
        while "wirbt" not in srv.state and time.monotonic() < t_end:
            await asyncio.sleep(0.05)
        assert "wirbt" in srv.state, srv.state
        name = srv._bus.unique_name
        assert bz.apps == [B.APP_PATH] and bz.advs == [B.APP_PATH + "/adv0"]
        await bz.read_tree(name)
        uuids = {v["org.bluez.GattCharacteristic1"]["UUID"].value
                 for k, v in bz.objects.items() if "org.bluez.GattCharacteristic1" in v}
        assert uuids == {P.RX_UUID, P.NOTIFY_UUID, *P.TX_UUIDS}
        svc = [v for v in bz.objects.values() if "org.bluez.GattService1" in v][0]
        assert svc["org.bluez.GattService1"]["UUID"].value == P.SERVICE_UUID
        assert bz.adv_props["ServiceUUIDs"].value == [P.SERVICE_UUID]
        assert bz.adv_props["LocalName"].value == "G1-Test"

        base = B.APP_PATH + "/service0"
        rx = await bz.char(name, base + "/char0")
        notify = await bz.char(name, base + "/char1")
        tx0 = await bz.char(name, base + "/char2")
        dev = "/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF"
        opt = {"device": Variant("o", dev)}

        # Kopplung ueber den echten D-Bus-Weg, Write in zwei Stuecken (offset)
        code = gw.start_pairing()
        np_ = P.nonce()
        line = json.dumps({"t": "pair", "dev": "d1", "name": "Pixel", "np": np_,
                           "mac": P.pair_mac(code, "d1", "Pixel", np_)}) + "\n"
        raw = line.encode()
        await rx.call_write_value(raw[:20], {**opt, "offset": Variant("q", 0)})
        await rx.call_write_value(raw[20:], {**opt, "offset": Variant("q", 20)})
        await asyncio.sleep(0.1)
        await notify.call_start_notify()
        n_before = (await notify.call_read_value({})).decode()
        assert int(n_before) >= 1
        # Langer Read in zwei Teilen: zweiter Teil kommt aus derselben Momentaufnahme
        first = await tx0.call_read_value({**opt, "offset": Variant("q", 0)})
        gw.publish_status({"neu": True})                # TX aendert sich zwischendurch
        await asyncio.sleep(0.1)
        rest = await tx0.call_read_value({**opt, "offset": Variant("q", 10)})
        assert rest == first[10:]
        n, doc = P.join_pages([first])
        pre = json.loads(doc)["pre"]
        assert pre["t"] == "paired" and pre["to"] == "d1"
        assert "d1" in gw.devices
        assert int((await notify.call_read_value({})).decode()) > int(n_before)

        # Anmelden, dann Verbindung weg -> Sitzung endet
        dkey = P.device_key(code, "d1", np_)
        nc = P.nonce()
        await rx.call_write_value((json.dumps({"t": "hello", "dev": "d1", "nc": nc}) + "\n").encode(), opt)
        await asyncio.sleep(0.05)
        pages = [await tx0.call_read_value(opt)]
        ns = json.loads(P.join_pages(pages)[1])["pre"]["ns"]
        await rx.call_write_value((json.dumps({"t": "auth", "dev": "d1",
                                               "cli": P.client_proof(dkey, "d1", nc, ns)}) + "\n").encode(), opt)
        await asyncio.sleep(0.05)
        assert gw.session is not None and gw.session.link == dev
        await bz.device_gone(dev)
        t_end = time.monotonic() + 3
        while gw.session is not None and time.monotonic() < t_end:
            await asyncio.sleep(0.05)
        assert gw.session is None
        srv.stop()
        assert srv.state == "aus"

    _run(scenario())


def test_without_dbus_next_reports_error(monkeypatch, tmp_path):
    monkeypatch.setattr(B, "HAVE_DBUS", False)
    srv = B.BleServer(G.PhoneGateway(store_path=str(tmp_path / "d.json")))
    assert srv.start() is False and "dbus-next" in srv.state

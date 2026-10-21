#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BlueZ-GATT-Server (BLE-Peripheral) fuer die Handy-Bedienung, ueber D-Bus
(dbus-next, asyncio im eigenen Thread). Reiner Transport: jede Zeile geht an
PhoneGateway.handle_line(), jeder neue TX-Wert kommt von gateway.on_tx.

Dienst SERVICE_UUID:
  RX      (write)          Handy -> PC, Zeilen mit "\\n"
  NOTIFY  (read, notify)   kurzer Zaehler: TX hat sich geaendert
  TX0..3  (read)           TX-Text in Seiten (protocol.tx_pages)

Adapter per G1_PHONE_BT_ADAPTER (Default hci0). Mit nur EINEM Dongle teilt
sich das Handy den Funkchip mit dem Controller -- spaeter zweiten Dongle
(hci1) nehmen. Der Server scannt nie und aendert am Adapter nichts ausser
dem Werben (Advertising), solange er laeuft.

encrypt=True verlangt zusaetzlich BLE-Kopplung (encrypt-read/-write) -- die
Sicherheit haengt aber NICHT davon ab (signierte Nachrichten, gateway.py).
"""

import asyncio
import threading
import traceback

from g1pilot.teleoperation.phone import protocol as P

try:
    from dbus_next import BusType, Message, Variant
    from dbus_next.aio import MessageBus
    from dbus_next.service import ServiceInterface, method, dbus_property, PropertyAccess
    from dbus_next.constants import MessageType
    HAVE_DBUS = True
except ImportError:          # Image ohne dbus-next: Handy-Zugang meldet sich als nicht verfuegbar
    HAVE_DBUS = False
    ServiceInterface = object

    def method(*_a, **_k):
        return lambda f: f

    def dbus_property(*_a, **_k):
        return lambda f: f

    class PropertyAccess:     # noqa: D401
        READ = None

BLUEZ = "org.bluez"
GATT_MANAGER = "org.bluez.GattManager1"
ADV_MANAGER = "org.bluez.LEAdvertisingManager1"
APP_PATH = "/org/g1pilot/phone"


if HAVE_DBUS:
    class _ObjectManager(ServiceInterface):
        def __init__(self, server):
            super().__init__("org.freedesktop.DBus.ObjectManager")
            self._server = server

        @method()
        def GetManagedObjects(self) -> "a{oa{sa{sv}}}":
            return self._server.managed_objects()

    class _Service(ServiceInterface):
        def __init__(self, path):
            super().__init__("org.bluez.GattService1")
            self.path = path

        @dbus_property(access=PropertyAccess.READ)
        def UUID(self) -> "s":
            return P.SERVICE_UUID

        @dbus_property(access=PropertyAccess.READ)
        def Primary(self) -> "b":
            return True

        def props(self):
            return {"UUID": Variant("s", P.SERVICE_UUID), "Primary": Variant("b", True)}

    class _Characteristic(ServiceInterface):
        def __init__(self, server, path, uuid, flags, service_path):
            super().__init__("org.bluez.GattCharacteristic1")
            self.server, self.path, self.uuid = server, path, uuid
            self.flags, self.service_path = list(flags), service_path
            self.value = b""
            self.notifying = False

        @dbus_property(access=PropertyAccess.READ)
        def UUID(self) -> "s":
            return self.uuid

        @dbus_property(access=PropertyAccess.READ)
        def Service(self) -> "o":
            return self.service_path

        @dbus_property(access=PropertyAccess.READ)
        def Flags(self) -> "as":
            return self.flags

        @dbus_property(access=PropertyAccess.READ)
        def Value(self) -> "ay":
            return self.value

        @method()
        def ReadValue(self, options: "a{sv}") -> "ay":
            return self.server.read(self, options)

        @method()
        def WriteValue(self, value: "ay", options: "a{sv}"):
            self.server.write(self, value, options)

        @method()
        def StartNotify(self):
            self.notifying = True

        @method()
        def StopNotify(self):
            self.notifying = False

        def props(self):
            return {"UUID": Variant("s", self.uuid), "Service": Variant("o", self.service_path),
                    "Flags": Variant("as", self.flags)}

    class _Advertisement(ServiceInterface):
        def __init__(self, name):
            super().__init__("org.bluez.LEAdvertisement1")
            self._name = name

        @dbus_property(access=PropertyAccess.READ)
        def Type(self) -> "s":
            return "peripheral"

        @dbus_property(access=PropertyAccess.READ)
        def ServiceUUIDs(self) -> "as":
            return [P.SERVICE_UUID]

        @dbus_property(access=PropertyAccess.READ)
        def LocalName(self) -> "s":
            return self._name

        @method()
        def Release(self):
            pass


def _opt(options, key, default=None):
    v = options.get(key) if options else None
    return default if v is None else getattr(v, "value", v)


class BleServer:
    """start() -> Thread mit asyncio-Loop; stop() raeumt auf. state: Text fuer
    die GUI ("aus" | "startet" | "wirbt" | "Fehler: ...")."""

    def __init__(self, gateway, adapter="hci0", name="G1", encrypt=False,
                 log=None, bus_address=None):
        self.gw = gateway
        self.adapter = adapter
        self.name = name
        self.encrypt = bool(encrypt)
        self._log = log or (lambda text: None)
        self._bus_address = bus_address
        self.state = "aus"
        self._loop = None
        self._thread = None
        self._bus = None
        self._lines = P.LineBuffer()
        self._tx = P.TxPages()   # Seiten + Momentaufnahme je Geraet
        self._chars = []
        gateway.on_tx = self._on_tx

    # ── Objektbaum ──────────────────────────────────────────────────────
    def _build(self):
        svc_path = f"{APP_PATH}/service0"
        rd = ["read"] + (["encrypt-read"] if self.encrypt else [])
        wr = ["write"] + (["encrypt-write"] if self.encrypt else [])
        self.service = _Service(svc_path)
        self.rx = _Characteristic(self, f"{svc_path}/char0", P.RX_UUID, wr, svc_path)
        self.notify = _Characteristic(self, f"{svc_path}/char1", P.NOTIFY_UUID,
                                      rd + ["notify"], svc_path)
        self.notify.value = b"0"
        self.tx = [_Characteristic(self, f"{svc_path}/char{2 + i}", u, rd, svc_path)
                   for i, u in enumerate(P.TX_UUIDS)]
        self._chars = [self.rx, self.notify] + self.tx
        self.adv = _Advertisement(self.name)
        self.objman = _ObjectManager(self)

    def managed_objects(self):
        objs = {self.service.path: {"org.bluez.GattService1": self.service.props()}}
        for ch in self._chars:
            objs[ch.path] = {"org.bluez.GattCharacteristic1": ch.props()}
        return objs

    # ── GATT-Zugriffe (Loop-Thread) ─────────────────────────────────────
    def read(self, ch, options):
        offset = int(_opt(options, "offset", 0) or 0)
        device = str(_opt(options, "device", "") or "")
        if ch is self.notify:
            value = ch.value
        elif ch in self.tx:
            return bytes(self._tx.read(device, self.tx.index(ch), offset))
        else:
            value = b""
        return bytes(value[offset:])

    def write(self, ch, value, options):
        if ch is not self.rx:
            return
        link = str(_opt(options, "device", "") or "unbekannt")
        offset = int(_opt(options, "offset", 0) or 0)
        for line in self._lines.feed(link, bytes(value), offset):
            try:
                self.gw.handle_line(link, line)
            except Exception as e:  # noqa: BLE001
                self._log(f"Handy: Zeile verworfen ({e})")

    def _on_tx(self, data):
        loop = self._loop
        if loop is None:
            self._apply_tx(data)
        else:
            loop.call_soon_threadsafe(self._apply_tx, data)

    def _apply_tx(self, data):
        n = self._tx.set(data)
        if not HAVE_DBUS or not getattr(self, "notify", None):
            return
        self.notify.value = str(n).encode("ascii")
        if self.notify.notifying:
            self.notify.emit_properties_changed({"Value": self.notify.value})

    # ── Verbindungsende erkennen ────────────────────────────────────────
    def _on_message(self, msg):
        if (msg.message_type != MessageType.SIGNAL or msg.member != "PropertiesChanged"
                or not msg.body or msg.body[0] != "org.bluez.Device1"):
            return
        changed = msg.body[1]
        conn = changed.get("Connected")
        if conn is not None and not conn.value:
            self._lines.drop(msg.path)
            self._tx.drop(msg.path)
            self.gw.link_lost(msg.path)

    # ── Start/Stop ──────────────────────────────────────────────────────
    def start(self):
        if not HAVE_DBUS:
            self.state = "Fehler: dbus-next fehlt (pip install dbus-next)"
            self._log("Handy: " + self.state)
            return False
        self.state = "startet"
        self._step = "Start"
        self._thread = threading.Thread(target=self._run, name="phone-ble", daemon=True)
        self._thread.start()
        return True

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._setup())
            self._loop.run_forever()
        except Exception as e:  # noqa: BLE001
            # Viele D-Bus-/asyncio-Fehler haben leeren Text -> Schritt + Typ mit ausgeben.
            name = getattr(e, "type", None) or type(e).__name__
            text = str(e) or repr(e)
            self.state = f"Fehler bei {self._step}: {name}: {text}"
            self._log(f"Handy-Bluetooth: {self.state}")
            traceback.print_exc()
        finally:
            try:
                self._loop.run_until_complete(self._teardown())
            except Exception:  # noqa: BLE001
                pass
            self._loop.close()
            self._loop = None

    async def _setup(self):
        self._step = "Verbindung zum System-D-Bus (/run/dbus/system_bus_socket)"
        if self._bus_address:
            self._bus = await MessageBus(bus_address=self._bus_address).connect()
        else:
            self._bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
        self._build()
        bus = self._bus
        bus.export(APP_PATH, self.objman)
        bus.export(self.service.path, self.service)
        for ch in self._chars:
            bus.export(ch.path, ch)
        bus.export(f"{APP_PATH}/adv0", self.adv)
        path = f"/org/bluez/{self.adapter}"
        self._step = f"BlueZ-Adapter {path} (laeuft bluetoothd?)"
        intro = await bus.introspect(BLUEZ, path)
        obj = bus.get_proxy_object(BLUEZ, path, intro)
        try:
            adapter = obj.get_interface("org.bluez.Adapter1")
            if not await adapter.get_powered():
                self._log(f"Handy: Bluetooth-Adapter {self.adapter} ist aus (bluetoothctl power on).")
        except Exception:  # noqa: BLE001
            pass
        self._gatt = obj.get_interface(GATT_MANAGER)
        self._advm = obj.get_interface(ADV_MANAGER)
        self._step = "GATT-Dienst anmelden"
        await self._gatt.call_register_application(APP_PATH, {})
        self._step = "Werbung anmelden"
        await self._advm.call_register_advertisement(f"{APP_PATH}/adv0", {})
        self._step = "Signale abonnieren"
        bus.add_message_handler(self._on_message)
        await bus.call(Message(
            destination="org.freedesktop.DBus", path="/org/freedesktop/DBus",
            interface="org.freedesktop.DBus", member="AddMatch", signature="s",
            body=["type='signal',sender='org.bluez',interface='org.freedesktop.DBus.Properties',"
                  "member='PropertiesChanged',arg0='org.bluez.Device1'"]))
        self.gw.refresh_tx()
        self.state = f"wirbt als »{self.name}« ({self.adapter})"
        self._log(f"Handy-Bluetooth bereit: {self.state}")

    async def _teardown(self):
        for call in (lambda: self._advm.call_unregister_advertisement(f"{APP_PATH}/adv0"),
                     lambda: self._gatt.call_unregister_application(APP_PATH)):
            try:
                await call()
            except Exception:  # noqa: BLE001
                pass
        if self._bus is not None:
            self._bus.disconnect()

    def stop(self):
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self.state = "aus"

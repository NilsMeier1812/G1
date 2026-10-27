"""Handy-Bedienung der Demo-GUI ueber Bluetooth LE (siehe docs/12-A-Handy.md).

protocol.py  : Kennungen, Rahmen, Schluessel/HMAC (ohne Abhaengigkeiten)
gateway.py   : Kopplung, Sitzung, Rechte, Herzschlag, Steuerhoheit (ohne Qt/BLE)
ble_server.py: BlueZ-GATT-Server ueber D-Bus (dbus-next), eigener Thread
link.py      : Einbindung in die Demo-GUI (Qt), Befehle -> bestehende Knopf-Methoden
web/         : Handy-Seite (Web Bluetooth, Chrome/Android)
"""

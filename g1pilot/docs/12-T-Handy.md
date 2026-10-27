# Handy-Bedienung — Technik

Richtet sich an Entwickler. Einrichten und Bedienen: [12-A-Handy.md](12-A-Handy.md).
Sicherheitsabwägung (WLAN, Bildschirmspiegelung, App): Analysebericht
`analysen/handy_gui_sicherheitsanalyse.md` im Projektordner (nicht im Repo).

## Aufbau

```
Handy (Chrome)                     PC: Demo-GUI-Prozess
web/app.js  ── BLE GATT ──►  ble_server.py (BlueZ/D-Bus, Thread)
web/proto.js                      │ handle_line() / on_tx
                                  ▼
                             gateway.py  (Kopplung, Sitzung, Rechte,
                                  │       Herzschlag, Steuerhoheit)
                                  ▼ tick() 20 Hz / publish_status() 5 Hz
                             link.py ──► DemoGUI-Methoden (emergency_stop,
                                         _start_robot, _start_step, request_mode,
                                         run_poses, _safe_pose, _home, _cancel)
```

| Datei | Inhalt |
|---|---|
| `g1pilot/teleoperation/phone/ble_server.py` | GATT-Server über BlueZ/D-Bus (`dbus-next`), Adapterwahl (`G1_PHONE_BT_ADAPTER=auto`: Dongle ohne gekoppelten Controller) |
| `g1pilot/teleoperation/phone/gateway.py` | Kopplung, Anmeldung, Sitzung, Rollen, Steuerhoheit, Herzschlag, Posen-Katalog |
| `g1pilot/teleoperation/phone/protocol.py` | Signaturen, Schlüsselableitung, Zeilenformat (Gegenstück `web/proto.js`) |
| `g1pilot/teleoperation/phone/link.py` | Gateway ↔ Demo-GUI (Status sammeln, Aktionen ausführen, 📱-Dialog) |
| `g1pilot/teleoperation/phone/web/` | Handy-Seite (`index.html`, `app.js`, `proto.js`, Service Worker `sw.js`, Manifest) |
| `.github/workflows/phone-pages.yml` (Repo-Wurzel) | veröffentlicht `phone/web/` auf GitHub Pages bei Änderungen auf `main` |
| `Makefile` → `make phone-web` | Handy-Seite per USB (`adb reverse`, Webserver nur auf `127.0.0.1:8790`) |

**Neue Version der Handy-Seite:** `CACHE` in `web/sw.js` **und** `VERSION` in
`web/app.js` hochzählen (derzeit `v9`). Erst dann bietet die Seite am Handy
„Aktualisieren“ an; ohne Hochzählen bleibt die alte Version im Cache.

## Protokoll

**GATT-Dienst** `6b1e0001-…`: `RX` (write) nimmt Zeilen mit `\n` an. Lange Writes kommen bei BlueZ in Stücken
mit `offset` an, `LineBuffer` setzt sie zusammen. `NOTIFY` (read, notify) ist ein kurzer Zähler „TX neu“.
`TX0..3` (read) enthält den TX-Text in Seiten zu höchstens 480 Byte mit Kopf `n:i:k:`, weil ein GATT-Wert
höchstens 512 Byte haben darf. Lange Reads mit `offset` werden je Gerät aus derselben Momentaufnahme bedient.

**Sicherheit im Protokoll** (`protocol.py`, identisch in `web/proto.js`): Die Kopplung nutzt den Code K
(80 Bit, nur optisch). Das Handy sendet `pair{dev,name,np,mac=HMAC(K,…)}`, beide leiten D = HMAC(K,
`dev|dev|np`) ab. Die Anmeldung läuft als `hello{nc}` → `chal{ns, srv=HMAC(D,…)}` (PC beweist D) →
`auth{cli=HMAC(D,…)}`. Der Sitzungsschlüssel ist S = HMAC(D, `ses|nc|ns`). Danach ist jede Zeile
`A<zähler>.<HMAC(S)>.<json>` mit streng steigendem Zähler in beide Richtungen. Inhalte sind lesbar, aber nicht
fälschbar und nicht wiederholbar. Deshalb braucht es keine BLE-Kopplung, sie ist aber zuschaltbar.

## Aktionen und Rechte

Jede Aktion vom Handy wird im Gateway (`ACTIONS` in `gateway.py`) gegen
Mindestrolle, Steuerhoheit und Statusbezug geprüft und dann über `link.py`
als Methode der Demo-GUI ausgeführt:

| Aktion | Rolle | Steuerhoheit | Statusbezug | GUI-Funktion |
|---|---|---|---|---|
| `estop` | Ansicht | nein | nein | NOT-HALT |
| `cancel` | Bediener | nein | nein | Bewegung stoppen |
| `take` / `release` | Bediener / Ansicht | – | ja / nein | Bedienung übernehmen / abgeben (Gateway selbst) |
| `ack` | Bediener | ja | ja | Start / NOT-HALT quittieren |
| `loco` | Bediener | ja | ja | Startleiste Damp / Preparation / Walk |
| `mode` | Bediener | ja | ja | GEHEN / GREIFEN |
| `hand` | Bediener | ja | ja | Hände öffnen / schließen |
| `seq`, `seq_next` | Bediener | ja | ja | Ablauf starten, Weiter |
| `seq_mode` | Bediener | ja | nein | Am Stück / Schritt für Schritt |
| `safe`, `home` | Bediener | ja | ja | Sichere Pose, Grundstellung |
| `pose`, `move` | Bediener | ja | ja | Pose anfahren, relative Bewegung |
| `pose_save` | Bediener | ja | nein | Pose speichern (bewegt nichts) |

**Regeln im Gateway:** Es gibt eine Sitzung gleichzeitig. Befehle brauchen Rolle, Steuerhoheit und einen
Statusbezug (`s` höchstens 15 Statusnummern alt, also ca. 3 s; das Handy sperrt selbst schon nach 1,5 s).
Ausnahmen sind `estop`, `cancel`, `seq_mode` und `pose_save` (bewegt nichts). Jede
Befehls-ID wirkt nur einmal. Die Posenliste geht nur auf `{"t":"cat"}`, jeder Status trägt ihren Hash `ph`. Nach 3 falschen Codes ist die Kopplung beendet. Nach 5 falschen Anmeldungen ist
das Gerät 60 s gesperrt.

## Zeiten

 Herzschlag 5 Hz (Handy), Steuerhoheit 2 s, Sitzung 4 s, Status 5 Hz, Handy „Daten alt“ nach 1,5 s.

## Tests

| Test | Inhalt |
|---|---|
| `test/test_phone_gateway.py` | Gateway: Kopplung, Anmeldung, Signaturen, Wiederholschutz, Rechte, Timeouts |
| `test/test_phone_ble_dbus.py` | BLE-Server gegen ein nachgebautes BlueZ auf privatem D-Bus |
| `test/test_phone_protocol_js.py` | `proto.js` (Node) gegen `protocol.py` |
| `test/test_phone_web_e2e.py` | Handy-Seite in Chromium gegen das echte Gateway (Web Bluetooth als Attrappe) |
| `test/test_phone_web_offline.py` | Offline aus dem Cache, neue Version erst nach „Aktualisieren“ |
| `test/test_phone_demo_gui.py` | Einbindung in die Demo-GUI (offscreen) |

Echtes Bluetooth (BlueZ, Android-Chrome) ist damit **nicht** abgedeckt. Das muss einmal am PC mit Handy
geprüft werden.

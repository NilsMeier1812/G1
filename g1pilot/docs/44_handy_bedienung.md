# Handy-Bedienung (Bluetooth LE)

Ein Android-Handy zeigt den Zustand der Demo-GUI und kann bedienen: **NOT-HALT
auslösen**, **NOT-HALT quittieren/Start**, die **Startleiste** (Damp,
Preparation, Walk, nur am echten G1), **GEHEN/GREIFEN** wählen, **Hände**
öffnen/schließen, **Abläufe**, **Sichere Pose** und **Grundstellung**. Der PC
rechnet weiter alles. Das Handy ruft nur dieselben Funktionen auf wie die Knöpfe
der Demo-GUI, mit denselben Prüfungen.

Gehen und Arm-Jog steuerst du weiter mit dem Controller. Das Handy kann das
bewusst nicht.

> **Der NOT-HALT vom Handy läuft über den PC.** Hängt der PC, wirkt er nicht. Maßgeblich bleiben
> Controller (PS bzw. R1+L1+PS), Unitree-Fernbedienung und Kran. Die müssen bei jedem Test greifbar sein.

Hintergrund und Abwägung (WLAN, Bildschirmspiegelung, App): Analysebericht
`analysen/handy_gui_sicherheitsanalyse.md` im Projektordner.

## Voraussetzungen

- **Android-Handy mit Chrome.** iOS kann kein Web Bluetooth.
- **Bluetooth-Adapter am PC** mit BLE. Heute ist das der eine USB-Dongle (`hci0`), der sich den Funk mit dem
  Controller teilt. Mit einem zweiten Dongle nur fürs Handy `G1_PHONE_BT_ADAPTER=hci1` setzen
  (`bluetoothctl list` zeigt die Adapter).
- **Demo-GUI** (`G1_GUI=demo`). Der Streamdeck hat keinen Handy-Zugang.
- **Docker-Image neu bauen** nach dem Update (neue Abhängigkeit `dbus-next`): im Start-Fenster unter
  „Erweitert → Docker-Images neu bauen“ einmal anhaken.

## Einrichten (einmalig)

1. **Handy-Seite aufs Handy bringen.** Am Handy in Chrome **https://nilsmeier1812.github.io/G1/** öffnen und
   über das Chrome-Menü „Zum Startbildschirm hinzufügen“ wählen. Danach läuft die Seite **offline aus dem
   Cache**, also auch ohne Netz in der Halle.
   **Kein automatisches Update:** Die installierte Version bleibt, bis du sie selbst wechselst. Liegt eine neue
   Version auf GitHub, lädt Chrome sie im Hintergrund, und oben erscheint „Neue Version … Aktualisieren“. Erst
   der Tipp darauf macht sie aktiv. Welche Version läuft, steht unter „Verbindung“ („Handy-Seite v2“).
   Die Seite ist öffentlich, das ist gewollt. Der Schutz hängt am Kopplungscode, der nur am PC erscheint.
   Veröffentlicht wird automatisch bei Änderungen unter `phone/web/` auf `main`
   (`.github/workflows/phone-pages.yml`, einmalig Settings → Pages → Source „GitHub Actions“).

   *Ohne Internet:* Handy per USB (USB-Debugging an), am PC im Ordner `g1pilot` `make phone-web`, am Handy
   `http://localhost:8790/index.html` öffnen. Der Webserver lauscht nur auf `127.0.0.1`.
2. **Start-Fenster:** Ausstattung → **Handy-Zugang (Bluetooth)** anhaken, starten. Ohne den Haken gibt es weder
   Knopf noch Funk.
3. **Koppeln:** In der Demo-GUI oben **📱 Handy** → **Als Bediener koppeln** (oder **Nur Ansicht koppeln**).
   Es erscheint ein 16-stelliger Code, 2 Minuten gültig und nur einmal. Am Handy Name und Code eingeben, dann
   **Verbinden & koppeln** und in der Chrome-Auswahl „G1 REAL“ bzw. „G1 SIM“ wählen.

Der Code geht nie über Funk. Aus ihm leiten PC und Handy getrennt denselben Geräteschlüssel ab.

## Bedienen

1. Handy-Seite öffnen → **Mit G1 verbinden** (Chrome fragt jedes Mal nach dem Gerät, das ist gewollt).
2. Du siehst den Zustand: Statuszeile der GUI, Onboard-Modus, Controller, Modus, NOT-HALT.
3. Zum Bedienen **Bedienung übernehmen**. Die PC-GUI zeigt dann oben „📱 *Name* bedient“.
4. Alles, was den Roboter bewegt oder umschaltet, bestätigst du mit einem **Schieber ganz nach rechts**. Tippen
   allein reicht nicht. Über dem Schieber steht, was passiert. Ausnahmen sind Hände öffnen/schließen und
   **Bewegung stoppen** (einfaches Tippen).
5. **NOT-HALT** (unten, immer da): einmal tippen. Das geht auch ohne Bedienung übernehmen und auch bei
   „Daten alt“.

| Anzeige / Ereignis | Wirkung |
|---|---|
| **Live** (grün) | Status jünger als 1 s, Bedienung frei. |
| **Daten alt** (rot) | Kein frischer Status seit 1 s. Alles gesperrt außer NOT-HALT. |
| Bildschirm aus, App gewechselt, Tab im Hintergrund | Handy gibt die Bedienung sofort ab. |
| Kein Herzschlag vom Handy für 2 s (Funk weg, Akku leer) | PC entzieht die Bedienung. **Ein vom Handy gestarteter Ablauf, die Sichere Pose und die Grundstellung werden abgebrochen.** |
| Keine Nachricht für 4 s | Sitzung beendet, PC-GUI meldet „Handy getrennt“. |
| PC-GUI: 📱 → **Bedienung zurückholen** | Handy ist sofort wieder nur Ansicht. |

Am PC kannst du unter **📱 Handy** Geräte entfernen. Ein entferntes Handy muss neu koppeln.

### Rollen

| Rolle | Darf |
|---|---|
| **Ansicht** | Zustand sehen, NOT-HALT (abschaltbar mit `G1_PHONE_VIEW_ESTOP=0`). |
| **Bediener** | Zusätzlich nach „Bedienung übernehmen“ alles aus der Liste oben. |

Es ist immer nur **ein Handy gleichzeitig** verbunden. Ein zweites Handy bekommt „Gerade ist … verbunden“.

### Was das Handy nicht kann

Gehen und Arm-/Hand-Jog, Posen speichern, „Real/Sim beenden“, Parameter, Kopplung neuer Geräte (nur am PC).

## Einstellungen

| Variable | Default | Bedeutung |
|---|---|---|
| `G1_PHONE` | `0` | `1` = Handy-Zugang an (Start-Fenster-Haken). |
| `G1_PHONE_BT_ADAPTER` | `hci0` | Bluetooth-Adapter für das Handy. |
| `G1_PHONE_BT_NAME` | `G1 SIM` / `G1 REAL` | Name beim Werben. |
| `G1_PHONE_BT_ENCRYPT` | `0` | `1` = zusätzlich Android-Bluetooth-Kopplung verlangen (die Sicherheit hängt nicht davon ab). |
| `G1_PHONE_VIEW_ESTOP` | `1` | Ansicht-Handys dürfen NOT-HALT. |
| `G1_PHONE_STORE` | `~/.config/g1pilot/phone_devices.json` | Gekoppelte Handys (Datei 0600; im Container auf `~/.config/g1pilot` des Hosts gemountet). |

## Fehlerbehebung

- **Tooltip/Dialog zeigt „Fehler: dbus-next fehlt“:** Image neu bauen.
- **„Fehler: … org.bluez …“:** Läuft `bluetoothd` auf dem Host? Ist der Adapter an (`bluetoothctl show`,
  `bluetoothctl power on`)? Stimmt `G1_PHONE_BT_ADAPTER`?
- **Chrome findet „G1 …“ nicht:** Bluetooth und Standort am Handy an? Läuft die GUI mit dem Haken? Bei geändertem
  Dienst einmal Bluetooth am Handy aus- und einschalten (Android merkt sich alte Dienste).
- **Controller hakt, seit das Handy verbunden ist:** Das ist der geteilte Dongle. Abhilfe ist ein zweiter
  Dongle (`hci1`), über ein kurzes USB-Verlängerungskabel und nicht direkt neben dem ersten.
- **„Nicht mehr gekoppelt“:** Gerät wurde am PC entfernt oder die Geräteliste fehlt. Neu koppeln.

---

## Technik

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

Quellcode unter `g1pilot/teleoperation/phone/`.

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

**Regeln im Gateway:** Es gibt eine Sitzung gleichzeitig. Befehle brauchen Rolle, Steuerhoheit und einen
Statusbezug (`s` höchstens 5 Statusnummern alt, also ca. 1 s). Ausnahmen sind `estop` und `cancel`. Jede
Befehls-ID wirkt nur einmal. Nach 3 falschen Codes ist die Kopplung beendet. Nach 5 falschen Anmeldungen ist
das Gerät 60 s gesperrt.

**Zeiten:** Herzschlag 5 Hz (Handy), Steuerhoheit 2 s, Sitzung 4 s, Status 5 Hz, Handy „Daten alt“ nach 1 s.

**Tests:**

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

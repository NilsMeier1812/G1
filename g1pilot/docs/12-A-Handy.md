# Handy-Bedienung (Bluetooth LE)

Ein Android-Handy zeigt den Zustand der Demo-GUI und kann bedienen: **NOT-HALT
auslösen**, **NOT-HALT quittieren/Start**, die **Startleiste** (Damp,
Preparation, Walk, nur am echten G1), **GEHEN/GREIFEN** wählen, **Hände**
öffnen/schließen, **Pose anfahren**, **Pose speichern**, **Abläufe**, **Sichere Pose** und
**Grundstellung**. Dazu zeigt es, was der Controller nur über die LED verrät (Verschieben/Drehen,
Tempo, Hand) und am echten G1 die Motortemperaturen. Der PC
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
- **Bluetooth-Adapter am PC** mit BLE. Am besten ein **eigener zweiter Dongle nur fürs Handy** (siehe
  „Zweiter Dongle“ unten). Mit nur einem Dongle teilen sich Handy und Controller den Funk.
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
2. Du siehst den Zustand: Statuszeile der GUI, Onboard-Modus, NOT-HALT und darunter drei Kacheln
   zum Controller: **Verschieben/Drehen** (bzw. Gehen, Geht, Hüfte), **Tempo** und **Hand**. Am echten
   G1 folgt eine Zeile mit den Temperaturen (Max und je Gruppe, gelb ab 60 °C, rot ab 75 °C wie in der
   PC-GUI).
3. Zum Bedienen oben **Bedienen** tippen (wird zu **Abgeben**). Die PC-GUI zeigt dann oben
   „📱 *Name* bedient“.
4. Alles, was den Roboter bewegt oder umschaltet, bestätigst du mit einem **Schieber ganz nach rechts**. Tippen
   allein reicht nicht. Über dem Schieber steht, was passiert. Ausnahmen sind Hände öffnen/schließen und
   **Bewegung stoppen** (einfaches Tippen).
5. **NOT-HALT** (unten, immer da): einmal tippen. Das geht auch ohne »Bedienen« und auch bei
   „Daten alt“.

| Anzeige / Ereignis | Wirkung |
|---|---|
| **Live** (grün) | Status jünger als 1,5 s, Bedienung frei. |
| **Daten alt** (rot) | Kein frischer Status seit 1,5 s. Alles gesperrt außer NOT-HALT und Pose speichern. |
| Bildschirm aus, App gewechselt, Tab im Hintergrund | Handy gibt die Bedienung sofort ab; nach 4 s ohne Herzschlag endet die Sitzung. Zurück: **Mit G1 verbinden** tippen. |
| Kein Herzschlag vom Handy für 2 s (Funk weg, Akku leer) | PC entzieht die Bedienung. **Ein vom Handy gestarteter Ablauf, die Sichere Pose und die Grundstellung werden abgebrochen.** |
| Keine Nachricht für 4 s | Sitzung beendet, PC-GUI meldet „Handy getrennt“. |
| PC-GUI: 📱 → **Bedienung zurückholen** | Handy ist sofort wieder nur Ansicht. |

**Handy-Seite v9:** automatisches Wiederverbinden (v7/v8) wieder entfernt, es führte zu ständigem Trennen. Herunterziehen zum Neuladen bleibt abgeschaltet.

Am PC kannst du unter **📱 Handy** Geräte entfernen. Ein entferntes Handy muss neu koppeln.

### Posen

Im Modus GREIFEN:

- **Pose anfahren …** öffnet die Liste aller gespeicherten Posen nach Kategorie, darunter die
  **↕ Relativen Bewegungen** (z. B. „Box anheben“, gerechnet ab der aktuellen Hand). Antippen, dann
  Schieber. Bricht die Verbindung ab, wird die Fahrt gestoppt.
- **Pose speichern …** speichert die aktuelle Haltung: Name, Kategorie (Vorgabe „Demo“) und was
  gespeichert wird (Arme/Hände einzeln, alle an). Gibt es den Namen schon, warnt das Handy vor dem
  Überschreiben. Unter **An Ablauf anhängen** hängst du die neue Pose direkt als letzten Schritt an
  einen Ablauf (auch einen noch leeren) – wie im PC-Dialog erst, wenn der arm_controller das Speichern
  quittiert hat; das Ergebnis steht danach in der Statuszeile. Neue Kategorien, Abläufe anlegen und
  Umsortieren gehen nur am PC.
- **Abläufe** sind dieselben wie in der PC-GUI (Fenster „Posen & Abläufe“). Darüber wählst du
  **Am Stück** oder **Schritt für Schritt** (gilt für PC, Handy und Controller gemeinsam). Wartet ein
  Ablauf auf Bestätigung, erscheint oben unter „Bewegung stoppen“ der grüne Knopf **Weiter ▶ (n/m)**;
  am Controller geht es auch mit L3 weiter. Zusammenstellen geht nur am PC.

Die Posenliste kommt nur auf Anfrage aufs Handy (bei jeder Änderung neu, der PC prüft alle 2 s).
Passt sie nicht ganz in die Funkübertragung (über ca. 1 KB), fehlen hintere Kategorien und die Liste
sagt das.

### Rollen

| Rolle | Darf |
|---|---|
| **Ansicht** | Zustand sehen, NOT-HALT (abschaltbar mit `G1_PHONE_VIEW_ESTOP=0`). |
| **Bediener** | Zusätzlich nach „Bedienen“ alles aus der Liste oben. |

Es ist immer nur **ein Handy gleichzeitig** verbunden. Ein zweites Handy bekommt „Gerade ist … verbunden“.

### Was das Handy nicht kann

Gehen und Arm-/Hand-Jog, Kategorien anlegen, „Real/Sim beenden“, Parameter, Kopplung neuer Geräte
(nur am PC).

## Einstellungen

| Variable | Default | Bedeutung |
|---|---|---|
| `G1_PHONE` | `0` | `1` = Handy-Zugang an (Start-Fenster-Haken). |
| `G1_PHONE_BT_ADAPTER` | `auto` | Adapter fürs Handy: `auto` = der Dongle ohne gekoppelten Controller, sonst `hciN` oder die MAC des Dongles. |
| `G1_PHONE_BT_NAME` | `G1 SIM` / `G1 REAL` | Name beim Werben. |
| `G1_PHONE_BT_ENCRYPT` | `0` | `1` = zusätzlich Android-Bluetooth-Kopplung verlangen (die Sicherheit hängt nicht davon ab). |
| `G1_PHONE_VIEW_ESTOP` | `1` | Ansicht-Handys dürfen NOT-HALT. |
| `G1_PHONE_STORE` | `~/.config/g1pilot/phone_devices.json` | Gekoppelte Handys (Datei 0600; im Container auf `~/.config/g1pilot` des Hosts gemountet). |

## Zweiter Dongle (Handy und Controller getrennt)

1. Zweiten Dongle einstecken, am besten über ein kurzes USB-Verlängerungskabel und nicht direkt neben
   dem ersten, und nicht an einem USB-3-Port (blaue Buchse), denn USB 3 stört 2,4 GHz.
2. Am PC `bluetoothctl list`: Jetzt stehen zwei `Controller`-Zeilen da. Der alte Dongle ist
   `C0:3A:55:A5:54:25`, der Controller bleibt an ihm gekoppelt und verbindet sich weiter dorthin.
   Falls der neue `Powered: no` zeigt: `bluetoothctl select <MAC-neu>` und `power on`.
3. Nichts einstellen: Mit `G1_PHONE_BT_ADAPTER=auto` nimmt der PC fürs Handy den Dongle, an dem
   **kein** Controller gekoppelt ist. Das klappt auch, wenn Linux die Nummern `hci0`/`hci1` nach einem
   Neustart vertauscht. Der 📱-Dialog zeigt den gewählten Adapter, z. B. `wirbt als »G1 REAL« (hci1, ohne Controller)`.
4. Am Handy einmal Bluetooth aus- und einschalten (es merkt sich sonst den alten Adapter) und neu
   verbinden. Koppeln musst du nicht neu, der Geräteschlüssel hängt nicht am Dongle.
5. Einen **neuen** Controller koppelst du vorher bewusst am alten Dongle:
   `bluetoothctl select C0:3A:55:A5:54:25`, dann wie gewohnt `scan on` / `pair` / `trust`.

Willst du es fest einstellen, schreib `G1_PHONE_BT_ADAPTER=<MAC-neu>` in die Datei `g1pilot/.env`.

## Fehlerbehebung

- **Tooltip/Dialog zeigt „Fehler: dbus-next fehlt“:** Image neu bauen.
- **„Fehler: … org.bluez …“:** Läuft `bluetoothd` auf dem Host? Ist der Adapter an (`bluetoothctl show`,
  `bluetoothctl power on`)? Stimmt `G1_PHONE_BT_ADAPTER`?
- **Chrome findet „G1 …“ nicht:** Bluetooth und Standort am Handy an? Läuft die GUI mit dem Haken? Bei geändertem
  Dienst einmal Bluetooth am Handy aus- und einschalten (Android merkt sich alte Dienste).
- **Controller hakt, seit das Handy verbunden ist:** Das ist der geteilte Dongle. Abhilfe ist ein zweiter
  Dongle (`hci1`), über ein kurzes USB-Verlängerungskabel und nicht direkt neben dem ersten.
- **„Nicht mehr gekoppelt“:** Gerät wurde am PC entfernt oder die Geräteliste fehlt. Neu koppeln.

# Startmenü — Anleitung

Richtet sich an alle Anwender. Wie man den Stack startet: grafisches
Startmenü, Text-Menü, `make` und `docker compose`. Installation vorher:
[01-A-Installation.md](01-A-Installation.md).

## Überblick

```bash
cd g1pilot
./start.sh            # grafisches Startmenü (Standard)
./start.sh --menu     # klassisches Text-Menü
```

Ohne Argumente öffnet `start.sh` das grafische Startmenü (`g1_gui.py`,
PyQt6, Code in `start_gui/`). Das Startmenü läuft auf dem PC selbst, nicht
im Docker-Image. Fehlt PyQt6 (neuer Rechner), fragt `start.sh` „Jetzt
installieren? [J/n]“: Ubuntu 24.04 per `sudo apt-get install -y python3-pyqt6
libqt6svg6`, Ubuntu 22.04 (dort gibt es das Paket nicht) per pip in
`g1pilot/.venv-gui`; bei „n“ nimmt es das Text-Menü;
ohne Display ebenso (erzwingbar mit `--menu` oder `G1_NO_GUI=1`). Beide
sammeln nur Optionen und rufen am Ende `start.sh --yes` mit den passenden
Umgebungsvariablen auf.

Das Startmenü zeigt drei Karten:

| Karte | Wirkung |
|---|---|
| **Simulation** | MuJoCo + Lauf-Policy, gefahrlos testen |
| **Echter Roboter** | G1 per LAN: Unitree-Loco + Arme + Hände (der Roboter bewegt sich!) |
| **Umgebungen** | Sim-Szenen anlegen, bearbeiten, mit dem G1 ansehen ([40-A-Simulation.md](40-A-Simulation.md#umgebungen)) |

Darunter: laufende Prozesse (Ansehen, Stoppen, Entfernen), *Alle Stacks
stoppen* und *Beenden*. Oben rechts öffnet **📖 Dokumentation** (oder `F1`)
den Doku-Viewer, siehe [unten](#doku-viewer).

Alles läuft in einem Fenster; Menü, Optionsseiten und Log-Ansicht werden
ausgetauscht. Startet man einen Stack, erscheint dessen Docker-Ausgabe live
mit einem Stop-Button. Über *‹ Menü* geht man zurück, ohne den Stack zu
beenden — er taucht unter den Prozessen wieder auf.

Die letzte Auswahl merkt sich das Startmenü in
`~/.config/g1pilot/launcher.json` (nicht die Sicherheitsbestätigung und nicht
„Images neu bauen“).

## Optionen

| Abschnitt | Simulation | Echter Roboter |
|---|---|---|
| Umgebung / Verbindung | Kachel mit Vorschaubild je Szene aus `scene_editor/scenes/` (siehe [Vorschaubilder](#vorschaubilder-der-umgebungen)); Kachel **+ Neue Umgebung** führt zu *Umgebungen* | Netzwerk-Interface mit Live-Anzeige (alle 3 s im Hintergrund: Adresse 192.168.123.x, Kabel, Ping zum G1, Inspire-Hände), Knopf **Netzwerk einrichten …** (siehe [unten](#netzwerk-einrichten)) |
| Lauf-Policy | Alt `g1_wholebody` (Standard), AGILE Velocity, AGILE Velocity-Height | — |
| Hände | Karten Rubber, Inspire (Standard) oder Custom; bei Custom Auswahl des Profils (z. B. Box-Haken) und Knopf **Bearbeiten …** fürs Bearbeitungstool; Eintrag **➕ Neue Hand anlegen …** + **Anlegen …** für eine neue Hand aus einer 3D-Datei ([25-A](25-A-Eigene-Haende.md)) | — |
| Ausstattung (Schalter) | Navigation, **Controller (PS4 / Xbox)**, RViz | Inspire-Hände + IPs, RViz (empfohlen), **Controller (PS4 / Xbox) für Arme + Hände** |
| Geh-Limits | — | vorwärts / seitlich / drehen (Default 0,4 m/s / 0,3 m/s / 0,4 rad/s) |
| Erweitert (eingeklappt) | Sim-Tempo (1.0 = Echtzeit), Images neu bauen | Modbus-Port, Gamepad-Name (alter Joystick), Images neu bauen |
| Sicherheit | — | Häkchen „Sicherheitshinweise gelesen, Bereich frei“, sonst bleibt *Starten* grau |

- Unten links fasst eine Zeile die Auswahl zusammen (z. B. *scene_A5 · Alt
  (Standard) · Inspire · Navigation*), **? Hilfe** oben rechts öffnet die
  passende Anleitung im Doku-Viewer.
- **Controller** gilt für PS4 und Xbox
  ([11-A-Controller.md](11-A-Controller.md)). Aus = alter Joystick-Modus
  ([13-A-Alter-Joystick.md](13-A-Alter-Joystick.md#alter-joystick-modus-ohne-schalter-ps4-controller)).
- **Handy-Zugang** ist keine Option mehr: Die Demo-GUI hat oben immer den Knopf
  *Handy*, und erst der Klick darauf startet Bluetooth
  ([12-A-Handy.md](12-A-Handy.md)). Ohne Dongle gibt es beim Start also keine
  Fehlermeldung.
- Die Bedienoberfläche ist immer die Demo-GUI; die frühere Auswahl
  *Bedienoberfläche* (Streamdeck) wurde 2026-10-30 entfernt.
- Welche Umgebungsvariable hinter welcher Option steht:
  [04-T-Umgebungsvariablen.md](04-T-Umgebungsvariablen.md).

## Netzwerk einrichten

Für den echten G1 braucht der PC eine feste Adresse im Roboter-LAN
`192.168.123.x`. Auf einem frisch installierten PC geht das direkt im
Startmenü: **Echter Roboter → Netzwerk einrichten …**

1. Interface zum G1 wählen (kabelgebunden: `enp…`, `eno…`, `eth…`, `enx…`)
   und die Adresse dieses PCs (Vorschlag `192.168.123.222`).
2. Der Dialog zeigt, was er vorfindet (vorhandenes Profil `g1-robot`, kein
   Kabel, WLAN gewählt, zweites Interface im Roboter-LAN, ungültige oder
   schon vergebene Adresse wie `.161`) und darunter **genau die Befehle**, die
   er ausführt.
3. **Jetzt einrichten** legt das NetworkManager-Profil `g1-robot` an (oder
   passt ein vorhandenes an): feste IP, kein Gateway, kein DNS, startet von
   selbst beim Einstecken. WLAN und andere Verbindungen bleiben unberührt.
   Das System fragt dabei nach dem Passwort (polkit). Klappt das nicht,
   wiederholt der Dialog den Schritt mit `pkexec`.
4. Ist `ufw` aktiv, öffnet ein Schalter die Firewall für dieses Interface
   (`ufw allow in on …`), sonst kommen die DDS-Daten nicht an.
5. Das Protokoll zeigt jede Ausgabe. Fehlt nur das Kabel, ist das kein
   Fehler: das Profil aktiviert sich beim Einstecken.

**Profil entfernen** löscht `g1-robot` wieder (mit Rückfrage). Ohne
NetworkManager zeigt der Dialog die `ip addr add`-Befehle zum Kopieren; die
gelten nur bis zum Neustart. Der manuelle Weg steht in
[50-A-Echter-Roboter.md](50-A-Echter-Roboter.md#b--netzwerk-ersteinrichtung).

**Live-Anzeige:** Solange die Seite *Echter Roboter* offen ist, prüft das
Startmenü alle 3 s im Hintergrund Adresse, Kabel, Ping zum G1
(`192.168.123.161`) und, wenn die Hände an sind, Modbus-Port der beiden
Inspire-Hände. Grün heißt: im Roboter-LAN und der G1 antwortet. Orange nennt,
was fehlt. Neu eingesteckte USB-Adapter oder eine eben vergebene IP tauchen
ohne neues Öffnen in der Interface-Liste auf. **Jetzt prüfen** prüft sofort.

## Vorschaubilder der Umgebungen

Die Kacheln zeigen ein Bild jeder Umgebung (schräg von oben, ohne G1; die
Standard-Kachel zeigt den G1 im Standard-Gelände). Die Bilder liegen in
`unitree_mujoco/scene_editor/scenes/thumbs/` und sind im Repo eingecheckt.
Ist eine Umgebung neu oder hat sich ihre XML geändert, rendert das
Startmenü das Bild beim Öffnen und nach jedem Scene-Editor-Lauf im
Hintergrund neu (unten rechts steht dann *Vorschaubilder werden erzeugt …*).
Dafür braucht es das `scene_editor/.venv` (dort ist MuJoCo installiert);
ohne zeigt die Kachel den Anfangsbuchstaben.

Von Hand, z. B. nach Änderungen an Meshes (die prüft der Hash nicht):

```bash
cd unitree_mujoco/scene_editor
.venv/bin/python render_thumbs.py --all     # oder: Umgebungen → Vorschaubilder neu erzeugen
```

## Doku-Viewer

**📖 Dokumentation** im Startmenü, `F1` oder **? Hilfe** auf den
Optionsseiten öffnen ein eigenes Fenster mit allen Dokumenten aus `docs/`,
gerendert (Überschriften, Tabellen, Code, Bilder). Links:

- Baum nach dem Doku-Schema (Gruppe → Anleitung / Technik, Technik grau).
  Neue Dateien `GT-X-Name.md` erscheinen automatisch.
- *Doku durchsuchen* filtert nach Titel **und** Inhalt.
- Links auf andere Dokumente und Abschnitte (`#anker`) öffnen im Viewer,
  Web-Links im Browser, Links auf Code-Dateien mit dem System-Programm.
- `‹` / `›` bzw. `Alt+←` / `Alt+→` wie im Browser, `Strg+F` sucht im
  Dokument, `F5` lädt neu (nach Änderungen an der Datei), *Als Datei öffnen*
  öffnet die `.md` im Texteditor.

## Text-Menü und ohne Menü

Das Text-Menü fragt dieselben Punkte nacheinander ab; ENTER nimmt den
markierten Default. Erste Frage: Simulation oder Echter Roboter. Den
Handy-Zugang ist auch hier immer dabei (Knopf *Handy*); `G1_PHONE=0`
schaltet ihn ganz ab.

```bash
# Nicht-interaktiv (Sim, Defaults/Env-Overrides):
USE_RVIZ=true G1_PS4_ARMS=1 ./start.sh --yes
# Nicht-interaktiv (Real, erfordert ausdrückliche Bestätigung):
G1_MODE=real ROBOT_INTERFACE=enp3s0 G1_REAL_CONFIRM=1 ./start.sh --yes
```

Im `--yes`-Modus startet REAL nur mit `G1_REAL_CONFIRM=1`.

## Was `start.sh` zusätzlich erledigt

- X11 für die Container freigeben (`xhost`), GPU-Compose-Dateien dazunehmen
  (`/dev/dri` → `docker-compose.gpu.yml`, NVIDIA → `docker-compose.nvidia.yml`),
  bei `G1_PS4_ARMS=1` in der Sim `docker-compose.ps4.yml` (`/dev/input`).
- Sim: die gewählte Umgebung mit `build_env_scene.py` zur Szene
  `scene_env_<name>.xml` kombinieren.
- **Shutdown-Watcher** starten: *Sim beenden* bzw. *Real beenden* in der
  Demo-GUI beendet darüber den Stack (`docker/sim_shutdown_watcher.sh`,
  `docker/real_shutdown_watcher.sh`, siehe [10-T-Demo-GUI.md](10-T-Demo-GUI.md#beenden-simreal)).

## `make` und `docker compose`

| Befehl | Wirkung |
|---|---|
| `make sim` | Stack im Vordergrund starten (Ctrl-C stoppt) |
| `make sim-bg` | Stack im Hintergrund (ohne Shutdown-Watcher: *Sim beenden* geht dann nur mit `make stop`) |
| `make real ROBOT_INTERFACE=<nic>` | Echten Roboter starten (schlank: Arme + Hände + Loco) |
| `make stop` | Stack stoppen |
| `make logs` / `make status` | Logs folgen / Container-Status |
| `make shell-sim` / `make shell-mujoco` / `make shell-real` | Shell im jeweiligen Container |
| `make clean` | Container + Images entfernen |

Jedes `make sim`/`make real` baut das benötigte Image bei
Bedarf automatisch nach — ein separater `make build-*`-Aufruf ist nur nötig,
um das Bauen vom Starten zu trennen (am echten G1: **vor** dem Wechsel ins
Roboter-LAN, solange Internet da ist). Weitere Ziele:
[61-A-Werkzeuge.md](61-A-Werkzeuge.md#make-ziele).

Intern ist `make` nur ein dünner Wrapper um
`docker compose --profile <sim|real>`:

```bash
# Simulation: MuJoCo-G1 + g1pilot (robot_state, Arme, RViz, Teleop)
G1_SIM_MODE=true docker compose --profile sim up

# Echter Roboter (schlank: Arme + Hände + Unitree-Loco-Controller)
ROBOT_INTERFACE=<iface> docker compose --profile real up
```

## Fenster-Icons

Jedes G1-Fenster hat ein eigenes Icon und eine eigene `WM_CLASS`, damit es
im Dock bzw. in der Taskleiste unterscheidbar ist: Startmenü (grün), Demo-GUI
Sim (blau) und Real (rot), MuJoCo-Viewer (türkis).
Details: `resource/icons/README.md`.

## Fehlerbehebung

| Symptom | Ursache / Abhilfe |
|---|---|
| `./start.sh` zeigt nur das Text-Menü | PyQt6 fehlt auf dem Host (`start.sh` meldet das): `sudo apt install python3-pyqt6 libqt6svg6`. |
| Doku-Viewer zeigt Diagramme nicht | `libqt6svg6` fehlt (SVG-Unterstützung von Qt). |
| Umgebungs-Kachel ohne Bild | `scene_editor/.venv` fehlt (*Umgebungen → Setup*), oder Rendern fehlgeschlagen: Ausgabe von `render_thumbs.py` im Terminal, aus dem `start.sh` lief. |
| Real: *Starten* bleibt grau | Sicherheits-Häkchen setzen. |
| Real: „Keine 192.168.123.x-Adresse auf …“ | **Netzwerk einrichten …** klicken ([unten](#netzwerk-einrichten)) oder von Hand, siehe [50-A-Echter-Roboter.md](50-A-Echter-Roboter.md#b--netzwerk-ersteinrichtung). |
| Real: „… aber der G1 antwortet nicht“ | IP stimmt, aber kein Ping zu 192.168.123.161: G1 eingeschaltet und hochgefahren? Richtiges Kabel/Interface? |
| *Sim beenden* meldet, es gehe nur mit `make stop` | Stack wurde ohne Watcher gestartet (`make sim-bg` oder `docker compose` direkt). |
| PS4-Controller in der Sim nicht gefunden | Start über `start.sh` (nimmt `docker-compose.ps4.yml` dazu); unter WSL gibt es kein `/dev/input`. |

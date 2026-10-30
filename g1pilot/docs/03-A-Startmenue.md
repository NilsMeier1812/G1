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
Tkinter). Fehlt Tkinter oder ein Display, fällt es automatisch auf das
Text-Menü zurück (erzwingbar mit `--menu` oder `G1_NO_GUI=1`). Beide
sammeln nur Optionen und rufen am Ende `start.sh --yes` mit den passenden
Umgebungsvariablen auf.

Das Startmenü zeigt drei Karten:

| Karte | Wirkung |
|---|---|
| **Simulation** | MuJoCo + Lauf-Policy, gefahrlos testen |
| **Echter Roboter** | G1 per LAN: Unitree-Loco + Arme + Hände (der Roboter bewegt sich!) |
| **Umgebungen** | Sim-Szenen anlegen, bearbeiten, mit dem G1 ansehen ([40-A-Simulation.md](40-A-Simulation.md#umgebungen)) |

Darunter: laufende Prozesse (Ansehen, Stoppen, Entfernen), *Alle Stacks
stoppen*, das Menü **Dokumentation ▾** (öffnet die Dokumente aus
`docs/`) und *Beenden*.

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
| Umgebung / Verbindung | Szene aus `scene_editor/scenes/` | Netzwerk-Interface (prüft auf 192.168.123.x) |
| Lauf-Policy | Alt `g1_wholebody` (Standard), AGILE Velocity, AGILE Velocity-Height | — |
| Ausstattung | Inspire-Hände (Default an), Navigation, **PS4-Controller**, **Handy-Zugang**, RViz | Inspire-Hände + IPs, RViz (empfohlen), **PS4-Controller für Arme + Hände**, **Handy-Zugang** |
| Geh-Limits | — | vorwärts / seitlich / drehen (Default 0,4 m/s / 0,3 m/s / 0,4 rad/s) |
| Erweitert (eingeklappt) | Sim-Tempo (1.0 = Echtzeit), Images neu bauen | Modbus-Port, Gamepad-Name (alter Joystick), Images neu bauen |
| Sicherheit | — | Häkchen „Sicherheitshinweise gelesen, Bereich frei“, sonst bleibt *Starten* grau |

- **PS4-Controller** gilt auch für den Xbox-Controller
  ([11-A-Controller.md](11-A-Controller.md)). Aus = alter Joystick-Modus
  ([13-A-Alter-Joystick.md](13-A-Alter-Joystick.md#alter-joystick-modus-ohne-schalter-ps4-controller)).
- **Handy-Zugang** läuft über die Demo-GUI ([12-A-Handy.md](12-A-Handy.md)).
- Die Bedienoberfläche ist immer die Demo-GUI; die frühere Auswahl
  *Bedienoberfläche* (Streamdeck) wurde 2026-10-30 entfernt.
- Welche Umgebungsvariable hinter welcher Option steht:
  [04-T-Umgebungsvariablen.md](04-T-Umgebungsvariablen.md).

## Text-Menü und ohne Menü

Das Text-Menü fragt dieselben Punkte nacheinander ab; ENTER nimmt den
markierten Default. Erste Frage: Simulation oder Echter Roboter. Den
Handy-Zugang gibt es dort nicht als Frage, nur über `G1_PHONE=1`.

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
| `./start.sh` zeigt nur das Text-Menü | `python3-tk` fehlt auf dem Host: `sudo apt install python3-tk`. |
| Real: *Starten* bleibt grau | Sicherheits-Häkchen setzen. |
| Real: „Keine 192.168.123.x-Adresse auf diesem Interface“ | Kabel/IP prüfen, siehe [50-A-Echter-Roboter.md](50-A-Echter-Roboter.md#b--netzwerk-ersteinrichtung). |
| *Sim beenden* meldet, es gehe nur mit `make stop` | Stack wurde ohne Watcher gestartet (`make sim-bg` oder `docker compose` direkt). |
| PS4-Controller in der Sim nicht gefunden | Start über `start.sh` (nimmt `docker-compose.ps4.yml` dazu); unter WSL gibt es kein `/dev/input`. |

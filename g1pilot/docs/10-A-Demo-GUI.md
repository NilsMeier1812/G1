# Demo-GUI — Anleitung

Richtet sich an alle, die den G1 bedienen oder vorführen (Messe, Besuch,
Lehre). Die Demo-GUI ist die Standard-Oberfläche in Sim und Real.
Aufbau im Code, Topics und offene Punkte: [10-T-Demo-GUI.md](10-T-Demo-GUI.md).

## Ziel

Die Streamdeck-GUI (`ui_interface.py`) zeigt alle 25 Funktionen gleichzeitig.
Für Entwicklung ist das richtig, für eine Vorführung zu viel: Besucher sehen
nicht, in welchem Zustand der Roboter ist und welche Knöpfe gerade Sinn
ergeben. Die Demo-GUI reduziert die Bedienung auf **eine Entscheidung zur Zeit**.

## Aufbau: drei Bereiche

```
┌──────────────────────────────────────────────────────────────────┐
│ 1 · MODUS                                                        │
│ ┌──────────────────────────┐  ┌──────────────────────────┐       │
│ │          GEHEN           │  │         GREIFEN          │       │
│ │  antippen zum Wechseln   │  │        ● AKTIV           │       │
│ └──────────────────────────┘  └──────────────────────────┘       │
│        (grau, inaktiv)          (orange, weißer Rand)            │
├──────────────────────────────────────────────────────────────────┤
│ 2 · STEUERUNG            Rahmen in der Farbe des aktiven Modus   │
│                                                                  │
│  GREIFEN              [ ARME | HÄNDE ]                           │
│  ──────────────────────────────────────────────────────────────  │
│   Abläufe: [ Arbeitsplatz 1 ] [ Arbeitsplatz 2 ]                 │
│   [Am Stück|Schritt für Schritt] [ Weiter ▶ ] [Posen & Abläufe…] │
│   [Sichere Pose]   [Grundstellung]   [Bewegung stoppen]          │
│   Erweitert ▸   (Pose anfahren … / Pose speichern … / Marker)    │
│                                                                  │
│  GEHEN:                                                          │
│     ( Knopf )        [⟲] [▲] [⟳]          Tempo                  │
│      ziehen          [◀]     [▶]          [Langsam]              │
│                          [▼]              [ Normal]              │
├──────────────────────────────────────────────────────────────────┤
│ 3 · STATUS                                                       │
│ ▌Arbeitsplatz 1 · Schritt 2/5 …  [Szene] [Schubsen] ( NOT-HALT ) │
└──────────────────────────────────────────────────────────────────┘
```

### 1 · Modus (oben)

Zwei große Kacheln, **genau eine** ist aktiv:

| Kachel | Bedeutung | Topics |
|---|---|---|
| GEHEN (blau) | Lauf-Regler; Arme eingeklappt (Sim mit AGILE-Policy: Arme frei) | `/g1pilot/start_walking` |
| GREIFEN (orange) | Stand mit geplanten Füßen, Arme frei | `/g1pilot/arms/enabled` + `/g1pilot/start_balancing` |

Jede Kachel hat drei klar unterscheidbare Zustände:

- **aktiv**: Vollfarbe, weißer Rand, Text „● AKTIV“
- **wechselt …**: gestrichelter Rand in Modusfarbe. Das gibt es nur beim Wechsel
  zu GEHEN, weil die Arme erst in die Lauf-Pose fahren. Bestätigt wird der Wechsel
  durch `/g1pilot/arms/walk_ready`, spätestens nach 6 s (Timeout, wie in
  `loco_sim`). Mit einer AGILE-Lauf-Policy in der Sim bleiben die Arme frei,
  `walk_ready` kommt sofort.
- **inaktiv**: dunkelgrau, „antippen zum Wechseln“

Auf dem echten Roboter sind die Kacheln gesperrt, bis **Start** gedrückt
wurde. Start ist der erste Knopf der Startleiste (eine Zeile: *Start | Damp |
Preparation | Walk |* Rückmeldung des loco_client) und sendet `/g1pilot/start`:
gibt die Kacheln frei, quittiert einen NOT-HALT (arm_controller, loco_client,
Controller) und stellt einen G1, der noch in Damp/Zero-Torque liegt, auf
Preparation (FSM 4); steht er schon (Preparation/Walk), schaltet Start nichts um.
Danach ist der Knopf grau (*Gestartet ✓*), nach einem NOT-HALT wieder frei
(*NOT-HALT quittieren*). Der Controller-Zustand steht nicht mehr als eigene
Zeile in der GUI (Modus, Hand und Tempo zeigt die GUI selbst); solange ein
Controller aktiv ist, zeigt *🎮 Belegung* oben rechts die Tastenbelegung.
In der Sim startet die GUI wie der Streamdeck automatisch in GREIFEN und
wiederholt die Anforderung, bis arm_controller und loco_sim sie bestätigt
haben ([40-T-Simulation.md › Auto-Start](40-T-Simulation.md#auto-start-greifen)).

**Startleiste (nur echter G1):** *Damp* und ein *Walk*, der den Walk-Modus
wechselt, brauchen einen zweiten Tipp innerhalb weniger Sekunden („Wirklich
Damp? nochmal tippen“, „Kran gelockert? nochmal tippen“). *Walk* zeigt schon vor
dem Klick die Bodenkontakt-Prüfung (Knie-Last aus `rt/lowstate`): »Walk
Bodenkontakt ✓« oder gesperrt mit „kein Bodenkontakt“; die GEHEN-Kachel zeigt
dann „gesperrt · kein Bodenkontakt“. Die Info-Zeile daneben zeigt den
Onboard-Modus des G1, z. B. „G1: Walk mit Taille (FSM 501)“, und bei einem
abgelehnten Befehl den RPC-Fehlercode. Details:
[50-A-Echter-Roboter.md](50-A-Echter-Roboter.md).

**Mit Controller (PS4/Xbox) gekoppelt:** Kacheln, Hand-Auswahl, Tempo »Hand
bewegen« und Geh-Tempo teilen sich GUI und Controller
(`/g1pilot/teleop_state`). L2 am Controller → Kachel GEHEN, R2 → GREIFEN;
die GUI sendet dabei selbst nichts. Details:
[11-A-Controller.md › Controller und Demo-GUI gekoppelt](11-A-Controller.md#controller-und-demo-gui-gekoppelt).

**Kopfzeile rechts:** *🎮 Belegung* (nur solange ein Controller aktiv ist,
zeigt die Tastenbelegung), *📱 Handy* (nur mit Handy-Zugang `G1_PHONE=1`,
Koppeln und Steuerhoheit, siehe [12-A-Handy.md](12-A-Handy.md)) und
*⛶ Vollbild*.

### 2 · Steuerung (Mitte)

Hier steht **nur** der Inhalt des aktiven Modus. Der Rahmen hat dessen Farbe,
damit die Zuordnung ohne Lesen klar ist.

**GEHEN**

- **Knopf** (`VirtualJoystick` aus dem Streamdeck): ziehen = laufen,
  loslassen = stehen.
- **Pfeiltasten** ▲▼◀▶ und **⟲/⟳ drehen**: *gedrückt halten* = laufen,
  loslassen = sofort 0. Das ist bewusst ein Tot-Mann-Prinzip: Ein einzelner Klick
  löst kein Dauerlaufen aus.
- **Tempo** Langsam (0.3) / Normal (0.6) als Faktor auf die normierte
  Geschwindigkeit. Vollgas (1.0) ist in der Demo absichtlich nicht wählbar.
- `/g1pilot/loco_cmd_vel` wird mit ~30 Hz gesendet, außerhalb von GEHEN immer 0.
- **AUTO NAV** (Toggle, `/g1pilot/auto_enable`): Der Roboter navigiert selbstständig
  zum Ziel, das in RViz gesetzt wurde (»2D Goal Pose«). Solange AUTO NAV an ist,
  sind Knopf und Pfeile gesperrt und ausgegraut, und die GUI sendet **kein**
  `loco_cmd_vel`. In der Sim sendet die Navigation über dasselbe Topic
  (`joy_to_cmdvel`), und die Nullen der GUI würden sie sonst ständig ausbremsen.
  Beim Wechsel zu GREIFEN und bei NOT-HALT geht AUTO NAV automatisch aus. Der
  Knopf ist nur aktiv, wenn der Nav-Stack läuft (`G1_ENABLE_NAV` bzw.
  `G1_ENABLE_LIDAR`); sonst erklärt ein Hinweis, wie man ihn einschaltet.
- **AUTO-NAV-Zustände** (aus `/g1pilot/nav_status` von `nav2point`): Der
  Knopf zeigt *aus* → *AN, wartet auf Ziel* → *läuft …* (gestrichelter Rahmen,
  wie beim Moduswechsel) → *AN, Ziel erreicht* (grün). Nach dem Ankommen bleibt
  AUTO NAV an, das nächste Ziel aus RViz wird direkt angefahren. „Angekommen“
  steht zusätzlich im STATUS-Bereich. Findet der Planer keinen Weg, meldet der
  Status das, und der Roboter bleibt stehen.
- **Zur Station** (Quickbefehle, ein Knopf je `station_<Name>` der Szene):
  setzt das Ziel und schaltet AUTO NAV ein. Während des Laufens ist der
  Stations-Knopf gestrichelt umrandet. Beim Ankommen schaltet sich AUTO NAV
  **automatisch wieder aus**, und der Status zeigt „Angekommen an Station …“.
  Ohne Weg (oder ohne Pfad nach 5 s) geht AUTO NAV ebenfalls wieder aus.

**GREIFEN**

In der Titelzeile neben „GREIFEN“ schaltet ein Umschalter **ARME | HÄNDE**
zwischen zwei Seiten um (beides braucht viel Platz, darum nie gleichzeitig).
Beim Wechsel in GREIFEN ist ARME aktiv. Eine Linie in Modusfarbe trennt die
Titelzeile vom Inhalt (auch bei GEHEN).

*Seite ARME*

- **Abläufe** (seit 2026-10-22, ausführlich in
  [23-A-Posen-und-Ablaeufe.md](23-A-Posen-und-Ablaeufe.md)): ein Ablauf ist eine Liste von **Verweisen auf
  gespeicherte Posen** im Pose-Store (`sequences`, siehe
  [23-T-Posen-und-Ablaeufe.md](23-T-Posen-und-Ablaeufe.md)). Dieselbe
  Pose (z. B. `Sichere_Pose`) kann in mehreren Abläufen und mehrfach in einem
  stehen, ohne Kopie und ohne Umbenennen. Jeder Ablauf mit mindestens einem
  Schritt wird ein Knopf „<Name>“, die Statuszeile zeigt „Schritt n/m“.
  Jeder Schritt ist ein geplantes POSE ANFAHREN (Kollisionsschutz wie immer);
  reine Hand-Posen melden nach `hand_only_settle_s` (1.8 s) `reached`.
  Beispiel-Abläufe für die Szene A5 installiert
  `python3 -m g1pilot.tools.install_example_sequences` (gelten genau an der
  jeweiligen Station). Fehlt eine Pose eines Ablaufs, startet er nicht und der
  Status nennt sie.
- **Am Stück | Schritt für Schritt** (Umschalter unter den Ablauf-Knöpfen):
  *Am Stück* fährt nach jedem `reached` (+ optionaler Pause des Schritts)
  sofort weiter. *Schritt für Schritt* hält nach jedem Schritt an; weiter geht
  es erst mit **Weiter ▶** (GUI), am Handy oder mit **L3** am PS4-/Xbox-
  Controller (Controller vibriert kurz, sobald der Ablauf wartet). Der
  Umschalter wirkt auch während eines Ablaufs ab dem nächsten Schritt; so
  hält man einen Ablauf am Stück an. Start: **real Schritt für Schritt, Sim
  am Stück** (`G1_SEQUENCE_MODE=auto|step`).
- **Abbruch**: `failed`/`rejected`/`cancelled` der laufenden Fahrt, NOT-HALT
  (GUI, Controller PS, Voll-Notaus, Handy, Streamdeck), Stopp (Knopf
  „Bewegung stoppen“, Controller Kreuz, `/g1pilot/pose_store/cancel`),
  Moduswechsel und „Hand bewegen“ während einer Fahrt beenden den Ablauf, auch
  wenn er gerade auf „Weiter“ wartet. Danach geht es nie von selbst weiter,
  auch nicht nach dem Quittieren. Wartet ein Ablauf, darf man die Hand mit Jog
  oder Controller nachkorrigieren; der nächste Schritt wird von dort geplant.
  Solange ein Schritt fährt, sind die Controller-Sticks gesperrt (Anzeige
  „ABLAUF fährt“), damit ein versehentlich bewegter Stick die Fahrt nicht mit
  „manueller Vorrang“ abbricht; Kreuz = Stopp geht immer.
  Kommt auf ein Anfahren 4 s keine Antwort vom Arm-Regler (Arme nicht
  übernommen), bricht der Ablauf mit Hinweis ab.
- **Posen & Abläufe …** öffnet das Fenster zum Zusammenstellen (nicht modal,
  man kann nebenher die Hand bewegen): links die Abläufe (neu, umbenennen,
  kopieren, löschen, starten), in der Mitte die Schritte (ziehen oder ▲/▼,
  entfernen, Pause nach dem Schritt für „Am Stück“) und **„＋ Aktuelle Stellung
  als Schritt …“** (speichert die Pose und hängt sie an, nachdem der
  Arm-Controller das Speichern quittiert hat), rechts alle Posen nach Kategorie
  (Doppelklick oder „← Hinzufügen“ hängt hinter dem markierten Schritt an,
  Anfahren, Löschen, aktuelle Stellung speichern). Ganz oben stehen die
  **relativen Bewegungen** (z. B. „Box anheben“ = Hände 5 cm hoch, egal wo sie
  gerade sind): *Neue relative Bewegung …* / *Bearbeiten …* mit Name, vor/
  links/hoch in cm (je höchstens ±30 cm, Bezug Oberkörper) und welche Hand;
  sie werden wie Posen eingefügt (Schritt „↕ Box anheben (hoch 5 cm · beide)“)
  und geplant angefahren. Jede Änderung wird sofort
  gespeichert. Auch der Dialog „Pose speichern“ hat das Feld **An Ablauf
  anhängen** (neuer Name legt den Ablauf an).
- **Sichere Pose** fährt die gespeicherte Pose `Sichere_Pose` geplant an
  (Arme hoch, Ellbogen hinten; Hinweis, falls sie fehlt; Name über
  `G1_SAFE_POSE` änderbar). **Grundstellung** sendet `/g1pilot/arms/home`: der
  Arm-Controller fährt geplant die **Lauf-Pose** an (Arme neben dem Körper,
  Hände auf Hüfthöhe), genau wie HOMING ARMS am Streamdeck. **Bewegung stoppen**
  sendet `/g1pilot/pose_store/cancel`. Hände öffnen/schließen liegt auf der
  Seite HÄNDE.
- **Hüfte** (Zeile unter den Knöpfen): zeigt den Stand der Hüftdrehung
  („Hüfte 12° links“) und hat den Knopf **Hüfte Mitte**. Gedreht wird mit
  dem Controller (R2 halten + rechter Stick), siehe
  [24-A-Hueftdrehung.md](24-A-Hueftdrehung.md).
- **Erweitert ▸** (eingeklappt) enthält die bisherigen Einzelfunktionen für den
  Betreuer: Pose anfahren (Dialog), Pose speichern (Dialog, Kategorie `Demo`
  vorausgewählt) und Marker folgen. Die Dialoge werden aus `ui_interface.py`
  wiederverwendet.

*Seite HÄNDE* (`teleoperation/hand_panel.py`) ersetzt die beiden Browser-GUIs
der Hand-Bridge (`hand_controller_viewer.html`, `inspire_hand_viewer.html`):

- **Ganze Hand**: beide Hände öffnen/schließen sowie je Hand eigene
  Öffnen/Schließen-Knöpfe (`/g1pilot/hand_action/{left,right}`, wie der
  Streamdeck).
- **Finger**: je Finger ein Soll-Schieber (oben = offen), die Ist-Öffnung
  (gelber Balken, %) und die gemessene Kraft in g (grün → orange → rot bei
  Erreichen des Griffkraft-Limits). Ein Ein/Aus-Hauptschalter fehlt bewusst:
  ein Finger-Befehl aktiviert die Hand bei Bedarf selbst.
- **Kraftzonen**: Handskizze je Hand mit den 17 Taktil-Zonen als Heatmap
  (Spitzenwert je Zone, Nulllage = erste Daten).
- **Erweitert ▸** (eingeklappt): Auswahl **Griffkraft** (setzt das Kraft-Limit
  aller Finger: Sanft 300 g, Mittel 800 g, Fest 1500 g) und **Tempo**
  (Langsam/Normal/Schnell), dazu „Kraftzonen nullen“ (aktuelle Taktil-Werte
  als Nulllage). Einzelne Kraft-Limits je Finger gibt es in der Demo-GUI nicht.
- Kommuniziert nur über ROS: `/g1pilot/hand_cmd` (JSON, dieselben Befehle wie
  der Controller-WebSocket `:8766`) und `/g1pilot/hand_status` (JSON, ~10 Hz).
  Kommt kein Status, erscheint ein Hinweis (Hand-Bridge läuft nicht).

### 3 · Status (unten)

- **Statuszeile** in Klartext mit farbigem Balken, z. B. „Arme werden eingeklappt
  …“, „Arbeitsplatz 1 · Schritt 2/5: …“, „Fertig.“ oder „Abgebrochen (failed) …“.
- **NOT-HALT** ist immer sichtbar, rund und rot. Er sendet dasselbe wie der
  Streamdeck. Beide Kacheln werden danach inaktiv. Was er genau auslöst:
  [51-A-Notaus.md](51-A-Notaus.md).
- **Temperaturen** (nur echter G1, links unten, „TEMP. MOTOREN“): heißester
  Motor und je eine Kachel für Bein L/R, Taille, Arm L/R mit dem Höchstwert,
  gelb ab 60 °C, rot ab 75 °C (`G1_TEMP_WARN` / `G1_TEMP_CRIT`). Klick öffnet
  die Tabelle aller 29 Motoren (beide Sensoren, Spannung) plus IMU. Quelle ist
  `robot_state` (`/g1pilot/temperatures`), reiner Lesepfad.
- **Szene zurücksetzen** (nur Sim): Objekte der Umgebung zurück an den Start.
- **Roboter zurücksetzen** (nur Sim): Roboter nach NOT-HALT oder Sturz zurück
  an den Startpunkt und gehalten (HOLD); beim nächsten Modus-Klick steht er
  sanft auf.
- **Roboter schubsen** (nur Sim): Störtest wie PUSH ROBOT im Streamdeck. Kurz
  tippen = Standard-Schubs (150 N); **gedrückt halten lädt auf** (Anzeige im
  Knopf, bis 2000 N nach 3 s), Loslassen schubst. Er zeigt, dass sich der
  Roboter in beiden Modi fängt. Einstellbar mit `G1_PUSH_MAX_N`,
  `G1_PUSH_CHARGE_S`, `G1_PUSH_TAP_S`.
- **Sim beenden** (nur Sim): schließt den ganzen Sim-Stack (MuJoCo, RViz,
  Demo-GUI) in unter einer Sekunde. Erster Klick schaltet scharf („Wirklich?
  nochmal tippen“, gestrichelt), ein zweiter Klick innerhalb von 4 s beendet.
  Die GUI legt dazu `.sim_shutdown_request` im gemounteten Repo an. Der
  Host-Watcher `docker/sim_shutdown_watcher.sh` (von `./start.sh` und
  `make sim` gestartet) macht daraufhin `docker compose kill` + `down`, also
  sofort und ohne langsames Herunterfahren. Ohne Watcher (z. B. `make sim-bg`)
  meldet die GUI nach 3 s, dass es nur mit `make stop` im Terminal geht.
- **Real beenden** (nur echter Roboter, gleiche Doppelklick-Bestätigung):
  beendet den Stack sicher. Zuerst wird alles angehalten (Ablauf, AUTO NAV,
  Hand bewegen, Gehen über `start_walking` False, also StopMove). Dann gehen
  die Arme aus (`arms/enabled` False). Der arm_controller fährt sie in die
  Pose vom ENABLE zurück und rampt das Gewicht auf 0. Erst wenn
  `/g1pilot/arms/state` False meldet und die Rampe durch ist (2,5 s), legt die
  GUI `.real_shutdown_request` an. Der Host-Watcher
  `docker/real_shutdown_watcher.sh` (von `./start.sh`, `make real` und
  `make real-full` gestartet) stoppt den Stack **sanft** mit
  `docker compose stop` (SIGTERM) + `down`. Die Beine bleiben im
  Onboard-Zustand, es gibt kein Damp: Den Roboter danach per Fernbedienung
  ablegen. Kommt vom Arm-Regler 20 s lang keine Quittung, wird trotzdem
  beendet. Er gibt die Arme beim SIGTERM selbst ab. NOT-HALT bricht
  „Real beenden“ ab.
- Die **Statuszeile** steht über die volle Breite, die Knöpfe darunter.

## Bedienen

```bash
./start.sh          # grafisches Startmenü → Simulation oder Echter Roboter → Bedienoberfläche: Demo-GUI (Default)
./start.sh --menu   # Text-Menü: 2) Bedienoberfläche → Demo-GUI
```

Ablauf anlegen: GREIFEN → **Posen & Abläufe …** → *Neu …*, dann Posen aus
der Liste rechts hinzufügen oder die Hand hinstellen und **„＋ Aktuelle
Stellung als Schritt …“**. Für „Box greifen, anheben, ablegen“: Pose
*Greifen*, dann relative Bewegung *Box anheben* (hoch 5 cm), dann die
Ablage-Pose. Der Ablauf erscheint als Knopf „<Name>“. Die alte
Arbeitsweise geht weiter: eine Pose in die Kategorie `Ablauf <Name>`
speichern hängt sie hinten an den Ablauf <Name> an. Bestehende Kategorien
`Ablauf …` werden beim ersten Speichern als Abläufe übernommen (Reihenfolge
wie bisher nach Namen).

**Fenstergröße / Vollbild:** Die Demo-GUI startet maximiert. Der Steuerbereich
liegt in einem Scrollbereich, damit die Mindesthöhe des Fensters klein bleibt
(~600 px; vorher ~1160 px, höher als ein 1080er-Schirm: der Fenstermanager bot
dann kein Maximieren an und auch das Vollbild blieb größer als der Schirm).
Ab ca. 1000 px Höhe passt alles ohne Scrollen. Knopf *⛶ Vollbild* oben rechts
in der Zeile „1 · MODUS“, im Vollbild heißt er *Vollbild beenden*. Tasten: **F11**
schaltet um, **Esc** verlässt das Vollbild. Funktioniert auch ohne Titelleiste
des Fenstermanagers (Roboter-Rechner). `G1_GUI_FULLSCREEN=1` startet die
Demo-GUI direkt im Vollbild (Default 0 = maximiert). Reine Fensterfunktion,
sendet nichts an den Roboter.

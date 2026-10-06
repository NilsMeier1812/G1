# Demo-Oberfläche — Konzept

Richtet sich an: alle, die den G1 vorführen (Messe, Besuch, Lehre), und an
Entwickler, die die Demo-GUI weiterbauen. Status: **Konzept / Prototyp**
(Branch `anne/demo-gui`).

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
| GEHEN (blau) | Lauf-Policy, Arme eingeklappt | `/g1pilot/start_walking` |
| GREIFEN (orange) | Stand mit geplanten Füßen, Arme frei | `/g1pilot/arms/enabled` + `/g1pilot/start_balancing` |

Jede Kachel hat drei klar unterscheidbare Zustände:

- **aktiv**: Vollfarbe, weißer Rand, Text „● AKTIV“
- **wechselt …**: gestrichelter Rand in Modusfarbe. Das gibt es nur beim Wechsel
  zu GEHEN, weil die Arme erst in die Lauf-Pose fahren. Bestätigt wird der Wechsel
  durch `/g1pilot/arms/walk_ready`, spätestens nach 6 s (Timeout, wie in
  `loco_sim`).
- **inaktiv**: dunkelgrau, „antippen zum Wechseln“

Auf dem echten Roboter sind die Kacheln gesperrt, bis „Roboter starten“
(`/g1pilot/start`) gedrückt wurde. Nach einem NOT-HALT ist das wieder nötig.
In der Sim startet die GUI wie der Streamdeck nach 3 s automatisch in GREIFEN.

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

- **Abläufe**: jede Pose-Store-Kategorie `Ablauf <Name>` wird ein eigener
  Knopf „<Name>“; ihre Posen laufen in Namensreihenfolge ab (`AP1_01_…`,
  `AP1_02_…`), die Statuszeile zeigt „Schritt n/m“. Ablauf-Posen erscheinen
  nicht als Einzelknöpfe. Reine Hand-Posen (nur `left_hand`/`right_hand`)
  melden nach `hand_only_settle_s` (1.8 s) `reached`. Beispiel-Abläufe für die
  Szene A5 (Arbeitsplatz 1: Zylinder links greifen und an rechts übergeben;
  Arbeitsplatz 2: blaue KLT mit beiden Händen anheben) installiert
  `python3 -m g1pilot.tools.install_example_sequences`; sie gelten für den
  Roboter genau an der jeweiligen Station. Der nächste Schritt startet erst,
  wenn `/g1pilot/arm_command/status` `reached` meldet. Bei
  `failed`/`rejected`/`cancelled` bricht der Ablauf ab und der Grund steht im
  Status. Gibt es noch keine Abläufe, erklärt ein Hinweis, wie man einen anlegt.
- **Sichere Pose** fährt die gespeicherte Pose `Sichere_Pose` geplant an
  (Arme hoch, Ellbogen hinten; Hinweis, falls sie fehlt; Name über
  `G1_SAFE_POSE` änderbar). **Grundstellung** sendet `/g1pilot/arms/home`: der
  Arm-Controller fährt geplant die **Lauf-Pose** an (Arme neben dem Körper,
  Hände auf Hüfthöhe), genau wie HOMING ARMS am Streamdeck. **Bewegung stoppen**
  sendet `/g1pilot/pose_store/cancel`. Hände öffnen/schließen liegt auf der
  Seite HÄNDE.
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
  Streamdeck. Beide Kacheln werden danach inaktiv.
- **Szene zurücksetzen** (nur Sim).
- **Roboter schubsen** (nur Sim): Störtest wie PUSH ROBOT im Streamdeck
  (400-ms-Impuls auf `/g1pilot/push`). Er zeigt, dass sich der Roboter in beiden
  Modi fängt.
- **Sim beenden** (nur Sim): schließt den ganzen Sim-Stack (MuJoCo, RViz,
  Demo-GUI) in unter einer Sekunde. Erster Klick schaltet scharf („Wirklich?
  nochmal tippen“, gestrichelt), ein zweiter Klick innerhalb von 4 s beendet.
  Die GUI legt dazu `.sim_shutdown_request` im gemounteten Repo an. Der
  Host-Watcher `docker/sim_shutdown_watcher.sh` (von `./start.sh` und
  `make sim` gestartet) macht daraufhin `docker compose kill` + `down`, also
  sofort und ohne langsames Herunterfahren. Ohne Watcher (z. B. `make sim-bg`)
  meldet die GUI nach 3 s, dass es nur mit `make stop` im Terminal geht.
- Die **Statuszeile** steht über die volle Breite, die Knöpfe darunter.

## Bedienen

```bash
./start.sh          # grafisches Startmenü → Simulation → Bedienoberfläche: Demo-GUI (Default)
./start.sh --menu   # Text-Menü: 2) Bedienoberfläche → Demo-GUI
```

Ablauf anlegen: GREIFEN → Erweitert → *Pose speichern …*, Kategorie
`Ablauf <Name>`, Posen in Reihenfolge benennen (`AP1_01_…`, `AP1_02_…`).
Der Ablauf erscheint danach als Knopf „<Name>“.

## Technik

| Datei | Änderung |
|---|---|
| `g1pilot/teleoperation/demo_gui.py` | neu, PyQt6. `DemoNode` erbt die Publisher von `StreamDeck` und abonniert zusätzlich `arms/walk_ready` und `arm_command/status`. |
| `setup.py` | Entry-Point `demo_gui` |
| `launch/teleoperation_launcher.launch.py` | startet je nach `G1_GUI` **entweder** `ui_interface` **oder** `demo_gui`. Nie beide, weil sonst der Sim-Auto-Start doppelt läuft. |
| `docker-compose.yml`, `start.sh` | `G1_GUI` durchreichen und Menüpunkt 2d |

Die Logik bleibt wie beim Streamdeck in den Ziel-Nodes. Die GUI publiziert nur
auf die bestehenden Topics. Es gibt keine neuen Schnittstellen.

## Offene Punkte / nächste Schritte

1. **Echter Zustand statt angenommener Zustand**: `loco_sim` und `loco_client`
   publizieren ihren FSM-Zustand (HOLD/STAND/WALK/DAMP) nicht. Ein Topic
   `/g1pilot/loco_state` würde die Modus-Kacheln *zuverlässig* machen, etwa
   wenn die Sim nach einem Sturz auf DAMP geht.
2. **Pose-Status-Zuordnung**: `pose_store/goto` hat keine Request-ID. Die GUI
   nimmt deshalb den nächsten Endzustand als „ihren“. Mit einer ID wie bei der
   Arm-API wäre das eindeutig.
3. **Touch/Vollbild**: Für ein Tablet am Stand `showFullScreen()` und größere
   Abstände einbauen. Optional als Kiosk ohne Fensterrahmen.
4. **Sprache**: Labels sind Deutsch. Ein EN-Umschalter wäre für Messen sinnvoll.
5. **Bilder statt Text** auf den Bewegungs-Knöpfen, z. B. ein Vorschaubild je
   Pose im Pose-Store.
6. **Navigation mit festen Zielen**: Statt eines Ziels in RViz könnten
   Ziel-Knöpfe (»zum Tisch«, »zur Tür«) direkt auf das Goal-Topic publizieren.

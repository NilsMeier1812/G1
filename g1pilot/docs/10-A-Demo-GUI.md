# Demo-GUI — Anleitung

Richtet sich an alle, die den G1 bedienen oder vorführen (Messe, Besuch,
Lehre). Die Demo-GUI ist die Standard-Oberfläche in Sim und Real.
Aufbau im Code, Topics und offene Punkte: [10-T-Demo-GUI.md](10-T-Demo-GUI.md).

## Ziel

Die frühere Streamdeck-GUI (2026-10-30 entfernt) zeigte alle 25 Funktionen gleichzeitig.
Für Entwicklung ist das richtig, für eine Vorführung zu viel: Besucher sehen
nicht, in welchem Zustand der Roboter ist und welche Knöpfe gerade Sinn
ergeben. Die Demo-GUI reduziert die Bedienung auf **eine Entscheidung zur Zeit**.

## Aufbau

Seit 2026-11-17 ist die Demo-GUI in vier feste Bereiche aufgeteilt. Jede
Funktion hat genau einen Platz, und alles wächst mit der Fenstergröße mit
(1920×1080 bis 1366×768-Laptop, darunter scrollt die Arbeitsfläche).

```
┌──────────────────────────────────────────────────────────────┬───────────────┐
│ G1·REAL [GEHEN][GREIFEN▌AKTIV] [Touch|Controller][PS4][Handy][⛶]│               │
├──────── Linie in Modusfarbe ─────────────────────────────────┤  ( NOT-HALT ) │
│ [ ARME | HÄNDE ]                          [Bewegung stoppen] │ [VOLL-NOTAUS] │
│ ┌ Hand bewegen ─────────────┐ ┌ Abläufe ───────────────────┐ │ STARTLEISTE   │
│ │ Arm [L|R|Beide|Gespiegelt]│ │ [Am Stück|Schritt] [Weiter]│ │ G1: Walk …    │
│ │ Tempo [Langsam|Normal|…]  │ │ (Suche ab 9 Abläufen)      │ │ [Start]       │
│ │ VERSCHIEBEN    DREHEN     │ │ [Ablauf][Ablauf][Ablauf]   │ │ [Damp]        │
│ │  ( ◎ ) ▯      ( ◎ ) ▯     │ │ [Ablauf][Ablauf] …  scrollt│ │ [Preparation] │
│ │ Hüfte 12° links ◄─◎─► [Mitte]│ [Posen & Abläufe bearb. …] │ │ [Walk]        │
│ └───────────────────────────┘ ├ Posen ─────────────────────┤ │ AKKU · TEMP.  │
│                               │ [Sichere Pose][Grundstell.]│ │ [Akku][Max]   │
│                               │ [Pose anfahren][speichern] │ │ [Bein L] …    │
│                               │ ✓ RViz-Marker folgen       │ │               │
│                               └────────────────────────────┘ │               │
├──────────────────────────────────────────────────────────────┤               │
│ ▌Ablauf »Kiste«: Schritt 2/5 …                     [Verlauf] │ [Real beenden]│
└──────────────────────────────────────────────────────────────┴───────────────┘
```

| Bereich | Inhalt |
|---|---|
| **Kopfleiste** (oben) | SIM/REAL-Kennung, Modus-Umschalter GEHEN \| GREIFEN, Ansicht Touch \| Controller, rechts Controller-Belegung, Handy, Vollbild |
| **Arbeitsfläche** (Mitte) | nur der aktive Modus, als Karten nebeneinander (schmales Fenster: untereinander) |
| **Seitenleiste** (rechts) | NOT-HALT und VOLL-NOTAUS (immer oben), real Startleiste + Akku/Temperaturen, Sim die Sim-Werkzeuge, ganz unten Beenden |
| **Statusleiste** (unten) | aktuelle Meldung in Klartext, *Verlauf* zeigt die letzten 200 |

Listen, die wachsen können (Abläufe, Stationen), sind **Raster**: Die
Spaltenzahl folgt der Breite, ab 9 Einträgen erscheint ein Suchfeld, was
nicht passt, scrollt. 20 und mehr Abläufe sind damit kein Problem.

### Kopfleiste: Ansicht Touch | Controller

Rechts in der Kopfleiste schaltet **Touch | Controller** die Arbeitsfläche um.
Es ändert sich nur, was angezeigt wird: Modus, Topics und Sicherheit bleiben
gleich, und alles, was zu sehen ist, lässt sich weiter anklicken.

| Ansicht | GREIFEN | GEHEN |
|---|---|---|
| **Touch** (Start) | Hand bewegen (Joysticks, Hüfte-Schieber), ARME \| HÄNDE, Abläufe, Posen | Manuell gehen (Joystick, Pfeiltasten), Auto-Navigation |
| **Controller** | Karte *Controller* (was der Controller gerade tut, Hand, Stick bewegt, Tempo, Hüfte), *Posen*, *Belegung*; rechts *Pose-Schnellwahl* und *Abläufe* | Karte *Controller* (läuft/steht, Geh-Tempo), *Belegung*, Auto-Navigation |

In der Controller-Ansicht fällt weg, was man ohnehin am Controller macht:
Joysticks, Pfeiltasten, Hüfte-Schieber, die Seite HÄNDE (Kreis/Viereck) und
der Knopf *Belegung* oben (die Belegung steht schon auf der Fläche, gefiltert
auf den aktiven Modus).

Die **Pose-Schnellwahl** zeigt die 16 Plätze (R2 halten + Steuerkreuz 2×) mit
den zugeordneten Posen: Zeile = erste Richtung, Spalte = zweite. Am Controller
leuchtet erst die Zeile (lila Rand), dann der gewählte Platz (grün, leer =
gelb). Ein Klick auf einen Platz fährt die Pose an wie *Pose anfahren*.
*Plätze belegen …* öffnet direkt den Reiter *Controller-Schnellwahl* im
Fenster *Posen & Abläufe*.

Start-Ansicht: `G1_DEMO_VIEW=controller` (Default `touch`).

### Kopfleiste: Modus

Der Modus-Umschalter oben hat zwei Felder, **genau eines** ist aktiv. Eine
Linie in der Farbe des aktiven Modus trennt die Kopfleiste von der
Arbeitsfläche.

| Feld | Bedeutung | Topics |
|---|---|---|
| GEHEN (blau) | Lauf-Regler; Arme eingeklappt (Sim mit AGILE-Policy: Arme frei) | `/g1pilot/start_walking` |
| GREIFEN (orange) | Stand mit geplanten Füßen, Arme frei | `/g1pilot/arms/enabled` + `/g1pilot/start_balancing` |

Jedes Feld hat drei klar unterscheidbare Zustände:

- **aktiv**: Vollfarbe, weißer Rand, Text „AKTIV“
- **wechselt …**: gestrichelter Rand in Modusfarbe. Das gibt es nur beim Wechsel
  zu GEHEN, weil die Arme erst in die Lauf-Pose fahren. Bestätigt wird der Wechsel
  durch `/g1pilot/arms/walk_ready`, spätestens nach 6 s (Timeout, wie in
  `loco_sim`). Mit einer AGILE-Lauf-Policy in der Sim bleiben die Arme frei,
  `walk_ready` kommt sofort.
- **inaktiv**: dunkelgrau, „antippen zum Wechseln“

Auf dem echten Roboter ist der Umschalter gesperrt, bis **Start** gedrückt
wurde. Start ist der erste Knopf der Startleiste in der Seitenleiste
(untereinander: Rückmeldung des loco_client, *Start*, *Damp*, *Preparation*,
*Walk*) und sendet `/g1pilot/start`: gibt den Umschalter frei, quittiert einen NOT-HALT (arm_controller, loco_client,
Controller) und stellt einen G1, der noch in Damp/Zero-Torque liegt, auf
Preparation (FSM 4); steht er schon (Preparation/Walk), schaltet Start nichts um.
Danach ist der Knopf grau (*Gestartet ✓*), nach einem NOT-HALT wieder frei
(*NOT-HALT quittieren*). Der Controller-Zustand steht nicht als eigene
Zeile in der GUI (Modus, Hand und Tempo zeigt die GUI selbst); solange ein
Controller aktiv ist, zeigt der Knopf *Controller* oben rechts die Tastenbelegung.
In der Sim startet die GUI automatisch in GREIFEN und
wiederholt die Anforderung, bis arm_controller und loco_sim sie bestätigt
haben ([40-T-Simulation.md › Auto-Start](40-T-Simulation.md#auto-start-greifen)).

**Startleiste (nur echter G1):** *Damp* und ein *Walk*, der den Walk-Modus
wechselt, brauchen einen zweiten Tipp innerhalb weniger Sekunden („Wirklich
Damp? nochmal tippen“, „Kran gelockert? nochmal tippen“). *Walk* zeigt schon vor
dem Klick die Bodenkontakt-Prüfung (Knie-Last aus `rt/lowstate`): »Walk
Bodenkontakt ok« oder gesperrt mit „kein Bodenkontakt“; das Feld GEHEN zeigt
dann „gesperrt · kein Bodenkontakt“. Die Info-Zeile darüber zeigt den
Onboard-Modus des G1, z. B. „G1: Walk mit Taille (FSM 501)“, und bei einem
abgelehnten Befehl den RPC-Fehlercode. Details:
[50-A-Echter-Roboter.md](50-A-Echter-Roboter.md).

**Mit Controller (PS4/Xbox) gekoppelt:** Modus, Hand-Auswahl, Tempo »Hand
bewegen« und Geh-Tempo teilen sich GUI und Controller
(`/g1pilot/teleop_state`). L2 am Controller → GEHEN, R2 → GREIFEN;
die GUI sendet dabei selbst nichts. Details:
[11-A-Controller.md › Controller und Demo-GUI gekoppelt](11-A-Controller.md#controller-und-demo-gui-gekoppelt).

**Kopfleiste rechts:** *Controller* (nur solange ein Controller aktiv ist,
zeigt die Tastenbelegung), *Handy* (immer da; der Klick startet erst
Bluetooth, dann Koppeln und Steuerhoheit, siehe [12-A-Handy.md](12-A-Handy.md)) und
*Vollbild*.

### Arbeitsfläche

Hier steht **nur** der Inhalt des aktiven Modus. Ohne Modus sagt sie, was als
Nächstes zu tun ist („Zuerst rechts »Start« drücken …“).

**GEHEN**: links die Karte *Manuell gehen*, rechts *Auto-Navigation*.

- **Knopf** (`VirtualJoystick` aus `gui_base.py`): ziehen = laufen,
  loslassen = stehen.
- **Pfeiltasten** (vor, zurück, links, rechts) und die beiden **Dreh-Pfeile** (links/rechts herum, nur Icon): *gedrückt halten* = laufen,
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
  Knopf ist nur aktiv, wenn der Nav-Stack läuft (nur Sim,
  `G1_ENABLE_NAV`); sonst erklärt ein Hinweis, wie man ihn einschaltet.
- **AUTO-NAV-Zustände** (aus `/g1pilot/nav_status` von `nav2point`): Der
  Knopf zeigt *aus* → *AN, wartet auf Ziel* → *läuft …* (gestrichelter Rahmen,
  wie beim Moduswechsel) → *AN, Ziel erreicht* (grün). Nach dem Ankommen bleibt
  AUTO NAV an, das nächste Ziel aus RViz wird direkt angefahren. „Angekommen“
  steht zusätzlich im STATUS-Bereich. Findet der Planer keinen Weg, meldet der
  Status das, und der Roboter bleibt stehen.
- **Stationen** (Quickbefehle, eine Kachel je `station_<Name>` der Szene, als
  Raster mit Suche):
  setzt das Ziel und schaltet AUTO NAV ein. Während des Laufens ist der
  Stations-Knopf gestrichelt umrandet. Beim Ankommen schaltet sich AUTO NAV
  **automatisch wieder aus**, und der Status zeigt „Angekommen an Station …“.
  Ohne Weg (oder ohne Pfad nach 5 s) geht AUTO NAV ebenfalls wieder aus.

**GREIFEN**

Oben links schaltet **ARME | HÄNDE** zwischen zwei Seiten um (beides braucht
viel Platz, darum nie gleichzeitig), oben rechts steht immer **Bewegung
stoppen**.
Beim Wechsel in GREIFEN ist ARME aktiv. **HÄNDE** ist nur mit Inspire-Händen
bedienbar: In der Sim mit Rubber- oder Custom-Händen (starr, `G1_HANDS`) ist der
Knopf ausgegraut, der Tooltip nennt den Grund. Am echten G1 ist er mit
Inspire-Händen frei, mit einem eigenen Profil ebenfalls ausgegraut. Ein eigenes
Profil steht oben neben *G1 · SIM* bzw. *G1 · REAL* (z. B. *G1 · REAL · Hand haken*).

*Seite ARME*: drei Karten, *Hand bewegen* links, *Abläufe* und *Posen* rechts.

- **Abläufe** (seit 2026-10-22, ausführlich in
  [23-A-Posen-und-Ablaeufe.md](23-A-Posen-und-Ablaeufe.md)): ein Ablauf ist eine Liste von **Verweisen auf
  gespeicherte Posen** im Pose-Store (`sequences`, siehe
  [23-T-Posen-und-Ablaeufe.md](23-T-Posen-und-Ablaeufe.md)). Dieselbe
  Pose (z. B. `Sichere_Pose`) kann in mehreren Abläufen und mehrfach in einem
  stehen, ohne Kopie und ohne Umbenennen. Jeder Ablauf mit mindestens einem
  Schritt wird eine Kachel „<Name>“ (Tooltip: die Schritte), der laufende
  Ablauf ist gestrichelt umrandet, die Statuszeile zeigt „Schritt n/m“.
  Jeder Schritt ist ein geplantes POSE ANFAHREN (Kollisionsschutz wie immer);
  reine Hand-Posen melden nach `hand_only_settle_s` (1.8 s) `reached`.
  Beispiel-Abläufe für die Szene A5 installiert
  `python3 -m g1pilot.tools.install_example_sequences` (gelten genau an der
  jeweiligen Station). Fehlt eine Pose eines Ablaufs, startet er nicht und der
  Status nennt sie.
- **Am Stück | Schritt für Schritt** (Umschalter oben in der Karte *Abläufe*):
  *Am Stück* fährt nach jedem `reached` (+ optionaler Pause des Schritts)
  sofort weiter. *Schritt für Schritt* hält nach jedem Schritt an; weiter geht
  es erst mit **Weiter** (GUI), am Handy oder mit **L3** am PS4-/Xbox-
  Controller (Controller vibriert kurz, sobald der Ablauf wartet). Der
  Umschalter wirkt auch während eines Ablaufs ab dem nächsten Schritt; so
  hält man einen Ablauf am Stück an. Start: **real Schritt für Schritt, Sim
  am Stück** (`G1_SEQUENCE_MODE=auto|step`).
- **Abbruch**: `failed`/`rejected`/`cancelled` der laufenden Fahrt, NOT-HALT
  (GUI, Controller PS, Voll-Notaus, Handy), Stopp (Knopf
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
- **Posen & Abläufe bearbeiten …** öffnet das Fenster zum Zusammenstellen (nicht modal,
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
- Karte **Posen**: **Sichere Pose** fährt die gespeicherte Pose `Sichere_Pose` geplant an
  (Arme hoch, Ellbogen hinten; Hinweis, falls sie fehlt; Name über
  `G1_SAFE_POSE` änderbar). **Grundstellung** sendet `/g1pilot/arms/home`: der
  Arm-Controller fährt geplant die **Lauf-Pose** an (Arme neben dem Körper,
  Hände auf Hüfthöhe) (früher HOMING ARMS). Dazu **Pose anfahren …** (Dialog),
  **Pose speichern …** (Dialog, Kategorie `Demo` vorausgewählt; beide in
  `gui_base.py`) und der Schalter **RViz-Marker folgen**. Hände
  öffnen/schließen liegt auf der Seite HÄNDE.
- **Bewegung stoppen** (oben rechts) sendet `/g1pilot/pose_store/cancel`.
- **Hand bewegen**: zwei Joysticks mit Schieber (VERSCHIEBEN, DREHEN), Arm-
  und Tempo-Auswahl wie am Controller. Die Joysticks wachsen mit der Breite.
- **Hüfte** (unten in *Hand bewegen*, neu 2026-11-17): zeigt den Stand
  („Hüfte 12° links“). Der **waagrechte Schieber** dreht den Oberkörper
  (nach links ziehen = links drehen, proportional, loslassen = steht), genau
  wie R2 + rechter Stick am Controller: Rate-Strom auf
  `/g1pilot/waist/yaw_rate` mit ~30 Hz, beim Loslassen einmal 0 (Totmann im
  arm_controller). Nur in GREIFEN/ARME, der arm_controller prüft wie beim
  Controller selbst, ob er die Taille drehen darf. **Hüfte Mitte** fährt sanft
  zur Mitte. Siehe [24-A-Hueftdrehung.md](24-A-Hueftdrehung.md).

*Seite HÄNDE* (`teleoperation/hand_panel.py`) ist die einzige grafische
Handsteuerung (die früheren Browser-GUIs der Hand-Bridge wurden 2026-11-04
entfernt):

- **Ganze Hand**: beide Hände öffnen/schließen sowie je Hand eigene
  Öffnen/Schließen-Knöpfe (`/g1pilot/hand_action/{left,right}`).
- **Finger**: je Finger ein Soll-Schieber (oben = offen), die Ist-Öffnung
  (gelber Balken, %) und die gemessene Kraft in g (grün → orange → rot bei
  Erreichen des Griffkraft-Limits). Ein Ein/Aus-Hauptschalter fehlt bewusst:
  ein Finger-Befehl aktiviert die Hand bei Bedarf selbst.
- **Kraftzonen**: Handskizze je Hand mit den 17 Taktil-Zonen als Heatmap
  (Spitzenwert je Zone, Nulllage = erste Daten).
- **Erweitert** (eingeklappt): Auswahl **Griffkraft** (setzt das Kraft-Limit
  aller Finger: Sanft 300 g, Mittel 800 g, Fest 1500 g) und **Tempo**
  (Langsam/Normal/Schnell), dazu „Kraftzonen nullen“ (aktuelle Taktil-Werte
  als Nulllage). Einzelne Kraft-Limits je Finger gibt es in der Demo-GUI nicht.
- Kommuniziert nur über ROS: `/g1pilot/hand_cmd` (JSON-Befehle, siehe
  [22-T-Inspire-Haende.md](22-T-Inspire-Haende.md)) und `/g1pilot/hand_status` (JSON, ~10 Hz).
  Kommt kein Status, erscheint ein Hinweis (Hand-Bridge läuft nicht).

### Statusleiste

- **Statuszeile** in Klartext mit farbigem Balken, z. B. „Arme werden eingeklappt
  …“, „Arbeitsplatz 1 · Schritt 2/5: …“, „Fertig.“ oder „Abgebrochen (failed) …“.
- **Verlauf** öffnet die letzten 200 Meldungen mit Uhrzeit (neueste oben),
  praktisch, wenn eine Meldung schon überschrieben ist.

### Seitenleiste

- **NOT-HALT** steht immer ganz oben, rund und rot, und scrollt nie weg. Der
  Modus-Umschalter wird danach inaktiv. Was er genau auslöst:
  [51-A-Notaus.md](51-A-Notaus.md).
- **VOLL-NOTAUS** direkt darunter (eckig, dunkelrot): ganzer G1 in Damp, auch
  die Beine, wie R1 + L1 + PS am Controller. **1,5 s gedrückt halten**, der
  Knopf füllt sich dabei. Schon der Druck löst den normalen NOT-HALT aus;
  vorher loslassen bricht nur die Eskalation ab. Danach steht in der
  Startleiste *VOLL-NOTAUS quittieren*; der G1 bleibt in Damp, Aufstehen über
  Preparation → Walk.
- **Startleiste** (nur echter G1), siehe oben bei *Kopfleiste: Modus*.
- **Akku** (nur echter G1, unter der Startleiste): Ladestand in Prozent, darunter
  die Spannung; gelb bis 30 %, rot bis 15 % (`G1_BATT_WARN` / `G1_BATT_CRIT`,
  Annahme). Tooltip: Zustand (SOH), Strom, Zellspannungen, Akku-Temperatur,
  Ladezyklen. Quelle ist das BMS-Topic `rt/lf/bmsstate`, weitergereicht von
  `robot_state` (`/g1pilot/battery`), reiner Lesepfad. »keine Daten«, wenn
  länger als 5 s nichts kam.
- **Temperaturen** (nur echter G1, neben dem Akku): heißester
  Motor und je eine Kachel für Bein L/R, Taille, Arm L/R mit dem Höchstwert,
  gelb ab 60 °C, rot ab 75 °C (`G1_TEMP_WARN` / `G1_TEMP_CRIT`). Klick öffnet
  die Tabelle aller 29 Motoren (beide Sensoren, Spannung) plus IMU. Quelle ist
  `robot_state` (`/g1pilot/temperatures`), reiner Lesepfad.
- **Simulation** (nur Sim): **Roboter zurücksetzen**, **Roboter schubsen**,
  **Szene zurücksetzen** untereinander.
- **Szene zurücksetzen** (nur Sim): Objekte der Umgebung zurück an den Start.
- **Roboter zurücksetzen** (nur Sim): Roboter nach NOT-HALT oder Sturz zurück
  an den Startpunkt und gehalten (HOLD); beim nächsten Modus-Klick steht er
  sanft auf.
- **Roboter schubsen** (nur Sim): Störtest. Kurz
  tippen = Standard-Schubs (150 N); **gedrückt halten lädt auf** (Anzeige im
  Knopf, bis 2000 N nach 3 s), Loslassen schubst. Er zeigt, dass sich der
  Roboter in beiden Modi fängt. Einstellbar mit `G1_PUSH_MAX_N`,
  `G1_PUSH_CHARGE_S`, `G1_PUSH_TAP_S`.
- **Sim beenden** (nur Sim, ganz unten): schließt den ganzen Sim-Stack (MuJoCo, RViz,
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
  `docker/real_shutdown_watcher.sh` (von `./start.sh` und `make real`
  gestartet) stoppt den Stack **sanft** mit
  `docker compose stop` (SIGTERM) + `down`. Die Beine bleiben im
  Onboard-Zustand, es gibt kein Damp: Den Roboter danach per Fernbedienung
  ablegen. Kommt vom Arm-Regler 20 s lang keine Quittung, wird trotzdem
  beendet. Er gibt die Arme beim SIGTERM selbst ab. NOT-HALT bricht
  „Real beenden“ ab.

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

**Fenstergröße / Vollbild:** Die Demo-GUI startet maximiert. Arbeitsfläche und
Seitenleiste liegen in Scrollbereichen, damit die Mindestgröße des Fensters
klein bleibt (unter 700 px hoch; 2026-10 lag sie bei ~1160 px, höher als ein
1080er-Schirm: der Fenstermanager bot dann kein Maximieren an und auch das
Vollbild blieb größer als der Schirm). Ab 1366×768 passt GREIFEN/ARME ohne
Scrollen, bei schmalerem Fenster stehen die Karten untereinander. Knopf
*Vollbild* ganz rechts in der Kopfleiste, im Vollbild heißt er *Vollbild beenden*. Tasten: **F11**
schaltet um, **Esc** verlässt das Vollbild. Funktioniert auch ohne Titelleiste
des Fenstermanagers (Roboter-Rechner). `G1_GUI_FULLSCREEN=1` startet die
Demo-GUI direkt im Vollbild (Default 0 = maximiert). Reine Fensterfunktion,
sendet nichts an den Roboter.

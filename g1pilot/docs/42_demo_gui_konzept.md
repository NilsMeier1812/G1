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
│  GREIFEN:                                [▶ Ganze Demo abspielen]│
│   [ Winken ] [ Zeigen ] [ Box greifen ] [ Ablegen ]              │
│   [Hände öffnen] [Hände schließen] [⌂ Grundstellung] [■ Stop]│
│   Erweitert ▸   (Pose anfahren … / Pose speichern … / Marker)    │
│                                                                  │
│  GEHEN:                                                          │
│     ( Knopf )        [⟲] [▲] [⟳]          Tempo                  │
│      ziehen          [◀]     [▶]          [Langsam]              │
│                          [▼]              [ Normal]              │
├──────────────────────────────────────────────────────────────────┤
│ 3 · STATUS                                                       │
│ ▌Fährt: Winken …             [↺ Szene] [➜ Schubsen]  ( NOT-HALT ) │
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

Auf dem echten Roboter sind die Kacheln gesperrt, bis „⏻ Roboter starten“
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

**GREIFEN**

- **Beispielbewegungen**: ein Knopf je Pose aus der Pose-Store-Kategorie
  `Demo` (änderbar über `G1_DEMO_CATEGORY`), maximal 8, sortiert nach Namen. Das
  Präfix `Demo_<n>_` wird ausgeblendet, aus `Demo_2_Winken` wird „Winken“. Diese
  Konvention gilt auch für `demo_sequence.py`.
  Gibt es die Kategorie noch nicht, erscheinen als Fallback alle Posen mit einem
  Hinweis.
- **▶ Ganze Demo abspielen** fährt alle Demo-Posen nacheinander an. Der nächste
  Schritt startet erst, wenn `/g1pilot/arm_command/status` `reached` meldet.
  Bei `failed`/`rejected`/`cancelled` bricht die Folge ab und der Grund steht im
  Status.
- **Hände öffnen/schließen** (beide Hände gleichzeitig), **Grundstellung**
  (`/g1pilot/arms/home`) und **Bewegung stoppen** (`/g1pilot/pose_store/cancel`).
- **Erweitert ▸** (eingeklappt) enthält die bisherigen Einzelfunktionen für den
  Betreuer: Pose anfahren (Dialog), Pose speichern (Dialog, Kategorie `Demo`
  vorausgewählt) und Marker folgen. Die Dialoge werden aus `ui_interface.py`
  wiederverwendet.

### 3 · Status (unten)

- **Statuszeile** in Klartext mit farbigem Balken, z. B. „Arme werden eingeklappt
  …“, „Fährt: Winken …“, „Fertig ✓“ oder „Abgebrochen (failed) …“.
- **NOT-HALT** ist immer sichtbar, rund und rot. Er sendet dasselbe wie der
  Streamdeck. Beide Kacheln werden danach inaktiv.
- **↺ Szene zurücksetzen** (nur Sim).
- **➜ Roboter schubsen** (nur Sim): Störtest wie PUSH ROBOT im Streamdeck
  (400-ms-Impuls auf `/g1pilot/push`). Er zeigt, dass sich der Roboter in beiden
  Modi fängt.

## Bedienen

```bash
./start.sh   # grafisches Startmenü → Simulation → Schalter „Demo-Oberflaeche statt Streamdeck“
./start.sh --menu         # Text-Menü: 2d) Bedienoberfläche → Demo
```

Demo-Posen anlegen: GREIFEN → Erweitert → *Pose speichern …*, Name
`Demo_1_Winken`, `Demo_2_Zeigen`, …, Kategorie `Demo`.

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

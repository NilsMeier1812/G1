# Posen und Abläufe — Anleitung

Richtet sich an alle, die Armstellungen speichern und als Vorführung
abfahren wollen. Technik (Dateiformat, Runner, Topics):
[23-T-Posen-und-Ablaeufe.md](23-T-Posen-und-Ablaeufe.md).

## Überblick

| Begriff | Bedeutung |
|---|---|
| **Pose** | Gespeicherte Stellung aus linkem Arm, rechtem Arm, linker Hand, rechter Hand (beliebige Kombination) unter einem Namen, einsortiert in eine **Kategorie** |
| **Relative Bewegung** | Verschiebung der Hände ab dort, wo sie gerade sind, z. B. „Box anheben“ = 5 cm hoch |
| **Ablauf** | Liste von Schritten; jeder Schritt verweist auf eine Pose oder eine relative Bewegung. Dieselbe Pose kann in mehreren Abläufen und mehrfach in einem stehen |

Jeder Schritt wird **geplant** angefahren (Bahn um Tisch, Körper und
Hindernisse herum, siehe [20-A-Arm-Manipulation.md](20-A-Arm-Manipulation.md)).
Alles liegt in einer Datei (`data/arm_poses.json`), die arm_controller,
Demo-GUI, Streamdeck und Handy gemeinsam nutzen.

## Pose speichern

Gespeichert wird immer die zuletzt **kommandierte** Stellung, nicht die
Messung.

| Wo | Wie |
|---|---|
| Demo-GUI | GREIFEN → Erweitert ▸ → **Pose speichern …** (Kategorie `Demo` vorausgewählt, `G1_DEMO_CATEGORY`), oder im Fenster **Posen & Abläufe** „Aktuelle Stellung speichern“ |
| Handy | **Pose speichern …** (Name, Kategorie, Komponenten, optional *An Ablauf anhängen*), siehe [12-A-Handy.md](12-A-Handy.md#posen) |
| Streamdeck | **POSE SPEICHERN** |
| Kommandozeile | `ros2 topic pub --once /g1pilot/pose_store/save …`, siehe [20-A-Arm-Manipulation.md](20-A-Arm-Manipulation.md) |
| Arm-API | `POST /arm/save`, siehe [21-A-Arm-API.md](21-A-Arm-API.md) |

Hände werden nur gespeichert, wenn die Inspire-Bridge läuft. Das Speichern
gilt erst, wenn der arm_controller es quittiert hat; erst dann hängt der
Dialog die Pose an einen gewählten Ablauf an.

## Pose anfahren

Demo-GUI: GREIFEN → Erweitert ▸ → **Pose anfahren …** oder im Fenster
Posen & Abläufe *Anfahren*. Handy: **Pose anfahren …** (mit Schieber).
Streamdeck: **POSE ANFAHREN**. Die feste Pose **Sichere Pose** (Arme hoch,
Ellbogen hinten) hat in Demo-GUI und Handy einen eigenen Knopf; ihr Name ist
`Sichere_Pose` (`G1_SAFE_POSE`).

Abbrechen: **Bewegung stoppen** (Demo-GUI, Handy), **POSE ABBRECHEN**
(Streamdeck), **Kreuz** am Controller. Ein Marker-Griff oder Stick-Ausschlag
bricht ebenfalls ab (manueller Vorrang).

## Abläufe zusammenstellen (Demo-GUI)

GREIFEN → **Posen & Abläufe …** öffnet das Fenster. Es ist nicht modal, man
kann nebenher die Hand bewegen.

- **Links:** Abläufe — neu, umbenennen, kopieren, löschen, starten.
- **Mitte:** Schritte des gewählten Ablaufs — ziehen oder ▲/▼, entfernen,
  Pause nach dem Schritt (nur für „Am Stück“, 0–600 s), und
  **„＋ Aktuelle Stellung als Schritt …“** (speichert die Pose und hängt sie
  an, sobald der arm_controller das Speichern quittiert hat).
- **Rechts:** alle Posen nach Kategorie — Doppelklick oder „← Hinzufügen“
  hängt hinter dem markierten Schritt an; Anfahren, Löschen, aktuelle
  Stellung speichern. Darüber die **relativen Bewegungen**: *Neue relative
  Bewegung …* / *Bearbeiten …* mit Name, vor / links / hoch in cm (je
  höchstens ±30 cm, Bezug Oberkörper) und welche Hand.

Jede Änderung wird sofort gespeichert. Jeder Ablauf mit mindestens einem
Schritt erscheint in GREIFEN als eigener Knopf.

**Beispiel „Box greifen, anheben, ablegen“:** Pose *Greifen*, dann relative
Bewegung *Box anheben* (hoch 5 cm), dann die Ablage-Pose.

**Alte Arbeitsweise:** Eine Pose in die Kategorie `Ablauf <Name>` speichern
hängt sie hinten an den Ablauf `<Name>` an. Bestehende Kategorien
`Ablauf …` werden beim ersten Speichern als Abläufe übernommen (Reihenfolge
nach Namen).

**Beispiel-Abläufe** für die Sim-Szene A5 (Arbeitsplatz 1 und 2, gelten genau
an der jeweiligen Station):

```bash
python3 -m g1pilot.tools.install_example_sequences
```

## Ablauf abfahren

| Modus | Verhalten |
|---|---|
| **Am Stück** | nach jedem erreichten Schritt (+ Pause des Schritts) sofort weiter |
| **Schritt für Schritt** | hält nach jedem Schritt an, weiter mit **Weiter ▶** (Demo-GUI), am Handy oder mit **L3** am Controller |

Start: **real Schritt für Schritt, Sim am Stück** (`G1_SEQUENCE_MODE=auto|step`).
Der Umschalter gilt für Demo-GUI, Handy und Controller gemeinsam und wirkt
auch während eines Ablaufs ab dem nächsten Schritt; so hält man einen Ablauf
„am Stück“ an.

- Wartet ein Ablauf, vibriert der Controller zweimal kurz. Man darf die Hand
  dann mit Jog oder Controller nachkorrigieren; der nächste Schritt wird von
  dort geplant.
- Solange ein Schritt fährt, sind die Controller-Sticks gesperrt (Anzeige
  „ABLAUF fährt (Sticks gesperrt, Kreuz = Stopp)“), damit ein versehentlich
  bewegter Stick die Fahrt nicht mit „manueller Vorrang“ abbricht.
- Reine Hand-Schritte (nur Finger) gelten nach 1,8 s als erreicht.
- Die Statuszeile zeigt „<Ablauf> · Schritt n/m: …“.

**Abbruch** — danach geht es nie von selbst weiter, auch nicht nach dem
Quittieren:

- NOT-HALT (GUI, Controller-PS, Voll-Notaus, Handy, Streamdeck),
- Stopp (Bewegung stoppen, Controller-Kreuz, POSE ABBRECHEN),
- Moduswechsel, „Hand bewegen“ während einer Fahrt,
- Fahrt endet mit `failed` / `rejected` / `cancelled`,
- 4 s keine Antwort vom arm_controller (Arme nicht übernommen),
- Handy-Verbindung weg (nur bei einem vom Handy gestarteten Ablauf).

Fehlt eine Pose eines Ablaufs, startet er nicht und der Status nennt sie.

## Posen aufräumen ohne GUI

`pose_tool.py` (Paketwurzel, nur Standardbibliothek, läuft auf dem Host)
arbeitet direkt auf der Datei. Der arm_controller liest sie bei jedem
Zugriff neu, Änderungen wirken ohne Neustart.

```bash
python3 pose_tool.py list                      # alles anzeigen
python3 pose_tool.py list --category Demo
python3 pose_tool.py delete Demo_5_switch
python3 pose_tool.py rename Demo_7_switch Demo_5_switch
python3 pose_tool.py move Left_up --category Demo
```

`demo_sequence.py` fährt alle Posen einer Kategorie in Namensreihenfolge
über die Arm-API ab (älterer Weg ohne Demo-GUI):

```bash
python3 demo_sequence.py --category Demo --loop 3 --pause 2
python3 demo_sequence.py --dry-run
```

## Fehlerbehebung

| Symptom | Ursache / Abhilfe |
|---|---|
| Ablauf-Knopf fehlt | Ablauf hat noch keinen Schritt. |
| „Pose … fehlt“ beim Start | Pose wurde gelöscht oder umbenannt; im Fenster Posen & Abläufe Schritt entfernen oder neu hinzufügen. |
| Ablauf bricht nach 4 s ab | Arme nicht übernommen → GREIFEN bzw. R2. |
| Schritt meldet `goal_in_collision` / `no_path_found` | Roboter steht anders als beim Speichern; näher an die Station oder Pose neu speichern. |
| Relative Bewegung bricht sofort ab | Hand-Pose unbekannt (arm_controller noch nicht kalibriert oder aus). |
| Gespeicherte Posen nach Neustart weg | Container ohne Bind-Mount des Repos, siehe `G1_POSE_STORE` in [23-T-Posen-und-Ablaeufe.md](23-T-Posen-und-Ablaeufe.md#speicherort). |

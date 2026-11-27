# Eigene Hände (Simulation und echter Roboter) — Anleitung

Richtet sich an: alle, die statt der Inspire- oder Rubber-Hand ein eigenes,
starres Werkzeug am G1 nutzen wollen, z. B. die gedruckten Box-Haken.
Wie es intern funktioniert: [25-T-Eigene-Haende.md](25-T-Eigene-Haende.md).

Gilt für die Simulation **und** (seit 2026-11-27) den echten Roboter, siehe
[Am echten Roboter](#am-echten-roboter).

## Hände auswählen

- **Start-GUI:** Simulation → Abschnitt **Hände** → *Rubber Hände*, *Inspire Hände*
  oder *Custom Hände*. Bei *Custom Hände* erscheint darunter die Liste aller
  eigenen Profile (Ordner unter `hands/`, z. B. *Box-Haken (gedruckt)*) und der
  Knopf **Bearbeiten …**, der das Bearbeitungstool für das gewählte Profil öffnet
  (siehe [Hand einpassen](#hand-einpassen-schieber-live)).
- **Startmenü im Terminal** (`./start.sh`): Frage *3) Welche Hände?* (Rubber /
  Inspire / Custom), bei Custom danach *3a)* welches Profil und *3b)* ob vorher
  das Bearbeitungstool aufgehen soll.
- **Direkt:** `G1_HANDS=haken ./start.sh --yes`

Mit einem eigenen Profil gibt es keine Finger und keine Hand-Bridge (wie bei
der Rubber-Hand). Arme, Jog, Posen und der Planer funktionieren normal; sie
zielen auf den Greifpunkt (TCP) des Profils, bei den Haken auf die Innenecke
der Lippe.

## Am echten Roboter

Sind am echten G1 eigene Hände montiert, muss das Startmenü das wissen: Armplaner
(Kugelmodell), Selbstkollisions-Gate, IK-Greifpunkt, Schwerkraft-Ausgleich der
Arme und die Hand in RViz rechnen alle mit dem gewählten Profil. Mit dem
Inspire-Modell und montierten Haken stimmen Kollisionsmodell, TCP und Masse
(Inspire 0,88 kg je Hand, Haken ca. 0,15 kg) nicht.

- **Start-GUI:** Echter Roboter → **Montierte Hände** → *Inspire* oder *Custom*
  (Rubber gibt es real nicht). Bei *Custom* dieselbe Liste wie in der Sim, mit
  **Bearbeiten …** und **➕ Neue Hand anlegen …**. Der Schalter *Inspire-Hände
  ansteuern* ist dann aus (starre Hand, keine Finger, keine Modbus-Bridge).
- **Startmenü im Terminal:** *R2) Welche Hände sind am G1 montiert?*, bei Custom
  *R2a)* Profil und *R2b)* Bearbeitungstool. *R3)* (Inspire-Bridge) kommt nur
  bei Inspire.
- **Direkt:** `G1_MODE=real G1_HANDS=haken ROBOT_INTERFACE=… G1_REAL_CONFIRM=1 ./start.sh --yes`

**Kein Rückfall:** Fehlt das Profil, ist der Name ungültig oder lässt sich das
Modell nicht erzeugen, bricht der Start ab (`start.sh` auf dem Host, zusätzlich
`hand_profile.urdf_path()` in den Nodes). In der Sim wird wie bisher mit der
Rubber-Hand weitergemacht.

Vor dem ersten Einsatz real:

1. Hand **wiegen** und `mass_kg` in `hand.yaml` eintragen (danach neu erzeugen,
   macht das Startmenü selbst). Die Masse geht in den Schwerkraft-Ausgleich ein.
2. In der Sim mit demselben Profil prüfen, ob Montage und TCP stimmen
   (Bearbeitungstool).
3. Real erst langsam joggen und in RViz kontrollieren, dass die gezeigte Hand
   zur echten passt. In der Demo-GUI steht das Profil oben neben *G1 · REAL*;
   **HÄNDE** ist ausgegraut.

Beide Seiten haben immer dasselbe Profil (links gespiegelt); eine Inspire- und
eine eigene Hand gemischt geht nicht.

## Eine neue Hand anlegen

Am einfachsten in der **Start-GUI**: Simulation → **Hände** → *Custom Hände* →
in der Liste **➕ Neue Hand anlegen …** wählen → Knopf **Anlegen …**. Im
Terminal-Startmenü ist es der letzte Punkt bei *3a)*, direkt:
`unitree_mujoco/unitree_robots/g1/tools/hand_tool.sh new [datei]`.

1. **Datei wählen** (STL, OBJ, PLY, OFF, GLB oder 3MF) und einen Namen geben. Der
   Ordner `hands/<name>/` wird aus dem Namen gebildet (steht daneben).
2. Das Fenster **rät die Montage** und zeigt bis zu vier **Vorschläge** als Bild:
   links beide Hände von vorn, rechts die rechte Hand von nah. Blaue Scheibe =
   Flansch am Handgelenk, grüner Punkt = Greifpunkt (TCP), der Unterarm ist
   durchsichtig. Darunter steht, woran der Vorschlag es festmacht.
   - **Einheit** (mm/m) wird aus der Größe geraten, **Masse** aus dem Volumen
     (PLA, 50 % Füllung). Beides kann man ändern, die Masse bitte nachwiegen.
   - **Linke Hand gespiegelt**: an, wenn eine Datei für beide Seiten gilt.
3. Vorschlag anklicken → **Anlegen und Bearbeitungstool öffnen**. Das legt
   `hands/<name>/` mit der Datei (andere Formate als STL) und einer kommentierten
   `hand.yaml` an, erzeugt das Modell (MuJoCo + URDF, Kollisions-Zerlegung) und
   öffnet gleich das [Bearbeitungstool](#hand-einpassen-schieber-live) zum
   Feinjustieren.
4. Die neuen Dateien committen (Liste in [25-T](25-T-Eigene-Haende.md#was-erzeugt-wird)).
   Vergisst man das, erzeugt `start.sh` sie beim nächsten Start selbst neu.

Wie die Erkennung rät (Kurzfassung, Details in
[25-T](25-T-Eigene-Haende.md#montage-erkennung)): Das Werkzeug ragt entlang
seiner **längsten Achse** aus dem Arm. Arm-Seite ist das **schmalere Ende**; wird
der Querschnitt dahinter deutlich anders (Steckplatte → Körper), steckt die
Platte im Arm und der Flansch liegt am Absatz. **Handfläche** ist die Seite, zu
der die Spitze herausragt (Lippe, Kralle), sonst die flache Seite. Bei den
Box-Haken trifft Vorschlag 2 die eingepasste Montage genau.

Ohne Fenster (z. B. per SSH):

```bash
cd unitree_mujoco/unitree_robots/g1/tools
python3 hand_detect.py meine_hand.stl                 # nur Vorschläge anzeigen
python3 hand_new.py --create meine_hand.stl --name "Meine Hand" --pick 2
```

Von Hand geht es weiterhin: Ordner `hands/<name>/` (nur a–z, 0–9, `_`) mit STL
und einer `hand.yaml` (Vorlage `hands/haken/hand.yaml`, Felder siehe unten),
dann `./hand_tool.sh build <name>`. Fehlt das Werkzeug-Python, einmal
`unitree_mujoco/scene_editor/setup.sh` ausführen (installiert trimesh, vhacdx,
mujoco und PyQt6 für die Fenster). Ein älteres venv ohne PyQt6 ergänzt
`hand_tool.sh` beim ersten Öffnen eines Fensters selbst (`pip install PyQt6`).

## Hand einpassen (Schieber, live)

```bash
cd unitree_mujoco/unitree_robots/g1/tools
./hand_tool.sh adjust haken
```

Öffnet den G1 im MuJoCo-Viewer (Kamera auf der rechten Hand) und daneben ein
Fenster mit sechs Schiebern. Jede Änderung wirkt sofort auf beide Hände,
links gespiegelt:

| Schieber | Wirkung |
|---|---|
| Versatz x | Hand weiter aus dem Arm heraus (+) oder tiefer hinein (−), mm |
| Versatz y | Richtung Handfläche (zur Kiste), mm |
| Versatz z | quer dazu, mm |
| Drehung um x / y / z | um die Flansch-Mitte, Grad |

- **Unterarm durchsichtig** (Standard an): man sieht, wie tief ein Adapter im
  Arm steckt.
- **Kollisions-Teile zeigen**: die konvexen Teile, mit denen MuJoCo rechnet.
- **Speichern** schreibt das Ergebnis in `hand.yaml` (`mount.offset_mm`,
  `mount.rotate_deg`), erzeugt das Modell neu (MuJoCo und URDF) und setzt die
  Schieber auf 0. Das Fenster zeigt laufend, was gespeichert würde.

Danach die geänderten Dateien committen (siehe unten).
Beide Fenster (Neue Hand anlegen, Hand einpassen) sind PyQt6 im Aussehen des
Startmenüs. Der MuJoCo-Viewer läuft als eigener Prozess daneben. Braucht ein
Python mit `PyQt6`, `mujoco` und `trimesh`; das Szenen-Editor-venv bringt alles
mit.

## Wo sitzt die Hand? (Adapter-Punkt)

Alle Hände sitzen am **Flansch** des Handgelenks: der Stirnfläche des letzten
Handgelenk-Glieds (`*_wrist_yaw_link`), dort wo Unitree die Rubber- und die
Inspire-Hand anschraubt. Der Flansch-Frame hat seinen Ursprung in der Mitte
dieser Fläche, **x zeigt vom Flansch weg** (Verlängerung des Unterarms).

### Weg 1 (empfohlen): im CAD gegen den Flansch konstruieren

Im Ordner `hands/` liegen `flansch_referenz_left.stl` und
`flansch_referenz_right.stl`: das Handgelenk-Glied in Millimetern, schon im
Flansch-Frame (Ursprung = Flansch-Mitte, x = vom Flansch weg).

1. Die Referenz ins CAD laden und das Werkzeug mit Verknüpfungen
   (Bohrbild, Anlagefläche) an den Flansch setzen, so wie es real montiert ist.
2. Das Werkzeug **im Koordinatensystem der Referenz** als STL exportieren.
3. In `hand.yaml` dann einfach:

   ```yaml
   unit: mm
   mount:
     origin: [0, 0, 0]
     out: [1, 0, 0]
     palm: [0, -1, 0]     # bei der linken Referenz; bei der rechten [0, 1, 0]
   ```

   und bei `mirror` die Seite, die du nicht konstruiert hast, auf `true`.

Dann muss nichts geschätzt werden.

### Weg 2: vorhandene STL einpassen

Kommt die STL mit beliebigem Ursprung aus dem CAD (wie die ersten Haken),
sagt man in `hand.yaml`, welcher Punkt am Flansch liegt und wohin die Teile
zeigen:

| Feld | Bedeutung |
|---|---|
| `mount.origin` | Punkt der STL, der auf der Flansch-Mitte liegt |
| `mount.out` | Richtung in der STL, die vom Flansch weg zeigt |
| `mount.palm` | Richtung in der STL zur Handfläche (dahin, wo die Inspire-Finger schließen) |
| `mount.offset_mm` | Feinkorrektur in mm, im Hand-Frame (x = out, y = palm, z = quer) |
| `mount.rotate_deg` | Feinkorrektur in Grad um dieselben Achsen |

Zur Kontrolle schreibt der Generator `hands/<name>/generated/flansch_in_stl_left.stl`
(und `_right`): das Handgelenk-Glied in **deinen** STL-Koordinaten. Lädt man das
zusammen mit der eigenen STL ins CAD oder in den Slicer, sieht man sofort, ob
der Flansch an der richtigen Stelle sitzt.

### Übrige Felder

| Feld | Bedeutung |
|---|---|
| `name` | Anzeige im Startmenü und in der Start-GUI |
| `mesh`, `unit` | STL im Profilordner, Einheit `mm` oder `m` |
| `mass_kg` | Masse **je Hand** inkl. Adapter und Schrauben (bitte wiegen) |
| `color` | Farbe in der Sim (RGBA) |
| `mirror.left`, `mirror.right` | Seite spiegeln. Eine STL für beide Seiten: eine Seite `true` |
| `tcp` | Greifpunkt in STL-Koordinaten; darauf zielen IK, Jog, Posen, Planer |
| `collision.max_hulls` | Anzahl konvexer Teile für die Kollision (mehr = genauer, langsamer) |

## Prüfen

- In der Vorschau: blaue Achsen = Flansch, grüne Achsen = TCP. Taste **3**
  blendet die Kollisions-Teile ein.
- `./hand_tool.sh snapshot <name>` speichert drei Ansichten als Bild.
- Test: `g1pilot/test/run_tests.sh sim` (enthält `test_hand_profile.py` und `test_hand_new.py`).

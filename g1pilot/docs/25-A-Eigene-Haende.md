# Eigene Hände in der Simulation — Anleitung

Richtet sich an: alle, die statt der Inspire- oder Rubber-Hand ein eigenes,
starres Werkzeug am G1 simulieren wollen, z. B. die gedruckten Box-Haken.
Wie es intern funktioniert: [25-T-Eigene-Haende.md](25-T-Eigene-Haende.md).

Gilt **nur für die Simulation**. Am echten Roboter bleibt alles wie bisher
(Inspire-URDF für IK und Planer).

## Hände auswählen

- **Start-GUI:** Simulation → Abschnitt **Hände** → *Inspire*, *Rubber* oder ein
  eigenes Profil (z. B. *Box-Haken (gedruckt)*).
- **Startmenü im Terminal** (`./start.sh`): Frage *3) Welche Hände?*
- **Direkt:** `G1_HANDS=haken ./start.sh --yes`

Mit einem eigenen Profil gibt es keine Finger und keine Hand-Bridge (wie bei
der Rubber-Hand). Arme, Jog, Posen und der Planer funktionieren normal; sie
zielen auf den Greifpunkt (TCP) des Profils, bei den Haken auf die Innenecke
der Lippe.

## Eine neue Hand anlegen

1. Ordner `unitree_mujoco/unitree_robots/g1/hands/<name>/` anlegen (`<name>`
   nur aus a–z, 0–9 und `_`) und die STL hineinlegen.
2. `hand.yaml` aus `hands/haken/` kopieren und anpassen (Felder siehe unten).
3. Modell erzeugen und ansehen:

   ```bash
   cd unitree_mujoco/unitree_robots/g1/tools
   ./hand_tool.sh preview <name>
   ```

   Das öffnet den G1 schwebend im MuJoCo-Viewer. Jedes Speichern von
   `hand.yaml` oder der STL erzeugt das Modell neu und lädt die Ansicht neu.
   Fehlt das Werkzeug-Python, einmal `unitree_mujoco/scene_editor/setup.sh`
   ausführen (installiert trimesh, vhacdx, mujoco).
4. Die erzeugten Dateien committen (Liste in [25-T](25-T-Eigene-Haende.md#was-erzeugt-wird)).
   Vergisst man das, erzeugt `start.sh` sie beim nächsten Start selbst neu.

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
- Test: `g1pilot/test/run_tests.sh sim` (enthält `test_hand_profile.py`).

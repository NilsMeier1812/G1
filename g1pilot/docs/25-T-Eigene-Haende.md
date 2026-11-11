# Eigene Hände in der Simulation — Technik

Bedienung: [25-A-Eigene-Haende.md](25-A-Eigene-Haende.md).

## Idee

Ein **Hand-Profil** ist ein Ordner `unitree_mujoco/unitree_robots/g1/hands/<name>/`
mit `hand.yaml` und einer STL. `tools/gen_hand_profile.py` macht daraus ein
MuJoCo-Modell und ein URDF, die beide dieselbe Hand am selben Ort haben. Alles
Erzeugte wird committet; zur Laufzeit braucht niemand trimesh.

Die Auswahl läuft über `G1_HANDS` (`inspire`, `rubber` oder Profilname). Ohne
`G1_HANDS` gilt wie bisher `G1_INSPIRE_HANDS`. Eigene Profile wirken nur bei
`G1_SIM_MODE=true`; der Real-Service in `docker-compose.yml` bekommt `G1_HANDS`
gar nicht. Zentrale Regel: `g1pilot/utils/hand_profile.py` (gleiche Regel ohne
g1pilot-Import in `simulate_python/config.py` und `scene_editor/build_env_scene.py`).

## Frames

| Frame | Lage |
|---|---|
| `*_wrist_yaw_link` | letztes Handgelenk-Glied (MuJoCo-Body bzw. URDF-Link) |
| Flansch (`*_hand_flange`) | `wrist_yaw_link` + (0,0415 m, 0, 0), Achsen wie `wrist_yaw_link`. Dort endet das Glied-Mesh; Rubber-Hand (`pos 0.0415 0.003 0`) und Inspire (`*_base_joint xyz 0.0415 0 0`) sitzen hier |
| Hand-Frame | x = vom Flansch weg, y = Handfläche (links −y, rechts +y des `wrist_yaw_link`), z = x × y. Versatz und Drehung aus `hand.yaml` wirken hier, darum sind beide Seiten automatisch spiegelgleich |
| TCP (`*_hand_point_contact`) | Punkt `tcp` aus `hand.yaml`, Orientierung = `wrist_yaw_link` (wie bisher). Rubber/Inspire: (0,0915, ±0,003, 0) |

Abbildung je Seite: `p_wrist = A · p_stl + b` mit
`A = H · R(rotate_deg) · Qᵀ · M · scale`, `b = Flansch + H · offset − A · origin`.
`Q` = Spalten (out, palm, out × palm) in STL-Koordinaten, `H` = Hand-Frame im
`wrist_yaw_link`, `M` = Spiegelung an der Ebene aus out und palm (falls
`mirror`). Bei einer Spiegelung werden die Dreiecke umgedreht, damit die
Normalen nach außen zeigen.

## Was erzeugt wird

| Datei | Zweck |
|---|---|
| `unitree_robots/g1/g1_29dof_hand_<name>.xml` | MuJoCo-Roboter: `g1_29dof.xml` ohne Rubber-Hand, je Seite Body `*_hand_tool_link` unter `*_wrist_yaw_link` mit Masse/Trägheit, Optik-Geom, Kollisions-Teilen und Sites `*_hand_flange`, `*_hand_tcp` |
| `unitree_robots/g1/scene_hand_<name>.xml` | Szene wie `scene.xml`, nur mit obigem Roboter |
| `unitree_robots/g1/hands/<name>/generated/*.stl` | Meshes im `wrist_yaw_link`-Frame (m), dazu `flansch_in_stl_*.stl` (Kontrolle im CAD) und `info.json` (Fingerabdruck der Eingaben) |
| `g1pilot/description_files/urdf/g1_29dof_hand_<name>.urdf` | `g1_29dof_inspire_ftp.urdf` ohne Inspire-Baum (alles unter `*_base_joint`), dafür `*_hand_tool_link` + `*_hand_flange`, TCP verschoben |
| `g1pilot/description_files/meshes/hand_<name>_*.STL` | dieselben Meshes fürs URDF (flach, weil `setup.py` den Mesh-Ordner flach installiert) |
| `unitree_robots/g1/hands/flansch_referenz_{left,right}.stl` | Handgelenk-Glied im Flansch-Frame, mm (CAD-Referenz) |

`gen_hand_profile.py --check <name>` vergleicht den Fingerabdruck (hand.yaml,
STL, Generator) mit `info.json`. `start.sh` ruft das beim Start auf und erzeugt
bei Bedarf neu; scheitert das, startet die Sim mit der Rubber-Hand.

## Kollision

MuJoCo kollidiert ein Mesh nur über seine konvexe Hülle; ein Haken wäre damit
ein voller Klotz. Darum zerlegt der Generator die STL mit V-HACD
(`trimesh` + `vhacdx`, wie `scene_editor/build_env_scene.py`) in bis zu
`collision.max_hulls` konvexe Teile. Die Teile (in STL-Koordinaten) liegen committet in
`hands/<name>/generated/hulls/`; nach einer reinen Lage-Änderung braucht das
Neu-Erzeugen darum nur `trimesh`, kein `vhacdx`.

| Wo | Was |
|---|---|
| MuJoCo | Teile mit `contype = conaffinity = 2` (Gruppe 3): wie die Inspire-Finger nur gegen greifbare Objekte und Umgebung, nicht gegen Boden/Körper. Optik-Geom ohne Kollision |
| IK-Kollisions-Gate (`ik_solver.py`) | die Teile sind Collision-Geometrien von `*_hand_tool_link`; `_GATE_ARM_KEYS` enthält dafür `hand_tool` -> Paare gegen Torso/Beine und gegen den anderen Arm |
| Armplaner (`arm_geometry.py`) | Kugelmodell wird aus denselben Teilen gefittet (Glied `wrist_yaw`), TCP aus dem URDF |

Die V-HACD-Teile haben nur wenige, weit auseinanderliegende Eckpunkte. Der
Kugel-Fit im Armplaner tastet darum die Dreiecksflächen jedes Teils mit
höchstens 5 mm Punktabstand ab und umschließt alle diese Punkte (vorher nur
die Eckpunkte: Flächen ragten bis 28 mm aus den Kugeln, jetzt ≤ 1,3 mm; Test
`test_planner_spheres_cover_hook_surfaces`).

## Justier-Werkzeug

`tools/hand_adjust.py` (über `hand_tool.sh adjust`): Tk-Fenster + passiver
MuJoCo-Viewer. Die Schieber setzen nur `body_pos`/`body_quat` von
`*_hand_tool_link` (Korrektur im Hand-Frame, gedreht um die Flansch-Mitte, je
Seite mit eigenem Hand-Frame, also gespiegelt). Speichern verrechnet die Korrektur
mit der gespeicherten Montage (`compose`: `offset' = t + R_adj·offset`,
`R' = R_adj·R`), schreibt nur die beiden Zeilen in `hand.yaml` (Kommentare
bleiben) und ruft den Generator. Ein Test prüft, dass Live-Lage und neu
erzeugte Lage übereinstimmen.

## Montage-Erkennung

`tools/hand_detect.py` (nur numpy + trimesh, kein scipy) rät für eine neue Datei
`mount.origin`, `out`, `palm` und `tcp`; `tools/hand_new.py` zeigt die Vorschläge
und legt das Profil an.

1. **Achsen:** CAD-Teile liegen meist achsparallel, also die STL-Achsen. Liegt
   das Teil schräg (Achsen > 10° gedreht und darin deutlich kompakter), die
   eigenen Achsen aus den größten ebenen Flächen (Normale mit der meisten
   Fläche + die meiste Fläche senkrecht dazu), bei organischen Formen die
   Hauptachsen der Voxel.
2. **Voxel:** gefülltes Gitter, 64 Zellen entlang der längsten Seite.
3. Je Richtung `out` (6 Kandidaten) Punkte für: Länge entlang `out`
   (Gewicht 2), schmales Arm-Ende (Umriss der ersten 8 % gegen den
   Gesamtumriss), breitere Spitze.
4. **Flansch-Punkt:** ab dem Arm-Ende die erste Stelle in den ersten 40 % der
   Länge, ab der der Querschnitt dauerhaft (3 Scheiben) anders ist: ≥ 10 % mehr
   Voxel oder ein Umriss, der sich um ≥ 15 % (und ≥ 2 Voxel) ändert. Dort liegt
   die Flanschfläche, die Platte davor steckt im Arm. Sonst das Ende selbst.
   Quer: Mitte des Umrisses am Arm-Ende.
5. Je `palm` (4 Kandidaten): wie weit die Spitze (letzte 25 %) über den
   Grundkörper (hinter der Platte bis 50 %) hinausragt; ragt nichts, kleiner
   Bonus für die flache Seite.
6. **TCP:** auf der Handflächen-Seite des Grundkörpers, 80 % der Länge vom
   Arm-Ende, quer in der Mitte.

Ausgegeben werden die besten Kombinationen (out, palm), sortiert. Die
Vorschau-Bilder baut `hand_new.py --analyse` in einem eigenen Prozess per
`MjSpec` im Speicher (G1 ohne Rubber-Hand + Mesh nach `side_transform`, ohne
Zerlegung); Tk zeigt sie im Fenster. Erst **Anlegen** schreibt in `hands/` und
ruft `hand_tool.sh build` (venv mit vhacdx); danach ersetzt sich der Prozess
durch `hand_adjust.py`.

Bei den Box-Haken liefert die Erkennung origin (25,4 / 149,8 / 14,0) mm statt
der eingepassten (25 / 150 / 14), `out` = −y; `palm` +z (eingepasst) ist
Vorschlag 2, Vorschlag 1 ist +x (die Spitze ragt auch zur Seite heraus).

## Masse

`mass_kg` je Hand, Trägheit aus dem Mesh bei gleichmäßiger Dichte. Das Inertial
von `*_wrist_yaw_link` bleibt wie im Basismodell (genau wie beim Inspire-Modell).
`loco_sim` lädt fürs Schwerpunkt-Modell das passende MJCF (`mjcf_file_name()`).

## Wer liest was

| Stelle | Liest |
|---|---|
| `simulate_python/config.py` | `G1_HANDS` -> `scene_hand_<name>.xml`; Greif-Testbox und Inspire nur bei `inspire` |
| `scene_editor/build_env_scene.py --hands` | Robotermodell in Umgebungs-Szenen (`start.sh`, `launch.sh` reichen `G1_HANDS` durch) |
| `bringup_sim.launch.py` | `inspire_active()` -> Hand-Bridge nur bei Inspire |
| `robot_state_launcher.launch.py` | URDF für robot_state_publisher/RViz |
| `arm_controller.py`, `ik_solver.py` | URDF für IK, Gate, Planer-Prozess, IK-Prozess |
| `loco_sim.py` | MJCF fürs Schwerpunkt-Modell |
| `start.sh`, `g1_gui.py` | Auswahl, setzt `G1_HANDS` und passend `G1_INSPIRE_HANDS` |

## Tests

`test/test_hand_profile.py`: Auswahlregel inkl. Real-Fall, Szenenwahl in
`config.py`, Ausgaben aktuell (`--check`), Spiegel-Symmetrie, MuJoCo-Modell
(keine Finger-Aktoren, Kollisions-Bits, Masse), TCP in MJCF = URDF, Gate und
Planer-Kugeln decken den Haken ab, Schwerpunkt-Modell lädt, Physik-Probe
(Würfel bleibt auf der Lippe liegen).

`test/test_hand_new.py`: Erkennung bei den Haken (auch schräg gedreht) und bei
einem künstlichen Werkzeug mit Platte und Kralle (gerade und zufällig gedreht),
Einheit, Anlegen aus STL und OBJ (in einem tmp-Ordner), Aufräumen bei kaputter
Datei, Vorschau-Modell legt die Hand wie der Generator.

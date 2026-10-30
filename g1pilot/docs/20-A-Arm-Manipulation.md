# Arm-Manipulation — Anleitung

Richtet sich an: Anwender, die die Arme des G1 über RViz, die Demo-GUI oder den
Controller bedienen wollen. Für Entwickler siehe
[20-T-Arm-Manipulation.md](20-T-Arm-Manipulation.md). Für die
HTTP-Schnittstelle für externe Projekte siehe
[21-A-Arm-API.md](21-A-Arm-API.md).

## Überblick

Die Arme des G1 (je 7 Freiheitsgrade: Schulter Pitch/Roll/Yaw, Ellbogen,
Handgelenk Roll/Pitch/Yaw) werden unabhängig von den Beinen gesteuert. Die
Zielpose kommt von einem interaktiven Marker in RViz, aus »Hand bewegen« in
der Demo-GUI, vom Controller (PS4/Xbox), über die Arm-API oder aus dem
Positionsspeicher; ein inverser Kinematik-Löser (IK) rechnet daraus
Gelenkwinkel, der `arm_controller` fährt sie sicher an. Alle Wege laufen
durch dieselben Sicherheitsprüfungen.

| Weg | Wo beschrieben |
|---|---|
| RViz-Marker, Kommandozeile | hier |
| Demo-GUI: GREIFEN → Arme, »Hand bewegen«, Grundstellung, Sichere Pose | [10-A-Demo-GUI.md](10-A-Demo-GUI.md) |
| Controller: Sticks, Hand-Auswahl, Arm spiegeln | [11-A-Controller.md](11-A-Controller.md) |
| Posen speichern/anfahren, Abläufe, relative Bewegungen | [23-A-Posen-und-Ablaeufe.md](23-A-Posen-und-Ablaeufe.md) |
| Oberkörper drehen (Hüfte) | [24-A-Hueftdrehung.md](24-A-Hueftdrehung.md) |
| Externe Programme (HTTP/JSON) | [21-A-Arm-API.md](21-A-Arm-API.md) |

## Voraussetzungen

- Stack läuft (Simulation oder echter Roboter, siehe
  [01-A-Installation.md](01-A-Installation.md)).
- RViz ist gestartet (Sim: optional über `USE_RVIZ`; Real: standardmäßig an).

## Bedienung

**1. Manipulation aktivieren**

Demo-GUI: Kachel **GREIFEN**. Controller: **R2**. Alternativ per Kommandozeile:

```bash
ros2 topic pub --once /g1pilot/arms/enabled std_msgs/msg/Bool "{data: true}"
```

Die Arme übernehmen dabei sanft ihre aktuelle Stellung (kein Sprung).

**2. Arme bewegen**

In RViz erscheint je ein würfelförmiger interaktiver Marker pro Hand. Marker
mit der Maus ziehen — der Arm folgt in Echtzeit (kartesisches Ziel, per IK
gelöst). Alternativ direkt eine `PoseStamped` veröffentlichen:

```bash
ros2 topic pub -1 /g1pilot/hand_goal/left geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'pelvis'}, pose: {position: {x: 0.20, y: 0.17, z: 0.09}, orientation: {w: 1.0}}}"
```

Solange ein Marker nicht gezogen wird, folgt er standardmäßig der Hand
(Leader-Follower) — nach einer fremden Bewegung (Homing, Positionsspeicher)
"springt" er also nicht, sondern schleicht sich unauffällig nach.

Nach dem Loslassen bleibt der Marker auf der Zielposition stehen, bis der Arm
dort angekommen ist oder das Ziel als nicht erreichbar erkannt wurde (Log:
`Ziel nicht erreichbar`); erst dann kehrt er zur Hand zurück. Während der
Fahrt kann man ihn erneut greifen und weiterziehen (z.B. erst hoch, dann nach
vorne) — das neue Ziel ersetzt das alte. Linker und rechter Marker sind
unabhängig, beide Arme können also gleichzeitig fahren.

Im Gehen-Modus (`/g1pilot/start_walking`) kommt es auf den Lauf-Regler an:

- **Simulation mit einer AGILE-Lauf-Policy** (im Startmenü wählbar): Die Arme bleiben frei. Sie behalten ihre
  Stellung, die Marker bleiben sichtbar und ziehbar, man kann also z. B. eine
  Box halten und dabei gehen. Nur das Nachführen der Marker pausiert, bis man
  wieder zu Greifen wechselt. Die Taille bewegt sich beim Laufen mit, die Hände
  wandern dadurch um einige Zentimeter mit dem Oberkörper.
- **Echter Roboter mit aktivierten Armen** (seit 2026-10-19): Die Arme bleiben
  genau in ihrer Pose, eine laufende Armfahrt endet an der aktuellen Stelle.
  `arm_sdk` behält Arme und Taille (Taille wie beim Greifen gehalten). Marker
  bleiben sichtbar, neue Ziele wirken weiter. NOT-HALT wie bisher (Arme
  schlaff, Taille gehalten). Altes Verhalten (Lauf-Pose) mit
  `G1_WALK_PARK_ARMS=true`. Sind die Arme nicht aktiviert, führt sie der
  Onboard-Regler.
- **Simulation mit der alten Policy** (Standard, bzw. real mit
  `G1_WALK_PARK_ARMS=true`): Die Arme fahren in die Lauf-Pose
  und die Marker sind ausgeblendet. Der Weg in die Lauf-Pose wird geplant (um
  Tisch und Hindernisse herum), genau wie bei Grundstellung und Pose anfahren. Beim Wechsel zu Greifen bzw. START
  BALANCING (`/g1pilot/start_balancing`) erscheinen sie wieder an der aktuellen
  Handposition.

Gesteuert wird das über das Launch-Argument `walk_park_arms` von
`manipulation_launcher.launch.py` (`bringup_sim` setzt es passend zur
Lauf-Policy, `bringup_real` aus `G1_WALK_PARK_ARMS`, Default `false`).

**3. Home-Position**

Button **HOMING ARMS** (Demo-GUI: **Grundstellung**) fährt beide Arme geplant
in die Lauf-Pose: Arme neben dem Körper, Hände auf Hüfthöhe (nur bei aktiver
Manipulation). Es ist dieselbe Pose, die die Arme auf dem echten Roboter beim
Gehen halten.

**4. Positionsspeicher — Posen sichern und wieder anfahren**

Der Positionsspeicher merkt sich beliebige Kombinationen aus linkem Arm,
rechtem Arm, linker Hand, rechter Hand unter einem Namen, gruppiert in
Kategorien ("Ordner"). Hier, was die Funktionen tun (in der Demo-GUI
unter GREIFEN → Erweitert ▸); Abläufe, relative Bewegungen und die genaue
Bedienung in Demo-GUI und Handy stehen in
[23-A-Posen-und-Ablaeufe.md](23-A-Posen-und-Ablaeufe.md).

- **Pose speichern** öffnet einen Dialog: Name, Kategorie (mit Button
  „Kategorie anlegen"), und vier Häkchen für die zu speichernden
  Komponenten. Gespeichert wird die zuletzt *kommandierte* Stellung (nicht
  die verrauschte Messung).
- **Pose anfahren** zeigt alle gespeicherten Posen gruppiert nach Kategorie,
  inklusive der enthaltenen Komponenten. Beim Anfahren wird die Bahn dorthin
  **jedes Mal neu geplant** (meist in 0,02-0,5 s) — kollisionsfrei um
  Hindernisse und den eigenen Körper, da sich Startpose und Umgebung seit
  dem Speichern geändert haben können. In der Mitte der Bewegung hält der
  Arm 5 cm Abstand zu Objekten; nahe am Start und am Ziel darf er näher
  heran, am Ziel auch aufliegen (Hand auf den Tisch legen geht). Greif-Objekte
  darf die Hand immer berühren. Nahe am Tisch kommt die Hand entlang ihrer
  Handachse an (geht das nicht: von oben).
- **Bewegung stoppen** bricht eine laufende geplante Bewegung ab. Das passiert
  auch automatisch, sobald ein Marker manuell angefasst wird — manuelle
  Eingabe hat immer Vorrang.

Über die Kommandozeile (Speichern-Payload als JSON):

```bash
ros2 topic pub --once /g1pilot/pose_store/save std_msgs/msg/String \
  "{data: '{\"name\": \"Regal oben\", \"category\": \"Greifen\", \"components\": [\"arms\", \"hands\"]}'}"
ros2 topic pub --once /g1pilot/pose_store/goto std_msgs/msg/String "{data: 'Regal oben'}"
```

Die Datei liegt persistent im Repository unter `data/arm_poses.json` (im
Container gebunden gemountet) und übersteht damit Container-Neustarts.

**5. Hände**

`OPEN LEFT/RIGHT HAND` und `CLOSE LEFT/RIGHT HAND` öffnen/schließen die
Inspire-FTP-Hände, sofern eingerichtet — siehe
[22-A-Inspire-Haende.md](22-A-Inspire-Haende.md).

## Sicherheitsverhalten (was der Anwender wissen sollte)

- **Kollisions-Gate:** Eine kommandierte Zielpose, die den Arm in den
  eigenen Körper fahren würde, wird zurückgehalten. Gegen Objekte zählt der
  Abstand: die Hand darf bis zur Berührung heran (auf den Tisch legen geht,
  in der Nähe wird sie langsamer), aber nicht hinein; ein weiterer Schritt in
  das Objekt wird so umgelenkt, dass die Hand an der Oberfläche entlang
  gleitet. Liegt die Hand schon am Tisch, kann man sie jederzeit wegziehen.
- **Geschwindigkeitslimits:** Kartesisch 0,25 m/s an Hand und Ellbogen
  (Industriewert für Betrieb mit Personen im Schutzraum), zusätzlich
  1,5 rad/s je Gelenk.
- **Notaus:** NOT-HALT (Demo-GUI, `EMERGENCY STOP`, Controller-PS, Handy)
  macht die Arme sofort drehmomentfrei (gedämpft, kein Sprung in eine
  Home-Pose) — Details in [51-A-Notaus.md](51-A-Notaus.md) und
  [50-A-Echter-Roboter.md](50-A-Echter-Roboter.md).
- **Marker hat immer Vorrang:** Wird während einer laufenden geplanten
  Bewegung (Positionsspeicher) ein Marker angefasst, bricht die Planung
  sofort ab.

## Konfiguration

Die wichtigsten Parameter (per `--ros-args -p <name>:=<wert>` beim Start
oder live via `ros2 param set /arm_controller <name> <wert>`):

| Parameter | Default (Sim / Real) | Bedeutung |
|---|---|---|
| `arm_velocity_limit` | 1.5 rad/s | Gelenk-Geschwindigkeitslimit |
| `ee_velocity_limit` | 0.25 m/s | Kartesisches Limit an Hand + Ellbogen |
| `self_collision_gate` | an | Selbstkollisionsprüfung vor jedem Kommando |
| `environment_collision_gate` | an | Prüfung gegen Hindernisse/Greif-Objekte |
| `planner_clearance` | 0.05 m | Abstand zu Objekten in der Mitte einer geplanten Bewegung |
| `planner_time_budget_s` | 2.5 s | Rechenzeit, die eine Planung höchstens bekommt |
| `kp_low` / `kd_low` | 150/12 (Sim) · 60/1.5 (Real) | PD-Gains der Armgelenke |
| `arm_weight_ramp_up_s` / `_down_s` | 0.0 (Sim) · 2.0 s (Real) | Übergangsrampe beim (De-)Aktivieren |

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| Arm bewegt sich nicht | Arme übernommen? (GREIFEN bzw. Controller R2, `/g1pilot/arms/enabled`) |
| Arm hält vor einem Hindernis an | Kollisions-Gate hat gegriffen — Marker zurückziehen; siehe Log „Kollisions-Gate: ...". |
| Pose anfahren meldet „no_path_found" / „goal_in_collision" | Kein Weg mit mindestens 1,5 cm Abstand gefunden bzw. Zielpose steckt im Körper oder > 2 cm in einem Objekt (Roboter steht anders als beim Speichern?). |
| Pose anfahren meldet „Planer startet noch" | Der Planer-Prozess lädt nach dem Start noch das Modell (einige Sekunden) — kurz warten, erneut anfahren. |
| Arbeitsplatz-Objekte wirken nicht (Sim) | Die Objekte brauchen den Frame `map` — in der Sim nur mit Navigation (Startmenü Frage 4). |
| Marker springt nach Homing | Erwartetes Verhalten: nach Homing/Positionsspeicher wird die IK auf die neue Ist-Pose ausgerichtet. |
| Pose lässt sich nicht speichern | Mindestens eine Komponente auswählen; bei Händen muss die Inspire-Bridge laufen (sonst „skipped"). |
| Positionsspeicher-Datei weg nach Neustart | Läuft der Container ohne Bind-Mount des Repos? Prüfen mit `G1_POSE_STORE` bzw. Log-Zeile „Positionsspeicher: ...". |

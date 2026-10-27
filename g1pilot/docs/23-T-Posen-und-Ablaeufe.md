# Posen und Abläufe — Technik

Richtet sich an Entwickler. Bedienung: [23-A-Posen-und-Ablaeufe.md](23-A-Posen-und-Ablaeufe.md).
Wie der arm_controller eine Pose plant und abfährt (Planer-Prozess,
Bahnuhr, Abbruch): [20-T-Arm-Manipulation.md › Positionsspeicher](20-T-Arm-Manipulation.md#positionsspeicher-plan-execute).

## Beteiligte Dateien

| Datei | Inhalt |
|---|---|
| `g1pilot/manipulation/pose_store.py` | Datei lesen/schreiben (atomar), Kategorien, Posen, Abläufe (`save_sequence`), relative Bewegungen (`save_move`), Migration v1/v2 → v3 |
| `g1pilot/manipulation/arm_controller.py` | `/g1pilot/pose_store/{save,goto,cancel}`, Status auf `/g1pilot/arm_command/status` und `/g1pilot/pose_store/save/status` |
| `g1pilot/teleoperation/sequence_runner.py` | `SequenceRunner`: Zustandsmaschine eines Ablaufs (ohne Qt/ROS) |
| `g1pilot/teleoperation/sequence_editor.py` | Fenster „Posen & Abläufe“ (PyQt6) |
| `g1pilot/teleoperation/demo_gui.py` | hängt den Runner an Knöpfe, Handy und Controller; rechnet relative Bewegungen in ein Ziel um |
| `g1pilot/teleoperation/ps4_arm_teleop.py` | L3 → `/g1pilot/sequence/next`, Sticks gesperrt während `moving` |
| `g1pilot/teleoperation/phone/gateway.py` | Handy-Aktionen `pose`, `move`, `pose_save`, `seq`, `seq_next`, `seq_mode` (siehe [12-T-Handy.md](12-T-Handy.md#aktionen-und-rechte)) |
| `g1pilot/tools/install_example_sequences.py` | Beispiel-Abläufe für Szene A5 |
| `pose_tool.py`, `demo_sequence.py` (Paketwurzel) | Host-Werkzeuge: Posen aufräumen, Kategorie über die Arm-API abfahren |
| `test/test_sequences.py`, `test/test_demo_gui_sequences.py` | Runner, Store v3, GUI-Einbindung |

## Dateiformat

Dateiformat Version 3 (seit 2026-10-22, Abläufe):

```json
{"version": 3,
 "categories": ["Allgemein", "Greifen"],
 "poses": {"<name>": {"category": "...", "left_arm": [...], "saved_at": 0.0}},
 "moves": {"<bewegung>": {"dx": 0.0, "dy": 0.0, "dz": 0.05, "sides": ["left", "right"]}},
 "sequences": {"<ablauf>": {"steps": [{"pose": "<name>", "wait_s": 0.0},
                                      {"move": "<bewegung>", "wait_s": 0.0}]}}}
```

Ein Ablauf (`sequences`) verweist nur auf Posennamen bzw. relative
Bewegungen; dieselbe Pose kann in mehreren Abläufen stehen. Eine **relative
Bewegung** (`moves`, z. B. „Box anheben“ = 5 cm hoch) verschiebt die Hände
von dort, wo sie gerade sind: Meter im Oberkörper-Bezug (`dx` vor, `dy`
links, `dz` hoch; mit dem Taillen-Yaw gedreht wie der Jog), je Achse höchstens
±0,30 m, Orientierung bleibt. Die Demo-GUI rechnet daraus ein Ziel (aktuelle
kommandierte Hand, sonst TF `hand_point_contact`, plus Verschiebung) und
schickt es als `/g1pilot/arm_command` `type=pose` -- geplant, mit Kollisionsschutz
und denselben Statusmeldungen wie ein Pose-Anfahren. Ist die Hand unbekannt,
bricht der Ablauf sofort ab. `wait_s` = Pause nach dem Schritt (nur „Am
Stück“, 0…600 s). Fehlt `sequences` (Version 2), werden die Abläufe aus den
Kategorien `Ablauf <Name>` abgeleitet (Posen nach Namen sortiert) und beim
nächsten Schreiben fest übernommen. `save()` in eine Kategorie
`Ablauf <X>` hängt die Pose zusätzlich hinten an Ablauf `<X>` an (nicht
doppelt). Ausgeführt werden Abläufe von der Demo-GUI
(`teleoperation/sequence_runner.py`: am Stück / Schritt für Schritt,
`/g1pilot/sequence/next` = Weiter, `/g1pilot/sequence/state` = Stand,
latched JSON); der arm_controller sieht davon nur einzelne
`/g1pilot/pose_store/goto` bzw. `arm_command`.

Speicherbare Komponenten: `left_arm`, `right_arm`, `left_hand`, `right_hand`
(`COMPONENTS`-Tupel). Speicherpfad-Priorität: `G1_POSE_STORE`-Env-Variable
> `<repo>/data/arm_poses.json` (bind-gemountet, überlebt Container-Neustarts)
> `~/.g1pilot/` (Host-Betrieb ohne Mount). Schreiben ist atomar
(`write-tmp-then-rename`), Version-1-Dateien (flaches Format ohne
Kategorien) und Version 2 (ohne Abläufe) werden transparent migriert.

## Speicherort

1. `G1_POSE_STORE` (Umgebungsvariable, voller Pfad), sonst
2. `<repo>/g1pilot/data/arm_poses.json` (im Container bind-gemountet, versioniert), sonst
3. `~/.g1pilot/` (Host-Betrieb ohne Mount).

## Ablauf-Zustand (`sequence_runner.py`)

Zustände: `idle` → `sent` (goto geschickt) → `moving` (`accepted` /
`executing`) → `pause` (am Stück, `wait_s` läuft) oder `waiting` (Schritt
für Schritt) → nächster Schritt bzw. `idle`.

Statusfolge des arm_controllers für **ein** goto: (`cancelled` der alten
Fahrt) → `accepted` → `executing` → `reached` | `failed` | `cancelled`, bzw.
sofort `rejected`. Endzustände **vor** dem `accepted` gehören zur alten
Fahrt und werden ignoriert. Kommt 4 s nach dem Senden keine Antwort, bricht
der Runner ab.

| Topic | Typ | Inhalt |
|---|---|---|
| `/g1pilot/sequence/next` | `std_msgs/Bool` | Weiter (Controller L3) |
| `/g1pilot/sequence/state` | `std_msgs/String`, latched JSON | `{state, name, step, n, mode, pose}` (`snapshot()`), für Controller und Handy |
| `/g1pilot/pose_store/goto` | `std_msgs/String` | Pose anfahren (Name) |
| `/g1pilot/arm_command` | `std_msgs/String`, JSON | relative Bewegung als `type=pose` |
| `/g1pilot/pose_store/cancel` | `std_msgs/Bool` | Stopp |

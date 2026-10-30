# Simulation — Anleitung

Richtet sich an alle, die ohne echten Roboter arbeiten: Bedienung testen,
Abläufe vorbereiten, Lauf-Policies vergleichen. Technik:
[40-T-Simulation.md](40-T-Simulation.md).

## Was die Sim ist

Die Sim ersetzt den echten G1 durch ein MuJoCo-Modell, das dieselben
Unitree-DDS-Topics spricht (`rt/lowstate`, `rt/lowcmd`, `rt/arm_sdk`). Der
ROS-Stack darüber ist fast derselbe wie am echten Roboter; nur die
Fortbewegung kommt aus einer eigenen Lauf-Policy (`loco_sim`) statt aus der
Unitree-Firmware. Fällt der Roboter um, passiert nichts, darum ist die Sim
der Ort für alles Neue.

## Starten

```bash
cd g1pilot
./start.sh      # Startmenü → Simulation
```

Optionen (Umgebung, Bedienoberfläche, Lauf-Policy, Hände, Navigation,
Controller, Handy, RViz, Sim-Tempo) stehen in
[03-A-Startmenue.md](03-A-Startmenue.md#optionen). Es öffnen sich das
MuJoCo-Fenster (türkises Icon), die Demo-GUI (blaues Icon) und auf Wunsch RViz.

## Auto-Start (GREIFEN)

In der Sim stellt sich der Roboter nach dem Start von selbst hin und die
Arme gehen an: die GUI schaltet automatisch in **GREIFEN**. Das kann auf
langsamen Rechnern ein paar Sekunden dauern, weil die GUI wartet, bis
`arm_controller` und `loco_sim` geladen sind und den Modus bestätigen. Wer
vorher selbst einen Modus wählt oder NOT-HALT drückt, bricht den Auto-Start
ab. Am echten Roboter gibt es diesen Auto-Start bewusst nicht.

## Umgebungen

Eine Umgebung ist nur das Drumherum (Tisch, Kisten, Regale, Greif-Objekte);
der G1 bleibt immer gleich und wird beim Start hineingeladen. Umgebungen
liegen in `unitree_mujoco/scene_editor/scenes/<name>.xml`; im Startmenü
erscheint jede automatisch.

Angelegt und bearbeitet werden sie über die Karte **Umgebungen** im
Startmenü (Editor öffnen, mit dem G1 ansehen, leere Umgebung, aus
Text-Prompt). Namensregeln im Editor:

| Objektname | Wirkung |
|---|---|
| `grasp_<name>` | Greif-Objekt: frei beweglich, die Hand darf ran |
| `g1_spawn` | Startpunkt und Blickrichtung des G1 (wird beim Laden entfernt) |
| `station_<Name>` | Ziel für AUTO NAV, in der Demo-GUI als Knopf »➜ Name« ([31-A-Navigation.md](31-A-Navigation.md)) |
| alles andere | statisches Hindernis, der Arm weicht aus |

Ausführlich (Setup des Editors, STL-Import, Stolpersteine):
[`scene_editor/README.md`](../../unitree_mujoco/scene_editor/README.md).

## Lauf-Policies

Im Startmenü unter *Lauf-Policy*: `g1_wholebody` (Standard),
`agile_velocity_g1`, `agile_velocity_height_g1`. Unterschiede:
[30-A-Stehen-und-Gehen.md](30-A-Stehen-und-Gehen.md#lauf-regler-wählen-nur-sim).

## Nur in der Sim

| Knopf (Demo-GUI, untere Leiste) | Wirkung |
|---|---|
| **Roboter schubsen** | Stoß in zufälliger Richtung. Kurz tippen = 150 N; halten lädt auf bis 2000 N nach 3 s, Loslassen schubst |
| **Szene zurücksetzen** | bewegliche Objekte zurück an den Start, der Roboter bleibt, wie er ist |
| **Roboter zurücksetzen** | Roboter zurück an den Startpunkt (bzw. `g1_spawn`), festgehalten; der nächste Modus-Klick stellt ihn wieder hin |
| **Sim beenden** | zweimal tippen, beendet den ganzen Stack |

Der Streamdeck hat zusätzlich **GRASP BOX**: eine greifbare Testkugel in den
Inspire-Handflächen an/aus ([22-A-Inspire-Haende.md](22-A-Inspire-Haende.md)).

## Sim-Tempo

Die Sim läuft im Gleichschritt mit der Lauf-Policy (50 Hz) und höchstens in
Echtzeit. Ist der Rechner zu langsam, läuft alles in Zeitlupe, aber die
Policy sieht trotzdem ihre gewohnte Dynamik. Mit *Sim-Tempo* (Startmenü,
Erweitert) lässt sich die Obergrenze bewusst senken, z. B. 0,5.

## Fehlerbehebung

| Symptom | Ursache / Abhilfe |
|---|---|
| Fenster offen, Arme reagieren nicht | Auto-Start wartet noch auf `arm_controller`/`loco_sim` (Infozeile der GUI). Nach 2 min gibt er auf; dann GREIFEN antippen. Hilft das nicht, Log nach Fehlern von `arm_controller` durchsuchen. |
| Sim ruckelt, Log: `MuJoCo rendert auf der CPU (llvmpipe)` | kein GPU-Zugriff im Container: `make gpu-check`, siehe [01-A-Installation.md](01-A-Installation.md#fehlerbehebung). |
| Roboter läuft in Zeitlupe | Rechner schafft keine Echtzeit; das ist gewollt (Gleichschritt). Weniger Fenster (ohne RViz) hilft. |
| Greif-Objekte fliegen beim Start weg | `trimesh`/`vhacdx` fehlen im `scene_editor/.venv`, siehe [01-A-Installation.md](01-A-Installation.md). |
| Umgebung lädt nicht, Log: »Umgebung konnte nicht erzeugt werden« | `build_env_scene.py` scheitert, die Sim startet mit der Standard-Szene. Szene im Editor prüfen. |
| Roboter fällt beim Gehen um | Andere Lauf-Policy probieren; »Roboter zurücksetzen«. |
| *Sim beenden* meldet, es gehe nur mit `make stop` | Stack wurde ohne Watcher gestartet (`make sim-bg`, `docker compose` direkt). |

# Inspire-FTP-Hände — Anleitung

Richtet sich an: Anwender, die die Inspire-RH56DFTP-2-Hände in Simulation
oder auf echter Hardware nutzen wollen. Für die interne Funktionsweise siehe
[22-T-Inspire-Haende.md](22-T-Inspire-Haende.md).

## Überblick

Jede Hand hat 6 steuerbare Freiheitsgrade (Kleiner, Ring-, Mittel-,
Zeigefinger, Daumen-Beugung, Daumen-Rotation) und liefert Kraft- sowie
Tastsinn-Rückmeldung. Bedient werden die Hände über:

- **Demo-GUI** (Normalfall): GREIFEN → Seite **HÄNDE** — ganze Hand
  öffnen/schließen, je Finger ein Soll-Schieber mit Ist-Öffnung und Kraft,
  Kraftzonen-Heatmap, unter *Erweitert* Griffkraft (Sanft 300 g / Mittel
  800 g / Fest 1500 g) und Tempo. Details: [10-A-Demo-GUI.md](10-A-Demo-GUI.md).
- **Controller:** Kreis = öffnen, Viereck = schließen (gewählte Hand),
  siehe [11-A-Controller.md](11-A-Controller.md).
- **Handy:** Hände öffnen/schließen, siehe [12-A-Handy.md](12-A-Handy.md).

Die früheren Browser-Oberflächen der Hand-Bridge (Controller/Viewer) wurden
2026-11-04 entfernt; alles davon steckt in der Demo-GUI.

Alle Wege sind in Simulation und auf echter Hardware identisch.

## Aktivieren

Beim Start über das grafische Startmenü: Häkchen „Inspire-FTP-Hände" setzen
(sonst wird das einfache Rubber-Hand-Modell ohne bewegliche Finger geladen).
Nicht-interaktiv:

```bash
G1_INSPIRE_HANDS=1 ./start.sh --yes
```

Auf echter Hardware zusätzlich die Modbus-IP-Adressen der Hände angeben
(Default `192.168.123.210`/`.211`, Port 6000).

## Bedienung

**Demo-GUI:** Greifen → Hände.

**Per Kommandozeile:**

```bash
ros2 topic pub --once /g1pilot/hand_action/right std_msgs/msg/String "{data: 'open'}"
ros2 topic pub --once /g1pilot/hand_action/right std_msgs/msg/String "{data: 'close'}"
```

**Handstellungen speichern:** Über den Positionsspeicher (Komponenten
linke/rechte Hand) — siehe [23-A-Posen-und-Ablaeufe.md](23-A-Posen-und-Ablaeufe.md).

## In der Simulation: greifbare Testkugel

Das Topic `/g1pilot/grasp_box` (Bool, `true` = an, `false` = aus, z. B.
`ros2 topic pub --once /g1pilot/grasp_box std_msgs/msg/Bool "{data: true}"`)
legt eine kleine, fest mit der
Handfläche verbundene greifbare Kugel in die Griffzone. Beim Schließen der
Hand (etwa 70 %) greifen alle Finger und die Handfläche zu — sofort sichtbare
Griffkräfte in der Demo-GUI (Kraftzonen). Standardmäßig aus; direkt beim Start aktiv
mit `G1_GRASP_TEST=1`.

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| Demo-GUI meldet „Keine Daten von der Hand-Bridge“ bzw. Hand nicht verbunden (real) | IP/Port prüfen (`nc -zv <ip> 6000`); Hand am G1 mit Strom versorgt? |
| Finger bewegen sich nicht in RViz | Ist die Inspire-Bridge aktiv (`G1_INSPIRE_HANDS=1`)? `robot_state` muss dann `publish_hand_joints:=false` laufen (passiert automatisch über die Launch-Datei). |
| Kraft/Taktil bleiben 0 (Simulation) | Backend `sim` (Stufe 1) hat keine Kontaktphysik — Default ist Stufe 2 (`mujoco`), die echte Kräfte liefert. |

# Dokumentation

Zentraler Einstieg in die G1Pilot-Dokumentation. Alle Dokumente sind auch
über das Startmenü erreichbar (`./start.sh` → **Dokumentation ▾**).

## Namensschema

Dateien heißen `GT-X-Name.md`:

| Teil | Bedeutung |
|---|---|
| `G` | Gruppe 0–9 (siehe unten) |
| `T` | Thema 0–9 innerhalb der Gruppe |
| `X` | `A` = **Anleitung** für Anwender (starten, bedienen, Fehler beheben, kein Code-Wissen nötig), `T` = **Technik** für Entwickler (Dateien, Topics, Parameter, Datenfluss, Tests) |
| `Name` | Thema, ohne Umlaute und Leerzeichen |

Zu jeder Gruppe gibt es ein Übersichtsdokument `G-Name.md` (nur die
Gruppenziffer) mit einer Kurzfassung je Thema und den Links, z. B.
[1-Bedienung.md](1-Bedienung.md) mit allen Bedienwegen.

Ein Thema hat in beiden Dokumenten dieselbe Nummer und denselben Namen
(`11-A-Controller.md` / `11-T-Controller.md`). Nicht jedes Thema braucht
beide. Neue Themen bekommen die nächste freie Nummer ihrer Gruppe; nach
jedem Umbenennen `python3 tools/check_doc_links.py` laufen lassen
([61-A-Werkzeuge.md](61-A-Werkzeuge.md#doku-links-prüfen)).

## 0 · Einstieg

Übersicht: [0-Einstieg.md](0-Einstieg.md)

| Thema | Anleitung | Technik |
|---|---|---|
| Installation | [01-A-Installation.md](01-A-Installation.md) | |
| Architektur (Container, DDS, Nodes) | | [02-T-Architektur.md](02-T-Architektur.md) |
| Startmenü, `start.sh`, `make` | [03-A-Startmenue.md](03-A-Startmenue.md) | |
| Umgebungsvariablen | | [04-T-Umgebungsvariablen.md](04-T-Umgebungsvariablen.md) |
| Nodes und Topics (Diagramme) | | [05-T-Nodes-und-Topics.md](05-T-Nodes-und-Topics.md) |

## 1 · Bedienung

Übersicht: [1-Bedienung.md](1-Bedienung.md)

| Thema | Anleitung | Technik |
|---|---|---|
| Demo-GUI (Standard-Oberfläche) | [10-A-Demo-GUI.md](10-A-Demo-GUI.md) | [10-T-Demo-GUI.md](10-T-Demo-GUI.md) |
| Controller (PS4/Xbox) | [11-A-Controller.md](11-A-Controller.md) | [11-T-Controller.md](11-T-Controller.md) |
| Handy (Bluetooth, Android) | [12-A-Handy.md](12-A-Handy.md) | [12-T-Handy.md](12-T-Handy.md) |
| Streamdeck und alter Joystick-Modus | [13-A-Streamdeck.md](13-A-Streamdeck.md) | [13-T-Streamdeck.md](13-T-Streamdeck.md) |

## 2 · Arme und Hände

Übersicht: [2-Arme-und-Haende.md](2-Arme-und-Haende.md)

| Thema | Anleitung | Technik |
|---|---|---|
| Arm-Manipulation (IK, Planer, Übergabe) | [20-A-Arm-Manipulation.md](20-A-Arm-Manipulation.md) | [20-T-Arm-Manipulation.md](20-T-Arm-Manipulation.md) |
| Arm-API (externe Projekte) | [21-A-Arm-API.md](21-A-Arm-API.md) | [21-T-Arm-API.md](21-T-Arm-API.md) |
| Inspire-Hände | [22-A-Inspire-Haende.md](22-A-Inspire-Haende.md) | [22-T-Inspire-Haende.md](22-T-Inspire-Haende.md) |
| Posen und Abläufe | [23-A-Posen-und-Ablaeufe.md](23-A-Posen-und-Ablaeufe.md) | [23-T-Posen-und-Ablaeufe.md](23-T-Posen-und-Ablaeufe.md) |
| Hüftdrehung | [24-A-Hueftdrehung.md](24-A-Hueftdrehung.md) | [24-T-Hueftdrehung.md](24-T-Hueftdrehung.md) |

## 3 · Fortbewegung

Übersicht: [3-Fortbewegung.md](3-Fortbewegung.md)

| Thema | Anleitung | Technik |
|---|---|---|
| Stehen und Gehen (Sim-Policy, Unitree-Loco) | [30-A-Stehen-und-Gehen.md](30-A-Stehen-und-Gehen.md) | [30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md) |
| Navigation (AUTO NAV, Stationen) | [31-A-Navigation.md](31-A-Navigation.md) | [31-T-Navigation.md](31-T-Navigation.md) |

## 4 · Simulation

Übersicht: [4-Simulation.md](4-Simulation.md)

| Thema | Anleitung | Technik |
|---|---|---|
| Simulation (MuJoCo, Umgebungen, Auto-Start) | [40-A-Simulation.md](40-A-Simulation.md) | [40-T-Simulation.md](40-T-Simulation.md) |

Umgebungen bauen: [`unitree_mujoco/scene_editor/README.md`](../../unitree_mujoco/scene_editor/README.md).

## 5 · Echter Roboter

Übersicht: [5-Echter-Roboter.md](5-Echter-Roboter.md)

| Thema | Anleitung | Technik |
|---|---|---|
| Echter Roboter (Sicherheit und Ablauf) | [50-A-Echter-Roboter.md](50-A-Echter-Roboter.md) | |
| Notaus (alle Halt-Wege) | [51-A-Notaus.md](51-A-Notaus.md) | [51-T-Notaus.md](51-T-Notaus.md) |

## 6 · Entwicklung

Übersicht: [6-Entwicklung.md](6-Entwicklung.md)

| Thema | Anleitung | Technik |
|---|---|---|
| Tests | [60-A-Tests.md](60-A-Tests.md) | |
| Werkzeuge (Make, Diagnose, Messskripte, Link-Prüfung) | [61-A-Werkzeuge.md](61-A-Werkzeuge.md) | |

Gruppen 7–9 sind frei.

## Aufbau der Dokumente

Anleitungen: Überblick, Bedienung Schritt für Schritt, Einstellungen,
Fehlerbehebung. Technik: beteiligte Dateien, Datenfluss und Topics,
Parameter, Tests, bekannte Einschränkungen. Der Aufbau ist ähnlich, aber
nicht starr.

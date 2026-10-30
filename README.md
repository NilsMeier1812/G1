# G1Pilot — MuJoCo-Simulation & Steuerungsstack für den Unitree G1

G1Pilot ist ein ROS-2-Steuerungsstack für den humanoiden Roboter Unitree G1
(29 Freiheitsgrade, optional Inspire-FTP-Hände). Er läuft **unverändert**
sowohl gegen eine MuJoCo-Simulation als auch gegen den echten Roboter — beide
sprechen dieselbe Unitree-DDS-Schnittstelle, sodass derselbe Code in Sim und
Real läuft.

| | |
|---|---|
| Roboter | Unitree G1, 29 DOF, optional Inspire-FTP-Hände |
| Simulator | MuJoCo (Python-Bindings), 1 kHz Physik |
| Middleware | Unitree SDK2 (CycloneDDS) + ROS 2 Humble |
| Laufzeit | Docker-Container, `network_mode: host` |

## Funktionsumfang

- **Arm-Manipulation** — kartesische Zielposen per interaktivem RViz-Marker,
  gelöst über eine Pinocchio-basierte inverse Kinematik, mit
  Kollisionsprüfung und Positionsspeicher.
- **Arm-API** — eine HTTP/JSON-Schnittstelle, über die auch Projekte ohne
  ROS Zielposen einspielen können.
- **Locomotion** — Stehen und Laufen; in der Simulation über einen
  RL-basierten Lauf-Regler plus modellbasierten Stand-Regler, auf echter
  Hardware über Unitrees Onboard-Controller.
- **Bedienung** — Demo-GUI (Standard), PS4-/Xbox-Controller für Arme,
  Hände, Hüfte und Gehen, Handy per Bluetooth, alternativ Streamdeck.
- **Posen und Abläufe** — Armposen speichern und als Ablauf abfahren.
- **Notaus** — NOT-HALT aus GUI, Controller und Handy, Voll-Notaus am
  Controller.
- **Autonome Navigation** — Punkt-zu-Punkt-Fahrt mit Pfadplanung und
  Hindernisvermeidung.
- **Inspire-FTP-Hände** — Fingersteuerung mit Kraft- und Tastsinn-Rückmeldung,
  in Simulation und auf echter Hardware.
- **Sim ↔ Real ohne Code-Änderung** — Umschalten allein über
  Umgebungsvariablen.

## Schnellstart

```bash
git clone https://github.com/NilsMeier1812/G1.git
cd G1/g1pilot
make build-sim
make sim
```

Echter Roboter: Schritt für Schritt vom frischen PC bis zur Demo-GUI in
[g1pilot/docs/50-A-Echter-Roboter.md](g1pilot/docs/50-A-Echter-Roboter.md).

Ausführliche Installationsanleitung (Linux, Windows/WSL2, Voraussetzungen):
[g1pilot/docs/01-A-Installation.md](g1pilot/docs/01-A-Installation.md).

Nach dem Start öffnet sich ein grafisches Startmenü (`./start.sh`), über das
sich sowohl die Simulation als auch der echte Roboter starten lassen, und
über das die gesamte Dokumentation erreichbar ist (Menüpunkt
„Dokumentation").

## Dokumentation

Die vollständige Dokumentation liegt in [g1pilot/docs/](g1pilot/docs/README.md)
und ist nach Gruppen und Themen nummeriert (`GT-X-Name.md`). Zu einem Thema
gibt es eine **Anleitung** (`-A-`) für Anwender und meist ein
**Technik**-Dokument (`-T-`) für Entwickler.

| Gruppe | Themen |
|---|---|
| [0 Einstieg](g1pilot/docs/0-Einstieg.md) | [Installation](g1pilot/docs/01-A-Installation.md), [Architektur](g1pilot/docs/02-T-Architektur.md), [Startmenü](g1pilot/docs/03-A-Startmenue.md), [Umgebungsvariablen](g1pilot/docs/04-T-Umgebungsvariablen.md) |
| [1 Bedienung](g1pilot/docs/1-Bedienung.md) | [Demo-GUI](g1pilot/docs/10-A-Demo-GUI.md), [Controller](g1pilot/docs/11-A-Controller.md), [Handy](g1pilot/docs/12-A-Handy.md), [Streamdeck](g1pilot/docs/13-A-Streamdeck.md) |
| [2 Arme und Hände](g1pilot/docs/2-Arme-und-Haende.md) | [Arm-Manipulation](g1pilot/docs/20-A-Arm-Manipulation.md), [Arm-API](g1pilot/docs/21-A-Arm-API.md), [Inspire-Hände](g1pilot/docs/22-A-Inspire-Haende.md), [Posen und Abläufe](g1pilot/docs/23-A-Posen-und-Ablaeufe.md), [Hüftdrehung](g1pilot/docs/24-A-Hueftdrehung.md) |
| [3 Fortbewegung](g1pilot/docs/3-Fortbewegung.md) | [Stehen und Gehen](g1pilot/docs/30-A-Stehen-und-Gehen.md), [Navigation](g1pilot/docs/31-A-Navigation.md) |
| [4 Simulation](g1pilot/docs/4-Simulation.md) | [Simulation](g1pilot/docs/40-A-Simulation.md) |
| [5 Echter Roboter](g1pilot/docs/5-Echter-Roboter.md) | [Echter Roboter](g1pilot/docs/50-A-Echter-Roboter.md), [Notaus](g1pilot/docs/51-A-Notaus.md) |
| [6 Entwicklung](g1pilot/docs/6-Entwicklung.md) | [Tests](g1pilot/docs/60-A-Tests.md), [Werkzeuge](g1pilot/docs/61-A-Werkzeuge.md) |

Verlinkt sind die Anleitungen; das Technik-Dokument zum selben Thema hat
dieselbe Nummer mit `-T-` statt `-A-`. Namensschema und vollständige
Tabelle: [g1pilot/docs/README.md](g1pilot/docs/README.md).

## Projektstruktur

```
.
├── g1pilot/                     ROS-2-Package + Docker/Compose/Makefile (Hauptprojekt)
│   ├── docs/                    vollständige Dokumentation
│   ├── g1pilot/                 Node-Quellcode (state, manipulation, navigation, teleoperation, utils)
│   ├── launch/                  ROS-2-Launchdateien
│   ├── policies/                RL-Lauf-Policies (ONNX, Sim): g1_wholebody (Standard), agile_velocity_g1, agile_velocity_height_g1
│   └── docker/, docker-compose.yml
├── unitree_mujoco/               MuJoCo-Simulation (Unitree, angepasst)
├── unitree_ros2/                 Unitree-ROS-2-Abhängigkeiten
└── unitree_sdk2_python/          Unitree-SDK (Python-Bindings)
```

Das Repository ist eigenständig (keine Git-Submodule) — alle Abhängigkeiten
liegen mit im Baum.

## Bekannte Einschränkungen

- Der Simulations-Lauf-Regler (`loco_sim`) ist ein reiner Sim-Stellvertreter
  für den Onboard-Regler des echten G1 und nicht für den Realeinsatz gedacht
  — Details in [g1pilot/docs/30-T-Stehen-und-Gehen.md](g1pilot/docs/30-T-Stehen-und-Gehen.md).
- Autonome Navigation hat keine Live-Perzeption; Hindernisvermeidung gilt
  nur für die beim Start geladene Umgebung.

## Lizenz

Siehe [g1pilot/LICENSE](g1pilot/LICENSE).

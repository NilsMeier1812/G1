# Alter Joystick — Technik

Richtet sich an: Entwickler, die den alten Joystick-Pfad oder den
Auto/Manuell-Mux ändern wollen. Der PS4-/Xbox-Controller ist in
[11-T-Controller.md](11-T-Controller.md) beschrieben, die Demo-GUI in
[10-T-Demo-GUI.md](10-T-Demo-GUI.md). Für die Bedienung siehe
[13-A-Alter-Joystick.md](13-A-Alter-Joystick.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/teleoperation/joystick.py` | Physischer Controller (evdev) → `/g1pilot/joy_manual` |
| `g1pilot/teleoperation/joy_mux.py` | Multiplexer manuell/autonom |

Die frühere Streamdeck-GUI (`ui_interface.py`) wurde am 2026-10-30
entfernt. Was die Demo-GUI von ihr geerbt hat (Bedien-Node mit allen
Publishern, Bildschirm-Joystick, Posen-Dialoge), liegt jetzt in
`g1pilot/teleoperation/gui_base.py` (`GuiNode`), siehe
[10-T-Demo-GUI.md](10-T-Demo-GUI.md).

## `joystick.py` (alter Joystick-Modus)

Läuft nur ohne `G1_PS4_ARMS=1` (`UnlessCondition(ps4_arms)` im
`teleoperation_launcher`).

`ManualJoystick(Node)` liest per `evdev` direkt von `/dev/input` (im
Docker-Compose-Profil `real` mit `privileged: true` und
`/dev/input:/dev/input` gemountet). Läuft in einem eigenen Thread
(`read_loop`), der Achsen-/Button-Zustand in eine gemeinsame, gelockte
Struktur schreibt; ein 50-Hz-Timer publiziert daraus `sensor_msgs/Joy` auf
`/g1pilot/joy_manual`. `BTN_NORTH` (Dreieck) togglet zusätzlich
`/g1pilot/auto_enable` direkt in diesem Node (steigende Flanke).

Die eigentliche Button-/Achsen-Interpretation (Deadman, Move, E-Stop,
Greifer) sitzt **nicht** hier, sondern in `loco_client.joystick_callback`
(siehe [30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md)) — `joystick.py` liefert nur
das rohe `Joy`-Signal.

## `joy_mux.py` (Auto/Manuell-Mux)

Mischt `/g1pilot/joy_manual` (Mensch) und `/g1pilot/auto_joy` (von
`nav2point`, siehe [31-T-Navigation.md](31-T-Navigation.md)) auf
ein gemeinsames Ausgabe-Topic `/g1pilot/joy`:

```python
if self.auto_enabled and self.last_auto is not None:
    self.pub.publish(self.last_auto)
elif use_manual:
    self.pub.publish(self.last_manual)
```

`use_manual` gilt, wenn entweder Autonomie aus ist oder eine manuelle
Eingabe innerhalb eines kurzen Vorrangfensters (`manual_priority_window`,
Default 50 ms) eingetroffen ist — ein kurzer manueller Eingriff kann die
Autonomie also jederzeit kurz überstimmen. Beim Ausschalten von
`auto_enable` wird einmalig ein neutraler Stop-`Joy` gesendet (sonst behielte
`loco_client`/`loco_sim` das zuletzt kommandierte Move bei).

## Datenfluss

```
joystick.py ──/g1pilot/joy_manual──┐
                                    ├──► joy_mux ──/g1pilot/joy──► loco_client.joystick_callback ──► Move()   (real)
nav2point   ──/g1pilot/auto_joy────┘                         └──► joy_to_cmdvel ──/g1pilot/loco_cmd_vel──► loco_sim   (Sim)
```

`nav2point` läuft nur in der Simulation mit Navigation (`G1_ENABLE_NAV=1`);
auf dem echten G1 kommt über `joy_mux` nur der manuelle Joystick. In der
Simulation liest `loco_sim` keinen `Joy` — `joy_to_cmdvel` übersetzt
`/g1pilot/joy` in `/g1pilot/loco_cmd_vel` (siehe
[31-T-Navigation.md](31-T-Navigation.md)).

## Bekannte Einschränkungen

- `joystick.py` erwartet einen exakten Gerätenamen-Treffer
  (`dev.name == self.joystick_name`) — kein Teilstring-Match.

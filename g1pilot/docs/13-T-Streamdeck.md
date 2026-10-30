# Streamdeck — Technik

Richtet sich an: Entwickler, die den Streamdeck, den alten Joystick-Pfad
oder den Auto/Manuell-Mux ändern wollen. Der PS4-/Xbox-Controller ist in
[11-T-Controller.md](11-T-Controller.md) beschrieben, die Demo-GUI in
[10-T-Demo-GUI.md](10-T-Demo-GUI.md). Für die Bedienung siehe
[13-A-Streamdeck.md](13-A-Streamdeck.md).

## Beteiligte Dateien

| Datei | Rolle |
|---|---|
| `g1pilot/teleoperation/ui_interface.py` | Streamdeck-GUI (PyQt6), Hauptbedienoberfläche |
| `g1pilot/teleoperation/joystick.py` | Physischer Controller (evdev) → `/g1pilot/joy_manual` |
| `g1pilot/teleoperation/joy_mux.py` | Multiplexer manuell/autonom |
| `g1pilot/teleoperation/sim_autostart.py` | Sim-Auto-Start mit Quittung (gemeinsam mit der Demo-GUI) |
| `g1_gui.py` (Paketwurzel) | Startmenü (getrennt von der Streamdeck-GUI, siehe [03-A-Startmenue.md](03-A-Startmenue.md)) |

## `ui_interface.py` (Streamdeck)

`StreamDeck(Node)` hält alle ROS-Publisher; `ButtonGUI(QWidget)` ist die
eigentliche PyQt6-Oberfläche. Jeder Button ist ein einfacher Callback, der
auf ein Bool-/String-Topic publiziert — die Logik liegt bewusst in den
empfangenden Nodes (`arm_controller`, `loco_sim`/`loco_client`,
`inspire_hand`), nicht in der GUI.

Wichtige Muster:

- `toggle_button` / `flash_button` / `radio_loco`: generische Hilfsfunktionen
  für Toggle-, Impuls- und Radio-Buttons (z. B. BALANCING/WALK sind
  gegenseitig exklusiv, `loco_group`).
- `VirtualJoystick`: zeichnet einen Kreis, mapped Mausposition relativ zum
  Zentrum auf `vx`/`vy` in `[-1, 1]`; `publish_cmd_vel()` läuft über einen
  33-ms-Timer und sendet kontinuierlich (auch 0 im Ruhezustand) — dauerhaftes
  Senden ist harmlos und garantiert beim Loslassen sauber das
  Stop-Kommando.
- `PoseSaveDialog` / `PoseLoadDialog`: siehe
  [20-T-Arm-Manipulation.md](20-T-Arm-Manipulation.md), Abschnitt
  Positionsspeicher — die GUI liest den `PoseStore` direkt von der Platte
  (derselbe Prozessraum/Container wie `arm_controller`, kein ROS-Service
  nötig für die Anzeige; Schreiben läuft über das Topic
  `/g1pilot/pose_store/save`, damit `arm_controller` die einzige Quelle für
  „was ist gerade kommandiert" bleibt).
- `open_hand_guis()`: `ui_interface` läuft im Container ohne Browser. Statt
  selbst einen Browser zu starten, fasst der Node eine Trigger-Datei
  (`.gui_open_request`) im bind-gemounteten Repo an; ein Host-Watcher in
  `start.sh` beobachtet die Datei und öffnet den Browser dort, wo tatsächlich
  einer existiert (siehe [22-T-Inspire-Haende.md](22-T-Inspire-Haende.md)).
- **Auto-Start nur in der Simulation** (`SimAutoStart`, `sim_autostart.py`):
  fordert Arme + BALANCING an und wiederholt nur, was noch fehlt, bis
  `arm_controller` (`/g1pilot/arms/state`) und `loco_sim`
  (`/g1pilot/loco_state`) bestätigen. Ein Eingriff des Nutzers (Modus,
  NOT-HALT, ENABLE) bricht ihn ab. Auf echter Hardware ist dieser Pfad
  **hart ausgeschaltet** (kein Parameter zum Umgehen) — der Operator
  entscheidet dort immer selbst, wann sich der Roboter bewegt
  (`self.sim_mode = is_sim_mode()`). Hintergrund (Bug 2026-10-13):
  [40-T-Simulation.md](40-T-Simulation.md#auto-start-greifen).
- `emergency_stop()`: setzt alle Buttons zurück und publiziert `False` auf
  sämtliche Steuertopics sowie `True` auf `/g1pilot/emergency_stop` — die
  eigentliche Wirkung (Arme drehmomentfrei, Beine real `StopMove`) passiert
  in den jeweiligen Regel-Nodes, nicht in der GUI. Ein NOT-HALT von außen
  (Controller-PS) wird nur angezeigt (`_show_emergency_stop`).

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

## Datenfluss (Real, mit Navigation)

```
joystick.py ──/g1pilot/joy_manual──┐
                                    ├──► joy_mux ──/g1pilot/joy──► loco_client.joystick_callback ──► Move()
nav2point   ──/g1pilot/auto_joy────┘
```

In der Simulation läuft derselbe Pfad, nur dass `loco_sim` keinen `Joy`
liest — `joy_to_cmdvel` übersetzt `/g1pilot/joy` zusätzlich in
`/g1pilot/loco_cmd_vel` (siehe [31-T-Navigation.md](31-T-Navigation.md)).

## Bekannte Einschränkungen

- Die Streamdeck-GUI hat keinen eigenen Zustand über den ROS-Graph hinaus —
  nach einem Neustart der GUI (nicht des Stacks) zeigt sie ggf. kurzzeitig
  einen anderen Button-Zustand als der tatsächliche Roboterzustand, bis der
  nächste Status-Callback eintrifft (z. B. Marker-Follow-Default).
- `joystick.py` erwartet einen exakten Gerätenamen-Treffer
  (`dev.name == self.joystick_name`) — kein Teilstring-Match.

# Herkunft & Lizenz der AGILE-Lauf-Policy „Velocity-Height"

`policy.onnx` ist die Lauf-Policy **Velocity-Height-G1-History-v0** aus NVIDIAs
[`nvidia-isaac/WBC-AGILE`](https://github.com/nvidia-isaac/WBC-AGILE), Commit
`6830cf995714e81c91ce63247e8e016d36e28f14`, Datei
`agile/data/policy/velocity_height_g1/unitree_g1_velocity_height_history_torchscript.pt`.

* **Lizenz:** Apache License 2.0, Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES
  (Text in `LICENSE`, aus `LICENCE` Abschnitt B des Quell-Repos; das Repo hat keine
  NOTICE-Datei). Erlaubt Nutzung, Aenderung und Weitergabe, auch kommerziell, solange
  Lizenztext und Hinweise mitgegeben werden.
* **Geaendert (Apache-2.0 §4b):** Das Format wurde von TorchScript nach ONNX
  konvertiert. Gewichte und Rechnung sind unveraendert (siehe unten).
* **deploy.yaml** ist hier geschrieben, nicht aus dem Quell-Repo kopiert. Die Werte
  stammen aus der Trainings-Config desselben Commits
  (`agile/rl_env/tasks/locomotion_height/g1/velocity_height_env_cfg.py`,
  `G1VelocityHeightHistoryEnvCfg`; `agile/rl_env/assets/robots/unitree_g1.py`,
  `G1_29DOF` und `G1_ACTION_SCALE_LOWER`). Nur `stand_pose_legs`, `rescue_policy` und
  die Begrenzung von `lin_vel_x` auf 1.0 m/s sind eigene Festlegungen (begruendet in
  der deploy.yaml).

## Pruefsummen

| Datei | sha256 |
|---|---|
| Quelle `unitree_g1_velocity_height_history_torchscript.pt` | `240a5ce0b121837eba2f886a523d284a2a263dceb78d3132639eaf74ad7650f2` |
| `policy.onnx` (hier) | `250817ec5666578a69da4e3238d9abec5abb0cc3edecd07857b3fa4b0696a3f7` |

Die Quell-sha256 steht ausserdem in den ONNX-Metadaten (`source_sha256`).

## Konvertierung (reproduzierbar)

```bash
curl -L -o /tmp/agile_velocity_height.pt \
  https://media.githubusercontent.com/media/nvidia-isaac/WBC-AGILE/6830cf995714e81c91ce63247e8e016d36e28f14/agile/data/policy/velocity_height_g1/unitree_g1_velocity_height_history_torchscript.pt
# braucht torch + onnx + onnxruntime (CPU), nur fuer die Konvertierung
python3 export_onnx.py /tmp/agile_velocity_height.pt policy.onnx
```

`export_onnx.py` prueft die Quell-sha256 und dass Obs-Normalizer und Ausgangsstufe
Identitaeten sind, baut das MLP (400 -> 512 -> 256 -> 128 -> 12, ELU) mit denselben
Gewichten nach, exportiert (opset 17) und vergleicht TorchScript und ONNX auf 2000
Zufallseingaben: max. Abweichung 4.6e-5.

## Pruefung gegen den offiziellen NVIDIA-Export

Das AGILE-Repo enthaelt fuer diese Policy zusaetzlich einen LEAPP-Export
(`agile/data/policy/velocity_height_g1/leapp/`, ONNX **mit eingebauter
Obs-Verarbeitung**: rohe Gelenkwinkel, Quaternion, Gyro rein, Gelenk-Sollwerte und
Gains raus). `test/test_walk_policy.py::test_vh_matches_official_leapp_export` prueft
die ganze Kette dieses Ordners (Obs-Aufbau aus `deploy.yaml`, History, Netz,
Aktions-Skalierung, Gelenk-Zuordnung) gegen feste Sollwerte, die mit dem LEAPP-Export
erzeugt wurden (Abweichung < 2e-4 rad). Dabei fiel auf: die Knoechel laufen mit
Aktions-Skalierung 1.0, nicht 0.25 * 50 / 20 (siehe `action_scale` in der deploy.yaml).

## Was die Policy erwartet

50 Hz (step_dt 0.02). Gesteuert werden nur die 12 Beingelenke (Isaac-Lab-Reihenfolge,
`joint_names`); die Taille haelt loco_sim auf 0 (kp 300, kd 5) wie im Training. Ziel:
`q = default_joint_pos + action_scale * clip(action, -6, 6)`.

Obs (400) = 6 Terme mit je 5 Zeitschritten, **Term fuer Term**, innerhalb eines Terms
aelteste zuerst. Nach einem Reset fuellt der erste Messwert alle 5 Slots.

| Bereich | Groesse | Inhalt |
|---|---|---|
| 0:20    | 5 x 4  | velocity_height_commands (vx, vy [m/s], vyaw [rad/s], Becken-Hoehe 0.72 m) |
| 20:35   | 5 x 3  | base_ang_vel (Gyro, Body-Frame) |
| 35:50   | 5 x 3  | projected_gravity (aufrecht = [0, 0, -1]) |
| 50:195  | 5 x 29 | joint_pos_rel aller 29 Gelenke (inkl. Arme) |
| 195:340 | 5 x 29 | joint_vel aller 29 Gelenke x 0.1 |
| 340:400 | 5 x 12 | last_action (roh, auf +-10 begrenzt) |

Trainiert fuer vx in [-0.5, 1.5], vy in [-0.5, 0.5] m/s und vyaw in [-1, 1] rad/s,
ohne Harness (History-Variante). Die Arme wurden im Training nur im Stand
zufaellig bewegt, beim Laufen standen sie still; die Policy SIEHT sie aber und kommt in
der Sim auch mit dauernd bewegten Armen und einer Box (1 kg je Hand) zurecht
(test_agile_walk_sim.py).

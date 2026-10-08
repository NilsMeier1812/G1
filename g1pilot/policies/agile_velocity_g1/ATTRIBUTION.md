# Herkunft & Lizenz der AGILE-Lauf-Policy

`policy.onnx` ist die Lauf-Policy **Velocity-G1-History-v0** aus NVIDIAs
[`nvidia-isaac/WBC-AGILE`](https://github.com/nvidia-isaac/WBC-AGILE), Commit
`6830cf995714e81c91ce63247e8e016d36e28f14`, Datei
`agile/data/policy/velocity_g1/unitree_g1_velocity_history_torchscript.pt`.

* **Lizenz:** Apache License 2.0, Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES
  (Text in `LICENSE`, aus `LICENCE` Abschnitt B des Quell-Repos; das Repo hat keine
  NOTICE-Datei). Erlaubt Nutzung, Aenderung und Weitergabe, auch kommerziell, solange
  Lizenztext und Hinweise mitgegeben werden.
* **Geaendert (Apache-2.0 §4b):** Das Format wurde von TorchScript nach ONNX
  konvertiert. Gewichte und Rechnung sind unveraendert (siehe unten).
* **deploy.yaml** ist hier geschrieben, nicht aus dem Quell-Repo kopiert. Jeder Wert
  stammt aus der Trainings-Config desselben Commits:
  `agile/rl_env/tasks/locomotion/g1/velocity_history_env_cfg.py` (Obs, Aktionen,
  Kommandos) und `agile/rl_env/assets/robots/unitree_g1.py`
  (`G1_29DOF_DELAYED_DC_MOTOR`: Default-Pose, kp/kd).

## Pruefsummen

| Datei | sha256 |
|---|---|
| Quelle `unitree_g1_velocity_history_torchscript.pt` | `f58db6f61c7546941fe814ac6f571a11e894260e9e638f99eadd9125a4e8ab12` |
| `policy.onnx` (hier) | `b621928331d4b9ce9aea262ffc6cb31ca257b9310ac7b5dfff95f03c67d43ba4` |

Die Quell-sha256 steht ausserdem in den ONNX-Metadaten (`source_sha256`).

## Konvertierung (reproduzierbar)

```bash
# Quelle (Git-LFS-Datei) holen
curl -L -o /tmp/agile_velocity.pt \
  https://media.githubusercontent.com/media/nvidia-isaac/WBC-AGILE/6830cf995714e81c91ce63247e8e016d36e28f14/agile/data/policy/velocity_g1/unitree_g1_velocity_history_torchscript.pt
# braucht torch + onnx + onnxruntime (CPU), nur fuer die Konvertierung
python3 export_onnx.py /tmp/agile_velocity.pt policy.onnx
```

`export_onnx.py` prueft die Quell-sha256 und dass der Obs-Normalizer der Datei eine
Identitaet ist, baut das MLP (255 -> 256 -> 256 -> 128 -> 14, ELU) mit denselben
Gewichten nach, exportiert (opset 17) und vergleicht TorchScript und ONNX auf 2000
Zufallseingaben: max. Abweichung 2.9e-6. `test/test_walk_policy.py` prueft die
ONNX-Datei zusaetzlich gegen feste Referenz-Ausgaben der Original-Datei.

## Was die Policy erwartet

50 Hz (step_dt 0.02). Gesteuert werden 14 Gelenke (Beine + Taille roll/pitch) in
Isaac-Lab-Reihenfolge (`joint_names` in `deploy.yaml`); `waist_yaw` haelt loco_sim
auf 0 (kp 300, kd 5) wie im Training. Ziel: `q = default_joint_pos + 0.5 * action`,
PD je Gelenk mit `stiffness`/`damping`.

Obs (255) = 6 Terme mit je 5 Zeitschritten, **Term fuer Term**, innerhalb eines Terms
aelteste zuerst. Nach einem Reset fuellt der erste Messwert alle 5 Slots.

| Bereich | Groesse | Inhalt |
|---|---|---|
| 0:15    | 5 x 3  | base_ang_vel (Gyro, Body-Frame) x 0.2 |
| 15:30   | 5 x 3  | projected_gravity (aufrecht = [0, 0, -1]) |
| 30:45   | 5 x 3  | velocity_command (vx, vy [m/s], vyaw [rad/s]); ‖cmd‖ < 0.1 -> 0 |
| 45:115  | 5 x 14 | joint_pos_rel = q - default_joint_pos |
| 115:185 | 5 x 14 | joint_vel x 0.05 |
| 185:255 | 5 x 14 | last_action (roh) |

Trainiert fuer vx, vy in [-0.5, 0.5] m/s und vyaw in [-1, 1] rad/s, mit 25 %
stehenden Umgebungen und **staendig zufaellig bewegten Armen** (die Arme gehoeren
nicht zur Policy). Deshalb braucht sie keine feste Lauf-Pose der Arme.

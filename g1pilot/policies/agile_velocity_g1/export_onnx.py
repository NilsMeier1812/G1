#!/usr/bin/env python3
"""Konvertiert die AGILE-TorchScript-Policy nach ONNX und prueft die Gleichheit.

Nur fuer die einmalige (reproduzierbare) Erzeugung von policy.onnx noetig, NICHT
zur Laufzeit. Braucht torch + onnx + onnxruntime (CPU reicht).

Quelle (Git-LFS-Datei im AGILE-Repo, Commit 6830cf9):
  agile/data/policy/velocity_g1/unitree_g1_velocity_history_torchscript.pt
  https://media.githubusercontent.com/media/nvidia-isaac/WBC-AGILE/6830cf995714e81c91ce63247e8e016d36e28f14/agile/data/policy/velocity_g1/unitree_g1_velocity_history_torchscript.pt

Aufruf:  python3 export_onnx.py <pfad/zur/torchscript.pt> [ausgabe.onnx]

Das TorchScript-Modul ist  normalizer (Identitaet) -> actor (MLP 255-256-256-128-14,
ELU). Es wird als normales nn.Sequential mit denselben Gewichten nachgebaut und
exportiert; anschliessend werden TorchScript und ONNX auf Zufallseingaben verglichen.
"""
import hashlib
import os
import sys

import numpy as np
import onnx
import onnxruntime as ort
import torch

EXPECTED_SHA256 = "f58db6f61c7546941fe814ac6f571a11e894260e9e638f99eadd9125a4e8ab12"
NUM_OBS, NUM_ACT = 255, 14


def main():
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "policy.onnx")

    sha = hashlib.sha256(open(src, "rb").read()).hexdigest()
    if sha != EXPECTED_SHA256:
        raise SystemExit(f"Unerwartete Quelldatei (sha256 {sha}), erwartet {EXPECTED_SHA256}.")

    ts = torch.jit.load(src, map_location="cpu").eval()
    # Der Normalizer ist im Export eine reine Identitaet (kein Obs-Normalizer trainiert);
    # das wird hier geprueft statt angenommen.
    x = torch.randn(64, NUM_OBS)
    if not torch.equal(ts.normalizer(x), x):
        raise SystemExit("Normalizer ist keine Identitaet -- Export muesste ihn mitnehmen.")

    layers = ts.actor.layers
    net = torch.nn.Sequential(
        torch.nn.Linear(NUM_OBS, 256), torch.nn.ELU(),
        torch.nn.Linear(256, 256), torch.nn.ELU(),
        torch.nn.Linear(256, 128), torch.nn.ELU(),
        torch.nn.Linear(128, NUM_ACT),
    ).eval()
    with torch.no_grad():
        for i in (0, 2, 4, 6):
            src_lin = getattr(layers, str(i))
            net[i].weight.copy_(src_lin.weight)
            net[i].bias.copy_(src_lin.bias)
        for i in (1, 3, 5):
            if getattr(layers, str(i)).original_name != "ELU":
                raise SystemExit(f"Schicht {i} ist keine ELU.")
        ref = ts(x)
        err = (net(x) - ref).abs().max().item()
    if err > 1e-6:
        raise SystemExit(f"Nachbau weicht vom TorchScript ab (max {err:g}).")

    torch.onnx.export(net, torch.zeros(1, NUM_OBS), dst, input_names=["obs"],
                      output_names=["actions"], opset_version=17, dynamo=False)
    model = onnx.load(dst)
    for k, v in {
        "source": "nvidia-isaac/WBC-AGILE@6830cf9 agile/data/policy/velocity_g1/"
                  "unitree_g1_velocity_history_torchscript.pt",
        "source_sha256": sha,
        "task": "Velocity-G1-History-v0",
        "license": "Apache-2.0 (NVIDIA CORPORATION & AFFILIATES)",
    }.items():
        p = model.metadata_props.add()
        p.key, p.value = k, v
    onnx.save(model, dst)

    sess = ort.InferenceSession(dst, providers=["CPUExecutionProvider"])
    xs = np.random.default_rng(0).normal(size=(2000, NUM_OBS)).astype(np.float32) * 2.0
    out_onnx = np.concatenate([sess.run(None, {"obs": xs[i:i + 1]})[0] for i in range(len(xs))])
    with torch.no_grad():
        out_ts = ts(torch.from_numpy(xs)).numpy()
    err = float(np.abs(out_onnx - out_ts).max())
    print(f"ONNX geschrieben: {dst}")
    print(f"Vergleich TorchScript vs. ONNX auf {len(xs)} Zufallseingaben: max |diff| = {err:.2e}")
    if err > 1e-4:
        raise SystemExit("ONNX weicht vom TorchScript ab.")
    print("sha256(policy.onnx) =", hashlib.sha256(open(dst, "rb").read()).hexdigest())


if __name__ == "__main__":
    main()

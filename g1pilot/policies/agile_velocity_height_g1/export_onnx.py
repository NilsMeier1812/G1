#!/usr/bin/env python3
"""Konvertiert die AGILE-TorchScript-Policy Velocity-Height-G1-History-v0 nach ONNX
und prueft die Gleichheit.

Nur fuer die einmalige (reproduzierbare) Erzeugung von policy.onnx noetig, NICHT
zur Laufzeit. Braucht torch + onnx + onnxruntime (CPU reicht).

Quelle (Git-LFS-Datei im AGILE-Repo, Commit 6830cf9):
  agile/data/policy/velocity_height_g1/unitree_g1_velocity_height_history_torchscript.pt
  https://media.githubusercontent.com/media/nvidia-isaac/WBC-AGILE/6830cf995714e81c91ce63247e8e016d36e28f14/agile/data/policy/velocity_height_g1/unitree_g1_velocity_height_history_torchscript.pt

Aufruf:  python3 export_onnx.py <pfad/zur/torchscript.pt> [ausgabe.onnx]

Das TorchScript-Modul ist  obs_normalizer (Identitaet) -> mlp (400-512-256-128-12,
ELU) -> deterministic_output (Identitaet). Es wird als nn.Sequential mit denselben
Gewichten nachgebaut und exportiert; anschliessend werden TorchScript und ONNX auf
Zufallseingaben verglichen.
"""
import hashlib
import os
import sys

import numpy as np
import onnx
import onnxruntime as ort
import torch

EXPECTED_SHA256 = "240a5ce0b121837eba2f886a523d284a2a263dceb78d3132639eaf74ad7650f2"
NUM_OBS, NUM_ACT = 400, 12
HIDDEN = (512, 256, 128)


def main():
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "policy.onnx")

    sha = hashlib.sha256(open(src, "rb").read()).hexdigest()
    if sha != EXPECTED_SHA256:
        raise SystemExit(f"Unerwartete Quelldatei (sha256 {sha}), erwartet {EXPECTED_SHA256}.")

    ts = torch.jit.load(src, map_location="cpu").eval()
    # Normalizer und Ausgangsstufe sind im Export reine Identitaeten; das wird hier
    # geprueft statt angenommen.
    x = torch.randn(64, NUM_OBS)
    if not torch.equal(ts.obs_normalizer(x), x):
        raise SystemExit("obs_normalizer ist keine Identitaet -- Export muesste ihn mitnehmen.")
    y = torch.randn(64, NUM_ACT)
    if not torch.equal(ts.deterministic_output(y), y):
        raise SystemExit("deterministic_output ist keine Identitaet.")

    dims = (NUM_OBS,) + HIDDEN + (NUM_ACT,)
    layers = []
    for i in range(len(dims) - 1):
        layers.append(torch.nn.Linear(dims[i], dims[i + 1]))
        if i < len(dims) - 2:
            layers.append(torch.nn.ELU())
    net = torch.nn.Sequential(*layers).eval()
    with torch.no_grad():
        for i in range(0, len(layers), 2):
            src_lin = getattr(ts.mlp, str(i))
            net[i].weight.copy_(src_lin.weight)
            net[i].bias.copy_(src_lin.bias)
        for i in range(1, len(layers), 2):
            if getattr(ts.mlp, str(i)).original_name != "ELU":
                raise SystemExit(f"Schicht {i} ist keine ELU.")
        ref = ts(x)
        err = (net(x) - ref).abs().max().item()
    if err > 1e-6:
        raise SystemExit(f"Nachbau weicht vom TorchScript ab (max {err:g}).")

    torch.onnx.export(net, torch.zeros(1, NUM_OBS), dst, input_names=["obs"],
                      output_names=["actions"], opset_version=17, dynamo=False)
    model = onnx.load(dst)
    for k, v in {
        "source": "nvidia-isaac/WBC-AGILE@6830cf9 agile/data/policy/velocity_height_g1/"
                  "unitree_g1_velocity_height_history_torchscript.pt",
        "source_sha256": sha,
        "task": "Velocity-Height-G1-History-v0",
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

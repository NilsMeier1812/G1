#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
install_example_sequences.py — Beispiel-Ablaeufe fuer die Szene A5 in den
Positionsspeicher (pose_store) schreiben.

Jede Kategorie "Ablauf <Name>" erscheint in der Demo-GUI (Greifen) als eigener
Knopf "▶ <Name>"; die Posen laufen in Namensreihenfolge ab (AP1_01, AP1_02, ...).
Reine Hand-Schritte (z.B. "Links_schliessen") melden nach hand_only_settle_s
"reached" (arm_controller), danach geht es weiter.

  Ablauf Arbeitsplatz 1 (Station "Arbeitsplatz 1", duenner gruener Zylinder):
    Sichere Pose -> links ueber den Tisch (Daumen eingedreht) -> absenken ->
    schliessen -> anheben  (keine Uebergabe an rechts: Griff zu instabil)
  Ablauf Arbeitsplatz 2 (Station "Arbeitsplatz 2", blaue KLT):
    Sichere Pose (Haende flach) -> seitlich neben die KLT (Handmitte ~ Wand-
    mitte) -> zudruecken -> direkt anheben (kein Finger-Schliessen)

Die Gelenkwinkel sind per IK auf dem MuJoCo-Modell (G1 + Inspire) berechnet,
kollisionsfrei (Tisch + Selbstkollision) und gelten fuer den Roboter GENAU an
der jeweiligen Station (AUTO NAV -> Station). "Sichere_Pose" (Ellbogen nach
hinten, Haende seitlich neben dem Koerper ueber Tischhoehe, nichts ragt ueber
die Tischkante) liegt zusaetzlich einzeln in "Allgemein".

Aufruf (Host oder Container):
    python3 -m g1pilot.tools.install_example_sequences
    python3 g1pilot/g1pilot/tools/install_example_sequences.py
Ablage: wie pose_store (G1_POSE_STORE > Repo-Mount > <repo>/g1pilot/data/).
Vorhandene Posen gleichen Namens werden ueberschrieben, andere bleiben.
"""
import os
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from g1pilot.manipulation import pose_store as ps  # noqa: E402

OPEN = [1000] * 6
SAVE_LEFT = [1.449, 0.426, 0.641, -0.374, -0.154, -1.008, -0.086]
SAVE_RIGHT = [1.449, -0.426, -0.641, -0.374, 0.154, -1.008, 0.086]

# (Name, Kategorie, Komponenten) -- Hand: Inspire 0..1000 (1000 = offen),
# Reihenfolge kleiner/Ring/Mittel/Zeige/Daumen-Beugung/Daumen-Rotation.
POSES = [
    ("Sichere_Pose", ps.DEFAULT_CATEGORY,
     {"left_arm": SAVE_LEFT, "right_arm": SAVE_RIGHT, "left_hand": OPEN, "right_hand": OPEN}),
    # ── Ablauf Arbeitsplatz 1 ───────────────────────────────────────
    ('AP1_01_Sichere_Pose', 'Ablauf Arbeitsplatz 1', {"left_arm": [1.449, 0.426, 0.641, -0.374, -0.154, -1.008, -0.086], "right_arm": [1.449, -0.426, -0.641, -0.374, 0.154, -1.008, 0.086], "left_hand": [1000, 1000, 1000, 1000, 1000, 1000], "right_hand": [1000, 1000, 1000, 1000, 1000, 1000]}),
    ('AP1_02_Links_ueber_Tisch', 'Ablauf Arbeitsplatz 1', {"left_arm": [-0.8633, 0.0742, 0.1242, 0.365, -0.0604, 0.4799, 0.0992], "left_hand": [1000, 1000, 1000, 1000, 1000, 0]}),
    ('AP1_03_Links_absenken', 'Ablauf Arbeitsplatz 1', {"left_arm": [-0.8663, 0.0374, 0.1888, 1.295, 0.0216, -0.3539, 0.0691]}),
    ('AP1_04_Links_schliessen', 'Ablauf Arbeitsplatz 1', {"left_hand": [0, 0, 0, 0, 500, 0]}),
    ('AP1_05_Anheben', 'Ablauf Arbeitsplatz 1', {"left_arm": [-0.7827, 0.0239, 0.1583, 0.8309, 0.0119, -0.0608, 0.0652]}),
    # ── Ablauf Arbeitsplatz 2 ───────────────────────────────────────
    ('AP2_01_Sichere_Pose', 'Ablauf Arbeitsplatz 2', {"left_arm": [1.449, 0.426, 0.641, -0.374, -0.154, -1.008, -0.086], "right_arm": [1.449, -0.426, -0.641, -0.374, 0.154, -1.008, 0.086], "left_hand": [1000, 1000, 1000, 1000, 1000, 1000], "right_hand": [1000, 1000, 1000, 1000, 1000, 1000]}),
    ('AP2_02_Neben_die_KLT', 'Ablauf Arbeitsplatz 2', {"left_arm": [0.1203, 0.3142, 0.2229, 0.3532, -0.3453, -0.3504, -0.3593], "right_arm": [0.1201, -0.314, -0.2231, 0.3538, 0.3454, -0.3506, 0.3596]}),
    ('AP2_03_Zudruecken', 'Ablauf Arbeitsplatz 2', {"left_arm": [0.2099, 0.0102, -0.0348, 0.0975, -0.0109, -0.2778, -0.0266], "right_arm": [0.2098, -0.0101, 0.0345, 0.0978, 0.011, -0.2781, 0.0269]}),
    ('AP2_04_KLT_anheben', 'Ablauf Arbeitsplatz 2', {"left_arm": [0.2502, 0.0268, -0.0542, -0.3667, -0.0216, 0.1415, -0.0066], "right_arm": [0.2502, -0.0267, 0.054, -0.3665, 0.0218, 0.1413, 0.0068]}),
]


def main():
    if not os.environ.get(ps.PATH_ENV_VAR) and not Path(ps.CONTAINER_REPO_MOUNT).is_dir():
        # Host ohne Mount: dieselbe Datei wie im Container (Repo-Bind-Mount).
        os.environ[ps.PATH_ENV_VAR] = str(
            Path(__file__).resolve().parents[2] / ps.REPO_DATA_SUBDIR / ps.STORE_FILENAME)
    store = ps.PoseStore()
    # Die Ablauf-Kategorien gehoeren diesem Skript: alte Schritte entfernen,
    # sonst liefen umbenannte/weggefallene Schritte im Ablauf weiter mit.
    own = {c for _, c, _ in POSES if c.startswith("Ablauf ")}
    for cat, names in store.list_grouped().items():
        if cat in own:
            for n in names:
                store.delete(n)
    for name, category, comp in POSES:
        store.save(name, category=category, **comp)
    print(f"{len(POSES)} Posen gespeichert in {ps.default_store_path()}")


if __name__ == "__main__":
    main()

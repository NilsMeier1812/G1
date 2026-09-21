#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
#  demo_sequence.py — gespeicherte Posen einer Kategorie nacheinander abfahren.
#
#  Nimmt alle Posen der Kategorie (Default "Demo") aus data/arm_poses.json in
#  alphabetischer Namensreihenfolge (darum Namen wie Demo_1_..., Demo_2_...) und
#  schickt sie ueber die Arm-API (POST /arm/joints) an den arm_controller. Jeder
#  Schritt wird GEPLANT (um Tisch/Koerper herum) und das Skript wartet, bis er
#  erreicht ist, bevor der naechste startet.
#
#  Laeuft auf dem Host (nur Python-Standardbibliothek; die Arm-API lauscht im
#  Container auf 127.0.0.1:8770, network_mode host).
#
#    python3 demo_sequence.py                 # Kategorie "Demo" einmal
#    python3 demo_sequence.py --loop 3        # dreimal hintereinander
#    python3 demo_sequence.py --category Box_modus --pause 2
#    python3 demo_sequence.py --dry-run       # nur anzeigen, nichts senden
#
#  Vorher im Sim: START -> START BALANCING (nicht WALK: da haengen die Arme
#  fest in der Lauf-Pose). ENABLE MANIPULATION ist im Sim nicht noetig.
# ════════════════════════════════════════════════════════════════════════
import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
POSES_FILE = HERE / "data" / "arm_poses.json"
STEP_TIMEOUT_S = 90       # Planung + Fahrt eines Schritts
HAND_SETTLE_S = 1.5       # Finger brauchen nach einem reinen Hand-Schritt etwas Zeit


def api(url, method="GET", body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    token = os.environ.get("ARM_API_TOKEN")
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(req, timeout=STEP_TIMEOUT_S + 10) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")
    except (urllib.error.URLError, OSError) as e:
        # Kein Server am Port (Sim aus / arm_api noch nicht hochgefahren) oder
        # Verbindung weg -- als normalen Fehler melden, nicht als Traceback.
        return 0, {"error": f"keine Verbindung zu {url} ({e})"}


def load_steps(category):
    doc = json.loads(POSES_FILE.read_text(encoding="utf-8"))
    steps = [(n, p) for n, p in doc.get("poses", {}).items() if p.get("category") == category]
    return sorted(steps, key=lambda s: s[0].lower())


def main():
    ap = argparse.ArgumentParser(description="Posen einer Kategorie als Demo abfahren")
    ap.add_argument("--category", default="Demo")
    ap.add_argument("--pause", type=float, default=1.0, help="Pause zwischen Schritten [s]")
    ap.add_argument("--loop", type=int, default=1, help="Wie oft die ganze Folge laufen soll")
    ap.add_argument("--url", default=os.environ.get("ARM_API_URL", "http://127.0.0.1:8770"))
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    steps = load_steps(args.category)
    if not steps:
        sys.exit(f"Keine Posen in Kategorie '{args.category}' ({POSES_FILE}).")
    print(f"Kategorie '{args.category}': " + " -> ".join(n for n, _ in steps))
    if args.dry_run:
        return

    code, health = api(f"{args.url}/arm/health")
    if code != 200:
        sys.exit(f"Arm-API nicht erreichbar: {health.get('error') or f'{code} {health}'}\n"
                 f"  -> Sim starten (./start.sh) und im Startmenue START + START BALANCING\n"
                 f"  -> laeuft sie schon? Test:  curl {args.url}/arm/health")

    # Letzte Arm-Ziele merken: ein reiner Hand-Schritt (z.B. "Hand zu") braucht
    # trotzdem ein Arm-Ziel -> dieselbe Stellung nochmal (Fahrt der Laenge 0).
    _, state = api(f"{args.url}/arm/state")
    arms = {s: v for s, v in (state.get("joints") or {}).items() if s in ("left", "right")}

    for rnd in range(1, args.loop + 1):
        if args.loop > 1:
            print(f"\n=== Durchlauf {rnd}/{args.loop} ===")
        for i, (name, pose) in enumerate(steps, 1):
            body = {"wait": STEP_TIMEOUT_S}
            for side in ("left", "right"):
                if f"{side}_arm" in pose:
                    arms[side] = pose[f"{side}_arm"]
            hand_only = not any(f"{s}_arm" in pose for s in ("left", "right"))
            body.update({s: arms[s] for s in ("left", "right") if s in arms})
            hands = {s: pose[f"{s}_hand"] for s in ("left", "right") if f"{s}_hand" in pose}
            if hands:
                body["hands"] = hands
            if not any(s in body for s in ("left", "right")):
                print(f"[{i}/{len(steps)}] {name}: uebersprungen (keine Arm-Stellung bekannt)")
                continue
            t0 = time.time()
            print(f"[{i}/{len(steps)}] {name} ...", end=" ", flush=True)
            code, res = api(f"{args.url}/arm/joints", "POST", body)
            state = res.get("state")
            if state != "reached":
                print(f"ABBRUCH: {state or code} {res.get('reason') or res.get('error', '')}")
                sys.exit(1)
            print(f"erreicht ({time.time() - t0:.1f} s)")
            time.sleep(HAND_SETTLE_S if hand_only else args.pause)
    print("Demo fertig.")


if __name__ == "__main__":
    main()

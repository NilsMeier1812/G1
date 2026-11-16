#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
winken.py -- Beispielprojekt fuer die Erweiterbarkeit des G1-Stacks.

Ein eigenstaendiges Programm AUSSERHALB von g1pilot/, das nur ueber die
dokumentierte Arm-Schnittstelle (HTTP/JSON, g1pilot/docs/21-A-Arm-API.md)
eine Armbewegung ausloest: der rechte Arm hebt sich, winkt zweimal mit dem
Ellbogen und kehrt in die Ausgangsstellung zurueck.

Nur Python-Standardbibliothek, kein ROS, kein Import aus g1pilot.
Sicherheitsgates (NOT-HALT, Arme uebernommen, Kollision, Grenzen) gelten
unveraendert -- sie liegen im arm_controller, nicht hier.

Aufruf (Stack laeuft, Arme uebernommen = GREIFEN):
    python3 beispiele/arm_winken/winken.py [--url http://localhost:8770] [--token TOKEN]
"""
import argparse
import json
import sys
import urllib.error
import urllib.request

ARM_HOCH = [-1.2, -0.3, 0.0, 0.3, 0.0, 0.0, 0.0]      # rechter Arm seitlich vorn oben
WINK_A = [-1.2, -0.3, 0.0, 0.9, 0.0, 0.0, 0.0]        # Ellbogen gebeugt
WINK_B = [-1.2, -0.3, 0.0, 0.2, 0.0, 0.0, 0.0]        # Ellbogen gestreckt


def request(url, method="GET", body=None, token=None, timeout=330):
    """Ein HTTP-Aufruf der Arm-API -> (status, json)."""
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("Content-Type", "application/json")
    if token:
        req.add_header("X-Auth-Token", token)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8") or "{}")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode("utf-8") or "{}")
        except Exception:
            return e.code, {}


def fahre(base, token, right, wait=60):
    st, d = request(f"{base}/arm/joints", "POST", {"right": right, "wait": wait}, token)
    print(f"  /arm/joints {right} -> HTTP {st} {d.get('state', '')} {d.get('reason', '')}".rstrip())
    return st == 200


def main(argv=None):
    ap = argparse.ArgumentParser(description="Beispiel: rechter Arm winkt ueber die Arm-API.")
    ap.add_argument("--url", default="http://localhost:8770")
    ap.add_argument("--token", default=None, help="X-Auth-Token, falls arm_api_token gesetzt ist")
    ap.add_argument("--mal", type=int, default=2, help="wie oft winken")
    a = ap.parse_args(argv)
    base = a.url.rstrip("/")

    st, d = request(f"{base}/arm/health", token=a.token, timeout=5)
    if st != 200:
        print(f"Arm-API nicht erreichbar ({base}, HTTP {st}). Laeuft der Stack?")
        return 1
    st, d = request(f"{base}/arm/state", token=a.token, timeout=5)
    start = ((d or {}).get("joints") or {}).get("right")      # GET /arm/state: {"joints": {"right": [7]}}
    print(f"Ausgangsstellung rechts: {start}")

    ok = fahre(base, a.token, ARM_HOCH)
    for _ in range(a.mal if ok else 0):
        ok = fahre(base, a.token, WINK_A) and fahre(base, a.token, WINK_B)
        if not ok:
            break
    if start and len(start) == 7:
        fahre(base, a.token, [float(x) for x in start])
    print("fertig." if ok else "abgebrochen (siehe HTTP-Status oben).")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())

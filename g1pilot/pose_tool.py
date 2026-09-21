#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
#  pose_tool.py — gespeicherte Posen auflisten, loeschen, umbenennen.
#
#  Der Streamdeck kann Posen nur SPEICHERN und ANFAHREN. Aufraeumen (loeschen,
#  umbenennen, in eine andere Kategorie schieben) geht hiermit -- direkt auf
#  derselben Datei data/arm_poses.json, die auch der arm_controller nutzt.
#  Er liest sie bei jedem Zugriff neu: Aenderungen wirken ohne Neustart.
#
#    python3 pose_tool.py list                      # alles anzeigen
#    python3 pose_tool.py list --category Demo
#    python3 pose_tool.py delete Demo_5_switch Demo_5_switch_over
#    python3 pose_tool.py rename Demo_7_switch Demo_5_switch
#    python3 pose_tool.py move Left_up --category Demo
#
#  Laeuft auf dem Host (nur Standardbibliothek). Tipp fuer die Demo-Reihenfolge:
#  demo_sequence.py faehrt eine Kategorie in ALPHABETISCHER Namensreihenfolge ab
#  -- darum die Nummern in Demo_1_..., Demo_2_... (umbenennen = umsortieren).
# ════════════════════════════════════════════════════════════════════════
import argparse
import json
import sys
from pathlib import Path

POSES_FILE = Path(__file__).resolve().parent / "data" / "arm_poses.json"
COMPONENTS = ("left_arm", "right_arm", "left_hand", "right_hand")


def load():
    if not POSES_FILE.is_file():
        sys.exit(f"Keine Posen-Datei: {POSES_FILE}")
    return json.loads(POSES_FILE.read_text(encoding="utf-8"))


def save(doc):
    POSES_FILE.write_text(json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")


def cmd_list(doc, args):
    poses = doc.get("poses", {})
    cats = sorted({p.get("category", "") for p in poses.values()})
    for cat in cats:
        if args.category and cat != args.category:
            continue
        names = sorted((n for n, p in poses.items() if p.get("category", "") == cat), key=str.lower)
        print(f"\n[{cat or 'ohne Kategorie'}]  {len(names)} Pose(n)")
        for n in names:
            parts = [c.replace("_arm", "-Arm").replace("_hand", "-Hand")
                     for c in COMPONENTS if c in poses[n]]
            print(f"   {n:28s} {', '.join(parts)}")


def cmd_delete(doc, args):
    for name in args.names:
        if doc["poses"].pop(name, None) is None:
            print(f"   {name}: nicht gefunden")
        else:
            print(f"   {name}: geloescht")
    save(doc)


def cmd_rename(doc, args):
    if args.name not in doc["poses"]:
        sys.exit(f"'{args.name}' gibt es nicht.")
    if args.new in doc["poses"]:
        sys.exit(f"'{args.new}' gibt es schon.")
    doc["poses"][args.new] = doc["poses"].pop(args.name)
    save(doc)
    print(f"   {args.name} -> {args.new}")


def cmd_move(doc, args):
    if args.name not in doc["poses"]:
        sys.exit(f"'{args.name}' gibt es nicht.")
    doc["poses"][args.name]["category"] = args.category
    if args.category not in doc.get("categories", []):
        doc.setdefault("categories", []).append(args.category)
    save(doc)
    print(f"   {args.name} -> Kategorie '{args.category}'")


def main():
    ap = argparse.ArgumentParser(description="Gespeicherte Arm-Posen verwalten")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("list", help="Posen anzeigen"); p.add_argument("--category"); p.set_defaults(fn=cmd_list)
    p = sub.add_parser("delete", help="Posen loeschen"); p.add_argument("names", nargs="+"); p.set_defaults(fn=cmd_delete)
    p = sub.add_parser("rename", help="Pose umbenennen"); p.add_argument("name"); p.add_argument("new"); p.set_defaults(fn=cmd_rename)
    p = sub.add_parser("move", help="Pose in eine andere Kategorie schieben")
    p.add_argument("name"); p.add_argument("--category", required=True); p.set_defaults(fn=cmd_move)
    args = ap.parse_args()
    args.fn(load(), args)


if __name__ == "__main__":
    main()

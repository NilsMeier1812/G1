#!/usr/bin/env bash
# =====================================================================
# Hand-Profile fuer die Sim (eigene starre Haende, z. B. gedruckte Haken).
# Nutzt das virtualenv des Szenen-Editors (trimesh, vhacdx, mujoco):
#   einmalig  unitree_mujoco/scene_editor/setup.sh
#
#   ./hand_tool.sh list                 Profile auflisten
#   ./hand_tool.sh build   <profil>     Modell (MuJoCo + URDF) neu erzeugen
#   ./hand_tool.sh preview <profil>     Vorschau; hand.yaml speichern = neu laden
#   ./hand_tool.sh snapshot <profil> [bild.png]   Vorschau als Bild
#
# Anleitung: g1pilot/docs/25-A-Eigene-Haende.md
# =====================================================================
set -euo pipefail
cd "$(dirname "$0")"
VENV_PY="../../../scene_editor/.venv/bin/python"
PY="python3"
if [[ -x "$VENV_PY" ]]; then PY="$VENV_PY"; fi

cmd="${1:-}"; shift || true
case "$cmd" in
  list)     exec "$PY" gen_hand_profile.py --list ;;
  build)    exec "$PY" gen_hand_profile.py "$@" ;;
  preview)  exec "$PY" hand_preview.py "$@" ;;
  snapshot) exec "$PY" hand_preview.py "$1" --snapshot "${2:-hand_${1}_vorschau.png}" ;;
  *) sed -n '2,13p' "$0"; exit 1 ;;
esac

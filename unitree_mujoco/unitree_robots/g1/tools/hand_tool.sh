#!/usr/bin/env bash
# =====================================================================
# Hand-Profile fuer die Sim (eigene starre Haende, z. B. gedruckte Haken).
# Nutzt das virtualenv des Szenen-Editors (trimesh, vhacdx, mujoco):
#   einmalig  unitree_mujoco/scene_editor/setup.sh
#
#   ./hand_tool.sh list                 Profile auflisten
#   ./hand_tool.sh build   <profil>     Modell (MuJoCo + URDF) neu erzeugen
#   ./hand_tool.sh adjust  <profil>     Einpassen mit Schiebern (live), Speichern -> hand.yaml
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
# Fuer "adjust" braucht es zusaetzlich tkinter: das venv-Python nehmen, wenn es
# tkinter hat, sonst ein System-Python mit mujoco + trimesh + tkinter.
pick_tk_python() {
  local c
  for c in "$PY" python3 python3.12 python3.11 python3.10; do
    command -v "$c" >/dev/null 2>&1 || [[ -x "$c" ]] || continue
    if "$c" -c "import tkinter, mujoco, trimesh, yaml" 2>/dev/null; then PY="$c"; return 0; fi
  done
  return 1
}

cmd="${1:-}"; shift || true
case "$cmd" in
  list)     exec "$PY" gen_hand_profile.py --list ;;
  build)    exec "$PY" gen_hand_profile.py "$@" ;;
  adjust)
    if ! pick_tk_python; then
      echo "Kein Python mit tkinter + mujoco + trimesh gefunden." >&2
      echo "  sudo apt install python3-tk   (Version passend zu ../../../scene_editor/.venv)" >&2
      echo "  oder ohne Schieber: ./hand_tool.sh preview $*" >&2
      exit 1
    fi
    exec "$PY" hand_adjust.py "$@" ;;
  preview)  exec "$PY" hand_preview.py "$@" ;;
  snapshot) exec "$PY" hand_preview.py "$1" --snapshot "${2:-hand_${1}_vorschau.png}" ;;
  *) sed -n '2,14p' "$0"; exit 1 ;;
esac

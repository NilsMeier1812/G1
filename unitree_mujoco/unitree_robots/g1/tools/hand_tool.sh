#!/usr/bin/env bash
# =====================================================================
# Hand-Profile fuer die Sim (eigene starre Haende, z. B. gedruckte Haken).
# Nutzt das virtualenv des Szenen-Editors (trimesh, vhacdx, mujoco):
#   einmalig  unitree_mujoco/scene_editor/setup.sh
#
#   ./hand_tool.sh list                 Profile auflisten
#   ./hand_tool.sh new [datei]          Neue Hand anlegen (Datei waehlen, Montage-Vorschlaege)
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
# Fuer "new"/"adjust" braucht es zusaetzlich PyQt6 (Fenster wie das Startmenue).
# Fehlt es im Szenen-Editor-venv, wird es dort einmalig nachinstalliert; sonst
# ein System-Python mit PyQt6 + mujoco + trimesh.
has_gui() { "$1" -c "import PyQt6.QtWidgets, mujoco, trimesh, yaml" 2>/dev/null; }
pick_gui_python() {
  local c
  if [[ -x "$VENV_PY" ]] && ! has_gui "$VENV_PY" \
     && "$VENV_PY" -c "import mujoco, trimesh, yaml" 2>/dev/null; then
    echo ">> Einmalig: PyQt6 ins Szenen-Editor-venv installieren ..." >&2
    "$VENV_PY" -m pip install -q "PyQt6>=6.4" >&2 || true
  fi
  for c in "$PY" python3 python3.12 python3.11 python3.10; do
    command -v "$c" >/dev/null 2>&1 || [[ -x "$c" ]] || continue
    if has_gui "$c"; then PY="$c"; return 0; fi
  done
  return 1
}
no_gui() {
  echo "Kein Python mit PyQt6 + mujoco + trimesh gefunden." >&2
  echo "  einmalig: ../../../scene_editor/setup.sh   (installiert auch PyQt6)" >&2
  echo "  $1" >&2
  exit 1
}

cmd="${1:-}"; shift || true
case "$cmd" in
  list)     exec "$PY" gen_hand_profile.py --list ;;
  build)    exec "$PY" gen_hand_profile.py "$@" ;;
  adjust)
    pick_gui_python || no_gui "oder ohne Schieber: ./hand_tool.sh preview $*"
    exec "$PY" hand_adjust.py "$@" ;;
  new)
    pick_gui_python || no_gui "oder ohne Fenster: $PY hand_new.py --create <datei> --name <name>"
    exec "$PY" hand_new.py "$@" ;;
  preview)  exec "$PY" hand_preview.py "$@" ;;
  snapshot) exec "$PY" hand_preview.py "$1" --snapshot "${2:-hand_${1}_vorschau.png}" ;;
  *) sed -n '2,15p' "$0"; exit 1 ;;
esac

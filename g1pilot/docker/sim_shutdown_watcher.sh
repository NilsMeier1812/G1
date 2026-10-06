#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════════
#  sim_shutdown_watcher.sh <pid> — "Sim beenden" aus der Demo-GUI ausfuehren.
#
#  Die Demo-GUI laeuft IM Container und kann den Stack nicht selbst stoppen.
#  Sie legt stattdessen .sim_shutdown_request im bind-gemounteten Repo an
#  (wie .gui_open_request beim Streamdeck). Dieser Watcher laeuft auf dem Host
#  (gestartet von start.sh / make sim), sieht die Datei und stoppt den
#  Sim-Stack SOFORT: docker compose kill (SIGKILL, kein 10-s-Warten auf das
#  Herunterfahren) + down. Alle Fenster (MuJoCo, RViz, Demo-GUI) gehen zu.
#
#  Laeuft, solange <pid> lebt (start.sh nach exec = docker compose up), und
#  endet danach von selbst -> kein verwaister Prozess.
# ════════════════════════════════════════════════════════════════════════
cd "$(dirname "$0")/.." || exit 0

trigger="$PWD/.sim_shutdown_request"   # == /ros2_ws/src/g1pilot/.sim_shutdown_request im Container
parent="${1:-$PPID}"

rm -f "$trigger" 2>/dev/null   # Rest eines frueheren Laufs

while kill -0 "$parent" 2>/dev/null; do
  if [ -f "$trigger" ]; then
    rm -f "$trigger" 2>/dev/null   # Quittung fuer die Demo-GUI
    echo "[sim] Demo-GUI: Sim beenden -> Sim-Container werden sofort gestoppt."
    docker compose --profile sim kill >/dev/null 2>&1
    docker compose --profile sim down --remove-orphans --timeout 0 >/dev/null 2>&1
    exit 0
  fi
  sleep 0.3
done

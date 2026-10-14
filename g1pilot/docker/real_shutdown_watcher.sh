#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════════
#  real_shutdown_watcher.sh <pid> <profil> — "Real beenden" der Demo-GUI.
#
#  Die Demo-GUI hat die Arme bereits an den Onboard-Regler zurueckgegeben
#  (ENABLE aus -> Rueckfahrt -> Gewichtsrampe) und legt dann
#  .real_shutdown_request im bind-gemounteten Repo an. Dieser Watcher laeuft
#  auf dem Host (gestartet von start.sh / make real / make real-full) und
#  stoppt den Real-Stack SANFT: docker compose stop (SIGTERM, Gnadenfrist aus
#  docker-compose.yml) -- NICHT kill wie bei der Sim. So gibt ein noch aktiver
#  arm_controller die Arme selbst langsam ab (shutdown_release). Danach down.
#  Die Beine bleiben im Onboard-Zustand (kein Damp).
#
#  Laeuft, solange <pid> lebt (start.sh nach exec = docker compose up), und
#  endet danach von selbst -> kein verwaister Prozess.
# ════════════════════════════════════════════════════════════════════════
cd "$(dirname "$0")/.." || exit 0

trigger="$PWD/.real_shutdown_request"   # == /ros2_ws/src/g1pilot/.real_shutdown_request im Container
parent="${1:-$PPID}"
profile="${2:-real}"

case "$profile" in
  real|real-full) ;;
  *) echo "[real] real_shutdown_watcher: unbekanntes Profil '$profile'" >&2; exit 1 ;;
esac

rm -f "$trigger" 2>/dev/null   # Rest eines frueheren Laufs

while kill -0 "$parent" 2>/dev/null; do
  if [ -f "$trigger" ]; then
    rm -f "$trigger" 2>/dev/null   # Quittung fuer die Demo-GUI
    echo "[real] Demo-GUI: Real beenden -> Container werden sanft gestoppt (SIGTERM)."
    docker compose --profile "$profile" stop >/dev/null 2>&1
    docker compose --profile "$profile" down --remove-orphans >/dev/null 2>&1
    exit 0
  fi
  sleep 0.3
done

#!/usr/bin/env bash
# Mehrfach-Start-Test: Sim-Stack N-mal frisch starten und jedes Mal pruefen, ob
# GREIFEN direkt nach dem Auto-Start bedienbar ist (test_sim_startup_grasp.py).
# Bug 2026-10-13: Arme reagierten nach dem Start oft nicht (Auto-Start verloren).
#
#   ./run_startup_grasp_loop.sh [N] [streamdeck|demo]
# Headless: vorher z.B. `Xvfb :99 &` und DISPLAY=:99 setzen. Images muessen gebaut sein.
set -u
N=${1:-5}
GUI=${2:-streamdeck}
cd "$(dirname "$0")"
export G1_SIM_MODE=true HOLD_BASE_MODE=off SIM_LOCKSTEP=1 SIM_REALTIME_FACTOR=1.0
export USE_RVIZ=false G1_ENABLE_NAV=0 G1_GUI="$GUI" G1_PS4_ARMS=0
export G1_WALK_POLICY=${G1_WALK_POLICY:-g1_wholebody}
pass=0
for i in $(seq 1 "$N"); do
  docker compose --profile sim down -t 5 >/dev/null 2>&1
  docker compose --profile sim up -d >/dev/null 2>&1 || { echo "[$i] compose up fehlgeschlagen"; continue; }
  c=$(docker compose --profile sim ps -q g1pilot-sim)
  # Probe sofort starten: sie wartet selbst auf den Auto-Start und braucht nur
  # ROS-Standardnachrichten (nicht das gerade erst gebaute g1pilot-Paket).
  out=$(docker exec "$c" bash -c '
      source /opt/ros/humble/setup.bash &&
      python3 /ros2_ws/src/g1pilot/test_sim_startup_grasp.py' 2>&1)
  rc=$?
  echo "$out" | grep "\[probe\]" | sed "s/^/[$i] /"
  [ $rc -eq 0 ] && pass=$((pass + 1))
  docker compose --profile sim logs g1pilot-sim 2>/dev/null \
    | grep -E "Arm ENABLED|START BALANCING|Auto-Start" | sed "s/^/[$i]   log: /" | head -8
done
docker compose --profile sim down -t 5 >/dev/null 2>&1
echo "ERGEBNIS: $pass/$N Starts mit funktionierendem GREIFEN (GUI=$GUI)"
[ "$pass" -eq "$N" ]
